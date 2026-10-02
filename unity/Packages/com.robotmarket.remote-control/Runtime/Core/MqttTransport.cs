// MQTT robot transport (MQTTnet 4.x). Mirrors python/remote_control/transports/mqtt.py.
//
// URL: mqtt://[user:pass@]host[:1883]/<prefix>   (mqtts:// for TLS, default port 8883; prefix default "rc")
// Topics:
//   <prefix>/<robot_id>/cmd      in    goals
//   <prefix>/<robot_id>/ctrl     in    pause/resume/cancel/stop/welcome/heartbeat
//   <prefix>/<robot_id>/status   out   everything the robot sends
//   <prefix>/<robot_id>/online   out   retained "1" / "0" (last will)
//   <prefix>/_controller/online  in    retained controller presence
// Connected = broker connected AND controller present, so the session pauses / re-announces exactly as
// over WebSocket.

using System;
using System.Collections.Concurrent;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using MQTTnet;
using MQTTnet.Client;
using MQTTnet.Formatter;
using MQTTnet.Protocol;

namespace RobotMarket.RemoteControl
{
    public sealed class MqttRobotTransport : IRobotTransport
    {
        public const string ControllerPresence = "_controller";
        public string Url { get; }
        public double MinBackoff = 1.0, MaxBackoff = 5.0;

        readonly string _host, _user, _pass, _prefix;
        readonly int _port;
        readonly bool _tls;
        string _robotId, _tCmd, _tCtrl, _tStatus, _tOnline, _tPresence;

        readonly ConcurrentQueue<TransportEvent> _inbound = new ConcurrentQueue<TransportEvent>();
        readonly ConcurrentQueue<(string json, bool heartbeat)> _outbound = new ConcurrentQueue<(string, bool)>();
        readonly SemaphoreSlim _wake = new SemaphoreSlim(0);
        readonly object _lock = new object();
        CancellationTokenSource _cts;
        IMqttClient _client;
        bool _brokerUp, _controllerUp;
        volatile bool _linked;

        public MqttRobotTransport(string url)
        {
            Url = url;
            var u = new Uri(url);
            _tls = u.Scheme == "mqtts";
            _host = u.Host;
            _port = u.IsDefaultPort || u.Port <= 0 ? (_tls ? 8883 : 1883) : u.Port;
            if (!string.IsNullOrEmpty(u.UserInfo))
            {
                var parts = u.UserInfo.Split(new[] { ':' }, 2);
                _user = Uri.UnescapeDataString(parts[0]);
                _pass = parts.Length > 1 ? Uri.UnescapeDataString(parts[1]) : null;
            }
            var prefix = u.AbsolutePath.Trim('/');
            _prefix = prefix.Length > 0 ? Uri.UnescapeDataString(prefix) : "rc";
        }

        public bool Connected => _linked;

        public void Bind(string robotId)
        {
            if (string.IsNullOrEmpty(robotId) || robotId.IndexOfAny(new[] { '/', '+', '#' }) >= 0 || robotId == ControllerPresence)
                throw new ArgumentException($"robot_id '{robotId}' cannot be used as an MQTT topic level");
            _robotId = robotId;
            _tCmd = $"{_prefix}/{robotId}/cmd";
            _tCtrl = $"{_prefix}/{robotId}/ctrl";
            _tStatus = $"{_prefix}/{robotId}/status";
            _tOnline = $"{_prefix}/{robotId}/online";
            _tPresence = $"{_prefix}/{ControllerPresence}/online";
        }

        public void Start()
        {
            if (_robotId == null) throw new InvalidOperationException("Bind(robotId) before Start()");
            if (_cts != null) return;
            _cts = new CancellationTokenSource();
            var token = _cts.Token;
            Task.Run(() => RunAsync(token));
        }

        public void Stop()
        {
            var cts = _cts;
            _cts = null;
            if (cts == null) return;
            var client = _client;
            if (client != null && client.IsConnected)
            {
                try   // graceful: mark offline (the will only fires on ungraceful disconnects)
                {
                    client.PublishAsync(Message(_tOnline, "0", 1, retain: true)).Wait(1000);
                    client.DisconnectAsync().Wait(1000);
                }
                catch { /* broker gone */ }
            }
            cts.Cancel();
        }

        public void Dispose() => Stop();

        public void Send(string json, Channel channel)
        {
            if (!_linked) return;
            _outbound.Enqueue((json, json.Contains("\"type\":\"heartbeat\"")));
            _wake.Release();
        }

        public bool Poll(out TransportEvent ev) => _inbound.TryDequeue(out ev);

        static MqttApplicationMessage Message(string topic, string text, int qos, bool retain = false) =>
            new MqttApplicationMessageBuilder()
                .WithTopic(topic)
                .WithPayload(Encoding.UTF8.GetBytes(text))
                .WithQualityOfServiceLevel(qos == 0 ? MqttQualityOfServiceLevel.AtMostOnce : MqttQualityOfServiceLevel.AtLeastOnce)
                .WithRetainFlag(retain)
                .Build();

        void UpdateLink()
        {
            lock (_lock)
            {
                bool up = _brokerUp && _controllerUp;
                if (up == _linked) return;
                if (up) while (_outbound.TryDequeue(out _)) { }
                _linked = up;
                _inbound.Enqueue(new TransportEvent(up ? TransportEventKind.Connected : TransportEventKind.Disconnected,
                                                    up ? null : (_brokerUp ? "controller offline" : "broker connection lost")));
            }
        }

        async Task RunAsync(CancellationToken token)
        {
            var factory = new MqttFactory();
            double backoff = MinBackoff;
            while (!token.IsCancellationRequested)
            {
                var client = factory.CreateMqttClient();
                _client = client;
                var dropped = new TaskCompletionSource<bool>(TaskCreationOptions.RunContinuationsAsynchronously);
                client.DisconnectedAsync += _ => { dropped.TrySetResult(true); return Task.CompletedTask; };
                client.ApplicationMessageReceivedAsync += e =>
                {
                    var topic = e.ApplicationMessage.Topic;
                    var seg = e.ApplicationMessage.PayloadSegment;
                    var text = seg.Array == null ? "" : Encoding.UTF8.GetString(seg.Array, seg.Offset, seg.Count);
                    if (topic == _tPresence)
                    {
                        lock (_lock) _controllerUp = text == "1";
                        UpdateLink();
                    }
                    else if ((topic == _tCmd || topic == _tCtrl) && _linked)
                    {
                        _inbound.Enqueue(new TransportEvent(TransportEventKind.Message, text));
                    }
                    return Task.CompletedTask;
                };

                var builder = new MqttClientOptionsBuilder()
                    .WithTcpServer(_host, _port)
                    .WithClientId("rc-robot-" + _robotId)
                    .WithProtocolVersion(MqttProtocolVersion.V311)
                    .WithCleanSession(true)
                    .WithKeepAlivePeriod(TimeSpan.FromSeconds(10))
                    .WithTimeout(TimeSpan.FromSeconds(5))
                    .WithWillTopic(_tOnline)
                    .WithWillPayload(Encoding.UTF8.GetBytes("0"))
                    .WithWillRetain(true)
                    .WithWillQualityOfServiceLevel(MqttQualityOfServiceLevel.AtLeastOnce);
                if (_user != null) builder = builder.WithCredentials(_user, _pass);
                if (_tls) builder = builder.WithTlsOptions(o => o.UseTls());

                try
                {
                    await client.ConnectAsync(builder.Build(), token).ConfigureAwait(false);
                    await client.SubscribeAsync(new MqttClientSubscribeOptionsBuilder()
                        .WithTopicFilter(f => f.WithTopic(_tCmd).WithQualityOfServiceLevel(MqttQualityOfServiceLevel.AtLeastOnce))
                        .WithTopicFilter(f => f.WithTopic(_tCtrl).WithQualityOfServiceLevel(MqttQualityOfServiceLevel.AtLeastOnce))
                        .WithTopicFilter(f => f.WithTopic(_tPresence).WithQualityOfServiceLevel(MqttQualityOfServiceLevel.AtLeastOnce))
                        .Build(), token).ConfigureAwait(false);
                    await client.PublishAsync(Message(_tOnline, "1", 1, retain: true), token).ConfigureAwait(false);
                    backoff = MinBackoff;
                    lock (_lock) _brokerUp = true;
                    UpdateLink();

                    using (var linkCts = CancellationTokenSource.CreateLinkedTokenSource(token))
                    {
                        var sendTask = SendLoopAsync(client, linkCts.Token);
                        await Task.WhenAny(dropped.Task, Task.Delay(Timeout.Infinite, token)).ConfigureAwait(false);
                        linkCts.Cancel();
                        try { await sendTask.ConfigureAwait(false); } catch { /* cancelled */ }
                    }
                }
                catch (OperationCanceledException) when (token.IsCancellationRequested) { }
                catch (Exception) { /* connect failed — retry with backoff */ }
                finally
                {
                    lock (_lock)
                    {
                        _brokerUp = false;
                        _controllerUp = false;   // re-learned from the retained presence on reconnect
                    }
                    UpdateLink();
                    try { client.Dispose(); } catch { /* ignore */ }
                }
                if (token.IsCancellationRequested) break;
                try { await Task.Delay(TimeSpan.FromSeconds(backoff), token).ConfigureAwait(false); }
                catch (OperationCanceledException) { break; }
                backoff = Math.Min(MaxBackoff, backoff * 2);
            }
        }

        async Task SendLoopAsync(IMqttClient client, CancellationToken token)
        {
            while (!token.IsCancellationRequested)
            {
                await _wake.WaitAsync(token).ConfigureAwait(false);
                while (_outbound.TryDequeue(out var item))
                    await client.PublishAsync(Message(_tStatus, item.json, item.heartbeat ? 0 : 1), token).ConfigureAwait(false);
            }
        }
    }
}
