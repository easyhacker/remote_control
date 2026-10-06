// Robot-side transport abstraction + WebSocket implementation.
//
// Transports run their I/O on background threads and hand everything to the main thread through
// Poll() (connected / disconnected / message events, in order), so the executor and Unity objects are
// only ever touched from one thread. To add MQTT, ROS 2 or serial: implement IRobotTransport and
// register its URL scheme in TransportFactory.

using System;
using System.Collections.Concurrent;
using System.IO;
using System.Net.WebSockets;
using System.Text;
using System.Threading;
using System.Threading.Tasks;

namespace RobotMarket.RemoteControl
{
    public enum TransportEventKind { Connected, Disconnected, Message }

    public readonly struct TransportEvent
    {
        public readonly TransportEventKind Kind;
        public readonly string Text;     // message JSON, or disconnect reason
        public TransportEvent(TransportEventKind kind, string text) { Kind = kind; Text = text; }
    }

    public interface IRobotTransport : IDisposable
    {
        string Url { get; }
        /// <summary>True while messages can reach the controller (MQTT: broker up AND controller present).</summary>
        bool Connected { get; }
        /// <summary>Called by the session before Start(); transports that address by robot (MQTT topics) need it.</summary>
        void Bind(string robotId);
        /// <summary>Begin connecting; reconnects automatically after drops until Stop().</summary>
        void Start();
        void Stop();
        /// <summary>Queue a message. Dropped silently while disconnected (state is re-sent in hello).</summary>
        void Send(string json, Channel channel);
        /// <summary>Main thread: take the next connection/message event, if any.</summary>
        bool Poll(out TransportEvent ev);
    }

    public static class TransportFactory
    {
        public static IRobotTransport FromUrl(string url)
        {
            var scheme = new Uri(url).Scheme;
            switch (scheme)
            {
                case "ws":
                case "wss":
                    return new WebSocketRobotTransport(url);
                case "mqtt":
                case "mqtts":
                    return new MqttRobotTransport(url);
                default:
                    throw new NotSupportedException($"no transport for '{scheme}://' (known: ws, wss, mqtt, mqtts)");
            }
        }
    }

    /// <summary>
    /// Dials ws://host:port/path (the controller listens). One JSON text frame per envelope. Outgoing frames
    /// use two lanes so control messages overtake queued status messages. Reconnects with backoff 0.5 s → 5 s.
    /// </summary>
    public sealed class WebSocketRobotTransport : IRobotTransport
    {
        public string Url { get; }
        public double MinBackoff = 0.5, MaxBackoff = 5.0;

        readonly ConcurrentQueue<TransportEvent> _inbound = new ConcurrentQueue<TransportEvent>();
        readonly ConcurrentQueue<string> _control = new ConcurrentQueue<string>();
        readonly ConcurrentQueue<string> _other = new ConcurrentQueue<string>();
        readonly SemaphoreSlim _wake = new SemaphoreSlim(0);
        CancellationTokenSource _cts;
        ClientWebSocket _ws;
        volatile bool _connected;

        public WebSocketRobotTransport(string url) { Url = url; }

        public bool Connected => _connected;

        public void Bind(string robotId) { }   // the controller learns the id from hello

        public void Start()
        {
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
            cts.Cancel();
            try { _ws?.Abort(); } catch { /* already closed */ }
        }

        public void Dispose() => Stop();

        /// <summary>Close the socket without stopping (tests); the run loop reconnects.</summary>
        public void SimulateDrop()
        {
            try { _ws?.Abort(); } catch { /* ignore */ }
        }

        public void Send(string json, Channel channel)
        {
            if (!_connected) return;
            (channel == Channel.Control ? _control : _other).Enqueue(json);
            _wake.Release();
        }

        public bool Poll(out TransportEvent ev) => _inbound.TryDequeue(out ev);

        async Task RunAsync(CancellationToken token)
        {
            double backoff = MinBackoff;
            while (!token.IsCancellationRequested)
            {
                var ws = new ClientWebSocket();
                ws.Options.KeepAliveInterval = TimeSpan.FromSeconds(10);
                _ws = ws;
                string reason = null;
                try
                {
                    using (var connectTimeout = CancellationTokenSource.CreateLinkedTokenSource(token))
                    {
                        connectTimeout.CancelAfter(TimeSpan.FromSeconds(5));
                        await ws.ConnectAsync(new Uri(Url), connectTimeout.Token).ConfigureAwait(false);
                    }
                    backoff = MinBackoff;
                    while (_control.TryDequeue(out _)) { }
                    while (_other.TryDequeue(out _)) { }
                    _connected = true;
                    _inbound.Enqueue(new TransportEvent(TransportEventKind.Connected, null));

                    using (var linkCts = CancellationTokenSource.CreateLinkedTokenSource(token))
                    {
                        // Either loop failing ends the connection: a send loop that died unnoticed would leave a
                        // half-open link (messages still arrive, nothing goes out) until the controller times out.
                        var sendTask = SendLoopAsync(ws, linkCts.Token);
                        var receiveTask = ReceiveLoopAsync(ws, linkCts.Token);
                        var first = await Task.WhenAny(sendTask, receiveTask).ConfigureAwait(false);
                        linkCts.Cancel();
                        try { ws.Abort(); } catch { /* already closed */ }
                        try { await Task.WhenAll(sendTask, receiveTask).ConfigureAwait(false); } catch { /* reported below */ }
                        if (token.IsCancellationRequested)
                        {
                            reason = "stopped";
                        }
                        else if (first.IsFaulted)
                        {
                            var e = first.Exception?.GetBaseException();
                            reason = (first == sendTask ? "send failed: " : "receive failed: ") + (e?.Message ?? "unknown error");
                        }
                        else
                        {
                            reason = "closed by controller";
                        }
                    }
                }
                catch (OperationCanceledException) when (token.IsCancellationRequested) { reason = "stopped"; }
                catch (Exception e) { reason = reason ?? e.Message; }
                finally
                {
                    bool wasConnected = _connected;
                    _connected = false;
                    try { ws.Dispose(); } catch { /* ignore */ }
                    if (wasConnected) _inbound.Enqueue(new TransportEvent(TransportEventKind.Disconnected, reason));
                }
                if (token.IsCancellationRequested) break;
                try { await Task.Delay(TimeSpan.FromSeconds(backoff), token).ConfigureAwait(false); }
                catch (OperationCanceledException) { break; }
                backoff = Math.Min(MaxBackoff, backoff * 2);
            }
        }

        async Task ReceiveLoopAsync(ClientWebSocket ws, CancellationToken token)
        {
            var buffer = new byte[64 * 1024];
            var message = new MemoryStream();
            while (ws.State == WebSocketState.Open && !token.IsCancellationRequested)
            {
                var r = await ws.ReceiveAsync(new ArraySegment<byte>(buffer), token).ConfigureAwait(false);
                if (r.MessageType == WebSocketMessageType.Close)
                {
                    try { await ws.CloseOutputAsync(WebSocketCloseStatus.NormalClosure, "", CancellationToken.None).ConfigureAwait(false); }
                    catch { /* ignore */ }
                    return;
                }
                message.Write(buffer, 0, r.Count);
                if (!r.EndOfMessage) continue;
                var text = Encoding.UTF8.GetString(message.GetBuffer(), 0, (int)message.Length);
                message.SetLength(0);
                _inbound.Enqueue(new TransportEvent(TransportEventKind.Message, text));
            }
        }

        async Task SendLoopAsync(ClientWebSocket ws, CancellationToken token)
        {
            while (!token.IsCancellationRequested)
            {
                await _wake.WaitAsync(token).ConfigureAwait(false);
                while (_control.TryDequeue(out var text) || _other.TryDequeue(out text))
                {
                    var bytes = Encoding.UTF8.GetBytes(text);
                    await ws.SendAsync(new ArraySegment<byte>(bytes), WebSocketMessageType.Text, true, token).ConfigureAwait(false);
                }
            }
        }
    }
}
