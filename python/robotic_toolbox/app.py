"""
Robotic Toolbox: jog joints, save / recall poses, and move to Cartesian targets on any Remote Control robot.

    python -m robotic_toolbox                       # connector + data folder from %RC_CONFIG_DIR%\\remote_control.json
    python -m robotic_toolbox --url ws://0.0.0.0:8765/motion --data-dir D:\\robot_data

The toolbox is the controller: start it, then start the robot (Unity: press Play).

Colours: only native widgets and system colours are used, so the window follows the desktop theme
(light / dark on Windows 10+, macOS and GTK).
"""
from __future__ import annotations

import argparse
import math
import os
import subprocess
import sys
from typing import Any, Callable, Dict, List, Optional

import wx

from . import __version__
from .backend import Backend
from .ik import IkResult

APP_NAME = "Robotic Toolbox"
STEPS = [("fine", 0.01, 0.001), ("medium", 0.05, 0.005), ("coarse", 0.2, 0.02)]   # (name, rad, m)
SLIDER_RANGE = 1000
LOG_LINES = 2000
GAP = 4


def fmt(x: Optional[float], digits: int = 3) -> str:
    return "—" if x is None else f"{x:.{digits}f}"


# ── log window (on demand) ────────────────────────────────────────────────────

class LogFrame(wx.Frame):
    def __init__(self, parent: wx.Window, lines: List[str], on_close: Callable[[], None]) -> None:
        super().__init__(parent, title=f"{APP_NAME} - log", size=parent.FromDIP(wx.Size(640, 300)),
                         style=wx.DEFAULT_FRAME_STYLE | wx.FRAME_FLOAT_ON_PARENT)
        self._on_close = on_close
        panel = wx.Panel(self)
        self.text = wx.TextCtrl(panel, style=wx.TE_MULTILINE | wx.TE_READONLY | wx.TE_DONTWRAP)
        self.text.SetValue("\n".join(lines) + ("\n" if lines else ""))
        self.text.ShowPosition(self.text.GetLastPosition())
        clear = wx.Button(panel, label="Clear")
        clear.Bind(wx.EVT_BUTTON, lambda e: self.text.Clear())
        s = wx.BoxSizer(wx.VERTICAL)
        s.Add(self.text, 1, wx.EXPAND | wx.ALL, GAP)
        s.Add(clear, 0, wx.ALIGN_RIGHT | wx.LEFT | wx.RIGHT | wx.BOTTOM, GAP)
        panel.SetSizer(s)
        self.Bind(wx.EVT_CLOSE, self._closing)

    def append(self, line: str) -> None:
        self.text.AppendText(line + "\n")

    def _closing(self, event: wx.CloseEvent) -> None:
        self._on_close()
        event.Skip()


# ── joint jog tab ─────────────────────────────────────────────────────────────

class JointRow:
    """name | value (editable: Enter moves there) | unit | ◀ | slider | ▶"""

    def __init__(self, parent: wx.Window, sizer: wx.FlexGridSizer, joint: Any) -> None:
        self.joint = joint
        self.unit = "m" if joint.type == "prismatic" else "rad"
        self.digits = 4 if self.unit == "m" else 3
        self.lower = joint.lower if joint.lower is not None else (-math.pi if self.unit == "rad" else -1.0)
        self.upper = joint.upper if joint.upper is not None else (math.pi if self.unit == "rad" else 1.0)
        limits = f"limits {fmt(joint.lower, self.digits)} … {fmt(joint.upper, self.digits)} {self.unit}"
        if joint.max_velocity:
            limits += f", max {joint.max_velocity:g} {self.unit}/s"
        self.label = wx.StaticText(parent, label=joint.name)
        self.label.SetToolTip(limits)
        self.value = wx.TextCtrl(parent, value="", size=(parent.FromDIP(64), -1),
                                 style=wx.TE_PROCESS_ENTER | wx.TE_RIGHT)
        self.value.SetToolTip(f"Type a value and press Enter to move there ({limits})")
        self.unit_label = wx.StaticText(parent, label=self.unit)
        self.minus = wx.Button(parent, label="◀", style=wx.BU_EXACTFIT)
        self.slider = wx.Slider(parent, minValue=0, maxValue=SLIDER_RANGE, size=(parent.FromDIP(90), -1))
        self.slider.SetToolTip(f"Drag and release to move ({limits})")
        self.plus = wx.Button(parent, label="▶", style=wx.BU_EXACTFIT)
        self.minus.SetToolTip(f"Jog {joint.name} down")
        self.plus.SetToolTip(f"Jog {joint.name} up")
        self.dragging = False
        self.editing = False
        self.items = [self.label, self.value, self.unit_label, self.minus, self.slider, self.plus]
        for w in self.items:
            sizer.Add(w, 0, wx.ALIGN_CENTER_VERTICAL | (wx.EXPAND if w is self.slider else 0))

    def show(self, visible: bool) -> None:
        for w in self.items:
            w.Show(visible)

    def to_slider(self, x: float) -> int:
        span = self.upper - self.lower
        return int(round((x - self.lower) / span * SLIDER_RANGE)) if span > 0 else 0

    def from_slider(self) -> float:
        return self.lower + self.slider.GetValue() / SLIDER_RANGE * (self.upper - self.lower)

    def update(self, x: Optional[float]) -> None:
        if not self.editing and not self.value.HasFocus():
            self.value.ChangeValue(fmt(x, self.digits))
        if x is not None and not self.dragging:
            self.slider.SetValue(max(0, min(SLIDER_RANGE, self.to_slider(x))))


class JogTab(wx.Panel):
    def __init__(self, parent: wx.Window, frame: "ToolboxFrame") -> None:
        super().__init__(parent)
        self.frame = frame
        self.rows: Dict[str, JointRow] = {}
        bar = wx.BoxSizer(wx.HORIZONTAL)
        self.group = wx.Choice(self, choices=["All"])
        self.group.SetSelection(0)
        self.group.SetToolTip("Show the joints of one arm / group")
        self.group.Bind(wx.EVT_CHOICE, lambda e: self.apply_group())
        self.step = wx.Choice(self, choices=[f"{r:g} rad / {m * 1000:g} mm" for n, r, m in STEPS])
        self.step.SetSelection(1)
        self.step.SetToolTip("Jog step per click (revolute / prismatic joints)")
        self.speed = wx.SpinCtrl(self, min=5, max=100, initial=50, size=(self.FromDIP(56), -1))
        self.speed.SetToolTip("Speed: percent of each joint's max velocity (jog, poses and targets)")
        self.continuous = wx.CheckBox(self, label="Hold")
        self.continuous.SetToolTip("Hold to move: keep ◀ / ▶ pressed to move until released (or the limit)")
        for w, label in [(self.group, None), (self.step, None), (self.speed, "Speed %"), (self.continuous, None)]:
            if label:
                bar.Add(wx.StaticText(self, label=label), 0, wx.ALIGN_CENTER_VERTICAL | wx.RIGHT, 3)
            bar.Add(w, 0, wx.ALIGN_CENTER_VERTICAL | wx.RIGHT, 6)

        buttons = wx.BoxSizer(wx.HORIZONTAL)
        home = wx.Button(self, label="Home")
        home.SetToolTip("Move to the home pose (built-in: all joints 0, unless you saved a pose named 'home')")
        home.Bind(wx.EVT_BUTTON, lambda e: frame.go_pose("home"))
        save = wx.Button(self, label="Save pose…")
        save.SetToolTip("Save the current joint positions as a named pose")
        save.Bind(wx.EVT_BUTTON, lambda e: frame.save_pose_dialog())
        buttons.Add(home, 0, wx.RIGHT, GAP)
        buttons.Add(save, 0)

        self.area = wx.ScrolledWindow(self, style=wx.VSCROLL)
        self.area.SetScrollRate(0, self.FromDIP(10))
        self.grid = wx.FlexGridSizer(cols=6, vgap=2, hgap=4)
        self.grid.AddGrowableCol(4, 1)
        self.area.SetSizer(self.grid)
        self.grid.Add(wx.StaticText(self.area, label="Waiting for a robot…"))

        s = wx.BoxSizer(wx.VERTICAL)
        s.Add(bar, 0, wx.EXPAND | wx.ALL, GAP)
        s.Add(self.area, 1, wx.EXPAND | wx.LEFT | wx.RIGHT, GAP)
        s.Add(buttons, 0, wx.ALL, GAP)
        self.SetSizer(s)

    @property
    def speed_fraction(self) -> float:
        return self.speed.GetValue() / 100.0

    def step_for(self, row: JointRow) -> float:
        _, rad, m = STEPS[self.step.GetSelection()]
        return m if row.unit == "m" else rad

    def build(self, joints: List[Any]) -> None:
        names = [j.name for j in joints]
        if names == list(self.rows):
            return
        self.grid.Clear(delete_windows=True)
        self.rows.clear()
        for j in joints:
            row = JointRow(self.area, self.grid, j)
            row.minus.Bind(wx.EVT_BUTTON, lambda e, r=row: self._click(r, -1))
            row.plus.Bind(wx.EVT_BUTTON, lambda e, r=row: self._click(r, +1))
            for btn, d in ((row.minus, -1), (row.plus, +1)):
                btn.Bind(wx.EVT_LEFT_DOWN, lambda e, r=row, d=d: self._press(e, r, d))
                btn.Bind(wx.EVT_LEFT_UP, self._release)
            row.slider.Bind(wx.EVT_SCROLL_THUMBTRACK, lambda e, r=row: self._drag(e, r))
            row.slider.Bind(wx.EVT_SCROLL_CHANGED, lambda e, r=row: self._slider_release(e, r))
            row.value.Bind(wx.EVT_TEXT, lambda e, r=row: setattr(r, "editing", True))
            row.value.Bind(wx.EVT_TEXT_ENTER, lambda e, r=row: self._enter(r))
            row.value.Bind(wx.EVT_KILL_FOCUS, lambda e, r=row: self._leave(e, r))
            row.value.Bind(wx.EVT_SET_FOCUS, lambda e, r=row: self._focus(e, r))
            self.rows[j.name] = row
        if not joints:
            self.grid.Add(wx.StaticText(self.area, label="Waiting for a robot…"))
        prefixes = sorted({n.split("_")[0] + "_" for n in names if "_" in n and len(n.split("_")[0]) <= 3})
        self.group.Set(["All"] + prefixes)
        self.group.SetSelection(0)
        self.apply_group()

    def apply_group(self) -> None:
        g = self.group.GetStringSelection()
        for name, row in self.rows.items():
            row.show(g in ("", "All") or name.startswith(g))
        self.area.Layout()
        self.area.FitInside()

    def update(self, pos: Dict[str, float]) -> None:
        for name, row in self.rows.items():
            row.update(pos.get(name))

    # events
    def _click(self, row: JointRow, direction: int) -> None:
        if not self.continuous.GetValue():
            self.frame.run(self.frame.backend.jog_step(row.joint.name, direction * self.step_for(row),
                                                       self.speed_fraction), what=f"jog {row.joint.name}")

    def _press(self, event: wx.MouseEvent, row: JointRow, direction: int) -> None:
        event.Skip()
        if self.continuous.GetValue():
            self.frame.run(self.frame.backend.jog_start(row.joint.name, direction, self.speed_fraction),
                           what=f"jog {row.joint.name}")

    def _release(self, event: wx.MouseEvent) -> None:
        event.Skip()
        if self.continuous.GetValue():
            self.frame.run(self.frame.backend.jog_stop(), what="jog")

    def _drag(self, event: wx.ScrollEvent, row: JointRow) -> None:
        row.dragging = True
        row.value.ChangeValue(fmt(row.from_slider(), row.digits))
        event.Skip()

    def _slider_release(self, event: wx.ScrollEvent, row: JointRow) -> None:
        row.dragging = False
        self._move(row, row.from_slider())
        event.Skip()

    def _enter(self, row: JointRow) -> None:
        text = row.value.GetValue().strip().replace(",", ".")
        try:
            x = float(text)
        except ValueError:
            self.frame.log(f"{row.joint.name}: '{text}' is not a number")
            return
        clamped = min(max(x, row.lower), row.upper)
        if clamped != x:
            self.frame.log(f"{row.joint.name}: {x:g} is outside the limits - moving to {clamped:g}")
        row.editing = False
        row.value.ChangeValue(fmt(clamped, row.digits))
        row.value.SelectAll()        # ready for the next value
        self._move(row, clamped)

    def _focus(self, event: wx.FocusEvent, row: JointRow) -> None:
        event.Skip()
        wx.CallAfter(lambda: row.value and row.value.SelectAll())   # typing replaces the shown value

    def _leave(self, event: wx.FocusEvent, row: JointRow) -> None:
        row.editing = False          # unconfirmed edits are dropped; the next update shows the position
        event.Skip()

    def _move(self, row: JointRow, target: float) -> None:
        b = self.frame.backend
        self.frame.run(b.move_joints({row.joint.name: target}, speed=self.speed_fraction,
                                     label=f"{row.joint.name} → {target:.{row.digits}f}"), what=row.joint.name)


# ── poses tab ─────────────────────────────────────────────────────────────────

class PosesTab(wx.Panel):
    def __init__(self, parent: wx.Window, frame: "ToolboxFrame") -> None:
        super().__init__(parent)
        self.frame = frame
        self.list = wx.ListCtrl(self, style=wx.LC_REPORT | wx.LC_SINGLE_SEL)
        self.builtin: set = set()
        for i, (title, width) in enumerate([("Name", 150), ("Joints", 50), ("Saved", 150)]):
            self.list.InsertColumn(i, title, width=self.FromDIP(width))
        self.list.Bind(wx.EVT_LIST_ITEM_ACTIVATED, lambda e: self.go())
        self.list.SetToolTip("Double-click a pose to move there")
        row = wx.BoxSizer(wx.HORIZONTAL)
        for label, handler, tip in [("Go", self.go, "Move to the selected pose"),
                                    ("Home", lambda: frame.go_pose("home"), "Move to the home pose"),
                                    ("Save pose…", frame.save_pose_dialog, "Save the current positions"),
                                    ("Delete", self.delete, "Delete the selected pose")]:
            btn = wx.Button(self, label=label)
            btn.SetToolTip(tip)
            btn.Bind(wx.EVT_BUTTON, lambda e, h=handler: h())
            row.Add(btn, 0, wx.RIGHT, GAP)
        s = wx.BoxSizer(wx.VERTICAL)
        s.Add(self.list, 1, wx.EXPAND | wx.ALL, GAP)
        s.Add(row, 0, wx.LEFT | wx.RIGHT | wx.BOTTOM, GAP)
        self.SetSizer(s)

    def names(self) -> List[str]:
        return [self.list.GetItemText(i) for i in range(self.list.GetItemCount())]

    def refresh(self) -> None:
        self.list.DeleteAllItems()
        try:
            poses = self.frame.backend.list_poses()
        except Exception as exc:
            self.frame.log(f"poses: {exc}")
            return
        self.builtin = {p["name"] for p in poses if p.get("builtin")}
        for p in poses:
            i = self.list.InsertItem(self.list.GetItemCount(), p["name"])
            self.list.SetItem(i, 1, str(p["joints"]))
            self.list.SetItem(i, 2, p["saved_at"] if p.get("builtin") else p["saved_at"].replace("T", " ")[:19])

    def selected(self) -> Optional[str]:
        i = self.list.GetFirstSelected()
        return self.list.GetItemText(i) if i >= 0 else None

    def go(self) -> None:
        name = self.selected()
        if name is None:
            self.frame.log("select a pose first")
            return
        self.frame.go_pose(name)

    def delete(self) -> None:
        name = self.selected()
        if name is None:
            return
        if name in self.builtin:
            wx.MessageBox("The built-in home pose can't be deleted.\n\nSave a pose named 'home' to replace it.",
                          APP_NAME, wx.OK | wx.ICON_INFORMATION, self)
            return
        if wx.MessageBox(f"Delete pose '{name}'?", APP_NAME, wx.YES_NO | wx.NO_DEFAULT | wx.ICON_QUESTION,
                         self) != wx.YES:
            return
        try:
            self.frame.backend.delete_pose(name)
            self.frame.log(f"deleted pose '{name}'")
        except Exception as exc:
            self.frame.log(f"delete pose: {exc}")
        self.refresh()


# ── target tab ────────────────────────────────────────────────────────────────

class TargetTab(wx.Panel):
    def __init__(self, parent: wx.Window, frame: "ToolboxFrame") -> None:
        super().__init__(parent)
        self.frame = frame
        top = wx.BoxSizer(wx.HORIZONTAL)
        self.tool = wx.Choice(self, size=(self.FromDIP(110), -1))
        self.tool.Bind(wx.EVT_CHOICE, lambda e: self.fill_bases())
        self.base = wx.Choice(self, size=(self.FromDIP(110), -1))
        self.position_only = wx.CheckBox(self, label="Position only")
        self.position_only.SetToolTip("Ignore the tool orientation")
        for label, w in [("Tool", self.tool), ("relative to", self.base)]:
            top.Add(wx.StaticText(self, label=label), 0, wx.ALIGN_CENTER_VERTICAL | wx.RIGHT, 3)
            top.Add(w, 0, wx.RIGHT, 10)
        top.Add(self.position_only, 0, wx.ALIGN_CENTER_VERTICAL)

        grid = wx.FlexGridSizer(cols=9, vgap=GAP, hgap=3)
        self.coord: Dict[str, wx.SpinCtrlDouble] = {}
        for key, unit, inc, rng, digits in [("x", "m", 0.005, 10, 4), ("y", "m", 0.005, 10, 4), ("z", "m", 0.005, 10, 4),
                                            ("roll", "°", 1, 360, 2), ("pitch", "°", 1, 360, 2), ("yaw", "°", 1, 360, 2)]:
            ctrl = wx.SpinCtrlDouble(self, min=-rng, max=rng, inc=inc, size=(self.FromDIP(84), -1))
            ctrl.SetDigits(digits)
            self.coord[key] = ctrl
            grid.Add(wx.StaticText(self, label=key), 0, wx.ALIGN_CENTER_VERTICAL | wx.ALIGN_RIGHT)
            grid.Add(ctrl, 0)
            grid.Add(wx.StaticText(self, label=unit), 0, wx.ALIGN_CENTER_VERTICAL | wx.RIGHT, 8)

        buttons = wx.BoxSizer(wx.HORIZONTAL)
        for label, handler in [("Use current", self.use_current), ("Check reachability", self.check),
                               ("Move to target", self.move)]:
            btn = wx.Button(self, label=label)
            btn.Bind(wx.EVT_BUTTON, lambda e, h=handler: h())
            buttons.Add(btn, 0, wx.RIGHT, GAP)
        buttons.Add(wx.StaticText(self, label="Time"), 0, wx.ALIGN_CENTER_VERTICAL | wx.LEFT | wx.RIGHT, 6)
        self.duration = wx.TextCtrl(self, value="auto", size=(self.FromDIP(50), -1))
        self.duration.SetToolTip("Seconds, or 'auto' (from Speed % and each joint's max velocity)")
        buttons.Add(self.duration, 0, wx.ALIGN_CENTER_VERTICAL)
        buttons.Add(wx.StaticText(self, label="s"), 0, wx.ALIGN_CENTER_VERTICAL | wx.LEFT, 3)

        self.result = wx.StaticText(self, label="")
        s = wx.BoxSizer(wx.VERTICAL)
        s.Add(top, 0, wx.ALL, GAP)
        s.Add(grid, 0, wx.ALL, GAP)
        s.Add(buttons, 0, wx.ALL, GAP)
        s.Add(self.result, 0, wx.EXPAND | wx.ALL, GAP)
        self.SetSizer(s)

    def fill_tools(self) -> None:
        current = self.tool.GetStringSelection()
        tools = self.frame.backend.tools()
        self.tool.Set(tools)
        if current in tools:
            self.tool.SetStringSelection(current)
        elif tools:
            self.tool.SetSelection(0)
        self.fill_bases()

    def fill_bases(self) -> None:
        tool = self.tool.GetStringSelection()
        bases = self.frame.backend.bases_for(tool) if tool else []
        current = self.base.GetStringSelection()
        self.base.Set(bases)
        if current in bases:
            self.base.SetStringSelection(current)
        elif bases:
            root = (self.frame.backend.description or {}).get("root")
            self.base.SetStringSelection(root if root in bases else bases[0])

    def args(self):
        tool = self.tool.GetStringSelection()
        if not tool:
            raise ValueError("no tool - the robot sent no kinematic tree")
        base = self.base.GetStringSelection() or None
        xyz = [self.coord[k].GetValue() for k in ("x", "y", "z")]
        rpy = [math.radians(self.coord[k].GetValue()) for k in ("roll", "pitch", "yaw")]
        return tool, base, xyz, rpy, self.position_only.GetValue()

    def show_result(self, r: IkResult) -> None:
        if r.reachable:
            text = (f"✓ Reachable: {r.position_error * 1000:.2f} mm"
                    + ("" if self.position_only.GetValue() else f" / {math.degrees(r.rotation_error):.2f}°")
                    + f", {len(r.positions)} joints")
            if r.near_limits:
                text += f"; near a limit: {', '.join(r.near_limits)}"
        else:
            text = f"✗ {r.message}"
        self.result.SetLabel(text)
        self.result.Wrap(max(200, self.GetClientSize().width - 2 * GAP))
        self.Layout()

    def use_current(self) -> None:
        try:
            tool, base, *_ = self.args()
            xyz, rpy = self.frame.backend.tool_pose(tool, base)
        except Exception as exc:
            self.frame.log(f"current tool pose: {exc}")
            return
        for k, v in zip(("x", "y", "z"), xyz):
            self.coord[k].SetValue(round(v, 4))
        for k, v in zip(("roll", "pitch", "yaw"), rpy):
            self.coord[k].SetValue(round(math.degrees(v), 2))
        self.result.SetLabel(f"current {tool} pose in {base}")

    def check(self) -> None:
        try:
            args = self.args()
        except Exception as exc:
            self.frame.log(f"check: {exc}")
            return
        self.result.SetLabel("checking…")
        self.frame.run(self.frame.backend.check_target_async(*args), self.show_result, "check")

    def move(self) -> None:
        try:
            args = self.args()
            text = self.duration.GetValue().strip().lower()
            duration = None if text in ("", "auto") else float(text)
            if duration is not None and duration <= 0:
                raise ValueError("time must be > 0 s")
        except Exception as exc:
            self.frame.log(f"move to target: {exc}")
            return
        self.result.SetLabel("solving…")

        def done(r: IkResult) -> None:
            self.show_result(r)
            self.frame.log(f"→ target {args[0]}" if r.reachable else f"target not sent: {r.message}")
        f = self.frame
        f.run(f.backend.move_to_target(*args, duration=duration, speed=f.jog.speed_fraction), done, "move to target")


# ── main window ───────────────────────────────────────────────────────────────

class ToolboxFrame(wx.Frame):
    def __init__(self, backend: Backend) -> None:
        super().__init__(None, title=APP_NAME)
        self.SetSize(self.FromDIP(wx.Size(500, 540)))
        self.SetMinSize(self.FromDIP(wx.Size(440, 380)))
        self.backend = backend
        self.robot_ids: List[str] = []
        self.log_lines: List[str] = []
        self.log_frame: Optional[LogFrame] = None
        self._build_menu()
        self._build_ui()
        self.CreateStatusBar(2)
        self.SetStatusWidths([-3, -2])
        self.Bind(wx.EVT_CLOSE, self.on_close)

        b = backend
        b.on_log = self.log
        b.on_robots = self.on_robots
        b.on_robot_changed = self.on_robot_changed
        b.on_positions = self.jog.update
        b.on_state = self.on_state
        self.SetStatusText(f"{b.endpoint}  ·  data: {b.data_dir or 'none (poses disabled)'}", 1)
        self.log(f"{APP_NAME} {__version__} listening on {b.endpoint} - start the robot (Unity: press Play)")
        self.on_robot_changed()

    def _build_menu(self) -> None:
        bar = wx.MenuBar()
        m = wx.Menu()
        self.Bind(wx.EVT_MENU, self.on_open_data, m.Append(wx.ID_ANY, "Open &data folder"))
        m.AppendSeparator()
        self.Bind(wx.EVT_MENU, lambda e: self.Close(), m.Append(wx.ID_EXIT, "&Quit\tCtrl+Q"))
        bar.Append(m, "&File")
        m = wx.Menu()
        self.log_item = m.AppendCheckItem(wx.ID_ANY, "&Log window\tCtrl+L")
        self.Bind(wx.EVT_MENU, lambda e: self.toggle_log(), self.log_item)
        bar.Append(m, "&View")
        m = wx.Menu()
        self.Bind(wx.EVT_MENU, self.on_describe, m.Append(wx.ID_ANY, "&Describe && save\tCtrl+D"))
        self.Bind(wx.EVT_MENU, self.on_reload, m.Append(wx.ID_REFRESH, "&Reload description\tF5"))
        self.Bind(wx.EVT_MENU, lambda e: self.save_pose_dialog(), m.Append(wx.ID_ANY, "&Save pose…\tCtrl+S"))
        m.AppendSeparator()
        self.Bind(wx.EVT_MENU, self.on_stop, m.Append(wx.ID_ANY, "S&top all\tEsc"))
        bar.Append(m, "&Robot")
        m = wx.Menu()
        self.Bind(wx.EVT_MENU, self.on_about, m.Append(wx.ID_ABOUT, "&About"))
        bar.Append(m, "&Help")
        self.SetMenuBar(bar)

    def _build_ui(self) -> None:
        panel = wx.Panel(self)
        head = wx.BoxSizer(wx.HORIZONTAL)
        self.robot_choice = wx.Choice(panel, size=(self.FromDIP(140), -1))
        self.robot_choice.SetToolTip("Robot to control")
        self.robot_choice.Bind(wx.EVT_CHOICE, self.on_select_robot)
        self.status = wx.StaticText(panel, label="○ no robot")
        self.state = wx.StaticText(panel, label="")
        head.Add(self.robot_choice, 0, wx.ALIGN_CENTER_VERTICAL | wx.RIGHT, 8)
        head.Add(self.status, 0, wx.ALIGN_CENTER_VERTICAL | wx.RIGHT, 14)
        head.Add(self.state, 1, wx.ALIGN_CENTER_VERTICAL)
        self.names = wx.StaticText(panel, label="")
        head2 = wx.BoxSizer(wx.HORIZONTAL)
        head2.Add(self.names, 1, wx.ALIGN_CENTER_VERTICAL)
        for label, handler, tip in [("Pause", self.on_pause, "Pause the last goal"),
                                    ("Resume", self.on_resume, "Resume the last goal"),
                                    ("Cancel", self.on_cancel, "Cancel the last goal"),
                                    ("■ Stop", self.on_stop, "Stop all motion (Esc)")]:
            btn = wx.Button(panel, label=label, style=wx.BU_EXACTFIT)
            btn.SetToolTip(tip)
            btn.Bind(wx.EVT_BUTTON, handler)
            head2.Add(btn, 0, wx.LEFT, 3)

        self.book = wx.Notebook(panel)
        self.jog = JogTab(self.book, self)
        self.poses = PosesTab(self.book, self)
        self.target = TargetTab(self.book, self)
        self.book.AddPage(self.jog, "Joint jog")
        self.book.AddPage(self.poses, "Poses")
        self.book.AddPage(self.target, "Target")

        s = wx.BoxSizer(wx.VERTICAL)
        s.Add(head, 0, wx.EXPAND | wx.LEFT | wx.RIGHT | wx.TOP, 6)
        s.Add(head2, 0, wx.EXPAND | wx.LEFT | wx.RIGHT | wx.TOP, 6)
        s.Add(self.book, 1, wx.EXPAND | wx.ALL, 6)
        panel.SetSizer(s)

    # ── helpers ──────────────────────────────────────────────────────────────

    def log(self, text: str) -> None:
        self.log_lines.append(text)
        del self.log_lines[:-LOG_LINES]
        if self.log_frame is not None:
            self.log_frame.append(text)
        self.SetStatusText(text, 0)

    def toggle_log(self) -> None:
        if self.log_frame is None:
            self.log_frame = LogFrame(self, self.log_lines, self._log_closed)
            pos, size = self.GetPosition(), self.GetSize()
            self.log_frame.SetPosition(wx.Point(pos.x, pos.y + size.height))
            self.log_frame.Show()
            self.log_item.Check(True)
        else:
            self.log_frame.Close()

    def _log_closed(self) -> None:
        self.log_frame = None
        self.log_item.Check(False)

    def run(self, coro, on_done=None, what: str = "") -> None:
        def on_error(exc: BaseException) -> None:
            self.log(f"{what + ': ' if what else ''}{exc or type(exc).__name__}")
        self.backend.call(coro, on_done, on_error)

    def go_pose(self, name: str) -> None:
        self.run(self.backend.go_to_pose(name, self.jog.speed_fraction),
                 lambda goal: self.log(f"→ pose '{name}' (goal {goal.goal_id})"), f"pose '{name}'")

    def save_pose_dialog(self) -> None:
        """Prompt for a name; confirm before replacing an existing pose."""
        if self.backend.robot is None:
            self.log("save pose: no robot connected")
            return
        existing = set(self.poses.names())
        name = self.poses.selected() or ""
        while True:
            with wx.TextEntryDialog(self, "Pose name:", "Save pose", name) as dlg:
                if dlg.ShowModal() != wx.ID_OK:
                    return
                name = dlg.GetValue().strip()
            if not name:
                wx.MessageBox("The name must not be empty.", APP_NAME, wx.OK | wx.ICON_WARNING, self)
                continue
            if name in existing:
                what = ("This replaces the built-in home pose (all joints 0)" if name in self.poses.builtin
                        else f"A pose named '{name}' already exists")
                answer = wx.MessageBox(f"{what}.\n\nReplace it with the current positions?", "Save pose",
                                       wx.YES_NO | wx.CANCEL | wx.NO_DEFAULT | wx.ICON_QUESTION, self)
                if answer == wx.CANCEL:
                    return
                if answer == wx.NO:
                    continue          # ask for another name
            break

        def done(path) -> None:
            self.log(f"saved pose '{name}'")
            self.poses.refresh()
        self.run(self.backend.save_pose(name), done, "save pose")

    # ── backend events ───────────────────────────────────────────────────────

    def on_robots(self, ids: List[str]) -> None:
        current = self.robot_choice.GetStringSelection()
        self.robot_ids = ids
        self.robot_choice.Set(ids)
        if current in ids:
            self.robot_choice.SetStringSelection(current)
        self.on_robot_changed()

    def on_robot_changed(self) -> None:
        robot = self.backend.robot
        if robot is None:
            self.status.SetLabel("○ no robot")
            self.names.SetLabel("Start the robot (Unity: press Play)")
            return
        if robot.robot_id in self.robot_ids:
            self.robot_choice.SetStringSelection(robot.robot_id)
        self.status.SetLabel("● connected" if robot.online else "○ offline")
        n = robot.names
        self.names.SetLabel(f"{n['project']} / {n['stage']} / {n['robot']}")
        self.on_state(robot.state or {})
        self.jog.build(list(robot.joints))
        self.target.fill_tools()
        self.poses.refresh()
        self.Layout()

    def on_state(self, state: Dict[str, Any]) -> None:
        text = f"state: {state.get('state', '—')}"
        if state.get("pause_reason"):
            text += f" [{state['pause_reason']}]"
        if state.get("queued"):
            text += f"  +{len(state['queued'])} queued"
        self.state.SetLabel(text)

    # ── commands ─────────────────────────────────────────────────────────────

    def on_select_robot(self, event) -> None:
        robot_id = self.robot_choice.GetStringSelection()
        if robot_id:
            self.run(self.backend.select(robot_id), what="select")

    def on_pause(self, event) -> None:
        self.run(self.backend.pause(), lambda ack: self.log(f"pause: {ack.get('message') or 'ok'}"), "pause")

    def on_resume(self, event) -> None:
        self.run(self.backend.resume(), lambda ack: self.log(f"resume: {ack.get('message') or 'ok'}"), "resume")

    def on_cancel(self, event) -> None:
        self.run(self.backend.cancel(), lambda ack: self.log(f"cancel: {ack.get('message') or 'ok'}"), "cancel")

    def on_stop(self, event) -> None:
        self.run(self.backend.stop_all(), lambda ack: self.log("stop: all motion stopped"), "stop")

    def on_describe(self, event) -> None:
        self.run(self.backend.describe_and_save(), lambda path: self.log(f"description saved: {path}"), "describe")

    def on_reload(self, event) -> None:
        robot = self.backend.robot
        if robot is not None:
            self.run(self.backend.select(robot.robot_id), lambda _: self.log("description reloaded"), "reload")

    def on_open_data(self, event) -> None:
        robot, data_dir = self.backend.robot, self.backend.data_dir
        if data_dir is None:
            wx.MessageBox('No data folder: set "data_dir" in remote_control.json.', APP_NAME, wx.OK, self)
            return
        folder = robot.store.dir if robot is not None and robot.store.dir.exists() else data_dir
        folder.mkdir(parents=True, exist_ok=True)
        if sys.platform.startswith("win"):
            os.startfile(str(folder))  # type: ignore[attr-defined]
        else:
            subprocess.Popen(["open" if sys.platform == "darwin" else "xdg-open", str(folder)])

    def on_about(self, event) -> None:
        wx.MessageBox(f"{APP_NAME} {__version__}\n\nJog joints, save poses and move to targets on robots that "
                      f"speak the Remote Control Motion Protocol.\n\nComm: {self.backend.endpoint}",
                      f"About {APP_NAME}", wx.OK | wx.ICON_INFORMATION, self)

    def on_close(self, event: wx.CloseEvent) -> None:
        if self.log_frame is not None:
            self.log_frame.Destroy()
        self.backend.stop()
        event.Skip()


def follow_system_theme(app: wx.App, theme: str = "auto") -> None:
    """Windows: use dark mode when the system does ("auto"), or always ("dark"). macOS and GTK follow the
    desktop theme by themselves; there "dark" / "light" only take effect through wx.SystemSettings.SelectLightDark."""
    enable = getattr(app, "MSWEnableDarkMode", None)
    if enable is not None and theme != "light":
        try:
            enable(wx.App.DarkMode_Always if theme == "dark" else wx.App.DarkMode_Auto)
        except Exception:
            pass
    elif enable is None and theme in ("dark", "light") and hasattr(wx.SystemSettings, "SelectLightDark"):
        try:
            wx.SystemSettings.SelectLightDark(wx.SystemSettings.Appearance_Dark if theme == "dark"
                                              else wx.SystemSettings.Appearance_Light)
        except Exception:
            pass


def main(argv: Optional[List[str]] = None) -> int:
    ap = argparse.ArgumentParser(prog="robotic_toolbox", description=APP_NAME)
    ap.add_argument("--url", default=None, help="connector URL, overriding the config file")
    ap.add_argument("--data-dir", default=None, help="robot data folder, overriding the config file")
    ap.add_argument("--theme", choices=["auto", "dark", "light"], default="auto",
                    help="auto (default) follows the desktop's light / dark setting")
    args = ap.parse_args(argv)

    app = wx.App()
    app.SetAppName(APP_NAME)
    follow_system_theme(app, args.theme)
    backend = Backend(post=wx.CallAfter, url=args.url, data_dir=args.data_dir)
    try:
        backend.start()
    except Exception as exc:
        wx.MessageBox(f"Could not start the controller:\n\n{exc}\n\nSet RC_CONFIG_DIR to the folder with "
                      f"remote_control.json, or start with --url ws://0.0.0.0:8765/motion",
                      APP_NAME, wx.OK | wx.ICON_ERROR)
        return 2
    frame = ToolboxFrame(backend)
    frame.Show()
    app.MainLoop()
    return 0
