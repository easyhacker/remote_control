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

from . import __version__, unity_focus
from .backend import Backend
from .ik import Chain, IkResult

APP_NAME = "Robotic Toolbox"
APP_ID = "LogixPlan.RoboticToolbox"     # Windows taskbar grouping / icon (instead of python.exe's)
ICON_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "resources", "robotic-arm.ico")
STEPS = [("fine", 0.01, 0.001), ("medium", 0.05, 0.005), ("coarse", 0.2, 0.02)]   # (name, rad, m)
SLIDER_RANGE = 1000
LOG_LINES = 2000
GAP = 4


_icons: Optional[wx.IconBundle] = None


def app_icons() -> Optional[wx.IconBundle]:
    """The window / taskbar icon in the usual sizes (the .ico holds one large image; Windows picks per size)."""
    global _icons
    if _icons is None and os.path.isfile(ICON_PATH):
        image = wx.Image(ICON_PATH, wx.BITMAP_TYPE_ICO)
        if image.IsOk():
            _icons = wx.IconBundle()
            for size in (16, 20, 24, 32, 40, 48, 64, 128, 256):
                _icons.AddIcon(wx.Icon(wx.Bitmap(image.Scale(size, size, wx.IMAGE_QUALITY_HIGH))))
    return _icons


def set_windows_app_id() -> None:
    """Windows groups a Python app under python.exe (and its icon) in the taskbar unless it has its own id."""
    if sys.platform.startswith("win"):
        try:
            import ctypes
            ctypes.windll.shell32.SetCurrentProcessExplicitAppUserModelID(APP_ID)
        except Exception:
            pass


_instance_lock = None          # the single-instance mutex handle, held for the process' lifetime


def activate_running_instance(key: str) -> bool:
    """Windows: one Toolbox per config. Each Toolbox is the controller, so a second one with the same config would
    fight the first for the port (WebSocket) or for the robots (MQTT). If one already runs, bring its window to the
    front and return True; otherwise take the per-config mutex and return False."""
    global _instance_lock
    if not sys.platform.startswith("win"):
        return False
    import ctypes
    import hashlib
    from ctypes import wintypes
    kernel32, user32 = ctypes.windll.kernel32, ctypes.windll.user32
    kernel32.CreateMutexW.restype = wintypes.HANDLE
    name = "Local\\LogixPlan.RoboticToolbox." + hashlib.sha1(key.lower().encode("utf-8")).hexdigest()[:16]
    _instance_lock = kernel32.CreateMutexW(None, False, name)
    if kernel32.GetLastError() != 183:          # ERROR_ALREADY_EXISTS
        return False
    title = "LogixPlan " + APP_NAME
    found = []

    @ctypes.WINFUNCTYPE(wintypes.BOOL, wintypes.HWND, wintypes.LPARAM)
    def visit(hwnd, _):
        buf = ctypes.create_unicode_buffer(256)
        user32.GetWindowTextW(hwnd, buf, 256)
        if buf.value == title and user32.IsWindowVisible(hwnd):
            found.append(hwnd)
            return False
        return True

    user32.EnumWindows(visit, 0)
    if found:
        if user32.IsIconic(found[0]):
            user32.ShowWindow(found[0], 9)      # SW_RESTORE
        user32.SetForegroundWindow(found[0])
    return True


def fmt(x: Optional[float], digits: int = 3) -> str:
    return "—" if x is None else f"{x:.{digits}f}"


# ── log window (on demand) ────────────────────────────────────────────────────

class LogFrame(wx.Frame):
    def __init__(self, parent: wx.Window, lines: List[str], on_close: Callable[[], None]) -> None:
        super().__init__(parent, title=f"LogixPlan {APP_NAME} - log", size=parent.FromDIP(wx.Size(640, 300)),
                         style=wx.DEFAULT_FRAME_STYLE | wx.FRAME_FLOAT_ON_PARENT)
        self._on_close = on_close
        if app_icons() is not None:
            self.SetIcons(app_icons())
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


ALL_JOINTS = "All joints"


class JogTab(wx.Panel):
    def __init__(self, parent: wx.Window, frame: "ToolboxFrame") -> None:
        super().__init__(parent)
        self.frame = frame
        self.rows: Dict[str, JointRow] = {}
        bar = wx.BoxSizer(wx.HORIZONTAL)
        self.groups: List[Dict[str, Any]] = []
        self.group = wx.Choice(self, choices=[ALL_JOINTS])
        self.group.SetSelection(0)
        self.group.SetToolTip("Joint group: shows its joints; Home, Save pose and Stop act on it only.\n"
                              "Groups move independently: one can move while another is moving.")
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
        for label, handler, tip in [
                ("Home", self.home, "Move the group to home (built-in: joints at 0, unless you saved a pose "
                                    "named 'home')"),
                ("Save pose…", lambda: frame.save_pose_dialog(self.group_name),
                 "Save the current positions of the group's joints as a named pose\n"
                 "(e.g. 'close_left_gripper' for the left gripper group)"),
                ("Stop group", self.stop_group, "Cancel the motions of this group's joints; other groups keep moving"),
                ("Groups…", frame.groups_dialog, "Create, edit and delete joint groups")]:
            btn = wx.Button(self, label=label)
            btn.SetToolTip(tip)
            btn.Bind(wx.EVT_BUTTON, lambda e, h=handler: h())
            buttons.Add(btn, 0, wx.RIGHT, GAP)

        self.area = wx.ScrolledWindow(self, style=wx.VSCROLL | wx.ALWAYS_SHOW_SB)
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
            wheel_scrolls_parent(row.value)
            wheel_scrolls_parent(row.slider, always=True)     # the wheel never moves a joint
            self.rows[j.name] = row
        if not joints:
            self.grid.Add(wx.StaticText(self.area, label="Waiting for a robot…"))
        self.refresh_groups()

    @property
    def group_name(self) -> Optional[str]:
        """The selected group, None for all joints."""
        i = self.group.GetSelection()
        return self.groups[i - 1]["name"] if 0 < i <= len(self.groups) else None

    def refresh_groups(self, select: Optional[str] = None) -> None:
        current = select or self.group_name
        try:
            self.groups = self.frame.backend.groups()
        except Exception as exc:
            self.frame.log(f"groups: {exc}")
            self.groups = []
        self.group.Set([ALL_JOINTS] + [g["name"] + (" (suggested)" if g["builtin"] else "") for g in self.groups])
        names = [g["name"] for g in self.groups]
        self.group.SetSelection(names.index(current) + 1 if current in names else 0)
        self.apply_group()

    def apply_group(self) -> None:
        joints = self.frame.backend.group_joints(self.group_name) if self.group_name else None
        for name, row in self.rows.items():
            row.show(joints is None or name in joints)
        self.area.Layout()
        self.area.FitInside()

    def home(self) -> None:
        g = self.group_name
        self.frame.run(self.frame.backend.go_home(g, self.speed_fraction),
                       lambda goal: self.frame.log(f"→ home{' ' + g if g else ''} (goal {goal.goal_id})"), "home")

    def stop_group(self) -> None:
        g = self.group_name
        b = self.frame.backend
        if g is None:
            self.frame.on_stop(None)
            return
        self.frame.run(b.cancel(b.group_joints(g)), lambda ack: self.frame.log(f"stop {g}: {ack.get('message')}"),
                       f"stop {g}")

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


class GroupsDialog(wx.Dialog):
    """Joint groups: pick a group (or type a new name), tick its joints, Save. Suggested groups become saved
    groups when saved."""

    def __init__(self, frame: "ToolboxFrame") -> None:
        super().__init__(frame, title="Joint groups", style=wx.DEFAULT_DIALOG_STYLE | wx.RESIZE_BORDER)
        self.frame = frame
        self.b = frame.backend
        self.last_saved: Optional[str] = None
        self.list = wx.ListBox(self, size=self.FromDIP(wx.Size(170, 260)), style=wx.LB_SINGLE | wx.LB_ALWAYS_SB)
        self.list.Bind(wx.EVT_LISTBOX, lambda e: self.show_group())
        self.name = wx.TextCtrl(self)
        self.joints = wx.CheckListBox(self, choices=list(self.b.robot.joint_names),
                                      size=self.FromDIP(wx.Size(220, 260)), style=wx.LB_ALWAYS_SB)
        save = wx.Button(self, label="Save")
        save.SetToolTip("Save the group under the name above (replaces a group with that name)")
        save.Bind(wx.EVT_BUTTON, lambda e: self.save())
        delete = wx.Button(self, label="Delete")
        delete.Bind(wx.EVT_BUTTON, lambda e: self.delete())
        new = wx.Button(self, label="New")
        new.Bind(wx.EVT_BUTTON, lambda e: self.new())

        right = wx.BoxSizer(wx.VERTICAL)
        right.Add(wx.StaticText(self, label="Name"), 0)
        right.Add(self.name, 0, wx.EXPAND | wx.BOTTOM, GAP)
        right.Add(wx.StaticText(self, label="Joints"), 0)
        right.Add(self.joints, 1, wx.EXPAND)
        left = wx.BoxSizer(wx.VERTICAL)
        left.Add(wx.StaticText(self, label="Groups (* = suggested, not saved)"), 0)
        left.Add(self.list, 1, wx.EXPAND)
        body = wx.BoxSizer(wx.HORIZONTAL)
        body.Add(left, 1, wx.EXPAND | wx.RIGHT, GAP)
        body.Add(right, 1, wx.EXPAND)
        buttons = wx.BoxSizer(wx.HORIZONTAL)
        for btn in (new, save, delete):
            buttons.Add(btn, 0, wx.RIGHT, GAP)
        buttons.AddStretchSpacer()
        buttons.Add(wx.Button(self, wx.ID_CLOSE), 0)
        self.Bind(wx.EVT_BUTTON, lambda e: self.EndModal(wx.ID_CLOSE), id=wx.ID_CLOSE)
        s = wx.BoxSizer(wx.VERTICAL)
        s.Add(body, 1, wx.EXPAND | wx.ALL, GAP)
        s.Add(buttons, 0, wx.EXPAND | wx.LEFT | wx.RIGHT | wx.BOTTOM, GAP)
        self.SetSizerAndFit(s)
        self.reload()

    def reload(self, select: Optional[str] = None) -> None:
        self.groups = self.b.groups()
        self.list.Set([g["name"] + (" *" if g["builtin"] else "") for g in self.groups])
        names = [g["name"] for g in self.groups]
        if select in names:
            self.list.SetSelection(names.index(select))
            self.show_group()
        elif names:
            self.list.SetSelection(0)
            self.show_group()

    def show_group(self) -> None:
        i = self.list.GetSelection()
        if i < 0:
            return
        g = self.groups[i]
        self.name.SetValue(g["name"])
        self.joints.SetCheckedStrings(g["joints"])

    def new(self) -> None:
        self.list.SetSelection(wx.NOT_FOUND)
        self.name.SetValue("")
        self.joints.SetCheckedItems([])
        self.name.SetFocus()

    def save(self) -> None:
        name = self.name.GetValue().strip()
        joints = list(self.joints.GetCheckedStrings())
        saved = {g["name"] for g in self.groups if not g["builtin"]}
        i = self.list.GetSelection()
        editing = self.groups[i]["name"] if i >= 0 else None
        if name in saved and name != editing and wx.MessageBox(
                f"A group named '{name}' already exists.\n\nReplace it?", "Joint groups",
                wx.YES_NO | wx.NO_DEFAULT | wx.ICON_QUESTION, self) != wx.YES:
            return
        try:
            self.b.save_group(name, joints)
        except Exception as exc:
            wx.MessageBox(str(exc), "Joint groups", wx.OK | wx.ICON_WARNING, self)
            return
        self.frame.log(f"saved group '{name}': {', '.join(joints)}")
        self.last_saved = name
        self.reload(name)

    def delete(self) -> None:
        i = self.list.GetSelection()
        if i < 0:
            return
        g = self.groups[i]
        if g["builtin"]:
            wx.MessageBox("A suggested group is not saved, so there is nothing to delete.", "Joint groups",
                          wx.OK | wx.ICON_INFORMATION, self)
            return
        self.b.delete_group(g["name"])
        self.frame.log(f"deleted group '{g['name']}'")
        self.reload()


# ── poses tab ─────────────────────────────────────────────────────────────────

class PosesTab(wx.Panel):
    def __init__(self, parent: wx.Window, frame: "ToolboxFrame") -> None:
        super().__init__(parent)
        self.frame = frame
        self.list = wx.ListCtrl(self, style=wx.LC_REPORT | wx.LC_SINGLE_SEL | wx.VSCROLL)
        self.list.SetMinSize(self.FromDIP(wx.Size(-1, 120)))     # scrolls instead of growing the window
        self.builtin: set = set()
        for i, (title, width) in enumerate([("Name", 150), ("Group", 100), ("Joints", 50), ("Saved", 150)]):
            self.list.InsertColumn(i, title, width=self.FromDIP(width))
        self.list.Bind(wx.EVT_LIST_ITEM_ACTIVATED, lambda e: self.go())
        self.list.SetToolTip("Double-click a pose to move there. A group pose moves only its group's joints,\n"
                             "so it can run while other groups move.")
        row = wx.BoxSizer(wx.HORIZONTAL)
        for label, handler, tip in [("Go", self.go, "Move to the selected pose"),
                                    ("Home", lambda: frame.go_pose("home"), "Move all joints to the home pose"),
                                    ("Save pose…", lambda: frame.save_pose_dialog(frame.jog.group_name),
                                     "Save the current positions (of the group selected in Joint jog)"),
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
            self.list.SetItem(i, 1, p.get("group") or "all")
            self.list.SetItem(i, 2, str(p["joints"]))
            self.list.SetItem(i, 3, p["saved_at"] if p.get("builtin") else p["saved_at"].replace("T", " ")[:19])

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


# ── Tool & Targets tab: chain, TCP, targets ────────────────────────────────────

def wheel_scrolls_parent(ctrl: wx.Window, always: bool = False) -> None:
    """The mouse wheel over an unfocused number field (any slider, with always=True) scrolls the panel instead of
    changing the value."""
    def on_wheel(event: wx.MouseEvent) -> None:
        if not always and (ctrl.HasFocus() or any(c.HasFocus() for c in ctrl.GetChildren())):
            event.Skip()
            return
        parent = ctrl.GetParent()
        while parent is not None and not isinstance(parent, wx.ScrolledWindow):
            parent = parent.GetParent()
        if parent is not None:
            lines = -event.GetWheelRotation() // max(1, event.GetWheelDelta()) * event.GetLinesPerAction()
            parent.ScrollLines(lines)
    for w in [ctrl] + list(ctrl.GetChildren()):
        w.Bind(wx.EVT_MOUSEWHEEL, on_wheel)


CUSTOM = "(custom)"
UNSAVED = "(unsaved)"
REFERENCE_KEYS = ["chain", "robot", "scene"]
CLICK_ORIENTATION_KEYS = ["approach", "surface", "tcp"]
CLICK_ORIENTATION_LABELS = ["Approach (z into surface)", "Surface (z out of surface)", "Keep TCP orientation"]
REFERENCE_LABELS = ["chain origin", "robot", "scene"]


class PoseEditor:
    """x y z (m) / roll pitch yaw (°) in two rows of three."""

    def __init__(self, parent: wx.Window, on_edit: Optional[Callable[[], None]] = None) -> None:
        self.sizer = wx.FlexGridSizer(cols=9, vgap=GAP, hgap=3)
        self.ctrls: Dict[str, wx.SpinCtrlDouble] = {}
        for key, unit, inc, rng, digits in [("x", "m", 0.005, 10, 4), ("y", "m", 0.005, 10, 4), ("z", "m", 0.005, 10, 4),
                                            ("roll", "°", 1, 360, 2), ("pitch", "°", 1, 360, 2), ("yaw", "°", 1, 360, 2)]:
            ctrl = wx.SpinCtrlDouble(parent, min=-rng, max=rng, inc=inc, size=(parent.FromDIP(82), -1))
            ctrl.SetDigits(digits)
            if on_edit is not None:
                ctrl.Bind(wx.EVT_SPINCTRLDOUBLE, lambda e: on_edit())
                ctrl.Bind(wx.EVT_TEXT, lambda e: on_edit())
            wheel_scrolls_parent(ctrl)
            self.ctrls[key] = ctrl
            self.sizer.Add(wx.StaticText(parent, label=key), 0, wx.ALIGN_CENTER_VERTICAL | wx.ALIGN_RIGHT)
            self.sizer.Add(ctrl, 0)
            self.sizer.Add(wx.StaticText(parent, label=unit), 0, wx.ALIGN_CENTER_VERTICAL | wx.RIGHT, 6)
        self._setting = False

    def get(self):
        xyz = [self.ctrls[k].GetValue() for k in ("x", "y", "z")]
        rpy = [math.radians(self.ctrls[k].GetValue()) for k in ("roll", "pitch", "yaw")]
        return xyz, rpy

    def set(self, xyz, rpy) -> None:
        self._setting = True
        try:
            for k, v in zip(("x", "y", "z"), xyz):
                self.ctrls[k].SetValue(round(float(v), 4) + 0.0)            # + 0.0: no "-0.0000"
            for k, v in zip(("roll", "pitch", "yaw"), rpy):
                self.ctrls[k].SetValue(round(math.degrees(float(v)), 2) + 0.0)
        finally:
            self._setting = False

    @property
    def setting(self) -> bool:
        return self._setting


class TreeDialog(wx.Dialog):
    """The robot's kinematic tree; pick the chain origin and end."""

    def __init__(self, parent: wx.Window, tree: Dict[str, Any], origin: str, end: str,
                 on_pick: Callable[[str, str], None]) -> None:
        super().__init__(parent, title="Kinematic tree", size=parent.FromDIP(wx.Size(380, 480)),
                         style=wx.DEFAULT_DIALOG_STYLE | wx.RESIZE_BORDER)
        self.origin, self.end, self.on_pick = origin, end, on_pick
        self.tree = wx.TreeCtrl(self, style=wx.TR_DEFAULT_STYLE | wx.TR_HIDE_ROOT | wx.TR_FULL_ROW_HIGHLIGHT)
        children: Dict[str, List[Dict[str, Any]]] = {}
        for j in tree.get("joints", []):
            if j.get("parent") and j.get("child"):
                children.setdefault(j["parent"], []).append(j)
        root = self.tree.AddRoot("robot")
        self.nodes: Dict[str, Any] = {}

        def add(parent_item, link: str, joint: Optional[Dict[str, Any]]) -> None:
            text = link
            if joint is not None and joint.get("type") != "fixed":
                text += f"   ({joint['type']} {joint.get('command_name') or joint.get('name')})"
            item = self.tree.AppendItem(parent_item, text)
            self.tree.SetItemData(item, link)
            self.nodes[link] = item
            for j in children.get(link, []):
                add(item, j["child"], j)

        add(root, tree.get("root") or "", None)
        self.tree.ExpandAll()
        self.status = wx.StaticText(self, label="")
        self._mark()
        buttons = wx.BoxSizer(wx.HORIZONTAL)
        for label, handler in [("Set as origin", self._set_origin), ("Set as end", self._set_end)]:
            b = wx.Button(self, label=label)
            b.Bind(wx.EVT_BUTTON, lambda e, h=handler: h())
            buttons.Add(b, 0, wx.RIGHT, GAP)
        buttons.AddStretchSpacer()
        buttons.Add(wx.Button(self, wx.ID_CLOSE, "Close"), 0)
        self.Bind(wx.EVT_BUTTON, lambda e: self.Destroy(), id=wx.ID_CLOSE)
        s = wx.BoxSizer(wx.VERTICAL)
        s.Add(self.tree, 1, wx.EXPAND | wx.ALL, GAP)
        s.Add(self.status, 0, wx.LEFT | wx.RIGHT, GAP + 2)
        s.Add(buttons, 0, wx.EXPAND | wx.ALL, GAP)
        self.SetSizer(s)

    def _mark(self) -> None:
        for link, item in self.nodes.items():
            self.tree.SetItemBold(item, link in (self.origin, self.end))
        self.status.SetLabel(f"chain: {self.origin} → {self.end}")
        if self.end in self.nodes:
            self.tree.EnsureVisible(self.nodes[self.end])

    def _selected(self) -> Optional[str]:
        item = self.tree.GetSelection()
        return self.tree.GetItemData(item) if item.IsOk() else None

    def _set_origin(self) -> None:
        link = self._selected()
        if link:
            self.origin = link
            self._apply()

    def _set_end(self) -> None:
        link = self._selected()
        if link:
            self.end = link
            self._apply()

    def _apply(self) -> None:
        self.on_pick(self.origin, self.end)
        self._mark()


class CartesianTab(wx.ScrolledWindow):
    def __init__(self, parent: wx.Window, frame: "ToolboxFrame") -> None:
        super().__init__(parent, style=wx.VSCROLL)
        self.SetScrollRate(0, self.FromDIP(10))
        self.frame = frame

        # chain
        chain_box = wx.StaticBoxSizer(wx.VERTICAL, self, "Kinematic chain")
        sb = chain_box.GetStaticBox()
        row = wx.BoxSizer(wx.HORIZONTAL)
        self.chain_name = wx.Choice(sb, size=(self.FromDIP(150), -1), choices=[UNSAVED])
        self.chain_name.SetSelection(0)
        self.chain_name.SetToolTip("Saved chains (each has its own TCP), or (unsaved) for origin / end picked below")
        self.chain_name.Bind(wx.EVT_CHOICE, lambda e: self.chain_selected())
        row.Add(wx.StaticText(sb, label="Chain"), 0, wx.ALIGN_CENTER_VERTICAL | wx.RIGHT, 4)
        row.Add(self.chain_name, 0, wx.RIGHT, 6)
        for label, handler, tip in [("Save chain…", self.save_chain, "Name and save this origin → end; creates its TCP"),
                                    ("Delete", self.delete_chain, "Delete the selected chain")]:
            b = wx.Button(sb, label=label, style=wx.BU_EXACTFIT)
            b.SetToolTip(tip)
            b.Bind(wx.EVT_BUTTON, lambda e, h=handler: h())
            row.Add(b, 0, wx.RIGHT, GAP)
        chain_box.Add(row, 0, wx.ALL, GAP)
        row = wx.BoxSizer(wx.HORIZONTAL)
        self.origin = wx.Choice(sb, size=(self.FromDIP(120), -1))
        self.origin.SetToolTip("Chain start (origin): targets and new frames are relative to this link")
        self.origin.Bind(wx.EVT_CHOICE, lambda e: (self.mark_modified(), self.chain_changed()))
        self.end = wx.Choice(sb, size=(self.FromDIP(120), -1))
        self.end.SetToolTip("Chain end: the link that carries the TCP")
        self.end.Bind(wx.EVT_CHOICE, lambda e: (self.fill_origins(), self.mark_modified(), self.chain_changed()))
        tree_btn = wx.Button(sb, label="Tree…", style=wx.BU_EXACTFIT)
        tree_btn.SetToolTip("Show the kinematic tree and pick origin / end")
        tree_btn.Bind(wx.EVT_BUTTON, lambda e: self.show_tree())
        row.Add(self.origin, 0, wx.RIGHT, 4)
        row.Add(wx.StaticText(sb, label="→"), 0, wx.ALIGN_CENTER_VERTICAL | wx.RIGHT, 4)
        row.Add(self.end, 0, wx.RIGHT, 6)
        row.Add(tree_btn, 0)
        chain_box.Add(row, 0, wx.ALL, GAP)
        self.chain_text = wx.StaticText(sb, label="")
        chain_box.Add(self.chain_text, 0, wx.EXPAND | wx.LEFT | wx.RIGHT | wx.BOTTOM, GAP)

        # TCP
        tcp_box = wx.StaticBoxSizer(wx.VERTICAL, self, "TCP (tool centre point) in the end link")
        self.tcp_box = tcp_box.GetStaticBox()
        sb = self.tcp_box
        self.tcp = PoseEditor(sb)
        tcp_box.Add(self.tcp.sizer, 0, wx.ALL, GAP)
        row = wx.BoxSizer(wx.HORIZONTAL)
        for label, handler, tip in [("Apply TCP", self.apply_tcp, "Save this TCP offset in the selected chain"),
                                    ("Reset", self.reset_tcp, "TCP at the end link's origin"),
                                    ("Edit in Unity", self.edit_tcp_in_viewer,
                                     "Select the TCP in Unity's Scene view: drag it with Move (W) / Rotate (E); "
                                     "the new offset is saved here")]:
            b = wx.Button(sb, label=label)
            b.SetToolTip(tip)
            b.Bind(wx.EVT_BUTTON, lambda e, h=handler: h())
            row.Add(b, 0, wx.RIGHT, GAP)
        tcp_box.Add(row, 0, wx.LEFT | wx.RIGHT | wx.BOTTOM, GAP)

        # target
        target_box = wx.StaticBoxSizer(wx.VERTICAL, self, "Target")
        sb = target_box.GetStaticBox()
        row = wx.BoxSizer(wx.HORIZONTAL)
        self._target_ids: List[str] = []
        self.target_choice = wx.Choice(sb, size=(self.FromDIP(150), -1), choices=[CUSTOM])
        self.target_choice.SetSelection(0)
        self.target_choice.SetToolTip("A target (Unity: the scene's targets, under 'Targets' or attached to objects), "
                                      "or (custom) values below. Clicking a target in Unity selects it here")
        self.target_choice.Bind(wx.EVT_CHOICE, lambda e: self.target_changed())
        self.reference = wx.Choice(sb, choices=REFERENCE_LABELS)
        self.reference.SetSelection(0)
        self.reference.SetToolTip("Coordinates of the values below: the chain origin link, the robot, or the scene")
        self.reference.Bind(wx.EVT_CHOICE, lambda e: self.reference_changed())
        self._reference_key = "chain"
        self.position_only = wx.CheckBox(sb, label="Position only")
        self.position_only.SetToolTip("Ignore the TCP orientation")
        row.Add(self.target_choice, 0, wx.RIGHT, 4)
        row.Add(wx.StaticText(sb, label="in"), 0, wx.ALIGN_CENTER_VERTICAL | wx.RIGHT, 4)
        row.Add(self.reference, 0, wx.RIGHT, 8)
        row.Add(self.position_only, 0, wx.ALIGN_CENTER_VERTICAL)
        target_box.Add(row, 0, wx.ALL, GAP)
        self.target = PoseEditor(sb, on_edit=self.target_edited)
        target_box.Add(self.target.sizer, 0, wx.ALL, GAP)
        row = wx.BoxSizer(wx.HORIZONTAL)
        for label, handler, tip in [("Use current TCP", self.use_current, "Fill in where the TCP is now"),
                                    ("Check", self.check, "Check that the TCP can reach the target"),
                                    ("Move", self.move, "Move the TCP to the target")]:
            b = wx.Button(sb, label=label)
            b.SetToolTip(tip)
            b.Bind(wx.EVT_BUTTON, lambda e, h=handler: h())
            row.Add(b, 0, wx.RIGHT, GAP)
        row.Add(wx.StaticText(sb, label="Time"), 0, wx.ALIGN_CENTER_VERTICAL | wx.LEFT | wx.RIGHT, 4)
        self.duration = wx.TextCtrl(sb, value="auto", size=(self.FromDIP(44), -1))
        self.duration.SetToolTip("Seconds, or 'auto' (from Speed % and each joint's max velocity)")
        row.Add(self.duration, 0, wx.ALIGN_CENTER_VERTICAL)
        target_box.Add(row, 0, wx.LEFT | wx.RIGHT | wx.BOTTOM, GAP)
        self.result = wx.StaticText(sb, label="")
        target_box.Add(self.result, 0, wx.EXPAND | wx.LEFT | wx.RIGHT | wx.BOTTOM, GAP)

        # targets
        frames_box = wx.StaticBoxSizer(wx.VERTICAL, self, "Targets")
        sb = frames_box.GetStaticBox()
        row = wx.BoxSizer(wx.HORIZONTAL)
        for label, handler, tip in [("New from TCP…", self.new_from_tcp, "A new target where the TCP is now"),
                                    ("New from values…", self.new_from_values, "A new target at the values above"),
                                    ("Delete", self.delete_target, "Delete the selected target")]:
            b = wx.Button(sb, label=label)
            b.SetToolTip(tip)
            b.Bind(wx.EVT_BUTTON, lambda e, h=handler: h())
            row.Add(b, 0, wx.RIGHT, GAP)
        frames_box.Add(row, 0, wx.ALL, GAP)
        row = wx.BoxSizer(wx.HORIZONTAL)
        row.Add(wx.StaticText(sb, label="Ctrl+click targets:"), 0, wx.ALIGN_CENTER_VERTICAL | wx.RIGHT, 4)
        self.click_orientation = wx.Choice(sb, choices=CLICK_ORIENTATION_LABELS)
        self.click_orientation.SetSelection(0)
        self.click_orientation.SetToolTip("Orientation of targets made by Ctrl+click in Unity. Approach: z into the "
                                          "surface, x towards the robot. Surface: z out of the surface. Keep TCP "
                                          "orientation: only the position comes from the click")
        self.click_orientation.Bind(wx.EVT_CHOICE, lambda e: self.send_click_settings())
        row.Add(self.click_orientation, 0)
        frames_box.Add(row, 0, wx.LEFT | wx.RIGHT | wx.BOTTOM, GAP)
        self.attach = wx.CheckBox(sb, label="Ctrl+click in Unity attaches the new target to the clicked object")
        self.attach.SetToolTip("Off: new targets go under the scene's 'Targets' object. On: they become children "
                               "of the clicked object and move with it")
        self.attach.Bind(wx.EVT_CHECKBOX, lambda e: self.send_click_settings())
        frames_box.Add(self.attach, 0, wx.LEFT | wx.RIGHT | wx.BOTTOM, GAP)
        self.show = wx.CheckBox(sb, label="Show chain and TCP in the viewer (Unity)")
        self.show.SetValue(True)
        self.show.Bind(wx.EVT_CHECKBOX, lambda e: self.refresh_markers())
        frames_box.Add(self.show, 0, wx.LEFT | wx.RIGHT | wx.BOTTOM, GAP)

        s = wx.BoxSizer(wx.VERTICAL)
        for box in (chain_box, tcp_box, target_box, frames_box):
            s.Add(box, 0, wx.EXPAND | wx.LEFT | wx.RIGHT | wx.TOP, GAP)
        self.SetSizer(s)

    # ── state ────────────────────────────────────────────────────────────────

    @property
    def b(self) -> Backend:
        return self.frame.backend

    def chain_args(self):
        end = self.end.GetStringSelection()
        if not end:
            raise ValueError("no chain - the robot sent no kinematic tree")
        return end, self.origin.GetStringSelection() or None

    def selected_chain(self) -> Optional[str]:
        name = self.chain_name.GetStringSelection()
        return None if name in ("", UNSAVED) else name

    def fill_chains(self, select: Optional[str] = None) -> None:
        current = select if select is not None else self.chain_name.GetStringSelection()
        names = [UNSAVED] + sorted(self.b.chains())
        self.chain_name.Set(names)
        self.chain_name.SetStringSelection(current if current in names else UNSAVED)

    def chain_selected(self) -> None:
        """A saved chain was picked: show its origin and end."""
        name = self.selected_chain()
        if name is not None:
            c = self.b.chains().get(name, {})
            end, origin = c.get("end"), c.get("origin")
            if end and end not in self.end.GetStrings():
                self.end.Append(end)
            if end:
                self.end.SetStringSelection(end)
            self.fill_origins()
            if origin in self.origin.GetStrings():
                self.origin.SetStringSelection(origin)
        self.chain_changed()

    def mark_modified(self) -> None:
        """Origin / end changed by hand: it is no longer the saved chain (Save chain… saves it)."""
        name = self.selected_chain()
        if name is None:
            return
        c = self.b.chains().get(name, {})
        if (c.get("origin"), c.get("end")) != (self.origin.GetStringSelection(), self.end.GetStringSelection()):
            self.chain_name.SetStringSelection(UNSAVED)

    def ref(self) -> str:
        return REFERENCE_KEYS[max(0, self.reference.GetSelection())]

    def origin_link(self) -> str:
        _, base = self.chain_args()
        return base or (self.b.description or {}).get("root")

    def selected_target(self) -> Optional[str]:
        i = self.target_choice.GetSelection()
        return self._target_ids[i - 1] if 0 < i <= len(self._target_ids) else None

    def selected_frame(self) -> Optional[str]:
        """The selected target's frame name, for targets stored here (robots without scene targets)."""
        t = self.selected_target()
        return t[len("frame:"):] if t and t.startswith("frame:") else None

    def fill_targets(self, select: Optional[str] = None) -> None:
        current = select if select is not None else self.selected_target()
        items = self.b.targets_list()
        self._target_ids = [tid for tid, _ in items]
        self.target_choice.Set([CUSTOM] + [name for _, name in items])
        if current in self._target_ids:
            self.target_choice.SetSelection(self._target_ids.index(current) + 1)
        else:
            self.target_choice.SetSelection(0)

    def robot_changed(self) -> None:
        current = self.end.GetStringSelection()
        tools = self.b.tools()
        self.end.Set(tools)
        if current in tools:
            self.end.SetStringSelection(current)
        elif tools:
            self.end.SetSelection(0)
        self.fill_origins()
        self.fill_targets()
        self.send_click_settings()
        self.fill_chains()
        if self.selected_chain() is None and self.b.chains():
            self.chain_name.SetSelection(1)          # first saved chain
        self.chain_selected()

    def fill_origins(self) -> None:
        end = self.end.GetStringSelection()
        bases = self.b.bases_for(end) if end else []
        current = self.origin.GetStringSelection()
        self.origin.Set(bases)
        if current in bases:
            self.origin.SetStringSelection(current)
        elif bases:
            root = (self.b.description or {}).get("root")
            self.origin.SetStringSelection(root if root in bases else bases[0])

    def chain_changed(self) -> None:
        try:
            end, base = self.chain_args()
            links = self.b.chain_links(end, base)
            name = self.selected_chain()
            movable = self.b.chain(end, base, name).joint_names
            self.chain_text.SetLabel(" → ".join(links) + f"   ({len(movable)} joints)")
            self.tcp.set(*self.b.tcp_for(end, name))
            self.tcp_box.SetLabel(f"TCP of chain '{name}' (in {end})" if name
                                  else f"TCP in {end} (unsaved chain - Save chain… to keep it)")
        except Exception as exc:
            self.chain_text.SetLabel(str(exc))
        self.chain_text.Wrap(max(200, self.frame.GetClientSize().width - self.FromDIP(60)))
        if self.selected_target():
            self.target_changed(from_viewer=True)
        self.FitInside()
        self.Layout()
        self.refresh_markers()

    # ── markers ──────────────────────────────────────────────────────────────

    def refresh_markers(self) -> None:
        if not self.b.can_visualize:
            return
        if not self.show.GetValue():
            self.frame.run(self.b.clear_markers(), what="markers")
            return
        try:
            end, base = self.chain_args()
        except ValueError:
            return
        self.frame.run(self.b.show_markers(end, base, *self._marker_target(), self.selected_chain()), what="markers")

    def _marker_target(self):
        """(selected frame name, custom target in chain coordinates) for the markers."""
        if self.selected_target():
            return self.selected_frame(), None
        try:
            xyz, rpy = self.target.get()
            return None, self.b.convert(xyz, rpy, self.ref(), "chain", self.origin_link())
        except Exception:
            return None, None

    def edited_in_viewer(self, payload: Dict[str, Any]) -> None:
        """The user moved the TCP or a frame in the viewer (Unity's Move / Rotate tools): store the new pose."""
        try:
            end, _ = self.chain_args()
            changed = self.b.apply_edit(payload, end, self.selected_chain())
        except Exception as exc:
            self.frame.log(f"edit from the viewer: {exc}")
            return
        if changed == "tcp":
            xyz, rpy = self.b.tcp_for(end, self.selected_chain())
            self.tcp.set(xyz, rpy)
            where = f"chain '{self.selected_chain()}'" if self.selected_chain() else f"{end} (unsaved chain)"
            self.frame.log(f"TCP of {where} moved in the viewer: "
                           f"({', '.join(f'{v:.4f}' for v in xyz)}) m, "
                           f"({', '.join(f'{math.degrees(v):.1f}' for v in rpy)})°")
        elif changed and changed.startswith("frame:"):
            name = changed[len("frame:"):]
            self.frame.log(f"frame '{name}' moved in the viewer")
            if self.selected_target() == changed:
                self.target_changed(from_viewer=True)
                return
        else:
            return
        self.refresh_markers()

    def picked_in_viewer(self, item_id: Optional[str], announce: str = "picked in the viewer") -> None:
        """The user clicked / selected a target in the viewer: make it the target here."""
        if not item_id or not (item_id.startswith("frame:") or item_id.startswith("target:")):
            return
        if item_id not in self._target_ids:
            self.fill_targets()                      # e.g. just created with Ctrl+click
        if item_id not in self._target_ids:
            self._pending_pick = (item_id, announce)  # appears with the next report from the robot
            return
        self.fill_targets(select=item_id)
        self.target_changed(from_viewer=True)
        self.frame.log(f"target: {self.target_choice.GetStringSelection()} ({announce})")

    def live_update(self, _positions=None) -> None:
        """Every robot report: new / removed scene targets, and moved targets follow in the fields."""
        if not self.b.uses_scene_targets:
            return
        ids = [tid for tid, _ in self.b.targets_list()]
        if ids != self._target_ids:
            self.fill_targets()
        pending = getattr(self, "_pending_pick", None)
        if pending and pending[0] in self._target_ids:
            self._pending_pick = None
            self.picked_in_viewer(*pending)
            return
        tid = self.selected_target()
        if tid and not any(c.HasFocus() for c in self.target.ctrls.values()):
            try:
                xyz, rpy = self.b.target_pose(tid, self.ref(), self.origin_link())
            except Exception:
                return
            old_xyz, old_rpy = self.target.get()
            if max(abs(a - b) for a, b in zip(list(xyz) + list(rpy), list(old_xyz) + list(old_rpy))) > 1e-4:
                self.target.set(xyz, rpy)

    # ── TCP ──────────────────────────────────────────────────────────────────

    def apply_tcp(self) -> None:
        xyz, rpy = self.tcp.get()
        if self.selected_chain() is None:
            if wx.MessageBox("A TCP belongs to a saved chain.\n\nSave this chain now?", APP_NAME,
                             wx.YES_NO | wx.ICON_QUESTION, self) != wx.YES or not self.save_chain():
                return
        name = self.selected_chain()
        try:
            self.b.set_chain_tcp(name, xyz, rpy)
            self.frame.log(f"TCP of chain '{name}' saved")
        except Exception as exc:
            self.frame.log(f"TCP: {exc}")
            return
        self.chain_changed()

    # ── chains ───────────────────────────────────────────────────────────────

    def save_chain(self) -> bool:
        """Name and save origin → end (creates its TCP). True if saved."""
        try:
            end, origin = self.chain_args()
            origin = origin or (self.b.description or {}).get("root")
        except Exception as exc:
            self.frame.log(f"chain: {exc}")
            return False
        chains = self.b.chains()
        default = self.selected_chain() or next((n for n, c in chains.items()
                                                 if (c.get("origin"), c.get("end")) == (origin, end)), end)
        name = self.frame.ask_name("Save chain", f"Name for the chain {origin} → {end}:", set(chains), default,
                                   lambda n: f"A chain named '{n}' already exists "
                                             f"({chains[n].get('origin')} → {chains[n].get('end')})")
        if name is None:
            return False
        try:
            self.b.save_chain(name, origin, end)
        except Exception as exc:
            self.frame.log(f"chain: {exc}")
            return False
        xyz, rpy = self.b.tcp_for(end, name)
        self.frame.log(f"saved chain '{name}': {origin} → {end}, TCP at "
                       f"({', '.join(f'{v:.3f}' for v in xyz)}) m in {end}")
        self.fill_chains(select=name)
        self.chain_changed()
        return True

    def delete_chain(self) -> None:
        name = self.selected_chain()
        if name is None:
            wx.MessageBox("Select a saved chain first.", APP_NAME, wx.OK, self)
            return
        if wx.MessageBox(f"Delete chain '{name}' and its TCP?", APP_NAME,
                         wx.YES_NO | wx.NO_DEFAULT | wx.ICON_QUESTION, self) != wx.YES:
            return
        try:
            self.b.delete_chain(name)
            self.frame.log(f"deleted chain '{name}'")
        except Exception as exc:
            self.frame.log(f"delete chain: {exc}")
        self.fill_chains(select=UNSAVED)
        self.chain_changed()

    def edit_tcp_in_viewer(self) -> None:
        if not self.b.can_visualize:
            self.frame.log("the robot has no viewer (Unity robots do)")
            return
        try:
            end, base = self.chain_args()
        except ValueError as exc:
            self.frame.log(str(exc))
            return
        if not self.show.GetValue():
            self.show.SetValue(True)
        key = self.selected_chain() or end
        self.frame.run(self.b.show_markers(end, base, *self._marker_target(), self.selected_chain(),
                                           focus=f"tcp:{key}"),
                       lambda _: self.frame.log(f"TCP_{key} selected in Unity: drag it with Move (W) / Rotate (E)"),
                       "edit in Unity")

    def reset_tcp(self) -> None:
        self.tcp.set((0, 0, 0), (0, 0, 0))
        self.apply_tcp()

    # ── target ───────────────────────────────────────────────────────────────

    def target_changed(self, from_viewer: bool = False) -> None:
        tid = self.selected_target()
        if tid is None:
            self.refresh_markers()
            return
        try:
            self.target.set(*self.b.target_pose(tid, self.ref(), self.origin_link()))
            self.result.SetLabel(f"{self.target_choice.GetStringSelection()} in {self.reference.GetStringSelection()}")
        except Exception as exc:
            self.frame.log(f"target: {exc}")
        if not from_viewer:
            self.frame.run(self.b.select_target_in_viewer(tid), what="select in viewer")
        self.refresh_markers()

    def reference_changed(self) -> None:
        """Show the same pose in the newly chosen coordinates."""
        old, new = self._reference_key, self.ref()
        self._reference_key = new
        if self.selected_target():
            self.target_changed(from_viewer=True)
            return
        try:
            xyz, rpy = self.target.get()
            self.target.set(*self.b.convert(xyz, rpy, old, new, self.origin_link()))
        except Exception as exc:
            self.frame.log(f"coordinates: {exc}")

    def target_edited(self) -> None:
        if not self.target.setting and self.selected_target():
            self.target_choice.SetSelection(0)   # edited values are no longer the saved target

    def target_args(self):
        """Arguments for check / move: the target in chain-origin coordinates."""
        end, base = self.chain_args()
        xyz, rpy = self.target.get()
        xyz, rpy = self.b.convert(xyz, rpy, self.ref(), "chain", self.origin_link())
        return end, base, xyz, rpy, self.position_only.GetValue()

    def show_result(self, r: IkResult) -> None:
        if r.reachable:
            text = (f"✓ Reachable: {r.position_error * 1000:.2f} mm"
                    + ("" if self.position_only.GetValue() else f" / {math.degrees(r.rotation_error):.2f}°")
                    + f", {len(r.positions)} joints")
            if r.near_limits:
                text += f"; near a limit: {', '.join(r.near_limits)}"
        else:
            text = f"✗ {r.message}"
            if r.position_reachable is True:
                text += ("\nThe position alone is reachable - the orientation is what fails. Tick Position only, "
                         "or turn the target (Unity: Rotate tool, E).")
            elif r.position_reachable is False:
                text += "\nThe position itself is out of reach of this chain."
            hint = self.short_chain_hint(len(r.positions))
            if hint:
                text += "\n" + hint
        self.result.SetLabel(text)
        self.result.Wrap(max(200, self.frame.GetClientSize().width - self.FromDIP(60)))
        self.FitInside()
        self.Layout()

    def short_chain_hint(self, joints: int) -> Optional[str]:
        """Why a full-pose target may fail: fewer than 6 joints can't set position and orientation freely."""
        if joints >= 6 or self.position_only.GetValue():
            return None
        try:
            origin = self.origin_link()
            above = Chain.ancestors(self.b.description, origin)[1:]
        except Exception:
            above = []
        lower = f" (e.g. at '{above[0]}', which adds the joint at '{origin}')" if above else ""
        return (f"⚠ This chain has only {joints} joint{'s' if joints != 1 else ''}; a full position + orientation "
                f"target usually needs 6. Start the chain lower{lower}, or tick Position only.")

    def use_current(self) -> None:
        try:
            end, base = self.chain_args()
            xyz, rpy = self.b.tool_pose(end, base, self.selected_chain())
            self.target.set(*self.b.convert(xyz, rpy, "chain", self.ref(), self.origin_link()))
        except Exception as exc:
            self.frame.log(f"current TCP: {exc}")
            return
        self.target_choice.SetSelection(0)
        self.result.SetLabel(f"current TCP of {end} in {self.reference.GetStringSelection()}")
        self.refresh_markers()

    def check(self) -> None:
        try:
            args = self.target_args()
        except Exception as exc:
            self.frame.log(f"check: {exc}")
            return
        self.result.SetLabel("checking…")
        self.refresh_markers()
        self.frame.run(self.b.check_target_async(*args, chain_name=self.selected_chain()), self.show_result, "check")

    def move(self) -> None:
        try:
            args = self.target_args()
            text = self.duration.GetValue().strip().lower()
            duration = None if text in ("", "auto") else float(text)
            if duration is not None and duration <= 0:
                raise ValueError("time must be > 0 s")
        except Exception as exc:
            self.frame.log(f"move: {exc}")
            return
        self.result.SetLabel("solving…")
        self.refresh_markers()

        def done(r: IkResult) -> None:
            self.show_result(r)
            what = self.target_choice.GetStringSelection() if self.selected_target() else "target"
            self.frame.log(f"→ {what}" if r.reachable else f"not sent: {r.message}")
        self.frame.run(self.b.move_to_target(*args, duration=duration, speed=self.frame.jog.speed_fraction,
                                             chain_name=self.selected_chain()), done, "move")

    # ── targets ──────────────────────────────────────────────────────────────

    def send_click_settings(self) -> None:
        """Ctrl+click settings to the robot's viewer (orientation of new targets, attach to the clicked object)."""
        if not self.b.uses_scene_targets:
            return
        mode = CLICK_ORIENTATION_KEYS[max(0, self.click_orientation.GetSelection())]
        self.frame.run(self.b.set_click_orientation(mode), what="click target orientation")
        self.frame.run(self.b.set_attach_new_targets(self.attach.GetValue()), what="attach setting")

    def _new_target(self, title: str, xyz, rpy, reference: str, prefix: str = "target") -> None:
        names = {name for _, name in self.b.targets_list()}
        name = self.frame.ask_name(title, "Target name:", names, self.b.suggest_target_name(prefix),
                                   lambda n: f"A target named '{n}' already exists")
        if name is None:
            return

        def done(tid: str) -> None:
            self.fill_targets()
            self.picked_in_viewer(tid, announce="new")
        try:
            origin = self.origin_link()
        except Exception as exc:
            self.frame.log(f"target: {exc}")
            return
        self.frame.run(self.b.create_target(name, reference, xyz, rpy, origin), done, "new target")

    def new_from_tcp(self) -> None:
        try:
            end, base = self.chain_args()
            xyz, rpy = self.b.tool_pose(end, base, self.selected_chain())
        except Exception as exc:
            self.frame.log(f"target: {exc}")
            return
        self._new_target("New target from TCP", xyz, rpy, "chain", prefix=self.selected_chain() or end)

    def new_from_values(self) -> None:
        xyz, rpy = self.target.get()
        self._new_target("New target", xyz, rpy, self.ref())

    def delete_target(self) -> None:
        tid = self.selected_target()
        if tid is None:
            wx.MessageBox("Select a target first.", APP_NAME, wx.OK, self)
            return
        name = self.target_choice.GetStringSelection()
        if wx.MessageBox(f"Delete target '{name}'?", APP_NAME, wx.YES_NO | wx.NO_DEFAULT | wx.ICON_QUESTION,
                         self) != wx.YES:
            return

        def done(_) -> None:
            self.frame.log(f"deleted target '{name}'")
            self.target_choice.SetSelection(0)
            self.refresh_markers()
        self.frame.run(self.b.delete_target(tid), done, "delete target")

    def show_tree(self) -> None:
        if not self.b.description:
            self.frame.log("no kinematic tree from the robot")
            return
        end, base = self.end.GetStringSelection(), self.origin.GetStringSelection()

        def pick(origin: str, new_end: str) -> None:
            if new_end != self.end.GetStringSelection():
                if new_end not in self.end.GetStrings():
                    self.end.Append(new_end)
                self.end.SetStringSelection(new_end)
                self.fill_origins()
            if origin in self.origin.GetStrings():
                self.origin.SetStringSelection(origin)
            else:
                self.frame.log(f"'{origin}' is not above '{new_end}' - origin stays {self.origin.GetStringSelection()}")
            self.mark_modified()
            self.chain_changed()

        TreeDialog(self, self.b.description, base, end, pick).Show()


# ── Motion tab: programs and the external planner ─────────────────────────────

NEW_PROGRAM = "(new program)"
STEP_KINDS = [("pose", "Saved pose"), ("joints", "Current position (scope joints)"), ("target", "TCP target")]


class StepDialog(wx.Dialog):
    """Add / edit one program step: a saved pose, the current position, or a TCP target (joint or linear move)."""

    def __init__(self, parent: wx.Window, frame: "ToolboxFrame", scope: Dict[str, Any],
                 step: Optional[Dict[str, Any]] = None) -> None:
        super().__init__(parent, title="Motion step", style=wx.DEFAULT_DIALOG_STYLE | wx.RESIZE_BORDER)
        self.frame, self.b, self.scope = frame, frame.backend, scope
        step = dict(step or {"type": "pose", "speed": 0.5, "wait": 0.0})
        self.kind = wx.RadioBox(self, label="Step", choices=[label for _, label in STEP_KINDS],
                                majorDimension=1, style=wx.RA_SPECIFY_COLS)
        self.kind.SetSelection([k for k, _ in STEP_KINDS].index(step.get("type", "pose")))
        self.kind.Bind(wx.EVT_RADIOBOX, lambda e: self.update())
        self.pose = wx.Choice(self, choices=[p["name"] for p in self.b.list_poses()])
        self.targets = self.b.targets_list() if self.b.description else []
        self.target = wx.Choice(self, choices=[label for _, label in self.targets])
        self.move = wx.Choice(self, choices=["Joint move (IK at the target)", "Linear move (TCP in a straight line)"])
        self.move.SetToolTip("Joint move: smooth, always possible when the target is reachable.\n"
                             "Linear move: the TCP follows a straight line; fails if the line leaves the "
                             "reachable space or the arm would flip on the way.")
        self.move.Bind(wx.EVT_CHOICE, lambda e: self.update())
        self.position_only = wx.CheckBox(self, label="Position only (keep the TCP orientation free)")
        self.speed = wx.SpinCtrlDouble(self, min=1, max=100, inc=5, initial=50)
        self.speed_label = wx.StaticText(self, label="Speed %")
        self.wait = wx.SpinCtrlDouble(self, min=0, max=3600, inc=0.5, initial=float(step.get("wait", 0.0)))
        self.wait.SetToolTip("Seconds to wait after this step (paused time does not count)")

        if step.get("pose") in self.pose.GetStrings():
            self.pose.SetStringSelection(step["pose"])
        elif self.pose.GetCount():
            self.pose.SetSelection(0)
        ids = [tid for tid, _ in self.targets]
        if step.get("target") in ids:
            self.target.SetSelection(ids.index(step["target"]))
        elif ids:
            self.target.SetSelection(0)
        linear = step.get("move") == "linear"
        self.move.SetSelection(1 if linear else 0)
        self.position_only.SetValue(bool(step.get("position_only")))
        self._captured = dict(step.get("positions", {})) if step.get("type") == "joints" else None
        self.update()
        speed = float(step.get("speed", 0.1 if linear else 0.5))
        self.speed.SetValue(speed * 1000 if linear and step.get("type") == "target" else speed * 100)

        grid = wx.FlexGridSizer(cols=2, vgap=GAP, hgap=6)
        grid.AddGrowableCol(1, 1)
        for label, ctrl in [("Pose", self.pose), ("Target", self.target), ("Move", self.move),
                            ("", self.position_only), (self.speed_label, self.speed), ("Wait s", self.wait)]:
            grid.Add(label if isinstance(label, wx.Window) else wx.StaticText(self, label=label),
                     0, wx.ALIGN_CENTER_VERTICAL)
            grid.Add(ctrl, 1, wx.EXPAND)
        s = wx.BoxSizer(wx.VERTICAL)
        s.Add(self.kind, 0, wx.EXPAND | wx.ALL, GAP * 2)
        s.Add(grid, 1, wx.EXPAND | wx.LEFT | wx.RIGHT, GAP * 2)
        s.Add(self.CreateButtonSizer(wx.OK | wx.CANCEL), 0, wx.EXPAND | wx.ALL, GAP * 2)
        self.SetSizerAndFit(s)
        self.Bind(wx.EVT_BUTTON, self.on_ok, id=wx.ID_OK)

    @property
    def kind_key(self) -> str:
        return STEP_KINDS[self.kind.GetSelection()][0]

    def update(self) -> None:
        kind = self.kind_key
        linear = kind == "target" and self.move.GetSelection() == 1
        self.pose.Enable(kind == "pose")
        for w in (self.target, self.move, self.position_only):
            w.Enable(kind == "target")
        old_linear = self.speed_label.GetLabel().startswith("TCP")
        if linear != old_linear:
            self.speed_label.SetLabel("TCP mm/s" if linear else "Speed %")
            self.speed.SetRange(1, 2000 if linear else 100)
            self.speed.SetValue(100 if linear else 50)
            self.speed.SetToolTip("TCP speed along the line (joints slow it down where needed)" if linear
                                  else "Percent of each joint's max velocity")

    def on_ok(self, event: wx.CommandEvent) -> None:
        kind = self.kind_key
        if kind == "pose" and self.pose.GetSelection() < 0:
            wx.MessageBox("Save a pose first (Poses tab).", "Motion step", wx.OK | wx.ICON_WARNING, self)
            return
        if kind == "target" and self.target.GetSelection() < 0:
            wx.MessageBox("Create a target first (Tool & Targets tab).", "Motion step", wx.OK | wx.ICON_WARNING, self)
            return
        if kind == "target" and self.scope.get("kind") != "chain":
            wx.MessageBox("A TCP target needs a chain scope: pick a saved chain in Scope first.", "Motion step",
                          wx.OK | wx.ICON_WARNING, self)
            return
        if kind == "joints" and self._captured is None:
            try:
                joints = self.b.motion.scope_joints(self.scope)
            except Exception as exc:
                wx.MessageBox(str(exc), "Motion step", wx.OK | wx.ICON_WARNING, self)
                return
            self._captured = {n: self.b.positions[n] for n in joints if n in self.b.positions}
        event.Skip()

    def step(self) -> Dict[str, Any]:
        kind = self.kind_key
        out: Dict[str, Any] = {"type": kind, "wait": round(self.wait.GetValue(), 3)}
        linear = kind == "target" and self.move.GetSelection() == 1
        out["speed"] = round(self.speed.GetValue() / (1000.0 if linear else 100.0), 4)
        if kind == "pose":
            out["pose"] = self.pose.GetStringSelection()
        elif kind == "joints":
            out["positions"] = {k: round(v, 6) for k, v in (self._captured or {}).items()}
        else:
            tid, label = self.targets[self.target.GetSelection()]
            out.update(target=tid, label=label, move="linear" if linear else "joint",
                       position_only=self.position_only.GetValue())
        return out


class MotionTab(wx.ScrolledWindow):
    """Motion programs (steps run automatically, with pause / resume / stop / step / loop) on a scope - all joints,
    a group, a chain or one joint - and the external motion planner stream."""

    def __init__(self, parent: wx.Window, frame: "ToolboxFrame") -> None:
        super().__init__(parent, style=wx.VSCROLL)
        self.SetScrollRate(0, self.FromDIP(10))
        self.frame = frame
        self.b = frame.backend
        self.steps: List[Dict[str, Any]] = []
        self.scopes: List[Dict[str, Any]] = []

        # program + scope
        prog_box = wx.StaticBoxSizer(wx.VERTICAL, self, "Program")
        sb = prog_box.GetStaticBox()
        row = wx.BoxSizer(wx.HORIZONTAL)
        self.program = wx.Choice(sb, size=(self.FromDIP(170), -1), choices=[NEW_PROGRAM])
        self.program.SetSelection(0)
        self.program.Bind(wx.EVT_CHOICE, lambda e: self.load_program())
        row.Add(self.program, 0, wx.RIGHT, 6)
        for label, handler, tip in [("Save…", self.save_program, "Save the steps, scope and loop under a name"),
                                    ("Delete", self.delete_program, "Delete the selected program")]:
            btn = wx.Button(sb, label=label, style=wx.BU_EXACTFIT)
            btn.SetToolTip(tip)
            btn.Bind(wx.EVT_BUTTON, lambda e, h=handler: h())
            row.Add(btn, 0, wx.RIGHT, GAP)
        prog_box.Add(row, 0, wx.ALL, GAP)
        row = wx.BoxSizer(wx.HORIZONTAL)
        self.scope = wx.Choice(sb, size=(self.FromDIP(170), -1))
        self.scope.SetToolTip("Joints this motion owns: all, a group, a saved chain (needed for TCP targets) or one "
                              "joint. Other joints stay free for jogging or their own motion.")
        self.loop = wx.CheckBox(sb, label="Loop")
        self.loop.SetToolTip("Start again at step 1 after the last step, until Stop")
        self.blend = wx.CheckBox(sb, label="Smooth")
        self.blend.SetValue(True)
        self.blend.SetToolTip("Pass through the steps without stopping (one continuous move; the robot only slows "
                              "where a joint turns back). Steps with a wait still stop there.\n"
                              "Off: stop at every step.")
        row.Add(wx.StaticText(sb, label="Scope"), 0, wx.ALIGN_CENTER_VERTICAL | wx.RIGHT, 4)
        row.Add(self.scope, 0, wx.RIGHT, 10)
        row.Add(self.loop, 0, wx.ALIGN_CENTER_VERTICAL | wx.RIGHT, 8)
        row.Add(self.blend, 0, wx.ALIGN_CENTER_VERTICAL)
        prog_box.Add(row, 0, wx.LEFT | wx.RIGHT | wx.BOTTOM, GAP)

        self.list = wx.ListCtrl(sb, style=wx.LC_REPORT | wx.LC_SINGLE_SEL)
        self.list.SetMinSize(self.FromDIP(wx.Size(-1, 150)))
        for i, (title, width) in enumerate([("#", 28), ("Step", 190), ("Move", 60), ("Speed", 70), ("Wait", 45)]):
            self.list.InsertColumn(i, title, width=self.FromDIP(width))
        self.list.Bind(wx.EVT_LIST_ITEM_ACTIVATED, lambda e: self.edit_step())
        prog_box.Add(self.list, 1, wx.EXPAND | wx.LEFT | wx.RIGHT, GAP)
        row = wx.BoxSizer(wx.HORIZONTAL)
        for label, handler, tip in [("Add…", self.add_step, "Add a step after the selected one"),
                                    ("Edit…", self.edit_step, "Edit the selected step (or double-click it)"),
                                    ("Remove", self.remove_step, "Remove the selected step"),
                                    ("▲", lambda: self.move_step(-1), "Move the selected step up"),
                                    ("▼", lambda: self.move_step(+1), "Move the selected step down")]:
            btn = wx.Button(sb, label=label, style=wx.BU_EXACTFIT)
            btn.SetToolTip(tip)
            btn.Bind(wx.EVT_BUTTON, lambda e, h=handler: h())
            row.Add(btn, 0, wx.RIGHT, GAP)
        prog_box.Add(row, 0, wx.ALL, GAP)

        # run controls
        run_box = wx.StaticBoxSizer(wx.VERTICAL, self, "Run")
        sb = run_box.GetStaticBox()
        row = wx.BoxSizer(wx.HORIZONTAL)
        for label, handler, tip in [
                ("▶ Run", lambda: self.run(0), "Run the program from step 1"),
                ("Run from selected", lambda: self.run(self.selected_index() or 0), "Run from the selected step"),
                ("Step", lambda: self.run(self.selected_index() or 0, single=True), "Run only the selected step"),
                ("Pause", lambda: self.frame.run(self.b.motion.pause(), what="pause"),
                 "Pause the program and the planner stream (they slow down to a halt and hold)"),
                ("Resume", lambda: self.frame.run(self.b.motion.resume(), what="resume"), "Continue"),
                ("■ Stop", lambda: self.frame.run(self.b.motion.stop(), what="stop"),
                 "Stop the program and the planner stream")]:
            btn = wx.Button(sb, label=label, style=wx.BU_EXACTFIT)
            btn.SetToolTip(tip)
            btn.Bind(wx.EVT_BUTTON, lambda e, h=handler: h())
            row.Add(btn, 0, wx.RIGHT, GAP)
        run_box.Add(row, 0, wx.ALL, GAP)
        self.status = wx.StaticText(sb, label="idle")
        run_box.Add(self.status, 0, wx.EXPAND | wx.LEFT | wx.RIGHT | wx.BOTTOM, GAP)

        # external planner
        plan_box = wx.StaticBoxSizer(wx.VERTICAL, self, "External motion planner")
        sb = plan_box.GetStaticBox()
        self.planner_text = wx.StaticText(sb, label="")
        self.planner_text.SetToolTip("A planner program connects here and sends joint poses (e.g. one per frame). "
                                     "See examples/planner_example.py.")
        plan_box.Add(self.planner_text, 0, wx.EXPAND | wx.ALL, GAP)
        row = wx.BoxSizer(wx.HORIZONTAL)
        self.planner_speed = wx.SpinCtrl(sb, min=5, max=100, initial=100, size=(self.FromDIP(56), -1))
        self.planner_speed.SetToolTip("Max joint speed while following, in % of each joint's max velocity")
        row.Add(wx.StaticText(sb, label="Speed %"), 0, wx.ALIGN_CENTER_VERTICAL | wx.RIGHT, 3)
        row.Add(self.planner_speed, 0, wx.RIGHT, 8)
        for label, handler, tip in [("Follow planner", self.follow, "Drive the scope's joints from the planner's poses"),
                                    ("Stop following", lambda: self.frame.run(self.b.motion.stop_following(),
                                                                              what="planner"),
                                     "Stop forwarding planner poses (the joints slow down to a halt)")]:
            btn = wx.Button(sb, label=label, style=wx.BU_EXACTFIT)
            btn.SetToolTip(tip)
            btn.Bind(wx.EVT_BUTTON, lambda e, h=handler: h())
            row.Add(btn, 0, wx.RIGHT, GAP)
        plan_box.Add(row, 0, wx.LEFT | wx.RIGHT | wx.BOTTOM, GAP)

        s = wx.BoxSizer(wx.VERTICAL)
        s.Add(prog_box, 1, wx.EXPAND | wx.ALL, GAP)
        s.Add(run_box, 0, wx.EXPAND | wx.LEFT | wx.RIGHT | wx.BOTTOM, GAP)
        s.Add(plan_box, 0, wx.EXPAND | wx.LEFT | wx.RIGHT | wx.BOTTOM, GAP)
        self.SetSizer(s)
        self.b.motion.on_status = self.show_status
        self.timer = wx.Timer(self)                    # planner counters change without status events
        self.Bind(wx.EVT_TIMER, lambda e: self.show_planner(), self.timer)
        self.timer.Start(1000)
        self.show_planner()

    # data

    @property
    def scope_value(self) -> Dict[str, Any]:
        i = self.scope.GetSelection()
        return dict(self.scopes[i]) if 0 <= i < len(self.scopes) else {"kind": "all"}

    def current_program(self) -> Dict[str, Any]:
        return {"scope": self.scope_value, "steps": [dict(s) for s in self.steps], "loop": self.loop.GetValue(),
                "blend": self.blend.GetValue()}

    def robot_changed(self) -> None:
        self.refresh()

    def refresh(self) -> None:
        """Re-read scopes and programs (groups / chains may have changed in the other tabs)."""
        current = self.scope_value
        try:
            self.scopes = self.b.motion.scopes()
        except Exception:
            self.scopes = []
        from .motion import scope_label
        self.scope.Set([scope_label(s) for s in self.scopes])
        if current in self.scopes:
            self.scope.SetSelection(self.scopes.index(current))
        elif self.scopes:
            self.scope.SetSelection(0)
        name = self.program.GetStringSelection()
        names = sorted(self.b.motion.programs()) if self.b.robot is not None else []
        self.program.Set([NEW_PROGRAM] + names)
        self.program.SetStringSelection(name if name in names else NEW_PROGRAM)

    def fill_list(self, select: Optional[int] = None) -> None:
        self.list.DeleteAllItems()
        for i, step in enumerate(self.steps):
            linear = step.get("type") == "target" and step.get("move") == "linear"
            row = self.list.InsertItem(i, str(i + 1))
            self.list.SetItem(row, 1, self.b.motion.step_label(step))
            self.list.SetItem(row, 2, ("linear" if linear else "joint") if step.get("type") == "target" else "joint")
            speed = float(step.get("speed", 0.5))
            self.list.SetItem(row, 3, f"{speed * 1000:g} mm/s" if linear else f"{speed * 100:g} %")
            self.list.SetItem(row, 4, f"{float(step.get('wait', 0)):g} s" if step.get("wait") else "")
        if select is not None and 0 <= select < len(self.steps):
            self.list.Select(select)
            self.list.EnsureVisible(select)

    def selected_index(self) -> Optional[int]:
        i = self.list.GetFirstSelected()
        return i if i >= 0 else None

    # programs

    def load_program(self) -> None:
        name = self.program.GetStringSelection()
        if name == NEW_PROGRAM:
            self.steps = []
            self.loop.SetValue(False)
            self.blend.SetValue(True)
        else:
            p = self.b.motion.programs().get(name, {})
            self.steps = [dict(s) for s in p.get("steps", [])]
            self.loop.SetValue(bool(p.get("loop")))
            self.blend.SetValue(bool(p.get("blend", True)))
            scope = p.get("scope") or {"kind": "all"}
            if scope in self.scopes:
                self.scope.SetSelection(self.scopes.index(scope))
            else:
                self.frame.log(f"program '{name}': its scope {scope} no longer exists - pick one")
        self.fill_list()

    def save_program(self) -> None:
        if self.b.data_dir is None:
            self.frame.log("motion: no data folder - programs cannot be saved")
            return
        current = self.program.GetStringSelection()
        name = self.frame.ask_name("Save program", "Program name:", set(self.b.motion.programs()),
                                   "" if current == NEW_PROGRAM else current)
        if name is None:
            return
        try:
            self.b.motion.save_program(name, self.current_program())
        except Exception as exc:
            self.frame.log(f"save program: {exc}")
            return
        self.frame.log(f"saved program '{name}' ({len(self.steps)} steps)")
        self.refresh()
        self.program.SetStringSelection(name)

    def delete_program(self) -> None:
        name = self.program.GetStringSelection()
        if name == NEW_PROGRAM:
            return
        if wx.MessageBox(f"Delete program '{name}'?", "Motion", wx.YES_NO | wx.NO_DEFAULT | wx.ICON_QUESTION,
                         self) != wx.YES:
            return
        self.b.motion.delete_program(name)
        self.frame.log(f"deleted program '{name}'")
        self.refresh()
        self.load_program()

    # steps

    def add_step(self) -> None:
        with StepDialog(self, self.frame, self.scope_value) as dlg:
            if dlg.ShowModal() != wx.ID_OK:
                return
            step = dlg.step()
        i = self.selected_index()
        at = len(self.steps) if i is None else i + 1
        self.steps.insert(at, step)
        self.fill_list(at)

    def edit_step(self) -> None:
        i = self.selected_index()
        if i is None:
            return
        with StepDialog(self, self.frame, self.scope_value, self.steps[i]) as dlg:
            if dlg.ShowModal() != wx.ID_OK:
                return
            self.steps[i] = dlg.step()
        self.fill_list(i)

    def remove_step(self) -> None:
        i = self.selected_index()
        if i is not None:
            del self.steps[i]
            self.fill_list(min(i, len(self.steps) - 1))

    def move_step(self, delta: int) -> None:
        i = self.selected_index()
        if i is None or not 0 <= i + delta < len(self.steps):
            return
        self.steps[i], self.steps[i + delta] = self.steps[i + delta], self.steps[i]
        self.fill_list(i + delta)

    # running

    def run(self, start: int, single: bool = False) -> None:
        self.frame.run(self.b.motion.run(self.current_program(), start, single), what="motion")

    def follow(self) -> None:
        self.frame.run(self.b.motion.follow(self.scope_value, self.planner_speed.GetValue() / 100.0),
                       what="planner")

    def show_status(self, status: Dict[str, Any]) -> None:
        state, step, steps = status.get("state"), status.get("step"), status.get("steps", 0)
        text = state or "idle"
        if state != "idle" and step is not None:
            text += f" · step {step + 1}/{steps}" + (f" · loop {status['loop'] + 1}" if status.get("loop") else "")
            if self.list.GetItemCount() > step and self.selected_index() != step:
                self.list.Select(step)
                self.list.EnsureVisible(step)
        if status.get("message"):
            text += f"  -  {status['message']}"
        self.status.SetLabel(text)
        self.show_planner()

    def show_planner(self) -> None:
        p = self.b.motion.planner_status()
        if not p["url"]:
            self.planner_text.SetLabel("Planner endpoint not available (see the log)")
            return
        text = f"{p['url']}  ·  {p['clients']} connected"
        if p["following"]:
            text += f"  ·  following on {', '.join(p['scope'])}  ·  {p['poses']} poses"
        self.planner_text.SetLabel(text)


# ── main window ───────────────────────────────────────────────────────────────

class ToolboxFrame(wx.Frame):
    def __init__(self, backend: Backend) -> None:
        super().__init__(None, title="LogixPlan " +APP_NAME)
        if app_icons() is not None:
            self.SetIcons(app_icons())
        self.SetSize(self.FromDIP(wx.Size(500, 660)))
        self.SetMinSize(self.FromDIP(wx.Size(440, 380)))
        self.backend = backend
        self.robot_ids: List[str] = []
        self.log_lines: List[str] = []
        self.log_frame: Optional[LogFrame] = None
        self.keep_unity = False
        self._unity_handed = False
        self._unity_last_note = ""
        self.unity_timer = wx.Timer(self)
        self.Bind(wx.EVT_TIMER, lambda e: self.keep_unity_tick(), self.unity_timer)
        self._build_menu()
        self._build_ui()
        if unity_focus.AVAILABLE and wx.Config.Get().ReadBool("keep_unity_in_front", False):
            self.set_keep_unity(True, save=False)
        self.CreateStatusBar(2)
        self.SetStatusWidths([-3, -2])
        self.Bind(wx.EVT_CLOSE, self.on_close)

        b = backend
        b.on_log = self.log
        b.on_robots = self.on_robots
        b.on_robot_changed = self.on_robot_changed
        b.on_positions = lambda pos: (self.jog.update(pos), self.cart.live_update(pos))
        b.on_state = self.on_state
        b.on_selected = self.cart.picked_in_viewer
        b.on_edited = self.cart.edited_in_viewer
        self.SetStatusText(f"{b.endpoint}  ·  data: {b.data_dir or 'none (poses disabled)'}", 1)
        self.log(f"{APP_NAME} {__version__} listening on {b.endpoint} - start the robot (Unity: press Play)")
        # robots that connected before these handlers were set (the controller starts first) are not missed
        self.on_robots(b.online_robots())

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
        if unity_focus.AVAILABLE:
            self.unity_item = m.AppendCheckItem(
                wx.ID_ANY, "Keep &Unity in front\tCtrl+U",
                "Toolbox on top; while the robot moves, focus goes back to Unity so the Editor runs at full speed")
            self.Bind(wx.EVT_MENU, lambda e: self.set_keep_unity(self.unity_item.IsChecked()), self.unity_item)
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
        self.cart = CartesianTab(self.book, self)
        self.motion = MotionTab(self.book, self)
        self.book.AddPage(self.jog, "Joint jog")
        self.book.AddPage(self.poses, "Poses")
        self.book.AddPage(self.cart, "Tool && Targets")
        self.book.AddPage(self.motion, "Motion")
        # groups, chains, poses and targets change in the other tabs: re-read them when Motion is shown
        self.book.Bind(wx.EVT_NOTEBOOK_PAGE_CHANGED,
                       lambda e: (self.book.GetCurrentPage() is self.motion and self.motion.refresh(), e.Skip()))

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

    # ── keep Unity in front ──────────────────────────────────────────────────
    # The Unity Editor runs only about one frame every 2 s while it is not focused, so the robot moves in jumps while
    # the Toolbox has focus. With this option the Toolbox stays on top, and while the robot moves and the user has
    # left the mouse and keyboard alone for a moment, the focus goes back to Unity (see unity_focus.py).

    def set_keep_unity(self, on: bool, save: bool = True) -> None:
        self.keep_unity = on
        self.unity_item.Check(on)
        style = self.GetWindowStyle()
        self.SetWindowStyle(style | wx.STAY_ON_TOP if on else style & ~wx.STAY_ON_TOP)
        if on:
            self.unity_timer.Start(200)
            if unity_focus.find_unity_window() is None:
                self.log("Keep Unity in front: no Unity window found yet (it is looked up again while the robot moves)")
        else:
            self.unity_timer.Stop()
        if save:
            wx.Config.Get().WriteBool("keep_unity_in_front", on)
            wx.Config.Get().Flush()

    def robot_moving(self) -> bool:
        b = self.backend
        return bool(b.running_goals()) or b.motion.running or b.motion.planner_status()["following"]

    def keep_unity_tick(self) -> None:
        """While the robot moves: once the user leaves the Toolbox alone, hand the focus to Unity."""
        try:
            moving = self.keep_unity and self.robot_moving()
            if not moving:
                self._unity_handed = False              # log the next hand-off again
                return
            if not unity_focus.foreground_is(self.GetHandle()):       # Unity already, another app, or a dialog
                return
            if unity_focus.mouse_button_down() or unity_focus.seconds_since_input() < unity_focus.IDLE_BEFORE_FOCUS:
                return
            robot = self.backend.robot
            names = [robot.project, robot.stage] if robot is not None else []
            hwnd = unity_focus.find_unity_window(names)
            if not hwnd:
                self._unity_note("Keep Unity in front: no Unity window found")
                return
            if unity_focus.focus(hwnd):
                if not self._unity_handed:
                    self._unity_handed = True
                    self.log(f"Keep Unity in front: focus → {unity_focus.window_title(hwnd)[:60]}")
            else:
                self._unity_note("Keep Unity in front: Windows refused to switch to Unity - click into Unity instead")
        except Exception as exc:                        # never let the timer die silently
            self._unity_note(f"Keep Unity in front: {exc or type(exc).__name__}")

    def _unity_note(self, text: str) -> None:
        """Log a hand-off problem once per motion."""
        if text != self._unity_last_note:
            self._unity_last_note = text
            self.log(text)

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

    def ask_name(self, title: str, prompt: str, existing: set, default: str = "",
                 replace_note: Optional[Callable[[str], str]] = None) -> Optional[str]:
        """Prompt for a name; if it exists, ask before replacing (No = ask again). None if cancelled."""
        name = default
        while True:
            with wx.TextEntryDialog(self, prompt, title, name) as dlg:
                if dlg.ShowModal() != wx.ID_OK:
                    return None
                name = dlg.GetValue().strip()
            if not name:
                wx.MessageBox("The name must not be empty.", APP_NAME, wx.OK | wx.ICON_WARNING, self)
                continue
            if name in existing:
                what = replace_note(name) if replace_note else f"'{name}' already exists"
                answer = wx.MessageBox(f"{what}.\n\nReplace it?", title,
                                       wx.YES_NO | wx.CANCEL | wx.NO_DEFAULT | wx.ICON_QUESTION, self)
                if answer == wx.CANCEL:
                    return None
                if answer == wx.NO:
                    continue          # ask for another name
            return name

    def save_pose_dialog(self, group: Optional[str] = None) -> None:
        """Prompt for a name; confirm before replacing an existing pose. With a group, only its joints are saved."""
        if self.backend.robot is None:
            self.log("save pose: no robot connected")
            return
        if group:
            joints = self.backend.group_joints(group)
            prompt = f"Pose name for group '{group}' ({len(joints)} joints: {', '.join(joints)}):"
            default = ""
        else:
            prompt, default = "Pose name (all joints):", self.poses.selected() or ""
        name = self.ask_name("Save pose", prompt, set(self.poses.names()), default,
                             lambda n: ("This replaces the built-in home pose (all joints 0)" if n in self.poses.builtin
                                        else f"A pose named '{n}' already exists"))
        if name is None:
            return

        def done(path) -> None:
            self.log(f"saved pose '{name}'" + (f" for group '{group}'" if group else ""))
            self.poses.refresh()
        self.run(self.backend.save_pose(name, group), done, "save pose")

    def groups_dialog(self) -> None:
        if self.backend.robot is None:
            self.log("groups: no robot connected")
            return
        if self.backend.data_dir is None:
            self.log("groups: no data folder (groups are saved in the robot's groups.json)")
            return
        with GroupsDialog(self) as dlg:
            dlg.ShowModal()
            selected = dlg.last_saved
        self.jog.refresh_groups(selected)

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
        self.jog.refresh_groups()
        self.cart.robot_changed()
        self.motion.robot_changed()
        self.poses.refresh()
        self.Layout()

    def on_state(self, state: Dict[str, Any]) -> None:
        text = f"state: {state.get('state', '—')}"
        if state.get("pause_reason"):
            text += f" [{state['pause_reason']}]"
        if len(state.get("active") or []) > 1:
            text += f"  {len(state['active'])} goals"
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

    # one Toolbox per config (or per --url): a second start brings the running one to the front
    config_dir = os.environ.get("RC_CONFIG_DIR", "")
    if activate_running_instance(args.url or (os.path.normcase(os.path.abspath(config_dir)) if config_dir else "")):
        return 0
    set_windows_app_id()          # before any window exists
    app = wx.App()
    app.SetAppName(APP_NAME)
    follow_system_theme(app, args.theme)
    backend = Backend(post=wx.CallAfter, url=args.url, data_dir=args.data_dir)
    try:
        backend.start()
    except Exception as exc:
        if getattr(exc, "winerror", None) == 10048 or getattr(exc, "errno", None) in (98, 10048):
            hint = ("The port is used by another program - probably a Robotic Toolbox started with a different "
                    "config. Close it, or give this config another port.")
        else:
            hint = ("Set RC_CONFIG_DIR to the folder with remote_control.json, or start with "
                    "--url ws://0.0.0.0:8765/motion")
        wx.MessageBox(f"Could not start the controller:\n\n{exc}\n\n{hint}", APP_NAME, wx.OK | wx.ICON_ERROR)
        return 2
    frame = ToolboxFrame(backend)
    frame.Show()
    app.MainLoop()
    return 0
