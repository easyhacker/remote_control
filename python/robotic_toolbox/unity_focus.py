"""
"Keep Unity in front" (Windows): the Unity Editor deliberately runs only about one frame every 2 s while it is not the
focused window, so a robot simulated in the Editor moves in jumps whenever the Toolbox has focus. With this option the
Toolbox stays on top, and while the robot moves and the user has left the mouse and keyboard alone for a moment, the
focus goes back to Unity, which then runs at full speed. Built players are not throttled and do not need this.
"""
from __future__ import annotations

import os
import sys
from typing import List, Optional, Sequence

IDLE_BEFORE_FOCUS = 0.25       # s without mouse / keyboard input before handing focus to Unity
EDITOR_CLASS = "UnityContainerWndClass"    # the Unity Editor's main window
PLAYER_CLASS = "UnityWndClass"             # a built player's window

AVAILABLE = sys.platform.startswith("win")

if AVAILABLE:
    import ctypes
    from ctypes import wintypes

    _user32 = ctypes.windll.user32
    _kernel32 = ctypes.windll.kernel32
    _EnumProc = ctypes.WINFUNCTYPE(wintypes.BOOL, wintypes.HWND, wintypes.LPARAM)

    class _LastInput(ctypes.Structure):
        _fields_ = [("cbSize", wintypes.UINT), ("dwTime", wintypes.DWORD)]


def _text(hwnd: int, fn) -> str:
    buf = ctypes.create_unicode_buffer(512)
    fn(hwnd, buf, 512)
    return buf.value


def unity_windows() -> List[tuple]:
    """(hwnd, title, is_editor) of every visible Unity Editor / player main window."""
    if not AVAILABLE:
        return []
    found: List[tuple] = []

    @_EnumProc
    def visit(hwnd, _):
        if _user32.IsWindowVisible(hwnd):
            cls = _text(hwnd, _user32.GetClassNameW)
            if cls in (EDITOR_CLASS, PLAYER_CLASS):
                found.append((hwnd, _text(hwnd, _user32.GetWindowTextW), cls == EDITOR_CLASS))
        return True

    _user32.EnumWindows(visit, 0)
    return found


def find_unity_window(names: Sequence[str] = ()) -> Optional[int]:
    """The Unity window to focus: the one whose title names most of `names` (the robot's project and scene),
    Editors before players; None if Unity is not running."""
    wins = unity_windows()
    if not wins:
        return None
    keys = [n.lower() for n in names if n]

    def score(w):
        title = w[1].lower()
        return (sum(k in title for k in keys), w[2])

    return max(wins, key=score)[0]


def seconds_since_input() -> float:
    """Seconds since the last mouse / keyboard input anywhere on the desktop."""
    if not AVAILABLE:
        return 0.0
    info = _LastInput(ctypes.sizeof(_LastInput), 0)
    if not _user32.GetLastInputInfo(ctypes.byref(info)):
        return 0.0
    return ((_kernel32.GetTickCount() - info.dwTime) & 0xFFFFFFFF) / 1000.0


def mouse_button_down() -> bool:
    if not AVAILABLE:
        return False
    return any(_user32.GetAsyncKeyState(vk) & 0x8000 for vk in (0x01, 0x02, 0x04))   # left, right, middle


def foreground_is(hwnd: int) -> bool:
    return AVAILABLE and _user32.GetForegroundWindow() == hwnd


def window_title(hwnd: int) -> str:
    return _text(hwnd, _user32.GetWindowTextW) if AVAILABLE and hwnd else ""


def focus(hwnd: int) -> bool:
    """Bring a window to the front and give it the keyboard focus (restores it if minimised). True if it is in front
    afterwards. Windows may ignore a plain SetForegroundWindow; then the input of this thread and the window's thread is
    attached for the switch (allowed between cooperating apps while this app has the foreground)."""
    if not AVAILABLE or not hwnd:
        return False
    if _user32.IsIconic(hwnd):
        _user32.ShowWindow(hwnd, 9)          # SW_RESTORE: a minimised Editor is throttled too
    _user32.SetForegroundWindow(hwnd)
    if _user32.GetForegroundWindow() == hwnd:
        return True
    me = _kernel32.GetCurrentThreadId()
    fg_thread = _user32.GetWindowThreadProcessId(_user32.GetForegroundWindow(), None)
    target_thread = _user32.GetWindowThreadProcessId(hwnd, None)
    attached = [t for t in {fg_thread, target_thread} if t and t != me and _user32.AttachThreadInput(me, t, True)]
    try:
        _user32.BringWindowToTop(hwnd)
        _user32.SetForegroundWindow(hwnd)
        _user32.SetFocus(hwnd)
    finally:
        for t in attached:
            _user32.AttachThreadInput(me, t, False)
    return _user32.GetForegroundWindow() == hwnd


def own_process_window(hwnd: int) -> bool:
    if not AVAILABLE or not hwnd:
        return False
    pid = wintypes.DWORD()
    _user32.GetWindowThreadProcessId(hwnd, ctypes.byref(pid))
    return pid.value == os.getpid()
