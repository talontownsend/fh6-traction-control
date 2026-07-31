#!/usr/bin/env python3
"""FH6 realistic traction control -- controller PASSTHROUGH middleware.

A driving aid for a human at the wheel, fed by the game's Data Out
telemetry stream (UDP port 7777).

Physical pad (winmm/DirectInput) --> this program --> ViGEm virtual X360 pad --> FH6.
Everything passes through 1:1 (sticks, d-pad, buttons, brake); only the THROTTLE
(right trigger) is modulated, using the game's own per-wheel slip telemetry, like
a real car's switchable traction control:

    OFF     pure passthrough, TC never touches the trigger
    LOW     "sport" TC: intervenes only past 150% of the friction circle --
            lets the car slide and light the tires, never cuts below 30%
            throttle; clips burnout, not fun
    MEDIUM  rides the circle edge at exactly 100%: every bit of available grip
            used, none wasted as wheelspin -- theoretically-perfect TC
    HIGH    holds 85%: always inside the circle, guaranteed traction at all
            times, will launch-control a standing start
    CUSTOM  any setpoint you like, 30-250% (GUI spinner or --custom-target);
            PID gains are derived by interpolating the three anchor modes, so
            the latency stability margin carries over

The regulated signal is the game's own per-wheel COMBINED slip -- Forza's
normalized friction-circle utilization, where 1.0 = the tire is exactly at the
circle's edge (verified on the rig: the limit measures ~0.8-1.2 here). No car-
or track-specific calibration is needed: the telemetry is already in
friction-circle units, so the setpoints transfer to any car and surface.
Watching the whole circle (not longitudinal slip_ratio) is also the power-on
oversteer guard: cornering load closes the circle, so the throttle tightens
mid-corner exactly as it should. One measured note behind LOW's setpoint: pure
longitudinal force stays near-max out to slip ~3, so sliding at 150% still
accelerates hard.

Per-axle LONG-ONLY toggles (GUI buttons; CLI --long-only-front/--long-only-rear):
when set for an axle, TC regulates ONLY that axle's forward/backward slip
component (slip_ratio) and ignores its sideways slip. Rear long-only is
drift-friendly TC: your drift angle stops counting against the setpoint while
genuine rear wheelspin still gets cut. Deliberate trade-off: ignoring lateral
slip on an axle disables the mid-corner power-oversteer guard for that axle.

Control law: a PID on the regulated slip (per telemetry tick ~62.5 Hz):
    e = target - regulated_slip        (regulated slip = fwd/back slip_ratio on
                                        LONG-ONLY axles, |combined| otherwise)
    I += ki * e * dt                   ki_dn over target, ki_up under; clamped
                                       into [floor, 1] (anti-windup)
    u = clamp(I + kp*e - kd*max(0, d slip/dt), floor, 1)
    sent_throttle = user_throttle * u, reopening slew-limited to `rise`/s
The INTEGRAL term is what pins slip right at the target (zero steady-state
error). The gains are deliberately modest: that is what gives the loop margin
against the game's ~100 ms throttle->slip transport delay (hotter gains
limit-cycle into a 2.5 Hz throttle sawtooth -- verified in the delayed-plant
selftests; the derivative can NOT fix latency, its input is itself lagged).
The derivative acts only on rising slip near the limit -- a spike pre-cut that
never kicks the throttle open when slip collapses. Cuts are instant, reopening
is paced, and below 2 m/s a per-mode launch cap bounds the open-loop torque
the latency would otherwise deliver on a standing start.

DOUBLE-INPUT WARNING: Forza listens to ALL connected devices at once. Until the
physical pad is hidden, the game sees BOTH pads, and:
  - TC throttle cuts LEAK (your raw trigger reaches the game and overrides them);
  - a plain BACK tap reaches the game TWICE (your real press + this program's
    replayed tap -- the replay is auto-suppressed once a leak is detected);
  - the mode chord (BACK + d-pad) is NOT hidden from the game -- the game still
    receives the raw BACK hold and d-pad taps directly from the physical pad.
Fix = HidHide (works for this pad -- it is a DirectInput/HID device, the one
class HidHide reliably cloaks):
    1. Install HidHide (github.com/nefarius/HidHide), reboot.
    2. HidHide Configuration Client > Devices: tick the physical gamepad (hide it).
    3. Applications tab: add the python.exe you run this with, so THIS script
       can still read the pad -- plus the base install's python.exe if that
       one is a venv (the exe that actually opens the pad can be either).
       The GUI's "Whitelist python" button adds both for you.
    4. Leave the ViGEm virtual pad visible (never hide it).
NOTE: with the pad hidden, the game has NO controller whenever this program is
not running -- launch this first, then the game (or unhide in HidHide).
The built-in LEAK detector cross-checks the game's echoed accepted throttle
(telemetry `accel` 0..255) against what we sent during cuts and warns if the
physical pad is bleeding through. The GUI's Leak test runs a 6 s in-engine
probe (commands a known 30% while you hold the trigger, reads the echo) with
no pad teardown; the standalone `--leak-test` is the headless equivalent.

On exit (Ctrl+C / --duration / crash) the virtual pad disappears and FH6 raises
its Controller-Disconnected dialog, which only answers to keyboard Enter.
This program clears it automatically on shutdown by foregrounding FH6 and
sending Enter (disable with --no-dialog-clear).

ABS: the same law mirrored onto the BRAKE (left trigger). Watches ALL FOUR
wheels for lockup (negative slip ratio; same normalized friction circle), with
the same modes / CUSTOM % / per-axle long-only, its own PID + gain schedule,
floors that always leave real brake pressure (10-30% by mode), and a real-car
low-speed disengage below ~11 km/h so you can always brake to a complete stop
(slip telemetry also degenerates as v->0).

Mode switching (any time, no restart):
    keyboard TC: F5=OFF F6=LOW F7=MEDIUM F8=HIGH; ABS: F1-F4 likewise --
      accepted only while FH6 or this console is the foreground window
    controller chord: hold BACK/Share (or --chord-mod RB/LB/RS/LS), tap
      D-pad UP/DOWN (TC mode up/down) or
      RIGHT/LEFT (ABS mode up/down).
      Quick BACK taps (<0.4 s) still reach the game (replayed on release);
      holds >=0.4 s pass through live, so hold-bindings keep working.

Run:
    python .\\traction_control.py             # MEDIUM
    ... .\\traction_control.py --mode high
    ... .\\traction_control.py --map          # one-time interactive pad calibration
    ... .\\traction_control.py --leak-test    # verify TC cuts actually reach the game
    ... .\\traction_control.py --selftest     # offline unit tests (no game/pad needed)

Do NOT run at the same time as anything else bound to UDP 7777: a second
listener on the port silently starves this one of telemetry.
Tuning: data\\tc_tune.json is dumped at startup and hot-reloaded ~2x/s
(values are validated and clamped to sane ranges).
Log: data\\tc_log.csv (one row per telemetry frame while racing).
Record button (GUI): dumps the FULL telemetry stream (every game Frame field
plus what the middleware sent) to data\\recording_<timestamp>.csv until Stop.
"""
from __future__ import annotations

import argparse
import csv
import ctypes
import json
import math
import os
import socket
import sys
import threading
import time
import winreg
from collections import deque
from ctypes import wintypes
from dataclasses import dataclass

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from fh6_telemetry import parse_packet, Frame


def is_frozen() -> bool:
    """True when running from a PyInstaller build rather than source."""
    return bool(getattr(sys, "frozen", False))


def state_dir() -> str:
    """Where user state (recordings, tune, pad map, settings) lives.

    From source: a data\\ dir next to this file, so the project stays
    location-independent and the test suite writes where it always has.

    Frozen: %LOCALAPPDATA%\\FH6 TC. An installed app cannot write next to its
    own exe (Program Files is read-only without admin, and a one-file build
    unpacks to a temp dir that is deleted on exit, which would silently throw
    away every recording), and per-user state must survive reinstalls.
    """
    if is_frozen():
        base = os.environ.get("LOCALAPPDATA") or os.path.expanduser("~")
        return os.path.join(base, "FH6 TC")
    return os.path.join(os.path.dirname(os.path.abspath(__file__)), "data")


REC_DIR = state_dir()
PADMAP_PATH = os.path.join(REC_DIR, "tc_padmap.json")
TUNE_PATH = os.path.join(REC_DIR, "tc_tune.json")
LOG_PATH = os.path.join(REC_DIR, "tc_log.csv")

# full-telemetry recorder (the Record button): every raw game Frame field plus
# what the middleware did that tick, deduped to the true telemetry rate
REC_FIELDS = list(Frame.__annotations__.keys())
REC_EXTRA = ["wall_t", "sent_thr", "sent_brk", "tc_mode", "abs_mode",
             "tc_scale", "abs_scale",
             # ABS-learning evidence trail: what the learner saw and did on
             # THIS frame (verdict codes in AbsLearner.V_*), so on-rig
             # behavior can be audited from a recording alone
             "learn_on", "learn_ceiling", "learn_cap", "learn_alat",
             "learn_dref", "learn_verdict", "learn_brake_d", "learn_open"]


class TelemetryRecorder:
    """Writes full telemetry to a fresh CSV until closed.  Deduped by
    timestamp so a ~250 Hz control loop still yields one row per ~62.5 Hz
    telemetry frame."""

    def __init__(self, path: str):
        self.path = path
        self.file = open(path, "w", newline="")
        self.w = csv.writer(self.file)
        self.w.writerow(REC_FIELDS + REC_EXTRA)
        self.rows = 0
        self._last_ts = None

    def write(self, fr: Frame, extra: list) -> bool:
        if fr.timestamp_ms == self._last_ts:
            return False
        self._last_ts = fr.timestamp_ms
        self.w.writerow([getattr(fr, k) for k in REC_FIELDS] + extra)
        self.rows += 1
        if self.rows % 120 == 0:        # ~2 s: readable live, safe on crash
            self.file.flush()
        return True

    def close(self):
        try:
            self.file.flush()
            self.file.close()
        except Exception:
            pass


GAME_EXE_DEFAULT = "forzahorizon6.exe"

# ---------------------------------------------------------------------------
# winmm physical-pad reader (the user's pad is DirectInput-only: invisible to
# XInput). CAUTION: winmm CAN also enumerate the ViGEm virtual X360 pad (XUSB
# devices surface through the HID joystick layer), so every scan is identity-
# gated: we capture the physical pad's (wMid, wPid, name) at startup, reconnect
# only to a matching device, and never adopt the X360 USB identity.
# ---------------------------------------------------------------------------
winmm = ctypes.WinDLL("winmm")
user32 = ctypes.windll.user32
_kernel32 = ctypes.windll.kernel32


class JOYINFOEX(ctypes.Structure):
    _fields_ = [("dwSize", wintypes.DWORD), ("dwFlags", wintypes.DWORD),
                ("X", wintypes.DWORD), ("Y", wintypes.DWORD), ("Z", wintypes.DWORD),
                ("R", wintypes.DWORD), ("U", wintypes.DWORD), ("V", wintypes.DWORD),
                ("btns", wintypes.DWORD), ("btnNo", wintypes.DWORD), ("pov", wintypes.DWORD),
                ("r1", wintypes.DWORD), ("r2", wintypes.DWORD)]


class JOYCAPSW(ctypes.Structure):
    _fields_ = [("wMid", wintypes.WORD), ("wPid", wintypes.WORD),
                ("szPname", ctypes.c_wchar * 32),
                ("wXmin", wintypes.UINT), ("wXmax", wintypes.UINT),
                ("wYmin", wintypes.UINT), ("wYmax", wintypes.UINT),
                ("wZmin", wintypes.UINT), ("wZmax", wintypes.UINT),
                ("wNumButtons", wintypes.UINT),
                ("wPeriodMin", wintypes.UINT), ("wPeriodMax", wintypes.UINT),
                ("wRmin", wintypes.UINT), ("wRmax", wintypes.UINT),
                ("wUmin", wintypes.UINT), ("wUmax", wintypes.UINT),
                ("wVmin", wintypes.UINT), ("wVmax", wintypes.UINT),
                ("wCaps", wintypes.UINT), ("wMaxAxes", wintypes.UINT),
                ("wNumAxes", wintypes.UINT), ("wMaxButtons", wintypes.UINT),
                ("szRegKey", ctypes.c_wchar * 32),
                ("szOEMVxD", ctypes.c_wchar * 260)]


JOY_RETURNALL = 0x000000FF
AXIS_NAMES = ("X", "Y", "Z", "R", "U", "V")
VIGEM_X360_ID = (0x045E, 0x028E)     # USB identity the virtual pad presents


def joy_read(i: int) -> JOYINFOEX | None:
    info = JOYINFOEX()
    info.dwSize = ctypes.sizeof(JOYINFOEX)
    info.dwFlags = JOY_RETURNALL
    return info if winmm.joyGetPosEx(i, ctypes.byref(info)) == 0 else None


def joy_axes(j: JOYINFOEX) -> dict[str, int]:
    return {"X": j.X, "Y": j.Y, "Z": j.Z, "R": j.R, "U": j.U, "V": j.V}


def joy_identity(i: int) -> tuple | None:
    c = JOYCAPSW()
    if winmm.joyGetDevCapsW(ctypes.c_size_t(i), ctypes.byref(c), ctypes.sizeof(c)) == 0:
        return (c.wMid, c.wPid, c.szPname)
    return None


def find_pad(preferred: int | None = None, expect: tuple | None = None) -> int | None:
    """winmm id of the physical pad.

    Startup (expect=None): the preferred id if given (strict), else the first
    responding id whose USB identity is NOT the virtual X360 pad's.
    Reconnect (expect = identity captured at startup): first id matching that
    identity -- a replugged pad may re-enumerate at a new id, and the scan must
    never adopt this script's own ViGEm pad.
    """
    n = min(winmm.joyGetNumDevs(), 16)
    if expect is None:
        if preferred is not None:
            return preferred if joy_read(preferred) is not None else None
        for i in range(n):
            if joy_read(i) is None:
                continue
            ident = joy_identity(i)
            if ident is not None and (ident[0], ident[1]) == VIGEM_X360_ID:
                continue
            return i
        return None
    order = ([preferred] if preferred is not None else []) + \
        [i for i in range(n) if i != preferred]
    for i in order:
        if joy_read(i) is None:
            continue
        ident = joy_identity(i)
        # defense in depth: never match the virtual pad even if expect was
        # somehow poisoned with its identity
        if ident is None or (ident[0], ident[1]) == VIGEM_X360_ID:
            continue
        if ident == expect:
            return i
    return None


# winmm's szPname is the useless generic string below for most gamepads, and
# the real product name lives in a registry OEM key many HID pads don't set --
# so map the USB VID/PID (which winmm DOES report, reliably) to a friendly name.
_GENERIC_PNAME = "Microsoft PC-joystick driver"
_KNOWN_PADS = {
    (0x054C, 0x05C4): "Sony DualShock 4",
    (0x054C, 0x09CC): "Sony DualShock 4 v2",
    (0x054C, 0x0BA0): "Sony DualShock 4 USB dongle",
    (0x054C, 0x0CE6): "Sony DualSense",
    (0x054C, 0x0DF2): "Sony DualSense Edge",
    (0x045E, 0x028E): "Xbox 360 pad / ViGEm virtual",
    (0x045E, 0x02FF): "Xbox One Controller",
    (0x045E, 0x0B12): "Xbox Series Controller",
}
_KNOWN_VENDORS = {0x054C: "Sony", 0x045E: "Microsoft", 0x057E: "Nintendo",
                  0x28DE: "Valve", 0x046D: "Logitech"}


def _usb_friendly_name(vid: int, pid: int) -> str | None:
    """The device's real product name (the string HidHide shows), read from
    the Windows device registry by VID/PID -- winmm's wMid/wPid ARE the USB
    VID/PID.  Read-only; called only at pad-detect, never on the hot loop."""
    prefix = f"VID_{vid:04X}&PID_{pid:04X}"
    for base in (r"SYSTEM\CurrentControlSet\Enum\USB",
                 r"SYSTEM\CurrentControlSet\Enum\HID"):
        try:
            root = winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE, base)
        except OSError:
            continue
        try:
            i = 0
            while True:
                dev = winreg.EnumKey(root, i)
                i += 1
                if not dev.upper().startswith(prefix):
                    continue
                with winreg.OpenKey(root, dev) as devk:
                    j = 0
                    while True:
                        try:
                            inst = winreg.EnumKey(devk, j)
                        except OSError:
                            break
                        j += 1
                        try:
                            with winreg.OpenKey(devk, inst) as ik:
                                nm = winreg.QueryValueEx(ik, "FriendlyName")[0]
                            if nm and nm.strip():
                                return nm.strip()
                        except OSError:
                            pass
        except OSError:
            pass
        finally:
            winreg.CloseKey(root)
    return None


def _pad_name_fallback(ident) -> str:
    vid, pid, szp = ident
    known = _KNOWN_PADS.get((vid, pid))
    if known:
        return f"{known} ({vid:04X}:{pid:04X})"
    if szp and szp.strip() and szp != _GENERIC_PNAME:
        return szp                                   # winmm gave a real name
    vend = _KNOWN_VENDORS.get(vid)
    return (f"{vend} controller " if vend else "controller ") + \
        f"({vid:04X}:{pid:04X})"


def pad_display_name(ident) -> str:
    """Friendly name from a (wMid, wPid, szPname) identity tuple: the real
    device product name if the registry has it, else a VID/PID-based label."""
    if not ident:
        return "?"
    try:
        nm = _usb_friendly_name(ident[0], ident[1])
    except Exception:
        nm = None
    return nm or _pad_name_fallback(ident)


def list_pads() -> list[tuple[int, str]]:
    """All responding winmm ids (excluding the ViGEm virtual pad) with names."""
    out = []
    for i in range(min(winmm.joyGetNumDevs(), 16)):
        if joy_read(i) is None:
            continue
        ident = joy_identity(i)
        if ident is not None and (ident[0], ident[1]) == VIGEM_X360_ID:
            continue
        out.append((i, pad_display_name(ident)))
    return out


# ---------------------------------------------------------------------------
# XUSB report bits (standard X360 wButtons layout -- matches vgamepad's enum,
# duplicated here so --selftest/--map run without vgamepad installed)
# ---------------------------------------------------------------------------
XB = {"DPAD_UP": 0x0001, "DPAD_DOWN": 0x0002, "DPAD_LEFT": 0x0004, "DPAD_RIGHT": 0x0008,
      "START": 0x0010, "BACK": 0x0020, "LS": 0x0040, "RS": 0x0080,
      "LB": 0x0100, "RB": 0x0200, "GUIDE": 0x0400,
      "A": 0x1000, "B": 0x2000, "X": 0x4000, "Y": 0x8000}
DPAD_MASK = XB["DPAD_UP"] | XB["DPAD_DOWN"] | XB["DPAD_LEFT"] | XB["DPAD_RIGHT"]

# POV hat centidegrees -> dpad bits (diagonals set two bits)
POV_BITS = {0: XB["DPAD_UP"], 4500: XB["DPAD_UP"] | XB["DPAD_RIGHT"],
            9000: XB["DPAD_RIGHT"], 13500: XB["DPAD_DOWN"] | XB["DPAD_RIGHT"],
            18000: XB["DPAD_DOWN"], 22500: XB["DPAD_DOWN"] | XB["DPAD_LEFT"],
            27000: XB["DPAD_LEFT"], 31500: XB["DPAD_UP"] | XB["DPAD_LEFT"]}
POV_UPS = frozenset((0, 4500, 31500))        # any hat value containing UP
POV_DOWNS = frozenset((13500, 18000, 22500))  # any hat value containing DOWN

# Default map: a DualShock-style DirectInput pad (buttons verified by
# interactive mapping; axes are the standard DS4 winmm layout, and both are
# confirmed/overwritten by --map). A trigger's "button" is used ONLY when it
# has no analog axis (this pad's bits 6/7 are digital switches that close
# early in the trigger travel -- honoring them alongside the axis would make
# the triggers binary).
DEFAULT_PADMAP = {
    "buttons": {"0": "X", "1": "A", "2": "B", "3": "Y", "4": "LB", "5": "RB",
                "8": "BACK", "9": "START", "10": "LS", "11": "RS"},
    "sticks": {"lx": {"axis": "X", "min": 0, "max": 65535, "invert": False},
               "ly": {"axis": "Y", "min": 0, "max": 65535, "invert": True},
               "rx": {"axis": "Z", "min": 0, "max": 65535, "invert": False},
               "ry": {"axis": "R", "min": 0, "max": 65535, "invert": True}},
    "triggers": {"lt": {"axis": "U", "rest": 0, "full": 65535, "button": 6},
                 "rt": {"axis": "V", "rest": 0, "full": 65535, "button": 7}},
}


def load_padmap(emit=print) -> dict:
    try:
        with open(PADMAP_PATH) as f:
            m = json.load(f)
        emit(f"padmap: loaded {PADMAP_PATH}")
        return m
    except Exception:
        emit(f"padmap: {PADMAP_PATH} missing -> DS4-layout defaults "
             f"(run --map once if sticks/triggers feel wrong)")
        return json.loads(json.dumps(DEFAULT_PADMAP))


def axis_to_short(raw: int, lo: int, hi: int, invert: bool) -> int:
    """winmm axis -> XInput thumbstick short (-32768..32767)."""
    span = float(hi - lo) if hi != lo else 1.0
    v = (raw - lo) / span            # 0..1
    if invert:
        v = 1.0 - v
    return max(-32768, min(32767, int(round(v * 65535.0 - 32768.0))))


def axis_to_unit(raw: int, rest: int, full: int) -> float:
    """winmm trigger axis -> 0..1 (handles rest>full conventions)."""
    span = float(full - rest) if full != rest else 1.0
    return max(0.0, min(1.0, (raw - rest) / span))


def trig_byte(v: float) -> int:
    """0..1 -> clamped trigger byte (guards against any scale>1 escape wrapping
    the c_ubyte report field mod 256)."""
    return max(0, min(255, int(round(v * 255))))


# ---------------------------------------------------------------------------
# Rumble passthrough: with the physical pad hidden, the game's force feedback
# lands on the ViGEm virtual pad (no motors). We take the FFB notification
# from ViGEm and drive the DualSense's motors directly over raw HID output
# reports -- winmm is read-only and a DualSense is not an XInput device, so
# this is the only road back to vibration.
# ---------------------------------------------------------------------------
DS_VID = 0x054C
DS_PIDS = (0x0CE6, 0x0DF2)          # DualSense, DualSense Edge


def ds_usb_report(out_len: int, large: int, small: int,
                  rgb: tuple | None = None, player_leds: int | None = None,
                  setup: bool = False) -> bytes:
    """DualSense USB output report 0x02 (layout per the Linux
    hid-playstation driver): valid_flag0 bit0 = compatible rumble (old
    firmware) + bit1 = haptics select; valid_flag2 bit2 = compatible
    rumble on new firmware; motor_right (small/high-freq) BEFORE
    motor_left (large/low-freq). Optional light fields ride in the same
    report: lightbar RGB (vf1 bit2), player-LED mask (vf1 bit4), and the
    one-time lightbar-setup enable after connect (vf2 bit1)."""
    buf = bytearray(out_len)
    buf[0] = 0x02
    buf[1] = 0x03                    # COMPATIBLE_VIBRATION | HAPTICS_SELECT
    buf[3] = small & 0xFF            # motor_right
    buf[4] = large & 0xFF            # motor_left
    buf[39] = 0x04                   # COMPATIBLE_VIBRATION2 (new firmware)
    if setup:
        buf[39] |= 0x02              # LIGHTBAR_SETUP_CONTROL_ENABLE
        buf[42] = 0x02               # LIGHTBAR_SETUP_LIGHT_ON
    if player_leds is not None:
        buf[2] |= 0x10               # PLAYER_INDICATOR_CONTROL_ENABLE
        buf[44] = player_leds & 0x1F
    if rgb is not None:
        buf[2] |= 0x04               # LIGHTBAR_CONTROL_ENABLE
        buf[45], buf[46], buf[47] = rgb
    return bytes(buf)


def ds_bt_report(out_len: int, large: int, small: int, seq: int,
                 rgb: tuple | None = None, player_leds: int | None = None,
                 setup: bool = False) -> bytes:
    """DualSense Bluetooth output report 0x31: seq tag, 0x10 tag, the same
    common block as USB shifted by 2, CRC32 (seeded with 0xA2) over the
    first 74 bytes in the last 4."""
    import zlib
    buf = bytearray(out_len)
    buf[0] = 0x31
    buf[1] = (seq & 0x0F) << 4
    buf[2] = 0x10                    # DS_OUTPUT_TAG
    buf[3] = 0x03                    # valid_flag0
    buf[5] = small & 0xFF            # motor_right
    buf[6] = large & 0xFF            # motor_left
    buf[41] = 0x04                   # valid_flag2
    if setup:
        buf[41] |= 0x02
        buf[44] = 0x02
    if player_leds is not None:
        buf[4] |= 0x10
        buf[46] = player_leds & 0x1F
    if rgb is not None:
        buf[4] |= 0x04
        buf[47], buf[48], buf[49] = rgb
    crc = zlib.crc32(b"\xa2" + bytes(buf[:74])) & 0xFFFFFFFF
    buf[74:78] = crc.to_bytes(4, "little")
    return bytes(buf)


# The DualSense's 5 player LEDs are wired as THREE mirrored channels
# (rig-verified: bit0 = outer pair, bit1 = inner pair, bit2 = center;
# bits 3-4 dead), so a left-to-right bar is impossible. Encode the mode
# level as the NUMBER of lit LEDs instead, growing from the center out:
# OFF=1 (center), LOW=2 (inner), MED=3, HIGH=4, CUSTOM=5 (all).
LED_BAR = (0x04, 0x02, 0x06, 0x03, 0x07)


def light_state(t: float, tc_mode: str, tc_target: float,
                abs_mode: str, abs_target: float,
                tc_cutting: bool, abs_cutting: bool,
                chord_held: bool, chord_ch: str,
                tc_mode_i: int, abs_mode_i: int,
                flash_until: float, flash_ch: str) -> tuple:
    """The controller light language (pure logic, unit-tested).

    Lightbar: each armed channel maps its setpoint to a hue within its OWN
    range -- green = tight assistance, red = loose/permissive (TC 30-250%,
    ABS 30-600% with the live learned cap) -- and the base color is the
    blend of the armed channels. A channel actively CUTTING strobes the bar
    at ~5 Hz in its own color (ABS wins over TC: braking is the critical
    one; 8 Hz read as a little too quick). Holding the mode chord turns the
    bar WHITE (adjustment mode). Both channels OFF = white base.

    Player LEDs: normally dark; while chording, and for a moment after any
    mode change, they show the mode level as the NUMBER of lit LEDs growing
    from the center out (OFF=1 ... CUSTOM=5), for the channel being adjusted
    (the five LEDs are wired as three mirrored channels, so a left-to-right
    bar is not addressable -- see LED_BAR).
    """
    def chan(name, target, lo, hi):
        if name == "OFF":
            return None
        n = max(0.0, min(1.0, (target - lo) / (hi - lo)))
        return (int(round(255 * n)), int(round(255 * (1.0 - n))), 0)

    tc_c = chan(tc_mode, tc_target, 0.3, 2.5)
    ab_c = chan(abs_mode, abs_target, 0.3, 6.0)

    if chord_held:
        i = abs_mode_i if chord_ch == "abs" else tc_mode_i
        return (255, 255, 255), LED_BAR[max(0, min(i, 4))]
    pleds = 0
    if t < flash_until:
        i = abs_mode_i if flash_ch == "abs" else tc_mode_i
        pleds = LED_BAR[max(0, min(i, 4))]
    strobe_on = int(t * 10) % 2 == 0    # ~5 Hz
    if abs_cutting and ab_c is not None:
        return (ab_c if strobe_on else (0, 0, 0)), pleds
    if tc_cutting and tc_c is not None:
        return (tc_c if strobe_on else (0, 0, 0)), pleds
    if tc_c is None and ab_c is None:
        return (255, 255, 255), pleds
    if tc_c is None:
        return ab_c, pleds
    if ab_c is None:
        return tc_c, pleds
    return tuple((a + b) // 2 for a, b in zip(tc_c, ab_c)), pleds


class DualSenseRumble:
    """Background writer that forwards (large, small) motor values to the
    physical DualSense over HID. Resilient to the device being re-cycled
    (pnputil restart invalidates handles): a failed write closes and
    re-enumerates with backoff. HidHide keeps the device visible to this
    (whitelisted) process even while the game can't see it."""

    def __init__(self, emit=print):
        self.emit = emit
        self._large = 0
        self._small = 0
        self._kick = threading.Event()
        self._stop = False
        self._seq = 0
        self._handle = None
        self._out_len = 0
        self._thread = None
        self._said = None            # dedup for state-change messages
        self.n_events = 0            # FFB notifications from the game
        self.n_writes = 0            # successful HID writes to the pad
        # lights: with the pad hidden nothing else drives them; the engine's
        # light language paints them via set_lights(), and the light state
        # rides in EVERY report so rumble writes never clobber it
        self._rgb = (0, 0, 255)      # pre-engine default: classic blue
        self._pleds = 0x00           # player LEDs dark until a mode event
        self._setup_needed = False   # lightbar enable, once per (re)connect
        self._lights_dirty = True

    # -- HID plumbing (pure ctypes; no new dependencies) --------------------
    def _find_and_open(self) -> bool:
        setupapi = ctypes.windll.setupapi
        hid = ctypes.windll.hid
        k32 = ctypes.windll.kernel32
        # HANDLEs are 64-bit pointers: the ctypes default (32-bit int
        # restype) silently truncates them, corrupting every later call
        setupapi.SetupDiGetClassDevsW.restype = ctypes.c_void_p
        setupapi.SetupDiEnumDeviceInterfaces.argtypes = [
            ctypes.c_void_p, ctypes.c_void_p, ctypes.c_void_p,
            wintypes.DWORD, ctypes.c_void_p]
        setupapi.SetupDiGetDeviceInterfaceDetailW.argtypes = [
            ctypes.c_void_p, ctypes.c_void_p, ctypes.c_void_p,
            wintypes.DWORD, ctypes.c_void_p, ctypes.c_void_p]
        setupapi.SetupDiDestroyDeviceInfoList.argtypes = [ctypes.c_void_p]
        k32.CreateFileW.restype = ctypes.c_void_p
        k32.CloseHandle.argtypes = [ctypes.c_void_p]
        k32.WriteFile.argtypes = [ctypes.c_void_p, ctypes.c_char_p,
                                  wintypes.DWORD,
                                  ctypes.POINTER(wintypes.DWORD),
                                  ctypes.c_void_p]
        hid.HidD_GetAttributes.argtypes = [ctypes.c_void_p, ctypes.c_void_p]
        hid.HidD_GetPreparsedData.argtypes = [ctypes.c_void_p, ctypes.c_void_p]

        class GUID(ctypes.Structure):
            _fields_ = [("d1", wintypes.DWORD), ("d2", wintypes.WORD),
                        ("d3", wintypes.WORD), ("d4", ctypes.c_ubyte * 8)]

        class DIDATA(ctypes.Structure):
            _fields_ = [("cbSize", wintypes.DWORD), ("guid", GUID),
                        ("Flags", wintypes.DWORD),
                        ("Reserved", ctypes.c_void_p)]

        class HIDD_ATTR(ctypes.Structure):
            _fields_ = [("Size", wintypes.ULONG), ("VendorID", wintypes.USHORT),
                        ("ProductID", wintypes.USHORT),
                        ("VersionNumber", wintypes.USHORT)]

        guid = GUID(0x4D1E55B2, 0xF16F, 0x11CF,
                    (ctypes.c_ubyte * 8)(0x88, 0xCB, 0x00, 0x11,
                                         0x11, 0x00, 0x00, 0x30))
        hdev = setupapi.SetupDiGetClassDevsW(ctypes.byref(guid), None, None,
                                             0x12)   # PRESENT | DEVICEINTERFACE
        if hdev is None or hdev == ctypes.c_void_p(-1).value:
            return False
        try:
            idx = 0
            while True:
                did = DIDATA()
                did.cbSize = ctypes.sizeof(DIDATA)
                if not setupapi.SetupDiEnumDeviceInterfaces(
                        hdev, None, ctypes.byref(guid), idx, ctypes.byref(did)):
                    return False
                idx += 1
                need = wintypes.DWORD()
                setupapi.SetupDiGetDeviceInterfaceDetailW(
                    hdev, ctypes.byref(did), None, 0, ctypes.byref(need), None)
                buf = ctypes.create_string_buffer(need.value)
                # cbSize of the detail struct: DWORD + WCHAR alignment = 8 on x64
                ctypes.cast(buf, ctypes.POINTER(wintypes.DWORD))[0] = 8
                if not setupapi.SetupDiGetDeviceInterfaceDetailW(
                        hdev, ctypes.byref(did), buf, need, None, None):
                    continue
                path = ctypes.wstring_at(ctypes.addressof(buf) + 4)
                h = k32.CreateFileW(path, 0xC0000000, 0x3, None, 3, 0, None)
                if h is None or h == ctypes.c_void_p(-1).value:
                    continue
                attr = HIDD_ATTR()
                attr.Size = ctypes.sizeof(HIDD_ATTR)
                ok = hid.HidD_GetAttributes(h, ctypes.byref(attr))
                if (ok and attr.VendorID == DS_VID
                        and attr.ProductID in DS_PIDS):
                    prep = ctypes.c_void_p()
                    if hid.HidD_GetPreparsedData(h, ctypes.byref(prep)):
                        caps = (ctypes.c_ushort * 32)()
                        hid.HidP_GetCaps(prep, ctypes.byref(caps))
                        hid.HidD_FreePreparsedData(prep)
                        out_len = caps[3]        # OutputReportByteLength
                        if out_len >= 48:
                            self._handle = h
                            self._out_len = out_len
                            return True
                k32.CloseHandle(h)
        finally:
            setupapi.SetupDiDestroyDeviceInfoList(hdev)

    def _close(self) -> None:
        if self._handle is not None:
            try:
                ctypes.windll.kernel32.CloseHandle(self._handle)
            except Exception:
                pass
            self._handle = None

    def _write_raw(self, rep: bytes) -> bool:
        if self._handle is None:
            return False
        written = wintypes.DWORD()
        ok = ctypes.windll.kernel32.WriteFile(
            self._handle, rep, len(rep), ctypes.byref(written), None)
        return bool(ok) and written.value == len(rep)

    def _set_output_report(self, rep: bytes) -> bool:
        """Alternate delivery path: some HID stacks act on
        HidD_SetOutputReport when plain WriteFile is silently ignored."""
        if self._handle is None:
            return False
        hid = ctypes.windll.hid
        hid.HidD_SetOutputReport.argtypes = [ctypes.c_void_p, ctypes.c_char_p,
                                             wintypes.ULONG]
        return bool(hid.HidD_SetOutputReport(self._handle, rep, len(rep)))

    def _write(self, large: int, small: int) -> bool:
        setup, self._setup_needed = self._setup_needed, False
        if self._out_len >= 78:
            rep = ds_bt_report(self._out_len, large, small, self._seq,
                               rgb=self._rgb, player_leds=self._pleds,
                               setup=setup)
            self._seq = (self._seq + 1) & 0x0F
        else:
            rep = ds_usb_report(self._out_len, large, small,
                                rgb=self._rgb, player_leds=self._pleds,
                                setup=setup)
        ok = self._write_raw(rep)
        if ok:
            self._lights_dirty = False
        return ok

    def _say(self, msg: str) -> None:
        if msg != self._said:
            self._said = msg
            self.emit(msg)

    # -- public API ----------------------------------------------------------
    def start(self) -> None:
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def set(self, large: int, small: int) -> None:
        self._large = max(0, min(255, int(large)))
        self._small = max(0, min(255, int(small)))
        self.n_events += 1
        self._kick.set()

    def set_lights(self, rgb: tuple, player_leds: int) -> None:
        self._rgb = tuple(max(0, min(255, int(c))) for c in rgb)
        self._pleds = player_leds & 0x1F
        self._lights_dirty = True
        self._kick.set()

    def stop(self) -> None:
        self._stop = True
        self._kick.set()
        if self._thread is not None:
            self._thread.join(2.0)

    def _loop(self) -> None:
        last = (None, None)
        retry_at = 0.0
        while not self._stop:
            self._kick.wait(timeout=2.0)     # 2 s keepalive while rumbling
            self._kick.clear()
            if self._stop:
                break
            want = (self._large, self._small)
            if want == last and want == (0, 0) and not self._lights_dirty:
                continue                     # idle: nothing to refresh
            now = time.monotonic()
            if self._handle is None:
                if now < retry_at:
                    continue
                if self._find_and_open():
                    self._setup_needed = True
                    self._lights_dirty = True
                    self._say(f"[rumble] DualSense found "
                              f"({'BT' if self._out_len >= 78 else 'USB'}) -- "
                              f"vibration + lights restored")
                else:
                    retry_at = now + 3.0
                    self._say("[rumble] no DualSense HID found -- vibration "
                              "off (retrying in the background)")
                    continue
            if self._write(*want):
                last = want
                self.n_writes += 1
            else:
                self._close()                # device cycled/unplugged:
                retry_at = 0.0               # re-enumerate on the next kick
                self._say("[rumble] lost the DualSense handle -- reopening")
        # parting gift: motors off
        if self._handle is not None:
            try:
                self._write(0, 0)
            except Exception:
                pass
            self._close()


@dataclass
class PadState:
    """Physical pad decoded to logical Xbox controls."""
    buttons: int = 0          # XUSB wButtons bitmask (incl. dpad)
    lx: int = 0
    ly: int = 0
    rx: int = 0
    ry: int = 0
    lt: float = 0.0           # 0..1
    rt: float = 0.0           # 0..1
    pov: int = 65535          # raw POV (for the chord detector)


def decode_pad(j: JOYINFOEX, pm: dict) -> PadState:
    st = PadState()
    ax = joy_axes(j)
    for bit_s, name in pm["buttons"].items():
        if j.btns & (1 << int(bit_s)) and name in XB:
            st.buttons |= XB[name]
    st.pov = j.pov
    st.buttons |= POV_BITS.get(j.pov, 0)
    s = pm["sticks"]
    st.lx = axis_to_short(ax[s["lx"]["axis"]], s["lx"]["min"], s["lx"]["max"], s["lx"]["invert"])
    st.ly = axis_to_short(ax[s["ly"]["axis"]], s["ly"]["min"], s["ly"]["max"], s["ly"]["invert"])
    st.rx = axis_to_short(ax[s["rx"]["axis"]], s["rx"]["min"], s["rx"]["max"], s["rx"]["invert"])
    st.ry = axis_to_short(ax[s["ry"]["axis"]], s["ry"]["min"], s["ry"]["max"], s["ry"]["invert"])
    for name in ("lt", "rt"):
        t = pm["triggers"][name]
        if t.get("axis"):
            v = axis_to_unit(ax[t["axis"]], t["rest"], t["full"])
        else:
            # digital-only trigger hardware: button bit = full pull
            b = t.get("button")
            v = 1.0 if (b is not None and j.btns & (1 << int(b))) else 0.0
        setattr(st, name, v)
    return st


# ---------------------------------------------------------------------------
# Traction controller (pure logic -- no I/O, unit-tested by --selftest)
# ---------------------------------------------------------------------------
@dataclass
class ModeParams:
    name: str
    target: float    # setpoint: regulated driven-wheel slip (1.0 = grip limit)
    kp: float        # proportional gain (throttle per unit slip error)
    ki_up: float     # integral rate while UNDER target (reopening, 1/s)
    ki_dn: float     # integral rate while OVER target (cutting, 1/s)
    kd: float        # derivative gain on RISING slip only (spike pre-cut)
    rise: float      # max reopening rate of the final output (1/s); cuts are
                     # never rate-limited
    floor: float     # minimum throttle multiplier while intervening
    launch: float    # throttle cap while nearly stationary (<2 m/s): bounds
                     # the open-loop slip spike the actuation latency would
                     # otherwise let through on a standing start (1.0 = off)


# Friction-circle setpoints (no calibration needed -- the game normalizes
# combined slip so 1.0 = the grip limit). Order matters (chord cycles it).
# GAINS CARRY DELAY MARGIN: the game's throttle->slip path has ~100 ms of
# transport delay, and hotter kp/ki_dn (e.g. 0.55/4.0) limit-cycle against it
# (verified: 2.5 Hz slip sawtooth 0.45<->1.34 instead of pinning 1.0). These
# values hold p2p slip ~0 across 0-130 ms delay and fast/slow tire response.
# Do NOT try to fix latency with kd -- its input is itself lagged and raising
# it destabilizes; kd is only a spike pre-cut. If a very torquey car still
# judders, LOWER `rise` via tc_tune.json.
MODES = [
    ModeParams("OFF",    target=1e9,  kp=0.0,  ki_up=9.0, ki_dn=0.0, kd=0.0,  rise=9.0, floor=1.0,  launch=1.0),
    ModeParams("LOW",    target=1.5,  kp=0.15, ki_up=2.0, ki_dn=1.0, kd=0.02, rise=2.5, floor=0.30, launch=1.0),
    ModeParams("MEDIUM", target=1.0,  kp=0.20, ki_up=1.4, ki_dn=1.2, kd=0.03, rise=1.5, floor=0.08, launch=0.5),
    ModeParams("HIGH",   target=0.85, kp=0.27, ki_up=0.8, ki_dn=1.8, kd=0.04, rise=0.8, floor=0.04, launch=0.35),
    # CUSTOM: user-chosen setpoint; gains derived by apply_custom_target()
    ModeParams("CUSTOM", target=1.0,  kp=0.20, ki_up=1.4, ki_dn=1.2, kd=0.03, rise=1.5, floor=0.08, launch=0.5),
]
MODE_IDX = {m.name: i for i, m in enumerate(MODES)}

# ABS: the braking mirror. Same PID family and setpoints, but: floors always
# leave real brake pressure (10-30% by mode -- worst-case mis-measurement
# never strands you brakeless; note this game's brakes are strong, full pedal
# locks to slip 8-10, so on some cars the floor is what holds slip near the
# setpoint), reapply (ki_up/rise) is faster than TC's reopen, and there is no
# launch cap (the low-speed disengage in scale() covers stopping).
ABS_MODES = [
    ModeParams("OFF",    target=1e9,  kp=0.0,  ki_up=9.0, ki_dn=0.0, kd=0.0,  rise=9.0, floor=1.0,  launch=1.0),
    ModeParams("LOW",    target=1.5,  kp=0.15, ki_up=2.5, ki_dn=1.0, kd=0.02, rise=3.0, floor=0.30, launch=1.0),
    ModeParams("MEDIUM", target=1.0,  kp=0.20, ki_up=2.0, ki_dn=1.2, kd=0.03, rise=2.5, floor=0.15, launch=1.0),
    ModeParams("HIGH",   target=0.85, kp=0.27, ki_up=1.5, ki_dn=1.8, kd=0.04, rise=2.0, floor=0.10, launch=1.0),
    ModeParams("CUSTOM", target=1.0,  kp=0.20, ki_up=2.0, ki_dn=1.2, kd=0.03, rise=2.5, floor=0.15, launch=1.0),
]

CUSTOM_MIN = 0.3
CUSTOM_MAX = 2.5       # TC ceiling: beyond ~250% throttle slip is pure burnout
CUSTOM_MAX_ABS = 6.0   # ABS ceiling is much higher: TRUE wheel lock measures
                       # ~800-1000% in this game's normalized slip (locked
                       # wheels spike to 8-10), so 400-600% setpoints permit
                       # real, visible lock-ups before ABS steps in


def custom_max(modes) -> float:
    return CUSTOM_MAX_ABS if modes is ABS_MODES else CUSTOM_MAX


def apply_custom_target(target: float, modes: list | None = None) -> None:
    """Set a channel's CUSTOM setpoint and derive its PID gains by
    interpolating that channel's three verified anchor modes in target space
    (clamped at the ends).  The anchors carry the ~100 ms delay margin, so
    gains between them inherit it (verified by the delayed-plant selftest
    sweep over the CUSTOM range)."""
    modes = MODES if modes is None else modes
    cm = modes[MODE_IDX["CUSTOM"]]
    hi = modes[MODE_IDX["HIGH"]]
    med = modes[MODE_IDX["MEDIUM"]]
    low = modes[MODE_IDX["LOW"]]
    t = max(CUSTOM_MIN, min(custom_max(modes), target))
    cm.target = t
    if t <= hi.target:
        a, b, f = hi, hi, 0.0
    elif t <= med.target:
        a, b = hi, med
        f = (t - hi.target) / max(med.target - hi.target, 1e-6)
    elif t <= low.target:
        a, b = med, low
        f = (t - med.target) / max(low.target - med.target, 1e-6)
    else:
        a, b, f = low, low, 0.0
    f = max(0.0, min(1.0, f))
    for fld in ("kp", "ki_up", "ki_dn", "kd", "rise", "floor", "launch"):
        av, bv = getattr(a, fld), getattr(b, fld)
        setattr(cm, fld, av + (bv - av) * f)


LEAK_MIN_SAMPLES = 30          # ~0.5 s of throttle-applied telemetry


def leak_verdict(n_telem: int, echo: list,
                 commanded: float = 0.30) -> tuple[str, float, float]:
    """Decide a leak-test outcome from the telemetry-frame count and the
    game-accepted-throttle samples taken while the user was ON the throttle.
    Pure -> unit-tested.

    Returns (verdict, mean echo, leaking-frame fraction).  A leak is often
    INTERMITTENT -- the game alternates between our pad and the physical one
    frame by frame -- which barely moves the mean while still stealing
    control, so the duty cycle (not just the average) decides.

    Gating is on the NUMBER of usable samples, never on a duty cycle: the test
    asks the user to squeeze AND release, so what fraction of the window they
    spend on the throttle is a matter of rhythm, not of evidence.
    """
    if n_telem <= 0:
        return "notelem", 0.0, 0.0
    if len(echo) < LEAK_MIN_SAMPLES:
        return "held", 0.0, 0.0
    avg = sum(echo) / len(echo)
    over = sum(1 for e in echo if e > commanded + 0.25) / len(echo)
    if over > 0.05 or avg > 0.55:
        return "leaking", avg, over
    return "clean", avg, over


ABS_MIN_SPEED = 3.0   # m/s (~11 km/h): below this ABS disengages, like a real
                      # car -- slip telemetry blows up as v->0 and you must
                      # always be able to brake to a complete stop


class AbsLearner:
    """Adaptive ABS setpoint (the LEARN toggle).

    Learns the STRAIGHT-LINE braking-slip ceiling from deceleration evidence,
    then derates it live by the friction circle so trail-braking is capped by
    what the cornered tire can actually deliver:

        effective cap = ceiling * (1 - (lat/lat_max)^2)   (floored)

    Grip evidence is the TOTAL g-vector hypot(decel, lat) against a plateau
    reference, and the slip signal is the LONGITUDINAL (lockup-direction)
    component only -- so grip spent on minor steering corrections still
    counts as grip, and cornering can't inflate the slip reading. (A pure
    "straight-line frames only" gate starved real braking zones: at 200 mph
    even small corrections exceed 0.6g lateral.) Only genuinely deep
    cornering (> CORNER_LAT) is excluded: there the brake is a minority
    shareholder in the circle and decel says little about the ceiling.
    Bidirectional from evidence:
      - g-vector near plateau at slip past the ceiling -> room    -> up
      - g-vector collapsed at slip near the ceiling    -> falloff -> down
    Starts from whatever setpoint the USER had active. a_lat_max is learned
    from LOW-PASSED sustained lateral (one curb strike measured 8.9g vs ~4g
    sustained -- raw peaks must never inflate the budget).

    Validated against recorded laps: straight tarmac knee ~6-8, corner
    ceiling ~2.5-3 at ~3.7g lateral.
    """

    LAT_TAU = 0.30          # s low-pass on |lateral| (spike rejection)
    CORNER_LAT = 15.0       # m/s^2: exclude only deep cornering from ceiling
                            # learning; below this the g-vector credits the
                            # lateral share instead of gating the frame away
    REF_TAU = 0.8           # s of low-slip hard-brake samples (the plateau
                            # reference -- real braking dwells at slip <1).
                            # Fast on purpose: grip is not static -- driving
                            # from tarmac onto dirt dropped matched decel
                            # 3.65g -> 2.40g on-rig, and a stale-high ref
                            # scored the new surface's probes as falloff,
                            # pinning the ceiling below even the dirt knee
    REF_MIN_DECEL = 10.0    # m/s^2: brake-ramp frames (decel still building)
                            # must not dilute the plateau reference
    SPEED_BANDS = (25.0, 45.0)   # m/s: aero makes available grip strongly
                            # speed-dependent (3.7g at 300 km/h, ~2.8g at
                            # 100), so each band keeps its OWN reference --
                            # judging low-speed braking against a high-speed
                            # reference read as falloff and bled the ceiling
    SLIP_RECOV = -10.0      # 1/s: slip collapsing this fast is the wheel
                            # re-gripping after our own cut (the recovery
                            # rides on REDUCED torque; its low g is ours,
                            # not the surface's) -- never evidence
    BRAKE_FULL = 0.9        # evidence needs a ~FULL delayed pedal: at 65%
                            # pedal decel reads 65% of plateau WITH grip to
                            # spare -- partial-brake frames look exactly like
                            # falloff and bled the ceiling down on-rig
    MIN_SPEED = 12.0        # m/s: below ~43 km/h braking frames are spins,
                            # crawls and parking-lot noise, not knee physics
    SLIP_SANE = 25.0        # a slip-80 frame (spun car, wheels near-stopped)
                            # says nothing about any braking ceiling
    AY_MIN, AY_MAX = -4.0, 12.0  # m/s^2 healthy vertical band (Forza's ay is
                            # 0 at rest): below = wheels unloading/airborne
                            # (free-fall reads ~-9.8), above = crest-lip or
                            # landing compression. Braking over a jump reads
                            # EXACTLY like total grip collapse and crashed
                            # the ceiling 9->2 on-rig -- flight is not a
                            # surface
    DN_GAP = 1.5            # s between down-ratchet jumps: one bad patch
                            # must not compound pair-jumps into a crash
    DN_CONFIRM = 30.0       # s: a jump needs falloff pairs in TWO separate
                            # braking events. A knee change (tires/surface)
                            # collapses EVERY stop; a grass excursion is one
                            # event and must not crash the ceiling (a 21-min
                            # track session's offs ground 9 -> 1.2 on-rig)
    GRIP = 0.90             # g >= 90% of plateau at slip s: knee >= s. Not
                            # higher: probe windows carry a measured ~5-10%
                            # low bias (brake torque still ramping when the
                            # window opens), and 0.93 left 2/3 of real grip
                            # evidence stranded in the deadband on-rig
    FALL = 0.85             # g < 85% of plateau at slip s: knee < s
    T_UP = 0.10             # s of grip evidence to chase the ceiling upward
                            # (evidence only flows in the short cut-onset
                            # windows, so the pull per frame must be strong)
    T_DN = 0.4              # s of falloff evidence to pull it down
    C_MIN, C_MAX = 0.5, 20.0  # the ceiling is free to go as high as the car
                              # can prove: the flat curve past slip 11 is
                              # CAR-specific -- huge downforce on slicks
                              # pushes extremely high on the friction circle,
                              # and the same car on rally tires would learn a
                              # far lower knee (the ~150% offroad number is a
                              # TIRE number, not a surface one).
                              # A slicks car never sees gravel, so a slow
                              # dirt walk-down is not a real cost. C_MAX only
                              # stays under SLIP_SANE so garbage can't ratchet
    DERATE_FLOOR = 0.25     # cornering never removes more than 75% of the cap
    BUDGET_RISE = 6.0       # s toward sustained lateral excess (braking only)
    BUDGET_DECAY = 0.001    # 1/s toward BUDGET_BASE (so a tarmac-learned
    BUDGET_BASE = 25.0      # budget doesn't over-permit corners on dirt later)

    # per-frame verdict codes (recorded for on-rig evidence review):
    # GATED not an open-loop full-pedal probe / too slow / deep corner / spin
    # REF   accumulated into the plateau grip reference
    # RAMP  ref-band slip but decel still building (brake application)
    # NOREF evidence-zone slip but no reference for THIS speed band yet
    # DEAD  evidence between FALL and GRIP (85-90%): no move either way
    # GRIP  plateau g past the ceiling -> pulled UP
    # FALL  collapsed g under full torque -> pulled DOWN
    # RECOV slip collapsing: post-cut re-grip, not surface evidence
    # AIR   wheels unloaded (crest/jump/landing): slip and decel meaningless
    (V_GATED, V_REF, V_RAMP, V_NOREF, V_DEAD, V_GRIP, V_FALL,
     V_RECOV, V_AIR) = range(9)

    def __init__(self):
        self.reset(4.0)

    def reset(self, ceiling: float, alat_max: float = 35.0) -> None:
        self.ceiling = max(self.C_MIN, min(self.C_MAX, ceiling))
        self.alat_max = alat_max      # m/s^2, learned from lateral WHILE
        self.lat_f = 0.0              # braking (sweeper aero-lat is 8g+ here
                                      # and irrelevant to the braking circle)
        self.s_ref = [0.0, 0.0, 0.0]  # bias-corrected plateau-grip reference,
        self.w_ref = [0.0, 0.0, 0.0]  # one per speed band (aero knots)
        self.v2_ref = [0.0, 0.0, 0.0]  # matching EMA of v^2 per band, so the
                                      # knots sit at the speeds actually seen
        self._slip_prev = None        # for the re-grip (slip collapse) guard
        self._grip_prev = None        # last frame's GRIP slip (ratchet pair)
        self._fall_prev = None        # last frame's FALL slip (down-ratchet)
        self._t = 0.0                 # learner-local clock (sum of dt)
        self._dn_t = -1e9             # last down-ratchet time (DN_GAP limit)
        self._dn_pend = -1e9          # last falloff-pair time (confirmation)
        self.verdict = self.V_GATED   # what the last update() frame did

    def _band(self, speed: float) -> int:
        lo, hi = self.SPEED_BANDS
        return 0 if speed < lo else (1 if speed < hi else 2)

    def ref_decel(self, speed: float) -> float:
        """Plateau-grip reference AT THIS SPEED (0 until established).

        Downforce scales with v^2, so available grip is a line in v^2 --
        fit it through the populated band knots and evaluate at the frame's
        own speed. Hard band edges stalled learning on-rig: a >162 km/h
        band learned its reference at ~380 km/h (3.8g) and then condemned
        honest 170-230 km/h braking (~3.2g available) as sub-grip."""
        pts = [(self.v2_ref[b] / self.w_ref[b], self.s_ref[b] / self.w_ref[b])
               for b in range(3) if self.w_ref[b] > 0.2]
        if not pts:
            return 0.0
        if len(pts) == 1:
            return pts[0][1]
        n = len(pts)
        mx = sum(p[0] for p in pts) / n
        my = sum(p[1] for p in pts) / n
        den = sum((p[0] - mx) ** 2 for p in pts)
        k = max(0.0, sum((p[0] - mx) * (p[1] - my) for p in pts)
                / max(den, 1e-9))          # aero only ever ADDS grip
        x = min(max(speed * speed, min(p[0] for p in pts)),
                max(p[0] for p in pts))    # never extrapolate past the knots
        return my + k * (x - mx)

    def update(self, dt: float, slip: float, decel: float, lat: float,
               brake: float, speed: float, open_loop: bool = True,
               vert: float = 0.0) -> None:
        """Feed one telemetry frame. `brake` and `open_loop` must be
        DELAY-MATCHED (~0.12 s old): the frame's slip/decel are the surface's
        response to the torque sent one transport delay ago. open_loop means
        that torque was (near-)uncut -- only such frames probe the surface;
        regulated frames measure our own cut, not the tire (holding slip at
        a low cap needs a deep cut, so decel there is tiny NO MATTER how much
        grip the surface has -- reading it as evidence collapsed the ceiling
        100%->65% on-rig)."""
        # lateral low-pass; the budget learns only from BRAKING-context
        # lateral, rising slowly on sustained excess
        a = dt / (dt + self.LAT_TAU)
        self.lat_f += a * (lat - self.lat_f)
        if brake > 0.3 and self.lat_f > self.alat_max:
            self.alat_max += (self.lat_f - self.alat_max) * \
                min(1.0, dt / self.BUDGET_RISE)
        self.alat_max -= (self.alat_max - self.BUDGET_BASE) * \
            self.BUDGET_DECAY * dt
        self.alat_max = max(15.0, min(90.0, self.alat_max))
        # ceiling learning: only open-loop full-torque frames probe the
        # surface. The transport delay makes every cut ONSET such a probe:
        # slip overshoots the cap for ~8 frames while the wheels still see
        # full brake -- deep slip at plateau g (tarmac) is up-evidence, deep
        # slip at collapsed g (dirt) is down-evidence, and both are free on
        # every hard stop. Everything regulated/re-ramping is ignored.
        # slip trend (before any gate: the guard needs frame-to-frame state)
        rate = (0.0 if self._slip_prev is None
                else (slip - self._slip_prev) / max(dt, 1e-3))
        self._slip_prev = slip
        self._t += dt
        gp, self._grip_prev = self._grip_prev, None
        fp, self._fall_prev = self._fall_prev, None
        self.verdict = self.V_GATED
        if (not open_loop or brake < self.BRAKE_FULL or speed < self.MIN_SPEED
                or self.lat_f > self.CORNER_LAT or slip > self.SLIP_SANE):
            return
        if not (self.AY_MIN < vert < self.AY_MAX):
            # crest/jump/landing: the wheels aren't carrying the car, so
            # slip and decel describe the flight, not the surface
            self.verdict = self.V_AIR
            return
        # the tire doesn't care WHERE the grip goes: judge the total g-vector,
        # so braking through minor corrections reads as grip, not falloff
        g_tot = math.hypot(decel, lat)
        C = self.ceiling
        b = self._band(speed)
        if slip < min(2.0, max(0.7 * C, 0.6)):
            # ref band floored at 0.6: with a bottomed-out ceiling, 0.7*C
            # alone starves the reference channel (real braking dwells at
            # slip ~0.4-1), freezing a stale ref and blocking any recovery
            # plateau reference: the tire gripping well below the cap
            # (skip brake-application transients where decel is still rising)
            if decel > self.REF_MIN_DECEL:
                ab = dt / (dt + self.REF_TAU)
                self.s_ref[b] = (1.0 - ab) * self.s_ref[b] + ab * g_tot
                self.v2_ref[b] = (1.0 - ab) * self.v2_ref[b] + ab * speed ** 2
                self.w_ref[b] = (1.0 - ab) * self.w_ref[b] + ab
                self.verdict = self.V_REF
            else:
                self.verdict = self.V_RAMP
            return
        if rate < self.SLIP_RECOV:
            # slip collapsing = the wheel re-gripping after our cut; its low
            # g rides on reduced torque and says nothing about the surface
            self.verdict = self.V_RECOV
            return
        d_ref = self.ref_decel(speed)
        if d_ref < 5.0:
            self.verdict = self.V_NOREF
            return
        r = g_tot / d_ref
        self.verdict = self.V_DEAD
        # knee bracketing from individual samples (real braking slip is
        # bimodal -- it dwells at <1 then jumps to wherever the brake force
        # lands, so band-occupancy schemes starve; each sample out there is
        # direct evidence about the knee instead)
        if r >= self.GRIP and slip > C:
            # tire still pulling plateau g at slip s: knee is >= s. Ease
            # toward the evidence -- and when TWO consecutive probe frames
            # agree, jump straight to just under it: the evidence is already
            # quadruple-gated (full pedal, open loop, plateau g, not
            # recovering), and a low cap only gets a few probe frames per
            # stop, so easing alone costs laps of climbing
            self.ceiling = min(self.C_MAX, C + (min(slip, 2.0 * C) - C)
                               * min(1.0, dt / self.T_UP))
            if gp is not None:
                self.ceiling = max(self.ceiling,
                                   min(self.C_MAX, 0.95 * min(slip, gp)))
            self._grip_prev = slip
            self.verdict = self.V_GRIP
        elif r < self.FALL:
            # g collapsed under full torque at slip s: the knee is below s.
            # Pull toward 0.85*slip when the falloff is near the ceiling,
            # but never less than a gentle nudge -- deep-overshoot falloff
            # (dirt lock blows straight past the cap) must still walk the
            # ceiling down instead of being ignored
            # ease only on falloff NEAR the cap: collapse at slip far above
            # an already-lowered ceiling carries no information about
            # whether it should be lower still (an unconditional 0.95C
            # nudge bled the ceiling to the floor on sustained dirt)
            if 0.85 * slip < 1.05 * C:
                tgt = max(self.C_MIN, min(0.85 * slip, 0.95 * C))
                self.ceiling = max(self.C_MIN, C + (tgt - C)
                                   * min(1.0, dt / self.T_DN))
            if fp is not None:
                # two consecutive collapsed frames bound the knee from above
                # (lowest collapsed slip). Jumping requires CONFIRMATION: a
                # pair in an earlier, separate braking event within
                # DN_CONFIRM (persistent falloff = real knee change), plus
                # the DN_GAP rate limit; the jump is bounded to half the
                # ceiling. Unconfirmed pairs only arm the pending state.
                gap = self._t - self._dn_pend
                if (self._t - self._dn_t > self.DN_GAP
                        and self.DN_GAP < gap < self.DN_CONFIRM):
                    self.ceiling = min(self.ceiling,
                                       max(self.C_MIN, 0.85 * min(slip, fp),
                                           0.5 * C))
                    self._dn_t = self._t
                self._dn_pend = self._t
            self._fall_prev = slip
            self.verdict = self.V_FALL

    def effective_cap(self) -> float:
        # linear-in-circle-fraction derate: fits the measured corner-ceiling
        # shrink (0.42x at 3.7g lateral with a ~5g braking-context budget)
        fr = 1.0 - (self.lat_f / max(self.alat_max, 1e-3)) ** 2
        der = max(self.DERATE_FLOOR, fr)
        return max(CUSTOM_MIN, self.ceiling * der)


STALE_S = 0.30       # telemetry older than this -> passthrough
SLIP_TAU = 0.025     # EMA time constant on slip (s); ~1.5 telemetry ticks
K_REF = 3.0          # plant gain (slip per unit sent throttle) the MODES
                     # tunings are verified at; the online estimate below
                     # normalizes the loop gain to this reference


def driven_slips(f: Frame, override: int | None = None,
                 long_front: bool = False, long_rear: bool = False
                 ) -> tuple[float, float, float]:
    """(s_long, s_comb, s_reg) over the DRIVEN wheels.

    s_long = max positive longitudinal slip_ratio and s_comb = max |combined|
    (diagnostics); s_reg = the signal TC actually regulates: per wheel, ONLY
    the forward/backward component (slip_ratio) when its axle is in LONG-ONLY
    mode, else the full friction-circle |combined|.  Drivetrain comes from
    telemetry unless overridden (0=FWD 1=RWD 2=AWD).
    """
    dt_ = f.drivetrain if override is None else override
    if dt_ == 0:               # FWD
        per = ((True, f.slip_ratio_fl, f.combined_slip_fl),
               (True, f.slip_ratio_fr, f.combined_slip_fr))
    elif dt_ == 1:             # RWD
        per = ((False, f.slip_ratio_rl, f.combined_slip_rl),
               (False, f.slip_ratio_rr, f.combined_slip_rr))
    else:                      # AWD
        per = ((True, f.slip_ratio_fl, f.combined_slip_fl),
               (True, f.slip_ratio_fr, f.combined_slip_fr),
               (False, f.slip_ratio_rl, f.combined_slip_rl),
               (False, f.slip_ratio_rr, f.combined_slip_rr))
    # positive ratio only: wheelspin is TC's job, lockup (<0) is the brakes'
    s_long = max(0.0, max(r for _, r, _ in per))
    s_comb = max(abs(c) for _, _, c in per)
    s_reg = max((max(0.0, r) if (long_front if is_f else long_rear) else abs(c))
                for is_f, r, c in per)
    return s_long, s_comb, s_reg


def abs_slips(f: Frame, override: int | None = None,
              long_front: bool = False, long_rear: bool = False
              ) -> tuple[float, float, float]:
    """ABS channel: (s_long, s_comb, s_reg) over ALL FOUR wheels -- every
    wheel has a brake, so drivetrain is irrelevant (`override` accepted only
    for signature parity with driven_slips).  Lockup is NEGATIVE slip_ratio;
    wheelspin is the throttle's problem and is ignored here."""
    per = ((True, f.slip_ratio_fl, f.combined_slip_fl),
           (True, f.slip_ratio_fr, f.combined_slip_fr),
           (False, f.slip_ratio_rl, f.combined_slip_rl),
           (False, f.slip_ratio_rr, f.combined_slip_rr))
    s_long = max(0.0, max(-r for _, r, _ in per))
    s_comb = max(abs(c) for _, _, c in per)

    def _wheel(is_f, r, c):
        if long_front if is_f else long_rear:
            return max(0.0, -r)
        # |combined| is unsigned -- gated by actual lockup so throttle-side
        # wheelspin and pure cornering load never cut the BRAKE. The circle
        # term fades in over slip_ratio -0.05..-0.10 (no step at the
        # threshold, which would chatter during trail-braking at the limit).
        w = min(1.0, max(0.0, (-r - 0.05) / 0.05))
        return max(max(0.0, -r), w * abs(c))
    s_reg = max(_wheel(*p) for p in per)
    return s_long, s_comb, s_reg


class TractionController:
    """One regulation channel.  channel="tc" watches the DRIVEN wheels'
    wheelspin and modulates the throttle; channel="abs" watches ALL wheels'
    lockup and modulates the brake (same PID, same gain scheduling; ABS
    additionally disengages below ABS_MIN_SPEED and has no launch cap)."""

    def __init__(self, mode: str = "MEDIUM", channel: str = "tc",
                 modes: list | None = None):
        self.channel = channel
        self.modes = (MODES if channel == "tc" else ABS_MODES) \
            if modes is None else modes
        self._abs_roll = False   # ABS speed latch (hysteresis around the gate)
        self.target_override: float | None = None   # live setpoint override
        # (the ABS learner's derated cap); gains/floor stay the mode's own
        self.mode_i = MODE_IDX[mode.upper()]
        self.drivetrain_override: int | None = None   # None = auto from telemetry
        self.long_only_front = False  # regulate fwd/back slip only on the fronts
        self.long_only_rear = False   # ... and/or on the rears (drift-friendly)
        self.integ = 1.0         # PID integral state (the "held" throttle level)
        self.u_prev = 1.0        # last output (for the reopening slew limit)
        self.k_est = K_REF       # online plant-gain estimate (slip per sent
                                 # throttle); schedules the PID gains so a
                                 # torquey car can't push the loop unstable
        self._sent_hist: deque = deque()   # (t, sent) -- slip is compared to
                                 # the throttle from ~120 ms ago, matching the
                                 # actuation delay (current-vs-current biases
                                 # the estimate low and re-destabilizes)
        self.s_long = 0.0        # EMA-filtered driven slip
        self.s_comb = 0.0
        self.s_reg = 0.0         # the regulated signal (respects long-only flags)
        self.s_reg_rate = 0.0    # low-passed d(s_reg)/dt (the PID's D input)
        self.last_frame: Frame | None = None
        self.last_frame_t = -1e9
        self._last_ts_ms = None
        self.excess = 0.0
        self.active = False      # currently cutting
        self.interventions = 0   # cut-onset counter

    @property
    def mode(self) -> ModeParams:
        return self.modes[self.mode_i]

    def set_mode(self, name_or_idx) -> None:
        if isinstance(name_or_idx, str):
            self.mode_i = MODE_IDX[name_or_idx.upper()]
        else:
            self.mode_i = name_or_idx % len(MODES)
        # preserve any accumulated cut (clamped into the new mode's range):
        # switching modes mid-wheelspin must NOT snap the throttle open
        self.integ = max(self.mode.floor, min(1.0, self.integ))
        self.u_prev = max(self.mode.floor, min(1.0, self.u_prev))

    def on_frame(self, f: Frame, t: float) -> None:
        """Feed one telemetry frame (call at telemetry rate)."""
        if f.timestamp_ms == self._last_ts_ms:
            return                               # duplicate packet
        self._last_ts_ms = f.timestamp_ms
        extract = driven_slips if self.channel == "tc" else abs_slips
        raw_long, raw_comb, raw_reg = extract(
            f, self.drivetrain_override, self.long_only_front, self.long_only_rear)
        if self.last_frame is None or (t - self.last_frame_t) > STALE_S:
            # first frame / reconnect after a dropout: seed the filters so the
            # EMA step-from-zero can't masquerade as a slip-rate spike (a
            # phantom D cut on the very first sample)
            self.s_long, self.s_comb, self.s_reg = raw_long, raw_comb, raw_reg
            self.s_reg_rate = 0.0
            self.last_frame = f
            self.last_frame_t = t
            return
        dt = min(max(t - self.last_frame_t, 1e-3), 0.1)
        a = dt / (dt + SLIP_TAU)
        prev_reg = self.s_reg
        self.s_long += a * (raw_long - self.s_long)
        self.s_comb += a * (raw_comb - self.s_comb)
        self.s_reg += a * (raw_reg - self.s_reg)
        a2 = dt / (dt + 0.04)                       # D input: low-passed rate
        self.s_reg_rate += a2 * ((self.s_reg - prev_reg) / dt - self.s_reg_rate)
        self.last_frame = f
        self.last_frame_t = t

    def scale(self, dt: float, user_throttle: float, t: float) -> float:
        """Advance the controller by dt and return the throttle multiplier (0..1)."""
        m = self.mode
        f = self.last_frame
        if self.channel == "abs" and f is not None:
            # hysteresis: engage rolling above gate+1, release below the gate,
            # so speed jitter around 11 km/h can't flap full-brake <-> cut
            if f.speed_mps > ABS_MIN_SPEED + 1.0:
                self._abs_roll = True
            elif f.speed_mps < ABS_MIN_SPEED:
                self._abs_roll = False
        eligible = (m.name != "OFF"
                    and f is not None
                    and (t - self.last_frame_t) < STALE_S
                    and f.is_race_on == 1
                    and f.gear >= 1                    # not reverse
                    and user_throttle > 0.03           # (the channel's pedal)
                    and (self.channel == "tc" or self._abs_roll))
        if not eligible:
            # relax fully open so a menu pause / lift never leaves a stale cut
            self.integ = min(1.0, self.integ + max(m.ki_up, 3.0) * dt)
            self.u_prev = self.integ
            self._sent_hist.clear()
            self.excess = 0.0
            self.active = False
            return 1.0

        # gain scheduling: estimate the plant gain (slip per unit of throttle
        # we actually sent ~120 ms AGO -- matched to the actuation delay) and
        # scale the PID so the LOOP gain stays at the verified K_REF level
        # regardless of how torquey/grippy the car is -- fixed gains
        # limit-cycle once k_plant * kp outruns the 100 ms delay
        while self._sent_hist and (t - self._sent_hist[0][0] > 0.12
                                   or self._sent_hist[0][0] > t):
            self._sent_hist.popleft()
        sent_lag = self._sent_hist[0][1] if self._sent_hist else 0.0
        # quasi-static samples only: while slip is collapsing after a cut (or
        # exploding into one) the slip/sent ratio is transient, not plant gain
        if sent_lag > 0.02 and self.s_reg > 0.1:
            # robust update: every sample counts (skewed sampling biases the
            # estimate inside an oscillation), but one sample can pull at most
            # a factor of 2 -- transients can't poison the schedule
            k_raw = self.s_reg / sent_lag
            k_raw = max(self.k_est * 0.5, min(self.k_est * 2.0, k_raw))
            k_raw = max(0.2, min(12.0, k_raw))
            ak = dt / (dt + 0.3)
            self.k_est += ak * (k_raw - self.k_est)
        gs = max(0.25, min(1.25, K_REF / max(self.k_est, 0.2)))

        tgt = self.target_override if self.target_override is not None \
            else m.target
        e = tgt - self.s_reg                # + = grip headroom, - = over target
        if abs(e) < 1e-6:                   # kill float dust at the setpoint
            e = 0.0
        self.excess = -e                    # logged as before: + = over target
        # I: holds slip RIGHT AT the target (zero steady-state error);
        # asymmetric rates = cut faster than reopen; clamp = anti-windup
        self.integ += (m.ki_up if e >= 0.0 else m.ki_dn) * gs * e * dt
        self.integ = max(m.floor, min(1.0, self.integ))
        # D on RISING slip only, gated to "near the limit AND climbing fast":
        # a preemptive cut ahead of the actuation latency. The gates keep it
        # from nibbling during cruise flicker or while slip merely settles AT
        # the setpoint; a falling-slip kick would snap the throttle open
        # mid-recovery, hence max(0, ...).
        rising = (self.s_reg_rate
                  if self.s_reg > 0.6 * tgt and self.s_reg_rate > 2.0
                  else 0.0)
        u = self.integ + m.kp * gs * e - m.kd * gs * max(0.0, rising)
        u = max(m.floor, min(1.0, u))
        # reopening is paced, cutting is instant; the pace scales with the
        # schedule too -- on a torquey car the equilibrium throttle is small,
        # and an absolute reopen rate would pump the actuation delay
        u = min(u, self.u_prev + m.rise * gs * dt)
        if f.speed_mps < 2.0:
            # standing start: cap throttle until rolling -- the actuation
            # latency would otherwise deliver ~100 ms of open-loop torque
            # before the first cut can land (slip spikes to 4-12x target)
            self.integ = min(self.integ, m.launch)
            u = min(u, m.launch)
        self.u_prev = u
        self._sent_hist.append((t, u * user_throttle))
        if u < 0.98:
            if not self.active:
                self.interventions += 1
            self.active = True
        else:
            self.active = False
        return u


# ---------------------------------------------------------------------------
# Leak detector: during a sustained cut, the game's echoed accepted throttle
# (`accel` 0..255) should track what WE sent.  If it tracks the raw trigger
# instead, the physical pad is still visible to the game (install HidHide).
# ---------------------------------------------------------------------------
class LeakDetector:
    WINDOW_S = 0.6       # sustained-cut sample window
    ONSET_SKIP_S = 0.15  # ignore samples right after cut onset (echo latency)
    MIN_CUT = 0.25       # user-sent gap that counts as "cutting"
    MARGIN = 0.20        # echo may exceed sent by this much
    HOLD_S = 5.0         # leak flag ages out this long after the last positive

    def __init__(self, emit=print, what="throttle"):
        self.emit = emit
        self.what = what
        self.win_t0 = None
        self.samples: list[tuple[float, float]] = []   # (sent, echo01)
        self.last_leak_t = -1e9
        self.last_warn = -1e9
        self.ever_leaked = False
        self.stray_frac = 0.0        # fraction of frames the game took OUR
                                     # pad's value vs the physical one

    def leaking(self, t: float) -> bool:
        return (t - self.last_leak_t) < self.HOLD_S

    def feed(self, t: float, sent: float, user: float, echo255: int) -> None:
        if user - sent >= self.MIN_CUT:
            if self.win_t0 is None:
                self.win_t0 = t
                self.samples = []
            if t - self.win_t0 >= self.ONSET_SKIP_S:
                self.samples.append((sent, echo255 / 255.0))
            if t - self.win_t0 >= self.ONSET_SKIP_S + self.WINDOW_S and len(self.samples) >= 8:
                n = len(self.samples)
                sent_avg = sum(s for s, _ in self.samples) / n
                echo_avg = sum(e for _, e in self.samples) / n
                # count FRAMES the game took clearly more than we sent: a leak
                # is usually intermittent (the game flipping between our pad
                # and the physical one), which leaves the average deceptively
                # close to ours while still stealing control
                stray = sum(1 for s, e in self.samples if e > s + self.MARGIN)
                self.stray_frac = stray / n
                if (stray >= 3 and stray / n > 0.08) or echo_avg > sent_avg + self.MARGIN:
                    self.last_leak_t = t
                    self.ever_leaked = True
                    if t - self.last_warn > 10.0:
                        self.last_warn = t
                        self.emit(f"[LEAK] your pad reached the game on "
                                  f"{stray / n:.0%} of {self.what} frames "
                                  f"(took {echo_avg:.0%}, we sent {sent_avg:.0%}). "
                                  f"If Steam is running: disable Steam Input for "
                                  f"the game, then restart Steam.")
                self.win_t0 = t          # rolling re-evaluation
                self.samples = []
        else:
            self.win_t0 = None
            self.samples = []


# ---------------------------------------------------------------------------
# Mode-switch UI: F-keys (foreground-gated) + BACK-chord on the pad
# ---------------------------------------------------------------------------
VK_F5, VK_F6, VK_F7, VK_F8 = 0x74, 0x75, 0x76, 0x77
FKEY_MODE = {VK_F5: "OFF", VK_F6: "LOW", VK_F7: "MEDIUM", VK_F8: "HIGH"}
FKEY_MODE_ABS = {0x70: "OFF", 0x71: "LOW", 0x72: "MEDIUM", 0x73: "HIGH"}  # F1-F4
# (was F9-F12: Steam's screenshot key is F12, so selecting ABS HIGH also
# took a screenshot)


def beep_mode(idx: int) -> None:
    def _b():
        try:
            import winsound
            winsound.Beep(300 + 200 * idx, 90)
        except Exception:
            pass
    threading.Thread(target=_b, daemon=True).start()


def _window_exe(hwnd) -> str:
    """Basename of the executable that owns hwnd (lowercased), or ''.
    (Match by process, not window title.)"""
    pid = wintypes.DWORD()
    user32.GetWindowThreadProcessId(hwnd, ctypes.byref(pid))
    h = _kernel32.OpenProcess(0x1000, False, pid)    # PROCESS_QUERY_LIMITED_INFORMATION
    if not h:
        return ""
    try:
        buf = ctypes.create_unicode_buffer(1024)
        size = wintypes.DWORD(1024)
        if _kernel32.QueryFullProcessImageNameW(h, 0, buf, ctypes.byref(size)):
            return os.path.basename(buf.value).lower()
        return ""
    finally:
        _kernel32.CloseHandle(h)


def hotkeys_allowed(game_exe: str) -> bool:
    """Accept F-keys only when the game or this console is foreground, so a
    browser/IDE F5 in another window can't silently disable TC."""
    fg = user32.GetForegroundWindow()
    if fg and fg == _kernel32.GetConsoleWindow():
        return True
    exe = _window_exe(fg)
    return exe in (game_exe, "windowsterminal.exe")


POV_ACTION = POV_UPS | POV_DOWNS | {9000, 27000}   # hat values that chord-step

# selectable chord modifiers (BACK + dpad is a two-jobs-for-one-thumb claw;
# RB/LB/stick-clicks pair the chord across both hands)
CHORD_MODS = ("BACK", "RB", "LB", "RS", "LS")


class ChordSwitcher:
    """Hold the MODIFIER (default BACK) + tap D-pad UP/DOWN = TC mode,
    LEFT/RIGHT = ABS mode; the chord is swallowed.

    Modifier semantics: a quick tap (<TAP_MS, no chord) is buffered and
    replayed on release, so plain taps still reach the game; a plain hold
    >= TAP_MS commits to live passthrough (modifier forwarded held, chord
    detection disarmed) so hold-bindings keep working. While the modifier is
    held pre-commit, ALL d-pad bits are swallowed -- a hat passing through
    diagonals (0<->4500<->9000...) must not leak d-pad presses to the game
    mid-chord, and any hat value containing UP/DOWN counts as a chord step
    (one step per press: the hat must leave the UP/DOWN family before
    another step is accepted).
    """
    TAP_MS = 0.40

    def __init__(self, emit=print, mod: int = XB["BACK"]):
        self.emit = emit
        self.mod = mod
        self.back_down_t = None
        self.chorded = False        # a mode step happened this press
        self.committed = False      # long plain hold -> live passthrough
        self.step_armed = True      # hat must leave the action set between steps
        self._pend_act = None       # 2-sample confirmation: a hat grazing a
                                    # diagonal on its way to LEFT/RIGHT must
                                    # not misroute to a TC (UP/DOWN) step
        self.replay_until = 0.0
        self.suppress_replay = False  # set once a leak is latched: the game
                                      # already saw the real tap, don't double it
        self.last_ch = None           # channel stepped during THIS hold (the
                                      # light language shows its mode count)

    def process(self, st: PadState, t: float, tc: TractionController,
                abs_ctl: TractionController | None = None) -> int:
        """Returns the wButtons mask to actually forward."""
        btn = st.buttons
        back = bool(btn & self.mod)
        pov_up = st.pov in POV_UPS
        pov_down = st.pov in POV_DOWNS
        if st.pov not in POV_ACTION:
            self.step_armed = True
            self._pend_act = None

        if back:
            if self.back_down_t is None:
                self.back_down_t = t
                self.chorded = False
                self.committed = False
            if self.committed:
                return btn                   # live passthrough, chord disarmed
            act = (("tc", 1) if pov_up else ("tc", -1) if pov_down else
                   ("abs", 1) if st.pov == 9000 else
                   ("abs", -1) if st.pov == 27000 else None)
            if act is not None and self.step_armed:
                if act != self._pend_act:
                    self._pend_act = act       # first sighting: confirm next tick
                else:
                    self._pend_act = None
                    self.step_armed = False
                    self.chorded = True
                    self.last_ch = act[0]
                    target = tc if act[0] == "tc" else abs_ctl
                    if target is not None:
                        new_i = max(0, min(len(target.modes) - 1,
                                           target.mode_i + act[1]))
                        if new_i != target.mode_i:
                            target.set_mode(new_i)
                            beep_mode(new_i)
                            tag = "TC" if act[0] == "tc" else "ABS"
                            self.emit(f"[{tag}] mode -> {target.mode.name}")
            if not self.chorded and t - self.back_down_t >= self.TAP_MS:
                self.committed = True        # plain long hold: forward live
                return btn
            return btn & ~(self.mod | DPAD_MASK)
        # BACK released
        if self.back_down_t is not None:
            if not self.chorded and not self.committed and not self.suppress_replay:
                self.replay_until = t + 0.06     # deliver the tap now
            self.back_down_t = None
            self.chorded = False
            self.committed = False
            self.last_ch = None
        if t < self.replay_until:
            btn |= self.mod
        return btn


def poll_fkeys(prev: dict, tc: TractionController, game_exe: str, emit=print,
               abs_ctl: TractionController | None = None) -> None:
    maps = [(FKEY_MODE, tc, "TC")]
    if abs_ctl is not None:
        maps.append((FKEY_MODE_ABS, abs_ctl, "ABS"))
    for fmap, ctl, tag in maps:
        for vk, name in fmap.items():
            down = bool(user32.GetAsyncKeyState(vk) & 0x8000)
            edge = down and not prev.get(vk, False)
            prev[vk] = down
            if edge and ctl.mode.name != name and hotkeys_allowed(game_exe):
                ctl.set_mode(name)
                beep_mode(ctl.mode_i)
                emit(f"[{tag}] mode -> {name}")


# ---------------------------------------------------------------------------
# Console output OFF the control path: the loop only stores strings; a daemon
# thread prints. A frozen console (QuickEdit mark-mode click, Ctrl+S) then
# blocks only that thread -- the pad keeps passing through instead of the game
# being stuck on our last report.
# ---------------------------------------------------------------------------
class StatusWriter:
    def __init__(self):
        self.lock = threading.Lock()
        self.status = ""
        self.events: list[str] = []
        threading.Thread(target=self._run, daemon=True).start()

    def set_status(self, s: str) -> None:
        with self.lock:
            self.status = s

    def event(self, msg: str) -> None:
        with self.lock:
            if len(self.events) < 64:
                self.events.append(msg)

    def _run(self):
        while True:
            time.sleep(0.25)
            with self.lock:
                ev, self.events = self.events, []
                st = self.status
            for m in ev:
                print(f"\n{m}", flush=True)
            if st:
                print(f"\r{st}", end="", flush=True)


def disable_quickedit() -> None:
    """Clear console QuickEdit mode so a stray click-drag can't freeze stdout."""
    try:
        h = _kernel32.GetStdHandle(-10)              # STD_INPUT_HANDLE
        mode = wintypes.DWORD()
        if _kernel32.GetConsoleMode(h, ctypes.byref(mode)):
            _kernel32.SetConsoleMode(h, (mode.value | 0x0080) & ~0x0040)
    except Exception:
        pass


# ---------------------------------------------------------------------------
# FH6 exit-dialog clearing: when the virtual pad disappears, FH6 raises its
# Controller-Disconnected dialog, which only answers to keyboard Enter.
# We clear it ourselves on shutdown.
# ---------------------------------------------------------------------------
class _KBDINPUT(ctypes.Structure):
    _fields_ = [("wVk", ctypes.c_ushort), ("wScan", ctypes.c_ushort),
                ("dwFlags", ctypes.c_uint), ("time", ctypes.c_uint),
                ("dwExtraInfo", ctypes.POINTER(ctypes.c_ulong))]


class _KBDU(ctypes.Union):
    _fields_ = [("ki", _KBDINPUT)]


class _KBDIN(ctypes.Structure):
    _fields_ = [("type", ctypes.c_uint), ("u", _KBDU)]


def _send_enter() -> None:
    for flags in (0, 2):                             # down, up (KEYEVENTF_KEYUP)
        inp = _KBDIN(type=1, u=_KBDU(ki=_KBDINPUT(0x0D, 0, flags, 0, None)))
        user32.SendInput(1, ctypes.byref(inp), ctypes.sizeof(_KBDIN))
        time.sleep(0.05)


def find_game_window(game_exe: str):
    found = []

    @ctypes.WINFUNCTYPE(wintypes.BOOL, wintypes.HWND, wintypes.LPARAM)
    def _cb(h, _):
        if user32.IsWindowVisible(h) and _window_exe(h) == game_exe:
            found.append(h)
        return True

    user32.EnumWindows(_cb, 0)
    return found[0] if found else None


def _force_foreground(hwnd) -> None:
    try:
        fg = user32.GetForegroundWindow()
        ct = user32.GetWindowThreadProcessId(fg, None)
        tt = user32.GetWindowThreadProcessId(hwnd, None)
        user32.AttachThreadInput(ct, tt, True)
        user32.ShowWindow(hwnd, 9)                   # SW_RESTORE
        user32.BringWindowToTop(hwnd)
        user32.SetForegroundWindow(hwnd)
        user32.AttachThreadInput(ct, tt, False)
    except Exception:
        pass


def clear_disconnect_dialog(game_exe: str, emit=print) -> None:
    """After the virtual pad is destroyed: give FH6 a moment to raise the
    dialog, foreground it, and press Enter (the one input it answers to)."""
    time.sleep(1.5)
    hwnd = find_game_window(game_exe)
    if hwnd is None:
        return
    _force_foreground(hwnd)
    time.sleep(0.3)
    _send_enter()
    emit("[exit] sent Enter to FH6 to clear the controller-disconnected dialog")


# ---------------------------------------------------------------------------
# Tune-file hot reload (dump at start, re-read ~2x/s, missing keys keep
# current values, malformed file skipped).  Every value is validated: a
# typo like "medium_floor": 2.0 must never produce scale>1 (which would wrap
# the c_ubyte trigger report mod 256).
# ---------------------------------------------------------------------------
def _tunef(d: dict, key: str, cur: float, lo: float, hi: float) -> float:
    try:
        x = float(d.get(key, cur))
    except (TypeError, ValueError):
        return cur
    if not math.isfinite(x):
        return cur
    return max(lo, min(hi, x))


def dump_tune() -> None:
    try:
        os.makedirs(REC_DIR, exist_ok=True)
        d = {}
        for prefix, modes in (("", MODES), ("abs_", ABS_MODES)):
            for m in modes[1:]:
                p = prefix + m.name.lower()
                d.update({f"{p}_target": m.target, f"{p}_kp": m.kp,
                          f"{p}_ki_up": m.ki_up, f"{p}_ki_dn": m.ki_dn,
                          f"{p}_kd": m.kd, f"{p}_rise": m.rise,
                          f"{p}_floor": m.floor, f"{p}_launch": m.launch})
        json.dump(d, open(TUNE_PATH, "w"), indent=2)
    except Exception:
        pass


def reload_tune() -> None:
    try:
        with open(TUNE_PATH) as fh:
            t = json.load(fh)
    except Exception:
        return
    for prefix, modes in (("", MODES), ("abs_", ABS_MODES)):
        for m in modes[1:]:
            p = prefix + m.name.lower()
            m.target = _tunef(t, f"{p}_target", m.target, 0.2, 1e9)
            m.kp = _tunef(t, f"{p}_kp", m.kp, 0.0, 50.0)
            m.ki_up = _tunef(t, f"{p}_ki_up", m.ki_up, 0.1, 50.0)
            m.ki_dn = _tunef(t, f"{p}_ki_dn", m.ki_dn, 0.0, 50.0)
            m.kd = _tunef(t, f"{p}_kd", m.kd, 0.0, 5.0)
            m.rise = _tunef(t, f"{p}_rise", m.rise, 0.05, 50.0)
            m.floor = _tunef(t, f"{p}_floor", m.floor, 0.0, 0.95)
            m.launch = _tunef(t, f"{p}_launch", m.launch, 0.05, 1.0)


# ---------------------------------------------------------------------------
# Telemetry thread: drain to freshest packet, publish latest Frame
# ---------------------------------------------------------------------------
class TelemetryFeed:
    def __init__(self, port: int):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("0.0.0.0", port))
        self.sock.settimeout(0.5)
        self.lock = threading.Lock()
        self.frame: Frame | None = None
        self.t = -1e9
        self.n = 0
        self._stop = False
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _run(self):
        while not self._stop:
            try:
                data, _ = self.sock.recvfrom(2048)
            except socket.timeout:
                continue
            except OSError:
                break
            # drain to freshest (close() may race us: any OSError = shutdown)
            try:
                self.sock.setblocking(False)
                try:
                    while True:
                        try:
                            data, _ = self.sock.recvfrom(2048)
                        except BlockingIOError:
                            break
                finally:
                    self.sock.settimeout(0.5)
            except OSError:
                break
            f = parse_packet(data)
            if f is not None:
                with self.lock:
                    self.frame = f
                    self.t = time.perf_counter()
                    self.n += 1

    def latest(self) -> tuple[Frame | None, float]:
        with self.lock:
            return self.frame, self.t

    def close(self):
        self._stop = True
        try:
            self.sock.close()
        except OSError:
            pass


# ---------------------------------------------------------------------------
# --map: interactive pad calibration wizard
# ---------------------------------------------------------------------------
def run_map(joy_id: int) -> int:
    if joy_read(joy_id) is None:
        print(f"no pad on winmm id {joy_id}; plug it in (or --joy-id N)")
        return 1
    pm = {"buttons": {}, "sticks": {}, "triggers": {}}
    time.sleep(0.3)
    rest = joy_axes(joy_read(joy_id))
    print(f"rest axis values: {rest}")
    NOISE = 6000
    used_bits: set[int] = set()

    def wait_neutral():
        t0 = time.time()
        warned = False
        nonlocal rest
        while True:
            j = joy_read(joy_id)
            if j is not None and j.btns == 0 and j.pov == 65535 and \
               all(abs(joy_axes(j)[a] - rest[a]) < NOISE for a in AXIS_NAMES):
                return
            el = time.time() - t0
            if el > 1.0 and not warned and j is not None:
                warned = True
                off = [f"btns={j.btns:#06x}" if j.btns else "",
                       f"pov={j.pov}" if j.pov != 65535 else ""]
                off += [f"{a}={joy_axes(j)[a]} (rest {rest[a]})"
                        for a in AXIS_NAMES if abs(joy_axes(j)[a] - rest[a]) >= NOISE]
                print(f"  (waiting for neutral: {' '.join(x for x in off if x)})", flush=True)
            if el > 10.0 and j is not None:
                # a settled-but-shifted axis is the common cause: re-baseline
                rest = joy_axes(j)
                print(f"  (neutral timeout -- re-baselined rest to {rest})", flush=True)
                return
            time.sleep(0.03)

    def capture(prompt, secs=4.0):
        """(moved_axis|None, extreme_value, NEW button bits pressed during the
        prompt).  Axis must move >= NOISE to count."""
        print(f"  >> {prompt} ... ", end="", flush=True)
        j0 = joy_read(joy_id)
        base_bits = j0.btns if j0 is not None else 0
        t0 = time.time()
        best_ax, best_dev, best_val = None, 0, 0
        bits = 0
        while time.time() - t0 < secs:
            j = joy_read(joy_id)
            if j is not None:
                bits |= (j.btns & ~base_bits)
                for a, v in joy_axes(j).items():
                    dev = abs(v - rest[a])
                    if dev > best_dev:
                        best_ax, best_dev, best_val = a, dev, v
            time.sleep(0.01)
        if best_dev < NOISE:
            best_ax = None
        print(f"axis={best_ax}({best_val if best_ax else '-'}) bits={bits:#06x}")
        return best_ax, best_val, bits

    print("\nPAD CALIBRATION -- follow the prompts; return to neutral between steps.\n")
    for logical, prompt in (("lx", "push LEFT STICK fully RIGHT"),
                            ("ly", "push LEFT STICK fully UP"),
                            ("rx", "push RIGHT STICK fully RIGHT"),
                            ("ry", "push RIGHT STICK fully UP")):
        wait_neutral()
        ax, val, _ = capture(prompt)
        if ax is None:
            print("  (nothing moved -- keeping default)")
            pm["sticks"][logical] = DEFAULT_PADMAP["sticks"][logical]
            continue
        # pushing right/up should give +32767: if the extreme is on the low
        # side, the axis is inverted relative to that convention
        pm["sticks"][logical] = {"axis": ax, "min": 0, "max": 65535,
                                 "invert": bool(val < rest[ax])}
    for logical, prompt in (("lt", "pull LEFT TRIGGER fully"),
                            ("rt", "pull RIGHT TRIGGER fully")):
        wait_neutral()
        ax, val, bits = capture(prompt)
        entry = {"axis": None, "rest": 0, "full": 65535, "button": None}
        # a real full pull moves ~32k+ counts; less than half that and we'd
        # rather trust a digital bit than bind throttle to axis noise
        if ax is not None and abs(val - rest[ax]) >= 12000:
            entry.update({"axis": ax, "rest": int(rest[ax]), "full": int(val)})
        else:
            for b in range(16):
                if bits & (1 << b) and b not in used_bits:
                    entry["button"] = b
                    used_bits.add(b)
                    break
            if entry["button"] is None:
                print(f"  ({logical}: no axis or button seen -- run --map again)")
        if entry["axis"] is not None:
            # remember the digital bit too (harmless: only used if axis removed)
            for b in range(16):
                if bits & (1 << b) and b not in used_bits:
                    entry["button"] = b
                    used_bits.add(b)
                    break
        pm["triggers"][logical] = entry
    for name in ("A", "B", "X", "Y", "LB", "RB", "BACK", "START", "LS", "RS"):
        wait_neutral()
        _, _, bits = capture(f"press {name}", secs=3.0)
        for b in range(16):
            if bits & (1 << b) and b not in used_bits:
                pm["buttons"][str(b)] = name
                used_bits.add(b)
                break
        else:
            print(f"  ({name}: no new button seen -- skipped)")
    os.makedirs(REC_DIR, exist_ok=True)
    json.dump(pm, open(PADMAP_PATH, "w"), indent=2)
    print(f"\nsaved -> {PADMAP_PATH}")
    return 0


# ---------------------------------------------------------------------------
# --selftest: offline unit tests (no pad, no game, no vgamepad)
# ---------------------------------------------------------------------------
def _mkframe(slip=0.0, comb=None, gear=3, race=1, ts=0, drivetrain=2, accel=255) -> Frame:
    comb = 1.14 * slip if comb is None else comb   # measured on-throttle relation
    return Frame(timestamp_ms=ts, is_race_on=race, lap_no=0, cur_lap_time=0.0,
                 cur_race_time=0.0, dist_traveled=0.0, pos_x=0, pos_y=0, pos_z=0,
                 speed_mps=20.0, yaw=0, pitch=0, roll=0, vel_x=0, vel_y=0, vel_z=0,
                 angvel_x=0, angvel_y=0, angvel_z=0, rpm=4000, max_rpm=7000,
                 gear=gear, accel=accel, brake=0, steer=0,
                 slip_ratio_fl=slip, slip_ratio_fr=slip,
                 slip_ratio_rl=slip, slip_ratio_rr=slip,
                 combined_slip_fl=comb, combined_slip_fr=comb,
                 combined_slip_rl=comb, combined_slip_rr=comb,
                 drivetrain=drivetrain)


def _sim(tc: TractionController, slips, thr=1.0, dt=0.016, ts0=1):
    """Feed a slip sequence at telemetry rate; return the per-tick scales."""
    out = []
    t = 0.0
    for i, s in enumerate(slips):
        tc.on_frame(_mkframe(slip=s, ts=ts0 + i), t)
        out.append(tc.scale(dt, thr, t))
        t += dt
    return out


def _mkjoy(btns=0, pov=65535, **axes) -> JOYINFOEX:
    j = JOYINFOEX()
    j.dwSize = ctypes.sizeof(JOYINFOEX)
    j.btns = btns
    j.pov = pov
    for a in AXIS_NAMES:
        setattr(j, a, axes.get(a, 32768))
    j.U = axes.get("U", 0)
    j.V = axes.get("V", 0)
    return j


def selftest() -> int:
    fails = []

    def check(name, cond):
        print(f"  {'ok  ' if cond else 'FAIL'} {name}")
        if not cond:
            fails.append(name)

    # conversions
    check("axis_to_short center", axis_to_short(32768, 0, 65535, False) == 0)
    check("axis_to_short max", axis_to_short(65535, 0, 65535, False) == 32767)
    check("axis_to_short invert", axis_to_short(0, 0, 65535, True) == 32767)
    check("axis_to_unit rest", axis_to_unit(0, 0, 65535) == 0.0)
    check("axis_to_unit full", axis_to_unit(65535, 0, 65535) == 1.0)
    check("axis_to_unit reversed-range", abs(axis_to_unit(0, 65535, 0) - 1.0) < 1e-9)
    check("pov diagonal", POV_BITS[4500] == (XB["DPAD_UP"] | XB["DPAD_RIGHT"]))
    check("trig_byte clamps", trig_byte(2.0) == 255 and trig_byte(-1.0) == 0
          and trig_byte(0.5) == 128)

    # decode_pad: analog trigger must NOT be overridden by its digital bit
    # (this pad's bits 6/7 close early in the trigger travel)
    pm = json.loads(json.dumps(DEFAULT_PADMAP))
    st = decode_pad(_mkjoy(btns=1 << 7, V=16384), pm)          # 25% pull + bit set
    check("partial trigger stays analog", abs(st.rt - 0.25) < 0.01)
    st = decode_pad(_mkjoy(btns=1 << 6, U=32768), pm)          # 50% brake + bit set
    check("partial brake stays analog", abs(st.lt - 0.50) < 0.01)
    pm2 = json.loads(json.dumps(DEFAULT_PADMAP))
    pm2["triggers"]["rt"]["axis"] = None                       # digital-only hardware
    check("digital-only trigger full", decode_pad(_mkjoy(btns=1 << 7), pm2).rt == 1.0)
    check("digital-only trigger off", decode_pad(_mkjoy(), pm2).rt == 0.0)

    # driven-wheel selection
    fw = _mkframe(slip=0.0, comb=0.0, drivetrain=0)
    fw.slip_ratio_rl = fw.slip_ratio_rr = 9.0        # rear spins, FWD ignores it
    check("FWD ignores rear slip", driven_slips(fw)[0] == 0.0)
    rw = _mkframe(slip=0.0, comb=0.0, drivetrain=1)
    rw.slip_ratio_rl = 2.0
    check("RWD sees rear slip", driven_slips(rw)[0] == 2.0)
    lk = _mkframe(slip=0.0, comb=0.0)
    lk.slip_ratio_fl = -5.0                          # lockup: not TC's problem
    check("lockup ignored", driven_slips(lk)[0] == 0.0)
    ov = _mkframe(slip=0.0, comb=0.0, drivetrain=1)  # telemetry says RWD...
    ov.slip_ratio_fl = 3.0                           # ...but the front spins
    check("drivetrain override widens", driven_slips(ov, 2)[0] == 3.0)
    check("auto drivetrain narrows", driven_slips(ov)[0] == 0.0)

    # per-axle LONG-ONLY: regulate the fwd/back component only
    dr = _mkframe(slip=0.0, comb=0.0, drivetrain=1)  # RWD drift: big rear angle,
    dr.slip_ratio_rl = dr.slip_ratio_rr = 0.3        # little actual wheelspin
    dr.combined_slip_rl = dr.combined_slip_rr = 2.5
    check("drift reads full circle by default", driven_slips(dr)[2] == 2.5)
    check("rear long-only reads fwd slip",
          driven_slips(dr, long_rear=True)[2] == 0.3)
    check("front long-only irrelevant on RWD",
          driven_slips(dr, long_front=True)[2] == 2.5)
    aw = _mkframe(slip=0.2, comb=0.0)                # AWD, front circle loaded
    aw.combined_slip_fl = 1.8
    check("mixed axles: front keeps full circle",
          driven_slips(aw, long_rear=True)[2] == 1.8)
    check("both long-only reads fwd slip everywhere",
          driven_slips(aw, long_front=True, long_rear=True)[2] == 0.2)

    def _drift_frames(tc_):
        out, t = [], 0.0
        for i in range(30):
            fr_ = _mkframe(slip=0.0, comb=0.0, drivetrain=1, ts=i + 1)
            fr_.slip_ratio_rl = fr_.slip_ratio_rr = 0.2      # holding a drift:
            fr_.combined_slip_rl = fr_.combined_slip_rr = 2.0  # angle, not spin
            tc_.on_frame(fr_, t)
            out.append(tc_.scale(0.016, 1.0, t))
            t += 0.016
        return out
    tc = TractionController("HIGH")
    tc.long_only_rear = True
    check("drift + rear long-only: no cut", all(x == 1.0 for x in _drift_frames(tc)))
    check("drift default: cuts", _drift_frames(TractionController("HIGH"))[-1] < 0.7)
    fw2 = _mkframe(slip=0.1, comb=0.0, drivetrain=0)   # FWD, front circle loaded
    fw2.combined_slip_fl = fw2.combined_slip_fr = 1.7
    check("FWD long_front reads fwd slip",
          driven_slips(fw2, long_front=True)[2] == 0.1
          and driven_slips(fw2)[2] == 1.7)
    # toggling long-only mid-cut releases GRADUALLY (integrator carries over)
    tc = TractionController("HIGH")
    _drift_frames(tc)                                   # pinned at floor 0.04
    tc.long_only_rear = True                            # drift no longer counts
    rel, t = [], 30 * 0.016
    for i in range(40):
        fr_ = _mkframe(slip=0.0, comb=0.0, drivetrain=1, ts=100 + i)
        fr_.slip_ratio_rl = fr_.slip_ratio_rr = 0.2
        fr_.combined_slip_rl = fr_.combined_slip_rr = 2.0
        tc.on_frame(fr_, t)
        rel.append(tc.scale(0.016, 1.0, t))
        t += 0.016
    check("mid-cut toggle: no snap open", rel[0] < 0.2 and rel[-1] < 0.7
          and all(b >= a for a, b in zip(rel, rel[1:])))
    # engine API: set_long_only flips, emits once, idempotent
    msgs = []
    eng = TCEngine(emit=msgs.append)
    eng.set_long_only("rear", True)
    eng.set_long_only("rear", True)
    check("set_long_only flips + emits once",
          eng.tc.long_only_rear and len([m for m in msgs if "rear" in m]) == 1)
    eng.set_long_only("front", True)
    eng.set_long_only("front", False)
    check("set_long_only front round-trip", not eng.tc.long_only_front)

    # no slip -> exact passthrough in every mode
    for name in ("OFF", "LOW", "MEDIUM", "HIGH"):
        tc = TractionController(name)
        scales = _sim(tc, [0.2] * 40)
        check(f"{name} passthrough at grip", all(s == 1.0 for s in scales))

    # launch spike: wheelspin to slip 10 -- mode ordering + floors
    end_scales = {}
    for name in ("OFF", "LOW", "MEDIUM", "HIGH"):
        tc = TractionController(name)
        scales = _sim(tc, [10.0] * 30)
        end_scales[name] = scales[-1]
    check("OFF never cuts", end_scales["OFF"] == 1.0)
    check("LOW floor holds 0.30", abs(end_scales["LOW"] - 0.30) < 1e-9)
    check("MEDIUM cuts below LOW", end_scales["MEDIUM"] < end_scales["LOW"])
    check("HIGH cuts to its floor", abs(end_scales["HIGH"] - 0.04) < 1e-9)
    check("MEDIUM cuts fast (<=4 ticks to <0.5)",
          _sim(TractionController("MEDIUM"), [10.0] * 4)[-1] < 0.5)
    # pre-floor dynamics: slightly past the circle edge (comb ~1.25) MEDIUM
    # regulates while LOW (150% setpoint) stays open
    med = _sim(TractionController("MEDIUM"), [1.1] * 20)[-1]
    low = _sim(TractionController("LOW"), [1.1] * 20)[-1]
    check("past circle edge: MEDIUM cuts, LOW doesn't", med < 1.0 <= low
          and med > MODES[MODE_IDX["MEDIUM"]].floor)

    # recovery: after a wheelspin event, grip returns (closed loop, demand now
    # below the limit) -- the throttle comes back progressively, no snap
    tc = TractionController("MEDIUM")
    _sim(tc, [10.0] * 30)                    # spike event pins the cut
    rec, u, s, t = [], 0.0, 10.0, 30 * 0.016
    for i in range(140):
        s += 0.4 * (0.8 * u - s)             # grippy again: full u -> slip 0.8
        tc.on_frame(_mkframe(slip=s / 1.14, ts=100 + i), t)
        u = tc.scale(0.016, 1.0, t)
        rec.append(u)
        t += 0.016
    check("recovery is gradual", 0.05 < rec[10] < 0.9)
    check("recovery completes", rec[-1] == 1.0)
    t_full = next((i for i, s_ in enumerate(rec) if s_ == 1.0), 999) * 0.016
    check("MEDIUM recovery ~0.4-2s (severe event)", 0.4 < t_full < 2.0)

    # mode switch mid-cut preserves the accumulated cut (no throttle snap)
    tc = TractionController("MEDIUM")
    _sim(tc, [10.0] * 30)                    # pinned at MEDIUM floor 0.08
    tc.set_mode("LOW")
    check("switch mid-cut clamps to new floor", abs(tc.integ - 0.30) < 1e-9)
    tc.on_frame(_mkframe(slip=0.1, ts=500), 30 * 0.016)
    s0 = tc.scale(0.016, 1.0, 30 * 0.016)
    check("switch mid-cut: no snap open", s0 < 0.5)
    tc2 = TractionController("HIGH")
    _sim(tc2, [10.0] * 30)                   # HIGH cut to its floor 0.04
    tc2.set_mode("OFF")
    check("switch to OFF opens", tc2.scale(0.016, 1.0, 30 * 0.016) == 1.0)

    # the circle closes under cornering load: high combined slip cuts even
    # when longitudinal slip_ratio is low (the oversteer guard)
    tc = TractionController("HIGH")
    out = []
    t = 0.0
    for i in range(30):
        tc.on_frame(_mkframe(slip=0.2, comb=1.6, ts=i + 1), t)
        out.append(tc.scale(0.016, 1.0, t))
        t += 0.016
    check("cornering-load circle cuts (oversteer guard)", out[-1] < 0.7)
    # MEDIUM holds "right at 100%": at comb exactly 1.0 it does not intervene,
    # a hair over it does
    at_edge = _sim(TractionController("MEDIUM"), [1.0 / 1.14] * 30)
    check("MEDIUM open at exactly 100% (settled)",
          all(s == 1.0 for s in at_edge[12:]))
    check("edge approach pre-cut is mild", min(at_edge) > 0.6)
    over = _sim(TractionController("MEDIUM"), [1.15 / 1.14] * 60)
    check("MEDIUM trims just past 100%", over[-1] < 1.0)

    # closed-loop setpoint tracking: a toy tire (slip follows throttle with a
    # short lag; full throttle would spin to slip 3) must be PINNED at the
    # MEDIUM target by the PID -- the whole point of the law
    tc = TractionController("MEDIUM")
    u, s, t = 1.0, 0.0, 0.0
    errs = []
    for i in range(400):                     # 6.4 s
        s += 0.4 * (3.0 * u - s)             # plant: slip chases 3*throttle
        tc.on_frame(_mkframe(slip=s / 1.14, ts=i + 1), t)   # comb = s
        u = tc.scale(0.016, 1.0, t)
        t += 0.016
        if i > 150:                          # after the transient
            errs.append(abs(tc.s_comb - 1.0))
    mean_err = sum(errs) / len(errs)
    print(f"       (closed-loop mean |slip-target| = {mean_err:.3f})")
    check("PID pins slip at the target (mean err < 0.06)", mean_err < 0.06)
    check("PID holds partial throttle (not bang-bang)",
          0.05 < u < 0.95)

    # the same loop with the game's ~100 ms actuation TRANSPORT DELAY and
    # fast/slow tire response: the gains must not limit-cycle (this is the
    # configuration that exposed the 2.5 Hz throttle-sawtooth regression)
    def _delayed_loop(mode, k_plant, lag, delay_ticks, n=600):
        tc_ = TractionController(mode)
        pipe = deque([1.0] * delay_ticks, maxlen=delay_ticks)
        u_, s_, t_ = 1.0, 0.0, 0.0
        hist = []
        for i in range(n):
            u_eff = pipe[0]
            pipe.append(u_)
            s_ += lag * (k_plant * u_eff - s_)
            tc_.on_frame(_mkframe(slip=s_ / 1.14, ts=i + 1), t_)
            u_ = tc_.scale(0.016, 1.0, t_)
            t_ += 0.016
            if i > 250:
                hist.append(tc_.s_comb)
        return min(hist), max(hist), sum(hist) / len(hist)

    for mode, tgt in (("MEDIUM", 1.0), ("HIGH", 0.85)):
        for k_plant, lag in ((3.0, 0.4), (3.0, 0.7), (6.0, 0.4), (6.0, 0.7)):
            lo, hi, mean = _delayed_loop(mode, k_plant, lag, 6)
            check(f"{mode} stable w/ 100ms delay, k={k_plant:.0f} lag {lag} "
                  f"(p2p {hi - lo:.2f}, mean {mean:.2f})",
                  hi - lo < 0.2 and abs(mean - tgt) < 0.1)
    lo, hi, mean = _delayed_loop("LOW", 4.0, 0.4, 6)
    check(f"LOW stable w/ 100ms delay (p2p {hi - lo:.2f})",
          hi - lo < 0.25 and abs(mean - 1.5) < 0.15)

    # CUSTOM mode: user setpoint with anchor-interpolated gains
    cm = MODES[MODE_IDX["CUSTOM"]]
    hi_m = MODES[MODE_IDX["HIGH"]]
    med_m = MODES[MODE_IDX["MEDIUM"]]
    apply_custom_target(0.85)
    check("custom at 85% == HIGH gains",
          abs(cm.kp - hi_m.kp) < 1e-9 and abs(cm.floor - hi_m.floor) < 1e-9)
    apply_custom_target(1.0)
    check("custom at 100% == MEDIUM gains", abs(cm.kp - med_m.kp) < 1e-9)
    apply_custom_target(0.925)
    check("custom midway interpolates",
          min(hi_m.kp, med_m.kp) < cm.kp < max(hi_m.kp, med_m.kp))
    apply_custom_target(9.9)
    check("custom target clamped high", cm.target == CUSTOM_MAX)
    apply_custom_target(0.01)
    check("custom target clamped low", cm.target == CUSTOM_MIN)
    check("chord cycles into CUSTOM", MODES[-1].name == "CUSTOM")
    for pct, k_plant in ((0.7, 4.0), (0.95, 4.0), (0.95, 5.0),
                         (1.2, 4.0), (1.8, 4.0)):
        apply_custom_target(pct)          # delay-margin holds across the range
        lo, hi, mean = _delayed_loop("CUSTOM", k_plant, 0.4, 6)
        check(f"CUSTOM {pct * 100:.0f}% k={k_plant:.0f} stable w/ 100ms delay "
              f"(p2p {hi - lo:.2f}, mean {mean:.2f})",
              hi - lo < 0.25 and abs(mean - pct) < 0.12)
    apply_custom_target(1.0)
    _orig_dump = globals()["dump_tune"]   # mute the file write: a live GUI
    globals()["dump_tune"] = lambda: None  # could hot-reload a test artifact
    try:
        msgs = []
        eng = TCEngine(emit=msgs.append)
        eng.set_custom_target(92)
        check("set_custom_target applies",
              abs(MODES[MODE_IDX["CUSTOM"]].target - 0.92) < 1e-9)
        check("set_custom_target emits", any("92%" in m for m in msgs))
    finally:
        globals()["dump_tune"] = _orig_dump
        apply_custom_target(1.0)

    # ABS channel: lockup regulation on the brake, all four wheels
    lk2 = _mkframe(slip=3.0)                          # wheelspin, no lockup
    check("ABS ignores wheelspin", abs_slips(lk2)[0] == 0.0)
    check("ABS wheelspin never cuts brake", abs_slips(lk2)[2] == 0.0)
    lat = _mkframe(slip=0.05, comb=1.4)               # at-limit cornering
    check("ABS cornering load never cuts brake", abs_slips(lat)[2] == 0.0)

    # friendly pad name (winmm's szPname is the useless generic string; the
    # real name comes from the registry at detect time -- these test the
    # deterministic fallback used when the registry has nothing)
    check("DualSense fallback names from VID/PID",
          _pad_name_fallback((0x054C, 0x0CE6, _GENERIC_PNAME)).startswith("Sony DualSense"))
    check("unknown vendor falls back to hex",
          _pad_name_fallback((0x1234, 0x5678, _GENERIC_PNAME)) == "controller (1234:5678)")
    check("real szPname preserved",
          _pad_name_fallback((0x1, 0x2, "My Nice Pad")) == "My Nice Pad")
    check("pad_display_name never crashes on junk",
          isinstance(pad_display_name((0x9999, 0x9999, "")), str))

    # leak-test verdict (in-engine test; 30% commanded)
    check("leak verdict: clean when echo=sent",
          leak_verdict(300, [0.30] * 40)[0] == "clean")
    check("leak verdict: leaking when echo=user",
          leak_verdict(300, [1.0] * 40)[0] == "leaking")
    check("leak verdict: inconclusive on too few throttle frames",
          leak_verdict(300, [1.0] * 5)[0] == "held")
    check("leak verdict: inconclusive with no telemetry",
          leak_verdict(0, [])[0] == "notelem")
    # squeeze/release rhythm must NOT decide the outcome: a low throttle duty
    # cycle still passes as long as enough samples were gathered
    check("leak verdict: verdict independent of trigger duty cycle",
          leak_verdict(375, [0.30] * LEAK_MIN_SAMPLES)[0] == "clean")
    # INTERMITTENT leak (game alternating pads ~13% of frames, as measured
    # on-rig): the mean stays near the commanded value, so an average-only
    # test called this CLEAN -- the duty cycle of STRAY frames is what catches it
    intermittent = [1.0 if i % 8 == 0 else 0.30 for i in range(64)]
    v, avg, over = leak_verdict(300, intermittent)
    check(f"leak verdict: catches intermittent leak (mean {avg:.0%}, "
          f"{over:.0%} of frames)", v == "leaking" and avg < 0.55)
    check("leak verdict: a single stray frame is not a leak",
          leak_verdict(300, [0.30] * 63 + [1.0])[0] == "clean")
    lk3 = _mkframe(slip=0.0, comb=0.0)
    lk3.slip_ratio_fl = -2.0                          # front lockup
    lk3.combined_slip_fl = -2.3
    check("ABS reads lockup", abs_slips(lk3)[0] == 2.0
          and abs_slips(lk3)[2] == 2.3)
    check("ABS long-front reads lockup only",
          abs_slips(lk3, long_front=True, long_rear=True)[2] == 2.0)
    rw2 = _mkframe(slip=0.0, comb=0.0, drivetrain=1)  # RWD car...
    rw2.slip_ratio_fl = -3.0
    rw2.combined_slip_fl = -3.4
    check("ABS watches ALL wheels regardless of drivetrain",
          abs_slips(rw2)[2] == 3.4)
    ab_t = TractionController("MEDIUM", channel="abs")
    out, t = [], 0.0
    for i in range(30):                               # hard lockup while braking
        ab_t.on_frame(_mkframe(slip=-2.0, ts=i + 1), t)
        out.append(ab_t.scale(0.016, 1.0, t))         # pedal = brake, floored
        t += 0.016
    check("ABS cuts brake on lockup", out[-1] < 0.7)
    check("ABS floor keeps brakes alive",
          min(out) >= ABS_MODES[MODE_IDX["MEDIUM"]].floor - 1e-9)
    ab_t = TractionController("HIGH", channel="abs")
    slow = _mkframe(slip=-5.0, ts=1)
    slow.speed_mps = 2.0                              # below ABS_MIN_SPEED
    ab_t.on_frame(slow, 0.0)
    check("ABS disengages at low speed (full brake to stop)",
          ab_t.scale(0.016, 1.0, 0.0) == 1.0)
    # speed-gate hysteresis: engages above gate+1, holds through jitter,
    # releases below the gate
    ab_h = TractionController("HIGH", channel="abs")
    seq = []
    for i, spd in enumerate((3.4, 4.2, 3.4, 2.8)):
        f_ = _mkframe(slip=-2.0, ts=i + 1)
        f_.speed_mps = spd
        ab_h.on_frame(f_, i * 0.016)
        seq.append(ab_h.scale(0.016, 1.0, i * 0.016))
    check("ABS speed hysteresis",
          seq[0] == 1.0 and seq[1] < 1.0 and seq[2] < 1.0 and seq[3] == 1.0)
    # ABS stability with the same 100 ms transport delay (braking plant)
    def _delayed_loop_abs(mode, k_plant, lag, delay_ticks, n=600):
        tc_ = TractionController(mode, channel="abs")
        pipe = deque([1.0] * delay_ticks, maxlen=delay_ticks)
        u_, s_, t_ = 1.0, 0.0, 0.0
        hist = []
        for i in range(n):
            u_eff = pipe[0]
            pipe.append(u_)
            s_ += lag * (k_plant * u_eff - s_)
            tc_.on_frame(_mkframe(slip=-s_ / 1.14, ts=i + 1), t_)
            u_ = tc_.scale(0.016, 1.0, t_)
            t_ += 0.016
            if i > 250:
                hist.append(tc_.s_comb)
        return min(hist), max(hist), sum(hist) / len(hist)
    for mode, tgt in (("MEDIUM", 1.0), ("HIGH", 0.85)):
        for k_plant in (3.0, 6.0):
            lo, hi, mean = _delayed_loop_abs(mode, k_plant, 0.4, 6)
            check(f"ABS {mode} stable w/ 100ms delay, k={k_plant:.0f} "
                  f"(p2p {hi - lo:.2f}, mean {mean:.2f})",
                  hi - lo < 0.2 and abs(mean - tgt) < 0.1)
    # ABS CUSTOM is independent of TC CUSTOM, with its own (much higher) range
    apply_custom_target(0.75, ABS_MODES)
    check("ABS custom applies to ABS only",
          abs(ABS_MODES[MODE_IDX["CUSTOM"]].target - 0.75) < 1e-9
          and abs(MODES[MODE_IDX["CUSTOM"]].target - 1.0) < 1e-9)
    apply_custom_target(9.9, ABS_MODES)
    check("ABS custom clamps at 600%",
          ABS_MODES[MODE_IDX["CUSTOM"]].target == CUSTOM_MAX_ABS)
    # a lock-permitting setpoint (500%) still regulates stably against the
    # 100 ms delay on strong brakes (plant gain 8: full pedal -> slip 8)
    apply_custom_target(5.0, ABS_MODES)
    lo, hi, mean = _delayed_loop_abs("CUSTOM", 8.0, 0.4, 6)
    check(f"ABS CUSTOM 500% stable on strong brakes "
          f"(p2p {hi - lo:.2f}, mean {mean:.2f})",
          hi - lo < 0.6 and abs(mean - 5.0) < 0.4)
    apply_custom_target(1.0, ABS_MODES)
    # chord: BACK + dpad RIGHT/LEFT steps the ABS mode, swallowed
    tc_c = TractionController("OFF")
    ab_c = TractionController("OFF", channel="abs")
    cs = ChordSwitcher(emit=lambda m: None)
    outs = []
    for back, pov, tt in ((True, 65535, 0.0), (True, 9000, 0.01),
                          (True, 9000, 0.02), (True, 65535, 0.03),
                          (False, 65535, 0.05)):
        stt = PadState(buttons=(XB["BACK"] if back else 0) | POV_BITS.get(pov, 0),
                       pov=pov)
        outs.append(cs.process(stt, tt, tc_c, abs_ctl=ab_c))
    check("chord RIGHT steps ABS up", ab_c.mode.name == "LOW"
          and tc_c.mode.name == "OFF")
    check("chord RIGHT swallowed", all((o & DPAD_MASK) == 0 for o in outs))
    # a hat grazing a diagonal on the way to RIGHT must not step TC:
    # 4500 (up-right transient) then settled 9000 -> ABS step only
    tc_c = TractionController("OFF")
    ab_c = TractionController("OFF", channel="abs")
    cs = ChordSwitcher(emit=lambda m: None)
    for back, pov, tt in ((True, 65535, 0.0), (True, 4500, 0.01),
                          (True, 9000, 0.02), (True, 9000, 0.03),
                          (False, 65535, 0.05)):
        stt = PadState(buttons=(XB["BACK"] if back else 0) | POV_BITS.get(pov, 0),
                       pov=pov)
        cs.process(stt, tt, tc_c, abs_ctl=ab_c)
    check("chord diagonal graze routes to ABS not TC",
          tc_c.mode.name == "OFF" and ab_c.mode.name == "LOW")

    # telemetry recorder: full Frame + extras, deduped, round-trips
    _rp = os.path.join(REC_DIR, "_rec_selftest.csv")
    try:
        os.makedirs(REC_DIR, exist_ok=True)
        rec = TelemetryRecorder(_rp)
        ex = [0.0, 0.5, 0.0, "MEDIUM", "HIGH", 0.5, 1.0,
              1, 4.0, 3.2, 35.0, 34.5, 0, 0.0, 0]
        check("recorder writes fresh frame", rec.write(_mkframe(slip=1.0, ts=1), ex))
        check("recorder dedupes same ts", not rec.write(_mkframe(slip=1.0, ts=1), ex))
        rec.write(_mkframe(slip=2.0, ts=2), ex)
        rec.close()
        with open(_rp) as _f:
            _rr = list(csv.reader(_f))
        check("recorder header = all Frame fields + extras",
              _rr[0] == REC_FIELDS + REC_EXTRA and len(REC_FIELDS) == 42)
        check("recorder row count", len(_rr) == 3 and rec.rows == 2)
        check("recorder captured a slip field",
              _rr[1][REC_FIELDS.index("slip_ratio_fl")] == "1.0")
    finally:
        try:
            os.remove(_rp)
        except OSError:
            pass

    # ABS LEARNING: ceiling adapts from decel evidence, corners derated
    def _feed(ln_, knee, n):
        # toy braking model: decel plateau 36 m/s^2 below the knee, fading
        # 8 m/s^2 per slip unit past it; slip sweeps 0.3C..1.1C (regulation
        # keeps slip near the cap, approach passes through the mid band)
        for i in range(n):
            C = ln_.ceiling
            slip = (0.3 + 0.8 * ((i % 20) / 19.0)) * C
            d = 36.0 if slip < knee else max(8.0, 36.0 - 8.0 * (slip - knee))
            ln_.update(0.016, slip, d, 0.0, 1.0, 40.0)
    ln = AbsLearner()
    ln.reset(4.0)
    _feed(ln, 6.0, 4000)                    # ~64 s hard braking, tarmac knee 6
    check(f"learner climbs to the tarmac knee (ceiling {ln.ceiling:.1f})",
          5.0 < ln.ceiling < 8.0)
    ln2 = AbsLearner()
    ln2.reset(4.0)
    _feed(ln2, 1.2, 4000)                   # dirt knee 1.2
    check(f"learner drops to the dirt knee (ceiling {ln2.ceiling:.1f})",
          1.0 < ln2.ceiling < 2.4)
    ln3 = AbsLearner()
    ln3.reset(6.0)
    ln3.alat_max = 40.0
    ln3.lat_f = 0.0
    check("no derate when straight", abs(ln3.effective_cap() - 6.0) < 1e-9)
    ln3.lat_f = 28.0
    mid = ln3.effective_cap()
    check("derate shrinks the cap with lateral load", 6.0 * 0.25 < mid < 6.0)
    ln3.lat_f = 40.0
    check("derate floors, never zeroes",
          CUSTOM_MIN <= ln3.effective_cap() <= 6.0 * AbsLearner.DERATE_FLOOR + 1e-9)
    ln4 = AbsLearner()
    ln4.reset(4.0)
    ln4.update(0.016, 0, 0, 90.0, 0.5, 40.0)      # single curb-strike frame
    check("curb spike can't inflate the lat budget", ln4.alat_max < 36.0)
    for _ in range(400):                          # sustained corner-braking
        ln4.update(0.016, 0, 0, 45.0, 0.5, 40.0)
    check("sustained cornering raises the lat budget", ln4.alat_max > 40.0)
    ln4b = AbsLearner()
    ln4b.reset(4.0)
    for _ in range(400):                          # same lateral, NO braking:
        ln4b.update(0.016, 0, 0, 45.0, 0.0, 40.0)  # aero sweepers don't count
    check("off-brake lateral never teaches the budget", ln4b.alat_max <= 35.0)
    ln5 = AbsLearner()
    ln5.reset(4.0)
    for _ in range(2000):                    # looks like falloff, but DEEP
        ln5.update(0.016, 3.8, 8.0, 20.0, 1.0, 40.0)   # cornering (>CORNER_LAT)
    check("deep-corner samples never teach the ceiling",
          abs(ln5.ceiling - 4.0) < 1e-9)
    ln6 = AbsLearner()
    ln6.reset(4.0)
    for i in range(4000):                    # braking THROUGH minor steering
        C = ln6.ceiling                      # corrections (lat 10 = ~1g): the
        slip = (0.3 + 0.8 * ((i % 20) / 19.0)) * C   # g-vector must credit the
        gt = 36.0 if slip < 6.0 else max(8.0, 36.0 - 8.0 * (slip - 6.0))
        d = math.sqrt(max(0.0, gt * gt - 100.0))     # lateral share as grip
        ln6.update(0.016, slip, d, 10.0, 1.0, 40.0)
    check(f"corrected braking still climbs to the knee (ceiling {ln6.ceiling:.1f})",
          5.0 < ln6.ceiling < 8.0)
    # rumble passthrough: DualSense HID output report layouts (per the
    # Linux hid-playstation driver) -- USB 0x02 and BT 0x31 with CRC
    import zlib as _zl
    u = ds_usb_report(48, 200, 55)
    check("rumble USB report: id/flags/motors/fw2-flag",
          u[0] == 0x02 and u[1] == 0x03 and u[3] == 55 and u[4] == 200
          and u[39] == 0x04 and len(u) == 48)
    b = ds_bt_report(78, 200, 55, seq=5)
    check("rumble BT report: id/seq/tag/motors",
          b[0] == 0x31 and b[1] == 0x50 and b[2] == 0x10
          and b[5] == 55 and b[6] == 200 and b[41] == 0x04 and len(b) == 78)
    check("rumble BT report: CRC32 (0xA2-seeded) matches",
          int.from_bytes(b[74:78], "little")
          == (_zl.crc32(b"\xa2" + b[:74]) & 0xFFFFFFFF))
    _rmb = DualSenseRumble(emit=lambda m: None)
    _rmb.set(999, -5)
    check("rumble set() clamps to byte range",
          _rmb._large == 255 and _rmb._small == 0)
    # the controller light language (pure logic)
    ls = light_state(0.0, "OFF", 1.0, "OFF", 1.0, False, False,
                     False, "tc", 0, 0, 0.0, "tc")
    check("lights: both OFF -> white, LEDs dark", ls == ((255, 255, 255), 0))
    ls = light_state(0.0, "MEDIUM", 1.0, "OFF", 1.0, False, False,
                     False, "tc", 2, 0, 0.0, "tc")
    check("lights: TC-only hue is its own normalized color",
          ls[0] == (81, 174, 0) and ls[1] == 0)
    a = light_state(0.0, "MEDIUM", 1.0, "MEDIUM", 1.0, False, False,
                    False, "tc", 2, 2, 0.0, "tc")
    check("lights: blend is the mean of the channel hues",
          a[0] == ((81 + 31) // 2, (174 + 224) // 2, 0))
    on = light_state(0.0, "MEDIUM", 1.0, "MEDIUM", 1.0, False, True,
                     False, "tc", 2, 2, 0.0, "tc")
    off = light_state(0.1, "MEDIUM", 1.0, "MEDIUM", 1.0, False, True,
                      False, "tc", 2, 2, 0.0, "tc")
    check("lights: ABS cut strobes its own color",
          on[0] == (31, 224, 0) and off[0] == (0, 0, 0))
    both = light_state(0.0, "MEDIUM", 1.0, "MEDIUM", 1.0, True, True,
                       False, "tc", 2, 2, 0.0, "tc")
    check("lights: ABS wins the bar when both cut", both[0] == (31, 224, 0))
    ch = light_state(0.0, "HIGH", 0.85, "MEDIUM", 1.0, False, False,
                     True, "abs", 3, 2, 0.0, "tc")
    check("lights: chord = white + adjusted channel's count bar",
          ch == ((255, 255, 255), LED_BAR[2]))
    fl = light_state(1.0, "HIGH", 0.85, "MEDIUM", 1.0, False, False,
                     False, "tc", 3, 2, 2.0, "tc")
    check("lights: mode-change flash shows the count bar",
          fl[1] == LED_BAR[3])
    fl2 = light_state(3.0, "HIGH", 0.85, "MEDIUM", 1.0, False, False,
                      False, "tc", 3, 2, 2.0, "tc")
    check("lights: flash window expires", fl2[1] == 0)
    cut_off = light_state(0.0, "OFF", 1.0, "OFF", 1.0, True, True,
                          False, "tc", 0, 0, 0.0, "tc")
    check("lights: an OFF channel never strobes", cut_off[0] == (255, 255, 255))

    ul = ds_usb_report(48, 0, 0, rgb=(1, 2, 3), player_leds=0x04, setup=True)
    check("lights USB: flags/rgb/player/setup bytes",
          ul[2] & 0x14 == 0x14 and ul[45:48] == bytes((1, 2, 3))
          and ul[44] == 0x04 and ul[39] & 0x02 and ul[42] == 0x02)
    bl = ds_bt_report(78, 0, 0, 1, rgb=(9, 8, 7), player_leds=0x1F, setup=True)
    check("lights BT: flags/rgb/player/setup bytes + CRC",
          bl[4] & 0x14 == 0x14 and bl[47:50] == bytes((9, 8, 7))
          and bl[46] == 0x1F and bl[41] & 0x02 and bl[44] == 0x02
          and int.from_bytes(bl[74:78], "little")
          == (_zl.crc32(b"\xa2" + bl[:74]) & 0xFFFFFFFF))

    # chord with a non-BACK modifier (configurable: RB pairs the chord
    # across both hands instead of double-loading the left thumb)
    cs_rb = ChordSwitcher(emit=lambda m: None, mod=XB["RB"])
    tc_rb = TractionController("MEDIUM")
    st_rb = PadState(buttons=XB["RB"], pov=0)
    outs = [cs_rb.process(st_rb, 0.0, tc_rb), cs_rb.process(st_rb, 0.016, tc_rb)]
    check("chord: RB modifier steps TC and is swallowed",
          tc_rb.mode.name == "HIGH"
          and all((o & (XB["RB"] | DPAD_MASK)) == 0 for o in outs))
    cs_rb2 = ChordSwitcher(emit=lambda m: None, mod=XB["RB"])
    tc_rb2 = TractionController("MEDIUM")
    cs_rb2.process(PadState(buttons=XB["RB"], pov=-1), 0.0, tc_rb2)
    out = cs_rb2.process(PadState(buttons=0, pov=-1), 0.1, tc_rb2)
    check("chord: RB tap replays like BACK", bool(out & XB["RB"]))

    ln7 = AbsLearner()
    ln7.reset(4.0)
    ln7.s_ref[1], ln7.w_ref[1] = 36.0, 1.0         # primed plateau reference
    for _ in range(2000):                    # regulated frames: deep cut holds
        ln7.update(0.016, 3.8, 8.0, 0.0, 1.0, 40.0, open_loop=False)
    check("regulated frames never read as falloff",   # slip at cap, decel tiny
          abs(ln7.ceiling - 4.0) < 1e-9)
    # the LIVE regime: the PID pins slip at the cap, so the ONLY surface
    # probes are the cut-onset overshoots (transport delay = ~8 frames of
    # full-torque physics past the cap). Model braking events as an onset
    # sweep (open loop) followed by a regulated phase (no evidence).
    def _event(ln_, lo, hi, curve, n_reg=30):
        for i in range(8):
            C = ln_.ceiling
            s = C * (lo + (hi - lo) * i / 7.0)
            ln_.update(0.016, s, curve(s), 0.0, 1.0, 40.0)
        for _ in range(n_reg):
            ln_.update(0.016, ln_.ceiling, 5.0, 0.0, 1.0, 40.0,
                       open_loop=False)
        for _ in range(5):                   # coast between stops (slip
            ln_.update(0.016, 0.0, 0.0, 0.0, 0.0, 40.0)   # settles, so the
        # next onset's low-slip frames aren't eaten by the RECOV guard)
    ln8 = AbsLearner()
    ln8.reset(1.0)
    ln8.s_ref[1], ln8.w_ref[1] = 36.0, 1.0
    for _ev in range(40):                    # tarmac, knee 6
        _event(ln8, 1.0, 3.0,
               lambda s: 36.0 if s < 6.0 else max(8.0, 36.0 - 8.0 * (s - 6.0)))
    check(f"onset overshoots climb to the knee (ceiling {ln8.ceiling:.1f})",
          4.5 < ln8.ceiling < 8.0)
    ln9 = AbsLearner()
    ln9.reset(4.0)
    ln9.s_ref[1], ln9.w_ref[1] = 36.0, 1.0   # tarmac ref, then dirt: plateau
    for _ev in range(60):                    # 20, locked 12, knee 1.5.
        for _ in range(12):                  # low-slip dwell first -- real
            ln9.update(0.016, 0.4, 20.0, 0.0, 1.0, 40.0)   # braking dwells
        _event(ln9, 0.3, 2.5,                # there (bimodal), re-teaching
               lambda s: 20.0 if s < 1.5 else 12.0,        # the reference;
               n_reg=90)                     # events >DN_GAP apart so the
        # persistent dirt falloff CONFIRMS across events and jumps chain
    check(f"dirt onsets pull the ceiling down (ceiling {ln9.ceiling:.1f})",
          0.8 < ln9.ceiling < 2.5)
    # per-frame verdicts (the recorded evidence trail must tell the truth)
    lnv = AbsLearner()
    lnv.reset(4.0)
    lnv.update(0.016, 5.0, 36.0, 0.0, 1.0, 40.0, open_loop=False)
    check("verdict: regulated frame -> GATED", lnv.verdict == lnv.V_GATED)
    lnv.update(0.016, 5.0, 36.0, 0.0, 0.7, 40.0)
    check("verdict: partial pedal -> GATED", lnv.verdict == lnv.V_GATED)
    lnv.update(0.016, 0.5, 36.0, 0.0, 1.0, 40.0)
    check("verdict: plateau sample -> REF", lnv.verdict == lnv.V_REF)
    lnv.update(0.016, 0.5, 4.0, 0.0, 1.0, 40.0)
    check("verdict: decel still building -> RAMP", lnv.verdict == lnv.V_RAMP)
    lnv.s_ref[1], lnv.w_ref[1] = 36.0, 1.0
    lnv.update(0.016, 5.0, 36.0, 0.0, 1.0, 40.0)
    check("verdict: grip past ceiling -> GRIP", lnv.verdict == lnv.V_GRIP)
    lnv.update(0.016, 5.0, 8.0, 0.0, 1.0, 40.0)
    check("verdict: collapsed g -> FALL", lnv.verdict == lnv.V_FALL)
    lnv.update(0.016, 5.0, 32.0, 0.0, 1.0, 40.0)      # r ~0.89: deadband
    check("verdict: deadband -> DEAD", lnv.verdict == lnv.V_DEAD)
    c_before = lnv.ceiling
    lnv.update(0.016, 2.0, 8.0, 0.0, 1.0, 40.0)       # slip collapsing 187/s:
    check("verdict: post-cut re-grip -> RECOV, no pull",   # re-grip, not
          lnv.verdict == lnv.V_RECOV and lnv.ceiling == c_before)  # falloff
    lnv.update(0.016, 5.0, 8.0, 0.0, 1.0, 20.0)       # low speed: judged vs
    check("verdict: low-speed falloff judged at nearest knot -> FALL",
          lnv.verdict == lnv.V_FALL and lnv.ceiling < c_before)   # the
    # interpolated (knot-clamped) reference -- a true collapse still counts
    lnr = AbsLearner()
    lnr.reset(1.0)
    lnr.s_ref[1], lnr.w_ref[1] = 36.0, 1.0
    lnr.update(0.016, 3.0, 36.0, 0.0, 1.0, 40.0)      # one probe frame: ease
    check("single GRIP frame only eases", lnr.ceiling < 1.5)
    lnr.update(0.016, 3.1, 36.0, 0.0, 1.0, 40.0)      # two agree: ratchet
    check(f"consecutive GRIP frames ratchet to the evidence "
          f"({lnr.ceiling:.2f})", abs(lnr.ceiling - 0.95 * 3.0) < 0.01)
    lnd = AbsLearner()
    lnd.reset(6.0)
    lnd.s_ref[1], lnd.w_ref[1] = 18.0, 1.0            # dirt-ish reference
    lnd.update(0.016, 2.1, 8.0, 0.0, 1.0, 40.0)       # collapsed at slip 2.1
    lnd.update(0.016, 3.2, 8.0, 0.0, 1.0, 40.0)       # a pair -- but a single
    check(f"one event's falloff never jumps (excursion; {lnd.ceiling:.2f})",
          5.5 < lnd.ceiling < 5.9)                    # event only ARMS it
    for _ in range(110):                              # next braking event
        lnd.update(0.016, 0.0, 0.0, 0.0, 0.0, 40.0)   # (>DN_GAP later):
    lnd.update(0.016, 2.1, 8.0, 0.0, 1.0, 40.0)       # falloff persists ->
    lnd.update(0.016, 3.0, 8.0, 0.0, 1.0, 40.0)       # confirmed, jump
    check(f"second-event falloff jumps, bounded to half ({lnd.ceiling:.2f})",
          2.6 < lnd.ceiling < 2.95)
    for _ in range(110):
        lnd.update(0.016, 0.0, 0.0, 0.0, 0.0, 40.0)
    lnd.update(0.016, 2.1, 8.0, 0.0, 1.0, 40.0)
    lnd.update(0.016, 3.0, 8.0, 0.0, 1.0, 40.0)       # chain continues:
    check(f"third event reaches the collapsed slip ({lnd.ceiling:.2f})",
          abs(lnd.ceiling - 0.85 * 2.1) < 0.1)
    lnj = AbsLearner()
    lnj.reset(4.0)
    lnj.s_ref[1], lnj.w_ref[1] = 36.0, 1.0
    for _ in range(20):                               # braking over a crest:
        lnj.update(0.016, 2.6, 2.0, 0.0, 1.0, 40.0, vert=-9.5)   # free-fall
    lnj.update(0.016, 2.6, 2.0, 0.0, 1.0, 40.0, vert=30.0)       # landing hit
    check("airborne frames never teach (verdict AIR)",
          lnj.verdict == lnj.V_AIR and abs(lnj.ceiling - 4.0) < 1e-9)
    lna = AbsLearner()
    lna.reset(4.0)                           # aero line: 30 m/s^2 mechanical
    lna.s_ref[1], lna.v2_ref[1], lna.w_ref[1] = 30.0, 1600.0, 1.0   # at 40 m/s
    lna.s_ref[2], lna.v2_ref[2], lna.w_ref[2] = 42.0, 6400.0, 1.0   # at 80 m/s
    mid = lna.ref_decel(60.0)                # 3600 -> 30 + 12*(2000/4800)
    check(f"aero reference interpolates between knots ({mid:.1f})",
          abs(mid - 35.0) < 0.1)
    check("aero reference never extrapolates past the knots",
          abs(lna.ref_decel(120.0) - 42.0) < 0.1
          and abs(lna.ref_decel(10.0) - 30.0) < 0.1)
    # controller obeys the live override (the learner's derated cap)
    ab_o = TractionController("HIGH", channel="abs")
    ab_o.on_frame(_mkframe(slip=-2.0, ts=1), 0.0)   # lockup comb ~2.28
    ab_o.target_override = 5.0
    check("learned cap lifts the mode setpoint",
          ab_o.scale(0.016, 1.0, 0.0) == 1.0)
    ab_o.target_override = None
    ab_o.on_frame(_mkframe(slip=-2.0, ts=2), 0.016)
    check("cleared override restores the mode setpoint",
          ab_o.scale(0.016, 1.0, 0.016) < 1.0)
    # engine toggle: refuses on OFF, starts from the user's setpoint
    _msgs = []
    _eng = TCEngine(abs_mode="OFF", emit=_msgs.append)
    _eng.set_abs_learning(True)
    check("learning refuses ABS OFF", not _eng.abs_learning)
    _eng2 = TCEngine(abs_mode="MEDIUM", emit=lambda m: None)
    _eng2.set_abs_learning(True)
    check("learning starts from the user's setpoint",
          _eng2.abs_learning and abs(_eng2.abs_learn.ceiling - 1.0) < 1e-9)
    _eng2.set_abs_learning(False)
    check("learning off clears the override",
          _eng2.abs_ctl.target_override is None)
    # the central mode watcher: F-keys and the chord bypass set_mode_ch, so
    # learning-restart must hang off the mode CHANGE, not the input path
    _eng3 = TCEngine(abs_mode="MEDIUM", emit=lambda m: None)
    _eng3.set_abs_learning(True)
    _eng3.abs_learn.ceiling = 3.3            # drifted
    _eng3.abs_ctl.set_mode("HIGH")           # as poll_fkeys/chord would
    _eng3._on_abs_mode_change()
    check("hotkey mode change restarts learning from the new setpoint",
          abs(_eng3.abs_learn.ceiling - 0.85) < 1e-9)
    _eng3.abs_ctl.set_mode("OFF")
    _eng3._on_abs_mode_change()
    check("hotkey ABS OFF disarms learning",
          not _eng3.abs_learning and _eng3.abs_ctl.target_override is None)

    # standing-start launch cap: bounds the open-loop spike the latency would
    # deliver; lifts once rolling
    tc = TractionController("HIGH")
    fr0 = _mkframe(slip=0.0, comb=0.0, ts=1)
    fr0.speed_mps = 0.5
    tc.on_frame(fr0, 0.0)
    check("HIGH standing start capped at launch", tc.scale(0.016, 1.0, 0.0) == 0.35)
    tc = TractionController("LOW")
    tc.on_frame(fr0, 0.0)
    check("LOW standing start uncapped", tc.scale(0.016, 1.0, 0.0) == 1.0)

    # cold start: first frame seeds the filters -- no phantom D cut
    tc = TractionController("MEDIUM")
    tc.on_frame(_mkframe(slip=0.8, ts=1), 0.0)
    check("cold start: rate seeded to 0", tc.s_reg_rate == 0.0)
    check("cold start: no phantom cut", tc.scale(0.016, 1.0, 0.0) == 1.0)

    # safety gates
    tc = TractionController("HIGH")
    tc.on_frame(_mkframe(slip=10.0, ts=1), 0.0)
    check("stale telemetry -> passthrough", tc.scale(0.016, 1.0, 10.0) == 1.0)
    tc = TractionController("HIGH")
    tc.on_frame(_mkframe(slip=10.0, gear=0, ts=1), 0.0)
    check("reverse -> passthrough", tc.scale(0.016, 1.0, 0.0) == 1.0)
    tc = TractionController("HIGH")
    tc.on_frame(_mkframe(slip=10.0, race=0, ts=1), 0.0)
    check("menus -> passthrough", tc.scale(0.016, 1.0, 0.0) == 1.0)
    tc = TractionController("HIGH")
    tc.on_frame(_mkframe(slip=10.0, ts=1), 0.0)
    check("no throttle -> passthrough", tc.scale(0.016, 0.0, 0.0) == 1.0)
    tc = TractionController("MEDIUM")
    _sim(tc, [10.0] * 30)
    t = 30 * 0.016
    for i in range(40):
        tc.on_frame(_mkframe(slip=0.1, ts=100 + i), t)
        tc.scale(0.016, 0.0, t)             # throttle lifted
        t += 0.016
    check("lift relaxes integrator", tc.integ > 0.9)

    # tune validation
    m = ModeParams("T", 1.0, 0.20, 1.4, 1.2, 0.03, 1.5, 0.08, 0.5)
    check("tune clamps floor>1", _tunef({"f": 2.0}, "f", m.floor, 0.0, 1.0) == 1.0)
    check("tune rejects nan", _tunef({"f": float("nan")}, "f", 0.08, 0.0, 1.0) == 0.08)
    check("tune rejects junk", _tunef({"f": "x"}, "f", 0.08, 0.0, 1.0) == 0.08)
    check("tune keeps missing", _tunef({}, "f", 0.08, 0.0, 1.0) == 0.08)

    # leak detector
    ld = LeakDetector(emit=lambda m: None)
    t = 0.0
    for i in range(60):                      # cutting to 0.3, game echoes 1.0
        ld.feed(t, 0.30, 1.0, 255)
        t += 0.016
    check("leak detected when echo=user", ld.leaking(t))
    ld = LeakDetector(emit=lambda m: None)
    t = 0.0
    for i in range(60):                      # cutting to 0.3, game echoes 0.3
        ld.feed(t, 0.30, 1.0, int(0.3 * 255))
        t += 0.016
    check("no false leak when echo=sent", not ld.leaking(t))
    ld = LeakDetector(emit=lambda m: None)   # echo lags the cut by ~130 ms
    t = 0.0
    for i in range(60):
        ld.feed(t, 0.30, 1.0, 255 if i < 8 else int(0.3 * 255))
        t += 0.016
    check("no false leak from echo latency", not ld.leaking(t))
    ld = LeakDetector(emit=lambda m: None)   # leak flag ages out after the cut
    t = 0.0
    for i in range(60):
        ld.feed(t, 0.30, 1.0, 255)
        t += 0.016
    was = ld.leaking(t)
    for i in range(60):
        ld.feed(t, 1.0, 1.0, 255)            # cut over
        t += 0.016
    check("leak flag ages out", was and not ld.leaking(t + LeakDetector.HOLD_S))

    # chord switcher
    def chord_seq(seq, tc=None, cs=None):
        """seq: list of (back, pov, t). Returns (forwarded masks, tc, cs)."""
        tc = tc or TractionController("OFF")
        cs = cs or ChordSwitcher(emit=lambda m: None)
        outs = []
        for back, pov, t in seq:
            st = PadState(buttons=(XB["BACK"] if back else 0) | POV_BITS.get(pov, 0),
                          pov=pov)
            outs.append(cs.process(st, t, tc))
        return outs, tc, cs

    # diagonal transient: no dpad bits ever forwarded, exactly one step
    outs, tc, _ = chord_seq([(True, 65535, 0.0), (True, 4500, 0.01),
                             (True, 0, 0.02), (True, 65535, 0.03),
                             (False, 65535, 0.05)])
    check("chord: no dpad leak on diagonals",
          all((o & DPAD_MASK) == 0 for o in outs))
    check("chord: diagonal counts once", tc.mode.name == "LOW")
    check("chord: no BACK replay after chord",
          all((o & XB["BACK"]) == 0 for o in outs))
    # bounce through diagonal mid-press: still one step
    outs, tc, _ = chord_seq([(True, 65535, 0.0), (True, 0, 0.01),
                             (True, 4500, 0.02), (True, 0, 0.03),
                             (False, 65535, 0.05)])
    check("chord: one step per press", tc.mode.name == "LOW")
    # two separate presses: two steps (each press held >= 2 samples -- the
    # 2-sample confirmation filters single-tick hat transients)
    outs, tc, cs = chord_seq([(True, 65535, 0.0), (True, 0, 0.01), (True, 0, 0.02),
                              (True, 65535, 0.03), (True, 0, 0.04),
                              (True, 0, 0.05), (False, 65535, 0.07)])
    check("chord: re-press steps again", tc.mode.name == "MEDIUM")
    # quick tap, no dpad: swallowed while held, replayed on release
    outs, _, _ = chord_seq([(True, 65535, 0.0), (True, 65535, 0.1),
                            (False, 65535, 0.2), (False, 65535, 0.30)])
    check("tap: swallowed while held", (outs[0] | outs[1]) & XB["BACK"] == 0)
    check("tap: replayed on release", (outs[2] & XB["BACK"]) != 0)
    check("tap: replay ends", (outs[3] & XB["BACK"]) == 0)
    # long hold: passes through live from TAP_MS on, no replay after release
    outs, _, _ = chord_seq([(True, 65535, 0.0), (True, 65535, 0.2),
                            (True, 65535, 0.45), (True, 65535, 0.9),
                            (False, 65535, 1.0)])
    check("hold: forwarded live after TAP_MS",
          (outs[2] & XB["BACK"]) and (outs[3] & XB["BACK"]))
    check("hold: no replay on release", (outs[4] & XB["BACK"]) == 0)
    # a tap spanning a loop stall (single observation, long apparent hold):
    # still delivered (replay is NOT duration-gated)
    outs, _, _ = chord_seq([(True, 65535, 0.0), (False, 65535, 0.35)])
    check("stall-spanning tap still delivered", (outs[1] & XB["BACK"]) != 0)
    # leak-latched: replay suppressed (game already saw the physical tap)
    cs = ChordSwitcher(emit=lambda m: None)
    cs.suppress_replay = True
    outs, _, _ = chord_seq([(True, 65535, 0.0), (False, 65535, 0.1)], cs=cs)
    check("leak latched: no replay", (outs[1] & XB["BACK"]) == 0)

    print(f"\n{'ALL PASS' if not fails else f'{len(fails)} FAILURES: {fails}'}")
    return 0 if not fails else 1


# ---------------------------------------------------------------------------
# TCEngine: the reusable control loop (pad -> TC -> virtual pad) on a thread.
# Frontends -- the CLI below, tc_gui.py -- start it, read snapshot(), and call
# set_mode()/set_drivetrain()/set_pad().
# ---------------------------------------------------------------------------
def rumble_test() -> int:
    """Interactive human-in-the-loop rumble/LED diagnosis (--rumble-test).

    Walks the DualSense output-report chain with the USER as the sensor:
    visual tests first (lightbar / player LEDs / mic LED prove whether our
    reports reach the firmware AT ALL), then vibration flag variants, then
    the full engine path. Prints a verdict matrix at the end."""
    print("\n=== DualSense output test -- keep the pad in your hands ===")
    r = DualSenseRumble(emit=print)
    if not r._find_and_open():
        print("FAIL: no DualSense HID interface found. Is the pad connected?")
        return 1
    ol = r._out_len
    print(f"opened: out_len={ol} -> {'Bluetooth' if ol >= 78 else 'USB'}\n")

    def usb(vf0=0, vf1=0, vf2=0, mr=0, ml=0, mute=0, setup=0, bright=0,
            pleds=0, rgb=(0, 0, 0)):
        b = bytearray(ol)
        b[0] = 0x02
        b[1], b[2] = vf0, vf1
        b[3], b[4] = mr, ml
        b[9] = mute
        b[39] = vf2
        b[42], b[43], b[44] = setup, bright, pleds
        b[45], b[46], b[47] = rgb
        return bytes(b)

    def ask(q):
        return input(f"   >>> {q} [y/n] ").strip().lower().startswith("y")

    results = []

    def step(name, rep, question, hold=1.0, via_sor=False, off=None):
        print(f"\n-- {name}")
        sent = (r._set_output_report(rep) if via_sor else r._write_raw(rep))
        time.sleep(hold)
        if off is not None:
            (r._set_output_report(off) if via_sor else r._write_raw(off))
        if not sent:
            print("   (the WRITE ITSELF failed)")
            results.append((name, "write-failed"))
            return False
        got = ask(question)
        results.append((name, "yes" if got else "no"))
        return got

    zero = usb()
    # --- visual: does ANY report reach the firmware? -----------------------
    step("lightbar RED (flags vf1=0x04)",
         usb(vf1=0x04, rgb=(255, 0, 0)),
         "did the lightbar turn red?", hold=1.5, off=zero)
    step("player LEDs all-on (vf1=0x10)",
         usb(vf1=0x10, bright=0, pleds=0x1F),
         "did all 5 player LEDs light up?", hold=1.5, off=zero)
    # --- vibration variants -------------------------------------------------
    step("vibration A: current flags (vf0=0x03, vf2=0x04)",
         usb(vf0=0x03, vf2=0x04, mr=255, ml=255),
         "did it vibrate?", hold=1.2, off=usb(vf0=0x03, vf2=0x04))
    step("vibration B: compat-only (vf0=0x01)",
         usb(vf0=0x01, mr=255, ml=255),
         "did it vibrate?", hold=1.2, off=usb(vf0=0x01))
    step("vibration C: kitchen-sink flags (vf0=0xFF, vf1=0xF7)",
         usb(vf0=0xFF, vf1=0xF7, vf2=0x04, mr=255, ml=255),
         "did it vibrate?", hold=1.2, off=usb(vf0=0xFF, vf1=0xF7, vf2=0x04))
    step("vibration D: same as A but via HidD_SetOutputReport",
         usb(vf0=0x03, vf2=0x04, mr=255, ml=255),
         "did it vibrate?", hold=1.2, via_sor=True,
         off=usb(vf0=0x03, vf2=0x04))
    r._write_raw(zero)
    r._close()

    # --- engine path (only meaningful if some variant vibrated) ------------
    if any(n.startswith("vibration") and v == "yes" for n, v in results):
        print("\n-- engine path: full production chain (virtual pad FFB -> "
              "writer thread -> pad)")
        eng = TCEngine(emit=lambda m: print(f"   {m}"))
        eng.start()
        time.sleep(2.5)
        try:
            xi = ctypes.windll.xinput1_4

            class _VIB(ctypes.Structure):
                _fields_ = [("l", ctypes.c_ushort), ("r", ctypes.c_ushort)]

            class _ST(ctypes.Structure):
                _fields_ = [("n", ctypes.c_uint32), ("g", ctypes.c_byte * 12)]

            slots = [i for i in range(4)
                     if xi.XInputGetState(i, ctypes.byref(_ST())) == 0]
            for i in slots:
                xi.XInputSetState(i, ctypes.byref(_VIB(65535, 65535)))
            time.sleep(1.2)
            for i in slots:
                xi.XInputSetState(i, ctypes.byref(_VIB(0, 0)))
            time.sleep(0.3)
        finally:
            eng.stop()
        results.append(("engine path", "yes" if
                        input("   >>> did it vibrate? [y/n] ").strip().lower()
                        .startswith("y") else "no"))

    print("\n=== results ===")
    for n, v in results:
        print(f"  {v:>12s}  {n}")
    vis = [v for n, v in results if "lightbar" in n or "LEDs" in n]
    vib = [v for n, v in results if n.startswith("vibration")]
    print("\n=== reading ===")
    if all(v != "yes" for v in vis):
        print("  Nothing visual changed -> our reports never reach the pad's")
        print("  firmware (wrong interface/report path), even though Windows")
        print("  accepts the writes. Next step: report-descriptor dump.")
    elif all(v != "yes" for v in vib):
        print("  LEDs respond but motors don't -> writes reach the firmware;")
        print("  the vibration flag combination is wrong for this firmware.")
    else:
        good = [n for n, v in results if n.startswith("vibration") and v == "yes"]
        print(f"  Working vibration variant(s): {', '.join(good)}")
        print("  -> adopt those flags in ds_usb_report.")
    return 0


class TCEngine:
    def __init__(self, mode="MEDIUM", port=7777, joy_id=None, tick=0.003,
                 game_exe=GAME_EXE_DEFAULT, duration=0.0, dialog_clear=True,
                 long_front=False, long_rear=False, abs_mode="MEDIUM",
                 custom_tc=None, custom_abs=None, rumble=True, emit=print):
        self.cfg = dict(port=port, joy_id=joy_id, tick=tick, game_exe=game_exe,
                        duration=duration, dialog_clear=dialog_clear,
                        custom_tc=custom_tc, custom_abs=custom_abs,
                        rumble=rumble)
        self.emit = emit
        self.tc = TractionController(mode)
        self.tc.long_only_front = bool(long_front)
        self.tc.long_only_rear = bool(long_rear)
        self.abs_ctl = TractionController(abs_mode, channel="abs")
        self.abs_learn = AbsLearner()
        self.abs_learning = False
        self.leak_brk = LeakDetector(emit=emit, what="brake")
        self._rec_req: str | None = None      # GUI thread -> engine thread
        self._rec_lock = threading.Lock()
        self.recording = False                # engine-thread owned; bool read
        self.rec_rows = 0                     # is GIL-safe from the GUI thread
        self.rec_name = ""
        self.rec_t0 = 0.0
        self._lt_start = False                # leak-test request (GIL-safe bool)
        self._lt_left = 0.0                   # seconds remaining (for the GUI)
        self.leak = LeakDetector(emit=emit)
        self.chord = ChordSwitcher(emit=emit)
        self._want_pad: int | None = None
        self._stop = False
        self.finished = threading.Event()
        self.error: str | None = None
        self._snap: dict = {}
        self.thread: threading.Thread | None = None

    # -- frontend API -------------------------------------------------------
    def start(self) -> None:
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def stop(self, join_s: float = 5.0) -> None:
        self._stop = True
        if self.thread is not None:
            self.thread.join(join_s)

    def snapshot(self) -> dict:
        return self._snap

    def set_mode(self, name: str) -> None:
        if self.tc.mode.name != name.upper():
            self.tc.set_mode(name)
            beep_mode(self.tc.mode_i)
            self.emit(f"[TC] mode -> {self.tc.mode.name}")

    def set_drivetrain(self, val: int | None) -> None:
        """None = auto (telemetry), else 0=FWD 1=RWD 2=AWD."""
        if self.tc.drivetrain_override != val:
            self.tc.drivetrain_override = val
            label = "AUTO" if val is None else {0: "FWD", 1: "RWD", 2: "AWD"}[val]
            self.emit(f"[TC] driven wheels -> {label}")

    def start_leak_test(self) -> None:
        self._lt_start = True

    def start_recording(self) -> None:
        with self._rec_lock:
            self._rec_req = "start"

    def stop_recording(self) -> None:
        with self._rec_lock:
            self._rec_req = "stop"

    def _ctl(self, channel: str) -> TractionController:
        return self.tc if channel == "tc" else self.abs_ctl

    def long_only(self, channel: str, axle: str) -> bool:
        """Authoritative long-only flag read (snapshots can freeze)."""
        ctl = self._ctl(channel)
        return ctl.long_only_front if axle == "front" else ctl.long_only_rear

    def set_mode_ch(self, channel: str, name: str) -> None:
        ctl = self._ctl(channel)
        if ctl.mode.name != name.upper():
            ctl.set_mode(name)
            beep_mode(ctl.mode_i)
            self.emit(f"[{'TC' if channel == 'tc' else 'ABS'}] mode -> {ctl.mode.name}")
            # (learning restart on ABS mode change is handled centrally by
            # the loop's mode watcher -- GUI, F-keys and the BACK chord all
            # change the mode through different paths, one hook covers all)

    def set_custom_target_ch(self, channel: str, pct: float) -> None:
        """Set a channel's CUSTOM setpoint, in percent (e.g. 92)."""
        modes = MODES if channel == "tc" else ABS_MODES
        t = max(CUSTOM_MIN, min(custom_max(modes), pct / 100.0))
        if abs(modes[MODE_IDX["CUSTOM"]].target - t) < 1e-9:
            return
        apply_custom_target(t, modes)
        dump_tune()       # keep the hot-reload file in agreement (and persist)
        tag = "TC" if channel == "tc" else "ABS"
        self.emit(f"[{tag}] CUSTOM setpoint -> {t * 100:.0f}%")
        if (channel == "abs" and self.abs_learning
                and self.abs_ctl.mode.name == "CUSTOM"):
            self.abs_learn.reset(min(t, AbsLearner.C_MAX))
            self.emit(f"[ABS] LEARNING restarted from {t * 100:.0f}%")

    def set_custom_target(self, pct: float) -> None:
        self.set_custom_target_ch("tc", pct)

    def set_long_only_ch(self, channel: str, axle: str, on: bool) -> None:
        """axle 'front'/'rear': regulate only that axle's fwd/back component."""
        ctl = self._ctl(channel)
        cur = ctl.long_only_front if axle == "front" else ctl.long_only_rear
        if cur == bool(on):
            return
        if axle == "front":
            ctl.long_only_front = bool(on)
        else:
            ctl.long_only_rear = bool(on)
        what = ("LONG-ONLY (fwd/back slip only)" if on
                else "full friction circle")
        tag = "TC" if channel == "tc" else "ABS"
        self.emit(f"[{tag}] {axle} wheels -> {what}")

    def set_long_only(self, axle: str, on: bool) -> None:
        self.set_long_only_ch("tc", axle, on)

    def _on_abs_mode_change(self) -> None:
        """Learning always starts from what the USER just picked, no matter
        which input changed the ABS mode (GUI button, F1-F4, BACK chord)."""
        if not self.abs_learning:
            return
        m = self.abs_ctl.mode
        if m.name == "OFF":
            self.set_abs_learning(False)
        else:
            self.abs_learn.reset(min(m.target, AbsLearner.C_MAX))
            self.emit(f"[ABS] LEARNING restarted from {m.target * 100:.0f}%")

    def set_chord_mod(self, name: str) -> None:
        """Pick which held button arms the d-pad mode chord."""
        name = (name or "").upper()
        if name not in CHORD_MODS:
            return
        if self.chord.mod != XB[name]:
            self.chord.mod = XB[name]
            self.emit(f"[pad] chord modifier -> hold {name} + d-pad")

    def set_abs_learning(self, on: bool) -> None:
        """LEARN toggle: adaptive ABS setpoint. Starts from the USER's active
        ABS setpoint and adapts from braking evidence; corners are derated
        live by the friction circle."""
        if bool(on) == self.abs_learning:
            return
        if on:
            m = self.abs_ctl.mode
            if m.name == "OFF":
                self.emit("[ABS] pick an ABS mode before enabling LEARNING")
                return
            self.abs_learn.reset(min(m.target, AbsLearner.C_MAX))
            self.abs_learning = True
            self.emit(f"[ABS] LEARNING on -- starting from your "
                      f"{m.target * 100:.0f}% setpoint, adapting from braking "
                      f"evidence (corners derated by lateral load)")
        else:
            self.abs_learning = False
            self.abs_ctl.target_override = None
            self.emit(f"[ABS] LEARNING off -- last learned ceiling "
                      f"{self.abs_learn.ceiling * 100:.0f}%; mode setpoint back "
                      f"in charge")

    def set_pad(self, joy_id: int) -> None:
        self._want_pad = joy_id

    # -- the loop -----------------------------------------------------------
    def _run(self) -> None:
        # every exit path must set finished and surface the error, or the
        # frontends (CLI poll loop, GUI) wait forever on a dead thread
        try:
            self._run_inner()
        except Exception:
            import traceback
            self.error = self.error or f"engine crashed:\n{traceback.format_exc(limit=8)}"
            self.emit(f"[TC] ERROR: {self.error}")
        finally:
            self.finished.set()

    def _run_inner(self) -> None:
        c = self.cfg
        try:
            import vgamepad as vg
        except Exception as e:
            self.error = f"vgamepad unavailable ({e}) -- is ViGEmBus installed?"
            self.emit(f"[TC] ERROR: {self.error}")
            return
        pm = load_padmap(emit=self.emit)
        joy_id = find_pad(c["joy_id"])
        if joy_id is None:
            # WAIT rather than die: quitting here left the engine permanently
            # stopped, so plugging the controller in afterwards could never
            # recover it and the app had to be restarted.
            self.emit("[pad] no controller found -- waiting for you to plug "
                      "one in (the engine starts by itself when you do)")
            while joy_id is None and not self._stop:
                time.sleep(0.5)
                joy_id = find_pad(c["joy_id"])
            if joy_id is None:                  # stopped while waiting
                return
            self.emit("[pad] controller detected -- starting")
        pad_ident = joy_identity(joy_id)
        pad_name = pad_display_name(pad_ident)
        self.emit(f"[pad] physical pad: winmm id {joy_id} ({pad_name})")

        tc, leak, chord = self.tc, self.leak, self.chord
        ab = self.abs_ctl
        abs_mode_seen = ab.mode.name    # the loop's central mode watcher
        fkeys: dict[int, bool] = {}
        feed = gp = logf = None
        rumble: DualSenseRumble | None = None
        recorder: TelemetryRecorder | None = None
        try:
            feed = TelemetryFeed(c["port"])
            gp = vg.VX360Gamepad()
            if c.get("rumble", True):
                # the game rumbles the VIRTUAL pad; forward it to the real one
                rumble = DualSenseRumble(emit=self.emit)
                rumble.start()

                # NB: vgamepad validates this signature by PARAMETER NAMES
                def _ffb(client, target, large_motor, small_motor,
                         led_number, user_data):
                    rumble.set(large_motor, small_motor)
                try:
                    gp.register_notification(callback_function=_ffb)
                except Exception as e:
                    self.emit(f"[rumble] FFB notifications unavailable ({e}) "
                              f"-- vibration stays off")
            reload_tune()    # pick up last session's tuning (incl. CUSTOM %)
            # CLI-requested custom targets apply AFTER the reload (so they win
            # over the persisted value instead of being wiped/overridden)
            if c["custom_tc"] is not None:
                self.set_custom_target_ch("tc", c["custom_tc"])
            if c["custom_abs"] is not None:
                self.set_custom_target_ch("abs", c["custom_abs"])
            dump_tune()      # then rewrite the file in the current schema
            os.makedirs(REC_DIR, exist_ok=True)
            logf = open(LOG_PATH, "w", newline="")
            logw = csv.writer(logf)
            logw.writerow(["t", "mode", "user_thr", "sent_thr", "scale", "s_long",
                           "s_comb", "excess", "game_accel", "spd_kmh", "gear",
                           "leak", "interventions", "s_reg", "long_f", "long_r",
                           "abs_mode", "user_brk", "sent_brk", "abs_scale",
                           "abs_s_reg", "game_brake", "abs_interventions"])
            try:
                winmm.timeBeginPeriod(1)
            except Exception:
                pass

            last_tune = 0.0
            last_log_ts = None
            last_rec_ts = None
            lt_until = 0.0            # leak-test window end (0 = inactive)
            lt_echo: list = []        # game-accepted throttle samples
            lt_total = 0              # telemetry frames seen in the window
            lt_last_ts = None         # dedup: one sample per telemetry frame
            learn_last_ts = None      # dedup for the ABS learner
            learn_hist = deque(maxlen=8)   # (sent_brk, asc) ~0.13 s ago: the
                                      # frame's physics answers THAT torque
                                      # (transport delay ~120 ms), so the
                                      # evidence gates judge the delayed
                                      # actuator state. Full 8 frames on
                                      # purpose: slip PEAKS ~8 frames after
                                      # the cut command, and that peak is the
                                      # deepest probe; recovery leakage past
                                      # the true delay is caught by the
                                      # learner's slip-collapse (RECOV) guard
            learn_bd = 0.0            # last delay-matched brake / open-loop
            learn_ol = False          # flag (recorded as the evidence trail)
            learn_probe_until = 0.0   # active probe pulse window (see below)
            learn_probe_last = -1e9
            lt_tc_i, lt_ab_i = tc.mode_i, ab.mode_i   # light language state
            lt_flash_until = 0.0
            lt_flash_ch = "tc"
            lt_last = None
            pad_missing = False
            n_loops = 0
            n_log_rows = 0
            prev_t = 0.0
            hz_t0 = 0.0
            hz_n0 = 0
            hz_val = 0.0
            t0 = time.perf_counter()
            while not self._stop:
                t = time.perf_counter() - t0
                if c["duration"] and t > c["duration"]:
                    self.emit("[TC] duration reached")
                    break

                if t - last_tune > 0.5:               # hot-reload tune ~2x/s
                    last_tune = t
                    reload_tune()

                # full-telemetry recording (Record button): start/stop requests
                # are handled here on the engine thread so the file is only ever
                # touched by one thread
                with self._rec_lock:
                    rec_req, self._rec_req = self._rec_req, None
                if rec_req == "start" and recorder is None:
                    try:
                        os.makedirs(REC_DIR, exist_ok=True)
                        path = os.path.join(
                            REC_DIR, time.strftime("recording_%Y%m%d_%H%M%S.csv"))
                        recorder = TelemetryRecorder(path)
                        last_rec_ts = None
                        self.rec_t0 = t
                        self.rec_rows = 0
                        self.rec_name = os.path.basename(path)
                        self.recording = True
                        self.emit(f"[rec] recording telemetry -> {self.rec_name}")
                    except Exception as e:
                        self.emit(f"[rec] could not start: {e}")
                        recorder = None
                        self.recording = False
                elif rec_req == "stop" and recorder is not None:
                    recorder.close()
                    self.emit(f"[rec] stopped: {recorder.rows} frames -> "
                              f"{self.rec_name}")
                    recorder = None
                    self.recording = False

                poll_fkeys(fkeys, tc, c["game_exe"], emit=self.emit, abs_ctl=ab)
                if ab.mode.name != abs_mode_seen:     # any input path
                    abs_mode_seen = ab.mode.name
                    self._on_abs_mode_change()

                if self._want_pad is not None:        # frontend pad switch
                    wid, self._want_pad = self._want_pad, None
                    if wid != joy_id:
                        ident = joy_identity(wid)
                        if joy_read(wid) is None:
                            self.emit(f"[pad] id {wid} not responding -- kept {joy_id}")
                        elif ident is None or (ident[0], ident[1]) == VIGEM_X360_ID:
                            self.emit(f"[pad] id {wid} is the virtual pad -- refused")
                        else:
                            joy_id = wid
                            pad_ident = ident
                            pad_name = pad_display_name(ident)
                            self.emit(f"[pad] switched to winmm id {wid} ({pad_name})")

                j = joy_read(joy_id)
                if j is None:
                    if not pad_missing:
                        pad_missing = True
                        self.emit("[pad] disconnected -- outputting neutral")
                    if self._snap:
                        # keep the mode/toggle display live during the outage
                        self._snap = {**self._snap, "pad_ok": False,
                                      "mode": tc.mode.name,
                                      "long_front": tc.long_only_front,
                                      "long_rear": tc.long_only_rear,
                                      "abs_mode": ab.mode.name,
                                      "abs_long_front": ab.long_only_front,
                                      "abs_long_rear": ab.long_only_rear}
                    gp.reset()
                    gp.update()
                    time.sleep(0.25)
                    hz_t0, hz_n0 = t, n_loops     # don't average across the outage
                    jj = find_pad(c["joy_id"], expect=pad_ident)
                    if jj is not None:
                        joy_id = jj
                        pad_missing = False
                        self.emit(f"[pad] reconnected (winmm id {joy_id})")
                    continue
                pad_missing = False

                st = decode_pad(j, pm)
                fr, ft = feed.latest()
                if fr is not None:
                    ftime = ft - t0 if ft > 0 else -1e9
                    tc.on_frame(fr, ftime)
                    ab.on_frame(fr, ftime)

                dt = max(1e-4, min(0.05, t - prev_t)) if n_loops else 1e-4
                prev_t = t
                sc = tc.scale(dt, st.rt, t)
                sent_rt = st.rt * sc
                asc = ab.scale(dt, st.lt, t)       # ABS: same law, brake pedal
                # LEARN probe pulse: with the ABS regulating AT the cap, slip
                # beyond it is never observable (the cut suppresses the very
                # evidence needed to climb -- proven twice on-rig: the
                # ceiling stalls wherever it starts). So while learning,
                # briefly release the cut during hard straight braking and
                # let the delay window sample the surface past the cap; the
                # learner turns the response into GRIP or FALL evidence and
                # regulation resumes. Bounded: 0.12 s at most every 2 s,
                # full pedal, near-straight, >72 km/h, learning only.
                if (self.abs_learning and t < learn_probe_until):
                    asc = 1.0
                elif (self.abs_learning and asc < 0.9 and st.lt >= 0.9
                        and fr is not None and fr.speed_mps > 20.0
                        and self.abs_learn.lat_f <= 12.0
                        and -4.0 < fr.ay < 12.0
                        and t - learn_probe_last > 2.0):
                    # lat gate 12 (not 6): braking-zone corrections are
                    # normal at speed and must not starve the probes; the
                    # ay band skips crests -- never probe airborne wheels
                    learn_probe_until = t + 0.12
                    learn_probe_last = t
                    asc = 1.0
                sent_lt = st.lt * asc

                # in-engine LEAK TEST: for 6 s, force a known 30% throttle on
                # the (already-live) virtual pad while the user holds their
                # trigger, and read back what the game accepted -- no pad
                # teardown, so no controller-disconnect churn
                if self._lt_start:
                    self._lt_start = False
                    lt_until = t + 6.0
                    lt_echo, lt_total, lt_last_ts = [], 0, None
                    # NB: the user must WORK the trigger, not hold it still.
                    # The game switches to whichever pad shows activity, so a
                    # frozen physical trigger never provokes the switch and the
                    # test reports a false CLEAN (measured on-rig).
                    self.emit("[leak-test] SQUEEZE AND RELEASE the throttle "
                              "trigger repeatedly for 6 s (car somewhere safe "
                              "-- we send a steady 30%)...")
                if lt_until > 0.0:
                    self._lt_left = max(0.0, lt_until - t)
                    sent_rt = 0.30                 # command a known value
                    # sample per TELEMETRY frame (the 250 Hz loop would
                    # otherwise count the same frame ~4x), and only while the
                    # user is actually on the throttle -- that's when their
                    # input differs enough from our 30% for a leak to show
                    if (fr is not None and (t - tc.last_frame_t) < STALE_S
                            and fr.timestamp_ms != lt_last_ts):
                        lt_last_ts = fr.timestamp_ms
                        lt_total += 1
                        if st.rt > 0.5:
                            lt_echo.append(fr.accel / 255.0)
                    if t >= lt_until:
                        lt_until = 0.0
                        self._lt_left = 0.0
                        verdict, avg, over = leak_verdict(lt_total, lt_echo)
                        if verdict == "held":
                            self.emit(f"[leak-test] INCONCLUSIVE -- only "
                                      f"{len(lt_echo)} throttle frames (need "
                                      f"{LEAK_MIN_SAMPLES}); re-run and keep "
                                      f"squeezing the trigger through the 6 s")
                        elif verdict == "notelem":
                            self.emit("[leak-test] INCONCLUSIVE -- no telemetry "
                                      "(is the game focused with Data Out on?)")
                        elif verdict == "leaking":
                            self.emit(f"[leak-test] LEAKING: your trigger reached "
                                      f"the game on {over:.0%} of frames (mean "
                                      f"{avg:.0%} vs 30% sent). If Steam is "
                                      f"running, disable Steam Input for the game "
                                      f"and restart Steam -- Steam re-broadcasts "
                                      f"the pad even when HidHide hides it.")
                        else:
                            self.emit(f"[leak-test] CLEAN: game followed 30% "
                                      f"(got {avg:.0%}, 0 stray frames) -- hiding "
                                      f"works, TC is real.")

                chord.suppress_replay = (leak.ever_leaked
                                         or self.leak_brk.ever_leaked)
                btn = chord.process(st, t, tc, abs_ctl=ab)
                gp.report.wButtons = btn
                gp.report.sThumbLX = st.lx
                gp.report.sThumbLY = st.ly
                gp.report.sThumbRX = st.rx
                gp.report.sThumbRY = st.ry
                gp.report.bLeftTrigger = trig_byte(sent_lt)
                gp.report.bRightTrigger = trig_byte(sent_rt)
                gp.update()

                # controller light language (see light_state)
                if rumble is not None:
                    if tc.mode_i != lt_tc_i or ab.mode_i != lt_ab_i:
                        lt_flash_ch = "abs" if ab.mode_i != lt_ab_i else "tc"
                        lt_flash_until = t + 1.5
                        lt_tc_i, lt_ab_i = tc.mode_i, ab.mode_i
                    lstate = light_state(
                        t, tc.mode.name, tc.mode.target, ab.mode.name,
                        (ab.target_override if ab.target_override is not None
                         else ab.mode.target),
                        sc < 0.98, asc < 0.98,
                        chord.back_down_t is not None and not chord.committed,
                        chord.last_ch or "tc",
                        tc.mode_i, ab.mode_i, lt_flash_until, lt_flash_ch)
                    if lstate != lt_last:
                        lt_last = lstate
                        rumble.set_lights(*lstate)

                if fr is not None and (t - tc.last_frame_t) < STALE_S:
                    if lt_until == 0.0:            # skip during the explicit test
                        leak.feed(t, sent_rt, st.rt, fr.accel)
                    self.leak_brk.feed(t, sent_lt, st.lt, fr.brake)
                    # adaptive ABS setpoint: learn per telemetry frame, and
                    # keep the controller's live cap = ceiling x corner derate
                    if self.abs_learning and fr.timestamp_ms != learn_last_ts:
                        learn_last_ts = fr.timestamp_ms
                        # longitudinal (lockup-direction) slip only: cornering
                        # must not inflate the learner's slip reading
                        s_long_l, _, _ = abs_slips(fr)
                        # evidence = open-loop probes only: the cut-onset
                        # overshoot rides on torque sent BEFORE the cut bit
                        # (transport delay), so gate on the DELAYED sent
                        # brake/cut depth; min() with the live pedal so a
                        # brake release can't masquerade as grip falloff
                        sent_d, asc_d = (learn_hist[0] if len(learn_hist) == 8
                                         else (0.0, 0.0))
                        learn_hist.append((sent_lt, asc))
                        learn_bd = min(sent_d, st.lt)
                        learn_ol = asc_d >= 0.9
                        self.abs_learn.update(0.016, s_long_l, -fr.az,
                                              abs(fr.ax), learn_bd,
                                              fr.speed_mps, open_loop=learn_ol,
                                              vert=fr.ay)
                        ab.target_override = self.abs_learn.effective_cap()
                    # full-telemetry recording: every fresh frame, any game
                    # state (menus/free-roam included -- capture everything)
                    if recorder is not None and fr.timestamp_ms != last_rec_ts:
                        last_rec_ts = fr.timestamp_ms
                        lrn = self.abs_learn
                        dref = lrn.ref_decel(fr.speed_mps)
                        recorder.write(fr, [round(t, 3), round(sent_rt, 3),
                                            round(sent_lt, 3), tc.mode.name,
                                            ab.mode.name, round(sc, 3),
                                            round(asc, 3),
                                            # learning evidence trail
                                            int(self.abs_learning),
                                            round(lrn.ceiling, 3),
                                            round(ab.target_override, 3)
                                            if ab.target_override is not None
                                            else 0.0,
                                            round(lrn.alat_max, 2),
                                            round(dref, 2),
                                            lrn.verdict if self.abs_learning
                                            else -1,
                                            round(learn_bd, 3), int(learn_ol)])
                        self.rec_rows = recorder.rows
                    # one log row per telemetry frame while in the driving world
                    if fr.is_race_on and fr.timestamp_ms != last_log_ts:
                        last_log_ts = fr.timestamp_ms
                        logw.writerow([round(t, 3), tc.mode.name, round(st.rt, 3),
                                       round(sent_rt, 3), round(sc, 3),
                                       round(tc.s_long, 3), round(tc.s_comb, 3),
                                       round(tc.excess, 3), fr.accel,
                                       round(fr.speed_mps * 3.6, 1), fr.gear,
                                       int(leak.leaking(t)), tc.interventions,
                                       round(tc.s_reg, 3),
                                       int(tc.long_only_front),
                                       int(tc.long_only_rear),
                                       ab.mode.name, round(st.lt, 3),
                                       round(sent_lt, 3), round(asc, 3),
                                       round(ab.s_reg, 3), fr.brake,
                                       ab.interventions])
                        n_log_rows += 1
                        if n_log_rows % 60 == 0:   # ~1x/s, keeps the file live-readable
                            logf.flush()

                n_loops += 1
                if t - hz_t0 >= 1.0:                 # windowed loop rate
                    hz_val = (n_loops - hz_n0) / max(t - hz_t0, 1e-3)
                    hz_t0, hz_n0 = t, n_loops
                self._snap = {
                    "t": t,
                    "mode": tc.mode.name,
                    "target": tc.mode.target,
                    "user_thr": st.rt, "sent_thr": sent_rt, "scale": sc,
                    "brake": st.lt,
                    "s_long": tc.s_long, "s_comb": tc.s_comb, "s_reg": tc.s_reg,
                    "long_front": tc.long_only_front,
                    "long_rear": tc.long_only_rear,
                    "abs_mode": ab.mode.name,
                    "abs_target": (ab.target_override
                                   if ab.target_override is not None
                                   else ab.mode.target),
                    "abs_learn": self.abs_learning,
                    "learn_ceiling": self.abs_learn.ceiling,
                    "learn_alat_g": self.abs_learn.alat_max / 9.81,
                    "sent_brk": sent_lt, "abs_scale": asc,
                    "abs_s_reg": ab.s_reg, "abs_s_long": ab.s_long,
                    "abs_long_front": ab.long_only_front,
                    "abs_long_rear": ab.long_only_rear,
                    "abs_interventions": ab.interventions,
                    "wheels": None if fr is None else {
                        "FL": (fr.slip_ratio_fl, fr.slip_angle_fl, fr.combined_slip_fl),
                        "FR": (fr.slip_ratio_fr, fr.slip_angle_fr, fr.combined_slip_fr),
                        "RL": (fr.slip_ratio_rl, fr.slip_angle_rl, fr.combined_slip_rl),
                        "RR": (fr.slip_ratio_rr, fr.slip_angle_rr, fr.combined_slip_rr)},
                    "telem_age": t - tc.last_frame_t,
                    "race": bool(fr.is_race_on) if fr is not None else False,
                    "speed_kmh": fr.speed_mps * 3.6 if fr is not None else 0.0,
                    "gear": fr.gear if fr is not None else 0,
                    "drivetrain": fr.drivetrain if fr is not None else 2,
                    "drivetrain_eff": (tc.drivetrain_override
                                       if tc.drivetrain_override is not None
                                       else (fr.drivetrain if fr is not None else 2)),
                    "leak": leak.leaking(t) or self.leak_brk.leaking(t),
                    "ever_leaked": (leak.ever_leaked
                                    or self.leak_brk.ever_leaked),
                    "ffb_events": rumble.n_events if rumble is not None else -1,
                    "ffb_writes": rumble.n_writes if rumble is not None else -1,
                    "recording": self.recording, "rec_rows": self.rec_rows,
                    "rec_name": self.rec_name,
                    "rec_secs": (t - self.rec_t0) if self.recording else 0.0,
                    "leak_test_left": self._lt_left,
                    "interventions": tc.interventions,
                    "pad_ok": True, "pad_id": joy_id, "pad_name": pad_name,
                    "hz": hz_val,
                }

                time.sleep(c["tick"])
        finally:
            if rumble is not None:
                try:
                    gp.unregister_notification()
                except Exception:
                    pass
                rumble.stop()               # motors off + handle closed
            if gp is not None:
                try:
                    gp.reset()
                    gp.update()
                except Exception:
                    pass
            try:
                winmm.timeEndPeriod(1)
            except Exception:
                pass
            if feed is not None:
                feed.close()
            if logf is not None:
                try:
                    logf.close()
                except Exception:
                    pass
            if recorder is not None:        # finalize an in-progress recording
                recorder.close()            # (Stop, engine stop, or crash)
                self.emit(f"[rec] finalized: {recorder.rows} frames -> "
                          f"{self.rec_name}")
                self.recording = False
            if gp is not None:
                # removing the virtual pad pops FH6's keyboard-only Controller-
                # Disconnected dialog; clear it so the session is left usable
                # (critical when HidHide hides the physical pad: no controller
                # exists to navigate with until this program runs again)
                try:
                    del gp
                except Exception:
                    pass
                if c["dialog_clear"]:
                    try:
                        clear_disconnect_dialog(c["game_exe"], emit=self.emit)
                    except Exception:
                        pass


# ---------------------------------------------------------------------------
# CLI frontend
# ---------------------------------------------------------------------------
def run(args) -> int:
    disable_quickedit()
    sw = StatusWriter()
    engine = TCEngine(mode=args.mode.upper(), port=args.port, joy_id=args.joy_id,
                      tick=args.tick, game_exe=args.game_exe,
                      duration=args.duration,
                      dialog_clear=not args.no_dialog_clear,
                      long_front=args.long_only_front,
                      long_rear=args.long_only_rear,
                      abs_mode=args.abs_mode.upper(),
                      custom_tc=args.custom_target,
                      custom_abs=args.abs_custom_target,
                      rumble=not args.no_rumble, emit=sw.event)
    engine.set_chord_mod(args.chord_mod)
    print(f"TC mode: {args.mode.upper()}   (F5 off / F6 low / F7 med / F8 high "
          f"while FH6 or this console is focused, or hold "
          f"{args.chord_mod.upper()} + dpad up/down)")
    print(f"telemetry: UDP :{args.port} | tune: {TUNE_PATH} | log: {LOG_PATH}")
    engine.start()
    try:
        while not engine.finished.is_set():
            time.sleep(0.25)
            s = engine.snapshot()
            if not s:
                continue
            age = s["telem_age"] * 1000
            tag = ("no-pad" if not s.get("pad_ok") else
                   "no-telem" if age > 1000 else
                   "menu" if not s["race"] else "drive")
            lk = "LEAK!" if s["leak"] else "ok"
            sw.set_status(
                f"[{s['mode']:6s}] thr {s['user_thr']:4.2f}->{s['sent_thr']:4.2f} "
                f"x{s['scale']:4.2f} | slip C{s['s_comb']:5.2f} "
                f"| ABS[{s.get('abs_mode', '?'):6s}] brk "
                f"{s['brake']:4.2f}->{s.get('sent_brk', 0):4.2f} "
                f"| {tag:8s} leak:{lk} cuts:{s['interventions']:3d}"
                f"/{s.get('abs_interventions', 0):3d} | {s['hz']:3.0f}Hz   ")
    except KeyboardInterrupt:
        print("\nstopping (Ctrl+C)")
    finally:
        engine.stop()
        if engine.error:
            print(f"\nERROR: {engine.error}", flush=True)
    return 1 if engine.error else 0


# ---------------------------------------------------------------------------
# --leak-test: guided probe -- is the physical pad hidden from the game?
# ---------------------------------------------------------------------------
def leak_test(args) -> int:
    import vgamepad as vg

    disable_quickedit()
    pm = load_padmap()
    joy_id = find_pad(args.joy_id)
    pad_readable = joy_id is not None
    if not pad_readable:
        print("NOTE: this process cannot read the physical pad (unplugged, or "
              "HidHide is hiding it without python whitelisted). The probe will "
              "run echo-only: it cannot verify you were actually holding the "
              "trigger, so treat a CLEAN verdict as tentative.")
    feed = TelemetryFeed(args.port)
    gp = vg.VX360Gamepad()
    print("\nLEAK TEST: get the car somewhere safe in FH6 (driveable, on the")
    print("ground, in a forward gear). This tool commands a steady 30% throttle")
    print("on the virtual pad while YOU hold the physical RIGHT TRIGGER fully;")
    print("the game's echoed accepted throttle tells the truth.\n")
    try:
        if pad_readable:
            print("HOLD the physical right trigger fully down now (waiting)...")
            t0 = time.perf_counter()
            while True:
                j = joy_read(joy_id)
                rt = decode_pad(j, pm).rt if j is not None else 0.0
                if rt >= 0.9:
                    break
                if time.perf_counter() - t0 > 15.0:
                    print("\nINCONCLUSIVE: no full trigger pull detected in 15 s. "
                          "If you WERE pulling it, the trigger mapping is wrong -- "
                          "run --map first.")
                    return 2
                time.sleep(0.02)
            print("trigger detected -- sampling for 6 s, keep holding...\n")
        else:
            print("Hold the trigger when the countdown ends...")
            for k in range(5, 0, -1):
                print(f"  {k}...")
                time.sleep(1.0)
        echoes = []
        held = 0
        total = 0
        t0 = time.perf_counter()
        while time.perf_counter() - t0 < 6.0:
            gp.right_trigger_float(value_float=0.30)
            gp.update()
            if pad_readable:
                j = joy_read(joy_id)
                rt = decode_pad(j, pm).rt if j is not None else 0.0
                total += 1
                if rt >= 0.9:
                    held += 1
                else:
                    time.sleep(0.01)
                    continue          # only judge echo while the pull is real
            fr, ft = feed.latest()
            if fr is not None and time.perf_counter() - ft < 0.3:
                echoes.append(fr.accel)
                print(f"\r  cmd 30% | game echo {fr.accel / 255:5.0%}   ",
                      end="", flush=True)
            time.sleep(0.01)
    finally:
        gp.reset()
        gp.update()
        feed.close()
    if pad_readable and total and held / total < 0.5:
        print(f"\n\nINCONCLUSIVE: the trigger was only held {held / total:.0%} of "
              "the window -- re-run and keep it pinned.")
        return 2
    if not echoes:
        print("\nno telemetry received -- is FH6 running with Data Out on "
              f"127.0.0.1:{args.port}?")
        return 1
    avg = sum(echoes) / len(echoes) / 255.0
    print(f"\n\naverage game-accepted throttle: {avg:.0%} (commanded 30%)")
    if avg > 0.55:
        print("VERDICT: LEAKING -- the game still sees the physical pad. TC cuts")
        print("will be overridden. Install HidHide, hide the pad, whitelist python")
        print("(see the docstring at the top of this file).")
        return 1
    caveat = "" if pad_readable else " (echo-only: physical hold was unverified)"
    print(f"VERDICT: CLEAN{caveat} -- the game follows the virtual pad; TC will work.")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description="FH6 traction-control passthrough middleware")
    ap.add_argument("--mode", default="medium",
                    choices=["off", "low", "medium", "high", "custom"])
    ap.add_argument("--custom-target", type=float, default=None,
                    help="TC CUSTOM setpoint in percent (e.g. 92)")
    ap.add_argument("--abs-mode", default="medium",
                    choices=["off", "low", "medium", "high", "custom"])
    ap.add_argument("--abs-custom-target", type=float, default=None,
                    help="ABS CUSTOM setpoint in percent")
    ap.add_argument("--no-rumble", action="store_true",
                    help="don't forward the game's force feedback to the "
                         "physical DualSense")
    ap.add_argument("--chord-mod", default="BACK",
                    choices=[m for m in CHORD_MODS] + [m.lower() for m in CHORD_MODS],
                    help="held button that arms the d-pad mode chord")
    ap.add_argument("--port", type=int, default=7777, help="FH6 Data Out UDP port")
    ap.add_argument("--joy-id", type=int, default=None, help="winmm device id (default: first found)")
    ap.add_argument("--tick", type=float, default=0.003, help="loop sleep (s); ~250 Hz default")
    ap.add_argument("--duration", type=float, default=0.0, help="auto-stop after N s (0 = run forever)")
    ap.add_argument("--game-exe", default=GAME_EXE_DEFAULT,
                    help="game process name (F-key foreground gate + exit dialog clearing)")
    ap.add_argument("--long-only-front", action="store_true",
                    help="regulate only the fwd/back slip of the FRONT wheels")
    ap.add_argument("--long-only-rear", action="store_true",
                    help="regulate only the fwd/back slip of the REAR wheels (drift-friendly)")
    ap.add_argument("--no-dialog-clear", action="store_true",
                    help="don't send Enter to FH6 on exit to clear the controller-disconnected dialog")
    ap.add_argument("--map", action="store_true", help="interactive pad calibration -> tc_padmap.json")
    ap.add_argument("--selftest", action="store_true", help="offline unit tests")
    ap.add_argument("--leak-test", action="store_true", help="probe whether the physical pad leaks into the game")
    ap.add_argument("--rumble-test", action="store_true",
                    help="interactive DualSense rumble/LED diagnosis (no game needed)")
    args = ap.parse_args()
    if args.selftest:
        return selftest()
    if args.rumble_test:
        return rumble_test()
    if args.map:
        return run_map(args.joy_id if args.joy_id is not None else (find_pad(None) or 0))
    if args.leak_test:
        return leak_test(args)
    return run(args)


if __name__ == "__main__":
    raise SystemExit(main())
