#!/usr/bin/env python3
"""FH6 TC -- GUI front end for the traction-control + ABS middleware.

Runs traction_control.TCEngine (physical pad -> TC/ABS -> ViGEm pad -> FH6) on
a background thread, with two tabs:

  TRACTION -- live per-wheel FRICTION CIRCLES (dot = tire state in the game's
    normalized friction-circle units, ring = 100% = the grip limit, dashed
    ring/line = the active setpoint), mode selector (OFF / LOW 150% / MEDIUM
    100% / HIGH 85% / CUSTOM 30-250% with spinner), driven-wheel selector
    (AUTO/FWD/RWD/AWD), per-axle LONG-ONLY toggles, controller picker, HidHide
    panel, throttle input-vs-sent bars.
  ABS -- the same visualization and controls mirrored onto the BRAKE: watches
    ALL FOUR wheels for lockup (its long-only setpoint line draws BELOW center
    -- lockup is negative longitudinal slip), same modes + CUSTOM + per-axle
    long-only, brake input-vs-sent bars. ABS disengages below ~11 km/h so you
    can always stop.

Mode sync: GUI buttons, keyboard (TC F5-F8, ABS F1-F4, foreground-gated), and
the controller chord (hold BACK: dpad UP/DOWN = TC, RIGHT/LEFT = ABS) all stay
in sync both ways.

Run:
    python .\\tc_gui.py
    ... .\\tc_gui.py --demo       # synthetic data, no game/pad/vgamepad needed
    ... .\\tc_gui.py --smoke      # headless build + a few frames, then exit

HidHide buttons shell out to HidHideCLI.exe (github.com/nefarius/HidHide).
Do NOT run at the same time as anything else bound to UDP 7777.
"""
from __future__ import annotations

import argparse
import base64
import ctypes
import json
import math
import os
import queue
import re
import subprocess
import sys
import threading
import time
from collections import deque
from ctypes import wintypes

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import traction_control as tcm

try:                                    # crisp text under display scaling
    ctypes.windll.shcore.SetProcessDpiAwareness(2)
except Exception:
    pass

import tkinter as tk
from tkinter import ttk

REPO = os.path.dirname(os.path.abspath(__file__))


def _tool_command(flag):
    """Argv for running a traction_control CLI tool in its own console.

    From source that is <python.exe> traction_control.py <flag>; under
    pythonw (the windowless launcher) the console interpreter is substituted,
    or the tool's prompts would go nowhere.

    Frozen there is no .py on disk and sys.executable is this GUI, which is
    built windowed and has no console to prompt in, so the installed build
    ships a separate console exe beside it for exactly this.
    """
    if tcm.is_frozen():
        tools = os.path.join(os.path.dirname(sys.executable), "fh6tc-tools.exe")
        if os.path.exists(tools):
            return [tools, flag]
        return [sys.executable, "--tool", flag]      # single-exe fallback
    pyexe = sys.executable
    if pyexe.lower().endswith("pythonw.exe"):
        alt = os.path.join(os.path.dirname(pyexe), "python.exe")
        if os.path.exists(alt):
            pyexe = alt
    return [pyexe, os.path.join(REPO, "traction_control.py"), flag]

WHEELS = ("FL", "FR", "RL", "RR")
DRIVEN = {0: ("FL", "FR"), 1: ("RL", "RR"), 2: WHEELS}          # FWD/RWD/AWD

# per-channel wiring: which snapshot keys, modes table, and bar layout
CHANNELS = {
    "tc": dict(tag="TC", k_mode="mode", k_target="target", k_scale="scale",
               k_in="user_thr", k_out="sent_thr", k_lf="long_front",
               k_lr="long_rear", k_intv="interventions", lockup=False,
               bars=(("INPUT", "k_in"), ("SENT", "k_out"), ("BRAKE", "brake"))),
    "abs": dict(tag="ABS", k_mode="abs_mode", k_target="abs_target",
                k_scale="abs_scale", k_in="brake", k_out="sent_brk",
                k_lf="abs_long_front", k_lr="abs_long_rear",
                k_intv="abs_interventions", lockup=True,
                bars=(("BRAKE", "k_in"), ("SENT", "k_out"), ("THR", "user_thr"))),
}

# one-click presets: both channels to CUSTOM at these setpoints + long-only
# flags (fully deterministic -- a preset always produces the same state)
PRESETS = {
    "OFFROAD": {"tc": dict(target=250, long_front=True, long_rear=True),
                "abs": dict(target=150, long_front=True, long_rear=True)},
    "TOUGE":   {"tc": dict(target=125, long_front=True, long_rear=True),
                "abs": dict(target=400, long_front=True, long_rear=True)},
    "CIRCUIT": {"tc": dict(target=105, long_front=True, long_rear=False),
                "abs": dict(target=400, long_front=False, long_rear=False)},
}

C = dict(bg="#0d1117", panel="#161b22", inset="#0a0e13", ring="#30363d",
         ring2="#1d232b", grid="#171d25", text="#c9d1d9", dim="#8b949e",
         faint="#4d5761", green="#3fb950", amber="#d29922", red="#f85149",
         blue="#58a6ff", target="#e3b341", white="#e6edf3")
F = ("Segoe UI", 10)
F_SMALL = ("Segoe UI", 9)
F_BOLD = ("Segoe UI Semibold", 10)
F_HEAD = ("Segoe UI Semibold", 15)
F_MONO = ("Consolas", 9)


def util_color(v: float) -> str:
    return C["green"] if v <= 0.85 else C["amber"] if v <= 1.05 else C["red"]


def _is_vigem_x360(path: str) -> bool:
    """True if a HidHide device-instance path is an X360 pad (VID 045E /
    PID 028E) -- what ViGEm's virtual output presents as. Tagged in the hide
    list so it is never hidden by mistake (hiding it would break the tool)."""
    return "VID_045E&PID_028E" in (path or "").upper()


SETTINGS_PATH = os.path.join(tcm.REC_DIR, "tc_settings.json")


def load_settings() -> dict:
    try:
        with open(SETTINGS_PATH) as f:
            return json.load(f)
    except Exception:
        return {}


def save_settings(d: dict) -> None:
    try:
        os.makedirs(tcm.REC_DIR, exist_ok=True)
        json.dump(d, open(SETTINGS_PATH, "w"), indent=2)
    except Exception:
        pass


def is_elevated() -> bool:
    try:
        return bool(ctypes.windll.shell32.IsUserAnAdmin())
    except Exception:
        return False


_NAME_PREFIX = ("Sony Interactive Entertainment ", "Sony ", "Microsoft Corp. ",
                "Microsoft ", "Nintendo Co., Ltd. ", "Nintendo ")


def _short_name(fn: str) -> str:
    """Trim the redundant manufacturer prefix so the name fits the list."""
    fn = (fn or "").strip()
    for p in _NAME_PREFIX:
        if fn.startswith(p):
            return fn[len(p):]
    return fn


# --- phantom virtual-pad cleanup -------------------------------------------
# Every engine run mints a fresh ViGEm X360 pad; Windows keeps a non-present
# device node for each after it disconnects (normal PnP behavior), so they
# pile up. This removes ONLY the disconnected ones with the ViGEm identity
# (VID 045E / PID 028E) -- never a present device, never the DualSense (054C).
_PHANTOM_FILTER = ("Get-PnpDevice | Where-Object { -not $_.Present -and "
                   "$_.InstanceId -like '*VID_045E&PID_028E*' }")


def running_pad_holders() -> list[str]:
    """Processes that may already hold an OPEN handle on the physical pad.
    HidHide only decides visibility when a process opens the device, so these
    keep reading it after we hide -- unless the device is cycled."""
    try:
        r = subprocess.run(
            ["powershell", "-NoProfile", "-Command",
             "Get-Process | Where-Object { $_.Name -match '^(steam|forzahorizon6)$' }"
             " | Select-Object -ExpandProperty Name -Unique"],
            capture_output=True, text=True, timeout=20,
                           creationflags=subprocess.CREATE_NO_WINDOW)
        return [x.strip() for x in (r.stdout or "").splitlines() if x.strip()]
    except Exception:
        return []


def restart_pad_device(tag: str) -> tuple[bool, str]:
    """Cycle the pad's USB node -- an unplug/replug with no cable. Every open
    handle is invalidated, so apps that grabbed the pad BEFORE it was hidden
    must re-open it, which the cloak then refuses. This is what removes the
    "close Steam and the game first" requirement."""
    script = ("$ids = Get-PnpDevice | Where-Object { $_.Present -and "
              "$_.Class -eq 'USB' -and "
              f"$_.InstanceId -like '*{tag}*' -and "
              "$_.InstanceId -notlike '*&MI_*' } | "
              "Select-Object -ExpandProperty InstanceId; "
              "if (-not $ids) { exit 2 }; "
              "foreach ($id in $ids) { pnputil /restart-device \"$id\" | Out-Null }")
    try:
        r = subprocess.run(["powershell", "-NoProfile", "-Command", script],
                           capture_output=True, text=True, timeout=90,
                           creationflags=subprocess.CREATE_NO_WINDOW)
        if r.returncode == 0:
            return True, "controller cycled"
        if r.returncode == 2:
            return False, "controller USB node not found"
    except Exception as e:
        return False, str(e)
    # not elevated -> retry through UAC
    b64 = base64.b64encode(script.encode("utf-16-le")).decode()
    try:
        cmd = ("Start-Process -FilePath 'powershell' -ArgumentList "
               f"'-NoProfile','-EncodedCommand','{b64}' -Verb RunAs -Wait "
               "-WindowStyle Hidden")
        r2 = subprocess.run(["powershell", "-NoProfile", "-Command", cmd],
                            capture_output=True, text=True, timeout=120,
                           creationflags=subprocess.CREATE_NO_WINDOW)
        return r2.returncode == 0, ("controller cycled" if r2.returncode == 0
                                    else "cycle failed (needs admin)")
    except Exception as e:
        return False, str(e)


def count_phantom_pads() -> int:
    try:
        r = subprocess.run(
            ["powershell", "-NoProfile", "-Command", f"@({_PHANTOM_FILTER}).Count"],
            capture_output=True, text=True, timeout=25,
                           creationflags=subprocess.CREATE_NO_WINDOW)
        return int((r.stdout or "").strip() or "0")
    except Exception:
        return -1


def remove_phantom_pads() -> tuple[bool, str]:
    """Elevated (UAC) removal of disconnected ViGEm virtual-pad phantoms."""
    before = count_phantom_pads()
    if before == 0:
        return True, "no phantom pads to remove"
    script = (f"$ids = {_PHANTOM_FILTER} | Select-Object -ExpandProperty "
              "InstanceId; foreach ($id in $ids) "
              "{ pnputil /remove-device \"$id\" 2>$null }")
    b64 = base64.b64encode(script.encode("utf-16-le")).decode()
    try:
        cmd = ("Start-Process -FilePath 'powershell' -ArgumentList "
               f"'-NoProfile','-EncodedCommand','{b64}' -Verb RunAs -Wait "
               "-WindowStyle Hidden")
        subprocess.run(["powershell", "-NoProfile", "-Command", cmd],
                       capture_output=True, text=True, timeout=120,
                           creationflags=subprocess.CREATE_NO_WINDOW)
    except Exception as e:
        return False, str(e)
    after = count_phantom_pads()
    if before >= 0 and after >= 0:
        return True, f"removed {max(0, before - after)} phantom pad(s)"
    return True, "cleanup attempted"


def ch_modes(ch: str) -> list:
    return tcm.MODES if ch == "tc" else tcm.ABS_MODES


# ---------------------------------------------------------------------------
# HidHide CLI wrapper (config changes need admin: try direct, then UAC retry)
# ---------------------------------------------------------------------------
class HidHide:
    PATHS = [r"C:\Program Files\Nefarius Software Solutions\HidHide\x64\HidHideCLI.exe",
             r"C:\Program Files\Nefarius Software Solutions\HidHide\HidHideCLI.exe"]

    def __init__(self):
        self.cli = next((p for p in self.PATHS if os.path.exists(p)), None)

    def available(self) -> bool:
        return self.cli is not None

    def _exec(self, args: list[str]) -> tuple[bool, str]:
        try:
            r = subprocess.run([self.cli] + args, capture_output=True,
                               text=True, timeout=15,
                               creationflags=subprocess.CREATE_NO_WINDOW)
            if r.returncode == 0:
                return True, r.stdout or ""       # stdout only: parsers eat this
            return False, (r.stdout or "") + (r.stderr or "")
        except Exception as e:
            return False, str(e)

    def _exec_elevated(self, args: list[str]) -> tuple[bool, str]:
        """UAC elevation. -Verb RunAs cannot capture output, so -PassThru
        propagates the CLI's exit code and callers must verify state after."""
        try:
            arglist = ",".join("'" + a.replace("'", "''") + "'" for a in args)
            cmd = (f"$p = Start-Process -FilePath '{self.cli}' -ArgumentList {arglist} "
                   f"-Verb RunAs -Wait -WindowStyle Hidden -PassThru; "
                   f"if ($null -eq $p -or $null -eq $p.ExitCode) {{ exit 1 }} "
                   f"else {{ exit $p.ExitCode }}")
            r = subprocess.run(["powershell", "-NoProfile", "-Command", cmd],
                               capture_output=True, text=True, timeout=90,
                           creationflags=subprocess.CREATE_NO_WINDOW)
            return r.returncode == 0, (r.stdout or "") + (r.stderr or "")
        except Exception as e:
            return False, str(e)

    def modify(self, args: list[str]) -> tuple[bool, str]:
        ok, out = self._exec(args)
        if ok:
            return True, out
        return self._exec_elevated(args)

    def gaming_devices(self) -> list[tuple[str, str, bool]]:
        ok, out = self._exec(["--dev-gaming"])
        devs: list[tuple[str, str, bool]] = []
        try:
            for entry in json.loads(out):
                fn = entry.get("friendlyName", "?")
                for d in entry.get("devices", []):
                    p = d.get("deviceInstancePath")
                    if p:
                        devs.append((fn, p, bool(d.get("present", True))))
        except Exception:
            for m in re.finditer(r'"deviceInstancePath"\s*:\s*"([^"]+)"', out or ""):
                devs.append(("device", m.group(1).replace("\\\\", "\\"), True))
        return devs

    def hidden_paths(self) -> set[str] | None:
        # --dev-list prints lines shaped like:  --dev-hide "HID\VID_...\..."
        # NOTE: this read needs admin -- returns None (unknown) when it can't
        # read, so the UI shows "?" instead of a false "shown"
        ok, out = self._exec(["--dev-list"])
        if not ok:
            return None
        paths = set(re.findall(r'--dev-hide\s+"([^"]+)"', out or ""))
        if not paths:                              # older CLI: bare paths
            for ln in (out or "").splitlines():
                ln = ln.strip().strip('"')
                if ln and not ln.startswith("--") and "\\" in ln:
                    paths.add(ln)
        return paths

    def app_list(self) -> str:
        ok, out = self._exec(["--app-list"])
        return out or ""

    def cloak_on(self) -> bool | None:
        ok, out = self._exec(["--cloak-state"])
        if not ok:
            return None
        return "--cloak-on" in out


# ---------------------------------------------------------------------------
# Demo engine: synthetic snapshots so the GUI runs with no game/pad/vgamepad
# ---------------------------------------------------------------------------
class DemoEngine:
    def __init__(self, mode="MEDIUM", abs_mode="MEDIUM"):
        self.modes = {"tc": mode.upper(), "abs": abs_mode.upper()}
        self.dt_override: int | None = None
        self.long = {"tc": {"front": False, "rear": False},
                     "abs": {"front": False, "rear": False}}
        self.finished = threading.Event()
        self.error = None
        self.t0 = time.perf_counter()
        self.recording = False
        self._rec_t0 = 0.0
        self.abs_learning = False

    def start(self):
        pass

    def start_recording(self):
        self.recording = True
        self._rec_t0 = time.perf_counter() - self.t0

    def stop_recording(self):
        self.recording = False

    def start_leak_test(self):
        pass

    def stop(self, join_s=0.0):
        self.finished.set()

    def set_mode_ch(self, ch, name):
        self.modes[ch] = name.upper()

    def set_mode(self, name):
        self.set_mode_ch("tc", name)

    def set_drivetrain(self, val):
        self.dt_override = val

    def set_long_only_ch(self, ch, axle, on):
        self.long[ch][axle] = bool(on)

    def set_long_only(self, axle, on):
        self.set_long_only_ch("tc", axle, on)

    def long_only(self, ch, axle):
        return self.long[ch][axle]

    def set_custom_target_ch(self, ch, pct):
        tcm.apply_custom_target(pct / 100.0, ch_modes(ch))

    def set_custom_target(self, pct):
        self.set_custom_target_ch("tc", pct)

    def set_pad(self, joy_id):
        pass

    def set_chord_mod(self, name):
        pass

    def set_abs_learning(self, on):
        if on and self.modes["abs"] == "OFF":
            return
        self.abs_learning = bool(on)

    def snapshot(self) -> dict:
        t = time.perf_counter() - self.t0
        phase = math.sin(0.22 * t)
        braking = phase < -0.15
        eff = self.dt_override if self.dt_override is not None else 2
        wheels = {}
        combs_tc, combs_abs = [], []
        sign = -1.0 if braking else 1.0
        for i, w in enumerate(WHEELS):
            ph = i * 1.7
            comb = max(0.0, 0.62 + 0.62 * math.sin(0.8 * t + ph))
            if 8 < (t % 12) < 9.5:
                comb += 1.2 if (braking or w in DRIVEN[eff]) else 0.2
            th = 0.9 * t + ph
            wheels[w] = (sign * comb * abs(math.sin(th)), comb * math.cos(th), comb)
            if w in DRIVEN[eff]:
                combs_tc.append(comb)
            combs_abs.append(comb)
        m_tc = ch_modes("tc")[tcm.MODE_IDX[self.modes["tc"]]]
        m_ab = ch_modes("abs")[tcm.MODE_IDX[self.modes["abs"]]]
        # demo learning: ceiling breathes around the mode target, corner
        # derate follows the synthetic "cornering" phase of the animation
        lc = m_ab.target * (1.0 + 0.25 * math.sin(0.15 * t)) \
            if self.abs_learning else m_ab.target
        l_tgt = lc * (0.55 if (self.abs_learning and not braking
                               and abs(phase) > 0.6) else 1.0)
        s_tc = max(combs_tc) if combs_tc else 0.0
        s_ab = max(combs_abs)
        user = max(0.0, phase) * 0.9 + 0.1
        brk = max(0.0, -phase)
        sc = 1.0 if (self.modes["tc"] == "OFF" or braking) else \
            max(m_tc.floor, min(1.0, 1.0 - 0.5 * max(0.0, s_tc - m_tc.target)))
        asc = 1.0 if (self.modes["abs"] == "OFF" or not braking) else \
            max(m_ab.floor, min(1.0, 1.0 - 0.5 * max(0.0, s_ab - m_ab.target)))
        return {"t": t, "mode": self.modes["tc"], "target": m_tc.target,
                "user_thr": user if not braking else 0.0,
                "sent_thr": (user if not braking else 0.0) * sc, "scale": sc,
                "brake": brk, "sent_brk": brk * asc, "abs_scale": asc,
                "abs_mode": self.modes["abs"],
                "abs_target": l_tgt if self.abs_learning else m_ab.target,
                "abs_learn": self.abs_learning, "learn_ceiling": lc,
                "learn_alat_g": 3.6,
                "abs_s_reg": s_ab if braking else 0.0, "abs_s_long": 0.0,
                "abs_long_front": self.long["abs"]["front"],
                "abs_long_rear": self.long["abs"]["rear"],
                "abs_interventions": int(t / 11),
                "s_long": s_tc / 1.14, "s_comb": s_tc, "s_reg": s_tc,
                "wheels": wheels, "telem_age": 0.012, "race": True,
                "speed_kmh": 150 + 60 * math.sin(0.2 * t), "gear": 4,
                "drivetrain": 2, "drivetrain_eff": eff,
                "long_front": self.long["tc"]["front"],
                "long_rear": self.long["tc"]["rear"],
                "leak": False, "ever_leaked": False,
                "ffb_events": int(t * 3), "ffb_writes": int(t * 3),
                "recording": self.recording, "rec_name": "demo.csv",
                "rec_secs": (t - self._rec_t0) if self.recording else 0.0,
                "rec_rows": int((t - self._rec_t0) * 62.5) if self.recording else 0,
                "interventions": int(t / 7), "pad_ok": True, "pad_id": 0,
                "pad_name": "DEMO (synthetic data)", "hz": 250.0}


# ---------------------------------------------------------------------------
# The app
# ---------------------------------------------------------------------------
class App:
    CELL = 250          # friction-circle cell size (px)
    R = 68              # radius of the 100% ring (px)

    def __init__(self, root: tk.Tk, engine, evq: queue.Queue, demo: bool,
                 make_engine=None, cli_args: list[str] | None = None):
        self.root = root
        self.engine = engine
        self.evq = evq
        self.demo = demo
        self.make_engine = make_engine
        self.cli_args = cli_args or []
        self.tool_busy = False
        self.hh = HidHide()
        self.hh_devices: list[tuple[str, str, bool]] = []
        self.hh_busy = False
        self.settings = load_settings()
        # hiding is tied to this app's lifetime: cloak ON at start, OFF at
        # exit, so the controller works normally everywhere else
        self.unhide_var = tk.BooleanVar(
            value=bool(self.settings.get("unhide_on_exit", True)))
        self.trails = {ch: {w: deque(maxlen=14) for w in WHEELS}
                       for ch in CHANNELS}
        self.ui = {ch: {"last_mode": None, "last_long": None,
                        "last_custom": -1.0, "last_learn": None,
                        "last_learn_txt": None} for ch in CHANNELS}
        self._last_pad_seen = "unset"
        self._quitting = False
        self._cloak_fix_t = 0.0
        self._build()
        self.engine.set_chord_mod(self.settings.get("chord_mod", "BACK"))
        if self.hh.available():
            if not demo and not is_elevated():
                self._log_line("[hidhide] not running as admin -- hide/unhide "
                               "will pop UAC prompts; use Launch FH6 TC.bat "
                               "to avoid them", "warn")
            if self.unhide_var.get() and not demo:
                # hiding is scoped to this app running: turn it on now, and
                # _on_close turns it back off. Serialized (write, THEN read) so
                # the panel can't render a stale "cloak OFF" from a refresh
                # that raced the cloak-on.
                def _startup():
                    self._ensure_pad_hidden()   # device into the list FIRST,
                    self._set_cloak(True)       # then the master switch
                    # anything already running grabbed the pad before we hid
                    # it and keeps its handle -- cycle the device so they lose
                    # it (this is what removes the "close Steam/game first"
                    # requirement). Skipped when nothing else holds the pad.
                    holders = running_pad_holders()
                    if holders:
                        tag = self._pad_tag()
                        self.evq.put(("log",
                                      f"[pad] {', '.join(holders)} already had your "
                                      f"controller -- cycling it so they let go..."))
                        if tag:
                            ok, msg = restart_pad_device(tag)
                            self.evq.put(("log", (
                                f"[pad] {msg}",
                                None if ok else "err")))
                            if ok:
                                self._after_cycle()
                                return          # _after_cycle refreshes
                    self.evq.put(("hh_refresh", None))
                threading.Thread(target=_startup, daemon=True).start()
            else:
                self._hh_refresh_async()
            if not demo:
                # cloak watchdog: catches "device listed but master switch
                # off" (a real leak seen on-rig) for the whole app lifetime
                self.root.after(3000, self._cloak_watch)

    # ---------------- UI construction ----------------
    def _build(self):
        r = self.root
        r.title("FH6 TC: friction-circle traction control + ABS")
        r.configure(bg=C["bg"])
        r.resizable(False, False)

        style = ttk.Style(r)
        try:
            style.theme_use("clam")
        except Exception:
            pass
        style.configure("TNotebook", background=C["bg"], borderwidth=0)
        style.configure("TNotebook.Tab", background=C["panel"],
                        foreground=C["dim"], padding=(22, 7), font=F_BOLD,
                        borderwidth=0)
        style.map("TNotebook.Tab",
                  background=[("selected", C["inset"])],
                  foreground=[("selected", C["white"])])
        style.configure("TCombobox", fieldbackground="#ffffff")

        # presets strip: visible from both tabs
        pf = tk.Frame(r, bg=C["bg"])
        pf.pack(fill="x", padx=12, pady=(8, 4))
        tk.Label(pf, text="PRESETS", font=("Segoe UI Semibold", 8),
                 fg=C["faint"], bg=C["bg"]).pack(side="left", padx=(0, 10))
        self.preset_btns = {}
        self._last_preset_shown = "unset"
        for name in PRESETS:
            b = tk.Button(pf, text=name.title(), font=F_SMALL, bd=0,
                          relief="flat", bg=C["inset"], fg=C["text"],
                          padx=16, pady=4, activebackground=C["ring"],
                          activeforeground=C["white"],
                          command=lambda n=name: self._apply_preset(n))
            b.pack(side="left", padx=(0, 6))
            self.preset_btns[name] = b

        # record telemetry (right-aligned): reachable from both tabs
        self.rec_status = tk.Label(pf, text="", font=F_MONO, fg=C["dim"],
                                   bg=C["bg"])
        self.rec_status.pack(side="right", padx=(8, 0))
        self.rec_btn = tk.Button(pf, text="● Record", font=F_BOLD, bd=0,
                                 relief="flat", bg=C["inset"], fg=C["red"],
                                 padx=16, pady=4, activebackground=C["ring"],
                                 activeforeground=C["white"],
                                 command=self._toggle_record)
        self.rec_btn.pack(side="right")
        self._last_rec_shown = None

        self.nb = ttk.Notebook(r)
        self.nb.pack(fill="both", expand=True)
        tc_page = tk.Frame(self.nb, bg=C["bg"])
        abs_page = tk.Frame(self.nb, bg=C["bg"])
        self.nb.add(tc_page, text="TRACTION")
        self.nb.add(abs_page, text="ABS")

        # ---- TRACTION page
        left = tk.Frame(tc_page, bg=C["panel"], padx=14, pady=12)
        left.grid(row=0, column=0, sticky="ns")
        right = tk.Frame(tc_page, bg=C["bg"], padx=10, pady=10)
        right.grid(row=0, column=1, sticky="nsew")
        tk.Label(left, text="FH6 TC", font=F_HEAD, fg=C["white"],
                 bg=C["panel"]).pack(anchor="w")
        tk.Label(left, text="friction-circle traction control", font=F_SMALL,
                 fg=C["dim"], bg=C["panel"]).pack(anchor="w", pady=(0, 10))
        self._build_mode_section(left, "tc")

        self._section(left, "DRIVEN WHEELS  (TC watches these)")
        dtf = tk.Frame(left, bg=C["panel"])
        dtf.pack(fill="x", pady=(2, 10))
        self.dt_var = tk.StringVar(value="AUTO")
        for i, (label, val) in enumerate((("AUTO", None), ("FWD", 0),
                                          ("RWD", 1), ("AWD", 2))):
            tk.Radiobutton(dtf, text=label, value=label, variable=self.dt_var,
                           indicatoron=0, width=6, font=F_SMALL, bd=0,
                           bg=C["inset"], fg=C["dim"], selectcolor=C["ring"],
                           activebackground=C["ring"],
                           command=lambda v=val: self.engine.set_drivetrain(v)
                           ).grid(row=0, column=i, padx=2, sticky="ew")
            dtf.columnconfigure(i, weight=1)

        self._build_long_section(left, "tc")

        self._section(left, "MODE CHORD  (hold this + d-pad)")
        cf = tk.Frame(left, bg=C["panel"])
        cf.pack(fill="x", pady=(2, 10))
        self.chord_var = tk.StringVar(
            value=str(self.settings.get("chord_mod", "BACK")))
        chord_combo = ttk.Combobox(cf, textvariable=self.chord_var,
                                   state="readonly", font=F_SMALL, width=7,
                                   values=list(tcm.CHORD_MODS))
        chord_combo.pack(side="left")
        chord_combo.bind("<<ComboboxSelected>>",
                         lambda e: self._chord_changed())
        tk.Label(cf, text="▲▼ TC · ◄► ABS   (tap = normal button)",
                 font=F_SMALL, fg=C["dim"], bg=C["panel"]).pack(
                 side="left", padx=(6, 0))

        self._section(left, "CONTROLLER")
        self.pad_var = tk.StringVar()
        self.pad_combo = ttk.Combobox(left, textvariable=self.pad_var,
                                      state="readonly", font=F_SMALL, width=30)
        self.pad_combo.pack(fill="x", pady=(2, 3))
        pf = tk.Frame(left, bg=C["panel"])
        pf.pack(fill="x", pady=(0, 10))
        self._btn(pf, "Rescan", self._rescan_pads).pack(side="left", padx=(0, 4))
        self._btn(pf, "Use selected", self._use_pad).pack(side="left", padx=(0, 4))
        self._btn(pf, "Restart engine", self._restart_engine).pack(side="left")
        self._rescan_pads()

        self._section(left, "TOOLS")
        tf = tk.Frame(left, bg=C["panel"])
        tf.pack(fill="x", pady=(2, 10))
        self._btn(tf, "Calibrate pad (--map)",
                  lambda: self._spawn_console("--map")).pack(side="left", padx=(0, 4))
        self._btn(tf, "Leak test", self._leak_test).pack(side="left")
        self._btn(left, "Re-cycle controller (fixes leaks, no restarts)",
                  self._recycle_pad).pack(fill="x", pady=(4, 2))
        self._btn(left, "Clean phantom virtual pads (admin)",
                  self._clean_phantoms).pack(fill="x", pady=(0, 6))

        self._section(left, "HIDE FROM GAME  (HidHide)")
        if self.hh.available():
            self.hh_list = tk.Listbox(left, height=4, font=F_SMALL, bd=0,
                                      bg=C["inset"], fg=C["text"],
                                      selectbackground=C["ring"],
                                      highlightthickness=0, exportselection=False)
            self.hh_list.pack(fill="x", pady=(2, 1))
            tk.Label(left, text="Hide your controller; leave the virtual pad shown.",
                     font=F_SMALL, fg=C["faint"], bg=C["panel"], wraplength=250,
                     justify="left", anchor="w").pack(fill="x", pady=(0, 3))
            hf = tk.Frame(left, bg=C["panel"])
            hf.pack(fill="x", pady=(0, 2))
            self._btn(hf, "Hide", lambda: self._hh_toggle(True)).pack(side="left", padx=(0, 4))
            self._btn(hf, "Unhide", lambda: self._hh_toggle(False)).pack(side="left", padx=(0, 4))
            self._btn(hf, "Refresh", self._hh_refresh_async).pack(side="left")
            self._btn(left, "Whitelist python (so TC keeps reading the pad)",
                      self._hh_whitelist).pack(fill="x", pady=(3, 2))
            tk.Checkbutton(left, text="Hide my controller only while this app runs",
                           variable=self.unhide_var, command=self._save_prefs,
                           font=F_SMALL, fg=C["text"], bg=C["panel"],
                           selectcolor=C["inset"], activebackground=C["panel"],
                           activeforeground=C["white"], anchor="w",
                           wraplength=250, justify="left",
                           highlightthickness=0).pack(fill="x", pady=(2, 0))
            self.hh_status = tk.Label(left, text="", font=F_SMALL, fg=C["dim"],
                                      bg=C["panel"], anchor="w")
            self.hh_status.pack(fill="x", pady=(0, 8))
        else:
            tk.Label(left, text="HidHide not installed.\nWithout it the game sees "
                     "BOTH pads and TC/ABS cuts leak.\nInstall from "
                     "github.com/nefarius/HidHide,\nreboot, then reopen this panel.",
                     font=F_SMALL, fg=C["amber"], bg=C["panel"], justify="left",
                     anchor="w").pack(fill="x", pady=(2, 8))

        self._section(left, "EVENTS")
        self.log = tk.Text(left, height=9, width=36, font=F_MONO, bd=0,
                           bg=C["inset"], fg=C["dim"], state="disabled",
                           highlightthickness=0, wrap="word")
        self.log.pack(fill="both", expand=True, pady=(2, 0))
        self.log.tag_configure("warn", foreground=C["amber"])
        self.log.tag_configure("err", foreground=C["red"])

        self._build_canvas(right, "tc")
        self._build_bars(right, "tc")

        # ---- ABS page
        aleft = tk.Frame(abs_page, bg=C["panel"], padx=14, pady=12)
        aleft.grid(row=0, column=0, sticky="ns")
        aright = tk.Frame(abs_page, bg=C["bg"], padx=10, pady=10)
        aright.grid(row=0, column=1, sticky="nsew")
        tk.Label(aleft, text="ABS", font=F_HEAD, fg=C["white"],
                 bg=C["panel"]).pack(anchor="w")
        tk.Label(aleft, text="anti-lock braking: same law, brake pedal",
                 font=F_SMALL, fg=C["dim"], bg=C["panel"]).pack(anchor="w",
                                                                pady=(0, 10))
        self._build_mode_section(aleft, "abs")
        self._build_learn_section(aleft)
        self._build_long_section(aleft, "abs")
        self._section(aleft, "HOW IT WORKS")
        tk.Label(aleft, text=(
            "Watches ALL FOUR wheels for lockup\n"
            "(negative fwd slip / the same friction\n"
            "circle) and modulates the brake with\n"
            "the same PID + gain scheduling.\n\n"
            "• disengages below ~11 km/h so you\n"
            "  can always brake to a full stop\n"
            "• a cut always leaves brake pressure\n"
            "  (10-30% floor by mode)\n"
            "• long-only line draws BELOW center:\n"
            "  lockup is negative fwd slip\n"
            "• LEARN: starts from YOUR setpoint,\n"
            "  learns from FULL-trigger stops only\n"
            "  (partial braking teaches nothing;\n"
            "  minor steering corrections count)\n"
            "  and derates it live in corners so\n"
            "  trail-braking can't lock or spin.\n"
            "  A firmer bite every ~2 s in long\n"
            "  hard stops is the learner probing\n"
            "  past the cap -- normal, straight-\n"
            "  line only\n\n"
            "Keys: F1 off / F2 low / F3 med /\n"
            "F4 high · chord: modifier + dpad ◄ ►"),
            font=F_SMALL, fg=C["dim"], bg=C["panel"], justify="left",
            anchor="w").pack(fill="x", pady=(2, 8))

        self._build_canvas(aright, "abs")
        self._build_bars(aright, "abs")

        # ---- shared status strip below the notebook
        sf = tk.Frame(r, bg=C["bg"])
        sf.pack(fill="x", padx=12, pady=(0, 8))
        self.status_vars = {}
        for key, w in (("speed", 11), ("gear", 6), ("telem", 13), ("pad", 24),
                       ("leak", 9), ("cuts", 8), ("abs_cuts", 9), ("ffb", 12)):
            v = tk.StringVar(value="--")
            self.status_vars[key] = v
            tk.Label(sf, textvariable=v, font=F_MONO, fg=C["dim"], bg=C["bg"],
                     width=w, anchor="w").pack(side="left", padx=(0, 6))

        r.protocol("WM_DELETE_WINDOW", self._on_close)

    def _section(self, parent, text):
        tk.Label(parent, text=text, font=("Segoe UI Semibold", 8), fg=C["faint"],
                 bg=parent["bg"], anchor="w").pack(fill="x", pady=(4, 0))

    def _btn(self, parent, text, cmd):
        return tk.Button(parent, text=text, font=F_SMALL, bd=0, relief="flat",
                         bg=C["inset"], fg=C["text"], padx=8, pady=3,
                         activebackground=C["ring"], activeforeground=C["white"],
                         command=cmd)

    def _build_mode_section(self, parent, ch):
        tag = CHANNELS[ch]["tag"]
        self._section(parent, f"{tag} MODE")
        grid = tk.Frame(parent, bg=C["panel"])
        grid.pack(fill="x", pady=(2, 10))
        descr = {"OFF": "passthrough", "LOW": "150% · sport",
                 "MEDIUM": "100% · perfect", "HIGH": "85% · always grip"}
        if ch == "abs":
            descr["LOW"] = "150% · late lock"
            descr["HIGH"] = "85% · never lock"
        btns = {}
        for i, name in enumerate(("OFF", "LOW", "MEDIUM", "HIGH")):
            b = tk.Button(grid, text=f"{name}\n{descr[name]}", font=F_SMALL,
                          width=13, height=2, bd=0, relief="flat",
                          bg=C["inset"], fg=C["dim"],
                          activebackground=C["ring"], activeforeground=C["white"],
                          command=lambda n=name, c=ch: self.engine.set_mode_ch(c, n))
            b.grid(row=i // 2, column=i % 2, padx=3, pady=3, sticky="ew")
            btns[name] = b
        b = tk.Button(grid, text="CUSTOM\n100% setpoint", font=F_SMALL,
                      width=13, height=2, bd=0, relief="flat",
                      bg=C["inset"], fg=C["dim"],
                      activebackground=C["ring"], activeforeground=C["white"],
                      command=lambda c=ch: self.engine.set_mode_ch(c, "CUSTOM"))
        b.grid(row=2, column=0, padx=3, pady=3, sticky="ew")
        btns["CUSTOM"] = b
        spinf = tk.Frame(grid, bg=C["panel"])
        spinf.grid(row=2, column=1, padx=3, pady=3, sticky="nsew")
        var = tk.StringVar(value="100")
        # ABS's range extends far higher: true lock in this game reads
        # ~800-1000%, so 400-600% setpoints allow visible lock-ups
        to_max = 600 if ch == "abs" else 250
        spin = tk.Spinbox(spinf, from_=30, to=to_max, increment=5, width=5, font=F,
                          textvariable=var, bd=0, justify="right",
                          bg=C["inset"], fg=C["white"],
                          buttonbackground=C["panel"],
                          insertbackground=C["white"],
                          command=lambda c=ch: self._custom_changed(c))
        spin.pack(side="left", expand=True, pady=8)
        spin.bind("<Return>", lambda e, c=ch: self._custom_changed(c))
        spin.bind("<FocusOut>", lambda e, c=ch: self._custom_changed(c))
        tk.Label(spinf, text="% of grip", font=F_SMALL, fg=C["dim"],
                 bg=C["panel"]).pack(side="left", padx=(4, 0))
        grid.columnconfigure(0, weight=1)
        grid.columnconfigure(1, weight=1)
        self.ui[ch]["mode_btns"] = btns
        self.ui[ch]["custom_var"] = var
        self.ui[ch]["custom_spin"] = spin

    def _build_learn_section(self, parent):
        """ABS-only LEARN toggle: the PID keeps tracking a % setpoint, but the
        setpoint itself is learned from braking evidence (starting from the
        user's active mode/custom value) and derated live in corners."""
        self._section(parent, "LEARNING  (adaptive setpoint)")
        lf = tk.Frame(parent, bg=C["panel"])
        lf.pack(fill="x", pady=(2, 10))
        b = tk.Button(lf, text="LEARN\nadapt from braking", font=F_SMALL,
                      height=2, bd=0, relief="flat", bg=C["inset"], fg=C["dim"],
                      activebackground=C["ring"], activeforeground=C["white"],
                      command=self._toggle_learn)
        b.grid(row=0, column=0, padx=3, sticky="ew")
        lbl = tk.Label(lf, text="off, setpoint is fixed", font=F_MONO,
                       fg=C["dim"], bg=C["panel"], justify="left", anchor="w")
        lbl.grid(row=0, column=1, padx=(6, 3), sticky="w")
        lf.columnconfigure(0, weight=1)
        lf.columnconfigure(1, weight=2)
        self.ui["abs"]["learn_btn"] = b
        self.ui["abs"]["learn_lbl"] = lbl

    def _toggle_learn(self):
        self.engine.set_abs_learning(not bool(self.ui["abs"]["last_learn"]))

    def _build_long_section(self, parent, ch):
        what = "sideways slip" if ch == "tc" else "cornering load"
        self._section(parent, f"LONG-ONLY  (ignore {what})")
        lof = tk.Frame(parent, bg=C["panel"])
        lof.pack(fill="x", pady=(2, 10))
        sub = "fwd/back slip only" if ch == "tc" else "lockup only"
        btns = {}
        for i, axle in enumerate(("front", "rear")):
            b = tk.Button(lof, text=f"{axle.upper()}\n{sub}",
                          font=F_SMALL, height=2, bd=0, relief="flat",
                          bg=C["inset"], fg=C["dim"],
                          activebackground=C["ring"], activeforeground=C["white"],
                          command=lambda a=axle, c=ch: self._toggle_long(c, a))
            b.grid(row=0, column=i, padx=3, sticky="ew")
            btns[axle] = b
            lof.columnconfigure(i, weight=1)
        self.ui[ch]["long_btns"] = btns

    def _build_canvas(self, parent, ch):
        size = self.CELL * 2
        cv = tk.Canvas(parent, width=size, height=size, bg=C["bg"], bd=0,
                       highlightthickness=0)
        cv.pack()
        half = self.CELL // 2
        for i, w in enumerate(WHEELS):
            cx = half + (i % 2) * self.CELL
            cy = half + (i // 2) * self.CELL
            # (the 50%/100% rings are drawn per-frame: the view auto-zooms
            # when the active setpoint exceeds the 150% display range)
            cv.create_line(cx - self.R * 1.55, cy, cx + self.R * 1.55, cy,
                           fill=C["grid"])
            cv.create_line(cx, cy - self.R * 1.55, cx, cy + self.R * 1.55,
                           fill=C["grid"])
            cv.create_text(cx - half + 16, cy - half + 14, text=w,
                           font=F_BOLD, fill=C["dim"], tags=f"label_{w}")
        cv.create_text(half + self.R * 1.18, half + 10, text="lat",
                       font=("Segoe UI", 8), fill=C["faint"])
        cv.create_text(half + 12, half - self.R * 1.28, text="long",
                       font=("Segoe UI", 8), fill=C["faint"])
        self.ui[ch]["canvas"] = cv

    def _build_bars(self, parent, ch):
        b = tk.Canvas(parent, width=self.CELL * 2, height=76, bg=C["bg"], bd=0,
                      highlightthickness=0)
        b.pack(pady=(6, 2))
        self.ui[ch]["bars"] = b

    # ---------------- controls plumbing ----------------
    def _toggle_record(self):
        if getattr(self.engine, "recording", False):
            self.engine.stop_recording()
        else:
            self.engine.start_recording()

    def _apply_preset(self, name):
        p = PRESETS[name]
        # a preset is an explicit user action: it overrides any in-progress
        # spinner edit. Drop spinner focus and sync the displayed values --
        # otherwise the spinbox keeps showing (and, on its next FocusOut,
        # RE-APPLIES) the old number over the preset's target
        self.root.focus_set()
        for ch in ("tc", "abs"):
            cfg = p[ch]
            self.engine.set_custom_target_ch(ch, cfg["target"])
            self.engine.set_mode_ch(ch, "CUSTOM")
            self.engine.set_long_only_ch(ch, "front", cfg["long_front"])
            self.engine.set_long_only_ch(ch, "rear", cfg["long_rear"])
            self.ui[ch]["custom_var"].set(f"{cfg['target']:.0f}")
        self._log_line(f"[preset] {name} applied")

    def _match_preset(self, s):
        """The preset the CURRENT state exactly equals, else None."""
        for name, p in PRESETS.items():
            ok = True
            for ch in ("tc", "abs"):
                cfg = CHANNELS[ch]
                pc = p[ch]
                if (s.get(cfg["k_mode"]) != "CUSTOM"
                        or abs(ch_modes(ch)[tcm.MODE_IDX["CUSTOM"]].target
                               - pc["target"] / 100.0) > 1e-6
                        or bool(s.get(cfg["k_lf"])) != pc["long_front"]
                        or bool(s.get(cfg["k_lr"])) != pc["long_rear"]):
                    ok = False
                    break
            if ok:
                return name
        return None

    def _toggle_long(self, ch, axle):
        self.engine.set_long_only_ch(ch, axle, not self.engine.long_only(ch, axle))

    def _custom_changed(self, ch, *_):
        try:
            pct = float(self.ui[ch]["custom_var"].get())
        except (ValueError, tk.TclError):
            return
        hi = 600.0 if ch == "abs" else 250.0
        self.engine.set_custom_target_ch(ch, max(30.0, min(hi, pct)))

    def _rescan_pads(self):
        if self.demo:
            self.pad_combo["values"] = ["demo pad"]
            self.pad_combo.current(0)
            return
        pads = tcm.list_pads()
        vals = [f"id {i}: {n}" for i, n in pads] or ["(no pads found)"]
        prev = self.pad_var.get()
        self.pad_combo["values"] = vals
        # keep the user's pick if that pad is still there, else show the first
        self.pad_combo.current(vals.index(prev) if prev in vals else 0)

    def _use_pad(self):
        m = re.match(r"id (\d+)", self.pad_var.get() or "")
        if m:
            self.engine.set_pad(int(m.group(1)))

    def _save_prefs(self):
        self.settings["unhide_on_exit"] = bool(self.unhide_var.get())
        save_settings(self.settings)

    def _chord_changed(self):
        name = self.chord_var.get()
        self.engine.set_chord_mod(name)
        self.settings["chord_mod"] = name
        save_settings(self.settings)
        self.root.focus_set()

    def _restart_engine(self):
        """Bring a stopped engine back without restarting the app."""
        if self.demo or self.make_engine is None:
            self._log_line("[gui] engine restart unavailable here", "warn")
            return
        if not self.engine.finished.is_set():
            self._log_line("[gui] engine is already running", "warn")
            return
        old = self.engine
        eng = self.make_engine()
        self.engine = eng
        eng.start()
        eng.set_mode_ch("tc", old.tc.mode.name)
        eng.set_mode_ch("abs", old.abs_ctl.mode.name)
        eng.set_drivetrain(old.tc.drivetrain_override)
        for chn in ("tc", "abs"):
            for ax in ("front", "rear"):
                eng.set_long_only_ch(chn, ax, old.long_only(chn, ax))
        eng.set_chord_mod(self.settings.get("chord_mod", "BACK"))
        self._log_line("[gui] engine restarted")

    def _pad_tag(self) -> str | None:
        """VID_xxxx&PID_xxxx of the pad the engine is reading."""
        try:
            jid = tcm.find_pad(None)
            ident = tcm.joy_identity(jid) if jid is not None else None
            return f"VID_{ident[0]:04X}&PID_{ident[1]:04X}" if ident else None
        except Exception:
            return None

    def _after_cycle(self):
        """Wait for the pad to finish re-enumerating, then re-hide the CURRENT
        device instance and refresh. A cycle can hand the pad a new device
        path, which would leave the hide entry pointing at the old one (pad
        listed as 'disconnected', and potentially no longer hidden)."""
        for _ in range(24):                     # up to ~12 s
            time.sleep(0.5)
            if tcm.find_pad(None) is not None:
                break
        if self.unhide_var.get():
            self._ensure_pad_hidden()           # re-hide the new instance
        self.evq.put(("hh_refresh", None))

    def _recycle_pad(self, reason: str = ""):
        """Cycle the controller so apps that opened it before it was hidden
        (Steam, the game) drop their handle -- no restarts needed."""
        if self.demo:
            self._log_line("[pad] cycle unavailable in --demo", "warn")
            return
        if self.hh_busy:
            return
        self.hh_busy = True

        def work():
            tag = self._pad_tag()
            if not tag:
                self.evq.put(("log", ("[pad] no controller to cycle", "warn")))
            else:
                if reason:
                    self.evq.put(("log", reason))
                ok, msg = restart_pad_device(tag)
                self.evq.put(("log", (
                    f"[pad] {msg} -- Steam/the game must now re-open it, and "
                    f"the cloak refuses them" if ok else f"[pad] {msg}",
                    None if ok else "err")))
                if ok:
                    self._after_cycle()
            self.hh_busy = False
        threading.Thread(target=work, daemon=True).start()

    def _on_pad_appeared(self):
        """Hide a controller that turned up after startup, then refresh."""
        if self.demo or not self.hh.available() or self.hh_busy:
            return
        if not self.unhide_var.get():
            self._hh_refresh_async()
            return
        self.hh_busy = True

        def work():
            self._ensure_pad_hidden()
            self.hh_busy = False
            self.evq.put(("hh_refresh", None))
        threading.Thread(target=work, daemon=True).start()

    def _ensure_pad_hidden(self):
        """Put the pad we're actually reading into HidHide's hide list.

        The cloak is only a master switch -- turning it on does NOTHING if the
        device was never added, which reads as a reassuring "hiding ON" while
        the pad still reaches the game. So hide the device explicitly, matched
        to the controller this engine is using by USB VID/PID.
        """
        try:
            jid = tcm.find_pad(None)
            ident = tcm.joy_identity(jid) if jid is not None else None
            if not ident:
                return
            tag = f"VID_{ident[0]:04X}&PID_{ident[1]:04X}"
            hidden = self.hh.hidden_paths()          # None = unreadable
            for fn, path, present in self.hh.gaming_devices():
                if (not present or tag not in path.upper()
                        or _is_vigem_x360(path)):
                    continue
                if hidden is not None and path in hidden:
                    continue                          # already hidden
                ok, out = self.hh.modify(["--dev-hide", path])
                self.evq.put(("log", (
                    f"[hidhide] auto-hid {_short_name(fn)}" if ok else
                    f"[hidhide] could not hide {_short_name(fn)}: "
                    f"{out.strip()[:80]}", None if ok else "err")))
        except Exception as e:
            self.evq.put(("log", (f"[hidhide] auto-hide skipped: {e}", "warn")))

    def _set_cloak(self, on: bool):
        """Flip HidHide's master cloak (the device stays configured either
        way). Runs off the UI thread; elevates only if we aren't already."""
        if self.demo or not self.hh.available():
            return
        ok, out = self.hh.modify(["--cloak-on" if on else "--cloak-off"])
        self.evq.put(("log", (
            f"[hidhide] hiding {'ON' if on else 'OFF -- controller back to normal'}"
            if ok else
            f"[hidhide] could not turn hiding {'on' if on else 'off'}: "
            f"{out.strip()[:90]}", None if ok else "err")))

    def _leak_test(self):
        """Run the leak test inside the live engine (no pad teardown, so the
        game's controller never disconnects)."""
        if self.demo:
            self._log_line("[leak-test] unavailable in --demo", "warn")
            return
        if getattr(self.engine, "finished", None) and self.engine.finished.is_set():
            self._log_line("[leak-test] engine not running", "warn")
            return
        self.engine.start_leak_test()

    def _clean_phantoms(self):
        """Remove disconnected ViGEm virtual-pad phantoms (UAC prompt)."""
        if self.hh_busy or self.demo:
            if self.demo:
                self._log_line("[cleanup] unavailable in --demo", "warn")
            return
        self.hh_busy = True

        def work():
            ok, msg = remove_phantom_pads()
            self.evq.put(("log", (f"[cleanup] {msg}", None if ok else "err")))
            self.hh_busy = False
            if self.hh.available():
                self.evq.put(("hh_refresh", None))
        threading.Thread(target=work, daemon=True).start()

    def _spawn_console(self, flag):
        """Run a traction_control.py tool in its own console; the engine is
        paused for the tool's lifetime and restarted after (they'd fight over
        UDP 7777 and the virtual pads)."""
        if self.demo:
            self._log_line(f"[gui] {flag} unavailable in --demo", "warn")
            return
        if self.tool_busy:
            self._log_line("[gui] a tool is already running", "warn")
            return
        self.tool_busy = True
        old = self.engine
        was_running = not old.finished.is_set()

        def work():
            try:
                if was_running:
                    self.evq.put(("log", f"[gui] pausing TC engine for {flag}"))
                    old.stop()
                try:
                    p = subprocess.Popen(
                        _tool_command(flag) + self.cli_args,
                        creationflags=subprocess.CREATE_NEW_CONSOLE)
                    self.evq.put(("log", f"[gui] running {flag} "
                                         f"-- close its console to resume"))
                    p.wait()
                except Exception as e:
                    self.evq.put(("log", (f"[gui] launch failed: {e}", "err")))
                if was_running and self.make_engine is not None:
                    eng = self.make_engine()
                    self.engine = eng
                    eng.start()
                    # carry state over as it stands NOW (not at tool launch)
                    eng.set_mode_ch("tc", old.tc.mode.name)
                    eng.set_mode_ch("abs", old.abs_ctl.mode.name)
                    eng.set_drivetrain(old.tc.drivetrain_override)
                    for chn in ("tc", "abs"):
                        for ax in ("front", "rear"):
                            eng.set_long_only_ch(chn, ax, old.long_only(chn, ax))
                    self.evq.put(("log", "[gui] TC engine restarted"))
            finally:
                self.tool_busy = False
        threading.Thread(target=work, daemon=True).start()

    # ---------------- HidHide ----------------
    def _cloak_watch(self):
        """Cloak watchdog: 'device in the hide list' means NOTHING while the
        master cloak is off (the two-part trap: the pad leaked on-rig with
        the list showing HIDDEN but 'Enable device hiding' unchecked).
        Poll the admin-free cloak state every few seconds and RE-ASSERT the
        cloak whenever hiding is expected but inactive."""
        if self._quitting or self.demo or not self.hh.available():
            return
        def work():
            val = self.hh.cloak_on()
            self.evq.put(("cloak_state", val))
        threading.Thread(target=work, daemon=True).start()
        self.root.after(7000, self._cloak_watch)

    def _cloak_state_event(self, val):
        if val is not None and hasattr(self, "hh_status"):
            self.hh_status.config(
                text=f"cloak {'ON' if val else 'OFF -- hiding inactive!'}",
                fg=C["green"] if val else C["red"])
        if (val is False and self.unhide_var.get() and not self.tool_busy
                and not self._quitting
                and time.time() - self._cloak_fix_t > 60.0):
            self._cloak_fix_t = time.time()
            self._log_line("[hidhide] cloak is OFF while hiding is expected "
                           "-- re-enabling it now", "err")

            def fix():
                self._ensure_pad_hidden()
                self._set_cloak(True)
                self.evq.put(("log", (
                    "[hidhide] cloak re-enabled. If the game is running it "
                    "STILL holds the unhidden pad -- click 'Re-cycle "
                    "controller' when safe.", "warn")))
                self._hh_refresh_async()
            threading.Thread(target=fix, daemon=True).start()

    def _hh_refresh_async(self):
        if self.hh_busy:
            return
        self.hh_busy = True

        def work():
            devs = self.hh.gaming_devices()
            hidden = self.hh.hidden_paths()
            cloak = self.hh.cloak_on()
            self.evq.put(("hh_state", (devs, hidden, cloak)))
            self.hh_busy = False
        threading.Thread(target=work, daemon=True).start()

    def _hh_apply_state(self, devs, hidden, cloak):
        # only CONNECTED devices are actionable; the disconnected ones are
        # phantoms Windows remembers (mostly leftover ViGEm virtual pads from
        # past sessions) -- filtering them matches HidHide's own default and
        # stops the list from filling with dead XBOX 360 entries
        present = [d for d in devs if d[2]]
        n_absent = len(devs) - len(present)
        self.hh_devices = present
        self.hh_list.delete(0, "end")
        for fn, path, _ in present:
            # relabel our own virtual output plainly (it is never a hide
            # target) and trim the manufacturer prefix off real controllers,
            # so each row fits the panel instead of running off the edge
            name = "Virtual pad (Xbox 360)" if _is_vigem_x360(path) \
                else _short_name(fn)
            # hidden is None when the state can't be read without admin
            state = ("?" if hidden is None
                     else "HIDDEN" if path in hidden else "shown")
            self.hh_list.insert("end", f"{name}  ·  {state}")
        if not present:
            self.hh_list.insert("end", "(no controller connected)")
        if n_absent:
            self.hh_list.insert("end", f"(+{n_absent} disconnected, hidden)")
        if self.demo:
            # demo never manages hiding, so the real cloak state is not ours
            # to alarm about -- a red "hiding inactive!" here is a false alarm
            self.hh_status.config(text="cloak: not managed (demo)",
                                  fg=C["dim"])
        elif cloak is None:
            self.hh_status.config(text="cloak: unknown (state unreadable)",
                                  fg=C["amber"])
        else:
            self.hh_status.config(
                text=f"cloak {'ON' if cloak else 'OFF -- hiding inactive!'}",
                fg=C["green"] if cloak else C["red"])

    def _hh_toggle(self, hide: bool):
        sel = self.hh_list.curselection()
        if not sel or sel[0] >= len(self.hh_devices):
            self._log_line("[hidhide] select a device in the list first", "warn")
            return
        fn, path, _ = self.hh_devices[sel[0]]
        if self.hh_busy:
            return
        self.hh_busy = True

        def work():
            ok, out = self.hh.modify(["--dev-hide" if hide else "--dev-unhide", path])
            if hide and ok:
                self.hh.modify(["--cloak-on"])       # always ensure cloak active
            nm = _short_name(fn)
            if ok and hide:
                # the state read needs admin so we can't verify here; the real
                # gate is HidHide's own requirement -- reconnect + game restart
                msg = (f"[hidhide] hid {nm}. NOW: reconnect the pad, then start "
                       f"the game AFTER this app -- else it keeps the old handle "
                       f"and the pad still leaks.")
                tag = None
            elif ok:
                msg = f"[hidhide] unhid {nm}."
                tag = None
            else:
                msg = f"[hidhide] {'hide' if hide else 'unhide'} failed: {out.strip()[:100]}"
                tag = "err"
            self.evq.put(("log", (msg, tag)))
            self.hh_busy = False
            self.evq.put(("hh_refresh", None))
        threading.Thread(target=work, daemon=True).start()

    def _hh_whitelist(self):
        if self.hh_busy:
            return
        self.hh_busy = True
        exes: list[str] = []

        def _add(p):
            if p and os.path.exists(p) and p.lower() not in (e.lower() for e in exes):
                exes.append(p)

        if tcm.is_frozen():
            # Installed build: the process that opens the pad IS this exe, so
            # that is what must be whitelisted. Whitelisting an interpreter
            # here would register something that never touches the device, and
            # the app would read its own cloaked pad as absent. The console
            # tools exe opens the pad too (--map, --leak-test).
            _add(sys.executable)
            _add(os.path.join(os.path.dirname(sys.executable),
                              "fh6tc-tools.exe"))
        else:
            # From source: register BOTH the running interpreter and the base
            # install, since from a venv the exe that opens the pad can be
            # either one, and a missing whitelist target reads as absent.
            for base in {os.path.dirname(sys.executable), sys.base_prefix}:
                for name in ("python.exe", "pythonw.exe"):
                    _add(os.path.join(base, name))

        def work():
            for exe in exes:
                ok, out = self.hh.modify(["--app-reg", exe])
                listed = exe.lower() in self.hh.app_list().lower()
                verdict = "ok" if listed else \
                    "FAILED (not in --app-list) " + out.strip()[:120]
                self.evq.put(("log", (f"[hidhide] whitelist {os.path.basename(exe)}: "
                                      f"{verdict}", None if listed else "err")))
            self.hh_busy = False
        threading.Thread(target=work, daemon=True).start()

    # ---------------- render loop ----------------
    def schedule(self):
        self._tick_once()
        self.root.after(33, self.schedule)

    def _tick_once(self):
        while True:
            try:
                kind, payload = self.evq.get_nowait()
            except queue.Empty:
                break
            if kind == "log":
                msg, tag = payload if isinstance(payload, tuple) else (payload, None)
                self._log_line(msg, tag)
            elif kind == "hh_state":
                self._hh_apply_state(*payload)
            elif kind == "cloak_state":
                self._cloak_state_event(payload)
            elif kind == "hh_refresh":
                self._hh_refresh_async()
        s = self.engine.snapshot()
        if s:
            self._render(s)
        if self.tool_busy:
            self.status_vars["pad"].set("tool running...")
        elif self.engine.finished.is_set() and self.engine.error:
            self.status_vars["pad"].set("ENGINE STOPPED")

    def _log_line(self, msg, tag=None):
        if tag is None and ("[LEAK]" in msg or "ERROR" in msg):
            tag = "err"
        self.log.config(state="normal")
        self.log.insert("end", msg.rstrip() + "\n", tag or ())
        if int(self.log.index("end-1c").split(".")[0]) > 200:
            self.log.delete("1.0", "2.0")
        self.log.see("end")
        self.log.config(state="disabled")

    def _render(self, s):
        for ch in CHANNELS:
            self._render_controls(ch, s)
        cur = self._match_preset(s)
        if cur != self._last_preset_shown:
            self._last_preset_shown = cur
            for name, b in self.preset_btns.items():
                on = name == cur
                b.config(bg=C["green"] if on else C["inset"],
                         fg=C["bg"] if on else C["text"])

        # the pad list is built at startup / on Rescan, so it goes stale when a
        # controller is plugged in later -- refresh it whenever the engine's
        # pad changes (plug, unplug, cycle, switch)
        pad_now = s.get("pad_id") if s.get("pad_ok") else None
        if pad_now != self._last_pad_seen:
            self._last_pad_seen = pad_now
            if not self.demo:
                self._rescan_pads()
                if self.hh.available():
                    if pad_now is not None:
                        # a controller that appears AFTER startup was never
                        # seen by the startup auto-hide, so hide it now --
                        # otherwise it reaches the game unhidden
                        self._on_pad_appeared()
                    else:
                        self._hh_refresh_async()

        rec = bool(s.get("recording"))
        if rec != self._last_rec_shown:
            self._last_rec_shown = rec
            self.rec_btn.config(
                text="■ Stop" if rec else "● Record",
                bg=C["red"] if rec else C["inset"],
                fg=C["white"] if rec else C["red"])
        if rec:
            secs = s.get("rec_secs", 0.0)
            self.rec_status.config(
                text=f"REC {int(secs) // 60:d}:{int(secs) % 60:02d} "
                     f"({s.get('rec_rows', 0)})", fg=C["red"])
        elif self.rec_status["text"]:
            self.rec_status.config(text="")
        active = "tc" if self.nb.index(self.nb.select()) == 0 else "abs"
        self._render_circles(active, s)
        self._render_bars(active, s)

        sv = self.status_vars
        sv["speed"].set(f"{s['speed_kmh']:5.0f} km/h")
        sv["gear"].set(f"gear {s['gear']}")
        age = s["telem_age"] * 1000
        sv["telem"].set("telem: none" if age > 1000 else
                        f"telem {age:3.0f}ms" + ("" if s["race"] else " menu"))
        if not self.tool_busy:
            sv["pad"].set(("pad: " + s["pad_name"][:18]) if s.get("pad_ok")
                          else "pad: DISCONNECTED")
        lt = s.get("leak_test_left", 0.0)
        sv["leak"].set(f"TEST {lt:.0f}s" if lt > 0 else
                       "LEAK!" if s["leak"] else
                       "leaked" if s["ever_leaked"] else "leak: ok")
        sv["cuts"].set(f"tc {s['interventions']}")
        sv["abs_cuts"].set(f"abs {s.get('abs_interventions', 0)}")
        fe = s.get("ffb_events", -1)
        sv["ffb"].set("ffb off" if fe < 0 else f"ffb {fe}")

    def _render_controls(self, ch, s):
        cfg = CHANNELS[ch]
        ui = self.ui[ch]
        mode = s.get(cfg["k_mode"], "?")
        if mode != ui["last_mode"]:
            ui["last_mode"] = mode
            for name, b in ui["mode_btns"].items():
                on = name == mode
                b.config(bg=C["blue"] if on else C["inset"],
                         fg=C["bg"] if on else C["dim"])
        cm_t = ch_modes(ch)[tcm.MODE_IDX["CUSTOM"]].target
        if abs(cm_t - ui["last_custom"]) > 1e-9:
            ui["mode_btns"]["CUSTOM"].config(
                text=f"CUSTOM\n{cm_t * 100:.0f}% setpoint")
            # don't fight an in-progress edit -- but if the sync is skipped,
            # leave last_custom stale so it RETRIES next tick instead of the
            # spinner displaying (and later re-applying) an outdated value
            try:
                focused = self.root.focus_get() is ui["custom_spin"]
            except Exception:
                focused = False
            if not focused:
                ui["custom_var"].set(f"{cm_t * 100:.0f}")
                ui["last_custom"] = cm_t
        cur_long = (bool(s.get(cfg["k_lf"])), bool(s.get(cfg["k_lr"])))
        if cur_long != ui["last_long"]:
            ui["last_long"] = cur_long
            for axle, on in zip(("front", "rear"), cur_long):
                ui["long_btns"][axle].config(
                    bg=C["target"] if on else C["inset"],
                    fg=C["bg"] if on else C["dim"])
        if "learn_btn" in ui:
            lr = bool(s.get("abs_learn"))
            if lr != ui["last_learn"]:
                ui["last_learn"] = lr
                ui["learn_btn"].config(bg=C["green"] if lr else C["inset"],
                                       fg=C["bg"] if lr else C["dim"])
            if lr:
                txt = (f"ceiling {s.get('learn_ceiling', 0) * 100:5.0f}%\n"
                       f"live cap {s.get(cfg['k_target'], 0) * 100:5.0f}%\n"
                       f"corner budget {s.get('learn_alat_g', 0):.1f}g")
            else:
                txt = "off, setpoint is fixed"
            if txt != ui["last_learn_txt"]:
                ui["last_learn_txt"] = txt
                ui["learn_lbl"].config(text=txt,
                                       fg=C["green"] if lr else C["dim"])

    def _render_circles(self, ch, s):
        cfg = CHANNELS[ch]
        cv = self.ui[ch]["canvas"]
        cv.delete("dyn")
        half = self.CELL // 2
        stale = s["wheels"] is None or s["telem_age"] > tcm.STALE_S
        watched = WHEELS if ch == "abs" else DRIVEN.get(s["drivetrain_eff"], WHEELS)
        mode = s.get(cfg["k_mode"], "OFF")
        tgt = s.get(cfg["k_target"], 1.0)
        lockup = cfg["lockup"]
        # auto-zoom: a setpoint beyond the default 150% display range (e.g.
        # CUSTOM 250%) shrinks the value scale so the setpoint AND the slip
        # around it stay on screen -- otherwise the dots pin at an invisible
        # display ceiling below the setpoint and the regulation is unreadable
        show_t = tgt if (mode != "OFF" and tgt < 50.0) else None
        vs = self.R * min(1.0, 1.5 / (show_t if show_t else 1.0))
        disp_cap = 1.55 * self.R / vs          # max displayable value (units)
        for i, w in enumerate(WHEELS):
            cx = half + (i % 2) * self.CELL
            cy = half + (i // 2) * self.CELL
            is_watched = w in watched
            axle_long = bool(s.get(cfg["k_lf"]) if w in ("FL", "FR")
                             else s.get(cfg["k_lr"]))
            cv.itemconfig(f"label_{w}",
                          fill=C["white"] if is_watched else C["faint"])
            for frac, col in ((0.5, C["ring2"]), (1.0, "#46505c")):
                rr = vs * frac
                cv.create_oval(cx - rr, cy - rr, cx + rr, cy + rr,
                               outline=col, width=2 if frac == 1.0 else 1,
                               tags="dyn")
            if show_t is not None and is_watched:
                if axle_long:
                    # the constrained component is fwd/back only; lockup is
                    # NEGATIVE longitudinal slip, so ABS's line sits BELOW
                    y_t = cy + vs * show_t if lockup else cy - vs * show_t
                    cv.create_line(cx - self.R * 1.15, y_t, cx + self.R * 1.15,
                                   y_t, fill=C["target"], dash=(4, 4), tags="dyn")
                else:
                    rr = vs * show_t
                    cv.create_oval(cx - rr, cy - rr, cx + rr, cy + rr,
                                   outline=C["target"], dash=(4, 4), tags="dyn")
            if stale:
                continue
            ratio, angle, comb = s["wheels"][w]
            if is_watched and axle_long:
                x = cx + max(-disp_cap, min(disp_cap, angle)) * vs
                y = cy - max(-disp_cap, min(disp_cap, ratio)) * vs
            else:
                mag = min(abs(comb), disp_cap)
                if abs(ratio) < 1e-4 and abs(angle) < 1e-4:
                    th = math.pi / 2
                else:
                    th = math.atan2(ratio, angle)
                x = cx + math.cos(th) * mag * vs
                y = cy - math.sin(th) * mag * vs
            trail = self.trails[ch][w]
            trail.append((x, y))
            n = len(trail)
            for k, (tx, ty) in enumerate(trail):
                if k == n - 1:
                    continue
                rr = 1.5 + 1.5 * (k / max(n - 1, 1))
                cv.create_oval(tx - rr, ty - rr, tx + rr, ty + rr,
                               fill=C["ring"], outline="", tags="dyn")
            if is_watched and axle_long:
                sig = max(0.0, -ratio) if lockup else max(0.0, ratio)
            else:
                sig = abs(comb)
            col = util_color(sig) if is_watched else C["faint"]
            cv.create_oval(x - 5, y - 5, x + 5, y + 5, fill=col, outline="",
                           tags="dyn")
            cv.create_text(cx + half - 26, cy + half - 14,
                           text=f"{sig * 100:3.0f}%", font=F_MONO,
                           fill=col, tags="dyn")
        if stale:
            cv.create_text(self.CELL, self.CELL, text="waiting for telemetry",
                           font=F, fill=C["faint"], tags="dyn")
            for t_ in self.trails[ch].values():
                t_.clear()

    def _render_bars(self, ch, s):
        cfg = CHANNELS[ch]
        b = self.ui[ch]["bars"]
        b.delete("all")
        W = self.CELL * 2 - 120
        x0 = 70
        scale = s.get(cfg["k_scale"], 1.0)
        cutting = scale < 0.995
        vals = {"k_in": s.get(cfg["k_in"], 0.0), "k_out": s.get(cfg["k_out"], 0.0)}
        for r_i, (lab, key) in enumerate(cfg["bars"]):
            frac = vals.get(key, s.get(key, 0.0))
            col = (C["dim"] if r_i == 0 else
                   (C["amber"] if cutting else C["green"]) if r_i == 1 else
                   C["blue"])
            y = 8 + r_i * 22
            h = 14 if r_i < 2 else 8
            b.create_text(x0 - 10, y + h / 2, text=lab, anchor="e",
                          font=F_MONO, fill=C["dim"])
            b.create_rectangle(x0, y, x0 + W, y + h, fill=C["inset"], outline="")
            b.create_rectangle(x0, y, x0 + W * max(0.0, min(1.0, frac)), y + h,
                               fill=col, outline="")
        b.create_text(x0 + W + 26, 23, text=f"x{scale:.2f}", font=F_MONO,
                      fill=C["amber"] if cutting else C["dim"])

    def _on_close(self):
        # keep the window UP during shutdown (it can take a few seconds, or a
        # UAC prompt when not elevated). Withdrawing would leave an invisible
        # process still holding the virtual pad and the single-instance mutex.
        self._quitting = True          # stop the cloak watchdog re-asserting
        self._closing = {}
        try:
            self.root.title("FH6 TC: restoring your controller, one moment...")
            self.rec_status.config(text="shutting down...", fg=C["amber"])
        except Exception:
            pass

        def shutdown():
            # give the controller back FIRST, so it is usable again even if
            # the engine teardown is slow
            if self.unhide_var.get() and not self.demo and self.hh.available():
                try:
                    self._set_cloak(False)
                    # verify with --cloak-state (no admin needed, unlike
                    # --dev-list) so a failed unhide isn't swallowed
                    self._closing["cloak"] = self.hh.cloak_on()
                except Exception:
                    self._closing["cloak"] = None
            self.engine.stop()
        w = threading.Thread(target=shutdown, daemon=True)
        w.start()
        self._poll_close(w)

    def _poll_close(self, w):
        if w.is_alive():
            self.root.after(100, lambda: self._poll_close(w))
            return
        # cloak still ON means the pad is STILL hidden from every other game,
        # and the events log is about to die with the window -- say so plainly
        if getattr(self, "_closing", {}).get("cloak") is True:
            try:
                from tkinter import messagebox
                messagebox.showwarning(
                    "Controller still hidden",
                    "FH6 TC could not turn HidHide's cloak off, so your "
                    "controller is still hidden from other games.\n\n"
                    "Fix it either way:\n"
                    "  • relaunch FH6 TC and close it again, or\n"
                    "  • open the HidHide Configuration Client and untick "
                    "'Enable device hiding'.")
            except Exception:
                pass
        self.root.destroy()


_SINGLE_INSTANCE_MUTEX = None       # kept alive for the process lifetime
_MUTEX_NAME = "Local\\FH6-TC-single-instance"
# everyone-full-access + LOW mandatory label. The launcher elevates, so the
# first instance is High integrity; without this SD its mutex would be
# invisible to a normal-integrity second copy (CreateMutexW would fail with
# ERROR_ACCESS_DENIED, not ALREADY_EXISTS) and the guard would fail OPEN --
# exactly the two-instance bug it exists to prevent.
_MUTEX_SDDL = "D:(A;;GA;;;WD)S:(ML;;NW;;;LW)"


class _SECURITY_ATTRIBUTES(ctypes.Structure):
    _fields_ = [("nLength", wintypes.DWORD),
                ("lpSecurityDescriptor", ctypes.c_void_p),
                ("bInheritHandle", wintypes.BOOL)]


def _single_instance() -> bool:
    """True if we are the only instance. A second copy = a second ViGEm pad
    fighting the first for the game's controller slot and UDP 7777, which
    destabilizes the game's input -- so refuse to start twice."""
    global _SINGLE_INSTANCE_MUTEX
    try:
        k = ctypes.WinDLL("kernel32", use_last_error=True)
        adv = ctypes.WinDLL("advapi32", use_last_error=True)
        k.CreateMutexW.argtypes = [ctypes.c_void_p, wintypes.BOOL, wintypes.LPCWSTR]
        k.CreateMutexW.restype = wintypes.HANDLE
        conv = adv.ConvertStringSecurityDescriptorToSecurityDescriptorW
        conv.argtypes = [wintypes.LPCWSTR, wintypes.DWORD,
                         ctypes.POINTER(ctypes.c_void_p),
                         ctypes.POINTER(wintypes.DWORD)]
        conv.restype = wintypes.BOOL

        psa = None
        psd = ctypes.c_void_p()
        if conv(_MUTEX_SDDL, 1, ctypes.byref(psd), None):
            sa = _SECURITY_ATTRIBUTES(ctypes.sizeof(_SECURITY_ATTRIBUTES), psd, False)
            psa = ctypes.byref(sa)
        h = k.CreateMutexW(psa, False, _MUTEX_NAME)
        err = ctypes.get_last_error()
        _SINGLE_INSTANCE_MUTEX = h
        if h and err == 183:                    # ERROR_ALREADY_EXISTS
            return False
        if not h and err == 5:                  # ERROR_ACCESS_DENIED: it
            return False                        # exists, we just can't open it
        return True
    except Exception:
        return True                             # never block on a guard failure


def main() -> int:
    ap = argparse.ArgumentParser(description="FH6 TC GUI")
    ap.add_argument("--mode", default="medium",
                    choices=["off", "low", "medium", "high", "custom"])
    ap.add_argument("--abs-mode", default="medium",
                    choices=["off", "low", "medium", "high", "custom"])
    ap.add_argument("--custom-target", type=float, default=None,
                    help="TC CUSTOM setpoint in percent (e.g. 92)")
    ap.add_argument("--abs-custom-target", type=float, default=None,
                    help="ABS CUSTOM setpoint in percent")
    ap.add_argument("--port", type=int, default=7777)
    ap.add_argument("--joy-id", type=int, default=None)
    ap.add_argument("--tick", type=float, default=0.003)
    ap.add_argument("--game-exe", default=tcm.GAME_EXE_DEFAULT)
    ap.add_argument("--no-dialog-clear", action="store_true")
    ap.add_argument("--demo", action="store_true", help="synthetic data (no game/pad needed)")
    ap.add_argument("--smoke", action="store_true", help="headless build + a few frames, then exit")
    args = ap.parse_args()

    # a real run must be the only instance (demo/smoke are exempt for testing)
    if not (args.demo or args.smoke) and not _single_instance():
        from tkinter import messagebox
        warn = tk.Tk()
        warn.withdraw()
        messagebox.showwarning(
            "FH6 TC already running",
            "FH6 TC is already open in another window.\n\n"
            "Running two copies plugs two virtual controllers into the game "
            "and makes it drop your controller. Close this and use the one "
            "that's already open.")
        warn.destroy()
        return 1

    evq: queue.Queue = queue.Queue()
    demo = args.demo or args.smoke
    cli_args = ["--port", str(args.port)]
    if args.joy_id is not None:
        cli_args += ["--joy-id", str(args.joy_id)]
    if demo:
        engine = DemoEngine(args.mode, args.abs_mode)
        make_engine = None
    else:
        tcm.disable_quickedit()

        def make_engine():
            return tcm.TCEngine(mode=args.mode.upper(), port=args.port,
                                joy_id=args.joy_id, tick=args.tick,
                                game_exe=args.game_exe,
                                dialog_clear=not args.no_dialog_clear,
                                abs_mode=args.abs_mode.upper(),
                                custom_tc=args.custom_target,
                                custom_abs=args.abs_custom_target,
                                emit=lambda m: evq.put(("log", m)))
        engine = make_engine()
    root = tk.Tk()
    if args.smoke:
        root.withdraw()
    app = App(root, engine, evq, demo=demo, make_engine=make_engine,
              cli_args=cli_args)
    if demo:                     # demo engine has no tune file: apply directly
        if args.custom_target is not None:
            engine.set_custom_target_ch("tc", args.custom_target)
        if args.abs_custom_target is not None:
            engine.set_custom_target_ch("abs", args.abs_custom_target)
    engine.start()
    if args.smoke:
        for tab in (0, 1, 0):                    # exercise both tabs
            app.nb.select(tab)
            for _ in range(6):
                app._tick_once()
                root.update()
                time.sleep(0.02)
        for name in PRESETS:                     # exercise the presets
            app._apply_preset(name)
            app._tick_once()
            root.update()
        app._toggle_record()                     # exercise record start/stop
        app._tick_once()
        root.update()
        app._toggle_record()
        app._tick_once()
        root.update()
        engine.stop()
        root.destroy()
        print("SMOKE OK")
        return 0
    app.schedule()
    root.mainloop()
    return 0


if __name__ == "__main__":
    # Single-exe fallback: a frozen build with no sibling tools exe can still
    # reach the console tools as "FH6 TC.exe --tool --map". The installed
    # build ships fh6tc-tools.exe instead, which is preferred because this
    # one is windowed and has no console for the tool's prompts.
    if len(sys.argv) > 2 and sys.argv[1] == "--tool":
        del sys.argv[1]
        raise SystemExit(tcm.main())
    raise SystemExit(main())
