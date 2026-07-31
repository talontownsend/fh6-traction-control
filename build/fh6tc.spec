# PyInstaller spec for FH6 TC.
#
# Builds TWO executables into one --onedir distribution:
#   "FH6 TC.exe"      windowed GUI, the thing users launch
#   "fh6tc-tools.exe" console companion for the interactive CLI tools
#                     (--map, --leak-test, --rumble-test, --selftest)
#
# Why two: the interactive tools prompt on stdin, and a windowed build has no
# console to prompt in. Why --onedir rather than --onefile: a one-file build
# unpacks to a temp directory on every launch, which is slow, is a common
# antivirus heuristic trigger, and would break the ViGEmClient DLL lookup.
#
# Build:  pyinstaller build/fh6tc.spec --noconfirm
# Output: dist/FH6 TC/

import os

# vgamepad ships the ViGEm client DLL inside the package and loads it by
# path at import time, so it has to travel with us in the same layout.
import vgamepad
VG = os.path.dirname(vgamepad.__file__)
vg_binaries = [
    (os.path.join(r, f), os.path.relpath(r, os.path.dirname(VG)))
    for r, _, fs in os.walk(VG) for f in fs if f.lower().endswith(".dll")
]

common = dict(
    pathex=[],
    binaries=vg_binaries,
    datas=[],
    # winmm/setupapi/hid are reached through ctypes.windll, which PyInstaller
    # cannot see statically; they are OS libraries so they need no bundling,
    # but vgamepad's own imports do need declaring.
    hiddenimports=["vgamepad", "vgamepad.win"],
    hookspath=[],
    runtime_hooks=[],
    # tkinter IS used; trim only what is definitely dead weight.
    excludes=["matplotlib", "numpy", "PIL", "pytest", "setuptools"],
    noarchive=False,
)

gui_a = Analysis(["../tc_gui.py"], **common)
tools_a = Analysis(["../traction_control.py"], **common)

MERGE((gui_a, "gui", "FH6 TC"), (tools_a, "tools", "fh6tc-tools"))

gui_pyz = PYZ(gui_a.pure)
tools_pyz = PYZ(tools_a.pure)

gui_exe = EXE(
    gui_pyz, gui_a.scripts, [],
    exclude_binaries=True,
    name="FH6 TC",
    console=False,              # windowed: no console flash on launch
    # The app self-elevates today via a .bat wrapper. Requesting elevation in
    # the manifest replaces that: HidHide hide/unhide and reading cloak state
    # need admin, and without it every toggle throws a UAC prompt, including
    # an invisible one during shutdown.
    uac_admin=True,
    disable_windowed_traceback=False,
)

tools_exe = EXE(
    tools_pyz, tools_a.scripts, [],
    exclude_binaries=True,
    name="fh6tc-tools",
    console=True,               # the interactive tools prompt on stdin
    # Deliberately NOT uac_admin: when the GUI spawns a tool the child
    # inherits the GUI's elevated token, and demanding a manifest here would
    # make plain "fh6tc-tools --selftest" throw a UAC prompt for a test run
    # that touches nothing privileged.
)

COLLECT(
    gui_exe, gui_a.binaries, gui_a.datas,
    tools_exe, tools_a.binaries, tools_a.datas,
    strip=False,
    upx=False,                  # UPX packing is a strong antivirus heuristic
    name="FH6 TC",
)
