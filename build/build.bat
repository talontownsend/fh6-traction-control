@echo off
rem Full release build: PyInstaller executables, then the Inno Setup installer.
rem Run from anywhere; paths are resolved relative to this script.
rem
rem Output: build\out\FH6-TC-Setup-<version>.exe
setlocal
cd /d "%~dp0.."

set "PY=python"
if exist ".venv\Scripts\python.exe" set "PY=.venv\Scripts\python.exe"

set "ISCC=%LOCALAPPDATA%\Programs\Inno Setup 6\ISCC.exe"
if not exist "%ISCC%" set "ISCC=%ProgramFiles(x86)%\Inno Setup 6\ISCC.exe"

echo.
echo === 1/3  offline test suite ===
"%PY%" traction_control.py --selftest
if errorlevel 1 goto :failed

echo.
echo === 2/3  building executables ===
"%PY%" -m PyInstaller build\fh6tc.spec --noconfirm --distpath dist --workpath build\work
if errorlevel 1 goto :failed

echo.
echo === verifying the frozen build actually runs ===
"dist\FH6 TC\fh6tc-tools.exe" --selftest >nul
if errorlevel 1 (
    echo    FAILED: the frozen build does not pass its own tests.
    goto :failed
)
echo    frozen build passes its tests.

if not exist "%ISCC%" (
    echo.
    echo === 3/3  SKIPPED: Inno Setup not found ===
    echo    Install it with:  winget install JRSoftware.InnoSetup
    echo    The unpacked app is still ready in  dist\FH6 TC\
    goto :done
)

echo.
echo === 3/3  building installer ===
"%ISCC%" build\installer.iss
if errorlevel 1 goto :failed

:done
echo.
echo BUILD OK
dir /b build\out\*.exe 2>nul
endlocal
exit /b 0

:failed
echo.
echo BUILD FAILED
endlocal
exit /b 1
