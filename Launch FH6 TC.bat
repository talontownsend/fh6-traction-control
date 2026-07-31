@echo off
rem FH6 TC -- double-click to launch the traction-control + ABS GUI.
rem Elevates once up front so HidHide hide/unhide (and reading its state)
rem work silently, instead of throwing a UAC prompt every time -- including
rem an invisible one while the app is closing.
net session >nul 2>&1
if %errorlevel% neq 0 (
    powershell -NoProfile -Command "Start-Process -FilePath '%~f0' -Verb RunAs" 2>nul
    if not errorlevel 1 exit /b
    echo.
    echo  Could not run as administrator - the UAC prompt was declined or blocked.
    echo.
    echo  FH6 TC will start anyway, but hiding/unhiding your controller will pop
    echo  a UAC prompt each time, including one while the app is closing.
    echo.
    timeout /t 6 >nul
)
set "PYW=pythonw.exe"
if exist "%~dp0.venv\Scripts\pythonw.exe" set "PYW=%~dp0.venv\Scripts\pythonw.exe"
start "" "%PYW%" "%~dp0tc_gui.py"
