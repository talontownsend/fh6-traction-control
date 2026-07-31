@echo off
rem FH6 TC demo -- synthetic data, needs no game, controller, or ViGEmBus
set "PYW=pythonw.exe"
if exist "%~dp0.venv\Scripts\pythonw.exe" set "PYW=%~dp0.venv\Scripts\pythonw.exe"
start "" "%PYW%" "%~dp0tc_gui.py" --demo
