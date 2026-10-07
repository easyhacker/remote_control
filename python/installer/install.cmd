@echo off
rem Installs the LogixPlan Robotic Toolbox for the current user (no administrator rights needed).
rem Options are passed on to install.ps1, e.g.:  install.cmd -InstallDir D:\Apps\RoboticToolbox -Port 9000
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0install.ps1" %*
set RC=%ERRORLEVEL%
if not "%RC%"=="0" echo. & echo Installation FAILED (exit code %RC%).
if "%NO_PAUSE%"=="" pause
exit /b %RC%
