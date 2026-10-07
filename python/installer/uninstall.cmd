@echo off
rem Uninstalls the LogixPlan Robotic Toolbox. Add -RemoveData to also delete the config and the robot data.
rem
rem uninstall.ps1 deletes the install folder, this file included. So the script runs from a copy in %TEMP%, and the
rem last line starts with "(goto) 2>nul": that ends the batch file (cmd would otherwise re-read the deleted file and
rem fail), while the rest of the line, already read and expanded, still runs.
set "LP_DIR=%~dp0."
cd /d "%TEMP%"
copy /y "%LP_DIR%\uninstall.ps1" "%TEMP%\logixplan-uninstall.ps1" >nul
(goto) 2>nul & powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%TEMP%\logixplan-uninstall.ps1" -InstallDir "%LP_DIR%" %* & if errorlevel 1 (echo. & echo Uninstall FAILED. & (if not defined NO_PAUSE pause) & cmd /c exit 1) else (del "%TEMP%\logixplan-uninstall.ps1" >nul 2>&1 & if not defined NO_PAUSE pause)
