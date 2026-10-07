<#
.SYNOPSIS
    Removes the LogixPlan Robotic Toolbox installed by install.ps1.

.DESCRIPTION
    Removes the install folder (with its private Python and the demo robot), the shortcuts and the Settings > Apps
    entry. The config folder and the robot data (poses, chains, groups …) are kept unless -RemoveData is given.
    RC_CONFIG_DIR is removed only if it points to this install's config folder.

    Run it with uninstall.cmd: it runs a copy of this script from %TEMP%, so the install folder can be deleted.
#>
[CmdletBinding()]
param(
    [string]$InstallDir = (Split-Path -Parent $MyInvocation.MyCommand.Path),
    [string]$ConfigDir = (Join-Path $env:LOCALAPPDATA "LogixPlan\config"),
    [switch]$RemoveData
)

$ErrorActionPreference = "Stop"
$InstallDir = [IO.Path]::GetFullPath($InstallDir.TrimEnd('\', '.'))
$UninstallKey = "HKCU:\Software\Microsoft\Windows\CurrentVersion\Uninstall\LogixPlanRoboticToolbox"

if (-not (Test-Path (Join-Path $InstallDir "python\python.exe"))) {
    throw "$InstallDir does not look like a Robotic Toolbox install"
}
$running = Get-Process -ErrorAction SilentlyContinue | Where-Object {
    $_.Path -and $_.Path.StartsWith($InstallDir + "\", [StringComparison]::OrdinalIgnoreCase) }
if ($running) {
    throw "close these first: $(($running | ForEach-Object { $_.ProcessName } | Sort-Object -Unique) -join ', ')"
}

Write-Host "Removing the LogixPlan Robotic Toolbox from $InstallDir"
Remove-Item -Recurse -Force (Join-Path ([Environment]::GetFolderPath("Programs")) "LogixPlan") -ErrorAction SilentlyContinue
Remove-Item -Force (Join-Path ([Environment]::GetFolderPath("Desktop")) "Robotic Toolbox.lnk") -ErrorAction SilentlyContinue
Remove-Item -Recurse -Force $UninstallKey -ErrorAction SilentlyContinue
if ([Environment]::GetEnvironmentVariable("RC_CONFIG_DIR", "User") -eq $ConfigDir) {
    [Environment]::SetEnvironmentVariable("RC_CONFIG_DIR", $null, "User")
    Write-Host "removed the RC_CONFIG_DIR variable"
}

if ($RemoveData) {
    $config = Join-Path $ConfigDir "remote_control.json"
    if (Test-Path $config) {
        $data = (Get-Content $config -Raw | ConvertFrom-Json).data_dir
        if ($data -and (Test-Path $data)) { Remove-Item -Recurse -Force $data; Write-Host "removed $data" }
    }
    Remove-Item -Recurse -Force $ConfigDir -ErrorAction SilentlyContinue
    Write-Host "removed $ConfigDir"
} else {
    Write-Host "kept the config ($ConfigDir) and the robot data (uninstall.cmd -RemoveData deletes them)"
}

Set-Location $env:TEMP          # a process cannot delete the folder it is working in
Remove-Item -Recurse -Force $InstallDir
Write-Host "Uninstalled." -ForegroundColor Green
