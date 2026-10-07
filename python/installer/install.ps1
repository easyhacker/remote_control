<#
.SYNOPSIS
    Installs the LogixPlan Robotic Toolbox and the Remote Control Python component for the current user.

.DESCRIPTION
    No administrator rights needed. Steps:
      1. a private Python (python.org's NuGet package: bundled in python\, or downloaded) in <InstallDir>\python
      2. the compiled remote_control + robotic_toolbox packages and their dependencies, from wheels\ (offline)
      3. the Unity demo robot (built with IL2CPP), examples and the docs in <InstallDir>
      4. the config file <ConfigDir>\remote_control.json (kept if it exists) and the RC_CONFIG_DIR user variable
         (only if it is not set yet)
      5. Start menu and desktop shortcuts, and an entry in Settings > Apps for uninstalling
      6. a check that the installed app imports its compiled modules

    Run it with install.cmd (it bypasses the PowerShell script policy for this script only).

.EXAMPLE
    .\install.cmd
    .\install.cmd -InstallDir D:\Apps\RoboticToolbox -Port 9000
#>
[CmdletBinding()]
param(
    [string]$InstallDir = (Join-Path $env:LOCALAPPDATA "Programs\LogixPlan\RoboticToolbox"),
    [string]$ConfigDir = (Join-Path $env:LOCALAPPDATA "LogixPlan\config"),
    [string]$DataDir = (Join-Path ([Environment]::GetFolderPath("MyDocuments")) "LogixPlan\data"),
    [int]$Port = 8765,
    [switch]$NoShortcuts,         # no Start menu / desktop shortcuts
    [switch]$NoRegister,          # no Settings > Apps entry, no RC_CONFIG_DIR variable (testing)
    [switch]$Online               # download Python and the wheels even if they are bundled
)

$ErrorActionPreference = "Stop"
$ProgressPreference = "SilentlyContinue"      # Invoke-WebRequest is very slow with the progress bar
[Net.ServicePointManager]::SecurityProtocol = [Net.SecurityProtocolType]::Tls12

$Here = Split-Path -Parent $MyInvocation.MyCommand.Path
$Release = Get-Content (Join-Path $Here "release.json") -Raw | ConvertFrom-Json
$AppName = "LogixPlan Robotic Toolbox"
$UninstallKey = "HKCU:\Software\Microsoft\Windows\CurrentVersion\Uninstall\LogixPlanRoboticToolbox"

function Step($text) { Write-Host ""; Write-Host "== $text" -ForegroundColor Cyan }
function Info($text) { Write-Host "   $text" }

function Invoke-Checked {
    # Run a program; stop the install if it fails (PowerShell 5.1 does not do that for native programs).
    param([string]$Exe, [string[]]$Arguments)
    & $Exe @Arguments
    if ($LASTEXITCODE -ne 0) { throw "failed (exit code $LASTEXITCODE): $Exe $($Arguments -join ' ')" }
}

Write-Host "$AppName $($Release.version) - installing to $InstallDir"
if (-not [Environment]::Is64BitOperatingSystem) { throw "64-bit Windows is required" }

# ── 1. Python ────────────────────────────────────────────────────────────────
# python.org publishes every Windows release as a NuGet package: a complete Python in a zip (tools\), with no
# installer and no registry entries, so it stays private to this app and is removed with it.
Step "1/6 Python $($Release.python)"
$PythonDir = Join-Path $InstallDir "python"
$Python = Join-Path $PythonDir "python.exe"
$PythonW = Join-Path $PythonDir "pythonw.exe"
New-Item -ItemType Directory -Force -Path $InstallDir | Out-Null
if (Test-Path $PythonDir) {
    Info "removing the previous Python"
    Remove-Item -Recurse -Force $PythonDir
}
$Package = Get-ChildItem (Join-Path $Here "python") -Filter "python.*.nupkg" -ErrorAction SilentlyContinue |
    Select-Object -First 1
$Temp = Join-Path ([IO.Path]::GetTempPath()) ("logixplan-" + [Guid]::NewGuid().ToString("N"))
New-Item -ItemType Directory -Path $Temp | Out-Null
try {
    $Zip = Join-Path $Temp "python.zip"            # Expand-Archive only accepts .zip
    if ($Package -and -not $Online) {
        Info "bundled: $($Package.Name)"
        Copy-Item $Package.FullName $Zip
    } else {
        $Url = "https://www.nuget.org/api/v2/package/python/$($Release.python)"
        Info "downloading $Url"
        Invoke-WebRequest -Uri $Url -OutFile $Zip -UseBasicParsing
    }
    Expand-Archive -Path $Zip -DestinationPath (Join-Path $Temp "pkg")
    Move-Item (Join-Path $Temp "pkg\tools") $PythonDir
} finally {
    Remove-Item -Recurse -Force $Temp -ErrorAction SilentlyContinue
}
$Version = (& $Python -c "import sys; print('%d.%d.%d' % sys.version_info[:3])").Trim()
if ($Version -ne $Release.python) { throw "Python $Version installed, $($Release.python) expected" }
Info "Python $Version in $PythonDir"

# ── 2. packages ──────────────────────────────────────────────────────────────
Step "2/6 Remote Control + Robotic Toolbox (compiled) and dependencies"
$Wheels = Join-Path $Here "wheels"
$Wheel = Join-Path $Wheels $Release.wheel
if (-not (Test-Path $Wheel)) { throw "missing $Wheel" }
& $Python -m pip --version *> $null
if ($LASTEXITCODE -ne 0) { Invoke-Checked $Python @("-m", "ensurepip", "--default-pip") }   # pip from Python itself
$PipArgs = @("-m", "pip", "install", "--no-warn-script-location", "--disable-pip-version-check", "-q")
if ($Online) {
    Invoke-Checked $Python ($PipArgs + @("$Wheel[toolbox]"))
} else {
    # offline: only the wheels shipped with the release
    Invoke-Checked $Python ($PipArgs + @("--no-index", "--find-links", $Wheels, "$Wheel[toolbox]"))
}
Info ((& $Python -m pip list --disable-pip-version-check --format freeze) -join ", ")

# ── 3. files ─────────────────────────────────────────────────────────────────
Step "3/6 demo robot, examples, docs"
# robot\ is the Unity demo robot built with IL2CPP (native code); unity\ (the package as C# source) is only in
# releases built with --unity-source.
foreach ($item in @("robot", "examples", "unity", "README.md", "PROTOCOL.md", "release.json", "uninstall.ps1",
                    "uninstall.cmd")) {
    $src = Join-Path $Here $item
    if (Test-Path $src) {
        $dst = Join-Path $InstallDir $item
        if (Test-Path $dst) { Remove-Item -Recurse -Force $dst }
        Copy-Item -Recurse $src $dst
    }
}
$Robot = Join-Path $InstallDir "robot\RemoteControlDemo.exe"
if (Test-Path $Robot) { Info "demo robot (Unity, IL2CPP): $Robot" }
Info "examples: $(Join-Path $InstallDir 'examples')"
if (Test-Path (Join-Path $InstallDir "unity")) { Info "Unity package: $(Join-Path $InstallDir 'unity')" }

# ── 4. config ────────────────────────────────────────────────────────────────
# One config file for the whole system (Toolbox, examples and the Unity robot read the same file), found through
# the RC_CONFIG_DIR environment variable.
Step "4/6 configuration"
# An RC_CONFIG_DIR that is already set (e.g. an earlier setup of Unity or the examples) wins over the default
# folder: everything reads the config through that variable, so the config must be in the folder it names.
$Current = [Environment]::GetEnvironmentVariable("RC_CONFIG_DIR", "User")
if (-not $Current) { $Current = [Environment]::GetEnvironmentVariable("RC_CONFIG_DIR", "Machine") }
if ($Current -and -not $NoRegister -and -not $PSBoundParameters.ContainsKey("ConfigDir")) {
    $ConfigDir = $Current
    Info "RC_CONFIG_DIR is already set: using $ConfigDir"
}
$ConfigFile = Join-Path $ConfigDir "remote_control.json"
New-Item -ItemType Directory -Force -Path $ConfigDir, $DataDir | Out-Null
if (Test-Path $ConfigFile) {
    Info "kept the existing $ConfigFile"
} else {
    $Config = [ordered]@{
        data_dir  = $DataDir
        connector = [ordered]@{ type = "websocket"; host = "localhost"; port = $Port; path = "/motion";
                                listen_host = "0.0.0.0"; tls = $false; min_backoff = 0.5; max_backoff = 5.0 }
        heartbeat = [ordered]@{ interval = 0.5; timeout = 2.0 }
    }
    # UTF-8 without BOM (Set-Content -Encoding UTF8 writes a BOM in PowerShell 5.1)
    [IO.File]::WriteAllText($ConfigFile, ($Config | ConvertTo-Json -Depth 5), (New-Object Text.UTF8Encoding $false))
    Info "created $ConfigFile (WebSocket on port $Port, data in $DataDir)"
}
# Where the Toolbox is: Unity's "IVI Dynamic > Robotic Toolbox > Open Robotic Toolbox" starts what this names.
# Set in an existing config too (this install is now the Toolbox); every other setting there is kept.
$Doc = Get-Content $ConfigFile -Raw | ConvertFrom-Json
$Toolbox = [ordered]@{ command = $PythonW; args = "-m robotic_toolbox"; working_dir = $InstallDir }
$Doc | Add-Member -NotePropertyName toolbox -NotePropertyValue $Toolbox -Force
[IO.File]::WriteAllText($ConfigFile, ($Doc | ConvertTo-Json -Depth 10), (New-Object Text.UTF8Encoding $false))
Info "toolbox location saved in $ConfigFile"
if ($NoRegister) {
    Info "RC_CONFIG_DIR not changed (-NoRegister)"
} elseif ($Current -ne $ConfigDir) {
    # not set yet, or -ConfigDir given explicitly: point the variable at this install's config
    [Environment]::SetEnvironmentVariable("RC_CONFIG_DIR", $ConfigDir, "User")   # also tells Explorer
    Info "RC_CONFIG_DIR = $ConfigDir"
}

# ── 5. shortcuts and uninstall entry ─────────────────────────────────────────
Step "5/6 shortcuts"
$Icon = Join-Path $PythonDir "Lib\site-packages\robotic_toolbox\resources\robotic-arm.ico"
$StartMenu = Join-Path ([Environment]::GetFolderPath("Programs")) "LogixPlan"
$Uninstaller = Join-Path $InstallDir "uninstall.cmd"

function New-Shortcut($Path, $Target, $Arguments, $Description, $IconPath) {
    $shell = New-Object -ComObject WScript.Shell
    $s = $shell.CreateShortcut($Path)
    $s.TargetPath = $Target
    $s.Arguments = $Arguments
    $s.WorkingDirectory = $InstallDir
    $s.Description = $Description
    if ($IconPath) { $s.IconLocation = "$IconPath,0" }
    $s.Save()
}

if ($NoShortcuts) {
    Info "skipped (-NoShortcuts)"
} else {
    New-Item -ItemType Directory -Force -Path $StartMenu | Out-Null
    # pythonw: no console window behind the Toolbox
    New-Shortcut (Join-Path $StartMenu "Robotic Toolbox.lnk") $PythonW "-m robotic_toolbox" $AppName $Icon
    New-Shortcut (Join-Path ([Environment]::GetFolderPath("Desktop")) "Robotic Toolbox.lnk") $PythonW `
        "-m robotic_toolbox" $AppName $Icon
    if (Test-Path $Robot) {
        New-Shortcut (Join-Path $StartMenu "Demo robot (Unity).lnk") $Robot "" `
            "Unity demo arm that connects to the Toolbox" $Robot
    }
    New-Shortcut (Join-Path $StartMenu "Fake robot (demo).lnk") $Python "`"$InstallDir\examples\fake_robot.py`"" `
        "Simulated arm without graphics that connects to the Toolbox" $null
    New-Shortcut (Join-Path $StartMenu "Uninstall Robotic Toolbox.lnk") $Uninstaller "" "Uninstall $AppName" $null
    Info "Start menu: $StartMenu"
    Info "Desktop: Robotic Toolbox"
}
if (-not $NoRegister) {
    New-Item -Path $UninstallKey -Force | Out-Null
    $values = @{ DisplayName = $AppName; DisplayVersion = $Release.version; Publisher = "LogixPlan";
                 InstallLocation = $InstallDir; DisplayIcon = $Icon; UninstallString = "`"$Uninstaller`"";
                 NoModify = 1; NoRepair = 1 }
    foreach ($k in $values.Keys) { Set-ItemProperty -Path $UninstallKey -Name $k -Value $values[$k] }
    Info "listed in Settings > Apps"
}

# ── 6. check ─────────────────────────────────────────────────────────────────
Step "6/6 check"
$Check = "import remote_control.controller as c, robotic_toolbox.app as a, wx, numpy; " +
         "assert c.__file__.endswith('.pyd') and a.__file__.endswith('.pyd'), (c.__file__, a.__file__); " +
         "print('compiled modules OK -', 'wxPython', wx.version().split()[0], '- numpy', numpy.__version__)"
Invoke-Checked $Python @("-c", $Check)

Write-Host ""
Write-Host "Installed $AppName $($Release.version)." -ForegroundColor Green
Write-Host "Start it from the Start menu or the desktop (Robotic Toolbox), or:"
Write-Host "   `"$Python`" -m robotic_toolbox"
Write-Host "Robot: Start menu > LogixPlan > Demo robot (Unity), or Fake robot (demo) for one without graphics."
