param(
    [string]$Map = "crossfire",
    [int]$WaitSeconds = 30,
    # Non-default port -- avoids colliding with any other HLDS instance
    # that might already be running on the machine's default 27015.
    [int]$Port = 29015,
    # Console commands to run once the map has finished loading. Written to a
    # generated cfg and exec'd over rcon after readiness -- never from the
    # command line, and never behind "wait". See the note below.
    [string[]]$Commands = @(),
    # Unique config filename (default: _crossbot_autotest.cfg). Use map-specific
    # name to avoid collisions when running parallel tests on different maps.
    [string]$ConfigName = "_crossbot_autotest.cfg",
    [string]$HLDSDir = "D:\hlds",
    # The CrossPlay hyper-game the plugin is deployed into, and the real game
    # DLLs it loads. -xgameN is what CrossPlay iterates as gGameDirs; -sgameN is
    # the engine-side content search path for the same mod.
    [string]$GameDir = "crossplay",
    [string[]]$Games = @("valve"),
    # Per-instance capture of everything the server prints. qconsole.log is
    # shared by every concurrent instance; this file is not.
    [string]$LogFile = "",
    # Extra engine launch arguments, inserted before +map so any cvar they set
    # is already in effect when the level loads. Use for +sys_ticrate and
    # friends when stress-testing the bot at an unusual server framerate.
    [string[]]$ExtraArgs = @(),
    # How long the stdin pipe writer lives. It only has to outlast the server; a run
    # that exceeds it loses console input, not the server. See the launch note below.
    [int]$StdinKeepaliveSeconds = 36000,
    # Refuse to start unless the plugin was built with -DCROSSBOT_VERBOSE=ON.
    # Tests that assert on per-bot narration (reached goal, placed a mine, is
    # shooting) parse lines that compile out when the flag is off; without this
    # guard they would read an empty log as a behavioural failure. Same shape as
    # the crossbot_perf probe in profile_hlds.ps1.
    [switch]$RequireVerbose
)

$exe = Join-Path $HLDSDir "hlds.exe"
if (-not (Test-Path $exe)) {
    "EXITED code=-1 (hlds.exe not found at $exe)"
    exit
}

if (-not $LogFile) {
    $LogFile = Join-Path $HLDSDir "logs\hlds_$Port.log"
}
$logDir = Split-Path $LogFile -Parent
if ($logDir -and -not (Test-Path $logDir)) { New-Item -ItemType Directory -Path $logDir -Force | Out-Null }
$errFile = "$LogFile.err"

Remove-Item $LogFile, $errFile -ErrorAction SilentlyContinue

# cmd.exe runs in $HLDSDir, so its redirection targets have to be absolute -- a caller
# passing a repo-relative -LogFile would otherwise write next to hlds.exe. Rooted paths
# are left alone; test_nav_goal_all.ps1 passes one.
if (-not [System.IO.Path]::IsPathRooted($LogFile)) { $LogFile = Join-Path (Get-Location) $LogFile }
$LogFile = [System.IO.Path]::GetFullPath($LogFile)
$errFile = "$LogFile.err"

# qconsole.log is shared - don't delete it here, let the test orchestrator handle it
# Remove-Item (Join-Path $HLDSDir "qconsole.log") -ErrorAction SilentlyContinue

$addonDir = Join-Path $HLDSDir "$GameDir\addons\crossbot"
$deployed = Join-Path $addonDir "crossbot_mm.dll"
$pluginsIni = Join-Path $HLDSDir "$GameDir\addons\metamod\plugins.ini"

# CrossPlay owns everything else under <gamedir>\; the plugin is only ever loaded
# by a line the operator adds to plugins.ini by hand. Warn, never edit.
if (Test-Path $pluginsIni) {
    if (-not (Select-String -Path $pluginsIni -Pattern "crossbot_mm" -Quiet)) {
        Write-Warning "$pluginsIni has no crossbot_mm.dll line -- the plugin will not load. Add: win32 addons/crossbot/crossbot_mm.dll"
    }
} else {
    Write-Warning "no plugins.ini at $pluginsIni"
}

$built = Join-Path (Split-Path $PSScriptRoot -Parent) "build\crossbot_mm.dll"
if ((Test-Path $built) -and (Test-Path $deployed)) {
    if ((Get-Item $built).LastWriteTime -gt (Get-Item $deployed).LastWriteTime) {
        Write-Warning "$deployed is older than build\crossbot_mm.dll -- rerun build.bat, the server was probably still up."
    }
}

$launchArgs = @(
    "-console", "-condebug", "-insecure", "-game", $GameDir
)
for ($i = 0; $i -lt $Games.Count; $i++) {
    $launchArgs += @("-xgame$($i + 1)", $Games[$i], "-sgame$($i + 1)", $Games[$i])
}
$launchArgs += @(
    "+sv_lan", "1", "+deathmatch", "1", "+developer", "2", "+maxplayers", "16",
    "+rcon_password", "crossbot_test", "+port", "$Port"
)
if ($ExtraArgs.Count -gt 0) { $launchArgs += $ExtraArgs }
$launchArgs += @("+map", $Map)

if ($Commands.Count -gt 0) {
    $cfgPath = Join-Path $HLDSDir "$GameDir\$ConfigName"
    (($Commands -join "`n") + "`n") | Set-Content -Path $cfgPath -NoNewline
}

# hlds.exe must be given a real pipe on stdin. ReHLDS's console loop falls back to
# CTextConsoleWin32::GetPipeLine whenever stdin is not a console, and that calls
# PeekNamedPipe -- which fails on a plain file handle and takes the server down with
# "failed to read STDIN as a pipe". So the process is launched through cmd with a
# writer piped into it, and `waitfor` is that writer: it prints nothing on stdout and
# stays alive holding the write end open. Start-Process -RedirectStandardInput hands
# over a *file*, which is exactly the case that fails.
$quoted = $launchArgs | ForEach-Object { if ($_ -match '\s') { '"' + $_ + '"' } else { $_ } }
$inner = 'waitfor /t {0} cbstdin{1} | "{2}" {3} 1>"{4}" 2>"{5}"' -f `
    ($StdinKeepaliveSeconds, $Port, $exe, ($quoted -join ' '), $LogFile, $errFile)

# -NoNewWindow keeps the server off the screen entirely: hlds.exe is a console-subsystem
# binary, so it is created with no console at all rather than being shown and hidden
# afterwards -- nothing flashes, nothing steals focus, and there is no window to click
# into and freeze the server in QuickEdit select mode.
Start-Process -FilePath "cmd.exe" -ArgumentList @("/c", $inner) `
    -WorkingDirectory $HLDSDir -NoNewWindow | Out-Null

# Readiness is the server answering RCON, not a line in the log. Redirected stdout is
# block-buffered, so the engine's own "N player server started" can sit unflushed for a
# long time after the map is live; RCON answers the moment it is.
$deadline = (Get-Date).AddSeconds($WaitSeconds)
$ready = $false
$serverPid = 0
while ((Get-Date) -lt $deadline) {
    $probe = try { & "$PSScriptRoot\rcon.ps1" -Port $Port -Command "echo crossbot_ready" -FirstTimeoutMs 1000 | Out-String } catch { "" }
    if ($probe -match "crossbot_ready") { $ready = $true; break }
    Start-Sleep -Milliseconds 300
}

# cmd is the process we started; the server is its grandchild, and callers stop it by
# PID. Match on +port so a server someone else is running is never touched.
$found = Get-CimInstance Win32_Process -Filter "Name='hlds.exe'" -ErrorAction SilentlyContinue |
    Where-Object { $_.CommandLine -and $_.CommandLine -match "\+port\s+$Port(\s|$)" }
if ($found) { $serverPid = @($found)[0].ProcessId }

if (-not $serverPid) {
    "EXITED code=-1 (no hlds.exe on port $Port, see $LogFile)"
    exit
}

if ($ready -and $RequireVerbose) {
    $probe = try { & "$PSScriptRoot\rcon.ps1" -Port $Port -Command "crossbot_verbose" | Out-String } catch { "" }
    if ($probe -notmatch 'crossbot_verbose"?\s+is') {
        Stop-Process -Id $serverPid -Force -ErrorAction SilentlyContinue
        "EXITED code=-3 (crossbot_verbose cvar not registered -- is crossbot_mm.dll in plugins.ini, and was it built with -DCROSSBOT_VERBOSE=ON?)"
        exit
    }
}

if ($ready -and $Commands.Count -gt 0) {
    & "$PSScriptRoot\rcon.ps1" -Port $Port -Command "exec $ConfigName" | Out-Null
    Remove-Item $cfgPath -ErrorAction SilentlyContinue
}

if (-not $ready) {
    Stop-Process -Id $serverPid -Force -ErrorAction SilentlyContinue
    "EXITED code=-2 (map '$Map' did not load within ${WaitSeconds}s, see $LogFile)"
} else {
    "PID=$serverPid"
}
