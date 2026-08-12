param(
    [string]$Map = "crossfire",
    [int]$WaitSeconds = 30,
    # Non-default port -- avoids colliding with any other HLDS instance
    # that might already be running on the machine's default 27015.
    [int]$Port = 29015,
    # Console commands to run once the map has finished loading. Implemented
    # via a generated cfg file exec'd after +map, with a "wait" buffer first
    # -- game.cfg/server.cfg run too early (before the level is active), and
    # a per-map cfg wasn't observed to run at all on this dedicated server.
    [string[]]$Commands = @(),
    # Unique config filename (default: _crossbot_autotest.cfg). Use map-specific
    # name to avoid collisions when running parallel tests on different maps.
    [string]$ConfigName = "_crossbot_autotest.cfg",
    [string]$HLDSDir = "D:\projects\trash\crossbot\hlds",
    # Per-instance capture of everything the server prints. qconsole.log is
    # shared by every concurrent instance; this file is not.
    [string]$LogFile = ""
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
$inFile = "$LogFile.in"

Remove-Item $LogFile, $errFile -ErrorAction SilentlyContinue
Set-Content -Path $inFile -Value "" -NoNewline

# qconsole.log is shared - don't delete it here, let the test orchestrator handle it
# Remove-Item (Join-Path $HLDSDir "qconsole.log") -ErrorAction SilentlyContinue

$launchArgs = @(
    "-console", "-condebug", "-insecure", "-game", "valve",
    "+sv_lan", "1", "+deathmatch", "1", "+developer", "2", "+maxplayers", "16",
    "+rcon_password", "crossbot_test", "+port", "$Port", "+map", $Map
)

if ($Commands.Count -gt 0) {
    $cfgPath = Join-Path $HLDSDir "valve\$ConfigName"
    $waits = (@("wait") * 40) -join "`n"
    ($waits + "`n" + ($Commands -join "`n") + "`n") | Set-Content -Path $cfgPath -NoNewline
    $launchArgs += @("+exec", $ConfigName)
}

# -NoNewWindow plus redirection is what keeps the server off the screen entirely.
# hlds.exe is a console-subsystem binary, so it is created with no console at all
# rather than being shown and hidden afterwards -- there is no window to flash, to
# steal focus, or to freeze the server by clicking into its QuickEdit select mode.
$proc = Start-Process -FilePath $exe -ArgumentList $launchArgs `
    -WorkingDirectory $HLDSDir -NoNewWindow -PassThru `
    -RedirectStandardOutput $LogFile -RedirectStandardError $errFile -RedirectStandardInput $inFile

# Readiness is the engine announcing the level is live, not a window handle.
$deadline = (Get-Date).AddSeconds($WaitSeconds)
$ready = $false
while ((Get-Date) -lt $deadline) {
    $proc.Refresh()
    if ($proc.HasExited) { break }
    if (Test-Path $LogFile) {
        $head = Get-Content $LogFile -Raw -ErrorAction SilentlyContinue
        if ($head -match "player server started") { $ready = $true; break }
    }
    Start-Sleep -Milliseconds 200
}

$proc.Refresh()
if ($proc.HasExited) {
    "EXITED code=$($proc.ExitCode)"
} elseif (-not $ready) {
    Stop-Process -Id $proc.Id -Force -ErrorAction SilentlyContinue
    "EXITED code=-2 (map '$Map' did not load within ${WaitSeconds}s, see $LogFile)"
} else {
    "PID=$($proc.Id)"
}
