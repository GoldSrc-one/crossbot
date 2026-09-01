param(
    [string]$Map = "crossfire",
    [int]$Bots = 32,
    [int]$Port = 29020,
    [int]$WarmupSeconds = 25,
    [int]$ProfileSeconds = 60,
    [double]$PerfInterval = 10,
    [double]$SpikeMs = 3,
    [string]$OutDir = "",
    [string]$HLDSDir = "D:\projects\trash\crossbot\hlds",
    [string]$UProf = "D:\Program Files\AMD\AMDuProf\bin\AMDuProfCLI.exe",
    [switch]$NoProfiler,
    [switch]$NoConDebug,
    [switch]$NullStdout,
    [string[]]$ExtraCommands = @()
)

$ErrorActionPreference = "Stop"
$scriptDir = $PSScriptRoot
$repoRoot = Split-Path $scriptDir -Parent

if (-not $OutDir) {
    $OutDir = Join-Path $repoRoot "nav_debug\perf"
}
if (-not (Test-Path $OutDir)) { New-Item -ItemType Directory -Path $OutDir -Force | Out-Null }

$stamp = Get-Date -Format "yyyyMMdd_HHmmss"
$logFile = Join-Path $OutDir "$($Map)_$stamp.log"
$sessionDir = Join-Path $OutDir "uprof_$($Map)_$stamp"

$exe = Join-Path $HLDSDir "hlds.exe"
if (-not (Test-Path $exe)) { throw "hlds.exe not found at $exe" }

$inFile = "$logFile.in"
$errFile = "$logFile.err"
Set-Content -Path $inFile -Value "" -NoNewline

$launchArgs = @("-console")
if (-not $NoConDebug) { $launchArgs += "-condebug" }
$launchArgs += @(
    "-insecure", "-game", "valve",
    "+sv_lan", "1", "+deathmatch", "1", "+developer", "1",
    "+maxplayers", "32",
    "+rcon_password", "crossbot_test", "+port", "$Port",
    "+map", $Map
)

Write-Host "profile_hlds: starting $Map on port $Port (maxplayers 32)"
$stdoutTarget = $logFile
if ($NullStdout) {
    $stdoutTarget = "NUL"
    Remove-Item (Join-Path $HLDSDir "qconsole.log") -ErrorAction SilentlyContinue
}
$proc = Start-Process -FilePath $exe -ArgumentList $launchArgs `
    -WorkingDirectory $HLDSDir -NoNewWindow -PassThru `
    -RedirectStandardOutput $stdoutTarget -RedirectStandardError $errFile -RedirectStandardInput $inFile

$deadline = (Get-Date).AddSeconds(60)
$ready = $false
while ((Get-Date) -lt $deadline) {
    $proc.Refresh()
    if ($proc.HasExited) { break }
    $probe = if ($NullStdout) { Join-Path $HLDSDir "qconsole.log" } else { $logFile }
    $head = Get-Content $probe -Raw -ErrorAction SilentlyContinue
    if ($head -match "player server started") { $ready = $true; break }
    Start-Sleep -Milliseconds 250
}
if (-not $ready) {
    if (-not $proc.HasExited) { Stop-Process -Id $proc.Id -Force }
    throw "server did not come up; see $logFile"
}
Write-Host "profile_hlds: server up, pid $($proc.Id)"

function Rcon([string]$cmd) {
    try { & "$scriptDir\rcon.ps1" -Port $Port -Command $cmd | Out-Null } catch { }
}

function RconRead([string]$cmd) {
    try { return (& "$scriptDir\rcon.ps1" -Port $Port -Command $cmd | Out-String) } catch { return "" }
}

# The zone profiler and its cvars are behind CROSSBOT_PERF, off by default. Against a stock build
# this run would finish and simply print nothing, which reads as "no spikes" rather than "not
# measured", so refuse to start instead.
$probe = RconRead "crossbot_perf"
if ($probe -notmatch 'crossbot_perf"?\s+is') {
    if (-not $proc.HasExited) { Stop-Process -Id $proc.Id -Force }
    throw "crossbot_perf cvar not registered -- rebuild with: build.bat -DCROSSBOT_PERF=ON"
}

Rcon "crossbot_perf 0"
foreach ($c in $ExtraCommands) { Rcon $c }

for ($i = 1; $i -le $Bots; $i++) {
    Rcon "crossbot_add PerfBot$i"
}
Write-Host "profile_hlds: requested $Bots bots, warming up ${WarmupSeconds}s"
Start-Sleep -Seconds $WarmupSeconds

Rcon "crossbot_perf_spike $SpikeMs"
Rcon "crossbot_perf $PerfInterval"

if (-not $NoProfiler) {
    Write-Host "profile_hlds: collecting ${ProfileSeconds}s with AMDuProf -> $sessionDir"
    $uprofArgs = @(
        "collect", "--config", "tbp", "--timer-interval", "1",
        "--call-graph-mode", "fpo", "--call-graph-type", "user",
        "--call-graph-depth", "64", "--call-graph-interval", "1",
        "-p", "$($proc.Id)", "-d", "$ProfileSeconds",
        "-o", $sessionDir
    )
    & $UProf @uprofArgs 2>&1 | Tee-Object -FilePath (Join-Path $OutDir "uprof_$($Map)_$stamp.out")
} else {
    Start-Sleep -Seconds $ProfileSeconds
}

Rcon "crossbot_perf 0"
Start-Sleep -Seconds 1

$cacheStats = RconRead "crossbot_airmove_cache_stats"
if ($cacheStats) { Write-Host $cacheStats.Trim() }

Write-Host "profile_hlds: stopping server"
if (-not $proc.HasExited) { Stop-Process -Id $proc.Id -Force }
Start-Sleep -Seconds 2

Write-Host "profile_hlds: server log   $logFile"
Write-Host "profile_hlds: uprof session $sessionDir"
