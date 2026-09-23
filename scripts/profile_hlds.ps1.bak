param(
    [string]$Map = "crossfire",
    [int]$Bots = 32,
    [int]$Port = 29020,
    [int]$WarmupSeconds = 25,
    [int]$ProfileSeconds = 60,
    [double]$PerfInterval = 10,
    [double]$SpikeMs = 3,
    [string]$OutDir = "",
    [string]$HLDSDir = "D:\hlds",
    [string]$UProf = "D:\Program Files\AMD\AMDuProf\bin\AMDuProfCLI.exe",
    [switch]$NoProfiler,
    [switch]$NoConDebug,
    [switch]$NullStdout,
    [string[]]$ExtraCommands = @()
)

$ErrorActionPreference = "Stop"
$scriptDir = $PSScriptRoot
$repoRoot = Split-Path $scriptDir -Parent

. "$scriptDir\common.ps1"

if (-not $OutDir) {
    $OutDir = Join-Path $repoRoot "nav_debug\perf"
}
if (-not (Test-Path $OutDir)) { New-Item -ItemType Directory -Path $OutDir -Force | Out-Null }

$stamp = Get-Date -Format "yyyyMMdd_HHmmss"
$logFile = Join-Path $OutDir "$($Map)_$stamp.log"
$sessionDir = Join-Path $OutDir "uprof_$($Map)_$stamp"

$exe = Join-Path $HLDSDir "hlds.exe"
if (-not (Test-Path $exe)) { throw "hlds.exe not found at $exe" }

$errFile = "$logFile.err"

if (-not [System.IO.Path]::IsPathRooted($logFile)) { $logFile = Join-Path (Get-Location) $logFile }
$logFile = [System.IO.Path]::GetFullPath($logFile)
$errFile = "$logFile.err"

$launchArgs = @("-console")
if (-not $NoConDebug) { $launchArgs += "-condebug" }
$launchArgs += @(
    "-insecure", "-game", "crossplay", "-xgame1", "valve", "-sgame1", "valve",
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
$quoted = $launchArgs | ForEach-Object { if ($_ -match '\s') { '"' + $_ + '"' } else { $_ } }
$inner = 'waitfor /t 36000 cbstdin{0} | "{1}" {2} 1>"{3}" 2>"{4}"' -f `
    ($Port, $exe, ($quoted -join ' '), $stdoutTarget, $errFile)
Start-Process -FilePath "cmd.exe" -ArgumentList @("/c", $inner) `
    -WorkingDirectory $HLDSDir -NoNewWindow | Out-Null

$deadline = (Get-Date).AddSeconds(60)
$ready = $false
while ((Get-Date) -lt $deadline) {
    $probe = & "$PSScriptRoot\rcon.ps1" -Port $Port -Command "echo crossbot_ready" -FirstTimeoutMs 1000 | Out-String
    if ($probe -match "crossbot_ready") { $ready = $true; break }
    Start-Sleep -Milliseconds 250
}

$found = Get-CimInstance Win32_Process -Filter "Name='hlds.exe'" -ErrorAction SilentlyContinue |
    Where-Object { $_.CommandLine -and $_.CommandLine -match "\+port\s+$Port(\s|$)" }
$proc = if ($found) { Get-Process -Id (@($found)[0].ProcessId) -ErrorAction SilentlyContinue } else { $null }

if (-not $ready -or -not $proc) {
    if ($proc) { Stop-Process -Id $proc.Id -Force }
    throw "server did not come up; see $logFile"
}
Write-Host "profile_hlds: server up, pid $($proc.Id)"

$probe = Invoke-CbRcon $Port "crossbot_perf"
if ($probe -notmatch 'crossbot_perf"?\s+is') {
    if (-not $proc.HasExited) { Stop-Process -Id $proc.Id -Force }
    throw "crossbot_perf cvar not registered -- rebuild with: build.bat -DCROSSBOT_PERF=ON"
}

Invoke-CbRconQuiet $Port "crossbot_perf 0"
foreach ($c in $ExtraCommands) { Invoke-CbRconQuiet $Port $c }

for ($i = 1; $i -le $Bots; $i++) {
    Invoke-CbRconQuiet $Port "crossbot_add valve PerfBot$i"
}
Write-Host "profile_hlds: requested $Bots bots, warming up ${WarmupSeconds}s"
Start-Sleep -Seconds $WarmupSeconds

Invoke-CbRconQuiet $Port "crossbot_perf_spike $SpikeMs"
Invoke-CbRconQuiet $Port "crossbot_perf $PerfInterval"

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

Invoke-CbRconQuiet $Port "crossbot_perf 0"
Start-Sleep -Seconds 1

$cacheStats = Invoke-CbRcon $Port "crossbot_airmove_cache_stats"
if ($cacheStats) { Write-Host $cacheStats.Trim() }

Write-Host "profile_hlds: stopping server"
if (-not $proc.HasExited) { Stop-Process -Id $proc.Id -Force }
Start-Sleep -Seconds 2

Write-Host "profile_hlds: server log   $logFile"
Write-Host "profile_hlds: uprof session $sessionDir"
