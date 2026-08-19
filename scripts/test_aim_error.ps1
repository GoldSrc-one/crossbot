<#
.SYNOPSIS
    Measures the intentional aim error: fires repeated flicks with crossbot_aim_test
    and reports the spread of settle times at each crossbot_aim_error scale.

.DESCRIPTION
    The brake multiplier is sampled once per flick, so a single flick proves
    nothing -- the number you get is one draw from a distribution. This script
    fires the same flick many times and reports the spread.

    At crossbot_aim_error 0 the spread must collapse to the phase-1 numbers: the
    brake is forced to exactly 1 and the controller is the time-optimal one again.
    That is the regression check. At scale 1 the settle times must spread out,
    and the slow tail is the overshoot-plus-correction case.

    Settle is reported on the first frame within kTestSettleTolerance *and* under
    kTestSettleSpeed, so a view flying through the target on its way to an
    overshoot does not report a false early settle. A small enough overshoot can
    still slip under that speed gate, so read the spread, not any single row.

    crossbot_aim_drift is left at its default: micro-drift is suppressed while a
    test is active, and this run is what proves it.

.PARAMETER Scales
    crossbot_aim_error values to test (default: 0, 1).

.PARAMETER Flicks
    Flicks fired per scale per shape (default: 8).
#>
param(
    [string]$Map = "crossfire",
    [int]$Port = 0,
    [float[]]$Scales = @(0.0, 1.0),
    [int]$Flicks = 8,
    [string]$HLDSDir = "D:\projects\trash\crossbot\hlds"
)

$ErrorActionPreference = 'Stop'
$scriptDir = $PSScriptRoot
$repoRoot = Split-Path $scriptDir -Parent

if ($Port -eq 0) { $Port = 29015 + (Get-Random -Maximum 85) }

$mapName = [System.IO.Path]::GetFileNameWithoutExtension($Map)

function Invoke-Rcon {
    param([string]$Cmd)
    $out = try { & "$scriptDir\rcon.ps1" -Port $Port -Command $Cmd } catch { "" }
    if ($null -eq $out) { return "" }
    return ($out | Out-String)
}

$hldsMap = Join-Path $HLDSDir "valve\maps\$mapName.bsp"
if (-not (Test-Path $hldsMap)) { Write-Host "FAIL: no such map $hldsMap"; exit 1 }

$navDir = Join-Path $HLDSDir "valve\addons\crossbot\nav"
if (-not (Test-Path $navDir)) { New-Item -ItemType Directory -Path $navDir | Out-Null }
if (-not (Test-Path (Join-Path $navDir "$mapName.nav"))) {
    & (Join-Path $repoRoot "build\tools\navgen\navgen.exe") $hldsMap -o (Join-Path $navDir "$mapName.nav") 2>&1 |
        Select-Object -Last 3 | ForEach-Object { Write-Host "navgen: $_" }
}

$logFile = Join-Path $repoRoot "nav_debug\logs\${mapName}_aimerror.log"

Write-Host "Starting HLDS on port $Port"
$startResult = & "$scriptDir\test_server_start.ps1" -Map $mapName -Port $Port -Commands @() `
    -ConfigName "_crossbot_aimerror_$mapName.cfg" -HLDSDir $HLDSDir -LogFile $logFile

if ($startResult -match "^EXITED") { Write-Host "FAIL: $startResult"; exit 1 }
$serverPid = [int]($startResult -replace "PID=", "")
Write-Host "Server PID=$serverPid"

try {
    Invoke-Rcon "crossbot_gather 0" | Out-Null
    Invoke-Rcon "crossbot_sense 0" | Out-Null
    Invoke-Rcon "crossbot_add Flicker" | Out-Null
    Start-Sleep -Seconds 3

    foreach ($scale in $Scales) {
        Invoke-Rcon "crossbot_aim_error $scale" | Out-Null

        foreach ($shape in @("180 0", "0 90")) {
            Write-Host "`n--- error=$scale flick=($shape) ---"
            $delta = ($shape -split " ")[0]
            $pitch = ($shape -split " ")[1]
            for ($i = 0; $i -lt $Flicks; $i++) {
                $sign = if ($i % 2 -eq 0) { "" } else { "-" }
                Invoke-Rcon "crossbot_aim_test Flicker $sign$delta $sign$pitch" | Out-Null
                Start-Sleep -Milliseconds 2000
            }
        }
    }

    Start-Sleep -Seconds 2
}
finally {
    Stop-Process -Id $serverPid -Force -ErrorAction SilentlyContinue
}

Write-Host "`n===== settle times from $logFile ====="
if (-not (Test-Path $logFile)) { Write-Host "FAIL: no server log"; exit 1 }

$lines = Get-Content $logFile | Where-Object { $_ -match 'aim test settled in ([\d.]+)s, peak ([\d.]+)' }
if (-not $lines) { Write-Host "FAIL: no settle lines in the log -- did the bot spawn?"; exit 1 }

$idx = 0
$perGroup = $Flicks
foreach ($scale in $Scales) {
    foreach ($shape in @("180 0", "0 90")) {
        $group = $lines | Select-Object -Skip $idx -First $perGroup
        $idx += $perGroup
        if (-not $group) { continue }

        $settle = @()
        $peak = @()
        foreach ($line in $group) {
            if ($line -match 'settled in ([\d.]+)s, peak ([\d.]+)') {
                $settle += [double]$Matches[1]
                $peak += [double]$Matches[2]
            }
        }
        if (-not $settle) { continue }

        $min = ($settle | Measure-Object -Minimum).Minimum
        $max = ($settle | Measure-Object -Maximum).Maximum
        $avg = ($settle | Measure-Object -Average).Average
        $pavg = ($peak | Measure-Object -Average).Average
        Write-Host ("error={0,-4} flick=({1,-5}) n={2}  settle min {3:F3} avg {4:F3} max {5:F3}  spread {6:F3}s  peak avg {7:F0} deg/s" -f `
            $scale, $shape, $settle.Count, $min, $avg, $max, ($max - $min), $pavg)
    }
}

Write-Host "`nServer log: $logFile"
