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

    Each flick also reports a rest error: the angular miss still standing two seconds
    after the flick began, once everything has stopped moving. That is the number the
    hand quantum sets, and it is the one that should scale with FOV -- a zoomed bot
    turns 4.5x slower but places the crosshair 4.5x finer. Sweep -Fovs to see it.

.PARAMETER Scales
    crossbot_aim_error values to test (default: 0, 1).

.PARAMETER Fovs
    pev->fov values to force for each flick. 0 leaves the bot's fov alone (default).
    Use -Fovs 90,20 with a small -Shapes to measure what zoom buys.

.PARAMETER Shapes
    "<yaw> <pitch>" flick deltas (default: "180 0", "0 90"). At fov 20 a 40 degree yaw
    flick is the same 180 degrees of hand travel as a 180 at fov 90, so use "40 0"
    there rather than letting a 180 run past the two second rest sample.

.PARAMETER Flicks
    Flicks fired per scale per fov per shape (default: 8).

.PARAMETER Quantum
.PARAMETER Impulse
    Override crossbot_aim_quantum / crossbot_aim_impulse. Negative leaves the shipped
    default alone. Pass -Quantum 0 -Impulse 0 for the phase-4 regression check: with the
    hand resolution off the settle distribution must collapse back onto 0.349s / 895 deg/s
    for a 180 and 0.266s / 563 for 90 of pitch.
#>
param(
    [string]$Map = "crossfire",
    [int]$Port = 0,
    [float[]]$Scales = @(0.0, 1.0),
    [float[]]$Fovs = @(0.0),
    [string[]]$Shapes = @("180 0", "0 90"),
    [int]$Flicks = 8,
    [float]$Quantum = -1,
    [float]$Impulse = -1,
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
    if ($Quantum -ge 0) { Invoke-Rcon "crossbot_aim_quantum $Quantum" | Out-Null }
    if ($Impulse -ge 0) { Invoke-Rcon "crossbot_aim_impulse $Impulse" | Out-Null }
    Invoke-Rcon "crossbot_add Flicker" | Out-Null
    Start-Sleep -Seconds 3

    foreach ($scale in $Scales) {
        Invoke-Rcon "crossbot_aim_error $scale" | Out-Null

        foreach ($fov in $Fovs) {
            foreach ($shape in $Shapes) {
                Write-Host "`n--- error=$scale fov=$fov flick=($shape) ---"
                $delta = ($shape -split " ")[0]
                $pitch = ($shape -split " ")[1]
                for ($i = 0; $i -lt $Flicks; $i++) {
                    $sign = if ($i % 2 -eq 0) { "" } else { "-" }
                    $cmd = "crossbot_aim_test Flicker $sign$delta $sign$pitch"
                    if ($fov -gt 0) { $cmd = "$cmd $fov" }
                    Invoke-Rcon $cmd | Out-Null
                    Start-Sleep -Milliseconds 2500
                }
            }
        }
    }

    Start-Sleep -Seconds 2
}
finally {
    Stop-Process -Id $serverPid -Force -ErrorAction SilentlyContinue
}

Write-Host "`n===== settle and rest error from $logFile ====="
if (-not (Test-Path $logFile)) { Write-Host "FAIL: no server log"; exit 1 }

# Group by the announce line rather than by slicing the settle lines: a flick that never
# settles prints no settle line, and slicing would then silently shift every later group.
$flicksSeen = @()
$cur = $null
foreach ($line in Get-Content $logFile) {
    if ($line -match 'aim test yaw') {
        if ($cur) { $flicksSeen += ,$cur }
        $cur = @{ settle = $null; peak = $null; rest = $null }
        continue
    }
    if (-not $cur) { continue }
    if ($line -match 'settled in ([\d.]+)s, peak ([\d.]+)') {
        $cur.settle = [double]$Matches[1]
        $cur.peak = [double]$Matches[2]
    }
    elseif ($line -match 'rest err ([\d.]+) deg') {
        $cur.rest = [double]$Matches[1]
    }
}
if ($cur) { $flicksSeen += ,$cur }

if (-not $flicksSeen) { Write-Host "FAIL: no flicks in the log -- did the bot spawn?"; exit 1 }

$idx = 0
foreach ($scale in $Scales) {
    foreach ($fov in $Fovs) {
        foreach ($shape in $Shapes) {
            $group = @($flicksSeen | Select-Object -Skip $idx -First $Flicks)
            $idx += $Flicks
            if (-not $group) { continue }

            $settle = @($group | Where-Object { $null -ne $_.settle } | ForEach-Object { $_.settle })
            $peak = @($group | Where-Object { $null -ne $_.peak } | ForEach-Object { $_.peak })
            $rest = @($group | Where-Object { $null -ne $_.rest } | ForEach-Object { $_.rest })

            $fovLabel = if ($fov -gt 0) { "$fov" } else { "-" }

            if ($settle) {
                $min = ($settle | Measure-Object -Minimum).Minimum
                $max = ($settle | Measure-Object -Maximum).Maximum
                $avg = ($settle | Measure-Object -Average).Average
                $pavg = ($peak | Measure-Object -Average).Average
                Write-Host ("error={0,-4} fov={1,-4} flick=({2,-6}) n={3}/{4}  settle min {5:F3} avg {6:F3} max {7:F3}  spread {8:F3}s  peak avg {9:F0} deg/s" -f `
                    $scale, $fovLabel, $shape, $settle.Count, $group.Count, $min, $avg, $max, ($max - $min), $pavg)
            }
            else {
                Write-Host ("error={0,-4} fov={1,-4} flick=({2,-6}) n=0/{3}  NO SETTLE -- the view never came within tolerance" -f `
                    $scale, $fovLabel, $shape, $group.Count)
            }

            if ($rest) {
                $rmin = ($rest | Measure-Object -Minimum).Minimum
                $rmax = ($rest | Measure-Object -Maximum).Maximum
                $ravg = ($rest | Measure-Object -Average).Average
                Write-Host ("                                   rest err min {0:F3} avg {1:F3} max {2:F3} deg" -f `
                    $rmin, $ravg, $rmax)
            }
        }
    }
}

Write-Host "`nServer log: $logFile"
