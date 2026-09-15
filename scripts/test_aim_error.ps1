param(
    [string]$Map = "crossfire",
    [int]$Port = 0,
    [float[]]$Scales = @(0.0, 1.0),
    [float[]]$Fovs = @(0.0),
    [string[]]$Shapes = @("180 0", "0 90"),
    [int]$Flicks = 8,
    [float]$Quantum = -1,
    [float]$Impulse = -1,
    [string]$HLDSDir = "D:\hlds"
)

$ErrorActionPreference = 'Stop'
$scriptDir = $PSScriptRoot
$repoRoot = Split-Path $scriptDir -Parent

if ($Port -eq 0) { $Port = 29015 + (Get-Random -Maximum 85) }

$mapName = [System.IO.Path]::GetFileNameWithoutExtension($Map)

. "$scriptDir\common.ps1"

$hldsMap = Join-Path $HLDSDir "valve\maps\$mapName.bsp"
if (-not (Test-Path $hldsMap)) { Write-Host "FAIL: no such map $hldsMap"; exit 1 }

Ensure-CbNav -HldsMap $hldsMap -NavPath (Join-Path $HLDSDir "crossplay\addons\crossbot\nav\$mapName.nav") `
    -NavGenExe (Join-Path $repoRoot "build-x64\tools\navgen\navgen.exe")

$logFile = Join-Path $repoRoot "nav_debug\logs\${mapName}_aimerror.log"

Write-Host "Starting HLDS on port $Port"
$startResult = & "$scriptDir\test_server_start.ps1" -Map $mapName -Port $Port -Commands @() `
    -ConfigName "_crossbot_aimerror_$mapName.cfg" -HLDSDir $HLDSDir -LogFile $logFile

$serverPid = Get-CbServerPid $startResult
Write-Host "Server PID=$serverPid"

try {
    Invoke-CbRcon $Port "crossbot_gather 0" | Out-Null
    Invoke-CbRcon $Port "crossbot_sense 0" | Out-Null
    if ($Quantum -ge 0) { Invoke-CbRcon $Port "crossbot_aim_quantum $Quantum" | Out-Null }
    if ($Impulse -ge 0) { Invoke-CbRcon $Port "crossbot_aim_impulse $Impulse" | Out-Null }
    Invoke-CbRcon $Port "crossbot_add valve Flicker" | Out-Null
    Start-Sleep -Seconds 3

    foreach ($scale in $Scales) {
        Invoke-CbRcon $Port "crossbot_aim_error $scale" | Out-Null

        foreach ($fov in $Fovs) {
            foreach ($shape in $Shapes) {
                Write-Host "`n--- error=$scale fov=$fov flick=($shape) ---"
                $delta = ($shape -split " ")[0]
                $pitch = ($shape -split " ")[1]
                for ($i = 0; $i -lt $Flicks; $i++) {
                    $sign = if ($i % 2 -eq 0) { "" } else { "-" }
                    $cmd = "crossbot_aim_test Flicker $sign$delta $sign$pitch"
                    if ($fov -gt 0) { $cmd = "$cmd $fov" }
                    Invoke-CbRcon $Port $cmd | Out-Null
                    Start-Sleep -Milliseconds 2500
                }
            }
        }
    }

    Invoke-CbRcon $Port "crossbot_aim_error -1" | Out-Null
    Start-Sleep -Seconds 2
}
finally {
    Stop-Process -Id $serverPid -Force -ErrorAction SilentlyContinue
}

Write-Host "`n===== settle and rest error from $logFile ====="
if (-not (Test-Path $logFile)) { Write-Host "FAIL: no server log"; exit 1 }

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
