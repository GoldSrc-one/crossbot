param(
    [Parameter(Mandatory=$true)]
    [string]$Map,
    [int]$Port = 29015,
    [double]$TimeoutMultiplier = 2.0,
    [int]$MaxWaitSeconds = 240,
    [int]$PollIntervalSeconds = 3,
    [double]$ArrivalDistance = 150.0
)

$ErrorActionPreference = "Stop"
$scriptDir = $PSScriptRoot
$hldsDir = "D:\hlds"

. "$scriptDir\common.ps1"

Write-Host "=== crossbot spawn-pair test: $Map ==="

$startResult = & "$scriptDir\test_server_start.ps1" -Map $Map -Port $Port `
    -Commands @("crossbot_add valve SpawnTestBot", "crossbot_list_spawns")

$serverPid = Get-CbServerPid $startResult
Write-Host "server started, PID=$serverPid"

Start-Sleep -Seconds 10

try {
    $log = Get-Content (Join-Path $hldsDir "qconsole.log") -Raw
    $spawns = Get-CbSpawns $log

    if ($spawns.Count -lt 2) {
        Write-Host "FAIL: found only $($spawns.Count) spawn point(s) in qconsole.log -- can't pick a pair"
        exit 1
    }
    Write-Host "found $($spawns.Count) spawn points"

    $pair = Get-CbFarthestPair $spawns
    $bestA, $bestB, $bestDist = $pair.A, $pair.B, $pair.Dist
    Write-Host ("most distant pair: A=({0},{1},{2}) B=({3},{4},{5}) dist={6:F0}" -f `
        $bestA.X, $bestA.Y, $bestA.Z, $bestB.X, $bestB.Y, $bestB.Z, $bestDist)

    Invoke-CbRconQuiet $Port "crossbot_teleport $($bestA.X) $($bestA.Y) $($bestA.Z)"
    Start-Sleep -Milliseconds 500
    Invoke-CbRconQuiet $Port "crossbot_nav_goal $($bestB.X) $($bestB.Y) $($bestB.Z)"

    $expectedEta = $null
    $elapsed = 0
    $result = "FAIL"
    $reason = "unknown"

    while ($elapsed -lt $MaxWaitSeconds) {
        Start-Sleep -Seconds $PollIntervalSeconds
        $elapsed += $PollIntervalSeconds

        $status = Invoke-CbRcon $Port "crossbot_nav_status"
        $statusLine = ($status -split "`n") | Where-Object { $_ -match "CrossBot:.*origin=" } | Select-Object -First 1
        if (-not $statusLine) { continue }

        $originMatch = [regex]::Match($statusLine, "origin=\(([-\d.]+),([-\d.]+),([-\d.]+)\)")
        $goalMatch = [regex]::Match($statusLine, "goal=\(([-\d.]+),([-\d.]+),([-\d.]+)\)")
        $etaMatch = [regex]::Match($statusLine, "eta=([\d.]+)s")
        $hasGoalMatch = [regex]::Match($statusLine, "hasGoal=(\d)")

        if (-not $originMatch.Success) { continue }
        $ox = [double]$originMatch.Groups[1].Value
        $oy = [double]$originMatch.Groups[2].Value
        $oz = [double]$originMatch.Groups[3].Value

        $distToTarget = Get-CbDistance3D $ox $oy $oz $bestB.X $bestB.Y $bestB.Z

        if ($etaMatch.Success -and -not $expectedEta) {
            $expectedEta = [double]$etaMatch.Groups[1].Value
            Write-Host ("bot reports eta={0:N1}s -> failing past {1:N1}s" -f $expectedEta, ($expectedEta * $TimeoutMultiplier))
        }

        Write-Host ("t={0,4}s origin=({1:F0},{2:F0},{3:F0}) distToTarget={4:F0} hasGoal={5}" -f `
            $elapsed, $ox, $oy, $oz, $distToTarget, $hasGoalMatch.Groups[1].Value)

        if ($distToTarget -le $ArrivalDistance) {
            $result = "PASS"
            $reason = ("arrived, distToTarget={0:N0} at t={1}s" -f $distToTarget, $elapsed)
            break
        }

        if ($goalMatch.Success) {
            $gx = [double]$goalMatch.Groups[1].Value
            $gy = [double]$goalMatch.Groups[2].Value
            $gz = [double]$goalMatch.Groups[3].Value
            if ((Get-CbDistance3D $gx $gy $gz $bestB.X $bestB.Y $bestB.Z) -gt 1.0) {
                $reason = "goal was reassigned away from the target (pathfinding likely failed) before arriving"
                break
            }
        }

        if ($expectedEta -and $elapsed -gt ($expectedEta * $TimeoutMultiplier)) {
            $reason = ("exceeded {0:N1}s ({1}x the {2:N1}s ETA) without arriving, distToTarget={3:N0}" -f `
                ($expectedEta * $TimeoutMultiplier), $TimeoutMultiplier, $expectedEta, $distToTarget)
            break
        }
    }

    if ($elapsed -ge $MaxWaitSeconds -and $result -ne "PASS") {
        $reason = "hit the absolute $MaxWaitSeconds s safety cap without arriving"
    }

    Write-Host "=== RESULT: $result -- $reason ==="
}
finally {
    & "$scriptDir\test_server_stop.ps1" -Id $serverPid | Out-Null
}

if ($result -ne "PASS") { exit 1 }
exit 0
