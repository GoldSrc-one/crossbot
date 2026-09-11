param(
    [Parameter(Mandatory=$true)]
    [string]$Map,
    [int]$Port = 29015,
    # Fail the test once elapsed time exceeds this multiple of the bot's
    # own reported ETA (crossbot_nav_status's "eta=" field, itself already
    # a straight-line-distance/speed estimate with a 1.6x safety margin --
    # see kEtaSafetyMargin in bot_move_nav.cpp). 2x on top of that margin is
    # a generous allowance for detours/repaths/jump run-ups, not a tight
    # bound -- the point is catching "never arrives" bots, not shaving
    # seconds off a working route.
    [double]$TimeoutMultiplier = 2.0,
    # Absolute safety cap regardless of ETA, in case the ETA field itself
    # never populates (e.g. pathfinding fails immediately) -- without this
    # a bad run could hang the test indefinitely.
    [int]$MaxWaitSeconds = 240,
    [int]$PollIntervalSeconds = 3,
    # How close the bot's origin must get to the target spawn to count as
    # arrived. Generous relative to the 32-unit waypoint-reached distance
    # used internally -- the target is a raw spawn coordinate, not
    # necessarily a point the nav mesh's own waypoint chain lands exactly
    # on (e.g. it can sit a few units inside a wall/prop clearance zone).
    [double]$ArrivalDistance = 150.0
)

$ErrorActionPreference = "Stop"
$scriptDir = $PSScriptRoot
$hldsDir = "D:\hlds"

. "$scriptDir\common.ps1"

Write-Host "=== crossbot spawn-pair test: $Map ==="

# --- 1. Start the server, listing spawns as part of the boot cfg so we
#        don't need a separate rcon round-trip before the bot exists. ---
$startResult = & "$scriptDir\test_server_start.ps1" -Map $Map -Port $Port `
    -Commands @("crossbot_add valve SpawnTestBot", "crossbot_list_spawns")

$serverPid = Get-CbServerPid $startResult
Write-Host "server started, PID=$serverPid"

# Give the boot cfg (40 waits + our commands) time to actually run.
Start-Sleep -Seconds 10

try {
    # --- 2. Parse spawn points out of qconsole.log. ---
    $log = Get-Content (Join-Path $hldsDir "qconsole.log") -Raw
    $spawns = Get-CbSpawns $log

    if ($spawns.Count -lt 2) {
        Write-Host "FAIL: found only $($spawns.Count) spawn point(s) in qconsole.log -- can't pick a pair"
        exit 1
    }
    Write-Host "found $($spawns.Count) spawn points"

    # --- 3. Pick the two most (straight-line) distant spawns. ---
    $pair = Get-CbFarthestPair $spawns
    $bestA, $bestB, $bestDist = $pair.A, $pair.B, $pair.Dist
    Write-Host ("most distant pair: A=({0},{1},{2}) B=({3},{4},{5}) dist={6:F0}" -f `
        $bestA.X, $bestA.Y, $bestA.Z, $bestB.X, $bestB.Y, $bestB.Z, $bestDist)

    # --- 4. Place the bot at A, send it to B. ---
    Invoke-CbRconQuiet $Port "crossbot_teleport $($bestA.X) $($bestA.Y) $($bestA.Z)"
    Start-Sleep -Milliseconds 500
    Invoke-CbRconQuiet $Port "crossbot_nav_goal $($bestB.X) $($bestB.Y) $($bestB.Z)"

    # --- 5. Poll until arrival, timeout, or the bot abandons this goal. ---
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

        # Goal reassigned away from B (either PickRandomGoal kicked in after
        # a repath failure, or -- less likely, since we'd have caught the
        # arrival above first -- some other reset) -- this attempt is over,
        # one way or the other.
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
