param(
    [Parameter(Mandatory=$true)]
    [string]$Map,
    [int]$Port = 29015,
    # Same reasoning as nav_spawn_test.ps1's identical parameter: 2x on top
    # of Bot_MoveNav's own kEtaSafetyMargin is a generous allowance for
    # detours/repaths/jump run-ups, not a tight bound.
    [double]$TimeoutMultiplier = 2.0,
    [int]$MaxWaitSeconds = 240,
    [int]$PollIntervalSeconds = 3,
    [double]$ArrivalDistance = 150.0
)

$ErrorActionPreference = "Stop"
$scriptDir = $PSScriptRoot
$hldsDir = "D:\hlds"

. "$scriptDir\common.ps1"

Write-Host "=== crossbot single-target test: $Map ==="

# --- 1. Start the server, listing the spawn and target as part of the boot
#        cfg so we don't need a separate rcon round-trip before the bot
#        exists (same reasoning as nav_spawn_test.ps1). ---
$startResult = & "$scriptDir\test_server_start.ps1" -Map $Map -Port $Port -RequireVerbose `
    -Commands @("crossbot_add valve TargetTestBot", "crossbot_list_spawns", "crossbot_list_targets")

$serverPid = Get-CbServerPid $startResult
Write-Host "server started, PID=$serverPid"

# Give the boot cfg (40 waits + our commands) time to actually run.
Start-Sleep -Seconds 10

try {
    # --- 2. Parse the spawn point and the info_target out of qconsole.log. ---
    $log = Get-Content (Join-Path $hldsDir "qconsole.log") -Raw
    $spawns = Get-CbSpawns $log
    $targetMatches = [regex]::Matches($log, "CrossBot: target (\S+) \(([-\d.]+),([-\d.]+),([-\d.]+)\)")

    if ($spawns.Count -lt 1) {
        Write-Host "FAIL: found no spawn point in qconsole.log"
        exit 1
    }
    if ($targetMatches.Count -lt 1) {
        Write-Host "FAIL: found no info_target in qconsole.log"
        exit 1
    }

    $spawn = $spawns[0]
    $spawnX, $spawnY, $spawnZ = $spawn.X, $spawn.Y, $spawn.Z

    $target = $targetMatches[0]
    $targetX = [double]$target.Groups[2].Value
    $targetY = [double]$target.Groups[3].Value
    $targetZ = [double]$target.Groups[4].Value

    Write-Host ("spawn=({0},{1},{2}) target=({3},{4},{5})" -f `
        $spawnX, $spawnY, $spawnZ, $targetX, $targetY, $targetZ)

    # --- 3. Make sure the bot starts from the actual spawn point (it should
    #        already be there from natural spawn, but teleporting is cheap
    #        insurance against picking up leftover state from a prior run),
    #        then send it to the target. ---
    Invoke-CbRconQuiet $Port "crossbot_teleport $spawnX $spawnY $spawnZ"
    Start-Sleep -Milliseconds 500
    Invoke-CbRconQuiet $Port "crossbot_nav_goal $targetX $targetY $targetZ"

    # --- 4. Poll until arrival, timeout, or the bot abandons this goal.
    #        Primary signal is bot_move_nav.cpp's permanent "reached goal"
    #        log line, matched against the target's exact coordinates --
    #        NOT the live "goal=" field from crossbot_nav_status, which is
    #        racy: on a short path the bot can arrive and auto-pick a brand
    #        new wander goal well within one poll interval, so by the time
    #        we poll, "goal=" may already show the *next* destination and
    #        never the target we actually asked for. The live status poll
    #        is still useful for progress visibility and timeout/stuck
    #        detection, just not as the pass/fail source of truth. ---
    $expectedEta = $null
    $elapsed = 0
    $result = "FAIL"
    $reason = "unknown"
    $reachedPattern = ("CrossBot: \S+ reached goal \(\s*{0}\s*,\s*{1}\s*,\s*{2}\s*\)" -f `
        [regex]::Escape(("{0:F1}" -f $targetX)), [regex]::Escape(("{0:F1}" -f $targetY)), [regex]::Escape(("{0:F1}" -f $targetZ)))

    while ($elapsed -lt $MaxWaitSeconds) {
        Start-Sleep -Seconds $PollIntervalSeconds
        $elapsed += $PollIntervalSeconds

        $log = Get-Content (Join-Path $hldsDir "qconsole.log") -Raw
        if ($log -match $reachedPattern) {
            $result = "PASS"
            $reason = ("bot logged arrival at the target, t={0}s" -f $elapsed)
            break
        }

        $status = Invoke-CbRcon $Port "crossbot_nav_status"
        $statusLine = ($status -split "`n") | Where-Object { $_ -match "CrossBot:.*origin=" } | Select-Object -First 1
        if (-not $statusLine) { continue }

        $originMatch = [regex]::Match($statusLine, "origin=\(([-\d.]+),([-\d.]+),([-\d.]+)\)")
        $etaMatch = [regex]::Match($statusLine, "eta=([\d.]+)s")
        $hasGoalMatch = [regex]::Match($statusLine, "hasGoal=(\d)")

        if (-not $originMatch.Success) { continue }
        $ox = [double]$originMatch.Groups[1].Value
        $oy = [double]$originMatch.Groups[2].Value
        $oz = [double]$originMatch.Groups[3].Value

        $distToTarget = Get-CbDistance3D $ox $oy $oz $targetX $targetY $targetZ

        if ($etaMatch.Success -and -not $expectedEta) {
            $expectedEta = [double]$etaMatch.Groups[1].Value
            Write-Host ("bot reports eta={0:N1}s -> failing past {1:N1}s" -f $expectedEta, ($expectedEta * $TimeoutMultiplier))
        }

        Write-Host ("t={0,4}s origin=({1:F0},{2:F0},{3:F0}) distToTarget={4:F0} hasGoal={5}" -f `
            $elapsed, $ox, $oy, $oz, $distToTarget, $hasGoalMatch.Groups[1].Value)

        # Fallback in case the log-line match above has a formatting mismatch
        # (e.g. an unexpected float rounding) -- still catch a genuine
        # close-enough arrival via live distance, same as nav_spawn_test.ps1.
        if ($distToTarget -le $ArrivalDistance) {
            $result = "PASS"
            $reason = ("arrived, distToTarget={0:N0} at t={1}s" -f $distToTarget, $elapsed)
            break
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
