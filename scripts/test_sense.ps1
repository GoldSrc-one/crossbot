<#
.SYNOPSIS
    Manual driver for the sense system and the aim delay queue: boots HLDS, adds
    bots that can see each other, and polls crossbot_aim_status and
    crossbot_sense_status while stepping crossbot_aim_delay.

.DESCRIPTION
    The nav sweep cannot exercise any of this -- it spawns one bot, and with
    LOS-gated vision a lone bot never has a target and hears almost nothing.
    This script is the only automated way to see the queue and the suspicion
    list run.

    Two bots left to roam a real map is not enough either: they chase items
    independently and are almost never in each other's line of sight, so the
    queue sits empty. So the run has three phases.

    STATIC: gather off, no goals, all bots sent to one spawn by nav goal. They stand
    together, hold line of sight continuously, and the target is stationary --
    which is the case the design says must be exact at *any* delay, because
    extrapolating a still target forward and reading it back cancels perfectly.
    err should be small and flat across delays. It no longer goes to zero: the
    aim point is a randomly chosen hitbox bone each frame while err is measured
    against the target's eye, so the bone spread is a standing floor of a couple
    of degrees. A *trend* across delays is still a real defect.

    ROAM: gather back on, bots move. err is now allowed to be nonzero -- that is
    target acceleration, which is the whole point -- but the queue should be full
    and served should equal the requested delay whenever a target is held.

    SUSPICION: which senses actually fire, which tier wins the vote, and whether
    heard entries age out. Only SIGHT and SOUND are reachable here. SOUND is
    dominated by footsteps, which pmove emits through pmove->PM_PlaySound rather
    than pfnEmitSound and which the plugin picks up by hooking pfnPM_Move; item
    pickups, doors and chargers make up the rest. EVENT and DAMAGE both need
    weapon fire, which no bot does yet.

    Sources are read from the pushed= counters, not by sampling fresh=. The list
    holds one entry per entity, so a player's SIGHT entry overwrites their own
    SOUND entry within the same frame and fresh= would miss most of what fired.

.PARAMETER Delays
    crossbot_aim_delay values to walk through in the static phase (default: 0, 0.25, 0.5).
#>
param(
    [string]$Map = "crossfire",
    [int]$Port = 0,
    [float[]]$Delays = @(0.0, 0.25, 0.5),
    [int]$SecondsPerDelay = 10,
    [int]$RoamSeconds = 30,
    [int]$SuspicionSeconds = 30,
    [int]$Bots = 2,
    [switch]$SkipRoam,
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

function Show-AimStatus {
    (Invoke-Rcon "crossbot_aim_status") -split "`r?`n" |
        Where-Object { $_ -match 'target=|bot\(s\)' } |
        ForEach-Object { Write-Host ("  " + ($_ -replace '^CrossBot: ', '')) }
}

$hldsMap = Join-Path $HLDSDir "valve\maps\$mapName.bsp"
if (-not (Test-Path $hldsMap)) { Write-Host "FAIL: no such map $hldsMap"; exit 1 }

$navDir = Join-Path $HLDSDir "valve\addons\crossbot\nav"
if (-not (Test-Path $navDir)) { New-Item -ItemType Directory -Path $navDir | Out-Null }
if (-not (Test-Path (Join-Path $navDir "$mapName.nav"))) {
    & (Join-Path $repoRoot "build\tools\navgen\navgen.exe") $hldsMap -o (Join-Path $navDir "$mapName.nav") 2>&1 |
        Select-Object -Last 3 | ForEach-Object { Write-Host "navgen: $_" }
}

$logFile = Join-Path $repoRoot "nav_debug\logs\${mapName}_sense.log"

Write-Host "Starting HLDS on port $Port (join with: connect 127.0.0.1:$Port)"
$startResult = & "$scriptDir\test_server_start.ps1" -Map $mapName -Port $Port -Commands @() `
    -ConfigName "_crossbot_sense_$mapName.cfg" -HLDSDir $HLDSDir -LogFile $logFile

if ($startResult -match "^EXITED") { Write-Host "FAIL: $startResult"; exit 1 }
$serverPid = [int]($startResult -replace "PID=", "")
Write-Host "Server PID=$serverPid"

try {
    $failed = $false
    Invoke-Rcon "crossbot_gather 0" | Out-Null
    for ($i = 1; $i -le $Bots; $i++) { Invoke-Rcon "crossbot_add Sense$i" | Out-Null }
    Invoke-Rcon "crossbot_sense 1" | Out-Null

    $spawnOut = Invoke-Rcon "crossbot_list_spawns"
    $spawn = $null
    foreach ($line in ($spawnOut -split "`r?`n")) {
        if ($line -match 'spawn \S+ \((-?[\d.]+),(-?[\d.]+),(-?[\d.]+)\)') {
            $spawn = @{ X = $Matches[1]; Y = $Matches[2]; Z = $Matches[3] }
            break
        }
    }
    if (-not $spawn) { Write-Host "FAIL: no spawn point found on $mapName"; exit 1 }

    Write-Host "`n===== STATIC: bots converge on ($($spawn.X),$($spawn.Y),$($spawn.Z)) and idle ====="
    Write-Host "Read served and queue here, NOT err. Converging the bots is what holds line of sight,"
    Write-Host "but it is also where the hitbox-bone spread subtends its largest angle: err is measured"
    Write-Host "against the target's eye while the bot aims at a bone, and every bone is below the eye,"
    Write-Host "so err runs tens of degrees and is almost pure negative pitch. That is geometry, not a"
    Write-Host "defect. served must equal the requested delay exactly at every step."
    Write-Host "Do not use crossbot_teleport here: it stacks every bot on the identical origin, which"
    Write-Host "makes the eye-to-eye vector zero, so no angle exists and the queue is never fed."
    Write-Host "Idle micro-drift is switched off for this phase: it is a deliberate ~1 degree of error,"
    Write-Host "and leaving it on would mask exactly the small defects this phase exists to catch."

    Invoke-Rcon "crossbot_aim_drift 0" | Out-Null

    Invoke-Rcon "crossbot_nav_goal $($spawn.X) $($spawn.Y) $($spawn.Z)" | Out-Null
    Write-Host "Walking to the goal..."
    Start-Sleep -Seconds 20

    foreach ($delay in $Delays) {
        Invoke-Rcon "crossbot_aim_delay $delay" | Out-Null
        Write-Host "`n--- crossbot_aim_delay $delay ---"

        $deadline = (Get-Date).AddSeconds($SecondsPerDelay)
        while ((Get-Date) -lt $deadline) {
            Start-Sleep -Seconds 2
            Show-AimStatus
        }
    }

    if (-not $SkipRoam) {
        Write-Host "`n===== ROAM: gather on, bots move ====="
        Write-Host "err may be large -- that is target acceleration. Watch queue and served instead."
        Invoke-Rcon "crossbot_aim_delay 0.25" | Out-Null
        Invoke-Rcon "crossbot_aim_drift 1" | Out-Null
        Invoke-Rcon "crossbot_gather 1" | Out-Null

        $deadline = (Get-Date).AddSeconds($RoamSeconds)
        while ((Get-Date) -lt $deadline) {
            Start-Sleep -Seconds 3
            Show-AimStatus
        }
    }

    Write-Host "`n===== SUSPICION: which senses actually fired, which tier wins, and does the list decay ====="
    Write-Host "SOUND here is dominated by footsteps. They never reach pfnEmitSound -- pmove emits them"
    Write-Host "through pmove->PM_PlaySound, which the plugin swaps out inside its pfnPM_Move hook."
    Write-Host "Item pickups, doors and chargers make up the rest. Note a bot moving at 220 units/s or"
    Write-Host "slower emits no steps at all, and sv_footsteps 0 turns the whole sense off."
    Write-Host "EVENT and DAMAGE need weapon fire, which no bot does yet, so expect them to stay absent."
    Write-Host "tier= must read PLAYER whenever kinds=p: is nonzero: the other bot outranks every door"
    Write-Host "and lift in the map, no matter how recently they made a noise."

    Invoke-Rcon "crossbot_gather 1" | Out-Null
    $lastPushed = @{}
    $tierViolations = 0
    $tierChecks = 0
    # Cumulative high-water marks from keeps=, never the sampled keep= field. A bot holds a target on
    # well under half of frames, so a poll every few seconds reads keep=none almost always and would
    # fail an honest build -- which is exactly what it did the first time this was written.
    $keepCounts = @{ 'inc' = 0; 'new' = 0 }
    $deadline = (Get-Date).AddSeconds($SuspicionSeconds)
    while ((Get-Date) -lt $deadline) {
        Start-Sleep -Seconds 3
        $out = Invoke-Rcon "crossbot_sense_status"
        foreach ($m in [regex]::Matches($out, 'pushed=sgt:(\d+),snd:(\d+),evt:(\d+),dmg:(\d+)')) {
            foreach ($p in @(@('SIGHT', 1), @('SOUND', 2), @('EVENT', 3), @('DAMAGE', 4))) {
                $n = [int]$m.Groups[$p[1]].Value
                if ($n -gt [int]$lastPushed[$p[0]]) { $lastPushed[$p[0]] = $n }
            }
        }
        foreach ($m in [regex]::Matches($out, 'kinds=p:(\d+),m:\d+,o:\d+ tier=(\w+)')) {
            if ([int]$m.Groups[1].Value -le 0) { continue }
            $tierChecks++
            if ($m.Groups[2].Value -ne 'PLAYER') { $tierViolations++ }
        }
        foreach ($m in [regex]::Matches($out, 'keeps=inc:(\d+),new:(\d+)')) {
            if ([int]$m.Groups[1].Value -gt [int]$keepCounts['inc']) { $keepCounts['inc'] = [int]$m.Groups[1].Value }
            if ([int]$m.Groups[2].Value -gt [int]$keepCounts['new']) { $keepCounts['new'] = [int]$m.Groups[2].Value }
        }
        ($out -split "`r?`n") |
            Where-Object { $_ -match 'susp=|bot\(s\)' } |
            ForEach-Object { Write-Host ("  " + ($_ -replace '^CrossBot: ', '')) }
    }

    $fired = @($lastPushed.Keys | Where-Object { [int]$lastPushed[$_] -gt 0 } | Sort-Object)
    Write-Host "sources observed: $(($fired) -join ', ')"
    Write-Host ("tier=PLAYER held on {0} of {1} polls that had a live player entry" -f ($tierChecks - $tierViolations), $tierChecks)

    if ($tierChecks -gt 0 -and $tierViolations -gt 0) {
        Write-Host "FAIL: a live player entry lost the vote to a lower tier -- check LookDirection's tier pass"
        $failed = $true
    }
    if ($fired -notcontains 'SIGHT' -or $fired -notcontains 'SOUND') {
        Write-Host "FAIL: SIGHT and SOUND must both fire during roam"
        $failed = $true
    }

    # Vision spends at most two traces: the incumbent enemy, then a weighted-random pick. If
    # Incumbent() ever returns null unconditionally, everything still works -- the bot just re-rolls
    # its enemy every frame -- and nothing else in this suite would notice. keep=inc is the witness
    # that the lock is reachable at all. Guard the empty case: an all-none run says the bots never
    # saw each other, which is a broken phase, not a passing one.
    Write-Host ("keeps= inc:{0} new:{1} (cumulative frames, not samples)" -f $keepCounts['inc'], $keepCounts['new'])
    if (($keepCounts['inc'] + $keepCounts['new']) -eq 0) {
        Write-Host "FAIL: vision never saw anyone across the whole phase -- both bots blind, not a pass"
        $failed = $true
    }
    elseif ($keepCounts['inc'] -eq 0) {
        Write-Host "FAIL: keep=inc never held -- the incumbent enemy is never being re-traced, so the"
        Write-Host "      bot re-rolls its target every frame; check Incumbent() and BotCombat::Enemy"
        $failed = $true
    }

    Write-Host "`n--- the decay window actually bounds entry age ---"
    Write-Host "Do not assert 'no entries remain' after a quiet period, and do not use crossbot_kill to"
    Write-Host "force quiet: respawn resets the bot with CBot(), which empties the list whether or not"
    Write-Host "the decay works, so it would pass unconditionally."
    Write-Host "Gathering stays ON here. It used to be turned off, but one entry per entity means an"
    Write-Host "idle map really does go silent -- susp= sits at 0 and the age assertion then passes"
    Write-Host "against an empty list, which is worse than not running it. Moving bots keep emitting"
    Write-Host "footsteps, so there is always something live to measure."
    Write-Host "The invariant: no live entry may ever be older than crossbot_sense_decay. Shrink the"
    Write-Host "window to 1s and check the age column."

    Invoke-Rcon "crossbot_gather 1" | Out-Null
    Invoke-Rcon "crossbot_sense_decay 1" | Out-Null
    Start-Sleep -Seconds 5

    $worst = 0.0
    $sampled = 0
    for ($i = 0; $i -lt 5; $i++) {
        $out = Invoke-Rcon "crossbot_sense_status"
        ($out -split "`r?`n") |
            Where-Object { $_ -match 'susp=|bot\(s\)' } |
            ForEach-Object { Write-Host ("  " + ($_ -replace '^CrossBot: ', '')) }
        foreach ($m in [regex]::Matches($out, 'age=([\d.]+)s')) {
            $sampled++
            $a = [double]$m.Groups[1].Value
            if ($a -gt $worst) { $worst = $a }
        }
        Start-Sleep -Seconds 2
    }
    Invoke-Rcon "crossbot_sense_decay 5" | Out-Null

    Write-Host ("oldest live entry seen at decay=1: {0:N2}s over {1} sample(s)" -f $worst, $sampled)
    if ($sampled -eq 0) {
        Write-Host "FAIL: no live entry was ever observed -- the age assertion proved nothing"
        $failed = $true
    } elseif ($worst -gt 1.05) {
        Write-Host "FAIL: an entry outlived the decay window -- check the weight function"
        $failed = $true
    } else {
        Write-Host "PASS: the decay window bounds entry age"
    }

    if ($failed) { exit 1 }
    Write-Host "PASS: sense system"
}
finally {
    Stop-Process -Id $serverPid -Force -ErrorAction SilentlyContinue
    Write-Host "`nServer log: $logFile"
}
