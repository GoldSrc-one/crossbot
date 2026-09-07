<#
.SYNOPSIS
    Monster engagement: snarks hunt the bots, and the bots they hunt shoot back.

.DESCRIPTION
    Two phases, each on its own server.

      census  -- crossbot_monsters with live snarks on the map. This is the
                 assumption check, and it runs first because everything else is
                 built on it: monster_snark must carry FL_MONSTER, and
                 CBaseMonster::m_hEnemy must read back an edict the plugin can
                 resolve. If the offset is wrong the census fails and the engage
                 phase's failure would be unreadable.
      engage  -- the behaviour. A snark that names a bot as its enemy must be
                 traced, must enter that bot's combat state, and must die.

    Snarks come from crossbot_spawn, in front of each bot, refreshed every few
    seconds. See Start-SnarkFight for why they are not thrown by the bots.

    The census phase is what distinguishes "the bot ignores snarks" from "there
    were no snarks". Both look identical in the engage counters alone.

.PARAMETER Seconds
    Seconds per phase (default 150). The floor is about 40: a snark lives ~25s
    on its own detonate timer, so a shorter window can end mid-life and see no
    exit at all.

.PARAMETER Port
    Base port (default 29300). Each phase takes the next one.

.EXAMPLE
    .\scripts\test_monster.ps1

.EXAMPLE
    .\scripts\test_monster.ps1 -Phase engage -Seconds 300
#>
param(
    [ValidateSet("all", "census", "engage")]
    [string]$Phase = "all",
    [int]$Seconds = 150,
    [int]$Bots = 4,
    [int]$Port = 29300,
    [string]$HLDSDir = "D:\hlds"
)

$ErrorActionPreference = "Stop"
$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$repoRoot = Split-Path -Parent $scriptDir
$logDir = Join-Path $repoRoot "nav_debug\logs"
if (-not (Test-Path $logDir)) { New-Item -ItemType Directory -Path $logDir -Force | Out-Null }

$failures = New-Object System.Collections.ArrayList
$results = New-Object System.Collections.ArrayList

function Assert-That([bool]$ok, [string]$what) {
    $tag = if ($ok) { "PASS" } else { "FAIL" }
    Write-Host "  [$tag] $what"
    if (-not $ok) { [void]$failures.Add($what) }
}

function Count-Of($lines, [string]$pattern) {
    return @($lines | Select-String -Pattern $pattern -SimpleMatch:$false).Count
}

function Invoke-Phase([string]$name, [string]$map, [int]$port, [string[]]$boot,
        [scriptblock]$body) {
    $log = Join-Path $logDir "monster_$name.log"
    if (Test-Path $log) { Remove-Item $log }

    Write-Host ""
    Write-Host "=== $name ($map) ==="

    $start = & "$scriptDir\test_server_start.ps1" -Map $map -Port $port -LogFile $log `
        -WaitSeconds 90 -ConfigName "_monster_$name.cfg" -Commands $boot -HLDSDir $HLDSDir `
        -RequireVerbose
    if ($start -notmatch 'PID=(\d+)') {
        Assert-That $false "$name server started ($start)"
        return @()
    }
    $srv = [int]$Matches[1]

    try {
        & $body $port
    } finally {
        & "$scriptDir\test_server_stop.ps1" -Id $srv | Out-Null
    }

    if (-not (Test-Path $log)) { return @() }
    return Get-Content $log
}

function Rcon([int]$port, [string]$cmd) {
    & "$scriptDir\rcon.ps1" -Command $cmd -Port $port | Out-Null
}

# Adds the bots, arms them, and keeps a snark in front of each one.
#
# Snarks are spawned with crossbot_spawn rather than thrown by the bots, and
# that is not a shortcut -- letting the bots produce them does not work. The
# snark is reachable only through the ambush path, which needs an opponent who
# has not noticed the bot; crossbot_fire_pin gets a bot to hold one, and then
# the uphill veto refuses the throw, because it refuses any throwable at a
# target more than 16 units above the eye and crossfire is vertical almost
# everywhere. Measured over a minute with the pin on, two bots reached
# uphill=510 and throwshots=0. A phase built on that would be measuring the
# ambush picker, not monster engagement, and would fail for reasons that have
# nothing to do with either.
#
# The 9mmAR is equipped because a snark has 2 health and dies to one bullet:
# what is being measured is whether the bot engages at all, not whether it can
# win, and a bot that spawned with a crowbar would confound the two.
function Start-SnarkFight([int]$port, [int]$seconds) {
    for ($i = 1; $i -le $Bots; $i++) {
        Rcon $port "crossbot_add valve Snark$i"
        Start-Sleep -Milliseconds 400
    }
    Start-Sleep -Seconds 3
    Rcon $port "crossbot_equip weapon_9mmAR"

    $deadline = (Get-Date).AddSeconds($seconds)
    while ((Get-Date) -lt $deadline) {
        Rcon $port "crossbot_spawn monster_snark"
        Start-Sleep -Seconds 2
        Rcon $port "crossbot_monsters"
        Rcon $port "crossbot_sense_status"
        Start-Sleep -Seconds 2
    }
}

# --- census ------------------------------------------------------------------
# Reads only the entity facts the engage phase depends on. A snark line with
# enemy=-1 for its whole life means m_hEnemy is being read from the wrong place,
# which is a different failure from a bot that sees the snark and ignores it.
function Test-Census([int]$port) {
    $lines = Invoke-Phase "census" "crossfire" $port @() {
        param($p)
        Start-SnarkFight $p ([int]($Seconds / 3))
    }

    $snarks = @($lines | Select-String -Pattern "monster #\d+ monster_snark")
    $withEnemy = @($snarks | Where-Object { $_ -notmatch "enemy=-1:" })
    $flagged = @($snarks | Where-Object { $_ -match "flags=0x[0-9a-f]*" })
    $offset = @($lines | Select-String -Pattern "mon_hEnemy offset (\d+)")

    $badOffset = $true
    if ($offset.Count -gt 0 -and $offset[-1] -match "mon_hEnemy offset (\d+)") {
        $badOffset = ([int]$Matches[1] -le 0)
    }

    Assert-That ($snarks.Count -gt 0) "snarks reached the census ($($snarks.Count) lines)"
    Assert-That (-not $badOffset) "mon_hEnemy resolved to a real offset"
    Assert-That ($snarks.Count -gt 0 -and $flagged.Count -eq $snarks.Count) `
        "every snark line carried FL_MONSTER ($($flagged.Count)/$($snarks.Count))"
    Assert-That ($withEnemy.Count -gt 0) `
        "at least one snark named an enemy ($($withEnemy.Count)/$($snarks.Count))"

    [void]$results.Add("census snarks=$($snarks.Count) withEnemy=$($withEnemy.Count)")
}

# --- engage ------------------------------------------------------------------
# The whole chain: the gate lets a hunting snark through, a trace lands on it,
# combat enters naming it, and it dies. mine>0 with hit==0 would mean the gate
# works but every trace is blocked -- a different bug from mine==0.
function Test-Engage([int]$port) {
    $lines = Invoke-Phase "engage" "crossfire" $port @("crossbot_combat_debug 1") {
        param($p)
        Start-SnarkFight $p $Seconds
        Rcon $p "crossbot_sense_status"
        Rcon $p "crossbot_combat_status"
        Start-Sleep -Milliseconds 800
    }

    $mine = 0
    $traces = 0
    $hits = 0
    foreach ($m in ($lines | Select-String -Pattern "mon=list:\d+,seen:\d+,mine:(\d+),trc:(\d+),hit:(\d+)" -AllMatches)) {
        foreach ($g in $m.Matches) {
            $mine += [int]$g.Groups[1].Value
            $traces += [int]$g.Groups[2].Value
            $hits += [int]$g.Groups[3].Value
        }
    }

    $entered = Count-Of $lines "entered combat with monster_snark"
    $switched = Count-Of $lines "switches from .* to monster_snark"
    $stopped = Count-Of $lines "left combat with monster_snark .*: the monster stopped hunting us"
    $died = Count-Of $lines "left combat with monster_snark .*: the enemy died"
    $targeted = Count-Of $lines "target=monster_snark"

    Assert-That ($mine -gt 0) "a snark named a bot as its enemy and passed the gate ($mine)"
    Assert-That ($traces -gt 0) "monster tracelines were spent ($traces)"
    Assert-That ($hits -gt 0) "a monster trace reached its target ($hits of $traces)"
    Assert-That ($targeted -gt 0) "a snark became a bot's sense target ($targeted)"
    Assert-That ($entered -gt 0) "a bot entered combat with a snark ($entered)"
    Assert-That ($died -gt 0) "a snark died in a fight a bot was having with it ($died)"

    [void]$results.Add(
        "engage mine=$mine trc=$traces hit=$hits entered=$entered switched=$switched " +
        "stopped=$stopped died=$died")
}

# --- run ---------------------------------------------------------------------
$p = $Port
if ($Phase -eq "all" -or $Phase -eq "census") { Test-Census $p; $p++ }
if ($Phase -eq "all" -or $Phase -eq "engage") { Test-Engage $p; $p++ }

Write-Host ""
Write-Host "=== summary ==="
foreach ($r in $results) { Write-Host "  $r" }

Write-Host ""
if ($failures.Count -eq 0) {
    Write-Host "ALL PASS"
    exit 0
}

Write-Host "$($failures.Count) FAILURE(S):"
foreach ($f in $failures) { Write-Host "  - $f" }
exit 1
