param(
    [ValidateSet("all", "census", "engage", "teams")]
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

. "$scriptDir\common.ps1"

function Invoke-Phase([string]$name, [string]$map, [int]$port, [string[]]$boot,
        [scriptblock]$body, [string[]]$games = @("valve")) {
    Invoke-CbPhase $name $map $port $boot $body -LogDir $logDir -HLDSDir $HLDSDir `
        -LogPrefix "monster" -ConfigPrefix "_monster" -Failures $failures -Games $games
}

function Start-SnarkFight([int]$port, [int]$seconds) {
    for ($i = 1; $i -le $Bots; $i++) {
        Invoke-CbRconQuiet $port "crossbot_add valve Snark$i"
        Start-Sleep -Milliseconds 400
    }
    Start-Sleep -Seconds 3
    Invoke-CbRconQuiet $port "crossbot_equip weapon_9mmAR"

    $deadline = (Get-Date).AddSeconds($seconds)
    while ((Get-Date) -lt $deadline) {
        Invoke-CbRconQuiet $port "crossbot_spawn monster_snark"
        Start-Sleep -Seconds 2
        Invoke-CbRconQuiet $port "crossbot_monsters"
        Invoke-CbRconQuiet $port "crossbot_sense_status"
        Start-Sleep -Seconds 2
    }
}

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

    Assert-Cb ($snarks.Count -gt 0) "snarks reached the census ($($snarks.Count) lines)"
    Assert-Cb (-not $badOffset) "mon_hEnemy resolved to a real offset"
    Assert-Cb ($snarks.Count -gt 0 -and $flagged.Count -eq $snarks.Count) `
        "every snark line carried FL_MONSTER ($($flagged.Count)/$($snarks.Count))"
    Assert-Cb ($withEnemy.Count -gt 0) `
        "at least one snark named an enemy ($($withEnemy.Count)/$($snarks.Count))"

    [void]$results.Add("census snarks=$($snarks.Count) withEnemy=$($withEnemy.Count)")
}

function Test-Engage([int]$port) {
    $lines = Invoke-Phase "engage" "crossfire" $port @("crossbot_combat_debug 1") {
        param($p)
        Start-SnarkFight $p $Seconds
        Invoke-CbRconQuiet $p "crossbot_sense_status"
        Invoke-CbRconQuiet $p "crossbot_combat_status"
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

    $entered = Measure-CbCount $lines "entered combat with monster_snark"
    $switched = Measure-CbCount $lines "switches from .* to monster_snark"
    $stopped = Measure-CbCount $lines "left combat with monster_snark .*: the monster stopped hunting us"
    $died = Measure-CbCount $lines "left combat with monster_snark .*: the enemy died"
    $targeted = Measure-CbCount $lines "target=monster_snark"

    Assert-Cb ($mine -gt 0) "a snark named a bot as its enemy and passed the gate ($mine)"
    Assert-Cb ($traces -gt 0) "monster tracelines were spent ($traces)"
    Assert-Cb ($hits -gt 0) "a monster trace reached its target ($hits of $traces)"
    Assert-Cb ($targeted -gt 0) "a snark became a bot's sense target ($targeted)"
    Assert-Cb ($entered -gt 0) "a bot entered combat with a snark ($entered)"
    Assert-Cb ($died -gt 0) "a snark died in a fight a bot was having with it ($died)"

    [void]$results.Add(
        "engage mine=$mine trc=$traces hit=$hits entered=$entered switched=$switched " +
        "stopped=$stopped died=$died")
}

function Test-Teams([int]$port, [string]$side) {
    $bsp = Join-Path $HLDSDir "cstrike\maps\cs_monsters.bsp"
    Ensure-CbNav -HldsMap $bsp -NavPath (Join-Path $HLDSDir "crossplay\addons\crossbot\nav\cs_monsters.nav") `
        -NavGenExe (Join-Path $repoRoot "build-x64\tools\navgen\navgen.exe")

    $lines = Invoke-Phase "teams_$side" "cs_monsters" $port @("crossbot_combat_debug 1", "mp_freezetime 0") {
        param($p)
        Invoke-CbRconQuiet $p "crossbot_add cstrike Mon_$side $side"
        $deadline = (Get-Date).AddSeconds($Seconds)
        while ((Get-Date) -lt $deadline) {
            Start-Sleep -Seconds 1
            Invoke-CbRconQuiet $p "crossbot_monsters"
        }
        Invoke-CbRconQuiet $p "crossbot_teams_status"
        Invoke-CbRconQuiet $p "crossbot_combat_status"
        Start-Sleep -Milliseconds 800
    } @("valve", "cstrike", "gearbox")

    $census = @{}
    foreach ($m in ($lines | Select-String -Pattern "monster #\d+ (monster_sentry|monster_human_grunt_ally) .* team=(\d+) ")) {
        $census["$($m.Matches[0].Groups[1].Value):$($m.Matches[0].Groups[2].Value)"] = $true
    }
    $teamsOn = Measure-CbCount $lines "-> teams on"
    $friends = Measure-CbCount $lines "rel=friend "
    $enemies = Measure-CbCount $lines "rel=enemy "
    $hunting = $false
    $friendHunting = 0
    foreach ($line in $lines) {
        if ($line -match "monster #d+ monster_") { $hunting = $line -notmatch "enemy=-1:" }
        elseif ($hunting -and $line -match "rel=friend ") { $friendHunting++ }
    }
    $friendHits = Measure-CbCount $lines "rel=friend (tgt=1|tgt=\d fight=1)"
    $enemyTgt = Measure-CbCount $lines "rel=enemy tgt=1"
    $enemyFight = Measure-CbCount $lines "rel=enemy tgt=\d fight=1"
    $entered = Measure-CbCount $lines "entered combat with monster_"
    $died = Measure-CbCount $lines "left combat with monster_.*: the enemy died"

    foreach ($k in @("monster_sentry:1", "monster_sentry:2", "monster_human_grunt_ally:1", "monster_human_grunt_ally:2")) {
        Assert-Cb ($census.ContainsKey($k)) "census saw $k"
    }
    Assert-Cb ($teamsOn -gt 0) "teams are on for cs_monsters"
    Assert-Cb ($friends -gt 0) "friendly monster lines were judged ($friends)"
    Assert-Cb ($friendHunting -gt 0) "a friendly monster was hunting someone while judged ($friendHunting)"
    Assert-Cb ($friendHits -eq 0) "no bot targeted or fought a friendly monster ($friendHits)"
    Assert-Cb ($enemyTgt -gt 0) "an enemy monster became a bot's sense target ($enemyTgt of $enemies)"
    Assert-Cb ($enemyFight -gt 0) "a bot fought an enemy monster ($enemyFight)"
    Assert-Cb ($entered -gt 0) "a bot entered combat with a monster ($entered)"
    Assert-Cb ($died -gt 0) "a monster died in a fight a bot was having with it ($died)"

    [void]$results.Add("teams_$side census=$($census.Count) friends=$friends friendHunting=$friendHunting friendHits=$friendHits " +
        "enemies=$enemies enemyTgt=$enemyTgt enemyFight=$enemyFight entered=$entered died=$died")
}

$p = $Port
if ($Phase -eq "all" -or $Phase -eq "census") { Test-Census $p; $p++ }
if ($Phase -eq "all" -or $Phase -eq "engage") { Test-Engage $p; $p++ }
if ($Phase -eq "all" -or $Phase -eq "teams") { Test-Teams $p "ct"; $p++; Test-Teams $p "t"; $p++ }

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
