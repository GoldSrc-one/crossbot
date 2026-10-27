param(
    [ValidateSet("all", "gates", "tripmine", "standoff", "satchel", "crossgame", "reroute")]
    [string]$Phase = "all",
    [int]$Seconds = 150,
    [int]$Port = 29200,
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
        [scriptblock]$body) {
    Invoke-CbPhase $name $map $port (@("amxx pause crossbot_menu.amxx") + $boot) $body -LogDir $logDir -HLDSDir $HLDSDir `
        -LogPrefix "deploy" -ConfigPrefix "_deploy" -Failures $failures
}

function Test-Gates([int]$port) {
    foreach ($map in @("crossfire", "boot_camp", "stalkyard", "test2_doorway")) {
        $lines = Invoke-Phase "gates_$map" $map $port @() {
            param($p)
            Invoke-CbRconQuiet $p "crossbot_list_gates"
            Start-Sleep -Milliseconds 1500
        }
        $port++

        $gates = @($lines | Select-String "CrossBot: gate " | Where-Object { $_.Line -match ' ok$' })
        Assert-Cb ($gates.Count -gt 0) "$map finds at least one deployable gate ($($gates.Count))"

        $wide = @($gates | Where-Object {
            $_.Line -match 'open (\d+)' -and [int]$Matches[1] -gt 144
        })
        Assert-Cb ($wide.Count -eq 0) "$map has no gate wider than the 144 opening bound"

        [void]$results.Add("$map gates=$($gates.Count)")
    }
}

function Assert-BeamJumps($lines, [string]$who, [bool]$expected = $true) {
    $cleared = Measure-CbCount $lines "${who}beam jump cleared"
    $failed = Measure-CbCount $lines "${who}beam jump FAILED"
    $tried = $cleared + $failed
    if ($expected) { Assert-Cb ($tried -gt 0) "beams were crouch-jumped ($tried)" }
    if ($tried -gt 0) {
        $rate = [Math]::Round(100.0 * $failed / $tried, 1)
        Assert-Cb ($failed * 20 -le $tried) "beam jumps fail at most 5% ($failed of $tried, $rate%)"
    }
    return "jumps=$cleared/$tried"
}

function Test-Tripmine([int]$port) {
    $lines = Invoke-Phase "tripmine" "crossfire" $port @("crossbot_deploy_debug 1") {
        param($p)
        foreach ($n in @("Sap1", "Sap2", "Sap3")) {
            Invoke-CbRconQuiet $p "crossbot_add valve $n"
            Start-Sleep -Milliseconds 400
        }
        Start-Sleep -Seconds 3
        $deadline = (Get-Date).AddSeconds($Seconds)
        while ((Get-Date) -lt $deadline) {
            Invoke-CbRconQuiet $p "crossbot_equip weapon_tripmine"
            Start-Sleep -Seconds 4
        }
        Invoke-CbRconQuiet $p "crossbot_deploy_status"
        Start-Sleep -Milliseconds 800
    }

    $placed = Measure-CbCount $lines "placed a mine"
    $stand = Measure-CbCount $lines ", standing\)"
    $duck = Measure-CbCount $lines ", ducked\)"
    $crawl = Measure-CbCount $lines "avoid duck monster_tripmine"
    $removed = Measure-CbCount $lines "Tripmine at .* removed"
    $goals = Measure-CbCount $lines "reached goal"
    $contact = @($lines | Select-String "KILLED BY ITS OWN MINE.*onbeam 1" |
        Where-Object { $_.Line -notmatch "avoid jump vs" }).Count
    $blast = Measure-CbCount $lines "KILLED BY ITS OWN MINE.*onbeam 0"
    $armedIn = Measure-CbCount $lines "STILL IN BEAM AT ARMING"

    Assert-Cb ($placed -gt 0) "tripmines placed ($placed)"
    Assert-Cb ($stand -gt 0 -and $duck -gt 0) "both stances used (stand=$stand duck=$duck)"
    $jumps = Assert-BeamJumps $lines ""
    Assert-Cb ($crawl -gt 0) "a beam was crawled under ($crawl)"
    Assert-Cb ($removed -eq 0) "no mine failed to find a surface ($removed removed)"
    Assert-Cb ($goals -gt 0) "bots still reach goals with beams up ($goals)"
    Assert-Cb ($contact -eq 0) "no bot walked into its own beam ($contact contact deaths)"
    Assert-Cb ($armedIn -eq 0) "no mine armed around its placer ($armedIn)"

    [void]$results.Add(
        "tripmine placed=$placed duck=$crawl $jumps goals=$goals contact=$contact blast=$blast")
}

function Test-Standoff([int]$port) {
    $lines = Invoke-Phase "standoff" "test2_doorway" $port @("crossbot_deploy_surface 3", "crossbot_deploy_debug 1") {
        param($p)
        Invoke-CbRconQuiet $p "crossbot_add valve Miner"
        Start-Sleep -Seconds 2
        Invoke-CbRconQuiet $p "crossbot_equip weapon_9mmhandgun"
        $goals = @("400 384 32", "128 384 48")
        $i = 0
        $deadline = (Get-Date).AddSeconds($Seconds)
        while ((Get-Date) -lt $deadline) {
            Invoke-CbRconQuiet $p "crossbot_equip weapon_tripmine"
            Invoke-CbRconQuiet $p ("crossbot_nav_goal " + $goals[$i % 2])
            $i++
            Start-Sleep -Seconds 7
        }
        Invoke-CbRconQuiet $p "crossbot_deploy_status"
        Start-Sleep -Milliseconds 800
    }

    $placed = Measure-CbCount $lines "placed a mine"
    $shoot = Measure-CbCount $lines "avoid shooting monster_tripmine"
    $engaged = Measure-CbCount $lines "is shooting monster_tripmine"
    $killed = Measure-CbCount $lines "destroyed monster_tripmine"
    $stuck = Measure-CbCount $lines "blocked by monster_tripmine"

    Assert-Cb ($placed -gt 0) "ground mines placed under the pin ($placed)"
    Assert-Cb ($shoot -gt 0) "side search gave up and chose to shoot ($shoot)"
    Assert-Cb ($engaged -gt 0) "standoff engaged ($engaged)"
    Assert-Cb ($killed -gt 0) "mine destroyed by gunfire ($killed)"
    Assert-Cb ($stuck -eq 0) "bot never sat blocked by a beam ($stuck)"

    [void]$results.Add("standoff placed=$placed shoot=$shoot killed=$killed")
}

function Test-Crossgame([int]$port, [string]$name, [int]$jump) {
    $boot = @("mp_timelimit 0", "mp_freezetime 0", "mp_roundtime 9", "amxx pause crossbot_menu.amxx",
        "crossbot_deploy_debug 1", "crossbot_strategic 0", "crossbot_combat 0", 'crossbot_name_prefix ""',
        "crossbot_beam_jump $jump")
    $lines = Invoke-CbPhase $name "de_dust2" $port $boot {
        param($p)
        Invoke-CbRconQuiet $p "crossbot_add valve Miner"
        Invoke-CbRconQuiet $p "crossbot_add cstrike Follower t"
        Start-Sleep -Seconds 10
        $deadline = (Get-Date).AddSeconds($Seconds)
        while ((Get-Date) -lt $deadline) {
            Invoke-CbRconQuiet $p "crossbot_equip weapon_tripmine Miner"
            Invoke-CbRconQuiet $p "crossbot_teleport -420 -280 40 Miner"
            Invoke-CbRconQuiet $p "crossbot_teleport -437 -420 47 Follower"
            Invoke-CbRconQuiet $p "crossbot_nav_goal -392 160 40"
            Start-Sleep -Seconds 8
            Invoke-CbRconQuiet $p "crossbot_nav_status"
            Start-Sleep -Seconds 12
            Invoke-CbRconQuiet $p "crossbot_nav_status"
            Start-Sleep -Milliseconds 500
        }
        Invoke-CbRconQuiet $p "crossbot_deploy_status"
        Start-Sleep -Milliseconds 800
    } -LogDir $logDir -HldsDir $HLDSDir -LogPrefix "deploy" -ConfigPrefix "_deploy" -Failures $failures `
        -Games @("valve", "cstrike")

    $placed = Measure-CbCount $lines "Miner placed a mine"
    $reached = Measure-CbCount $lines "Follower reached goal"
    $guard = Measure-CbCount $lines "Follower beam guard stop"

    Assert-Cb ($placed -gt 0) "the HL bot placed a mine at the gate ($placed)"
    Assert-Cb ($reached -gt 0) "the CS bot still reached its goal ($reached)"
    if ($jump -ne 0) {
        $low = Measure-CbCount $lines "Miner is mining .*, ducked\)"
        $extra = Assert-BeamJumps $lines "Follower " ($low -gt 0)
    } else {
        $reroutes = Measure-CbCount $lines "Follower avoid rerouting around monster_tripmine"
        $jumped = Measure-CbCount $lines "avoid beam-jump"
        Assert-Cb ($reroutes -gt 0) "the CS bot rerouted before shooting ($reroutes)"
        Assert-Cb ($jumped -eq 0) "crossbot_beam_jump 0 stops every beam jump ($jumped)"
        $extra = "reroutes=$reroutes"
    }

    [void]$results.Add("$name placed=$placed reached=$reached guardstop=$guard $extra")
}

function Test-Satchel([int]$port) {
    $lines = Invoke-Phase "satchel" "crossfire" $port @() {
        param($p)
        foreach ($n in @("Sap1", "Sap2", "Sap3", "Sap4")) {
            Invoke-CbRconQuiet $p "crossbot_add valve $n"
            Start-Sleep -Milliseconds 400
        }
        Start-Sleep -Seconds 3
        $deadline = (Get-Date).AddSeconds($Seconds)
        while ((Get-Date) -lt $deadline) {
            Invoke-CbRconQuiet $p "crossbot_equip weapon_satchel"
            Start-Sleep -Seconds 5
        }
        Invoke-CbRconQuiet $p "crossbot_deploy_status"
        Start-Sleep -Milliseconds 800
    }

    $placed = Measure-CbCount $lines "placed a mine"
    $nothing = Measure-CbCount $lines "placed nothing"
    $dets = Measure-CbCount $lines "detonated a satchel"
    $tries = Measure-CbCount $lines "is detonating a satchel"

    $near = @($lines | Select-String "is detonating a satchel" | Where-Object {
        $_.Line -match 'contact (\d+) away' -and [int]$Matches[1] -lt 16
    })

    Assert-Cb ($placed -gt 0) "satchels thrown ($placed)"
    Assert-Cb ($placed -gt $nothing) "more throws landed than failed ($placed vs $nothing)"
    Assert-Cb ($dets -gt 0) "satchels detonated ($dets)"
    Assert-Cb ($dets -le $tries) "no detonation counted twice ($dets of $tries)"
    Assert-Cb ($near.Count -eq 0) "no self-triggered detonation ($($near.Count) contacts under 16u)"

    [void]$results.Add("satchel placed=$placed dets=$dets tries=$tries")
}

Write-Host "=== crossbot deployable test ==="
Write-Host "Phase: $Phase, $Seconds s per bot phase"

if ($Phase -eq "all" -or $Phase -eq "gates")    { Test-Gates    ($Port) }
if ($Phase -eq "all" -or $Phase -eq "tripmine") { Test-Tripmine ($Port + 10) }
if ($Phase -eq "all" -or $Phase -eq "standoff") { Test-Standoff ($Port + 11) }
if ($Phase -eq "all" -or $Phase -eq "satchel")  { Test-Satchel  ($Port + 12) }
if ($Phase -eq "crossgame") { Test-Crossgame ($Port + 13) "crossgame" 1 }
if ($Phase -eq "reroute")   { Test-Crossgame ($Port + 14) "reroute" 0 }

Write-Host ""
Write-Host "=== summary ==="
foreach ($r in $results) { Write-Host "  $r" }

if ($failures.Count -gt 0) {
    Write-Host ""
    Write-Host "FAILED ($($failures.Count)):"
    foreach ($f in $failures) { Write-Host "  $f" }
    exit 1
}

Write-Host ""
Write-Host "PASS"
exit 0
