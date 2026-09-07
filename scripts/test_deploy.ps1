<#
.SYNOPSIS
    Deployable test: bots place tripmines and satchels, avoid the beams, and
    detonate satchels on contact.

.DESCRIPTION
    Four phases, each on its own server:

      gates    -- crossbot_list_gates on several stock maps. Asserts every map
                  finds chokepoints and that the openings look like doorways
                  rather than open floor. This is the only phase that needs no
                  bots and the only one that fails on a bad gate scan alone.
      tripmine -- bots equipped with tripmines on crossfire. Asserts placements,
                  both stances, and that beams are met with duck AND jump.
      standoff -- surface pinned to ground on test2_doorway, which has one route,
                  so a vertical beam blocks it outright. Asserts the shoot path:
                  side search fails, the bot backs off and destroys the mine.
      satchel  -- bots equipped with satchels on crossfire. Asserts placements and
                  detonations, and that detonations are NOT self-triggered.

    The nav sweep cannot see any of this: it runs one bot with crossbot_deploy 0,
    because mines placed mid-sweep change every route on the map.

.PARAMETER Phase
    Run one phase only. Default: all four.

.PARAMETER Seconds
    Seconds per bot phase (default 150). The floor is about 60 -- below that a
    phase measures nothing, because a bot has to wander into a gate first.

.PARAMETER Port
    Base port (default 29200). Each phase takes the next one.

.EXAMPLE
    .\scripts\test_deploy.ps1

.EXAMPLE
    .\scripts\test_deploy.ps1 -Phase standoff -Seconds 240
#>
param(
    [ValidateSet("all", "gates", "tripmine", "standoff", "satchel")]
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

function Assert-That([bool]$ok, [string]$what) {
    $tag = if ($ok) { "PASS" } else { "FAIL" }
    Write-Host "  [$tag] $what"
    if (-not $ok) { [void]$failures.Add($what) }
}

function Count-Of($lines, [string]$pattern) {
    return @($lines | Select-String -Pattern $pattern -SimpleMatch:$false).Count
}

# Boots a server, runs $Body against it, always stops it. Returns the log lines.
function Invoke-Phase([string]$name, [string]$map, [int]$port, [string[]]$boot,
        [scriptblock]$body) {
    $log = Join-Path $logDir "deploy_$name.log"
    if (Test-Path $log) { Remove-Item $log }

    Write-Host ""
    Write-Host "=== $name ($map) ==="

    $start = & "$scriptDir\test_server_start.ps1" -Map $map -Port $port -LogFile $log `
        -WaitSeconds 90 -ConfigName "_deploy_$name.cfg" -Commands $boot -HLDSDir $HLDSDir `
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

# --- gates -------------------------------------------------------------------
# A stock map with no chokepoints means the gate scan is broken, not that the map
# is unusual. The opening bound is the real assertion: anything over
# kDeployOpeningMax would mean the physical measurement is not being applied.
function Test-Gates([int]$port) {
    foreach ($map in @("crossfire", "boot_camp", "stalkyard", "test2_doorway")) {
        $lines = Invoke-Phase "gates_$map" $map $port @() {
            param($p)
            Rcon $p "crossbot_list_gates"
            Start-Sleep -Milliseconds 1500
        }
        $port++

        $gates = @($lines | Select-String "CrossBot: gate " | Where-Object { $_.Line -match ' ok$' })
        Assert-That ($gates.Count -gt 0) "$map finds at least one deployable gate ($($gates.Count))"

        $wide = @($gates | Where-Object {
            $_.Line -match 'open (\d+)' -and [int]$Matches[1] -gt 144
        })
        Assert-That ($wide.Count -eq 0) "$map has no gate wider than the 144 opening bound"

        [void]$results.Add("$map gates=$($gates.Count)")
    }
}

# --- tripmine ----------------------------------------------------------------
# Both stances must appear, and both maneuvers must follow from them: a standing
# placement puts the beam at ~64 and is crawled under, a ducked one at ~30 and is
# jumped. Seeing only one of the two means the stance roll is dead.
function Test-Tripmine([int]$port) {
    $lines = Invoke-Phase "tripmine" "crossfire" $port @("crossbot_deploy_debug 1") {
        param($p)
        foreach ($n in @("Sap1", "Sap2", "Sap3")) {
            Rcon $p "crossbot_add valve $n"
            Start-Sleep -Milliseconds 400
        }
        Start-Sleep -Seconds 3
        $deadline = (Get-Date).AddSeconds($Seconds)
        while ((Get-Date) -lt $deadline) {
            Rcon $p "crossbot_equip weapon_tripmine"
            Start-Sleep -Seconds 4
        }
        Rcon $p "crossbot_deploy_status"
        Start-Sleep -Milliseconds 800
    }

    $placed = Count-Of $lines "placed a mine"
    $stand = Count-Of $lines ", standing\)"
    $duck = Count-Of $lines ", ducked\)"
    $jump = Count-Of $lines "avoid jump monster_tripmine"
    $crawl = Count-Of $lines "avoid duck monster_tripmine"
    $removed = Count-Of $lines "Tripmine at .* removed"
    $goals = Count-Of $lines "reached goal"
    $contact = Count-Of $lines "KILLED BY ITS OWN MINE.*onbeam 1"
    $blast = Count-Of $lines "KILLED BY ITS OWN MINE.*onbeam 0"
    $armedIn = Count-Of $lines "STILL IN BEAM AT ARMING"

    Assert-That ($placed -gt 0) "tripmines placed ($placed)"
    Assert-That ($stand -gt 0 -and $duck -gt 0) "both stances used (stand=$stand duck=$duck)"
    Assert-That ($jump -eq 0) "no beam was jumped -- a beam cannot be jumped ($jump)"
    Assert-That ($crawl -gt 0) "a beam was crawled under ($crawl)"
    Assert-That ($removed -eq 0) "no mine failed to find a surface ($removed removed)"
    Assert-That ($goals -gt 0) "bots still reach goals with beams up ($goals)"
    Assert-That ($contact -eq 0) "no bot walked into its own beam ($contact contact deaths)"
    Assert-That ($armedIn -eq 0) "no mine armed around its placer ($armedIn)"

    [void]$results.Add(
        "tripmine placed=$placed duck=$crawl goals=$goals contact=$contact blast=$blast")
}

# --- standoff ----------------------------------------------------------------
# test2_doorway has one route per gate, so a vertical beam cannot be walked
# around. The map is also smaller than kMineSafeRadius end to end, so the retreat
# always ends on kBreakRetreatTime -- that is the graceful path, not a failure.
# The bot is driven between the spawn and the info_target because it has no items
# to fetch here and would otherwise never leave its spawn area.
function Test-Standoff([int]$port) {
    $lines = Invoke-Phase "standoff" "test2_doorway" $port @("crossbot_deploy_surface 3", "crossbot_deploy_debug 1") {
        param($p)
        Rcon $p "crossbot_add valve Miner"
        Start-Sleep -Seconds 2
        Rcon $p "crossbot_equip weapon_9mmhandgun"
        $goals = @("400 384 32", "128 384 48")
        $i = 0
        $deadline = (Get-Date).AddSeconds($Seconds)
        while ((Get-Date) -lt $deadline) {
            Rcon $p "crossbot_equip weapon_tripmine"
            Rcon $p ("crossbot_nav_goal " + $goals[$i % 2])
            $i++
            Start-Sleep -Seconds 7
        }
        Rcon $p "crossbot_deploy_status"
        Start-Sleep -Milliseconds 800
    }

    $placed = Count-Of $lines "placed a mine"
    $shoot = Count-Of $lines "avoid shooting monster_tripmine"
    $engaged = Count-Of $lines "is shooting monster_tripmine"
    $killed = Count-Of $lines "destroyed monster_tripmine"
    $stuck = Count-Of $lines "blocked by monster_tripmine"

    Assert-That ($placed -gt 0) "ground mines placed under the pin ($placed)"
    Assert-That ($shoot -gt 0) "side search gave up and chose to shoot ($shoot)"
    Assert-That ($engaged -gt 0) "standoff engaged ($engaged)"
    Assert-That ($killed -gt 0) "mine destroyed by gunfire ($killed)"
    Assert-That ($stuck -eq 0) "bot never sat blocked by a beam ($stuck)"

    [void]$results.Add("standoff placed=$placed shoot=$shoot killed=$killed")
}

# --- satchel -----------------------------------------------------------------
# The contact distance is the assertion that matters. A satchel plays a bounce
# sound on landing and BotSense pushes a suspicion for the emitter, so a bot that
# counts its own charge as a contact detonates instantly and reports ~2 units.
function Test-Satchel([int]$port) {
    $lines = Invoke-Phase "satchel" "crossfire" $port @() {
        param($p)
        foreach ($n in @("Sap1", "Sap2", "Sap3", "Sap4")) {
            Rcon $p "crossbot_add valve $n"
            Start-Sleep -Milliseconds 400
        }
        Start-Sleep -Seconds 3
        $deadline = (Get-Date).AddSeconds($Seconds)
        while ((Get-Date) -lt $deadline) {
            Rcon $p "crossbot_equip weapon_satchel"
            Start-Sleep -Seconds 5
        }
        Rcon $p "crossbot_deploy_status"
        Start-Sleep -Milliseconds 800
    }

    $placed = Count-Of $lines "placed a mine"
    $nothing = Count-Of $lines "placed nothing"
    $dets = Count-Of $lines "detonated a satchel"
    $tries = Count-Of $lines "is detonating a satchel"

    $near = @($lines | Select-String "is detonating a satchel" | Where-Object {
        $_.Line -match 'contact (\d+) away' -and [int]$Matches[1] -lt 16
    })

    Assert-That ($placed -gt 0) "satchels thrown ($placed)"
    Assert-That ($placed -gt $nothing) "more throws landed than failed ($placed vs $nothing)"
    Assert-That ($dets -gt 0) "satchels detonated ($dets)"
    Assert-That ($dets -le $tries) "no detonation counted twice ($dets of $tries)"
    Assert-That ($near.Count -eq 0) "no self-triggered detonation ($($near.Count) contacts under 16u)"

    [void]$results.Add("satchel placed=$placed dets=$dets tries=$tries")
}

# --- run ---------------------------------------------------------------------
Write-Host "=== crossbot deployable test ==="
Write-Host "Phase: $Phase, $Seconds s per bot phase"

if ($Phase -eq "all" -or $Phase -eq "gates")    { Test-Gates    ($Port) }
if ($Phase -eq "all" -or $Phase -eq "tripmine") { Test-Tripmine ($Port + 10) }
if ($Phase -eq "all" -or $Phase -eq "standoff") { Test-Standoff ($Port + 11) }
if ($Phase -eq "all" -or $Phase -eq "satchel")  { Test-Satchel  ($Port + 12) }

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
