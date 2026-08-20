<#
.SYNOPSIS
    Driver for the hitscan firing loop: boots HLDS, lets a handful of bots roam
    a real deathmatch map, and checks that they actually shoot each other.

.DESCRIPTION
    Nothing else in the suite can exercise this. The nav sweep spawns one bot per
    server, so it never acquires a target and the firing code exits on its first
    branch every frame. test_sense.ps1 converges its bots onto a single spawn,
    which is exactly the arrangement firing cannot be measured in: converged bots
    end up on the same origin, the eye-to-eye vector degenerates and vision drops
    them.

    So this script does the opposite of test_sense.ps1 and leaves gathering on.
    Roaming bots collect weapons, cross each other's line of sight at real ranges
    and fight, which is the only cheap way to get the whole chain -- sense, aim
    queue, weapon choice, the fire ray and the button -- under load at once.
    Encounters are stochastic, so the duel phase is generous with time and the
    pass condition is "shots were fired at real targets", not a fixed count.

    CONTROL: crossbot_fire 0. Bots roam and must fire nothing at all. This is
    what separates "firing works" from "the shot counter is broken" -- the
    counter watches the game DLL's own attack timers, not the plugin's buttons,
    so a stuck counter would otherwise read as success.

    DUEL: crossbot_fire 1. Poll until shots land on real targets. Deaths, weapon
    variety, weapon switches and reloads are reported rather than asserted:
    every one of them depends on which guns happen to spawn near which bot.

.PARAMETER Bots
    More bots means more encounters per second of wall clock. Four is enough to
    make crossfire produce a fight inside a minute; two can take several.
#>
param(
    [string]$Map = "crossfire",
    [int]$Port = 0,
    [int]$Bots = 4,
    [int]$ControlSeconds = 20,
    [int]$DuelSeconds = 180,
    [int]$PollSeconds = 2,
    # Run the whole duel window instead of stopping at the first confirmed
    # engagement. Deaths, reloads and the rarer suspicion sources need several
    # fights before they show up, and the pass condition needs only one.
    [switch]$Full,
    [string]$HLDSDir = "D:\projects\trash\crossbot\hlds"
)

$ErrorActionPreference = 'Stop'
$scriptDir = $PSScriptRoot
$repoRoot = Split-Path $scriptDir -Parent

if ($Port -eq 0) { $Port = 29115 + (Get-Random -Maximum 85) }

$mapName = [System.IO.Path]::GetFileNameWithoutExtension($Map)

function Invoke-Rcon {
    param([string]$Cmd)
    $out = try { & "$scriptDir\rcon.ps1" -Port $Port -Command $Cmd } catch { "" }
    if ($null -eq $out) { return "" }
    return ($out | Out-String)
}

# One poll of crossbot_fire_status, folded into $state. Shots are cumulative per
# life and reset to zero on respawn, so a drop is a death and the tally has to be
# banked before it is lost rather than differenced blindly.
function Update-FireState {
    param([hashtable]$state)

    $text = Invoke-Rcon "crossbot_fire_status"
    foreach ($line in ($text -split "`r?`n")) {
        if ($line -notmatch 'held=') { continue }
        if ($line -notmatch '^CrossBot: (\S+) held=(\S+) mode=(\S+) why=(\S+)') { continue }

        $name = $Matches[1]
        $held = $Matches[2]
        $why = $Matches[4]

        $shots = 0
        if ($line -match 'shots=(\d+)') { $shots = [int]$Matches[1] }
        $switches = 0
        if ($line -match 'switches=(\d+)') { $switches = [int]$Matches[1] }
        $reloads = 0
        if ($line -match 'reloads=(\d+)') { $reloads = [int]$Matches[1] }
        $target = '-'
        if ($line -match 'target=(\S+)') { $target = $Matches[1] }
        $clip = -1
        if ($line -match 'clip=(-?\d+)') { $clip = [int]$Matches[1] }
        $prim = -1
        if ($line -match 'prim=(-?\d+)') { $prim = [int]$Matches[1] }

        if (-not $state.Bots.ContainsKey($name)) {
            $state.Bots[$name] = @{
                Shots = 0; Switches = 0; Reloads = 0
                Total = 0; TotalSwitches = 0; TotalReloads = 0; Deaths = 0
                Held = '-'; Clip = -1; Prim = -1; Burned = 0
            }
        }
        $bot = $state.Bots[$name]

        # Every counter the plugin reports is per life -- respawn zeroes the
        # whole fire state -- so a drop is a death, and the running tally has to
        # be banked at that point rather than differenced across the gap.
        if ($shots -lt $bot.Shots -or $switches -lt $bot.Switches -or $reloads -lt $bot.Reloads) {
            $bot.Total += $bot.Shots
            $bot.TotalSwitches += $bot.Switches
            $bot.TotalReloads += $bot.Reloads
            $bot.Deaths++
        }
        else {
            $bot.Total += $shots - $bot.Shots
            $bot.TotalSwitches += $switches - $bot.Switches
            $bot.TotalReloads += $reloads - $bot.Reloads
        }
        $bot.Shots = $shots
        $bot.Switches = $switches
        $bot.Reloads = $reloads

        # Ammo actually consumed, read independently of anything the plugin
        # reports about itself. A clip only ever falls by firing -- reloading
        # raises it -- and a clipless weapon spends its reserve directly, so
        # one of the two is a true shot witness for every weapon in the game.
        if ($held -eq $bot.Held -and $held -ne '-') {
            if ($clip -ge 0 -and $bot.Clip -ge 0 -and $clip -lt $bot.Clip) {
                $bot.Burned += $bot.Clip - $clip
            }
            elseif ($clip -lt 0 -and $bot.Clip -lt 0 -and $prim -ge 0 -and $bot.Prim -ge 0 -and $prim -lt $bot.Prim) {
                $bot.Burned += $bot.Prim - $prim
            }
        }
        $bot.Held = $held
        $bot.Clip = $clip
        $bot.Prim = $prim

        if ($held -ne '-') { $state.Weapons[$held] = $true }
        if ($why -eq 'firing') { $state.Firing++ }
        if ($target -ne '-') { $state.Targets++ }
        if ($state.Reasons.ContainsKey($why)) { $state.Reasons[$why]++ } else { $state.Reasons[$why] = 1 }
    }
}

function New-FireState {
    return @{
        Bots = @{}
        Weapons = @{}
        Reasons = @{}
        Firing = 0
        Targets = 0
    }
}

function Get-TotalShots {
    param([hashtable]$state)
    $sum = 0
    foreach ($bot in $state.Bots.Values) { $sum += $bot.Total + $bot.Shots }
    return $sum
}

function Get-TotalBurned {
    param([hashtable]$state)
    $sum = 0
    foreach ($bot in $state.Bots.Values) { $sum += $bot.Burned }
    return $sum
}

$hldsMap = Join-Path $HLDSDir "valve\maps\$mapName.bsp"
if (-not (Test-Path $hldsMap)) { Write-Host "FAIL: no such map $hldsMap"; exit 1 }

$navDir = Join-Path $HLDSDir "valve\addons\crossbot\nav"
if (-not (Test-Path $navDir)) { New-Item -ItemType Directory -Path $navDir | Out-Null }
if (-not (Test-Path (Join-Path $navDir "$mapName.nav"))) {
    & (Join-Path $repoRoot "build\tools\navgen\navgen.exe") $hldsMap -o (Join-Path $navDir "$mapName.nav") 2>&1 |
        Select-Object -Last 3 | ForEach-Object { Write-Host "navgen: $_" }
}

$logFile = Join-Path $repoRoot "nav_debug\logs\${mapName}_fire.log"

Write-Host "Starting HLDS on port $Port (join with: connect 127.0.0.1:$Port)"
$startResult = & "$scriptDir\test_server_start.ps1" -Map $mapName -Port $Port -Commands @() `
    -ConfigName "_crossbot_fire_$mapName.cfg" -HLDSDir $HLDSDir -LogFile $logFile

if ($startResult -match "^EXITED") { Write-Host "FAIL: $startResult"; exit 1 }
$serverPid = [int]($startResult -replace "PID=", "")
Write-Host "Server PID=$serverPid"

$failures = @()

try {
    Invoke-Rcon "crossbot_gather 1" | Out-Null
    Invoke-Rcon "crossbot_sense 1" | Out-Null

    # Before the bots exist, not after. Several of them spawn inside each
    # other's line of sight, and the handful of milliseconds an rcon round trip
    # takes is long enough for the default crossbot_fire 1 to produce a few
    # shots that the control phase would then have to explain.
    Invoke-Rcon "crossbot_fire 0" | Out-Null

    for ($i = 1; $i -le $Bots; $i++) { Invoke-Rcon "crossbot_add Fire$i" | Out-Null }

    Write-Host "`n===== CONTROL: crossbot_fire 0, ${ControlSeconds}s ====="
    Write-Host "Bots roam and pick up guns. Nothing may fire."

    $control = New-FireState
    Update-FireState $control
    $deadline = (Get-Date).AddSeconds($ControlSeconds)
    while ((Get-Date) -lt $deadline) {
        Start-Sleep -Seconds $PollSeconds
        Update-FireState $control
    }

    $controlShots = Get-TotalShots $control
    $controlBurned = Get-TotalBurned $control
    Write-Host ("  shots={0} burned={1} weapons={2}" -f $controlShots, $controlBurned,
        (($control.Weapons.Keys | Sort-Object) -join ','))
    if ($controlShots -ne 0) { $failures += "CONTROL: $controlShots shot(s) fired with crossbot_fire 0" }
    if ($controlBurned -ne 0) { $failures += "CONTROL: $controlBurned round(s) of ammo spent with crossbot_fire 0" }
    if ($control.Firing -ne 0) { $failures += "CONTROL: $($control.Firing) firing decision(s) with crossbot_fire 0" }

    Write-Host "`n===== DUEL: crossbot_fire 1, up to ${DuelSeconds}s ====="
    Invoke-Rcon "crossbot_fire 1" | Out-Null

    $duel = New-FireState
    Update-FireState $duel
    $deadline = (Get-Date).AddSeconds($DuelSeconds)
    $senseSeen = @{}
    while ((Get-Date) -lt $deadline) {
        Start-Sleep -Seconds $PollSeconds
        Update-FireState $duel

        # crossbot_sense_status only ever reports the single freshest entry, and
        # footsteps make SOUND the freshest almost every frame. Sampling it four
        # times per poll is the only way the rarer sources are ever the newest
        # thing in the ring when the question is asked.
        for ($s = 0; $s -lt 4; $s++) {
            foreach ($line in ((Invoke-Rcon "crossbot_sense_status") -split "`r?`n")) {
                if ($line -match 'fresh=(\w+) ') { $senseSeen[$Matches[1]] = $true }
            }
            Start-Sleep -Milliseconds 150
        }

        if ($Full) { continue }

        $shots = Get-TotalShots $duel
        if ($shots -gt 0 -and (Get-TotalBurned $duel) -gt 0 -and $duel.Targets -gt 0 -and $duel.Firing -gt 0) { break }
    }

    $duelShots = Get-TotalShots $duel
    $duelBurned = Get-TotalBurned $duel
    $deaths = 0
    $switches = 0
    $reloads = 0
    foreach ($bot in $duel.Bots.Values) {
        $deaths += $bot.Deaths
        $switches += $bot.TotalSwitches + $bot.Switches
        $reloads += $bot.TotalReloads + $bot.Reloads
    }

    Write-Host ("  shots={0} burned={1} firing={2} targets={3} deaths={4} switches={5} reloads={6}" -f `
        $duelShots, $duelBurned, $duel.Firing, $duel.Targets, $deaths, $switches, $reloads)
    Write-Host ("  weapons held: {0}" -f (($duel.Weapons.Keys | Sort-Object) -join ','))
    Write-Host ("  reasons: {0}" -f (($duel.Reasons.Keys | Sort-Object | ForEach-Object { "$_=$($duel.Reasons[$_])" }) -join ' '))
    Write-Host ("  suspicion sources seen: {0}" -f (($senseSeen.Keys | Sort-Object) -join ','))

    if ($duel.Targets -eq 0) { $failures += "DUEL: no bot ever acquired a target" }
    if ($duel.Firing -eq 0) { $failures += "DUEL: no bot ever decided to fire" }
    if ($duelShots -eq 0) { $failures += "DUEL: no shot was observed leaving a weapon" }
    if ($duelBurned -eq 0) { $failures += "DUEL: no ammo was spent" }
}
finally {
    & "$scriptDir\test_server_stop.ps1" -Id $serverPid | Out-Null
}

Write-Host ""
if ($failures.Count -eq 0) {
    Write-Host "PASS"
    exit 0
}

foreach ($f in $failures) { Write-Host "FAIL: $f" }
exit 1
