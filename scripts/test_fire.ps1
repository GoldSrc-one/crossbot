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

    SELECT: one bot, alone on the map, handed four weapons. With no target the
    only weapon decision left is the out-of-combat one, and with no combat
    anywhere every weapon+mode pair is tied at zero seconds -- so the bot must
    hold onto what it has instead of rotating on every 3s tick. Rotation needs
    usage to rank and is therefore the duel's to prove. It runs before the other
    bots join because once four of them roam crossfire, a three-second gap
    without a sighting is the exception rather than the rule.

    CONTROL: crossbot_fire 0. Bots roam and must fire nothing at all. This is
    what separates "firing works" from "the shot counter is broken" -- the
    counter watches the game DLL's own attack timers, not the plugin's buttons,
    so a stuck counter would otherwise read as success.

    DUEL: crossbot_fire 1. Poll until shots land on real targets. Deaths, weapon
    variety, weapon switches and reloads are reported rather than asserted:
    every one of them depends on which guns happen to spawn near which bot.

    WEAPON: one pass per weapon that needs its own aim or trigger, each pinned
    with crossbot_fire_pin. Equipping alone is not enough and no amount of
    patience fixes it -- in a fight a bot only re-selects when the weapon in its
    hands stops working, and out of combat it rotates to whichever pair it has
    used least, which is not the one a phase wants. The pin makes every other
    weapon unusable, which is the only lever that names one. Separate passes
    are what make aim=lead,
    aim=homing, the zoom decision and the charge each attributable to the
    weapon that produced them.

.PARAMETER Bots
    More bots means more encounters per second of wall clock. Four is enough to
    make crossfire produce a fight inside a minute; two can take several.
#>
param(
    [string]$Map = "crossfire",
    [int]$Port = 0,
    [int]$Bots = 4,
    [int]$SelectSeconds = 20,
    [int]$ControlSeconds = 20,
    [int]$DuelSeconds = 180,
    [int]$ProjectileSeconds = 420,
    [int]$PollSeconds = 2,
    [float]$Quantum = -1,
    [float]$Impulse = -1,
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
        if ($line -notmatch '^CrossBot: (\S+) held=(\S+) zoom=(\d) mode=(\S+) why=(\S+)') { continue }

        $name = $Matches[1]
        $held = $Matches[2]
        $why = $Matches[5]

        if ($Matches[3] -eq '1') { $state.Zoomed++ }

        $shots = 0
        if ($line -match ' shots=(\d+)') { $shots = [int]$Matches[1] }
        $switches = 0
        if ($line -match 'switches=(\d+)') { $switches = [int]$Matches[1] }
        $reloads = 0
        if ($line -match 'reloads=(\d+)') { $reloads = [int]$Matches[1] }
        $emptySwitches = 0
        if ($line -match 'emptyswitch=(\d+)') { $emptySwitches = [int]$Matches[1] }
        $idleSwitches = 0
        if ($line -match 'idleswitch=(\d+)') { $idleSwitches = [int]$Matches[1] }
        $idleReloads = 0
        if ($line -match 'idlereload=(\d+)') { $idleReloads = [int]$Matches[1] }
        $leadShots = 0
        if ($line -match 'leadshots=(\d+)') { $leadShots = [int]$Matches[1] }
        $chargeShots = 0
        if ($line -match 'chargeshots=(\d+)') { $chargeShots = [int]$Matches[1] }
        $arcShots = 0
        if ($line -match 'arcshots=(\d+)') { $arcShots = [int]$Matches[1] }
        $throwShots = 0
        if ($line -match 'throwshots=(\d+)') { $throwShots = [int]$Matches[1] }
        $guides = 0
        if ($line -match 'guides=(\d+)') { $guides = [int]$Matches[1] }
        if ($line -match 'capped=(\d+)') { if ([int]$Matches[1] -gt 0) { $state.Capped++ } }
        if ($line -match 'uphill=(\d+)') { if ([int]$Matches[1] -gt 0) { $state.Uphill++ } }
        if ($line -match 'selfveto=(\d+)') { if ([int]$Matches[1] -gt 0) { $state.SelfVeto++ } }
        if ($line -match 'aim=(\w+)') { $state.Aims[$Matches[1]] = $true }
        $target = '-'
        if ($line -match 'target=(\S+)') { $target = $Matches[1] }
        $clip = -1
        if ($line -match 'clip=(-?\d+)') { $clip = [int]$Matches[1] }
        $prim = -1
        if ($line -match 'prim=(-?\d+)') { $prim = [int]$Matches[1] }

        # A bot met for the first time is entered at whatever it already reads,
        # not at zero. Every counter is per life and each phase starts a fresh
        # tally, so entering at zero would make the first poll of a phase import
        # everything the bot did in the phase before it.
        if (-not $state.Bots.ContainsKey($name)) {
            $state.Bots[$name] = @{
                Shots = $shots; Switches = $switches; Reloads = $reloads; Lead = $leadShots
                Charge = $chargeShots; BaseCharge = $chargeShots; TotalCharge = 0
                Arc = $arcShots; BaseArc = $arcShots; TotalArc = 0
                Throw = $throwShots; BaseThrow = $throwShots; TotalThrow = 0
                Guides = $guides; BaseGuides = $guides; TotalGuides = 0
                Base = $shots; BaseSwitches = $switches; BaseReloads = $reloads; BaseLead = $leadShots
                Total = 0; TotalSwitches = 0; TotalReloads = 0; TotalLead = 0; Deaths = 0
                Held = '-'; Clip = -1; Prim = -1; Burned = 0
                EmptySw = $emptySwitches; BaseEmptySw = $emptySwitches; TotalEmptySw = 0
                IdleSw = $idleSwitches; BaseIdleSw = $idleSwitches; TotalIdleSw = 0
                IdleRel = $idleReloads; BaseIdleRel = $idleReloads; TotalIdleRel = 0
                HeldSeen = @{}
            }
        }
        $bot = $state.Bots[$name]

        # Every counter the plugin reports is per life -- respawn zeroes the
        # whole fire state -- so a drop is a death, and the running tally has to
        # be banked at that point rather than differenced across the gap.
        if ($shots -lt $bot.Shots -or $switches -lt $bot.Switches -or $reloads -lt $bot.Reloads) {
            $bot.Total += $bot.Shots - $bot.Base
            $bot.TotalSwitches += $bot.Switches - $bot.BaseSwitches
            $bot.TotalReloads += $bot.Reloads - $bot.BaseReloads
            $bot.TotalLead += $bot.Lead - $bot.BaseLead
            $bot.TotalCharge += $bot.Charge - $bot.BaseCharge
            $bot.TotalArc += $bot.Arc - $bot.BaseArc
            $bot.TotalThrow += $bot.Throw - $bot.BaseThrow
            $bot.TotalGuides += $bot.Guides - $bot.BaseGuides
            $bot.TotalEmptySw += $bot.EmptySw - $bot.BaseEmptySw
            $bot.TotalIdleSw += $bot.IdleSw - $bot.BaseIdleSw
            $bot.TotalIdleRel += $bot.IdleRel - $bot.BaseIdleRel
            $bot.Base = 0; $bot.BaseSwitches = 0; $bot.BaseReloads = 0; $bot.BaseLead = 0
            $bot.BaseCharge = 0; $bot.BaseArc = 0; $bot.BaseThrow = 0; $bot.BaseGuides = 0
            $bot.BaseEmptySw = 0; $bot.BaseIdleSw = 0; $bot.BaseIdleRel = 0
            $bot.Deaths++
        }
        else {
            $bot.Total += $shots - $bot.Shots
            $bot.TotalSwitches += $switches - $bot.Switches
            $bot.TotalReloads += $reloads - $bot.Reloads
            $bot.TotalLead += $leadShots - $bot.Lead
            $bot.TotalCharge += $chargeShots - $bot.Charge
            $bot.TotalArc += $arcShots - $bot.Arc
            $bot.TotalThrow += $throwShots - $bot.Throw
            $bot.TotalGuides += $guides - $bot.Guides
            $bot.TotalEmptySw += $emptySwitches - $bot.EmptySw
            $bot.TotalIdleSw += $idleSwitches - $bot.IdleSw
            $bot.TotalIdleRel += $idleReloads - $bot.IdleRel
        }
        $bot.EmptySw = $emptySwitches
        $bot.IdleSw = $idleSwitches
        $bot.IdleRel = $idleReloads
        $bot.Shots = $shots
        $bot.Switches = $switches
        $bot.Reloads = $reloads
        $bot.Lead = $leadShots
        $bot.Charge = $chargeShots
        $bot.Arc = $arcShots
        $bot.Throw = $throwShots
        $bot.Guides = $guides

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
        if ($held -ne '-') { $bot.HeldSeen[$held] = $true }

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
        Aims = @{}
        Firing = 0
        Targets = 0
        Zoomed = 0
        Uphill = 0
        SelfVeto = 0
        Capped = 0
    }
}

function Get-TotalArc {
    param([hashtable]$state)
    $sum = 0
    foreach ($bot in $state.Bots.Values) { $sum += $bot.TotalArc + $bot.Arc - $bot.BaseArc }
    return $sum
}

function Get-TotalThrow {
    param([hashtable]$state)
    $sum = 0
    foreach ($bot in $state.Bots.Values) { $sum += $bot.TotalThrow + $bot.Throw - $bot.BaseThrow }
    return $sum
}

function Get-TotalGuides {
    param([hashtable]$state)
    $sum = 0
    foreach ($bot in $state.Bots.Values) { $sum += $bot.TotalGuides + $bot.Guides - $bot.BaseGuides }
    return $sum
}

function Get-TotalLead {
    param([hashtable]$state)
    $sum = 0
    foreach ($bot in $state.Bots.Values) { $sum += $bot.TotalLead + $bot.Lead - $bot.BaseLead }
    return $sum
}

function Get-TotalCharge {
    param([hashtable]$state)
    $sum = 0
    foreach ($bot in $state.Bots.Values) { $sum += $bot.TotalCharge + $bot.Charge - $bot.BaseCharge }
    return $sum
}

function Get-TotalShots {
    param([hashtable]$state)
    $sum = 0
    foreach ($bot in $state.Bots.Values) { $sum += $bot.Total + $bot.Shots - $bot.Base }
    return $sum
}

function Get-TotalBurned {
    param([hashtable]$state)
    $sum = 0
    foreach ($bot in $state.Bots.Values) { $sum += $bot.Burned }
    return $sum
}

function Get-TotalEmptySwitch {
    param([hashtable]$state)
    $sum = 0
    foreach ($bot in $state.Bots.Values) { $sum += $bot.TotalEmptySw + $bot.EmptySw - $bot.BaseEmptySw }
    return $sum
}

function Get-TotalIdleSwitch {
    param([hashtable]$state)
    $sum = 0
    foreach ($bot in $state.Bots.Values) { $sum += $bot.TotalIdleSw + $bot.IdleSw - $bot.BaseIdleSw }
    return $sum
}

function Get-TotalIdleReload {
    param([hashtable]$state)
    $sum = 0
    foreach ($bot in $state.Bots.Values) { $sum += $bot.TotalIdleRel + $bot.IdleRel - $bot.BaseIdleRel }
    return $sum
}

# The usage table is the one thing the plugin reports that does not reset on
# respawn, so it needs no banking -- the largest pair count any bot reaches is
# the whole measurement.
function Get-UsagePairs {
    $best = 0
    $detail = @()
    foreach ($line in ((Invoke-Rcon "crossbot_weapon_usage") -split "`r?`n")) {
        if ($line -notmatch '^CrossBot: (\S+) held=(\S+)') { continue }
        $name = $Matches[1]
        $pairs = 0
        if ($line -match 'pairs=(\d+)') { $pairs = [int]$Matches[1] }
        if ($pairs -gt $best) { $best = $pairs }
        if ($pairs -gt 0) { $detail += "${name}:$pairs" }
    }
    return @{ Pairs = $best; Detail = ($detail -join ' ') }
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
    if ($Quantum -ge 0) { Invoke-Rcon "crossbot_aim_quantum $Quantum" | Out-Null }
    if ($Impulse -ge 0) { Invoke-Rcon "crossbot_aim_impulse $Impulse" | Out-Null }

    Invoke-Rcon "crossbot_add Fire1" | Out-Null

    # One bot, so there is never a target and every decision this phase sees is
    # the out-of-combat one. It runs before the others join because that is the
    # only cheap way to guarantee an empty combat memory for twenty seconds --
    # once four bots are roaming crossfire, three-second gaps in sight are the
    # exception rather than the rule.
    #
    # This measures the tie rule rather than rotation. With no combat anywhere,
    # every weapon+mode pair sits at zero seconds, the held weapon wins every
    # tie and the bot must therefore stay put -- a rotation that ignored ties
    # would switch on every 3s tick and show up here as six or seven. Rotation
    # itself needs usage to rank, so its witness is the duel.
    Write-Host "`n===== SELECT: one bot, no targets, ${SelectSeconds}s ====="
    Write-Host "Idle ticks must not churn the weapon when nothing has been used."

    Invoke-Rcon "crossbot_fire 1" | Out-Null
    foreach ($give in @('weapon_shotgun', 'weapon_crossbow', 'weapon_9mmAR', 'weapon_hornetgun')) {
        Invoke-Rcon "crossbot_equip $give Fire1" | Out-Null
    }

    $select = New-FireState
    Update-FireState $select
    $deadline = (Get-Date).AddSeconds($SelectSeconds)
    while ((Get-Date) -lt $deadline) {
        Start-Sleep -Seconds $PollSeconds
        Update-FireState $select
    }

    $selectHeld = @{}
    foreach ($bot in $select.Bots.Values) {
        foreach ($k in $bot.HeldSeen.Keys) { $selectHeld[$k] = $true }
    }
    $selectIdleSw = Get-TotalIdleSwitch $select
    $selectIdleRel = Get-TotalIdleReload $select
    $selectShots = Get-TotalShots $select

    # A weapon picked up off the map can arrive with an empty clip -- the RPG
    # always does -- and loading it is a legitimate idle switch, so the budget is
    # a handful rather than zero. A tie rule that failed would produce one every
    # 3s tick for the whole phase.
    $churnBudget = [int]($SelectSeconds / 6)

    Write-Host ("  idleswitch={0} idlereload={1} shots={2} held: {3}" -f $selectIdleSw, $selectIdleRel,
        $selectShots, (($selectHeld.Keys | Sort-Object) -join ','))

    if ($selectIdleSw -gt $churnBudget) {
        $failures += "SELECT: $selectIdleSw out-of-combat switch(es) with nothing used yet, so the held weapon is losing ties"
    }
    if ($selectShots -ne 0) { $failures += "SELECT: $selectShots shot(s) fired with no target on the map" }

    Invoke-Rcon "crossbot_fire 0" | Out-Null
    for ($i = 2; $i -le $Bots; $i++) { Invoke-Rcon "crossbot_add Fire$i" | Out-Null }

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

        # The suspicion list holds one entry per entity, so an enemy's DAMAGE entry
        # is overwritten by their own SIGHT entry inside the same BotSense::Update
        # and fresh= would essentially never report it. The pushed= counters are
        # cumulative, so a single poll cannot miss a source that fired between polls.
        foreach ($line in ((Invoke-Rcon "crossbot_sense_status") -split "`r?`n")) {
            if ($line -notmatch 'pushed=sgt:(\d+),snd:(\d+),evt:(\d+),dmg:(\d+)') { continue }
            foreach ($p in @(@('SIGHT', 1), @('SOUND', 2), @('EVENT', 3), @('DAMAGE', 4))) {
                if ([int]$Matches[$p[1]] -gt 0) { $senseSeen[$p[0]] = $true }
            }
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
        $switches += $bot.TotalSwitches + $bot.Switches - $bot.BaseSwitches
        $reloads += $bot.TotalReloads + $bot.Reloads - $bot.BaseReloads
    }

    $duelEmptySw = Get-TotalEmptySwitch $duel
    $duelIdleSw = Get-TotalIdleSwitch $duel
    $duelIdleRel = Get-TotalIdleReload $duel
    $usage = Get-UsagePairs

    Write-Host ("  shots={0} burned={1} firing={2} targets={3} deaths={4} switches={5} reloads={6}" -f `
        $duelShots, $duelBurned, $duel.Firing, $duel.Targets, $deaths, $switches, $reloads)
    Write-Host ("  emptyswitch={0} idleswitch={1} idlereload={2} usage pairs: {3}" -f `
        $duelEmptySw, $duelIdleSw, $duelIdleRel, $usage.Detail)
    Write-Host ("  weapons held: {0}" -f (($duel.Weapons.Keys | Sort-Object) -join ','))
    Write-Host ("  reasons: {0}" -f (($duel.Reasons.Keys | Sort-Object | ForEach-Object { "$_=$($duel.Reasons[$_])" }) -join ' '))
    Write-Host ("  suspicion sources seen: {0}" -f (($senseSeen.Keys | Sort-Object) -join ','))

    foreach ($src in @('SIGHT', 'SOUND', 'EVENT', 'DAMAGE')) {
        if (-not $senseSeen[$src]) { $failures += "DUEL: suspicion source $src never fired" }
    }
    if ($duel.Targets -eq 0) { $failures += "DUEL: no bot ever acquired a target" }
    if ($duel.Firing -eq 0) { $failures += "DUEL: no bot ever decided to fire" }
    if ($duelShots -eq 0) { $failures += "DUEL: no shot was observed leaving a weapon" }
    if ($duelBurned -eq 0) { $failures += "DUEL: no ammo was spent" }

    # Only under -Full. Each of these needs a bot to run a clip dry with a loaded
    # spare in the bag, or to survive three seconds unseen with a part-empty
    # magazine, and which guns spawn near which bot decides whether that happens
    # at all inside the short window.
    if ($Full) {
        if ($usage.Pairs -lt 2) {
            $failures += "DUEL: no bot accumulated time on more than one weapon+mode pair"
        }
        if ($duelIdleRel -eq 0) { $failures += "DUEL: no bot ever reloaded out of combat" }
        if ($duelIdleSw -eq 0) { $failures += "DUEL: no bot ever rotated weapons between fights" }
    }

    $projFailures = @()

    # The pin is per weapon and, for the gauss, per mode. Weapon stickiness has
    # a second storey: the random pick names a weapon+mode pair but only the
    # weapon id survives the switch, so the frame after a select the mode order
    # starts at primary again. A gauss whose primary is usable therefore never
    # reaches FIRE_CHARGE on its own, exactly as a bot holding a glock never
    # reaches a crossbow.
    $pins = @(
        @{ Weapon = 'weapon_crossbow';  Mode = '' }
        @{ Weapon = 'weapon_hornetgun'; Mode = '' }
        @{ Weapon = 'weapon_snark';     Mode = '' }
        @{ Weapon = 'weapon_gauss';     Mode = 'secondary' }
        # The MP5's own give is clip ammo only, so the contact grenade needs its
        # ammo entity equipped alongside the weapon or the mode is never usable.
        @{ Weapon = 'weapon_9mmAR';     Mode = 'secondary'; Ammo = 'ammo_ARgrenades' }
        # The grenade's own give is five, and re-equipping every poll refills them.
        # Its reach is about 550 units, so this phase is the one that most depends
        # on the bots wandering close enough to each other.
        @{ Weapon = 'weapon_handgrenade'; Mode = '' }
        @{ Weapon = 'weapon_rpg';       Mode = ''; Ammo = 'ammo_rpgclip' }
    )

    $seconds = [int]($ProjectileSeconds / $pins.Count)
    if ($seconds -lt 15) { $seconds = 15 }

    foreach ($entry in $pins) {
        $pin = $entry.Weapon
        $mode = $entry.Mode
        Write-Host "`n===== WEAPON: $pin $mode, ${seconds}s ====="

        Invoke-Rcon ("crossbot_fire_pin $pin $mode").Trim() | Out-Null

        # Respawn every bot before the phase is measured. The pin reaches weapon
        # selection, and selection alone is not enough to isolate a phase: a bot
        # carrying a gauss charge keeps its mode without consulting ModeUsable,
        # by design, so a charge started in the previous phase survives the pin
        # and discharges into this one's counters. Dying drops the inventory,
        # the charge and every per-life counter at once.
        Invoke-Rcon "crossbot_kill" | Out-Null
        Start-Sleep -Seconds $PollSeconds

        $proj = New-FireState
        Update-FireState $proj
        $deadline = (Get-Date).AddSeconds($seconds)
        while ((Get-Date) -lt $deadline) {
            # Re-issued every poll rather than once. Respawning drops the whole
            # inventory, and a second pickup of a weapon already held is what
            # refills the five bolts a crossbow carries.
            Invoke-Rcon "crossbot_equip $pin" | Out-Null
            if ($entry.Ammo) { Invoke-Rcon "crossbot_equip $($entry.Ammo)" | Out-Null }

            Start-Sleep -Seconds $PollSeconds
            Update-FireState $proj
        }

        $shots = Get-TotalShots $proj
        $lead = Get-TotalLead $proj
        $charged = Get-TotalCharge $proj
        $arced = Get-TotalArc $proj
        $thrown = Get-TotalThrow $proj
        $guided = Get-TotalGuides $proj

        Write-Host ("  shots={0} leadshots={1} chargeshots={2} arcshots={3} throwshots={4} guides={5}" -f `
            $shots, $lead, $charged, $arced, $thrown, $guided)
        Write-Host ("  firing={0} zoomed={1} uphill={2} selfveto={3} capped={4}" -f `
            $proj.Firing, $proj.Zoomed, $proj.Uphill, $proj.SelfVeto, $proj.Capped)
        Write-Host ("  weapons held: {0}" -f (($proj.Weapons.Keys | Sort-Object) -join ','))
        Write-Host ("  aim kinds seen: {0}" -f (($proj.Aims.Keys | Sort-Object) -join ','))
        Write-Host ("  reasons: {0}" -f (($proj.Reasons.Keys | Sort-Object | ForEach-Object { "$_=$($proj.Reasons[$_])" }) -join ' '))

        if ($shots -eq 0) { $projFailures += "WEAPON ${pin}: nothing was ever fired" }

        switch ($pin) {
            'weapon_crossbow' {
                # Zoomed primary is FireSniperBolt, a hitscan trace, so a pinned
                # crossbow has to show both aims: lead below 300 units, target
                # once the range decision has zoomed it.
                if ($proj.Zoomed -eq 0) { $projFailures += "WEAPON ${pin}: the bot never zoomed" }
                if (-not $proj.Aims.ContainsKey('target')) {
                    $projFailures += "WEAPON ${pin}: aim=target never appeared, so primaryZoomed was never selected"
                }
                if (-not $proj.Reasons.ContainsKey('off-target')) {
                    $projFailures += "WEAPON ${pin}: no shot was ever withheld"
                }
            }
            'weapon_hornetgun' {
                # Primary is the homing cone, secondary's dart is genuinely led.
                if (-not $proj.Aims.ContainsKey('homing')) { $projFailures += "WEAPON ${pin}: aim=homing never appeared" }
                if ($lead -ge $shots) {
                    $projFailures += "WEAPON ${pin}: every shot was a solved lead, so the cone never replaced the solve"
                }
            }
            'weapon_snark' {
                if (-not $proj.Aims.ContainsKey('homing')) { $projFailures += "WEAPON ${pin}: aim=homing never appeared" }
                if ($lead -ne 0) { $projFailures += "WEAPON ${pin}: $lead shot(s) were led, and a thrown snark is never led" }
                if ($proj.Uphill -eq 0) {
                    $projFailures += "WEAPON ${pin}: the uphill veto never fired, so it is not running"
                }
            }
            'weapon_gauss' {
                # charged means a bot reached full charge and waited for the
                # shot instead of releasing on a timer, which is the whole
                # difference between a charge and a slow trigger.
                if ($charged -eq 0) { $projFailures += "WEAPON ${pin}: no charged shot was ever observed" }
                if (-not $proj.Reasons.ContainsKey('charged')) {
                    $projFailures += "WEAPON ${pin}: no bot ever carried a full charge"
                }
                if ($proj.Reasons.ContainsKey('charge-timeout')) {
                    $projFailures += "WEAPON ${pin}: a charge ran to the safety deadline"
                }
            }
            'weapon_handgrenade' {
                # The pin pull and the throw are half a second apart, and the bot
                # never sees its own grenade leave -- m_flStartThrow going back to
                # zero is what closes the commit, so a throw count that moves is
                # the proof the field is being read rather than assumed.
                if ($thrown -eq 0) { $projFailures += "WEAPON ${pin}: no grenade was ever thrown" }
                if (-not $proj.Aims.ContainsKey('arc')) { $projFailures += "WEAPON ${pin}: aim=arc never appeared" }
                if (-not $proj.Reasons.ContainsKey('primed')) {
                    $projFailures += "WEAPON ${pin}: no bot ever held a pulled pin, so the commit is not running"
                }
                # selfveto is the cumulative counter, so a single refusal anywhere in
                # the phase reaches the poll. The reason strings are a two-second
                # sample of one frame and miss most of what they describe, so they
                # widen this test rather than carrying it.
                $refused = $proj.SelfVeto -gt 0 -or $proj.Reasons.ContainsKey('arc-self') -or
                    $proj.Reasons.ContainsKey('arc-blocked') -or $proj.Reasons.ContainsKey('arc-wide')
                if (-not $refused) {
                    $projFailures += "WEAPON ${pin}: no throw was ever withheld, so the impact gates are not running"
                }
                if ($proj.Capped -gt 0) {
                    $projFailures += "WEAPON ${pin}: a pulled pin ran to the safety cap, so a press never reached the weapon"
                }
            }
            'weapon_rpg' {
                # The rocket is tracked, never led: the laser spot sits under the
                # crosshair, so leading it would steer the rocket off the target.
                if ($guided -eq 0) { $projFailures += "WEAPON ${pin}: no launch ever opened a guide window" }
                if (-not $proj.Reasons.ContainsKey('guiding')) {
                    $projFailures += "WEAPON ${pin}: why=guiding never appeared, so KEEP_AIMING is not holding the bot"
                }
                if ($proj.Aims.ContainsKey('arc') -or $proj.Aims.ContainsKey('lead')) {
                    $projFailures += "WEAPON ${pin}: aim was $(($proj.Aims.Keys | Sort-Object) -join ','), and a guided rocket is aimed at the target"
                }
                # Every window has to close on the rocket rather than on the six
                # second valve. Reaching the valve means m_cActiveRockets never came
                # back to zero, which the SDK does leak on the self-detonation path,
                # and it would pin the bot to a launcher it can neither fire nor
                # reload. Same standing as charge-timeout on the gauss.
                if ($proj.Capped -gt 0) {
                    $projFailures += "WEAPON ${pin}: a guide window ran to the safety cap"
                }
            }
            'weapon_9mmAR' {
                # The contact grenade is the only plain parabola in the game, so
                # it is the only witness the elevation solve and the traced
                # flight sim have.
                if (-not $proj.Aims.ContainsKey('arc')) { $projFailures += "WEAPON ${pin}: aim=arc never appeared" }
                if ($arced -ne $shots) {
                    $projFailures += "WEAPON ${pin}: $arced of $shots shot(s) went through the arc path"
                }
                # A gate that never refuses is a gate that is not running, and
                # selfveto is the only witness that survives a two-second poll:
                # the reason strings sample one frame and usually miss.
                $refused = $proj.SelfVeto -gt 0 -or $proj.Reasons.ContainsKey('arc-self') -or
                    $proj.Reasons.ContainsKey('arc-blocked') -or $proj.Reasons.ContainsKey('arc-wide')
                if (-not $refused) {
                    $projFailures += "WEAPON ${pin}: no shot was ever withheld, so the impact gates are not running"
                }
            }
        }
    }

    Invoke-Rcon "crossbot_fire_pin off" | Out-Null
    $failures += $projFailures
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
