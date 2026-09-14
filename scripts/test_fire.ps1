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
    [switch]$Full,
    [string]$HLDSDir = "D:\hlds"
)

$ErrorActionPreference = 'Stop'
$scriptDir = $PSScriptRoot
$repoRoot = Split-Path $scriptDir -Parent

if ($Port -eq 0) { $Port = 29115 + (Get-Random -Maximum 85) }

$mapName = [System.IO.Path]::GetFileNameWithoutExtension($Map)

. "$scriptDir\common.ps1"

function Update-FireState {
    param([hashtable]$state)

    $text = Invoke-CbRcon $Port "crossbot_fire_status"
    foreach ($line in ($text -split "`r?`n")) {
        if ($line -notmatch 'held=') { continue }
        if ($line -notmatch '^CrossBot: (\S+) held=(\S+) zoom=(\d) mode=(\S+) why=(\S+)') { continue }

        $name = $Matches[1]
        $held = $Matches[2]
        $why = $Matches[5]

        if ($Matches[3] -eq '1') { $state.Zoomed++ }

        $n = @{}
        foreach ($f in @('shots', 'switches', 'reloads', 'emptyswitch', 'idleswitch', 'idlereload',
                'leadshots', 'chargeshots', 'arcshots', 'throwshots', 'stickyshots', 'guides')) {
            $n[$f] = 0
            if ($line -match " $f=(\d+)") { $n[$f] = [int]$Matches[1] }
        }
        $shots, $switches, $reloads, $emptySwitches, $idleSwitches, $idleReloads,
            $leadShots, $chargeShots, $arcShots, $throwShots, $stickyShots, $guides =
            $n['shots'], $n['switches'], $n['reloads'], $n['emptyswitch'], $n['idleswitch'], $n['idlereload'],
            $n['leadshots'], $n['chargeshots'], $n['arcshots'], $n['throwshots'], $n['stickyshots'], $n['guides']
        if ($line -match 'capped=(\d+)') { if ([int]$Matches[1] -gt 0) { $state.Capped++ } }
        if ($line -match 'uphill=(\d+)') { if ([int]$Matches[1] -gt 0) { $state.Uphill++ } }
        if ($line -match 'selfveto=(\d+)') { if ([int]$Matches[1] -gt 0) { $state.SelfVeto++ } }
        if ($line -match 'aimveto=(\d+)') { if ([int]$Matches[1] -gt 0) { $state.AimVeto++ } }
        if ($line -match 'targetaims=(\d+)') { if ([int]$Matches[1] -gt 0) { $state.TargetAims++ } }
        if ($line -match 'aim=(\w+)') { $state.Aims[$Matches[1]] = $true }
        $target = '-'
        if ($line -match 'target=(\S+)') { $target = $Matches[1] }
        $clip = -1
        if ($line -match 'clip=(-?\d+)') { $clip = [int]$Matches[1] }
        $prim = -1
        if ($line -match 'prim=(-?\d+)') { $prim = [int]$Matches[1] }

        if (-not $state.Bots.ContainsKey($name)) {
            $state.Bots[$name] = @{
                Shots = $shots; Switches = $switches; Reloads = $reloads; Lead = $leadShots
                Charge = $chargeShots; BaseCharge = $chargeShots; TotalCharge = 0
                Arc = $arcShots; BaseArc = $arcShots; TotalArc = 0
                Throw = $throwShots; BaseThrow = $throwShots; TotalThrow = 0
                Sticky = $stickyShots; BaseSticky = $stickyShots; TotalSticky = 0
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

        $died = ($shots -lt $bot.Shots -or $switches -lt $bot.Switches -or $reloads -lt $bot.Reloads)
        foreach ($t in @( @('Total', 'Shots', 'Base', $shots),
                @('TotalSwitches', 'Switches', 'BaseSwitches', $switches),
                @('TotalReloads', 'Reloads', 'BaseReloads', $reloads),
                @('TotalLead', 'Lead', 'BaseLead', $leadShots),
                @('TotalCharge', 'Charge', 'BaseCharge', $chargeShots),
                @('TotalArc', 'Arc', 'BaseArc', $arcShots),
                @('TotalThrow', 'Throw', 'BaseThrow', $throwShots),
                @('TotalSticky', 'Sticky', 'BaseSticky', $stickyShots),
                @('TotalGuides', 'Guides', 'BaseGuides', $guides),
                @('TotalEmptySw', 'EmptySw', 'BaseEmptySw', $emptySwitches),
                @('TotalIdleSw', 'IdleSw', 'BaseIdleSw', $idleSwitches),
                @('TotalIdleRel', 'IdleRel', 'BaseIdleRel', $idleReloads) ) ) {
            if ($died) { $bot[$t[0]] += $bot[$t[1]] - $bot[$t[2]]; $bot[$t[2]] = 0 }
            else { $bot[$t[0]] += $t[3] - $bot[$t[1]] }
        }
        if ($died) { $bot.Deaths++ }
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
        $bot.Sticky = $stickyShots
        $bot.Guides = $guides

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
        AimVeto = 0
        TargetAims = 0
        Capped = 0
    }
}

function Get-TotalField {
    param([hashtable]$state, [string]$total, [string]$current, [string]$base)
    $sum = 0
    foreach ($bot in $state.Bots.Values) { $sum += $bot[$total] + $bot[$current] - $bot[$base] }
    return $sum
}

function Get-TotalArc { param([hashtable]$state) Get-TotalField $state 'TotalArc' 'Arc' 'BaseArc' }
function Get-TotalSticky { param([hashtable]$state) Get-TotalField $state 'TotalSticky' 'Sticky' 'BaseSticky' }
function Get-TotalThrow { param([hashtable]$state) Get-TotalField $state 'TotalThrow' 'Throw' 'BaseThrow' }
function Get-TotalGuides { param([hashtable]$state) Get-TotalField $state 'TotalGuides' 'Guides' 'BaseGuides' }
function Get-TotalLead { param([hashtable]$state) Get-TotalField $state 'TotalLead' 'Lead' 'BaseLead' }
function Get-TotalCharge { param([hashtable]$state) Get-TotalField $state 'TotalCharge' 'Charge' 'BaseCharge' }
function Get-TotalShots { param([hashtable]$state) Get-TotalField $state 'Total' 'Shots' 'Base' }
function Get-TotalBurned { param([hashtable]$state) Get-TotalField $state 'Burned' 'Burned' 'Burned' }
function Get-TotalEmptySwitch { param([hashtable]$state) Get-TotalField $state 'TotalEmptySw' 'EmptySw' 'BaseEmptySw' }
function Get-TotalIdleSwitch { param([hashtable]$state) Get-TotalField $state 'TotalIdleSw' 'IdleSw' 'BaseIdleSw' }
function Get-TotalIdleReload { param([hashtable]$state) Get-TotalField $state 'TotalIdleRel' 'IdleRel' 'BaseIdleRel' }

function Get-CarryLines {
    return ((Invoke-CbRcon $Port "crossbot_weapon_carry") -split "`r?`n")
}

function Get-CarrySpread {
    $best = 0
    $detail = @()
    foreach ($line in (Get-CarryLines)) {
        if ($line -notmatch '^CrossBot: (\S+) held=(\S+)') { continue }
        $name = $Matches[1]
        $seen = @{}
        foreach ($m in ([regex]'weapon_\S+?=([\d.]+)/[\d.]+').Matches($line)) {
            $seen[$m.Groups[1].Value] = $true
        }
        $distinct = $seen.Count
        if ($distinct -gt $best) { $best = $distinct }
        if ($distinct -gt 0) { $detail += "${name}:$distinct" }
    }
    return @{ Distinct = $best; Detail = ($detail -join ' ') }
}

function Get-CarryTable {
    $table = @{}
    foreach ($line in (Get-CarryLines)) {
        if ($line -notmatch '^CrossBot: (\S+) held=(\S+)') { continue }
        foreach ($m in ([regex]'(weapon_\S+?)=([\d.]+)/([\d.]+)').Matches($line)) {
            $table[$m.Groups[1].Value] = @{
                Carry = [double]$m.Groups[2].Value
                Score = [double]$m.Groups[3].Value
            }
        }
        return $table
    }
    return $table
}

function Test-SelectionRule {
    param([switch]$RequireRarest)

    $held = @(Get-HeldWeapons)[0]
    $table = Get-CarryTable

    if (-not $held -or $held -eq '-') { return "nothing held" }
    if ($table.Count -eq 0) { return "the carry table is empty" }
    if (-not $table.ContainsKey($held)) { return "held $held is absent from the carry table" }

    $max = ($table.Values | ForEach-Object { $_.Score } | Measure-Object -Maximum).Maximum
    if ($table[$held].Score -lt $max - 0.001) {
        return "held $held scores $($table[$held].Score) while the bag holds a $max"
    }

    if ($RequireRarest) {
        $rarest = ($table.Values | Where-Object { $_.Score -ge $max - 0.001 } |
            ForEach-Object { $_.Carry } | Measure-Object -Minimum).Minimum
        if ($table[$held].Carry -gt $rarest + 0.15) {
            return "held $held has been carried $($table[$held].Carry)s while a weapon of the same score has only $rarest s"
        }
    }

    return $null
}

function Wait-SelectionRule {
    param([switch]$RequireRarest, [int]$Tries = 5)

    $why = $null
    for ($i = 0; $i -lt $Tries; $i++) {
        $why = Test-SelectionRule -RequireRarest:$RequireRarest
        if (-not $why) { return $null }
        Start-Sleep -Seconds 3
    }
    return $why
}

function Get-HeldWeapons {
    $held = @()
    foreach ($line in ((Invoke-CbRcon $Port "crossbot_fire_status") -split "`r?`n")) {
        if ($line -match '^CrossBot: (\S+) held=(\S+)') { $held += $Matches[2] }
    }
    return $held
}

$hldsMap = Join-Path $HLDSDir "valve\maps\$mapName.bsp"
if (-not (Test-Path $hldsMap)) { Write-Host "FAIL: no such map $hldsMap"; exit 1 }

Ensure-CbNav -HldsMap $hldsMap -NavPath (Join-Path $HLDSDir "crossplay\addons\crossbot\nav\$mapName.nav") `
    -NavGenExe (Join-Path $repoRoot "build\tools\navgen\navgen.exe")

$logFile = Join-Path $repoRoot "nav_debug\logs\${mapName}_fire.log"

Write-Host "Starting HLDS on port $Port (join with: connect 127.0.0.1:$Port)"
$startResult = & "$scriptDir\test_server_start.ps1" -Map $mapName -Port $Port -Commands @() `
    -ConfigName "_crossbot_fire_$mapName.cfg" -HLDSDir $HLDSDir -LogFile $logFile

$serverPid = Get-CbServerPid $startResult
Write-Host "Server PID=$serverPid"

$failures = @()

try {
    Invoke-CbRcon $Port "crossbot_gather 1" | Out-Null
    Invoke-CbRcon $Port "crossbot_sense 1" | Out-Null

    Invoke-CbRcon $Port "crossbot_fire 0" | Out-Null
    if ($Quantum -ge 0) { Invoke-CbRcon $Port "crossbot_aim_quantum $Quantum" | Out-Null }
    if ($Impulse -ge 0) { Invoke-CbRcon $Port "crossbot_aim_impulse $Impulse" | Out-Null }

    Invoke-CbRcon $Port "crossbot_add valve Fire1" | Out-Null

    Write-Host "`n===== SELECT: one bot, no targets, ${SelectSeconds}s ====="
    Write-Host "Score picks the band; carry breaks the tie inside it."

    Invoke-CbRcon $Port "crossbot_fire 1" | Out-Null
    foreach ($give in @('weapon_shotgun', 'weapon_crossbow', 'weapon_9mmAR', 'weapon_hornetgun')) {
        Invoke-CbRcon $Port "crossbot_equip $give Fire1" | Out-Null
    }

    $select = New-FireState
    Update-FireState $select
    $deadline = (Get-Date).AddSeconds($SelectSeconds)
    while ((Get-Date) -lt $deadline) {
        Start-Sleep -Seconds $PollSeconds
        Update-FireState $select
    }

    $scoreHeld = @(Get-HeldWeapons)[0]
    $scoreWhy = Wait-SelectionRule

    Invoke-CbRcon $Port "crossbot_equip weapon_gauss Fire1" | Out-Null

    $deadline = (Get-Date).AddSeconds($SelectSeconds)
    while ((Get-Date) -lt $deadline) {
        Start-Sleep -Seconds $PollSeconds
        Update-FireState $select
    }

    $carryHeld = @(Get-HeldWeapons)[0]
    $carryWhy = Wait-SelectionRule -RequireRarest

    $selectHeld = @{}
    foreach ($bot in $select.Bots.Values) {
        foreach ($k in $bot.HeldSeen.Keys) { $selectHeld[$k] = $true }
    }
    $selectIdleSw = Get-TotalIdleSwitch $select
    $selectIdleRel = Get-TotalIdleReload $select
    $selectShots = Get-TotalShots $select
    $selectCarry = Get-CarrySpread

    $churnBudget = 4 + [int]($SelectSeconds / 5)

    Write-Host ("  idleswitch={0} idlereload={1} shots={2} scoreheld={3} carryheld={4}" -f `
        $selectIdleSw, $selectIdleRel, $selectShots, $scoreHeld, $carryHeld)
    Write-Host ("  held seen: {0}" -f (($selectHeld.Keys | Sort-Object) -join ','))
    Write-Host ("  carry spread: {0}" -f $selectCarry.Detail)

    if ($scoreWhy) { $failures += "SELECT: the score filter is not deciding -- $scoreWhy" }
    if ($carryWhy) { $failures += "SELECT: rarity is not breaking score ties -- $carryWhy" }
    if ($selectIdleSw -gt $churnBudget) {
        $failures += "SELECT: $selectIdleSw out-of-combat switch(es), so the bot is churning instead of settling on its best weapon"
    }
    if ($selectShots -ne 0) { $failures += "SELECT: $selectShots shot(s) fired with no target on the map" }

    Invoke-CbRcon $Port "crossbot_fire 0" | Out-Null
    for ($i = 2; $i -le $Bots; $i++) { Invoke-CbRcon $Port "crossbot_add valve Fire$i" | Out-Null }

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
    Invoke-CbRcon $Port "crossbot_fire 1" | Out-Null

    $duel = New-FireState
    Update-FireState $duel
    $deadline = (Get-Date).AddSeconds($DuelSeconds)
    $senseSeen = @{}
    while ((Get-Date) -lt $deadline) {
        Start-Sleep -Seconds $PollSeconds
        Update-FireState $duel

        foreach ($line in ((Invoke-CbRcon $Port "crossbot_sense_status") -split "`r?`n")) {
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
    $duelCarry = Get-CarrySpread
    $duelSticky = Get-TotalSticky $duel

    Write-Host ("  shots={0} burned={1} firing={2} targets={3} deaths={4} switches={5} reloads={6}" -f `
        $duelShots, $duelBurned, $duel.Firing, $duel.Targets, $deaths, $switches, $reloads)
    Write-Host ("  emptyswitch={0} idleswitch={1} idlereload={2} stickyshots={3}" -f `
        $duelEmptySw, $duelIdleSw, $duelIdleRel, $duelSticky)
    Write-Host ("  carry spread: {0}" -f $duelCarry.Detail)
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
    if ($duelSticky -eq 0) {
        $failures += "DUEL: no shot was ever held through an off-target frame, so the sticky trigger is not running"
    }

    if ($Full) {
        if ($duelCarry.Distinct -lt 2) {
            $failures += "DUEL: no bot reported two carry times that differ, so the tally is running as one uniform clock rather than tracking pickups"
        }
        if ($duelIdleRel -eq 0) { $failures += "DUEL: no bot ever reloaded out of combat" }
        if ($duelIdleSw -eq 0) { $failures += "DUEL: no bot ever rotated weapons between fights" }
    }

    $projFailures = @()

    $pins = @(
        @{ Weapon = 'weapon_crossbow';  Mode = '' }
        @{ Weapon = 'weapon_hornetgun'; Mode = '' }
        @{ Weapon = 'weapon_snark';     Mode = '' }
        @{ Weapon = 'weapon_gauss';     Mode = 'secondary' }
        @{ Weapon = 'weapon_9mmAR';     Mode = 'secondary'; Ammo = 'ammo_ARgrenades' }
        @{ Weapon = 'weapon_handgrenade'; Mode = '' }
        @{ Weapon = 'weapon_rpg';       Mode = ''; Ammo = 'ammo_rpgclip' }
    )

    $seconds = [int]($ProjectileSeconds / $pins.Count)
    if ($seconds -lt 15) { $seconds = 15 }

    Invoke-CbRcon $Port "crossbot_combat 0" | Out-Null

    foreach ($entry in $pins) {
        $pin = $entry.Weapon
        $mode = $entry.Mode
        Write-Host "`n===== WEAPON: $pin $mode, ${seconds}s ====="

        Invoke-CbRcon $Port ("crossbot_fire_pin $pin $mode").Trim() | Out-Null

        Invoke-CbRcon $Port "crossbot_kill" | Out-Null
        Start-Sleep -Seconds $PollSeconds

        $proj = New-FireState
        Update-FireState $proj
        $deadline = (Get-Date).AddSeconds($seconds)
        while ((Get-Date) -lt $deadline) {
            Invoke-CbRcon $Port "crossbot_equip $pin" | Out-Null
            if ($entry.Ammo) { Invoke-CbRcon $Port "crossbot_equip $($entry.Ammo)" | Out-Null }

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
        Write-Host ("  firing={0} zoomed={1} uphill={2} selfveto={3} capped={4} aimveto={5} targetaims={6}" -f `
            $proj.Firing, $proj.Zoomed, $proj.Uphill, $proj.SelfVeto, $proj.Capped,
            $proj.AimVeto, $proj.TargetAims)
        Write-Host ("  weapons held: {0}" -f (($proj.Weapons.Keys | Sort-Object) -join ','))
        Write-Host ("  aim kinds seen: {0}" -f (($proj.Aims.Keys | Sort-Object) -join ','))
        Write-Host ("  reasons: {0}" -f (($proj.Reasons.Keys | Sort-Object | ForEach-Object { "$_=$($proj.Reasons[$_])" }) -join ' '))

        if ($shots -eq 0) { $projFailures += "WEAPON ${pin}: nothing was ever fired" }

        switch ($pin) {
            'weapon_crossbow' {
                if ($proj.Zoomed -eq 0) { $projFailures += "WEAPON ${pin}: the bot never zoomed" }
                if ($proj.TargetAims -eq 0 -and -not $proj.Aims.ContainsKey('target')) {
                    $projFailures += "WEAPON ${pin}: aim=target never appeared, so primaryZoomed was never selected"
                }
                if ($proj.AimVeto -eq 0 -and -not $proj.Reasons.ContainsKey('off-target')) {
                    $projFailures += "WEAPON ${pin}: no shot was ever withheld"
                }
            }
            'weapon_hornetgun' {
                if ($shots -eq 0) { $projFailures += "WEAPON ${pin}: nothing was fired" }
                if ($lead -ge $shots) {
                    $projFailures += "WEAPON ${pin}: every shot was a solved lead, so the cone never replaced the solve"
                }
            }
            'weapon_snark' {
                if ($shots -eq 0) { $projFailures += "WEAPON ${pin}: nothing was thrown" }
                if ($lead -ne 0) { $projFailures += "WEAPON ${pin}: $lead shot(s) were led, and a thrown snark is never led" }
                if ($proj.TargetAims -ne 0) {
                    $projFailures += "WEAPON ${pin}: $($proj.TargetAims) poll(s) published an aim=target, and the snark's only mode is AIM_LEAD"
                }
                if ($proj.Uphill -eq 0) {
                    $projFailures += "WEAPON ${pin}: the uphill veto never fired, so it is not running"
                }
            }
            'weapon_gauss' {
                if ($charged -eq 0) { $projFailures += "WEAPON ${pin}: no charged shot was ever observed" }
                if (-not $proj.Reasons.ContainsKey('charged')) {
                    $projFailures += "WEAPON ${pin}: no bot ever carried a full charge"
                }
                if ($proj.Reasons.ContainsKey('charge-timeout')) {
                    $projFailures += "WEAPON ${pin}: a charge ran to the safety deadline"
                }
            }
            'weapon_handgrenade' {
                if ($thrown -eq 0) { $projFailures += "WEAPON ${pin}: no grenade was ever thrown" }
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
                if ($guided -eq 0) { $projFailures += "WEAPON ${pin}: no launch ever opened a guide window" }
                if ($proj.Aims.ContainsKey('arc') -or $proj.Aims.ContainsKey('lead')) {
                    $projFailures += "WEAPON ${pin}: aim was $(($proj.Aims.Keys | Sort-Object) -join ','), and a guided rocket is aimed at the target"
                }
                if ($proj.Capped -gt 0) {
                    $projFailures += "WEAPON ${pin}: a guide window ran to the safety cap"
                }
            }
            'weapon_9mmAR' {
                if ($shots -eq 0) { $projFailures += "WEAPON ${pin}: nothing was fired" }
                if ($arced -ne $shots) {
                    $projFailures += "WEAPON ${pin}: $arced of $shots shot(s) went through the arc path"
                }
                $refused = $proj.SelfVeto -gt 0 -or $proj.Reasons.ContainsKey('arc-self') -or
                    $proj.Reasons.ContainsKey('arc-blocked') -or $proj.Reasons.ContainsKey('arc-wide')
                if (-not $refused) {
                    $projFailures += "WEAPON ${pin}: no shot was ever withheld, so the impact gates are not running"
                }
            }
        }
    }

    Invoke-CbRcon $Port "crossbot_fire_pin off" | Out-Null
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
