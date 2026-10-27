param(
    [ValidateSet("cstrike", "tfc", "dod", "valve")]
    [string]$Game = "cstrike",
    [string]$Map = "",
    [int]$Port = 29370,
    [string]$HLDSDir = "D:\hlds"
)

$ErrorActionPreference = "Stop"
$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$repoRoot = Split-Path -Parent $scriptDir
$logDir = Join-Path $repoRoot "nav_debug\logs"
if (-not (Test-Path $logDir)) { New-Item -ItemType Directory -Path $logDir -Force | Out-Null }
. "$scriptDir\common.ps1"

if (-not $Map) { $Map = @{ cstrike = "de_dust2"; tfc = "casbah"; dod = "dod_anzio"; valve = "crossfire" }[$Game] }
$failures = New-Object System.Collections.ArrayList
$log = Join-Path $logDir "directives_$Map.log"

function Get-Pos([string]$status, [string]$name) {
    $m = [regex]::Match($status, "CrossBot: $name at \((-?\d+),(-?\d+),(-?\d+)\)")
    if (-not $m.Success) { return $null }
    return @([double]$m.Groups[1].Value, [double]$m.Groups[2].Value, [double]$m.Groups[3].Value)
}

$boot = @("mp_timelimit 0", "mp_freezetime 0", "mp_roundtime 9", "mp_autoteambalance 0", "mp_limitteams 0")
$games = if ($Game -eq "valve") { @("valve") } else { @("valve", $Game) }
if ($Game -eq "cstrike") { $games = @("cstrike") }
if ($Game -eq "valve") { $boot += "mp_teamplay 1" }
$start = & "$scriptDir\test_server_start.ps1" -Map $Map -Games $games -Port $Port -LogFile $log -WaitSeconds 90 `
    -RequireVerbose -Commands $boot
$followGaps = @()
try {
    if ($start -notmatch 'PID=(\d+)') { throw "server did not start: $start" }
    $srv = [int]$Matches[1]
    function Rc($c) { Invoke-CbRcon $Port $c }

    if ($Game -eq "cstrike") {
        foreach ($b in @("Lead t", "Wing t", "Guard t", "Cop ct")) { Rc "crossbot_add cstrike $b" | Out-Null; Start-Sleep -Milliseconds 400 }
        Start-Sleep -Seconds 8
        Rc 'crossbot_do all rules {"hold_fire":true,"no_auto_objective":true,"seconds":600}' | Out-Null
        Rc 'crossbot_do Cop act {"seconds":200,"steps":[{"do":"wait","seconds":200}]}' | Out-Null

        Rc 'crossbot_do Lead act {"steps":[{"do":"go","target":"site:B"}]}' | Out-Null
        Rc 'crossbot_do Wing act {"steps":[{"do":"follow","target":"Lead","within":128,"seconds":60}]}' | Out-Null
        Rc 'crossbot_do Guard act {"steps":[{"do":"hold","target":"site:A","watch":"site:B","stance":"crouch","seconds":30}]}' | Out-Null
        for ($i = 0; $i -lt 6; $i++) {
            Start-Sleep -Seconds 5
            $s = Rc "crossbot_directive_status"
            $a = Get-Pos $s "Lead"; $b = Get-Pos $s "Wing"
            if ($a -and $b -and $i -ge 4) { $followGaps += (Get-CbDistance3D $a[0] $a[1] $a[2] $b[0] $b[1] $b[2]) }
        }
        Rc 'crossbot_do Cop act {"steps":[{"do":"go","target":"goose"},{"do":"wait","seconds":1}]}' | Out-Null

        $carrier = if ((Rc "crossbot_weapons Lead") -match "weapon_c4") { "Lead" } elseif ((Rc "crossbot_weapons Wing") -match "weapon_c4") { "Wing" } else { "Guard" }
        $other = if ($carrier -eq "Lead") { "Wing" } else { "Lead" }
        Rc ('crossbot_do ' + $carrier + ' act {"steps":[{"do":"go","target":"' + $other + '","within":96},{"do":"look","target":"' + $other + '","seconds":1},{"do":"drop","item":"weapon_c4"}]}') | Out-Null
        Start-Sleep -Seconds 20
        Rc ('crossbot_do ' + $other + ' act {"steps":[{"do":"go","target":"site:B"},{"do":"ability","ability":"plant"},{"do":"wait","seconds":2}]}') | Out-Null
        Start-Sleep -Seconds 45

        Rc 'crossbot_do all rules {"hold_fire":true,"no_auto_objective":true,"seconds":600}' | Out-Null
        Rc 'crossbot_do Lead rules {"no_auto_objective":true,"seconds":600}' | Out-Null
        Rc 'crossbot_do Lead act {"steps":[{"do":"fire","target":"aim:Lead","count":3}]}' | Out-Null
        Rc 'crossbot_do Wing act {"steps":[{"do":"fire","target":"Lead"}]}' | Out-Null
        Start-Sleep -Seconds 6

        Rc 'crossbot_do Cop rules {"melee_only":true,"seconds":60}' | Out-Null
        Rc 'crossbot_do Cop act {"steps":[{"do":"go","target":"Lead"}]}' | Out-Null
        Start-Sleep -Seconds 20
    }
    elseif ($Game -eq "tfc") {
        foreach ($b in @("Eng engineer", "Spook spy", "Boom demoman", "Doc medic", "Hurt soldier")) { Rc "crossbot_add tfc $b" | Out-Null; Start-Sleep -Milliseconds 400 }
        Start-Sleep -Seconds 12
        Rc 'crossbot_do all rules {"hold_fire":true,"seconds":600}' | Out-Null
        Rc 'crossbot_do Boom act {"steps":[{"do":"ability","ability":"detpack","arg":"5"}]}' | Out-Null
        Rc 'crossbot_do Doc act {"steps":[{"do":"ability","ability":"build_sentry"}]}' | Out-Null
        Rc 'crossbot_do Eng act {"steps":[{"do":"look","target":"aim:Eng","seconds":1},{"do":"ability","ability":"build_sentry"}]}' | Out-Null
        Start-Sleep -Seconds 10
        Rc 'crossbot_do Eng act {"steps":[{"do":"ability","ability":"detsentry"},{"do":"ability","ability":"build_dispenser"}]}' | Out-Null
        Rc 'crossbot_do Spook act {"steps":[{"do":"ability","ability":"disguise_enemy","arg":"soldier"},{"do":"wait","seconds":3}]}' | Out-Null
        Rc 'crossbot_do Spook act {"steps":[{"do":"ability","ability":"disguise_enemy","arg":"spy"}]}' | Out-Null
        Start-Sleep -Seconds 2
        Rc 'crossbot_do Spook act {"steps":[{"do":"ability","ability":"disguise_enemy","arg":"pyro"},{"do":"wait","seconds":3}]}' | Out-Null
        $teams = @{}
        foreach ($m in [regex]::Matches((Rc "crossbot_teams_status"), "player #\d+ (\S+) team=(\d+)")) { $teams[$m.Groups[1].Value] = $m.Groups[2].Value }
        $patient = @($teams.Keys | Where-Object { $_ -ne "Doc" -and $teams[$_] -eq $teams["Doc"] }) | Select-Object -First 1
        Write-Host "Doc's patient: $patient"
        Rc ('crossbot_do ' + $patient + ' act {"seconds":60,"steps":[{"do":"wait","seconds":60}]}') | Out-Null
        Rc ('crossbot_do Doc act {"steps":[{"do":"go","target":"' + $patient + '","within":48},{"do":"ability","ability":"heal","target":"' + $patient + '"}]}') | Out-Null
        Start-Sleep -Seconds 25
    }
    elseif ($Game -eq "dod") {
        foreach ($b in @("Gunner 30cal", "Rifle garand")) { Rc "crossbot_add dod $b" | Out-Null; Start-Sleep -Milliseconds 400 }
        Start-Sleep -Seconds 30
        Rc 'crossbot_do all rules {"hold_fire":true,"seconds":600}' | Out-Null
        Rc 'crossbot_do Gunner act {"steps":[{"do":"hold","target":"aim:Gunner","stance":"prone","seconds":15}]}' | Out-Null
        Rc 'crossbot_do Rifle act {"steps":[{"do":"hold","target":"aim:Rifle","stance":"prone","seconds":10}]}' | Out-Null
        Start-Sleep -Seconds 20
    }
    elseif ($Map -eq "test2_doorway") {
        Rc "crossbot_add valve Miner" | Out-Null
        Start-Sleep -Seconds 3
        Rc 'crossbot_deploy 0' | Out-Null
        Rc 'crossbot_equip weapon_tripmine Miner' | Out-Null
        Rc 'crossbot_do Miner act {"steps":[{"do":"place","item":"weapon_tripmine","target":"400 384 32"}]}' | Out-Null
        Start-Sleep -Seconds 20
    }
    else {
        foreach ($b in @("Miner", "Sapper", "Mate")) { Rc "crossbot_add valve $b" | Out-Null; Start-Sleep -Milliseconds 400 }
        Start-Sleep -Seconds 6
        Rc 'crossbot_deploy 0' | Out-Null
        Rc 'crossbot_do all rules {"hold_fire":true,"seconds":600}' | Out-Null
        Rc 'crossbot_equip weapon_tripmine Miner' | Out-Null
        Rc 'crossbot_equip weapon_handgrenade Sapper' | Out-Null
        Rc 'crossbot_do Miner act {"steps":[{"do":"place","item":"weapon_tripmine","target":"aim:Miner"}]}' | Out-Null
        Rc 'crossbot_do Mate act {"steps":[{"do":"place","item":"weapon_tripmine","target":"aim:Mate"}]}' | Out-Null
        Start-Sleep -Seconds 20
        Rc 'crossbot_do Sapper rules {"seconds":600}' | Out-Null
        Rc 'crossbot_do Sapper act {"steps":[{"do":"look","target":"aim:Sapper","seconds":1},{"do":"fire","item":"weapon_handgrenade","target":"aim:Sapper"}]}' | Out-Null
        Start-Sleep -Seconds 10
    }
    $final = Rc "crossbot_directive_status"
} finally {
    if ($srv) { & "$scriptDir\test_server_stop.ps1" -Id $srv | Out-Null }
}

$lines = Get-Content $log
Write-Host $final
if ($Game -eq "cstrike") {
    $maxGap = if ($followGaps.Count) { ($followGaps | Measure-Object -Maximum).Maximum } else { 99999 }
    Write-Host ("follow gaps: " + (($followGaps | ForEach-Object { [math]::Round($_) }) -join ", "))
    Assert-Cb (@($lines | Select-String "Lead directive step 1/1 go done: arrived").Count -ge 1) "go to site B arrived"
    Assert-Cb ($followGaps.Count -ge 2 -and $maxGap -lt 400) "follower stayed near its leader (max gap $([math]::Round($maxGap)))"
    Assert-Cb (@($lines | Select-String "Guard holds on orders .* crouched").Count -ge 1) "hold took the crouch stance"
    Assert-Cb (@($lines | Select-String "Guard directive step 1/1 hold done: held").Count -ge 1) "hold reached its spot and held"
    Assert-Cb (@($lines | Select-String "Lead directive step 1/1 fire done: fired").Count -ge 1) "fire shot three rounds at where Lead looked"
    Assert-Cb (@($lines | Select-String "Wing directive step 1/1 fire failed: will not shoot a teammate").Count -ge 1) "fire refused a teammate as target"
    Assert-Cb (@($lines | Select-String 'triggered "Dropped_The_Bomb"').Count -ge 1) "the carrier dropped the C4"
    Assert-Cb (@($lines | Select-String 'triggered "Planted_The_Bomb"').Count -ge 1) "the C4 was planted on orders"
    Assert-Cb (@($lines | Select-String "is planting the bomb in the bomb zone|is planting the bomb$").Count -eq 0) "nobody planted on its own while no_auto_objective was set"
    $coward = @($lines | Select-String "Cop .*fires weapon_(?!knife)").Count
    Assert-Cb (@($lines | Select-String "Cop directive step 1/2 go done: arrived").Count -ge 1) "a named place from places.txt ('goose') resolved and was reached"
    Assert-Cb ($coward -eq 0) "melee_only bot fired nothing but the knife ($coward)"
}
elseif ($Game -eq "tfc") {
    $sentry = @($lines | Select-String "Eng finished building 2").Count -ge 1
    Assert-Cb ($sentry -or @($lines | Select-String "Eng directive step 2/2 ability failed: (build refused|no room to build)").Count -ge 1) "the ordered sentry was built, or the game's refusal failed the step"
    Assert-Cb (@($lines | Select-String "Eng directive step 1/2 ability done: issued").Count -ge 1) "detsentry was issued"
    Assert-Cb (@($lines | Select-String "Eng finished building 1|Eng directive step 2/2 ability failed: (not enough metal|enemy|build refused|no room to build)").Count -ge 1) "the ordered dispenser was built or failed with a reason"
    Assert-Cb (@($lines | Select-String "Spook directive step 1/1 ability failed: bad argument").Count -ge 1) "disguise as spy was refused as a bad argument"
    Assert-Cb (@($lines | Select-String "Spook directive step 1/2 ability done: issued").Count -ge 2) "disguise with a class name was issued"
    Assert-Cb (@($lines | Select-String "Boom sets a 5 s detpack on orders").Count -ge 1) "demoman set an ordered detpack"
    Assert-Cb (@($lines | Select-String "Doc heals \S+ on orders|Doc directive step 2/2 ability failed: already at full health").Count -ge 1) "medic took the heal order on a teammate (or the teammate was at full health)"
    Assert-Cb (@($lines | Select-String "Doc directive step 1/1 ability failed: not an engineer").Count -ge 1) "a medic refused to build"
}
elseif ($Game -eq "dod") {
    Assert-Cb (@($lines | Select-String "Gunner holds on orders .*deploying").Count -ge 1) "the machine gunner held to deploy"
    Assert-Cb (@($lines | Select-String "Gunner deploys its gun").Count -ge 1) "the machine gunner deployed its gun"
    Assert-Cb (@($lines | Select-String "Rifle holds on orders .* prone").Count -ge 1) "the rifleman went prone"
    Assert-Cb (@($lines | Select-String "Rifle directive step 1/1 hold done: held").Count -ge 1) "the prone hold finished"
}
elseif ($Map -eq "test2_doorway") {
    Assert-Cb (@($lines | Select-String "Miner directive step 1/1 place done: placed").Count -ge 1) "an ordered tripmine was placed at the doorway with crossbot_deploy 0"
}
else {
    Assert-Cb (@($lines | Select-String "Miner directive step 1/1 place (done|failed)").Count -ge 1) "an ordered tripmine resolved (test2_doorway is the one that must place it)"
    Assert-Cb (@($lines | Select-String "Mate directive step 1/1 place failed: nothing it can place|Mate directive step 1/1 place failed: none left to place").Count -ge 1) "a bot without mines refused to place one"
    Assert-Cb (@($lines | Select-String "Sapper directive step 2/2 fire (done|failed)").Count -ge 1) "an ordered grenade throw resolved"
}

if ($failures.Count -gt 0) { Write-Host "FAILED: $($failures.Count)"; exit 1 }
Write-Host "PASS"
