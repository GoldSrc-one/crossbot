<#
.SYNOPSIS
    Measures damage-per-second against range for every weapon/firemode on one map.

.DESCRIPTION
    Boots one HLDS on the given map, adds bots, and walks the weapon table one
    pin at a time. The plugin writes the raw rows; this script only sequences
    the phases and reports what landed.

    One server per map, all phases inside it. The matrix is the same either way
    round, but a phase is 60 seconds and an HLDS boot is nearly a third of that,
    so paying the boot once per map rather than once per (map, weapon) is most
    of the wall clock.

    Every phase respawns its bots first. The pin reaches weapon selection, and
    selection alone does not isolate a phase: a commit -- a gauss charge, a
    thrown grenade, a rocket in the air -- bypasses ModeUsable by design and
    would discharge into the next phase's counters.

    The pin now equips as well, so a phase no longer starves. Nothing here has
    to hand out ammo.

.PARAMETER Seconds
    Seconds per weapon phase. Sixty is a coarse sample by design; the log is
    appended to, so running the sweep again accumulates rather than replaces.

.PARAMETER Weapons
    Restrict to a subset, matched against the pin classname. Useful for
    checking one mechanism without paying for the whole table.

.PARAMETER Modes
    Restrict to primary or secondary. Combines with -Weapons.
#>
param(
    [string]$Map = "crossfire",
    [int]$Port = 0,
    [int]$Bots = 6,
    [int]$Seconds = 60,
    [int]$SettleSeconds = 3,
    [string[]]$Weapons = @(),
    [string[]]$Modes = @(),
    [string]$OutputDir = "nav_debug\dps",
    [string]$HLDSDir = "D:\projects\trash\crossbot\hlds"
)

$ErrorActionPreference = 'Stop'
$scriptDir = $PSScriptRoot
$repoRoot = Split-Path $scriptDir -Parent

if ($Port -eq 0) { $Port = 29215 + (Get-Random -Maximum 60) }

$mapName = [System.IO.Path]::GetFileNameWithoutExtension($Map)

# Every mode BotFire::Supported accepts. It rejects a WeaponToggle, AIM_NONE,
# AIM_SURFACE and WPN_Q_HIDDEN_STATE, which removes the tripmine, the satchel
# and the three toggle modes.
#
# The two zoom weapons get a phase per zoom state. Left to itself ZoomButtons
# picks by range and crossfire is short, so an unforced crossbow phase zoomed on
# 12 shots of 65 and landed none of them -- the 120 damage sniper bolt had no
# samples at all. The third pin argument forces the state instead of measuring
# whatever the map happened to produce.
$phases = @(
    @{ Pin = "weapon_crowbar";     Mode = "primary" },
    @{ Pin = "weapon_9mmhandgun";  Mode = "primary" },
    @{ Pin = "weapon_9mmhandgun";  Mode = "secondary" },
    @{ Pin = "weapon_357";         Mode = "primary"; Zoom = "nozoom" },
    @{ Pin = "weapon_357";         Mode = "primary"; Zoom = "zoom" },
    @{ Pin = "weapon_9mmAR";       Mode = "primary" },
    @{ Pin = "weapon_9mmAR";       Mode = "secondary" },
    @{ Pin = "weapon_crossbow";    Mode = "primary"; Zoom = "nozoom" },
    @{ Pin = "weapon_crossbow";    Mode = "primary"; Zoom = "zoom" },
    @{ Pin = "weapon_shotgun";     Mode = "primary" },
    @{ Pin = "weapon_shotgun";     Mode = "secondary" },
    @{ Pin = "weapon_rpg";         Mode = "primary" },
    @{ Pin = "weapon_gauss";       Mode = "primary" },
    @{ Pin = "weapon_gauss";       Mode = "secondary" },
    @{ Pin = "weapon_egon";        Mode = "primary" },
    @{ Pin = "weapon_hornetgun";   Mode = "primary" },
    @{ Pin = "weapon_hornetgun";   Mode = "secondary" },
    @{ Pin = "weapon_handgrenade"; Mode = "primary" },
    @{ Pin = "weapon_snark";       Mode = "primary" }
)

if ($Weapons.Count -gt 0) {
    $phases = @($phases | Where-Object { $Weapons -contains $_.Pin })
}
if ($Modes.Count -gt 0) {
    $phases = @($phases | Where-Object { $Modes -contains $_.Mode })
}
if ($phases.Count -eq 0) {
    Write-Host "measure_dps: no phase matches -Weapons/-Modes"
    exit 1
}

function Invoke-Rcon {
    param([string]$Cmd)
    $out = try { & "$scriptDir\rcon.ps1" -Port $Port -Command $Cmd } catch { "" }
    if ($null -eq $out) { return "" }
    return ($out | Out-String)
}

$outDir = Join-Path $repoRoot $OutputDir
New-Item -ItemType Directory -Force -Path $outDir | Out-Null
New-Item -ItemType Directory -Force -Path (Join-Path $repoRoot "nav_debug\logs") | Out-Null

# The plugin resolves this against the HLDS working directory, so the two forms
# differ: forward slashes for the server, a real path for us to read back.
$logRel = "valve/addons/crossbot/dps/$mapName.csv"
$logAbs = Join-Path $HLDSDir "valve\addons\crossbot\dps\$mapName.csv"
New-Item -ItemType Directory -Force -Path (Split-Path $logAbs -Parent) | Out-Null

$serverLog = Join-Path $repoRoot "nav_debug\logs\${mapName}_dps.log"

$boot = @(
    "crossbot_dps_log $logRel",
    "crossbot_dps 1",
    "crossbot_fire 1",
    "crossbot_gather 1",
    "crossbot_fire_pin off"
)
for ($i = 1; $i -le $Bots; $i++) { $boot += "crossbot_add Dps$i" }

Write-Host "measure_dps: $mapName on port $Port, $($phases.Count) phase(s) x ${Seconds}s, $Bots bots"

$startOut = & "$scriptDir\test_server_start.ps1" -Map $mapName -Port $Port -LogFile $serverLog `
    -HLDSDir $HLDSDir -ConfigName "_crossbot_dps_$mapName.cfg" -Commands $boot

if ($startOut -notmatch 'PID=(\d+)') {
    Write-Host "measure_dps: server did not start: $startOut"
    exit 1
}
$serverPid = [int]$Matches[1]

$sizeBefore = if (Test-Path $logAbs) { (Get-Item $logAbs).Length } else { 0 }
$ok = $true

try {
    $sw = [System.Diagnostics.Stopwatch]::StartNew()

    foreach ($phase in $phases) {
        $zoom = if ($phase.Zoom) { $phase.Zoom } else { "any" }
        $label = "$($phase.Pin) $($phase.Mode) $zoom"

        # Order matters. The pin flush closes the previous phase's exposure
        # bins, then crossbot_kill drops any commit that would otherwise fire
        # into this phase.
        Invoke-Rcon "crossbot_fire_pin off" | Out-Null
        Invoke-Rcon "crossbot_kill" | Out-Null
        Start-Sleep -Seconds $SettleSeconds
        Invoke-Rcon "crossbot_fire_pin $($phase.Pin) $($phase.Mode) $zoom" | Out-Null

        Start-Sleep -Seconds $Seconds

        $status = Invoke-Rcon "crossbot_dps_status"
        $rows = 0; $shots = 0; $dmg = 0; $kills = 0
        if ($status -match 'rows=(\d+)') { $rows = [int]$Matches[1] }
        if ($status -match 'shots=(\d+)') { $shots = [int]$Matches[1] }
        if ($status -match ' dmg=(\d+)') { $dmg = [int]$Matches[1] }
        if ($status -match 'kills=(\d+)') { $kills = [int]$Matches[1] }

        Write-Host ("  {0,-40} rows={1,-7} shots={2,-6} dmg={3,-6} kills={4}" -f `
            $label, $rows, $shots, $dmg, $kills)
    }

    # Close the last phase's bins while the server is still up.
    Invoke-Rcon "crossbot_fire_pin off" | Out-Null
    Start-Sleep -Seconds 1

    $sw.Stop()
    Write-Host ("measure_dps: {0} done in {1:N0}s" -f $mapName, $sw.Elapsed.TotalSeconds)
}
finally {
    & "$scriptDir\test_server_stop.ps1" -Id $serverPid | Out-Null

    Get-CimInstance Win32_Process -Filter "Name='hlds.exe'" -ErrorAction SilentlyContinue |
        Where-Object { $_.CommandLine -match "\+port\s+$Port(\s|$)" } |
        ForEach-Object { Stop-Process -Id $_.ProcessId -Force -ErrorAction SilentlyContinue }
}

if (-not (Test-Path $logAbs)) {
    Write-Host "measure_dps: $mapName wrote no log at $logAbs"
    exit 1
}

Copy-Item $logAbs (Join-Path $outDir "$mapName.csv") -Force

$sizeAfter = (Get-Item $logAbs).Length
$grew = $sizeAfter - $sizeBefore

$lines = Get-Content (Join-Path $outDir "$mapName.csv")
$counts = @{}
foreach ($kind in @('meta', 'shot', 'dmg')) {
    $counts[$kind] = @($lines | Where-Object { $_ -like "$kind,*" }).Count
}

Write-Host ("measure_dps: {0} meta={1} shot={2} dmg={3} (+{4} bytes)" -f `
    $mapName, $counts['meta'], $counts['shot'], $counts['dmg'], $grew)

if ($grew -le 0) {
    Write-Host "measure_dps: $mapName produced no new rows"
    $ok = $false
}

if ($ok) { exit 0 } else { exit 1 }
