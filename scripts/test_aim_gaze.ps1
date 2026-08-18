<#
.SYNOPSIS
    Manual driver for the aim delay queue: boots HLDS, adds bots that can see
    each other, and polls crossbot_aim_status while stepping crossbot_aim_delay.

.DESCRIPTION
    The nav sweep cannot exercise the aim queue -- it spawns one bot, and with
    LOS-gated gaze a lone bot never has a target. This script is the only
    automated way to see the queue run.

    Two bots left to roam a real map is not enough either: they chase items
    independently and are almost never in each other's line of sight, so the
    queue sits empty. So the run has two phases.

    STATIC: gather off, no goals, all bots teleported to one spawn. They stand
    together, hold line of sight continuously, and the target is stationary --
    which is the case the design says must be exact at *any* delay, because
    extrapolating a still target forward and reading it back cancels perfectly.
    err should be near zero at every delay. It is the sharpest correctness test
    there is, and any nonzero trend across delays is a real defect.

    ROAM: gather back on, bots move. err is now allowed to be nonzero -- that is
    target acceleration, which is the whole point -- but the queue should be full
    and served should equal the requested delay whenever a target is held.

.PARAMETER Delays
    crossbot_aim_delay values to walk through in the static phase (default: 0, 0.25, 0.5).
#>
param(
    [string]$Map = "crossfire",
    [int]$Port = 0,
    [float[]]$Delays = @(0.0, 0.25, 0.5),
    [int]$SecondsPerDelay = 10,
    [int]$RoamSeconds = 30,
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
        Where-Object { $_ -match 'gaze=|bot\(s\)' } |
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

$logFile = Join-Path $repoRoot "nav_debug\logs\${mapName}_aimgaze.log"

Write-Host "Starting HLDS on port $Port (join with: connect 127.0.0.1:$Port)"
$startResult = & "$scriptDir\test_server_start.ps1" -Map $mapName -Port $Port -Commands @() `
    -ConfigName "_crossbot_aimgaze_$mapName.cfg" -HLDSDir $HLDSDir -LogFile $logFile

if ($startResult -match "^EXITED") { Write-Host "FAIL: $startResult"; exit 1 }
$serverPid = [int]($startResult -replace "PID=", "")
Write-Host "Server PID=$serverPid"

try {
    Invoke-Rcon "crossbot_gather 0" | Out-Null
    for ($i = 1; $i -le $Bots; $i++) { Invoke-Rcon "crossbot_add Gazer$i" | Out-Null }
    Invoke-Rcon "crossbot_aim_gaze 1" | Out-Null

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
    Write-Host "err should stay near zero at every delay -- a stationary target cancels exactly."
    Write-Host "Do not use crossbot_teleport here: it stacks every bot on the identical origin, which"
    Write-Host "makes the eye-to-eye vector zero, so no angle exists and the queue is never fed."

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
        Invoke-Rcon "crossbot_gather 1" | Out-Null

        $deadline = (Get-Date).AddSeconds($RoamSeconds)
        while ((Get-Date) -lt $deadline) {
            Start-Sleep -Seconds 3
            Show-AimStatus
        }
    }
}
finally {
    Stop-Process -Id $serverPid -Force -ErrorAction SilentlyContinue
    Write-Host "`nServer log: $logFile"
}
