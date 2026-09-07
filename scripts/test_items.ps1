<#
.SYNOPSIS
    Manual driver for the item gathering goals: boots HLDS on a map, adds a bot,
    and polls crossbot_list_items / crossbot_nav_status while the bot gathers.
#>
param(
    [string]$Map = "maps\test56_recharger.bsp",
    [int]$Port = 0,
    [int]$Seconds = 60,
    [int]$Bots = 1,
    [switch]$GatherDebug,
    [string]$HLDSDir = "D:\hlds"
)

$ErrorActionPreference = 'Stop'
$scriptDir = $PSScriptRoot
$repoRoot = Split-Path $scriptDir -Parent

if ($Port -eq 0) { $Port = 29015 + (Get-Random -Maximum 85) }

$mapPath = if (Test-Path $Map) { $Map } else { Join-Path $repoRoot $Map }
$mapName = [System.IO.Path]::GetFileNameWithoutExtension($mapPath)

function Invoke-Rcon {
    param([string]$Cmd)
    $out = try { & "$scriptDir\rcon.ps1" -Port $Port -Command $Cmd } catch { "" }
    if ($null -eq $out) { return "" }
    return ($out | Out-String)
}

$hldsMapPath = Join-Path $HLDSDir "valve\maps\$mapName.bsp"
if ((Resolve-Path $mapPath).Path -ne $hldsMapPath) { Copy-Item $mapPath $hldsMapPath -Force }

$botNavDir = Join-Path $HLDSDir "crossplay\addons\crossbot\nav"
if (-not (Test-Path $botNavDir)) { New-Item -ItemType Directory -Path $botNavDir | Out-Null }
& (Join-Path $repoRoot "build\tools\navgen\navgen.exe") $mapPath -o (Join-Path $botNavDir "$mapName.nav") 2>&1 |
    Select-Object -Last 3 | ForEach-Object { Write-Host "navgen: $_" }

$logFile = Join-Path $repoRoot "nav_debug\logs\${mapName}_items.log"

Write-Host "Starting HLDS on port $Port..."
$startResult = & "$scriptDir\test_server_start.ps1" -Map $mapName -Port $Port -Commands @() `
    -ConfigName "_crossbot_items_$mapName.cfg" -HLDSDir $HLDSDir -LogFile $logFile

if ($startResult -match "^EXITED") { Write-Host "FAIL: $startResult"; exit 1 }
$serverPid = [int]($startResult -replace "PID=", "")
Write-Host "Server PID=$serverPid"

try {
    Write-Host "`n=== crossbot_list_items ==="
    Invoke-Rcon "crossbot_list_items"

    if ($GatherDebug) { Invoke-Rcon "crossbot_gather_debug 1" | Out-Null }

    for ($i = 1; $i -le $Bots; $i++) { Invoke-Rcon "crossbot_add valve Gatherer$i" | Out-Null }

    $deadline = (Get-Date).AddSeconds($Seconds)
    while ((Get-Date) -lt $deadline) {
        Start-Sleep -Seconds 5
        Write-Host "`n=== status t+$([int]($Seconds - ($deadline - (Get-Date)).TotalSeconds)) ==="
        Invoke-Rcon "crossbot_nav_status"
    }
}
finally {
    Stop-Process -Id $serverPid -Force -ErrorAction SilentlyContinue
    Write-Host "`n=== server log tail ==="
    if (Test-Path $logFile) { Get-Content $logFile -Tail 80 }
}
