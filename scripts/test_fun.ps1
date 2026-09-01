<#
.SYNOPSIS
    Manual driver for the fun goal tier: boots HLDS on a map, prints the fun
    inventory, adds bots, and polls crossbot_nav_status while they play with it.
#>
param(
    [string]$Map = "hlds\valve\maps\crossfire.bsp",
    [int]$Port = 0,
    [int]$Seconds = 180,
    [int]$Bots = 4,
    [switch]$FunDebug,
    [switch]$NoGather,
    [string]$HLDSDir = "D:\projects\trash\crossbot\hlds"
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

$botNavDir = Join-Path $HLDSDir "valve\addons\crossbot\nav"
if (-not (Test-Path $botNavDir)) { New-Item -ItemType Directory -Path $botNavDir | Out-Null }
& (Join-Path $repoRoot "build\tools\navgen\navgen.exe") $mapPath -o (Join-Path $botNavDir "$mapName.nav") 2>&1 |
    Select-Object -Last 3 | ForEach-Object { Write-Host "navgen: $_" }

$logFile = Join-Path $repoRoot "nav_debug\logs\${mapName}_fun.log"

Write-Host "Starting HLDS on port $Port..."
$startResult = & "$scriptDir\test_server_start.ps1" -Map $mapName -Port $Port -Commands @() `
    -ConfigName "_crossbot_fun_$mapName.cfg" -HLDSDir $HLDSDir -LogFile $logFile

if ($startResult -match "^EXITED") { Write-Host "FAIL: $startResult"; exit 1 }
$serverPid = [int]($startResult -replace "PID=", "")
Write-Host "Server PID=$serverPid"

try {
    Write-Host "`n=== crossbot_list_fun ==="
    Invoke-Rcon "crossbot_list_fun"

    if ($FunDebug) { Invoke-Rcon "crossbot_fun_debug 1" | Out-Null }
    if ($NoGather) { Invoke-Rcon "crossbot_gather 0" | Out-Null }

    for ($i = 1; $i -le $Bots; $i++) { Invoke-Rcon "crossbot_add Player$i" | Out-Null }

    $deadline = (Get-Date).AddSeconds($Seconds)
    while ((Get-Date) -lt $deadline) {
        Start-Sleep -Seconds 10
        Write-Host "`n=== status t+$([int]($Seconds - ($deadline - (Get-Date)).TotalSeconds)) ==="
        Invoke-Rcon "crossbot_nav_status"
    }

    Write-Host "`n=== crossbot_list_fun (after) ==="
    Invoke-Rcon "crossbot_list_fun"
}
finally {
    Stop-Process -Id $serverPid -Force -ErrorAction SilentlyContinue
    Write-Host "`n=== fun decisions ==="
    if (Test-Path $logFile) {
        Get-Content $logFile |
            Select-String -Pattern 'wants (fun|gather|strategic)|detours to|the tank|bored of|bailing out|pressed|walked into|shot open|could not|fun table' |
            Select-Object -Last 60
    }
}
