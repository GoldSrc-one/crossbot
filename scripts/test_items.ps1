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

. "$scriptDir\common.ps1"

$hldsMapPath = Join-Path $HLDSDir "valve\maps\$mapName.bsp"
Copy-CbMap $mapPath $hldsMapPath

Ensure-CbNav -HldsMap $mapPath -NavPath (Join-Path $HLDSDir "crossplay\addons\crossbot\nav\$mapName.nav") `
    -NavGenExe (Join-Path $repoRoot "build\tools\navgen\navgen.exe") -Force

$logFile = Join-Path $repoRoot "nav_debug\logs\${mapName}_items.log"

Write-Host "Starting HLDS on port $Port..."
$startResult = & "$scriptDir\test_server_start.ps1" -Map $mapName -Port $Port -Commands @() `
    -ConfigName "_crossbot_items_$mapName.cfg" -HLDSDir $HLDSDir -LogFile $logFile

$serverPid = Get-CbServerPid $startResult
Write-Host "Server PID=$serverPid"

try {
    Write-Host "`n=== crossbot_list_items ==="
    Invoke-CbRcon $Port "crossbot_list_items"

    if ($GatherDebug) { Invoke-CbRcon $Port "crossbot_gather_debug 1" | Out-Null }

    for ($i = 1; $i -le $Bots; $i++) { Invoke-CbRcon $Port "crossbot_add valve Gatherer$i" | Out-Null }

    $deadline = (Get-Date).AddSeconds($Seconds)
    while ((Get-Date) -lt $deadline) {
        Start-Sleep -Seconds 5
        Write-Host "`n=== status t+$([int]($Seconds - ($deadline - (Get-Date)).TotalSeconds)) ==="
        Invoke-CbRcon $Port "crossbot_nav_status"
    }
}
finally {
    Stop-Process -Id $serverPid -Force -ErrorAction SilentlyContinue
    Write-Host "`n=== server log tail ==="
    if (Test-Path $logFile) { Get-Content $logFile -Tail 80 }
}
