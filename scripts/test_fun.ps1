param(
    [string]$Map = "maps\crossfire.bsp",
    [int]$Port = 0,
    [int]$Seconds = 180,
    [int]$Bots = 4,
    [switch]$FunDebug,
    [switch]$NoGather,
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
    -NavGenExe (Join-Path $repoRoot "build-x64\tools\navgen\navgen.exe") -Force

$logFile = Join-Path $repoRoot "nav_debug\logs\${mapName}_fun.log"

Write-Host "Starting HLDS on port $Port..."
$startResult = & "$scriptDir\test_server_start.ps1" -Map $mapName -Port $Port -Commands @() `
    -ConfigName "_crossbot_fun_$mapName.cfg" -HLDSDir $HLDSDir -LogFile $logFile

$serverPid = Get-CbServerPid $startResult
Write-Host "Server PID=$serverPid"

try {
    Write-Host "`n=== crossbot_list_fun ==="
    Invoke-CbRcon $Port "crossbot_list_fun"

    if ($FunDebug) { Invoke-CbRcon $Port "crossbot_fun_debug 1" | Out-Null }
    if ($NoGather) { Invoke-CbRcon $Port "crossbot_gather 0" | Out-Null }

    for ($i = 1; $i -le $Bots; $i++) { Invoke-CbRcon $Port "crossbot_add valve Player$i" | Out-Null }

    $deadline = (Get-Date).AddSeconds($Seconds)
    while ((Get-Date) -lt $deadline) {
        Start-Sleep -Seconds 10
        Write-Host "`n=== status t+$([int]($Seconds - ($deadline - (Get-Date)).TotalSeconds)) ==="
        Invoke-CbRcon $Port "crossbot_nav_status"
    }

    Write-Host "`n=== crossbot_list_fun (after) ==="
    Invoke-CbRcon $Port "crossbot_list_fun"
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
