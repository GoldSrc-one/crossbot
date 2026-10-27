param(
    [string]$Map = "de_dust2",
    [string]$Game = "cstrike",
    [int]$Port = 0,
    [int]$Bots = 6,
    [int]$Seconds = 120,
    [switch]$StrategicDebug,
    [switch]$TacticsDebug,
    [switch]$Plugin,
    [int]$Tactics = 1,
    [int]$Snipe = 1,
    [string]$HLDSDir = "D:\hlds",
    [string]$BotGame = ''
)

$ErrorActionPreference = 'Stop'
$scriptDir = $PSScriptRoot
$repoRoot = Split-Path $scriptDir -Parent

if (-not $BotGame) { $BotGame = if ($Game -in 'cstrike', 'czero') { $Game } else { 'valve' } }

if ($Port -eq 0) { $Port = 29015 + (Get-Random -Maximum 85) }

. "$scriptDir\common.ps1"

$logFile = Join-Path $repoRoot "nav_debug\logs\${Map}_objectives.log"

$commands = @('mp_timelimit 0', 'amxx pause crossbot_menu.amxx')
if ($StrategicDebug) { $commands += 'crossbot_strategic_debug 1' }
if ($Plugin) { $commands += 'crossbot_objectives 0' }
$commands += "crossbot_tactics $Tactics"
$commands += "crossbot_snipe $Snipe"
if ($TacticsDebug) { $commands += 'crossbot_tactics_debug 1' }

Write-Host "Starting HLDS on port $Port..."
$startResult = & "$scriptDir\test_server_start.ps1" -Map $Map -Port $Port -Games @('valve', $Game) -WaitSeconds 90 `
    -Commands $commands -ConfigName "_crossbot_objectives_$Map.cfg" -HLDSDir $HLDSDir -LogFile $logFile

$serverPid = Get-CbServerPid $startResult
Write-Host "Server PID=$serverPid"

try {
    Write-Host "`n=== crossbot_list_objectives ==="
    Invoke-CbRcon $Port "crossbot_list_objectives"

    for ($i = 1; $i -le $Bots; $i++) {
        $side = if ($BotGame -in 'cstrike', 'czero') { @('t', 'ct')[$i % 2] } else { '' }
        Invoke-CbRconQuiet $Port "crossbot_add $BotGame Player$i $side"
    }

    Start-Sleep -Seconds $Seconds

    Write-Host "`n=== crossbot_list_objectives (after) ==="
    Invoke-CbRcon $Port "crossbot_list_objectives"
}
finally {
    Stop-Process -Id $serverPid -Force -ErrorAction SilentlyContinue
    Write-Host "`n=== objective decisions ==="
    if (Test-Path $logFile) {
        Get-Content $logFile |
            Select-String -Pattern 'objectives:|wants strategic|holding the|held the|works |strategic goal|the bomb|Bomb|Hostage|_Win"' |
            Select-Object -Last 60
    }
}
