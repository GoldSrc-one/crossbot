param(
    [string]$Game = "valve",
    [string]$Map = "crossfire",
    [int]$Port = 29016,
    [int]$WaitSeconds = 60,
    [string]$HLDSDir = "D:\hlds"
)

. "$PSScriptRoot\common.ps1"

function Get-BotOrigins {
    param([string]$Status)
    $origins = @{}
    foreach ($m in [regex]::Matches($Status, "player #\d+ (\S+) game=\S+ team=(\d+) bot=1 .*?origin=\(([^)]+)\)")) {
        $origins[$m.Groups[1].Value] = [pscustomobject]@{ Team = [int]$m.Groups[2].Value; Origin = $m.Groups[3].Value }
    }
    return $origins
}

$repo = Split-Path $PSScriptRoot -Parent
$logDir = Join-Path $repo "nav_debug\logs"
New-Item -ItemType Directory -Path $logDir -Force | Out-Null
$log = Join-Path $logDir "standalone_$Game.log"

& "$PSScriptRoot\deploy_standalone.ps1" -Game $Game -Map $Map -HLDSDir $HLDSDir | Out-Null

$extra = @("-dll", "addons\metamod\metamod.dll", "+localinfo", "mm_pluginsfile", "addons/crossbot/plugins.ini")
$start = & "$PSScriptRoot\test_server_start.ps1" -Map $Map -GameDir $Game -Games @() -Port $Port `
    -WaitSeconds $WaitSeconds -HLDSDir $HLDSDir -LogFile $log -ExtraArgs $extra `
    -Commands @("crossbot_add Bot1", "crossbot_add Bot2") 3>$null

if ("$start" -notmatch "PID=(\d+)") {
    "FAIL ${Game}: $start"
    exit 1
}
$serverPid = [int]$matches[1]

Start-Sleep -Seconds 15
$before = Get-BotOrigins (Invoke-CbRcon -Port $Port -Command "crossbot_teams_status")
Start-Sleep -Seconds 20
$after = Get-BotOrigins (Invoke-CbRcon -Port $Port -Command "crossbot_teams_status")
$alive = [bool](Get-Process -Id $serverPid -ErrorAction SilentlyContinue)

if ($alive) { & "$PSScriptRoot\test_server_stop.ps1" -Id $serverPid | Out-Null }

$text = if (Test-Path $log) { Get-Content $log -Raw } else { "" }
$moved = @($before.Keys | Where-Object { $after.ContainsKey($_) -and $after[$_].Origin -ne $before[$_].Origin }).Count
$teamless = @($after.Values | Where-Object { $_.Team -eq 0 }).Count

$failed = @()
if ($text -notmatch [regex]::Escape("game #0 '$Game' ready")) { $failed += "game not ready" }
if ($text -notmatch [regex]::Escape("added Bot1 to '$Game'")) { $failed += "Bot1 not added" }
if ($text -match "CrossBot: FATAL") { $failed += "FATAL in log" }
if (-not $alive) { $failed += "server died" }
if ($moved -lt 1) { $failed += "no bot moved" }
if ($Game -in @("cstrike", "czero", "dod", "tfc") -and $teamless) { $failed += "$teamless bot(s) on team 0" }

if ($failed.Count) {
    "FAIL ${Game} ${Map}: $($failed -join ', ') (see $log)"
    exit 1
}
"PASS ${Game} ${Map}: $moved bot(s) moved"
