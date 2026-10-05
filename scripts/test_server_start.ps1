param(
    [string]$Map = "crossfire",
    [int]$WaitSeconds = 30,
    [int]$Port = 29015,
    [string[]]$Commands = @(),
    [string]$ConfigName = "_crossbot_autotest.cfg",
    [string]$HLDSDir = "D:\hlds",
    [string]$GameDir = "crossplay",
    [string[]]$Games = @("valve"),
    [string]$LogFile = "",
    [string[]]$ExtraArgs = @(),
    [int]$StdinKeepaliveSeconds = 36000,
    [switch]$RequireVerbose
)

$exe = Join-Path $HLDSDir "hlds.exe"
if (-not (Test-Path $exe)) {
    "EXITED code=-1 (hlds.exe not found at $exe)"
    exit
}

if (-not $LogFile) {
    $LogFile = Join-Path $HLDSDir "logs\hlds_$Port.log"
}
$logDir = Split-Path $LogFile -Parent
if ($logDir -and -not (Test-Path $logDir)) { New-Item -ItemType Directory -Path $logDir -Force | Out-Null }
$errFile = "$LogFile.err"

Remove-Item $LogFile, $errFile -ErrorAction SilentlyContinue

if (-not [System.IO.Path]::IsPathRooted($LogFile)) { $LogFile = Join-Path (Get-Location) $LogFile }
$LogFile = [System.IO.Path]::GetFullPath($LogFile)
$errFile = "$LogFile.err"


$addonDir = Join-Path $HLDSDir "$GameDir\addons\crossbot"
$deployed = Join-Path $addonDir "crossbot_mm.dll"
$pluginsIni = Join-Path $HLDSDir "$GameDir\addons\metamod\plugins.ini"

if (Test-Path $pluginsIni) {
    if (-not (Select-String -Path $pluginsIni -Pattern "crossbot_mm" -Quiet)) {
        Write-Warning "$pluginsIni has no crossbot_mm.dll line -- the plugin will not load. Add: win32 addons/crossbot/crossbot_mm.dll"
    }
} else {
    Write-Warning "no plugins.ini at $pluginsIni"
}

$built = Join-Path (Split-Path $PSScriptRoot -Parent) "build\crossbot_mm.dll"
if ((Test-Path $built) -and (Test-Path $deployed)) {
    if ((Get-Item $built).LastWriteTime -gt (Get-Item $deployed).LastWriteTime) {
        Write-Warning "$deployed is older than build\crossbot_mm.dll -- rerun build.bat, the server was probably still up."
    }
}

$launchArgs = @(
    "-console", "-condebug", "-insecure", "-game", $GameDir
)
for ($i = 0; $i -lt $Games.Count; $i++) {
    $launchArgs += @("-xgame$($i + 1)", $Games[$i], "-sgame$($i + 1)", $Games[$i])
}
$launchArgs += @(
    "+sv_lan", "1", "+deathmatch", "1", "+developer", "2", "+maxplayers", "16",
    "+rcon_password", "crossbot_test", "+port", "$Port"
)
if ($ExtraArgs.Count -gt 0) { $launchArgs += $ExtraArgs }
$launchArgs += @("+map", $Map)

if ($Commands.Count -gt 0) {
    $cfgPath = Join-Path $HLDSDir "$GameDir\$ConfigName"
    (($Commands -join "`n") + "`n") | Set-Content -Path $cfgPath -NoNewline
}

$quoted = $launchArgs | ForEach-Object { if ($_ -match '\s') { '"' + $_ + '"' } else { $_ } }
$inner = 'waitfor /t {0} cbstdin{1} | "{2}" {3} 1>"{4}" 2>"{5}"' -f `
    ($StdinKeepaliveSeconds, $Port, $exe, ($quoted -join ' '), $LogFile, $errFile)

Start-Process -FilePath "cmd.exe" -ArgumentList @("/c", $inner) `
    -WorkingDirectory $HLDSDir -NoNewWindow | Out-Null

$deadline = (Get-Date).AddSeconds($WaitSeconds)
$ready = $false
$serverPid = 0
while ((Get-Date) -lt $deadline) {
    $probe = try { & "$PSScriptRoot\rcon.ps1" -Port $Port -Command "echo crossbot_ready" -FirstTimeoutMs 1000 | Out-String } catch { "" }
    if ($probe -match "crossbot_ready") { $ready = $true; break }
    Start-Sleep -Milliseconds 300
}

$found = Get-CimInstance Win32_Process -Filter "Name='hlds.exe'" -ErrorAction SilentlyContinue |
    Where-Object { $_.CommandLine -and $_.CommandLine -match "\+port\s+$Port(\s|$)" }
if ($found) { $serverPid = @($found)[0].ProcessId }

if (-not $serverPid) {
    "EXITED code=-1 (no hlds.exe on port $Port, see $LogFile)"
    exit
}

if ($ready -and $RequireVerbose) {
    $probe = try { & "$PSScriptRoot\rcon.ps1" -Port $Port -Command "crossbot_verbose" | Out-String } catch { "" }
    if ($probe -notmatch 'crossbot_verbose"?\s+is') {
        Stop-Process -Id $serverPid -Force -ErrorAction SilentlyContinue
        "EXITED code=-3 (crossbot_verbose cvar not registered -- is crossbot_mm.dll in plugins.ini, and was it built with -DCROSSBOT_VERBOSE=ON?)"
        exit
    }
}

if ($ready) {
    & "$PSScriptRoot\rcon.ps1" -Port $Port -Command "crossbot_verbose 1" | Out-Null
}

if ($ready -and $Commands.Count -gt 0) {
    & "$PSScriptRoot\rcon.ps1" -Port $Port -Command "exec $ConfigName" | Out-Null
    Remove-Item $cfgPath -ErrorAction SilentlyContinue
}

if (-not $ready) {
    Stop-Process -Id $serverPid -Force -ErrorAction SilentlyContinue
    "EXITED code=-2 (map '$Map' did not load within ${WaitSeconds}s, see $LogFile)"
} else {
    "PID=$serverPid"
}
