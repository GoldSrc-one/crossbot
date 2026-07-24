param(
    [string]$Map = "crossfire",
    [int]$WaitSeconds = 15,
    # Non-default port -- avoids colliding with any other HLDS instance
    # that might already be running on the machine's default 27015.
    [int]$Port = 29015,
    # Console commands to run once the map has finished loading. Implemented
    # via a generated cfg file exec'd after +map, with a "wait" buffer first
    # -- game.cfg/server.cfg run too early (before the level is active), and
    # a per-map cfg wasn't observed to run at all on this dedicated server.
    [string[]]$Commands = @(),
    # Unique config filename (default: _crossbot_autotest.cfg). Use map-specific
    # name to avoid collisions when running parallel tests on different maps.
    [string]$ConfigName = "_crossbot_autotest.cfg"
)

$hldsDir = "D:\projects\trash\crossbot\hlds"
$exe = Join-Path $hldsDir "hlds.exe"

# qconsole.log is shared - don't delete it here, let the test orchestrator handle it
# Remove-Item (Join-Path $hldsDir "qconsole.log") -ErrorAction SilentlyContinue

$launchArgs = "-console -condebug -insecure -game valve +sv_lan 1 +deathmatch 1 +developer 2 +maxplayers 16 +rcon_password crossbot_test +port $Port +map $Map"

if ($Commands.Count -gt 0) {
    $cfgPath = Join-Path $hldsDir "valve\$ConfigName"
    $waits = (@("wait") * 40) -join "`n"
    ($waits + "`n" + ($Commands -join "`n") + "`n") | Set-Content -Path $cfgPath -NoNewline
    $launchArgs += " +exec $ConfigName"
}

$psi = New-Object System.Diagnostics.ProcessStartInfo
$psi.FileName = $exe
$psi.Arguments = $launchArgs
$psi.WorkingDirectory = $hldsDir
$psi.UseShellExecute = $true

$proc = [System.Diagnostics.Process]::Start($psi)

Add-Type @"
using System;
using System.Runtime.InteropServices;
public class CrossBotWin32 {
    [DllImport("user32.dll")]
    public static extern bool ShowWindow(IntPtr hWnd, int nCmdShow);
}
"@ -ErrorAction SilentlyContinue

$deadline = (Get-Date).AddSeconds($WaitSeconds)
while ((Get-Date) -lt $deadline) {
    $proc.Refresh()
    if ($proc.HasExited) { break }
    if ($proc.MainWindowHandle -ne 0) {
        [CrossBotWin32]::ShowWindow($proc.MainWindowHandle, 0) | Out-Null
        break
    }
    Start-Sleep -Milliseconds 300
}

$proc.Refresh()
if ($proc.HasExited) {
    "EXITED code=$($proc.ExitCode)"
} else {
    "PID=$($proc.Id)"
}
