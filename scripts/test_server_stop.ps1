param(
    [Parameter(Mandatory=$true)]
    [int]$Id
)

# Only ever stops the one PID this session's own test_server_start.ps1
# call returned -- never scans for/kills every hlds.exe on the machine,
# since the user may have other HLDS instances running independently.
$port = 0
$proc = Get-CimInstance Win32_Process -Filter "ProcessId=$Id" -ErrorAction SilentlyContinue
if ($proc -and $proc.CommandLine -match "\+port\s+(\d+)") { $port = [int]$matches[1] }

Stop-Process -Id $Id -Force -ErrorAction SilentlyContinue

# The server's stdin pipe is held open by a `waitfor` that would otherwise sit there
# until its own timeout -- see the launch note in test_server_start.ps1. It is tagged
# with the port, so this never touches another instance's writer.
if ($port) {
    Get-CimInstance Win32_Process -Filter "Name='waitfor.exe'" -ErrorAction SilentlyContinue |
        Where-Object { $_.CommandLine -and $_.CommandLine -match "cbstdin$port(\s|$)" } |
        ForEach-Object { Stop-Process -Id $_.ProcessId -Force -ErrorAction SilentlyContinue }
}

Start-Sleep -Milliseconds 500
"stopped"
