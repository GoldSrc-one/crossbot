param(
    [Parameter(Mandatory=$true)]
    [int]$Id
)

# Only ever stops the one PID this session's own test_server_start.ps1
# call returned -- never scans for/kills every hlds.exe on the machine,
# since the user may have other HLDS instances running independently.
Stop-Process -Id $Id -Force -ErrorAction SilentlyContinue

Start-Sleep -Milliseconds 500
"stopped"
