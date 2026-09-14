param(
    [Parameter(Mandatory=$true)]
    [int]$Id
)

$port = 0
$proc = Get-CimInstance Win32_Process -Filter "ProcessId=$Id" -ErrorAction SilentlyContinue
if ($proc -and $proc.CommandLine -match "\+port\s+(\d+)") { $port = [int]$matches[1] }

Stop-Process -Id $Id -Force -ErrorAction SilentlyContinue

if ($port) {
    Get-CimInstance Win32_Process -Filter "Name='waitfor.exe'" -ErrorAction SilentlyContinue |
        Where-Object { $_.CommandLine -and $_.CommandLine -match "cbstdin$port(\s|$)" } |
        ForEach-Object { Stop-Process -Id $_.ProcessId -Force -ErrorAction SilentlyContinue }
}

Start-Sleep -Milliseconds 500
"stopped"
