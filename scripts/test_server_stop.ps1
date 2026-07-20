param(
    [int]$Id = 0
)

$hldsExe = "D:\projects\trash\crossbot\hlds\hlds.exe"

if ($Id -ne 0) {
    Stop-Process -Id $Id -Force -ErrorAction SilentlyContinue
} else {
    Get-Process -Name hlds -ErrorAction SilentlyContinue |
        Where-Object { $_.Path -eq $hldsExe } |
        Stop-Process -Force -ErrorAction SilentlyContinue
}

Start-Sleep -Milliseconds 500
"stopped"
