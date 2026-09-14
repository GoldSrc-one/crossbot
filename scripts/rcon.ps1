param(
    [Parameter(Mandatory=$true)]
    [string]$Command,
    [string]$RconPassword = "crossbot_test",
    [string]$ServerAddr = "127.0.0.1",
    [int]$Port = 29015,
    [int]$FirstTimeoutMs = 3000,
    [int]$DrainTimeoutMs = 1000
)

$ErrorActionPreference = "Continue"


$udp = New-Object System.Net.Sockets.UdpClient
$udp.Client.ReceiveTimeout = $FirstTimeoutMs
$endpoint = New-Object System.Net.IPEndPoint ([System.Net.IPAddress]::Parse($ServerAddr), $Port)
$udp.Connect($endpoint)

function Send-Raw([byte[]]$bytes) {
    $udp.Send($bytes, $bytes.Length) | Out-Null
}

function Recv-Raw() {
    $remote = New-Object System.Net.IPEndPoint ([System.Net.IPAddress]::Any, 0)
    $data = $udp.Receive([ref]$remote)
    return [System.Text.Encoding]::GetEncoding(1252).GetString($data)
}

$prefix = [byte[]](0xFF,0xFF,0xFF,0xFF)

Send-Raw ($prefix + [System.Text.Encoding]::ASCII.GetBytes("challenge rcon`0"))
try {
    $challengeResp = Recv-Raw
} catch {
    $udp.Close()
    return ""
}
if ($challengeResp -notmatch "challenge rcon (-?\d+)") {
    $udp.Close()
    return ""
}
$challenge = $matches[1]

$cmdStr = "rcon $challenge $RconPassword $Command`0"
Send-Raw ($prefix + [System.Text.Encoding]::ASCII.GetBytes($cmdStr))

$out = ""
try {
    $out += Recv-Raw
    $udp.Client.ReceiveTimeout = $DrainTimeoutMs
    while ($true) {
        $out += Recv-Raw
    }
} catch {}

$udp.Close()
$marker = ([string][char]0xFF) * 4 + "l"
$out = $out.Replace($marker, "")
$out
