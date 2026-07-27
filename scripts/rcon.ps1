param(
    [Parameter(Mandatory=$true)]
    [string]$Command,
    [string]$RconPassword = "crossbot_test",
    [string]$ServerAddr = "127.0.0.1",
    [int]$Port = 29015
)

# GoldSrc out-of-band rcon protocol: request a challenge, then send the
# command tagged with that challenge + password. Lets a live test server be
# driven mid-run instead of only via a scripted +exec cfg at startup (which
# can't observe real elapsed wall-clock time -- see the Milestone 2d
# avoidance test notes for why that matters).

$udp = New-Object System.Net.Sockets.UdpClient
$udp.Client.ReceiveTimeout = 3000
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
$challengeResp = Recv-Raw
if ($challengeResp -notmatch "challenge rcon (-?\d+)") {
    Write-Error "no challenge in response: $challengeResp"
    $udp.Close()
    exit 1
}
$challenge = $matches[1]

$cmdStr = "rcon $challenge $RconPassword $Command`0"
Send-Raw ($prefix + [System.Text.Encoding]::ASCII.GetBytes($cmdStr))

$out = ""
try {
    while ($true) {
        $out += Recv-Raw
    }
} catch {}

$udp.Close()
# Strip GoldSrc RCON response prefix: 0xFF 0xFF 0xFF 0xFF 'l' (4 bytes + 'l')
$out = $out -replace "^....l", ""
$out
