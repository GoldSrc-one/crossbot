param(
    [Parameter(Mandatory=$true)]
    [string]$Command,
    [string]$RconPassword = "crossbot_test",
    [string]$ServerAddr = "127.0.0.1",
    [int]$Port = 29015,
    # How long to wait for the server's first packet. A live server answers in
    # milliseconds; this budget only matters while it is still booting.
    [int]$FirstTimeoutMs = 3000,
    # How long to wait for each *additional* packet of a multi-packet reply.
    # Draining at the full timeout costs that budget on every single call, which
    # across a poll loop over ~45 maps dominates the whole run. Do not cut this
    # much further: a dozen servers sharing the box can leave gaps of several
    # hundred ms between packets, and a short drain silently truncates the reply
    # -- which reads downstream as "this map has no info_target".
    [int]$DrainTimeoutMs = 1000
)

# The caller may run under $ErrorActionPreference = 'Stop'. A server that is not
# up yet is a normal, expected outcome here (callers poll), so it must return
# empty rather than throw a terminating error into the middle of a test.
$ErrorActionPreference = "Continue"

# GoldSrc out-of-band rcon protocol: request a challenge, then send the
# command tagged with that challenge + password. Lets a live test server be
# driven mid-run instead of only via a scripted +exec cfg at startup (which
# can't observe real elapsed wall-clock time -- see the Milestone 2d
# avoidance test notes for why that matters).

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
# Always return a string, never $null: callers regex the result, and a null
# input throws rather than simply not matching.
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
# Strip the GoldSrc RCON response prefix: 0xFF 0xFF 0xFF 0xFF 'l'. It heads every
# packet, not just the first, so a long reply carries one at each packet boundary
# -- and that boundary can land mid-line, splitting an entity the caller is
# regexing for. Remove all of them, not only the leading one.
$marker = ([string][char]0xFF) * 4 + "l"
$out = $out.Replace($marker, "")
$out
