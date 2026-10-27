param(
    [int]$Port = 29399,
    [int]$Seconds = 600,
    [string]$Log = ""
)

$listener = New-Object System.Net.HttpListener
$listener.Prefixes.Add("http://127.0.0.1:$Port/")
$listener.Start()
$deadline = (Get-Date).AddSeconds($Seconds)

function Write-MockLog([string]$line) {
    if ($Log) { Add-Content -Path $Log -Value $line }
}

function Send-Json($ctx, [int]$status, [string]$json) {
    $bytes = [Text.Encoding]::UTF8.GetBytes($json)
    $ctx.Response.StatusCode = $status
    $ctx.Response.ContentType = "application/json"
    $ctx.Response.OutputStream.Write($bytes, 0, $bytes.Length)
    $ctx.Response.Close()
}

function Get-Trigger([string]$text) {
    $m = [regex]::Match($text, "(?m)^Message to judge: (.*)$")
    if ($m.Success) { return $m.Groups[1].Value }
    $m = [regex]::Match($text, "(?s)## Trigger\n(.*?)(\n## |$)")
    if ($m.Success) { return $m.Groups[1].Value.Trim() }
    return ""
}

function Get-Speaker([string]$trigger) {
    $m = [regex]::Match($trigger, "\) ([^:*]+?)( \*DEAD\*)?: ")
    if ($m.Success) { return $m.Groups[1].Value.Trim() }
    return ""
}

function Get-Bots([string]$context) {
    return @([regex]::Matches($context, "(?m)^- (\S+) \[bot,") | ForEach-Object { $_.Groups[1].Value })
}

while ((Get-Date) -lt $deadline) {
    $task = $listener.GetContextAsync()
    while (-not $task.Wait(500)) { if ((Get-Date) -ge $deadline) { break } }
    if (-not $task.IsCompleted) { break }
    $ctx = $task.Result
    $reader = New-Object IO.StreamReader($ctx.Request.InputStream, [Text.Encoding]::UTF8)
    $raw = $reader.ReadToEnd()
    $auth = $ctx.Request.Headers["Authorization"]
    try { $req = $raw | ConvertFrom-Json -ErrorAction Stop } catch {
        Write-MockLog "BADJSON $($ctx.Request.Url.AbsolutePath): $($_.Exception.Message)"
        Send-Json $ctx 400 '{"error":{"message":"body is not JSON"}}'
        continue
    }
    if ($auth -ne "Bearer mock-key") {
        Send-Json $ctx 401 '{"error":{"message":"bad key"}}'
        continue
    }

    if ($ctx.Request.Url.AbsolutePath -like "*audio/speech*") {
        $script:lastSpeech = $req.input
        $rate = 44100; $dur = 1.5; $n = [int]($rate * $dur)
        $bytes = New-Object byte[] ($n * 2)
        for ($i = 0; $i -lt $n; $i++) {
            $v = [int16](8000 * [Math]::Sin(2 * [Math]::PI * 300 * $i / $rate))
            $bytes[2 * $i] = $v -band 0xFF; $bytes[2 * $i + 1] = ($v -shr 8) -band 0xFF
        }
        Write-MockLog "TTS input=$($req.input) format=$($req.response_format)"
        $ctx.Response.StatusCode = 200
        $ctx.Response.ContentType = "audio/pcm; rate=$rate; channels=1"
        $ctx.Response.OutputStream.Write($bytes, 0, $bytes.Length)
        $ctx.Response.Close()
        continue
    }

    if ($ctx.Request.Url.AbsolutePath -like "*audio/transcriptions*") {
        $wav = [Convert]::FromBase64String($req.input_audio.data)
        $rate = [BitConverter]::ToInt32($wav, 24)
        $n = ($wav.Length - 44) / 2
        $sum = 0.0; $cross = 0; $prev = 0
        for ($i = 0; $i -lt $n; $i++) {
            $v = [BitConverter]::ToInt16($wav, 44 + 2 * $i)
            $sum += [double]$v * $v
            if (($v -ge 0) -ne ($prev -ge 0)) { $cross++ }
            $prev = $v
        }
        $dur = $n / $rate
        $rms = [Math]::Sqrt($sum / [Math]::Max(1, $n))
        $freq = $cross / 2 / [Math]::Max(0.001, $dur)
        $good = $rate -eq 16000 -and [Math]::Abs($dur - 1.5) -lt 0.3 -and $rms -gt 2000 -and [Math]::Abs($freq - 300) -lt 30
        $text = if ($good) { $script:lastSpeech } else { "garbled" }
        Write-MockLog ("STT rate={0} seconds={1:N2} rms={2:N0} freq={3:N0} -> {4}" -f $rate, $dur, $rms, $freq, $text)
        Send-Json $ctx 200 (@{ text = $text; usage = @{ seconds = $dur } } | ConvertTo-Json)
        continue
    }

    if ($ctx.Request.Url.AbsolutePath -like "*decisions*") {
        $trigger = Get-Trigger $req.state
        $ids = @($req.questions.PSObject.Properties.Name)
        $act = if ($trigger -match "lol") { 0.05 } else { 0.9 }
        $ignore = if ($trigger -match "STUPID") { 3.0 } else { 0.0 }
        $recheck = if ($trigger -match "check") { 1.0 } else { 0.0 }
        $remember = if ($trigger -match "remember") { 0.95 } else { 0.05 }
        $answers = @{}
        foreach ($id in $ids) {
            switch -Wildcard ($id) {
                "act" { $answers[$id] = @{ type = "noul"; noul = $act } }
                "remember" { $answers[$id] = @{ type = "noul"; noul = $remember } }
                "ignore" { $answers[$id] = @{ type = "score"; score = $ignore; confidence = 0.9 } }
                "recheck" { $answers[$id] = @{ type = "score"; score = $recheck; confidence = 0.9 } }
                "involve_*" { $answers[$id] = @{ type = "noul"; noul = 0.8 } }
            }
        }
        Write-MockLog "JEV questions=$($ids.Count) trigger=$trigger act=$act ignore=$ignore recheck=$recheck"
        Send-Json $ctx 200 (@{ model = "mock-jev"; answers = $answers; usage = @{ input_tokens = 300; output_tokens = 0 } } | ConvertTo-Json -Depth 6)
        continue
    }

    $user = ($req.messages | Where-Object { $_.role -eq "user" }).content
    $trigger = Get-Trigger $user
    $speaker = Get-Speaker $trigger
    $bots = @(Get-Bots $user | Where-Object { $_ -ne $speaker })
    $bot = if ($bots.Count -gt 0) { $bots[0] } else { "nobody" }
    $calls = @()
    if ($trigger -match "team voice\) .*follow me") {
        $calls += @{ name = "act"; args = @{ bot = $bot; steps = @(@{ do = "follow"; target = "me"; seconds = 30 }) } }
        $calls += @{ name = "speak"; args = @{ bot = $bot; text = "on my way" } }
    } elseif ($trigger -match "follow me") {
        $calls += @{ name = "act"; args = @{ bot = $bot; steps = @(@{ do = "follow"; target = "me"; seconds = 30 }) } }
        $calls += @{ name = "radio"; args = @{ bot = $bot; command = "roger" } }
    } elseif ($trigger -match "remember") {
        $calls += @{ name = "remember"; args = @{ text = "$speaker likes the AWP"; players = $speaker } }
    } elseif ($trigger -match "Re-check") {
        $calls += @{ name = "say_team"; args = @{ bot = $bot; text = "checked, all quiet" } }
    } else {
        $calls += @{ name = "say_team"; args = @{ bot = $bot; text = "ok" } }
    }
    $toolCalls = @()
    $i = 0
    foreach ($c in $calls) {
        $toolCalls += @{ id = "call_$i"; type = "function"; function = @{ name = $c.name; arguments = ($c.args | ConvertTo-Json -Depth 6 -Compress) } }
        $i++
    }
    Write-MockLog "LLM tools=$(@($req.tools).Count) trigger=$trigger speaker=$speaker -> $(($calls | ForEach-Object { $_.name }) -join ',')"
    $resp = @{
        choices = @(@{ index = 0; message = @{ role = "assistant"; content = $null; tool_calls = $toolCalls } })
        usage = @{ prompt_tokens = 4000; completion_tokens = 60 }
    }
    Send-Json $ctx 200 ($resp | ConvertTo-Json -Depth 8)
}
$listener.Stop()
