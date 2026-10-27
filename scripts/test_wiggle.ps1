param(
    [string]$Map = "de_dust2",
    [string[]]$Games = @('cstrike', 'dod', 'valve', 'tfc'),
    [int]$BotsMax = 8,
    [int]$Seconds = 240,
    [int]$Port = 0,
    [double]$Window = 5,
    [double]$Spread = 96,
    [int]$Reversals = 3,
    [double]$MaxSpeed = 60,
    [string[]]$Commands = @(),
    [string]$HLDSDir = "D:\hlds"
)

$ErrorActionPreference = 'Stop'
$scriptDir = $PSScriptRoot
$repoRoot = Split-Path $scriptDir -Parent
. "$scriptDir\common.ps1"

if ($Port -eq 0) { $Port = 29015 + 10 * (Get-Random -Maximum 8) }

$logDir = Join-Path $repoRoot "nav_debug\logs"
$logFile = Join-Path $logDir "${Map}_wiggle.log"
$samplesFile = Join-Path $logDir "${Map}_wiggle_samples.txt"
$reportFile = Join-Path $logDir "${Map}_wiggle_report.txt"
Remove-Item $samplesFile, $reportFile -ErrorAction SilentlyContinue

$boot = @('mp_timelimit 0', "bots_max $BotsMax", 'crossbot_camp_debug 1', 'crossbot_strategic_debug 1') + $Commands

function Rc([string]$c, [int]$drain = 1000) {
    $o = try { & "$scriptDir\rcon.ps1" -Port $Port -Command $c -DrainTimeoutMs $drain } catch { "" }
    if ($null -eq $o) { return "" }
    return ($o | Out-String)
}

$start = & "$scriptDir\test_server_start.ps1" -Map $Map -Port $Port -Games $Games -WaitSeconds 120 `
    -Commands $boot -ConfigName "_crossbot_wiggle_$Map.cfg" -HLDSDir $HLDSDir -LogFile $logFile -RequireVerbose
$serverPid = Get-CbServerPid $start
Write-Host "Server PID=$serverPid port=$Port"

$hist = @{}
$lastFlag = @{}
$flags = New-Object System.Collections.ArrayList
$re = 'CrossBot: (\S+) origin=\(([-\d]+),([-\d]+),([-\d]+)\) vel=\(([-\d]+),([-\d]+),([-\d]+)\) angles=\(([-\d]+),([-\d]+)\) move=\(([-\d]+),([-\d]+)\) hasGoal=(\d) .*?waypoint=(\d+)/(\d+).*? tier=(\S+) stopTier=(\S+)'

try {
    $t0 = Get-Date
    while (((Get-Date) - $t0).TotalSeconds -lt $Seconds) {
        $now = ((Get-Date) - $t0).TotalSeconds
        $out = Rc "crossbot_nav_status" 120
        foreach ($line in ($out -split "`n")) {
            if ($line -notmatch $re) { continue }
            $s = [pscustomobject]@{
                T = $now; Name = $Matches[1]
                X = [double]$Matches[2]; Y = [double]$Matches[3]; Z = [double]$Matches[4]
                VX = [double]$Matches[5]; VY = [double]$Matches[6]
                Yaw = [double]$Matches[9]; Fwd = [double]$Matches[10]; Side = [double]$Matches[11]
                Goal = $Matches[12]; Wp = "$($Matches[13])/$($Matches[14])"; Tier = $Matches[15]; Stop = $Matches[16]
            }
            $s | Add-Member Speed ([math]::Sqrt($s.VX * $s.VX + $s.VY * $s.VY))
            Add-Content $samplesFile ("{0:F2} {1} ({2},{3},{4}) vel=({5},{6}) yaw={7} move=({8},{9}) goal={10} wp={11} tier={12}/{13}" -f `
                $s.T, $s.Name, $s.X, $s.Y, $s.Z, $s.VX, $s.VY, $s.Yaw, $s.Fwd, $s.Side, $s.Goal, $s.Wp, $s.Tier, $s.Stop)

            if (-not $hist.ContainsKey($s.Name)) { $hist[$s.Name] = New-Object System.Collections.ArrayList }
            $h = $hist[$s.Name]
            [void]$h.Add($s)
            while ($h.Count -gt 0 -and $h[0].T -lt $now - $Window) { $h.RemoveAt(0) }
            if ($h.Count -lt 6 -or ($h[$h.Count - 1].T - $h[0].T) -lt $Window * 0.8) { continue }

            $mx = ($h | Measure-Object X -Average).Average
            $my = ($h | Measure-Object Y -Average).Average
            $far = ($h | ForEach-Object { [math]::Sqrt(($_.X - $mx) * ($_.X - $mx) + ($_.Y - $my) * ($_.Y - $my)) } |
                Measure-Object -Maximum).Maximum
            $flips = 0
            for ($j = 1; $j -lt $h.Count; $j++) {
                $a = $h[$j - 1]; $b = $h[$j]
                if ($a.Speed -gt 10 -and $b.Speed -gt 10 -and ($a.VX * $b.VX + $a.VY * $b.VY) -lt 0) { $flips++ }
            }
            $avg = ($h | Measure-Object Speed -Average).Average
            if ($far -ge $Spread -or $flips -lt $Reversals -or $avg -ge $MaxSpeed) { continue }
            if ($lastFlag.ContainsKey($s.Name) -and $now - $lastFlag[$s.Name] -lt 10) { continue }
            $lastFlag[$s.Name] = $now

            $msg = "{0:F1}s WIGGLE {1} at ({2:F0},{3:F0},{4:F0}) spread={5:F0} flips={6} samples={7} goal={8} wp={9} tier={10}/{11}" -f `
                $now, $s.Name, $mx, $my, $s.Z, $far, $flips, $h.Count, $s.Goal, $s.Wp, $s.Tier, $s.Stop
            Write-Host $msg
            [void]$flags.Add($msg)
            Add-Content $reportFile "`n===== $msg"
            Add-Content $reportFile $line
            foreach ($c in 'crossbot_combat_status', 'crossbot_vantage_status', 'crossbot_camp_status',
                'crossbot_directive_status', 'crossbot_sense_status', 'crossbot_waypoints') {
                $r = (Rc $c) -split "`n" | Where-Object { $_ -match [regex]::Escape($s.Name) -or $_ -notmatch 'CrossBot: \S+ ' }
                Add-Content $reportFile "--- $c"
                Add-Content $reportFile ($r -join "`n")
            }
        }
    }
}
finally {
    & "$scriptDir\test_server_stop.ps1" -Id $serverPid | Out-Null
}

Write-Host "`n=== $($flags.Count) wiggle episode(s); samples $samplesFile, report $reportFile, log $logFile ==="
$flags | ForEach-Object { ($_ -split ' ')[2] } | Group-Object | ForEach-Object { "  $($_.Name): $($_.Count)" }
