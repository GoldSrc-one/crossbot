<#
.SYNOPSIS
    Turns the raw DPS rows into damage-per-second against range, per weapon phase.

.DESCRIPTION
    Attribution is by time, not by any field on the row. A dmg or shot row
    carries no pin, so each one belongs to the phase named by the first meta row
    at or after its timestamp: BotFire::Pin calls BotDps::PinChange before it
    stores the new pin, so that meta row names the phase that just ended, and the
    periodic 20s flush names the phase still running.

    The estimator is total damage over total gaze, not the mean of the per-row
    ratios. A row whose gaze is a few milliseconds would otherwise dominate the
    phase on its own.

    A phase is keyed by pin, mode and pinned zoom state. Rows written before the
    zoom, fatal and release-reason fields existed still parse; they report as
    zoom any, zero kills and a release reason of -.

.PARAMETER Csv
    One or more raw CSVs. Defaults to every map in nav_debug\dps.

.PARAMETER PerMap
    Report each map separately instead of pooling them.

.PARAMETER Ranges
    Add the DPS-against-distance curve and the per-bucket sample counts.
#>
param(
    [string[]]$Csv = @(),
    [string]$InputDir = "nav_debug\dps",
    [switch]$PerMap,
    [switch]$Ranges
)

$ErrorActionPreference = 'Stop'
$repoRoot = Split-Path $PSScriptRoot -Parent

if ($Csv.Count -eq 0) {
    $dir = Join-Path $repoRoot $InputDir
    $Csv = @(Get-ChildItem -Path $dir -Filter '*.csv' -ErrorAction SilentlyContinue |
        Where-Object { $_.Name -ne 'manifest.csv' } | ForEach-Object { $_.FullName })
}

if ($Csv.Count -eq 0) {
    Write-Host "analyze_dps: no CSVs found"
    exit 1
}

$edges = @(0, 192, 384, 640, 1024, 1536, 2048, 3072, [double]::MaxValue)
$edgeNames = @('0-192', '192-384', '384-640', '640-1k', '1k-1.5k', '1.5k-2k', '2k-3k', '3k+')

function Get-Bucket {
    param([double]$Dist)
    for ($i = 0; $i -lt $edgeNames.Count; $i++) {
        if ($Dist -lt $edges[$i + 1]) { return $i }
    }
    return $edgeNames.Count - 1
}

$dmgBits = @(1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024, 4096, 8192, 16384)
$dmgNames = @('crush', 'bullet', 'slash', 'burn', 'freeze', 'fall', 'blast', 'club',
    'shock', 'sonic', 'energybeam', 'nevergib', 'alwaysgib', 'drown')

function Get-BitNames {
    param([int]$Bits)
    if ($Bits -eq 0) { return 'generic' }
    $out = @()
    $rest = $Bits
    for ($i = 0; $i -lt $dmgBits.Count; $i++) {
        if (($Bits -band $dmgBits[$i]) -ne 0) {
            $out += $dmgNames[$i]
            $rest = $rest -band (-bnot $dmgBits[$i])
        }
    }
    if ($rest -ne 0) { $out += "+$rest" }
    return ($out -join '|')
}

# --- Parse -----------------------------------------------------------------

$phases = @{}

function Get-Phase {
    param([string]$Key, [string]$Map, [string]$Pin, [string]$Mode, [string]$Zoom = 'any')
    if (-not $phases.ContainsKey($Key)) {
        $phases[$Key] = [pscustomobject]@{
            Map = $Map; Pin = $Pin; Mode = $Mode; Zoom = $Zoom
            Shots = 0; Rows = 0; Damage = 0.0; Gaze = 0.0
            Kills = 0; ZoomShots = 0
            SelfRows = 0; SelfDamage = 0.0
            WorldRows = 0; WorldDamage = 0.0
            NoGazeRows = 0; NoGazeDamage = 0.0
            Span = 0.0
            Bits = @{}; Inflictors = @{}; Why = @{}
            BinDamage = New-Object 'double[]' $edgeNames.Count
            BinGaze = New-Object 'double[]' $edgeNames.Count
            BinRows = New-Object 'int[]' $edgeNames.Count
        }
    }
    return $phases[$Key]
}

foreach ($path in $Csv) {
    if (-not (Test-Path $path)) { continue }
    $mapLabel = if ($PerMap) { [System.IO.Path]::GetFileNameWithoutExtension($path) } else { '*' }

    $lines = Get-Content $path
    $meta = @()
    $events = @()

    foreach ($line in $lines) {
        $f = $line.Split(',')
        switch ($f[0]) {
            'meta' {
                if ($f.Count -lt 8) { break }
                $meta += [pscustomobject]@{
                    T = [double]$f[1]; Map = $f[2]; Pin = $f[3]; Mode = $f[4]
                    Span = [double]$f[6]
                    Zoom = if ($f.Count -ge 9) { $f[8] } else { 'any' }
                }
            }
            'shot' {
                if ($f.Count -lt 10) { break }
                $events += [pscustomobject]@{
                    Kind = 'shot'; T = [double]$f[1]
                    Zoomed = [int]$f[5]
                    Why = if ($f.Count -ge 11) { $f[10] } else { '-' }
                    Held = if ($f.Count -ge 12) { [double]$f[11] } else { -1.0 }
                }
            }
            'dmg' {
                if ($f.Count -lt 9) { break }
                $events += [pscustomobject]@{
                    Kind = 'dmg'; T = [double]$f[1]
                    Victim = $f[2]; Attacker = $f[3]
                    Damage = [double]$f[4]; Seconds = [double]$f[5]
                    Dist = [double]$f[6]; Bits = [int]$f[7]; Inflictor = $f[8]
                    Fatal = if ($f.Count -ge 10) { [int]$f[9] } else { 0 }
                }
            }
        }
    }

    if ($meta.Count -eq 0) {
        Write-Host "analyze_dps: $path has no meta rows, cannot attribute"
        continue
    }

    $mi = 0
    foreach ($e in $events) {
        while ($mi -lt $meta.Count - 1 -and $meta[$mi].T -lt $e.T) { $mi++ }
        $m = $meta[$mi]
        if ($m.Pin -eq 'off') { continue }

        $key = if ($PerMap) { "$mapLabel/$($m.Pin)/$($m.Mode)/$($m.Zoom)" }
               else { "$($m.Pin)/$($m.Mode)/$($m.Zoom)" }
        $p = Get-Phase $key $mapLabel $m.Pin $m.Mode $m.Zoom

        if ($e.Kind -eq 'shot') {
            $p.Shots++
            if ($e.Zoomed -ne 0) { $p.ZoomShots++ }
            if (-not $p.Why.ContainsKey($e.Why)) { $p.Why[$e.Why] = 0 }
            $p.Why[$e.Why]++
            continue
        }

        if ($e.Attacker -eq '?') {
            $p.WorldRows++; $p.WorldDamage += $e.Damage; continue
        }
        if ($e.Attacker -eq $e.Victim) {
            $p.SelfRows++; $p.SelfDamage += $e.Damage; continue
        }

        $p.Rows++
        $p.Damage += $e.Damage
        $p.Gaze += $e.Seconds
        if ($e.Fatal -ne 0) { $p.Kills++ }

        $bn = Get-BitNames $e.Bits
        if (-not $p.Bits.ContainsKey($bn)) { $p.Bits[$bn] = 0 }
        $p.Bits[$bn]++
        if (-not $p.Inflictors.ContainsKey($e.Inflictor)) { $p.Inflictors[$e.Inflictor] = 0 }
        $p.Inflictors[$e.Inflictor]++

        if ($e.Seconds -le 0) { $p.NoGazeRows++; $p.NoGazeDamage += $e.Damage }

        if ($e.Dist -ge 0) {
            $b = Get-Bucket $e.Dist
            $p.BinDamage[$b] += $e.Damage
            $p.BinGaze[$b] += $e.Seconds
            $p.BinRows[$b]++
        }
    }

    foreach ($m in $meta) {
        if ($m.Pin -eq 'off') { continue }
        $key = if ($PerMap) { "$mapLabel/$($m.Pin)/$($m.Mode)/$($m.Zoom)" }
               else { "$($m.Pin)/$($m.Mode)/$($m.Zoom)" }
        (Get-Phase $key $mapLabel $m.Pin $m.Mode $m.Zoom).Span += $m.Span
    }
}

# --- Report ----------------------------------------------------------------

$rows = @()
foreach ($p in $phases.Values) {
    $dps = if ($p.Gaze -gt 0) { $p.Damage / $p.Gaze } else { 0 }
    $perShot = if ($p.Shots -gt 0) { $p.Damage / $p.Shots } else { 0 }
    $rows += [pscustomobject]@{
        Weapon = $p.Pin -replace '^weapon_', ''
        Mode = $p.Mode
        Zoom = $p.Zoom
        Shots = $p.Shots
        Hits = $p.Rows
        Kills = $p.Kills
        Damage = [math]::Round($p.Damage, 0)
        Gaze = [math]::Round($p.Gaze, 1)
        DPS = [math]::Round($dps, 1)
        DmgPerShot = [math]::Round($perShot, 2)
        Self = $p.SelfRows
        NoGaze = $p.NoGazeRows
        NoGazeDmg = [math]::Round($p.NoGazeDamage, 0)
        Span = [math]::Round($p.Span, 0)
        Key = "$($p.Pin)/$($p.Mode)/$($p.Zoom)"
        Phase = $p
    }
}

$rows = @($rows | Sort-Object -Property @{Expression = 'DPS'; Descending = $true})

Write-Host ""
Write-Host "=== DPS by weapon phase (damage / gaze seconds) ==="
$rows | Format-Table Weapon, Mode, Zoom, Shots, Hits, Kills, Damage, Gaze, DPS, DmgPerShot, Self, NoGaze, NoGazeDmg `
    -AutoSize | Out-String -Width 220 | Write-Host

Write-Host "=== damage kinds and inflictors ==="
foreach ($r in $rows) {
    $p = $r.Phase
    $bits = ($p.Bits.GetEnumerator() | Sort-Object Value -Descending |
        ForEach-Object { "$($_.Key)x$($_.Value)" }) -join ' '
    $inf = ($p.Inflictors.GetEnumerator() | Sort-Object Value -Descending |
        ForEach-Object { "$($_.Key)x$($_.Value)" }) -join ' '
    Write-Host ("  {0,-34} {1}" -f $r.Key, $bits)
    Write-Host ("  {0,-34} {1}" -f '', $inf)
}

Write-Host ""
Write-Host "=== shot release reason, and how many shots were zoomed ==="
foreach ($r in $rows) {
    $p = $r.Phase
    if ($p.Shots -eq 0) { continue }
    $why = ($p.Why.GetEnumerator() | Sort-Object Value -Descending |
        ForEach-Object { "$($_.Key)x$($_.Value)" }) -join ' '
    Write-Host ("  {0,-34} zoomed={1}/{2}  {3}" -f $r.Key, $p.ZoomShots, $p.Shots, $why)
}

if ($Ranges) {
    Write-Host ""
    Write-Host "=== DPS against range ==="
    Write-Host ("  {0,-24} {1}" -f 'phase', (($edgeNames | ForEach-Object { "{0,10}" -f $_ }) -join ''))
    foreach ($r in $rows) {
        $p = $r.Phase
        $cells = @()
        for ($i = 0; $i -lt $edgeNames.Count; $i++) {
            if ($p.BinGaze[$i] -gt 0) {
                $cells += "{0,10}" -f ([math]::Round($p.BinDamage[$i] / $p.BinGaze[$i], 1))
            }
            elseif ($p.BinRows[$i] -gt 0) { $cells += "{0,10}" -f '-' }
            else { $cells += "{0,10}" -f '.' }
        }
        Write-Host ("  {0,-24} {1}" -f $r.Key, ($cells -join ''))
        $counts = @()
        for ($i = 0; $i -lt $edgeNames.Count; $i++) { $counts += "{0,10}" -f "n=$($p.BinRows[$i])" }
        Write-Host ("  {0,-24} {1}" -f '', ($counts -join ''))
    }
}

Write-Host ""
Write-Host ("analyze_dps: {0} phase(s) from {1} file(s)" -f $rows.Count, $Csv.Count)
