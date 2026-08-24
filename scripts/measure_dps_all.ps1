<#
.SYNOPSIS
    Runs measure_dps.ps1 over every stock Half-Life deathmatch map in parallel.

.DESCRIPTION
    Two stages. First navgen, serially, over every map in the list -- only
    crossfire ships with a mesh and bots that cannot move measure nothing. Then
    one child per map, throttled, each cycling the whole weapon table.

    MaxParallel defaults to 6 rather than one per core. Six is the measured
    ceiling in doc/testing.md past which box load alone changes bot behaviour,
    and for this sweep that matters more than usual: the aim model is frame-rate
    coupled, so a loaded server measures a worse weapon than an idle one. Each
    run records its achieved frame time in the meta rows, so distortion shows up
    in the data rather than hiding in it.

    The log is appended to and the sweep is resumable. A map whose CSV already
    carries a meta row for every phase is skipped unless -Force, and running the
    whole sweep twice accumulates samples rather than replacing them -- which is
    what sixty-second phases need.

.PARAMETER Seconds
    Seconds per weapon phase, per map.
#>
param(
    [int]$MaxParallel = 6,
    [int]$Bots = 6,
    [int]$Seconds = 60,
    [string[]]$Maps = @(),
    [string[]]$Weapons = @(),
    [switch]$Force,
    [switch]$SkipNavGen,
    [string]$OutputDir = "nav_debug\dps",
    [string]$NavGenPath = "build\tools\navgen\navgen.exe",
    [string]$HLDSDir = "D:\projects\trash\crossbot\hlds"
)

$ErrorActionPreference = 'Stop'
$scriptDir = $PSScriptRoot
$repoRoot = Split-Path $scriptDir -Parent

# The stock HLDM set. hlds\valve\maps also holds ~110 single-player c<n>a<n>
# maps, which were never designed around multi-spawn deathmatch flow.
$allMaps = @(
    'boot_camp', 'bounce', 'contamination', 'crossfire', 'datacore', 'disposal',
    'doublecross', 'frenzy', 'gasworks', 'lambda_bunker', 'pool_party', 'rapidcore',
    'rocket_frenzy', 'rustmill', 'snark_pit', 'stalkyard', 'subtransit', 'undertow',
    'xen_dm'
)

if ($Maps.Count -gt 0) { $allMaps = @($allMaps | Where-Object { $Maps -contains $_ }) }

$phaseCount = 17
if ($Weapons.Count -gt 0) { $phaseCount = 0 }

$outDir = Join-Path $repoRoot $OutputDir
New-Item -ItemType Directory -Force -Path $outDir | Out-Null
New-Item -ItemType Directory -Force -Path (Join-Path $repoRoot "nav_debug\logs") | Out-Null
New-Item -ItemType Directory -Force -Path (Join-Path $HLDSDir "valve\addons\crossbot\dps") | Out-Null

$navGen = Join-Path $repoRoot $NavGenPath
$mapsDir = Join-Path $HLDSDir "valve\maps"
$navDir = Join-Path $HLDSDir "valve\addons\crossbot\nav"

$manifest = @()

# --- Stage 1: meshes -------------------------------------------------------

if (-not $SkipNavGen) {
    Write-Host "measure_dps_all: generating nav for $($allMaps.Count) map(s)"

    foreach ($m in $allMaps) {
        $bsp = Join-Path $mapsDir "$m.bsp"
        if (-not (Test-Path $bsp)) {
            Write-Host "  $m : no bsp, skipped"
            continue
        }

        $nav = Join-Path $navDir "$m.nav"
        $text = & $navGen $bsp -o $nav 2>&1 | Out-String

        $areas = 0; $conns = 0; $gateless = 0; $illformed = 0
        if ($text -match '->\s*(\d+)\s*areas') { $areas = [int]$Matches[1] }
        if ($text -match '(\d+) walk connections') { $conns = [int]$Matches[1] }
        if ($text -match '(\d+) connections have no gate') { $gateless = [int]$Matches[1] }
        if ($text -match '(\d+) areas have an ill-formed') { $illformed = [int]$Matches[1] }

        $manifest += [pscustomobject]@{
            Map = $m; Areas = $areas; Connections = $conns
            Gateless = $gateless; IllFormed = $illformed
            Meta = 0; Shot = 0; Dmg = 0; Result = 'pending'
        }

        Write-Host ("  {0,-16} areas={1,-6} conn={2,-6} gateless={3,-5} illformed={4}" -f `
            $m, $areas, $conns, $gateless, $illformed)
    }
}
else {
    foreach ($m in $allMaps) {
        $manifest += [pscustomobject]@{
            Map = $m; Areas = 0; Connections = 0; Gateless = 0; IllFormed = 0
            Meta = 0; Shot = 0; Dmg = 0; Result = 'pending'
        }
    }
}

# --- Stage 2: servers ------------------------------------------------------

function Get-PhaseCount {
    param([string]$Path)
    if (-not (Test-Path $Path)) { return 0 }
    return @(Get-Content $Path | Where-Object { $_ -like 'meta,*' -and $_ -notlike '*,off,*' }).Count
}

$child = Join-Path $scriptDir "measure_dps.ps1"
$psExe = [System.Diagnostics.Process]::GetCurrentProcess().Path

$queue = New-Object System.Collections.Queue
$index = 0
foreach ($m in $allMaps) {
    $csv = Join-Path $outDir "$m.csv"
    if (-not $Force -and $phaseCount -gt 0 -and (Get-PhaseCount $csv) -ge $phaseCount) {
        Write-Host "measure_dps_all: $m already complete, skipping (use -Force to redo)"
        ($manifest | Where-Object { $_.Map -eq $m }) | ForEach-Object { $_.Result = 'skipped' }
        continue
    }
    $queue.Enqueue(@{ Map = $m; Port = 29215 + $index })
    $index++
}

if ($index -gt 60) {
    Write-Host "measure_dps_all: more maps than the 29215-29275 port pool allows"
    exit 1
}

$tracked = @()
$sw = [System.Diagnostics.Stopwatch]::StartNew()
$deadline = ($Seconds * 17 + 300)

Write-Host "measure_dps_all: $($queue.Count) map(s), MaxParallel $MaxParallel, ${Seconds}s per phase"

while ($queue.Count -gt 0 -or @($tracked | Where-Object { -not $_.Proc.HasExited }).Count -gt 0) {

    while ($queue.Count -gt 0 -and
           @($tracked | Where-Object { -not $_.Proc.HasExited }).Count -lt $MaxParallel) {

        $job = $queue.Dequeue()
        $log = Join-Path $repoRoot "nav_debug\logs\$($job.Map)_dpsrun.log"
        $err = "$log.err"

        $childArgs = @(
            '-NoProfile', '-NonInteractive', '-File', $child,
            '-Map', $job.Map, '-Port', $job.Port, '-Bots', $Bots,
            '-Seconds', $Seconds, '-OutputDir', $OutputDir, '-HLDSDir', $HLDSDir
        )
        if ($Weapons.Count -gt 0) { $childArgs += @('-Weapons', ($Weapons -join ',')) }

        $proc = Start-Process $psExe -ArgumentList $childArgs -NoNewWindow -PassThru `
            -RedirectStandardOutput $log -RedirectStandardError $err

        $tracked += [pscustomobject]@{
            Map = $job.Map; Port = $job.Port; Proc = $proc; Log = $log
            Started = $sw.Elapsed.TotalSeconds
        }

        Write-Host ("  started {0,-16} port {1} pid {2}" -f $job.Map, $job.Port, $proc.Id)
    }

    foreach ($t in $tracked) {
        if ($t.Proc.HasExited) { continue }
        if (($sw.Elapsed.TotalSeconds - $t.Started) -lt $deadline) { continue }

        Write-Host "  timeout $($t.Map), killing"
        Stop-Process -Id $t.Proc.Id -Force -ErrorAction SilentlyContinue
        Get-CimInstance Win32_Process -Filter "Name='hlds.exe'" -ErrorAction SilentlyContinue |
            Where-Object { $_.CommandLine -match "\+port\s+$($t.Port)(\s|$)" } |
            ForEach-Object { Stop-Process -Id $_.ProcessId -Force -ErrorAction SilentlyContinue }
    }

    Start-Sleep -Milliseconds 500
}

$sw.Stop()

# --- Aggregate -------------------------------------------------------------

foreach ($t in $tracked) {
    $row = $manifest | Where-Object { $_.Map -eq $t.Map }
    if (-not $row) { continue }

    $row.Result = if ($t.Proc.ExitCode -eq 0) { 'ok' } else { "exit $($t.Proc.ExitCode)" }

    $csv = Join-Path $outDir "$($t.Map).csv"
    if (-not (Test-Path $csv)) { continue }

    $lines = Get-Content $csv
    $row.Meta = @($lines | Where-Object { $_ -like 'meta,*' }).Count
    $row.Shot = @($lines | Where-Object { $_ -like 'shot,*' }).Count
    $row.Dmg = @($lines | Where-Object { $_ -like 'dmg,*' }).Count
}

$manifestPath = Join-Path $outDir "manifest.csv"
$manifest | Export-Csv -Path $manifestPath -NoTypeInformation

Write-Host ""
Write-Host ("measure_dps_all: {0} map(s) in {1:N0} min" -f $tracked.Count, ($sw.Elapsed.TotalMinutes))
$manifest | Format-Table Map, Areas, Connections, Shot, Dmg, Result -AutoSize | Out-String | Write-Host
Write-Host "manifest: $manifestPath"

$failed = @($manifest | Where-Object { $_.Result -notin @('ok', 'skipped') }).Count
if ($failed -gt 0) {
    Write-Host "measure_dps_all: $failed map(s) did not finish cleanly"
    exit 1
}
exit 0
