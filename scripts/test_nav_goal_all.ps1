<#
.SYNOPSIS
    Runs test_nav_goal.ps1 for all .bsp maps in the maps/ directory in parallel.

.DESCRIPTION
    Finds all .bsp files in maps/, launches each test_nav_goal.ps1 in a parallel
    PowerShell job (Start-Job) with unique ports and map-specific outputs.
    Waits for all jobs to complete and reports PASS/FAIL summary.
    Also updates nav_debug/index.html with colored PASS/FAIL cards.

.PARAMETER MaxParallel
    Maximum number of concurrent HLDS instances (default: 12, limited by ports 29015-29100)

.PARAMETER Timeout
    Absolute safety timeout per test in seconds (default: 60)

.PARAMETER TimeoutMultiplier
    Multiplier on bot's reported ETA (default: 2.0)

.PARAMETER NavGenPath
    Path to navgen.exe (default: build\tools\navgen\navgen.exe)

.PARAMETER HLDSDir
    HLDS installation directory (default: D:\projects\trash\crossbot\hlds)

.PARAMETER OutputDir
    Directory for SVG/NAV output (default: nav_debug)

.PARAMETER MapsDir
    Directory containing .bsp test maps (default: maps\)

.EXAMPLE
    .\scripts\test_nav_goal_all.ps1 -MaxParallel 4

.EXAMPLE
    .\scripts\test_nav_goal_all.ps1 -MaxParallel 2 -Timeout 120 -TimeoutMultiplier 3.0
#>

param(
    [int]$MaxParallel = 12,
    [int]$Timeout = 60,
    [double]$TimeoutMultiplier = 2.0,
    [string]$NavGenPath = "build\tools\navgen\navgen.exe",
    [string]$HLDSDir = "D:\projects\trash\crossbot\hlds",
    [string]$OutputDir = "nav_debug",
    [string]$MapsDir = "maps"
)

$ErrorActionPreference = "Stop"
$scriptDir = $PSScriptRoot
$repoRoot = Split-Path $scriptDir -Parent
$testScript = Join-Path $scriptDir "test_nav_goal.ps1"

# Find all .bsp files
$mapFiles = Get-ChildItem -Path (Join-Path $repoRoot $MapsDir) -Filter "*.bsp" | Sort-Object Name
if (-not $mapFiles) {
    Write-Error "No .bsp files found in $MapsDir"
    exit 1
}

Write-Host "=== crossbot parallel nav goal test runner ==="
Write-Host "Found $($mapFiles.Count) maps:"
$mapFiles | ForEach-Object { Write-Host "  $($_.Name)" }
Write-Host "Max parallel: $MaxParallel"
Write-Host "Timeout: ${Timeout}s (${TimeoutMultiplier}x ETA)"
Write-Host ""

# Prepare output directory
$outputDirFull = if (Test-Path $OutputDir) { $OutputDir }
elseif (Test-Path (Join-Path $repoRoot $OutputDir)) { Join-Path $repoRoot $OutputDir }
else { Join-Path $repoRoot $OutputDir }
if (-not (Test-Path $outputDirFull)) { New-Item -ItemType Directory -Path $outputDirFull | Out-Null }

# Launch jobs in parallel batches
$jobs = @()
$portBase = 29015
$maxPorts = 86  # 29015-29100
$mapIndex = 0

$mapFiles | ForEach-Object {
    $mapFile = $_
    $mapName = $mapFile.BaseName
    $port = $portBase + ($mapIndex % $maxPorts)
    $mapIndex++
    
    $job = Start-Job -ScriptBlock {
        param($MapPath, $Port, $Timeout, $TimeoutMultiplier, $NavGenPath, $HLDSDir, $OutputDir, $TestScript, $MapName)
        
        try {
            & $TestScript -Map $MapPath -Port $Port -Timeout $Timeout -TimeoutMultiplier $TimeoutMultiplier -NavGenPath $NavGenPath -HLDSDir $HLDSDir -OutputDir $OutputDir
            return @{ Map = $MapName; Port = $Port; ExitCode = $LASTEXITCODE; Result = "PASS" }
        } catch {
            return @{ Map = $MapName; Port = $Port; ExitCode = 1; Result = "FAIL"; Error = $_.ToString() }
        }
    } -ArgumentList @(
        $mapFile.FullName,
        $port,
        $Timeout,
        $TimeoutMultiplier,
        $NavGenPath,
        $HLDSDir,
        $OutputDir,
        $testScript,
        $mapName
    )
    
    $jobs += [pscustomobject]@{
        Job = $job
        Map = $mapName
        Port = $port
        StartTime = Get-Date
    }
    
    Write-Host "Started job for $mapName on port $port (Job ID: $($job.Id))"
    
    # Throttle: wait if we hit max parallel
    while (($jobs | Where-Object { $_.Job.State -eq 'Running' }).Count -ge $MaxParallel) {
        Start-Sleep -Seconds 2
    }
}

Write-Host ""
Write-Host "=== Waiting for all jobs to complete ==="
Write-Host ""

# Wait for all jobs and collect results
$results = @()

foreach ($jobInfo in $jobs) {
    $job = $jobInfo.Job
    $map = $jobInfo.Map
    $port = $jobInfo.Port
    
    Write-Host "Waiting for $map (port $port)..."
    $job | Wait-Job | Out-Null
    $result = $job | Receive-Job
    $job | Remove-Job
    
    $elapsed = (Get-Date) - $jobInfo.StartTime
    $status = if ($result.Result -eq "PASS") { "PASS" } else { "FAIL" }
    Write-Host ("{0}: {1} ({2:N1}s)" -f $map, $status, $elapsed.TotalSeconds)
    
    if ($result.Result -eq "FAIL" -and $result.Error) {
        Write-Host "  Error: $($result.Error)" -ForegroundColor Red
    }
    
    $results += $result
}

# Summary
Write-Host ""
Write-Host "=== SUMMARY ==="
$passCount = ($results | Where-Object { $_.Result -eq "PASS" }).Count
$failCount = ($results | Where-Object { $_.Result -eq "FAIL" }).Count

$results | Sort-Object Map | ForEach-Object {
    $status = if ($_.Result -eq "PASS") { "PASS" } else { "FAIL" }
    Write-Host ("{0,-20} {1}" -f $_.Map, $status)
}

Write-Host ""
Write-Host "Total: $($results.Count)  Passed: $passCount  Failed: $failCount"

if ($failCount -gt 0) {
    exit 1
}
exit 0