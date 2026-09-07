<#
.SYNOPSIS
    Runs test_nav_goal.ps1 for all .bsp maps in the maps/ directory in parallel.

.DESCRIPTION
    Finds all .bsp files in maps/ and runs each one as its own PowerShell child
    process on a unique port, at most -MaxParallel at a time. Each child's full
    transcript is captured to nav_debug/logs/<map>.log and its verdict is taken
    from its real process exit code. Reports a PASS/FAIL summary and rebuilds
    nav_debug/index.html once, serially, at the end.

.PARAMETER MaxParallel
    Maximum number of concurrent HLDS instances (default: 6, hard-limited by the
    86 ports 29015-29100). Higher is faster but not free: at 12 the box is loaded
    enough that bots themselves behave differently -- maps that pass alone and at
    6 have been seen to fall off geometry and fail -- so the verdict gets less
    trustworthy, which is the opposite of what this suite is for.

.PARAMETER Timeout
    Absolute safety timeout per test in seconds (default: 60). This is the real bound.

.PARAMETER TimeoutMultiplier
    Multiplier on bot's reported ETA (default: 6.0)

.PARAMETER Overhead
    Seconds the child is allowed on top of -Timeout before it is killed as wedged (default:
    120). This covers everything outside the goal poll: the navgen run that builds the .nav,
    HLDS boot, and the second navgen that draws the SVG. The big open maps spend over a
    minute in navgen's vantage pass alone, so a budget that fits the small maps kills them
    mid-SVG and reports a failure the navigation never had.

.PARAMETER NavGenPath
    Path to navgen.exe (default: build\tools\navgen\navgen.exe)

.PARAMETER HLDSDir
    HLDS installation directory (default: D:\hlds)

.PARAMETER OutputDir
    Directory for SVG/NAV output (default: nav_debug)

.PARAMETER MapsDir
    Directory containing .bsp test maps (default: maps\)

.PARAMETER Filter
    Only run maps whose filename matches this wildcard (default: *)

.EXAMPLE
    .\scripts\test_nav_goal_all.ps1 -MaxParallel 4

.EXAMPLE
    .\scripts\test_nav_goal_all.ps1 -MaxParallel 2 -Timeout 120 -TimeoutMultiplier 3.0
#>

param(
    [int]$MaxParallel = 6,
    [int]$Timeout = 60,
    [double]$TimeoutMultiplier = 6.0,
    [int]$Overhead = 120,
    [string]$NavGenPath = "build\tools\navgen\navgen.exe",
    [string]$HLDSDir = "D:\hlds",
    [string]$OutputDir = "nav_debug",
    [string]$MapsDir = "maps",
    [string]$Filter = "*",
    # Forwarded to each child's -ServerArgs. -ServerArgs "+sys_ticrate","10" runs the whole
    # suite at a 10 Hz server framerate, which catches follower logic that only works because
    # a 100 Hz frame is too short for the engine to re-simulate anything mid-command.
    [string[]]$ServerArgs = @(),
    # Forwarded to each child's -BootCommands, run once the level is active. This is the only
    # way to set a plugin cvar for a sweep: -ServerArgs "+crossbot_sense","0" is parsed before
    # GameDLLInit registers the cvar, so the default overwrites it and the run silently uses 1.
    [string[]]$BootCommands = @()
)

$ErrorActionPreference = "Stop"
$scriptDir = $PSScriptRoot
$repoRoot = Split-Path $scriptDir -Parent
$testScript = Join-Path $scriptDir "test_nav_goal.ps1"

. "$scriptDir\nav_index.ps1"

$mapFiles = @(Get-ChildItem -Path (Join-Path $repoRoot $MapsDir) -Filter "*.bsp" |
    Where-Object { $_.Name -like $Filter } | Sort-Object Name)
if ($mapFiles.Count -eq 0) {
    Write-Error "No .bsp files matching '$Filter' found in $MapsDir"
    exit 1
}

$portBase = 29015
$maxPorts = 86  # 29015-29100
if ($mapFiles.Count -gt $maxPorts) {
    Write-Error "$($mapFiles.Count) maps exceeds the $maxPorts unique ports available from $portBase"
    exit 1
}
if ($MaxParallel -lt 1) { $MaxParallel = 1 }

Write-Host "=== crossbot parallel nav goal test runner ==="
Write-Host "Found $($mapFiles.Count) maps:"
$mapFiles | ForEach-Object { Write-Host "  $($_.Name)" }
Write-Host "Max parallel: $MaxParallel"
Write-Host "Timeout: ${Timeout}s (${TimeoutMultiplier}x ETA)"
Write-Host ""

$outputDirFull = if (Test-Path $OutputDir) { (Resolve-Path $OutputDir).Path }
elseif (Test-Path (Join-Path $repoRoot $OutputDir)) { (Resolve-Path (Join-Path $repoRoot $OutputDir)).Path }
else { Join-Path $repoRoot $OutputDir }
if (-not (Test-Path $outputDirFull)) { New-Item -ItemType Directory -Path $outputDirFull | Out-Null }
$logDir = Join-Path $outputDirFull "logs"
if (-not (Test-Path $logDir)) { New-Item -ItemType Directory -Path $logDir -Force | Out-Null }

# Run each map in its own PowerShell process rather than a background job. A real
# child process means the verdict is its exit code, its Write-Host transcript is
# capturable, and native stderr goes to a file instead of becoming an ErrorRecord
# that $ErrorActionPreference = 'Stop' would turn into a spurious failure.
$psExe = [System.Diagnostics.Process]::GetCurrentProcess().Path

# Killing a child does not run its cleanup, so remember which ports we own and
# sweep for survivors at the end. Never a blanket kill: the user may be running
# their own server, and the default 27015 is outside this range.
$ourPorts = [System.Collections.Generic.List[int]]::new()

function Stop-ServerOnPort {
    param([int[]]$Ports)
    if ($Ports.Count -eq 0) { return @() }
    $killed = @()
    $procs = @(Get-CimInstance Win32_Process -Filter "Name='hlds.exe'" -ErrorAction SilentlyContinue)
    foreach ($p in $procs) {
        foreach ($port in $Ports) {
            if ($p.CommandLine -and $p.CommandLine -match "\+port\s+$port(\s|$)") {
                Stop-Process -Id $p.ProcessId -Force -ErrorAction SilentlyContinue
                $killed += "$($p.ProcessId) (port $port)"
                break
            }
        }
    }
    return $killed
}

$tracked = [System.Collections.Generic.List[object]]::new()

for ($i = 0; $i -lt $mapFiles.Count; $i++) {
    $mapFile = $mapFiles[$i]
    $mapName = $mapFile.BaseName
    $port = $portBase + $i
    $ourPorts.Add($port)

    $log = Join-Path $logDir "$mapName.log"
    $errLog = Join-Path $logDir "$mapName.err.log"
    $resultFile = Join-Path $outputDirFull "$mapName.result"
    Remove-Item $resultFile, $log, $errLog -ErrorAction SilentlyContinue

    $childArgs = @(
        "-NoProfile", "-NonInteractive", "-File", $testScript,
        "-Map", $mapFile.FullName,
        "-Port", "$port",
        "-Timeout", "$Timeout",
        "-TimeoutMultiplier", "$TimeoutMultiplier",
        "-NavGenPath", $NavGenPath,
        "-HLDSDir", $HLDSDir,
        "-OutputDir", $outputDirFull,
        "-ServerLog", (Join-Path $logDir "${mapName}_server.log"),
        "-NoIndexUpdate"
    )

    if ($ServerArgs.Count -gt 0) {
        $childArgs += "-ServerArgs"
        $childArgs += ($ServerArgs -join ',')
    }

    if ($BootCommands.Count -gt 0) {
        $childArgs += "-BootCommands"
        $childArgs += ($BootCommands -join ',')
    }

    $proc = Start-Process -FilePath $psExe -ArgumentList $childArgs `
        -WorkingDirectory $repoRoot -NoNewWindow -PassThru `
        -RedirectStandardOutput $log -RedirectStandardError $errLog

    $tracked.Add([pscustomobject]@{
        Proc       = $proc
        Map        = $mapName
        Port       = $port
        Log        = $log
        ErrLog     = $errLog
        ResultFile = $resultFile
        StartTime  = Get-Date
        Deadline   = (Get-Date).AddSeconds($Timeout + $Overhead)
        Killed     = $false
    })

    Write-Host "Started $mapName on port $port (PID $($proc.Id))"

    while (@($tracked | Where-Object { -not $_.Proc.HasExited }).Count -ge $MaxParallel) {
        foreach ($t in $tracked) {
            if (-not $t.Proc.HasExited -and (Get-Date) -gt $t.Deadline) {
                Write-Host "  deadline exceeded for $($t.Map), killing"
                Stop-Process -Id $t.Proc.Id -Force -ErrorAction SilentlyContinue
                Stop-ServerOnPort -Ports @($t.Port) | Out-Null
                $t.Killed = $true
            }
        }
        Start-Sleep -Milliseconds 500
    }
}

Write-Host ""
Write-Host "=== Waiting for remaining runs to finish ==="

while (@($tracked | Where-Object { -not $_.Proc.HasExited }).Count -gt 0) {
    foreach ($t in $tracked) {
        if (-not $t.Proc.HasExited -and (Get-Date) -gt $t.Deadline) {
            Write-Host "  deadline exceeded for $($t.Map), killing"
            Stop-Process -Id $t.Proc.Id -Force -ErrorAction SilentlyContinue
            Stop-ServerOnPort -Ports @($t.Port) | Out-Null
            $t.Killed = $true
        }
    }
    Start-Sleep -Milliseconds 500
}

$results = [System.Collections.Generic.List[object]]::new()

foreach ($t in $tracked) {
    $t.Proc.Refresh()
    $exitCode = $t.Proc.ExitCode
    $elapsed = try { ($t.Proc.ExitTime - $t.StartTime).TotalSeconds } catch { ((Get-Date) - $t.StartTime).TotalSeconds }

    $reason = ""
    if (Test-Path $t.ResultFile) {
        $parts = (Get-Content $t.ResultFile -Raw).TrimEnd() -split "`t", 2
        if ($parts.Count -ge 2) { $reason = $parts[1] }
    }
    if (-not $reason -and (Test-Path $t.Log)) {
        $line = Select-String -Path $t.Log -Pattern '=== RESULT: ' | Select-Object -Last 1
        if ($line) { $reason = ($line.Line -replace '^.*=== RESULT: ', '') -replace ' ===$', '' }
    }
    if ($t.Killed) {
        $reason = if ($reason) { "killed at deadline ($([int]($Timeout + $Overhead))s) after: $reason" }
                  else { "killed at deadline ($([int]($Timeout + $Overhead))s)" }
    }
    if (-not $reason) { $reason = "no verdict recorded (exit $exitCode), see $($t.Log)" }

    $results.Add([pscustomobject]@{
        Map      = $t.Map
        Result   = if ($exitCode -eq 0 -and -not $t.Killed) { "PASS" } else { "FAIL" }
        ExitCode = $exitCode
        Elapsed  = $elapsed
        Reason   = $reason
        Log      = $t.Log
    })
}

$orphans = Stop-ServerOnPort -Ports $ourPorts
if ($orphans.Count -gt 0) {
    Write-Host ""
    Write-Host "Cleaned up $($orphans.Count) orphaned server(s): $($orphans -join ', ')"
}

Write-Host ""
Write-Host "=== Rebuilding index.html ==="
foreach ($r in ($results | Sort-Object Map)) {
    Update-IndexHtml -MapName $r.Map -Result $r.Result -OutputDir $outputDirFull
}

Write-Host ""
Write-Host "=== SUMMARY ==="
$passCount = @($results | Where-Object { $_.Result -eq "PASS" }).Count
$failCount = @($results | Where-Object { $_.Result -eq "FAIL" }).Count

foreach ($r in ($results | Sort-Object Map)) {
    $color = if ($r.Result -eq "PASS") { "Green" } else { "Red" }
    Write-Host ("{0,-24} {1,-4} {2,6:N1}s  {3}" -f $r.Map, $r.Result, $r.Elapsed, $r.Reason) -ForegroundColor $color
}

Write-Host ""
Write-Host "Total: $($results.Count)  Passed: $passCount  Failed: $failCount"
Write-Host "Per-map transcripts: $logDir"

if ($failCount -gt 0) {
    exit 1
}
exit 0
