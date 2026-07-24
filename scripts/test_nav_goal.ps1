<#
.SYNOPSIS
    Fully automatic nav goal test: spins up HLDS, runs a bot to a goal,
    collects movement trace, generates SVG with trace overlay, cleans up.

.DESCRIPTION
    Input: a test map (.bsp). The script will:
    1. Start HLDS on port 29015 with the given map
    2. Add a bot via RCON
    3. Auto-select a goal (farthest spawn pair, or info_target)
    4. Send the bot to the goal via crossbot_nav_goal
    5. Poll crossbot_nav_status until goal reached or timeout
       - Timeout = min(2x bot's reported ETA, 60s absolute) [configurable]
    6. Read trace log written by plugin (valve/crossbot_data/nav_trace.txt)
    7. Run navgen --dump-svg --trace to produce nav_debug/<map>.svg with overlay
    8. Also generate .nav file for runtime
    9. Stop HLDS
    Exit code: 0 = PASS (goal reached), 1 = FAIL (timeout/error)

.PARAMETER Map
    Path to the .bsp file (relative to repo root or absolute)

.PARAMETER Timeout
    Absolute safety timeout in seconds (default: 60)

.PARAMETER TimeoutMultiplier
    Multiplier on bot's reported ETA (default: 2.0)

.PARAMETER Port
    HLDS port (default: 29015)

.PARAMETER NavGenPath
    Path to navgen.exe (default: build\tools\navgen\navgen.exe)

.PARAMETER HLDSDir
    HLDS installation directory (default: D:\projects\trash\crossbot\hlds)

.PARAMETER OutputDir
    Directory for SVG/NAV output (default: nav_debug)

.EXAMPLE
    .\scripts\test_nav_goal.ps1 -Map maps\test1_simple.bsp

.EXAMPLE
    .\scripts\test_nav_goal.ps1 -Map maps\test35_smoothwall.bsp -Timeout 120 -TimeoutMultiplier 3.0
#>

param(
    [Parameter(Mandatory=$true)]
    [string]$Map,

    [int]$Timeout = 60,

    [double]$TimeoutMultiplier = 2.0,

    [int]$Port = 29015,

    [string]$NavGenPath = "build\tools\navgen\navgen.exe",

    [string]$HLDSDir = "D:\projects\trash\crossbot\hlds",

    [string]$OutputDir = "nav_debug"
)

$ErrorActionPreference = "Stop"
$scriptDir = $PSScriptRoot
$repoRoot = Split-Path $scriptDir -Parent

# Resolve paths
$mapPath = if (Test-Path $Map) { $Map } elseif (Test-Path (Join-Path $repoRoot $Map)) { Join-Path $repoRoot $Map } elseif (Test-Path (Join-Path $repoRoot "maps\$Map")) { Join-Path $repoRoot "maps\$Map" } elseif (Test-Path (Join-Path $repoRoot "maps\$Map.bsp")) { Join-Path $repoRoot "maps\$Map.bsp" } else { Write-Error "Map not found: $Map"; exit 1 }
$mapName = [System.IO.Path]::GetFileNameWithoutExtension($mapPath)
$navGenExe = if (Test-Path $NavGenPath) { $NavGenPath } elseif (Test-Path (Join-Path $repoRoot $NavGenPath)) { Join-Path $repoRoot $NavGenPath } else { Write-Error "navgen.exe not found at $NavGenPath"; exit 1 }
$hldsExe = Join-Path $HLDSDir "hlds.exe"
if (-not (Test-Path $hldsExe)) { Write-Error "hlds.exe not found at $hldsExe"; exit 1 }
$outputDir = if (Test-Path $OutputDir) { $OutputDir } elseif (Test-Path (Join-Path $repoRoot $OutputDir)) { Join-Path $repoRoot $OutputDir } else { Join-Path $repoRoot $OutputDir }
if (-not (Test-Path $outputDir)) { New-Item -ItemType Directory -Path $outputDir | Out-Null }

Write-Host "=== crossbot automatic nav goal test: $mapName ==="
Write-Host "Map:      $mapPath"
Write-Host "Output:   $outputDir"
Write-Host "Timeout:  ${Timeout}s (${TimeoutMultiplier}x ETA)"
Write-Host "Port:     $Port"
Write-Host "navgen:   $navGenExe"
Write-Host "HLDS:     $hldsExe"

# --- Copy map to HLDS maps directory if needed ---
$hldsMapPath = Join-Path $HLDSDir "valve\maps\$mapName.bsp"
if (-not (Test-Path $hldsMapPath) -or (Get-Item $mapPath).LastWriteTime -gt (Get-Item $hldsMapPath).LastWriteTime) {
    Write-Host "Copying map to HLDS maps directory..."
    Copy-Item $mapPath $hldsMapPath -Force
}

# --- 1. Start HLDS ---
$qconsoleLog = Join-Path $HLDSDir "qconsole.log"
$traceLogPath = Join-Path $HLDSDir "valve\crossbot_data\nav_trace.txt"

Remove-Item $qconsoleLog -ErrorAction SilentlyContinue
Remove-Item $traceLogPath -ErrorAction SilentlyContinue

# Boot commands: add bot, list spawns/targets
$bootCommands = @(
    "crossbot_add TestBot",
    "crossbot_list_spawns",
    "crossbot_list_targets"
)

Write-Host "Starting HLDS..."
$startResult = & "$scriptDir\test_server_start.ps1" -Map $mapName -Port $Port -Commands $bootCommands

if ($startResult -match "^EXITED") {
    Write-Host "FAIL: server failed to start ($startResult)"
    exit 1
}
$serverPid = [int]($startResult -replace "PID=", "")
Write-Host "Server started, PID=$serverPid"

# Give boot cfg time to run (40 waits + commands)
Start-Sleep -Seconds 10

try {
    # --- 2. Parse spawn points and targets from qconsole.log ---
    function Parse-SpawnsAndTargets {
        param([string]$LogPath)
        $log = Get-Content $LogPath -Raw -ErrorAction SilentlyContinue
        if (-not $log) { return @{ Spawns = @(); Targets = @() } }

        $spawnMatches = [regex]::Matches($log, "CrossBot: spawn (\S+) \(([-\d.]+),([-\d.]+),([-\d.]+)\)")
        $targetMatches = [regex]::Matches($log, "CrossBot: target (\S+) \(([-\d.]+),([-\d.]+),([-\d.]+)\)")

        $spawns = @()
        foreach ($m in $spawnMatches) {
            $spawns += [pscustomobject]@{
                Classname = $m.Groups[1].Value
                X = [double]$m.Groups[2].Value
                Y = [double]$m.Groups[3].Value
                Z = [double]$m.Groups[4].Value
            }
        }
        $spawns = $spawns | Sort-Object X,Y,Z -Unique

        $targets = @()
        foreach ($m in $targetMatches) {
            $targets += [pscustomobject]@{
                Name = $m.Groups[1].Value
                X = [double]$m.Groups[2].Value
                Y = [double]$m.Groups[3].Value
                Z = [double]$m.Groups[4].Value
            }
        }

        return @{ Spawns = $spawns; Targets = $targets }
    }

    Write-Host "Parsing spawn points and targets from qconsole.log..."
    $parsed = Parse-SpawnsAndTargets $qconsoleLog
    $spawns = $parsed.Spawns
    $targets = $parsed.Targets

    Write-Host "Found $($spawns.Count) spawn point(s), $($targets.Count) target(s)"

    # --- 3. Select goal ---
    $goal = $null
    if ($targets.Count -ge 1) {
        $goal = $targets[0]
        Write-Host ("Using info_target '{0}' as goal: ({1:F1},{2:F1},{3:F1})" -f $goal.Name, $goal.X, $goal.Y, $goal.Z)
    }
    elseif ($spawns.Count -ge 2) {
        # Find farthest pair
            function Get-Dist3D($a, $b) {
                $dx = $a.X - $b.X; $dy = $a.Y - $b.Y; $dz = $a.Z - $b.Z
                return [math]::Sqrt($dx*$dx + $dy*$dy + $dz*$dz)
            }
        $bestDist = -1
        $bestA = $null
        $bestB = $null
        for ($i=0; $i -lt $spawns.Count; $i++) {
            for ($j=$i+1; $j -lt $spawns.Count; $j++) {
                $d = Get-Dist3D $spawns[$i] $spawns[$j]
                if ($d -gt $bestDist) { $bestDist = $d; $bestA = $spawns[$i]; $bestB = $spawns[$j] }
            }
        }
        $goal = $bestB
        Write-Host ("No targets; using farthest spawn pair as goal: dist={0:F0}" -f $bestDist)
        Write-Host ("  Spawn A ({0:F1},{1:F1},{2:F1}) -> Goal B ({3:F1},{4:F1},{5:F1})" -f $bestA.X,$bestA.Y,$bestA.Z,$goal.X,$goal.Y,$goal.Z)
    }
    else {
        Write-Host "FAIL: Need at least 1 info_target or 2 spawn points to define a goal"
        exit 1
    }

    # --- 4. Teleport bot to start (if using spawn pair) and set goal ---
    if ($targets.Count -eq 0 -and $spawns.Count -ge 2) {
        # Teleport to the first spawn of the pair
        & "$scriptDir\rcon.ps1" -Port $Port -Command "crossbot_teleport $($bestA.X) $($bestA.Y) $($bestA.Z)" | Out-Null
        Start-Sleep -Milliseconds 500
    }

    Write-Host ("Setting nav goal to ({0:F1},{1:F1},{2:F1})..." -f $goal.X, $goal.Y, $goal.Z)
    & "$scriptDir\rcon.ps1" -Port $Port -Command "crossbot_nav_goal $($goal.X) $($goal.Y) $($goal.Z)" | Out-Null

    # --- 5. Poll until arrival or timeout ---
    $reachedPattern = ("CrossBot: \S+ reached goal \(\s*{0}\s*,\s*{1}\s*,\s*{2}\s*\)" -f `
        [regex]::Escape(("{0:F1}" -f $goal.X)), [regex]::Escape(("{0:F1}" -f $goal.Y)), [regex]::Escape(("{0:F1}" -f $goal.Z)))

    $expectedEta = $null
    $elapsed = 0
    $pollInterval = 3
    $result = "FAIL"
    $reason = "unknown"

    # Absolute deadline
    $absoluteDeadline = (Get-Date).AddSeconds($Timeout)

    Write-Host "Polling for goal arrival (interval=${pollInterval}s)..."

    while ($elapsed -lt $Timeout) {
        Start-Sleep -Seconds $pollInterval
        $elapsed += $pollInterval

        # Check qconsole.log for "reached goal" line (source of truth)
        $log = Get-Content $qconsoleLog -Raw -ErrorAction SilentlyContinue
        if ($log -and $log -match $reachedPattern) {
            $result = "PASS"
            $reason = "bot logged arrival at the target, t=${elapsed}s"
            break
        }

        # Poll live status for ETA and progress visibility
        $status = & "$scriptDir\rcon.ps1" -Port $Port -Command "crossbot_nav_status"
        $statusLine = ($status -split "`n") | Where-Object { $_ -match "CrossBot:.*origin=" } | Select-Object -First 1
        if ($statusLine) {
            $etaMatch = [regex]::Match($statusLine, "eta=([\d.]+)s")
            $originMatch = [regex]::Match($statusLine, "origin=\(([-\d.]+),([-\d.]+),([-\d.]+)\)")
            $hasGoalMatch = [regex]::Match($statusLine, "hasGoal=(\d)")

            if ($etaMatch.Success -and -not $expectedEta) {
                $expectedEta = [double]$etaMatch.Groups[1].Value
                $etaDeadline = $expectedEta * $TimeoutMultiplier
                Write-Host ("Bot reports ETA={0:N1}s -> timeout at {1:N1}s (or absolute {2}s)" -f $expectedEta, $etaDeadline, $Timeout)
            }

            if ($originMatch.Success) {
                $ox = [double]$originMatch.Groups[1].Value
                $oy = [double]$originMatch.Groups[2].Value
                $oz = [double]$originMatch.Groups[3].Value
                            $dx = $ox - $goal.X; $dy = $oy - $goal.Y; $dz = $oz - $goal.Z
                            $dist = [math]::Sqrt($dx*$dx + $dy*$dy + $dz*$dz)
                            $hasGoal = if ($hasGoalMatch.Success) { [int]$hasGoalMatch.Groups[1].Value } else { 0 }
                            Write-Host ("t={0,4}s origin=({1:F0},{2:F0},{3:F0}) dist={4:F0} hasGoal={5}" -f $elapsed, $ox, $oy, $oz, $dist, $hasGoal)
                        }
        }

        # Check timeout conditions
        if ($expectedEta -and $elapsed -gt ($expectedEta * $TimeoutMultiplier)) {
            $reason = ("exceeded {0:N1}s ({1}x the {2:N1}s ETA) without arriving" -f ($expectedEta * $TimeoutMultiplier), $TimeoutMultiplier, $expectedEta)
            break
        }

        if ((Get-Date) -ge $absoluteDeadline) {
            $reason = "hit absolute ${Timeout}s safety cap"
            break
        }
    }

    if ($result -ne "PASS" -and $elapsed -ge $Timeout) {
        $reason = "hit absolute ${Timeout}s safety cap without arriving"
    }

    Write-Host "=== RESULT: $result -- $reason ==="

    # --- 6. Collect trace log ---
    Write-Host "Collecting trace log..."
    Start-Sleep -Seconds 1  # Give plugin time to flush

    $tracePoints = @()
    if (Test-Path $traceLogPath) {
        $traceContent = Get-Content $traceLogPath -Raw
        if ($traceContent) {
            foreach ($line in $traceContent -split "`n") {
                $line = $line.Trim()
                if (-not $line) { continue }
                $parts = $line -split '\s+'
                if ($parts.Count -ge 3) {
                    $tracePoints += [pscustomobject]@{
                        X = [double]$parts[0]
                        Y = [double]$parts[1]
                        Z = [double]$parts[2]
                    }
                }
            }
            Write-Host "Read $($tracePoints.Count) trace points from nav_trace.txt"
        }
    } else {
        Write-Warning "Trace log not found at $traceLogPath (bot may not have reached goal or trace writing failed)"
    }

    # --- 7. Generate SVG with trace overlay ---
    Write-Host "Generating debug SVG..."
    $svgOut = Join-Path $outputDir "$mapName.svg"
    $navOut = Join-Path $outputDir "$mapName.nav"
    $traceOut = Join-Path $outputDir "nav_trace.txt"

    # Write trace to output dir for navgen
    if ($tracePoints.Count -gt 0) {
            $tracePoints | ForEach-Object { "{0:F1} {1:F1} {2:F1}" -f $_.X, $_.Y, $_.Z } | Set-Content -Path $traceOut
    } else {
        # Create empty trace file so navgen doesn't error
            "" | Set-Content -Path $traceOut
    }

    $navgenArgs = @($mapPath, "--dump-svg", $svgOut, "--trace", $traceOut, "-o", $navOut)
    Write-Host "Running: navgen.exe $($navgenArgs -join ' ')"
    $navgenResult = & $navGenExe @navgenArgs
    Write-Host $navgenResult

    if (-not (Test-Path $svgOut)) {
        Write-Warning "SVG generation may have failed: $svgOut not found"
    } else {
        Write-Host "SVG written: $svgOut"
    }
    if (Test-Path $navOut) { Write-Host "NAV written: $navOut" }

}
finally {
    # --- 8. Cleanup ---
    Write-Host "Stopping HLDS (PID=$serverPid)..."
    & "$scriptDir\test_server_stop.ps1" -Id $serverPid | Out-Null
    # Also kill any child processes just in case
    Get-CimInstance Win32_Process -Filter "ParentProcessId=$serverPid" | ForEach-Object { Stop-Process -Id $_.ProcessId -Force -ErrorAction SilentlyContinue }
}

if ($result -ne "PASS") { exit 1 }
exit 0