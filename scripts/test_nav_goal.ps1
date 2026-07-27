<#
.SYNOPSIS
    Fully automatic nav goal test: spins up HLDS, runs a bot to a goal,
    collects movement trace, generates SVG with trace overlay, cleans up.

.DESCRIPTION
    Input: a test map (.bsp). The script will:
    1. Start HLDS on a unique port with the given map
    2. Set crossbot_trace_path cvar to a map-specific trace file
    3. Add a bot via RCON
    4. Auto-select a goal (info_target preferred, else farthest spawn pair)
    5. Send the bot to the goal via crossbot_nav_goal
    6. Poll crossbot_nav_status until goal reached (hasGoal=0) or timeout
       - Timeout = min(2x bot's reported ETA, 60s absolute) [configurable]
    7. Read trace log written by plugin (valve/crossbot_data/nav_trace_<map>.txt)
    8. Run navgen --dump-svg --trace to produce nav_debug/<map>.svg with overlay
    9. Also generate .nav file for runtime
    10. Stop HLDS
    Exit code: 0 = PASS (goal reached), 1 = FAIL (timeout/error)

.PARAMETER Map
    Path to the .bsp file (relative to repo root or absolute)

.PARAMETER Timeout
    Absolute safety timeout in seconds (default: 60)

.PARAMETER TimeoutMultiplier
    Multiplier on bot's reported ETA (default: 2.0)

.PARAMETER Port
    HLDS port (default: auto-assign from 29015-29100)

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

    [int]$Port = 0,  # 0 = auto-assign from 29015-29100

    [string]$NavGenPath = "build\tools\navgen\navgen.exe",

    [string]$HLDSDir = "D:\projects\trash\crossbot\hlds",

    [string]$OutputDir = "nav_debug"
)

$ErrorActionPreference = "Stop"
$scriptDir = $PSScriptRoot

function Update-IndexHtml {
    param(
        [string]$MapName,
        [string]$Result,
        [string]$OutputDir
    )

    $indexHtml = Join-Path $OutputDir "index.html"
    if (-not (Test-Path $indexHtml)) {
        Write-Host "Index.html not found at $indexHtml, skipping update"
        return
    }

    $html = Get-Content $indexHtml -Raw
    $color = if ($Result -eq "PASS") { "#1a5c1a" } else { "#5c1a1a" }
    $borderColor = if ($Result -eq "PASS") { "#2e8b2e" } else { "#8b2e2e" }
    $label = if ($Result -eq "PASS") { "✓ PASS" } else { "✗ FAIL" }

    # Match existing card format (single-line): <div class="card"><h2><a href="mapname.svg" target="_blank">mapname</a></h2>...
    $pattern = '<div class="card"><h2><a href="' + [regex]::Escape($MapName) + '\.svg" target="_blank">' + [regex]::Escape($MapName) + '</a></h2>'

    $newCard = '<div class="card" style="background: {0}; border-color: {1};"><h2><a href="{2}.svg" target="_blank">{2} <span style="float:right; color:{3};">{4}</span></a></h2><a href="{2}.svg" target="_blank"><img src="{2}.svg"></a></div>' -f $color, $borderColor, $MapName, $borderColor, $label

    if ($html -match $pattern) {
        # Replace existing card - use single-line regex mode for cross-line matching
        $regex = [regex]::new($pattern + '.*?</div>', 'Singleline')
        $html = $regex.Replace($html, $newCard)
    } else {
        # Add new card after the grid opening
        $html = $html -replace '(<div class="grid">)', "`$1`n$newCard"
    }

    $html | Set-Content -Path $indexHtml -Encoding UTF8
    Write-Host "Updated index.html: $MapName -> $label"
}
$repoRoot = Split-Path $scriptDir -Parent

# Resolve paths
$mapPath = if (Test-Path $Map) { $Map }
elseif (Test-Path (Join-Path $repoRoot $Map)) { Join-Path $repoRoot $Map }
elseif (Test-Path (Join-Path $repoRoot "maps\$Map")) { Join-Path $repoRoot "maps\$Map" }
elseif (Test-Path (Join-Path $repoRoot "maps\$Map.bsp")) { Join-Path $repoRoot "maps\$Map.bsp" }
else { Write-Error "Map not found: $Map"; exit 1 }

$mapName = [System.IO.Path]::GetFileNameWithoutExtension($mapPath)
$navGenExe = if (Test-Path $NavGenPath) { $NavGenPath }
elseif (Test-Path (Join-Path $repoRoot $NavGenPath)) { Join-Path $repoRoot $NavGenPath }
else { Write-Error "navgen.exe not found at $NavGenPath"; exit 1 }

$hldsExe = Join-Path $HLDSDir "hlds.exe"
if (-not (Test-Path $hldsExe)) { Write-Error "hlds.exe not found at $hldsExe"; exit 1 }

$outputDir = if (Test-Path $OutputDir) { $OutputDir }
elseif (Test-Path (Join-Path $repoRoot $OutputDir)) { Join-Path $repoRoot $OutputDir }
else { Join-Path $repoRoot $OutputDir }
if (-not (Test-Path $outputDir)) { New-Item -ItemType Directory -Path $outputDir | Out-Null }

# Auto-assign port if not provided
if ($Port -eq 0) {
    $Port = 29015 + (Get-Random -Maximum 85)  # 29015-29100
}

# Map-specific filenames for parallel isolation
$configName = "_crossbot_autotest_$mapName.cfg"
$traceCvarPath = "valve/crossbot_data/nav_trace_$mapName.txt"
$traceLogPath = Join-Path $HLDSDir $traceCvarPath
$svgOut = Join-Path $outputDir "$mapName.svg"
$navOut = Join-Path $outputDir "$mapName.nav"
$traceOut = Join-Path $outputDir "nav_trace_$mapName.txt"

Write-Host "=== crossbot automatic nav goal test: $mapName ==="
Write-Host "Map:      $mapPath"
Write-Host "Output:   $outputDir"
Write-Host "Timeout:  ${Timeout}s (${TimeoutMultiplier}x ETA)"
Write-Host "Port:     $Port"
Write-Host "Trace:    $traceCvarPath"
Write-Host "navgen:   $navGenExe"
Write-Host "HLDS:     $hldsExe"

# --- Copy map to HLDS maps directory if needed ---
$hldsMapPath = Join-Path $HLDSDir "valve\maps\$mapName.bsp"
if (-not (Test-Path $hldsMapPath) -or (Get-Item $mapPath).LastWriteTime -gt (Get-Item $hldsMapPath).LastWriteTime) {
    Write-Host "Copying map to HLDS maps directory..."
    Copy-Item $mapPath $hldsMapPath -Force
}

# --- 0. Run navgen FIRST to generate fresh .nav file ---
$botNavDir = Join-Path $HLDSDir "valve\crossbot_data\nav"
if (-not (Test-Path $botNavDir)) { New-Item -ItemType Directory -Path $botNavDir | Out-Null }
$botNavPath = Join-Path $botNavDir "$mapName.nav"

Write-Host "Running navgen to generate fresh nav file..."
$navgenResult = & $navGenExe $mapPath -o $botNavPath 2>&1
if ($LASTEXITCODE -ne 0) {
    Write-Host "WARNING: navgen failed: $navgenResult"
} else {
    Write-Host "navgen succeeded, nav file: $botNavPath"
}

# --- 1. Start HLDS ---
$qconsoleLog = Join-Path $HLDSDir "qconsole.log"
Remove-Item $traceLogPath -ErrorAction SilentlyContinue

# Boot commands: set trace path, add bot, list spawns/targets
$bootCommands = @(
    "crossbot_trace_path $traceCvarPath",
    "crossbot_add TestBot",
    "crossbot_list_spawns",
    "crossbot_list_targets"
)

Write-Host "Starting HLDS on port $Port..."
$startResult = & "$scriptDir\test_server_start.ps1" -Map $mapName -Port $Port -Commands $bootCommands -ConfigName $configName

if ($startResult -match "^EXITED") {
    Write-Host "FAIL: server failed to start ($startResult)"
    exit 1
}
$serverPid = [int]($startResult -replace "PID=", "")
Write-Host "Server started, PID=$serverPid"

# Give boot cfg time to run (40 waits + commands)
Start-Sleep -Seconds 10

try {
    # --- 2. Get spawn points and targets via RCON (not qconsole.log) ---
    Write-Host "Querying spawn points and targets via RCON..."

    function Parse-SpawnsAndTargetsFromRcon($port) {
            $spawnOut = & "$scriptDir\rcon.ps1" -Port $port -Command "crossbot_list_spawns"
            $targetOut = & "$scriptDir\rcon.ps1" -Port $port -Command "crossbot_list_targets"

            Write-Host "Raw spawn output: '$spawnOut'"
            Write-Host "Raw target output: '$targetOut'"

            $spawnMatches = [regex]::Matches($spawnOut, "CrossBot: spawn (\S+) \(([-\d.]+),([-\d.]+),([-\d.]+)\)")
            $targetMatches = [regex]::Matches($targetOut, "CrossBot: target (\S+) \(([-\d.]+),([-\d.]+),([-\d.]+)\)")

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

        $parsed = Parse-SpawnsAndTargetsFromRcon $Port
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

    # --- 4. Teleport bot to start (use first spawn point) and set goal ---
        $startSpawn = $spawns[0]
        Write-Host "Teleporting bot to first spawn: ($($startSpawn.X), $($startSpawn.Y), $($startSpawn.Z))"
        & "$scriptDir\rcon.ps1" -Port $Port -Command "crossbot_teleport $($startSpawn.X) $($startSpawn.Y) $($startSpawn.Z)"
        Start-Sleep -Milliseconds 500

        # Verify teleport worked
        $status = & "$scriptDir\rcon.ps1" -Port $Port -Command "crossbot_nav_status"
        Write-Host "After teleport status: $status"

    Write-Host ("Setting nav goal to ({0:F1},{1:F1},{2:F1})..." -f $goal.X, $goal.Y, $goal.Z)
    & "$scriptDir\rcon.ps1" -Port $Port -Command "crossbot_nav_goal $($goal.X) $($goal.Y) $($goal.Z)" | Out-Null

        # Wait for Repath to run and waypoints to be generated (poll until we have waypoints)
        Write-Host "Waiting for path planning..."
        $waypointsReady = $false
        for ($i = 0; $i -lt 20; $i++) {  # up to 2 seconds
            Start-Sleep -Milliseconds 100
            $status = & "$scriptDir\rcon.ps1" -Port $Port -Command "crossbot_nav_status"
            if ($status -match "waypoint=\d+/(\d+)") {
                $wpCount = [int]$matches[1]
                if ($wpCount -gt 0) {
                    $waypointsReady = $true
                    break
                }
            }
        }

        # Dump waypoints for debugging
        $wpDump = & "$scriptDir\rcon.ps1" -Port $Port -Command "crossbot_waypoints"
        Write-Host "Waypoints dump:"
        Write-Host $wpDump

        # --- 5. Poll until arrival or timeout ---
    # Use hasGoal=0 from crossbot_nav_status as primary signal (more reliable than qconsole.log for short goals)
        # Also collect bot positions during polling to build trajectory trace if test fails/times out
        $expectedEta = $null
        $elapsed = 0
        $pollInterval = 2  # shorter interval to catch quick completions
        $result = "FAIL"
        $reason = "unknown"
        $pollTracePoints = @()  # collect positions during polling for fallback trace

        # Absolute deadline
        $absoluteDeadline = (Get-Date).AddSeconds($Timeout)

        Write-Host "Polling for goal arrival (interval=${pollInterval}s)..."

        while ($elapsed -lt $Timeout) {
            Start-Sleep -Seconds $pollInterval
            $elapsed += $pollInterval

            # Poll live status for hasGoal, ETA, and progress
            $status = & "$scriptDir\rcon.ps1" -Port $Port -Command "crossbot_nav_status"
                        Write-Host "Full status: $status"
                        $statusLine = ($status -split "`n") | Where-Object { $_ -match "CrossBot:.*origin=" } | Select-Object -First 1

            if ($statusLine) {
                            # Print full status line if it contains waypoints
                            if ($statusLine -match "waypoints=") {
                                Write-Host "Full status: $statusLine"
                            }
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

                    # Record position for fallback trace
                    $pollTracePoints += [pscustomobject]@{ X = $ox; Y = $oy; Z = $oz }
                }

                # Goal reached when hasGoal becomes 0
                if ($hasGoalMatch.Success -and [int]$hasGoalMatch.Groups[1].Value -eq 0) {
                    $result = "PASS"
                    $reason = "bot reported hasGoal=0 (goal reached), t=${elapsed}s"
                    break
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
    Write-Host "Collecting trace log from $traceLogPath..."
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
            Write-Host "Read $($tracePoints.Count) trace points from $traceLogPath"
        }
    } else {
        Write-Warning "Trace log not found at $traceLogPath (bot may not have reached goal or trace writing failed)"
    }

        # Fallback: use positions collected during polling if trace log is empty
        if ($tracePoints.Count -eq 0 -and $pollTracePoints.Count -gt 0) {
            Write-Host "Using $($pollTracePoints.Count) fallback trace points from polling"
            $tracePoints = $pollTracePoints
        }

    # --- 7. Generate SVG with trace overlay ---
    Write-Host "Generating debug SVG..."
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

        # Update index.html with test result
        Update-IndexHtml -MapName $mapName -Result $result -OutputDir $outputDir

    }
        finally {
                    # --- Cleanup ---
                    Write-Host "Stopping HLDS (PID=$serverPid)..."
                    & "$scriptDir\test_server_stop.ps1" -Id $serverPid | Out-Null
                    # Also kill any child processes just in case
                    Get-CimInstance Win32_Process -Filter "ParentProcessId=$serverPid" | ForEach-Object { Stop-Process -Id $_.ProcessId -Force -ErrorAction SilentlyContinue }
                }

                if ($result -ne "PASS") { exit 1 }
                exit 0