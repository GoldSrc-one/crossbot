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
    7. Read trace log written by plugin (valve/addons/crossbot/trace/nav_trace_<map>.txt)
    8. Run navgen --dump-svg --trace to produce nav_debug/<map>.svg with overlay
    9. Also generate .nav file for runtime
    10. Stop HLDS
    Exit code: 0 = PASS (goal reached), 1 = FAIL (timeout/error)

.PARAMETER Map
    Path to the .bsp file (relative to repo root or absolute)

.PARAMETER Timeout
    Absolute safety timeout in seconds (default: 60). Measured on the wall clock.

.PARAMETER TimeoutMultiplier
    Multiplier on bot's reported ETA (default: 6.0). This is a fail-fast heuristic,
    not the real bound -- -Timeout is. Ladder maps take 2.6-3.1x their ETA even
    run alone, because the planner's estimate does not model falling off and
    reclimbing, and more than that under parallel load. A genuinely broken map
    never arrives at all, so it still fails fast on the absolute cap.

.PARAMETER Port
    HLDS port (default: auto-assign from 29015-29100)

.PARAMETER NavGenPath
    Path to navgen.exe (default: build\tools\navgen\navgen.exe)

.PARAMETER HLDSDir
    HLDS installation directory (default: D:\projects\trash\crossbot\hlds)

.PARAMETER OutputDir
    Directory for SVG/NAV output (default: nav_debug)

.PARAMETER ServerLog
    Where to capture this instance's server output (default:
    <OutputDir>\logs\<map>_server.log). Per-instance, unlike qconsole.log.

.PARAMETER NoIndexUpdate
    Skip the nav_debug/index.html rewrite. Parallel runners set this and rebuild
    the index once, serially, at the end -- concurrent read-modify-write on a
    single html file loses results.

.EXAMPLE
    .\scripts\test_nav_goal.ps1 -Map maps\test1_simple.bsp

.EXAMPLE
    .\scripts\test_nav_goal.ps1 -Map maps\test35_smoothwall.bsp -Timeout 120 -TimeoutMultiplier 3.0
#>

param(
    [Parameter(Mandatory=$true)]
    [string]$Map,

    [int]$Timeout = 60,

    [double]$TimeoutMultiplier = 6.0,

    [int]$Port = 0,  # 0 = auto-assign from 29015-29100

    [string]$NavGenPath = "build\tools\navgen\navgen.exe",

    [string]$HLDSDir = "D:\projects\trash\crossbot\hlds",

    [string]$OutputDir = "nav_debug",

    [string]$ServerLog = "",

    [switch]$NoIndexUpdate,

    # Extra engine launch arguments, forwarded to test_server_start.ps1 and applied
    # before the map loads. -ServerArgs "+sys_ticrate","10" runs the whole test at a
    # 10 Hz server framerate, which is the cheapest way to check that the follower
    # is frame-rate independent rather than accidentally tuned to 100 Hz frames.
    [string[]]$ServerArgs = @(),

    # Extra console commands run after the boot commands, once the level is active.
    # Plugin cvars must be set this way and not through -ServerArgs: they are registered
    # in GameDLLInit, after the engine has parsed the command line, so a "+crossbot_sense 0"
    # is overwritten by the cvar's own default and silently does nothing.
    [string[]]$BootCommands = @(),

    # How close the bot must end up for hasGoal=0 to count as arrival rather than giving up.
    # Legitimate arrivals land 24-50 units out (the follower stops at finalRadius, and the goal is
    # clamped into its area, so it never coincides with the raw entity origin).
    [double]$GoalRadius = 96
)

$ErrorActionPreference = "Stop"
$scriptDir = $PSScriptRoot
$repoRoot = Split-Path $scriptDir -Parent

. "$scriptDir\nav_index.ps1"

# Native tools report warnings on stderr. Under $ErrorActionPreference = 'Stop'
# a redirected stderr line becomes a terminating error, which failed maps for
# reasons that had nothing to do with the bot. Warnings must never fail a run.
function Invoke-Native {
    param([string]$Exe, [string[]]$Arguments)
    $ErrorActionPreference = 'Continue'
    $output = & $Exe @Arguments 2>&1 | Out-String
    return [pscustomobject]@{ Output = $output; ExitCode = $LASTEXITCODE }
}

function Invoke-Rcon {
    param([string]$Cmd)
    $out = try { & "$scriptDir\rcon.ps1" -Port $Port -Command $Cmd } catch { "" }
    if ($null -eq $out) { return "" }
    return ($out | Out-String)
}

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
$traceCvarPath = "valve/addons/crossbot/trace/nav_trace_$mapName.txt"
$traceLogPath = Join-Path $HLDSDir $traceCvarPath
$svgOut = Join-Path $outputDir "$mapName.svg"
$navOut = Join-Path $outputDir "$mapName.nav"
$traceOut = Join-Path $outputDir "nav_trace_$mapName.txt"
$resultOut = Join-Path $outputDir "$mapName.result"

if (-not $ServerLog) {
    $ServerLog = Join-Path $outputDir "logs\${mapName}_server.log"
}
$serverLogDir = Split-Path $ServerLog -Parent
if ($serverLogDir -and -not (Test-Path $serverLogDir)) { New-Item -ItemType Directory -Path $serverLogDir -Force | Out-Null }

Write-Host "=== crossbot automatic nav goal test: $mapName ==="
Write-Host "Map:      $mapPath"
Write-Host "Output:   $outputDir"
Write-Host "Timeout:  ${Timeout}s (${TimeoutMultiplier}x ETA)"
Write-Host "Port:     $Port"
Write-Host "Trace:    $traceCvarPath"
Write-Host "navgen:   $navGenExe"
Write-Host "HLDS:     $hldsExe"
Write-Host "SrvLog:   $ServerLog"
if ($ServerArgs.Count -gt 0) { Write-Host "SrvArgs:  $($ServerArgs -join ' ')" }

$result = "FAIL"
$reason = "did not run"

# --- Copy map to HLDS maps directory if needed ---
$hldsMapPath = Join-Path $HLDSDir "valve\maps\$mapName.bsp"
if (-not (Test-Path $hldsMapPath) -or (Get-Item $mapPath).LastWriteTime -gt (Get-Item $hldsMapPath).LastWriteTime) {
    Write-Host "Copying map to HLDS maps directory..."
    Copy-Item $mapPath $hldsMapPath -Force
}

# --- 0. Run navgen FIRST to generate fresh .nav file ---
$botNavDir = Join-Path $HLDSDir "valve\addons\crossbot\nav"
if (-not (Test-Path $botNavDir)) { New-Item -ItemType Directory -Path $botNavDir | Out-Null }
$botNavPath = Join-Path $botNavDir "$mapName.nav"

$botTraceDir = Join-Path $HLDSDir "valve\addons\crossbot\trace"
if (-not (Test-Path $botTraceDir)) { New-Item -ItemType Directory -Path $botTraceDir | Out-Null }

Write-Host "Running navgen to generate fresh nav file..."
$navgenRun = Invoke-Native -Exe $navGenExe -Arguments @($mapPath, "-o", $botNavPath)
if ($navgenRun.ExitCode -ne 0) {
    Write-Host "WARNING: navgen failed (exit $($navgenRun.ExitCode)): $($navgenRun.Output)"
} else {
    Write-Host "navgen succeeded, nav file: $botNavPath"
}

# --- 1. Start HLDS ---
Remove-Item $traceLogPath -ErrorAction SilentlyContinue

# Boot commands: set trace path, add bot, list spawns/targets
$bootCommands = @(
    "crossbot_trace_path $traceCvarPath",
    "crossbot_gather 0",
    "crossbot_add TestBot",
    "crossbot_list_spawns",
    "crossbot_list_targets"
) + $BootCommands

Write-Host "Starting HLDS on port $Port..."
$startResult = & "$scriptDir\test_server_start.ps1" -Map $mapName -Port $Port -Commands $bootCommands `
    -ConfigName $configName -HLDSDir $HLDSDir -LogFile $ServerLog -ExtraArgs $ServerArgs

if ($startResult -match "^EXITED") {
    Write-Host "FAIL: server failed to start ($startResult)"
    Write-Host "=== RESULT: FAIL -- server failed to start ($startResult) ==="
    "FAIL`tserver failed to start ($startResult)" | Set-Content -Path $resultOut
    exit 1
}
$serverPid = [int]($startResult -replace "PID=", "")
Write-Host "Server started, PID=$serverPid"

function Parse-SpawnsAndTargetsFromRcon {
    $spawnOut = Invoke-Rcon "crossbot_list_spawns"
    $targetOut = Invoke-Rcon "crossbot_list_targets"

    $spawnMatches = [regex]::Matches([string]$spawnOut, "CrossBot: spawn (\S+) \(([-\d.]+),([-\d.]+),([-\d.]+)\)")
    $targetMatches = [regex]::Matches([string]$targetOut, "CrossBot: target (\S+) \(([-\d.]+),([-\d.]+),([-\d.]+)\)")

    $spawns = @()
    foreach ($m in $spawnMatches) {
        $spawns += [pscustomobject]@{
            Classname = $m.Groups[1].Value
            X = [double]$m.Groups[2].Value
            Y = [double]$m.Groups[3].Value
            Z = [double]$m.Groups[4].Value
        }
    }
    $spawns = @($spawns | Sort-Object X,Y,Z -Unique)

    $targets = @()
    foreach ($m in $targetMatches) {
        $targets += [pscustomobject]@{
            Name = $m.Groups[1].Value
            X = [double]$m.Groups[2].Value
            Y = [double]$m.Groups[3].Value
            Z = [double]$m.Groups[4].Value
        }
    }

    return @{ Spawns = $spawns; Targets = @($targets) }
}

try {
    # --- 2. Wait for the boot cfg to have run, then read spawns/targets over RCON ---
    # The cfg is 40 engine "wait"s then the commands; poll for its observable effect
    # rather than sleeping a fixed amount, which under parallel load was both too
    # long on small maps and too short on big ones (0 spawns parsed -> bogus FAIL).
    # An RCON reply can arrive split across UDP packets, and a loaded box can drop
    # or delay one. A short read looks exactly like a map with fewer spawns or no
    # info_target at all, which silently picks a different goal. So require the
    # same counts twice in a row before trusting them.
    Write-Host "Waiting for boot commands and querying spawns/targets via RCON..."
    $bootSw = [System.Diagnostics.Stopwatch]::StartNew()
    $spawns = @()
    $targets = @()
    $botReady = $false
    $prevKey = ""
    while ($bootSw.Elapsed.TotalSeconds -lt 30) {
        $parsed = Parse-SpawnsAndTargetsFromRcon
        $spawns = $parsed.Spawns
        $targets = $parsed.Targets
        $status = Invoke-Rcon "crossbot_nav_status"
        $botReady = ($status -match "origin=")
        $key = "$($spawns.Count)/$($targets.Count)"
        if ($spawns.Count -ge 1 -and $botReady -and $key -eq $prevKey) { break }
        $prevKey = $key
        Start-Sleep -Milliseconds 500
    }
    Write-Host ("Boot ready after {0:N1}s: {1} spawn(s), {2} target(s), bot={3}" -f `
        $bootSw.Elapsed.TotalSeconds, $spawns.Count, $targets.Count, $botReady)

    if (-not $botReady) {
        Write-Host "WARNING: bot never reported an origin; continuing anyway"
    }

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
        $reason = "need at least 1 info_target or 2 spawn points to define a goal (found $($spawns.Count) spawns, $($targets.Count) targets)"
        Write-Host "FAIL: $reason"
        Write-Host "=== RESULT: FAIL -- $reason ==="
        "FAIL`t$reason" | Set-Content -Path $resultOut
        exit 1
    }

    # --- 4. Teleport bot to start (use first spawn point) and set goal ---
    $startSpawn = $spawns[0]
    Write-Host "Teleporting bot to first spawn: ($($startSpawn.X), $($startSpawn.Y), $($startSpawn.Z))"
    Invoke-Rcon "crossbot_teleport $($startSpawn.X) $($startSpawn.Y) $($startSpawn.Z)" | Out-Null
    Start-Sleep -Milliseconds 500

    $status = Invoke-Rcon "crossbot_nav_status"
    Write-Host "After teleport status: $status"

    Write-Host ("Setting nav goal to ({0:F1},{1:F1},{2:F1})..." -f $goal.X, $goal.Y, $goal.Z)
    Invoke-Rcon "crossbot_nav_goal $($goal.X) $($goal.Y) $($goal.Z)" | Out-Null

    # Wait for Repath to run and waypoints to be generated
    Write-Host "Waiting for path planning..."
    $planSw = [System.Diagnostics.Stopwatch]::StartNew()
    while ($planSw.Elapsed.TotalSeconds -lt 5) {
        Start-Sleep -Milliseconds 100
        $status = Invoke-Rcon "crossbot_nav_status"
        if ($status -match "waypoint=\d+/(\d+)" -and [int]$matches[1] -gt 0) { break }
    }

    $wpDump = Invoke-Rcon "crossbot_waypoints"
    Write-Host "Waypoints dump:"
    Write-Host $wpDump

    # --- 5. Poll until arrival or timeout ---
    # hasGoal=0 from crossbot_nav_status is the primary signal (more reliable than
    # qconsole.log for short goals). Positions collected here double as a fallback
    # trajectory if the plugin's own trace log comes back empty.
    # Everything below is timed on the wall clock: each RCON round trip costs real
    # time, so counting poll iterations understated elapsed time by ~2.5x and made
    # both -Timeout and -TimeoutMultiplier mean far more than they said.
    $expectedEta = $null
    $pollInterval = 2
    $reason = "unknown"
    $pollTracePoints = @()
    $sw = [System.Diagnostics.Stopwatch]::StartNew()

    Write-Host "Polling for goal arrival (interval=${pollInterval}s)..."

    while ($sw.Elapsed.TotalSeconds -lt $Timeout) {
        Start-Sleep -Seconds $pollInterval
        $elapsed = $sw.Elapsed.TotalSeconds

        $status = Invoke-Rcon "crossbot_nav_status"
        $statusLine = ($status -split "`n") | Where-Object { $_ -match "CrossBot:.*origin=" } | Select-Object -First 1

        if ($statusLine) {
            Write-Host "Full status: $statusLine"
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
                Write-Host ("t={0,5:N1}s origin=({1:F0},{2:F0},{3:F0}) dist={4:F0} hasGoal={5}" -f $elapsed, $ox, $oy, $oz, $dist, $hasGoal)

                $pollTracePoints += [pscustomobject]@{ X = $ox; Y = $oy; Z = $oz }
            }

            # hasGoal=0 means the bot stopped pursuing the goal -- which happens both when it
            # arrives and when Repath fails and it gives up. Only the first is a pass, so
            # confirm it actually ended up near the goal.
            if ($hasGoalMatch.Success -and [int]$hasGoalMatch.Groups[1].Value -eq 0) {
                if ($originMatch.Success -and $dist -gt $GoalRadius) {
                    $result = "FAIL"
                    $reason = ("bot abandoned the goal at ({0:F0},{1:F0},{2:F0}), {3:F0} units away (limit {4}), t={5:N1}s" -f $ox, $oy, $oz, $dist, $GoalRadius, $elapsed)
                } else {
                    $result = "PASS"
                    $reason = ("goal reached, {0:F0} units away, t={1:N1}s" -f $dist, $elapsed)
                }
                break
            }
        }

        if ($expectedEta -and $sw.Elapsed.TotalSeconds -gt ($expectedEta * $TimeoutMultiplier)) {
            $reason = ("exceeded {0:N1}s ({1}x the {2:N1}s ETA) without arriving" -f ($expectedEta * $TimeoutMultiplier), $TimeoutMultiplier, $expectedEta)
            break
        }
    }

    if ($result -ne "PASS" -and $sw.Elapsed.TotalSeconds -ge $Timeout) {
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
            $zStats = $tracePoints | Measure-Object -Property Z -Minimum -Maximum
            Write-Host ("Read {0} trace points from {1} (z {2:F0}..{3:F0})" -f $tracePoints.Count, $traceLogPath, $zStats.Minimum, $zStats.Maximum)
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
    $svgRun = Invoke-Native -Exe $navGenExe -Arguments $navgenArgs
    Write-Host $svgRun.Output

    if (-not (Test-Path $svgOut)) {
        Write-Warning "SVG generation may have failed: $svgOut not found"
    } else {
        Write-Host "SVG written: $svgOut"
    }
    if (Test-Path $navOut) { Write-Host "NAV written: $navOut" }

    "$result`t$reason" | Set-Content -Path $resultOut

    if (-not $NoIndexUpdate) {
        Update-IndexHtml -MapName $mapName -Result $result -OutputDir $outputDir
    }
}
finally {
    # --- Cleanup ---
    Write-Host "Stopping HLDS (PID=$serverPid)..."
    & "$scriptDir\test_server_stop.ps1" -Id $serverPid | Out-Null
    # Also kill any child processes just in case
    Get-CimInstance Win32_Process -Filter "ParentProcessId=$serverPid" -ErrorAction SilentlyContinue |
        ForEach-Object { Stop-Process -Id $_.ProcessId -Force -ErrorAction SilentlyContinue }
}

if ($result -ne "PASS") { exit 1 }
exit 0
