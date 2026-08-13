# Testing Documentation

What the test and sweep scripts do. `AGENTS.md` has the bare invocations; the `crossbot-nav` and
`crossbot-navgen` skills have the technique for reading the results.

## scripts/test_nav_goal.ps1

One end-to-end run: spins up HLDS on the map, adds a bot, picks a goal, drives the bot to it, and shuts
down. Exit code 0 on goal reached, 1 on timeout or failure. This is the only check that exercises the
follower — everything else measures the mesh or the plan.

What it does per run:
- Resolves the goal itself: the farthest spawn pair, or the `info_target` if the map has one. It is
  therefore the same start/goal pair `navgen --dump-path` uses, and usually *not* the pair a live bot
  gets from `crossbot_nav_goal`.
- Polls the bot over RCON rather than parsing the shared `qconsole.log`.
- Collects the movement trace, then runs `navgen --dump-svg ... --trace ...` to write
  `nav_debug/<map>.svg` and `<map>.nav`, so the failure is inspectable afterwards. A run that never
  produced a trace still draws — an empty or missing trace file draws nothing rather than failing.
- Updates `nav_debug/index.html` with the pass/fail mark.

Options: `-Timeout` (absolute cap, default 60s), `-TimeoutMultiplier` (on the bot's reported ETA,
default 6.0 — the run fails at min(ETA × multiplier, Timeout)), `-Port`, `-NavGenPath`, `-HLDSDir`,
`-OutputDir`, `-ServerLog`, `-NoIndexUpdate`.

It also writes `nav_debug/<map>.result` — a single `PASS|FAIL<tab>reason` line. That is the verdict in
machine-readable form; the runner reads it instead of scraping the transcript.

**Timing is wall clock.** The loop used to count poll iterations, which ignored the seconds each RCON
round trip costs and made both `-Timeout` and `-TimeoutMultiplier` mean roughly 2.5× what they said.
They are now honest, which is why the multiplier default had to go up: **the planner's ETA does not
model ladders.** Ladder maps take 2.6-3.1× their reported ETA running completely alone, and more under
parallel load. The multiplier is a fail-fast heuristic; `-Timeout` is the real bound. A genuinely stuck
bot never arrives at all, so tightening the multiplier buys nothing and costs false failures.

**Parallel-safe.** Every instance takes a unique port (29015-29100) and map-specific trace file, config,
server log and outputs; `crossbot_trace_path` isolates the trace log per instance. The one thing that is
*not* isolated is `nav_debug/index.html`, so parallel callers must pass `-NoIndexUpdate` and rebuild it
once at the end — concurrent read-modify-write on it loses results.

**The server is invisible, not absent.** `test_server_start.ps1` launches `hlds.exe` with
`-NoNewWindow` and its stdout/stderr redirected. The binary is console-subsystem, so it is created with
no console at all rather than being shown and hidden after the fact: nothing flashes, nothing steals
focus, and there is no window to click into and freeze the server in QuickEdit select mode. Readiness is
the engine printing `player server started` into that log, not a window handle appearing.

**The plugin's output is now per-instance.** Every `SERVER_PRINT` — `Repath`, `Replan`, the air-move
launch and landing lines — goes to `<OutputDir>/logs/<map>_server.log` for that run alone. A run that
passes tells you nothing about *why*; that log is where the route, the failures and the penalties are.
`hlds/qconsole.log` still receives everything too (via `-condebug`), but it is shared across concurrent
instances and appended to, so prefer the per-map log and read qconsole by tail if you use it at all.

## scripts/test_nav_goal_all.ps1

The same test over every `maps/*.bsp`, at most `-MaxParallel` at a time. Each map runs as its own
PowerShell child process, so its verdict is a real process exit code and its whole transcript lands in
`nav_debug/logs/<map>.log`. Summary prints map, PASS/FAIL, elapsed and reason; exit 1 if anything failed.

**`-MaxParallel` is a correctness knob, not just a speed one.** Default 6 (~210s for 45 maps). At 12 the
run takes ~140s, but the box is loaded enough that the *bots* behave differently — maps that pass alone
and at 6 have been observed falling off geometry and failing. A faster sweep you cannot trust is worth
nothing here. The hard ceiling is 86, from the port range.

**`-ServerArgs` lowers the server framerate, which is a different test from parallel load.**
`-ServerArgs "+sys_ticrate","10"` (forwarded to each child, and available on `test_nav_goal.ps1` alone)
runs the whole suite at 10 Hz instead of the default ~100. `-MaxParallel` starves the box and makes the
bot arrive at the right place late; a low ticrate changes the *shape* of a frame, and catches follower
logic that only works because a 10 ms command is too short for the engine to do anything surprising
inside it. `pfnRunPlayerMove` is where that bites — the engine halves any command over 50 ms and
re-simulates it, which is how a held `IN_JUMP` came to eject the bot off a ladder it had just touched
(see `Bot_RunPlayerMove` in `doc/bot.md`). At 100 Hz that bug was invisible; at 10 Hz it failed 5 runs
out of 5. Verify the rate actually took with `rcon stats`, whose FPS column should read ~10.9.

Killing a child does not run its cleanup, so the runner tracks the ports it owns and sweeps for
surviving `hlds.exe` at the end, matching on `+port` in the command line. It never mass-kills
`hlds.exe`: a server you started by hand on the default 27015 is outside its range and is left alone.

## Sweeping a change

Three different things can move, and each needs its own sweep. Running the wrong one is how a change
gets declared safe on no evidence.

**The mesh.** `sweep.ps1` (ships with the `crossbot-navgen` skill, not in `scripts/`) runs navgen bare
and records leaves, areas, connections and warnings. Every change to a navgen pipeline stage gets a
before/after diff over every map in `maps/` — improving one map while quietly wrecking five others is
the normal failure mode and nothing else catches it.

```powershell
& .\.claude\skills\crossbot-navgen\scripts\sweep.ps1 -Out before.txt
```

**The route.** `sweep.ps1` never invokes the planner: `BuildDebugPath` only runs behind a `--dump-*`
flag. So a change under `src/nav/` produces an empty mesh diff no matter how badly it broke routing, and
routing has to be swept separately. Diff the waypoints themselves, not the summary — counts match while
every point moves:

```powershell
Get-ChildItem maps\*.bsp | ForEach-Object { $n=$_.Name
    & .\build\tools\navgen\navgen.exe $_.FullName --dump-path 2>&1 |
        Select-String '^  wp' | ForEach-Object { "${n}: $_" } } | Set-Content before_wp.txt
```

Filter to the `wp[` lines. The `debug path:` summary carries counters that shift for harmless reasons,
and leaving it in reports every map as changed. For a route-shape-only check, `'^debug path: area|^debug
search:'` is the cheaper filter.

**The bot.** `test_nav_goal.ps1`, per map. A plan diff tells you which maps *can* have changed
behaviour, so a planner change only needs those plus a control. A change in `path_follower.cpp` or
`bot_move_nav.cpp` moves no waypoint at all, so there is no diff to narrow by and the map list has to
come from what the change touches.
