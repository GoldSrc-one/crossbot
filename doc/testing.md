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
default 2.0 — the run fails at min(ETA × multiplier, Timeout)), `-Port`, `-NavGenPath`, `-HLDSDir`,
`-OutputDir`.

**Parallel-safe.** Every instance takes a unique port (29015-29100) and map-specific trace file, config
and outputs; `crossbot_trace_path` isolates the trace log per instance. It still starts a real HLDS per
map, so a full-suite run is heavy and will collide with anyone using the server by hand.

**The verdict goes to the console, not the pipeline.** It is printed with `Write-Host`, so
`... | Select-String '=== RESULT'` captures nothing. Redirect the whole run to a file and parse that;
the exit code is easy to lose through a pipeline too.

**The script's output does not contain the plugin's.** The transcript is the script talking over RCON;
every `SERVER_PRINT` — `Repath`, `Replan`, the air-move launch and landing lines — goes to
`hlds/qconsole.log` instead. A run that passes tells you nothing about *why*; the log is where the
route, the failures and the penalties are. It is shared across parallel runs and appended to, so read
it by tail, not from the top.

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
