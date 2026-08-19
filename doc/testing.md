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

## scripts/test_items.ps1

Drives the gather goals, which `test_nav_goal.ps1` cannot: that script commands a goal and waits for
`hasGoal=0`, and gathering re-arms `hasGoal` a frame after every arrival, so it sets `crossbot_gather 0`
and never exercises any of this. `test_items.ps1` is the opposite — it adds bots, gives them no goal at
all, and lets them choose.

It has no pass/fail verdict, deliberately. There is no single event that means "gathering works"; what
you read is the decision log in `nav_debug/logs/<map>_items.log`, one line per outcome — `wants X`,
`picked up X`, `is full of X`, `drained X`, `topped up at X`, `reached X without touching it`. A healthy
run is a *cycle*: every `wants` is followed by an outcome and another `wants`. A `wants` with no outcome
is the bot stuck on the way; the same `wants` repeating with no outcome between is the case-D spin that
the ignore table exists to break.

`-GatherDebug` turns on `crossbot_gather_debug` before the bots are added, which puts the full
candidate scan — every item, its area, its cost in seconds or why it was skipped — into the same log.
That is the view that answers "why did it go *there*"; the outcome lines only say what happened.

`-Bots N` is how the contested cases get tested. One bot never races anyone, so case C (seeing from a
distance that an item is already taken) needs at least two. Three on `test56_recharger` also kill each
other, which is the only way `weaponbox` handling and the respawn `fullMemo` clear get exercised at all.

Run `crossbot_list_items` before believing anything else on a new map: it prints every item entity with
the area it resolved to, its `frame` and its `solid`. **An item reading `area none` is invisible to the
bot**, and on `test56_recharger` the health charger and both batteries do exactly that — navgen prunes
17 of that map's 20 areas as unreachable from the single spawn, so the upper floor has no mesh. That is
a map/mesh gap, not a gather bug, and no amount of staring at the bot will show it.

## scripts/test_aim_gaze.ps1

Drives the aim delay queue, which nothing else can. `test_nav_goal.ps1` spawns exactly one bot, and the
gaze scaffold that feeds the queue is line-of-sight gated, so a lone bot never has a target — the nav
sweep can only ever prove the queue is inert when `crossbot_aim_gaze` is off.

Two bots turned loose on a map is not enough either. They chase items independently and are almost never
in each other's line of sight; a first pass at this script did exactly that and the ring sat empty
through nearly the whole run. So the script sends both bots to one spawn point with `crossbot_nav_goal`
and `crossbot_gather 0`, waits for them to arrive, and lets them idle facing each other.

**Do not converge them with `crossbot_teleport`.** It puts every bot on the *identical* origin, which
makes the eye-to-eye vector zero — there is no angle to compute, so `err` reads `-`, the ring is never
fed, and the bots are wedged inside each other and cannot walk out. That looks exactly like a broken
queue and is not one.

Read the `err` column and nothing else. It is the angle between a bot's real view and the true
present-time angles to its target, and it is the only thing that separates "correctly delayed" from
"delayed and also wrong" — both of which look identical when spectating.

The **static** phase is the sharp test: a stationary target must be tracked exactly at *every* delay,
because extrapolating a still target forward and reading it back cancels perfectly. Any nonzero `err`
there, or a `served` that does not match the requested delay, is a real defect. It currently measures
0.00 at 0, 0.25 and 0.5.

The **roam** phase cannot be read that way and the script says so. `err` there runs 4 to 19 degrees and
mixes three unrelated causes — genuine target acceleration, the parallax cost of storing angles, and the
gate suppression pointing the bot at a ladder instead — with no way to attribute it. Watch `queue` and
`served` in that phase, not `err`.

Neither phase exercises the extrapolation, because a still target has no velocity to extrapolate and
`crossfire` has no stretch long enough to hold a constant one. That check is manual: join the server,
let a bot lock on, and strafe at constant speed — its head should stay *glued* to you, not trail you.
Reverse sharply and it should overshoot and correct. Step behind a wall and it should keep tracking for
about a delay before stopping; step back out and nothing should move for about a delay before the flick.

## scripts/test_aim_error.ps1

Measures the intentional aim error by firing the same flick many times through `crossbot_aim_test` and
reporting the spread of settle times at each `crossbot_aim_error` scale.

The point is the spread, not any single row. `brake` is sampled once per flick, so one flick is one draw
from a distribution and proves nothing on its own — the same argument as the `~3s` poll grid under
`test_nav_goal.ps1`, for a different reason.

Two things to read off it:

- **At `crossbot_aim_error 0` the spread must collapse** onto the phase 1 controller numbers (about 0.37s
  at 900 deg/s for a 180, 0.28s at 557 for 90 of pitch). Brake is forced to exactly 1 there, so anything
  else means the error model leaked outside its cvar. This is the regression check.
- **At scale 1 the distribution must open up asymmetrically** — the maximum should move much further than
  the mean. That long tail is the overshoot-and-correct case. A symmetric widening would mean the flick
  detector is resampling mid-movement instead of once per flick.

The script leaves `crossbot_aim_drift` at its default on purpose: micro-drift is suppressed while a test is
active, and a run that produces settle lines at all is what proves that suppression works. If drift ever
leaks into a test, no flick settles and the log is empty rather than wrong.

Settle is gated on hand speed as well as angle, so a view flying through the target on its way to an
overshoot cannot report a false early settle. It is a threshold and not a proof: a small enough overshoot
crosses slowly enough to slip under it, which is another reason to read the spread rather than a row.

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
