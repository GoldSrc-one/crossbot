# Testing

What the test and sweep scripts do and how to read what they print. `AGENTS.md` has the bare
invocations.

## The shared harness

Every script drives the live `D:\hlds` install through CrossPlay, except `test_standalone.ps1`, and they
share one way of starting, talking to and stopping a server.

### What the tests touch

`test_server_start.ps1` launches `hlds.exe -game crossplay -xgame1 valve -sgame1 valve` (`-GameDir` and
`-Games` override both), so the tests share the install with any server you run by hand. What they write
is deliberately narrow: maps into `valve\maps\`, nav and traces under `crossplay\addons\crossbot\`, and a
uniquely named `-ConfigName` cfg into `crossplay\`. They never touch `server.cfg`, `mapcycle.txt`,
`liblist.gam` or `plugins.ini` (`doc/games.md`).

Two preflight warnings come from the same script, and either means the result is not about the code you
just changed:

- the plugin is not listed in `crossplay\addons\metamod\plugins.ini`, so it will not load at all;
- the deployed `crossbot_mm.dll` is older than `build\crossbot_mm.dll`: rerun `build.bat`, the server was
  probably still up.

With `-RequireVerbose` it refuses to continue (`EXITED code=-3`) when the `crossbot_verbose` cvar is not
registered: the plugin did not load, or was built without `-DCROSSBOT_VERBOSE=ON`.

### Starting and talking to a server

- **The server is invisible, not absent.** `hlds.exe` is started with `-NoNewWindow` and redirected
  stdout/stderr. Being a console-subsystem binary it gets no console at all, so nothing flashes or steals
  focus, and there is no window to click into and freeze in QuickEdit mode.
- **Readiness is the server answering RCON** (`echo crossbot_ready` through `rcon.ps1`), not
  `player server started` in the log: redirected stdout is block-buffered, so that line can sit unflushed
  long after the map is live.
- **`rcon.ps1` speaks the GoldSrc out-of-band protocol** (challenge, then command with challenge and
  password), so a live server can be driven mid-run. It waits 3 s for the first packet and 1 s for each
  further one; a shorter drain truncates the reply, which reads downstream as a map with no
  `info_target`. It strips the `0xFF*4 + 'l'` prefix from *every* packet. A missing server is a normal
  poll outcome, so it returns an empty string rather than throwing under `Stop`, and always a string,
  since callers regex the result.
- **Stdin is a real pipe** held open by a per-port `waitfor`, and `test_server_stop.ps1` kills only its
  own pid and that waiter, never every `hlds.exe`.
- **Every scripted `crossbot_add` names a game**: `crossbot_add valve TestBot`.
- **The server has 16 slots unless `-MaxPlayers` says otherwise.** Bot 17 on the default logs
  `CrossBot: engine refused to create a fake client.` and the run silently shrinks; `-ExtraArgs
  -maxplayers 32` does not override `+maxplayers`. Grep for `engine refused` before quoting a bot count.
- **Plugin cvars go through `-BootCommands`, not `-ServerArgs`.** `-BootCommands "crossbot_sense 0"`
  (on `test_nav_goal.ps1` and forwarded by `test_nav_goal_all.ps1`) appends to the generated cfg that runs
  once the level is active. `-ServerArgs "+crossbot_sense","0"` **silently does nothing**: the engine
  parses the command line before `GameDLLInit` registers the cvar, whose default then overwrites it.
  Engine cvars like `sys_ticrate` exist at parse time and are fine. Confirm such a run took by reading the
  cvar back (`sense=` in `crossbot_sense_status`).
- **`bots_max` kicks bots.** The deployed `game.cfg` sets `bots_max 0`, and `crossbot_menu.amxx` kicks
  every bot above quota within a second (`Kicking a bot... (quota (cstrike))`). Scripts either pause the
  menu plugin (`test_objectives.ps1`) or set `bots_max N` (`test_wiggle.ps1`). Grep the server log for
  `Kicking a bot` before trusting any multi-bot result. `test_nav_goal_all.ps1` passes `-BootCommands` to its
  children unquoted, so a command with a space must carry its own quotes:
  `-BootCommands @('"amxx pause crossbot_menu.amxx"')`. Without them every child dies binding `crossbot_menu.amxx`
  to `-GoalRadius`, 51/51 "no verdict recorded".

### Where the output goes

Every `SERVER_PRINT` of a run (`Repath`, `Replan`, the air-move launch and landing lines) goes to
`<OutputDir>/logs/<map>_server.log` for that run alone. A pass says nothing about *why*; that log has the
route, the failures and the penalties. `D:\hlds\qconsole.log` gets everything too (`-condebug`), but it is
shared by concurrent instances and appended to, so prefer the per-map log and read qconsole by tail.

## scripts/common.ps1

Shared helpers, dot-sourced by every driver:

| helper | job |
| --- | --- |
| `Invoke-CbRcon` / `Invoke-CbRconQuiet` | the one rcon call shape, returning text or discarding it |
| `Get-CbDistance3D` | distance between two parsed points |
| `Copy-CbMap`, `Ensure-CbNav` | copy the map; run navgen when the `.nav` is missing or `-Force` |
| `Get-CbServerPid` | `EXITED` check plus pid parse |
| `Get-CbSpawns`, `Get-CbFarthestPair` | spawn list and the farthest spawn pair |
| `Assert-Cb`, `Measure-CbCount`, `Invoke-CbPhase` | the deploy/monster phase shape; `-LogPrefix` and `-ConfigPrefix` tell the suites apart |

Kept separate on purpose: `test_nav_goal_all.ps1`'s `Stop-ServerOnPort` kills by port range after a child
is shot, a different job from `test_server_stop.ps1`'s kill by pid; `profile_hlds.ps1` keeps its own
launch (32 slots, `-NoConDebug`, `-NullStdout`). `generate_all_svgs.ps1` rebuilds every map SVG and the
debug gallery's `index.html`.

`nav_spawn_test.ps1` and `nav_target_test.ps1` stay as the only drivers that assert on the `reached goal`
log line (`doc/diagnostics.md`). The single-use flags (`-GatherDebug`, `-FunDebug`, `-NoGather`,
`-SkipRoam`, `-Full`) stay too: each gates one rcon line, and deleting one only moves that line into a
chat message.

## scripts/test_nav_goal.ps1

One end-to-end run: boot HLDS on the map, add a bot, pick a goal, drive the bot there, shut down. Exit 0
on goal reached, 1 on timeout or failure. It is the only check that exercises the follower; everything
else measures the mesh or the plan.

Per run it:

- **Resolves the goal itself**: the `info_target` if the map has one, else the farthest spawn pair. That
  is the pair `navgen --dump-path` uses, usually *not* the one `crossbot_nav_goal` gives a live bot.
- **Polls the bot over RCON**, not the shared `qconsole.log`.
- **Draws the run**: collects the movement trace and runs `navgen --dump-svg ... --trace ... --no-vantage`
  into `nav_debug/<map>.svg`. A missing or empty trace draws nothing rather than failing. `--no-vantage`
  because only `--dump-vantage-svg` reads the visibility field, and computing it doubled the navgen cost.
  `nav_debug/<map>.nav` is copied from the file the bot ran on, so it keeps its vantage section.
- **Records the verdict**: `nav_debug/<map>.result` is one `PASS|FAIL<tab>reason` line, which the runner
  reads instead of scraping the transcript; `nav_debug/index.html` gets the pass/fail mark.

It boots with `crossbot_difficulty 1`, `crossbot_gather 0`, `crossbot_fun 0` and `crossbot_deploy 0`:

- **Gather and fun off:** the script treats `hasGoal=0` as arrival and polls every 2 s, and gathering
  re-arms `hasGoal` about 20 ms after each arrival, so the poll never sees it and every map times out.
  Use `test_items.ps1` and `test_fun.ps1` for those tiers.
- **Deploy off:** a mined doorway changes every route through it for the rest of the run (see "Sweeping
  a change").

Options: `-Timeout` (absolute cap, default 60 s), `-TimeoutMultiplier` (on the bot's reported ETA,
default 6.0; the run fails at `min(ETA × multiplier, Timeout)`), `-GoalRadius` (96), `-Port`,
`-NavGenPath`, `-HLDSDir`, `-OutputDir`, `-ServerLog`, `-NoIndexUpdate`, `-ServerArgs`, `-BootCommands`.

- **Timing is wall clock.** Counting poll iterations ignored each RCON round trip and stretched both
  timeouts about 2.5x. **The planner's ETA does not model ladders**: ladder maps take 2.6-3.1x their ETA
  alone, more under parallel load. The multiplier is a fail-fast heuristic and `-Timeout` the real bound;
  a stuck bot never arrives at all, so a tighter multiplier only adds false failures.
- **Arrival is `hasGoal=0` within `-GoalRadius`** (legitimate arrivals land 24-50 units out, since the goal
  is clamped into its area). `hasGoal=0` alone is not arrival: a failed repath also drops the goal.
- **Spawn and target counts must read the same twice in a row**: a split UDP reply looks like a map with
  fewer spawns.
- **Native stderr is kept out of the verdict**, or under `Stop` a warning line would fail the run.
- **Arrival times are bucketed** on the poll grid (2.0, 5.0, 8.1, 11.1 s ...), so a run that misses a poll
  by a tenth of a second reports ~3 s later. Never compare single numbers (`doc/perception.md`, "Where the
  constants come from", has a worked example).

**Parallel-safe.** Every instance takes a unique port (29015-29100) and its own trace file, config, server
log and outputs; `crossbot_trace_path` isolates the trace per instance. `nav_debug/index.html` is the one
shared file, so parallel callers pass `-NoIndexUpdate` and rebuild it once at the end.

## scripts/test_nav_goal_all.ps1

`test_nav_goal.ps1` over every `maps/*.bsp` (51 today; `-Filter` and `-MapsDir` narrow it), at most
`-MaxParallel` at a time. Each map runs as its own PowerShell child, so its verdict is a real exit code
and its transcript lands in `nav_debug/logs/<map>.log`. The summary prints map, PASS/FAIL, elapsed and
reason, and exits 1 if anything failed.

- **`-Overhead` (default 120 s) is the child's budget outside the goal poll**; the kill deadline is
  `-Timeout` plus it. It covers the navgen run that builds the `.nav`, HLDS boot, and the navgen run that
  draws the SVG. A killed child reports FAIL whatever the bot did, so when it had already recorded a
  verdict the summary prints it: `killed at deadline (180s) after: goal reached, ...` is a harness
  problem, not a nav one.
- **Run it under `pwsh` 7, not Windows PowerShell 5.1.** Under 5.1 the `Start-Process -PassThru` handle is
  broken and every map fails at once: **0/N**, a null `StartTime`, an absurd negative elapsed (around
  -63,922,828,000 s, `Get-Date` minus nothing), and every per-map reason still reading "goal reached". That
  combination is the shell. Confirm any mass failure with a single `test_nav_goal.ps1` run.
- **`-MaxParallel` is a correctness knob.** Default 6 (~210 s for 45 maps when measured). At 12 it takes
  ~140 s, but the box is loaded enough that bots behave differently: maps that pass alone and at 6 fall off
  geometry. The hard ceiling is 86, from the port range.
- **`-ServerArgs "+sys_ticrate","10"` is a different test from parallel load.** Starving the box makes the
  bot arrive late; a low ticrate changes the *shape* of a frame and catches follower logic that only works
  because a 10 ms command is too short for anything surprising. The engine halves any command over 50 ms
  and re-simulates it in `pfnRunPlayerMove`, which is how a held `IN_JUMP` ejected the bot off a ladder it
  had just touched: invisible at 100 Hz, 5 of 5 failures at 10 Hz (`doc/movement.md`,
  `Bot_RunPlayerMove`). Verify the rate took with `rcon stats` (FPS about 10.9).
- **Cleanup.** Killing a child skips its cleanup, so the runner tracks the ports it owns and at the end
  kills surviving `hlds.exe` whose command line has one of its `+port`s. A server on the default 27015 is
  outside its range and left alone.

## scripts/test_standalone.ps1

```powershell
.\scripts\test_standalone.ps1 -Game cstrike -Map de_dust2
```

The smoke check for `build.bat standalone` (`doc/games.md`, "Standalone build"). It needs no CrossPlay at
runtime: it runs `deploy_standalone.ps1`, which writes only `<game>\addons\crossbot\` and reads gamedata and
the `.nav` from `crossplay\`, then boots `-game <game>` through `test_server_start.ps1` with the `-dll` and
`+localinfo` pair. `crossbot_add` here takes no game. `liblist.gam` and `plugins.ini` stay untouched, and
the "no plugins.ini" warning is suppressed because it does not apply. The log is
`nav_debug\logs\standalone_<game>.log`.

Two bots join, and it prints `PASS` or `FAIL` from:

- `game #0 '<game>' ready` and `added Bot1 to '<game>'` in the log;
- two `crossbot_teams_status` reads 20 s apart with at least one bot's `origin=` changed;
- on `cstrike`, `czero`, `dod` and `tfc`, no bot on team 0;
- no `CrossBot: FATAL`, and the server still alive.

The team check is what catches a CS build that lost its `pev->team` mirror: the bots still spawn and walk.
It proves the plugin boots and bots play, not their quality; the nav, fire and deploy sweeps are
CrossPlay-only.

| game | map | `-WaitSeconds` | result (2026-10-30, 2 bots, RelWithDebInfo) |
|---|---|---|---|
| valve | crossfire | 60 | PASS |
| gearbox | op4_bootcamp | 60 | PASS |
| cstrike | de_dust2 | 60 | PASS |
| dod | dod_anzio | 180 (`.NOD` rebuild) | PASS |
| tfc | casbah | 60 | PASS |
| dmc | dcdm5 | 60 | PASS |

## Sweeping a change

Three things can move, and each needs its own sweep. Running the wrong one is how a change gets declared
safe on no evidence. **Never judge a change on one map**: improving one while wrecking five others is the
normal failure mode.

**The mesh.** Run navgen bare over every map before and after, and diff its summary lines (leaves, areas,
connections, ungated connections, ill-formed footprints, `no route`):

```powershell
Get-ChildItem maps\*.bsp | ForEach-Object { $n=$_.Name
    & .\build-x64\tools\navgen\navgen.exe $_.FullName 2>&1 |
        Select-String 'empty leaves|-> \d+ areas|walk connections|no gate|ill-formed|no route' |
        ForEach-Object { "${n}: $_" } } | Set-Content before_mesh.txt
```

**The route.** A bare navgen run never invokes the planner (`BuildDebugPath` runs only behind a `--dump-*`
flag), so a change under `src/nav/` gives an empty mesh diff however badly it broke routing. Diff the
waypoints, not the summary, since counts can match while every point moves:

```powershell
Get-ChildItem maps\*.bsp | ForEach-Object { $n=$_.Name
    & .\build-x64\tools\navgen\navgen.exe $_.FullName --dump-path 2>&1 |
        Select-String '^  wp' | ForEach-Object { "${n}: $_" } } | Set-Content before_wp.txt
```

Keep to the `wp[` lines: the `debug path:` summary carries counters that shift for harmless reasons. For a
route-shape-only check, `'^debug path: area|^debug search:'` is the cheaper filter.

**The bot.** `test_nav_goal.ps1`, per map. A plan diff says which maps *can* have changed, so a planner
change needs only those plus a control. A change in `path_follower.cpp` or `bot_move_nav.cpp` moves no
waypoint, so the map list has to come from what the change touches.

The nav sweep is blind to two systems, and that cuts both ways:

- **Combat** (`test_fire.ps1`). One bot per server means no target, so `BotFire::Think` exits on its first
  branch every frame. A firing change can be cleared for movement regressions by the nav sweep alone, and
  a full nav pass says nothing about whether firing works.
- **Deployables** (`test_deploy.ps1`), blind by construction: the sweep boots with `crossbot_deploy 0`,
  because a bot that mines a doorway changes every route through it and a map would fail for a beam
  rather than for the change. With deploy off `BotDeploy` never runs, the beam registry stays empty, and
  every deploy branch in `bot_avoid.cpp` early-outs. If a sweep failure's log has no `avoid` or
  `monster_tripmine` lines, the deployable code did not run there. A second sweep with deploy **on** is
  worth doing once for an avoidance change, read as an observation, not a gate: with three bots mining
  crossfire the routes stop being repeatable.

## scripts/test_items.ps1

Drives the gather tier, which the nav sweep switches off. It adds bots, gives them no goal, and lets them
choose. Defaults: `-Map maps\test56_recharger.bsp`, `-Bots 1`, `-Seconds 60`.

There is no pass/fail verdict: no single event means "gathering works". Read the decision log,
`nav_debug/logs/<map>_items.log`, one line per outcome: `wants X`, `picked up X`, `is full of X`,
`drained X`, `topped up at X`, `reached X without touching it`.

- **A healthy run is a cycle**: every `wants` is followed by an outcome and another `wants`.
- A `wants` with no outcome is the bot stuck on the way.
- The same `wants` repeating with no outcome between is the outcome-D spin the ignore table exists to break
  (`doc/goals.md`, "The four outcomes").

`-GatherDebug` turns on `crossbot_gather_debug` before the bots are added, putting the full candidate scan
in the same log: the view that answers "why did it go *there*" (`doc/goals.md`, "Reading
crossbot_gather_debug").

**Run `crossbot_list_items` first on a new map.** It prints every item entity with its area, `frame` and
`solid`. **An item reading `area none` is invisible to the bot.** On `test56_recharger` the health charger
and both batteries do: navgen prunes 17 of the map's 20 areas as unreachable from the single spawn, so the
upper floor has no mesh. That is a map/mesh gap, not a gather bug.

**`-Bots N` is how contested cases are tested.** Outcome C (seeing from afar that an item is taken) needs at
least two bots. Three on `test56_recharger` also kill each other, the only way `weaponbox` handling and the
respawn `fullMemo` clear get exercised.

**`pool_party` is the map for healing volumes**, the only one in `D:\hlds\valve\maps` that has one:

```powershell
pwsh -File scripts/test_items.ps1 -Map D:\hlds\valve\maps\pool_party.bsp -Bots 5 -Seconds 180 -GatherDebug
```

Five bots, because nothing selects a pool until something has been hurt: a lone bot stays at 100 and every
scan line reads `no need`. Look for `reached trigger_hurt at 66, wading in` followed by
`topped up at trigger_hurt after 1.7s, now at 100`. `is already full at trigger_hurt` from a bot that
arrived hurt is the failure: it stopped somewhere the engine does not count as inside the brush.
`crossbot_list_items` should show `kind 6` with a comma-separated list of areas, and the map's other
`trigger_hurt`, the `dmg 200` one, must not appear.

## scripts/test_fun.ps1

The `test_items.ps1` of the fun tier: no verdict, a decision log to read. It runs navgen, boots HLDS,
prints `crossbot_list_fun`, adds bots (default 4), polls `crossbot_nav_status` every 10 s, prints
`crossbot_list_fun` again at the end, and tails the decisions from `nav_debug/logs/<map>_fun.log`.

```powershell
pwsh -File scripts/test_fun.ps1 -Map D:\hlds\valve\maps\stalkyard.bsp -Bots 3 -Seconds 120
```

`-FunDebug` sets `crossbot_fun_debug 1`. `-NoGather` sets `crossbot_gather 0`, which strips the detour lines
and leaves only the tier's own decisions.

**Read the inventory before the behaviour.** The first `crossbot_list_fun` is a per-map assertion, and it
catches chain-analysis bugs that the bots would only show as vague listlessness:

| map | must list | must **not** list |
| --- | --- | --- |
| crossfire | 2 tank controls → `func_tankmortar`; the `trigger_multiple` at `(0,-2219,-1846)` | the 4 `lift*` buttons, the 4 inert tower buttons |
| doublecross | 2 tanks; the `fire_button_texture` button | the fan-lift and access-panel buttons |
| stalkyard | 2 tanks (`biggun`, `biggun2`) | `armorall`, the `lift_1` button |
| undertow | the `wait 142` button at `(1640,152,184)` | the `bunker_door` button |
| snark_pit | the `wait 120` snark button at `(304,-640,48)` | the four door/lift buttons |
| datacore | nothing: `0 fun entities` | — |
| subtransit | one `tram` row for the `func_tracktrain`, 19 board areas | the two `func_rot_button`s (their chain ends at `pipedoor`) |

- **crossfire's trigger is the reachability assertion.** It sits behind `strike_ready_door`, which opens
  180 s into the map, so before t=180 it must read `no nav area` and must not be a candidate.
- **subtransit's board points belong to the track, not the tram**: the 19 areas and the second listing
  must be identical whether the tram is parked at `speed 0` or circling at `speed 400`. The row prints
  `deck (-608.0,-896.0,-167.0)` while the tram sits at `(-544,-896,-200)`; the 64-unit offset is the brush
  model's offset from its origin. A deck equal to the origin means `DeckAt` was bypassed.

**A healthy run is a cycle**, and it takes more than one bot to see all of it:

```
Player1 wants fun func_tankcontrols ... 7 candidates
Player1 detours to item_healthkit ... 18.3s left after
Player1 detours to ammo_rpgclip   ... 18.0s left after
Player1 reached the tank func_tankcontrols, going to use it
Player1 mounted the tank func_tankmortar
Player1 is bored of the tank, nobody around for 10.0s, readiness 0.45
```

The other legitimate exit is `is shot from behind the tank by X, bailing out`, which must be followed by the
bot entering combat, not re-selecting a goal. **A lone bot never sees an enemy, so it always hits boredom
and never the damage exit**: use at least three.

Failure signatures, each of which looks like something else:

- **A mount immediately followed by a dismount with no reason logged** is the bot moving and breaking the
  30-unit `m_vecControllerUsePos` leash, not an aiming or gamedata problem.
- **A fresh `wants` line where a `detours to` should be** is broken target stickiness. Rerun with
  `-Bots 1`, where combat cannot explain it.
- **`could not aim at` or `got nothing out of` at a tank, with every gamedata read correct**, is the
  `UTIL_ClampVectorToBox` trap (`doc/goals.md`, `bot_fun.cpp`). Small brushes hide it, so undertow passing
  proves nothing about stalkyard.

**The switch-priority rule cannot be exercised on a stock map** (`doc/goals.md`, "Switches outrank
everything else in the tier"). Check instead that each kind selects on its own: undertow must log
`wants fun func_button`, then `pressed func_button ... leaving it for 142s`, and the tier must stay empty
for the rest of the run; stalkyard must still mount tanks; subtransit must still board the tram. `datacore`
is the negative control: an empty tier and gather untouched. subtransit is the only map with a `tram`, so
stalkyard and crossfire double as proof the tram branch is unreachable elsewhere.

### subtransit, the tram

Two transcripts must appear over a long enough run, the first once and the second repeatedly. Start it:

```
Player2 boarded the tram
Player2 is driving the tram, speed 400
Player2 is riding the tram at 400
Player2 is bored of the tram, nobody around for 10.0s, readiness 0.20
Player2 jumped off the tram onto area 403
```

and catch it, possible only once somebody has started it:

```
Player2 reached the track, tram due in 1.9s, waiting
Player2 boarded the tram
Player2 is riding the tram at 400
```

The readiness ban should be the **most common** line, since a ~25 s lap against a 3 s catch window makes most
arrivals too early:

```
Player1 reached the track, tram not due for 5.8s, leaving it until readiness beats 0.35
```

Failure signatures, each pointing at one mechanism (`doc/goals.md`, `bot_fun.cpp`):

- **`boarded the tram`, then `riding the tram at 0`**: the mount never took, and after `kTrainDriveTimeout`
  the bot rode a stationary tram. Either it stands on the deck *edge*, so `OnControls` fails its origin box
  test, or it is firing and `m_flNextAttack` keeps `PlayerUse` out of reach. Neither logs anything.
- **`could not get on the tram`, tram parked**: the board point is unreachable or the hop does not clear
  the deck lip. Look for a board-point stand z far from the deck z.
- **`could not get past func_tracktrain`**: a board point is inside the parked tram's footprint; the probe
  offsets must clear the tram's half-width *plus the bot's hull*.
- **The board list loses the areas nearest the parked tram, and nobody ever starts it.** No failure is
  logged anywhere: with no board point at arc 0 nobody reaches the tram, and catching needs someone to have
  started it. Both known causes are in `BuildBoardPoints`: the ground probe starting above the area point
  inside the tunnel ceiling (`fStartSolid`), or `point.z` moved to the traced floor, dropping the air-route
  trace behind the trench lip. Diff the board areas against the 19 above first.
- **`hops for the tram` never appears while `due in` does**: the bot waits and never commits. The jump
  window is `[clearFrom, clearTo]` against `eta`; anything subtracted from `eta` parks it below the window.

**Death accounting.** With `-FunDebug` every bot death logs `died at (x,y,z), D from the tram doing S,
onground N, tram phase P`, the only way to separate the populations:

```powershell
(Select-String -Path nav_debug\logs\subtransit_fun.log -Pattern '(\d+) from the tram' -AllMatches |
    ForEach-Object { $_.Matches } | Where-Object { [int]$_.Groups[1].Value -le 20 }).Count
```

Deaths at `D <= 20` are the tram; the rest are ordinary combat and vary run to run, so a total death count
is useless here. Of the tram deaths, `tram phase wait` is a bot dying mid-catch, the hop's fault, and should
be zero. `tram phase not engaged` is a bot run over while walking, which the hop cannot help: once started the
tram circles forever and the mesh knows nothing of it. On a 300 s four-bot run expect roughly 7 of those and
none of the former.

### Regression and cost

`test_nav_goal.ps1` sets `crossbot_fun 0` with `crossbot_gather 0`, so the nav sweep is unaffected by the
tier and stays comparable with older transcripts; all 51 maps pass. The sweep exists for maps that plan a
route the bot will not follow, which navgen alone calls healthy. `test54_window` was one twice: navgen could
not route it until the launch rescale (`doc/path_planning.md`, "Rescaling a missed launch"), and then the
planner designed launches at the engine default 320 while the valve DLL held the bot at 270, refusing every
launch until `Bot_MaxSpeed` read the engine's settled cap in `PM_Move_Post` (`doc/movement.md`).
`test39_skybox` and `test41_skydrop` hitting the deadline was the sky prune (`DropSkyFloorLeaves`,
`doc/navgen.md`), not the follower.

Perf, crossfire, 8 bots, `-DCROSSBOT_PERF=ON`: `item_select` self time is 0.0288 ms/f with the tier on and
0.0264 with `crossbot_fun 0`, inside run-to-run spread, because the cached `toTargetCosts` keeps a repath at
one `AreaCosts`. The worst *frame* moves from 1.6–2.9 ms to 3.1–6.8 ms: the retarget's extra
`AreaCostsToGoal` plus the hull-3 reachability traces (0.19–1.01 traces/frame). Amortised cost nil, peak
selection frame roughly doubled. The tram adds nothing measurable: track and board points are built once per
map on the first `Collect` with a mesh, and a waiting frame costs one `ProjectOnTrack` over 43 nodes, on
subtransit only.

## scripts/test_objectives.ps1

The objectives driver, built like `test_fun.ps1`: no verdict, a decision log to read. It boots HLDS with
`valve` plus the map's game, prints `crossbot_list_objectives`, adds bots, waits, prints it again, and tails
the strategic decisions from `nav_debug/logs/<map>_objectives.log`. The map and its `.nav` must already be
installed; nothing is copied or generated.

```powershell
pwsh -File scripts/test_objectives.ps1 -Map dod_escape -Game dod -Bots 6 -Seconds 120
```

The boot config runs `amxx pause crossbot_menu.amxx`, because of `bots_max 0` (see "The shared harness").
The pause lasts until the map changes, and also drops the menu's `[BOT] ` name prefix and its DoD
breakable-touch fix.

| option | effect |
| --- | --- |
| `-StrategicDebug` | `crossbot_strategic_debug 1` |
| `-TacticsDebug` | `crossbot_tactics_debug 1` |
| `-Tactics 0`, `-Snipe 0` | `crossbot_tactics` / `crossbot_snipe` off, the tactics A/B levers |
| `-Plugin` | `crossbot_objectives 0`: the same run on the AMXX plugin's goals, the A/B control |
| `-BotGame` | the bots' game. On a `cstrike`/`czero` map it defaults to that game, native CS bots alternating `t`/`ct`; elsewhere to `valve`. `-BotGame valve` is valve bots on a CS map; `-BotGame dod` adds native DoD bots with random classes (`doc/games.md`, `dod_profile.cpp`) |

With `crossbot_verbose 1` a DoD run shows `N of N sandbags became camp spots` at map load, and gunners log
`deploys its gun` and `folds its gun`.

**Read the first listing before the behaviour.** It runs before any bot joins, so it is a pure map
assertion:

| map | must list |
| --- | --- |
| de_dust2 | 2 `func_bomb_target`, attack 2 defend 1, need id 1 |
| cs_assault | 4 `hostage_entity` payloads, carry 1; 16 `info_player_start` rescue sites, because the map has no rescue-zone entity |
| as_oilrig | 1 `func_vip_safetyzone`, attack 1, need id 3 |
| dod_avalanche | 5 `dod_capture_area`, all `attack 1,2`; #304, #305 and #307 need 2 players; no `dod_control_point` rows |
| dod_escape | 2 capture areas needing group 1 (`allies_obj`), 1 `func_breakable` attack 2 with group -1, 22 `dod_object` in two groups |

**Then read the team columns.** On a cstrike map every player line must show `pev->team` and `m_iTeam`
disagreeing (1/2 or 2/1), and the bots on `pev->team 1` must be the ones fetching hostages or defending bomb
sites. If they agree, the inversion the reader rests on has changed (`doc/goals.md`, "Measured facts the
readers rest on").

Healthy runs, measured:

- **dod_avalanche:** capture areas change owner within 40 s, and each site's `attack` mask narrows to the
  team that does not hold it.
- **dod_escape:** carriers appear by t+50; allies carrying `allies_obj` list `deliver rank 3` for both
  capture areas, and the axis carrier lists `deliver rank 3` for the breakable.
- **de_dust2:** holders cycle, `is holding the defend goal` followed about 30 s later by `has held the
  defend goal ... moving on` and a different site. With native CS bots a T carrier logs
  `is planting the bomb` / `planted the bomb` next to the mod's `Planted_The_Bomb`, and rounds end in
  `Target_Bombed` or `Bomb_Defused`. Nobody spawning and `Terrorists_Win` every two seconds means the join
  stalled (`doc/games.md`, `cs_profile.cpp`).

Signatures worth recognising:

- **`follows the strategic goal` streaming for two bots at once** is two moving goals chasing each other
  and re-planning at 4 Hz. Escort intents were removed for exactly this, so look at what is moving.
- **`could not get past player reaching the strategic goal` repeating on one site** is a whole team piling
  into its only objective, as on as_oilrig when all six land on one team. Crowding is not handled yet.
- **A stalled route that never resolves** (`mid-route repath (stalled) ... wp 1/N` for the rest of the run)
  should now end in `made no progress across 3 stalled repaths` and `could not get past repeated stalls`,
  not a silent loop. On cs_assault this was a CT route through `func_breakable` *77, a vent grate a valve
  bot cannot damage; `BotBreak::IsTarget` now accepts only the bot's own game's entities, so the CT takes the
  vent ladder (`doc/movement.md`, "What counts as breakable").

## scripts/test_wiggle.ps1

Detects a bot crawling on the spot while its move commands flip direction (`doc/movement.md`, "Teammate
spacing"). It boots `de_dust2` with `cstrike dod valve tfc` and `bots_max 8` (`-BotsMax`), so
`crossbot_menu.amxx` fills the server with a random game mix as a live server does; `crossbot_add` alone
would be kicked.

It polls `crossbot_nav_status` at about 6 Hz for `-Seconds` (240) and flags a bot whose last `-Window` (5 s)
stayed inside `-Spread` (96 units), averaged under `-MaxSpeed` (60 u/s) and reversed its velocity at least
`-Reversals` (3) times. On a flag it captures the combat, vantage, camp, directive and sense status and the
waypoint list into `nav_debug/logs/<map>_wiggle_report.txt`.

- **Read `<map>_wiggle_samples.txt` first.** The bug is `move=` at full drive alternating sign between
  samples while `vel=` stays near 20.
- **Judge it from velocity, not the view.** The view yaw stays smooth throughout, so a detector on
  `angles=` finds nothing. The speed cap keeps combat strafing (100-250 u/s, `goal=0 wp=0/0`) out.
- **Another bot printing exactly the same `origin=` with `vel=(0,0)`** is a dead player in chase cam, not a
  teammate standing inside it, and a common sign of this bug.

Measured over 300 s runs: 0 and 0 episodes over two runs. The signature of a real one is an HL bot near CT
spawn for 5+ s, its move reversing on nearly every sample, a stalled repath every 2 s, and a dead DoD
teammate on its origin. Pathing bots with an observer on them reversed the move between consecutive slow samples 2 times
in 184 and 0 in 57.

## scripts/test_sense.ps1

Drives the sense system and the aim delay queue, which nothing else can: the nav sweep's lone bot never has
a target and hears almost nothing, so it only proves the system is inert when `crossbot_sense` is off.

Three phases on crossfire with two bots by default:

| phase | arrangement | read |
| --- | --- | --- |
| **static** | both bots sent to one spawn with `crossbot_nav_goal` and `crossbot_gather 0`, idling face to face, the delay swept over `-Delays` (0, 0.25, 0.5) | `served`, `queue`, `err` trend, `leadblk` flat |
| **roam** (`-SkipRoam` skips it) | gather on, bots move | `queue`, `served`, `leadblk` climbing |
| **suspicion** | which senses fired, which tier wins, does the table decay | `pushed=`, `tier`, entry ages |

Two loose bots almost never see each other (a first version left the ring empty the whole run), hence the
convergence. **Do not converge them with `crossbot_teleport`**: it puts every bot on the identical origin, so
the eye-to-eye vector is zero, `err` reads `-`, the ring is never fed, and the bots are wedged inside each
other. That looks exactly like a broken queue.

**Static phase.** A still target must be tracked consistently at *every* delay, because extrapolating a still
target and reading it back cancels. A `served` that does not match the requested delay is a defect. `err` is
not 0 here: the bots sit at point blank, where the bone-sampling floor is tens of degrees (see
`doc/perception.md`, `crossbot_aim_status`). Read `err` only for a trend across the delay sweep. `leadblk`
arrives non-zero (banked on the walk in) but must not move: still bots have no lead, so the trace is skipped.
Any increase is a bug in the skip. A map with no deathmatch spawn (every DoD map) skips this phase.

**Roam phase.** `err` here has read anywhere from 2 to 48 degrees, mixing real target acceleration with the
gate veto pointing the bot at a ladder; no run compares with the last. Watch `queue` and `served`.
`leadblk` must climb: roaming bots cross doorways and corners constantly, so a flat count means the trace is
wrong (flags, direction, or the skip swallowing everything), not an unusually open map. `leadblk` is on the
aim line as well as the sense line because the static phase polls only the former.

**Suspicion phase.** It reports which senses fired, asserts that the player tier always wins (`tier` is
`PLAYER` whenever `kinds=p:` is nonzero), and asserts that heard entries age out.

- **Read sources from the `pushed=` counters**, never by sampling `fresh=`: with one entry per entity a
  player's `SIGHT` overwrites their `SOUND` and `DAMAGE` within one update.
- `SIGHT` and `SOUND` are reliable. `EVENT` and `DAMAGE` turn up from weapon deploys and fall damage, but
  `test_fire.ps1` exercises all four deliberately.
- **Zero tier checks is not a pass.** Two roaming bots always produce a live player entry, so an empty tally
  means the status regex stopped matching (a new field moved `tier=`, say).
- **Decay is tested by shrinking `crossbot_sense_decay` to 1 s and checking ages**: no live entry may be
  older than the decay. Do not force quiet with `crossbot_kill` (respawn empties the table whatever decay
  does), and do not assert that no `SOUND` survives.
- **Gathering stays on**, so moving bots keep emitting footsteps and there is always a live entry. On an
  idle map `susp=` sits at `0/256` and the age check passes against an empty list, so the phase fails
  outright if it never sees one. The general rule: **an assertion over a set that can legitimately be empty
  must also assert the set is non-empty.** This one survived a full green run before anyone noticed.

Sound is mostly **footsteps**: two roaming bots on crossfire make around 85 step sounds in 25 s, against 64
sounds of every other kind over a similar run. If footsteps stop, check `sv_footsteps` before the hook, and
remember steps at or below 220 units/s are silent by design (`doc/perception.md`, "Footsteps arrive through
pmove").

`-Notice <scale>` pins `crossbot_notice` for A/B runs. `-Game dod` (or any loaded gamedir) runs on that
game's map with `valve` loaded first, with a 180 s start wait because DoD map loads can spend a minute
rebuilding the game's `.NOD`. `-Coverage 0/1` pins `crossbot_coverage` for the see-through A/B, and the
summary adds `coverage= partial/blocked of traces`. Four DoD bots on dod_forest rarely meet in two minutes,
so that is a smoke test; `crossbot_sight` over chosen sightlines is the measurement (`doc/perception.md`,
"What the bot sees through").

### Checking extrapolation by hand

Neither aim phase exercises the extrapolation: a still target has no velocity, and crossfire has no
stretch long enough to hold one. Join the server, let a bot lock on, and:

- **Strafe at constant speed**: its head should stay *glued* to you, not trail.
- **Reverse sharply**: it should overshoot and correct.
- **Step behind a wall at walking pace**: it should keep tracking for about a delay, then stop; step back
  out and nothing should move for about a delay before the flick. At a walk the lead is a few units, so
  the queue is written right up to the moment you vanish and you see the drain.
- **Do the same at a sprint**: the ~80-unit lead is vetoed while you are still in the open, and the head
  parks at the corner instead of swinging into the wall. That is the only test of the lead trace's purpose
  and no script makes it.
- **Fire near a bot that cannot see you**: its head turns toward the shot. **Shoot one in the back**: it
  turns around. That is `EVENT` and `DAMAGE` by hand.

## scripts/test_fire.ps1

Drives the firing loop, the whole chain at once: sense, aim queue, weapon choice, fire ray, button. It
does the **opposite** of `test_sense.ps1`: converged bots share an origin and vision drops them for having
no eye-to-eye vector, so this script leaves `crossbot_gather 1` and lets bots roam, collect guns and meet at
real ranges. Defaults: crossfire, `-Bots 4`, polled every 2 s.

Encounters are stochastic. More bots is the only reliable lever (six on crossfire fight within a minute;
two can take several), so the pass condition is "shots were fired at real targets", never a count. `-Full`
runs the whole duel window (`-DuelSeconds`, 180) instead of stopping at the first confirmed engagement, for
deaths, reloads and the rarer suspicion sources. `-Notice` and `-Coverage` pin `crossbot_notice` and
`crossbot_coverage` for A/B runs.

### Phases

1. **Select** (`-SelectSeconds`): one bot, nothing to shoot. The only arrangement where the
   out-of-combat weapon decision runs unopposed, and where its two stages can be taken apart.
2. **Control** (`-ControlSeconds`): `crossbot_fire 0`, nothing may fire.
3. **Duel**: bots roam and fight.
4. **Seven weapon phases**, `-ProjectileSeconds` (420) split between them, at least 15 s each, run with
   `crossbot_combat 0`.

**Select.** Each assertion is on where the bot ends up, not how. Part A hands over four weapons in three
score bands (crossbow 0.5, 9mmAR 0.3, shotgun and hornetgun 0.2) and requires the held weapon to be at the
top score present. Part B hands over a gauss, tied with the crossbow at 0.5, whose carry clock starts at
zero, so rarity must move the bot onto it: the one assertion that fails outright if carry time is not
accrued.

- **Neither part names the weapon it expects.** The bot roams during the phase, and an egon picked off the
  floor ties the crossbow at 0.5 and legitimately wins on carry. The assertions read
  `crossbot_weapon_carry` and check the *rule*: top score present, then least carried at that score.
- They retry for a few seconds, because the idle reload pass outranks the score filter by design and parks
  the bot on a part-empty gun briefly.
- Churn is budgeted: two settles expected, and a weapon picked up with an empty clip (the RPG always) makes
  loading it a legitimate third. Zero shots are asserted: a bot with no target that fires has a trigger off
  the target path.

What still needs the duel and `-Full`: `idlereload` (a magazine topped up between fights), and the carry
*spread*, at least two weapons with different carry times, which shows the tally tracks acquisition rather
than running one uniform clock. A bot met mid-phase enters the tally at its current counters, not zero.

**Control** is the load-bearing half. Nothing may fire, checked two independent ways: the plugin's shot
counter, and ammo actually spent, from clip and reserve. Ammo is the honest witness, since a clip only falls
by firing and clipless weapons spend reserve directly; it catches a counter that has quietly become a no-op.

- **`crossbot_fire 0` must be set before the bots are added.** Bots spawn inside each other's sight, and an
  rcon round trip is long enough for the default `crossbot_fire 1` to get shots away.
- **A shot is not "the attack timer became busy".** Deploying a weapon and `GetNextBestWeapon`'s
  empty-weapon switch arm `m_flNextPrimaryAttack` too, so the counter in `bot_fire.cpp` counts only when the
  bot actually pressed the frame before.

### The weapon phases

Each weapon gets its own pass so the aim kinds are attributable to it:

| pin | what it proves |
| --- | --- |
| `weapon_crossbow` | `zoomed > 0`; `targetaims > 0` or `aim=target` seen; `aimveto > 0` or `off-target` seen: the zoom decision fired, `primaryZoomed` was selected (the only path to `FireSniperBolt`), and the aim gate withheld shots |
| `weapon_hornetgun` | `shots > 0` and `leadshots < shots`: equality would mean the homing cone never replaced the lead solve |
| `weapon_snark` | `shots > 0`, `leadshots == 0` (a thrown monster is never led), `targetaims == 0` (its only mode is `AIM_LEAD`), and `uphill > 0` |
| `weapon_gauss secondary` | `chargeshots > 0`, `why=charged` seen, and **no** `charge-timeout`: a full charge held, waiting for a shot, not released on a clock |
| `weapon_9mmAR secondary` | `shots > 0`, `arcshots == shots`, and a refusal (`selfveto > 0` or an `arc-*` reason). `uphill` should read 0, since the contact grenade is the only fixed-speed parabola and exempt from the uphill veto; it is reported, not asserted |
| `weapon_handgrenade` | `throwshots > 0` (counted on the falling edge of `m_flStartThrow`, reachable only through `ArcOnTarget`), a refusal (`selfveto > 0` or an `arc-*` reason), and `capped == 0` |
| `weapon_rpg` | `guides > 0`, `capped == 0`, and **no** `aim=arc` or `aim=lead`: a laser-guided rocket aims at the target, since leading would steer the spot off them |

- **The pin, not equipping.** In a fight a bot re-selects only when its weapon stops working, and score
  ranking settles on the bot's best-scoring weapon and holds it, so a handed-over crossbow sits in the
  inventory (one attempt handed out 288 crossbows and hornetguns and recorded zero projectiles).
  `crossbot_fire_pin` makes every other weapon unusable, the only lever that names one.
- **The gauss needs the mode argument too.** The mode is drawn at random once the weapon wins and not
  redrawn while the keep-loop is satisfied, so a pinned gauss gets one coin flip at reaching `FIRE_CHARGE`.
  `crossbot_fire_pin weapon_gauss secondary` makes the phase deterministic. The hornetgun reaches its
  secondary by range: its primary's flight-time ceiling rejects targets past 900 units.
- **Ammo.** The MP5 phase equips `ammo_ARgrenades` and the RPG phase `ammo_rpgclip`, since their own gives
  carry clip ammo only and one rocket. The handgrenade's own give is five, refilled every poll.
- **Every weapon phase respawns its bots first** with `crossbot_kill`. A bot carrying a gauss charge keeps
  its mode without consulting `ModeUsable`, so a charge from the gauss phase discharged into the MP5
  phase's counters (six charge shots where the gauss was unusable). Respawn drops inventory, charge and
  every per-life counter at once.

What to expect from individual phases:

- **Handgrenade reach is about 550 units**, far shorter than anything else, so it depends most on bots
  wandering close. `throwshots=0` is more likely an unlucky minute than a broken solve; compare
  `nothing-usable` with `no-target` first.
- **The RPG phase expects deaths.** The splash rule fires whenever the blast lands nearer the enemy than the
  bot, so six bots with rocket launchers in a small arena kill themselves regularly. Counters are banked on
  death, so the tallies survive.
- **`capped == 0`** means every commit closed on the event rather than its valve: the rocket for the RPG
  (six-second valve), the throw for the handgrenade. It caught the tap commit opening on presses the weapon
  had already refused (six capped of 78 throws, from the half second of `m_flNextPrimaryAttack` that
  `CHandGrenade::WeaponIdle` arms *after* each throw) while `throwshots` looked healthy. `guides` above
  `shots` is not a leak: `CRpg::Reload` pushes the attack timer while reload is held, so presses launch
  nothing; a leak would show as too *few* shots.
- **`guides` matching `shots` exactly** is the healthiest single number: two independent observations, a
  commit the plugin opened and the game DLL's attack timer moving.
- **Hornetgun 986 shots against snark 40** is the range gate working: the snark is unusable past 300 units,
  and most polls in its phase read `nothing-usable`.

### Assertions must rest on counters, not sampled strings

`why=` and `aim=` are one frame sampled every two seconds, so absence proves nothing, and any change to how
often bots hold a target silently re-tunes every such assertion. When vision dropped to two traces per bot,
target-holding frames fell to about 27% and the crossbow's `off-target`/`aim=target` checks started failing
alternately on runs whose counters read `aimveto=44 targetaims=46`. Three more runs failed the same way: the
MP5's refusal, the handgrenade's, and `aim=homing` on a snark phase with 48 snarks thrown. So:

| was asserted as | now asserted as | why it is the same claim |
| --- | --- | --- |
| `off-target`, `aim=target` (crossbow) | `aimveto > 0`, `targetaims > 0` | cumulative counts of the same two outcomes |
| `aim=homing` (snark) | `shots > 0` and `leadshots == 0` | a solved-lead press moves `leadshots`, so none across real shots *is* "the cone replaced the solve" |
| `aim=homing` (hornetgun) | `shots > 0` and `leadshots < shots` | as above |
| `aim=arc`, `why=primed` (grenade) | `throwshots > 0` | a throw is reachable only through `ArcOnTarget` |
| `aim=arc` (9mmAR) | `shots > 0` and `arcshots == shots` | |
| `arc-self`, `arc-blocked` (both arc phases) | `selfveto > 0` | a refusal is usually true and almost never on the sampled frame |
| `why=guiding` (RPG) | `guides > 0` and `capped == 0` | the commit opened and closed on the rocket |

`shots > 0` was added because `leadshots == 0` and `arcshots == shots` pass happily on a phase that fired
nothing. Reason strings still appear in the output and widen the test; they never carry it.

- **The one exception is `why=charged` on the gauss.** `chargeshots` proves a charge completed, but only the
  string proves the bot *stood holding a full charge waiting for a shot*, which is the design. It samples
  reliably because a charged gauss holds that state for seconds (27, 29 and 30 across three runs).
  `off-target` can never be checked on the gauss: `ChargeButtons` overwrites `lastReason` every frame the
  ray test fails.
- **Negative string checks stay**: `why=charge-timeout` and the RPG's `aim=arc`/`aim=lead` fail *when
  present*, so sampling can only miss a fault, never invent one.
- **`uphill`, `selfveto` and `zoomed` are poll-level witnesses**: `uphill` and `selfveto` are
  per-frame-per-mode rejection tallies, `zoomed` the polls in which some bot was zoomed. All three answer
  "did this ever happen", which is all a stochastic phase can ask.
- **`reloads` is counted in the plugin**, since `why=reloading` lasts one frame. Holding the trigger through
  an empty clip clicks forever, so reload needs a witness independent of sampling luck.
- **The 9mmAR "no shot was ever withheld" check is flaky.** It rests on one poll landing on an `arc-*`
  reason or a nonzero `selfveto`: on crossfire it failed two coverage-on runs and passed the third
  (`selfveto=2`) and a coverage-off run (`arc-blocked=1`). Repeat it before blaming a change.

**The uphill veto's scope** is guarded from both sides. It exempts arcs that **detonate on impact**, whose
end the sim knows, not arcs in general, because a handgrenade lands and rolls for the rest of its fuse. So
the snark, homing and never solved, must trip it; the MP5's contact grenade never may; and the handgrenade
trips it freely, which is reported but not asserted since it depends on terrain.

**The sticky trigger is asserted in the duel**, not a pinned phase: every bot spawns with a glock and both
its modes are `WPN_Q_STICKY`, so the duel exercises it, and a pin would cost a minute to prove the same.
`stickyshots`, the shots that happened only because the trigger stayed down off-target, is counted off the
press flag like `leadshots`, so it is cumulative. It **never counts a gauss primary**
(`CGauss::PrimaryAttack` moves no weapon timer for the `fired` edge to see), and `burned` rises because
holding the trigger off-target spends ammo on purpose.

### Reference numbers

Orders of magnitude, not thresholds. A passing 6-bot, 300 s `-Full` run on crossfire: ~1000 shots, ~420
rounds burned, 36 firing decisions, 83 target acquisitions, 47 deaths, 43 weapon switches, 22 reloads, and
all four suspicion sources. The `reasons` histogram is the most useful line: `off-target` and `blocked` both
appearing shows the fire ray is rejecting shots rather than waving everything through.

A passing 6-bot run with `-DuelSeconds 120 -ProjectileSeconds 420 -Full`: duel 399 shots / 150 burned / 10
deaths / `stickyshots=119` with all four sources; crossbow 78 shots and 29 zoomed polls; hornetgun 986 shots,
206 led; snark 40 shots, `uphill=53`; gauss 68 charged shots, no timeout; 9mmAR 48 arc shots, `uphill=0`,
`selfveto=5`; handgrenade 62 throws, `uphill=48`, `selfveto=43`; RPG 50 shots against 50 guides; `capped=0`
everywhere.

`-Full` takes well over ten minutes: run it in the background.

## Combat state (bot_combat.cpp)

No dedicated script. `BotCombat` needs several bots fighting on a real map, which `test_fire.ps1`
arranges, but the transitions read better straight from a server log:

```powershell
$log = "D:\hlds\logs\combat.log"
.\scripts\test_server_start.ps1 -Map crossfire -Port 29015 -LogFile $log -Commands @(
  "crossbot_combat_debug 1",
  "crossbot_add valve bot1","crossbot_add valve bot2","crossbot_add valve bot3","crossbot_add valve bot4")
```

Two minutes is enough for every rule to fire. Check:

- **Every exit reason appears**: `the enemy died`, `respawned`, `lost every trace of the enemy`,
  `went too long without line of sight`. `entered combat` should be within a few of `left combat`; the
  difference is fights still running at shutdown.
- **Every entry is `(sight`.** Damage only turns the head; a `(damage` entry means something enters combat on
  damage again, which is a regression.
- **`walled off` is a minority of probes**, around 20% on crossfire with `less exposed` the largest share
  (73% exposed the bug `ResolveAlongRay` fixed).
- **Fights end.** The longest `left combat with X after Ns` should be in the teens; a fight that never ends
  is this state's characteristic failure.
- **Exposure trends down**, judged in aggregate since it is a biased random walk: on a stock crossfire run
  roughly three quarters of fights end less exposed than they began.

`crossbot_combat_status` is the live view: who each bot fights, for how long, how long unseen, whether the
movement half has taken over (`driving=`), and the current probe's exposure against the bot's own.

**`crossbot_vantage 0` is the no-vantage rehearsal**: it invalidates the blend without regenerating a
`--no-vantage` nav, so probes take the unscored branch. Bots must still fight and move, every probe reading
`no vantage`.

### The awareness flag and the ambush

Same run. Check:

- **Both verdicts appear on sight entries.** Only `aware=1` means the cone test is inverted or the fov
  fallback wrong; only `aware=0` means `v_angle` is not read. A stock crossfire run splits roughly evenly.
- **All three latch paths fire**: `turned around`, `new opponent`, `shot at us`. The last needs the bot hit by
  the enemy it already fights while still unspotted, so one occurrence passes and zero is worth a longer run,
  not a bug hunt.

**Crossfire cannot exercise the ambush alone**: one `weapon_handgrenade` and two `weapon_snark` for the whole
server mean nobody holds one when they surprise someone, and `ambush=` in `crossbot_fire_status` stays 0.
Force it:

```powershell
.\scripts\rcon.ps1 -Port 29015 -Command "crossbot_equip weapon_handgrenade"
.\scripts\rcon.ps1 -Port 29015 -Command "crossbot_equip weapon_snark"
```

Re-equip every few polls (five grenades go quickly). Here equipping is right and the pin would be wrong: the
pin makes every other weapon unusable, destroying what is under test, that an ambush weapon is chosen over a
working gun the bot would otherwise have kept.

- **`ambush=` is per life** and resets with `BotFire::State` on respawn: sum the increments across a run.
- **A bot holding a throwable out of combat** is the idle rotation: grenade and snark are worth 0.1, like
  the glock, so a fresh one wins the rarity tiebreak until its carry time catches up.
- **Many ambushes declining** is correct: the uphill veto refuses both throwables at a target more than 16
  units above the eye, which on crossfire is often.

### The melee duel

Same run, three mechanisms, different levers.

**Check the weapon reader first.** `crossbot_weapons` prints `walked active id N, peeked active id N --
match` per bot, comparing the duel's two-deref `ActiveWeaponId` with the inventory walk's `activeIndex`. Any
`MISMATCH` means the `m_pActiveItem` offset is wrong and every duel verdict is noise. Healthy: a match on all
bots across several weapon ids.

**Duels need the pin, and here the pin is right.** HLDM bots almost never hold the crowbar (worth 0.1, loses
to everything), so `melee=` in `crossbot_combat_status` stays 0. `crossbot_fire_pin weapon_crowbar` makes
every other weapon unusable *in combat*, the cheap way to get melee into enough hands:

```powershell
.\scripts\rcon.ps1 -Port 29015 -Command "crossbot_fire_pin weapon_crowbar"
```

The cost mirrors the ambush case: with everyone on a crowbar the duel *selection* mostly cannot be seen,
since the bot already holds melee and the block skips. Selections still trickle in from bots entering a
fight straight out of the idle rotation holding a glock (the pin does not govern out-of-combat selection).
`duel=` is per life too: sum increments.

A healthy pinned run on crossfire, over about a minute of polling:

- `melee=1` on most bots in combat, falling to 0 within a poll or two of the pin coming off.
- `inarea=1` **much** rarer than `melee=1` (two bots sharing a nav area is the exception on an open map),
  with `duel=1` tracking `inarea=1` exactly, about 20 `duel=1` samples per hundred polls. `inarea=1` never
  firing is worth chasing; `inarea=0` beside `melee=1` is not.
- `charging` far outnumbering `backing off`, and `swinging` possibly absent: the bot fires once inside 32
  units, which drops it straight into the backoff window, so the stand-still branch shows only in the
  ~0.1 s between the 0.4 s backoff and the crowbar's refire.

The movement reasons print on the **`no probe`** line, not the `probe` line: the duel drops the probe when it
takes over. `duel=1` with no `charging`/`backing off`/`swinging` anywhere *is* a bug: the state formed but
`DuelDrive` bailed, most likely because melee never reached the bot's hands.

## scripts/test_monster.ps1

Two phases, each on its own server, for what nothing else sees: a bot fighting something that is not a
player. `-Phase` runs one (default `all`); `-Seconds` sets each phase's length (default 150).

```powershell
.\scripts\test_monster.ps1
.\scripts\test_monster.ps1 -Phase engage -Seconds 300
```

Live monsters come from `crossbot_spawn`, refreshed in front of each bot every few seconds. Bots throwing
them does not work: the snark is reachable only through the ambush path against an unaware opponent, and the
uphill veto refuses the throw almost everywhere on crossfire (`uphill=510`, `throwshots=0` over a minute with
the pin on). The phase equips `weapon_9mmAR`, since a snark has 2 health and dies to one bullet: what is
measured is whether the bot engages, not whether it wins.

| phase | asserts |
| --- | --- |
| `census` | `crossbot_monsters` sees `monster_snark` lines, every one carries `FL_MONSTER`, `mon_hEnemy` resolved to a real offset, and at least one snark names an enemy |
| `engage` | `mine` and `trc`/`hit` advance in `crossbot_sense_status`, a snark becomes a bot's `target=`, and a bot logs `entered combat with monster_snark` |

**Believe `census` before reading `engage`.** From the engage counters alone, a bot ignoring hunting snarks and
a map with no snarks both read `mine:0`. The census is the only thing that tells them apart, and the only
check on the `m_hEnemy` offset: a wrong offset reads `enemy=-1` on every line while all else looks healthy.

`engage` is stochastic like the duel: a thrown snark must find a bot before its detonate timer. Counts are
polled across the window, and `switched`, `stopped` and `died` are reported, not asserted; `stopped` depends
on a snark re-targeting mid-fight on its own two-second hunt clock.

## scripts/test_deploy.ps1

Four phases, each on its own server, plus `crossgame` and `reroute`, which run only when named. `-Phase` runs one;
`-Seconds` sets the bot phases' length (default 150).

```powershell
.\scripts\test_deploy.ps1
```

| phase | what it proves |
| --- | --- |
| `gates` | `crossbot_list_gates` finds chokepoints on crossfire, boot_camp, stalkyard and test2_doorway, and no accepted gate exceeds the 144-unit opening bound. Needs no bots; it catches a broken gate scan on its own |
| `tripmine` | placements happen, **both stances appear**, beams are crawled under, low beams are crouch-jumped and **at most 5% of beam jumps fail** (`beam jump cleared` vs `beam jump FAILED`), no mine is removed for finding no surface, bots still reach goals with beams up, and **no bot walks into its own beam** (`onbeam 1`, not counting deaths with `avoid jump`, which the jump rate covers) and no mine arms around its placer. Sets `crossbot_deploy_debug 1` |
| `standoff` | with the surface pinned to ground on `test2_doorway`, the side search gives up, `EngageStandoff` runs, the mine is destroyed by gunfire, and the bot never sits `blocked by monster_tripmine` |
| `satchel` | satchels are thrown and detonated, no detonation is counted twice, and **none fires on a contact under 16 units** |
| `crossgame` | de_dust2 with `valve` + `cstrike`: an HL bot mines the T-side mid doorway (conn 655) and a CS bot must still reach the goal through it. Asserts the CS bot's jumps meet the same 5% bound. Reproduces the stuck-behind-a-beam bug (`doc/movement.md`); `crossbot_combat 0` stops the two killing each other and ending the round, since the HL bot lands on a random team. Not in `all` |
| `reroute` | the `crossgame` setup with `crossbot_beam_jump 0`: the CS bot must reroute (`avoid rerouting around`) before any shot, never jump, and still reach the goal. Not in `all` |

- **`contact` is the assertion the tripmine phase exists for.** `crossbot_deploy_debug` splits every own-mine
  death by `onbeam`: `1` is the bot walking into its own laser, a bug; `0` is standing in the blast when
  something else tripped it, the game working, never zero. The raw self-kill count would fail forever.
- **Both stances must appear.** A standing placement puts the beam 64 above the floor, a ducked one 30, and
  only the first can be crawled under. One stance means the roll is dead and half the avoidance code untested
  while every counter looks healthy.
- **`jump` must be zero.** A jump sweeps the player box up through the beam plane and trips it every time;
  both `walk`-state deaths in the instrumented baseline were exactly that.
- **The standoff phase needs the pin.** A vertical beam comes only from `crossbot_deploy_surface 3`: ceiling
  and ground mounts are gated behind a 48-unit opening, rare enough that six stock maps unpinned find none,
  and without it the side-search and shoot branches never run. `test2_doorway` has one route per gate, so a
  beam cannot be walked round. It is **smaller than the 375-unit standoff**, so the retreat ends on
  `kBreakRetreatTime`, not distance: the bot fires from as far back as it can and takes some blast. Non-zero
  `killed` means the path ran.
- **The bot is driven on `test2_doorway`.** With no items to gather it idles in its spawn area and never
  stands in a gate, so the phase alternates `crossbot_nav_goal` between the spawn and the `info_target`.
- **The satchel contact bound is the real assertion.** `monster_satchel` plays a bounce sound on landing, and
  a bot that counted its own charge as a contact detonated the instant it landed, reporting a contact about
  2 units away: 181 detonations off 7 satchels with every other counter fine.
- **Placements are stochastic.** A bot mines only gates in the area it stands in, so which get mined and who
  walks back through varies; every assertion is "more than zero". `placed=0` on crossfire is a failure; 9
  where a run once had 14 is not.

## scripts/test_aim_error.ps1

Measures the intentional aim error (the `crossbot_aim_gain` motor gain mismatch; `doc/perception.md`, "The
efference copy is wrong on purpose") by firing the same flick `-Flicks` times (8) through `crossbot_aim_test`
and reporting the spread of settle times and the resting error per `-Gains` value (default `0, 0.15`), per
`-Shapes` flick (default `180 0`, `0 90`), optionally across `-Fovs`.

Read the spread, never a row: the gain is drawn once per flick, so one flick is one draw.

- **At gain 0 the spread must collapse** onto the controller's own numbers (normal tier, crossfire: about
  0.485 s for a 180, 0.34 s for 90 of yaw) and rest error must read **0.000**. Anything else means the error
  model leaked outside its cvar. This is the regression check.
- **At 0.15 the mean must rise by about one reaction time** (normal's 0.30 s), the correction submovement,
  with rest error still 0. A flick that never settles means the correction loop diverged.
- Settle is gated on hand speed as well as angle (`doc/perception.md`, `crossbot_aim_test`), so a slow
  enough overshoot can still slip under it: another reason to read the spread.
- `-Fovs` sets `pev->fov` before each flick. Zoom divides hand speed by `fov / 90`, so settle time must rise
  at fov 20; rest error stays 0 at any fov, since no hand resolution is modelled.
- **`0 90` cannot report a rest error.** A 90° pitch flick from level clamps to `kPitchLimit`, and
  `BeginTest` clamps the target the same way, so view and target are both pinned at 89. Its settle time
  still means something.

## scripts/profile_hlds.ps1

Brings up a dedicated server at `maxplayers 32`, adds `-Bots` (32) over rcon, warms up (`-WarmupSeconds`),
turns the zone profiler on for `-ProfileSeconds`, and optionally runs `AMDuProfCLI collect` against the live
pid. It prints `crossbot_airmove_cache_stats` before stopping, so every profile run is also a cache
measurement.

**It needs a profiler build.** The instrumentation and the `crossbot_perf` cvars are behind
`CROSSBOT_PERF`, off by default. Against a default build the rcon commands come back unknown and no
`CrossBot PERF` lines appear, which looks exactly like a server with nothing to report.

**Profile with `-DCROSSBOT_VERBOSE=OFF` unless you are measuring the logging.** A `SERVER_PRINT` costs
~700 us on the redirected pipe, so a chatty path measures its own logging (`rp_log` was the worst offender
until it was compiled out; `doc/diagnostics.md`, `bot_log.cpp`):

```powershell
& .\build.bat -DCROSSBOT_PERF=ON -DCROSSBOT_VERBOSE=OFF
```

Both values cache in `build/CMakeCache.txt`, so a plain `build.bat` keeps the last setting and `=OFF`/`=ON`
puts them back. Neither prints anything when *missing*: check the CMake output for
`crossbot: zone profiler ENABLED` and `crossbot: verbose bot logging DISABLED`.

- `-NoProfiler` skips uProf and just sleeps, the usual mode: the zone report attributes, uProf only adds the
  module split.
- `-NullStdout` sends stdout to `NUL` and reads readiness and results from `D:\hlds\qconsole.log`. Not a
  convenience: a blocking `WriteFile` on the redirected pipe costs ~700 us per `SERVER_PRINT`, so a run on a
  chatty path must say which it measures.
- `-NoConDebug` drops `-condebug`, which changes almost nothing.
- `-PerfInterval` (10 s) is the report window, `-SpikeMs` (3) the spike threshold.

Output lands in `nav_debug/perf/<map>_<stamp>.log` (the whole console) next to the uProf session:

| line | what it is |
| --- | --- |
| `CrossBot PERF SPIKE` | one frame over `-SpikeMs`, with its worst bot and four heaviest zones |
| `CrossBot PERF <zone>` | per-zone self time for the window, sorted, with calls/frame and worst frame |
| `CrossBot PERF traces` | trace totals for the window |
| `CrossBot PERF  T <zone>` | traces charged to that zone: count, ms each, mean ray length, clip entities |

The `T` table answers "what are all these traces *for*": a trace is charged to whichever zone was running
when it started.

**`ZONE_SN_LOS traces/f` is the regression witness for vision cost.** No PowerShell suite counts tracelines,
so a change that quietly restored per-candidate tracing would pass all of them, slower but not wrong. Vision
is bounded at two `dont_ignore_monsters` traces per bot per frame, so `ZONE_SN_LOS` must read at most about
2.0 per bot per frame, and `ZONE_SN_BONE` must track it. Measured on crossfire at 32 bots: 51.6 traces/frame
over ~30.5 active bots, 1.69 per bot, against a mean `cands` of 6.18 (`doc/perception.md`, "Vision spends at
most two tracelines per bot"). `cands` counts what tracing every candidate would cost, so it is the
comparison without a second build. Normalise by a per-bot-per-frame zone (`sn_damage`'s `calls/f`), not the bot count:
not every bot updates every frame, and dividing by 32 understates the per-bot figure by about 5%.

### What AMD uProf can and cannot tell you about spikes

uProf finds *hot*, never *bursty*, and on Windows the CLI has no path to the latter (checked on uProf 5.3):

- **`report` has no time dimension.** `--time-slice`, `--timeline`, `--time-range` and `--start-time` are
  rejected; `--interval`, `--sort-by`, `--cutoff`, `--view` and `--detail` all aggregate the whole session. A
  150 ms spike in a 60 s run is 0.25% of the samples.
- **OS Runtime Tracing is Linux-only.** `--osrt-event`, `--osrt-threshold` and `--trace osrt` would catch a
  stall by duration, and the Windows CLI answers `Error: Invalid option --trace`. (The strings are present in
  `AMDApplicationViews.dll`, shared with the Linux build; that is not evidence they work here.)
- **Windowing exists only at collect time** (`--start-delay`, `--duration`, `--start-paused`), which needs
  knowing *when* the spike happens.
- **The translated database does carry time buckets**: `uprof_timeline.ps1` reads them.

uProf still cannot see a frame boundary, and its call graph dead-ends in unsymbolised `swds.dll`
(`doc/diagnostics.md`), so engine time never attributes to the plugin call that asked for it.
`crossbot_perf_spike` remains the tool for attributing a frame. What uProf adds is what the zone profiler
cannot see: time the game thread spent **not running at all**.

## scripts/uprof_timeline.ps1

Reads a uProf session as a 1 ms timeline. Needs `duckdb.exe` (the plain Windows CLI from duckdb.org) next to
`AMDuProfCLI.exe`, and says so if it is missing. With no `-Session` it takes the newest
`nav_debug/perf/uprof_*`, translating it if needed; `-MinRunMs` (20), `-MinGapMs` (4) and `-Top` (15) set
what it reports.

```powershell
.\scripts\profile_hlds.ps1 -Map rocket_frenzy -WarmupSeconds 25 -ProfileSeconds 60
.\scripts\uprof_timeline.ps1
```

**Why it works.** `AMDuProfCLI translate -i <session>` writes `cpu.db` and `callstack.db` in **DuckDB**
format, not SQLite (magic `DUCK` at offset 8). `UnifiedSampleSeries` has a **`windowId`** column, indexed as
`timeSeriesIdx`, and `session.uprof` records `"interval": 1` / `"isTimeSeriesData": true`, so `windowId` is
a millisecond index from the start of collection. The GUI timeline reads it; nothing in the CLI does. The
server's game loop is one thread, so one row per `windowId` for the busiest thread is a per-millisecond trace
of the loop; on a saturated 32-bot run that thread has a sample in ~90% of milliseconds.

**Runs and gaps are different bugs**, with opposite fixes:

- **Runs**: consecutive milliseconds in one module. The thread was on-CPU; module and RVA say where the
  compute went.
- **Gaps**: milliseconds with **no sample**. The thread was off-CPU: blocked on I/O, a lock, a page fault.
  Optimising the surrounding code does nothing, and the zone profiler cannot tell these from slow code
  because wall time passes either way.

**What it found, and its limit.** A saturated map often has **no stall to find**: crossfire at 32 bots has no
run over 8 ms, the loop bouncing between two `swds.dll` regions thousands of times a frame, so frame-time
variance is *how many* traces a frame issued. rocket_frenzy's longest run is 37 ms. The gaps earn their keep:
rocket_frenzy showed an 18 ms off-CPU gap right after `NavPlan::PlanRoute`, a 60 ms gap 97 ms later, and a
55 ms gap in `ntdll.dll` elsewhere, a stall cluster the zone report cannot tell from ordinary planning cost.

Before trusting a session:

- **The spike log caps at 8 entries per window**, so the worst frame is often missing. The same rocket_frenzy
  run reported `max 652.565ms` in its `PERF window` line while its largest `PERF SPIKE` was 22.86 ms.
- **A collect whose target died looks like a quiet server.** Check the raw `.prd` size; the script warns
  under 100 KB. A 32-bot `xen_dm` run that hit `FATAL ERROR (shutting down): ED_Alloc: no free edicts` left a
  528-byte `.prd`, an empty database, and no `PERF window` line.

## Airmove cache: correctness and tuning

The cache itself is `doc/path_planning.md`'s. One cvar and one command test it, both in a **stock** build
(neither is behind `CROSSBOT_PERF`):

- **`crossbot_airmove_verify 1`** re-simulates on every hit and compares with the stored result. It costs the
  whole speedup, so it is a correctness harness. On a mismatch the *fresh* result is returned, so verify mode
  is self-correcting. Set it with `-BootCommands`, never `-ServerArgs`. The landing log line carries
  `peak err` and `corrected` (frames), so in-flight correction effort and the miss read together.
- **`crossbot_airmove_cache_stats`**: occupancy/capacity, hits, misses, hit rate, inserts, evictions,
  uncacheable, and under verify the verified and `MISMATCH` counts. `MISMATCH (N ent)` splits out mismatches
  where the fresh sim touched an entity, the known and accepted divergence (`doc/path_planning.md`).

Counters reset on map change. There is **no sizing cvar**: capacity is `kAirmoveCacheSlots` in
`nav/airmove_cache.h`, a compile-time constant since the table is a plain member array, as are the eviction
tunables `kAirmoveAgeStep` and `kAirmoveMaxDecay`.

**Reading the stats.** Occupancy pins at capacity within a minute on any real map. Judge a change on **hit
rate and evictions together**: a policy that raises the hit rate while lowering evictions keeps the right
entries. If `uncacheable` dominates the misses, neither capacity nor eviction is the problem: the map hangs
doors, plats or trains in its flight paths, and those sims are refused by the purity rule.

**Judging a change.** Interleave the A/B on one map in one sitting: absolute frame times on this box drift
±50% over an hour (`sn_los` per-trace cost alone swings that much), so runs twenty minutes apart say nothing.
Capacity is compile-time, so an A/B is two builds; build them back to back and alternate runs:

```powershell
.\scripts\profile_hlds.ps1 -Map crossfire -NoProfiler -WarmupSeconds 20 -ProfileSeconds 60 `
    -PerfInterval 60 -SpikeMs 8
```

Read `PERF window` (frames, avg, p50/p90/p99) and `PERF  T rp_plan` from each log. At 32 bots the server is
saturated (`crossbot` ~98% of wall in every configuration), so freed planning time shows as **more frames and
a shorter tail**, not less trace time. Always check more than crossfire: `xen_dm` and `rocket_frenzy` are the
air-move-heavy maps, and crossfire caches at 94% where xen_dm sits at 56% with the table thrashing.

### BSP tracer at runtime: what a per-ray port gets wrong

The planner traces its own snapshot of the map (`doc/path_planning.md`, "The plan-time world"). Its numbers
there came from a compare harness: both backends on every planner ray, the engine's answer returned, counts of
fraction, solid and entity disagreements, deleted after the measurement. Rebuild it the same way to re-check a
map. A plain port of navgen's `TraceHullWithEntities` (world-only or every blocker, no box cull, translation
only) was measured first and lost, and the reasons are what the snapshot does differently. 16 bots, 60 s,
per-trace cost net of ~0.048 us timer overhead:

| map | blockers | engine | bsp | speedup | traces disagreeing |
| --- | --- | ---: | ---: | ---: | ---: |
| crossfire | off | 0.617us | 0.470us | 1.31x | 0.110% |
| crossfire | **on** | 0.615us | 0.709us | **0.87x** | 0.026% |
| rocket_frenzy | off | 1.346us | 0.549us | 2.45x | 3.763% |
| rocket_frenzy | **on** | 1.373us | 1.071us | **1.28x** | 1.551% |

- **The speedup and the wrongness are the same number.** The engine's cost doubles from crossfire to
  rocket_frenzy while the world-only tracer barely moves: that gap is entity handling, and skipping it is the
  whole win. With entities back crossfire is *slower* than the engine, because `TraceHullWithEntities` does a
  full clipnode descent per blocker with no bbox rejection (fine offline across threads, not in a frame)
  where the engine has a spatial index. Even with a bbox pre-filter and blockers rebuilt from live edicts,
  rocket_frenzy disagrees on 1.55%: translating a brush model by its origin ignores rotation, and
  `ignore_monsters` still lets the engine hit pushables and items.
- **`rp_plan`'s `vs ents 0%` is not evidence for it.** That column excludes `FL_WORLDBRUSH` *and* baked
  blockers, so a trace stopping on a static door reads "no entity" while a world-only tracer sails through
  it: exactly the 3.76%.
- **Per ray the ceiling is small.** `rp_plan` traces were 9.0 ms of 5440 ms of crossbot time on crossfire and
  104.2 of 5644 on rocket_frenzy. What pays is fewer rays (`Coast`) and taking planning off the game thread,
  which needs an engine-free tracer in the first place.

The engine trace ran first on each ray and warmed the cache for the BSP trace, so BSP figures from such a
harness are if anything flattering; time the backends in separate interleaved runs instead.

## scripts/test_directives.ps1 and scripts/test_brain.ps1

Both are described in `doc/brain.md`, "Testing". `test_directives.ps1` needs no keys. `test_brain.ps1` runs
against the local mock in `brain_mock.ps1` unless `-Live` is given, and restores `brain.json` and
`brain_memory.jsonl` afterwards.
