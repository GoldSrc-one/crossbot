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
- Collects the movement trace, then runs `navgen --dump-svg ... --trace ... --no-vantage` to write
  `nav_debug/<map>.svg`, so the failure is inspectable afterwards. A run that never produced a trace
  still draws — an empty or missing trace file draws nothing rather than failing. `--dump-svg` never
  reads the visibility field (only `--dump-vantage-svg` does), so computing one here bought nothing
  and doubled the run's navgen cost. `nav_debug/<map>.nav` is copied from the file the bot actually
  ran on rather than regenerated, so it still has its vantage section.
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

**Every test drives the live `D:\hlds` install, through CrossPlay.** `test_server_start.ps1` launches
`hlds.exe -game crossplay -xgame1 valve -sgame1 valve` (`-GameDir` and `-Games` override both), so the
tests share the install with any real server you run there. What they write is deliberately narrow:
maps into `valve\maps\`, nav and traces under `crossplay\addons\crossbot\`, and a uniquely-named
`-ConfigName` cfg into `crossplay\`. They never touch `server.cfg`, `mapcycle.txt`, `liblist.gam` or
`plugins.ini` — see `doc/crossplay.md`.

Two preflight warnings come out of the same script: the plugin is not listed in
`crossplay\addons\metamod\plugins.ini` (so it will not load at all), or the deployed
`crossbot_mm.dll` is older than `build\crossbot_mm.dll` (so you are testing a stale build). Neither
stops the run; both mean the result is not about the code you just changed.

Bots now belong to a game, so every scripted `crossbot_add` names one: `crossbot_add valve TestBot`.

**The server is invisible, not absent.** `test_server_start.ps1` launches `hlds.exe` with
`-NoNewWindow` and its stdout/stderr redirected. The binary is console-subsystem, so it is created with
no console at all rather than being shown and hidden after the fact: nothing flashes, nothing steals
focus, and there is no window to click into and freeze the server in QuickEdit select mode. Readiness is
the server answering RCON (`echo crossbot_ready` via `rcon.ps1`), not the engine printing
`player server started` into that log: redirected stdout is block-buffered, so that line can sit
unflushed long after the map is live while RCON answers the moment it is.

**The plugin's output is now per-instance.** Every `SERVER_PRINT` — `Repath`, `Replan`, the air-move
launch and landing lines — goes to `<OutputDir>/logs/<map>_server.log` for that run alone. A run that
passes tells you nothing about *why*; that log is where the route, the failures and the penalties are.
`D:\hlds\qconsole.log` still receives everything too (via `-condebug`), but it is shared across concurrent
instances and appended to, so prefer the per-map log and read qconsole by tail if you use it at all.

## scripts/test_nav_goal_all.ps1

The same test over every `maps/*.bsp`, at most `-MaxParallel` at a time. Each map runs as its own
PowerShell child process, so its verdict is a real process exit code and its whole transcript lands in
`nav_debug/logs/<map>.log`. Summary prints map, PASS/FAIL, elapsed and reason; exit 1 if anything failed.

**`-Overhead` (default 120s) is the child's budget outside the goal poll**, and the kill deadline is
`-Timeout` plus it. It covers the navgen run that builds the `.nav`, HLDS boot and the navgen run that
draws the SVG. It exists because a child killed here reports FAIL no matter what the bot did: the two
skybox maps used to blow the old fixed 90s budget in navgen alone and were reported as failures whose
transcripts said `RESULT: PASS`. When a killed child had already recorded a verdict, the summary now
prints it — `killed at deadline (180s) after: goal reached, ...` is a harness problem, not a nav one.

**Run it under `pwsh` 7, not Windows PowerShell 5.1.** Because the verdict is the child's exit code
rather than anything the child printed, a broken `Start-Process -PassThru` handle fails every map at
once — under 5.1 the sweep reports **0/45** with a null `StartTime`, a nonsensical negative elapsed
(around -63,922,828,000s, which is `Get-Date` minus nothing), and every per-map reason still reading
"goal reached". That combination — total failure, absurd elapsed, transcripts that all say PASS — is
the shell, not the bots. Confirm with a single `test_nav_goal.ps1` run before believing a regression.

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

**Plugin cvars must go through `-BootCommands`, not `-ServerArgs`.** `-BootCommands "crossbot_sense 0"`
(forwarded to each child, and available on `test_nav_goal.ps1` alone) appends to the generated cfg that
runs once the level is active. The obvious `-ServerArgs "+crossbot_sense","0"` **silently does nothing**:
the engine parses the command line before `GameDLLInit` registers the cvar, so the cvar's own default
overwrites it and the sweep runs at `1` while appearing to run at `0`. Confirm any such run took by
checking `sense=` in `crossbot_sense_status` rather than trusting the flag. Engine cvars like
`sys_ticrate` already exist at parse time and are unaffected.

Killing a child does not run its cleanup, so the runner tracks the ports it owns and sweeps for
surviving `hlds.exe` at the end, matching on `+port` in the command line. It never mass-kills
`hlds.exe`: a server you started by hand on the default 27015 is outside its range and is left alone.

## scripts/common.ps1

Shared helpers, dot-sourced by every driver: `Invoke-CbRcon` / `Invoke-CbRconQuiet` (the one rcon
call shape, returning text or discarding it), `Get-CbDistance3D`, `Copy-CbMap` + `Ensure-CbNav`
(map copy and conditional `-Force` navgen), `Get-CbServerPid` (`EXITED` check plus pid parse),
`Assert-Cb` / `Measure-CbCount` / `Invoke-CbPhase` (the deploy/monster phase shape, with `-LogPrefix`
and `-ConfigPrefix` telling the two suites apart). `test_nav_goal_all.ps1` keeps its own
`Stop-ServerOnPort`: that kills by port range after a child is shot, which is a different job from
`test_server_stop.ps1`'s kill-by-pid, and `profile_hlds.ps1` keeps its own launch (32 slots,
`-NoConDebug` / `-NullStdout`).

Kept, not cut: `nav_spawn_test.ps1` / `nav_target_test.ps1` stay as the only drivers that assert on
the `reached goal` log line (see `doc/bot.md`), and the single-use flags (`-GatherDebug`,
`-FunDebug`, `-NoGather`, `-SkipRoam`, `-Full`) stay — each gates one rcon line, and deleting the
flag just moves the same line into a chat message.

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

**`pool_party` is the map for healing volumes**, and the only one in `D:\hlds\valve\maps` that has one:

    pwsh -File scripts/test_items.ps1 -Map D:\hlds\valve\maps\pool_party.bsp -Bots 5 -Seconds 180 -GatherDebug

Five bots, because nothing selects a pool until something has hurt it — a lone bot stays at 100 and
every scan line reads `no need`. The cycle to look for is `reached trigger_hurt at 66, wading in`
followed by `topped up at trigger_hurt after 1.7s, now at 100`. `is already full at trigger_hurt` on
a bot that arrived hurt is the failure mode, not an outcome: it means the bot stopped somewhere the
engine does not count as inside the brush. `crossbot_list_items` should show `kind 6` with a comma-
separated list of areas, and the map's *other* `trigger_hurt` — the `dmg 200` one — must not appear.

`-Bots N` is how the contested cases get tested. One bot never races anyone, so case C (seeing from a
distance that an item is already taken) needs at least two. Three on `test56_recharger` also kill each
other, which is the only way `weaponbox` handling and the respawn `fullMemo` clear get exercised at all.

Run `crossbot_list_items` before believing anything else on a new map: it prints every item entity with
the area it resolved to, its `frame` and its `solid`. **An item reading `area none` is invisible to the
bot**, and on `test56_recharger` the health charger and both batteries do exactly that — navgen prunes
17 of that map's 20 areas as unreachable from the single spawn, so the upper floor has no mesh. That is
a map/mesh gap, not a gather bug, and no amount of staring at the bot will show it.

## scripts/test_fun.ps1

The `test_items.ps1` of the fun goal tier, and built the same way: no pass/fail verdict, a decision
log to read. It runs navgen, boots HLDS, prints `crossbot_list_fun`, adds bots, polls
`crossbot_nav_status` every 10 s, prints `crossbot_list_fun` again at the end, and tails the
decisions out of `nav_debug/logs/<map>_fun.log`.

    pwsh -File scripts/test_fun.ps1 -Map D:\hlds\valve\maps\stalkyard.bsp -Bots 3 -Seconds 120

`-FunDebug` sets `crossbot_fun_debug 1`; `-NoGather` sets `crossbot_gather 0`, which strips the
detour lines out and leaves only the tier's own decisions.

**Read the inventory before the behaviour.** The first `crossbot_list_fun` is a per-map assertion,
and it catches chain-analysis bugs that the bots would only show you as vague listlessness:

| map | must list | must **not** list |
| --- | --- | --- |
| crossfire | 2 tank controls → `func_tankmortar`; the `trigger_multiple` at `(0,-2219,-1846)` | the 4 `lift*` buttons, the 4 inert tower buttons |
| doublecross | 2 tanks; the `fire_button_texture` button | the fan-lift and access-panel buttons |
| stalkyard | 2 tanks (`biggun`, `biggun2`) | `armorall`, the `lift_1` button |
| undertow | the `wait 142` button at `(1640,152,184)` | the `bunker_door` button |
| snark_pit | the `wait 120` snark button at `(304,-640,48)` | the four door/lift buttons |
| datacore | nothing — `0 fun entities` | — |
| subtransit | one `tram` row for the `func_tracktrain`, 19 board areas | the two `func_rot_button`s (their chain ends at `pipedoor`) |

crossfire's trigger is also the reachability assertion: it sits behind `strike_ready_door`, which
opens 180 s into the map, so before t=180 it must read `no nav area` and must not be offered as a
candidate.

subtransit's assertion is that the board points are a property of the **track**, not of the tram: the
19 areas and the second `crossbot_list_fun` must be identical whether the tram is parked at
`speed 0` or circling at `speed 400`. The row also prints `deck (-608.0,-896.0,-167.0)` while the
tram sits at `(-544,-896,-200)` — the 64-unit offset is the brush model's offset from its origin and
is correct; a deck equal to the origin means `DeckAt` has been bypassed.

**A healthy run is a cycle**, and it takes more than one bot to see all of it:

```
Player1 wants fun func_tankcontrols ... 7 candidates
Player1 detours to item_healthkit ... 18.3s left after
Player1 detours to ammo_rpgclip   ... 18.0s left after
Player1 reached the tank func_tankcontrols, going to use it
Player1 mounted the tank func_tankmortar
Player1 is bored of the tank, nobody around for 10.0s, readiness 0.45
```

The other legitimate exit is `is shot from behind the tank by X, bailing out`, and it must be
followed by the bot entering combat rather than re-selecting a goal. **A lone bot never sees an
enemy, so it hits boredom every time and the damage exit is never exercised** — use at least three.

Three failure signatures are worth naming, because each looks like something else:

- **A mount immediately followed by a dismount with no reason logged** is the bot moving and breaking
  the 30-unit `m_vecControllerUsePos` leash. It is not an aiming or a gamedata problem.
- **A fresh `wants` line where a `detours to` should be** is broken target stickiness — run it again
  with `-Bots 1`, where combat cannot be the explanation, and the `wants`/`detour` alternation is
  unambiguous.
- **`could not aim at` or `got nothing out of` at a tank, with every gamedata read correct** is the
  `UTIL_ClampVectorToBox` trap (`doc/bot.md`, `bot_fun.cpp`). Small brushes hide it entirely, so
  undertow passing proves nothing about stalkyard.

### subtransit, the tram

Two transcripts have to appear over a long enough run — the first once, the second repeatedly. Start
it:

```
Player2 boarded the tram
Player2 is driving the tram, speed 400
Player2 is riding the tram at 400
Player2 is bored of the tram, nobody around for 10.0s, readiness 0.20
Player2 jumped off the tram onto area 403
```

and catch it, which is only possible after somebody has started it:

```
Player2 reached the track, tram due in 1.9s, waiting
Player2 boarded the tram
Player2 is riding the tram at 400
```

The negative branch is the readiness ban, and it should be the **most common** line in the log —
a ~25 s lap against a 3 s catch window means most arrivals are too early:

```
Player1 reached the track, tram not due for 5.8s, leaving it until readiness beats 0.35
```

Four failure signatures, each pointing at one mechanism:

- **`boarded the tram` then `riding the tram at 0`** — the mount never took, and after
  `kTrainDriveTimeout` the bot gave up and rode a stationary tram. Two causes, both silent: the bot
  is standing on the deck *edge* so `OnControls` fails its box test on the origin, or the bot is
  firing and `m_flNextAttack` is keeping `PlayerUse` out of reach. Neither logs anything by itself.
- **`could not get on the tram`, tram parked** — the board point is unreachable, or the hop is not
  clearing the deck lip. Check the board-point dump for a stand z far from the deck z.
- **`could not get past func_tracktrain`** — a board point is inside the parked tram's footprint. The
  probe offsets must clear the tram's half-width *plus the bot's hull*, not the deck centre.
- **The board list loses the areas nearest the parked tram, and nobody ever starts it.** The tram
  becomes completely inert with no failure logged anywhere, because with no board point at arc 0 no
  bot can reach it and the catch path needs somebody to have started it first. Both known causes are
  in `BuildBoardPoints`: the ground probe starting above the area point and hitting the tunnel
  ceiling (`fStartSolid`), or `point.z` being moved to the traced floor, which drops the air-route
  trace to deck level where the trench lip blocks it. Diff the board-area list against the 19 in the
  table above before looking anywhere else.
- **`hops for the tram` never appears while `due in` does** — the bot reaches the track, waits, and
  never commits. The jump window is `[clearFrom, clearTo]` against `eta`; if `eta` has anything
  subtracted from it (a deck-lead term, say) it sits permanently below the window.

**Death accounting.** With `-FunDebug`, every bot death logs `died at (x,y,z), D from the tram doing
S, onground N, tram phase P`. That is the only way to tell the three populations apart, and they need
different fixes:

```bash
grep -aoE '[0-9]+ from the tram' nav_debug/logs/subtransit_fun.log | awk '$1<=20' | wc -l
```

Deaths at `D <= 20` are the tram; everything further out is ordinary combat and moves a lot run to
run, so total death count is a useless metric here. Of the tram deaths, `tram phase wait` means the
bot died attempting a catch — that is the hop's fault and should be zero. `tram phase not engaged`
means a bot was run over while walking, which the hop logic cannot help: once started the tram
circles forever and the mesh knows nothing about it. On a 300 s four-bot run expect roughly 7 of the
latter and none of the former.

A bot occasionally dying under the tram is expected, not a bug: once started it circles forever and
the trench is lethal on a ~25 s cycle, and nothing in the nav mesh knows that.

### The switch-priority rule cannot be tested on a stock map

`BotFun::Prioritize` drops tank and tram candidates whenever a switch is available, but **no stock map
can exercise it**. crossfire and doublecross are the only ones carrying a switch alongside tanks, and
on both the switch prices as unreachable and never survives `PruneUnreachable` — so the fun tier only
ever holds one kind, and the filter is a no-op everywhere.

What can be checked is that each kind still selects on its own: undertow must log `wants fun
func_button` and then `pressed func_button ... leaving it for 142s`; stalkyard must still mount tanks;
subtransit must still board the tram. If the filter ever does bite, `crossbot_fun_debug` prints
`fun priority kept N switch candidates, dropped M others` — the absence of that line on stock maps is
expected and is not evidence of a bug.

`undertow` is the map for the long-`wait` switch: the bot presses once, logs `leaving it for 142s`,
and the tier must then stay empty for the rest of the run. `datacore` is the negative control — the
tier must be empty and gather behaviour untouched. `subtransit` is the only map with a `tram`, so
`stalkyard` and `crossfire` double as the assertion that the tram branch is unreachable everywhere
else.

**Regression.** `test_nav_goal.ps1` sets `crossbot_fun 0` alongside `crossbot_gather 0`, so the
51-map sweep is unaffected by the tier and stays comparable against transcripts taken before it
existed. All 51 maps pass. `test54_window` took two fixes. It used to abandon the goal because navgen
could not route to it at all (`no route from area 1 to area 18`) — that was the launch rescale, see
"Rescaling a missed launch" in `doc/nav.md`. It then failed a second time, ping-ponging between two
routes on the window ledge: once the sweep moved onto the CrossPlay server (`D:\hlds\crossplay`, which
sets no `sv_maxspeed`) the planner designed launches at the engine default 320 while the valve DLL held
the bot at 270, so every launch was refused for being 90% of design. `Bot_MaxSpeed` now reads the cap
the engine settled on, in `PM_Move_Post` — see `doc/bot.md`. A map that plans a route but will not
follow it is the failure mode this sweep exists to catch; navgen alone called it healthy both times. `test19_water` also used to abandon, and `test39_skybox` and `test41_skydrop`
used to hit the deadline; the skybox pair was the sky prune (see `DropSkyFloorLeaves` in
`doc/navgen.md`), not the follower.

**Perf.** Measured on crossfire, 8 bots, `-DCROSSBOT_PERF=ON`: `item_select` self time is
0.0288 ms/f with the tier on against 0.0264 ms/f with `crossbot_fun 0`, which is inside the
run-to-run spread — the cached `toTargetCosts` is what buys that, since a repath costs one
`AreaCosts` rather than two. The worst *frame* does move, 1.6–2.9 ms to 3.1–6.8 ms, and that is the
retarget: one extra `AreaCostsToGoal` plus the hull-3 reachability traces (0.19–1.01 traces/frame).
Amortised cost nil, peak selection frame roughly doubled.

The tram adds nothing measurable on top of that. Its track polyline and board points are built once
per map on the first `Collect` that has a mesh; per selection it contributes one cached lookup, and
per waiting frame one `ProjectOnTrack` over 43 nodes. subtransit is the only map that pays any of it.

## scripts/test_sense.ps1

Drives the sense system and the aim delay queue, which nothing else can. `test_nav_goal.ps1` spawns
exactly one bot, and vision is line-of-sight gated, so a lone bot never has a target and hears almost
nothing — the nav sweep can only ever prove the system is inert when `crossbot_sense` is off.

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

The **static** phase is the sharp test: a stationary target must be tracked consistently at *every* delay,
because extrapolating a still target forward and reading it back cancels perfectly. A `served` that does
not match the requested delay is a real defect, and so is an `err` that **trends** with the delay.

`err` no longer reads 0.00 there, and the old "any nonzero err is a defect" rule is gone with the gaze
scaffold. The bot now aims at a randomly chosen hitbox bone while `err` is measured against the target's
eye, so the bone spread is a standing floor. It is a floor that grows as the range closes, and the script
converges the bots to point-blank to hold line of sight, so there it is tens of degrees and almost pure
negative pitch (every bone sits below the eye). That floor is the bone sampling working. Read `served`
here, and read `err` only for its shape across the delay sweep, never for its value.

The **roam** phase cannot be read that way and the script says so. `err` there has been seen anywhere
between 2 and 48 degrees, mixing genuine target acceleration with the gate suppression pointing the bot at
a ladder instead, and no run is comparable to the last. Watch `queue` and `served` in that phase, not
`err`.

Both `crossbot_aim_status` and `crossbot_sense_status` carry `leadblk`, a per-bot count of frames where the
queue write was vetoed because the target's extrapolated lead point had already crossed into geometry. It is
a cumulative count rather than a current-frame flag precisely so a two-second poll can see it: the veto
lasts one frame and a flag would almost always be sampled low. It sits on the aim line as well as the sense
line because the static phase polls only the former.

The two phases say different things about it. In the **static** phase `leadblk` must not move at all across
the polls — the bots are motionless, so there is no lead, so the trace is skipped and the phase is identical
to a build without the feature. It will already be non-zero on arrival, banked during the walk to the goal;
what matters is that it is flat. Any increase there is a bug in the skip, not a behaviour change. In the
**roam** phase it must climb. Bots chasing items on `crossfire` cross doorways and corners constantly, and a
`leadblk` that stays put through a whole roam phase means the trace is wrong — flags, direction, or the skip
swallowing every case — not that the map was unusually open.

The **suspicion** phase reports which senses actually fired, asserts that the player tier always wins the
look vote, and asserts that heard entries age out. `SIGHT` and `SOUND` are the two it can rely on;
`EVENT` and `DAMAGE` do turn up from weapon deploys and fall damage, but `scripts/test_fire.ps1` is where
all four are exercised deliberately.

Sources are read from the `pushed=` counters, never by sampling `fresh=`. Since the list holds one entry
per entity, a player's `SIGHT` entry overwrites their own `SOUND` and `DAMAGE` entries within a single
`BotSense::Update`, so `fresh=` is a poor witness for anything but `SIGHT`.

Sound there now comes mostly from **footsteps**, plus item pickups, doors and chargers. Footsteps dominate
by a wide margin: two roaming bots on `crossfire` produce around 85 step sounds in 25 seconds, against 64
sounds of every other kind over a comparable run. They do not arrive through `pfnEmitSound` -- pmove emits
them through `pmove->PM_PlaySound`, which is hooked separately via `pfnPM_Move` (see `doc/bot.md`). If they
ever seem to stop, check `sv_footsteps` before suspecting the hook, and remember that a player moving at or
below 220 units/s emits none at all, by engine design.

Do not force the quiet period with `crossbot_kill`. Respawn resets the bot with `CBot()`, which empties the
suspicion list whether or not the decay works -- the assertion would pass unconditionally. Nor should you
assert that no `SOUND` entry survives. The invariant that actually holds is that no live entry may be older
than `crossbot_sense_decay`, so the test shrinks the window to 1s and checks the age column instead.

That phase **keeps gathering on**, which it did not used to. With one suspicion entry per entity, an idle
map really does go silent: `susp=` sits at `0/256` and the age assertion then passes against an empty list,
which is worse than not running it at all -- it looks like a green check and proves nothing. Moving bots
keep emitting footsteps, so there is always something live to measure, and the test now fails outright if
it never observed a single live entry.

The same trap is worth generalising: **an assertion over a set that can legitimately be empty needs to
assert the set is non-empty too.** This one survived a full green run before anyone noticed.

Neither aim phase exercises the extrapolation, because a still target has no velocity to extrapolate and
`crossfire` has no stretch long enough to hold a constant one. That check is manual: join the server,
let a bot lock on, and strafe at constant speed — its head should stay *glued* to you, not trail you.
Reverse sharply and it should overshoot and correct. Step behind a wall and it should keep tracking for
about a delay before stopping; step back out and nothing should move for about a delay before the flick.

Do that wall check at walking pace, not a sprint. A slow target leads by a few units, so the queue keeps
being written right up to the moment it vanishes and the drain above is what you see. A *sprinting* target
leads by ~80 units at the default delay, so the lead is vetoed while the target is still in the open and
the head parks at the corner rather than swinging into the wall behind it. Both are correct; they are
different checks. The sprint one is the only test of the lead trace's actual purpose and no script makes it.

Joining as a human is another way to exercise `EVENT` and `DAMAGE` by hand: fire a weapon near a bot that
cannot see you and its head should turn toward the shot, and shoot one in the back and it should turn
around.

## scripts/test_fire.ps1

Drives the hitscan firing loop, which is the whole chain at once — sense, aim queue, weapon choice, the
fire ray, the button. It does the **opposite** of `test_sense.ps1` and that is the point: converging
bots onto one spawn is precisely the arrangement firing cannot be measured in, because converged bots
land on the same origin and vision drops them for having no eye-to-eye vector at all. So this script
leaves `crossbot_gather 1` and lets bots roam, collect guns and meet at real ranges.

Encounters are stochastic. More bots is the only reliable lever — six on `crossfire` produce a fight
inside a minute where two can take several — so the pass condition is "shots were fired at real
targets", never a count. `-Full` runs the whole duel window instead of stopping at the first confirmed
engagement, which is what you want when the interesting numbers are deaths, reloads and the rarer
suspicion sources.

**The select phase runs first, with one bot and nothing to shoot at.** It is the only arrangement in
the suite where the out-of-combat weapon decision runs unopposed — once four bots roam `crossfire`, a
three-second gap without a sighting is the exception — and the only one where the two stages of
selection can be taken apart, because nothing gates the candidate list before ranking sees it.

It runs them in order, and each assertion is on **where the bot ends up**, not on how it got there.
Part A hands over four weapons spanning three score bands (crossbow 0.5, 9mmAR 0.3, shotgun and
hornetgun 0.2) and requires the held weapon to be at the top score present. Part B then hands over a
gauss, which ties the crossbow at 0.5 and so cannot be separated by score — but its carry clock starts
at zero, so rarity must move the bot onto the least-carried weapon of that score. Part B is the one
assertion in the suite that fails outright if carry time is not being accrued.

**Neither part may name the weapon it expects**, and the first version that did was wrong within two
runs: the bot roams while the phase runs, and an egon picked up off the floor ties the crossbow at 0.5
and legitimately beats it on carry. The assertions read `crossbot_weapon_carry` and check the *rule*
against whatever the bot is actually holding — top score present, then least carried among that score.
They also retry for a few seconds before failing, because the idle reload pass outranks the score
filter by design and parks the bot on a part-empty gun for a second or two.

Churn is still budgeted rather than forbidden: two settles are expected, and a weapon picked up with an
empty clip (the RPG always has one) makes going to load it a legitimate third. The phase also asserts
zero shots, since a bot with no target that fires anything has a trigger running off the target path.

Under the old least-used rule this phase could only assert a negative — that a bot with everything tied
at zero did not churn — because ranking needed combat seconds it had no way to accrue. Score and carry
are both available with nobody else on the map, which is what turned it into a positive test.

What still needs the duel and `-Full` is `idlereload` (a magazine topped up between fights) and the
carry *spread*: at least two weapons whose carry times differ, which is what shows the tally tracks
acquisition rather than running as one uniform clock. `crossbot_weapon_carry` is what that reads.

**The control phase is the load-bearing half.** With `crossbot_fire 0` nothing may fire, and it is
checked two independent ways: the plugin's own shot counter, and ammo the bots actually spent, read
from clip and reserve counts. Ammo is the honest witness — a clip only ever falls by firing, since
reloading raises it, and clipless weapons spend their reserve directly — and it is what catches a shot
counter that has quietly become a no-op.

Two things the control phase has already caught, both worth not reintroducing:

- **`crossbot_fire 0` has to be set before the bots are added, not after.** Several bots spawn inside
  each other's line of sight and the few milliseconds an rcon round trip takes is long enough for the
  default `crossbot_fire 1` to get shots away.
- **A shot is not "the attack timer became busy".** Deploying a weapon and `GetNextBestWeapon`'s
  empty-weapon switch arm `m_flNextPrimaryAttack` too, so an ungated counter reads several shots per
  weapon pickup with firing switched off entirely. The counter in `bot_fire.cpp` is gated on the bot
  having actually pressed the frame before.

Numbers from a passing 6-bot, 300s `-Full` run on `crossfire`, as an order-of-magnitude reference and
not a threshold: ~1000 shots, ~420 rounds burned, 36 firing decisions, 83 target acquisitions, 47
deaths, 43 weapon switches, 22 reloads, and all four suspicion sources. The `reasons` histogram is the
most useful line in the output — `off-target` and `blocked` both appearing is what shows the fire ray
is actually rejecting shots rather than waving everything through.

There are **seven** weapon phases now and the whole `-Full` run takes comfortably over ten minutes, so
run it in the background rather than waiting on it. `-ProjectileSeconds` defaults to 420, which is a
minute each; the floor is 15s per phase and a phase that short measures nothing.

A passing 6-bot run with `-DuelSeconds 120 -ProjectileSeconds 420 -Full`, again as a reference and not
a threshold: duel 399 shots / 150 burned / 10 deaths / `stickyshots=119`, with all four suspicion
sources; crossbow 78 shots and 29 zoomed polls; hornetgun 986 shots with 206 led; snark 40 shots and
`uphill=53`; gauss 68 charged shots with no timeout; 9mmAR 48 arc shots, `uphill=0`, `selfveto=5`;
handgrenade 62 throws, `uphill=48`, `selfveto=43`; RPG 50 shots against 50 guides, and `capped=0`
everywhere.

Two numbers in there are worth reading as warnings rather than as targets. The hornetgun's 986 against
the snark's 40 is the range gate doing its job, not an anomaly -- the snark is unusable past 300 units
and most polls in its phase catch it with `nothing-usable`. And the handgrenade sampled `primed=1` in
that run, one poll away from having failed the assertion that used to depend on it.

`guides` matching `shots` exactly is the healthiest single number in that output, because the two are
independent observations — one counts a commit the plugin opened, the other counts the game DLL's
attack timer moving.

**The sticky trigger is asserted in the duel, not in a pinned phase, and that is the cheap answer
rather than the lazy one.** Every bot spawns with a glock and both its modes are `WPN_Q_STICKY`, so the
path is exercised by the duel that already runs; a dedicated pin would cost another minute of runtime
to prove the same thing. `stickyshots` is the subset of `shots` that only happened because the trigger
stayed down off-target, counted off the press flag exactly as `leadshots` is, so `stickyshots > 0` is a
cumulative witness and not a sampled `why=` string.

Two things that phase cannot show, both by construction. `stickyshots` **never counts a gauss primary
shot**, because `CGauss::PrimaryAttack` moves no weapon timer for the `fired` edge to see -- pinning
this assertion to the gauss would fail against a working bot. And `burned` rises against the earlier
reference, because holding the trigger through off-target frames spends ammo on purpose; that is the
feature, and the assertions here are existence-based so nothing else moves.

**The weapon phases need `crossbot_fire_pin`, and equipping instead does not work.** In a fight a bot
re-selects only when the weapon in its hands stops working, so handing a roaming bot a crossbow puts it
in the inventory and nothing else happens while a target is up — a first attempt gave 288 crossbows and
hornetguns away across two minutes and recorded zero projectiles fired. Out-of-combat rotation loosened
this and score-then-rarity has tightened it again. Least-used rotation at least moved the held weapon
around on its 3 s tick, a fair coin among everything carried; score does not drift, so a bot that has
found a 0.5 weapon settles on it and holds it for the rest of the phase. Handing a bot the weapon a
phase wants is now actively counter-productive unless that weapon happens to be its best-scoring one.
The pin makes every other weapon unusable, which is still the only lever that names one.

**The gauss needs the mode argument as well, for the same reason one level down.** Selection carries
`pendingMode` through the switch, so a secondary *can* be chosen — but the mode is now drawn at random
once the weapon has won, and the draw is not repeated while the keep-loop is happy with what it has.
A pinned gauss whose primary works therefore has a coin-flip chance of reaching `FIRE_CHARGE` at the
one select that opens the phase, and no further chances after it. `crossbot_fire_pin weapon_gauss
secondary` remains the only way to make the phase deterministic. The hornetgun reaches its secondary without help,
by range rather than by tally: its primary's flight-time ceiling rejects targets past 900 units.

Each weapon runs its own pass so the aim kinds are attributable to the weapon that produced them, and
each pass asserts what only that weapon can show:

| pin | what it proves |
|---|---|
| `weapon_crossbow` | `zoomed > 0` and `aim=target` — the zoom decision fired and `primaryZoomed` was actually selected, which is the only thing that reaches `FireSniperBolt` |
| `weapon_hornetgun` | `aim=homing` present and `leadshots < shots` — equality would mean the cone never replaced the solve |
| `weapon_snark` | `aim=homing`, `leadshots == 0` (a thrown monster is never led) and `uphill > 0` |
| `weapon_gauss secondary` | `chargeshots > 0`, `why=charged` seen, and **no** `charge-timeout` — the bot reached a full charge and waited for a shot rather than releasing on a clock |
| `weapon_9mmAR secondary` | `aim=arc`, `arcshots == shots`, a refusal, and `uphill == 0` — the contact grenade is the only *fixed-speed* parabola in the game, and the only weapon that must stay exempt from the uphill veto |
| `weapon_handgrenade` | `throwshots > 0`, `aim=arc`, `why=primed` seen, and a refusal — the coupled solve's only witness, and `primed` is what proves `m_flStartThrow` is being read rather than the throw assumed |
| `weapon_rpg` | `guides > 0`, `why=guiding` seen, **no** `capped`, and **no** `aim=arc` or `aim=lead` — a laser-guided rocket is aimed at the target, because leading it would steer the spot off them |

The MP5 phase equips `ammo_ARgrenades` alongside the weapon and the RPG phase equips `ammo_rpgclip`;
the MP5's own give is clip ammo only and the RPG's default give is one rocket, so without them the
mode is never usable and the phase measures nothing. The handgrenade needs no ammo entity — its own
give is five grenades, and re-equipping every poll refills them.

The two new phases each have a failure mode worth naming before it is seen.

The **handgrenade's reach is about 550 units**, far shorter than any other weapon here, so it is the
phase most dependent on the bots wandering near each other. A phase that reports `throwshots=0` on
crossfire is more likely to be an unlucky sixty seconds than a broken solve; check `nothing-usable`
against `no-target` before believing anything else.

The **RPG phase expects deaths**, and a lot of them. The splash rule is deliberately permissive — it
fires whenever the blast lands nearer the enemy than the bot — so six bots pinned to rocket launchers
in a small arena kill themselves regularly. That is the rule working, not failing; every counter is
banked on death before it resets, so the tallies survive it.

`capped == 0` earns its keep in the handgrenade phase too, and it is the assertion that found the
part-5 tap commit opening on presses the weapon had already refused — six capped commits against 78
throws, from the half second of `m_flNextPrimaryAttack` that `CHandGrenade::WeaponIdle` arms *after*
each throw. Nothing else in the output moved; `throwshots` looked healthy throughout. A counter that
only ever reads zero is easy to mistake for a counter that is not wired up, and this is the run that
showed it was.

`capped == 0` is the assertion that matters there, and it has the same standing as the gauss's
`charge-timeout`: every guide window must close on the rocket rather than on the six-second valve. It
was first written as `guides <= shots` and that was the wrong diagnosis — the first run reported 56
windows against 50 shots, which is not a leak at all but presses that launched nothing, because
`CRpg::Reload` pushes the weapon's own attack timer forward while the reload button is held. A leak
makes a bot *stop*, so it would show as too few shots, not too many windows.

**Every weapon phase respawns its bots first, and that is not tidiness.** The pin reaches weapon
*selection*, and selection alone does not isolate a phase: a bot carrying a gauss charge keeps its
mode without ever consulting `ModeUsable`, deliberately, so a charge started in the gauss phase
survives the pin change and discharges into the next phase's counters. The first run of the MP5 phase
failed on exactly that — six charge shots in a phase where the gauss was unusable. `crossbot_kill`
drops the inventory, the charge and every per-life counter at once.

`off-target` must appear in the **crossbow** phase — an aim gate that never withholds a shot is an aim
gate that is not running. It is asserted there and nowhere else, for two different reasons. The gauss
cannot show it at all, because `ChargeButtons` overwrites `lastReason` with `charging`/`charged` on
every frame the ray test fails. Everywhere else it is a reason string and therefore a sampled one.

**That premise expired, exactly as this paragraph warned it might.** The crossbow's `off-target` check
survived on the reason string only because a target existed on most frames; when vision dropped to two
tracelines per bot (2026-09-03) the fraction of target-holding frames fell to about 27%, and the
two-second poll started missing both `off-target` and `aim=target` — two consecutive runs failed on
*different* halves of the pair, each having observed the other. The behaviour was never broken: the
cumulative counters added with that change read `aimveto=44 targetaims=46` on the very run whose reason
histogram caught `off-target` exactly once. Both crossbow assertions are now carried by
`aimveto`/`targetaims` and merely widened by the reason string, matching the arc phases below.

The general rule, twice learned: **an assertion carried by a sampled reason string is only as good as
the duty cycle of the state it samples, and that duty cycle is not a property of the test.** Any change
to how often bots hold targets silently re-tunes every such assertion in the suite.

**No blocking assertion may rest on a `why=` or `aim=` string, with exactly one documented
exception.** Both fields are one frame sampled every two seconds, so absence proves nothing. This has
now cost three separate runs -- the MP5's refusal check, then the handgrenade's, then `aim=homing` on
the snark, which failed against a bot that had thrown 48 snarks with `uphill=54` and `aimveto=6` in
the same phase. Every one of them had a cumulative counter sitting beside it that said the same thing:

| was asserted as | now asserted as | why it is the same claim |
|---|---|---|
| `aim=homing` (snark) | `shots > 0` and `leadshots == 0` | a solved-lead press moves `leadshots`, so zero of them across real shots *is* "the cone replaced the solve" |
| `aim=homing` (hornetgun) | `shots > 0` and `leadshots < shots` | as above |
| `aim=arc`, `why=primed` (grenade) | `throwshots > 0` | a throw is only reachable through `ArcOnTarget`, and it counts the falling edge of `m_flStartThrow` |
| `aim=arc` (9mmAR) | `shots > 0` and `arcshots == shots` | — |
| `why=guiding` (RPG) | `guides > 0` and `capped == 0` | the commit opened, and closed on the rocket rather than the valve |

Two of those needed a `shots > 0` clause added: `leadshots == 0` and `arcshots == shots` both pass
happily on a phase that fired nothing, so on their own they were weaker than the flaky string they
replaced.

The exception is **`why=charged` on the gauss**, which stays. It is the only assertion in the file
with no cumulative equivalent — `chargeshots` proves a charge completed, but only the reason string
proves the bot *stood there holding a full charge waiting for a shot* rather than releasing on a
clock, which is the whole design. It samples reliably because a charged gauss holds that state for
seconds at a time; it has read 27, 29 and 30 across three runs.

Negative string checks are unaffected and stay: `why=charge-timeout` and the RPG's `aim=arc`/`aim=lead`
fail *when present*, so sampling can only make them miss a real fault, never invent one.

**The two arc phases assert the refusal on `selfveto`, not on the reason string**, and that is a
correction rather than a preference. `why=` is one frame sampled every two seconds, so `arc-self` and
`arc-blocked` are usually true and almost never *observed*: the first run of the handgrenade phase
showed `selfveto=22` and not one arc reason in the histogram, and the MP5 phase — whose assertion had
been reason-only since it was written — would have failed the same run for the same reason. The
counters are cumulative and survive the poll; the reason strings widen the test rather than carrying
it.

`uphill`, `selfveto` and `zoomed` are counted as poll-level witnesses rather than event counts:
`uphill` and `selfveto` are per-frame-per-mode rejection tallies, `zoomed` is polls in which some bot
was zoomed. All three answer "did this ever happen", which is all a stochastic phase can honestly ask.

The snark's `uphill > 0` and the MP5's `uphill == 0` are the two halves of one guard on the uphill
veto's scope. The veto exempts arcs that **detonate on impact** — the sim knows where those end up —
and not arcs in general, because a handgrenade lands and then rolls for the rest of its fuse. So the
snark, homing and never solved, must keep tripping it; the MP5's contact grenade must never trip it;
and the handgrenade trips it freely, which is reported but not asserted because it depends on the
terrain the bots happen to be standing on.

`reloads` is counted in the plugin rather than scraped, because `why=reloading` is true for a single
frame and no 2-second poll would ever see it. Reload is the behaviour the SDK reading says is easiest
to get wrong — hold the trigger through an empty clip and the weapon clicks forever — so it needs a
witness that does not depend on sampling luck.

## Combat state

There is no dedicated script. `BotCombat` needs several bots fighting on a real map, which is what
`test_fire.ps1` already arranges -- but the transitions are easier to read straight out of a server
log than through that script's assertions:

```powershell
$log = "D:\hlds\logs\combat.log"
.\scripts\test_server_start.ps1 -Map crossfire -Port 29015 -LogFile $log -Commands @(
  "crossbot_combat_debug 1",
  "crossbot_add valve bot1","crossbot_add valve bot2","crossbot_add valve bot3","crossbot_add valve bot4")
```

Two minutes of that is enough for every rule to fire at least once. What to check in the log:

- **Every exit reason appears.** `the enemy died`, `respawned`, `lost every trace of the enemy` and
  `went too long without line of sight` should all be present, and `entered combat` should be within
  a few of `left combat` -- the difference is the fights still running when the server stopped.
- **Both entries appear.** `(sight)` dominates; `(damage)` is rarer and needs someone to be shot from
  outside their own view cone. If `(damage)` never fires, that is worth chasing -- it is the half that
  no amount of roaming guarantees.
- **`walled off` is a minority of probes.** It was 73% before `ResolveAlongRay` existed, which is what
  found that bug; healthy is around 20% on crossfire, with `less exposed` the largest share.
- **Fights end.** The longest `left combat with X after Ns` should be in the teens. A fight that never ends
  is the failure mode this state is most exposed to.

Exposure should trend down across a fight. It is a biased random walk, not a search, so judge it in
aggregate rather than per fight -- on a stock crossfire run roughly three quarters of fights end less
exposed than they began.

`crossbot_combat_status` is the live view: who each bot is fighting, how long for, how long unseen,
whether the movement half has taken over (`driving=`), and the current probe with its exposure
against the exposure where the bot stands.

**`crossbot_vantage 0` is the no-vantage rehearsal.** It makes the blend invalid without regenerating
a `--no-vantage` nav, which drives the probe down the same unscored branch. Bots must still fight and
still move, with every probe reading `no vantage`.

### The awareness flag and the ambush

The flag rides the same run. What to check in the log:

- **Both verdicts appear on sight entries.** `aware=1` only would mean the cone test is inverted or the
  fov fallback is wrong; `aware=0` only would mean `v_angle` is not being read. A stock crossfire run
  splits them roughly evenly.
- **Every `(damage` entry reads `aware=1`.** There is no cone test on that path, so any `aware=0`
  beside `damage` is a straight logic error.
- **All three latch paths fire**: `turned around`, `new opponent`, and `shot at us`. The last is much
  the rarest -- it needs the bot to be hit by the enemy it is already fighting while still unspotted --
  so treat one occurrence as a pass and zero as worth a longer run rather than a bug hunt.

**Crossfire cannot exercise the ambush on its own.** The map holds one `weapon_handgrenade` and two
`weapon_snark` for the whole server, so bots almost never have one in hand at the moment they surprise
someone, and `ambush=` in `crossbot_fire_status` stays 0 through a normal run. That is scarcity, not a
failure. Force it instead:
```powershell
.\scripts\rcon.ps1 -Port 29015 -Command "crossbot_equip weapon_handgrenade"
.\scripts\rcon.ps1 -Port 29015 -Command "crossbot_equip weapon_snark"
```
Re-equip every few polls -- five grenades go quickly -- and watch `ambush=` climb. Unlike the weapon
phases of `test_fire.ps1`, equipping *is* the right lever here and the pin would be wrong: the pin
makes every other weapon unusable, which would destroy the very thing under test, namely that an
ambush weapon is chosen over a working gun the bot would otherwise have kept.

`ambush=` is a per-life counter and resets with the rest of `BotFire::State` on respawn, so sum the
increments across a run rather than reading the last sample -- a tally taken at the end reads 0.

Two things that look like bugs and are not. A bot left **holding a throwable out of combat** is the
idle rotation working as designed: the grenade and the snark are worth 0.1, the same as the glock, so
a freshly picked-up one wins the rarity tiebreak until its carry time catches up. And a good share of
ambushes **declining themselves** is correct -- the uphill veto refuses both throwables at a target
more than 16 units above the eye, which on a map as vertical as crossfire is often.

### The melee duel

Same run, three separate mechanisms, and they need different levers.

**The weapon reader is self-checking.** `crossbot_weapons` prints
`walked active id N, peeked active id N -- match` per bot, comparing the duel's two-deref
`ActiveWeaponId` against the full inventory walk's own `activeIndex`. Any `MISMATCH` means the
`m_pActiveItem` offset is wrong and every duel verdict below is noise. Check this first; a healthy run
matches on all bots across several different weapon ids.

**`crossbot_duel` needs the pin, and here the pin is the right lever.** Left alone, HLDM bots
essentially never hold the crowbar -- it is worth 0.1 and loses to everything they pick up -- so
`melee=` in `crossbot_combat_status` stays 0 and no duel ever forms. `crossbot_fire_pin weapon_crowbar`
makes every other weapon unusable *in combat*, so bots switch to the crowbar as soon as they fight,
which is the only cheap way to get melee into enough hands at once.

```powershell
.\scripts\rcon.ps1 -Port 29015 -Command "crossbot_fire_pin weapon_crowbar"
```

Note what the pin costs, because it is the mirror image of the ambush case above: with *everyone* on a
crowbar the duel **selection** mostly cannot be observed, since the bot is already holding melee and
the block correctly skips. Selections still appear at a trickle, from bots that enter a fight straight
out of the idle rotation holding a glock -- the pin does not govern out-of-combat selection. Sum
`duel=` increments the same way as `ambush=`; it is per-life too.

What a healthy pinned run looks like on crossfire, over roughly a minute of polling:

- `melee=1` on most bots in combat, and falling back to 0 within a poll or two of the pin coming off.
- `inarea=1` **much** rarer than `melee=1`. Two bots sharing a nav area is the exception in a map this
  open, and `duel=1` tracks `inarea=1` exactly. Roughly 20 `duel=1` samples per hundred polls is
  normal; `inarea=1` that never fires at all is worth chasing, `inarea=0` alongside `melee=1` is not.
- `charging` far outnumbering `backing off`, and `swinging` possibly absent entirely. That last one is
  not a missing state: the bot fires as soon as it is inside 32 units, which drops it straight into
  the backoff window, so the stand-still branch only shows up in the ~0.1s gap between the 0.4s
  backoff and the crowbar's own refire.

The movement reasons print on the **`no probe`** line, not the `probe` line -- the duel drops the probe
when it takes over, which is deliberate. Looking for them on the probe line finds nothing and reads as
the duel movement never running.

`duel=1` with no `charging`/`backing off`/`swinging` anywhere is the one combination that *is* a bug:
it means the state formed but `DuelDrive` bailed, and the likely cause is the bot never actually
getting melee into its hands.

## scripts/test_monster.ps1

Two phases, each on its own server, for the one thing the rest of the suite cannot see: a bot fighting
something that is not a player. `-Phase` runs one; `-Seconds` sets each phase's length (default 150).

```powershell
.\scripts\test_monster.ps1
.\scripts\test_monster.ps1 -Phase engage -Seconds 300
```

Live monsters come from the bots themselves. `weapon_snark` is the profile's `AMBUSH_SNARK`, so an
equipped bot throws snarks at the first opponent it surprises -- the same machinery `test_fire.ps1`'s
WEAPON phase already exercises, which is why this needs no entity-spawning command. Snarks are consumed
per throw and the give is five, so the phase body re-equips every few seconds or the map runs dry.

| Phase | Asserts |
| --- | --- |
| `census` | `crossbot_monsters` sees `monster_snark` lines, every one carries `FL_MONSTER`, `mon_hEnemy` resolved to a real offset, and at least one snark names an enemy |
| `engage` | `mine` and `trc`/`hit` advance in `crossbot_sense_status`, a snark becomes a bot's `target=`, and a bot logs `entered combat with monster_snark` |

**Run `census` first and believe it before reading `engage`.** The two phases fail identically from the
engage counters alone: a bot that ignores hunting snarks and a map with no snarks on it both produce
`mine:0`. The census is the only thing that tells them apart, and it is also the only check on the
`m_hEnemy` offset -- a wrong offset reads as `enemy=-1` on every line while everything else looks healthy.

The phase is stochastic in the same way `test_fire.ps1`'s duel is: a thrown snark has to find a bot
before its detonate timer runs out, and crossfire is vertical enough that the uphill ambush veto refuses
plenty of throws. Counts are polled across the window rather than asserted at a tick, and `switched`,
`stopped` and `died` are reported rather than asserted -- `stopped` in particular depends on a snark
re-targeting mid-fight, which happens on its own two-second hunt clock.

## scripts/test_deploy.ps1

Four phases, each on its own server, each asserting something only it can show. `-Phase` runs one;
`-Seconds` sets the bot phases' length (default 150).

```powershell
.\scripts\test_deploy.ps1
```

| phase | what it proves |
|---|---|
| `gates` | `crossbot_list_gates` finds chokepoints on crossfire, boot_camp, stalkyard and test2_doorway, and no accepted gate exceeds the 144-unit opening bound. Needs no bots, and it is the phase that catches a broken gate scan on its own |
| `tripmine` | placements happen, **both stances appear**, beams are crawled under and **never jumped**, no mine was removed for finding no surface, bots still reach goals with beams up, and — the one that matters — **no bot walks into its own beam** (`onbeam 1`) and no mine arms around its placer. Needs `crossbot_deploy_debug 1`, which the phase sets |
| `standoff` | with the surface pinned to ground on `test2_doorway`, the side search gives up, `EngageStandoff` runs, and the mine is destroyed by gunfire — and the bot never sits `blocked by monster_tripmine` |
| `satchel` | satchels are thrown and detonated, no detonation is counted twice, and **no detonation fires on a contact under 16 units** |

**`contact` is the assertion this phase exists for.** `crossbot_deploy_debug` splits every own-mine
death by `onbeam`: `1` means the bot walked into its own laser and is a bug, `0` means it was standing
in the blast when something else tripped the mine, which is the game working as designed and will
never be zero. Asserting on the raw self-kill count would fail forever; asserting on `contact` is what
caught the real regressions.

**The stance assertion is the other load-bearing one.** A standing placement puts the beam 64 above
the floor and a ducked one at 30, and only the first can be crawled under. If only one stance ever
appears the roll is dead and half the avoidance code is untested while every counter looks healthy.

**`jump` is asserted to be zero, not positive.** The first version of the beam ladder jumped low beams
and this phase asserted it happened. That was backwards: a jump sweeps the player box up through the
beam plane, so it trips every time. Both `walk`-state deaths in the instrumented baseline were exactly
that.

**The standoff phase needs the pin, and equipping does not substitute.** A vertical beam is only
reachable through `crossbot_deploy_surface 3`: mounts on the ceiling or ground are gated behind a
48-unit opening, which is rare enough that an unpinned scan over six stock maps finds none. Without
the pin the side-search and shoot branches never run at all. `test2_doorway` is the map because it has
one route per gate, so a beam across it cannot be walked around.

That map is also **smaller than the 375-unit standoff**, so the retreat always ends on
`kBreakRetreatTime` rather than on distance. That is the designed degradation, not a failure — the bot
fires from as far back as the corridor allows and takes some blast. A run where `killed` is non-zero
has exercised the path whether or not the retreat completed.

**The bot has to be driven on `test2_doorway`.** There are no items to fetch, so `BotItems::SelectGoal`
returns false, the bot idles in its spawn area forever and never stands in a gate. The phase alternates
`crossbot_nav_goal` between the spawn and the `info_target` to walk it through the doorways.

**The satchel contact bound is the real assertion.** `monster_satchel` plays a bounce sound on landing,
`BotSense::Hear` pushes a suspicion for the emitter, and a bot that counts its own charge as a contact
detonates the instant it lands — reporting a contact about 2 units away. An early run scored 181
detonations off 7 satchels that way, with every other counter looking fine.

**Placements are stochastic and the counts will not repeat.** Deployment is opportunistic — a bot mines
only gates in the area it happens to be standing in — so which gates get mined, and whether anyone
walks back through them, varies per run. The assertions are all "more than zero" for that reason.
A phase reporting `placed=0` on crossfire is a real failure; a phase reporting 9 where it once
reported 14 is not.

## scripts/test_aim_error.ps1

Measures the intentional aim error by firing the same flick many times through `crossbot_aim_test` and
reporting the spread of settle times at each `crossbot_aim_error` scale, plus the resting error left by the
hand quantum at each `-Fovs` value.

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

### The rest error row, and what zoom is supposed to do to it

Each flick also reports a **rest error**: the miss still standing two seconds in, once everything has
stopped. Settle time measures the motor model; rest error measures the hand's *resolution*, and they move
independently — a change can leave settle untouched and still triple the resting miss.

`-Quantum 0 -Impulse 0` is the regression form. With the hand resolution off the controller is exact again,
so rest error must read **0.000 on both axes** and the settle rows must land back on the numbers above.
Anything non-zero there means a quantum leaked outside its cvar.

At the defaults the yaw rest error should spread across roughly zero to one quantum (0.448 degrees), because
the hand dithers across the count it is nearest rather than parking. A row where every sample reads the same
value means it *is* parking, which is the failure the speed floor exists to prevent.

**The `-Fovs` sweep is the one that shows what zoom buys**, and it is the reason the whole hand-resolution
model exists. Use a small flick with it — 40 degrees at fov 20 is the same 180 degrees of hand travel as a
180 at fov 90, so anything larger runs past the two second rest sample:

```powershell
& .\scripts\test_aim_error.ps1 -Scales 1.0 -Fovs 90,20 -Shapes "40 0" -Flicks 10
```

Mean rest error should fall by roughly the FOV ratio: measured 0.299 degrees at fov 90 against 0.052 at fov
20. Read the mean over the group and not a row — a single sample is one point on the tremor cycle. Two
sanity checks come free in the same output: settle time must *rise* at fov 20 (the trade is real, about
0.180s to 0.393s), and peak hand speed for a 40 at fov 20 must match peak hand speed for a 180 at fov 90,
since those are the same physical hand movement.

Read `-Scales 1.0` rather than 0 for this. At scale 0 the flick is deterministic, so every sample lands on
the same phase of the tremor and the spread collapses to a single repeated number that says nothing.

`test_fire.ps1` carries the same two overrides for the same reason. A DUEL result is only comparable
against another DUEL run of the same length on the same build, so do not diff it against a number recorded
before some other change — run the off case yourself:

```powershell
& .scripts	est_fire.ps1 -Full -Quantum 0 -Impulse 0
```

What should move is trigger discipline and nothing else. `targets` must hold roughly steady between the two
(the quantum does not touch acquisition), while `shots` and `deaths` drop — measured about -26% and -29% on
crossfire. `targets` falling with them would mean the change is suppressing engagements rather than shots.

**The `0 90` shape cannot report a rest error and never could.** A 90 degree pitch flick from level clamps
to `kPitchLimit`, and `BeginTest` clamps the target the same way, so the view and the target are both pinned
at 89 and the error is identically zero. That row's settle time is still meaningful; its rest error is not.

## scripts/profile_hlds.ps1

Brings up a dedicated server at `maxplayers 32`, adds N bots over rcon, warms up, turns the zone
profiler on, and optionally runs `AMDuProfCLI collect` against the live pid.

**It needs a profiler build.** The instrumentation and the `crossbot_perf` cvars are behind
`CROSSBOT_PERF`, off by default:

```
build.bat -DCROSSBOT_PERF=ON
```

(the value caches in `build/CMakeCache.txt`, so plain `build.bat` keeps whatever was last set, and
`-DCROSSBOT_PERF=OFF` puts it back). Against a default build the rcon commands come back as unknown
and the run produces no `CrossBot PERF` lines at all -- which looks exactly like a server that had
nothing to report.

**`ZONE_SN_LOS traces/f` is the regression witness for vision cost.** Nothing in the PowerShell suites
measures tracelines, so a change that quietly restores per-candidate line-of-sight tracing would pass
every one of them -- it would be slower, not wrong. Vision is bounded at two `dont_ignore_monsters`
traces per bot per frame by construction, so at 32 bots `ZONE_SN_LOS` must read at most about 2.0 per
frame per bot, and `ZONE_SN_BONE` must track it. Measured 2026-09-03 on crossfire at 32 bots: **51.6
traces/frame over ~30.5 active bots, 1.69 per bot**, against a mean `cands` of **6.18**.

`crossbot_sense_status`'s `cands` field gives the counterfactual in the same breath: it counts the
players that passed range and the cone, which is exactly what the old scan would have traced, so `cands`
against `ZONE_SN_LOS` is the before/after without needing two builds. Normalise by a per-bot-per-frame
zone (`sn_damage`'s `calls/f`) rather than assuming the bot count -- not every bot is updated on every
frame, and dividing by 32 understates the per-bot figure by about 5%.

`-NoProfiler` skips uProf and just sleeps, which is the usual mode: the zone report is the part that
attributes, and uProf only adds the module split. `-NullStdout` sends the server's stdout to `NUL` and
reads readiness and results from `D:\hlds\qconsole.log` instead. That is not a convenience -- a blocking
`WriteFile` on the redirected stdout pipe costs roughly 700us per `SERVER_PRINT`, enough to dominate
any zone that logs, so a run measuring a chatty code path has to say which of the two it is measuring.
`-NoConDebug` drops `-condebug`, which by contrast changes almost nothing.

Output lands in `nav_debug/perf/<map>_<stamp>.log` (the server's whole console) next to the uProf
session directory. The interesting lines:

| line | what it is |
| --- | --- |
| `CrossBot PERF SPIKE` | one frame over `-SpikeMs`, with its worst bot and four heaviest zones |
| `CrossBot PERF <zone>` | per-zone self time for the window, sorted, with calls/frame and worst frame |
| `CrossBot PERF traces` | trace totals for the window |
| `CrossBot PERF  T <zone>` | traces charged to that zone: count, ms each, mean ray length, how many clip entities |

The `T` table is the one that answers "what are all these traces *for*", since a trace is charged to
whichever zone was running when it started.

### Profile with `-DCROSSBOT_VERBOSE=OFF` unless you are measuring the logging

A `SERVER_PRINT` costs ~700us on the redirected pipe, so a chatty path measures its own logging. The
`rp_log` zone was the worst offender until it was compiled out (`doc/bot.md`, `bot_log.cpp`). A
profiling build wants both flags:

```
build.bat -DCROSSBOT_PERF=ON -DCROSSBOT_VERBOSE=OFF
```

Neither is a default, and neither prints anything reassuring when it is *missing* -- check for
`crossbot: zone profiler ENABLED` and `crossbot: verbose bot logging DISABLED` in the CMake output.

### What AMD uProf can and cannot tell you about spikes

uProf finds *hot*, never *bursty*, and on Windows there is no CLI path to the latter. Checked against
uProf 5.3:

- **`report` has no time dimension.** `--time-slice`, `--timeline`, `--time-range` and `--start-time`
  are all rejected as invalid options. `--interval`, `--sort-by`, `--cutoff`, `--view` and `--detail`
  exist but every one of them aggregates the whole session. A 150ms spike in a 60s run is 0.25% of the
  samples and is indistinguishable from noise in a flat profile.
- **OS Runtime Tracing is Linux-only.** `--osrt-event` / `--osrt-threshold` / `--trace osrt` are the
  one feature that would catch a stall by duration threshold, and the Windows CLI answers `Error:
  Invalid option --trace`. (The option strings are present in `AMDApplicationViews.dll`, which is
  shared with the Linux build -- their presence in the binary is not evidence they work here.)
- **Windowing exists only at collect time.** `--start-delay`, `--duration` and `--start-paused` are
  accepted, but they require knowing *when* the spike will happen, which is the thing you are trying to
  find out.
- **The translated database does carry time buckets, and that is the way in.** See below.

uProf still cannot see a frame boundary, and per `doc/bot.md` its call graph dead-ends in unsymbolised
`swds.dll`, so engine time never attributes back to the plugin call that asked for it.
`crossbot_perf_spike` remains the tool for attributing a frame. What uProf adds is the one thing the
zone profiler structurally cannot see: time the game thread spent **not running at all**.

## scripts/uprof_timeline.ps1

Reads a uProf session as a 1ms timeline. Needs `duckdb.exe` (the plain Windows CLI from duckdb.org)
next to `AMDuProfCLI.exe`; the script fails with that instruction if it is missing.

```powershell
.\scripts\profile_hlds.ps1 -Map rocket_frenzy -WarmupSeconds 25 -ProfileSeconds 60
.\scripts\uprof_timeline.ps1
```

With no `-Session` it takes the newest `nav_debug/perf/uprof_*`, translating it if needed.

### Why a 1ms timeline exists at all

`AMDuProfCLI translate -i <session>` writes `cpu.db` and `callstack.db` in **DuckDB** format, not
SQLite (magic `DUCK` at offset 8). `UnifiedSampleSeries` carries a **`windowId`** column, indexed as
`timeSeriesIdx`, and `session.uprof` records `"interval": 1` / `"isTimeSeriesData": true` -- so
**`windowId` is a millisecond index from the start of collection**. The GUI timeline reads it; nothing
in the CLI does.

The dedicated server's game loop is a single thread, so one row per `windowId` for the busiest thread
is a per-millisecond trace of the loop. On a saturated 32-bot run that thread has a sample in ~90% of
all milliseconds.

### Runs and gaps are different bugs

The script reports both, and they need opposite fixes:

- **Runs** -- consecutive milliseconds in one module. The thread was on-CPU. The module and RVA say
  where the compute went.
- **Gaps** -- milliseconds with **no sample at all**. The thread was off-CPU: blocked on I/O, a lock,
  a page fault. Optimising the surrounding code does nothing for these, and the zone profiler cannot
  tell them apart from slow code because wall-clock time passes either way.

### What it actually found, and the honest limit

On a saturated map there is often **no stall to find**. crossfire at 32 bots has no run longer than
8ms: the loop bounces between two `swds.dll` regions thousands of times a frame. That is the correct
answer -- the frame-time variance is *how many* traces a frame issued, not a different code path -- but
it means a sampling profiler has nothing to say. rocket_frenzy's longest run is 37ms.

The gaps are where it earns its keep. rocket_frenzy showed an 18ms off-CPU gap immediately after
`NavPlan::PlanRoute`, then a 60ms gap 97ms later, and a 55ms gap in `ntdll.dll` elsewhere in the run --
a stall cluster nothing in the zone report distinguishes from ordinary planning cost.

Two limits worth knowing before trusting a session:

- **The spike log caps at 8 entries per window**, so the worst frame is frequently not in it. That same
  rocket_frenzy run reported `max 652.565ms` in its `PERF window` line while the largest logged
  `PERF SPIKE` was 22.86ms. Do not assume the spike list contains the worst frame.
- **A collect whose target died looks like a quiet server.** Check the raw `.prd` size -- the script
  warns under 100KB. A 32-bot `xen_dm` run that hits `FATAL ERROR (shutting down): ED_Alloc: no free
  edicts` leaves a 528-byte `.prd`, an empty database and a server log with no `PERF window` line at
  all.

## `CROSSBOT_BSPTRACE`: the in-process BSP tracer, and why it is not the default

The planner's airmove sims call `pfnTraceHull` for every simulated frame, and navgen runs the *same*
`PmSim::PmSimulator` against its own in-process BSP tracer instead. The obvious idea is to use navgen's
tracer at runtime too, since airmove planning does not care about dynamic entities. It was measured, and
it does not pay off.

```
build.bat -DCROSSBOT_BSPTRACE=ON
```

adds `bsp_file.cpp`, `clipnode_tree.cpp` and `polytope.cpp` to the plugin, registers
`crossbot_bsptrace` (0 off, 1 compare, 2 BSP only) and `crossbot_bsptrace_blockers`, and the
`crossbot_bsptrace_stats` command (`crossbot_bsptrace_stats reset` to zero it). In mode 1 every planner
trace runs through **both** backends on the same ray, timed with `QueryPerformanceCounter`; the engine's
answer is the one returned, so behaviour is unchanged while measuring.

**Off by default and it costs nothing when off.** `BSPTRACE_WRAP` expands to its first argument, the
cvars are not declared let alone registered, the command is not added, and CMake does not compile the
three navgen translation units — verified by checking the ninja link line (only `crc32.cpp.obj` remains)
and by grepping the stock DLL for the cvar names. Stock is ~100KB smaller.

### The measurement

16 bots, 60s, per-trace cost net of the ~0.048us timer overhead:

| map | blockers | engine | bsp | speedup | traces disagreeing |
| --- | --- | ---: | ---: | ---: | ---: |
| crossfire | off | 0.617us | 0.470us | 1.31x | 0.110% |
| crossfire | **on** | 0.615us | 0.709us | **0.87x** | 0.026% |
| rocket_frenzy | off | 1.346us | 0.549us | 2.45x | 3.763% |
| rocket_frenzy | **on** | 1.373us | 1.071us | **1.28x** | 1.551% |

**The speedup and the wrongness are the same number.** The engine's per-trace cost more than doubles
between crossfire and rocket_frenzy while the world-only BSP tracer barely moves -- that gap is entity
handling, and skipping it is the whole win. Put the entities back and crossfire is *slower than the
engine*, because `TraceHullWithEntities` does a full clipnode descent per blocker with no bbox rejection
(fine offline across threads, not in a frame) while the engine has a spatial index. Even with a bbox
pre-filter and blockers rebuilt from live edicts, rocket_frenzy still disagrees on 1.55% of traces:
translating a brush model by its origin does not handle rotation, and `ignore_monsters` still lets the
engine hit pushables and items.

**`rp_plan`'s `vs ents 0%` is not evidence for this idea.** That column excludes `FL_WORLDBRUSH` *and*
baked blockers, so a trace stopping on a static door reads as "no entity" while a world-only tracer sails
straight through it. That is exactly the 3.76%.

**The ceiling.** `rp_plan` traces are 9.0ms of 5440ms of crossbot time on crossfire and 104.2ms of 5644ms
on rocket_frenzy. The correct variant is a 1.3ms *loss* on crossfire and saves 22.8ms on rocket_frenzy --
0.40% of crossbot time. On the 93ms `rp_plan` spike frame tracing is only 39% of the cost, so even the
incorrect 2.45x variant leaves a ~71ms spike. The airmove cache hit rate governs how many sims run at
all and is the far bigger lever; see `## Airmove cache: correctness and tuning`.

One bias to keep in mind if this is re-measured: the engine trace runs first on each ray and warms the
cache for the BSP trace that follows, so the BSP figure is if anything flattering.

## Airmove cache: correctness and tuning

One cvar and one command, both available in a **stock** build (neither is behind `CROSSBOT_PERF`):

- `crossbot_airmove_verify 1` — re-simulate on every cache hit and compare against the stored result.
  Costs the entire speedup, so this is a correctness harness, not something to leave on. On a mismatch the
  *fresh* result is the one returned, so verify mode is self-correcting.
- `crossbot_airmove_correct 0` — disable the in-flight trajectory correction, so the follower re-issues
  the launch heading unchanged for the whole flight. Default is `1`. This is the A/B switch for the
  correction: same binary, same nav, one boot command apart. Set it through `-BootCommands`, never
  `-ServerArgs` — plugin cvars register in `GameDLLInit`, after the engine has parsed the command line.
  The landing log line carries `peak err` and `corrected` (frames), so a run's correction effort and its
  miss can be read off together.
- `crossbot_airmove_cache_stats` — occupancy/capacity, hits, misses, hit rate, inserts, evictions,
  uncacheable, and (under verify) verified and MISMATCH counts. `MISMATCH (N ent)` splits out the ones
  where the fresh sim touched an entity, which is the known and accepted divergence — see `doc/nav.md`.

Counters reset on map change. `profile_hlds.ps1` prints the stats line before it stops the server, so any
profile run doubles as a cache measurement.

There is **no sizing cvar**. Capacity is `kAirmoveCacheSlots` in `nav/airmove_cache.h`, a compile-time
constant, because the table is a plain member array. Changing it, or the eviction tunables
`kAirmoveAgeStep` / `kAirmoveMaxDecay`, means a rebuild.

### Reading the stats

Occupancy is not the interesting number — it pins at capacity within a minute on any real map and stays
there. Judge a change on **hit rate and evictions together**: a policy that raises the hit rate while
lowering evictions is keeping the right entries.

If `uncacheable` dominates the misses, neither capacity nor eviction is the problem — that map hangs
doors, plats or trains in its flight paths and those sims are refused outright by the purity rule.

### Judging a change to the cache

Interleave the A/B on one map in a single sitting. Absolute frame times on this box drift by ±50% over an
hour — `sn_los` per-trace cost alone swings that much between runs — so a run taken now compared against
one taken twenty minutes later will tell you nothing. Since capacity is compile-time, an A/B means two
builds; do them back to back and alternate the runs:

```powershell
.\scripts\profile_hlds.ps1 -Map crossfire -NoProfiler -WarmupSeconds 20 -ProfileSeconds 60 `
    -PerfInterval 60 -SpikeMs 8
```

Read `PERF window` (frames, avg, p50/p90/p99) and `PERF  T rp_plan` from each log. The server is fully
saturated at 32 bots — `crossbot` is ~98% of wall in every configuration — so freed planning time shows up
as **more frames and a shorter tail**, not as less total trace time.

Always check more than crossfire. `xen_dm` and `rocket_frenzy` are the air-move-heavy maps and they behave
nothing like it: crossfire caches at 94% while xen_dm sits at 56% with the table thrashing.

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

**Combat.** `test_fire.ps1`. The nav sweep is blind to it -- one bot per server means no target, so
`BotFire::Think` exits on its first branch every frame and the whole system is inert. That cuts both
ways: it is also why a firing change can be cleared for movement regressions by the nav sweep alone,
and why a nav sweep result of 45/45 says nothing whatsoever about whether firing works.

**Deployables.** `test_deploy.ps1`. The nav sweep is blind to this one **by construction, not by
accident**: `test_nav_goal.ps1` boots with `crossbot_deploy 0`, alongside the `crossbot_gather 0` it
already set. It has to. A bot that mines a doorway changes every route through it for the rest of the
run, so the per-map waypoint diffs the sweep exists to produce would stop meaning anything, and a map
would fail for a beam rather than for the change under test.

The useful consequence is the same as the firing one: with deploy off, `BotDeploy` never runs and the
beam registry stays empty, so every branch this feature added to `bot_avoid.cpp` early-outs. A nav
sweep therefore clears a deployable change for movement regressions and says nothing at all about
whether deployment works. It also means the reverse check is cheap — if a sweep failure's per-map log
has no `avoid` or `monster_tripmine` lines in it, the deployable code did not run there and cannot be
the cause.

Running the sweep a second time with deploy left **on** is worth doing once for a change to the
avoidance side, but read it as an observation and not as a gate: with three bots mining crossfire the
route each one takes stops being repeatable, which is exactly what the sweep needs it to be.
