# Manager and Diagnostics

The manager that spawns and buries bots, and the logs and profilers watching it all.
Movement is `doc/movement.md`; difficulty is `doc/difficulty.md`; wants, senses and fighting are `doc/goals.md`, `doc/perception.md` and `doc/combat.md`.

## bot_manager.cpp

### Death and respawn

`StartFrame` drives a dead bot with `IGameProfile::Respawn` instead of `Bot_MoveNav`, and it has to keep
doing so every frame: `SV_Physics` skips client edicts entirely, so a fake client's think only runs from
inside `pfnRunPlayerMove` (`SV_RunCmd` sets `v.button` from the usercmd, then `pfnPlayerPreThink`, and
`CBasePlayer::PreThink` dispatches to `PlayerDeathThink` whenever `deadflag >= DEAD_DYING`). Stop calling
`pfnRunPlayerMove` for a dead bot and the game DLL never gets another chance to respawn it.

**The button has to be released before it is pressed.** `PlayerDeathThink` gates the `DEAD_DEAD` →
`DEAD_RESPAWNABLE` transition on *no* button being held (`player.cpp`, "wait for all buttons released"),
and only the following call, with `deadflag` already `DEAD_RESPAWNABLE`, reaches `respawn()`. Holding
`IN_ATTACK` on every dead frame — which is what `CHLDMProfile::Respawn` used to do — therefore parks the
bot at `DEAD_DEAD` permanently; it also never reaches the `mp_forcerespawn` branch further down, so
nothing else rescues it. `Respawn` now sends zero buttons until `deadflag == DEAD_RESPAWNABLE` and
`IN_ATTACK` on that frame alone.

Nothing resets the bot's plan across a death. It does not need to: the respawn moves the bot far enough
that `Bot_MoveNav`'s `kTeleportJumpThreshold` guard calls `ResetPath` on the first live frame, and when
the spawn point is *close* to where the bot died the stale plan still starts within that same distance and
is followed correctly. `hasGoal`/`goalPoint` survive on purpose, so a respawned bot resumes walking to the
goal it was given.

`crossbot_kill` exists to make this testable from the dedicated-server console.

A dead bot's only physics tick is the respawn frame. `pfnRunPlayerMove` receives one frame's worth of msec (not a token 1ms), and leaves `gpGlobals->frametime` holding whatever msec it was given. Every later bot in the frame would inherit that stale timing if nothing restored it, so the respawn is a full frame's worth to avoid propagating old command msec into the next bot's calculations.

### Level changes drop every bot, and that is the engine doing it

`SV_InactivateClients` disconnects fake clients outright ("Dropping fakeclient on level change"), which
runs `pfnClientDisconnect` and so clears the `CBot` slot. Bots are therefore gone after `changelevel` or
`map` and have to be re-added with `crossbot_add`; this is not a plugin bug and there is no roster to
restore. Re-adding on the new map — and after a plain `kick` — was verified to produce a fully working
bot, so the reused client slot and its leftover `CBasePlayer` private data need no special handling.

The two commands tear down in opposite orders, which matters to anything that ever wants to hook this:
`changelevel` runs `SV_InactivateClients` (so `ClientDisconnect`) *before* `pfnServerDeactivate`, while
`map` goes through `Host_ShutdownServer`, which calls `pfnServerDeactivate` first and drops the clients
after.

### PrintNavStatus
Backs `crossbot_nav_status`, and is the only view of what the runtime planner actually decided —
`--dump-path` shows what navgen would plan for its own spawn/target pair, which is usually a different
pair and often the opposite direction. Prints:
- Origin, velocity, angles, move speeds
- Goal state, target waypoint
- Elapsed time vs ETA (with OVERDUE flag)
- Planned and actual area paths (truncated)
- Full waypoint list with jump/crouch flags

The pairs that carry the diagnosis:
- `vel` against `move` — `move` is the forward/side pair last handed to `pfnRunPlayerMove`. Large
  `move` with `vel=(0,0,0)` is the bot pressing at full drive and not moving: it is against geometry,
  not lost and not idle.
- `planned=` against `actual=` — planned is the route's area sequence, actual is the areas walked.
  Actual diverging once is a repath; actual cycling is the air-move loop described under
  `RunAirmoveApproach`.
- `waypoint=i/n` against the waypoint list — which maneuver it is stuck before. `n` also reveals a
  repath, since a fresh plan renumbers from the bot's position.
- `hasGoal=0` with `waypoint=0/0` is ambiguous by itself: it is both "arrived and cleared the goal"
  and "gave up". Distinguish with the distance to the goal, and remember the goal was clamped
  (see `doc/path_planning.md`, `PlanPath`).
- `item=` with `itemArea=`, `memo=`, `full=` — the gather goal: which entity the bot is after, the
  area it resolved to, and how many entries are in each memo. `item=-` with `hasGoal=1` is a
  commanded `crossbot_nav_goal`, not a gathered one. `charging=` appears only while the charger
  runner owns the frame, and carries the value it is watching.
- `tier=` with `stopTier=`, `detour=`, `target=` and `targetArea=` — the goal hierarchy. `tier` is the
  distant target's tier, `stopTier` the current leg's, and they differ exactly when `detour=1`: a bot
  reading `tier=fun stopTier=gather detour=1` is on its way to a gimmick and picking something up en
  route, which is the intended shape and not a confusion. `tier` changing while `detour=1` holds is
  the broken-stickiness signature (`bot_goal.cpp`).
- `fun=` with `funKind=`, `funAct=`, `mounted=` and `shots=` — present only while the fun tier owns
  the goal. `mounted=1` with a nonzero `move=` is the 30-unit leash about to break.
- `combat=` with `enemy=`, `enemyAge=` and `ready=` — whether `BotCombat` owns the bot, who sense
  remembers, how stale the sighting is, and the bot's `EvaluateCombat` score. `combat=0` next to a
  live `enemy=` is normal and not a contradiction: sense remembers someone the bot never saw well
  enough to fight, or combat already exited on one of its own rules while the suspicion lingers.
  `ready=` no longer selects anything — it survives because carefulness still routes by it.
  `enemyAge=` above `crossbot_sense_decay` should never be seen; the record is dropped there.
  `crossbot_combat_status` is the fuller view.
- `breaking=` with `health=`, `swings=` and `for=` — present only while `BotBreak` owns the frame.
  `health` not falling across two samples while `swings` climbs is the unbreakable-glass signature; the
  no-damage grace will abort it a moment later.
- `avoid=` with `hit=` — the last obstacle decision (`duck`, `jump`, `side`, `blocked`, `break`) and the
  classname it was made against, plus `duckLeft` on the crawl latch, the `gap`/`top` the decision was
  made from, and `detour=` for how many spliced detour waypoints are still ahead. `avoid=none hit=-`
  on a bot stalled against something means the trace never qualified a hit, which is a different bug
  from a mistimed maneuver; `avoid=blocked` means it qualified and no route past it was found. Each
  new decision also prints its own line to the server console with distance, speed and trace count,
  which is what makes this tunable from one run — note those lines go to `qconsole.log`, not to the
  RCON transcript the test scripts capture.

## nav.cpp

### Nav namespace
Global mesh management:
- `LoadForMap(mapName)` - loads nav file for map
- `Unload()` - releases mesh
- `IsLoaded()` - checks if mesh loaded
- `Get()` - returns pointer to NavMesh

## bot_perf.cpp

A nested wall-clock zone profiler, compiled out unless `CROSSBOT_PERF` is defined. It exists because
neither of the obvious alternatives answers the question "which frame was slow and why":

**A sampling profiler cannot see into the plugin.** AMD uProf attributes work by module fine --
`swds.dll` 76%, `crossbot_mm.dll` 7%, `ntoskrnl` 7% at 32 bots on crossfire -- but the call graph
dead-ends. `swds.dll` ships without symbols and FPO unwinding fails there, so every engine leaf hangs
off one address in `hlds.exe`'s main loop and none of the engine time attributes back to the plugin
call that caused it. Since most of what crossbot costs *is* engine time it asked for, the module split
alone reads as "the engine is slow".

**CPU sampling under-counts stalls.** A blocking `WriteFile` on the server's stdout is off-CPU, so it
barely samples, and console logging measured at a fifth of all crossbot time was nearly invisible to
uProf. Wall-clock zones charge it.

`Enter`/`Leave` push a `StackEntry` and subtract child time from the parent, so a zone reports both
inclusive and self time; `BOTPERF_SCOPE(ZONE_X)` is the RAII form and is what most call sites use.
Depth is capped at 24 with no dynamic allocation on the timing path.

### Trace attribution needs the plugin's own table, not a metamod hook

Metamod's `GetEngineFunctions` hooks the *game DLL's* calls into the engine. A plugin's `g_engfuncs` is
a plain `memcpy` of the real engine table taken in `GiveFnptrsToDll` (`h_export.cpp`), so crossbot's own
`TRACE_LINE` never passes through metamod at all. The first version of this counted zero traces inside
`sn_los`, a zone that is nothing but traces.

`InstallTraceWrappers` fixes it by saving the four trace pointers out of the plugin's own `g_engfuncs`
and overwriting them with wrappers. The metamod hooks in `engine_hooks.cpp` stay -- they catch hl.dll's
traces, which is how the `runmove` zone accounts for the engine's own movement traces -- and the two
paths cannot double count because they are different function tables.

A trace is charged to whichever zone is on top of the stack when it starts, along with its ray length
and whether it clips entities, which is what makes a per-purpose breakdown possible. Only the outermost
trace is timed (`g_traceDepth`), so a wrapper calling into another cannot double count.

### Reporting

`crossbot_perf <seconds>` starts a window and reports every N seconds; `crossbot_perf_spike <ms>` logs
any frame over that threshold with its worst bot and its four heaviest zones, capped at 8 per window so
a bad window cannot itself become the bottleneck. The window keeps every frame's duration for
percentiles and a full zone snapshot of the single worst frame. Both cvars only exist in a
`CROSSBOT_PERF` build.

## bot_log.cpp

One function, `BotLog::Verbosef`, reached only through the `BOT_VLOG` macro in `bot_log.h`. With
`CROSSBOT_VERBOSE` defined the macro forwards to it; without, it expands to `((void)0)` and the
compiler drops the call, the arguments and every expression that only fed them.

### Why the logging is compiled out rather than gated at runtime

A `SERVER_PRINT` on this harness costs roughly 700us -- the server's stdout is a redirected pipe and
the write blocks (`profile_hlds.ps1` in `doc/testing.md` measures this). `Repath` emitted one print per
connection *and* one per waypoint, so a single 20-waypoint route paid about 40 of them. That showed up
in the profiler as the `rp_log` zone at 11--15ms per call, and it was the largest single contributor to
the worst frames: one 115ms spike on crossfire was `rp_log=56.01ms` over 5 calls, another 105ms frame
was `rp_log=45.85ms` over 3.

A cvar would not have fixed it. The cost is in the `snprintf` fan-out and the blocking write, both of
which happen before any runtime check could help unless every call site were individually guarded --
which is what the macro does, for free, at compile time. Measured at 32 bots, 60s windows, interleaved
in one sitting:

| map | verbose | frames | p99 | max |
| --- | --- | ---: | ---: | ---: |
| crossfire | on | 926 | 186.6ms | 273.9ms |
| crossfire | off | 1108 | 119.3ms | 171.6ms |
| crossfire | on | 796 | 209.7ms | 293.2ms |
| xen_dm | on | 608 | 403.9ms | 614.9ms |
| xen_dm | off | 1010 | 259.7ms | 473.3ms |

`rp_log` falls to 0.0ms and stops appearing in any spike breakdown.

### What is verbose and what is not

Verbose means **per-bot narration**: what a bot is doing and why. All of `bot_move_nav.cpp` bar two
exceptions, the item narration in `bot_items.cpp`, the touch dump in `bot_manager.cpp`, and the
event-level narration in `bot_avoid.cpp`, `bot_break.cpp`, `bot_deploy.cpp` and `bot_opportunist.cpp`.

Two categories stay unconditional, because something other than a human reading the console depends on
them:

- **Console command output.** Everything in `commands.cpp`, and the `Print*` handlers in
  `bot_manager.cpp`, `bot_items.cpp`, `bot_fun.cpp` and `bot_deploy.cpp` that they call. These are
  answers to an rcon question; a build that answered nothing would be broken, and they cost nothing
  when nobody asks. Command-driven results (`crossbot_aim_test`) and cvar-gated reports
  (`crossbot_time_debug`, `crossbot_perf`) count as the same thing: somebody asked.
- **Server lifecycle and fatal errors.** `gamedata.cpp`, `AddBot`, `ServerActivate`, `nav_loader.cpp`,
  and the `ALERT(at_logged, ...)` interface checks in `dll_hooks.cpp` / `engine_hooks.cpp`.

The split is not "cheap vs expensive" but **"who asked for it"**: a bot narrating itself is verbose
however cheap the line is, and an answer to an rcon command is not verbose however long it is.

### The event lines the test suite parses

An earlier revision kept `bot_avoid.cpp`, `bot_break.cpp`, `bot_deploy.cpp` and `WriteTraceLog`
unconditional on the grounds that scripts assert on them: `nav_target_test.ps1:84` regexes
`CrossBot: <bot> reached goal (x, y, z)`, and `test_deploy.ps1` counts `placed a mine`, `detonated a
satchel`, `avoid jump`, `is shooting`, `destroyed` and `blocked by`. That reasoning was sound about the
scripts and wrong about the requirement: at 32 bots these are the bulk of what a `-DCROSSBOT_VERBOSE=OFF`
build still printed, and "off" has to mean off.

They are verbose now. The scripts that parse them pass `-RequireVerbose` to `test_server_start.ps1`,
which probes for the `crossbot_verbose` cvar after the level loads and refuses to continue with
`EXITED code=-3` naming the rebuild flag. The cvar exists only in a verbose build, the same way
`crossbot_perf` marks a `CROSSBOT_PERF` one, and the probe is the same shape as the one in
`profile_hlds.ps1`. Without it an OFF build makes every assertion read zero, which looks like a
behavioural regression rather than a missing flag.

`WriteTraceLog` needed more than a wrapped print. `bot.traceLog` was appended every frame for every bot
and the file was rewritten on every goal arrival -- all bots to one path, so the result was already
meaningless with more than one -- which is per-arrival blocking I/O in a build that asked for silence.
The accumulation in `Bot_MoveNav` is inside `#if BOT_VLOG_ON`, so the vector stays empty and
`WriteTraceLog` returns at its first line. Its own prints are left alone: the failure branch is a real
error, and it can only be reached from a verbose build.

### The flag defaults on

```
build.bat -DCROSSBOT_VERBOSE=OFF
```

turns it off; the value caches in `build/CMakeCache.txt`, so plain `build.bat` keeps whatever was last
set and only an explicit `-DCROSSBOT_VERBOSE=ON` puts it back. The `option()` default applies on the
first configure of a build directory and never again, so a bare `build.bat` after an OFF build is still
an OFF build -- check the configure line, not the flag you think you passed. CMake prints `crossbot: verbose bot logging DISABLED`
when it is off, which is the only way to tell from the build output.

On by default is the deliberate choice: these lines are how nav work gets debugged, and most of
`doc/path_planning.md`, `doc/movement.md` and this file quote them. The flag exists so a profiling or saturated-server run can drop
them, not so the normal build goes quiet. Pair it with `-DCROSSBOT_PERF=ON` for a profiling build.

Two cvars follow the flag rather than surviving it: `crossbot_gather_debug` (both the item scan dump
and the touch dump) reports disabled in a non-verbose build instead of doing the work and printing
nothing.
