# Diagnostics

The zone profiler and the verbose log: what each costs, what it prints, how to turn it on, and which
lines the test scripts parse.

| Switch | Kind | Default | Controls |
| --- | --- | --- | --- |
| `CROSSBOT_PERF` | CMake option | OFF | compiles the profiler and registers `crossbot_perf`, `crossbot_perf_spike`, `crossbot_hitch`, `crossbot_hitch_report`, `crossbot_hitch_stack` |
| `CROSSBOT_VERBOSE` | CMake option | ON | compiles `BOT_VLOG` call sites and registers `crossbot_verbose` |
| `crossbot_verbose` | cvar | 0 | whether a verbose build actually prints; tests set it to 1 |

## bot_perf.cpp

A nested wall-clock zone profiler. `Enter`/`Leave` push and pop a `StackEntry` and subtract child time
from the parent, so every zone reports both inclusive and self time. Most call sites use the RAII form,
`BOTPERF_SCOPE(ZONE_X)`. The stack is a fixed 24 entries deep and the timing path never allocates.

### Why not a sampling profiler

- **A sampler cannot see into the plugin.** AMD uProf splits time by module fine -- `swds.dll` 76%,
  `crossbot_mm.dll` 7%, `ntoskrnl` 7% at 32 bots on crossfire -- but the call graph dead-ends.
  `swds.dll` ships without symbols and FPO unwinding fails inside it, so every engine leaf hangs off one
  address in `hlds.exe`'s main loop and no engine time is attributed back to the plugin call that caused
  it. Most of what crossbot costs *is* engine time it asked for, so the module split alone reads as "the
  engine is slow".
- **CPU sampling under-counts stalls.** A blocking `WriteFile` on the server's stdout is off-CPU and
  barely samples: console logging measured at a fifth of all crossbot time was nearly invisible to uProf.
  A wall-clock zone charges it.

### Trace attribution uses the plugin's own engine table

Metamod's `GetEngineFunctions` hooks the *game DLL's* calls into the engine. A plugin's `g_engfuncs` is a
plain copy of the real engine table taken in `GiveFnptrsToDll`, so crossbot's own `TRACE_LINE` never
passes through metamod; a metamod hook counts zero traces inside `sn_los`, a zone that is nothing but
traces.

`InstallTraceWrappers` saves the four trace pointers out of the plugin's own `g_engfuncs` and overwrites
them with wrappers. The metamod hooks in `engine_hooks.cpp` stay: they catch hl.dll's traces, which is
how the `runmove` zone accounts for the engine's own movement traces. The two paths are different
function tables, so they cannot double count.

A trace is charged to the zone on top of the stack when it starts, with its ray length and whether it
clips entities, which is what gives a per-purpose breakdown. Only the outermost trace is timed
(`g_traceDepth`), so a wrapper that calls another cannot double count either.

### Reporting

`crossbot_perf <seconds>` opens a window and reports every N seconds. The window keeps every frame's
duration for percentiles and a full zone snapshot of the single worst frame. `crossbot_perf_spike <ms>`
logs any frame over the threshold with its worst bot and its four heaviest zones, at most 8 per window so
a bad window cannot become its own bottleneck. `profile_hlds.ps1` (`doc/testing.md`) drives both and
refuses to run when `crossbot_perf` is not registered.

## bot_hitch.cpp and bot_stall.cpp

Also compiled only under `CROSSBOT_PERF`; without it every call is an empty inline. The zone profiler only
sees inside the plugin, so these two answer the question it cannot: **is a long frame ours or someone
else's?**

- **`crossbot_hitch <ms>`** (default 50, 0 off) times every `StartFrame`. The interval is previous end to
  this end, split into *outside crossbot* (previous end to this begin: engine, game DLLs, other plugins) and
  *crossbot* (by section `pre`/`bots`/`brain`, plus the slowest bot). A frame over the threshold prints one
  `CrossBot HITCH` line, at most 4 a second.
- **`crossbot_hitch_report <s>`** (default 0) prints p50/p99/max of the interval and of crossbot's share, and
  the count of frames over 16.7 ms.
- **`crossbot_hitch_stack <ms>`** (default 0, Windows only) starts a watchdog thread. When the main thread
  goes that long without a heartbeat, the watchdog suspends it, scans 32 KB of its stack for return
  addresses inside loaded modules and resumes it; `CrossBot HITCH STACK` lines print on the next frame.
  `scripts/resolve_stall_stacks.ps1 -Log <server log>` groups them and resolves symbols with x86 `cdb`,
  following the symlinks into the Debug builds to find their pdbs. uProf cannot do this: it does not unwind
  kernel waits in this 32-bit process.

What they found (fy_snow and dod_avalanche, 2026-10-19): the 500 ms spikes with 4 bots were all outside
crossbot, in file I/O blocking 100–190 ms a call (`-condebug` reopening `qconsole.log` per line, stdout
`WriteFile`, AMXX `CreateFile`, CrossPlay reads at round restart), most likely an on-access scanner on
`D:\hlds`. A clean 4-bot run has no hitch over 34 ms.

## bot_log.cpp

One function, `BotLog::Verbosef`, reached only through the `BOT_VLOG` macro in `bot_log.h`:

- **Without `CROSSBOT_VERBOSE`** the macro is `((void)0)`. The compiler drops the call, its arguments and
  every expression that only fed them. `BOT_VLOG_ON` is 0, so `#if BOT_VLOG_ON` blocks (trace
  accumulation, the `penalizedOnRoute` count) vanish too.
- **With it** the macro is `if (BotLog::Enabled()) Verbosef(...)`: one branch on `crossbot_verbose` per
  site, and neither the arguments nor the `snprintf` run while the cvar is 0.

### Why the logging is compiled out, not only gated

A `SERVER_PRINT` on this harness costs about 700us: the server's stdout is a redirected pipe and the
write blocks (`profile_hlds.ps1` measures it). `Repath` printed once per connection *and* once per
waypoint, so one 20-waypoint route paid about 40 of them. In the profiler that was the `rp_log` zone at
11-15ms per call, the largest single contributor to the worst frames: one 115ms spike on crossfire was
`rp_log=56.01ms` over 5 calls, a 105ms frame was `rp_log=45.85ms` over 3.

The cost is the `snprintf` fan-out plus the blocking write, both of which happen before a runtime check
can help unless every call site is guarded individually. The macro is that guard. Measured at 32 bots,
60s windows, interleaved in one sitting, with the cvar on:

| map | verbose build | frames | p99 | max |
| --- | --- | ---: | ---: | ---: |
| crossfire | on | 926 | 186.6ms | 273.9ms |
| crossfire | off | 1108 | 119.3ms | 171.6ms |
| crossfire | on | 796 | 209.7ms | 293.2ms |
| xen_dm | on | 608 | 403.9ms | 614.9ms |
| xen_dm | off | 1010 | 259.7ms | 473.3ms |

With the build off, `rp_log` reads 0.0ms and leaves every spike breakdown.

### Why the cvar defaults to 0

A normal server should not scroll a repath dump per bot per goal. `test_server_start.ps1` sets
`crossbot_verbose 1` once the level is up, so every test gets the narration.

### What is verbose and what is not

The rule is **who asked for it**, not how cheap the line is. A bot narrating itself is verbose however
cheap; an answer to an rcon command is not verbose however long.

- **Verbose:** per-bot narration -- what a bot is doing and why. Movement and planning
  (`bot_move_nav.cpp`, `bot_move.cpp`), goals, items, combat, deploy, camp, tactics, strategy, the event
  lines in `bot_avoid.cpp`, `bot_break.cpp`, `bot_opportunist.cpp`, the touch dump in `bot_manager.cpp`,
  and the per-game ability code under `src/game/`.
- **Always printed, console answers:** everything in `commands.cpp`, and the `Print*` handlers in
  `bot_manager.cpp`, `bot_items.cpp`, `bot_fun.cpp` and `bot_deploy.cpp` that it calls. A build that
  answered nothing would be broken, and they cost nothing when nobody asks. Command-driven results
  (`crossbot_aim_test`) and cvar-gated reports (`crossbot_time_debug`, `crossbot_perf`) count: somebody
  asked.
- **Always printed, lifecycle and fatal errors:** `gamedata.cpp`, `AddBot`, `ServerActivate`,
  `nav_loader.cpp`, and the `ALERT(at_logged, ...)` interface checks in `dll_hooks.cpp` /
  `engine_hooks.cpp`.

`crossbot_gather_debug` (the item scan dump and the touch dump) reports itself disabled in a non-verbose
build instead of doing the work and printing nothing.

### The event lines the test scripts parse

Scripts assert on verbose lines: `nav_target_test.ps1` regexes `CrossBot: <bot> reached goal (x, y, z)`,
and `test_deploy.ps1` counts `reached goal`, `placed a mine`, `detonated a satchel`, `avoid jump`,
`is shooting`, `destroyed` and `blocked by`. They stay verbose anyway: at 32 bots they are the bulk of
what a non-verbose build would otherwise still print, and "off" has to mean off.

So every script that parses them passes `-RequireVerbose` to `test_server_start.ps1`, which probes for the
`crossbot_verbose` cvar after the level loads and stops with `EXITED code=-3`, naming the rebuild flag,
when it is missing. The cvar exists only in a verbose build, the same way `crossbot_perf` marks a perf
build. Without the probe a non-verbose build makes every assertion read zero, which looks like a
behaviour regression rather than a missing flag.

### WriteTraceLog

`bot.traceLog` records a bot's origin every frame while `crossbot_trace_path` is set, and
`WriteTraceLog` rewrites that file on every goal arrival -- every bot to the same path, so with more than
one bot the file only holds whoever arrived last. That is per-arrival blocking I/O, so the accumulation in
`Bot_MoveNav` sits inside `#if BOT_VLOG_ON`: in a non-verbose build the vector stays empty and
`WriteTraceLog` returns at its first line. Its own prints are not wrapped; the failure branch is a real
error and is only reachable from a verbose build.

### Building without it

```
build.bat -DCROSSBOT_VERBOSE=OFF
```

The value caches in `build/CMakeCache.txt`. The `option()` default applies only on the first configure of
a build directory, so a bare `build.bat` after an OFF build is still an OFF build, and only an explicit
`-DCROSSBOT_VERBOSE=ON` restores it. CMake prints `crossbot: verbose bot logging DISABLED` when it is off;
that line is the only way to tell from the build output.

On is the default on purpose: these lines are how nav work gets debugged, and `doc/path_planning.md`,
`doc/movement.md` and this file quote them. The flag exists so a profiling or saturated-server run can
drop them. Pair it with `-DCROSSBOT_PERF=ON` for a profiling build.
