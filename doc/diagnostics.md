# Diagnostics

The profiler and the logs: what they cost, what they print, and which lines the test suite parses.

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

### And the cvar defaults off
 a verbose build `BOT_VLOG` is `if (BotLog::Enabled()) Verbosef(...)`, so each site is one branch on
`crossbot_verbose` and neither the arguments nor the `snprintf` run while it is 0 -- the per-site guard paragraph above asks for, done once in the macro. The cvar defaults to 0 because a normal server not scroll a repath dump per bot per goal; `test_server_start.ps1` sets it to 1 for every test. `#if BOT_VLOG_ON` blocks (trace accumulation, the `penalizedOnRoute` count) stay compile-time.

Two cvars follow the flag rather than surviving it: `crossbot_gather_debug` (both the item scan dump
and the touch dump) reports disabled in a non-verbose build instead of doing the work and printing
nothing.
