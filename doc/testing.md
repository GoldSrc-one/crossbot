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
a threshold: duel 420 shots / 193 burned / 21 deaths with all four suspicion sources; crossbow 81
shots and 21 zoomed polls; hornetgun 378 shots with 72 led; snark 122 shots and `uphill=40`; gauss 60
charged shots with no timeout; 9mmAR 60 arc shots, `uphill=0`, `selfveto=14`; handgrenade 62 throws,
`uphill=59`, `selfveto=44`, `primed=5`; RPG 52 shots against 52 guides and `capped=0` everywhere.

`guides` matching `shots` exactly is the healthiest single number in that output, because the two are
independent observations — one counts a commit the plugin opened, the other counts the game DLL's
attack timer moving.

**The weapon phases need `crossbot_fire_pin`, and equipping instead does not work.** A bot keeps any
weapon that still has a usable mode, so handing a roaming bot a crossbow puts it in the inventory and
nothing else ever happens — a first attempt gave 288 crossbows and hornetguns away across two minutes
and recorded zero projectiles fired. The pin makes every other weapon unusable, which is the only
lever that reaches selection.

**The gauss needs the mode argument as well, for the same reason one level down.** The random pick
names a weapon *and* a mode but only the weapon survives the switch, so the frame after selecting, the
mode order restarts at primary. Gauss primary is a hitscan that works at any range, so
`FIRE_CHARGE` is unreachable without `crossbot_fire_pin weapon_gauss secondary`. The hornetgun escapes
this only by accident: its primary's flight-time ceiling rejects targets past 900 units, which is the
one gap in the table that lets a secondary win.

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
every frame the ray test fails. Everywhere else it is a reason string and therefore a sampled one — it
survives on the crossbow only because a bolt's ray gate rejects on most frames a target exists, so a
two-second poll cannot miss it. Do not copy the pattern to a new phase without checking that.

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

## scripts/measure_dps.ps1 / measure_dps_all.ps1

Gathers the raw material for `getDPS(weapon, distance)`. `measure_dps.ps1` boots one server on one
map and walks the whole weapon table a pin at a time; `measure_dps_all.ps1` generates the meshes and
fans the children out. Neither asserts anything about a weapon — the plugin writes the rows, the
scripts sequence the phases, and the fit happens offline once there is enough data to fit.

**The plugin must be built with `-DCROSSBOT_DPS=ON`.** The whole measurement layer, the
`CBasePlayer::TakeDamage` hook included, is behind that `#ifdef`; a normal build does not even
register `crossbot_dps`, and a sweep against one gathers nothing. The option is cached in
`build/CMakeCache.txt`, so it survives until explicitly set OFF, and a measurement build announces
itself at load.

**Nav generation is stage one and not optional.** Only `crossfire.nav` was ever deployed; the other
eighteen stock DM maps have no mesh at all, and a bot that cannot move measures nothing. Every map is
generated and every map is kept regardless of what navgen reports about it, but the manifest records
areas, connections, gateless connections and ill-formed areas per map, so a map whose ranges look
suspiciously short can be checked against its mesh rather than guessed at.

**One server per map, not one per (map, weapon).** The matrix is the same either way. A phase is sixty
seconds and an HLDS boot is nearly a third of that, so cycling seventeen pins inside one server pays
the boot nineteen times rather than three hundred and twenty-three.

**Seventeen phases, eighteen modes.** The phase list is every mode `BotFire::Supported` accepts, which
leaves out the tripmine, the satchel and the three toggle modes. The crossbow is one phase producing
two modes: `primaryZoomed` is a hitscan with a different aim requirement and no pin reaches it
separately, so the phases cannot split it and the `zoomed` column on each `shot` row does.

**Every phase kills its bots first.** The pin reaches selection, and selection alone does not isolate a
phase: a commit — a gauss charge, a thrown grenade, a rocket in the air — bypasses `ModeUsable` by
design and would otherwise discharge into the next weapon's rows. Same reason `test_fire.ps1` does it.

The phase sequence is `crossbot_fire_pin off` → `crossbot_kill` → settle → `crossbot_fire_pin <cls>
[mode]`. The `off` matters as much as the kill: `BotFire::Pin` zeroes the gaze counters, so it is
what closes
the previous phase's denominator before the next weapon starts filling it.

### MaxParallel 6, and why not 12

Six is the measured ceiling recorded above and in `AGENTS.md`, past which box load alone changes bot
behaviour. It matters more here than in the nav sweep: the aim model is frame-rate coupled — the hand
quantum, the impulse floor and the accel clamp all step per frame — so a starved server measures a
*worse weapon*, not a slower test. That failure would look exactly like a real result.

Every `meta` row carries the server's achieved average frame time for that window, so the distortion is
visible in the data rather than inferred. An idle six-way run reads about 7-8 ms.

### Sixty seconds is a coarse sample, deliberately

A 6-bot 30s phase on crossfire produces on the order of 130-200 shots and 50-130 damage events, so a
60s phase over nineteen maps gives a few thousand samples per weapon — enough for a coarse range
histogram, not enough to fit a confident curve in the long-range bins where events are rarest.

The log is opened in **append** mode and the sweep is resumable for exactly this reason. Running the
whole thing again accumulates rather than replaces, and `-Force` is what re-does a map that already
carries a full set of phase `meta` rows.

Three weapons are sparse or awkward by nature, and none of them is a bug:

- **Crowbar** has `range 32`. Expect a handful of samples per map.
- **Handgrenade** reaches about 550 units, so it depends entirely on bots wandering close.
- **Snark and RPG smear in range.** Damage is credited at the distance where it *lands* while the
  decision was made at the distance it was thrown from, and a snark can bite fifteen seconds and a
  thousand units later.

### The `wait` trap that zeroed every skill value

Worth knowing because it silently falsified results for a long time and left no failure anywhere.

`test_server_start.ps1` used to deliver its boot commands by writing them into a cfg preceded by
forty `wait` commands and exec'ing it from the command line. The waits were there because the
commands need the level active, and a cfg exec'd at `+exec` time runs before `GameDLLInit` has even
registered the cvars.

But `wait` stalls the **whole** engine command buffer for a frame, not just the rest of that cfg.
`GameDLLInit` queues `exec skill.cfg` into that same buffer, so forty waits pushed it forty frames
down — past the map spawn, and therefore past
`CHalfLifeMultiplay::RefreshSkillData`, which snapshots every `sk_*` cvar into `gSkillData` once and
never runs again. Every value it reads was still at its registered default of `"0"`.

The server says so plainly, and grepping any run for it is the check:

```
GAME SKILL LEVEL:1
** GetSkillCVar Got a zero for sk_snark_dmg_bite1 **      (and 68 more)
...four hundred lines later...
execing skill.cfg
```

A healthy boot has `execing skill.cfg` **above** `Spawn Server`, and no `Got a zero` lines at all.

It only reached two weapons, because `CHalfLifeMultiplay::RefreshSkillData` hardcodes a multiplayer
override for every other one — crowbar 25, 9mm 12, .357 40, MP5 12, M203 100, buckshot 20, crossbow
20, RPG 120, egon 20/10, handgrenade 100, satchel 120, tripmine 150, hornet 10. The two it forgets
are the **snark** (`snarkDmgBite`, `snarkDmgPop`, `snarkHealth`) and **gauss primary**
(`plrDmgGauss`; the charged secondary computes `200 * charge` in code and was never affected). Both
measured a flat zero while every other weapon measured correctly, which is exactly the shape of
result that gets believed.

The fix is that the cfg is no longer exec'd from the command line at all: it is written without
waits and exec'd over rcon once the log reports `player server started`. Nothing else stalls the
buffer, so skill.cfg keeps its normal slot. A plain `hlds +map crossfire` never had the problem.

### Reading the raw rows

Row formats are in `doc/bot.md` under `bot_dps.cpp`. The file is opened in **append** mode, so a
schema change means deleting the old CSVs on **both** sides — `nav_debug/dps/` and
`hlds/valve/addons/crossbot/dps/` — or the two layouts interleave in one file and nothing complains.

Three things to do before computing any ratio:

- **Drop rows whose attacker is `?`.** Those are falls and other world damage, and they carry
  `seconds` 0.
- **Drop rows whose attacker equals its victim.** That is the splash rule working, and it is real
  damage, but it is not damage delivered to an enemy.
- **Remember `dist` is the range at impact, not at the press.** For hitscan the two are the same; for
  a rocket or a snark they are not, and the `shot` rows carry the press range if that is what a
  selection layer should be keyed on.

A `dmg` row with `seconds` 0 and a real attacker is damage delivered without the attacker ever having
had the victim in its sights — a grenade around a corner, or splash. It is real damage with no
denominator, so it belongs in the numerator of a pooled ratio but cannot form a per-row DPS.

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
