# Movement

How the bot moves: the frame clock, the movement driver, the profile tracker and velocity servo it steers with, the maneuvers that suspend them, and the obstacles that rewrite the plan — plus the gravity and push physics it all integrates against. The mesh and the plan are `doc/path_planning.md`.

## bot_move.h

### Frame time latching

A fake client receives no engine physics: `SV_Physics` skips every edict with index <= maxclients, so a bot advances only by the msec handed to `pfnRunPlayerMove`. The bot's movement clock is entirely our responsibility.

The engine's `pfnRunPlayerMove` does not restore `gpGlobals->frametime` after it executes: `SV_RunCmd` overwrites it with each command's msec and never restores it. After the first bot in the frame has moved, the global holds the previous bot's command length, not the frame time. Any bot code that runs between moves will read stale timing, so every movement calculation must latch the frame time before any bot runs.

`Bot_BeginFrame()` captures it at the top of `StartFrame`, before any bot has moved. `Bot_FrameTime()` and `Bot_FrameMsec()` return the latched value. Bot code must use these instead of `gpGlobals->frametime` for anything integrated per frame.

### Bot_DecomposeMove
Splits a world-space direction into the forward/side speeds pfnRunPlayerMove takes, which the engine reads along the view axes -- so this is what lets the bot move somewhere other than where it is looking. viewYawDeg must be the yaw passed to pfnRunPlayerMove in that same call plus punchangle, because PM_CheckParamters builds the movement basis from exactly that.

PM_CheckParamters clamps the pair down to maxspeed before anything reads it, so a driveLength well over maxspeed just means "full speed exactly this way", and one below it passes through untouched and becomes the actual speed.

## bot_timing.h

### Movement clock diagnostics

A bot that receives less sim time per second than the wall clock spends looks like it is wading through honey -- both running and gravity appear slower. This module measures, per bot per second:

- **host frame time actually elapsed** (the budget)
- **msec issued** and over how many `pfnRunPlayerMove` calls
- **how much the engine actually executed** — `SV_RunCmd` silently drops commands while `host_client->ignorecmdtime > realtime` (the "clockwindow" speedhack guard), which is invisible from the game DLL unless you watch for `PlayerPostThink` not firing
- **the resulting drift** between bot sim time and real time, which is the same quantity `SV_CheckCmdTimes` compares against `clockwindow`

`crossbot_time_debug 1` enables per-second reports and a once-per-second frame dump showing what each bot in order saw as its frame time.

## bot_move.cpp

### Bot_DecomposeMove
PM_WalkMove and PM_AirMove both flatten forward/right and renormalize before use, so whatever pitch and roll the engine puts in pmove->angles drops out and the basis is just the yaw.

### Bot_NormalizeAngle
Wraps an angle into (-180, 180]. Lives here rather than in bot_move_nav.cpp so both movement
translation units share one copy.

### Bot_BeginFrame and Bot_FrameTime
**`gpGlobals->frametime` is not the frame time once a bot has moved.** `PF_RunPlayerMove_I` calls
`SV_RunCmd`, and `SV_RunCmd` ends with `gGlobalVariables.frametime = ucmd->msec * 0.001` before
`pfnPlayerPostThink` (`sv_user.cpp:1055`). It never restores it. The engine only puts the real value back
at the top of `SV_Frame` and after each `SV_ExecuteClientMessage` in `SV_ReadPackets` -- nothing
restores it between two bots inside one `pfnStartFrame`. So the second bot in the loop reads the *first
bot's command length*, the third reads the second's, and so on down the array.

While every bot asks for the same msec that is self-sustaining and invisible. It stops being invisible
the moment one bot in the middle of the array issues a short command:

```
CrossBot TIME chain (frame 10.04ms): bot1[saw 10ms]=10ms ... bot6[saw 10ms]=10ms
  bot7(dead)[saw 10ms]=1ms bot8[saw 1ms]=1ms bot9[saw 1ms]=1ms ... bot12[saw 1ms]=1ms
```

One dead bot pressing `+attack` to respawn used to hand the engine a token 1 ms command, and every bot
after it in `g_bots` then ran at 1 ms of simulated time per 10 ms frame -- a tenth of real speed, for as
long as that one bot stayed dead. Movement *and* gravity slow together, because both are integrated
against the same msec, which is what makes it look like the bot is wading through honey rather than
like a bot that is lagging or stuck. With a dozen bots someone is nearly always dead, so nearly always
somebody is in it. The 50 ms first half of the jump split and the engine's own halving of commands over
50 ms are the same bug at lower server framerates.

`Bot_BeginFrame` latches `gpGlobals->frametime` at the top of `BotManager::StartFrame`, before any bot
has moved, and `Bot_FrameTime` hands that value out for the rest of the frame. Every per-frame
integration in the bot -- `Bot_FrameMsec`, the aim filter's `dt`, `AirmoveLaunchLead`, the weapon-usage
accumulator -- reads the latch. `gpGlobals->frametime` must not be read directly from bot code.

`crossbot_time_debug 1` prints the per-second budget-vs-issued report and the frame chain above; see
`bot_timing.cpp`.

### Bot_RunPlayerMove
The engine halves any usercmd longer than 50 ms and runs PM_PlayerMove once per half, so a
button held in a long command is re-read after the bot has already moved. IN_JUMP is the one
button where that matters: PM_LadderMove reads it with no oldbuttons gate and answers by
shoving the player 270 units off the ladder plane, so a jump that merely *starts* out of a
ladder's reach ejects the bot the moment a later sub-step brings it into contact.

Every actuation goes through here so a jump costs the same one frame of held button at any server
framerate. Below the split threshold this is exactly the single call it replaces.

It takes `CBot &` rather than `edict_t *` because it is also where the fire button is added. Being the
one funnel every drive branch passes through is exactly what firing wants, and this is the only place
that holds the angles actually being commanded this frame — see `bot_fire.cpp`, *Why the trigger lives
inside Bot_RunPlayerMove*. A fire button is held, never edged, so the jump split spanning it across
two sub-commands changes nothing.

**IN_DUCK and IN_JUMP in one command is a long jump, not a crouch jump.** `PM_Jump`
(`pm_shared.c:2554`) reads `cansuperjump` from the `slj` physinfo key — set by `CBasePlayer` the moment
the player picks up `item_longjump` — and if the command carries IN_DUCK with `flDuckTime > 0` and
`Length(velocity) > 50` it replaces the jump with `forward * 350 * 1.6` horizontally and
`sqrt(2*800*56)` up. `PM_Duck` sets `flDuckTime = 1000` on the press with the comment *"Use 1 second so
super long jump will work"*, so the very frame a crouch jump presses both buttons is inside the window.
`PmSim` models `kJumpImpulse` and the commanded speed, so nothing in the plan knows about 560 u/s: the
bot flies roughly twice as far as its arc and lands past the target area. Nothing was wrong on maps
without the module, which is why it only showed up once bots started gathering on `crossfire`.

So a command that carries IN_JUMP drops IN_DUCK, but **only while the bot holds the module** — the check
is one `pfnGetPhysicsKeyValue` call, and gating on it keeps every map without an `item_longjump` sending
byte-identical commands.

Delaying the duck by one frame costs nothing, and the reason is the call order. `PM_PlayerMove` runs
`PM_Duck` (2982) *before* `PM_Jump` (3038), and a duck pressed while still on the ground cannot finish
inside that frame — `PM_FinishDuck` needs either the 0.4 s `TIME_TO_DUCK` ramp to elapse or
`onground == -1`, and the jump that clears `onground` happens after `PM_Duck` has already returned. So
the launch frame is simulated with the standing hull whether or not IN_DUCK was in it, and the first
airborne frame finishes the duck either way: pressed on the launch frame it finishes because `bInDuck`
survives into the air, pressed one frame later it finishes on the press itself. The hull timeline
`JumpLandsIn` assumes — `duckTrace` for the whole arc — is met from the same frame as before. Jump
*height* never depended on the duck either; both branches of `PM_Jump` give the same 45-unit impulse.

## bot_move_nav.cpp

### Constants
- kFullDrive: Over maxspeed on purpose: PM_CheckParamters rescales the forward/side pair down to maxspeed, so this just means "full speed in exactly the commanded direction".
- kAimVelocitySpeed: Below this the velocity carries no usable heading, so the aim holds where it is rather than snapping about on numerical noise.
- kEdgeProbeAhead / kEdgeProbeDrop / kStandHullMinsZ / kDuckHullMinsZ: `PM_Friction`'s edge test, copied literally — 16 units ahead along the 3D velocity, dropped 34 from foot height. They are the engine's numbers, not tuning knobs; changing one desynchronises `AtFrictionEdge` from the friction it is trying to predict.

### Airmove simulation cache

`MakePlanTraces` attaches the process-wide `g_airmoveCache` to every `PlanTraceContext` it builds, so all
plan-time `JumpLandsIn` calls share one table. The cache itself, its key, the world-only rule that decides
what may be stored and the sizing measurements all live in `doc/path_planning.md` under `airmove_cache.cpp`; what
belongs here is the wiring.

`Bot_ResetAirmoveCache()` clears the table. It is called from both `BotManager::ServerActivate` and
`ServerDeactivate`, alongside `BotSense::ResetBoneCache()` — that pair is the only per-map clear site in
the plugin, and anything holding map-scoped state has to hook it. There is no sizing cvar: the table is a
fixed member array sized by `kAirmoveCacheSlots`, so changing it means a rebuild. `MakePlanTraces` reads
`crossbot_airmove_verify` into `SetVerify` on every plan.

`Bot_EngineTraceFn` fills `PmTrace::hitEntity` for this: `tr.pHit` non-null, `pfnIndexOfEdict` non-zero,
not a `SOLID_BSP` brush flagged `FL_WORLDBRUSH`, and not accepted by `crossbot_nav::IsBakedBlocker`. The
first two are the `HitQualifies` idiom from `bot_avoid.cpp` minus the solid-class filter, which the
`ignore_monsters` flag on the trace already covers. The last is the shared predicate in
`nav/baked_blockers.h` that navgen uses to decide what gets baked into the mesh as permanent geometry —
runtime and navgen must agree on that list, which is why it is one function and not two.

### WriteTraceLog
Writes bot trajectory trace to file specified by crossbot_trace_path cvar (defaults to crossplay/addons/crossbot/trace/nav_trace.txt).
Verbose builds only. The `bot.traceLog` append in `Bot_MoveNav` is inside `#if BOT_VLOG_ON`, so in a
`-DCROSSBOT_VERBOSE=OFF` build the vector is empty and this returns at its first line -- no file, no
print. Every bot writes the same path, so the file is only meaningful with one bot on the server;
`test_nav_goal.ps1` sets `crossbot_trace_path` per instance and passes `-RequireVerbose`.

### Repath
Finds path from current area to goal area using A* and builds waypoint plan.
- Clamps goal point into goal area bounds.
- On fresh attempt, calculates total path distance and estimated arrival time.
- Logs waypoint details including jump/crouch flags and target area IDs.

Two different log lines, and the distinction matters when reading a failure. A fresh path logs
`Repath` plus every waypoint; a mid-route re-plan logs the one-line `Replan`, because that is the one
that fires after a failed air move and it would otherwise be silent. Both carry `penalized=`, the
count of connections on the chosen route with a nonzero `runtimePenalty` — so `penalized=0` on the
replan after a failure means the penalty did its job and the new route avoids the bad edge.

The fresh dump also prints one `conn[i]=<index> <type> <a><->b cost= travel=` line per connection on
the route. A route used to be visible only as areas and waypoints, and neither says *which edge* the
plan took: two areas can be joined by a walk, a jump and a ladder at once, the waypoints for them look
alike, and the connection index is what every penalty, abort and `wanted <area>` message is keyed on.
Both of test53_ladder4's ledge failures were only legible once the alternation could be read as conn 15
against conn 26 rather than as two similar-looking waypoint lists.

### Bot_MoveNav
Main navigation-driven movement function. Repath triggers are: empty path, forced (air move landed
off-target), and stall (no progress for kRepathInterval).

An air move or ladder that finishes in the wrong area replans in the *same* frame, not the next one.
The forced-repath latch is read at the top of the frame, above the air-move and ladder blocks, so a
landing that arms it would otherwise spend a frame driving on a plan the bot has provably left — and
`trackState.started` is cleared on that same frame, which makes the reprojection global and can snap
`progressS` anywhere on the stale profile. Both blocks therefore re-check the latch through
`TryRepath` once their runner returns.

- Uses NavFollow::Track against the `PlanProfile` built at repath time (see `NavFollow::Track` below). Progress is
  arclength along the arc chain, so there is no per-waypoint arrival radius or advance loop, and
  waypoint 0 is no longer a special case — the chain starts at the bot's own position.
- Drive magnitude is the servo's `wishSpeed`, divided by `kDuckingMultiplier` when `FL_DUCKING`
  (the engine applies 0.333 after the maxspeed clamp) and then clamped to maxspeed. This replaced the
  constant `kFullDrive = 1000`, which meant the bot had no speed control at all: it entered every
  corner at full speed and walked past the goal until an arrival radius caught it.
- Stall detection is rebased on `trackState.progressS` advancing, not on the waypoint index changing —
  a bot correctly braking to a stop at a hard node would otherwise read as stalled. At 2 s the timer is
  short enough that a maneuver which suspends `Track` has to hold the clock open itself: `RunLadderMove`
  and `RunAirmoveFlight` both stamp `lastProgressTime` every frame, because no arclength accrues while
  they are driving. Without that a long fall (up to `kAirmoveMaxAirTime`, 6 s) trips the stall timer
  mid-air and repaths from an airborne origin. The cost of holding the clock open is that a flight which
  never ends is invisible to every other timer, which is why the flight bounds itself — see
  `RunAirmoveFlight`.
- `Bot_MaxSpeed` returns the cap the *engine* will actually enforce on this bot, which is not any cvar.
  `PM_CheckParamters` computes it as `min(clientmaxspeed, maxspeed)` and writes it back to
  `pmove->maxspeed`, so `PM_Move_Post` reads the finished value out of the `playermove_t` and
  `Bot_NoteMoveMaxSpeed` caches it per edict index; `ClientDisconnect` clears the slot. `pev->maxspeed`
  then `sv_maxspeed` then `kMoveSpeed` remain as the cold-start fallback chain, for the frames before a
  bot's first `PM_Move`.

  Reading `sv_maxspeed` was wrong twice over. Under CrossPlay there is no server-wide ground speed at
  all: each hosted game sets its own before handing the move to the game DLL, so two bots in the same
  server legitimately have different caps. And even on one game the cvar is only half the `min` — the
  test server (`D:\hlds\crossplay`) never sets `sv_maxspeed`, so it read the engine default 320 while
  the valve DLL held players at 270. Every number the planner derives from `physics.maxSpeed` was then
  designed for a bot 50 u/s faster than the one flying it: on `test54_window` the launch was designed at
  299, the bot arrived at its real 270, `RunAirmoveApproach` failed `kAirmoveLaunchSpeedTolerance` (0.95)
  at 90% of design, penalized the connection and forced a repath — which planned the same launch again
  from a standstill. The bot ping-ponged between two 320-speed routes on the window ledge until the test
  timed out. Getting the cap right is not a tuning improvement; it is what makes the design speed and
  the achievable speed the same number.

  `Bot_MoveMaxSpeedKnown` exposes whether a bot's slot has been filled yet, so a caller that must not
  read the fallback can wait for the real value; `BotManager::CheckDriftSpeeds` is the one that does
  (`doc/navgen.md`, drift pruning).
- `Bot_Physics` reads sv_airaccelerate, sv_accelerate, sv_friction, sv_stopspeed, sv_edgefriction and
  sv_maxvelocity, so plan-time physics match a non-default server. Only the first three were read
  before, which silently diverged the profile from the engine. It does *not* read `sv_gravity` — the
  800 is `PmSim::kGravity` (see `doc/games.md`) — but it does multiply by `pev->gravity`, which is what
  `trigger_gravity` writes onto the player.
- `Bot_PushFields` snapshots the live `trigger_push` entities into a `PmSim::PmPushFn` once per plan,
  so `JumpLandsIn` simulates a launch through a push field the same way navgen sampled it. A field
  that is `SOLID_NOT` (START_OFF, or toggled off) is skipped; the next repath picks up the change.
- `Bot_DriftVelocity` is the steering counterpart: the world-frame shove acting on the bot right now,
  read live as `groundentity->v.movedir * speed` when the ground entity carries `FL_CONVEYOR`, plus the
  cached push closure at the bot's origin. It goes into `NavFollow::Config::driftVelocity`, which is what
  stops a bot on a conveyor or in a current being carried off its own plan — see `Following a drifting path` below. The closure
  lives on `CBot::pushFields` and is refreshed with each repath, because scanning every edict per bot per
  frame is not free.
- `sv_maxvelocity` matters only where a push field is involved: `bounce`'s pods push at 3800 u/s
  squared, which runs the simulated velocity into the engine's clamp within a second.
- Jump waypoints are commitments — not passed until the jump is made, and gated on on-ground,
  distance and facing tolerance, with duck added while rising.
- Aim leads the next waypoint rather than the current one, so the bot looks around corners; the move
  direction comes from the steered direction, not from where it is looking.

### RunAirmoveApproach / RunAirmoveDrive
The steering that takes over once the tracker's `wpIndex` reaches an air-move launch waypoint. It aims
`kAirmoveApproachLead` past the launch along the launch heading and commits when the bot is within one
frame of the launch plane **and is up to speed**.

The lead is `NavPlan::AirmoveLaunchLead` — `speed * frametime`, one frame of travel, shared with the
planner so both sides agree on where the bot stops accelerating. It used to carry a flat
`kAirmoveLaunchSlack` of 4 units on top, which bought nothing and cost every jump 4 units of run-up;
on `test37_walkent`, whose approach only has 19.5 units of straight, that was most of the margin.

**A launch under `kAirmoveLaunchSpeedTolerance` of its design speed is not taken.** `PmSim`,
`JumpLandsIn` and the sampler all begin their simulation *at* the design speed, so a launch delivered
short is a jump the plan never modelled. Nothing checked it, and the gap was systemic rather than
per-map — `qconsole.log` showed 222 planned against 146 delivered, repeatedly, and the launches that
matched their design speed were the ones that landed where they were aimed. `test37_walkent` crossed
the plane at 239 of 270 and fell 274 units into the pit it was jumping over.

When the bot reaches the plane still short, it penalizes the connection, requests a repath and brakes
(`RunAirmoveDrive` with drive 0 solves to full reverse thrust) instead of jumping. The bot pays one
approach to learn the edge is undeliverable, and the +5 s penalty routes the next plan around it —
the same penalize-and-repath loop a bad landing already used, moved earlier so the lesson costs an
abort rather than a fall.

**Before blaming the tolerance, check that the follower modelled the ledge it is standing on.**
test53_ladder4's crouch jump (conn 15, area 4 → 2) aborted at 94 % of design on *every* run, including
runs that got the full planned run-up, and 94 % misses `kAirmoveLaunchSpeedTolerance` by one point. The
tolerance was right and the profile was right: the bot arrived at the launch plane holding 66.4 against
a design 70.8, and held it — flat, frame after frame, not still decelerating. A servo that settles a
fixed distance short of its setpoint is a gain problem, not a margin problem.

The gain came from `SolveDrive` being handed `sv_friction` when the engine was applying
`sv_friction * sv_edgefriction`. `PM_Friction` probes 16 units ahead along the velocity, from foot
height, 34 units down, and doubles friction when that finds nothing — which is precisely the state
every air-move approach ends in, since it is walking at the lip it is about to launch from.
`NavFollow::SolveDrive` (below) derives the residual: a flat `V* - stopSpeed * (f_real - f_model) * dt`, invisible above a
design speed of 80 and unavoidable below it. Crouched approaches are capped near 90, so they are the
ones that live under the threshold.

`BotDrivePhysics` runs the same probe the engine runs — same 16/34 offsets, same hull, same
`fraction == 1` test — and doubles `groundFriction` when it misses, feeding that to every ground
`SolveDrive`: the tracker's, the air-move approach's, and the ladder, elevator, activation and charger
drives. `BotPhysics` stays untouched, so `MakePlanTraces` and `BuildPathProfile` keep planning against
open-ground friction. Do not merge the two: a plan whose design speeds depend on whether the bot
happened to be standing at a ledge when it replanned would move its own waypoints under it.

With the probe in, the same launch reads `actual yaw 105 speed 71 (100%)` and lands in area 2, and the
map stops taking the ladder detour it had been falling back to.

Two things about the drive are easy to get wrong and both stalled real routes:

- **The command is a target velocity, not a wish speed.** `RunAirmoveDrive` puts `drive` through
  `NavFollow::SolveDrive` while on the ground, exactly as `Track` does. Passing `launchSpeed` straight to
  `pfnRunPlayerMove` looks equivalent and is not: below `sv_stopspeed` the engine's friction drop is
  constant, so a raw wish speed under `stopSpeed * friction * edgeFriction / accelerate` moves the bot
  nowhere at all. At a ledge — which is where every dropoff launches from — `sv_edgefriction` doubles
  that to 80 at defaults. A dropoff planned at 57 pressed at full drive and crawled at 2 u/s until the
  stall timer repathed it, forever — see `NavFollow::SolveDrive` below.
- **Ducking scales the command, so the command has to be scaled back.** `PM_Duck` multiplies
  forwardmove/sidemove by `kDuckingMultiplier` *after* `PM_CheckParamters` has clamped them, so
  delivering wish speed `W` while `FL_DUCKING` means commanding `W / kDuckingMultiplier` and accepting
  that nothing above `maxspeed * kDuckingMultiplier` is reachable. `Bot_MoveNav` did this and
  `RunAirmoveDrive` did not, so every crouched air move was driven at a third of its planned speed.

**`RunAirmoveFlight` steers, and the wish-speed cap is why it can.** The flight used to re-issue the launch
heading unchanged every frame, on the grounds that `PM_AirAccelerate` caps wish speed at 30 and the
trajectory was already committed. That reasoning holds for one axis only. The cap is on the *projection of
velocity onto the commanded direction*, not on total authority:

| Commanded direction | `currentspeed` | `addspeed` | Authority |
|---|---|---|---|
| Along the current heading, 270 u/s | 270 | −240 | **Nothing.** `addspeed <= 0` returns before touching velocity. |
| Perpendicular to velocity | ~0 | 30 | **~30 u/s**, reached in a few frames — ≈60 units of shift over a 2 s flight. |
| Opposite velocity | −270 | 300 | **~53 u/s per frame**; here `airAccelerate * wishspeed * frameTime` binds, not the cap. |
| Perpendicular, re-aimed each frame | ~0 | 30 | `sqrt(270² + 30²) − 270` = **+1.66 u/s per frame**, ≈100 u/s per second. |

So the old command was not a weak correction, it was exactly zero — every frame, on every nominal flight.
The first row is what it did.

The last row is the one that is easy to miss. Perpendicular to the *launch* heading buys nothing: both
sides only ever add to the lateral axis and the forward component never changes. Perpendicular to the
*current* velocity is the ordinary strafe gain, and alternating the side returns the heading while keeping
it — `(270, 0)` → `(270, 30)` → `(273.3, 0.2)`, with about half a unit of lateral excursion per half
cycle. Along-track undershoot is therefore correctable too; it just needs the wish direction re-derived
from live velocity rather than frozen at launch.

**The target is the planned X-Y at time `t`, not the landing point.** In flight `z` is a function of time
alone, so a bot that speeds up to arrive early arrives *high* and sails over the ledge it was aimed at.
Matching X-Y at the planned time matches the whole 3-D arc for free — and it is why only X-Y is ever
compared, which in turn is what lets the reference live in the planner's duck-origin z-space while the bot
flies in world z.

`AirmoveCorrection` builds the demand as a feed-forward plus a proportional term that closes the gap by
landing, `want = refVel + err / remaining`, and splits `dv = want - velocity` about the current velocity:

1. `|dvPerp| > kAirmoveCorrectMinDelta` → **steer** along `dvPerp`.
2. `dvAlong < -kAirmoveCorrectMinDelta` → **brake** along `-vHat`.
3. `dvAlong > kAirmoveCorrectMinDelta` → **micro-strafe**, `±perp(vHat)` with the side alternating each
   frame via `airmoveStrafeSide`.
4. otherwise → the launch heading at the launch speed, exactly as before.

Order matters. A mixed error whose forward component dominates would normalize to a nearly-forward wish
direction, and row 1 of the table says that does nothing *at all* — including nothing for the lateral part
riding along with it. Taking `dvPerp` first is what keeps the achievable component achievable.

`drive` is sized to the need rather than pinned at full: `need / (airAccelerate * frameTime)`, clamped to
`maxspeed`. Braking is one-way — air acceleration can never give speed back along the direction of travel —
so an over-brake is a permanent undershoot, and full drive would shed 53 u/s in a single frame.

On plan, `err` is zero and `velocity` equals `refVel`, so `dv` is zero and branch 4 fires. The correction is
a function of error only, which is also why the planner stays valid: it modelled the open-loop arc, and a
zero-error flight still flies it.

**The reference is simulated at launch, and checked before it is trusted.** `Bot_SimulateAirmoveArc` runs a
`PmSimulator` from the bot's real launch state with the *planned* yaw and speed, sampling every
`kPlanSimFrameTime`; `Bot_AirmoveArcAt` interpolates it by air time. Three things about that are load-bearing,
each of which was a bug first:

- **Seed from the bot's actual origin, not `wp.launchSimFrom`.** That field records where the *first*
  simulation started, and `Respeed`/`Retract` move the launch point and rewrite `launchSimTo` without
  rewriting it. Re-simulating from the stale seed put a test5_jump reference 102 units from the planner's
  own landing, and the bot flew to it — turning an 18.9-unit miss into a 77.9-unit one in the wrong area.
- **Do not break out of the step loop until the mover has actually been airborne.** A drop is still
  `OnGround` for the first few frames while it walks off the edge, so breaking on the first grounded frame
  ends the arc after 0.02 s. Only a jump is airborne from step one, which is why the opportunist's original
  loop never hit this.
- **Validate the endpoint against `wp.launchSimTo`** within `kAirmoveArcAgreeDist`, and drop the arc when it
  disagrees. `launchSimTo` is an output, so unlike `launchSimFrom` it is always current. A rejected arc costs
  nothing: `AirmoveCorrection` returns false and the flight is the open-loop one it always was.

Mid-flight, an error past `kAirmoveCorrectAbandon` clears the arc for the rest of the flight. Past that the
reference has stopped describing this flight — the bot was knocked, or hit something the sim did not — and
steering hard toward a stale target is worse than coasting. One bounce flight showed the failure it prevents:
a peak error of 264 units, corrected for 96 frames, landing 675 units out.

`crossbot_airmove_correct 0` disables the whole thing and restores branch 4 unconditionally.

**A landing outside `airmoveTargetArea` penalizes the connection, then forces a repath.**
`RunAirmoveFlight` resolves the landed area and calls `Nav::PenalizeConnection` on the index latched at
launch, or `RewardConnection` when it landed where it was aimed (see `doc/path_planning.md`, "Runtime connection penalties"). Without that, a
maneuver the follower cannot execute was an unbounded loop: both repath triggers re-plan from the
bot's current position with the full graph, nothing recorded the failure, and the new start position
could flip which route was cheapest — so the bot alternated between two routes into the same dead end
and `actual=` grew as `[11,12,13,11,12,13,...]` for the whole run.

test50_stairs is the map that motivated it, and it shows why this is not a planner or a mesh bug.
Jumping up a staircase is roughly a coin flip: a clipped riser eats all forward speed and throws the
mover sideways. The plan is valid and the sim proves the arc — but the follower launches from the same
point at the same yaw and speed every time, so an edge a human clears half the time fails every time
for the bot. With the penalty disabled the run is a clean 2-cycle, conn 38 → replan → conn 35 → replan
→ conn 38, until the test times out at 12 s; with it the bot penalizes both and walks up instead,
arriving in ~2 s.

An oscillating `actual=` list therefore now means the penalty has not caught up yet — one more failure
per bad edge — or that every remaining route is bad. It no longer means "no memory".

**The flight ends when the bot has settled, and `FL_ONGROUND` is not the only way to settle.** Two
states end an air move with the ground flag never set again: swimming, because `PM_CategorizePosition`
forces `onground = -1` at `waterlevel >= 2`, and a ladder, because `PM_LadderMove` puts the player on
`MOVETYPE_FLY`. An air move that ends in water or against a ladder therefore used to hold
`airmoveActive` until the flat 6 s `kAirmoveMaxAirTime`, and because `RunAirmoveFlight` stamps
`lastProgressTime` every frame the 2 s stall timer could not break it either — the bot swam or hung on
the yaw it launched at, and the only thing that ever repathed it was that timeout. `AirmoveSettled` tests
all three, and `AirmoveEndReason` names which one so the log line distinguishes `landed`,
`splashed down` and `caught a ladder`.

**The timeout is the connection's own flight time, not a flat 6 s.** `AirmoveTimeLimit` latches
`conn.flightTime * kAirmoveTimeFactor + kAirmoveTimeMargin` at launch (capped at `kAirmoveMaxAirTime`,
and the flat cap still applies when the connection carries no flight time), so a 0.4 s hop that goes
wrong is not worth six seconds of driving on a plan the bot has left. The limit is *suspended while the
bot is still falling* — `velocity.z < kAirmoveFallingZ` — because a bot in free fall is going to answer
the question itself in a moment, and a fall deeper than the plan modelled is exactly the case whose
verdict is worth waiting for. What the limit catches is the bot that is neither falling nor settled.

**A verdict is only scored when the bot settled *inside an area*.** The lookup is the strict
`Nav::FindAreaExact`, not `FindAreaNearest`. The forgiving one cannot fail: with no containing
area it falls back to any area whose AABB holds the point, and failing that to the globally nearest
centroid — so a bot that ended in the void, in mid-air or on a lip outside the mesh was still scored, and
could be scored as the area it was aiming at. `FinishLadder` was moved off it for exactly this reason and
the air move was left behind. Settled-but-in-no-area is now `undecided`, like the timeout: neither
penalized nor rewarded, but repathed, because the plan is void either way.

**The verdict is the distance to the plan's landing waypoint; the area is the fallback.** Areas are large —
crossfire's area 66 is 36 × 388 units — so an area-only verdict passes a landing hundreds of units from the
modelled one, and at an area seam it fails a landing that was perfect: two of four "failures" in one
crossfire run were `miss 0.3` and `miss 0.5`, the bot exactly where the plan put it while the strict lookup
resolved the neighbour. So `failed` is `miss > kAirmoveLandingTolerance`, and the area comparison is used
only when there is no landing waypoint to measure against. `miss` is 3D against
`waypoints[waypointIndex]`, which after the launch's `++waypointIndex` *is* the plan's landing waypoint.

Two things about that measurement are easy to get wrong and both produced garbage before the sweep caught
them:

- **Measure against the plan's waypoint, never `conn.landingPos`.** The connection's landing is what
  navgen's sampler recorded for the edge; the planner re-derives the arc per route from the funnel's launch
  point. The two differ by 65 to 134 units on landings whose flight time matched plan to a hundredth and
  which landed in the wanted area — nine good connections penalized in one sweep.
- **Waypoint z is origin space, not `BotNavZ` space.** `BotNavZ` exists for elevator nav-z comparisons and
  subtracting `kPlanStandHeight` here put a systematic ~18-unit floor under every miss. The log settles it:
  landing waypoint `(-436.6, 841.9, -1626.2)` against an actual landing origin of `(-438.1, 843.5, -1628.0)`.

With both fixed the measurement is tight enough to set a threshold from: 0.2–8.2 units across the whole
`maps/` sweep, 0.2–14.0 across a two-bot crossfire run, and the failures in that same run at 93, 100 and
155. `kAirmoveLandingTolerance` is 48 — three times the worst good landing, half the cheapest bad one.

`airmoveOverrunLogged` prints one line — not one per frame — the first time a flight passes its planned
time by `kAirmoveOverrunGrace`, carrying `onground`, `waterlevel`, `movetype`, `FL_DUCKING`, velocity and
the resolved area. A flight that hangs is hard to reproduce and the exit line alone cannot say *which*
of those states it was stuck in.

A repath inside an air move's run-up leg makes this worse in a way worth recognising. `Repath` starts
the plan at the bot's own position and `Cross` then places the run-up node a fixed distance back along
the launch heading, so a bot that has already closed most of that distance gets a first waypoint
*behind* it and walks backwards to reach it. The plan is not wrong — it is the same plan it would
build from anywhere — but the bot arrives at the run-up node with no speed, which is the worst state
to start a launch that has to build speed at a ledge. Two waypoints at the same 2D point at the head
of the list in `crossbot_nav_status` are the tell.

### Ladders

`RunLadderApproach` walks the bot to the mount and latches `CBot::ladderActive`; `RunLadderMove` then
owns every frame until it finishes, exactly as the air move does. Eight rules make the difference between
a climb and the jump-on-jump-off loop test53_ladder4 used to run. A ninth is the descent aim, and a tenth
is how the attempt is scored; both are below. The last two are about giving up on purpose: a shaft
something is standing in, and a bot that gets shot on the way.

**Drive from the plane the engine picked, not the face navgen planned.** `PM_LadderMove` chooses its
plane with `PM_TraceModel(origin -> ladder centre)` in the *point* hull, so the face is decided by where
the bot actually stands. When that disagrees with the planned face, `dot(velocity, N)` is ~0, `lateral`
collapses to the raw view vector, and the bot flies horizontally off the ladder at 200 u/s.
`ProbeLadderPlane` repeats the engine's own trace every frame and every aim is derived from its result;
`LadderPlane::why` records which of `no-entity` / `startsolid` / `allsolid` / `no-hit` / `degenerate`
rejected a probe. With a valid plane every motion is reachable with `IN_FORWARD` alone plus view angles —
climb faces `-N` at pitch 0, descent faces `-N` at pitch +89, sideways faces `±perp`. A degenerate plane
is not a failure to route around: velocity is then exactly `vpn * 200`, so the fallback simply aims pitch
∓89 and rides that.

**A ladder mount bars the profile, like a ride does.** Mount to exit is near-zero in XY, so
`NavFollow::Track` sails straight through it and `waypointIndex` reaches the exit while the bot is still
short of the volume. `RunLadderApproach` is then never called for the mount, and the moment the bot
drifts onto the ladder anyway the `ladderEject` rule sees no ladder waypoint in range and jumps it back
off at 270 u/s — forever. The mount therefore sets `NavFollow::Config::sLimit` while
`i >= bot.waypointIndex`, released by the `++bot.waypointIndex` that commits the climb.

**Never duck on a ladder while `FL_ONGROUND`.** `PM_CategorizePosition` treats the player as airborne
only above `velocity[2] > 180` (`pm_shared.c:1542`); below that it traces two units down and, if it finds
floor, snaps the origin back to it. A ducked climb is `MAX_CLIMB_SPEED * 0.333 = 67`, under the
threshold, so at the foot of a ladder the bot is re-glued to the floor every frame: correct plane,
correct aim, `vel=(0,0,67)`, and an origin that never leaves z 18-20. Only the two `[crouch]` ladders in
test53_ladder4 stalled, and only at their mounts. `RunLadderDrive` drops `IN_DUCK` while on a ladder and
on the ground; the bot detaches in one frame at 200 and re-ducks in the air, where the two-unit trace no
longer reaches. `PM_UnDuck` refuses to unduck into a ceiling, so a genuinely low mount degrades safely.

**Never hold `IN_JUMP` while `MOVETYPE_FLY`.** `PM_LadderMove` reads the jump button before anything
else and answers with `movetype = MOVETYPE_WALK; velocity = normal * 270` — one frame of jump ejects the
bot off the ladder at 270 u/s. The reach-up hop that grabs a dangling rung is therefore gated on
`!onLadder`, not just `FL_ONGROUND`: at the foot of a ladder that reaches the floor the bot is on the
ladder *and* on the ground at once, so the ungated version hopped on, ejected, walked back and repeated
forever. `avoid.jump` is suppressed the same way, and `RunAirmoveDrive` drops the button outright when
the mover is on a ladder, so a flight that ends on one cannot bounce off it.

**A ladder need not reach the floor, and reaching one costs exactly one frame of jump.** test53_ladder4's
`*2` brush stops 62 units above the floor of its shaft: hull 1 expands it to `z >= -212` and a standing
bot's origin is at -220, so no amount of walking touches it. `LadderOutOfReach` detects that, waits
`kLadderReachDwell`, and hops. What made the hop framerate-dependent was the engine re-simulating the
held button mid-flight — see `Bot_RunPlayerMove`, which is why the hop is now a 50 ms command and the
rest of the frame is jump-free. The contact sub-step then finds `IN_JUMP` clear, and the bot grabs at the
top of its arc instead of being thrown back down the shaft.

**Aim before driving.** `PM_LadderMove` builds its wish from the *buttons*, so `IN_MOVERIGHT` is a full
`±MAX_CLIMB_SPEED` sideways input no matter how small the `smove` that set it. A bot still turning toward
its target yaw is therefore commanding a large wrong-axis vector, and the ladder converts wrong-axis
input into the wrong motion: perp becomes pure horizontal at 200, into-the-face becomes +200 climb.
A descent that committed while still facing its run-up yaw climbed straight out of the top of the ladder
and walked off the ledge — `caught the ladder` then `fell off` 0.4 s later, 61 units sideways, twice per
run. Every on-ladder phase now zeroes `drive` while the yaw error exceeds `kLadderAimTolerance`; with no
buttons `PM_LadderMove` clears the velocity, so the bot simply hangs there and turns.

**Latch the dismount phase.** The height test that switches from climb yaw to dismount yaw sits at a
single z, so a bot hovering on it alternated targets every frame and the aim gate held `drive` at 0 for
both — a 12.7 s climb that should take 2 s. `ladderAtTop` is set once and never cleared before the next
commit.

**A climb approach aims past the mount.** `pointA` is deliberately outside the carved volume, so
steering *to* it stops the bot short of the capture region — for a ladder whose volume starts two units
beyond the stand point that is a permanent stall, jumping in place at full drive. The approach aims
`kLadderMountLead` further along `ladderYaw`, i.e. into the ladder, and lets the capture happen on the
way through.

**A descent approach aims past the mount and never turns back.** A climb has `ladderYaw` to steer by; a
descent has nothing equivalent, because the bot cannot grab the ladder from where it stands. `PM_Ladder`
is a *point* test of the origin against the brush's hull-3 volume, and where a ladder ends flush with the
ledge it serves, that volume tops out at exactly the walkable duck-origin z — test53_ladder4's `2 <-> 4`
ladder ends at 64, the ledge floor is 64, and both put the top of the volume at z 82 with the bot standing
on z 82. Nothing catches until the bot walks off the ledge and starts to fall, so the descent aim exists
to carry it over the edge, `kAirmoveApproachLead` beyond the mount along the line it is already on.

Deriving that line from `origin -> mount` inverts it the instant the bot passes the mount, and the guard
that swapped in `ladderEntryYaw` within eight units only moved the inversion to the edge of that ball:
the bot settled on the shell `|origin - mount| = 8` on the far side and hovered there at ~16 u/s, drifting
in and out, until `kLadderStallDist` gave up. It needed y 272 and the shell held it at y 270. The branch
is now chosen by the *signed* progress along `ladderEntryYaw` — `dot(toMount, entryDir) >
kLadderMountApproach` still steers through a mount genuinely ahead, level or behind drives on `entryDir`
alone — which is the projection `RunLadderApproach` already commits on. Every origin the old
`never caught` lines printed was 7.9-8.1 units from the mount; that number was the entire bug.

Because both ways out of that ledge failed, the failures alternated: the crouch jump aborted under speed
and penalized its connection, the next plan picked the ladder, the ladder never caught and penalized
*its* connection, and the plan came back to the jump. Nothing was oscillating in the planner — penalties
were working exactly as designed, on two edges that were both genuinely broken. A repath that keeps
flipping between two routes is worth reading as a symptom of that, not as a planner-stability problem.

**The verdict judges the maneuver, and only then the area.** `FinishLadder` decides whether the
connection earns a `RewardConnection` or a `PenalizeConnection`, and it used to decide entirely on
`FindAreaNearest(origin)`. That function never returns NULL — after the exact half-space test it falls
back to nearest-centroid-with-AABB and then to plain nearest centroid — so it always names *an* area, and
where the ladder shaft is carved out of the mesh the nearest centroid is routinely the area the bot just
left. test53_ladder4's `2 <-> 4` descent ends at (442.7, 272.8, 19.7), inside no area at all: area 2
starts around y 280, and the answer came back as area 4, the ledge 62 units above. A descent that worked
was charged `kConnFailPenalty` every single time, and the next `Replan` reported `penalized=1` because
of it.

So the verdict now takes two inputs. `FinishLadder`'s `LadderOutcome` says whether the maneuver itself
finished as designed — `reached the bottom of` and `dismounted off` pass `kLadderOk`, `never caught`,
`fell off`, `timed out on`, `stalled on` and `blocked on` pass `kLadderFailed` — and a maneuver that did
not complete is a failure whatever the geometry says. Only when it did complete does the area matter,
and then it is read with the strict `Nav::FindAreaExact`: the target area rewards, a *different* area
penalizes, and no area at all is `undecided` — neither rewarded nor penalized, because a bot still in the
shaft is not evidence of anything. The third outcome, `kLadderAbandoned`, is the bot choosing to leave
(see the last rule) and is scored exactly like `undecided`. The log line carries one of `ok`, `failed`,
`undecided` or `abandoned`, so a run tells you which happened.

Deferring the undecided case instead — hold the verdict, settle it once the bot is back on the ground
and strictly inside something — was tried and is worse. By the time the bot satisfies that test it has
walked on: test53_ladder4's `11 <-> 8` descent settled two areas later, in 13 rather than the 11 it was
promised, and penalized a ladder that had just worked. Deferring only moves the wrong answer. Nothing is
lost by declining to answer, either, because `RewardEnteredConnection` still decays the penalty when the
bot actually walks into the planned next area, and that is the reward path that matters.

`RunAirmoveFlight` scores its connection the same way and has the same forgiving lookup. It is far less
exposed — its verdict fires once `AirmoveSettled`, so the bot is on a floor, in water or on a ladder —
and the timeout, which is decided in mid-air, is the same trap and so declines to score at all.

**Stalls are measured in distance, not speed.** A bot wedged against the brush can hold a healthy
`vel=(0,0,67)` forever while its origin does not move, so the detector compares against
`ladderStallOrigin` and gives up after `kLadderStallDist`. Two escapes are allowed, each a push-off;
the third gives up on the connection and repaths.

**Look where the climb is going, and count only entities.** The stall detector finds a blocked shaft
eventually, and badly: with a `cycler` sitting on top of the ladder, the climb's head meets its bottom,
two push-offs drop the bot back down the shaft, and the second landing is scored `dismounted off` —
a completed maneuver — so the penalty only arrives because `FindAreaExact` happens to name the pit.
`LadderBlocked` instead casts one hull trace a frame, at most `kLadderProbeAhead` long, along where the
current phase is heading: straight up to `ladderDismountZ` on a climb, straight down to it on a descent,
at `ladderExitPoint` once `ladderAtTop` has latched, and at the mount itself on the approach. The same
entity has to stay in the way for `kLadderBlockDwell`; then the climb ends `blocked on` with
`kLadderFailed`, which penalizes `ladderConnIndex` and repaths. On the approach nothing is latched yet,
so the mount's own `wp.connIndex` takes the penalty and the frame goes back to the walker.

What counts is `Bot_TraceHitEntity` — the predicate `Bot_EngineTraceFn` already uses, which drops the
world, world brushes and `IsBakedBlocker` classnames — minus live players. The world has to be out:
a shaft is a hull-wide tube of world brush, and a vertical probe that counted walls would abort every
good ladder on the first frame its hull grazed one. Geometry that genuinely blocks a ladder is navgen's
mistake and is what `never caught` and `stalled on` already catch. Players have to be out too: a
teammate standing at the top is gone a second later, and the penalty is mesh-global and long-lived.
The approach probe only runs inside `kLadderCommitRadius`, because the walker's `BotAvoid` is not
running during a ladder approach — a probe that fired at 200 units would penalize a can the bot could
have stepped around, where one at 48 is saying the mount itself is occupied.

test52_avoid is the map for it. The target pocket is only reachable by climbing out of a pit, the pit
has two ladders, and a `cycler` sits on top of the `y 80..112` one. The run reads: `ladder climb ... conn
16`, `ladder path blocked by cycler at (640,96,0) for 0.41s`, `blocked on ... conn 16 failed`, a
`Replan ... penalized=0` that now goes the other way, `ladder climb ... conn 17`, `dismounted off ...
conn 17 ok`, `reached goal`. Until navgen stopped starting that ladder's dismount exactly on a clip plane
and stopped merging two ladders into one edge (both in `doc/navgen.md`, "A sample that finds nothing is
retried" and "Two ladders are two connections"), the blocked ladder was never a connection and the can
was never met.

With no second way up the penalty cannot help. On a copy of the map with the can moved onto the other
ladder, the next plan is the same ladder, the bot is still on it and re-commits on the spot, and the
cycle repeats every 0.4 s while the penalty climbs by `kConnFailPenalty` each time. That is the penalty
working as designed — a price, never a ban — on an edge with no alternative, and nothing above the
ladder gives up on the goal.

**Being shot ends a climb, unless the fall would kill.** Combat enters on the damage frame but
`BotCombat::Run` waits for `Bot_ManeuverActive`, and a bot that finishes a 144-unit climb while being
shot in the back is a free kill. So `Bot_MoveNav` calls `Bot_BailManeuver` right after
`BotCombat::Update` whenever combat is active, `BotDiff::CombatInterrupt()` would let `Run` drive, and
`BotCombat::DamageSource` names an attacker this frame — world damage, falls and the bot's own rockets
resolve to nothing there, so they never cause a bail. The bail estimates the drop the eject implies —
back down to the mount on a climb, down to `ladderDismountZ` on a descent — prices it with
`PmSim::FallDamageFor`, and refuses only when that damage would be at least the bot's health. A fall
that hurts is still cheaper than hanging on a ladder under fire.

It does not clear `ladderActive`. It sets `ladderBail`, and `RunLadderMove` drives the existing
`kLadderPushOff` aim until `OnLadder` goes false — one frame of `IN_JUMP` in practice, measured at
`vel=(270,0,-8)` the frame after the button — and only then finishes `bailed off` with
`kLadderAbandoned`. If the bot is still on the ladder `kLadderStallTime` later it finishes `could not
bail off` anyway and lets combat drive from where it hangs. That is not hypothetical: a player whose
hull overlaps a solid is frozen by `PM_CheckStuck` before `PM_LadderMove` ever reads the jump, and a
bail without that bound held `IN_JUMP` into the wall until `kLadderMaxTime`. Clearing the state outright would hand the frame to
combat with the bot still in `MOVETYPE_FLY`, and the `ladderEject` rule that would have kicked it off
only lives on the walker path that combat bypasses. `kLadderAbandoned` neither rewards nor penalizes:
the ladder was fine, and pricing it would feed combat noise into every bot's routing.

### Ladder debugging

`bot_move_nav.cpp` carries a per-frame ladder trace behind `#ifdef CROSSBOT_LADDER_DEBUG`, off by
default. Turn it on with

```
build.bat -DCROSSBOT_LADDER_DEBUG=ON
```

(the value is cached in `build/CMakeCache.txt`, so plain `build.bat` keeps whatever was last set). It
prints to the server console, which the test harness already captures at
`nav_debug/logs/<map>_server.log`, and it is emitted only while the bot is in the ladder pipeline. The
macro compiles its arguments away when the flag is off.

Lines read `CrossBot: LDBG <bot> t=<time> <phase> key=val ...`, one per frame per phase:

| phase | what it shows |
| --- | --- |
| `approach` | the whole pre-commit state: mount, dist, remaining, launch lead, computed aim, `blocked`/`reachUp` |
| `past` | the approach handed back to the walker because the bot is already beyond the dismount height |
| `entry` | pre-catch, with all three "never caught" predicates evaluated separately |
| `state` | plane validity and reason, normal, chosen step, dismount dots, stall age and displacement |
| `aim` | target vs current angles, the error, and whether `drive` was gated by turning or commanded off |
| `drive` | `fmove`/`smove` and the decoded button names actually sent to `pfnRunPlayerMove` |
| `after` | position and velocity immediately after that call, so one frame's cause and effect sit together |
| `exit` | off-ladder, seeking the exit point |
| `eject` | the walker's `OnLadder` branch: which of the next three waypoints are ladder waypoints, and whether `ladderEject` fired |
| `walk` | the walker's own actuation whenever a ladder is involved — the only other place `IN_JUMP` can reach the bot |

`drive`/`after` and `eject`/`walk` between them cover every button the bot can be sent, which is what
makes "who pressed jump" answerable rather than a matter of reading gates.

### Elevators

`RunElevatorApproach` / `RunElevatorRide` / `FinishElevatorRide` are the ladder trio again, but the hard
part is not the driving — it is that **the ride is invisible to the profile**. The planner's boarding and
arrival waypoints are the mover's column centre at two heights (`elevators.cpp` builds both from
`centerX`/`centerY`), so they differ only in Z. `BuildPathProfile` measures `Dist2D`, gets zero, and never
pushes the segment; `ProjectOntoProfile` skips zero-length segments outright. The `gap` flag `IsGap` sets
for a ride lands on a segment that does not exist.

Three consequences, all of which had to be answered separately:

**The tracker walks through the ride.** With `projectAhead` of 64 units and both ends at the same XY, the
bot aims at the far-side corridor while standing at the near-side stop. `NavFollow::Config::sLimit` already
exists for the doubled-back activation detour, which is the same failure in weaker form, so a *pending*
ride sets the barrier too. Pending means the flag is still set: `FinishElevatorRide` clears
`requiresElevatorRide` on the bot's own copy of the waypoint rather than remembering an index, because
`SpliceDetour` rebuilds `bot.waypoints` with a shifted base and an index would not survive it. No memo is
needed the way activations need one — a repath from the far stop plans a route with no ride in it at all.

**Reprojection cannot be global.** The ladder can clear `trackState.started` when it finishes because its
two ends differ in XY. A ride's do not, so a global reprojection is free to resolve the tie in favour of
the *lower* `s` and put the bot back on the approach. `FinishElevatorRide` sets `trackState.progressS` to
the arrival waypoint's `ProfileWaypointS` by hand and leaves `started` true; the incremental window then
reaches at most `projectBack` behind, where the profile is 32 units away and cannot win the tie.

**A ride is only scored when the bot settled *inside an area*.** `FinishElevatorRide` grades its
connection exactly as `FinishLadder` and the air move do, and for a long time it was the odd one out: it
read the landing with `FindAreaNearest`, which never fails. An elevator column is precisely where two
areas overlap in AABB, and a bot standing on the mover's brush is routinely outside every area's exact
planes, so a ride that ended on the wrong floor snapped onto a plausible neighbour and a ride that never
arrived earned `RewardConnection`. It uses `FindAreaExact` now, with the same three-way verdict the other
two use: the target area rewards, a different area penalizes, and no area at all is `undecided` —
neither, because a bot the mesh cannot place is not evidence about the connection. Undecided still
repaths, since the bot no longer knows where it stands relative to the plan.

**But the exact lookup has to be handed a constructed point, not `pEdict->v.origin`.** Switching to
`FindAreaExact` alone made every *correct* arrival report `undecided` and repath, for two independent
reasons that each hide the other:

- **The origin is in stand space, the mesh is in duck space.** Areas are built in hull 3, so their z
  range is duck-origin z — `BotNavZ` exists for exactly this conversion. That is harmless for an
  ordinary area, which spans floor to ceiling (`test18_elevator` area 1 is `z 18..110`, and a standing
  origin at 36 is well inside), and fatal for an elevator stop, which is a carved slab two units tall:
  `test18_elevator` area 7 is `16..18`, and crossfire's top stops 11, 154 and 188 are `-1520..-1518`.
  A standing bot's origin sits 18 units above that, past `kAabbEpsilon`, so the raw origin can
  *never* resolve a top stop. On crossfire the bot stands at `-1500`, `BotNavZ` is `-1518`, and the
  slab is `-1520..-1518`.
- **The ride ends up to `kElevatorArriveSlack` short.** On `test18_elevator` it ends at nav z 8.6
  against a stop at 16, and a two-unit slab does not reach down 7.4 units even with the AABB epsilon.

So the probe is the bot's XY at `rideArriveZ` whenever the residual is inside the slack, and at
`BotNavZ` otherwise. The snap condition is the arrival predicate reused: the slack that is allowed to
*end* the ride early is the slack that is allowed to *judge* it at the stop. A ride that stalled
mid-shaft is hundreds of units from `rideArriveZ`, gets no snap, resolves to the shaft area and is
penalized exactly as before. `test55_plat` is the map that pins both halves down: it resolves today
only by luck, because its stop area is `18..110` and the raw origin 28.5 happens to fall inside, while
its nav z at ride end is 10.5 and needs the snap to reach 18.

**The target area wins a boundary tie.** Stop slabs abut the shaft exactly — `test18_elevator` area 6
ends at `z 16` and area 7 begins at `16` — so a probe at the stop is inside both within
`kPlaneEpsilon`, and `FindAreaExact` returns whichever comes first in id order, which is the shaft.
`FinishElevatorRide` therefore asks `Nav::AreaContainsPoint(*target, probe)` first and only falls back
to the sweep when the answer is no. That is also the more honest question: the verdict being sought is
"did I arrive at the stop I wanted", not "which of two overlapping areas wins the iteration". A ride
that ended on the wrong floor is nowhere near the target in z, fails the test, and is penalized by the
fallback.

The same raw-origin probe is still used by `RunAirmoveFlight` and `FinishLadder`. Neither has a
reproducible failure: crossfire's two-unit stops carry only walk and elevator connections, no air
moves, and ladder tops are ordinary volumetric areas. The `undecided` lines those two still emit
(`test25_ladder2`, `test53_ladder4`) are genuine — a descent that ends mid-drop is in no area.

**`totalTime` under-reports by the whole ride.** The ETA integrates over segments, and there is no segment,
so `BuildPathProfile` adds `elevatorWaitTime + elevatorTravelTime` after the loop. Without it
`test55_plat`'s ETA is 3.8 s against a 4.96 s ride and `test_nav_goal.ps1` — which times out at twice the
bot's own ETA — cannot pass the map however well the bot behaves.

**The ride waypoint may own no profile segment at all**, so the approach cannot be dispatched off
`bot.waypointIndex` the way the air move and the ladder are. On `test18_elevator` the funnel corner
before the board point is the shaft centre at floor height, the board point is the same XY at the stop's
top surface, and *that* pair is zero-length too — the segment reaching the shaft carries the corner's
index and index 10 never appears in the profile. The runner is therefore driven by the barrier index the
`followCfg` loop already computed, exactly as `RunActivation` is driven by `activationWp`. Dispatching on
`waypointIndex` looked right on `test55_plat`, where the approach happens to be the ride's own segment,
and silently never boarded on test18.

Beyond that the runner is deliberately dull. It latches on `FL_ONGROUND` within `kElevatorBoardRadius`,
holds station with a `kElevatorHoldRadius` deadzone, never jumps, and aims at the waypoint *after* the
arrival one so the bot is already turned to walk off — `func_plat`'s `PLAT_DELAY` gives it three seconds.
`RunElevatorDrive` skips `SolveDrive` when drive is zero: the servo answers a residual velocity with a
full-rate reverse command, which is right on open ground and wrong on a platform edge, so friction does
the stopping instead. `lastProgressTime` is stamped every frame for the same reason the ladder stamps it,
and the 150-unit teleport reset at the top of `Bot_MoveNav` is skipped while riding, since a server hitch
would otherwise repath from mid-shaft where `FindAreaNearest` finds nothing and the bot drops its goal.

The arrival test is `BotNavZ` against the arrival waypoint's own floor Z, directional with
`kElevatorArriveSlack`. The slack means the bot can step off up to 8 units short, and that is safe for a
reason worth stating: navgen only builds a walk seam off a stop when the step is under 18 units, so
anything within the slack is steppable by construction. On `test55_plat` the ride ends at nav 10 against
a stop at nav 18 and the bot walks out over the remainder. That same slack is what the landing
verdict snaps its probe by, above.

The engage radius is pure XY distance to the board point, so a route that merely *passes* within 48 units
of a shaft it has not reached yet would hand the frame over early. Ordering is what protects this today:
the barrier sits at whichever comes first, so while a call button is unserviced `rideWp` is unset and the
runner is never called. On `test18_elevator` that matters concretely — the air move lands 40 units from
the board point, inside the radius, with the button still to press. If a map ever defeats that, the ride
times out, penalizes the connection and repaths; tying engagement to `progressS` reaching `sLimit` is the
tighter fix if one is needed.

`rideDeadline` is `wait + travel * kElevatorTimeMargin + kElevatorTimeGrace`; the
wait term matters because a ride *back* to the default stop does not begin until the platform's auto-return
dwell elapses. Timing out finishes the ride like any other failure — penalize the connection, repath — and
that is also what recovers a bot that walked to a boarding point whose platform someone else had ridden
away.

`bot_avoid.cpp` needed nothing: `RouteExemptsHit` already exempts every `SOLID_BSP` hit while a ride
waypoint is in the lookahead window, so the bot does not fight the platform on the way in, and the runner
returns before `BotAvoid::Update` while riding.

**What the two test maps actually cover.** `test55_plat` is the only one that exercises a sustained ride:
the bot boards the parked plat in the pit, `CPlatTrigger` fires under it, and it rides 248 units to the
ledge — the log reads `boarded ... (up)` then `ride reached the stop of after 4.18s ... area 4 (wanted 4)`
against a 4.96 s plan. Note it boards at nav −199, not the parked −230, because stepping onto the plat
starts it moving well before the bot reaches the column centre; the runner does not care, since it watches
the arrival height rather than counting from the board point. `test18_elevator` covers summon-then-board —
drop into the shaft, walk to the call button, press, board — but **not** the ride loop: the door travels
for 2.38 s while `RunActivation` holds its 2.4 s button wait, so the bot is already at the top when it
boards and the ride finishes in 0.00 s. That is correct behaviour, not a skipped test, but a map with a
call button *off* the platform is what would exercise waiting for a lift to arrive.

### BotNavZ
A waypoint's point is at player height in its own area, so the floor under it is the point less the
stand lift unless the plan wants the bot ducked -- the convention BotNavZ reads off the live stance.

### SolveDrive skip on zero drive
Zero drive means stand exactly still. SolveDrive answers a residual velocity with a full-rate reverse
command, which is the right servo answer on open ground and the wrong one on a platform edge, so the
solve is skipped and friction does the stopping.

### Elevator ride flag handling
The ride is spent. Clearing the flag is what lifts the sLimit barrier, and it travels with the
waypoint through SpliceDetour, which an index watermark would not.

Boarding and arrival share an XY, so a global reprojection is free to snap back below the
ride. Place progress on the far side by hand instead of clearing trackState.started, and
floor it there: the two segments meet at one point, and walking away from a lift you entered
and left on the same side is equidistant from both, which the earlier segment wins.

No arclength accrues while the platform does the work, so the 2 s stall timer would fire twice
over on a 5 s ride.

This is called for the whole stretch the ride bars, which is most of the path. Leave the walking
to the tracker and take the frame only once the platform is close enough to step onto.

Every arclength is about to change, and the rebuilt chain starts at the bot's own position, so
there is no pre-ride geometry left to guard against.

### Server hitch during ride
A server hitch during a ride can move the bot further in one frame than a teleport would, and resetting
mid-shaft repaths from a point that resolves to no area at all -- the bot then drops its goal.

### Pending ride bars profile
A pending ride bars the profile for the same reason an unserviced activator does, only harder: its two
waypoints share an XY, so the ride is a zero-length segment the tracker cannot see at all and would
otherwise walk straight through. A ride is "pending" until FinishElevatorRide clears its flag.

### Ladder mount bars profile
A ladder mount is the same shape -- mount to exit is near-zero in XY -- and is "pending" until
RunLadderApproach commits and steps the index past it. Letting the tracker sail through one is what
stranded test53_ladder4 at ladder *5: the index reached the exit waypoint while the bot was still
two units short of the volume, so the mount was never serviced, and the moment the bot drifted onto
the ladder anyway the ladderEject rule below saw no ladder waypoint in range and jumped it straight
back off at 270 u/s, forever.

### An air-move launch bars the profile too
Same shape again, and it was missing from the barrier list for a long time — `barrier` covered rides and
ladder mounts only. The gap segment starts *at* the launch point, so once the bot is within a unit or
two of it the nearest point on the chain flips to the far side of the hole and `wpIndex` becomes the
landing waypoint. `RunAirmoveApproach` is only entered when the *current* waypoint carries
`airmoveLaunch`, so a launch skipped that way is never taken at all: the bot keeps walking at a waypoint
on the other side of a gap and drops into it.

crossfire is where it showed. Area 66 is a ledge `x ∈ [140,176]`, area 47 is another at `x ∈ [240,276]`,
there is no walk connection between them — 66's only walk neighbours are 54 and 80 — and the bot fell at
`x = 204.7`, squarely in the 64 units of void, with `heading to wp 5/11` naming a waypoint inside area 47
and no launch line anywhere above it. Nothing else in the follower would catch that: `ProjectOntoProfile`,
`lateral` and the `done` test are all horizontal, so a 180-unit drop reads as zero deviation and only the
2 s stall timer eventually fires.

### Barrier driven by barrier index
Driven by the barrier index rather than bot.waypointIndex, because a ride waypoint need not own any
profile segment: on test18_elevator the funnel corner before the board point shares its XY, so the
segment that reaches the shaft carries the corner's index and the ride's index never appears at all.


### Activation at runtime
`src/bot/bot_move_nav.cpp` services activation waypoints; see `doc/navgen.md` for how the detour is built.

- **Barrier.** `NavFollow::Config` carries `sLimit` and `sFloor`, both arc lengths from
  `NavPlan::ProfileWaypointS`. They exist because a detour doubles back on itself: the outbound and return
  legs are collinear in XY, so `ProjectOntoProfile` cannot tell them apart by distance and resolves the tie
  in favour of the lower `s`. Without `sLimit` the bot's `s` jumps to the return leg and it never stands on
  the activator (worse on first acquisition, which searches the whole profile); without `sFloor` it can never
  leave the outbound leg afterwards and stalls in place. `sLimit` is the next unserviced activation *or
  pending elevator ride*, `sFloor` the last serviced one, both computed in one pass over the waypoints each
  frame. A ride is the same tie in its extreme form — its two waypoints share an XY exactly, so the
  segment between them has zero length and does not exist in the profile at all; see `doc/movement.md`
- **Press.** `RunActivation` takes over the frame inside `kActivationEngageRadius`, aims at `activatorCenter`
  in pitch and yaw, and emits a single `IN_USE` rising edge once the aim converges and the bot is within
  `kActivationUseRange` of the centre in 3D. One edge, not a held button — repeated presses would toggle a
  toggle-style button back off. It then holds for the door's `buttonWaitTime` before releasing the barrier
- **Triggers** need no press, only arrival, so they are serviced by proximity before `Track` runs
- **Memo.** Serviced activators are remembered by centre for `kActivationMemory` seconds. This is what stops a
  mid-detour repath from pressing twice (waypoint indices change, world state does not), and what makes a
  failed press retry on its own once the memo lapses

### RunActivation ordering
Ahead of RunActivation, since a board point can sit inside the engage radius of a later button.

### RunChargerUse

`RunActivation` again, with two differences that matter. `IN_USE` is **held every frame** rather
than pressed once, because `CRecharge::Use` and `CWallHealth::Use` are `FCAP_CONTINUOUS_USE` and
transfer exactly one point per call — a single press buys one armor. And the exit condition is
measured, not timed: the runner watches `ChargerValue` and gives up `kChargerNoGainGrace` (0.5 s)
after it stops rising.

What stopped rising means depends on `pev->frame`, which is the charger's own empty flag (`Use` sets
it to 1 when `m_iJuice` hits zero, `Recharge` puts it back to 0). Frame 1 means the charger is
drained and the bot leaves without a memo entry — the live `frame` test in selection already filters
it, and unlike a pickup there is no guesswork about when it returns. Frame 0 with no gain means the
bot is full, and the classname goes in `fullMemo`.

**The charger's capacity is not the bot's ceiling.** `sk_suitcharger` is 30 on `test56_recharger`'s
skill level, so a bot at 0 armor drains the whole thing and leaves at 30, comes back after the 30 s
recharge, and needs four visits to reach 100. The run reads `0 -> 30 -> 60 -> 90 -> topped up at
100`. A runner that assumed one visit fills you up would have called the first three visits failures.

It is dispatched from inside the `follow.done` branch rather than from the waypoint loop, because a
charger is not a waypoint — the plan's goal is the use spot and the charger is off the mesh
entirely. `lastProgressTime` is stamped every frame for the same reason `RunActivation` stamps it:
no arclength accrues while standing still and the 2 s stall detector would otherwise repath.

**A charger holds the bot's view hostage, so being shot at one ends the visit.** `RunChargerUse`
writes `desiredAngles` at the fixture every frame, so a bot shot mid-charge cannot turn to answer.
`Bot_BailManeuver` calls `FinishCharger` — which clears `chargerActive` and forgets the goal item —
and the next frame `BotCombat::Run` drives. There is deliberately no `NoteIgnored`: nothing is wrong
with the charger, and `BotCombat::Exit` zeroes `nextSelectTime`, so the bot can walk straight back once
the fight is over. `RunHealVolume` is not bailed out of, even though it shares the `chargerActive`
bookkeeping. It leaves the view to `BotSense::ApplyLook` and `BotFire::Think` runs before any movement,
so a bot standing in a healing pool is already shooting back while it heals, and leaving would only
stop the healing.

## path_follower.cpp

### NavFollow::Track
Follows a `PlanProfile`. All the old heuristics (chase point, corner cut, corridor pull, `brakeRadius`
blend, arrival radii) are gone — the geometry is precomputed, so the tracker only has to stay on it.

**Projection** - monotone: each frame searches only `[s - projectBack, s + speed*dt*4 + projectAhead]`,
so the bot cannot snap onto a distant part of the route that happens to pass nearby.

**Aim direction** - the tangent one frame ahead, plus a cross-track correction toward the chain scaled
by `min(1, offset/rejoinDist)`. Deliberately *not* pure pursuit: aiming at a point some lookahead
ahead makes the bot start turning as soon as that point rounds a corner, cutting the corner by up to
the lookahead distance. Since the arc already encodes the turn, no anticipation is wanted.

**Speed** - the profile speed at the current arclength, derated by `1 - (lateral - C)/C` (floored at
`derateFloor`) when outside the clearance budget, so the bot slows to rejoin instead of arcing wider.

**Velocity servo** - `SolveDrive`, below.

**Abort** - `needRepath` once lateral exceeds `rejoinAbort` (96). Previously nothing detected being
off-path; the corridor pull fought forever until the stall timer fired.

**`done` means arrived, and only that.** The cross-track correction can cancel the tangent exactly — a
bot barred at a node by `sLimit` and standing beyond it along the tangent sees the chain point directly
behind, and the two unit vectors sum to zero. That used to fall through to `d.done`, which
`Bot_MoveNav` announces as reaching the goal and acts on by dropping it, wherever the bot happened to
be. `test18_elevator` hit it while barred at a lift, printing `reached goal` 312 units away and mid
shaft. A degenerate blend now keeps the raw tangent, and the surviving zero-direction case — which
needs an empty tangent, not a cancelled one — asks for a repath instead of claiming success.

The rejoin pull can cancel the tangent exactly -- a bot barred at a node and standing beyond it
along the tangent sees the chain point directly behind. Falling through to `done` there reports
an arrival wherever the bot happens to be, so keep the raw tangent and let the lateral test
decide whether this is worth a repath.

### NavFollow::SolveDrive
Inverts one engine frame to hit a target velocity `V*`. This is the only correct way to ask the engine
for a speed, and everything that drives the bot goes through it — `Track` for ordinary following,
`RunAirmoveDrive` for the air-move approach.

```
v_af      = velocity after this frame's friction   (guard |v| < 0.1, as PM_Friction does)
need      = V* - v_af
wishdir   = normalize(need)
wishspeed = max(|need| / (sv_accelerate * dt * friction), along + |need|)
```

The second term matters: `PM_Accelerate` caps its delta at `wishspeed - v.wishdir`, so the rate-based
form alone commands zero acceleration when already cruising at the target. One formula covers
acceleration, braking, counter-strafing and arc tracking, saturating when a frame cannot deliver.

The *first* term is what makes low speeds reachable at all, and it is the reason commanding `V*`
directly as a wish speed is a bug rather than an approximation. Below `sv_stopspeed` the engine's
friction drop is the constant `stopspeed * f * dt` no matter how slowly the bot is moving, while
`PM_Accelerate` delivers `sv_accelerate * wishspeed * dt`. A raw wish speed therefore does nothing
whatsoever until

```
wishspeed > stopSpeed * groundFriction * edgeFriction / accelerate
```

which at defaults is 40 on open ground and **80 within a hull's reach of a ledge**, where the engine
doubles friction by `sv_edgefriction`. `byRate` sidesteps the *stall*: the further the velocity is from
`V*`, the larger the commanded wish speed, so the bot saturates the frame and closes on `V*` from any
starting speed. See `RunAirmoveApproach` above, for the dropoff this stalled.

What it does *not* do by itself is arrive at `V*`, and that is the next rule.

**`byRate` does not cover for an unmodelled friction — it converts it into a standing speed error.**
The formula is a deadbeat solve: it asks for the one wish speed whose `sv_accelerate * dt * wishspeed`
exactly replaces the drop, so its gain is set by whatever `f` the caller passed. Feed it a friction
lower than the engine's and it settles short, permanently. At the fixed point the engine takes
`stopSpeed * f_real * dt` per frame and returns at most `accelerate * dt * W`, so holding still needs
`W = stopSpeed * f_real / accelerate`; `byRate` produces that W at

```
v = V* - stopSpeed * (f_real - f_model) * dt
```

which for an unmodelled ledge doubling is a flat `V* - 400 * dt` at defaults — 4 units short at 100 fps,
6.7 at 60, whatever `V*` is. Measured on test53_ladder4's crouch jump: 66.8 / 66.4 / 66.2 against a
design 70.8, on frames of 10.0 / 11.0 / 11.6 ms.

The error exists only *below* the `stopSpeed * groundFriction * edgeFriction / accelerate` threshold
above — 80 at defaults. Above it `byCap` wins instead, `W = V*`, and a target that high is
self-sustaining against edge friction anyway, so the model error is invisible. That is why this hid for
so long: it needs a design speed under 80, and the one place that reliably produces one is a **crouched**
approach, capped at `maxspeed * kDuckingMultiplier` ≈ 90 and usually decelerating into a launch below
that.

`physics.edgeFriction` is therefore consumed by the follower, but not by `SolveDrive` itself, which stays
a pure function of the `PmPhysics` it is handed. `Bot_MoveNav` runs the probe and hands it physics whose
`groundFriction` is already doubled — see `BotDrivePhysics` above. `BuildPathProfile` is
deliberately left alone: it is a whole-route model built once at plan time, and folding a momentary
ledge state into it would make design speeds depend on where the bot stood when it replanned.

## Following a drifting path

`NavFollow::Config::driftVelocity` is the world-frame shove the bot is currently subject to.
`Bot_DriftVelocity` reads it live from the engine — `groundentity->v.movedir * speed` when the ground entity
has `FL_CONVEYOR`, plus the cached `Bot_PushFields` closure at the bot's origin — so a `Use`-reversed
conveyor or a switched-on field is handled without rebaking. The closure is cached on `CBot::pushFields` and
refreshed with each repath, because scanning every edict per frame per bot is not free.

`Track` does two things with it. The world-speed cap becomes `DriftSpeedAlong(drift, aimDir, maxSpeed)`
instead of `physics.maxSpeed`, so a bot riding a belt is allowed to exceed its own run speed. And the wanted
velocity handed to `SolveDrive` is `aimDir * target - drift`: `pev->velocity` is own-only, `SolveDrive`
solves in the own frame, and its clamp to `physics.maxSpeed` stays correct untouched. Without that
subtraction a bot crossing undertow's 420 u/s current is simply carried away from its own plan.

`BuildPathProfile`'s `NodeMaxSpeed` applies the same `DriftSpeedAlong` using the waypoint's outgoing segment
direction (the last waypoint uses its incoming one), which is where `PlanWaypoint::drift` — copied from
`PlanArea::drift` in `PlanPath` — is used. Corner radii, the forward/backward speed pass and `totalTime` all
follow from it, so `estimatedArrivalTime` stops being wrong by up to 2x on a belt.

## Physics simulation with gravity

In `pm_sim.cpp`, `PM_AddCorrectGravity` folds the vertical push component into velocity as an acceleration and clears it from the base velocity, so only the horizontal remainder wraps `FlyMove` below.

The horizontal base velocity wraps the move the way `PM_AirMove` and `PM_WalkMove` do: it is added before the clip-and-slide loop, then pulled back out once the ground check has seen it. This ensures consistent handling across all movement types.

## bot_opportunist.cpp

The nav mesh is generated once at a fixed design speed and every connection's `costSeconds`,
`flightTime` and `exitSpeed` is baked at it. A bot that is *faster* than that — `sv_maxspeed` raised, or
holding `item_longjump`, which replaces a jump with `forward * 560` and `sqrt(2*800*56)` up — has a
movement option no plan on that mesh can contain. This checks, four times a second, whether taking it
right now beats walking the plan, and if so hops off-plan and repaths from wherever it lands. It is not a
planner change: the plan stays what it is and the bot occasionally short-circuits it. Can be disabled with
`crossbot_oppo_jump = 0`.

### Why the design speed had to go in the nav file

The trigger is "the bot is faster than the mesh assumes", and until version 16 nothing recorded what the
mesh assumed — navgen's `--maxspeed` was a command-line option that vanished after the run. The obvious
substitute, `kNavRunSpeed`, is the wrong number and fails loudly: it is 250, so a bot at the stock 320
already clears `1.25 x` and the check would arm on every map. `NavHeader::designMaxSpeed` is the number
navgen actually built with. The loader falls back to `kNavDefaultDesignSpeed` if it reads as zero, which
only happens for a file this field predates — and the version check rejects those anyway.

### One reverse search, not one per candidate

The comparison needs the cost-to-goal of two different positions, and needs it again every tick.
`Nav::AreaCostsToGoal` prices *every* area to the goal in one Dijkstra (`doc/path_planning.md`), so both lookups
are array indexes and the per-tick cost is a hull-traced `PmSim` run and nothing else.

The field is cached per bot and rebuilt when the goal area changes **or on any successful `Repath`**. The
second condition is the one that is easy to miss: `runtimePenalty` mutates edge costs between repaths
through the penalize/reward loop, so a field built before a penalty landed prices a route the planner has
already given up on. Rebuilding is cheap — this is the search with no `PlanWalker` and no `PmSim`, the
one `BotItems::SelectGoal` already runs at up to 2 Hz per bot.

### Comparing plan-seconds against real seconds

`AreaCostsToGoal` values are centroid-anchored and priced at `kNavRunSpeed`; `PmSimulator::Time()` is
real seconds. Subtracting one from the other directly is wrong in a way that always errs the same
direction: both cost terms are inflated relative to what the bot really achieves on the ground, the
flight is not, so the difference between them looks bigger than the flight it is being weighed against
and the bot jumps for gains that do not exist. The flight is converted into plan-seconds instead:

```
gain = C[here] - C[landing] - flightTime * (maxSpeed / kNavRunSpeed)
```

and has to clear `kMinGainSeconds` to fire, which absorbs the centroid noise both terms carry.

**The ratio is `maxSpeed`, not the bot's current speed**, and that distinction is worth more than it
looks. A long jump overwrites horizontal velocity with a flat 560 whatever the bot was doing, so a
launch at 68 u/s and a launch at 270 u/s produce the *same* 0.8 s flight — but scaling by instantaneous
speed charged the first one 0.22 plan-seconds and the second 0.86, making a jump look nearly free
precisely when the bot was too slow to be jumping. `maxSpeed` is what the cost field is being compared
against: the field prices ground travel at `kNavRunSpeed` while the bot actually cruises at `maxSpeed`,
and that ratio is a property of the map's physics, not of this frame.

No sub-area `|origin - centroid|` term is added to either side. Both endpoints price to the same goal, so
the centroid offsets largely cancel; the item-ranking formula in `bot_items.cpp` does add one, but it is
ranking *targets* against each other, not two routes to one target, and copying it here would charge the
bot for walking back to its own centroid. A landing in the area it started in is rejected outright rather
than priced: the two costs are then identical by construction and the flight is pure loss.

### Simulating a long jump without teaching PmSim about it

`PmSim` models `kJumpImpulse` and the commanded speed and knows nothing about the module. It does not
need to: `BeginAirborne` takes an explicit launch velocity, so the caller builds the one `PM_Jump` would
have produced — `kLongJumpSpeed` horizontally, `kLongJumpImpulse` up. That impulse is `sqrt(2*800*56)`
with the 800 **hardcoded in `pm_shared.c`**, not `sv_gravity`; a map with altered gravity changes the
arc's fall but not its launch. The arc is traced with the duck hull throughout, the same timeline
`JumpLandsIn` assumes and the same one the engine produces (see "IN_DUCK and IN_JUMP in one command").

The loop has to notice the bot leaving the ground before it can notice it landing. `Begin(..., jump)`
returns with `OnGround()` still true — the impulse is applied inside the first `Step` — so a
`while (!OnGround())` never runs a single frame. It tracks an `airborne` flag instead, and a sim that
never gets airborne is rejected rather than treated as an instant landing.

### The long jump leaves along the view, not along velocity

The plain jump and the long jump take their direction from different places, and the difference is not
cosmetic. `PM_Jump` keeps horizontal velocity for an ordinary jump, so simulating one from
`atan2(vel.y, vel.x)` is exact. The module branch instead *overwrites* it:

```c
for (i = 0; i < 2; i++)
    pmove->velocity[i] = pmove->forward[i] * PLAYER_LONGJUMP_SPEED * 1.6;
```

`pmove->forward` is `AngleVectors(cmd.viewangles + punchangle)` (`PM_CheckParamters`, then
`PM_PlayerMove`), so the whole horizontal launch is *where the bot is looking* and the velocity it had is
used for nothing but the `Length(velocity) > 50` gate. The comment sitting above that branch in
`pm_shared.c` — "UNDONE -- note this should be based on forward angles, not current velocity" — reads
like the opposite of what the code does, and simulating the velocity yaw makes the bot predict a landing
area it then flies nowhere near: navigation aim points at `waypointIndex + 1` and `ApplyLook` can drag it
onto an enemy, so the two yaws routinely disagree by tens of degrees.

`forward` is also not flattened. `forward[0..1]` is `cos(pitch) * (cos(yaw), sin(yaw))`, so pitch scales
the horizontal launch while `velocity[2]` stays the flat `sqrt(2*800*56)` — a bot looking 30° down long
jumps at 485, not 560.

**The launch frame is one `Bot_UpdateFacing` step away from the angles the check reads.** `TryJump` runs
before the follower's aim code and the command is issued by `RunAirmoveDrive` right after, which sets
`desiredAngles` to the launch yaw — so the yaw error is nulled, but residual `aim.yawVel` and drift still
move the view during that step, and pitch is dragged toward 0. Both are handled by refusing to launch
rather than by modelling them: `kMaxLaunchYawRate` skips the tick while the aim is slewing (it retries in
250 ms), and `kMaxLaunchPitch` keeps the `cos(pitch)` term inside a couple of percent of whatever the
step leaves behind. The oppo log prints the launch yaw and the velocity yaw side by side, so a hop that
misses can be read as "wrong prediction" or "aim moved" without guessing.

### Letting the command through, and what happens after

`Bot_RunPlayerMove` strips `IN_DUCK` from any `IN_JUMP` command while `slj` is set, which is exactly the
long jump this wants. The strip is now gated on an `allowLongJump` argument defaulting to false, so every
other call site sends byte-identical commands and only the opportunistic launch frame opts in.

The hop reuses the airmove flight machinery with `airmoveConnIndex = -1` and `airmoveOpportunistic` set.
That flag does three things at landing: it suppresses `AirmoveLandingMiss`, which measures distance to the
plan's next waypoint and is meaningless for a jump that deliberately left the plan; it forces a repath
unconditionally, since the landing is off-plan by construction; and a landing in an area other than the
predicted one starts a cooldown. Without that cooldown a bot whose sim disagrees with the engine retries
from the same spot at 4 Hz forever. There is no connection to grade, so the reward/penalty branch is
skipped — it already keys on `connIndex >= 0`.

## bot_avoid.cpp

Reactive obstacle handling, layered between the follower's steering decision and the engine call. The
nav mesh only knows world geometry and the brush entities navgen carves, so anything else solid — a
`cycler`, a monster, another player, a door that has swung out of its own doorway — is invisible to
the planner: it routes straight through and the follower drives into it and stalls. `BotAvoid::Update`
traces forward along the planned path each frame and answers with one of four moves: crouch-jump onto
the obstacle, duck under it, go around it, or — when none of those work and the thing can be destroyed
— break it. `BotAvoid` only ever *decides* to break; `bot_break.cpp` does it.

### Which entities get avoided

**`MOVETYPE_NONE` does not identify the static ones** — this is the obvious filter and it is a no-op.
Every solid brush entity in HL is `MOVETYPE_PUSH`, `func_wall` included (`dlls/bmodels.cpp`,
`doors.cpp`, `func_break.cpp`, `plats.cpp`). The brush entities that are `MOVETYPE_NONE`
(`func_illusionary`, `CMultiSource`) are all `SOLID_NOT`, so a trace never reaches them.

`FL_WORLDBRUSH` is the flag that does identify them. `CFuncWall::Spawn` sets it — "If it can't
move/go away, it's really part of the world" — `const.h` defines it as "Not moveable/removeable brush
entity", and the HLSDK's own node graph uses it for this same test. `func_conveyor` and
`func_wall_toggle` inherit it; the only other setter is unbreakable glass. It is also precisely the
set navgen carves as blockers, so skipping it avoids fighting the mesh over geometry the mesh already
encodes.

A hit is skipped when it is worldspawn, when `solid` is none of `SOLID_BSP`/`SOLID_BBOX`/
`SOLID_SLIDEBOX`, when it is a `SOLID_BSP` with `FL_WORLDBRUSH`, or when it is the passable the route
runs through (below). Everything else is avoided, which includes other players — bot-vs-bot is not
special-cased, it simply never qualifies for either maneuver.

**The door on the route is exempt, and identifying it needs no entity id.** `RouteExemptsHit` scans
the next `kAvoidRouteLookahead` waypoints for `requiresDoor`/`requiresBreak`, resolves
`targetAreaId`, and skips the hit when the area's bounds overlap the entity's `absmin`/`absmax`. That
works because navgen builds a door/breakable area *from the entity's own brush-model bounds*: a
closed door sits inside its area and overlaps, so the bot walks into it and touches it open; an
opened door has translated out of that volume, no longer overlaps, and is avoided like anything else.
This matters because `NavAreaRecord` stores no entity or model index — a direct edict↔area link does
not exist in the format. `requiresElevatorRide` in the same window exempts every `SOLID_BSP` hit, so
the bot does not fight a plat it is about to board.

**The goal item is exempt too, and that one does need the edict.** `RouteExemptsHit` returns true for
`bot.goalItem`, before the `SOLID_BSP` test, because avoiding the thing you are deliberately walking up
to is never right. In practice it only fires for chargers — every other item is `SOLID_TRIGGER` and
never qualifies in the first place — and it only matters for the last stride, which is exactly where a
32-wide hull grazes the brush it is about to press `+use` against. Without it that graze costs a
forced repath, and the charger case turns out to be the one where a forced repath cannot help (below).

Note the carve set and the `FL_WORLDBRUSH` set are close but not identical. `func_wall`,
`func_conveyor` and `func_wall_toggle` are both carved by navgen and flagged, so the two filters agree.
`func_healthcharger` and `func_recharge` are carved but **not** flagged — `CWallHealth::Spawn` never
sets it — so the mesh routes around them while a trace still sees them. That asymmetry is deliberate:
the plan keeps its 16 units of clearance, and a bot that has drifted off the plan into one still gets
the detour search rather than pushing at it.

### Breaking through

Breaking is decided in two places, and they answer different questions.

**On the route.** The plan deliberately goes through the breakable: navgen split it into its own
`NAV_AREA_BREAKABLE` and the planner emitted a `requiresBreak` waypoint and charged `openSeconds` for
it. That is exactly the case `RouteExemptsHit` already skipped, so the avoid chain never ran at all and
the bot simply walked into the brush and stalled. `RouteExemptsHit` now reports *why* it matched
through a `wantsBreak` out-param, and a hit exempted for `requiresBreak` goes straight to
`BotBreak::Engage`. No duck, no jump, no side search — the planner has already decided and already paid.

**Off the route.** An unplanned breakable in the way, which is the ordinary avoidance path. Here
breaking is the *last* resort, after `canJump`, `canDuck` and `RunSideSearch` have all failed:
`TakeBreakTarget` sits on both non-receding failure paths in `RunSideSearch`, ahead of the
`searchFailed` / `blockedUntil` assignment. That ordering matters in both directions. A crate the bot
can hop onto gets hopped onto rather than smashed; and a breakable it genuinely cannot get past no
longer reaches `PenalizeCurrentConnection` and `BotItems::OnBlocked`, because it is about to stop being
in the way. A receding mover never gets here at all — `!recedes` guards both call sites.

Neither path sets `searchFailed`, so `avoid=break` is the only kind that survives the end-of-frame
stickiness test on its own terms rather than through `detourExpire` / `nextSearchTime`.

### The three maneuvers

The forward trace runs in the hull the bot currently occupies (1 standing, 3 ducked) with
`dont_ignore_monsters`, over `speed * kAvoidLookaheadTime` clamped to
`[kAvoidLookaheadMin, kAvoidLookaheadMax]`. The existing `EngineTraceFn` cannot be reused for it: it
passes `ignore_monsters` and discards `tr.pHit`. Heights are measured against the bot's feet, so
`gap = absmin.z - feetZ` and `top = absmax.z - feetZ`.

**Jump-on is tried first, because it is the faster way past** — crawling costs `kDuckingMultiplier`
of speed for the whole traverse. It applies when `top` is above step height (below that the engine
steps up for free) and within `PlainJumpHeight(gravity) + 18` less a margin. The 18 is the feet lift
from ducking in the air; the jump fires when `dist <= speed * riseTime + kAvoidJumpLead`, where
`riseTime` is the smaller root of `top - 18 = kJumpImpulse*t - g*t²/2`. At a standstill against the
obstacle that degenerates to a fixed 8-unit trigger, which still fires.

**A ducked bot cannot crouch-jump.** From a ducked stance the origin rises 45 and the feet rise with
it — there is no 18-unit credit, because the hull is already small. So when the bot is ducked and
`top` exceeds a plain jump, the latch is cleared and `suppressPlanCrouch` is set instead of jumping;
unducking is instant where there is room and the jump fires a frame or two later. On `test52_avoid`
this is also what ends the crawl under the first can row in time for the jump onto the second, which
is only 64 units further on. `kAvoidLookaheadMin` is 64 rather than smaller for the same reason: a
ducked bot moves at 83 u/s, and a speed-proportional probe alone would not see the second row until
there was no room left to stand up.

**Crawl-under** applies otherwise, when the bot is standing and `gap` clears the 36-tall duck clip
hull with margin. The hold is `(dist + depth + 2*kPlanClearance) / duckSpeed * kAvoidCrawlPenalty`:
the requested `min(x,y)/speed*penalty`, plus the remaining approach and the bot's own 32-unit depth,
evaluated at ducked speed rather than max speed. `slowApproach` caps the drive at ducked speed while
the latch is set but `FL_DUCKING` has not appeared — `PM_Duck` takes 0.4 s on the ground before the
small hull exists, which is 100 units at full speed, and on test52 the bot only has 48 from spawn to
the first row. `kAvoidDuckMinHold` is 0.5 s for the same reason: releasing `IN_DUCK` part-way through
aborts the transition.

**The hold time does not have to be accurate**, in either direction, which is why nothing maintains
it. Too short is covered by the engine — `PM_UnDuck` traces the stand hull and returns without
clearing `FL_DUCKING` when it is `startsolid`, so a bot whose latch expired under an obstacle stays
ducked and keeps crawling, and the `drive /= kDuckingMultiplier` compensation keys off `FL_DUCKING`
rather than off the latch, so its speed stays right too. Too long is covered by the jump branch,
which clears the latch when it wants to stand. An earlier design carried a second "can I stand here"
probe to maintain the latch; it was solving a problem the engine already solves.

A plan-mandated crouch always wins: avoidance may add a duck and may drop *its own* duck, but it never
overrides `requiresCrouch` or `NAV_AREA_CROUCH`.

**Going around** is the third answer, taken when neither height predicate applies — a standing player,
a full-height crate, a door that has swung open. It searches for a route past the obstacle and splices
it into the front of the plan.

### The side search

The obstacle's `absmin`/`absmax` is expanded in x and y by `kAvoidHullHalf + kAvoidCornerMargin`, so
its four corners are positions the bot's own hull can occupy with two units to spare. Projected into a
frame where *forward* is the direction from the bot to the obstacle centre and *right* is the bot's
right, two corners fall on each side. The search then walks the box perimeter: from the current point,
trace to the first corner on the chosen side, and after each corner it reaches, try to trace straight
to the destination. Up to all four corners are used, so the route can wrap around a far face.

**Two corners is not enough,** which is why the walk exists. Two suffice only when the destination is
roughly opposite the approach; when it is off to the side the leg from the far corner re-enters the
expanded box and clips the obstacle it just cleared.

**Corners are not required to be standable.** The downward hull probe at a corner supplies its z, but
when it comes back `startsolid` — the corner is buried inside a neighbouring obstacle — the corner is
kept at the previous point's height and the leg is traced to it anyway. The leg then reports the
neighbour, and *that* is the useful outcome: the search recurses on the neighbour with the **final
destination** as its target, not the unreachable corner, starting from the point the blocked trace
began. On a cluster of touching obstacles this is the only thing that works; requiring standable
corners abandons the side, and merging touching obstacles into one box over-blocks whenever the
merged rectangle spans empty diagonal quadrants.

A leg blocked by *world* geometry does not fail the side — the walk skips that corner and tries the
next. A leg blocked by an already-visited obstacle fails, which with `kAvoidMaxDepth` and
`kAvoidMaxTraces` is what terminates the recursion. The visited set is restored to its entry state
before each side is tried, so the two sides never poison each other.

**Which side first.** If the obstacle is moving, go where it is not: `dot(MoverVelocityAt, right)`
above `kAvoidMoverSpeed` picks the opposite side and takes it without evaluating the other — an
entity sliding to the bot's left sends the bot right. Otherwise both sides are evaluated and the shorter
route wins, ties going right. Right-on-tie is the useful convention for two bots meeting head-on:
their `right` vectors point opposite ways in world space, so both stepping right passes them cleanly.

**A flat hull trace has no step-up,** so a stair nosing or a slope lip reads as a wall. When a leg is
blocked by non-qualifying geometry whose plane normal is floor-like (`z >= kAvoidFloorNormal`), the
same leg is retried raised by `kAvoidStepHeight` and accepted if that clears. This is the navgen idiom
from `airmove_sampler.cpp`.

### Where the detour rejoins the plan

Not at a waypoint. navgen cannot see a `cycler` or a player, so a planned waypoint is frequently
*inside* the obstacle — no route to it exists at all, and aiming at one is how the search fails
without any geometry being to blame. Instead the remaining planned polyline is sampled forward at
`kAvoidDestStep`, and the destination is the first sample that is both outside the expanded box and
passes the ground probe. The waypoint that sample belongs to becomes `detourReplaceUntil`; waypoints
before it are dropped, since they are the ones inside the obstacle.

The detour is capped at `kAvoidDetourMaxLength` of total path. **This cap must not be relative to the
distance to the rejoin point** — the rejoin point is deliberately close, so a relative cap rejects
exactly the long routes that a cluster legitimately requires. The five-can corner on `test52_avoid`
needs 432 units to clear a rejoin point 60 units away.

### Splicing

`BotAvoid::Update` returns the points; `SpliceDetour` in `bot_move_nav.cpp` owns the surgery, because
it already has `MakePlanTraces`. The waypoint list is rebuilt as
`[bot origin] + [detour points] + [plan from detourReplaceUntil]` with the index reset to 0, then
`BuildPathProfile`, a fresh `TrackState`, and `lastProgressS`/`lastProgressTime` reset — the same
sequence `Repath` performs. Splicing rather than running a second follower means the detour gets the
arc profile, the speed solve, the stall detector and the status dump for free.

**The list has to start at the bot's own origin.** A profile whose first segment begins at the far
corner leaves the bot over `Config::rejoinAbort` (96) lateral on frame one, and `follow.needRepath`
undoes the detour immediately. Dropping the already-traversed waypoints matters for the same reason
`Repath` does it: a from-scratch projection over a profile that still contains the route behind the
bot can snap backwards onto it.

Detour waypoints carry `BotAvoid::kDetourClearance` rather than the default 16. `BuildPathProfile`
sizes corner arcs from clearance, and a wide arc at a detour corner cuts back into the obstacle the
corner was placed to clear. `estimatedArrivalTime` is extended by the added length so the test's
ETA-derived timeout stays honest. The splice invalidates the `wp` pointer, so the frame ends with
`RunIdleMove` and a return.

### When no route exists

Treated like a failed air move: `Nav::PenalizeConnection` on the connection the bot is trying to
cross, then an immediate repath — **but only when the penalty actually lands.**
`PenalizeCurrentConnection` returns whether it found a connection to charge, and the repath is forced
only on true. When the plan is a single area there is no connection on the route, the penalty is a
no-op, and the replan is guaranteed to return the identical plan; forcing it is not merely wasted work
but actively harmful, because `TryRepath` resets `lastProgressTime` and the stall detector below can
then never fire. See "When there is nothing to reroute". `PlanWaypoint::connIndex` is only set on airmove launches, so the
connection is recovered from `plannedConnPath` (captured from `path.connPath` in `Repath`) at the
index where `plannedAreaPath` holds `currentAreaId`.

Airmove penalties are paired with `RewardConnection` on success and walk connections need that pairing
more, not less: the blocker may be a player who moves on a second later, and without the reward one
transient obstruction poisons a corridor for the rest of the map. `RewardEnteredConnection` fires when
`currentAreaId` advances to the next planned area over a connection that carries a penalty.
`kAvoidSearchCooldown` bounds the failure loop to one penalty per second when the map genuinely has
only one way through.

`crossbot_nav_avoid 0` disables the layer, which is how to A/B a suspected regression without
rebuilding.

### Obstacles that are leaving anyway

A `func_door_rotating` has `v.velocity == 0` — all of its motion is in `v.avelocity` — so without
help it reads as a stationary wall, gets penalised, and forces a repath over an obstacle that will
have removed itself within a second. `MoverVelocityAt` gives the linear velocity of a chosen point on
a mover, `v.velocity + omega x r`, and both the side ordering and the failure handling use it.

**`avelocity` is a rate on the angles triple `(pitch, yaw, roll)` in degrees per second, and the
mapping to world axes is not the identity.** From `AngleVectors`, `forward = (cp*cy, cp*sy, -sp)`, so
at yaw 0 a positive pitch swings `+X` toward `-Z`: pitch is rotation about **+Y**. `right` at
pitch = yaw = 0 is `(0, -cos r, -sin r)`, so roll swings `-Y` toward `-Z`: roll is about **+X**. Yaw
is about **+Z** — `avelocity (0,30,0)` is 30 degrees per second counterclockwise seen from above. So
`omega = (av.z, av.x, av.y)` in radians, and getting this wrong costs a sign, which is the whole
answer.

**`r` starts at the trace contact point, then is stretched to the model's own reach.** It cannot come
from `absmin`/`absmax`: whenever a `SOLID_BSP` entity's `angles` are non-zero, `SV_LinkEdict` throws
away the real bounds and publishes a cube of the largest half-extent found over all three axes,
centred on `v.origin`. `test9_door`'s door leaf is 18 x 66 x 98 and its box is measured at
`(318,78,-2)…(450,210,130)` — a 132-unit cube centred on its own hinge. Any radius taken from that is
meaningless, and a point on the wrong side of the hinge inverts the result.

Note the condition is `angles != 0`, not "is moving": a door that has finished opening and come to a
dead stop still sits at 90 degrees, so it keeps the cube permanently. Only a door closed in its spawn
orientation gets a tight box. That, not the swing, is why `PickDestination`'s lookahead is swamped
near an open rotating door. The contact point is a real position on the leaf, so its *direction* from the axis is right —
but its *distance* is whatever the probe happened to graze, and a probe that clips the leaf near the
hinge reports a few units per second for a door doing 100 degrees per second. Scaling `r` out to
`sqrt(max|mins.x|,|maxs.x|)^2 + (max|mins.y|,|maxs.y|)^2)` treats the leaf as the bar it is. The
magnitude can only be overestimated, and the sign — the thing being decided — never changes.

**The direction to dot against is the next waypoint, not `pathDir`.** `pathDir` is the follower's aim
point, which leads the bot around corner arcs and can already be pointing at the far side of a turn
while the bot still needs to travel the near leg. On `test9_door` the aim was `(0.16,0.99)` — north,
around the corner — while the door was sweeping `(+112,-24)` east out of the corridor the bot was
about to walk down. Dotting against the aim gave `-6` and the door was called a wall; dotting against
the next waypoint gives `+102` and it is correctly called receding.

Above `kAvoidMoverSpeed` a failed search sets `AVOID_FOLLOW` instead of `AVOID_BLOCKED`, and
`bot_move_nav.cpp` skips **both** the connection penalty and the forced repath, so the bot keeps
driving at its current waypoint and walks after the door. Three things bound that: the door reaches
its stop and `avelocity` goes to zero, at which point the next search penalises normally;
`kAvoidSearchCooldown` re-takes the verdict at most once a second; and the 2 s `kRepathInterval`
stall detector is the outer net if the classification is ever wrong. A stationary obstacle scores
exactly zero and keeps the old behaviour, which is what the `closing` field in the decision line is
for.

### When there is nothing to reroute

A blocked search used to be able to livelock the bot outright, and the failure is worth understanding
because nothing about it looks wrong in isolation. On crossfire a bot printed this once a second,
forever, from a position that never changed:

```
2 avoid blocked by func_healthcharger at (464.1,368.0,-1500.0) no clear rejoin point closing 0 traces 0
2 mid-route repath (forced) at (464.1,368.0,-1500.0) wp 1/2
2 Replan from=(464.1,368.0,-1500.0) waypoints=2 areas=1 eta=0.4s | penalized=0
```

Read the three numbers together. `traces 0` says `PickDestination` gave up before probing any
geometry, which happens when every sample of the remaining path lies inside the obstacle's box padded
by `kAvoidHullHalf + kAvoidCornerMargin` — here the charger's padded box was `x 430…474, y 366…434`
and it contained the bot's own position and its goal. `areas=1` says the plan is a single area, so
`plannedConnPath` holds nothing `PenalizeCurrentConnection` can charge, which `penalized=0` confirms.
And the forced repath therefore rebuilt the same two waypoints. One second later, all of it again.

The trap is that the forced repath was also **suppressing the recovery**. `TryRepath` resets
`lastProgressTime`, the avoid failure repeats every `kAvoidSearchCooldown` (1 s), and the stall
detector needs `kRepathInterval` (2 s) of no progress — so the loop kept its own safety net
permanently one second from firing. A useless repath is not a no-op.

Two changes close it. The repath is conditional on the penalty landing (above), so `lastProgressTime`
is allowed to age. And when the stall does fire while the bot is still reporting blocked, the bot
gives up on the goal rather than repathing into the same plan: `BotItems::OnBlocked` prints what
stopped it, `NoteIgnored`s the item for `ItemUnreachableTimeout`, and clears the goal so `SelectGoal`
picks something else next frame. It is the same escalation `OnRepathFailed` already used for "no route
exists at all", which is the honest description of this situation too.

**The "still blocked" test uses its own timestamp, not `state.lastKind`.** The sticky window on
`lastKind` expires exactly at `nextSearchTime`, which with a 1 s cooldown and a 2 s stall is exactly
when the stall lands — a coin flip on frame timing. `state.blockedUntil`, set to
`now + 2 * kAvoidSearchCooldown` on every non-receding failure, covers the gap by construction.

A receding mover never reaches any of this: `hitRecedes` short-circuits before the penalty, so a bot
walking after a swinging door neither penalises nor gives up.

The root cause of that particular loop was upstream, in navgen — the charger was not in the mesh at
all, so the goal point was inside it. `doc/navgen.md` has that half. This section is about why the
runtime could not save itself from a bad goal, which is a property worth keeping fixed independently.

### Tripmine beams, which no trace can see

A tripmine beam is a `CBeam`, not geometry: it is `SOLID_NOT`, it has no bounding box, and the forward
hull trace goes straight through it. The mine detonates on a `FTRACE_SIMPLEBOX` traceline it casts
itself, from its own origin along its own normal, so what the bot has to avoid is a **line segment**
that nothing in the engine will report as a hit.

**The registry is shared and refreshed at 4 Hz**, in `BotManager::StartFrame` rather than per bot per
frame — sixteen bots times `maxEntities` every frame is the expensive way to learn something that
changes only when a mine is placed or destroyed. One entry per armed `monster_tripmine`: handle,
origin, and the beam end from a single 2048-unit traceline, mirroring `CTripmineGrenade::MakeBeam`.

Armed is `solid == SOLID_BBOX`, which the mine only becomes after its 2.5 s powerup — before that
there is no beam, and a bot that walks through the space is in no danger.

**The direction has to be un-negated.** The mine's angles come from `UTIL_VecToAngles(planeNormal)`,
whose pitch convention is the opposite of a view angle's, and `CTripmineGrenade::Spawn` recovers the
normal with `UTIL_MakeAimVectors`, which negates pitch before `MAKE_VECTORS`. The registry does the
same. Skipping that negation gives a beam that is mirrored vertically and mostly still plausible.

### The beam as an obstacle

`Probe` is the single choke point for "is this leg blocked, and by what", so teaching *it* about beams
makes `Solve`, `SolveAround` and the destination search beam-aware together. It marches the hull AABB
along the leg in 8-unit steps and slab-clips each beam segment against it; whichever of the solid hit
and the beam is nearer wins, and the beam's mine edict is reported as `tr.pHit` so the visited set and
every message keep working unchanged.

`SolveAround` and `PickDestination` take explicit bounds now, because the mine's own 16-unit box is
not the obstacle — the **beam's** bounding box is, and for a wall mount that box spans the doorway.

**The mine body and its beam must be one obstacle.** A ground-mounted mine sits 8 units off the floor,
so the hull trace hits the *body* first, and the body alone reads as a 16-tall box: under step height,
so no jump; no gap, so no duck; and `BotBreak::IsTarget` rejects `MOVETYPE_FLY`, so no break either.
The bot stood in a doorway repeating `blocked by monster_tripmine` forever. `BotDeploy::BeamFor` maps
a hit edict back to its beam, and both the forward trace and `Probe` use it, so hitting the body is
the same event as hitting the beam.

### Two predicates change, and both follow from the beam having no thickness

**The step-height floor is dropped.** `canJump` normally requires `top > kAvoidStepHeight`, because
below that the engine steps up for free. It does no such thing for a beam — a 12-unit-high beam trips
exactly like a 60-unit one — so for a beam the lower bound goes and only the reachability ceiling
remains.

**`depth` is the bot's own 32, not `min(box.x, box.y)`.** A wall-to-wall beam's AABB spans the
corridor, and feeding that width into the crawl-hold would ask the bot to stay ducked for the length
of the room.

The three outcomes then fall out of the existing ladder, with `gap` and `top` taken from the z range
of the beam *inside* the hull rather than from a box:

- a beam 40 or more above the feet clears the 36-tall duck hull, so it is **crawled under**;
- a beam above 72 is not an obstacle at all and never enters the ladder;
- anything else is **walked around** by the side search, and shot if that fails. A **vertical** beam
  spans the whole column, so `gap` is 0 and `top` is 72 and it always lands here.

### A beam is never jumped, and the first version got this wrong

`canJump` originally dropped its step-height floor for a beam, on the reasoning that a beam has no
thickness so the usual "the engine steps over anything under 18" exemption does not apply. That much
is true. The conclusion drawn from it — that a low beam should therefore be *jumped* — is not.

**You cannot jump a tripmine beam.** The trip test is a traceline against the player's simple box, so
it fires on any frame where the box intersects the beam, and a jump sweeps the box up *through* the
beam plane before the feet ever clear it. Ducking works because the ducked box never reaches the beam
height at all; jumping has no equivalent, because the box has to pass through the beam to get above it.

The instrumented run that found this is unambiguous: both `walk`-state deaths in the baseline read
`avoid jump vs monster_tripmine` — the bot decided to jump the beam and died doing it. `canJump` is
now `!beam && top > kAvoidStepHeight && ...`, and the `avoid jump monster_tripmine` count went to zero
and stayed there.

The consequence is that a ducked placement (beam ~30 above the floor, `gap` 26, under the 40 needed to
crawl) has no passive answer at all — it must be walked around or destroyed. That is why the standoff
shot fires an order of magnitude more often than it used to, and it is correct: a knee-high laser
across a doorway really does have to be shot.

**When nothing works, shoot it.** Where the side search fails, `TakeMineTarget` sets `mineTarget`
instead of `breakTarget`, and `Bot_MoveNav` hands it to `BotBreak::EngageStandoff` at
`kMineSafeRadius`. `TakeBreakTarget` cannot serve here: it requires `IsTarget`, which excludes
`MOVETYPE_FLY` deliberately, and a crowbar at melee range against a 150-damage blast is not a plan.

**Own mines are not exempt, and must not be.** A tripmine's beam-break test traces from the mine, and
`pev->owner` is the wall it is stuck to, so the player who placed it breaks the beam like anyone else.
There is no owner filter anywhere in this path.

**Placed satchels get no code at all.** A live `monster_satchel` is 8 units tall, well under the
engine's 18-unit step, so the bot walks over it and `HitQualifies` is left alone.

### The guard in Bot_RunPlayerMove, and why the ladder was not enough

`BotAvoid::Update` runs in exactly one place: the walk drive at the end of `Bot_MoveNav`. Every
maneuver that owns the frame — combat, break, deploy, ladder, elevator, airmove, charger — returns
before it. So a bot in any of those states had **no beam avoidance whatsoever**, and the measurement
says that is where the deaths were: of nine own-mine kills in the instrumented baseline, six were in
`maneuver combat` and reported `avoid none`. The bot ran through its own lasers while fighting.

The fix is a guard in `Bot_RunPlayerMove`, which is the one funnel every subsystem's movement passes
through. It reconstructs the intended world direction from `fmove`/`smove` (`ComposeMove`, the inverse
of `Bot_DecomposeMove`), asks `BotDeploy::BeamGuard` whether that step enters a beam, and if so forces
a duck when the ducked hull would clear it and refuses the step otherwise. Putting it here rather than
in each subsystem is what makes the rule unconditional: no maneuver can forget it.

**Two details are load-bearing.**

**It must ignore beams the hull is already inside.** A bot placing a tripmine is standing on its own
beam line by construction (see `bot_deploy.cpp` below), and that beam is registered from spawn. A
guard that stopped the bot for beams it already occupies would pin it inside its own mine until the
mine armed around it — turning the placement path into a suicide. `BeamGuard` tests each beam at the
current origin first and skips any that already meet the hull; it only refuses steps that would enter
a beam the bot is *out* of.

**Zeroing the input does not stop the bot.** The first version cleared `fmove`/`smove` and the movement
buttons, and bots kept dying in combat with `onbeam 1`: at 250 u/s a bot coasts for a third of a second
on momentum alone, which is far enough to cross a doorway. The guard now drives *against* the current
velocity at full speed — an actual brake, not a release. That single change took contact deaths from
two per run to zero.

The margin lives in the beam's own XY extent rather than in the lookahead. `BeamMeetsHull` takes a
`pad` that inflates the hull box horizontally, so `BeamGuard` (`kDeployGuardPad`) and `BeamAlong`
(`kAvoidBeamXY`) each get room to stop or decide without either needing its own timing constant, and
the diagnostics keep `pad` at 0 so they still report true contact. Padding z would be wrong — it would
make a beam above head height read as an obstacle.

## bot_break.cpp

Owns the frame while the bot is destroying something, the same way `RunElevatorRide` and
`RunChargerUse` do: aim, drive, buttons and `Bot_RunPlayerMove` all come from here, and `Bot_MoveNav`
returns. It is called *before* the progress/stall/repath block so a repath can never interrupt a swing,
and it refreshes `lastProgressTime` every frame so the 2 s stall detector and `BotItems::OnBlocked`
stay off its back — without that the bot would throw away the goal it is two swings from reaching.

### What counts as breakable

`IsTarget` is `SOLID_BSP` or `SOLID_BBOX`, `MOVETYPE_PUSH` or `MOVETYPE_PUSHSTEP`, not a player or
monster, `takedamage != DAMAGE_NO`, and `0 < health <= kBreakMaxHealth`. There is deliberately no
classname test: `takedamage` and `health` are the properties that actually decide it, and dropping the
classname picks up mod breakables for free. Nothing that is meant to be avoided qualifies — `func_wall`,
the chargers and the doors are all `DAMAGE_NO`.

`takedamage` is the exact inverse of `SF_BREAK_TRIGGER_ONLY`: `CBreakable::Spawn` sets `DAMAGE_NO` for
that flag and `DAMAGE_YES` otherwise, which is the same distinction navgen makes with
`kSfBreakTriggerOnly`.

**The movetype test is what makes `SOLID_BBOX` safe, and `SOLID_BBOX` is not optional.**
`func_pushable` spawns `SOLID_BBOX`, and with `SF_PUSH_BREAKABLE` (128) it runs `CBreakable::Spawn`
first, so it is `DAMAGE_YES` and the map author has explicitly said it may be smashed. Excluding it
wedges the bot: `test13_break` has two, and a bot that has just broken through the wall in front of
them meets them with `no clear rejoin point closing 0 traces 0` — the padded box contains the bot's own
position, exactly the shape of the charger livelock. But `SOLID_BBOX` alone would also admit live
satchel charges, which spawn `DAMAGE_YES` with `health` 1. `MOVETYPE_PUSH`/`MOVETYPE_PUSHSTEP` is the
line between a brush entity and a thrown prop: grenades and satchels are `MOVETYPE_BOUNCE`, tripmines
`MOVETYPE_FLY`, monsters `MOVETYPE_STEP`.

`health <= 100` is the request, and the numbers back it: `CBreakable::TakeDamage` doubles `DMG_CLUB`,
so a crowbar lands `2 * plrDmgCrowbar` = 50 per swing, always the full-damage branch under
`IsMultiplayer()`. A qualifying target dies in at most two swings, which is why no interaction with
combat is needed — the whole engagement is under a second.

**`takedamage != DAMAGE_NO` is necessary but not sufficient, and nothing can make it sufficient.**
`TakeDamage` early-returns on `!IsBreakable()`, so `matUnbreakableGlass` keeps `DAMAGE_YES` and
silently absorbs everything. `m_Material` is private data with no gamedata offset, so this cannot be
tested up front. It is caught instead by watching `pev->health` fail to drop: after the first swing,
`kBreakNoDamageGrace` seconds without the health moving aborts the attempt.

### Giving up

`Abort` fires on the no-damage grace or on `kBreakTimeout`, and does three things beyond restoring the
weapon. It records the entity in `refused` for `kBreakRefuseTime` so the next frame does not re-engage
the same brush. It calls `Bot_PenalizeCurrentConnection`, so the connection through the breakable gets
more expensive and the planner routes around it. And it forces a repath to spend that penalty
immediately. This is the whole answer to breakables navgen cannot classify offline — unbreakable glass
and anything over 100 health are discovered once, at runtime, and then avoided.

### The weapon

`IGameProfile::MeleeWeapon()` names it; HLDM returns the crowbar. If the bot is not carrying it the
active weapon is used instead and the bot shoots the thing, which is worse for ammo but never wrong.
The switch is `FakeCmd::Run(pEdict, spec->classname)` rate-limited to `kBreakSelectInterval`, the same
mechanism and the same interval `BotFire::Think` uses. `prevWeaponId` is captured at `Engage` and
redrawn on both `Finish` and `Abort`; if the weapon is gone by then nothing happens and `BotFire`
re-picks within half a second anyway.

**`BotFire::Think` early-outs while a break is active**, and it has to. `Bot_RunPlayerMove`
unconditionally ORs in `BotFire::Buttons`, and `Think` re-rolls a weapon at random every half second
whenever it has a target — it would fight the melee switch for as long as the bot could see anyone.
Idling it leaves `fire.mode == MODE_NONE`, so `Buttons` returns 0 and `BotBreak` is the only source of
`IN_ATTACK`.

### Aim and reach

`CCrowbar::Swing` traces from `GetGunPosition()` along `v_forward` for 32 units, so reach is measured
**eye to surface**, not eye to centre. The aim point is the eye clamped into the entity's
`absmin`/`absmax` and then nudged `kBreakNudge` toward the centre, so the traceline ends inside the
brush rather than grazing its face. A bot whose eye is already inside the box — which happens, the
hull overlaps before the trace does — falls back to aiming at the centre.

### The standoff variant

`EngageStandoff(bot, ent, minRange)` is the same machine with the approach inverted, and it exists for
tripmine beams (`bot_deploy.cpp`). It shares `Run`, the weapon save/restore, the `lastProgressTime`
hold and the `Abort` path; the differences are four.

**It bypasses `IsTarget`.** A tripmine is `MOVETYPE_FLY`, which `IsTarget` excludes on purpose — that
exclusion is what keeps live satchels out of the melee path, and widening it would undo that. The
standoff caller names its target explicitly instead.

**`inRange` inverts.** Melee wants `los <= kBreakReach`; standoff wants `los >= standoff`, and the
drive direction flips with it, so `moveDir` is negated and the bot backs away along the line it came
in on. `kMineSafeRadius` is 375 because that is where a 150-damage blast reaches zero — the bot is
retreating out of its own explosion, not merely out of reach.

**A corridor shorter than the standoff must not deadlock.** `kBreakRetreatTime` (4 s) forces `inRange`
true once the retreat has had its chance, so a bot backed against a wall shoots anyway and eats some
blast. That is strictly better than standing there. On `test2_doorway` it is the only case — the map
is smaller than 375 units end to end, so every observed standoff fires on the timer and still kills
the mine in one shot.

**The weapon choice inverts too.** `WantWeaponId` returns the melee weapon for a break; for a standoff
it returns the active weapon unless that *is* the melee weapon, in which case it takes the first
non-melee weapon carried. A crowbar at 375 units hits nothing.

A tripmine is `health == 1` and `takedamage == DAMAGE_YES`, so one hit anywhere destroys it, and
`Killed` -> `DelayDeathThink` -> `Explode` is the expected outcome rather than a failure. The
no-damage grace still applies and still means what it means: shots that are not landing.

## bot_deploy.cpp

Placing tripmines and satchels, and detonating satchels. It owns the frame the way `BotBreak` does —
aim, drive, buttons and `Bot_RunPlayerMove` all come from here and `Bot_MoveNav` returns — and
`BotFire::Think` idles with `why=deploying` while it runs, for the same reason it idles during a
break: `Buttons` would otherwise fight the weapon switch every half second.

It is called from `Bot_MoveNav` immediately after `BotBreak::Run`, before goal selection.
`Bot_ManeuverActive` includes `deploy.active`, which is what stops `TryStart` re-entering and what
keeps `BotCombat::Run` from tearing the mine placement out from under it.

### Why the deployables never fit in BotFire

`BotFire::Think` returns `no-target` several branches before any mode is considered, and both
deployables fire with **no target at all**. "What is not here" named this: they want a decision layer
that runs on the map rather than on an enemy. That layer is this file. The `Supported` gate was never
widened — `AIM_SURFACE`, `AIM_NONE` and `WPN_Q_HIDDEN_STATE` are still rejected there, and nothing
about firing changed.

### The damage rule is one number, and it comes from the SDK

`sk_plr_tripmine` and `sk_plr_satchel` are both 150. `CGrenade::Explode` calls
`RadiusDamage(pev->dmg, pev->dmg * 2.5)`, so the radius is 375 and the falloff is `150/375 = 0.4` per
unit: damage at distance `d` is `150 - 0.4d`. Two constants fall out and both are load-bearing.

- **`kDeployKillRadius` is 125** — where the blast still does 100. This is the whole meaning of "a gate
  small enough that anyone in it dies", and `Covers` applies it directly rather than through a width
  threshold, sampling the gate cross-section at three points along the span and three body-centre
  heights and requiring every sample within 125 *with line of sight to the mount*.
- **`kMineSafeRadius` is 375** — where it does nothing. That is the standoff range in `bot_break.cpp`.

### The mesh already knows where the chokepoints are

Every walk connection carries `gateA`/`gateB`, a two-endpoint floor segment per side, and they survive
the `.nav` round trip. **No format change and no map regeneration was needed for any of this.** The
gate supplies the crossing point and the beam direction; it does not supply a usable width.

**The stored gate is inset twice, and the second inset is easy to miss.** navgen builds areas by
carving the *hull 3* clipnode tree, so area boundaries already sit a player radius inside the real
walls; `GateFromEdge` then insets each end by `kPlanClearance` again. So the true opening is
`|left - right| + 4 * kPlanClearance`, not `+ 2`. Getting this wrong is invisible — every number is
still plausible — which is why the physical measurement below is what the code actually trusts.

### A short gate is not a chokepoint

The first version accepted a gate on `|left - right|` alone and found **1238 of them on crossfire**.
Two mesh seams across an open floor produce a short gate for the same reason two areas are small, and
a mine there traps nobody. Worse, the ceiling/ground fallback rescued every one of them: there is
always a floor below you.

`EvaluateGate` measures the opening instead. From the eye at the gate midpoint it traces both ways
along the gate direction; **both sides must hit something** within `kDeployReach`, and
`opening = distA + distB` must be at most `kDeployOpeningMax` (144). Crossfire drops to 99, and the
opening histogram is 96 and 128 — Half-Life's two standard doorway widths, which is the sanity check
that matters more than the count.

### The gate direction, and the clamped case that had none

For an ordinary gate the direction is `normalize(right - left)`. A **clamped** gate — one whose raw
opening was under 32, so `GateFromEdge` collapsed it to a single point — has `left == right` and no
direction at all.

Guessing it from the two areas' centroids is wrong often enough to matter: on `test2_doorway` every
gate is clamped, and the centroid guess measured the openings diagonally as 113 when they are 64, then
rejected all four for `no-surface`. **The far side's gate midpoint is the right source.** `gateA` and
`gateB` are the two sides of the same portal, so `gateB.mid - gateA.mid` is the crossing normal by
construction, and the gate direction is its horizontal perpendicular. Centroids remain the last
fallback. This was not only a count fix — it fixed the measurement, and it raised every map
(`stalkyard` 93 -> 114, `datacore` 91 -> 118).

### Where the mine goes, and why the wall comes first

`CTripmine::PrimaryAttack` traces 128 units from the gun position along `v_forward`, spawns
`monster_tripmine` at `tr.vecEndPos + tr.vecPlaneNormal * 8`, and sets its angles from the plane
normal — so **the beam runs back out along the surface normal**. A beam that crosses a doorway
therefore needs a mount on a *side* wall, one whose normal lies in the gate plane. That is what
"a wall with normal parallel to the gate" means, and `Probe` enforces it: the hit's normal must be
within `kDeployNormalTol` (~20 degrees) of facing back down the trace.

Preference is wall, then ceiling, then ground, and the vertical two are gated behind
`opening <= kDeployVerticalMax` (48). The gate is the point: a vertical beam is a line, and in
anything wider than about a player's width people simply walk past it. Forced open, ground mounts in
crossfire's 128-wide doorways were stepped around without the avoidance system ever engaging.

The bot stands **at the gate midpoint**, and `EvaluateGate` probes from exactly that point. Evaluating
from anywhere else stops the pre-scan predicting what the shot will do.

### The arming window, and the trace that captures you

`CTripmineGrenade` is `SOLID_NOT` with no beam entity at all for its first 2.5 seconds
(`Spawn` sets `m_flPowerUp = time + 2.5`; `PowerupThink` runs every 0.1 s until then). At arming it
goes `SOLID_BBOX` and calls `MakeBeam`, which stores `m_flBeamLength = tr.flFraction` from a trace
made with **`dont_ignore_monsters`**. `BeamBreakThink` then retraces every 0.1 s and detonates on
`fabs(m_flBeamLength - tr.flFraction) > 0.001`.

The consequence is sharper than "a beam you cannot see yet". **If a player is standing on the beam
line at the moment of arming, the baseline is measured to their body** — the mine arms *around* them
and fires the instant they step off. Being inside at arming is not a risk; it is a guaranteed kill on
departure.

And the bot is always standing on that line: `out.stand` is the gate midpoint, the probe eye is that
midpoint plus 46, and the mine mounts where that ray hits the side wall and fires back along the wall
normal — straight through the eye. No mount geometry avoids this, because a beam leaves along the
surface normal, so any beam placed from where the bot stands returns to where the bot stands. The only
beams that would miss it are above its head or below its feet, and those are not traps.

`RefreshBeams` therefore registers mines **from spawn**, not from arming, with `Beam::armed` recording
which state they are in. An unarmed beam is one that is about to be lethal, and there is no version of
"walk through here" that is correct for it. The `SOLID_BBOX`-only filter it used to apply made the
registry blind for precisely the window in which the bot occupies the beam.

**Measurement disagreed with the theory, and the theory lost.** The instrumented baseline shows the
bot clearing its own beam in **0.13 to 0.28 seconds** — an order of magnitude inside the 2.5 s it has —
and `STILL IN BEAM AT ARMING` fired once in 73 placements and killed nobody. The arm-around-you case is
real, and the pre-arm registry closes it for free, but it was never what was killing bots. What was
killing them is in `bot_avoid.cpp` above: no avoidance during combat, and a beam the ladder decided to
jump.

### Keeping the laser inside the doorway

`Probe` records the surface `normal`, which is the direction the mine will fire, and `BeamContained`
traces `kDeployBeamLength` from the candidate mount along it and requires the hit within
`opening + kDeployBeamSlack`. A mount whose beam runs long is skipped and the opposite wall is tried;
if neither contains its beam the gate is rejected as `REJECT_ESCAPE`, which `crossbot_list_gates`
prints like any other reason.

Nothing checked this before, and `kDeployNormalTol` (0.94, so nearly 20 degrees of slop) is loose
enough for it to matter: across a 128-unit opening that is up to ~46 units of lateral drift, enough to
miss the far jamb entirely and run down a corridor for the full 2048.

A long beam is its own hazard, and not the obvious one. The **blast is at the mine** while the trip
point can be hundreds of units away, so a stranger crossing the far end of a runaway beam kills whoever
is standing in the doorway. That is the shape of most of what is left: after the avoidance fixes, the
remaining own-mine deaths are `onbeam 0` — the bot was 40 to 240 units from its own mine, not touching
the beam, when something else set it off.

This check subsumes the weaker proxy of demanding a facing wall on *both* sides. If the far side is not
really a wall, the beam runs long and the gate is rejected on its own evidence rather than on a normal
test that only ever looked at the near side.

### Stance is rolled, and that is what makes the avoidance visible

Standing, the eye is 64 above the floor; ducked it is 30 — `VEC_DUCK_VIEW` is 12 above a duck origin
that sits 18 up. The mine mounts at eye height, so the beam does too, and the two stances land on
opposite sides of every avoidance predicate: a beam at 64 is crawled under, a beam at 30 is jumped.
`TryStart` flips a per-bot coin. Without it every horizontal beam is a crawl-under and the jump branch
never runs outside a test map.

Nav areas are in duck-origin space, so the gate's own z is `floor + 18` and the eye offsets from it
are 46 and 12 rather than 64 and 30.

### The phases, and the one that is not optional

`APPROACH` -> `SELECT` -> `AIM` -> `FIRE` -> `VERIFY`, plus `DETONATE` entered from its own trigger.
The chassis is `RunActivation` almost verbatim, including the three details that are easy to lose:
`lastProgressTime` is refreshed every frame or the 2 s stall detector repaths mid-placement; the press
is gated on the angles `Bot_UpdateFacing` *returned* rather than the ones requested, because the aim
controller is rate-limited; and the approach drive is `NavFollow::SolveDrive` + `Bot_DecomposeMove`
straight at the spot. Straight-line drive is safe because an area is a convex prism and the gate
midpoint lies on its boundary.

`VERIFY` asks a different question per weapon, and the obvious one is wrong for the satchel. A
tripmine is confirmed by a `monster_tripmine` within 48 of the predicted mount. A satchel is **not**:
it leaves at 274 u/s and is 80 units away before the 0.3 s verify delay is up. It is confirmed by
`m_chargeReady == 1`, which is the game's own record that a satchel is deployed.

**The memo is not optional.** Every exit writes `Memo { connIndex, expireTime }`, success or failure,
and `FindBest` honours it. Without it the bot re-picks the gate it just failed at, walks zero
distance, and spins at full framerate exactly where it wants to be — the failure mode described for
`BotItems::OnArrived`, and no stall detector catches this one either.

### Satchels are dropped, not lobbed

The satchel shares the whole chassis with the surface search skipped: the spot is the gate midpoint,
the coverage test runs from the floor rest point, and the press is `IN_ATTACK2`.
`CSatchel::SecondaryAttack` throws for any `m_chargeReady != 2`, so the throw needs no state knowledge
— only the detonation does. Tripmine is preferred; a satchel is placed only when there is no tripmine
ammo, and `CSatchel::AddDuplicate` refuses new satchels while any are deployed, so the live group is
self-limiting.

### Detonation, and the contact that was the bot's own satchel

The trigger runs every frame in `TryStart`, ahead of the placement throttle: for each live
`monster_satchel` the bot owns, fire when a live suspicion sits within 375 of it **and** closer to it
than the bot is, out of combat and with no fire commit open.

`BotSense` had no spatial query — `Freshest` and `LiveCounts` have no position filter — so
`NearestLive` was added beside them, running the same `[0, count)` scan through the file-static
`LiveWeight`, keeping the file's one-liveness-test rule.

**`livingOnly` is the whole feature.** Without it the first run detonated 181 times off 7 satchels,
every one reporting `contact 2 away`: `monster_satchel` plays `weapons/g_bounce*.wav` when it lands,
`BotSense::Hear` pushes a suspicion for the emitter, and the bot blew up its own charge because it
heard it hit the floor. `NearestLive` now skips `KIND_OTHER` and skips any entity owned by the bot.
"An entity near the satchel" has to mean something that can be killed.

**And with teams on it also has to be an enemy.** `livingOnly` now means "something that can be killed
*and* that this bot may shoot at", which is what stops a bot blowing a charge under a teammate walking
past it. The filter is inside the scan rather than at the call site on purpose: `NearestLive` returns
only the nearest match, so a teammate standing closer to the satchel would otherwise mask a real enemy
inside the same 375 units and the charge would never fire.

**The press must be confirmed, not assumed.** The same run counted a detonation on the frame it
pressed and then re-fired forever. `DETONATE` now presses only when `m_chargeReady == 1` and both the
player's and the weapon's own attack timers are clear, then waits `kDeployVerifyDelay` and calls it
done only if `m_chargeReady` has left 1.

### crossbot_deploy_surface

`0` is off; `1`/`2`/`3` pin wall/ceiling/ground and bypass the `kDeployVerticalMax` gate. It exists for
the same reason `crossbot_fire_pin` does: vertical mounts are rare by design — a 48-unit opening is
uncommon, and across six stock maps the unpinned scan finds none — so the side-search and standoff
branches are unreachable in a normal run and would ship untested. Pinned to ground on `test2_doorway`
they are reached every time.

### crossbot_list_gates and crossbot_deploy_status

`crossbot_list_gates` dumps every area's walk gates with the mesh width, the measured opening, the
chosen surface, the mount point and the rejection reason. It prints a gate that reached the surface or
coverage stage even when it failed, and stays quiet about ones filtered earlier, so the interesting
rejections are visible without burying them.

`crossbot_deploy_status` is per bot: phase, target connection, measured opening, surface, stance,
`placed`, `dets`, `aborts`, `selfhits`, `escapes`, memo size and `why`. Unlike the fire counters these are **not** per life —
`OnRespawn` clears the memo and the active placement but leaves the tallies alone.

## Trigger Forces: gravity and push

### trigger_gravity

A brush entity (`CBaseTrigger`, classname `trigger_gravity`) that changes the player's gravity multiplier when touched. The bot's `pev->gravity` field stores this multiplier as of the last touch. It persists until the next respawn, and it decides which air moves in the mesh are usable.

When the bot spawns, the game DLL resets `pev->gravity` to 0 (default). Any trigger_gravity has to be touched again to take effect. Because `trigger_gravity` sticks to whoever touches it, a bot only ever runs at one of these multipliers at a time — the default (gravity 800) plus one per distinct trigger_gravity on the map.

This affects routing: `trigger_gravity` multipliers filter air moves the same way `PlanWalker::Cross` does during planning, so goal selection and route search agree about which jumps exist. Only default-gravity air moves count as an alternative route during pruning; a jump that exists solely at some `trigger_gravity` value is not a route the bot can generally rely on.

During navgen sampling, a full air move set is sampled per multiplier found on the map. At plan time, only the air moves baked for the gravity we are currently at exist for us. This means the runtime's route search honors the gravity the bot is actually under.

### trigger_push

A brush entity (`CBaseTrigger`, classname `trigger_push`) that provides a base velocity to the player for as long as it is being touched. Unlike trigger_gravity, it does not persist after leaving the field.

`trigger_push` hands the player a base velocity that gets added to the movement simulation. The bot snapshots the live fields once per plan; a plan runs long enough that a toggled field is caught by the next repath.

In navigation, `trigger_push` is `SOLID_TRIGGER`: it never blocks, carves or supports the nav mesh. It only shows up in the movement simulation during air move sampling and as a launch seed.

### Physics integration

The base velocity contributed by trigger_push fields overlapping the bot's origin is added at plan time. In the physics simulation (`pm_sim.cpp`), `PM_AddCorrectGravity` folds the vertical push component into velocity as an acceleration and clears it from the base velocity, so only the horizontal remainder wraps `FlyMove`. The horizontal base velocity wraps the move the way `PM_AirMove` and `PM_WalkMove` do: added before the clip-and-slide loop, pulled back out once the ground check has seen it.

During navgen air move sampling, the trigger touch test uses the same point-test of the player origin against trigger hull-expanded clipnodes that the engine uses in `SV_TouchLinks`.

### START_OFF fields

A trigger_push with the `START_OFF` spawnflag (1024) spawns as `SOLID_NOT`, so nothing touches it until something turns it on via trigger.

### Push field seeding

A push field sitting on flat floor has no perimeter edge with void ahead, so the perimeter sweep never launches there. For these fields, the bot's own footprint boundary is seeded instead, aimed inwards at the field. This accounts for a bot running into the shaft and jumping.

Push-field seeds have to launch at the boundary: backing up first only spends the boost.

### Preview path at lowest gravity

A bot only ever holds one gravity multiplier. The lowest one the map hands out is the one that unlocks the most air moves. During offline path preview (`--dump-path`), navgen samples at the lowest gravity multiplier found, so a map whose route exists only under a trigger_gravity does not read as unroutable offline.

