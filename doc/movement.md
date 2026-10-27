# Movement

How the bot moves: the frame clock, the one function every movement command goes through, the nav drive and
the profile tracker and velocity servo it steers with, the maneuvers that take over the frame (air moves,
ladders, elevators, activations, chargers), and the layers that react to what the plan cannot see (obstacles,
breakables, tripmine beams, floor hazards). The mesh and the plan itself are `doc/path_planning.md`.

## The frame clock (`bot_move.cpp`, `bot_timing.cpp`)

### Bot_BeginFrame and Bot_FrameTime

**Bot code must never read `gpGlobals->frametime`.** `Bot_BeginFrame` latches it at the top of
`BotManager::StartFrame`, before any bot has moved, and `Bot_FrameTime()` / `Bot_FrameMsec()` hand out that
value for the rest of the frame. Every per-frame integration reads the latch: `Bot_FrameMsec`, the aim filter's
`dt`, `AirmoveLaunchLead`, the weapon-usage accumulator.

Why: a fake client gets no engine physics — `SV_Physics` skips every edict with index <= maxclients — so a bot
advances only by the msec handed to `pfnRunPlayerMove`, and its clock is entirely ours. `PF_RunPlayerMove_I`
calls `SV_RunCmd`, which sets `gGlobalVariables.frametime = ucmd->msec * 0.001` before `pfnPlayerPostThink`
(`sv_user.cpp:1055`) and never restores it. The engine only puts the real value back at the top of `SV_Frame`
and after each `SV_ExecuteClientMessage` in `SV_ReadPackets`, so inside one `pfnStartFrame` the second bot reads
the first bot's command length, the third the second's, and so on.

That is invisible while every bot sends the same msec, and catastrophic when one in the middle sends a short
command:

```
CrossBot TIME chain (frame 10.04ms): bot1[saw 10ms]=10ms ... bot6[saw 10ms]=10ms
  bot7(dead)[saw 10ms]=1ms bot8[saw 1ms]=1ms bot9[saw 1ms]=1ms ... bot12[saw 1ms]=1ms
```

Reading the global, one dead bot pressing `+attack` with a token 1 ms command runs every later bot at a tenth of
real speed for as long as it stays dead. Movement and gravity slow together, because both integrate the same msec,
so the symptom is "wading through honey", not lag. With a dozen bots someone is nearly always dead. The 50 ms jump
split and the engine halving commands over 50 ms are the same effect at lower server framerates.

### Movement clock diagnostics

`crossbot_time_debug 1` prints, per bot per second, and a once-per-second frame chain like the one above:
- the host frame time actually elapsed (the budget);
- the msec issued, over how many `pfnRunPlayerMove` calls;
- how much the engine actually executed — `SV_RunCmd` silently drops commands while
  `host_client->ignorecmdtime > realtime` (the `clockwindow` speedhack guard), which the game DLL can only see as
  `PlayerPostThink` not firing;
- the drift between bot sim time and real time, the same quantity `SV_CheckCmdTimes` compares against
  `clockwindow`.

## Actuation (`bot_move.cpp`)

### Bot_DecomposeMove

Splits a world-space direction into the forward/side speeds `pfnRunPlayerMove` takes, which the engine reads along
the view axes — this is what lets the bot move somewhere other than where it looks.
- `viewYawDeg` must be the yaw passed to `pfnRunPlayerMove` in the same call, plus punchangle, because
  `PM_CheckParamters` builds the movement basis from exactly that. `PM_WalkMove` and `PM_AirMove` flatten
  forward/right and renormalize, so pitch and roll drop out and the basis is the yaw alone.
- `PM_CheckParamters` clamps the pair to maxspeed before anything reads it, so a `driveLength` well over maxspeed
  means "full speed exactly this way" (that is `kFullDrive`, 1000), and one below it passes through and becomes
  the actual speed.
- `ComposeMove` is the inverse, used by the beam guard below.

`Bot_NormalizeAngle` (wrap into (-180, 180]) lives here too, so both movement translation units share one copy.

### Bot_RunPlayerMove

**Every movement command goes through this one function.** It takes `CBot &` rather than `edict_t *` because it
is also where the fire button is added (`doc/combat.md`, "Why the trigger lives inside Bot_RunPlayerMove") and
where the tripmine beam guard sits (see "The guard in Bot_RunPlayerMove"). Being the one funnel is what makes
both rules unconditional.

**Long commands are split so a jump costs one frame of held button.** The engine halves any usercmd longer than
50 ms and runs `PM_PlayerMove` once per half, so a button held in a long command is re-read after the bot has
moved. `IN_JUMP` is the one that matters: `PM_LadderMove` reads it with no `oldbuttons` gate and shoves the player
270 units off the ladder plane, so a jump that merely *starts* out of a ladder's reach ejects the bot the moment a
later sub-step brings it into contact. Below the split threshold this is exactly the single call it replaces. A
fire button is held, never edged, so splitting it changes nothing.

**`IN_DUCK` + `IN_JUMP` in one command is a long jump while the bot holds `item_longjump`, so the duck is
dropped.** `PM_Jump` (`pm_shared.c:2554`) reads `cansuperjump` from the `slj` physinfo key, and when the command
carries `IN_DUCK` with `flDuckTime > 0` and `Length(velocity) > 50` it replaces the jump with
`forward * 350 * 1.6` horizontally and `sqrt(2*800*56)` up. `PM_Duck` sets `flDuckTime = 1000` on the press, so a
crouch jump's first frame is always inside the window, and the bot flies roughly twice its planned arc. The
check is one `pfnGetPhysicsKeyValue`; gating on it keeps every map without the module byte-identical. Only the
opportunist's launch frame opts out, through `allowLongJump` (see `bot_opportunist.cpp`).

Delaying the duck one frame costs nothing because of call order: `PM_Duck` (2982) runs before `PM_Jump` (3038),
and a duck pressed on the ground cannot finish that frame — `PM_FinishDuck` needs the 0.4 s `TIME_TO_DUCK` ramp or
`onground == -1`, and the jump clears `onground` only after `PM_Duck` returned. So the launch frame uses the
standing hull either way, and the first airborne frame finishes the duck either way (`bInDuck` survives into the
air, or the press finishes it). `JumpLandsIn` assumes the duck hull for the whole arc, and that holds from the
same frame either way. Jump height does not depend on the duck: both `PM_Jump` branches give the same 45-unit
impulse.

### Teammate spacing

`ApplySpacing` bends the commanded move away from every live teammate (`Teams::Of == REL_FRIEND`) within
`kSpacingRadius` (64) in 2D and `kSpacingHeight` (72) vertically, so a floor above does not count. Each teammate
contributes its away direction weighted `1 - d/R`, fading to nothing at the radius; two bots on the same spot split
along a golden-angle direction from their entity index. A standing bot walks off at `push * maxspeed`; a moving bot
keeps its speed and has its direction bent, so the follower sees a lateral nudge, not a new heading.

It lives in the actuation funnel because clumps form in every drive — spawn holds, camp spots, goal arrival,
combat, two bots in one corridor — and one call site covers them all. It runs before `MoveAdjust` and the beam
guard, so both still win.

**A friend is a live friend.** `CBotIsLivePlayer` only means "a connected player", so this loop and `LineBlock`
also skip `IsDead`. A dead DoD teammate in chase cam (`iuser1 > 0`, `doc/games.md`) sits on the very origin of the
bot it watches and, counted, pushes it at full weight for the whole respawn wave; where that opposes the path the
residual flips sign every frame — a bot snapping left and right in third person at ~20 u/s with a stall repath
every 2 s.
`scripts/test_wiggle.ps1` detects it.

It is skipped wherever a sideways nudge breaks a maneuver: in the air, off `MOVETYPE_WALK`, swimming, any command
carrying `IN_JUMP`, riding a lift or train (`rideActive`, `fun.aboard`), `ladderActive`, `airmoveActive`, and while
`BotFire::Brace` has planted the bot for a shot. A push whose 24-unit probe leaves the nav mesh is dropped, so a bot
is never shouldered off a ledge.

### Teammate line of sight

`LineBlock` tests the 2D segment from the bot to a target against every live teammate between the two ends
(`0 < along < len`, within `kSpacingHeight` of the segment there). A friend blocks with weight
`1 - |lateral| / halfWidth` and wants the bot to step to the side *opposite* it, the shorter way past.

- All blockers on one side: step to the other side at the strongest weight. Blockers on both sides: give up and go
  straight (`blocked both sides, goes middle`).
- A friend within `kLineSticky * halfWidth` (half the band) of the line keeps last frame's side (`CBot::lineSide`),
  falling back to entity-index parity, so the bot passes on its chosen side instead of turning back. Each change of
  `lineSide` logs one verbose line; a stream of them is oscillation. Why the band is this wide: a 4-unit band
  flips right-left-right within a few frames in a three-bot follow chain, because the middle bot's sidestep walks
  it across the tail bot's line.

Two users, both only on the ground and only while teams are active:
- **`Bot_ClearLineOfSight`**, from the nav drive just before the move is decomposed: target the current waypoint,
  `halfWidth` 24. The move is turned up to 45° scaled by the weight — a drift that stops about a box-width off the
  line.
- **`Bot_ClearLineOfFire`**, from `BotCombat::Run`: target the enemy, `halfWidth` 64. It adds a sideways strafe of
  `weight * maxspeed`, which also moves a bot standing still in a duel.

Both drop the change if the 24-unit probe in the new direction leaves the mesh (`SpacingProbeOnMesh`).

## The nav drive (`bot_move_nav.cpp`)

### Repath

`Repath` plans from the bot's current area to the goal (`PlanPath`, `doc/path_planning.md`), builds the profile and
computes the ETA. The goal point is clamped into the goal area, and the plan ends at the clamped point.

Two log lines, and the distinction matters when reading a failure:
- a fresh path logs `Repath` plus every waypoint (jump/crouch flags, target areas) and one
  `conn[i]=<index> <type> <a><->b cost= travel=` line per connection. Waypoints alone do not say *which edge* was
  taken — two areas can be joined by a walk, a jump and a ladder at once — and the connection index is what every
  penalty, abort and `wanted <area>` message is keyed on;
- a mid-route re-plan logs the one-line `Replan`, so a re-plan after a failed air move is not silent.

Both carry `penalized=`, the number of connections on the chosen route with a nonzero `runtimePenalty`;
`penalized=0` after a failure means the penalty worked and the new route avoids the bad edge.

### Bot_MoveNav

The main movement function. A maneuver that owns the frame — break, deploy, air move, ladder, elevator,
activation, charger — drives and returns; otherwise the walker runs (`NavFollow::Track`, avoidance, the drive).

**Repath triggers:** an empty path, a forced repath (an air move or ladder finished somewhere unplanned, an avoid
penalty landed), and a stall. An air move or ladder that finishes in the wrong area replans in the **same** frame:
the forced-repath latch is read at the top of the frame, above those blocks, so both re-check it through
`TryRepath` once their runner returns. Otherwise the bot would spend a frame on a plan it has provably left, with
`trackState.started` cleared and its reprojection free to snap anywhere on the stale profile.

**Tracking.** `NavFollow::Track` follows the `PlanProfile` built at repath time. Progress is arclength along the
arc chain, so there is no per-waypoint arrival radius, and the chain starts at the bot's own position. The drive
magnitude is the servo's `wishSpeed`, divided by `kDuckingMultiplier` while `FL_DUCKING` (the engine applies 0.333
after the maxspeed clamp), then clamped to maxspeed. Why not a constant full drive: it gives the bot no speed
control, so it takes every corner flat out and overruns the goal. Aim leads the *next* waypoint so the bot looks around corners; the
move direction comes from the steered direction, not the view.

**Stall detection** keys on `trackState.progressS` advancing, not on the waypoint index — a bot correctly braking
to a stop at a node would otherwise read as stalled. After `kRepathInterval` (2 s) without progress it repaths.
- Every maneuver that suspends `Track` must hold the clock open itself: `RunLadderMove`, `RunAirmoveFlight`, the
  elevator, activation, charger, break and deploy runners all stamp `lastProgressTime` every frame. Without that a
  long fall (up to `kAirmoveMaxAirTime`, 6 s) trips the timer mid-air. The cost is that a flight that never ends is
  invisible to every other timer, which is why the flight bounds itself.
- **Repeated stalls escalate.** `stallRepaths` counts stalled repaths since the bot was last more than
  `kStallEscapeDistance` (64) from `stallOrigin`; at `kStallRepathLimit` (3) the bot calls
  `BotGoal::OnBlocked(bot, "repeated stalls")` instead of planning again. A repath that succeeds resets
  `lastProgressTime`, so without this a plan the bot cannot follow loops forever (a bot stranded on a cs_assault
  vent grate repaths as stalled, indefinitely). The count is **displacement, not `progressS`**: every repath rebases
  arclength to zero, and a bot pinned against a wall slides 18 units along it, which reads as progress.
- **A bot that gives up from the same spot three times kills itself.** `stuckGiveUps` counts those
  give-ups while each lands within `kStallEscapeDistance` of the first and within `kStuckWindow` (15 s) of
  the previous; at `kStuckGiveUpLimit` (3), about 20 s pinned, the bot calls `ClientKill` (log: `is stuck
  at ... killing itself`) and still gives the goal up, so a refused suicide changes nothing. A new goal
  does not free a bot from a hole: on dod_avalanche a 30cal bot fell into a pit by flag #307 at
  (1018,-753,-394) and gave up 21 goals there over 150 s.
- **A frozen bot is never stalled.** `Bot_Frozen` is `0 < pev->maxspeed < 10`, how the engine pins a player the
  mod has stopped: CS freeze time and the 3 s arming / 5–10 s defusing all set maxspeed 1. While frozen,
  `lastProgressTime` is stamped every frame and the strategic hold limit (`kHoldLimit`) is not checked, so a CT
  defusing a bomb is not moved on. Buttons still go through `Bot_RunPlayerMove`; only the bookkeeping pauses.

**Teleports and respawns.** A jump of more than `kTeleportJumpThreshold` (150) in one frame resets tracking and
repaths. One that lands off `plannedAreaPath` is a CS/DoD round-start respawn, not a teleporter, so it sets the
same `spawnHoldUntil` a bot gets after death: a random 0–1 s idle before goal selection, which spreads a whole
team's `AreaCosts` floods and `PlanPath` calls over a second. A planned teleport lands on its own path and repaths
at once. The reset is skipped while riding an elevator, because a server hitch mid-ride can move the bot further
than a teleport and a repath from mid-shaft resolves to no area, so the bot drops its goal.

**Jump waypoints are commitments**: not passed until the jump is made, gated on on-ground, distance and facing,
with duck added while rising.

### Live physics: what the engine actually enforces

**`Bot_MaxSpeed`** returns the cap the engine will enforce on this bot, which is not any cvar.
`PM_CheckParamters` computes `min(clientmaxspeed, maxspeed)` and writes it to `pmove->maxspeed`; `PM_Move_Post`
reads the finished value and `Bot_NoteMoveMaxSpeed` caches it per edict index (cleared in `ClientDisconnect`).
`pev->maxspeed`, then `sv_maxspeed`, then `kMoveSpeed` are the cold-start fallbacks for the frames before a bot's
first `PM_Move`. `Bot_MoveMaxSpeedKnown` says whether the slot is filled, for callers that must not read a
fallback (`BotManager::CheckDriftSpeeds`, `doc/navgen.md`).

Why not `sv_maxspeed`: under CrossPlay each hosted game sets its own ground speed before handing over the move, so
two bots on one server legitimately have different caps; and even on one game the cvar is only half the `min` —
the test server never sets it, so the cvar reads 320 while the valve DLL holds players at 270. Every planner number
derived from `physics.maxSpeed` is then designed for a bot 50 u/s faster than the one flying it (`test54_window`
ping-pongs between two 320-speed routes until it times out). Getting the cap right is what makes the design speed
and the achievable speed the same number.

**`Bot_Physics`** reads `sv_airaccelerate`, `sv_accelerate`, `sv_friction`, `sv_stopspeed`, `sv_edgefriction` and
`sv_maxvelocity`, so plan-time physics match a non-default server. It does not read `sv_gravity` — the 800 is
`PmSim::kGravity` (`doc/games.md`) — but multiplies by `pev->gravity`, which `trigger_gravity` writes onto the
player and which decides which air moves exist (`doc/path_planning.md`, "Planning with gravity multipliers").
`sv_maxvelocity` matters only with push fields: `bounce`'s pods push at 3800 u/s², which hits the engine's clamp
within a second.

**`Bot_DrivePhysics`** is `Bot_Physics` with `groundFriction` doubled when the bot is at a drop. It runs
`PM_Friction`'s own probe — 16 ahead along the 3D velocity, 34 down from foot height, same hull, same
`fraction == 1` test (`AtFrictionEdge`, using `kEdgeProbeAhead`, `kEdgeProbeDrop`, `kStandHullMinsZ`,
`kDuckHullMinsZ`, which are the engine's numbers, not tuning knobs) — and feeds every ground `SolveDrive`: the
tracker, the air-move approach, and the ladder, elevator, activation and charger drives.
- `Bot_Physics` stays untouched, so `MakePlanTraces` and `BuildPathProfile` plan against open-ground friction. Do
  not merge them: design speeds that depended on whether the bot stood at a ledge when it replanned would move its
  own waypoints under it.
- Why it exists: an air-move approach always ends at the lip it launches from, where the engine runs double
  friction. A servo told the open-ground friction settles a fixed distance short of its target (see
  `NavFollow::SolveDrive`): without the probe test53_ladder4's crouch jump holds 66.4 against a 70.8 design, 94%,
  and aborts every run; with it the launch reads `speed 71 (100%)` and lands.

**`Bot_PushFields`** wraps the `trigger_push` entities of the frame's world snapshot (`Bot_WorldSnapshot`,
`doc/path_planning.md`, "The plan-time world") into a `PmSim::PmPushFn`, so `JumpLandsIn` simulates a launch
through a push field the way navgen sampled it. `Nav::PushAt` point-tests the origin against the field's clip hull
in the same hull as the sim, the test `pfnTraceModel` makes. A `SOLID_NOT` field (start-off or toggled off) is not
in the snapshot; the next repath picks up a change. The closure is cached on `CBot::pushFields` and refreshed per
repath.

### Following a drifting path

`NavFollow::Config::driftVelocity` is the world-frame shove the bot is under right now. `Bot_DriftVelocity` reads it
live: `groundentity->v.movedir * speed` when the ground entity has `FL_CONVEYOR`, plus the cached push closure at
the bot's origin — so a `Use`-reversed conveyor or a switched-on field needs no rebake.

`Track` uses it twice:
- the world-speed cap becomes `DriftSpeedAlong(drift, aimDir, maxSpeed)` instead of `physics.maxSpeed`, so a bot on
  a belt may exceed its own run speed;
- the wanted velocity handed to `SolveDrive` is `aimDir * target - drift`. `pev->velocity` is own-only and
  `SolveDrive` solves in the own frame, so its clamp to `physics.maxSpeed` stays correct untouched. Without the
  subtraction a bot crossing undertow's 420 u/s current is carried off its plan.

`BuildPathProfile`'s `NodeMaxSpeed` applies the same `DriftSpeedAlong` along each waypoint's outgoing segment (the
last uses its incoming one), from `PlanWaypoint::drift` (copied from `PlanArea::drift` in `PlanPath`). Corner radii,
both speed passes and `totalTime` follow from it, so `estimatedArrivalTime` stops being wrong by up to 2x on a belt.

### Barriers: sLimit and sFloor

The tracker resolves position by nearest point on the profile, and that is ambiguous wherever the path doubles back
on itself or has a zero-length hole. `NavFollow::Config` carries two arclengths from `NavPlan::ProfileWaypointS`,
computed in one pass over the waypoints each frame:
- **`sLimit`** — projection may not pass it. It sits at the first of: the next unserviced activation, a pending
  elevator ride, a pending ladder mount, or an air-move launch.
- **`sFloor`** — projection may not fall below it: the last serviced activation.

Why each barrier is needed:
- **Activation detour.** Outbound and return legs are collinear in XY, so `ProjectOntoProfile` resolves the tie to
  the lower `s`. Without `sLimit` the bot's `s` jumps to the return leg and it never stands on the activator (worse
  on first acquisition, which searches the whole profile); without `sFloor` it can never leave the outbound leg.
- **Elevator ride.** Its boarding and arrival waypoints share an XY exactly, so the ride is a zero-length segment the
  tracker cannot see and would walk straight through (aiming at the far-side corridor while standing at the near
  stop). A ride is pending while `requiresElevatorRide` is still set on the bot's own waypoint copy;
  `FinishElevatorRide` clears it. A flag rather than an index because `SpliceDetour` rebuilds `bot.waypoints` with
  a shifted base, and the flag travels with the waypoint.
- **Ladder mount.** Mount to exit is near-zero in XY, so without a barrier the tracker sails past the mount and
  `waypointIndex` reaches the exit while the bot is still short of the volume; the mount is never serviced, and when
  the bot drifts onto the ladder anyway the `ladderEject` rule sees no ladder waypoint in range and jumps it off at
  270 u/s, forever. The mount bars while `i >= bot.waypointIndex`, released by the `++bot.waypointIndex` that commits the
  climb.
- **Air-move launch.** The gap segment starts *at* the launch, so within a unit or two the nearest chain point flips
  across the hole and `wpIndex` becomes the landing. `RunAirmoveApproach` only engages when the *current* waypoint is
  a launch, so a launch skipped that way is never taken: the bot walks off into the gap (crossfire, between two
  ledges 64 units apart). Nothing else catches it — projection, `lateral` and `done` are all horizontal, so a
  180-unit drop reads as zero deviation until the stall timer.

**The ride runner is driven by the barrier index, not `bot.waypointIndex`**, because a ride waypoint may own no
profile segment at all: on `test18_elevator` the funnel corner before the board point shares its XY, so the segment
reaching the shaft carries the corner's index and the ride's never appears. `RunActivation` is driven the same way
by `activationWp`. The elevator runner is called ahead of `RunActivation`, since a board point can sit inside the
engage radius of a later button.

**A remembered gate only floors once the bot is there.** "Serviced" can come from the activation memo, not this
plan: a bot that rides a lift up and plans back through it within `kActivationMemory` finds the button detour still
in its path, 100+ units ahead. Flooring there pins projection ahead of the bot, `lateral` passes `rejoinAbort`, and
it repaths every frame into the same plan (0.7 units per repath for 15 s on crossfire). So the floor applies only
when the gate is behind the bot (`i < waypointIndex`; a segment carries the index of the waypoint it reaches) or the
bot is within `kActivationEngageRadius` (40) of it. Not `kActivationStandRadius` (8): a press can finish its hold 16
units short.

### Activation at runtime

Activation waypoints (buttons, triggers, doors) come from navgen's detours (`doc/navgen.md`).
- **Press.** `RunActivation` takes the frame inside `kActivationEngageRadius`, aims at `activatorCenter` in pitch
  and yaw, and emits a single `IN_USE` rising edge once the aim converges and the bot is within
  `kActivationUseRange` (56) of the centre in 3D. One edge, not a held button — repeated presses toggle a
  toggle-style button back off. It then holds for the door's `buttonWaitTime` before releasing the barrier.
- **Triggers** need only arrival, so they are serviced by proximity before `Track` runs.
- **Memo.** Serviced activators are remembered by centre for `kActivationMemory` (15 s). That stops a mid-detour
  repath from pressing twice (waypoint indices change, world state does not) and lets a failed press retry once the
  memo lapses. A failed or undecided elevator ride erases its entry (see Elevators).

### Air moves

The air move is three runners: `RunAirmoveApproach` / `RunAirmoveDrive` on the ground, `RunAirmoveFlight` in the
air, then a verdict on the connection.

#### Approach

The approach takes over once the tracker's `wpIndex` reaches a launch waypoint. It aims `kAirmoveApproachLead` (32)
past the launch along the launch heading and commits when the bot is within one frame of the launch plane **and up
to speed**. The lead is `NavPlan::AirmoveLaunchLead` — `speed * frametime`, one frame of travel — shared with the
planner so both agree on where the bot stops accelerating. Why no extra slack: a flat 4 units on top costs every
jump 4 units of run-up, most of the margin on `test37_walkent`'s 19.5-unit straight.

**A launch under `kAirmoveLaunchSpeedTolerance` (0.95) of its design speed is not taken.** `PmSim`, `JumpLandsIn`
and the sampler all simulate *at* the design speed, so a short launch is a jump the plan never modelled. Measured
without the gate: 222 planned against 146 delivered, and only launches at design speed land where aimed. At the plane still short,
the bot penalizes the connection, requests a repath and brakes (`RunAirmoveDrive` with drive 0 solves to full
reverse thrust). One aborted approach teaches the planner the edge is undeliverable, instead of one fall. Before
blaming the tolerance on a near miss, check the follower modelled the ledge friction (`Bot_DrivePhysics`): a servo
that holds steady a fixed distance short is a gain problem, not a margin problem.

Two drive rules, each of which stalls real routes when broken:
- **The command is a target velocity, not a wish speed.** `RunAirmoveDrive` puts `drive` through
  `NavFollow::SolveDrive` on the ground, exactly as `Track` does. A raw wish speed under
  `stopSpeed * friction * edgeFriction / accelerate` — 80 at a ledge — moves the bot nowhere: a dropoff planned at
  57 crawls at 2 u/s forever.
- **Ducking scales the command, so scale it back.** `PM_Duck` multiplies forward/side by `kDuckingMultiplier`
  *after* the maxspeed clamp, so delivering `W` while `FL_DUCKING` means commanding `W / kDuckingMultiplier`, and
  nothing above `maxspeed * kDuckingMultiplier` is reachable. Without it every crouched air move runs at a third of
  plan.

`RunAirmoveDrive` also drops `IN_JUMP` while the mover is on a ladder, so a flight that ends on one does not bounce
off it.

**A repath inside the run-up leg walks backwards.** `Repath` starts at the bot and `Cross` places the run-up node a
fixed distance back along the launch heading, so a bot that has already closed most of that distance gets a first
waypoint behind it and arrives at the node with no speed — the worst start for a launch that builds speed at a
ledge. Two waypoints at the same 2D point at the head of `crossbot_nav_status` are the tell.

#### Flight

**`RunAirmoveFlight` steers, re-deriving the wish direction from live velocity each frame.** `PM_AirAccelerate`
caps wish speed at 30, but the cap is on the *projection of velocity onto the commanded direction*, not on total
authority:

| Commanded direction | `currentspeed` | `addspeed` | Authority |
|---|---|---|---|
| Along the current heading, 270 u/s | 270 | −240 | **Nothing.** `addspeed <= 0` returns before touching velocity. |
| Perpendicular to velocity | ~0 | 30 | **~30 u/s**, reached in a few frames — ≈60 units of shift over a 2 s flight. |
| Opposite velocity | −270 | 300 | **~53 u/s per frame**; here `airAccelerate * wishspeed * frameTime` binds, not the cap. |
| Perpendicular, re-aimed each frame | ~0 | 30 | `sqrt(270² + 30²) − 270` = **+1.66 u/s per frame**, ≈100 u/s per second. |

Re-issuing the launch heading (row 1) is exactly zero correction. Perpendicular to the *launch* heading buys only
lateral motion; perpendicular to the *current* velocity, alternating sides, is the strafe gain — `(270, 0)` →
`(270, 30)` → `(273.3, 0.2)` with about half a unit of lateral excursion — so along-track undershoot is correctable
too.

**The target is the planned X-Y at time `t`, not the landing point.** In flight z is a function of time alone, so a
bot that speeds up to arrive early arrives high and sails over its ledge. Matching X-Y at the planned time matches
the whole 3D arc, which is also why only X-Y is compared — letting the reference stay in the planner's duck-origin z
while the bot flies in world z.

`AirmoveCorrection` builds `want = refVel + err / remaining` (feed-forward plus closing the gap by landing) and splits
`dv = want - velocity` about the current velocity:
1. `|dvPerp| > kAirmoveCorrectMinDelta` → **steer** along `dvPerp`;
2. `dvAlong < -kAirmoveCorrectMinDelta` → **brake** along `-vHat`;
3. `dvAlong > kAirmoveCorrectMinDelta` → **micro-strafe**, `±perp(vHat)`, the side alternating each frame via
   `airmoveStrafeSide`;
4. otherwise the launch heading at launch speed.

Order matters: a mixed error dominated by its forward part would normalize to a nearly-forward wish, which row 1 of
the table says does nothing, lateral part included. `drive` is sized to the need, `need / (airAccelerate *
frameTime)` clamped to maxspeed, because braking is one-way — air acceleration never gives speed back along the
direction of travel — and full drive would shed 53 u/s in a frame. On plan, `err` is zero and branch 4 fires, so the
planner's open-loop arc is still what a zero-error flight flies.

**The reference arc is simulated at launch and checked before use.** `Bot_SimulateAirmoveArc` runs a `PmSimulator`
from the bot's real launch state with the *planned* yaw and speed, sampling every `kPlanSimFrameTime`;
`Bot_AirmoveArcAt` interpolates it by air time.
- **Seed from the bot's actual origin, not `wp.launchSimFrom`.** That records where the *first* simulation started,
  and `Respeed`/`Retract` move the launch without rewriting it (a stale seed turns an 18.9-unit miss into 77.9 in
  the wrong area).
- **Do not stop stepping until the mover has been airborne.** A drop is still `OnGround` for its first frames, so
  breaking on the first grounded frame ends the arc after 0.02 s.
- **Validate the endpoint against `wp.launchSimTo`** within `kAirmoveArcAgreeDist` (16) and drop the arc on
  disagreement; `launchSimTo` is an output, so it is always current. A dropped arc costs nothing: the flight is
  open-loop.

Mid-flight, an error past `kAirmoveCorrectAbandon` (64) clears the arc for the rest of the flight: the bot was
knocked or hit something the sim did not, and steering hard at a stale target is worse than coasting (uncapped, a
bounce flight corrects toward a 264-unit error for 96 frames and lands 675 units out).

#### Ending a flight

**The flight ends when the bot settles, and three states count** (`AirmoveSettled`): `FL_ONGROUND`, swimming
(`PM_CategorizePosition` forces `onground = -1` at `waterlevel >= 2`) and a ladder (`MOVETYPE_FLY`).
`AirmoveEndReason` names which in the log: `landed`, `splashed down`, `caught a ladder`. Testing only the ground flag
leaves a bot hanging in water or on a ladder until the 6 s cap, with the stall timer held off.

**The timeout is the connection's own flight time.** `AirmoveTimeLimit` latches
`conn.flightTime * kAirmoveTimeFactor + kAirmoveTimeMargin` (2x + 1 s) at launch, capped at `kAirmoveMaxAirTime`
(6 s), which also applies when the connection has no flight time. It is suspended while the bot is still falling
(`velocity.z < kAirmoveFallingZ`, −60), because a fall deeper than planned is exactly the case worth waiting for;
it catches the bot that is neither falling nor settled. `airmoveOverrunLogged` prints one line the first time a
flight passes its planned time by `kAirmoveOverrunGrace` (0.5 s), with `onground`, `waterlevel`, `movetype`,
`FL_DUCKING`, velocity and resolved area — the exit line alone cannot say which state a hung flight was stuck in.

#### Scoring the landing

`RunAirmoveFlight` calls `Nav::PenalizeConnection` on the index latched at launch (`airmoveConnIndex`) when the
landing failed, `RewardConnection` when it succeeded (`doc/path_planning.md`, "Runtime connection penalties"), and
forces a repath on failure. Without the penalty a maneuver the follower cannot execute loops forever: each repath
starts from a new position that may flip which route is cheapest, and the bot alternates between two routes into
one dead end. test50_stairs shows it is not a planner bug — jumping a staircase is a coin flip, but the follower
launches identically every time, so an edge a human clears half the time fails every time; with the penalty the bot
gives up both jumps and walks up in ~2 s instead of timing out. An oscillating `actual=` list means the penalty has
not caught up yet (one more failure per bad edge), or every remaining route is bad.

**The verdict is distance to the plan's landing waypoint; the area is only a fallback.** `failed` is
`miss > kAirmoveLandingTolerance` (48), `miss` being 3D against `waypoints[waypointIndex]` — after the launch's
`++waypointIndex` that is the landing waypoint. The area comparison runs only when there is no landing waypoint.
Areas are too coarse (crossfire's area 66 is 36 × 388): an area verdict passes landings hundreds of units off and
fails perfect landings at a seam (`miss 0.3` scored as failure). Good landings measure 0.2–8.2 across the `maps/`
sweep and 0.2–14.0 on crossfire, bad ones 93+; 48 is three times the worst good one.
- **Measure against the plan's waypoint, never `conn.landingPos`.** The connection's landing is the sampler's;
  the planner re-derives the arc per route, and the two differ by 65–134 units on landings that are on time and in
  the right area.
- **Waypoint z is origin space, not `BotNavZ` space.** Subtracting `kPlanStandHeight` puts an 18-unit floor under
  every miss.

**A verdict is scored only when the bot settled inside an area**, looked up with the strict `Nav::FindAreaExact`.
The forgiving lookup never fails, so with it a bot in the void or on a lip outside the mesh is scored, possibly as
the area it aimed at. Settled-in-no-area is `undecided`, like the timeout: neither penalized nor rewarded, but repathed,
since the plan is void either way.

### Ladders

`RunLadderApproach` walks the bot to the mount and latches `CBot::ladderActive`; `RunLadderMove` owns every frame
until `FinishLadder`. The rules below are what separate a climb from a jump-on/jump-off loop.

**Drive from the plane the engine picked, not the face navgen planned.** `PM_LadderMove` chooses its plane with
`PM_TraceModel(origin -> ladder centre)` in the point hull, so the face depends on where the bot stands. If it
disagrees with the planned face, `dot(velocity, N)` is ~0 and the bot flies horizontally off at 200 u/s.
`ProbeLadderPlane` repeats the engine's trace every frame and every aim derives from it; `LadderPlane::why` records
which of `no-entity` / `startsolid` / `allsolid` / `no-hit` / `degenerate` rejected a probe. With a valid plane every
motion is `IN_FORWARD` plus view angles — climb faces `-N` at pitch 0, descent `-N` at pitch +89, sideways `±perp`.
A degenerate plane gives velocity exactly `vpn * 200`, so the fallback aims pitch ∓89 and rides that.

**Never duck on a ladder while `FL_ONGROUND`.** `PM_CategorizePosition` treats the player as airborne only above
`velocity[2] > 180` (`pm_shared.c:1542`); below that it snaps the origin to floor found two units down. A ducked
climb is `MAX_CLIMB_SPEED * 0.333 = 67`, so at the foot of a ladder the bot is re-glued to the floor every frame.
`RunLadderDrive` drops `IN_DUCK` while on a ladder and on the ground; the bot detaches at 200 in one frame and
re-ducks in the air. `PM_UnDuck` refuses to unduck into a ceiling, so a genuinely low mount degrades safely.

**Never hold `IN_JUMP` while `MOVETYPE_FLY`.** `PM_LadderMove` answers the jump button with
`movetype = MOVETYPE_WALK; velocity = normal * 270`. So the reach-up hop is gated on `!onLadder`, not just
`FL_ONGROUND` (at a ladder that reaches the floor the bot is on both at once, and would hop on and off forever),
`avoid.jump` is suppressed the same way, and `RunAirmoveDrive` drops the button on a ladder.

**A ladder that stops short of the floor costs exactly one frame of jump.** `LadderOutOfReach` detects a ladder the
standing origin cannot touch (test53_ladder4's `*2` brush ends 62 units up: hull 1 puts it at `z >= -212`, the bot's
origin is at −220), waits `kLadderReachDwell` (0.3 s) and hops. The hop is a 50 ms command and the rest of the frame
is jump-free (see `Bot_RunPlayerMove`), so the contact sub-step finds `IN_JUMP` clear and the bot grabs at the top
of its arc instead of being thrown back down.

**Aim before driving.** `PM_LadderMove` builds its wish from the *buttons*, so `IN_MOVERIGHT` is a full
`±MAX_CLIMB_SPEED` input however small the `smove`; a bot still turning commands a large wrong-axis vector, and the
ladder turns that into wrong motion (a descent still facing its run-up yaw climbs out of the top and walks off the
ledge). Every on-ladder phase zeroes `drive` while the yaw error exceeds `kLadderAimTolerance` (6°); with no buttons
`PM_LadderMove` clears velocity, so the bot hangs and turns.

**Latch the dismount phase.** `ladderAtTop` is set once and never cleared before the next commit. The height test
that switches from climb yaw to dismount yaw sits at one z; unlatched, a bot hovering on it alternates every frame
while the aim gate holds `drive` at 0, and a 2 s climb takes 12.7 s.

**A climb approach aims `kLadderMountLead` (32) past the mount, into the ladder.** `pointA` is deliberately outside
the carved volume, so steering *to* it stops the bot short of the capture region — a permanent stall for a ladder
whose volume starts two units beyond the stand point.

**A descent approach aims past the mount and never turns back.** `PM_Ladder` is a point test of the origin against
the brush's hull-3 volume, and where a ladder ends flush with its ledge the volume tops out at exactly the standing
duck-origin z, so nothing catches until the bot walks off and starts to fall. The descent aim carries it over the
edge, `kAirmoveApproachLead` beyond the mount along `ladderEntryYaw`. The branch is chosen by *signed* progress along
`ladderEntryYaw`: `dot(toMount, entryDir) > kLadderMountApproach` (8) still steers at a mount genuinely ahead; level
or behind drives on `entryDir` alone. Why not derive the line from `origin -> mount`: it inverts the instant the bot
passes the mount, and the bot hovers on an 8-unit shell around it until it gives up.

**Stalls are measured in distance, not speed.** A bot wedged against the brush can hold `vel=(0,0,67)` forever, so
the detector compares against `ladderStallOrigin` and gives up after `kLadderStallDist` (8) in `kLadderStallTime`.
Two push-off escapes are allowed; the third gives up on the connection and repaths. `kLadderMaxTime` (25 s) bounds
the whole climb.

**Look where the climb is going, and count only entities.** `LadderBlocked` casts one hull trace a frame, at most
`kLadderProbeAhead` (48) long, along the current phase's heading: up to `ladderDismountZ` on a climb, down to it on a
descent, at `ladderExitPoint` once `ladderAtTop` latched, and at the mount on the approach. If the same entity stays
in the way for `kLadderBlockDwell` (0.4 s) the climb ends `blocked on` with `kLadderFailed`, penalizing
`ladderConnIndex` and repathing; on the approach nothing is latched yet, so the mount's `wp.connIndex` takes the
penalty.
- What counts is `Bot_TraceHitEntity` (world, world brushes and `IsBakedBlocker` classnames excluded) minus live
  players. The world must be out because a shaft is a hull-wide tube of world brush; geometry that truly blocks a
  ladder is navgen's mistake and shows as `never caught` or `stalled on`. Players must be out because a teammate at
  the top is gone a second later and the penalty is mesh-global.
- The approach probe only runs inside `kLadderCommitRadius` (48): `BotAvoid` is not running during a ladder approach,
  and a probe at 200 units would penalize a can the bot could have stepped around.
- Without the probe the stall detector finds a blocked shaft late and wrongly: with a `cycler` on top the climb's
  head meets it, two push-offs drop the bot, and the landing scores as a completed `dismounted off`.
- test52_avoid exercises it: a pit with two ladders and a `cycler` on one; the run reads `ladder path blocked by
  cycler`, `blocked on ... conn 16 failed`, `Replan`, `ladder climb ... conn 17`, `dismounted off ... ok`. With no
  second way up the penalty only climbs by `kConnFailPenalty` every 0.4 s re-commit — a price, never a ban, and
  nothing above the ladder gives up on the goal.

**The verdict judges the maneuver first, then the area.** `FinishLadder`'s `LadderOutcome`:
- `kLadderOk` — `reached the bottom of`, `dismounted off`;
- `kLadderFailed` — `never caught`, `fell off`, `timed out on`, `stalled on`, `blocked on`;
- `kLadderAbandoned` — the bot chose to leave (`bailed off`, `could not bail off`); scored like undecided.

A failed maneuver is penalized whatever the geometry says. Only a completed one looks at the area, with the strict
`Nav::FindAreaExact`: the target area rewards, a different area penalizes, no area is `undecided` (a bot still in the
shaft is not evidence). The log carries `ok`, `failed`, `undecided` or `abandoned`. Why not `FindAreaNearest`: it
never returns null, and where the shaft is carved out of the mesh the nearest centroid is often the area the bot just
left, so test53_ladder4's working `2 <-> 4` descent is penalized every time. Why not defer an undecided verdict until
the bot is strictly inside something: by then it has walked on and the deferred answer is wrong too (a working ladder
gets penalized two areas later). Nothing is lost by declining: `RewardEnteredConnection` still decays the penalty when
the bot walks into the planned next area.

**Being shot ends a climb, unless the fall would kill.** `BotCombat::Run` waits for `Bot_ManeuverActive`, so a bot
being shot on a ladder is a free kill. `Bot_MoveNav` therefore calls `Bot_BailManeuver` right after
`BotCombat::Update` whenever `BotDiff::CombatInterrupt()` would let `Run` drive and `BotCombat::DamageSource` names an
attacker this frame (world damage, falls and the bot's own rockets resolve to nothing, so never cause a bail). The
bail estimates the drop the eject implies — down to the mount on a climb, to `ladderDismountZ` on a descent — prices
it with `PmSim::FallDamageFor` under the game's rule (`Bot_FallDamage`), and refuses only when the damage would be at
least the bot's health.
- It does not clear `ladderActive`. It sets `ladderBail`, and `RunLadderMove` drives the `kLadderPushOff` aim until
  `OnLadder` goes false (one frame of `IN_JUMP`, measured `vel=(270,0,-8)`), then finishes `bailed off` with
  `kLadderAbandoned`. Clearing the state outright would hand combat a bot still in `MOVETYPE_FLY`, and the
  `ladderEject` rule lives only on the walker path combat bypasses.
- If the bot is still on the ladder `kLadderStallTime` later it finishes `could not bail off` and lets combat drive
  from where it hangs: a hull overlapping a solid is frozen by `PM_CheckStuck` before `PM_LadderMove` reads the jump.
- `kLadderAbandoned` neither rewards nor penalizes: the ladder was fine, and pricing it would feed combat noise into
  every bot's routing.

Two failures that alternate between routes — e.g. a crouch jump aborting under speed and a ladder never catching on
the same ledge — mean both edges are broken and penalties are working, not that the planner is unstable.

#### Ladder debugging

`bot_move_nav.cpp` has a per-frame ladder trace behind `CROSSBOT_LADDER_DEBUG`, off by default:

```
build.bat -DCROSSBOT_LADDER_DEBUG=ON
```

The value is cached in `build/CMakeCache.txt`, so plain `build.bat` keeps whatever was last set. Output goes to the
server console (captured at `nav_debug/logs/<map>_server.log`) only while the bot is in the ladder pipeline, and the
macro compiles away when off. Lines read `CrossBot: LDBG <bot> t=<time> <phase> key=val ...`:

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

`drive`/`after` and `eject`/`walk` between them cover every button the bot can be sent, so "who pressed jump" is
answerable from the log.

### Elevators

`RunElevatorApproach` / `RunElevatorRide` / `FinishElevatorRide` mirror the ladder trio. The hard part is that
**the ride is invisible to the profile**: the boarding and arrival waypoints are the mover's column centre at two
heights (`elevators.cpp` builds both from `centerX`/`centerY`), so `BuildPathProfile` measures zero `Dist2D` and
never pushes the segment, and `ProjectOntoProfile` skips zero-length segments outright.

Consequences, each handled separately:
- **The tracker would walk through the ride** — a pending ride sets `sLimit` (see Barriers).
- **The runner is dispatched by the barrier index**, because the ride waypoint may own no segment (see Barriers).
- **Reprojection after the ride cannot be global.** Both ends share an XY, so a global reprojection is free to pick
  the lower `s` and put the bot back on the approach. `FinishElevatorRide` sets `trackState.progressS` to the
  arrival waypoint's `ProfileWaypointS` by hand, leaves `started` true, and floors progress there: the two segments
  meet at one point, and walking away from a lift entered and left on the same side is equidistant from both, which
  the earlier one would win. The incremental window then reaches at most `projectBack` behind, where the profile is
  32 units away.
- **`totalTime` under-reports by the whole ride**, so `BuildPathProfile` adds `elevatorWaitTime +
  elevatorTravelTime` after the loop. Without it `test55_plat`'s ETA is 3.8 s against a 4.96 s ride, and
  `test_nav_goal.ps1`, which times out at twice the ETA, cannot pass.
- **No arclength accrues while the platform works**, so `lastProgressTime` is stamped every frame; the 2 s stall
  timer would fire twice over on a 5 s ride.

The runner itself is deliberately dull:
- It engages within `kElevatorEngageRadius` (48, pure XY to the board point), latches on `FL_ONGROUND` within
  `kElevatorBoardRadius` (12), holds station with a `kElevatorHoldRadius` (6) deadzone, never jumps, and aims at the
  waypoint *after* the arrival one so the bot is already turned to walk off — `func_plat`'s `PLAT_DELAY` gives it
  three seconds.
- `RunElevatorDrive` skips `SolveDrive` when drive is zero: zero means stand still, and the servo answers a residual
  velocity with a full-rate reverse command — right on open ground, wrong on a platform edge — so friction does the
  stopping.
- **Arrival** is `BotNavZ` against the arrival waypoint's floor z, directional, with `kElevatorArriveSlack` (8). The
  slack lets the bot step off up to 8 short, which is safe because navgen only builds a walk seam off a stop when the
  step is under 18. `BotNavZ` is the floor under a waypoint's point: the point less the stand lift unless the bot is
  ducked, read off the live stance.
- **Deadline** `rideDeadline` = `wait + travel * kElevatorTimeMargin + kElevatorTimeGrace` (2x + 3 s). The wait term
  matters because a ride back to the default stop does not start until the platform's auto-return dwell elapses.
  Timing out is a failure like any other — penalize, repath — and that also recovers a bot whose platform someone
  else rode away.
- The engage radius could hand over the frame early on a route that merely *passes* within 48 of a shaft. Ordering
  protects it today: the barrier sits at whichever comes first, so with a call button still unserviced `rideWp` is
  unset (on `test18_elevator` the air move lands 40 units from the board point with the button still to press). If a
  map defeats that, the ride times out and repaths; tying engagement to `progressS` reaching `sLimit` would be the
  tighter fix.
- `bot_avoid.cpp` needs nothing: `RouteExemptsHit` exempts every `SOLID_BSP` hit while a ride waypoint is in the
  lookahead, and the runner returns before `BotAvoid::Update` while riding.

**The verdict uses the strict lookup on a constructed point.** `FinishElevatorRide` grades its connection
(`rideConnIndex`) like the ladder and air move: target area rewards, a different area penalizes, no area is
`undecided` (repathed, not scored). `FindAreaNearest` is wrong here in particular — an elevator column is where areas
overlap in AABB and a bot on the mover's brush is often outside every area's planes, so failed rides get rewarded.
But `FindAreaExact` on raw `pEdict->v.origin` makes every *correct* arrival undecided, for two reasons that hide each
other:
- **The origin is in stand space, the mesh in duck space.** A stop is a carved slab two units tall (`test18_elevator`
  area 7 is `16..18`; crossfire's top stops 11, 154 and 188 are `-1520..-1518`), and a standing origin sits 18 above
  it, past `kAabbEpsilon`.
- **The ride ends up to `kElevatorArriveSlack` short** (nav z 8.6 against a stop at 16 on `test18_elevator`).

So the probe is the bot's XY at `rideArriveZ` when the residual is inside the slack — the slack that may *end* the
ride early is the slack that may *judge* it at the stop — and at `BotNavZ` otherwise. A ride stalled mid-shaft is
hundreds of units off, gets no snap, resolves to the shaft and is penalized.

**The target area wins a boundary tie.** Stop slabs abut the shaft exactly (`test18_elevator` area 6 ends at z 16,
area 7 begins there), so the probe is inside both and `FindAreaExact` returns the shaft by id order.
`FinishElevatorRide` asks `Nav::AreaContainsPoint(*target, probe)` first and sweeps only if that fails — which is
also the honest question, "did I arrive at the stop I wanted". A wrong floor fails the test on z and is penalized by
the sweep. `RunAirmoveFlight` and `FinishLadder` probe the raw origin; neither has a reproducible failure
(crossfire's thin stops carry no air moves, ladder tops are volumetric), and their `undecided` lines on
`test25_ladder2`/`test53_ladder4` are genuine.

**A failed ride forgets the press that summoned it.** `RunActivation` only knows it sent one `IN_USE` edge, not that
the lift moved, and crossfire's lifts drop presses that land mid-cycle (a `func_door` ignores Use unless parked, the
`wait 3` button until it returns). Kept, the memo makes every repath skip the button and board a parked lift, time
out and repeat. On a failed or undecided ride `FinishElevatorRide` erases the memo entry for the gate right before the
ride waypoint, scanning back no further than the previous barrier, so the repath presses again.

**What the test maps cover.** `test55_plat` is the only sustained ride: the bot boards the parked plat in the pit
(at nav −199, not the parked −230, because stepping on starts it) and rides 248 units, `ride reached the stop of after
4.18s` against a 4.96 s plan. `test18_elevator` covers summon-then-board — drop into the shaft, press, board — but not
the ride loop: the door finishes its 2.38 s travel inside `RunActivation`'s 2.4 s button wait, so the ride takes
0.00 s. A map with a call button *off* the platform would exercise waiting for a lift to arrive.

### Chargers

`RunChargerUse` is `RunActivation` with two differences:
- **`IN_USE` is held every frame**, because `CRecharge::Use` and `CWallHealth::Use` are `FCAP_CONTINUOUS_USE` and
  transfer one point per call.
- **The exit is measured, not timed.** The runner watches `ChargerValue` and leaves `kChargerNoGainGrace` (0.5 s)
  after it stops rising. What that means depends on `pev->frame`, the charger's empty flag (`Use` sets 1 when
  `m_iJuice` hits zero, `Recharge` restores 0): frame 1 means drained, and the bot leaves with no memo entry — the
  live `frame` test in selection already filters it; frame 0 with no gain means the bot is full, and the classname
  goes in `fullMemo`.

The charger's capacity is not the bot's ceiling: at `sk_suitcharger` 30 on `test56_recharger`, a bot at 0 armor
needs four visits — `0 -> 30 -> 60 -> 90 -> topped up at 100` — so a drained-before-full visit is not a failure.

It is dispatched inside the `follow.done` branch, not the waypoint loop, because a charger is not a waypoint: the
plan's goal is the use spot and the charger is off the mesh. `lastProgressTime` is stamped every frame. Selection and
the heal volume are `doc/goals.md`.

**Being shot at a charger ends the visit.** `RunChargerUse` writes `desiredAngles` at the fixture every frame, so the
bot cannot turn to answer. `Bot_BailManeuver` calls `FinishCharger` (clears `chargerActive`, forgets the goal item)
and `BotCombat::Run` drives next frame. No `NoteIgnored`: nothing is wrong with the charger, and `BotCombat::Exit`
zeroes `nextSelectTime`, so the bot walks back after the fight. `RunHealVolume` is not bailed out of: it leaves the
view to `BotSense::ApplyLook` and `BotFire::Think` runs before movement, so the bot already shoots back while it heals.

### Diagnostics and wiring

- **`WriteTraceLog`** writes the bot's trajectory to `crossbot_trace_path` (empty by default, which records nothing).
  Verbose builds only: the `bot.traceLog` append is inside `#if BOT_VLOG_ON`, so a `-DCROSSBOT_VERBOSE=OFF` build
  writes nothing. Every bot writes the same path, so the file means something only with one bot;
  `test_nav_goal.ps1` sets the path per instance and passes `-RequireVerbose`.
- **Airmove cache.** `MakePlanTraces` attaches the process-wide `g_airmoveCache` to every `PlanTraceContext` and reads
  `crossbot_airmove_verify` into `SetVerify` on every plan; `Bot_ResetAirmoveCache()` clears it from
  `BotManager::ServerActivate` and `ServerDeactivate`, alongside `BotSense::ResetBoneCache()` — the plugin's per-map
  clear site, which anything holding map-scoped state must hook. `Nav::TraceWorld` fills `PmTrace::hitEntity` from
  `WorldSolid::counts` of the solid that set the result: not worldspawn, not a `SOLID_BSP` with `FL_WORLDBRUSH`, and not
  `crossbot_nav::IsBakedBlocker`. Key, eviction and sizing: `doc/path_planning.md`, "Airmove cache".
- `kAimVelocitySpeed` (20): below it velocity carries no usable heading, so the aim holds rather than snapping on
  numerical noise.

## The follower (`path_follower.cpp`)

### NavFollow::Track

Follows a `PlanProfile`. The geometry is precomputed, so the tracker only has to stay on it — no chase point, corner
cut, corridor pull or arrival radii.

- **Projection** is monotone: each frame searches only `[s - projectBack, s + speed*dt*4 + projectAhead]`, so the
  bot cannot snap onto a distant part of the route that passes nearby; `sLimit`/`sFloor` clamp it further.
- **Aim direction** is the tangent one frame ahead plus a cross-track correction toward the chain scaled by
  `min(1, offset/rejoinDist)`. Deliberately not pure pursuit: aiming at a lookahead point starts the turn as soon as
  that point rounds a corner, cutting it by up to the lookahead; the arc already encodes the turn.
- **Speed** is the profile speed at the current arclength, derated by `1 - (lateral - C)/C` (floored at
  `derateFloor`) when outside the clearance budget `C`, so the bot slows to rejoin instead of arcing wider.
- **Velocity servo**: `SolveDrive`, below.
- **Abort**: `needRepath` once lateral exceeds `rejoinAbort` (96).
- **`done` means arrived, and only that.** The cross-track correction can cancel the tangent exactly — a bot barred
  by `sLimit` standing beyond the node sees the chain point directly behind it — and falling through to `done` there
  announces "reached goal" wherever the bot stands (312 units from the goal, mid-shaft, on `test18_elevator`). A
  degenerate blend keeps the raw tangent and lets the lateral test decide on a repath; the remaining zero-direction
  case, which needs an empty tangent, asks for a repath.

### NavFollow::SolveDrive

Inverts one engine frame to hit a target velocity `V*`. It is the only correct way to ask the engine for a speed, and
everything that drives the bot on the ground goes through it.

```
v_af      = velocity after this frame's friction   (guard |v| < 0.1, as PM_Friction does)
need      = V* - v_af
wishdir   = normalize(need)
wishspeed = max(|need| / (sv_accelerate * dt * friction), along + |need|)
```

One formula covers acceleration, braking, counter-strafing and arc tracking, saturating when a frame cannot deliver.
- **The second term (`byCap`)** matters because `PM_Accelerate` caps its delta at `wishspeed - v.wishdir`; the
  rate-based form alone commands nothing when already cruising at the target.
- **The first term (`byRate`)** is what makes low speeds reachable. Below `sv_stopspeed` the friction drop is the
  constant `stopspeed * f * dt` however slowly the bot moves, while `PM_Accelerate` adds `sv_accelerate * wishspeed
  * dt`, so a raw wish speed does nothing at all until `wishspeed > stopSpeed * groundFriction * edgeFriction /
  accelerate` — 40 on open ground, **80 at a ledge**. `byRate` commands more the further velocity is from `V*`, so the
  bot closes on it from any start.

**`byRate` turns an unmodelled friction into a standing speed error.** It is a deadbeat solve whose gain is set by the
`f` the caller passes; given less friction than the engine applies, it settles at

```
v = V* - stopSpeed * (f_real - f_model) * dt
```

— for an unmodelled ledge doubling, a flat `V* - 400 * dt` at defaults (4 units at 100 fps, 6.7 at 60), whatever
`V*` is. Measured on test53_ladder4's crouch jump: 66.8 / 66.4 / 66.2 against 70.8 on 10.0 / 11.0 / 11.6 ms frames.
The error exists only below the 80 threshold; above it `byCap` wins (`W = V*`) and the target is self-sustaining. So
it only shows with a design speed under 80, which in practice means a crouched approach (capped near 90).

So `SolveDrive` stays a pure function of the `PmPhysics` it is handed, and the caller hands it `Bot_DrivePhysics`,
whose `groundFriction` is already doubled at a ledge. `BuildPathProfile` deliberately stays on open-ground friction
(see `Bot_DrivePhysics`).

## Opportunist hops (`bot_opportunist.cpp`)

The mesh is generated at one design speed, and every connection's `costSeconds`, `flightTime` and `exitSpeed` is
baked at it. A bot *faster* than that — `sv_maxspeed` raised, or holding `item_longjump` (a jump becomes
`forward * 560` and `sqrt(2*800*56)` up) — has movement options no plan on that mesh contains. `BotOppo::TryJump`
checks four times a second whether taking such a hop now beats walking the plan, and if so hops off-plan and repaths
from the landing. The plan is not changed; the bot occasionally short-circuits it.

**The trigger needs the mesh's design speed**, `NavHeader::designMaxSpeed`: the speed navgen built with
(`--max-speed`), stored in the file since version 16. `kNavRunSpeed` is the wrong substitute — it is 250, so a stock
320 bot would already clear the threshold on every map. The loader falls back to `kNavDefaultDesignSpeed` if the
field is unset.

**One reverse search, not one per candidate.** The comparison needs the cost-to-goal of two positions every tick.
`Nav::AreaCostsToGoal` prices every area to the goal in one Dijkstra (`doc/path_planning.md`), so both lookups are
array indexes and a tick costs one hull-traced `PmSim` run. `BotOppo::RebuildCosts` caches the field per bot and
rebuilds it when the goal area changes **or on any successful `Repath`** — `runtimePenalty` changes edge costs
between repaths, and a field built before a penalty prices a route the planner has abandoned. Rebuilding is cheap;
it is the same walker-free search goal selection runs.

**Plan-seconds against real seconds.** `AreaCostsToGoal` values are centroid-anchored and priced at `kNavRunSpeed`;
`PmSimulator::Time()` is real seconds. Subtracting directly always errs the same way — both cost terms are inflated
relative to what the bot achieves and the flight is not — so the bot would jump for gains that do not exist. The
flight is converted to plan-seconds:

```
gain = C[here] - C[landing] - flightTime * (maxSpeed / kNavRunSpeed)
```

and must clear `kMinGainSeconds` (0.35), which absorbs the centroid noise in both terms.
- **The ratio uses `maxSpeed`, not current speed.** A long jump overwrites horizontal velocity with a flat 560, so a
  launch at 68 u/s and one at 270 fly the same 0.8 s; scaling by instantaneous speed would make a jump look nearly
  free exactly when the bot is too slow to be jumping. The ratio is a property of the map's physics, not of the frame.
- **No sub-area `|origin - centroid|` term.** Both ends price to the same goal, so centroid offsets largely cancel;
  the item-ranking formula adds one because it ranks *targets*, and copying it here would charge the bot for walking
  back to its own centroid. A landing in the start area is rejected outright: the costs are identical by
  construction and the flight is pure loss.

**Simulating a long jump without teaching PmSim about it.** `BeginAirborne` takes an explicit launch velocity, so the
caller builds the one `PM_Jump` would: `kLongJumpSpeed` (560) horizontally and `kLongJumpImpulse` up. That impulse is
`sqrt(2*800*56)` with the 800 **hardcoded in `pm_shared.c`**, not `sv_gravity`: altered gravity changes the arc's fall
but not its launch. The arc uses the duck hull throughout, as `JumpLandsIn` assumes and the engine produces (see
`Bot_RunPlayerMove`). The loop tracks an `airborne` flag, because after `Begin(..., jump)` the bot is still
`OnGround()` (the impulse lands in the first `Step`), so `while (!OnGround())` would never run; a sim that never gets
airborne is rejected, not treated as an instant landing.

**The long jump leaves along the view, not along velocity.** An ordinary `PM_Jump` keeps horizontal velocity, so
simulating from `atan2(vel.y, vel.x)` is exact. The module branch *overwrites* it:

```c
for (i = 0; i < 2; i++)
    pmove->velocity[i] = pmove->forward[i] * PLAYER_LONGJUMP_SPEED * 1.6;
```

`pmove->forward` is `AngleVectors(cmd.viewangles + punchangle)`, so the launch goes where the bot *looks*, and
velocity only feeds the `Length(velocity) > 50` gate (the `pm_shared.c` comment above that branch reads like the
opposite). Navigation aims at `waypointIndex + 1` and `ApplyLook` can drag the view onto an enemy, so view and
velocity yaw routinely differ by tens of degrees. `forward` is not flattened either: pitch scales the horizontal
launch while `velocity[2]` stays flat — a bot looking 30° down long-jumps at 485, not 560.

**The launch frame is one `Bot_UpdateFacing` step from the angles the check reads.** `TryJump` runs before the
follower's aim code and `RunAirmoveDrive` issues the command right after, setting `desiredAngles` to the launch yaw;
residual `aim.yawVel` and drift still move the view in that step, and pitch is dragged toward 0. Both are refused
rather than modelled: `kMaxLaunchYawRate` (30) skips the tick while the aim is slewing (retry in 250 ms), and
`kMaxLaunchPitch` (20) keeps the `cos(pitch)` term within a couple of percent. The oppo log prints launch yaw and
velocity yaw side by side, so a miss reads as "wrong prediction" or "aim moved".

**Letting the command through, and after.** The launch frame passes `allowLongJump` so `Bot_RunPlayerMove` does not
strip `IN_DUCK`. The hop reuses the air-move flight with `airmoveConnIndex = -1` and `airmoveOpportunistic` set,
which at landing suppresses `AirmoveLandingMiss` (the plan's next waypoint is meaningless for a deliberate off-plan
jump), forces a repath unconditionally, and starts a cooldown if it landed outside the predicted area — otherwise a
bot whose sim disagrees with the engine retries from the same spot at 4 Hz forever. The reward/penalty branch is
skipped because it keys on `connIndex >= 0`.

## Obstacle avoidance (`bot_avoid.cpp`)

Reactive handling between the follower's steering decision and the engine call. The mesh knows world geometry and the
brush entities navgen carves; anything else solid — a `cycler`, a monster, another player, a door swung out of its
doorway — is invisible to the planner, which routes through it. `BotAvoid::Update` traces forward along the planned
path each frame and answers with one of four moves: **jump onto** the obstacle, **duck under** it, **go around** it,
or — when none works and it can be destroyed — **break** it (`BotAvoid` decides; `bot_break.cpp` does it).
`crossbot_nav_avoid 0` disables the layer, for A/B testing without a rebuild.

### Which hits are avoided

A hit is skipped when it is worldspawn; when `solid` is none of `SOLID_BSP`/`SOLID_BBOX`/`SOLID_SLIDEBOX`; when it is
a `SOLID_BSP` with `FL_WORLDBRUSH`; or when `RouteExemptsHit` matches it. Everything else is avoided, players
included — bot-vs-bot is not special-cased, it simply never qualifies for jump-on.

- **`FL_WORLDBRUSH`, not `MOVETYPE_NONE`, marks static brushes.** Every solid HL brush entity is `MOVETYPE_PUSH`,
  `func_wall` included; the `MOVETYPE_NONE` ones (`func_illusionary`, `CMultiSource`) are `SOLID_NOT` and never hit.
  `CFuncWall::Spawn` sets `FL_WORLDBRUSH` ("not moveable/removeable brush entity"), `func_conveyor` and
  `func_wall_toggle` inherit it, unbreakable glass is the only other setter, and the HLSDK node graph uses it the same
  way. It is also the set navgen carves, so skipping it avoids fighting the mesh. A named `func_wall_toggle` is not
  carved but a barrier (`doc/path_planning.md`, "Barriers"); skipping it is still right, because the planner never
  routes through a barrier while the wall stands.
- **The door on the route is exempt, without an entity id.** `RouteExemptsHit` scans the next `kAvoidRouteLookahead`
  (3) waypoints for `requiresDoor`/`requiresBreak`, resolves `targetAreaId`, and skips the hit when that area's bounds
  overlap the entity's `absmin`/`absmax`. navgen builds a door or breakable area from the entity's own brush bounds,
  so a closed door overlaps (the bot walks in and touches it open) and an opened door has moved out of the volume and
  is avoided normally. `NavAreaRecord` stores no entity index, so this is the only link. `requiresElevatorRide` in the
  window exempts every `SOLID_BSP` hit, so the bot does not fight the plat it is boarding.
- **The goal item is exempt**, by edict (`bot.goalItem`), before the `SOLID_BSP` test. In practice only chargers
  qualify — other items are `SOLID_TRIGGER` — and only on the last stride, where the hull grazes the brush it is
  about to `+use`; that graze would otherwise force a useless repath.
- **Chargers are carved but not flagged.** `func_healthcharger` and `func_recharge` are carved by navgen but
  `CWallHealth::Spawn` never sets `FL_WORLDBRUSH`, so the mesh keeps 16 units of clearance while a bot that drifted
  into one still gets the detour search. Deliberate.

### The three maneuvers

The forward trace uses the hull the bot occupies (1 standing, 3 ducked), `dont_ignore_monsters`, over
`speed * kAvoidLookaheadTime` (0.5 s) clamped to `[kAvoidLookaheadMin, kAvoidLookaheadMax]` (64, 160) — its own
trace, since `EngineTraceFn` passes `ignore_monsters` and discards `tr.pHit`. Heights are against the bot's feet:
`gap = absmin.z - feetZ`, `top = absmax.z - feetZ`.

**Jump-on is tried first**, because crawling costs `kDuckingMultiplier` of speed for the whole traverse. It applies
when `top` is above step height (below that the engine steps up free) and within `PlainJumpHeight(gravity) + 18`
less a margin; 18 is the feet lift from ducking in the air. It fires when `dist <= speed * riseTime +
kAvoidJumpLead`, `riseTime` being the smaller root of `top - 18 = kJumpImpulse*t - g*t²/2`; at a standstill that is a
fixed 8-unit trigger.
- **A body is jumped only when speed carries the bot over it.** Against a player or monster the jump also needs
  `speed * DropTime >= dist + depth + 32` (`DropTime` is the larger root, when the feet sink back below `top`).
  Otherwise it falls through to duck or side search. Crates are exempt — landing on one is fine. Without this, the
  standstill trigger hops bots onto crouched teammates' heads (on de_inferno it froze a pair for over a minute).
- **A ducked bot cannot crouch-jump**: its origin rises 45 and the feet with it, with no 18-unit credit. So when
  ducked and `top` exceeds a plain jump, the duck latch is cleared and `suppressPlanCrouch` set instead; the bot
  unducks (instant where there is room) and jumps a frame or two later. `kAvoidLookaheadMin` is 64 for the same
  reason: a ducked bot moves at 83 u/s, and on `test52_avoid` a purely speed-proportional probe would not see the
  second can row in time to stand up.

**Crawl-under** applies when the bot is standing and `gap` clears the 36-tall duck hull with margin. The hold is
`(dist + depth + 2*kPlanClearance) / duckSpeed * kAvoidCrawlPenalty` (1.25) — remaining approach plus the bot's own
32-unit depth, at ducked speed. `slowApproach` caps the drive at ducked speed while the latch is set but `FL_DUCKING`
has not appeared (`PM_Duck` takes 0.4 s, 100 units at full speed, and test52 has 48 from spawn to the first row).
`kAvoidDuckMinHold` (0.5 s) exists because releasing `IN_DUCK` mid-transition aborts it.
- **The hold does not need to be accurate.** Too short: `PM_UnDuck` refuses to stand into a solid, so the bot stays
  ducked, and the `drive /= kDuckingMultiplier` compensation keys off `FL_DUCKING`, not the latch. Too long: the jump
  branch clears it. A separate "can I stand here" probe would solve a problem the engine already solves.
- A plan-mandated crouch always wins: avoidance may add a duck and drop its own, never override `requiresCrouch` or
  `NAV_AREA_CROUCH`.

**Going around** is the answer when neither height predicate applies — a standing player, a full-height crate, a
swung-open door: a side search whose route is spliced into the front of the plan.

### The side search

The obstacle box is expanded in x and y by `kAvoidHullHalf + kAvoidCornerMargin` (16 + 2), so its corners are
positions the bot's hull can occupy. In a frame where *forward* points from the bot to the obstacle centre, two
corners fall on each side. The search walks the perimeter: trace to the first corner on the chosen side, and after
each corner reached try straight to the destination, using up to all four corners so the route can wrap a far face
(two corners only suffice when the destination is roughly opposite the approach).

- **Corners need not be standable.** When the downward probe at a corner is `startsolid` (buried in a neighbour), the
  corner keeps the previous height and the leg is traced anyway; the leg then reports the neighbour, and the search
  recurses on it with the **final destination** as target, starting where the blocked trace began. On a cluster of
  touching obstacles this is the only thing that works: requiring standable corners abandons the side, and merging
  the boxes over-blocks empty diagonal quadrants.
- A leg blocked by *world* skips to the next corner; a leg blocked by an already-visited obstacle fails the side,
  which with `kAvoidMaxDepth` (6) and `kAvoidMaxTraces` (256) terminates the recursion. The visited set is restored
  before each side.
- **Which side first.** A moving obstacle is passed where it is not going: `dot(MoverVelocityAt, right)` above
  `kAvoidMoverSpeed` (16) picks the opposite side without evaluating the other. Otherwise both sides are evaluated and
  the shorter wins, ties going right — which passes two bots meeting head-on cleanly, since their `right` vectors are
  opposite.
- **A flat hull trace has no step-up**, so a stair nosing reads as a wall: a leg blocked by geometry whose normal is
  floor-like (`z >= kAvoidFloorNormal`, 0.7) is retried raised by `kAvoidStepHeight` (18), the idiom from
  `airmove_sampler.cpp`.

**The detour rejoins the plan at a sampled point, not a waypoint.** navgen cannot see a `cycler`, so a planned
waypoint is often *inside* the obstacle. The remaining polyline is sampled forward at `kAvoidDestStep` (16), and the
destination is the first sample outside the expanded box that passes the ground probe; its waypoint becomes
`detourReplaceUntil` and the waypoints before it are dropped. If the route enters the box at all, the rejoin point
must come *after* it. A beam is reported from as far as the lookahead reaches, so the first sample outside its box
can be a point still short of the beam; the search then returns no points and does nothing, every second.

The detour is capped at `kAvoidDetourMaxLength` (768) of total path. **The cap must not be relative to the distance
to the rejoin point**, which is deliberately close: `test52_avoid`'s five-can corner needs 432 units to reach a rejoin
point 60 away.

`pathDir` for the search is `Norm2D(follow.aimPoint - origin)`; within `kAvoidAimMin` (1) of the aim point — a bot
held still by the beam guard — that is noise, so the follower's wish direction is used instead.

### Splicing

`SpliceDetour` (`bot_move_nav.cpp`, because it has `MakePlanTraces`) rebuilds the list as `[bot origin] + [detour
points] + [plan from detourReplaceUntil]` with the index reset to 0, then `BuildPathProfile`, a fresh `TrackState`
and the progress clock reset — the same sequence as `Repath`. The detour therefore gets the arc profile, speed solve,
stall detector and status dump for free.
- **The list starts at the bot's own origin.** Starting at the far corner leaves the bot over `rejoinAbort` (96)
  lateral on frame one and `needRepath` undoes the detour; keeping traversed waypoints lets a from-scratch projection
  snap backwards.
- Detour waypoints carry `BotAvoid::kDetourClearance` (4) instead of 16, because a wide arc at a detour corner cuts
  back into the obstacle it clears.
- `estimatedArrivalTime` is extended by the added length so the ETA-derived test timeout stays honest.
- The splice invalidates the `wp` pointer, so the frame ends with `RunIdleMove` and a return.

### When no route exists

A failed search penalizes the connection the bot is crossing and forces a repath — **only when the penalty actually
lands**. `Bot_PenalizeCurrentConnection` recovers the connection from `plannedConnPath` (captured from
`path.connPath` in `Repath`) at the index where `plannedAreaPath` holds `currentAreaId`, and returns whether it found
one. On a single-area plan there is nothing to charge and the replan returns the identical plan; forcing it anyway
also resets `lastProgressTime`, which keeps the stall detector permanently one second from firing. That livelock
reads as `no clear rejoin point ... traces 0` followed by `Replan ... areas=1 ... penalized=0`, once a second,
forever.

- **Walk penalties need the reward too.** The blocker may be a player who moves a second later; without a reward one
  transient obstruction poisons a corridor for the map. `RewardEnteredConnection` fires when `currentAreaId` advances
  to the next planned area over a penalized connection. `kAvoidSearchCooldown` (1 s) bounds the loop to one penalty a
  second where the map has only one way through.
- **A stall while still blocked gives up the goal.** `BotGoal::OnBlocked` prints what stopped the bot and drops the
  goal (an item is `NoteIgnored` for `ItemUnreachableTimeout`), the same escalation `OnRepathFailed` uses for "no route
  at all" (`doc/goals.md`).
- **"Still blocked" has its own timestamp**, `state.blockedUntil = now + 2 * kAvoidSearchCooldown`, set on every
  non-receding failure. The sticky window on `state.lastKind` expires exactly at `nextSearchTime`, which with a 1 s
  cooldown and 2 s stall is exactly when the stall lands — a coin flip on frame timing.
- Breakables join the ladder before this: on both non-receding failure paths `RunSideSearch` calls
  `TakeBreakTarget` ahead of the `searchFailed`/`blockedUntil` assignment (see Breaking through). Neither break path
  sets `searchFailed`, so `avoid=break` is the only kind that survives the end-of-frame stickiness test on its own
  terms.
- That loop usually has a root cause upstream worth fixing, such as a charger missing from the mesh so the goal point
  is inside it (`doc/navgen.md`). The runtime guarantee here is only that a bad goal cannot livelock the bot.

### Obstacles that are leaving anyway

A `func_door_rotating` has `v.velocity == 0` — its motion is all `v.avelocity` — so by velocity alone it reads as a
wall, gets penalized, and forces a repath over something gone within a second. `MoverVelocityAt` gives the linear velocity of a chosen point
on a mover, `v.velocity + omega x r`, used by side ordering and failure handling.

- **`avelocity` is a rate on `(pitch, yaw, roll)` in degrees per second, and the mapping to world axes is not the
  identity.** From `AngleVectors`, pitch is rotation about **+Y**, roll about **+X**, yaw about **+Z** (counterclockwise
  from above), so `omega = (av.z, av.x, av.y)` in radians. Getting it wrong costs a sign, which is the whole answer.
- **`r` starts at the trace contact point and is stretched to the model's reach.** It cannot come from
  `absmin`/`absmax`: whenever a `SOLID_BSP` entity's `angles` are non-zero, `SV_LinkEdict` publishes a cube of the
  largest half-extent centred on `v.origin` (`test9_door`'s 18 × 66 × 98 leaf gets a 132-unit cube on its hinge). The
  condition is `angles != 0`, not "moving", so an open door keeps the cube permanently, which is also what swamps
  `PickDestination` near one. The contact point's *direction* from the axis is right but its distance is whatever the
  probe grazed, so `r` is scaled out to `sqrt(max(|mins.x|,|maxs.x|)^2 + max(|mins.y|,|maxs.y|)^2)`, treating the
  leaf as a bar. Only the magnitude can be overestimated; the sign never changes.
- **Dot against the direction to the next waypoint, not `pathDir`.** The aim point leads around corner arcs and can
  already point past a turn (on `test9_door` it gave −6, a wall; the next waypoint gives +102, receding).

Above `kAvoidMoverSpeed` a failed search sets `AVOID_FOLLOW` instead of `AVOID_BLOCKED`, and `bot_move_nav.cpp` skips
**both** the penalty and the forced repath (`hitRecedes`), so the bot keeps driving and walks after the door. Bounded
by: the door stops and `avelocity` drops to zero, so the next search penalizes normally; `kAvoidSearchCooldown`
re-takes the verdict once a second; and the 2 s stall detector is the outer net. A stationary obstacle scores exactly
zero — the `closing` field in the decision line.

### Breaking through

Breaking is decided in three places:
- **On the route.** navgen split the breakable into its own `NAV_AREA_BREAKABLE`, and the planner emitted a
  `requiresBreak` waypoint and charged `openSeconds`. `RouteExemptsHit` reports *why* it matched through a
  `wantsBreak` out-param, and a hit exempted for `requiresBreak` goes straight to `BotBreak::Engage` — no duck, jump or
  side search; the planner already decided and paid.
- **Under the feet.** The planner's traces (`MakePlanTraces`) set `TraceFilter::phaseBreakables`, which skips every
  snapshot solid that passed `BotBreak::IsTarget`, so an air move may fall *through* a breakable,
  such as a grate over a shaft, and the bot lands on it, off the mesh. `BreakBlockedAirmove` falls back to
  `v.groundentity` when it passes `IsTarget` (`AirmovePlug` found the target area unplugged) and engages it. `Run`
  holds `IN_DUCK` for an underfoot target: standing, the floor is 64 below the eye, ducked (`VEC_DUCK_VIEW`) it is 30,
  and `CCrowbar::Swing` traces 32 from the gun; an underfoot target is measured against `kBreakFloorReach` (31) instead
  of `kBreakReach` (28), inside the real 32 even at the 89° pitch clamp. `Finish` forces the repath once the floor is
  gone.
- **Off the route.** An unplanned breakable in the way is the *last* resort, after jump, duck and side search
  (`TakeBreakTarget`, see above). A crate the bot can hop onto is hopped, and a breakable it cannot pass is broken
  instead of reaching the penalty and `OnBlocked`. A receding mover never gets here (`!recedes` guards both sites).

### Tripmine beams

A tripmine beam is a `CBeam`: `SOLID_NOT`, no bounding box, invisible to the hull trace. The mine detonates on its own
`FTRACE_SIMPLEBOX` traceline from its origin along its normal, so the bot must avoid a **line segment** the engine
never reports.

**The registry** (`BotDeploy::RefreshBeams`) is shared and refreshed at 4 Hz at the top of `BotManager::StartFrame` —
sixteen bots scanning `maxEntities` every frame is the expensive way to learn something that changes only when a mine
is placed or destroyed. One entry per `monster_tripmine`: handle, origin, and the beam end from one 2048-unit
traceline, mirroring `CTripmineGrenade::MakeBeam`.
- **Registered from spawn, not from arming**, with `Beam::armed` recording which (see `bot_deploy.cpp`, "The arming
  window"): an unarmed beam is about to be lethal, and the placing bot stands in it.
- **The direction is un-negated.** The mine's angles come from `UTIL_VecToAngles(planeNormal)`, whose pitch is the
  opposite of a view angle's, and `CTripmineGrenade::Spawn` recovers the normal with `UTIL_MakeAimVectors`, which
  negates pitch. Skipping that gives a vertically mirrored, mostly plausible beam.
- **It reads every loaded game.** `RefreshBeams` runs before any game is entered, after the previous frame's
  `XPlay::LeaveGame()` nulled `g_pGameProfile`, so it collects classnames from `XPlay::ProfileFor(g)` for every loaded
  game, which needs no context: `DeployEntity(DEPLOY_TRIPMINE)` is a beam, `IGameProfile::HazardEntity()` a floor
  hazard. Asking `g_pGameProfile` there leaves the registry silently empty.

**The beam as an obstacle.** `Probe` is the one choke point for "is this leg blocked, and by what", so teaching it
about beams makes `Solve`, `SolveAround` and the destination search beam-aware together. It marches the hull AABB
along the leg in 8-unit steps and slab-clips each beam against it; the nearer of solid hit and beam wins, and the
mine's edict is reported as `tr.pHit`, so the visited set and messages work unchanged. `SolveAround` and
`PickDestination` take explicit bounds, because the obstacle is the **beam's** bounding box, which for a wall mount
spans the doorway.
- **Mine body and beam are one obstacle.** A ground mine sits 8 off the floor, so the trace hits the *body* first: a
  16-tall box, too low to jump, no gap to duck, and `BotBreak::IsTarget` rejects `MOVETYPE_FLY`. `BotDeploy::BeamFor`
  maps a hit edict back to its beam in both the forward trace and `Probe`.
- **`depth` is the bot's own 32**, not the beam box's width, or a wall-to-wall beam would hold the duck the length of
  the room.
- `gap` and `top` come from the beam's z range *inside* the hull: a beam 40 or more above the feet is **crawled
  under**; above 72 it is not an obstacle; anything else is **walked around**, and shot if that fails. A vertical
  beam spans the column (`gap` 0, `top` 72) and always lands there.

**A low beam is crouch-jumped, with its own timing (`TryBeamJump`).** The ladder for a beam is duck → crouch-jump →
reroute → shoot. The trip test is a traceline against the player's box, so the box must never touch the beam on any
frame. The solid-obstacle jump (`canJump`, still `!beam && ...`) cannot do that: it launches so the feet *reach* the
top as the bot arrives, which sweeps the box up through the beam, and every `walk`-state own-mine death in the first
instrumented run was that. A beam jump is timed so the **apex is over the beam**:
- An in-air duck keeps the origin, so the tucked feet sit 18 higher: `h(t) = 18 + J·t − g·t²/2`, peak about 63. The
  roots of `h = beamHi + pad − feetZ` give the window `[t1, t2]` the feet are above the beam.
- Along the travel direction the 32-wide box overlaps the beam line for `2·reach`, `reach = 16·(|n.x|+|n.y|)/|dir·n|
  + pad`. Feasible when `v·(t2−t1) ≥ 2·reach` and the bot is not already inside the window; it launches at
  `D = v·J/g`. A beam 30 up at 250 u/s has about 135 units of window for about 40 of box; one 55 up barely fits; a
  standing mount (64) never does, and those are crawled under anyway. A vertical (ground-mounted) beam has no 2D
  length and is never jumped, and the approach must be within 60° of square.
- **It plans on the speed it can reach, not the speed it has.** Bots meet low beams slow — just braked, or just done
  placing — and a 0-speed bot reaches full speed in about 0.1 s. The launch speeds that fit are
  `[max(100, 2·reach/(t2−t1)), min(maxSpeed, (D−reach)/t1)]`: the approach continues while that range is non-empty and
  the bot is not already faster than its top, and it launches once its actual speed is in range and `D ≤ v·J/g`. The
  moment the range closes, `jumpBeam` is cleared and the guard brakes again.
- **Validated by simulation before launch** (`BeamJumpLands`): one `PmSimulator::BeginAirborne` on the duck hull with
  `(vx, vy, J)`, checking `BeamMeetsHull` every tick and requiring a landing on the far side of the beam line. That
  covers ceilings, walls, players in the way and the landing. A refusal logs `avoid beam-jump ... refused`.
- **The guard is told.** While the jump is feasible and through the flight, `jumpBeam` exempts that one beam from
  `BeamGuard` (`BotAvoid::BeamJumpActive`), held 0.1 s past each frame that confirms it. Without it the guard brakes
  the bot at about 127 units, before the about 84-unit launch point — the same guard-against-ladder conflict as the
  dust2 bug.
- A ducked bot that could jump stands up first (`suppressPlanCrouch`), as the solid jump does.
- **Every jump is scored.** `beam jump cleared` (landed on the far side, mine alive), `beam jump FAILED (mine went off)`
  (the mine vanished within 2 s), `beam jump FAILED (killed)` (from `NoteDeath`), `beam jump inconclusive` (neither in
  2 s, usually a player in the way). `test_deploy.ps1` asserts the failure rate is at most 5%. `crossbot_beam_jump 0`
  turns the branch off.

**Reroute before shooting.** Where the side search fails against a mine the bot has not met, `RerouteAroundMine`
puts it on the bot's own list (`blockedMines`, last 4) and sets `searchFailed`, so the existing branch penalizes the
current connection and repaths. No route check follows: if the new route runs into the same beam, the next hit finds
it on the list and goes to the shot. The list is per bot; a stale handle drops out when the mine dies.

**When nothing works, shoot it.** For a mine already on the list, `TakeMineTarget` sets `mineTarget`, and `Bot_MoveNav`
hands it to `BotBreak::EngageStandoff` at `kMineSafeRadius` (375). `TakeBreakTarget` cannot serve: `IsTarget` excludes
`MOVETYPE_FLY` on purpose, and a crowbar against a 150-damage blast is not a plan.
- **Own mines are not exempt.** The beam test traces from the mine, and its `pev->owner` is the wall, so the placer
  breaks the beam like anyone; there is no owner filter anywhere in this path.
- **Placed satchels get no code.** A live `monster_satchel` is 8 units tall, under the 18-unit step.
- **A standoff against an arming mine ignores `SOLID_NOT`.** `BotBreak::Run` finishes on `SOLID_NOT`, which is how a
  broken breakable looks and also how a tripmine looks for 2.5 s; finishing on it would report `destroyed after
  0.0s` and re-engage every second. A standoff backs off to range and waits instead. A CS bullet does destroy an HL mine:
  CrossPlay proxies the TraceAttack/TakeDamage pair.

### The guard in Bot_RunPlayerMove

`BotAvoid::Update` runs in exactly one place, the walk drive at the end of `Bot_MoveNav`; every maneuver that owns the
frame returns before it. So a bot in combat, break, deploy, ladder, elevator, airmove or charger gets no beam avoidance
from it. Measured without the guard, six of nine own-mine kills were in `maneuver combat` with `avoid none`.

The guard sits in `Bot_RunPlayerMove`, the funnel every subsystem's movement passes through, so no maneuver can forget
it. It reconstructs the intended world direction from `fmove`/`smove` (`ComposeMove`), asks `BotDeploy::BeamGuard`
whether that step enters a beam, and forces a duck when the ducked hull would clear it, refusing the step otherwise.
- **It ignores beams the hull is already inside.** A bot placing a tripmine stands on its own beam line by
  construction, and the beam is registered from spawn; a guard that stopped it there would pin it in its own mine
  until it armed. `BeamGuard` tests each beam at the current origin first and only refuses steps *into* a beam.
- **Refusing means braking.** The guard drives against the current velocity at full speed. Merely zeroing the input
  leaves a bot coasting a third of a second at 250 u/s, far enough to cross a doorway and die with `onbeam 1`
  (about two contact deaths per run, against zero with braking).
- **Margin is in the beam's XY extent.** `BeamMeetsHull` takes a `pad` inflating the hull horizontally, so
  `BeamGuard` and `BeamAlong` get room to stop or decide without their own timing constant; diagnostics use `pad` 0
  to report true contact. Padding z would make a beam above head height an obstacle.
- **Both use `BotDeploy::kBeamGuardPad`.** Different pads (say guard 24, avoid 16) leave a band where the guard
  brakes and avoidance sees nothing to decide. The guard logs `beam guard stop|duck` with the bot's lateral distance at most
  once a second per bot; a run of those at one spot with no `avoid` line between them is this class of bug.

`test_deploy.ps1 -Phase crossgame` reproduces the cross-game case (an HL mine across de_dust2's T-side mid doorway,
a CS bot behind it), which needed both the rejoin-after-the-box rule and the `kAvoidAimMin` fallback above.

### Floor hazards: TFC caltrops

A caltrop (`tf_weapon_caltrop`) is a `SOLID_TRIGGER` that hurts on touch — 10 damage plus leg damage — so no hull
trace reports it either. Caltrops hurt enemies, the thrower, and teammates whenever `teamplay` enables friendly fire
(`CaltropTouch`, tfc.c), so every one is avoided. They scatter for a moment, then lie for 10–15 s.

A landed caltrop (`FL_ONGROUND`; one still flying is skipped, or the bot would duck under it) is registered as a
vertical `Beam` from `absmin.z` to `absmax.z + 2` with `hazard` set, so `BeamAlong`, the side search and `BeamGuard`
all apply unchanged. Two differences from a tripmine:
- **It is never shot**: `TakeMineTarget` refuses a hazard.
- **With no way around, the bot walks over it.** `WalkOverHazard` logs `avoid walks over`, returns no maneuver, and
  opens `walkOverUntil` for 1 s, during which `BeamGuard` ignores hazards so it does not brake the bot straight back.
  Standing still for 15 s costs more than 10 damage.

On casbah, 15 scared caltrop throws produce 2 caltrop hits with hazards registered, against 16 without.

## Breaking (`bot_break.cpp`)

`BotBreak` owns the frame while the bot destroys something, like the elevator and charger runners: aim, drive, buttons
and `Bot_RunPlayerMove` come from here and `Bot_MoveNav` returns. It is called *before* the progress/stall/repath
block so a repath never interrupts a swing, and refreshes `lastProgressTime` every frame so the stall detector and
`OnBlocked` do not throw away a goal two swings away.

### What counts as breakable

`IsTarget`: `SOLID_BSP` or `SOLID_BBOX`, `MOVETYPE_PUSH` or `MOVETYPE_PUSHSTEP`, not a player or monster,
`takedamage != DAMAGE_NO`, and `0 < health <= kBreakMaxHealth` (100).
- **No classname test.** `takedamage` and `health` decide it, which picks up mod breakables for free. Nothing meant to
  be avoided qualifies — `func_wall`, chargers and doors are all `DAMAGE_NO`. `takedamage` is the exact inverse of
  `SF_BREAK_TRIGGER_ONLY` (`CBreakable::Spawn`), the same distinction navgen makes with `kSfBreakTriggerOnly`.
- **No owner test: another game's entities qualify.** Each game DLL keeps its own `gMultiDamage`, but CrossPlay's
  `TraceAttack` proxy (`crossplay/vfproxies.cpp`) runs the call in the owner's context and re-adds what it queued to
  the caller's, so a valve glock breaks a cstrike `func_breakable`. An owner test would only hide breakables like the
  cs_assault grate from the planner.
- **`SOLID_BBOX` is required, and the movetype test makes it safe.** `func_pushable` with `SF_PUSH_BREAKABLE` (128)
  spawns `SOLID_BBOX` and `DAMAGE_YES`; excluding it wedges bots on `test13_break` with the charger-livelock shape. But
  `SOLID_BBOX` alone admits live satchels (`DAMAGE_YES`, `health` 1). `MOVETYPE_PUSH`/`PUSHSTEP` separates brush
  entities from thrown props: grenades and satchels are `MOVETYPE_BOUNCE`, tripmines `MOVETYPE_FLY`, monsters
  `MOVETYPE_STEP`.
- **`health <= 100` because a crowbar does 50.** `CBreakable::TakeDamage` doubles `DMG_CLUB`, so a swing lands
  `2 * plrDmgCrowbar` = 50 (always the full-damage branch under `IsMultiplayer()`), and a qualifying target dies in at
  most two swings — under a second, so no interaction with combat is needed.
- **`takedamage` is necessary, not sufficient, and nothing can make it sufficient.** `TakeDamage` early-returns on
  `!IsBreakable()`, so `matUnbreakableGlass` keeps `DAMAGE_YES` and absorbs everything, and `m_Material` is private
  data with no gamedata offset. It is caught by watching `pev->health`: after the first swing, `kBreakNoDamageGrace`
  (2 s) without the health moving aborts.

### Giving up

`Abort` fires on the no-damage grace or `kBreakTimeout` (8 s). Beyond restoring the weapon it records the entity in
`refused` for `kBreakRefuseTime` (15 s) so the next frame does not re-engage, calls `Bot_PenalizeCurrentConnection` so
the planner routes around it, and forces a repath to spend that penalty. That is the whole answer to breakables navgen
cannot classify offline: unbreakable glass and anything over 100 health are discovered once, at runtime.

### The weapon

`IGameProfile::MeleeWeapon()` names it (HLDM: the crowbar). Without it the active weapon is used and the bot shoots —
worse for ammo, never wrong. The switch is `FakeCmd::Run(pEdict, spec->classname)` rate-limited to
`kBreakSelectInterval` (0.5 s), the mechanism `BotFire::Think` uses. `prevWeaponId` is captured at `Engage` and redrawn
on `Finish` and `Abort`; if it is gone, `BotFire` re-picks within half a second.

**`BotFire::Think` idles while a break is active.** `Bot_RunPlayerMove` ORs in `BotFire::Buttons`, and `Think`
re-rolls a weapon every half second whenever it has a target, which would fight the melee switch. Idling leaves
`fire.mode == MODE_NONE`, so `BotBreak` is the only source of `IN_ATTACK`.

### Aim and reach

`CCrowbar::Swing` traces 32 units from `GetGunPosition()` along `v_forward`, so reach is **eye to surface**. The aim
point is the eye clamped into the entity's `absmin`/`absmax`, nudged `kBreakNudge` (2) toward the centre so the
traceline ends inside the brush rather than grazing its face. With the eye already inside the box (the hull overlaps
before the trace does), it aims at the centre.

### The standoff variant

`EngageStandoff(bot, ent, minRange)` is the same machine with the approach inverted, for tripmines. It shares `Run`,
the weapon save/restore, the `lastProgressTime` hold and `Abort`. Four differences:
- **It bypasses `IsTarget`**, which excludes `MOVETYPE_FLY` to keep live satchels out of the melee path; the caller
  names its target instead.
- **`inRange` inverts**: `los >= standoff`, and `moveDir` is negated so the bot backs away along its approach.
  `kMineSafeRadius` is 375 because that is where a 150-damage blast reaches zero.
- **A short corridor must not deadlock.** `kBreakRetreatTime` (4 s) forces `inRange` true, so a bot backed against a
  wall shoots anyway and eats some blast. On `test2_doorway`, smaller than 375 end to end, every standoff fires on the
  timer and still kills the mine in one shot.
- **The weapon choice inverts.** `WantWeaponId` returns the active weapon unless it *is* the melee weapon, then the
  first non-melee weapon carried.

A tripmine is `health == 1`, `DAMAGE_YES`: one hit destroys it, and `Killed` → `DelayDeathThink` → `Explode` is the
expected outcome. The no-damage grace still means shots are not landing.

## Deploying (`bot_deploy.cpp`)

Placing tripmines and satchels across chokepoints, and detonating satchels. It owns the frame like `BotBreak` and is
called from `Bot_MoveNav` right after `BotBreak::Run`, before goal selection. `BotFire::Think` idles with
`why=deploying` for the same reason it idles during a break. `Bot_ManeuverActive` includes `deploy.active`, which stops
`TryStart` re-entering and keeps `BotCombat::Run` from tearing a placement out from under it.

**Why not in `BotFire`:** `BotFire::Think` returns `no-target` before any mode is considered, and deployables fire with
no target at all; they need a decision layer that runs on the map, not on an enemy. `Supported` still rejects
`AIM_SURFACE`, `AIM_NONE` and `WPN_Q_HIDDEN_STATE`.

### The damage rule is one number

`sk_plr_tripmine` and `sk_plr_satchel` are both 150, and `CGrenade::Explode` calls `RadiusDamage(pev->dmg,
pev->dmg * 2.5)`: radius 375, damage at distance `d` is `150 - 0.4d`.
- **`kDeployKillRadius` = 125**, where the blast still does 100 — "a gate small enough that anyone in it dies".
  `Covers` applies it directly: three points across the gate span at three body-centre heights, every sample within
  125 *with line of sight to the mount*.
- **`kMineSafeRadius` = 375**, where it does nothing: the standoff range.

### Finding chokepoints

Every walk connection carries `gateA`/`gateB`, which survive the `.nav` round trip, so deployment needs nothing extra
in the `.nav`. The gate supplies the crossing point and the beam direction, not a usable width.
- **The stored gate is inset twice.** Areas come from carving the hull-3 clipnode tree, so their boundaries already sit
  a player radius inside the walls, and `GateFromEdge` insets each end by `kPlanClearance` again: the true opening is
  `|left - right| + 4 * kPlanClearance`. Getting this wrong is invisible, which is why the code trusts a physical
  measurement instead.
- **A short gate is not a chokepoint.** Seams across open floor are short for the same reason areas are small, so
  width alone accepts 1238 gates on crossfire, all rescued by the ceiling/ground fallback. `EvaluateGate` measures:
  from the eye at the gate midpoint it traces both ways along the gate; **both sides must hit something** within
  `kDeployReach` (128), and `opening = distA + distB <= kDeployOpeningMax` (144). That leaves crossfire 99 gates, with
  an opening histogram at 96 and 128 — HL's two standard doorway widths, the sanity check that matters.
- **Direction.** Ordinarily `normalize(right - left)`. A clamped gate (raw opening under 32) has `left == right` and no
  direction; the far side's gate midpoint supplies it, since `gateB.mid - gateA.mid` is the crossing normal and the
  gate direction is its horizontal perpendicular. Centroids are only the last fallback: they measure `test2_doorway`'s
  64-wide openings diagonally as 113, and preferring the gate midpoints moves accepted gates from 93 to 114 on
  `stalkyard` and 91 to 118 on `datacore`.

### Where the mine goes

`CTripmine::PrimaryAttack` traces 128 from the gun along `v_forward`, spawns `monster_tripmine` at
`tr.vecEndPos + tr.vecPlaneNormal * 8`, and sets its angles from the plane normal — **the beam runs back out along the
surface normal**. A beam across a doorway therefore needs a *side* wall whose normal lies in the gate plane; `Probe`
requires the hit normal within `kDeployNormalTol` (0.94, ~20°) of facing back down the trace.
- Preference is wall, then ceiling, then ground, and the vertical two need `opening <= kDeployVerticalMax` (48): a
  vertical beam is a line, and in anything wider than a player people walk past it (forced onto crossfire's 128-wide
  doorways, they are stepped around without avoidance ever engaging).
- The bot stands **at the gate midpoint**, and `EvaluateGate` probes from exactly there, or the pre-scan stops
  predicting what the shot will do.
- **The beam must stay inside the doorway.** `BeamContained` traces `kDeployBeamLength` (2048) from the mount along its
  normal and requires a hit within `opening + kDeployBeamSlack` (24); otherwise the opposite wall is tried, and failing
  both the gate is rejected as `REJECT_ESCAPE`. `kDeployNormalTol`'s 20° allows ~46 units of drift across a 128
  opening — enough to miss the far jamb and run 2048 down a corridor. A runaway beam is dangerous because the blast is
  at the mine while the trip point can be hundreds of units away: the remaining own-mine deaths are `onbeam 0`, 40–240
  units from the mine when a stranger tripped the far end. This check subsumes demanding a facing wall on both sides.

### Stance

Standing the eye is 64 above the floor, ducked 30 (`VEC_DUCK_VIEW` 12 above a duck origin 18 up), and the mine mounts at
eye height, so the two stances land on opposite sides of the avoidance predicates (64 is crawled under, 30 must be
walked around or shot). `TryStart` flips a per-bot coin so both get exercised. Nav areas are in duck-origin space, so
from the gate's z (`floor + 18`) the eye offsets are 46 and 12.

### The arming window

`CTripmineGrenade` is `SOLID_NOT` with no beam for 2.5 s (`m_flPowerUp`, `PowerupThink` every 0.1 s). On arming it goes
`SOLID_BBOX` and `MakeBeam` stores `m_flBeamLength = tr.flFraction` from a `dont_ignore_monsters` trace;
`BeamBreakThink` retraces every 0.1 s and detonates on `fabs(m_flBeamLength - tr.flFraction) > 0.001`. **A player on the
beam line at arming becomes the baseline**: the mine arms around them and fires the instant they step off.

The placing bot is always on that line — it stands at the gate midpoint, the mount is where its eye ray hits the side
wall, and the beam returns along the normal through the eye; no mount geometry avoids it. That is why `RefreshBeams`
registers mines from spawn. Measured, it is not what kills bots: they clear their own beam in 0.13–0.28 s, and
`STILL IN BEAM AT ARMING` fires about once in 73 placements, killing nobody. Own-mine deaths come from avoidance gaps
(see the guard and "A low beam is crouch-jumped" above).

### Phases

`APPROACH` → `SELECT` → `AIM` → `FIRE` → `VERIFY`, plus `DETONATE` from its own trigger. The chassis is `RunActivation`
almost verbatim, including three easy-to-lose details: `lastProgressTime` is refreshed every frame; the press is gated on
the angles `Bot_UpdateFacing` *returned*, not those requested, because the aim controller is rate-limited; and the
approach is `NavFollow::SolveDrive` + `Bot_DecomposeMove` straight at the spot, safe because an area is a convex prism
and the gate midpoint lies on its boundary.

- **`VERIFY` differs per weapon.** A tripmine is confirmed by a `monster_tripmine` within 48 of the predicted mount. A
  satchel leaves at 274 u/s and is 80 units away by the 0.3 s verify (`kDeployVerifyDelay`), so it is confirmed by
  `m_chargeReady == 1`, the game's own record of a deployed satchel.
- **The memo is not optional.** Every exit writes `Memo { connIndex, expireTime }`, success or failure, and `FindBest`
  honours it. Without it the bot re-picks the gate it just failed at and spins at full framerate where it already
  stands — the failure mode described for `BotItems::OnArrived`, which no stall detector catches.

### Satchels

The satchel shares the chassis with the surface search skipped: the spot is the gate midpoint, coverage runs from the
floor rest point, and the press is `IN_ATTACK2`. `CSatchel::SecondaryAttack` throws for any `m_chargeReady != 2`, so
throwing needs no state; only detonation does. Tripmine is preferred, a satchel placed only without tripmine ammo, and
`CSatchel::AddDuplicate` refuses new satchels while any are deployed, so the live group limits itself.

**Detonation** runs every frame in `TryStart`, ahead of the placement throttle: for each live `monster_satchel` the bot
owns, fire when a live suspicion sits within 375 of it **and** closer to it than the bot, out of combat and with no fire
commit open. `BotSense::NearestLive` is the spatial query (same `[0, count)` scan through `LiveWeight` as `Freshest`
and `LiveCounts`).
- **`livingOnly` is the whole feature.** Without it a run detonates 181 times off 7 satchels, each `contact 2
  away`: the satchel's own landing sound (`weapons/g_bounce*.wav`) makes `BotSense::Hear` push a suspicion.
  `NearestLive` skips `KIND_OTHER` and anything the bot owns.
- **With teams it also means "an enemy".** The filter is inside the scan, not at the call site, because `NearestLive`
  returns only the nearest match and a teammate closer to the satchel would mask a real enemy.
- **The press is confirmed, not assumed.** `DETONATE` presses only when `m_chargeReady == 1` and both the player's and
  the weapon's attack timers are clear, waits `kDeployVerifyDelay`, and is done only if `m_chargeReady` has left 1.
  Counting it as done on the press frame re-fires forever.

### Console

`crossbot_list_gates`, `crossbot_deploy_status`, `crossbot_deploy_surface` and `crossbot_deploy_debug` are in
`doc/console.md`. Pinned to ground on `test2_doorway`, `crossbot_deploy_surface` runs the side-search and standoff
branches every time.

## Trigger forces at runtime

How navgen samples `trigger_gravity` and `trigger_push` — one air-move set per gravity multiplier, push fields as
launch seeds, start-off fields, the lowest-gravity preview path — is `doc/navgen.md`. The simulator's push integration
is `doc/path_planning.md`, `pm_sim`. The runtime side:
- **`trigger_gravity`** writes its multiplier onto `pev->gravity` of whoever touches it, and it sticks until respawn.
  `Bot_Physics` multiplies it into plan physics, and only air moves baked at that multiplier exist for the bot
  (`doc/path_planning.md`, "Planning with gravity multipliers").
- **`trigger_push`** adds a base velocity only while touched. It is `SOLID_TRIGGER`, so it never blocks or supports
  the mesh. `Bot_PushFields` reads the fields from the plan's world snapshot (which skips `SOLID_NOT` ones), so a toggled field is
  picked up by the next repath; `Bot_DriftVelocity` reads the shove live for steering (Following a drifting path).
