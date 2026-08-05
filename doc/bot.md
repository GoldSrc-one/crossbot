# Bot System Comments

## bot_move.cpp

### Bot_DecomposeMove
PM_WalkMove and PM_AirMove both flatten forward/right and renormalize before use, so whatever pitch and roll the engine puts in pmove->angles drops out and the basis is just the yaw.

### Bot_FrameMsec
Frame time converted to milliseconds, clamped to [1, 100].

### Bot_NormalizeAngle
Wraps an angle into (-180, 180]. Lives here rather than in bot_move_nav.cpp so both movement
translation units share one copy.

## bot_move_nav.cpp

### Constants
- kFullDrive: Over maxspeed on purpose: PM_CheckParamters rescales the forward/side pair down to maxspeed, so this just means "full speed in exactly the commanded direction".
- kAimVelocitySpeed: Below this the velocity carries no usable heading, so the aim holds where it is rather than snapping about on numerical noise.
- kTeleportJumpThreshold: Distance threshold to detect teleport/jump and reset path.
- kRepathInterval: Interval for forced repath when stalled.
- kEtaSafetyMargin: Safety margin multiplier for ETA calculation.

### ResetPath
Clears all path state: waypoints, indices, timers, progress tracking, visited areas.

### WriteTraceLog
Writes bot trajectory trace to file specified by crossbot_trace_path cvar (defaults to valve/crossbot_data/nav_trace.txt).

### Repath
Finds path from current area to goal area using A* and builds waypoint plan.
- Clamps goal point into goal area bounds.
- On fresh attempt, calculates total path distance and estimated arrival time.
- Logs waypoint details including jump/crouch flags and target area IDs.

### Bot_MoveNav
Main navigation-driven movement function:
- Detects teleportation via origin jump and resets path if needed.
- Updates current area ID.
- If no goal, runs idle move.
- Triggers repath on empty path, forced repath (airmove landed off-target), or stall (no progress for kRepathInterval).
- Tracks visited areas for debug.
- Logs position to trace log (capped at 100k points).
- On waypoint completion: logs goal reached, writes trace, clears goal.
- Uses NavFollow::Track against the `PlanProfile` built at repath time (see doc/nav.md). Progress is
  arclength along the arc chain, so there is no per-waypoint arrival radius or advance loop, and
  waypoint 0 is no longer a special case — the chain starts at the bot's own position.
- Drive magnitude is the servo's `wishSpeed`, divided by `kDuckingMultiplier` when `FL_DUCKING`
  (the engine applies 0.333 after the maxspeed clamp) and then clamped to maxspeed. This replaced the
  constant `kFullDrive = 1000`, which meant the bot had no speed control at all: it entered every
  corner at full speed and walked past the goal until an arrival radius caught it.
- Stall detection is rebased on `trackState.progressS` advancing, not on the waypoint index changing —
  a bot correctly braking to a stop at a hard node would otherwise read as stalled.
- `BotPhysics` reads sv_gravity, sv_airaccelerate, sv_accelerate, sv_friction, sv_stopspeed and
  sv_edgefriction, so plan-time physics match a non-default server. Only the first three were read
  before, which silently diverged the profile from the engine.
- Handles jump waypoints as commitments - not passed until jump is made.
- Steers toward next waypoint for aiming (looking around corners).
- Computes move direction from the steered direction.
- Sets desired angles toward next waypoint or velocity direction.
- Decomposes move into forward/side speeds for RunPlayerMove.
- Sets movement buttons (forward/back, left/right, duck, jump).
- For jumps: checks on-ground, distance, facing tolerance; adds duck while rising.

### RunAirmoveApproach / RunAirmoveDrive
The steering that takes over once the tracker's `wpIndex` reaches an air-move launch waypoint. It aims
`kAirmoveApproachLead` past the launch along the launch heading and commits when the bot is within one
frame of the launch plane.

Two things about the drive are easy to get wrong and both stalled real routes:

- **The command is a target velocity, not a wish speed.** `RunAirmoveDrive` puts `drive` through
  `NavFollow::SolveDrive` while on the ground, exactly as `Track` does. Passing `launchSpeed` straight to
  `pfnRunPlayerMove` looks equivalent and is not: below `sv_stopspeed` the engine's friction drop is
  constant, so a raw wish speed under `stopSpeed * friction * edgeFriction / accelerate` moves the bot
  nowhere at all. At a ledge — which is where every dropoff launches from — `sv_edgefriction` doubles
  that to 80 at defaults. A dropoff planned at 57 pressed at full drive and crawled at 2 u/s until the
  stall timer repathed it, forever. `doc/nav.md` has the derivation.
- **Ducking scales the command, so the command has to be scaled back.** `PM_Duck` multiplies
  forwardmove/sidemove by `kDuckingMultiplier` *after* `PM_CheckParamters` has clamped them, so
  delivering wish speed `W` while `FL_DUCKING` means commanding `W / kDuckingMultiplier` and accepting
  that nothing above `maxspeed * kDuckingMultiplier` is reachable. `Bot_MoveNav` did this and
  `RunAirmoveDrive` did not, so every crouched air move was driven at a third of its planned speed.

`RunAirmoveFlight` keeps commanding the raw heading — no servo — because `PM_AirAccelerate` caps its wish
speed at 30 in flight and the trajectory is already committed.

**Repathing has no memory, so a maneuver the follower cannot execute becomes an unbounded loop.** Both
repath triggers — `follow.needRepath` when lateral exceeds `rejoinAbort`, and the `kRepathInterval`
stall timer — re-plan from the bot's current position with the full graph. Nothing records that the
bot just failed this jump, so the planner is free to hand back a route through it again. Worse, the
new start position can flip which route is cheapest, and the bot alternates: on test50_stairs it ran
areas 11 → 12 → 13, missed the launch at (124,221,-124), fell back to 11, re-planned onto the
crouch-jump 11 → 13 instead, landed in 13, missed the same launch, and repeated — `actual=` grows as
`[11,12,13,11,12,13,...]` for the whole run. Two routes into the same dead end read as progress to
every check the bot has. Penalising recently-failed connections across repaths is the missing piece;
until it exists, an oscillating `actual=` list means "the follower cannot do this maneuver", not "the
planner keeps changing its mind".

A repath inside an air move's run-up leg makes this worse in a way worth recognising. `Repath` starts
the plan at the bot's own position and `Cross` then places the run-up node a fixed distance back along
the launch heading, so a bot that has already closed most of that distance gets a first waypoint
*behind* it and walks backwards to reach it. The plan is not wrong — it is the same plan it would
build from anywhere — but the bot arrives at the run-up node with no speed, which is the worst state
to start a launch that has to build speed at a ledge. Two waypoints at the same 2D point at the head
of the list in `crossbot_nav_status` are the tell.

## bot_manager.cpp

### AppendAreaList
Helper to format area ID lists for debug output, truncating to 24 entries with ellipsis.

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
  Actual diverging once is a repath; actual cycling is the loop described under `Bot_MoveNav`.
- `waypoint=i/n` against the waypoint list — which maneuver it is stuck before. `n` also reveals a
  repath, since a fresh plan renumbers from the bot's position.
- `hasGoal=0` with `waypoint=0/0` is ambiguous by itself: it is both "arrived and cleared the goal"
  and "gave up". Distinguish with the distance to the goal, and remember the goal was clamped
  (see `doc/nav.md`, `PlanPath`).

### PrintWaypoints
Prints detailed waypoint list for each bot.

## bot.h

### CBot struct
- Last forward/side pair handed to pfnRunPlayerMove, for crossbot_nav_status.

## bot_move.h

### Bot_DecomposeMove
Splits a world-space direction into the forward/side speeds pfnRunPlayerMove takes, which the engine reads along the view axes -- so this is what lets the bot move somewhere other than where it is looking. viewYawDeg must be the yaw passed to pfnRunPlayerMove in that same call plus punchangle, because PM_CheckParamters builds the movement basis from exactly that.

PM_CheckParamters clamps the pair down to maxspeed before anything reads it, so a driveLength well over maxspeed just means "full speed exactly this way", and one below it passes through untouched and becomes the actual speed.