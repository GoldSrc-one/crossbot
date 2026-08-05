# Bot System Comments

## bot_move.cpp

### Bot_DecomposeMove
PM_WalkMove and PM_AirMove both flatten forward/right and renormalize before use, so whatever pitch and roll the engine puts in pmove->angles drops out and the basis is just the yaw.

### Bot_NormalizeAngle
Wraps an angle into (-180, 180]. Lives here rather than in bot_move_nav.cpp so both movement
translation units share one copy.

## bot_move_nav.cpp

### Constants
- kFullDrive: Over maxspeed on purpose: PM_CheckParamters rescales the forward/side pair down to maxspeed, so this just means "full speed in exactly the commanded direction".
- kAimVelocitySpeed: Below this the velocity carries no usable heading, so the aim holds where it is rather than snapping about on numerical noise.

### WriteTraceLog
Writes bot trajectory trace to file specified by crossbot_trace_path cvar (defaults to valve/crossbot_data/nav_trace.txt).

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

### Bot_MoveNav
Main navigation-driven movement function. Repath triggers are: empty path, forced (air move landed
off-target), and stall (no progress for kRepathInterval).

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
- Jump waypoints are commitments — not passed until the jump is made, and gated on on-ground,
  distance and facing tolerance, with duck added while rising.
- Aim leads the next waypoint rather than the current one, so the bot looks around corners; the move
  direction comes from the steered direction, not from where it is looking.

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

**A landing outside `airmoveTargetArea` penalizes the connection, then forces a repath.**
`RunAirmoveFlight` resolves the landed area and calls `Nav::PenalizeConnection` on the index latched at
launch, or `RewardConnection` when it landed where it was aimed (see `doc/nav.md`). Without that, a
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

A repath inside an air move's run-up leg makes this worse in a way worth recognising. `Repath` starts
the plan at the bot's own position and `Cross` then places the run-up node a fixed distance back along
the launch heading, so a bot that has already closed most of that distance gets a first waypoint
*behind* it and walks backwards to reach it. The plan is not wrong — it is the same plan it would
build from anywhere — but the bot arrives at the run-up node with no speed, which is the worst state
to start a launch that has to build speed at a ledge. Two waypoints at the same 2D point at the head
of the list in `crossbot_nav_status` are the tell.

## bot_manager.cpp

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
  (see `doc/nav.md`, `PlanPath`).

## bot_move.h

### Bot_DecomposeMove
Splits a world-space direction into the forward/side speeds pfnRunPlayerMove takes, which the engine reads along the view axes -- so this is what lets the bot move somewhere other than where it is looking. viewYawDeg must be the yaw passed to pfnRunPlayerMove in that same call plus punchangle, because PM_CheckParamters builds the movement basis from exactly that.

PM_CheckParamters clamps the pair down to maxspeed before anything reads it, so a driveLength well over maxspeed just means "full speed exactly this way", and one below it passes through untouched and becomes the actual speed.