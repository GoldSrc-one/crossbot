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

## bot_avoid.cpp

Reactive obstacle handling, layered between the follower's steering decision and the engine call. The
nav mesh only knows world geometry and the brush entities navgen carves, so anything else solid — a
`cycler`, a monster, another player, a door that has swung out of its own doorway — is invisible to
the planner: it routes straight through and the follower drives into it and stalls. `BotAvoid::Update`
traces forward along the planned path each frame and either crouch-jumps onto the obstacle or ducks
under it.

It handles exactly those two answers. An obstacle that is neither too low to jump nor high enough to
crawl under produces no decision and the bot still stalls against it; steering around is not
implemented.

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

### The two maneuvers

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

`crossbot_nav_avoid 0` disables the layer, which is how to A/B a suspected regression without
rebuilding.

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
- `avoid=` with `hit=` — the last obstacle decision and the classname it was made against, plus
  `duckLeft` on the crawl latch and the `gap`/`top` the decision was made from. `avoid=none hit=-`
  on a bot stalled against something means the trace never qualified a hit, which is a different bug
  from a mistimed maneuver; each new decision also prints its own line to the server console with
  distance and speed, which is what makes the timing tunable from one run.

## bot_move.h

### Bot_DecomposeMove
Splits a world-space direction into the forward/side speeds pfnRunPlayerMove takes, which the engine reads along the view axes -- so this is what lets the bot move somewhere other than where it is looking. viewYawDeg must be the yaw passed to pfnRunPlayerMove in that same call plus punchangle, because PM_CheckParamters builds the movement basis from exactly that.

PM_CheckParamters clamps the pair down to maxspeed before anything reads it, so a driveLength well over maxspeed just means "full speed exactly this way", and one below it passes through untouched and becomes the actual speed.