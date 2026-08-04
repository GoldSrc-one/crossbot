# Nav System Documentation

## nav.cpp

### Nav namespace
Global mesh management:
- `LoadForMap(mapName)` - loads nav file for map
- `Unload()` - releases mesh
- `IsLoaded()` - checks if mesh loaded
- `Get()` - returns pointer to NavMesh

## nav_mesh.h

### NavMesh struct
`planAreas` and `planConns` are the planner's view of the same graph, built once when the file is loaded. The area gates in particular are what the funnel routes through, and rebuilding them per repath would repeat the same plane intersections every time.

### Area struct
- `planes` - inward-facing bound planes (for prism areas, includes footprint vertical planes + z-slab)
- `doorActivation`, `doorActivatorPos`, `doorWaitTime` - door interaction data

### Connection struct
- `gateA`, `gateB` - funnel gates on each side, baked by navgen with player radius already applied (inset at both ends and pushed that far into the owning area). Zero-length gate means opening was too narrow to inset; midpoint is the only way through. `gateValid` says which sides carry one; teleport carries neither.

## nav_path.cpp

### FindPath
Converts NavMesh to A* graph format and runs NavAStar::FindPath.

### BuildWaypointPath
Delegates to NavPlan::BuildWaypointPlan for string-pulled waypoints.

## nav_query.cpp

### FindAreaForPoint
Two-phase lookup:
1. AABB test with 4-unit epsilon, then exact plane test (2-unit epsilon)
2. If no containing area, fallback to nearest centroid within AABB
3. If still none, global nearest centroid

### FindAreaById
Direct index lookup with fallback to linear search.

## nav_loader.cpp

### LoadNavFile
Loads and validates .nav file:
- Checks magic ("CBNV") and version (9)
- Reads area records, connection records, area-connection indices, area planes
- Verifies BSP checksum against live map
- Converts to runtime NavMesh with PlanArea/PlanConnection views
- Builds PlanArea boundPlanes from Area planes
- Builds PlanConnection from Connection (including gates, launch params)

## astar_graph.cpp

### NavAStar::FindPath
Standard A* on the area graph, **costed in seconds**:
- Nodes = areas (centroid positions)
- Edge cost = `(approachDist + crossDist) / kNavRunSpeed + conn.costSeconds`
- Heuristic = `centroid-to-centroid distance / kNavRunSpeed`, in the same units so it stays admissible —
  nothing can beat running the straight line
- Respects one-way flags
- Reconstructs area path and connection path

`costSeconds` is **additive time**, not the distance multiplier `traverseCost` used to be. That is what lets a
door contribute the seconds it takes to open; a multiplier had nowhere to put "wait 4 seconds". Both `g` and `h`
scale by the same constant, so with `costSeconds` zero everywhere the routes are identical to the old model —
only door and air-move routing changed.

## funnel_graph.cpp

### NavFunnel::StringPull
Implements string-pulling (funnel algorithm) on 2D gates:
- Gates oriented by travel direction (left/right determined by cross product)
- Maintains funnel apex, left/right bounds
- When funnel constricts, emits corner at tight bound and restarts from there
- Returns pulled points with gate indices for corner attribution

Coincident consecutive points are collapsed as they are pushed, keeping the later one so the goal
always ends the list with `kNoGate`. The last iteration feeds the goal in as *both* funnel bounds, so
whenever the goal itself became the tight bound the algorithm emitted it as a corner and then pushed
it again — a zero-length step at the end of nearly every route.

It mattered more than a stray waypoint. `BuildPathProfile` skips a corner whose adjacent segment is
shorter than `kEpsLen`, and skips it before assigning a speed cap, so the duplicate silently disabled
arc fitting *and* speed limiting at both corners touching it. In `test5_jump` that was the 104-degree
turn immediately before the first launch: no arc, and the profile claiming 320 through it. With the
duplicate gone it fits an 11.7-unit arc capped at 191, and the run-up leg is long enough to get back
to the 300 the launch wants.

### NavFunnel::MakeGate
Orders gate endpoints so left/right matches travel direction.

## path_profile.cpp

Fits circular arcs to the waypoint polyline and computes a speed profile the engine can actually
hold. Runs at plan time, so navgen and the runtime produce identical geometry and the result is
inspectable offline via `--dump-path`.

### Movement physics it derives from

Each frame on ground the engine runs `PM_Friction` then `PM_WalkMove`. Acceleration scales with
*wishspeed*, not current speed, so at full drive it is the constant `A = sv_accelerate * maxspeed`
(3200 at defaults). Friction drag is `D(v) = sv_friction * max(v, sv_stopspeed)`.

Turning at constant speed requires tilting the wish direction forward enough to cancel drag, leaving

```
a_lat(v) = sqrt(A^2 - (f*v)^2)          r = v^2 / a_lat(v)
```

which is `r = 34.9` at 320, not the drag-free `v^2/A = 32`. `SpeedForRadius` inverts this.

Braking is the coasting case: `dv/dx = -f` exactly while `v > stopspeed`, so braking distance is
linear (320 to 0 costs 67.5 units). `BrakeBack` handles the sub-stopspeed branch separately.

### Corner fitting
For deflection `theta` at a corner with budget `C`:

```
R_geo = C*cos(theta/2) / (1 - cos(theta/2))     apex stays within C of the vertex
R_len = min(availIn, availOut) / tan(theta/2)   neighbouring arcs must not overlap
R     = min(R_geo, R_len)
```

Three fit passes, because shrinking one arc frees length for its neighbours. When trace functions are
available the arc is hull-traced entry to apex to exit and shrunk up to 3 times on a hit — this is
what makes the clearance number a prior rather than an assumption.

Below `v_pivot = sv_accelerate * maxspeed * dt` (53.3, the one-frame velocity quantum) an arc is
meaningless — the fitted radius is under a unit. Those corners emit no arc; the two lines meet at the
vertex and the servo pivots through it. Arcs only become geometrically visible above about 200.

Corners at an airmove launch itself and at a landing emit no arc: the launch has to be crossed on the
launch heading, and a landing is entered with whatever velocity the flight left. The corner *into* the
run-up node does get one — its outward tangent is clamped to half the run-up leg, which keeps the last
stretch into the launch straight while still letting the bot round the turn before it.

### Speed profile
A backward pass propagates each node cap through `BrakeBack`, with the final waypoint pinned to 0 and
airmove launches capped at `launchSpeed`. Entry speeds are additionally clamped by the cap of the node
the segment starts from. `vEntry`/`vExit` are ceilings, not predictions: a segment leaving an arc may
report 320 while the bot is still accelerating from the arc exit speed. A forward pass integrates the
real speed against those ceilings to produce `totalTime`, which replaces the old fabricated ETA, and
records what it actually reached at the end of each segment in `vReach`.

`ProfileReachAtWaypoint(profile, i)` reads that back for the segment ending at waypoint `i`.
`BuildWaypointPlan` uses it to re-validate air moves at the speed the route can really deliver rather
than the one the sampler assumed.

`NodeMaxSpeed` applies `kDuckingMultiplier` for `requiresCrouch`, except at a launch waypoint, where
the flag means duck in flight and the approach is a full-speed run.

### Speed within a segment
`SegSpeedAt(seg, local)` is the single definition of "how fast should the bot be `local` units into
this segment", and both the forward pass and `SampleProfile` go through it. It is
`min(max(vEntry, vExit), BrakeBack(vExit, sLen - local))`: hold the ceiling until the physical brake
curve for the segment's exit speed drops below it, then follow that curve down.

It replaces a straight interpolation between `vEntry` and `vExit`, which was wrong in a way that cost
both accuracy and time. Braking is `dv/dx = -f` regardless of segment length — 320 to 0 takes 67.5
units, whether the segment is 70 units or 700. Lerping instead spread the deceleration across the
whole segment, so on a 196-unit approach to a 191-cap corner the bot was told to run at 238 halfway
along rather than hold 320 and brake in the last 32 units. Measured ground speeds tracked the lerp
almost exactly, which is why the bot never reached `maxSpeed` on a straight.

The forward pass suffered the same error compounded: as the lerped limit fell toward zero at the end
of a decelerating segment, `v` hit its 1 u/s floor and each step advanced only `v * dt`, so the tail of
a segment ending at rest cost roughly `(sLen / maxSpeed) * ln(maxSpeed)` seconds. That inflated
`totalTime` by about 1.5x across the test maps; it now lands within 10% of measured arrival.

The ceiling is `max(vEntry, vExit)` rather than `vEntry` so an accelerating segment is not capped
below its own exit speed. On an arc the two are equal and the result is the constant `vCap`.

### Sampling
`SampleProfile(s)` returns point, tangent, speed, clearance and waypoint index at arclength `s`.
Speed comes from `SegSpeedAt`, so the servo is commanded to hold speed and brake late rather than to
coast down gently; `PlanProfile` carries `friction`/`stopSpeed` so the sampler can evaluate the brake
curve without the physics struct.
`ProjectOntoProfile(pos, sMin, sMax)` finds the nearest arclength within a window and the unsigned
lateral distance to it.

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

**Velocity servo** - inverts one engine frame to hit the target velocity `V*`:

```
v_af      = velocity after this frame's friction   (guard |v| < 0.1, as PM_Friction does)
need      = V* - v_af
wishdir   = normalize(need)
wishspeed = max(|need| / (sv_accelerate * dt * friction), along + |need|)
```

The second term matters: `PM_Accelerate` caps its delta at `wishspeed - v.wishdir`, so the rate-based
form alone commands zero acceleration when already cruising at the target. One formula covers
acceleration, braking, counter-strafing and arc tracking, saturating when a frame cannot deliver.

**Abort** - `needRepath` once lateral exceeds `rejoinAbort` (96). Previously nothing detected being
off-path; the corridor pull fought forever until the 10s stall timer fired.

## pm_sim.cpp

### PmSim::PmSimulator
Frame-by-frame HL player movement simulation:
- `Begin(origin, yaw, speed, jump)` - initializes state
- `Step()` - advances one frame, returns false if wedged
- Implements: gravity, jump impulse, air acceleration, clip hull movement (FlyMove), ground categorization
- Bunny hop cap: 1.7x maxSpeed
- Air accel wish speed cap: 30
- Ground normal threshold: 0.7
- Ground trace distance: 2

### Physics constants
- Jump impulse: 268.328
- Max safe fall speed: 580 (no damage)
- Fatal fall speed: 1024
- Damage per fall speed unit: 100/(1024-580)

## waypoint_plan.cpp

### NavPlan::BuildWaypointPlan
Builds waypoint sequence from area/connection path:

**Core data structures:**
- `PendingGate` - funnel gate with inside area ID, target area ID, crouch flag
- `PlanTraceContext` - duck/stand trace functions, physics, airmove dump flag

**Gate handling:**
- `BuildAreaGates` - builds inset gates per bound plane (wall planes only)
- `BuildConnectionGates` - trims shared face to actual overlap; air moves skip (use sampled gates)
- Gates pushed per connection: leaving area first, then entering area

**Funnel operations:**
- `PeekFunnel` - dry-run StringPull to see where route crosses gates
- `FlushFunnelTo` - runs StringPull, emits corners as waypoints with crouch/targetArea from gate

**Connection types:**

**Teleport:**
- Flushes funnel to launchPos
- Adds trigger waypoint (requiresTeleport=true, teleportDest=landingPos)
- Adds destination waypoint
- Anchor moves to landingPos

**Air Move:**
- Pushes arrival gate (entry side only)
- If traces available: re-simulates from funnel's preferred launch point
  - Computes funnel crossing on launch gate (exit side)
  - Interpolates yaw/speed from gateAYaw/gateASpeed
  - Re-simulates with PmSim; if lands in dest area, uses simulated params
  - Falls back to baked params on miss
- Pops arrival gate
- Flushes funnel to launch point
- Adds launch waypoint (airmoveLaunch=true, requiresJump, launchSpeed/Yaw, jumpBackDist, requiresCrouch, targetArea)
- Adds landing waypoint (targetArea)
- Anchor moves to landing point

**Walk (including doors):**
- Door: adds button/trigger waypoint if needed, then door waypoint (requiresDoor, doorWaitTime)
- Non-door: pushes both gates to funnel, continues

**Goal:**
- Flushes funnel to goal point
- Adds final waypoint

### Gate/Geometry Utilities

**GateFloorZ** - interpolates floor Z along gate from endpoint heights

**NearestPointOnGate** - finds closest point on gate segment to funnel polyline

**CrossingOnGate** - finds where funnel polyline crosses gate plane, clamps to gate span

**GateFraction** - parametric position along stored gate (0=left, 1=right)

**LerpYaw** - shortest angular interpolation (handles +/-180 wrap)

**PointInArea** - tests point against boundPlanes with epsilon

**InAreaColumn** - tests point against wall planes only (ignores floor/ceiling), for column test

**FloorPlaneOf** - finds bound plane with highest normal.z (most upward)

**AreaFloorZ** - computes floor height at (x,y) from floor plane

**EdgeOfPlane** - extracts wall plane as 2D line segment, clips by other walls in area

**ClipLineByWalls** - clips line segment by other wall planes in area

**GateFromEdge** - builds gate from edge line: insets ends, pushes inward by kPlanClearance (16), clamps if too narrow

**BuildAreaGates** - builds edgeGates for all bound planes

**BuildConnectionGates** - for walk connections: finds shared face, builds gates on both sides; falls back to pre-built edgeGates
- A shared wall between two areas runs the full length of the boundary, but only the stretch where the two floors meet within a step is actually crossable -- the rest is a cliff face. Narrows [lo, hi] to the longest run that a walker can step across, and reports whether anything survived.
- Push each end outwards to the exact crossing, so a gate is not quantized away by the spacing.
- It is the same wall from either side, but only one side may have bound planes that close it off: a wedge under a ramp is capped by its sloped ceiling, so its own edge line runs to infinity and EdgeOfPlane rejects it. Take the line from whichever neighbour bounds it.

**ConnectionCrossing** - midpoint of gates (or launchPos for non-walk)

## waypoint_plan.h

### PlanFloorFn
Samples the walkable floor of one area at a 2D point. Returns false where the area has no floor there. PlanArea only models a single floor plane, so callers that know the real geometry (navgen) supply this to keep gates off the parts of a shared wall where the two floors are a cliff apart.

### PlanArea
- `boundPlanes` - inward-facing planes (for prism: footprint walls + z-slab)
- `edgeGates` - one per bound plane, inset/pushed gates
- `door`, `breakable`, `doorActivation`, `doorActivatorPos`, `doorWaitTime` - a door area makes the plan stop at
  it; BUTTON/TRIGGER activation first emits a waypoint at the activator

### PlanConnection
- `planeA`, `planeB` - bound plane indices for connection (baking only)
- `gateA`, `gateB` - shared face gates (built by BuildConnectionGates)
- `gateAYaw[2]`, `gateASpeed[2]` - launch params at gateA ends

### PlanWaypoint
- `point` - position at player height (duck origin + 18 for stand)
- Flags: requiresElevatorRide, requiresCrouch, requiresDoor, requiresBreak, requiresButtonUse, requiresTriggerTouch, requiresTeleport, teleportDest, airmoveLaunch, requiresJump, launchSpeed, launchYaw, jumpBackDist, launchSimulated, launchSimFailed, launchSimFrom/To/Aim, targetAreaId
- `requiresBreak` is recorded and plumbed but the bot does not act on it yet — it never shoots the breakable

### Constants
- `kPlanClearance = 16` - player hull radius; gates inset by this at ends and pushed this far off connection plane
- `kPlanClampedClearance = 6.63` - corner-cut budget at a clamped gate, `16*(sqrt(2)-1)`: the diagonal
  slack at a right-angle corner of a minimum-width corridor
- `kPlanStandHeight = 18` - duck hull origin to stand origin

### Corner clearance
`PlanWaypoint::clearance` is how far an arc may deviate from the vertex. It is derived in
`GateClearance` from the gate the corner came from, *not* stored on `PlanGate` — gates round-trip
through `MergedConnection` and the `.nav` file, both of which copy field by field, so anything added
to `PlanGate` is silently dropped on load.

Gate width is not the budget. `GateFromEdge` already insets endpoints `kPlanClearance` from the wall,
so a surviving endpoint is 16 units off geometry by construction and the remaining width is *extra*
freedom: `clearance = min(16 + halfWidth, 32)`. A clamped gate collapsed to a point, so it gets
`kPlanClampedClearance`. Over-estimating here is safe — the hull trace in `BuildPathProfile` shrinks
any arc that does not actually fit — whereas under-estimating stops the bot at every tight gate.

### PlanTraceContext
- `duckTrace`, `standTrace` - trace functions for re-simulation
- `physics` - movement physics
- `dumpAirmoves` - debug output flag
- `collectRunupTraces` - vestigial. The steered-approach simulator it fed no longer exists, so nothing
  reads it and nothing fills `PlanStats::runupTraces`.

### PlanStats
- `airmovesSimulated`, `airmovesFromStored`, `airmovesRespeeded`, `airmovesUnplannable`, `corners`,
  `gatesClamped`
- `airmovesUnplannable` - launches the simulation could not make, each of which truncated the route.
- `airmovesRespeeded` - launches the profile could not deliver the design speed to, which were
  re-simulated at the speed it could and still landed.
- `runupTraces` - vestigial, never populated. `--dump-svg` still accepts them and draws nothing.

### JumpLandsIn (waypoint_plan.cpp)
`JumpLandsIn` re-simulates one air move at plan time and accepts it only if the mover ends up inside
the destination area. It begins on the launch point itself, at a given speed and the launch heading,
and requests the jump on the first frame.

It used to start some distance back along the launch heading, on the theory that simulating the
run-up validated the approach. It did not: a straight line drawn backwards from the launch is not
where the route comes from, so it ran through whatever geometry happened to be behind the launch and
the mover wedged or slid off a wall before ever reaching the launch plane. At the last value of 64
this failed on **every air move in every test map**, truncating all fifteen routes at the first jump.
The approach is now checked by the profile instead, which knows the real route — see the run-up node
below.

### The run-up node
A waypoint inserted `launchSpeed * kPlanAirmoveRunupTime` (0.1 s of travel, floored at
`kPlanAirmoveRunupMin` = 8 units) back along the launch heading, so the route's last leg into a launch
is straight and aligned. Roughly six frames of approach at a full-speed launch, proportionally less
for the slow drops, where a fixed distance used to leave the node stranded far off the natural route —
`test15_staircase` had two plan segments leaving the mesh entirely for that reason.

It is load-bearing for a second reason that has nothing to do with alignment. `Bot_MoveNav` hands
control to `RunAirmoveApproach` when the tracker's `wpIndex` reaches the launch waypoint, and
`wpIndex` comes from whichever profile segment the bot projects onto. `BuildPathProfile` drops
segments shorter than `kEpsLen`, and the funnel's last corner lands on the launch point exactly, so
without this node the segment ending at the launch is zero-length, gets dropped, and no segment ever
carries the launch's index. The bot then walks straight through the launch point and never jumps.

### Re-speeding a launch
The stored `exitSpeed` is what the air-move sampler launched at, not what the route can deliver. A
corner 30 units before the launch caps the bot well under it, and nothing used to notice: the plan
asserted the design speed and the jump was validated at a speed the bot would never reach.

`BuildWaypointPlan` therefore builds a probe `PlanProfile` over its own finished waypoint list before
returning. `ProfileReachAtWaypoint` reads the forward pass's achieved speed on the segment ending at
each launch — the arc fit, the backward braking pass and the acceleration integral all folded into one
number — and if it falls short of the design speed by more than `kPlanAirmoveRespeedEps`, the jump is
re-simulated at that speed. The landing waypoint moves to wherever it actually lands. If it no longer
lands in the destination area the route truncates at the run-up node, same as any other unplannable
launch. The yaw needs no adjustment: the last leg is straight along the launch heading by construction.

Only one iteration runs. The node's own position depends on the design speed, and the funnel was
already string-pulled through it, so re-placing it from the re-speeded result would invalidate the
geometry the result was computed on.

Two corners of the profile are what make the number meaningful, and both had to change for it:
- the corner *into* the run-up node is no longer suppressed, so it gets a real arc and a real speed
  cap. Its outward tangent is clamped to half the run-up leg so the straight approach survives.
- `NodeMaxSpeed` no longer applies `kDuckingMultiplier` at a launch waypoint. `requiresCrouch` there
  means duck *in flight*; the approach is a full-speed run. Capping it at 107 made every crouch jump
  arrive at a third of its launch speed — `test6_crouchjump` and `test40_walljump` both failed
  re-speeding outright until this was fixed.

- `BuildAreaGates(PlanArea&)` - fills edgeGates from boundPlanes
- `BuildConnectionGates(areas, conns)` - builds shared face gates
- `ConnectionCrossing(areas, conn)` - crossing point for A* cost
- `BuildWaypointPlan(...)` - main entry point

## nav_format.h

### Nav File Format (version 10)
- Magic: "CBNV"
- Header: magic, version, mapName[64], bspChecksum, flags, counts
- AreaRecord: id, mins[3], maxs[3], centroid[3], areaFlags, connOffset, connCount, planeOffset, planeCount, doorActivation, doorActivatorPos[3], doorWaitTime
- ConnectionRecord: areaA, areaB, connType, connFlags, gateValid, pad, costSeconds, gateA[2][3], gateB[2][3], gateAYaw[2], gateASpeed[2], exitSpeed, exitAngle, jumpBackDist, fallDamage, flightTime, launchPos[3], landingPos[3]

`nav_loader.cpp` rejects a version mismatch outright, so bumping forces regeneration rather than silently
loading a stale mesh. Area planes are the exported `boundPlanes`, which for a prism area is its footprint hull
plus a z-slab rather than its true faces.

### Enums
- AreaFlags: CROUCH(1), LADDER_TOP(2), WATER(4), DOOR(128), BREAKABLE(256)
- ConnType: WALK(0), TELEPORT(1), AIRMOVE(2)
- ConnFlags: ONE_WAY(1), NEEDS_CROUCH(2), JUMP(4) - jump means left ground under impulse, not derivable from jumpBackDist
- DoorActivation: TOUCH(0), BUTTON(1), TRIGGER(2)
- GateValidFlags: VALID_A(1), VALID_B(2), CLAMPED_A(4), CLAMPED_B(8)

## pm_sim.h

### PmPhysics
- `accelerate`, `groundFriction`, `stopSpeed` - sv_accelerate, sv_friction and sv_stopspeed. Only the ground phase uses them; air movement is governed by airAccelerate and its own 30-unit wish cap.
- `friction` - pmove->friction, the per-entity multiplier. Not sv_friction.

### PmSimulator
- `SetWish(dir, speed)` - overrides the wish direction and speed used from the next Step onward, which is what lets a controller steer the mover frame by frame instead of holding the heading Begin was given. dir need not be normalised; only its horizontal part is used.
- `SetVelocity(velocity)` - seeds the velocity directly rather than deriving it from a yaw and a speed, for starting a simulation from a state that was measured rather than commanded.

## pm_sim.cpp

### Friction
PM_Friction. Only the horizontal speed matters here because the caller zeroes velocity[2] while on the ground, so the drop is applied to the ground speed alone.

### Accelerate
PM_Accelerate. Unlike the air variant there is no 30-unit cap on the wish speed, so commanding a lower speed really does hold the mover at that speed once friction balances it.

### FlyMove
Reaching here with i == numplanes means the loop above found no single plane the velocity could slide along, which is PM_FlyMove's "go along the crease" case. The early break out of that loop already stored the sliding velocity, so it must not fall in here -- testing i != numplanes instead zeroes the velocity of every successful slide.

### PM_PlayerMove
PM_PlayerMove rubs friction off before the move and then hands the ground and the air to different accelerators. The air one caps its wish speed at 30, so it can only ever nudge a trajectory; the ground one does not, which is what makes a run-up controllable at all.