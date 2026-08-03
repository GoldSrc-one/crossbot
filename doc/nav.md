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

### NavFunnel::MakeGate
Orders gate endpoints so left/right matches travel direction.

## path_follower.cpp

### NavFollow::Steer
Computes steering decision for current waypoint:

**Advance logic:**
- Final waypoint: reached within `finalRadius` (24)
- Intermediate: reached within `cornerRadius` (12) OR past segment endpoint

**Aim point:**
- Base: current waypoint
- With lookahead: projects ahead along segment from previous waypoint

**Corner pre-steer (cuts corner early):**
- Only for non-final, non-airmove waypoints
- Blend factor based on speed/maxSpeed, capped at `cornerCut` (16)
- Activates within `cornerCut + cornerRadius` of corner
- Displaces aim along outgoing segment

**Cross-track correction (pulls to planned line):**
- Dead band: `pathFreeplay` (0) to `pathCorridor` (16)
- Inside freeplay: no correction
- At corridor edge: full pull to line
- Blends aim direction with pull direction

**Corner brake (velocity-based correction):**
- Activates within `brakeRadius` (16) of waypoint
- Computes velocity error = desiredDir * max(speed, maxSpeed) - currentVel
- Blends aim with error direction weighted by error/maxSpeed
- Exempt for air-move launches (must hit exact sampled heading/speed)

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

**ConnectionCrossing** - midpoint of gates (or launchPos for non-walk)

## waypoint_plan.h

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
- `kPlanStandHeight = 18` - duck hull origin to stand origin

### PlanTraceContext
- `duckTrace`, `standTrace` - trace functions for re-simulation
- `physics` - movement physics
- `dumpAirmoves` - debug output flag
- `collectRunupTraces` - keeps the simulated run-up paths in PlanStats so they can be drawn. Off by default: the bot replans mid-route and has no use for them.

### PlanStats
- `airmovesSimulated`, `airmovesFromStored`, `airmovesUnplannable`, `corners`, `gatesClamped`
- `airmovesUnplannable` - launches the simulation could not make, each of which truncated the route.
- `runupTraces` - one simulated approach per air move, when the trace context asks for them.

### Run-up Length Tuning (waypoint_plan.cpp)
- `kPlanAirmoveRunup = 48` - How far back along the route the steered approach is simulated from.
  - Measured over the six launches on test22_gap, test41_skydrop, test5_jump, test39_skybox and test38_islands at 16, 32, 48 and 64. Sixteen is enough whenever the bot arrives already moving, and buys about 231 u/s from a standstill -- that figure recurs on four independent launches, so it is the acceleration limit rather than anything about the maps. Launches wanting more than that need the longer run-up: test22_gap asks for 297 and only lands from rest at 48.
  - Longer is not simply better. Walking back along the route diverges from the launch line at the sine of the angle between them, and the controller does not fully recover the offset, so lateral error at the jump grows with the distance -- about 0.54 units per unit of run-up. test38_islands 5->4 lands at 16, 32 and 48 and misses at 64 for that reason alone. Forty-eight is the longest that still lands everything it can.

### TruncatePolylineAt
Truncates a polyline at the point nearest to cut, so what remains is the ground approach up to the launch and not the flight past it.

### RunupTail
Walks back from the end of a polyline until backDist has been covered, or the polyline runs out, and returns the tail from there to the end. Reports how far back it actually got, which is less than asked for when the route into the launch is shorter than that.

### SimulateApproach
One attempt at reaching a launch through a steered ground approach.

### AirmoveSimResult
- `launchState` - State at the moment the jump was taken, against what the launch wanted.
- `runupTrace` - Where the simulated mover actually went, for drawing over the route.

### ArcLengthToEnd
Arc length from the point on path nearest pos to the end of path. This is the distance still to run, measured along the route rather than straight at the launch, so a route that turns on the way in reports honestly.

### SimulateFullAirmove
Runs the whole move: a steered ground approach down the route the bot actually takes, then the flight. The controller is the point of it. Rather than assuming the bot shows up on the launch line at the launch speed -- which is what the sampler assumes and nothing had checked -- it starts back along the path at whatever velocity the caller specifies and steers toward the launch velocity, braking if it is coming in too hot. What comes out is an answer to the question that matters: can a mover obeying HL ground physics reach this launch in the state the jump needs, from where the route brings it in.

### Z Height Handling
Only x and y come from the polyline. Its z cannot be trusted: the funnel line runs from the bot's own origin, which is a standing player height, through gate points that are in nav space 18 units lower, so interpolating along it climbs or drops for no physical reason. The run-up happens on the source area's floor, so that is where its height comes from.

### Distance Measurement
Distance still to run measured against the launch plane, which is what decides the jump. The along-path figure is only used to notice the ground running out.

### Velocity Steering
Steer down the velocity error toward the launch velocity, flat out, for the whole run-up. Commanding along the error is what allows braking at all: PM_Accelerate only ever adds along the wish direction, so asking for a lower speed merely lets friction do the work, whereas aiming against the current velocity at full drive actually sheds it.

The target is the launch velocity from the first frame rather than something blended out of the path heading. Blending the two vectors reads well until the route comes in against the launch -- on test5_jump the approach runs south-west and the jump goes east, so the interpolation passes through zero and the mover obediently brakes to a standstill halfway along its run-up.

### Run-up Length Selection
Tries the run-up lengths shortest first, and each length from rest as well as at speed. The shortest that lands is the interesting answer: it is the least distance the runtime would have to hand the controller, and starting from rest is the harder of the two cases because the mover has to build the speed as well as aim it.

From rest first, because it is the harder of the two: the mover has to build the speed as well as aim it. A launch that survives that needs nothing of the bot on the way in.

### Ground Approach
The ground the bot covers on the way in, which is the funnel's own line cut off at the launch -- not a straight extension backwards along the launch heading. Those are different wherever the route turns into the launch, and it is the route the controller has to work with.
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