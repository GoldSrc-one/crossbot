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
Standard A* on area graph:
- Nodes = areas (centroid positions)
- Edges = connections with traverse cost = approach + cross cost
- Heuristic = centroid-to-centroid distance
- Respects one-way flags
- Reconstructs area path and connection path

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

### PlanConnection
- `planeA`, `planeB` - bound plane indices for connection (baking only)
- `gateA`, `gateB` - shared face gates (built by BuildConnectionGates)
- `gateAYaw[2]`, `gateASpeed[2]` - launch params at gateA ends

### PlanWaypoint
- `point` - position at player height (duck origin + 18 for stand)
- Flags: requiresElevatorRide, requiresCrouch, requiresDoor, requiresButtonUse, requiresTriggerTouch, requiresTeleport, teleportDest, airmoveLaunch, requiresJump, launchSpeed, launchYaw, jumpBackDist, launchSimulated, launchSimFailed, launchSimFrom/To/Aim, targetAreaId

### Constants
- `kPlanClearance = 16` - player hull radius; gates inset by this at ends and pushed this far off connection plane
- `kPlanStandHeight = 18` - duck hull origin to stand origin

### PlanTraceContext
- `duckTrace`, `standTrace` - trace functions for re-simulation
- `physics` - movement physics
- `dumpAirmoves` - debug output flag

### PlanStats
- `airmovesSimulated`, `airmovesFromStored`, `corners`, `gatesClamped`

### Functions
- `BuildAreaGates(PlanArea&)` - fills edgeGates from boundPlanes
- `BuildConnectionGates(areas, conns)` - builds shared face gates
- `ConnectionCrossing(areas, conn)` - crossing point for A* cost
- `BuildWaypointPlan(...)` - main entry point

## nav_format.h

### Nav File Format (version 9)
- Magic: "CBNV"
- Header: magic, version, mapName[64], bspChecksum, flags, counts
- AreaRecord: id, mins[3], maxs[3], centroid[3], areaFlags, connOffset, connCount, planeOffset, planeCount, doorActivation, doorActivatorPos[3], doorWaitTime
- ConnectionRecord: areaA, areaB, connType, connFlags, gateValid, pad, traverseCost, gateA[2][3], gateB[2][3], gateAYaw[2], gateASpeed[2], exitSpeed, exitAngle, jumpBackDist, fallDamage, flightTime, launchPos[3], landingPos[3]

### Enums
- AreaFlags: CROUCH(1), LADDER_TOP(2), WATER(4), DOOR(128)
- ConnType: WALK(0), TELEPORT(1), AIRMOVE(2)
- ConnFlags: ONE_WAY(1), NEEDS_CROUCH(2), JUMP(4) - jump means left ground under impulse, not derivable from jumpBackDist
- DoorActivation: TOUCH(0), BUTTON(1), TRIGGER(2)
- GateValidFlags: VALID_A(1), VALID_B(2), CLAMPED_A(4), CLAMPED_B(8)