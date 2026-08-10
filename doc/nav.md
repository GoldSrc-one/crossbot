# Nav System Documentation

## nav.cpp

### Nav namespace
Global mesh management:
- `LoadForMap(mapName)` - loads nav file for map
- `Unload()` - releases mesh
- `IsLoaded()` - checks if mesh loaded
- `Get()` - returns pointer to NavMesh

### Runtime connection penalties
`PenalizeConnection` / `RewardConnection` / `ConnectionPenalty` are the runtime's only way to write to
the mesh. They move `Connection::runtimePenalty`, which `BuildGraph` adds to `costSeconds`, so a
connection the bot failed on gets more expensive and the next plan routes around it if it can. Failure
adds `kConnFailPenalty` (5 s), success halves it and snaps to zero under 0.1.

Four properties are deliberate:

- **Not a ban.** The edge stays in the graph at a higher price, because it may be the only way into an
  area. 5 s is far below the `1.0e4f` soft-infinity `ConnectionCostSeconds` uses for lethal falls, so a
  penalized one-way drop still gets planned rather than stranding the bot.
- **Mesh-global and in-memory.** One table shared by every bot, cleared when `Unload` frees the mesh.
  `runtimePenalty` is not in the file format — it is learned state, and navgen would overwrite it.
- **Offline planning never sees it.** It defaults to 0 and only the runtime writes it, and navgen
  builds its own graph, so `--dump-path` output is unchanged by this whole mechanism. A route sweep
  that diffs after a change here must come out empty.
- **The reward only fires on edges still being planned.** Once a penalty is large enough that an
  alternative wins, nothing brings that edge back — there is no exploration term. Acceptable because an
  edge never planned is an edge never paid for.

Attribution runs launch waypoint → bot → mesh: `Cross` stamps `PlanWaypoint::connIndex` on the air-move
launch waypoint, `RunAirmoveApproach` latches it into `CBot::airmoveConnIndex`, and `RunAirmoveFlight`
spends it on landing. Only the air-move branch stamps it today; teleport and walk/door keep `-1` and
are the extension points when those grow failure detection.

## nav_mesh.h

### NavMesh struct
`planAreas` and `planConns` are the planner's view of the same graph, built once when the file is loaded. The area gates in particular are what the funnel routes through, and rebuilding them per repath would repeat the same plane intersections every time.

### Area struct
- `planes` - inward-facing bound planes (for prism areas, includes footprint vertical planes + z-slab)
- `doorActivation`, `doorActivatorPos`, `doorWaitTime` - door interaction data

### Connection struct
- `gateA`, `gateB` - funnel gates on each side, baked by navgen with player radius already applied (inset at both ends and pushed that far into the owning area). Zero-length gate means opening was too narrow to inset; midpoint is the only way through. `gateValid` says which sides carry one; teleport carries neither.

## nav_path.cpp

### PlanPath
The NavMesh-shaped wrapper over `NavPlan::PlanRoute` — builds the connection graph once and hands it
over. This is what `Repath` calls, and it is the only entry point into planning. There is no
graph-only `FindPath` and no separate `BuildWaypointPath`: searching and string-pulling are the same
pass now, so there is nothing meaningful to call in between.

**The goal is clamped into its area, and the plan ends at the clamped point.** `Repath` resolves the
requested point to an area and clamps it into that area's bounds; `PlanWalker::Finish` emits that
clamped point as the `goal` waypoint. When the requested point lies outside the mesh the plan
therefore ends legitimately short of it, and the bot reports "reached goal" from there. This is not a
planning failure and there is nothing to fix in the planner — but it looks exactly like one from
outside, because the caller compares against the point it asked for. `test21_teleport`'s `info_target`
sits 112 units past the last area, so `--dump-path` ends at `goal (448,336,32)` while the entity is at
(448,448,32), and `test_nav_goal.ps1` calls that a failure. Read the plan's last waypoint before
believing a "stopped short" report.

## plan_search.cpp

### NavPlan::PlanRoute
The planner. A* over the area graph where **each search node carries a `PlanWalker`** — the funnel
state, the waypoints emitted so far, and the running cost. Expanding an edge copies the parent's
walker and calls `Cross`, so the string pull, the air-move simulation and the door/teleport waypoints
all happen *during* the search rather than after it.

This exists because A* and the waypoint planner used to disagree. A* only knew a connection existed;
the planner decided whether the bot could actually be steered across it, and an air-move edge that
survived navgen's sampling could still fail at plan time — the funnel re-derives the launch point from
the actual route, and re-simulating from *that* point can miss the destination area. The old answer
was to truncate the plan and then re-run the whole search with that connection blocked, up to eight
times. Now there is only one funnel, the one the search built, so the disagreement cannot arise.

Three things follow from folding them together:

**Cost is real.** `g` is the string-pulled travel time to the node's frontier (`PlanWalker::Time()`,
pulled distance over `kNavRunSpeed`) plus each connection's `costSeconds`. The old model summed
centroid → crossing → centroid per hop, which charged the path for visiting every centroid and so
systematically overpriced routes through many small areas. Measured effect: on test50_stairs the
planner now walks up through two areas where it used to crouch-jump, because the jump was never
actually cheaper — the old cost model just could not see the walk.

**Recovery is free.** An air move whose launch cannot be simulated makes `Cross` return false, so the
edge simply does not relax and the search continues through other edges. Nothing is truncated and
nothing is retried. `PlannedRoute::airmovesRejected` counts these.

**`h` is sharper.** The heuristic is the straight line from the node's funnel frontier to the goal
*point*, not centroid to centroid, which is why the search expands so few areas — 165 of crossfire's
503. `g` and `h` share the same reference point (the last pending gate's midpoint, or the anchor when
the funnel is empty), so they are consistent with each other. They are not strictly admissible: the
real route need not cross at the midpoint, so `g + h` can overprice a node whose true crossing is off
to one side, and the search can return a slightly suboptimal route. It cannot fail to find one.

Two properties worth knowing before changing it:

- **Nodes are keyed by area, not by `(area, incoming connection)`.** Two routes into one area have
  different funnels; the cheaper one wins and the other funnel is discarded. This is the standard
  Theta*/TA* approximation. Keying by the pair would be exact but multiplies the node count by the
  average degree.
- **Reopening is allowed.** Funnel-derived `g` is path-dependent, so the usual closed-set argument
  does not hold; a node that receives a strictly better `g` is un-closed and re-pushed. `g` is
  clamped to never fall below its parent's, which is what makes the parent chain acyclic — the
  reconstruction loop still carries a hop guard in case that reasoning is ever broken.

The goal node's walker **is** the finished plan: reconstruction calls `Finish` on it and takes its
waypoints. There is no replay pass, so the plan cannot drift from what the search costed.

### Cost of planning in the search
Every air-move edge the search relaxes runs a full `PmSim` re-simulation, so the price is worth
knowing. On crossfire (503 areas, 773 walk and 1377 air-move connections) one `PlanRoute` expands 165
areas and runs 243 simulations, which navgen measures at ~0.1s against BSP-hull traces. `--dump-path`
prints both numbers on the `debug search:` line, and `Repath` prints them with a wall-clock time.

One early-out does most of the work and is worth not deleting by accident:

```cpp
if (state[current].g + kPlanCostEpsilon >= state[neighbor].g)
    continue;
```

Because `g` is clamped to never fall below its parent's, a parent that already costs more than the
neighbour's best cannot improve it no matter what the edge costs — so the edge can be skipped *before*
`Cross` runs, which is what would have paid for the simulation. It cut crossfire from 402 simulations
to 243 and the test maps from 207 to 144, with byte-identical routes on all 52 maps. It is a pure
early-out, not an approximation.

Validating on *pop* instead of on every relaxation would cut simulations further, to at most one per
expanded node, but it needs dead-edge tracking, node resetting and re-opening already closed
neighbours to avoid missing routes. Eager validation is correct by construction; if the runtime cost
ever justifies that trade, these are the numbers to beat.

### What the planner guarantees, and what it does not
The search only emits an air move whose launch it re-simulated from the funnel's own launch point and
watched land inside the destination area. So every maneuver in a returned plan is **simulable** — a
`PmSim` run from that point at that speed reaches that area.

A ladder gets no such check. Its mount and dismount were both established by navgen against the real
hulls, so the planner takes the connection at face value and only decides which end it entered from.

It does not follow that the bot can *execute* it. `PmSim` starts at the exact launch point, at the
exact yaw, already at the launch speed; the follower has to arrive there having tracked an arc chain,
and arrives with whatever speed and heading it actually has. `NavFollow::Track` derates speed when it
is off the chain, so a bot rejoining late reaches the launch slower than planned and undershoots. The
plan is a proof that the route exists, not a proof that the steering will hit it.

This matters when reading test results, because the two failures look nothing alike. A planning
failure shows up in `--dump-path` — no route, a rejected connection, a plan ending somewhere
unexpected. An execution failure has a correct plan and a bot that does not follow it, and the only
place it shows up is the runtime. Changing the cost model can convert the second into a visible
regression without any planning bug: a route the follower handles badly becomes cheaper, the search
starts choosing it, and end-to-end pass rates drop while every plan remains valid.

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

## astar_graph.h

Header only — there is no `astar_graph.cpp` any more. The search itself lives in `plan_search.cpp`;
what remains here is the two things both it and the graph builders need: `kNavRunSpeed`, and the
`Connection` view of an edge (endpoints, one-way flag, `costSeconds`, crossing point).

`costSeconds` is **additive time**, not the distance multiplier `traverseCost` used to be. That is what lets a
door contribute the seconds it takes to open; a multiplier had nowhere to put "wait 4 seconds". It already
includes an air move's `flightTime` and a door area's `openSeconds` (see `ConnectionCostSeconds` in
`area_merger.cpp`), so `PlanWalker` must not add either again — the search adds `costSeconds` once per
edge and the walker only measures string-pulled distance.

Being seconds is also what lets `runtimePenalty` be added straight on top in `BuildGraph` (see
`nav.cpp`). Anything else fed into the cost model has to be in seconds too, or it will quietly outrank
everything.

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

`Suppressed` is defined as `IsGap(i) || IsGap(i - 1)`, so the same rule covers teleports and ladder
mounts. A ladder is a gap for the same reason a flight is: the profile is planar, and a climb is almost
entirely vertical, so there is no ground path between the mount and the dismount to arc through or to
walk at running speed. The gap segment's cap is lowered to the connection's `ladderSpeed` (200, or 67
when the climb ends in a crouch-only area), which keeps the profile from believing the bot exits the
climb at 320 and mis-fitting the first corner after the dismount. The segment's length is still the 2D
distance across the dismount hop, so `totalTime` under-reports a climb; the *planner's* cost is the
honest one, and it comes from `ConnectionCostSeconds`, not from here.

### Speed profile
A backward pass propagates each node cap through `BrakeBack`, with the final waypoint pinned to 0 and
airmove launches capped at `launchSpeed`. Entry speeds are additionally clamped by the cap of the node
the segment starts from. `vEntry`/`vExit` are ceilings, not predictions: a segment leaving an arc may
report 320 while the bot is still accelerating from the arc exit speed. A forward pass integrates the
real speed against those ceilings to produce `totalTime`, which replaces the old fabricated ETA.

A second forward pass records the prediction the ceilings deliberately omit. Walking front to back, it
carries the reachable speed through `AccelForward` — the exact solution of `dv/ds = (a - f·max(v,s0))/v`,
bisected on the closed form, the mirror of `BrakeBack` — and stores it per segment as
`vReachEntry`/`vReachExit`. A flight neither accelerates nor decelerates horizontally, so a gap segment
passes the launch speed straight through. `--dump-path` prints this as the `reach` column beside `v`,
and the two disagreeing is the point: on `test37_walkent` the segment into the launch reads
`v 320 -> 288` next to `reach 81 -> 288`, which is the 155° hairpin before it showing up as a number.

**The reachable ramp is not a ceiling.** Feeding `AccelForward(vEntry, local)` into `SegSpeedAt` looks
like the honest thing to do and breaks every map: the follower's target speed is `here.speed`, at `s=0`
the ramp is 0, `SolveDrive` gets `wanted = 0` and commands nothing, and the bot never leaves the spawn.
Less obviously it also throttles any long segment that should accelerate and then brake, because
`max(vEntry, vExit)` with an honest entry collapses the trapezoid — ETAs rose 50% on `test28_slopecase`
before the deadlock was even reached. A prediction of the bot's speed is a lower bound on nothing; the
bot may well arrive faster. Ceilings steer, predictions validate, and they are separate fields.

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

**Velocity servo** - `SolveDrive`, below.

**Abort** - `needRepath` once lateral exceeds `rejoinAbort` (96). Previously nothing detected being
off-path; the corridor pull fought forever until the stall timer fired.

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
doubles friction by `sv_edgefriction`. `byRate` sidesteps the threshold entirely: the further the
velocity is from `V*`, the larger the commanded wish speed, so the bot saturates the frame and then
settles onto `V*`. See `doc/bot.md`, `RunAirmoveApproach`, for the dropoff this stalled.

`physics.edgeFriction` is read from `sv_edgefriction` and stored, but nothing consumes it — neither
`SolveDrive` nor `BuildPathProfile` models the ledge doubling, so both under-estimate drag near a drop.
`byRate` covers for it in the follower; the profile just runs slightly optimistic there.

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

### NavPlan::PlanWalker
One partial route: the funnel state, the waypoints emitted so far, the running string-pulled length and
the accumulated `costSeconds`. It is deliberately **copyable**, because the search keeps one per node and
expanding an edge means copying the parent's and stepping it forward.

- `Reset(startPoint)` — begin at a point, clearing waypoints, pending gates and stats.
- `Cross(conn, connIndex, fromId, toId)` — advance over one connection. Returns **false** when the
  connection cannot be planned, which today means only one thing: an air move whose launch could not be
  simulated into the destination. The search treats that as "this edge does not exist".
- `Finish(goalPoint, lastAreaId)` — flush the funnel to the goal and append the goal waypoint.
- `Respeed()` — the profile pass, see "Re-speeding a launch".
- `Time()` — committed corners plus the pull to the current frontier, over `kNavRunSpeed`, plus
  `AddCost` contributions. This is the search's `g`.
- `Frontier()` — the last pending gate's midpoint, or the anchor when the funnel is empty. This is what
  the heuristic measures from.

There is no batch `BuildWaypointPlan` any more; `PlanRoute` drives the walker directly and the goal
node's walker is the finished plan.

**Barriers.** Air moves, teleports, doors, buttons and triggers all flush the funnel and re-anchor it at
the waypoint they emit. So the string pull never crosses one of them, which is why a forward search can
resolve each air move exactly once, at the moment it traverses it, and never has to revisit an earlier
landing.

**Crouch belongs to the segment, not the corner.** `requiresCrouch` on a waypoint means "be ducked while
travelling to it", so it has to be set on the waypoint that *ends* every pulled segment passing through a
crouch area. It used to be read off the gate a corner sat on, which silently lost it whenever the funnel
ran straight through a crouch gate without emitting a corner there — and running straight through is the
normal case, since a gate only becomes a corner when the funnel constricts. `FlushFunnelTo` therefore
walks the pending gates alongside the pulled points: a corner sitting on gate `i` ends a segment spanning
gates `[previous corner's gate, i]`, and if any of those gates is a crouch gate the corner is flagged. It
returns whether the *tail* segment — the one into the flush target — needs it, because that waypoint is
pushed by the caller (run-up node, door, button, trigger, teleport trigger, goal), and each of those ORs
the returned flag with its own area's `crouch`.

The lost flag was not a cosmetic one. On `test45_slopes2` the funnel pulls straight from an air-move
landing in area 8 to the run-up node in area 9, crossing both 8↔9 gates without a corner; area 9 is
crouch-only, so the bot walked into the ceiling lip at exactly the 8/9 plane and stalled there. Nothing
else on the plan said "duck": the follower's only other source is the area it is *already* standing in,
which a standing bot can never enter.

`PendingGate::crouch` is `fromArea.crouch || toArea.crouch`, so both gates of a crouch connection carry
it and the flag lands one segment early on the way in and one segment late on the way out. The early one
is wanted — `PM_Duck` takes 0.4 s on the ground before `FL_DUCKING` and the smaller hull appear, so a
flag that arrived exactly at the boundary plane would be too late. The late one costs a segment held at
`kDuckingMultiplier` speed and is the price for it.

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

**Ladder:**
- Direction is read from the connection, not from a flag: `fromId == conn.areaA` means climbing, so the
  mount is `launchPos` and the exit is `landingPos`. Otherwise the bot is descending: the exit is
  `launchPos`, but the mount is **`gateB.left`, not `landingPos`** — where you land coming up is not
  where you get on going down. navgen puts the descent entry there; see `doc/navgen.md`. When the two
  differ it is a ledge point the bot runs off to catch the ladder mid-fall, which is why a descending
  mount can sit at the lip of a drop rather than against the ladder. It runs at `ladderEntryYaw`
  (from the connection's `gateAYaw`) and holds `ladderYaw` once on the ladder; the two are the same
  angle except for a side approach, where catching the column means coming at it from ~90° off the
  face normal.
  A one-way ladder never reaches the descending case, because `PlanRoute` refuses the reverse edge.
- Flushes funnel to the mount, at player height for the area being left
- Adds mount waypoint (requiresLadder=true, ladderUp, ladderYaw, ladderSpeed, ladderExit, targetArea,
  connIndex)
- Adds exit waypoint (targetArea), not counted toward path length — the climb is vertical and its cost
  already arrived through `AddCost`, from `ConnectionCostSeconds`' ladder arm
- Anchor moves to the exit point
- No gate is pushed. Ladder gates are degenerate points at the mount and dismount, so there is nothing
  for the funnel to string-pull through; the mount is simply a hard waypoint, like a teleport trigger.

**Air Move:**
- Pushes arrival gate (entry side only)
- If traces available: re-simulates from funnel's preferred launch point
  - Computes funnel crossing on launch gate (exit side)
  - Interpolates yaw/speed from gateAYaw/gateASpeed
  - Re-simulates with PmSim; if lands in dest area, uses simulated params
  - On a miss, `Cross` returns false and the whole edge is refused — there is no fallback to the baked
    params, because a launch that does not land where the plan says is worse than no edge at all
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

### Implied Walls from Floor/Ceiling Pairs

**ForEachImpliedWall** - an area's footprint is not fixed by its vertical planes alone. A floor rising one way and a ceiling falling the other pinch the column shut where they meet, and that closure is a vertical half-plane no bound plane records: a wedge under a ramp is capped by the ramp itself, and the sliver between two faces of a pyramid is bounded that way at both ends. Eliminating z from the half-space system recovers those bounds exactly - for a floor f and a ceiling c, "the floor stays under the ceiling" is one linear constraint in x and y, and it is implied by the two planes, so it can only trim overreach, never cut into the area. The function iterates all floor/ceiling pairs; parallel in plan pairs (they never converge, so they bound nothing here) are skipped. Each implied wall is emitted as an inward-facing half-plane in x/y.

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
- Flags: requiresElevatorRide, requiresCrouch, requiresDoor, requiresBreak, requiresButtonUse, requiresTriggerTouch, requiresTeleport, teleportDest, requiresLadder, ladderUp, ladderYaw, ladderEntryYaw, ladderSpeed, ladderExit, airmoveLaunch, requiresJump, launchSpeed, launchYaw, jumpBackDist, launchSimulated, launchSimFailed, launchSimFrom/To/Aim, targetAreaId
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
These are per-walker, so they describe the route that was kept and nothing else — a walker copied down a
branch the search abandoned takes its counters with it.

- `airmovesSimulated` - launches re-simulated and accepted, i.e. air moves actually on the plan.
- `airmovesUnplannable` - launches the simulation could not make. On the winning walker this is always
  0, because a failed launch means `Cross` returned false and the walker was discarded. It is the
  per-walker view of what `PlannedRoute::airmovesRejected` counts across the whole search.
- `airmovesRespeeded` - launches the profile could not deliver the design speed to, which were
  re-simulated at the speed it could and still landed.
- `airmovesRespeedKept` - launches where that slower re-simulation also missed, so the design speed was
  kept. See "Re-speeding a launch": this no longer truncates.
- `corners`, `gatesClamped` - funnel diagnostics.
- `runupTraces` - vestigial, never populated. `--dump-svg` still accepts them and draws nothing.

`airmovesFromStored` and `unplannableConn` are gone. The first never counted what its name said — it was
incremented on the branch that gave up, so it only ever equalled `airmovesUnplannable`. The second existed
only so the retry loop knew which connection to block, and there is no retry loop.

### Unplannable air moves
A launch that `JumpLandsIn` cannot make (or one with no gate to funnel through, so no simulation was
attempted) makes `Cross` return false. That is all that happens: the search drops the edge and carries
on. Nothing is truncated, nothing is retried, and the route that comes back has no unplannable air moves
in it by construction.

This is worth stating plainly because it used to be the opposite. The planner would truncate the route at
the bad launch, and `PlanRoute` would then block that connection and re-run A* from scratch, up to eight
times. Both of those are gone.

The failure is genuinely route-dependent, not a property of the connection alone: the stored launch comes
from `ProbeAirMoves` sampling a whole edge, while the plan launches from wherever the funnel crosses that
edge on *this* route. `test45_slopes2` is the worked example — a drop off the upper floor's south lip is
sampled fine, but the funnel pulls the launch to the corner of the gate, from where the mover clears the
lip and lands on the ramp below instead of the pit floor, outside the destination area's bound planes.

Because rejection now happens inside the search, `--dump-airmove`'s `sim miss` lines are no longer
failures — they are the edges the search routed around, and their count matches `rejected in search`
exactly. Three on test45_slopes2, four each on test28_slopecase and test15_staircase. A connection that
appears there on every route is still an air move `ProbeAirMoves` should not have exported, and chasing
it belongs there rather than here.

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

Two things bound that distance from underneath and above. `AccelDistance(0, launchSpeed)` plus one
`AirmoveLaunchLead` is the distance the physics actually needs to reach the design speed from a
standstill, and the node is never placed closer than that. It never binds in the current speed range —
0.1 s of travel is the larger term from roughly 70 u/s upward — but the 0.1 s is a travel-time guess
and this is the real requirement, so the guess can no longer silently fall under it.

`ClampTravelInsideArea` then walks the wall planes of the source area and shortens the offset so the
node lands inside it, `kLaunchInset` clear of the wall. Placing it blindly put the node **through** the
wall whenever the launch sat less than the offset from the far side: on `test37_walkent` the 5→4 launch
at y=43.6 in an area starting at y=16 got a run-up node at y=14.9, 1.1 units inside solid. The funnel
then pulled the approach against geometry that is not walkable and the arc fitter had nothing to work
with. The offset is length, not direction: it does not stop the node landing behind the bot when the
route arrives perpendicular to the launch heading, which is what produces the hairpin above.

It is load-bearing for a second reason that has nothing to do with alignment. `Bot_MoveNav` hands
control to `RunAirmoveApproach` when the tracker's `wpIndex` reaches the launch waypoint, and
`wpIndex` comes from whichever profile segment the bot projects onto. `BuildPathProfile` drops
segments shorter than `kEpsLen`, and the funnel's last corner lands on the launch point exactly, so
without this node the segment ending at the launch is zero-length, gets dropped, and no segment ever
carries the launch's index. The bot then walks straight through the launch point and never jumps.

### Launch speed is clamped to the server
`candSpeed` comes out of the nav file's `gateASpeed`, which the sampler produced at navgen's assumed
`maxSpeed` — 320 by default. The server need not agree: `hlds/valve/server.cfg` ships `sv_maxspeed 270`,
and `BotPhysics` reads the live value into `PlanTraceContext::physics`. `Cross` therefore clamps
`candSpeed` to `physics.maxSpeed` before handing it to `JumpLandsIn`.

This is what keeps a nav file server-independent. Sampling stores the fastest jump the geometry allows;
each server's planner then asks whether *it* can make that jump, and an air move needing 315 on a
270 server simply fails `JumpLandsIn` and is rejected in the search like any other unplannable launch.
No regen, no per-server nav.

The clamp has to live here because nothing downstream applies it. `PmSim::Begin` sets the launch
velocity to the commanded speed **unclamped** — its `maxSpeed` check bounds the air-accel wish speed
during flight, not the launch — so an unclamped `candSpeed` produces a simulation that succeeds at a
speed the bot can never hold, and a plan that asserts it. `RunAirmoveDrive` does clamp its drive to
`BotMaxSpeed`, so the bot then chases a launch speed it is not allowed to reach and the approach never
arms: `test40_walljump` planned 315, drove at 270, and aborted every attempt.

`Respeed` alone did not cover this. It measures the shortfall correctly, but when the slower
re-simulation also misses it keeps the design speed by design, so the unreachable 315 survived to the
follower as `airmovesRespeedKept`.

### Re-speeding a launch
The stored `exitSpeed` is what the air-move sampler launched at, not what the route can deliver. A
corner 30 units before the launch caps the bot well under it, and nothing used to notice: the plan
asserted the design speed and the jump was validated at a speed the bot would never reach.

`PlanWalker::Respeed` therefore builds a probe `PlanProfile` over the finished waypoint list.
`ProfileLaunchReach` reads the reachable-speed pass on the segment ending at each launch — the arc fit,
the backward braking pass and the acceleration integral all folded into one number — and if it falls
short of the design speed by more than `kPlanAirmoveRespeedEps`, the jump is re-simulated at that
speed. The landing waypoint moves to wherever it actually lands. The yaw needs no adjustment: the last
leg is straight along the launch heading by construction.

**The reach is taken one `AirmoveLaunchLead` short of the launch point, not at it.** The bot commits to
the jump a frame of travel before the plane, so that is where its speed stops rising, and measuring at
the launch point over-reports by exactly the speed gained in that last frame. Measuring at the point of
no return is also what makes the check bite at all: read at the launch point it saturates, because
`vExit` on that segment *is* the design speed, so the comparison could only ever fail when the profile
fell short over the whole segment.

**A slower jump that no longer lands does not truncate the route.** The design speed is kept and
`airmovesRespeedKept` counts it. Truncating here was the last place a complete plan could still be cut
short, and with the retry loop gone there is nothing left to recover it — a jump the bot may come up
short on beats a plan that stops dead in the middle of the map, and the follower already has
`launchSimFailed` to reason about. In practice this branch has never fired: `airmovesRespeeded` and
`airmovesRespeedKept` are both 0 on all 52 test maps and on crossfire, because the profile has so far
always delivered the design speed.

This is also why the search does not carry an incremental reach-speed estimate. Folding one into
`PlanWalker` would let launches be simulated at the right speed the first time, but it would change
launch speeds on every map to replace a pass that never fires.

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
- `ConnectionCrossing(areas, conn)` - crossing point, still stored on the graph edge
- `PlanWalker` - the entry point; `NavPlan::PlanRoute` drives it

## nav_format.h

### Nav File Format (version 11)
- Magic: "CBNV"
- Header: magic, version, mapName[64], bspChecksum, flags, counts
- AreaRecord: id, mins[3], maxs[3], centroid[3], areaFlags, connOffset, connCount, planeOffset, planeCount, doorActivation, doorActivatorPos[3], doorWaitTime
- ConnectionRecord: areaA, areaB, connType, connFlags, gateValid, pad, costSeconds, gateA[2][3], gateB[2][3], gateAYaw[2], gateASpeed[2], exitSpeed, exitAngle, jumpBackDist, fallDamage, flightTime, launchPos[3], landingPos[3]

`nav_loader.cpp` rejects a version mismatch outright, so bumping forces regeneration rather than silently
loading a stale mesh. Area planes are the exported `boundPlanes`, which for a prism area is its footprint hull
plus a z-slab rather than its true faces.

### Enums
- AreaFlags: CROUCH(1), LADDER_TOP(2), WATER(4), DOOR(128), BREAKABLE(256)
- ConnType: WALK(0), TELEPORT(1), AIRMOVE(2), LADDER(3) - a ladder reuses the air-move fields rather than
  adding its own: launchPos is the mount, landingPos the dismount, exitAngle the climbing yaw, exitSpeed
  the climb speed, and flightTime/fallDamage belong to the dismount hop
- ConnFlags: ONE_WAY(1), NEEDS_CROUCH(2), JUMP(4) - jump means left ground under impulse, not derivable from jumpBackDist
- DoorActivation: TOUCH(0), BUTTON(1), TRIGGER(2)
- GateValidFlags: VALID_A(1), VALID_B(2), CLAMPED_A(4), CLAMPED_B(8)

## pm_sim.h

### PmPhysics
- `accelerate`, `groundFriction`, `stopSpeed` - sv_accelerate, sv_friction and sv_stopspeed. `PmSimulator` never reads them; they are carried for `BuildPathProfile` and `NavFollow::SolveDrive`, which are where ground physics is actually modelled.
- `edgeFriction` - sv_edgefriction. Read, stored, and consumed by nothing. See `NavFollow::SolveDrive`.
- `friction` - pmove->friction, the per-entity multiplier. Not sv_friction.

### PmSimulator
- `Begin(origin, yaw, speed, jump)` - starts on the ground with velocity derived from the yaw and speed.
- `BeginAirborne(origin, velocity, yaw, speed)` - starts mid-flight with the velocity given directly,
  for a state that was measured rather than commanded. This is how a ladder dismount launches: the climb
  velocity is a fact of `PM_LadderMove`, not something a yaw and a speed can express. Both entry points
  delegate to the same private `Reset`.

## pm_sim.cpp

### Step
One frame of **airborne** movement only: gravity, the pending jump impulse, `AirAccelerate`, `FlyMove`,
`CatagorizePosition`. There is no `PM_Friction` and no ground accelerator anywhere in this file.

That is the right shape for what the simulator is for — proving where a launch lands — but it is the
reason a plan can assert an air move the bot cannot begin. `JumpLandsIn` starts the mover *at* the launch
point already travelling at the launch speed; nothing simulates the run-up, so a launch speed the bot
cannot accelerate to on the ground is indistinguishable from one it can. The ground approach is checked
by `BuildPathProfile` instead, which does model friction and braking, and executed by
`NavFollow::SolveDrive`.

### FlyMove
Reaching here with i == numplanes means the loop above found no single plane the velocity could slide along, which is PM_FlyMove's "go along the crease" case. The early break out of that loop already stored the sliding velocity, so it must not fall in here -- testing i != numplanes instead zeroes the velocity of every successful slide.

### AirAccelerate
PM_AirAccelerate, wish speed capped at 30, so it can only ever nudge a trajectory in flight.