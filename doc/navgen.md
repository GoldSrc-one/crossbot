# Navgen System Documentation

## main.cpp

### Entry Point & Debug Output
Command-line tool with extensive debug flags:
- `--dump-leaves` - per-leaf (pre-merge) id/faces/volume/centroid stats
- `--dump-areas` - per-area (post-merge) id/faces/volume/centroid/member-leaf stats
- `--dump-conns` - every final connection: type, flags, endpoints, air-move parameters
- `--dump-activators` - activator discovery and the precomputed door detours: each gated entity's activator
  brushes with their use/touch volumes, every candidate service point with the area it stands in and why it
  was kept or rejected, and per `(door area ← entering neighbour)` the chosen service point, the detour
  polyline with the activation point marked, and its length — or why that entry is blocked
- `--dump-steps` - every walk connection the step filter trimmed or dropped: area pair, shallowest
  crossing, both floor heights, surviving walkable runs, each floor face's normal z and 2D offset
- `--dump-support` - every leaf fragment dropped as unsupported: floor centroid, floor normal, volume
- `--dump-airmove` - air-move launch derivation: why no funnel launch was found, every re-simulation
  miss with its launch, yaw, speed and where it landed, every rescaled retry with the speed it
  corrected to and the wanted/travelled distances behind it, and every `retract` attempt with the
  run-up it needed against the run-up it had (see `doc/nav.md`)
- `--dump-planes <id>` - one final area's exact plane list (normal+dist) and face windings
- `--dump-obj <path>` - walkable floor of every area as OBJ mesh (+ MTL): crouch areas colored apart, planned route as ribbon
- `--dump-svg <path>` - top-down SVG of final area graph
- `--dump-airmove-prune` - the walk-vs-air comparison behind every kept or dropped air move, and the
  walk+air-vs-ladder comparison behind every kept or dropped ladder connection
- `--dump-ladders` - ladder discovery: each `func_ladder`'s hull-3 volume, why each face was kept or
  rejected, the mount and top positions, and every dismount simulation with its height, direction,
  landing area, fall damage and loopback verdict
- `--dump-elevators` - elevator classification: every `func_door`/`func_plat`, its movedir and footprint,
  whether it was reclassified as a rideable platform or left to the door path and why, and for each accepted
  one its stop positions with ride and dwell times, activation, shaft column, the boxes carved out of the
  walkable leaves, the tagged stop leaves and the ride connections built. Also, per non-default stop area,
  the air-move sweep run with the mover placed there: perimeter edges, how many of them are supported with
  the mover parked versus placed, and the launch edges and samples that came out
- `--dump-path` - per-waypoint clearance and per-segment arc/line/gap rows (radius, turn, length,
  entry/exit speed, cap, and the `reach` pair — the speed the bot can really carry, see `doc/nav.md`)
  for the debug route, plus the profile summary line
- `--trace <path>` - overlays recorded bot trajectory on SVG (one "x y z" per line, empty file = nothing).
  Also reports mean and max lateral deviation of the trace from the planned profile, and the clearance
  budget at the worst point — the headline number for whether the bot actually held its arcs.
- `--airmove-samples <n>` - launch samples per unit edge length (default 1; 0.5 halves; 0 disables)
- `--airmove-walk-ratio <n>` - drop an air move a walk covers within n times its cost (default 1.2; 0 keeps all)
- `--ladder-prune-ratio <n>` - drop a ladder a walk+air-move route covers within n times its cost
  (default 2.0; 0 keeps all)
- `--max-speed <n>` - assumed ground speed in units/sec (default 320), used for air-move sampling and
  for the `--dump-path` debug route. It deliberately does **not** have to match the server: a nav file
  is server-independent, and the runtime planner clamps every launch to its own `sv_maxspeed` (see
  "Launch speed is clamped to the server" in `doc/nav.md`). Sampling at 320 and letting a 270 server
  reject what it cannot reach is the intended arrangement — the same nav then works on any server
  without a regen. Lower it only to keep the graph small when you know the target speed — to see what
  a slower server actually plans, use `--plan-speed` instead.
- `--plan-speed <n>` - ground speed the `--dump-path` debug route is planned at (default: whatever
  `--max-speed` is). This is the flag that reproduces a server offline: the mesh stays sampled at
  `--max-speed`, and only `PlanTraceContext::physics.maxSpeed` is overridden, which is the single
  input to `PlanRoute`, `JumpLandsIn` and `BuildPathProfile`. So `--plan-speed 270` against a nav
  baked at 320 gives the same waypoints, launch speeds and counters the runtime produces on a
  `sv_maxspeed 270` server, without starting HLDS. The baked `.nav` is byte-identical with and
  without it.
- `--dump-carve` - report what the blocker carve and the passable split do to the leaf set: leaf and
  volume totals before and after each stage, per-leaf fragment counts over `kCarveDumpThreshold`, a
  per-blocker table of carves/pieces/wasted carves, and every carve that came back with more volume
  than it was given
- `--no-blockers` - skip ExtractBlockingVolumes, so no func_wall/func_wall_toggle/func_conveyor/
  func_healthcharger/func_recharge brush is carved out of the leaves (debug)
- `--no-passables` - skip ExtractPassableVolumes, so no func_door/func_door_rotating/func_breakable
  is split out as its own tagged leaf (debug). Doors already claimed as elevators are unaffected:
  they never reach the passable path
- `--no-floor-split` - keep leaves whose floor is not solid (debug)
- `--max-merge-group <n>` - most leaf groups one merge may fuse (default 64). `1` blocks the first
  absorption, so nothing merges at all and each BSP hull-3 leaf becomes its own area — this replaces the
  old `--no-merge`, whose output it reproduces exactly
- `--no-prune` - skip PruneUnreachableAreas (debug: keep areas unreachable from any spawn)
- `-o <path>` - write baked .nav file

### Debug Path Building
`BuildDebugPath`:
- Finds spawn areas (info_player_start/deathmatch) and target areas (info_target)
- Runs `NavPlan::PlanRoute` from first spawn to first target (or farthest area centroid) — the same
  single-pass planner the runtime uses, which string-pulls and simulates air moves *during* the A*
  search. An air move the planner cannot steer across never enters the route: the edge does not relax
  and the search continues. The summary line counts those as `rejected in search`, and a second
  `debug search:` line reports how many areas were expanded and how many air-move simulations that
  cost. A connection that keeps being rejected is an air move `ProbeAirMoves` should not have exported;
  `--dump-airmove` prints each miss with its launch, yaw, speed and where it actually landed. The
  `sim miss` lines match `rejected in search`; a `rescale` line without one following it is an edge the
  retry rescued
- Re-simulates air moves from funnel's preferred launch point if traces available
- Validates route stays over areas (skips air-move segments)
- Builds the `PlanProfile` and tessellates it, so the exported route is the arc chain the bot will
  actually follow, not the raw waypoint polyline
- Outputs path points + segment kinds (walk/airmove_simulated/airmove_stored) + per-segment speeds,
  which the SVG uses to tint slow stretches red

### OBJ Export
`WriteFloorObj`:
- Writes floor faces (normal.z > 0.7) per area, grouped by area with crouch material
- Adds planned route as flat ribbon (waypoints at centroid height, well above floor)
- Materials: nav_floor (blue), nav_floor_crouch (orange), nav_path (green)

### SVG Export
`WriteDebugSvg`:
- Areas drawn as 2D convex hull of vertices (prism areas = footprint)
- Painter's order: lowest floor first (higher areas paint over)
- Connections: white (walk), dashed red (air move), green (teleport)
- Gates: purple dashed (walk), red dashed (air move)
- Planned route: orange polyline; air move segments overlaid (green=simulated, red=stored, yellow=missed)
- Bot trace: cyan polyline under orange route
- Targets: magenta crosshairs
- Legend panel auto-sized

## area_merger.cpp

### MergeAreas
Union-find merge of leaves through walkable floor seams, with two merge modes:

**1. 3D Convex Merge**
- `MergeStaysConvex`: all vertices of B inside all planes of A, and vice versa (epsilon 0.5)

**2. Prism Merge (vertical extrusion)**
- Allowed when 3D convex fails but the XY footprint union stays convex
- `MergedFloorStaysWalkable`: no internal step > 18 units between any floor face pair
- `CoverageOk`: the group's footprint hull equals the union of its *member leaves'* footprints to within
  1 unit². Coverage is `Σ area(fp_i) − Σ_{i<j} overlap(fp_i, fp_j)`; by Bonferroni, truncating
  inclusion–exclusion at the pairwise terms is always a **lower bound** on the true union, so the test can
  only reject, never falsely accept — which is what lets it work at any group size without a polygon-union
  primitive. It iterates member leaves rather than the groups' stored hulls on purpose: a group's stored
  footprint is a hull, and hull-of-hull compounds across successive merges
- The 1 unit² is floating-point noise slack, not a fudge factor. It used to scale with the merged area
  (0.1%), which on a room-sized footprint bought ~50 units² of unearned claim — enough for the hull to
  chamfer straight across a pillar, which is exactly what test49_pyrascew used to do
- `FirstSlabConflict`: no leaf outside group has XY overlap with merged footprint AND Z overlap with merged slab
- Slab test evaluated against *leaves*, never groups (earlier prism can't be invalidated by later merge)
- If slab conflict: absorbs conflicting group if a candidate seam reaches it, retries (ramp closes as whole)

**Growth past a non-monotone predicate.** Coverage is checked only at accept time, never as a reason to
abandon a candidate pair. This matters because coverage is *not monotone*: on a pyramid, the ring of
wedges plus the apex column is one convex prism, yet every proper subset of it leaves a hole and fails.
Validating after each addition — as the merger used to — can therefore never reach the valid group, no
matter what order candidates are tried in. Instead, when coverage fails, `FirstFootprintGap` asks which
outside leaf claims the uncovered part of the hull and the group absorbs it, exactly as it already did for
slab conflicts. A gap **nobody** claims means the space is solid (a wall, or the inside of a slope), and
the merge is abandoned.

`FirstFootprintGap` is `FirstSlabConflict` without the Z-overlap requirement, and that difference is load
bearing rather than an optimisation: an apex column sits *on top of* the frustum its wedges bound, so the
two Z ranges touch at exactly one plane and `OverlapsZ` is false. Dropping the Z test is safe because the
`sharedSeams(...)` guard still requires a walkable floor seam into the group, which a leaf on an unrelated
floor does not have. Keeping the two finders separate leaves `FirstSlabConflict`'s Z rule — the thing that
stops stacked floors merging — untouched.

**Group cap.** `maxMergeGroup` (`--max-merge-group`, default `kDefaultMaxMergeGroup` = 64) bounds how many
leaf groups one merge operation may fuse. It is checked at the top of the growth loop, *before* absorbing,
so `1` refuses even the first pair and no merging happens at all — that is why there is no separate
"no merge" mode. Checking it after absorption instead would let a 3D-convex pair through at `1`, since
that path accepts without ever reaching the cap.

Raising the cap from `1` upwards is also the cheapest way to bisect a merge bug: it bounds *how much*
merging it takes to reproduce, which is usually faster than reading the growth loop.

**The same tolerance validates the result.** `ValidateAreaFootprints` re-measures the finished areas with
`WalkableFootprintGapArea` and warns (`ill-formed walkable footprint`) past the same 1 unit². Keep the two
numbers equal. When the merge test was the looser of the two, the validator could never fire on anything
the merger had just accepted, which is how the test49_pyrascew overclaim went unreported.

### Area geometry vs exported region
An area carries two descriptions of itself and they are not interchangeable:

- `Area::polytope` — the true surface, multi-tread floors and all. Floor lookups, the step filter, air-move
  launch edges and the OBJ/SVG dumps all want this.
- `Area::boundPlanes` — the convex region written to the nav file. A convex area exports its own face planes; a
  prism exports its X-Y footprint hull plus a z-slab, which is a convex region the runtime's `ContainsPoint`
  can test unchanged. The slab check during merging is what makes the substitution sound.

**Anything asking "is this point inside this area" must read `boundPlanes`.** A prism's `polytope.faces`
include interior risers, and their halfspace intersection is empty — the test silently answers "no area" for
every point. That bug cost real connectivity twice before it was understood: `FindAreaStrict` (air-move landing
resolution) and `PointInArea` (spawn/target resolution) both returned nothing and gutted the graph.

`--dump-planes <id>` prints the exported bound planes first, then the true face windings.

### Why seams and not portals
Merging is a volume operation, so for a long time it ran off `BuildPortals` — a global list of shared faces
between leaves. That coupling was the bug behind the staircase regression: deleting the floorless air column
above a tread also deleted the faces that made two treads portal-adjacent, and the flight stopped merging even
though every walk connection survived. Raw portals collapsed 261 → 150 on test15_staircase while leaf count
barely moved.

Floor seams carry the same adjacency without the coupling, because `BuildBoundPlanes` already derives a prism's
region from the walkable footprint rather than from faces. What portals still supplied was face *cancellation*
for the convex path, and that is now computed on demand per candidate pair by `FindSharedFace` (opposed planes
plus `IntersectCoplanarPolygons`) instead of globally up front. Seam-adjacent leaves separated by a riser share
no face at all; they simply take the prism path.

Two leaves whose walkable floors meet on an XY line are not necessarily the same open space — hull-3 expansion
makes floor edges meet at *every* vertical drop, an 8-unit step and a 200-unit cliff alike, because the top
face expands outward by 16 and the side wall expands the lower floor back by the same 16. What makes a seam a
merge candidate is the step test, not the seam.

### Seam Filtering
- Crouch mismatch: skip
- Passable (door/breakable) mismatch: skip
- `SeamIsWalkable`: the walkable spans must cover the *whole* seam

Full coverage is the point. A slope running alongside a floor meets it on one XY line but is only steppable at
one end, and a predicate that asked "is it walkable anywhere" would fuse the two into a single area whose
footprint claims ground a bot cannot reach.

### Walkable Spans (WalkableSpansAcross)
The stretch of the segment a + ab * t, t in [0, 1], whose XY projection lies within `epsilon` of this face's XY footprint. Windings are convex, so that stretch is one interval, found by clipping the parameter range against each edge's outward half-plane. The epsilon is what makes the answer stable: a seam lies on a leaf boundary, so the segment routinely runs exactly along a floor face's edge, and without slack whether the face counts would come down to floating-point luck.

Parallel to this edge: the segment is either wholly inside the half-plane or wholly out.

Each floor face is a plane, so wherever the same pair of faces applies both heights are linear in t and their difference is too; the set of applicable faces only changes at the ends of the spans above. The gap is therefore piecewise linear with known breakpoints, and `|gap| <= maxStep` solves on each piece to a single sub-interval - there is nothing to sample. The union of those sub-intervals is the answer: the exact stretches of the seam a bot can step across.

Where two faces abut, both are candidates along the shared stretch and a point counts as walkable if *any* face pair clears the step there. That is the same existential question the caller is asking, and it replaces a tie broken by face iteration order.

The shallowest crossing found is kept alongside the spans purely so `--dump-steps` has something to report: area pair, where it is, both floor heights, the step, and each floor face's normal z and 2D offset from the query point. A nonzero offset means `FloorFaceUnder` fell back to its nearest face instead of finding a containing one.

`valid` is false when no stretch of the segment has floor on both sides. That is a different answer from "the step is large", and the two callers want opposite things from it, so it is left to them.

One sided over the whole stretch, so the closest approach is at an end.

The floors cross somewhere in here, so they meet exactly.

A wedge closed off by a sloped ceiling (the crawlspace under a ramp) ends where the ceiling meets the floor, and no near-vertical plane records that -- so the gate builder cannot see the end of it from the bound planes alone. Reporting "no floor here" outside the real floor footprint keeps gates inside the area they belong to.

No floor on both sides anywhere along the seam is not the same as a big step, and there is nothing to judge the connection on, so it stays.

Refusing to merge on missing information is the safe direction, so unlike the connection filter this treats "no floor on either side" as not walkable.

### Processing Order
Candidates sorted by seam length descending (longest seams first). Each candidate attempts to absorb as many neighbors as possible via the `sharedSeams` chain.

### Post-Merge
- `FilterWalkConnectionsByStep`: replaces each connection with its walkable runs. A seam walkable end to end
  passes through untouched; a partially walkable one is cut down to the stretches that clear the step, one
  connection each, dropping runs shorter than `kMinSeamLength`; a seam walkable nowhere is dropped. This is
  what stops a slope beside a floor from exporting a gate spanning the part of the seam that is 60 units up
- `ProbeAirMoves`: samples launch edges, simulates jumps/drops, fits gates to successful samples
- `SnapConnectionsToFloor`: walk = both endpoints to side A floor; air move = launch only (landing = sim result)
- `PruneUnreachableAreas`: BFS from spawn areas over connections (respects one-way)
- `BakeConnectionGates`: runs shared planner code to build gates, lifts endpoints to true floor

### Cost Calculation
`ConnectionCostSeconds` returns the seconds the bot loses on this edge **beyond simply running its length**:
waiting for a door to open, shooting a breakable, or hanging in the air on a jump. The A* graph adds it to the
travel time, so it is real time and **not** a distance multiplier — it used to be one, and reintroducing a
multiplier would silently break door and air-move routing.

- plain walk — 0
- air move — `flightTime` plus a fall-damage penalty, keeping the "lethal ⇒ effectively infinite" sentinel

Door and breakable waits are **not** here. They used to be, added to every connection *touching* a gated area,
which charged a through-route twice and skewed `airmove_prune`'s Dijkstra near doors. `PlanWalker::Cross` now
adds `openSeconds` to `m_extraTime` once, on entry only. The activator detour costs its own travel time through
`m_committedLength`, which is the whole point of expressing cost as time.

`ConnectionTravelSeconds` is the other half: the running time this edge costs, which the planner does not
need (it measures the walk itself) but a walker-less search cannot live without. Centroid to centroid over
`kNavRunSpeed`, funneled through the gate — the straight line if it crosses the gate segment, otherwise
bent around the nearer gate endpoint, so an L-shaped area is not priced as if the bot walked through the
wall. Non-walk connections get only the ground legs to `pointA` and from `pointB`; their flight, climb or
ride is already in `ConnectionCostSeconds` and must not be counted twice. It goes in the nav file as
`travelSeconds` (see `doc/nav.md`) because it is fixed by the mesh and the funnel is not worth redoing per
frame.

`airmove_prune.cpp`'s own `WalkDijkstra` still uses its local centroid-to-centroid measure rather than this
one. Routing it through the shared helper would change pruning decisions on every map for no benefit here.

### Door/Breakable Isolation
A merge never crosses `ExtractedLeaf::passableIndex`. A door or breakable has to stay its own area so it keeps
its activation, and it must not absorb a different one either.

### Door Connection Logic
Both halves come out of the same clip planes, so their floors tile without a gap and the seam collector connects the door piece to its neighbours with no extra linking.

## airmove_sampler.cpp

### PM_MoveSimulator
Wraps shared PmSim core with navgen's trace function (world + blocking volumes).

`OnLadder` is `PM_Ladder`: a point-content test of the origin against each `func_ladder`'s clipnode tree
for this hull, AABB-prefiltered. It must be the clipnode test and not the AABB, because a slanted ladder's
AABB is far larger than the brush — test53_ladder4's model 3 is 160 units wide as a box and a thin slab in
fact. `Fly` ends the flight with no landing the first airborne frame the origin is on a ladder: a
`func_ladder` is `SOLID_NOT`, so it stops no trace, but the engine switches the player to `PM_LadderMove`
the moment they enter one, and with no input they hang there. A trajectory that grazes a ladder therefore
reports a landing the player never reaches. Ladders mount on walls, so the flights this catches are the
ones that clip a wall and slide down it.

The check is skipped while the mover is on the ground, so a launch edge that overlaps a ladder's footprint
still gets sampled; only leaving the ground next to one is refused. `HullContext::ladders` carries the
volumes per hull. `BuildLadderConnections` deliberately leaves it null on its own simulator — a dismount
starts inside the ladder it is leaving and would abort on frame one.

### ProbeAirMoves
For each connection:
1. **Collect launch edges** - perimeter of floor faces (outward normal from face center)
2. **Seed drops** - from each edge, fan yaws ±45° around outward, 1.7x maxSpeed run-up
3. **Seed jumps** - same edges, add jump impulse, fan yaws ±90°
4. **Seed reverse jumps (climbs)** - from landing spread of drops, edge along landings, outward toward ledge
5. **Simulate each sample** - PmSim until land/wedge/timeout; check landing in dest area
6. **Bin by hull and jump** - four bins: stand/duck hull × drop/jump
7. **Select best per bin** - prefer: jump > drop; shorter flight time; less fall damage
8. **Fit gates** - `FitGate` on launch/landing clouds (robust Tukey-weighted PCA segment, local yaw/speed regression at ends)
9. **Store on connection** - gates, exitSpeed/Yaw, jumpBackDist, fallDamage, flightTime, launchPos/landingPos

### FitGate (robust segment fitting)
- Iteratively reweighted (Tukey biweight, 4 iterations)
- PCA on weighted points → segment direction
- Local regression (tricube kernel, bandwidth = 0.5×segment) at segment ends for yaw/speed
- Gate endpoints = mean ± 1σ (min 1.0), clamped to sample range
- Returns invalid if cloud too thin/tight

### Reverse-Jump Seeds
A drop under `maxJumpHeight` seeds a climb back the other way: `SeedEdgeFromLandings` turns the drop's landing
points into a synthetic launch edge in the destination area, swept like any other edge.

Every edge, seeded or not, fans yaw over the full `±yawSpread` (90°) around its outward normal.

**A narrower arc for reverse seeds was designed but is not implemented.** The idea was that a reverse seed
knows where it is going — the climb has to retrace the drop — so it could aim only along the drop's own
headings reversed, and `LaunchEdge` still carries the `yawRange`/`yawLo`/`yawHi` fields for it. Nothing
sets or reads them; they are dead. Anyone reviving this should know why it looked worth doing:
`BuildSampledGates` **drops the whole connection, not just its gate**, when `FitGate` cannot establish
lateral spread, so fanning wide at a narrow opening can lose a genuinely traversable crossing.

### Sample Cloud Aggregation
Samples from every edge the pair was reached from land in one cloud. Which edge a sample left through does not matter: the robust fit keeps whichever cluster is the majority and rejects the rest as outliers, so the gate settles on the face the move is really about.

### Probe Variables
Three sweep modes run over every area, each with its own hull, speed and z offsets:
- **stand** - stand hull, `probeZ`/`launchZ` = `kStandProbeLift` (18), `navZAdjust` = `-kStandProbeLift`
  to bring standing-origin positions back into hull-3 nav space
- **crouchjump** - duck hull at standing speed, `launchZ` = `kStandProbeLift`, jumps only
- **duck** - duck hull at ducking speed, all offsets 0 because duck space *is* nav space

`emitted` maps pair key -> whether the winning move needs a crouch.

## ladder_connections.cpp

### BuildLadderConnections
Runs after `ProbeAirMoves` and before the prune passes. Emits `CONN_LADDER` connections for every
`func_ladder`.

### The volumes are carved out of the walkable leaves
`SubtractBoxVolumes` (blocking_entities.cpp) removes each ladder's hull-3 volume from every leaf it
overlaps, before areas are built. A nav area is duck-origin space and the expanded volume is exactly the
set of origins the engine puts on a ladder, so after the carve **no nav area contains a ladder-capture
point**. That makes routing past a ladder structural: neither the planner nor `bot_avoid` can send the bot
through a spot where `PM_PlayerMove` would switch it to `MOVETYPE_FLY` and convert its forward input into
a climb. Only the ladder state machine ever puts the bot inside one.

The carve uses the ladder's **axis-aligned hull bounds, not its clipnode tree**, even though the tree is
exact. `ClipPolytope` drops a face whose clipped winding falls under 3 vertices, which loses that face's
plane from the polytope; the next clip rebuilds its cut face from `BaseWindingForPlane(plane, mapRadius)`
against the remaining planes, so a lost bounding plane lets the piece escape to the map radius.
Axis-aligned planes rarely trip this. test53_ladder4's slanted ladder did, catastrophically — total leaf
volume 94,346,287,097 against 25,747,456 uncarved, with areas reaching `maxs 8192`. `ClipPolytope` now
rejects a clip result with fewer than four faces, which closes that hole at the source (see
"SubtractBlockingVolumes"), so the box carve here is belt and braces rather than the only defence. The box carve is
conservative for a slanted ladder (it removes walkable floor under the slope) and exact for a vertical one.

Because the mount now lands inside the carved hole, `FindStandArea` steps outward from it in 4-unit steps
up to 48, tracing down at each step, and returns the first floor point that is in a real area. That point,
not the mount, becomes `pointA` and both gate-A corners — it is where the bot stands to start the climb.

`PM_Ladder` is a **point-content test**, not a trace: the engine asks whether the player origin is inside
the ladder brush expanded by the current player hull. There is no "which face did I touch", so the whole
pass works off the hull-3 clipnode volume, never `LUMP_FACES` (which is not even loaded, and is empty for
`func_ladder` anyway once ZHLT has nullified the trigger texture).

Everything runs in hull 3, like the rest of navgen, so hull-3 leaf space *is* nav space and no z-adjust is
needed. Ducking enters only when the dismount area is crouch-only: the connection gets
`CONN_FLAG_NEEDS_CROUCH` and is costed at the ducking climb speed.

Per ladder:
1. **Extract the volume** - walk `model.headnode[kHullDuck]` for `CONTENTS_SOLID` leaves and build a
   polytope per leaf. Face normals from `BuildPolytopeFromChain` point *inward*, so the outward normal
   the player stands against is `-face.plane.normal`.
2. **Keep climb faces** - `|normal.z| <= kFloorNormalZ`. Near-horizontal caps are skipped, which is what
   stops the top cap being treated as a mount surface.
3. **Search the face for a free probe** (`FindFaceProbe`). It tries the face centroid inset 2 units into
   the volume first — the historical probe, so a normal ladder behaves exactly as before — then sweeps
   along `perp` in 8-unit steps out to the winding's extents, trying insets of 2, 1 and 0.5 at each
   offset, and takes the first point a duck hull fits in. A face with no free point anywhere is rejected,
   which is still what buries the three faces of a wall-mounted ladder that face into the wall.

   The sweep is not belt-and-braces: a thin ladder plate flush against a wall leaves a capture band only a
   unit or two deep, and the centroid can miss it in *both* axes at once. test53_ladder4's shaft ladder is
   a 1-unit plate on the `y = 0` wall of a shaft whose walls sit at `x = 304` and `x = 384`. Its climbable
   face centroid is `(320, 17, -24)`: `y = 15` after the inset is behind the wall for a duck hull, and
   `x = 320` is exactly on the hull-clip boundary of the `x = 304` wall. Both fail. `(328, 16.5, -24)` —
   one perp step over, half a unit in — is free, and is the only reason that ladder produces connections
   at all. Without it the map lost 7 areas to the void prune and the target became unreachable.
4. **Trace down** along the face-projected down vector (so a slanted ladder follows its own slope) to
   `2 * kStandProbeLift + PlainJumpHeight(gravity)` = 81 units below the expanded volume bottom. The bot
   is *standing* when it jumps for the rung, hence 18 + 18 + 45 rather than a bare 45. The hit point is
   the mount; no floor in reach, or a mount in no area, rejects the face.
5. **Trace up** from the mount to the expanded top, no overshoot. Whether the climb ended because the
   ladder ended or because a ceiling blocked it decides if the forward dismount is available.

### Dismounts
Four ways off, scanned from the top downward in 16-unit steps, with the standing reach height inserted
into the list in descending order. Sides and jump-off apply at every height;
**forward applies only at the top, and only when the ladder ended** — you cannot stop mid-ladder and step
forward. test29_ladder3 is why the scan exists at all: its ladders hit the ceiling, so the top is where no
floor is reachable in any direction and the ledge it should serve is passed at mid-height.

| Dismount | Start | Initial velocity | Sim yaw / speed |
| --- | --- | --- | --- |
| forward | top position | `(0, 0, +200)` | toward `-N`, `maxSpeed` |
| left / right | face end along `±perp` | `±perp * 200` | same direction, `maxSpeed` |
| jump off | column at that height | `N * 270` | toward `N`, speed 0 |

Working the `PM_LadderMove` algebra for a player facing the face, `lateral` is exactly zero and the result
is **pure +Z at 200** — you cannot move in the `-N` direction while on a ladder, because that input is what
*becomes* the climb. Forward still reaches a ledge because the bot keeps rising ~25 units after leaving the
volume and air-accelerates sideways; `kAirAccelWishSpeedCap` buys ~15 units of drift, which is exactly what
gets it from test12_ladder's column at x≈354 onto the ledge at x≥368. Jump-off uses commanded speed 0 so
the sim does not air-accelerate the bot back into the ladder.

Each sim runs through `PM_MoveSimulator::SimulateAirborne`, which starts the shared `PmSim` mid-flight with
a given velocity instead of on the ground. It also starts the flight already flagged airborne, so the first
ground contact counts as the landing — a sideways step onto a ledge at the same height is a dismount, not a
slide.

Sampling is in hull 3 so that a dismount into a crouch-only area is found, but a sample a *standing* player
cannot occupy is discarded unless the destination area carries `NAV_AREA_CROUCH`. `StandFits` is a point
test of the sample lifted by `kStandProbeLift` against the hull-1 tree; rejections print
`[standing hull does not fit]` under `--dump-ladders`. The same test derives the climb's `reach`: step down
from the true top until a standing hull fits, rather than assuming a fixed offset below it.

Selection per landing area: **one** winner, ranked by lowest fall damage, then by whether the dismount is
reversible (below), then by distance from the landing point to that area's centroid. A dismount landing back
in the ladder's own bottom area is a **loopback** and produces nothing.

### Reversibility, and why it decides the gates
Gate A is the mount. Gate B is **where a descending bot gets onto the ladder** — not, in general, where an
ascending one steps off. Those coincide for a dismount that barely left the column and diverge for one that
flew, so `landingPos` carries the ascent's exit and gate B carries the descent's entry.

A dismount is **directly reversible** when it is not a jump-off *and* its landing lies inside the ladder's
hull-expanded volume, with `kStandProbeLift` of slack in z because a standing player's ladder volume reaches
18 units further in each z direction than the hull-3 one. Gate B is then the landing itself: the bot steps
back on from where it stepped off.

Otherwise the pass asks a second question — **can the bot walk off an edge of the landing area and fall onto
the ladder?** `EdgeDropReachesLadder` takes the landing area's drop edges from `FilterLaunchEdges`, samples
along each every 16 units, and simulates a plain run-off (no jump) at full, half and quarter speed against a
`PM_MoveSimulator` carrying **only this ladder**. A run that trips `grabbedLadder` makes the connection
two-way, and gate B becomes the ledge point it launched from. Grabbing cancels fall damage, so the drop is
free however far it falls — the only thing that has to be true is that the bot ends up inside the volume.

This applies to every dismount kind, not just jump-offs. `PM_Ladder` is a point test, so the yaw the bot
runs at has no bearing on whether it catches the ladder — it only has to arrive. Each sample therefore tries
two yaws: toward the nearest point of the ladder's volume in xy, and `climbYaw`. The first is what makes a
**side** dismount reversible, where the bot has to come at the column from the side and the two yaws are 90°
apart. test53_ladder4's `6 <-> 4` is that case: the dismount steps off sideways to (428.7, 30, 19.2), and the
bot gets back by walking to (370, 30.8, 19) and running west at yaw 180 — a head-on run at `climbYaw` would
have taken it along the face and never into it.

Because the run-off yaw and the climb yaw are then different angles, the descent needs both. The climb yaw
is `exitAngle`; the run-off yaw goes in `gateAYaw`, which is dead weight on a ladder — it parameterises an
air move's launch along gate A and nothing reads it for another type. The planner hands them to the mount
waypoint as `ladderYaw` and `ladderEntryYaw`.

Candidates are ranked by **time to the grab**, not first hit. Without that the scan's edge order decides,
and it picks whatever it stumbles on: test53_ladder4's `6 <-> 4` first resolved to a 161-unit fall down the
side of the pit that happens to clip the column near the bottom, instead of the 18-unit step off the lip
directly above it. Shortest grab is the closest, safest, and least sensitive to the follower drifting.

Both tests are conservative in the same way: the connection only goes two-way when a simulation shows the
bot ending up on the ladder, so it never invents a route. What stays one-way is a landing the bot can reach
but cannot fall back to the ladder from, and on the test maps that is always because the drop would have to
go *upward* — test53_ladder4's `6 -> 9` and `4 -> 7` both dismount onto the pit floor at z −238, below the
bottom of the ladder they came off. test29_ladder3's `4 -> 2` is the other shape: a ledge that simply does
not overhang the column. Its other two ladders are the drop case, and that is what lets it descend into the
pit instead of taking a 256-unit fall for 10.5 damage.

### What the connection carries
No new record fields. `launchPos` is the mount, `landingPos` the dismount, `exitAngle` the yaw the bot holds
to climb, `exitSpeed` the climb speed, `flightTime`/`fallDamage` come from the winning dismount, and the
climb length is `|landingPos - launchPos|`. Gates are degenerate points: gate A at the mount, gate B at the
descent entry (above), which is why `CONN_LADDER` is exempt from both `BuildConnectionGates` and the
missing-gate counter. Gate B is the only place the descent entry lives, so the planner reads its point
rather than `landingPos` when `fromId` is not `areaA`, and `gateAYaw` is reused for the yaw it runs at.

### Scope
Ladder connections count for `PruneUnreachableAreas` and are expanded by A\* like any other connection —
both graph builders (`main.cpp` and `src/nav/nav_path.cpp`) now feed `CONN_LADDER` straight through. The
planner turns one into a mount/dismount waypoint pair; see `doc/nav.md` for that half. The **follower**
is still missing: the bot has no `MOVETYPE_FLY` awareness and its pitch is hard-zeroed, so a plan that
contains a climb waypoint is not yet executable.

`--dump-path` shows the climb on test12_ladder, test25_ladder2, test29_ladder3 and test53_ladder4.

The sampler refuses a flight that grabs a ladder (see `PM_MoveSimulator`), but the planner's launch
re-simulation cannot: `JumpLandsIn` runs on `PlanTraceContext`'s trace functions, and the `.nav` carries no
ladder volumes, so a re-simulated launch from a different point on the gate can still fly through one.
Closing that needs the volumes in the nav format.

## airmove_prune.cpp

### PruneWalkableAirmoves
Runs between `ProbeAirMoves` and `SnapConnectionsToFloor`. For each `CONN_AIRMOVE` it asks what the same
two areas cost on foot, and drops the air move when walking is not much worse:
`walkSeconds < kDefaultAirmoveWalkRatio * airSeconds`, ratio 1.2, tunable with `--airmove-walk-ratio`
(0 disables the pass). `--dump-airmove-prune` prints the comparison per connection.

Both sides are measured the same way — centroid to centroid at `maxSpeed`, plus `ConnectionCostSeconds`
— so the air move is paying for its `flightTime` and fall damage against a walk that pays only distance.
The walk query is a Dijkstra over **walk connections only**, memoised per source area.

Two properties make this safe to run as a single pass. Reachability cannot change, because a connection
is only dropped when a route between the same endpoints already exists — **in every direction the
connection itself offered**. And the result does not depend on the order connections are visited, because
the judging graph never contains the edge type being judged, so dropping one cannot alter any later query
— no fixed-point iteration needed.

Both halves of that first property need care, and getting either wrong silently orphans areas:
- The graph is **directed**. A one-way connection contributes one edge, not two. Walk connections are
  always bidirectional, so this was latent until `PruneRedundantLadders` put air moves in the graph:
  on test53_ladder4 two one-way drops *out* of a crouch tunnel were being traversed backwards, which
  made a 64-unit climb look free and deleted both ladders that were the only way *in*. The tunnel and
  the room behind it — with the map's `info_target` in it — then pruned away as void.
- A **bidirectional** connection is only dropped when the alternative works both ways, so the query takes
  the worse of `A->B` and `B->A`.

**A jump the planner prefers is not automatically a jump worth having.** A\* ranks edges on crossing
distance plus `ConnectionCostSeconds` and has no idea what the approach corner will cost, so it happily
picks jumps the profile then rates slower than walking: on `test37_walkent` the 5→4 crouch-jump route
came out at 3.5 s against 3.2 s for walking round the pit, over an identical 888 units. That connection
is the one this pass drops (walk 0.81 s vs air 1.43 s).

Most of what goes is stairs and slopes, where the sampler finds a jump between nearly every pair of
treads: `test15_staircase` 33 air moves to 7, `test50_stairs` 20 to 1, `test28_slopecase` 19 to 8,
`test45_slopes2` 17 to 8. Areas, walk connections and route existence are untouched. The cost is real
and shows up as slower routes where the discarded jumps were genuine shortcuts that simply did not beat
walking by the ratio. That trade is what the ratio buys, and it is the knob to turn if those maps matter
more than the graph size.

One-way drops survive on the no-walk-route branch, which is most of what is left: 21 of `test37_walkent`'s
30 air moves fall into a pit with no way back out, and no walk route means the air move is load-bearing.

### PruneRedundantLadders
A second phase over the same machinery, enforcing **walk > jump > ladder**. It judges `CONN_LADDER`
against a graph of walk connections *plus* the air moves phase 1 kept, and drops the ladder when
`altSeconds < kDefaultLadderRatio * ladderSeconds` — ratio 2.0, tunable with `--ladder-prune-ratio`
(0 disables). A ladder gets a looser ratio than an air move because climbing is slow but reliable: it
only has to be within 2× of the alternative to be worth keeping.

This is why `BuildLadderConnections` runs *after* `ProbeAirMoves` rather than before. Emitting ladders
first would let them suppress jumps through the sampler's `existingPairs` set, which is the opposite of
the intended priority.

Both safety properties carry over, but only because the graph is directed and bidirectional connections
are checked both ways — see the caveats under `PruneWalkableAirmoves`, which this phase is what exposed.
On `test53_ladder4` it drops 2 of 6: the degenerate 3-unit "climb" and a horizontal jump-off that walking
already covers. The two long climbs and the two crouch-tunnel ladders are all kept, the latter because
nothing else reaches the tunnel at all.

## floor_support.cpp

### SplitLeavesByFloorSupport
For each leaf:
1. Take every floor face (normal.z > 0.7)
2. Probe 1 unit down from each → carves against world + blockers
3. Cut planes from seams between supported/unsupported (vertical through edge midpoints, outward from supported centroid, check 2 units beyond for void)
4. Fragment leaf by cut planes
5. Fragments with supported floor become leaves; the rest go to `AbsorbFloorlessPieces`
6. Outputs stats: split, absorbed, dropped, steep-bottomed

### Floorless Fragment Absorption
The air above a staircase tread is routinely cut off at the *neighbouring* step's height: hull-3 expansion
leaves a step-shaped ceiling, and the clipnode plane that caps one tread runs straight over the next one, so
the column above tread N gets a bottom face floating 8 units up. Deleting it as unsupported is what used to
shred staircases — the tread was left as an 8-unit sliver with no standing room and no faces touching its
neighbours.

So a fragment with no supported floor is not dropped, it is handed *down*. `AbsorbFloorlessPieces` looks for a
leaf whose top face the fragment rests on and whose union with it is convex, verified by comparing the merged
volume against the sum. If the fragment overhangs several steps, `SplitAgainstHostColumn` carves off the column
directly above one host — clipping only by that host's side planes, and only by planes the fragment is not
already inside, since `ClipPolytope` would otherwise append a duplicate coplanar face and corrupt `Volume()` —
absorbs that, and returns the remainder to the pool. Whatever cannot be placed under any leaf is dropped.

Absorption is strictly downward: the shared facet must be the host's top and the fragment's bottom. Sideways
absorption looks superficially reasonable and is badly wrong — a floorless band beside a pyramid's apex
plateau will happily union with it and produce a full-width phantom area with floor over 8 units of an
80-unit span.

### ComputeSupportedFloor
Vertical probe from floor face downward against clip hull + blocking volumes.

### Edge Coverage Recording
Records the stretch of the edge a + ab * t, t in [0, 1], that the segment c -> d lies along. Only collinear segments cover anything; ones that merely cross the edge's line contribute nothing.

### Edge Support Classification
True when the whole edge either abuts one of `peers` or runs along the leaf's own floor boundary, i.e. no part of it is an edge the leaf has to be cut along. The supported windings tile the floor exactly, so whatever lies past an uncovered stretch belongs to some other class of floor by construction - no probing needed. Coverage is per-stretch rather than per-edge because one edge is routinely both: the strip of upper floor beside a ramp opening meets its neighbour along part of the edge and overhangs the ramp along the rest, and only the overhanging part must be cut away.

### Duplicate Cut Plane Prevention
Two support patches that meet along an edge each want to cut there, once from either side. Clipping a polytope twice by the same plane leaves it with two coincident faces, which double-counts in Polytope::Volume, so a plane goes in the cut list only once regardless of which side asked for it. `ClipPolytope` appends its cross-section face unconditionally, so clipping by a plane a polytope already has a face on produces the same duplicate — callers that clip against another polytope's planes must skip the ones it is already inside.

### Support Probe Direction
Straight down, not along the face normal. PM_WalkMove drops the hull vertically by stepsize to step up, PM_AirMove drops it vertically by 1 when falling. The probe matches that: dest[2] += stepsize to step up, dest[2] -= stepsize to drop, wherever the floor is tilted. Probing along the normal would drop the patch along its slope, sliding 12.7 sideways, testing solidity in the wrong place. The probe starts at the already dropped patch, so take the next step from there.

### Lowest Floor Face Selection
Every floor face of the leaf, not just the lowest one. A leaf beside a ramp holds both the flat ground at the ramp's foot and the ramp itself, and taking the lowest picks the ground - which is real floor everywhere, yields no cuts, and leaves the ramp face untested however far it extends past the surface it belongs to.

### Patch Cutting
A supported patch only has to be cut out where it stops being supported, so an edge it shares with another supported patch is a seam and stays uncut. There is no such thing as a patch supported by ground a step below any more: a floor face is supported where solid is directly under it and nowhere else, and the space over the ground below is reunited with that ground by absorption rather than by inventing floor at the upper height.

## stand_classifier.cpp

### SplitLeavesByStandClearance
For each leaf:
1. Probe stand hull (lift = duckClipMinZ - humanMinZ = 18 + epsilon) from every floor face
2. Compute standable area fraction over all of them together
3. ≤1% or no floor → crouch-only
4. ≥99% → standable
5. Partial → cut at standable/crouch boundaries (vertical planes through standable winding edges), classify each
   fragment by centroid probe

### Every Floor Face, Not The Lowest One
The same trap `SplitLeavesByFloorSupport` fell into, and for the same reason. Hull-3 expansion routinely leaves
the flat ground at a ramp's foot and the ramp itself in one convex leaf — the region above both is the
intersection of two halfspaces — so a leaf's lowest floor face is the ground, which has all the headroom in the
world. Classifying off it alone reports the leaf fully standable, no cut is made, and the ramp's high end keeps
whatever clearance the ground had.

test51_pyraduck is the map that exposed it: a 45° pyramid under a ceiling 48 above its apex, so the top 24 units
of every face are crouch-only. Three faces came out right because the BSP happened to split their slope away
from the surrounding floor; the east face arrived as one leaf holding both, and its crouch cap vanished. With
all floor faces classified the four faces are identical — same area volumes, same 24-unit band.

Which face a standable winding came from matters when deciding whether its edge is a cut: `OnWindingBoundary`
has to test against that face's own winding, or the ramp's bottom edge (a real face boundary) is mistaken for a
crouch/stand boundary and cuts the leaf in half for nothing.

Cut planes are deduplicated with `HasPlane`. Two standable windings that meet along an edge each want to cut
there, and clipping a polytope twice by the same plane leaves it with coincident faces and a `Polytope::Volume`
that silently double-counts — on test51 that inflated leaf volumes by up to 15% before the dedup went in.

`ClassifyPolytope` likewise probes every floor face of a fragment and calls it crouch if any of them is
crouch-only. Cutting is supposed to make fragments uniform; where it does not quite, ducking in a spot that
allows standing costs nothing, while standing in a spot that does not wedges the bot.

## reachability.cpp

### FindSpawnAreas / FindTargetAreas
- Spawns: info_player_start, info_player_deathmatch, info_player_coop, info_player_teamspawn
- Targets: info_target
- Parses "origin" key, finds containing area (boundPlanes test, ε=1.0), falls back to nearest centroid

### PruneUnreachableAreas
BFS from seed areas over connection graph (respects one-way). Remaps area IDs to compact 0..N-1. Filters connections.

## clipnode_tree.cpp

### WalkClipnodeTree
Recursive traversal collecting plane chain (planeIndex, front/back) for each leaf. Calls callback for CONTENTS_EMPTY (and CONTENTS_WATER if includeWater).

### CarvePolytopeAgainstTree
Recursive clipping with subtree classification (memoized):
- All-removed: discard
- All-kept: keep if volume > minPieceVolume
- Mixed: clip by plane, recurse both sides

### TraceHull / TraceHullWithEntities
GoldSrc RecursiveHullCheck:
- Traverses clipnodes from headnode
- Plane test at each node → recurse front/back or split
- On solid leaf: backtrack along fraction to find exact contact
- Entities: trace each blocker in local space, keep earliest hit

## blocking_entities.cpp

### Carved vs passable
Two classes of brush entity, handled oppositely:

- **The permanently solid brush entities** are returned by `ExtractBlockingVolumes` and **carved** out of
  the world leaves — the space simply ceases to exist. `IsBlockingEntity` lists them:

  | classname | carved when |
  | --- | --- |
  | `func_wall` | always |
  | `func_healthcharger`, `func_recharge` | always |
  | `func_conveyor` | `!(spawnflags & 2)` — `SF_CONVEYOR_NOTSOLID` spawns it `SOLID_NOT` |
  | `func_wall_toggle` | `!(spawnflags & 1)` — `SF_WALL_START_OFF` spawns it `SOLID_NOT` |

  The two conditional ones are conditional because the spawnflag is the difference between a wall and a
  decoration, and carving a decoration deletes a route that exists. `func_detail` is not listed at all: the
  compiler folds it into world geometry, so it never reaches the entity lump.

  **The chargers are why this list is not just `func_wall`.** A wall charger protrudes a few units out of
  the wall it hangs on, and with only `func_wall` carved the mesh had no idea: on crossfire the charger at
  `(448,384,-1504)…(456,416,-1456)` sat in front of a wall at x=448, and the area in front of it started at
  x=464 — the wall plus the 16-unit hull, with nothing subtracted for the charger. Every consumer then
  inherited that lie. `ResolveChargerArea` clamped the charger centre into that area and produced a service
  point at x=464, where a bot's hull would be 8 units inside the brush, so the bot could never arrive; and
  any route hugging that wall planned straight through the charger. Carving moves the area edge to x=472 and
  both problems go away at the source. See `doc/bot.md` for the runtime half of that story.
- **`func_door`, `func_door_rotating`, `func_breakable`** are not, so `ExtractPassableVolumes` returns them and
  they **split** the leaf they sit in. The interior piece survives as its own area tagged `NAV_AREA_DOOR` or
  `NAV_AREA_BREAKABLE`, carrying activation, activator position and opening time.

Carving them was the old behaviour and it disconnected the graph: every door map lost most of its areas
(test9_door kept 3 of 9) because the doorway became a hole between two components.

### ExtractBlockingVolumes
- Parse model index from "model" (*N)
- Get model bounds, expand by hull size (human: ±16 XY, -36..36 Z; duck: ±16 XY, -18..32 Z for clip hull)
- Translate by entity origin
- Return BlockingVolume {headnode, origin, mins, maxs}

### ExtractPassableVolumes
Same volume extraction, plus the classification that decides how the bot gets through. Returns a `PassableSet`:
entities it can classify go in `passable`, ones it cannot go in `impassable` and join the carve list.

- **No `targetname`, not trigger-only** → `ACT_NONE`, walk straight through. Trigger-only is
  `SF_BREAK_TRIGGER_ONLY` (1) for breakables, `SF_DOOR_USE_ONLY` (256) for doors.
- **Unnamed use-only door** → `ACT_BUTTON` with the door as its own activator (walk up, press use).
- **Named** → `CollectActivatorsFor` gathers **every** `func_button` / `func_rot_button` and **every**
  `trigger_multiple` / `trigger_once` whose target reaches the entity, directly or through one
  `trigger_relay` / `multi_manager` hop. Each becomes an `ActivatorVolume` holding the **raw** model AABB
  (unexpanded — button range and trigger touch need different expansions) plus its model centre. The area's
  `activation` is `ACT_BUTTON` if any activator is a button, else `ACT_TRIGGER`.
- **Named, no activator at all** → impassable, carved like a wall so whatever is behind it prunes out. That
  is deliberate: pruning then reports honestly that there is no way through. A door whose activators exist but
  are unreachable is *not* carved — it keeps its area and gets blocked detour entries instead.
- **`SF_DOOR_START_OPEN` / `SF_DOOR_PASSABLE`** never occupy their brush space, so neither carved nor split.

Opening time, from the real SDK (`external/hlsdk-full`; the vendored `external/hlsdk` is headers-only):

- `func_door` — `travel / speed`, where
  `travel = Σ|movedir.k · (size.k − 2)| − lip`, speed defaulting to 100. The −2 per axis is
  `CBaseDoor::Spawn` compensating for the engine expanding bboxes by 1 each way.
- `func_door_rotating` — `distance / speed`, defaults 90° and 100°/s.
- `func_breakable` — `health * 0.1`, a guess at how long the bot spends breaking it; health defaults to 1.
  This is a plan-time estimate only. navgen splits out every breakable, including ones the bot cannot
  actually destroy: `takedamage` is set in `CBreakable::Spawn` and `matUnbreakableGlass` is private data,
  so neither is visible in the entity lump. The runtime engages only at `health <= 100` and gives up on
  anything that takes no damage, penalizing the connection so the planner routes around it next time.

### Which passes see a passable as solid
- **Floor support** does — a door brush still holds up whatever rests on it.
- **The leaf carve, stand clearance and air-move sampling** do not. The bot opens or breaks it, so a jump may
  pass straight through; and leaving doors in the stand test would classify every door interior as crouch-only.

### SubtractBlockingVolumes
For each overlapping blocker:
- Translate leaf polytope to blocker local space
- Carve against blocker's clip tree (remove CONTENTS_SOLID)
- Translate pieces back
- Keep the piece whole when the carve reports it removed nothing

`CarvePolytopeAgainstTree` sets `removedAny` when a piece with real volume lands in a removed-contents
subtree. Without that flag the carve still fragments a piece along every brush plane it straddles even when
the brush takes nothing out of it, because the tree's node planes are unbounded and the bounds test only
compares boxes. On `disposal` that was 655 of 905 carves and 1091 of the 1373 extra pieces; `test13_break`,
with six `func_breakable` and no blockers at all, spent 22 of its 30 passable splits that way. The pieces
are geometrically identical to their parent, so keeping the parent is exact and strictly coarser.

**This is the carve that made the four 25th-anniversary maps unbuildable.** Unlike the ladder carve it uses
the clipnode tree, so it hit the escape-to-map-radius bug described under "func_ladder" above: a piece whose
faces degenerate below three vertices loses the plane that bounded it, and the next clip rebuilds its cut
face from `BaseWindingForPlane(plane, mapRadius)`. On `disposal` 439 of 727 carves came back with **more**
volume than they were given, the worst by 7.3e10x — one 8-face 3,090,431 cu.unit leaf carved into a
**2-face** piece measuring 198,070,724,914. Two faces cannot bound a solid, but `Polytope::Empty()` only
tests `faces.empty()` and the `Volume() > kMinPieceVolume` filter *keeps* such a piece precisely because its
bogus volume is enormous. Its bounds then span the map, so it overlaps every blocker and every door, each of
which splits it again: total leaf volume went 3.93e9 in to 3.70e12 out, and the passable split that followed
climbed past 1.2M leaves until the 32-bit navgen ran out of address space. `ClipPolytope` now returns an
empty polytope when a clip leaves fewer than `kMinSolidFaces` faces, which fixes the ladder case at its root
too. With the guard `disposal` carves 340 leaves into 372 pieces (was 1658), worst leaf 4 pieces (was 247),
and builds in 75s at 26MB.

### SplitByPassableVolume
Same carve run twice against the passable's clip tree — `CONTENTS_SOLID` removed gives the pieces outside,
`CONTENTS_EMPTY` removed gives the piece inside. Every leaf-splitting pass downstream
(`SplitLeavesByFloorSupport`, `SplitLeavesByStandClearance`) must forward `ExtractedLeaf::passableIndex` or the
door loses its identity the moment its leaf is split again.

### Door Angle Convention
`SetMovedir` (hlsdk `subs.cpp`) is the forward vector of the **full angles triple** — up is `(-90 0 0)`, down
`(90 0 0)`, east `(0 0 0)`, north `(0 90 0)` — plus two legacy sentinels the engine still honours,
`angles == (0 -1 0)` → up and `(0 -2 0)` → down. It is not a yaw-only convention.

**The triple may not be under the `angles` key.** Valve's own maps write the legacy scalar `angle` instead,
which the engine widens to `(0 value 0)` — that is why the sentinels live in `.y` at all. `EntityAngles` reads
`angles` first and falls back to `angle`, and every movedir consumer (`DoorTravelDistance`, `ClassifyMover`)
goes through it. Reading only `angles` yields the empty-string default `(0 0 0)`, i.e. movedir `(1 0 0)`: a
vertical lift then looks like a door that slides east. All four `crossfire` lifts (`lift1`..`lift4`, `angle
-1`) failed `ClassifyMover` that way with `movedir is not axis-aligned vertical`, and with no elevator
connection to ride, the whole `z = -1518` roof ring — 50 areas, 18 spawn points, 17 items — became a one-way
trap: reachable from a spawn, so never pruned, but with no edge leading back up into it.

### Door/Neighbor Connection
Both halves come out of the same clip planes, so their floors tile without a gap and `CollectFloorSeams`
connects the door piece to its neighbours with no extra linking. This is the same property prism merging relies
on, and it is why splitting needs no new linking code at all.

## elevators.cpp

### What an elevator is, and why it is not a door
A vertical `func_door` is the classic HLDM lift: you stand on top and it carries you. Modelling it as a door —
something you wait for and walk *through* — produces a wrong route rather than a missing one. On
`test18_elevator` it used to yield a 16-unit slab tagged `door(button, 2.38s)` under the parked platform, and
the pit column's floor became the parked platform top instead of the real pit floor.

### Classification
Two tests, both required; failing either leaves the entity on the normal door path.

1. classname `func_door` (not `func_door_rotating`) or `func_plat`, and `|movedir.z| > kFloorNormalZ` with the
   other two components under `kElevatorAxisEpsilon`. Pure translation only — `TraceHullWithEntities` translates
   a blocker by its `origin` and cannot rotate a clipnode tree, so a rotating mover is not expressible.
2. footprint at least `kElevatorMinFootprint` (48) in **both** horizontal axes.

The movedir test alone separates nothing: every `func_door` in `test9_door`, `test10_doorbutton`,
`test11_doorbutton2` and `test20_doortrigger` also has `angles "-90 0 0"` and moves straight up. The four test
doors are 16 × 64 and are rejected on footprint. A 48-unit floor in both axes is not a doorway.

### Stops
`func_door` — travel from `DoorTravelDistance`, ride time `travel / DoorSpeed`. `SF_DOOR_START_OPEN` makes the
moved position the spawn position (hlsdk `doors.cpp` swaps `m_vecPosition2` with `m_vecPosition1`). Dwell at the
moved stop is `wait`; `wait < 0` or `SF_DOOR_NO_AUTO_RETURN` means it never leaves on its own.

`func_plat` — the brush as compiled *is* the top position (hlsdk `plats.cpp`: "plats are always drawn in the
extended position"). Bottom is `origin.z - height`, or `origin.z - size.z + 8` when `height` is 0. Speed
defaults to 150. A **named** plat spawns at the top and is brought down by its button; an **unnamed** one spawns
at the bottom with its own auto-spawned touch trigger. Dwell at the top is 3 s unless `SF_PLAT_TOGGLE`.

Dwell is keyed to the mover's **position**, not to which stop is the default. `CFuncPlat::HitTop` waits at the
top whether or not the top is where it spawned, so a named plat (default = top) and an unnamed one
(default = bottom) both dwell at travel 0. Keying it to default-ness gets the unnamed case backwards and
silently drops the ride-down edge. A door is the other way round — it dwells at its *opened* position — which is
its non-spawn stop either way, including under `SF_DOOR_START_OPEN` where hlsdk swaps the two positions.

### A stop you cannot summon can be left but not entered
An unnamed `func_plat` has no activator in the entity lump — its trigger is auto-spawned at runtime — so its
raised stop is `ACT_NONE` and nothing gates walking onto it. That would let a bot walk out over a shaft with the
platform parked at the bottom. A `CONN_WALK` is one bidirectional record, so dropping it would take the way
*off* the platform with it, which is the seam that matters once you have ridden up. `BuildElevatorConnections`
therefore orients it instead: `CONN_FLAG_ONE_WAY` with the stop as `areaA`, so the seam can only be walked
outwards. `reachability.cpp` and `nav_path.cpp` both honour the flag, so a stop nobody can summon still does not
become reachable through that seam. Nothing summons this stop, so the seam can only be walked outwards -- stepping in from the far side is stepping into the shaft. Dropping the record would take the way off with it. Separate from this are the walk edges dropped *between* two stops of one
mover, which the step filter usually kills first but which are removed structurally.

### Geometry: a stop area is the space the mover floors
All in nav (duck-clip origin) space. Everything needed is per stop and already built by `ExtractElevators`:

- `duckVolume` — the mover's model at that stop's offset, expanded by the hull, traced through the model's own
  clipnodes. `MoverVolume` builds it; its AABB is the model bounds `±16` in XY, `±18` in Z.
- `topZ` — `origin.z + model.maxs.z + travel + kDuckClipHullMaxZ`, the top of that body. `slabTopZ` is the next
  stop's `topZ`, or `mapRadius` for the highest, which is what keeps two stops of one mover from overlapping —
  on `test18_elevator` they meet exactly at nav 16.

`SplitLeavesByElevatorStops` runs **before** `SplitLeavesByFloorSupport` and is the only place a mover touches
the leaves. Per leaf, per stop, `ClaimStopFloor` takes the space between `topZ` and `slabTopZ` whose floor is
the mover and not the world:

- the candidate is the leaf clipped to `duckVolume`'s AABB in XY and `[topZ, slabTopZ]` in Z;
- `ComputeSupportedFloor` on its floor face, once against the world plus every non-elevator blocker and once
  against `{duckVolume}` alone (with `CONTENTS_EMPTY` as the headnode, so the world contributes nothing), says
  which of the two holds each part of it up. True when the only thing holding this piece up is the mover parked at this stop. CONTENTS_EMPTY as the headnode makes the world contribute nothing, which is how the mover is asked in isolation.
- where those disagree, `VerticalPlaneThroughEdge` and `ClipPolytope` cut the candidate along the boundary;
- mover-floored pieces are claimed and tagged `elevatorStopId`. Anything the world already floors stays behind,
  and if nothing at all is claimed the leaf is left whole rather than fragmented. Takes from `piece` the space between this stop's top surface and the next stop's, wherever the mover is what you would be standing on. Anything the world already floors at that height stays behind in `keep`, which is what lets a stop flush with a landing keep the landing as ordinary ground.

Only then is the **default** stop's body subtracted from what remains, and only from pieces that strictly
overlap it — `OverlapsBlockingVolume()` counts a shared face as an overlap, and carving against a tree the piece only
touches still splits it along that tree's planes. The claim leaves pieces flush against the mover, so the
parked-body subtraction needs a strict test or it fragments them. Claiming before subtracting is what lets a
lift parked at its *top* stop still produce a lower stop area; the general blocker carve in `main.cpp` runs on
`airmoveWorlds.baseDuck`, which is every blocker except the elevator volumes, precisely so this stage can own
the order.

The movers carve their own leaves in SplitLeavesByElevatorStops, which has to claim the stop
floors before the parked body is subtracted -- otherwise a lift parked at its top stop swallows
the region its lower stop needs. baseDuck is `blockers` without any elevator volume.

The second half of the rule is what a flush landing needs. `test55_plat`'s raised stop is level with the ledge
beside it, and the mover's own footprint reaches 16 units into that ledge; the world floors that strip, so it
stays ordinary ground and walking onto the ledge does not require summoning the plat. The first half is what
makes the strip on the *other* side — 256 units of pit under it — part of the stop, gated with it, instead of a
free-standing ledge in mid-air.

Injecting as **leaves** rather than as finished areas is what makes the rest free: floor support, stand
clearance, `MergeAreas`, `BuildBoundPlanes`, `ValidateAreaFootprints`, `BuildWalkConnections`,
`FilterWalkConnectionsByStep`, `ProbeAirMoves`, `PruneUnreachableAreas`, `BakeConnectionGates` and
`BuildActivationDetours` all run unmodified. In particular `BuildWalkConnections` re-derives every seam with
`CollectFloorSeams` on the final area polytopes, so a stop area gets its walk connections with no linking code —
on `test18_elevator` the raised stop's floor (nav 16) and the corridor slivers' floors (nav 18) share the planes
`x = 144` and `x = 368`, and the 2-unit step survives the step filter. On `test55_plat` the same machinery finds
the 8-unit step from the pit floor at nav `-238` onto the parked plat at nav `-230`, which is how a bot boards.

### Blocker lists
- `blockers` / `standBlockers` get the **default stop only**, so traces see the map as it spawns. This is what
  makes a drop into the shaft land on the parked platform. The leaf carve uses `airmoveWorlds.baseDuck`
  instead — the same list *without* the mover — because `SplitLeavesByElevatorStops` subtracts the parked body
  itself, after claiming.
- `supportBlockers` gets **every stop**, or each stop's floor face would probe into air and
  `SplitLeavesByFloorSupport` would drop it.
- `ProbeAirMoves` gets neither. It takes an `AirmoveWorlds` — the base lists with **no** mover volumes at all,
  plus every stop's volume indexed by `Area::elevatorStopId` — and composes its own world as
  `base + one volume per mover`, which is why `BuildAirmoveWorlds` is called before the default stops are
  appended to `blockers`.

### Where the mover stands during sampling
The sampler cannot use one static world. A stop's floor is the boundary between its own volume and the area
below it, so making a raised stop standable necessarily seals the stop beneath it — on `test18_elevator` the
raised platform blocks nav `−260…16` and area 6 is `−222…16`, entirely inside it. Making every stop solid at
once would leave area 6 with no free space at all, delete the drop into the pit, and strand the route at the
call button. That is not an artifact: when the lift is up you genuinely cannot drop into the pit. A thin slab
at `topZ` fails the same way, and a swept volume is worse, being the union over a continuum.

So the world is a property of the **sweep**, not of the sampler. `PM_MoveSimulator` holds its blocker list by
reference, and `sweepArea` is the single lambda both the main loop and the reverse-seed loop go through, so
selecting a configuration there costs nothing:

- Sweeping an area tagged with a **non-default** stop selects that stop for its mover, recomposes the world,
  sweeps, and restores. Selecting replaces the mover's volume rather than adding one, so the composed world
  never has a platform in two places.
- Every other area sweeps in the default configuration, exactly as before. On a map with no elevators the
  composed world is the base list unchanged, and the whole sweep is byte-identical to the pre-`AirmoveWorlds`
  output across every map under `maps/`.

With the platform placed, `FilterLaunchEdges`, `IsSupported` and `Simulate` need no special cases — relaxing
`IsSupported` would not have worked anyway, since `Simulate` runs in the same world and the sampled player
would fall through the floor it launched from. `--dump-elevators` reports the flip per stop area, e.g.
`support 0 parked -> 4 placed`. `test18_elevator` stops there — its raised top is flush with both corridors, so
`GroundAhead` filters every perimeter edge — but `test55_plat` goes the whole way: its raised stop reaches
`x=112`, where 256 units of pit lie beyond, giving `2 launch edges, 508 samples` and a real `airmove 4 -> 7`
off the platform into the pit. That is the end-to-end case, and it only exists because the stop claims the
space the mover floors rather than a shrunken box.

Nothing is needed at runtime: `PlanWalker::JumpLandsIn` re-simulates every air move against the live engine
trace, so an edge that only exists with the lift raised simply fails to relax while it is parked.

### Gating and the ride edge
The default stop is real ground: `ACT_NONE`. Every other stop carries `NAV_AREA_ELEVATOR`, the mover's
activation, and `openSeconds` = the ride time — the cost of *summoning* the platform. Walking onto a non-default
stop therefore goes through the same `ActivationDetour` machinery a door uses, keyed the same way
(`FindDetour(toArea, fromId)`). On `test18_elevator` that is exactly right: entering the raised stop from either
corridor needs a route to the call button, the button is only reachable from the parked platform, the detour
comes back `blocked`, and `PlanWalker::Cross` refuses. You cannot summon that lift from the top.

`BuildElevatorConnections` runs after `FilterWalkConnectionsByStep` and before `ProbeAirMoves`, so the stop pair
is already in the sampler's `existingPairs` and no duplicate 238-unit lethal drop is sampled. It also removes any
`CONN_WALK` the seam collector built between two stops of one mover.

`CONN_ELEVATOR` carries `rideSeconds`, `rideSecondsMax` (equal to `rideSeconds` today; the hook for a future
swept-interval sampler) and `rideWaitSeconds`. Cost is `rideSeconds + rideWaitSeconds` — time, no distance. The
ride *toward* the non-default stop is gated by that area's activation and waits 0; the ride *back* is ungated and
waits the departure stop's dwell. A stop that never leaves on its own emits no return edge, which makes the lift
one-way in the mesh rather than silently wrong.

In `PlanWalker::Cross` the branch follows the teleport template: optional activation detour, a counted waypoint
carrying `requiresElevatorRide` / `elevatorTravelTime`, then `PushWaypoint(arrive, countLength=false)` so the
238 units never enter the funnel, then re-anchor.

### Known limitations
- **No air moves *onto* a non-default stop.** Launching off one works (see "Where the mover stands during
  sampling"), because the launch area is the stop itself and the sweep places the mover. Landing on one does
  not: the launch area is an ordinary corridor swept in the default configuration, so a jump toward the shaft
  simulates a fall straight past the raised stop down to the parked platform and `landingArea` resolves to the
  default stop. Correct for the world it simulated, but nothing can be found that lands on a raised stop. The
  fix is to sweep areas near a mover once per configuration, recording only pairs that touch that mover's
  stops so the default world stays authoritative elsewhere; `PlanWalker::Cross` already rejects a non-walk,
  non-ride entry into a gated stop, so such an edge would also need `BuildActivationDetours` extended past its
  walk-and-ride entry check.
- **A large ceiling-retracting door misclassifies.** With the footprint test as the sole discriminator, a
  vertical door 48+ units in both horizontal axes that slides up into a recess becomes an elevator: its sweep is
  carved and the upper stop area is unreachable inside the recess. A wasted area and a lost doorway, not a crash.
- **`test18_elevator` depends on its 2-unit flushness.** The raised top lands at nav 16 against corridor floors
  at nav 18, which is why the exits are walk seams. Change the `lip` and the map becomes an air-move case.
- **Auto-return dwell is not enforced.** `rideWaitSeconds` records that the platform leaves after `wait`
  seconds, but nothing prevents planning a top-side walk-off slower than the dwell. That is a runtime concern.
- **An unnamed `func_plat` cannot be summoned from outside.** `CollectActivatorsFor` only runs when `targetname`
  is non-empty ([elevators.cpp:183](../src/navgen/elevators.cpp:183)), so a plat summoned by its own implicit
  `CPlatTrigger` reports `activation none, 0 activator(s)` and every seam into its raised stop is exit-only.
  `test55_plat` does not need entering — pit → board → ride → alight — but a plat parked at its *top* stop is
  unreachable from below, which is what `topplat` (that map with a `targetname` patched in) shows: both stop
  areas and a ride-down edge exist, and nothing can call it. Modelling `CPlatTrigger` as an `ACT_TOUCH`
  activator over the mover's own column would close it.

## activation_detours.cpp

### Why precompute
An activator is rarely in the door's own area, and the route to it is a property of *(door, which side you came
from)* — static map geometry, so it belongs in navgen. Planning it at search time would mean a sub-A* per A*
expansion (the walker is copied per node), and putting activation in the search state means revisit waves.

The stage runs after `PruneUnreachableAreas` (which renumbers every area id, so `fromAreaId` would otherwise be
garbage) and after `BakeConnectionGates` (the polyline anchors on a baked gate midpoint).

### FindServicePoints
Where the bot has to stand to work an activator. Gated areas are never service areas — you cannot stand inside a
closed door to press the button that opens it.

- **Trigger** — touch is an AABB overlap of the player hull with the trigger's `absmin/absmax`, so the set of
  player *origins* that touch it is `[act.mins − hullMaxs, act.maxs − hullMins]`. Each floor winding is clipped
  against that box's four vertical planes; a survivor over `kMinServiceArea` yields its centroid, lifted to
  player-origin height, with the origin z checked against the box's z span.
- **Button** — GoldSrc `+USE` is `SV_FindEntityInSphere(pev->origin, 64)` against the brush model **centre**,
  then a facing dot test. It does **no trace at all**: pressing a button through a thin wall works in vanilla
  HL. So the test is 3D distance from the player origin to `act.center` ≤ `kUseRange` (56, leaving margin), and
  there is deliberately no line-of-sight check — an earlier one rejected the *common* case, since a
  flush-mounted button has its centre inside world solid. Because the range is 3D from `floorZ + 18`, a button
  high on a wall is out of reach from directly beneath it.

A button sample is also rejected when the player hull at that origin would overlap the activator brush itself
(`HullClearsBox`, the same origin-space box the trigger case uses). `func_button` is not in
`IsBlockingClassname`, so `StandablePoint` alone happily returns a point inside the button — and since samples
are ranked by *closest to the centre*, that unreachable point always won. The bot then jammed against the brush
short of the waypoint, `progressS` never advanced, and it stall-repathed forever.

Containment uses `boundPlanes`, never the polytope — areas can be non-convex prisms.

### BuildActivationDetours
Per gated area D and per **directed walk** connection `X → D`, `PlanRoute` runs from the X-side gate midpoint to
each candidate service point and back again (planned separately — gate string-pulling is not symmetric). The
cheapest pair becomes one closed polyline that starts and ends at that gate midpoint, with `actIndex` marking
the activation waypoint.

Non-walk entries into a gated area are rejected outright. They were silently broken before: the teleport, ladder
and air-move branches of `Cross` all return *before* the door check, so a jump landing inside a door area emitted
no activation waypoint at all.

**Bootstrapping** needs no flag. The planner rule is a lookup — "no detour for this neighbour ⇒ refuse the edge"
— so during the pass every gated area has an empty `detours` and seals itself automatically. Results go to a
side buffer and are written back only after the last `PlanRoute` returns, so nothing depends on door iteration
order. This is also the conservative no-recursion rule: a button behind another button-door reads as blocked,
and `--dump-activators` says so rather than hiding it.

The precompute graph is walk-only, which also means the stored polyline can carry no air-move/ladder/teleport
flags and `traces` can be `nullptr`.

**Known limitation:** the detour starts at the door's own gate, so a bot walks up to the door, turns round to
the activator, and comes back. Plausible, not optimal — diverging at the ideal point would need activation in
the search state.

## nav_file.cpp

### WriteNavFile
Serializes MergeResult to crossbot_nav format (v16):
- Header: magic "CBNV", version 16, mapName[64], bspChecksum, counts (including `detourCount` and
  `detourPointCount`), and `designMaxSpeed`
- `designMaxSpeed` is the `--maxspeed` the run was built with. Nothing navgen does reads it back; it is
  recorded because the runtime has no other way to know what speed the mesh assumes, and a bot faster
  than that has movement options no connection on this mesh describes (`doc/bot.md`,
  `bot_opportunist.cpp`)
- Area records: id, mins/maxs/centroid[3], areaFlags, connOffset/Count, planeOffset/Count, `activation`,
  `openSeconds`, and `detourOffset`/`detourCount`
- Activation detour records: fromAreaId, pointOffset/Count, actIndex, waitSeconds, detourSeconds,
  `actCenter[3]`, actKind, flags (`DETOUR_FLAG_BLOCKED`). Detour points: pos[3], areaId, clearance, flags
  (`DETOUR_PT_CROUCH`)
- `actCenter` is the activator brush model centre. It is the only thing the runtime cannot recompute: pressing
  needs the engine's own aim target, and the service point is where the bot *stands*, not what it looks at
- The detour offsets are `uint32_t`, unlike `connOffset`/`planeOffset` which are `uint16_t` into the global
  arrays and throw past 65535 via `ToU16Count`
- Every record is `fwrite`n raw, so each has explicit padding and a `static_assert` on its size — a new field
  that shifts the layout fails the build instead of silently corrupting nav files
- Area planes come from `boundPlanes`, not `polytope.faces` — see "Area geometry vs exported region"
- Connection records: areaA/B, connType/Flags, gateValid, costSeconds, travelSeconds, gates[2][2][3], gateAYaw/Speed[2], exitSpeed/Angle, jumpBackDist, fallDamage, flightTime, launchPos/landingPos[3]
- Area-connection index (uint32 per connection per area)
- Area planes (normal[3], dist)

## leaf_extractor.cpp

### ExtractHullLeaves
For each empty leaf from clipnode walk:
- For each plane in chain: create base winding, clip by all other planes in chain
- Non-empty windings → faces of leaf polytope

## polytope.cpp

### Core Geometry
- `Vec3` - 3D vector with +,-,*, Dot, Cross, Length, Normalized
- `Plane` - normal + dist
- `Winding` - vector<Vec3> (coplanar polygon)
- `Face` - Plane + Winding
- `Polytope` - vector<Face> (convex 3D polytope)

### Operations
- `BaseWindingForPlane(plane, radius)` - large quad on plane
- `ClipWinding(winding, plane, keepFront, epsilon)` - Sutherland-Hodgman
- `WindingArea(winding, normal)` - 3D area via cross sum projected to normal
- `ConvexHullXY(points)` - 2D Andrew's monotone chain
- `IntersectCoplanarPolygons(subject, clip, planeNormal)` - clip by clip's edge half-planes
- `Polytope::Volume()` - tetrahedra from centroid
- `Polytope::Centroid()` - vertex average
- `ClipPolytope(poly, plane, keepFront, radius, epsilon)` - clips faces, adds cross-section face, returns
  an empty polytope when fewer than `kMinSolidFaces` faces survive — a sub-4-face result cannot bound a
  solid, and `Volume()` reports a huge bogus number for one instead of a small one
- `TranslatePolytope(poly, offset)` - translates vertices, updates plane dists

## crc32.cpp

### BspChecksumLumps
CRC32 of concatenated planes + clipnodes + models lumps (standard GoldSrc BSP checksum).

## entity_parser.cpp

### ParseEntities
Parses GoldSrc entity lump (nested "key" "value" pairs in braces) into vector<Entity> (unordered_map<string,string>).

## svg_debug.cpp

### WriteDebugSvg - Run-up Traces
The plan-time run-up simulations, drawn over the route they were derived from so the two can be read against each other: where they diverge is where the mover could not hold the planned line.