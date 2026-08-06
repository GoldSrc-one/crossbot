# Navgen System Documentation

## main.cpp

### Entry Point & Debug Output
Command-line tool with extensive debug flags:
- `--dump-leaves` - per-leaf (pre-merge) id/faces/volume/centroid stats
- `--dump-areas` - per-area (post-merge) id/faces/volume/centroid/member-leaf stats
- `--dump-conns` - every final connection: type, flags, endpoints, air-move parameters
- `--dump-steps` - every walk connection the step filter trimmed or dropped: area pair, shallowest
  crossing, both floor heights, surviving walkable runs, each floor face's normal z and 2D offset
- `--dump-support` - every leaf fragment dropped as unsupported: floor centroid, floor normal, volume
- `--dump-airmove` - air-move launch derivation: why no funnel launch was found, and every
  re-simulation miss with its launch, yaw, speed and where it landed
- `--dump-planes <id>` - one final area's exact plane list (normal+dist) and face windings
- `--dump-obj <path>` - walkable floor of every area as OBJ mesh (+ MTL): crouch areas colored apart, planned route as ribbon
- `--dump-svg <path>` - top-down SVG of final area graph
- `--dump-path` - per-waypoint clearance and per-segment arc/line/gap rows (radius, turn, length,
  entry/exit speed, cap) for the debug route, plus the profile summary line
- `--trace <path>` - overlays recorded bot trajectory on SVG (one "x y z" per line, empty file = nothing).
  Also reports mean and max lateral deviation of the trace from the planned profile, and the clearance
  budget at the worst point — the headline number for whether the bot actually held its arcs.
- `--airmove-samples <n>` - launch samples per unit edge length (default 1; 0.5 halves; 0 disables)
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
  `--dump-airmove` prints each miss with its launch, yaw, speed and where it actually landed, and its
  count matches `rejected in search`
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
- walk touching a door or breakable area — that area's `openSeconds`
- air move — `flightTime` plus a fall-damage penalty, keeping the "lethal ⇒ effectively infinite" sentinel

Button and trigger doors get no extra penalty on top: the detour to the activator already costs its own travel
time, which is the whole point of expressing cost as time.

### Door/Breakable Isolation
A merge never crosses `ExtractedLeaf::passableIndex`. A door or breakable has to stay its own area so it keeps
its activation, and it must not absorb a different one either.

### Door Connection Logic
Both halves come out of the same clip planes, so their floors tile without a gap and the seam collector connects the door piece to its neighbours with no extra linking.

## airmove_sampler.cpp

### PM_MoveSimulator
Wraps shared PmSim core with navgen's trace function (world + blocking volumes).

### ProbeAirMoves
For each connection:
1. **Collect launch edges** - perimeter of floor faces (outward normal from face center)
2. **Seed drops** - from each edge, fan yaws ±45° around outward, 1.7x maxSpeed run-up
3. **Seed jumps** - same edges, add jump impulse, fan yaws ±90°
4. **Seed reverse jumps (climbs)** - from landing spread of drops, edge along landings, outward toward ledge
5. **Simulate each sample** - PmSim until land/wedge/timeout; check landing in dest area
6. **Bin by launch edge** - group samples by `SameLaunchEdge` (outward dot ≥ 0.999, plane offset match 0.1)
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

A normal perimeter edge fans yaw over the full `±yawSpread` (90°) around its outward normal, because nothing is
known about where the bot wants to go. A reverse seed does know: the climb has to retrace the drop, so it can
only usefully aim along the drop's own headings reversed. The seed therefore carries a `yawRange` — the circular
mean of the drop samples' inverse yaws, widened to the spread those samples actually covered — and the sweep
draws from that arc instead.

This matters more than it sounds, because `BuildSampledGates` **drops the whole connection, not just its gate**,
when `FitGate` cannot establish lateral spread. Fanning 180° at a narrow opening lands too few successes to fit
a gate, so genuinely traversable crossings disappeared: at the default sample density test43_vent lost both vent
crossings and pruned to 3 of 8 areas. With the seeded arc it is 8 of 8 with nothing unfitted, using slightly
fewer trajectories — every air-move map gained connections and none lost reachability.

### Sample Cloud Aggregation
Samples from every edge the pair was reached from land in one cloud. Which edge a sample left through does not matter: the robust fit keeps whichever cluster is the majority and rejects the rest as outliers, so the gate settles on the face the move is really about.

### Probe Variables
- `probeZ = 0.0` - edge filtering and the ground-support test happen here
- `launchZ = 0.0` - the simulated origin starts here
- `navZAdjust = 0.0` - added to simulated positions to express them in nav space
- `emitted` map - pair key -> whether the winning move needs a crouch

### Reverse-Jump Yaw Calculation
The climb has to retrace the drop, so its yaw only ever wants to be one of the drop's headings reversed. Take the mean of those inverse headings as a circular average, then keep the spread the samples actually covered.

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

- **`func_wall`** is permanently solid, so `ExtractBlockingVolumes` returns it and it is
  **carved** out of the world leaves — the space simply ceases to exist. `func_detail` is not listed: the
  compiler folds it into world geometry, so it never reaches the entity lump.
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

- **No `targetname`, not trigger-only** → `DOOR_ACTIVATION_TOUCH`, walk straight through. Trigger-only is
  `SF_BREAK_TRIGGER_ONLY` (1) for breakables, `SF_DOOR_USE_ONLY` (256) for doors.
- **Unnamed use-only door** → `BUTTON` with the door as its own activator (walk up, press use).
- **Named** → resolve through `func_button` (chasing `trigger_relay` and `multi_manager`) else
  `trigger_multiple`, giving `BUTTON` / `TRIGGER` and `doorActivatorPos` at the activator's model centre.
- **Named, activator unresolvable** → impassable, carved like a wall so whatever is behind it prunes out. That
  is deliberate: pruning then reports honestly that there is no way through.
- **`SF_DOOR_START_OPEN` / `SF_DOOR_PASSABLE`** never occupy their brush space, so neither carved nor split.

Opening time, from the real SDK (`external/hlsdk-full`; the vendored `external/hlsdk` is headers-only):

- `func_door` — `travel / speed`, where
  `travel = Σ|movedir.k · (size.k − 2)| − lip`, speed defaulting to 100. The −2 per axis is
  `CBaseDoor::Spawn` compensating for the engine expanding bboxes by 1 each way.
- `func_door_rotating` — `distance / speed`, defaults 90° and 100°/s.
- `func_breakable` — `health * 0.1`, a guess at how long the bot spends shooting it; health defaults to 1.

### Which passes see a passable as solid
- **Floor support** does — a door brush still holds up whatever rests on it.
- **The leaf carve, stand clearance and air-move sampling** do not. The bot opens or breaks it, so a jump may
  pass straight through; and leaving doors in the stand test would classify every door interior as crouch-only.

### SubtractBlockingVolumes
For each overlapping blocker:
- Translate leaf polytope to blocker local space
- Carve against blocker's clip tree (remove CONTENTS_SOLID)
- Translate pieces back

### SplitByPassableVolume
Same carve run twice against the passable's clip tree — `CONTENTS_SOLID` removed gives the pieces outside,
`CONTENTS_EMPTY` removed gives the piece inside. Every leaf-splitting pass downstream
(`SplitLeavesByFloorSupport`, `SplitLeavesByStandClearance`) must forward `ExtractedLeaf::passableIndex` or the
door loses its identity the moment its leaf is split again.

### Door Angle Convention
`SetMovedir` (hlsdk `subs.cpp`) is the forward vector of the **full angles triple** — up is `(-90 0 0)`, down
`(90 0 0)`, east `(0 0 0)`, north `(0 90 0)` — plus two legacy sentinels the engine still honours,
`angles == (0 -1 0)` → up and `(0 -2 0)` → down. It is not a yaw-only convention.

### Door/Neighbor Connection
Both halves come out of the same clip planes, so their floors tile without a gap and `CollectFloorSeams`
connects the door piece to its neighbours with no extra linking. This is the same property prism merging relies
on, and it is why splitting needs no new linking code at all.

## nav_file.cpp

### WriteNavFile
Serializes MergeResult to crossbot_nav format (v10):
- Header: magic "CBNV", version 10, mapName[64], bspChecksum, counts
- Area records: id, mins/maxs/centroid[3], areaFlags, connOffset/Count, planeOffset/Count, and the door fields
  (`doorActivation`, `doorActivatorPos[3]`, `doorWaitTime` = the area's `openSeconds`)
- Area planes come from `boundPlanes`, not `polytope.faces` — see "Area geometry vs exported region"
- Connection records: areaA/B, connType/Flags, gateValid, costSeconds, gates[2][2][3], gateAYaw/Speed[2], exitSpeed/Angle, jumpBackDist, fallDamage, flightTime, launchPos/landingPos[3]
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
- `ClipPolytope(poly, plane, keepFront, radius, epsilon)` - clips faces, adds cross-section face
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