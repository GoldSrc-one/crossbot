# Navgen System Documentation

## main.cpp

### Entry Point & Debug Output
Command-line tool with extensive debug flags:
- `--dump-leaves` - per-leaf (pre-merge) id/faces/volume/centroid stats
- `--dump-areas` - per-area (post-merge) id/faces/volume/centroid/portal stats
- `--dump-conns` - every final connection: type, flags, endpoints, air-move parameters
- `--dump-planes <id>` - one final area's exact plane list (normal+dist) and face windings
- `--dump-obj <path>` - walkable floor of every area as OBJ mesh (+ MTL): crouch areas colored apart, planned route as ribbon
- `--dump-svg <path>` - top-down SVG of final area/portal graph
- `--dump-path` - per-waypoint clearance and per-segment arc/line/gap rows (radius, turn, length,
  entry/exit speed, cap) for the debug route, plus the profile summary line
- `--trace <path>` - overlays recorded bot trajectory on SVG (one "x y z" per line, empty file = nothing).
  Also reports mean and max lateral deviation of the trace from the planned profile, and the clearance
  budget at the worst point — the headline number for whether the bot actually held its arcs.
- `--airmove-samples <n>` - launch samples per unit edge length (default 1; 0.5 halves; 0 disables)
- `--no-floor-split` - keep leaves whose floor is not solid (debug)
- `--no-merge` - skip MergeAreas (debug: each BSP hull-3 leaf becomes its own area)
- `--no-prune` - skip PruneUnreachableAreas (debug: keep areas unreachable from any spawn)
- `-o <path>` - write baked .nav file

### Debug Path Building
`BuildDebugPath`:
- Finds spawn areas (info_player_start/deathmatch) and target areas (info_target)
- Runs A* + string-pull from first spawn to first target (or farthest area centroid)
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
Union-find merge of leaves through portals, with three merge modes:

**1. 3D Convex Merge**
- `MergeStaysConvex`: all vertices of B inside all planes of A, and vice versa (epsilon 0.5)

**2. Prism Merge (vertical extrusion)**
- Allowed when 3D convex fails but XY footprint union stays convex
- `UnionFootprintIsConvex`: merged footprint area ≈ sum of parts (tolerance 0.1% of merged or 1.0)
- `MergedFloorStaysWalkable`: no internal step > 18 units between any floor face pair
- `FirstSlabConflict`: no leaf outside group has XY overlap with merged footprint AND Z overlap with merged slab
- Slab test evaluated against *leaves*, never groups (earlier prism can't be invalidated by later merge)
- If slab conflict: absorbs conflicting group if candidate portal reaches it, retries (ramp closes as whole)

**3. No Merge** (--no-merge flag)

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

### Portal Filtering
- Crouch mismatch: skip
- Whole-face portal (`PortalSharesWholeFace`): portal area ≈ both face areas (2% tolerance)
- Partial-face: `SeamIsWalkable` at portal midpoint (step ≤ 18)

### Processing Order
Candidates sorted by portal area descending (larger portals first). Each candidate attempts to absorb as many neighbors as possible via `sharedPortals` chain.

### Post-Merge
- `FilterWalkConnectionsByStep`: samples portal span (8 points), keeps if ANY point walkable (step ≤ 18), prints shallowest crossing with face normals/offsets
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

### Portal Connection Logic
Both halves come out of the same clip planes, so their shared faces are exactly coplanar and BuildPortals connects the door piece to its neighbours with no extra linking.

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
1. Find lowest floor face (highest normal.z > 0.7)
2. Probe 1 unit down from floor face → carves against world + blockers
3. Supported pieces kept; unsupported pieces get second chance: probe additional 17 units (step height - 1) → if solid, rescue if area ≥ 1024
4. Cut planes from seams between supported/unsupported (vertical through edge midpoints, outward from supported centroid, check 2 units beyond for void)
5. Fragment leaf by cut planes; drop fragments with total supported area ≤ 1.0 or no floor face
6. Outputs stats: dropped, split, steep-bottomed

### ComputeSupportedFloor
Vertical probe from floor face downward against clip hull + blocking volumes.

## stand_classifier.cpp

### SplitLeavesByStandClearance
For each leaf:
1. Probe stand hull (lift = duckClipMinZ - humanMinZ = 18 + epsilon) from floor face
2. Compute standable area fraction
3. ≤1% or no floor → crouch-only
4. ≥99% → standable
5. Partial → cut at standable/crouch boundaries (vertical planes through standable winding edges), classify each fragment by centroid probe

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

## portal_builder.cpp

### BuildPortals
1. Canonicalize planes (flip to positive octant)
2. Quantize (normal*100, dist*100) → hash buckets
3. For each bucket: split faces by sign
4. Cross-pair opposite-sign faces from different leaves
5. Intersect windings on plane (coplanar polygon intersection)
6. Keep if area ≥ 1.0 sq units
7. Portal endpoints = diameter of overlap polygon

## blocking_entities.cpp

### Carved vs passable
Two classes of brush entity, handled oppositely:

- **`func_wall`, `func_detail`** are permanently solid, so `ExtractBlockingVolumes` returns them and they are
  **carved** out of the world leaves — the space simply ceases to exist.
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
Both halves come out of the same clip planes, so their shared faces are exactly coplanar and `BuildPortals`
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