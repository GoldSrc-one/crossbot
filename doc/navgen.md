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
- `--trace <path>` - overlays recorded bot trajectory on SVG (one "x y z" per line, empty file = nothing)
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
- Outputs path points + segment kinds (walk/airmove_simulated/airmove_stored)

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

### ExtractBlockingVolumes
Entities with classname in {func_wall, func_breakable, func_detail, func_door, func_door_rotating}:
- Parse model index from "model" (*N)
- Get model bounds, expand by hull size (human: ±16 XY, -36..36 Z; duck: ±16 XY, -18..32 Z for clip hull)
- Translate by entity origin
- Return BlockingVolume {headnode, origin, mins, maxs}

### SubtractBlockingVolumes
For each overlapping blocker:
- Translate leaf polytope to blocker local space
- Carve against blocker's clip tree (remove CONTENTS_SOLID)
- Translate pieces back

## nav_file.cpp

### WriteNavFile
Serializes MergeResult to crossbot_nav format (v9):
- Header: magic "CBNV", version 9, mapName[64], bspChecksum, counts
- Area records: id, mins/maxs/centroid[3], areaFlags, connOffset/Count, planeOffset/Count, door fields
- Connection records: areaA/B, connType/Flags, gateValid, traverseCost, gates[2][2][3], gateAYaw/Speed[2], exitSpeed/Angle, jumpBackDist, fallDamage, flightTime, launchPos/landingPos[3]
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