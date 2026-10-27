# navgen

navgen compiles a GoldSrc BSP into a `.nav` file: convex walkable areas, the connections between them,
and the visibility field and camp spots the bots read at runtime. It shares the planner and the movement
simulator with the plugin through `src/nav/`, so a route previewed offline is the route a bot plans.

Sections follow the source files in `src/navgen/` (plus the format headers in `src/nav/`), in the order the
pipeline runs them.

## Pipeline

All geometry lives in hull 3 (the duck clip hull), so a leaf's floor is the origin plane of a ducking
player: "nav z" or "area z" is real floor + 18 (`kStandProbeLift`).

1. **Leaves.** `ExtractHullLeaves` turns every empty hull-3 leaf into a convex polytope.
2. **Entities.** Liquids, elevators, blockers, ladders, push fields, drift fields and passables are read
   from the entity lump (`blocking_entities.cpp`, `elevators.cpp`, `water_volumes.cpp`).
3. **Carve.** Permanently solid brush entities and ladder volumes are subtracted from the leaves; doors,
   breakables and push fields split them.
4. **Leaf filters.** Elevator stops claim their floors, floor support drops what nothing holds up, then sky
   floor, lethal floor and slime/lava floor go. Stand clearance and water levels split what remains;
   drift is tagged.
5. **Merge.** `MergeAreas` fuses leaves across walkable floor seams into areas.
6. **Connections.** Walk seams, step filter, elevator rides, air moves (once per gravity), ladders,
   teleports, then the two prunes, floor snapping, water tagging and swim links.
7. **Hazard cut and reachability.** Lethal and liquid-hazard routes go, drift terms are computed, and
   everything a spawn cannot reach is removed. Area ids are renumbered here.
8. **Gates and detours.** `BakeConnectionGates`, then `BuildActivationDetours`.
9. **Vantage field and camp spots.**
10. **Write.** `WriteNavFile`, plus whichever dumps were asked for.

## main.cpp

### Flags

`navgen.exe` with no arguments prints the authoritative list. What each one is for:

**Output**
- `-o <path>` - write the baked `.nav`.
- `--dump-svg <path>` - top-down SVG of the final area graph (see `svg_debug.cpp`).
- `--dump-obj <path>` - walkable floor of every area as an OBJ mesh plus `.mtl`; crouch areas coloured
  apart, the planned route as a ribbon.
- `--dump-vantage-svg <path>` - sample the visibility field and draw it as its own SVG, one glyph per
  vantage (see `vantage_sampler.cpp`).
- `--dump-camp-svg <path>` - camp spots as their own SVG, one glyph per spot (see `svg_debug.cpp`).
- `--dump-camps` - every camp objective (name, entities, nearby samples) and every camp spot with the
  objective indices it sees (see `camp_sampler.cpp`).
- `--trace <path>` - overlay a recorded bot trajectory on the SVG (one `x y z` per line, as
  `crossbot_trace_path` writes it; an empty file draws nothing). It also reports mean and max lateral
  deviation from the planned profile and the clearance budget at the worst point -- the headline number
  for whether the bot held its arcs.

**Dumps**
- `--dump-leaves` / `--dump-areas` - per-leaf (pre-merge) and per-area (post-merge) id, faces, volume,
  centroid; areas also list their member leaves.
- `--dump-planes <id>` - one final area's exported bound planes, then its true face windings (see "Area
  geometry vs exported region").
- `--dump-conns` - every final connection: type, flags, endpoints, air-move parameters.
- `--dump-steps` - every walk connection the step filter trimmed or dropped: area pair, shallowest
  crossing, both floor heights, surviving runs, each floor face's normal z and 2D offset from the query
  point.
- `--dump-support` - every leaf fragment dropped as unsupported: floor centroid, normal, volume.
- `--dump-footprints` - re-measure every finished area with `WalkableFootprintGapArea` and warn
  `ill-formed walkable footprint` past 1 unit². Off by default: the gap subtraction multiplies its
  remainder per floor face and exhausted the 32-bit address space on holed footprints (dod_kalt).
- `--dump-carve` - what the blocker carve and the passable split do to the leaf set: per-leaf fragment
  counts over `kCarveDumpThreshold`, per-carve totals, and how many carves fragmented a piece without
  removing any volume.
- `--dump-sky` - classify every area's floor by the BSP leaf contents just below it, list the lethal
  `trigger_hurt` volumes, and report how much footprint stands on sky, how much inside a kill volume, and
  the fifteen largest areas. A `world contents -1 (empty) under the floor` line well above a few percent
  means the mesh stands on something that is not world geometry; that is how xen_dm's void floor was
  found.
- `--dump-water` - every liquid volume (type, brush model, bounds, both surface levels, travel and speed),
  each leaf's water tag, and every swim link built.
- `--dump-drift` - every `func_conveyor` and horizontal `trigger_push` that can shove a player: movedir,
  speed, spawnflags, hull-expanded bounds, and "taken" or the reason it was skipped.
- `--dump-coverage <dir>` - every sight-coverage entity, every rasterized model with its chunk count and
  mean alpha per view ring, missing assets as `note:` lines, and one SVG per model of all 34 views.
- `--dump-airmove` - air-move launch derivation in the debug plan: why no funnel launch was found, every
  re-simulation miss (`sim miss`) with launch, yaw, speed and landing, every `rescale` retry with the
  speed it corrected to, and every `retract` with the run-up it needed against the run-up it had
  (`doc/path_planning.md`).
- `--dump-airmove-prune` - the comparison behind every kept or dropped air move and ladder.
- `--dump-ladders` - each `func_ladder`'s hull-3 volume, why each face was kept or rejected, mount and
  top positions, and every dismount simulation with height, direction, landing area, fall damage and
  verdict.
- `--dump-elevators` - every `func_door`/`func_plat` with its movedir and footprint, whether it became an
  elevator and why; for accepted ones the stops, ride and dwell times, activation, shaft column, carved
  boxes, tagged stop leaves and ride connections; and per non-default stop area the air-move sweep with
  the mover placed there.
- `--dump-teleports` - every `trigger_teleport`, why it was skipped, where its destination lands with
  drop and fall damage, and every area that can walk into it.
- `--dump-activators` - activator discovery and door detours: each gated entity's activator brushes and
  use/touch volumes, every candidate service point and why it was kept, and per `(door area, entering
  neighbour)` the chosen service point and detour polyline, or why that entry is blocked.
- `--dump-path` - the debug route: per-waypoint clearance, per-segment arc/line/gap rows (radius, turn,
  length, entry/exit speed, cap, and the `reach` pair -- see `doc/path_planning.md`), and the profile
  summary.
- `--dump-path-between "x y z x y z"` - the same, between two given points. z is area space (floor + 18).
  This is how a leg a live bot got stuck on is replayed offline.
- `--dump-airmove-pair <a> <b>` - every gate attempt for the air move `a -> b`, with pre-prune area ids
  as `--dump-airmove-prune` prints them: bin, cloud (full or base density), sample count, fit and
  re-simulation result. Silence means no sample of that pair ever landed.
- `--dump-vis-probe "x y z x y z"` - what the visibility sampler sees between two world points: each
  point's BSP leaf, whether the PVS links them both ways, and a full hull-0 `TraceHull` result.
  Repeatable. It is the independent cross-check that the boolean fast paths in `clipnode_tree.cpp` agree
  with a real trace.

**Tuning**
- `--threads <n>` - workers for air-move sampling, the vantage pair loop and camp evaluation. Default
  is one per logical processor plus one; `1` spawns no thread at all. Output never depends on it (see
  `work_pool.cpp`).
- `--max-speed <n>` - ground speed air moves are sampled at (default 320), recorded as
  `designMaxSpeed`. It does **not** have to match the server: the runtime clamps every launch to its own
  speed ("Launch speed is clamped to the server" in `doc/path_planning.md`), so a nav sampled at 320
  works on any slower server without a regen. Lower it only to shrink the graph when the target speed is
  known; to see what a slower server plans, use `--plan-speed`.
- `--plan-speed <n>` - speed the `--dump-path` route is planned at (default `--max-speed`). It overrides
  only `PlanTraceContext::physics.maxSpeed`, the single input to `PlanRoute`, `JumpLandsIn` and
  `BuildPathProfile`, so `--plan-speed 270` against a nav baked at 320 gives the waypoints, launch speeds
  and counters a `sv_maxspeed 270` server produces, without HLDS. The baked `.nav` is byte-identical
  either way.
- `--airmove-samples <n>` - launch samples per unit of edge length (default 1; 0.5 halves; 0 disables).
- `--airmove-reverse-density <n>` - multiplier on `--airmove-samples` for reverse-jump seeds (default 8;
  1 samples them like any other edge). See "Reverse-jump seeds".
- `--airmove-walk-ratio <n>`, `--ladder-prune-ratio <n>` - see `airmove_prune.cpp`.
- `--max-merge-group <n>` - most leaf groups one merge may fuse (default 64). `1` disables merging
  entirely.
- `--vantage-grid`, `--vantage-points`, `--vantage-spacing`, `--vantage-cone`, `--vantage-neighbors` -
  see `vantage_sampler.cpp`.
- `--camp-spacing`, `--camp-count`, `--camp-min-visible`, `--camp-climb-steps` - see
  `camp_sampler.cpp`.

**Debug switches** - each isolates what one stage changes.
- `--no-vantage` - skip the visibility field. The `.nav` stays valid without a vantage section; bots
  lose the idle look and cone weighting. The field is the most expensive thing navgen does, so this is
  the fast-iteration switch. Implies `--no-camp`.
- `--no-camp` - skip camp spots; the `.nav` has no camp section.
- `--no-coverage` - sightlines ignore sight coverage and trace translucent blockers as solid.
- `--no-blockers` - skip `ExtractBlockingVolumes` (no `func_wall`, `func_wall_toggle`, `func_conveyor`
  or charger carve). Also skips sight coverage.
- `--no-passables` - skip `ExtractPassableVolumes` (no door or breakable split). Doors already claimed as
  elevators are unaffected.
- `--no-floor-split` - keep leaves whose floor is not solid.
- `--no-sky-prune` - keep floors whose underside is sky.
- `--no-lethal` - keep floors and routes behind lethal `trigger_hurt`, and inside slime and lava.
- `--no-drift` - ignore conveyor and push-field drift; every connection costs the same both ways.
- `--no-prune` - skip `PruneUnreachableAreas`.
- `--vantage-verify-neighbors` - cross-check every nearest-neighbour query of the reduction against a
  brute-force scan.
- `--vantage-group-cull` - experiment: resolve visibility per vis-leaf group pair from convex-hull
  corner traces, exact-tracing only mixed groups. Single-threaded; reports the open/blocked/mixed split,
  corner traces spent, and false-visible/false-blocked rates over verification samples.

### Debug route

`BuildDebugPath` finds the spawn areas (`info_player_start`, `info_player_deathmatch`) and target areas
(`info_target`) and runs `NavPlan::PlanRoute` from the first spawn to the first target, or to the
farthest area centroid. This is the runtime planner: it string-pulls and simulates air moves *during*
the A* search, so an air move it cannot steer across never relaxes and never enters the route.

- The summary line counts those as `rejected in search`; a `debug search:` line reports areas expanded
  and the air-move simulations that cost. A connection rejected again and again is one `ProbeAirMoves`
  should not have exported. With `--dump-airmove` each `sim miss` line matches one rejection, and a
  `rescale` line with no miss after it is an edge the retry rescued.
- Air moves are re-simulated from the funnel's preferred launch point when traces are available, and the
  route is checked to stay over areas (air-move segments excepted).
- The `PlanProfile` is built and tessellated, so the exported route is the arc chain the bot follows,
  with segment kinds (walk, simulated air move, stored air move) and per-segment speeds.
- On a map with `trigger_gravity` the route is planned at the **lowest** multiplier the map offers, not at
  1.0, and navgen prints `debug path planned at gravity xN`. A bot holds one multiplier at a time and the
  lowest unlocks the most air moves, so a route that only exists under a `trigger_gravity` does not read
  as unroutable offline.

### OBJ export

`WriteFloorObj` writes each area's floor faces (normal z > 0.7), grouped by area, with materials
`nav_floor` (blue), `nav_floor_crouch` (orange) and `nav_path` (green). The planned route is a flat
ribbon at waypoint height.

## leaf_extractor.cpp

`ExtractHullLeaves` walks the hull-3 clipnode tree. For each empty leaf, every plane in its chain gets a
base winding clipped by all the others; the non-empty windings are the faces of the leaf's polytope.

## polytope.cpp

The geometry core: `Vec3`, `Plane`, `Winding` (a coplanar polygon), `Face` (plane plus winding) and
`Polytope` (a convex list of faces).

- `BaseWindingForPlane(plane, radius)` - a large quad on the plane.
- `ClipWinding` - Sutherland-Hodgman against one plane.
- `WindingArea`, `ConvexHullXY` (Andrew's monotone chain), `IntersectCoplanarPolygons`.
- `Polytope::Volume()` - tetrahedra from the centroid. `Polytope::Centroid()` - vertex average.
- `ClipPolytope(poly, plane, keepFront, radius, epsilon)` - clips every face and appends the
  cross-section face.
- `TranslatePolytope`, `WalkableFootprintGapArea`.

**`ClipPolytope` returns an empty polytope when fewer than `kMinSolidFaces` (4) faces survive.** A face
whose clipped winding falls under 3 vertices is dropped, which loses its plane; the next clip then rebuilds
its cut face from `BaseWindingForPlane(plane, mapRadius)` against the remaining planes and the piece
escapes to the map radius. Fewer than four faces cannot bound a solid, but `Polytope::Empty()` only tests
`faces.empty()` and `Volume()` reports a huge bogus number for such a piece, so it would pass every
`Volume() > kMinPieceVolume` filter. "SubtractBlockingVolumes" below has what this cost on `disposal`.

**`ClipPolytope` appends its cross-section face unconditionally.** Clipping by a plane the polytope already
has a face on, or by the same plane twice, leaves two coincident faces and `Volume()` double-counts. Every
caller that clips by another polytope's planes skips the ones it is already inside, and cut lists are
deduplicated with `HasPlane`.

## clipnode_tree.cpp

### Traversal

- `WalkClipnodeTree` - recursive traversal that hands each leaf's plane chain to a callback, for
  `CONTENTS_EMPTY` leaves (and `CONTENTS_WATER` when asked).
- `CarvePolytopeAgainstTree` - recursive clipping with memoized subtree classification: an all-removed
  subtree discards the piece, an all-kept one keeps it if its volume exceeds `minPieceVolume`, a mixed
  one clips and recurses both sides. It sets `removedAny` when a piece with real volume lands in a
  removed subtree.
- `TraceHull` / `TraceHullWithEntities` - the engine's `RecursiveHullCheck`: descend from the headnode,
  split at planes the segment crosses, and on a solid leaf back off along the fraction to the exact
  contact. Entities are traced in their own local space and the earliest hit wins.

The recursion is written against a `HullTree` (a clipnode array plus a plane array) rather than
`BspFile`, so one implementation serves the movement hulls from `LUMP_CLIPNODES` and the point hull
synthesized from `LUMP_NODES` (see `bsp_vis.cpp`). `WorldHull(bsp)` builds the clipnode-lump view, and the
`BspFile` overloads of `ClassifyPoint`/`TraceHull` are one-line wrappers over it.

### TraceHullWithEntities culls blockers like the engine

The trace's swept box is tested against each entity's absolute bounds grown by 1, the same cull as the
engine's `SV_ClipToLinks` over `SV_LinkEdict` bounds. Blocker bounds are already hull-expanded (see
`ExtractBlockingVolumes`), so the sweep is just the segment. A tighter segment-vs-box test changed
results, because qbsp's hull expansion can reach past those bounds.

### SegmentBlocked: a bit, not a trace

Visibility wants a bit. `Occluded` (`vis_sampling.h`) is blocked iff `fraction < 1.0f || startsolid`, and
reads nothing else of the 112-byte `TraceResult`. Hull-0 traces for visibility once ran 184.8M times on
crossfire, where `RecursiveHullCheck` and `ClassifyPoint` together were 53% of all navgen CPU.

`HullCheck<BoolOnly, Trace>` is one function with two instantiations, not two copies of an
epsilon-sensitive algorithm. `TraceHull` instantiates `<false, TraceResult>`; `SegmentBlocked`
instantiates `<true, NullTrace>`, carries no state, and returns a `HullStep` of `Continue` / `Stop`
(blocked) / `Clear`. On the full path `Continue`/`Stop` are the old `true`/`false` and `Clear` never
occurs.

The saving is the **back-off loop**, which re-descends from `headnode` in a `while`. The boolean path can
skip it because:

- a `true` return only originates at a leaf;
- `fraction` is written only in the terminal block, at most once per top-level trace;
- `startsolid` is set only on reaching a `CONTENTS_SOLID` leaf;
- in the terminal block `midf` is non-increasing (`frac` only decreases and `p2f >= p1f`).

The boolean path returns `Stop` the moment it reaches a solid leaf. So `allsolid` is unreachable at the
terminal block (it would mean a solid leaf was reached, which already returned), reaching the terminal
block implies `startsolid` is false, and the answer is exactly "final fraction < 1.0f". Since `midf` only
falls, `(float)midf < 1.0f` on entry already means blocked. Only when `(float)midf` rounds to exactly
`1.0f` does the loop decide, and there it runs verbatim. Both of the loop's exits set `fraction`/`endpos`
from the current `midf`/`mid` and return, so the `frac < 0.0` early return is a `break` with no change in
behaviour.

### The shared-origin chain cache

`SegmentBlocked(hull, headnode, start, end, chain)` reuses the descent shared by every trace from one
origin. Today only the `--vantage-group-cull` experiment uses it: the default vantage pair loop and the
camp sampler both trace through `SegmentsBlockedBatch` (below), and blocker traces always take the plain
`SegmentBlocked`. It stays because it is the fastest exact single-segment query for a fan from one point.

In all three branches of `HullCheck` the **first** recursive call descends to `p1`'s side: both-front
takes `children[0]` and `t1 >= 0` puts `p1` in front, both-back takes `children[1]`, and on a split
`side = (t1 < 0) ? 1 : 0` is `p1`'s side too. Along that near chain `p1`/`p1f` pass through untouched; only
`p2`/`p2f` shrink to `mid`/`midf`. So:

- the near-chain node sequence is exactly the path `ClassifyPoint(hull, headnode, p1)` walks, a function
  of the origin alone;
- every trace walks it in full, because the recursion only bottoms out at `nodeIndex < 0` -- the hit rate
  is 100%;
- `t1` at each chain node, and the leaf `p1` lands in, are pure functions of the origin.

It is also most of the traversal: 15.11 of the 25.47 clipnodes a boolean trace visits on crossfire (59%),
14.47 of 25.03 on boot_camp (58%).

`HullOriginChain` caches it. Each `HullChainStep` holds the plane's `normal`/`dist` inline, the memoized
`t1` and the far child -- 32 bytes, so walking the chain is one sequential read per node instead of a
random `clipnodes[n]` followed by a dependent random `planes[n.planenum]`, and the `t1` half of the plane
arithmetic disappears. `SegmentBlockedCached` replaces only the near descent, walking the cached steps and
pushing each split onto `chain.splits`; far subtrees still go through the recursive `HullCheck<true>`,
since they start mid-tree from a varying `mid`. Unwinding the split stack top-down mirrors the recursion's
returns exactly: every branch after the near call returns, so a non-`Continue` result propagates straight
out, while a far subtree that returns `Continue` lets the next shallower split take its turn.

The cache validates itself: the overload rebuilds whenever the clipnode array, headnode or origin differ
from what it holds, so no caller has to clear it. Arithmetic is preserved literally (`p1f` stays an
explicit `0.0` term in `p1f + (p2f - p1f) * frac`, the `kDistEpsilon` expressions are unchanged), and
`.nav` output was byte-identical on all maps under `maps\` plus crossfire and boot_camp, at
`--threads 1, 3, 7, 13`. When the vantage and camp stages used it, it took crossfire's vantage stage from
25.5s to 22.5s (summed trace CPU 180.9s to 146.3s, 0.98us to 0.79us per trace) and the camp climb from
3.9s to 3.0s.

**There is a start leaf but no end leaf.** Knowing the next trace begins in leaf L skips nothing: `t1 =
PlaneDiff(plane, newOrigin)` is still needed at every node above L to decide the splits, which is the work
being saved. Harvesting those values on the way out of the previous trace does not work either: the
traversal computes `PlaneDiff(plane, mid)` for assorted `mid`s, not for the endpoint. The air-move path
is left out for the same reason: its only shared origin is `CatagorizePosition`'s ground probe and the
next frame's `FlyMove` trace from an unmodified `m_pos`, a 50% hit rate on a stage worth 3.8s of a 46s
run.

### SegmentsBlockedBatch: n x m segments in one descent

`SegmentsBlockedBatch(hull, headnode, starts, n, ends, m, blocked, scratch)` sets `blocked[i * m + j]` to
whether segment `starts[i] -> ends[j]` reaches a `CONTENTS_SOLID` leaf, for all n x m pairs at once.
Starts and ends are xyz float triplets, all plane math is float, and n, m <= 65535. It allocates only when
`scratch` has to grow, and it is fastest when the starts share a leaf and the ends share a leaf, which is
how the vantage and camp samplers call it.

**Fallback.** The descent needs SSE4.1 (which implies SSSE3) and POPCNT, checked once via CPUID leaf 1,
ECX bits 19 and 23. Without them every pair goes through `SegmentBlocked`.

**Layout.**
- Coordinates are split per axis (SoA), padded to a multiple of 4 by repeating the last point, which
  leaves the min and max of the distances unchanged.
- A pair is a `combo = (i << shift) | j`, which is also its index into `blocked`. That equals `i * m + j`
  only when m is a power of two; any other m works on a shifted copy that is remapped at the end.
- Each combo carries its own `p1f`/`p2f`, the fraction of the segment still alive below this node.

**Start node.** Above the node where the starts or the ends first disagree, every segment lies wholly on
one side, so `SkipSharedPrefix` walks down to that node first. When starts and ends each share a leaf this
is their common ancestor.

**Plane distances.** At each node either every start and end distance is filled with SSE, four points per
step (`FillAllDistances`), or, for a list narrower than half of n + m, only the ones it uses
(`FillUsedDistances`). The SSE fill uses `PlaneDistF`'s operation order, so both are bit-identical. Both
return 1 when every distance is >= 0 and 2 when every one is < 0: each entry's t1 and t2 lerp between its
own start and end distance, so the whole list belongs to one child and is handed down as is. 0 means the
list has to be split.

**The split (`BatchDescend`).**
- The entries at `[base, base + count)` belong to this node and are never written. Children get two fresh
  lists above `top`, each at most `count` long plus a 4-slot gap, so nothing is restored on the way back
  up. The distance buffers are shared by every level: a node is done with them before it recurses.
- Unlike `HullCheck<true>` there is no hit position -- any solid leaf means blocked -- so the
  `ClassifyPoint` pre-check and the back-off loop go away, and the order of the two child calls is free.
- Entries go through four SSE lanes per step. The split fraction is computed for every lane and blended
  in (branching around it measured slower), so the only per-entry decisions are two movemasks. The split
  follows `HullCheck`: eps leans toward p1's side, p1's half goes to p1's child, the rest to the other.
- Each list is written a whole vector at a time and advances by popcount; a `pshufb` table packs the live
  lanes of a 4-bit mask to the front. The gap after each list absorbs the unused lanes, and reads past
  `count` are masked out.
- The four lane distances are built from register values with `_mm_setr_ps`; four float stores and a
  16-byte reload would stall store forwarding.
- Scratch capacity: each level adds two lists of at most `combos` plus their gaps, and 8 more slots cover
  reads past the end of the deepest list: `combos + depth * (2 * combos + 8) + 8`.

**Measured** on crossfire 16x16: ~58ns per combo, against ~220ns for a scalar per-entry version of the same
descent and ~550ns for `SegmentBlocked`. AVX2 (8 lanes) gained up to 8% at 16x16 and lost 4-7% at 6x6 and
below, so it stays SSE.

**Entity blockers are not in the batch.** `BlockersOccluded` in `vis_sampling.h` is the blocker half of
`Occluded`, for callers that traced the world with the batch.

## bsp_vis.cpp

### Hull 0 is synthesized, not loaded

`LUMP_CLIPNODES` only describes hulls 1-3, the player-sized boxes. Line of sight is a ray -- hull 0 -- and
hull 0 lives in `LUMP_NODES`. It needs no separate traversal: `dnode_t` and `dclipnode_t` share a
byte-identical `planenum` + `short children[2]` prefix, and `BuildHull0Clipnodes` does what the engine's
`Mod_MakeHull0` does: copy `planenum` and every non-negative child, and replace each negative child (a leaf
reference, `-1 - leafIndex`) with that leaf's `contents`. The result is a clipnode array the hull traversal
walks unmodified. Only `CONTENTS_SOLID` blocks, the same rule as the engine's `TRACE_LINE`; water and sky
stay transparent to sight.

### PVS

`VisIndex` decompresses PVS rows on demand; each worker reads through its own `VisRowReader`, which
caches one row. The vantage pair loop visits sources in leaf order, so that single-row cache hits on
essentially every query. Decompressing the whole table up front would cost `numleafs * (numleafs + 7) / 8`
bytes -- fine at 8k leaves, 112 MB at 30k -- to save nothing. The cache cannot live on the shared
`VisIndex`: `mutable` state on a shared object is a data race once the loop is threaded.

Row length is `(leafCount + 7) >> 3` and the bit for leaf `L` is at index `L - 1`, as in the engine. Leaf
0 is the outside-the-map solid leaf and has no row. A `visofs` of -1, an empty `LUMP_VISIBILITY` or an
out-of-range leaf all fall back to "everything visible", which is conservative: the trace still decides, it
just costs more traces. A map compiled without VIS says so on stderr.

### LeafForPoint

A plain `dnode_t` descent from node 0, returning `-1 - nodeIndex` once the index goes negative. It walks
the nodes, not the synthesized hull-0 clipnodes, because the synthesis keeps leaf *contents* and throws
away leaf *identity* -- all tracing needs, and nothing the PVS lookup can use.

## blocking_entities.cpp

### Carved vs passable

Two classes of brush entity, handled oppositely.

**Permanently solid entities are carved.** `ExtractBlockingVolumes` returns them and they are subtracted
from the world leaves -- the space ceases to exist. `IsBlockingEntity` lists them:

| classname | carved when |
| --- | --- |
| `func_wall` | always |
| `func_healthcharger`, `func_recharge` | always |
| `func_conveyor` | `!(spawnflags & 2)` -- `SF_CONVEYOR_NOTSOLID` spawns it `SOLID_NOT` |
| `func_wall_toggle` | no `targetname` and `!(spawnflags & 1)` -- `SF_WALL_START_OFF` spawns it `SOLID_NOT`; a named one is a barrier |

The conditional ones are conditional because the spawnflag is the difference between a wall and a
decoration, and carving a decoration deletes a route that exists. `func_detail` is not listed: the
compiler folds it into world geometry, so it never reaches the entity lump.

**The chargers are why this list is not just `func_wall`.** A wall charger protrudes a few units from its
wall. Uncarved, on crossfire the charger at `(448,384,-1504)…(456,416,-1456)` sat in front of a wall at
x=448 and the area in front of it started at x=464 -- the wall plus the 16-unit hull, nothing for the
charger. `ResolveChargerArea` then put the service point at x=464, 8 units inside the brush for a bot's
hull, so the bot could never arrive, and routes hugging that wall planned straight through it. Carving moves
the area edge to x=472. The runtime half is in `doc/goals.md`.

**Doors and breakables split instead.** `func_door`, `func_door_rotating` and `func_breakable` come back
from `ExtractPassableVolumes` and **split** the leaf they sit in. The interior piece survives as its own
area tagged `NAV_AREA_DOOR` or `NAV_AREA_BREAKABLE`, carrying activation, activator position and opening
time. Carving them disconnects the graph: the doorway becomes a hole between two components (test9_door
kept 3 of 9 areas that way).

**Barriers split too, and the runtime decides whether they stand.** A named `func_wall_toggle` (either
start state) and, on a DoD map, a `func_breakable` only explosives can destroy are split the same way into
an area tagged `NAV_AREA_BARRIER`, with no activation and no opening time. Nothing a bot does opens one:
the map does, when an objective is taken or a scripted event fires. The writer gives each barrier entity a
`NavBarrierRecord` (brush model index and world bounds) and its areas the record's index. At runtime
`Nav::BarrierClosed` (`nav_barrier.cpp`) closes every connection into a barrier area while the brush stands;
see `doc/path_planning.md`, "Barriers". Carved, the wall's far side was a separate component for the whole
round: on dod_charlie the `shingle1`/`shingle2` toggles the Allies blow kept every point past the beach
unreachable from their spawn after the shingle was gone. An unnamed toggle can never change state and keeps
the carve rule above. navgen's own planning treats a barrier as open, like a door.

**Floor on top of a barrier goes with it.** Floor support counts a passable brush as solid, so the top of a
barrier grows walkable areas. `TagBarrierFloors` (`floor_support.cpp`), run on the final areas, probes 1 unit
below each area's lowest floor centroid: not world solid but inside a barrier's duck clip tree means the
area stands on that barrier, and its `barrierFloor` names it. The runtime closes such an area while the barrier
is open. On dod_charlie the route north crossed the top of the `*253` sandbag wall (top z 328, waypoints at
364); once the capture blew it, 43 stalled repaths and 7 suicides piled up there in 100 s. Charlie tags 108
areas.

### ExtractBlockingVolumes

For each listed entity: parse the model index from `model` (`*N`), take the model bounds expanded by the
hull (human: ±16 XY, ±36 Z; duck: ±16 XY, ±18 Z), translate by the entity origin, and return a
`BlockingVolume {headnode, origin, mins, maxs}`.

### ExtractPassableVolumes

Same volume extraction, plus the classification of how the bot gets through. It returns a `PassableSet`:
what it can classify goes in `passable`, what it cannot goes in `impassable` and joins the carve list.

- **No `targetname`, not trigger-only** → `ACT_NONE`, walk straight through. Trigger-only is
  `SF_BREAK_TRIGGER_ONLY` (1) for breakables, `SF_DOOR_USE_ONLY` (256) for doors.
- **Unnamed use-only door** → `ACT_BUTTON` with the door as its own activator.
- **Named** → `CollectActivatorsFor` gathers **every** `func_button` / `func_rot_button` and **every**
  `trigger_multiple` / `trigger_once` whose target reaches the entity, directly or through one
  `trigger_relay` / `multi_manager` hop. Each becomes an `ActivatorVolume` with the **raw** model AABB
  (unexpanded: button range and trigger touch need different expansions) and the model centre. The area's
  activation is `ACT_BUTTON` if any activator is a button, else `ACT_TRIGGER`.
- **Named door, no activator at all** → impassable, carved like a wall, so whatever is behind it prunes out and
  pruning reports honestly that there is no way through. **A named trigger-only breakable with no activator**
  is a barrier instead: only map logic can break it (dod_charlie's `midnest_boom` sandbags `*213`/`*253`, blown
  by the capture), and breaking removes it, which the barrier check sees. **A named breakable a player can
  break** is an ordinary breakable whatever its activators (hl_c6's `crate_barrier`, spawnflags 256): the bot
  breaks it like any other. A door that map logic opens is moved, not removed, so it stays carved. A door whose activators exist but are
  unreachable is *not* carved: it keeps its area and gets blocked detour entries instead.
- **`SF_DOOR_START_OPEN` / `SF_DOOR_PASSABLE`** never occupy their brush space: neither carved nor split.
- **DoD breakables a bot cannot break** → barrier. `dod.fgd` flags 8 (only grenades), 64 (object cap /
  TNT only), 256 (bazooka or grenades), 512 (only bazooka). Applied only on maps with
  `info_player_allies` / `info_player_axis`, because HL reads 256 as instant crowbar. The route through the
  gap opens at runtime once the brush is blown.
- **Named `func_wall_toggle`** → barrier, whatever its start state.

Opening time, from the full SDK (`external/hlsdk-full`; the vendored `external/hlsdk` is headers only):

- `func_door` -- `travel / speed`, with `travel = Σ|movedir.k · (size.k − 2)| − lip` (`DoorTravelDistance`)
  and speed defaulting to 100. The −2 per axis is `CBaseDoor::Spawn` compensating for the engine growing
  bboxes by 1 each way.
- `func_door_rotating` -- `distance / speed`, defaults 90° and 100°/s.
- `func_breakable` -- `health * 0.1`, a guess at time spent breaking it; health defaults to 1. A plan-time
  estimate only: navgen splits out every breakable, including ones the bot cannot destroy, because
  `takedamage` is set in `CBreakable::Spawn` and `matUnbreakableGlass` is private data, so neither is in
  the entity lump. The runtime engages only at `health <= 100` and penalizes the connection when the brush
  takes no damage.

### Which passes see a passable as solid

- **Floor support** does: a door brush still holds up whatever rests on it.
- **The leaf carve, stand clearance and air-move sampling** do not. The bot opens or breaks it, so a jump
  may pass straight through, and a door in the stand test would make every door interior crouch-only.

### SubtractBlockingVolumes

For each overlapping blocker: translate the leaf polytope into the blocker's local space, carve it against
the blocker's clip tree (removing `CONTENTS_SOLID`), translate the pieces back, and **keep the piece whole
when the carve reports it removed nothing**. Without the `removedAny` check the carve fragments a piece
along every brush plane it straddles even when the brush takes nothing out of it, because the tree's node
planes are unbounded and the bounds test only compares boxes. On `disposal` that was 655 of 905 carves and
1091 of 1373 extra pieces; `test13_break` spent 22 of its 30 passable splits that way. The pieces are
geometrically identical to their parent, so keeping the parent is exact and strictly coarser.

This carve uses the clipnode tree, so it is exposed to the escape-to-map-radius failure `ClipPolytope`
now guards against (`polytope.cpp`). Unguarded, the four 25th-anniversary maps could not be built: on
`disposal` 439 of 727 carves came back with **more** volume than they were given, the worst by 7.3e10x --
one 8-face 3,090,431 cu.unit leaf carved into a **2-face** piece measuring 198,070,724,914. Its bounds
spanned the map, so every blocker and door split it again: total leaf volume went from 3.93e9 to 3.70e12,
and the passable split climbed past 1.2M leaves until the 32-bit navgen ran out of address space. With the
guard `disposal` carves 340 leaves into 372 pieces (1658 without), worst leaf 4 pieces (247), and builds
in 75s at 26MB.

### SplitByPassableVolume

The same carve run twice against the passable's clip tree: removing `CONTENTS_SOLID` gives the pieces
outside, removing `CONTENTS_EMPTY` gives the piece inside. Both halves come out of the same clip planes, so
their floors tile without a gap and `CollectFloorSeams` connects the door piece to its neighbours with no
extra linking -- the same property prism merging relies on.

Every leaf-splitting pass downstream (`SplitLeavesByFloorSupport`, `SplitLeavesByStandClearance`,
`SplitLeavesByWaterLevels`) must forward `ExtractedLeaf::passableIndex`, or the door loses its identity the
moment its leaf is split again.

### Door angle convention

`SetMovedir` (hlsdk `subs.cpp`) is the forward vector of the **full angles triple** -- up `(-90 0 0)`,
down `(90 0 0)`, east `(0 0 0)`, north `(0 90 0)` -- plus two legacy sentinels, `angles == (0 -1 0)` → up
and `(0 -2 0)` → down. It is not yaw-only.

**The triple may not be under the `angles` key.** Valve's maps write the legacy scalar `angle`, which the
engine widens to `(0 value 0)` -- that is why the sentinels live in `.y`. `EntityAngles` reads `angles`
first and falls back to `angle`, and every movedir consumer (`DoorTravelDistance`, `ClassifyMover`) goes
through it. Reading only `angles` yields `(0 0 0)`, movedir `(1 0 0)`, and a vertical lift looks like a door
sliding east: all four crossfire lifts (`lift1`..`lift4`, `angle -1`) would fail `ClassifyMover`, and the
`z = -1518` roof ring -- 50 areas, 18 spawn points, 17 items -- would be a one-way trap, reachable from a
spawn so never pruned, with no edge back up.

### Other extractions

- `ExtractLadderVolumes` - each `func_ladder`'s brush per hull (see `ladder_connections.cpp`).
- `ExtractPushVolumes` - `trigger_push` per hull (see "Push fields" under `airmove_sampler.cpp`).
- `ExtractDriftVolumes` - conveyors and horizontal push (see "Drift Fields" under `area_merger.cpp`).
- `ExtractHurtVolumes` - lethal `trigger_hurt` (see `DropLethalFloorLeaves`).
- `CollectGravityMultipliers` - see "Gravity" under `airmove_sampler.cpp`.
- `ExtractVisBlockers`, `ExtractSightCoverage` - see `vantage_sampler.cpp`.

## elevators.cpp

### What an elevator is, and why it is not a door

A vertical `func_door` is the classic HLDM lift: you stand on top and it carries you. Modelled as a door --
something you wait for and walk *through* -- it produces a wrong route rather than a missing one: on
`test18_elevator` a door model gives a 16-unit slab tagged `door(button, 2.38s)` under the parked platform,
and the pit column's floor becomes the platform top instead of the real pit floor.

### Classification

Both tests are required; failing either leaves the entity on the door path.

1. classname `func_door` (not `func_door_rotating`) or `func_plat`, and `|movedir.z| > kFloorNormalZ` with
   the other two components under `kElevatorAxisEpsilon`. Pure translation only: `TraceHullWithEntities`
   translates a blocker by its `origin` and cannot rotate a clipnode tree.
2. footprint at least `kElevatorMinFootprint` (48) in **both** horizontal axes.

Movedir alone separates nothing: every `func_door` in `test9_door`, `test10_doorbutton`,
`test11_doorbutton2` and `test20_doortrigger` also has `angles "-90 0 0"` and moves straight up. Those doors
are 16 × 64 and fail on footprint. A 48-unit floor in both axes is not a doorway.

### Stops

`func_door` -- travel from `DoorTravelDistance`, ride time `travel / DoorSpeed`. `SF_DOOR_START_OPEN` makes
the moved position the spawn position (hlsdk `doors.cpp` swaps `m_vecPosition2` with `m_vecPosition1`).
Dwell at the moved stop is `wait`; `wait < 0` or `SF_DOOR_NO_AUTO_RETURN` means it never leaves on its own.

`func_plat` -- the compiled brush *is* the top position (hlsdk `plats.cpp`: plats are drawn extended).
Bottom is `origin.z - height`, or `origin.z - size.z + 8` when `height` is 0. Speed defaults to 150. A
**named** plat spawns at the top and is brought down by its button; an **unnamed** one spawns at the bottom
with an auto-spawned touch trigger. Dwell at the top is 3s unless `SF_PLAT_TOGGLE`.

Dwell is keyed to the mover's **position**, not to which stop is the default. `CFuncPlat::HitTop` waits at
the top whether or not it spawned there, so a named plat (default = top) and an unnamed one (default =
bottom) both dwell at travel 0; keying it to default-ness gets the unnamed case backwards and drops the
ride-down edge. A door dwells at its *opened* position, which is its non-spawn stop either way, including
under `SF_DOOR_START_OPEN`.

### Geometry: a stop area is the space the mover floors

All in nav space. Per stop, `ExtractElevators` builds:

- `duckVolume` -- the mover's model at that stop's offset, expanded by the hull, traced through the
  model's own clipnodes. `MoverVolume` builds it; its AABB is the model bounds ±16 in XY, ±18 in Z.
- `topZ` -- `origin.z + model.maxs.z + travel + kDuckClipHullMaxZ`, the top of that body. `slabTopZ` is
  the next stop's `topZ`, or `mapRadius` for the highest, which keeps two stops of one mover from
  overlapping (on `test18_elevator` they meet exactly at nav 16).

`SplitLeavesByElevatorStops` runs **before** `SplitLeavesByFloorSupport` and is the only place a mover
touches the leaves. Per leaf, per stop, `ClaimStopFloor` takes the space between `topZ` and `slabTopZ`
whose floor is the mover and not the world:

- the candidate is the leaf clipped to `duckVolume`'s AABB in XY and `[topZ, slabTopZ]` in Z;
- `ComputeSupportedFloor` on its floor face runs twice: against the world plus every non-elevator blocker,
  and against `{duckVolume}` alone with `CONTENTS_EMPTY` as the headnode so the world contributes nothing.
  A piece is the stop's when the only thing holding it up is the mover parked at that stop;
- where the two disagree, `VerticalPlaneThroughEdge` and `ClipPolytope` cut the candidate along the
  boundary;
- mover-floored pieces are claimed and tagged `elevatorStopId`. Anything the world already floors stays
  behind as ordinary ground, and if nothing is claimed the leaf stays whole rather than fragmented.

Only then is the **default** stop's body subtracted from what remains, and only from pieces that strictly
overlap it. `OverlapsBlockingVolume()` counts a shared face as overlap, and carving against a tree the piece
only touches still splits it along that tree's planes; the claim leaves pieces flush against the mover, so
the subtraction needs the strict test. Claiming before subtracting is what lets a lift parked at its *top*
stop still produce a lower stop area -- otherwise the parked body swallows the region the lower stop needs.
That is why the general blocker carve in `main.cpp` runs on `airmoveWorlds.baseDuck`, which is every
blocker except the elevator volumes: this stage owns the order.

The two halves of the claim rule each have a map. `test55_plat`'s raised stop is level with the ledge
beside it and the mover's footprint reaches 16 units into that ledge; the world floors that strip, so it
stays ordinary ground and walking onto the ledge does not require summoning the plat. The strip on the
*other* side, with 256 units of pit under it, is floored only by the mover, so it becomes part of the stop
and is gated with it instead of being a free-standing ledge in mid-air.

Injecting **leaves** rather than finished areas makes the rest free: floor support, stand clearance,
`MergeAreas`, `BuildBoundPlanes`, `ValidateAreaFootprints`, `BuildWalkConnections`,
`FilterWalkConnectionsByStep`, `ProbeAirMoves`, `PruneUnreachableAreas`, `BakeConnectionGates` and
`BuildActivationDetours` run unmodified. `BuildWalkConnections` re-derives every seam with
`CollectFloorSeams`, so a stop area gets walk connections with no linking code: on `test18_elevator` the
raised stop's floor (nav 16) and the corridor floors (nav 18) share the planes `x = 144` and `x = 368`, and
the 2-unit step survives the step filter. On `test55_plat` the same machinery finds the 8-unit step from
the pit floor at nav −238 onto the parked plat at nav −230, which is how a bot boards.

### Blocker lists

- `blockers` / `standBlockers` get the **default stop only**, so traces see the map as it spawns. That is
  what makes a drop into the shaft land on the parked platform.
- The leaf carve uses `airmoveWorlds.baseDuck`, the same list *without* any mover, because
  `SplitLeavesByElevatorStops` subtracts the parked body itself after claiming.
- `supportBlockers` gets **every stop**, or each stop's floor face would probe into air and
  `SplitLeavesByFloorSupport` would drop it.
- `ProbeAirMoves` gets an `AirmoveWorlds`: the base lists with no mover volumes at all, plus every stop's
  volume, indexed by `Area::elevatorStopId`. `BuildAirmoveWorlds` is called before the default stops are
  appended to `blockers` for exactly that reason.

### Where the mover stands during sampling

No single static world works. A stop's floor is the boundary between its own volume and the area below,
so making a raised stop standable seals the stop beneath it: on `test18_elevator` the raised platform
blocks nav −260…16 and area 6 is −222…16, entirely inside it. Every stop solid at once would leave area 6
no free space, delete the drop into the pit and strand the route at the call button. That is not an
artifact -- with the lift up you cannot drop into the pit. A thin slab at `topZ` fails the same way, and a
swept volume is worse, being the union over a continuum.

So the world is a property of the **sweep**. `ProbeAirMoves` composes one immutable world per
configuration up front: the default (every mover at its default stop) plus one per non-default stop, with
that stop's mover moved there and every other mover at default. A stop *replaces* its mover's volume, so a
composed world never has a platform in two places. An area tagged with a non-default stop is swept in its
stop's world; every other area in the default world. On a map with no elevators the composed world is the
base list unchanged.

With the platform placed, `FilterLaunchEdges`, `IsSupported` and `Simulate` need no special cases.
(Relaxing `IsSupported` instead would not have worked: `Simulate` runs in the same world, and the sampled
player would fall through the floor it launched from.) `--dump-elevators` reports the flip per stop area,
e.g. `support 0 parked -> 4 placed`. `test18_elevator` stops there -- its raised top is flush with both
corridors, so `GroundAhead` filters every perimeter edge -- but `test55_plat` goes the whole way: its
raised stop reaches `x=112` with 256 units of pit beyond, giving `2 launch edges, 508 samples` and a real
`airmove 4 -> 7` off the platform into the pit. That case exists only because the stop claims the space
the mover floors rather than a shrunken box.

The runtime needs nothing: `PlanWalker::JumpLandsIn` re-simulates every air move against the live engine
trace, so an edge that only exists with the lift raised fails to relax while it is parked.

### Gating and the ride edge

The default stop is real ground: `ACT_NONE`. Every other stop carries `NAV_AREA_ELEVATOR`, the mover's
activation, and `openSeconds` = the ride time, the cost of *summoning* the platform. Walking onto a
non-default stop goes through the same `ActivationDetour` machinery as a door, keyed the same way
(`FindDetour(toArea, fromId)`). On `test18_elevator` entering the raised stop from either corridor needs a
route to the call button, the button is only reachable from the parked platform, the detour comes back
blocked, and `PlanWalker::Cross` refuses: you cannot summon that lift from the top.

`BuildElevatorConnections` runs after `FilterWalkConnectionsByStep` and before `ProbeAirMoves`, so the
stop pair is already in the sampler's `existingPairs` and no duplicate 238-unit lethal drop is sampled. It
removes any `CONN_WALK` the seam collector built between two stops of one mover (the step filter usually
kills those first, but this removes them structurally).

`CONN_ELEVATOR` carries `rideSeconds`, `rideSecondsMax` (equal to `rideSeconds`; the hook for a
swept-interval sampler) and `rideWaitSeconds`. Cost is `rideSeconds + rideWaitSeconds` -- time, no
distance. The ride *toward* the non-default stop is gated by that area's activation and waits 0; the ride
*back* is ungated and waits the departure stop's dwell. A stop that never leaves on its own emits no return
edge, so the lift is one-way in the mesh rather than silently wrong.

In `PlanWalker::Cross` the branch follows the teleport template: optional activation detour, a counted
waypoint carrying `requiresElevatorRide` / `elevatorTravelTime`, then `PushWaypoint(arrive,
countLength=false)` so the ride length never enters the funnel, then re-anchor.

### A stop you cannot summon can be left but not entered

An unnamed `func_plat` has no activator in the entity lump -- its trigger is auto-spawned at runtime -- so
its raised stop is `ACT_NONE` and nothing gates walking onto it, which would let a bot walk out over a shaft
with the platform at the bottom. Dropping that `CONN_WALK` would also drop the way *off* the platform, which
is the seam that matters once you have ridden up, so `BuildElevatorConnections` orients it instead:
`CONN_FLAG_ONE_WAY` with the stop as `areaA`, walkable only outwards. `reachability.cpp` and
`nav_path.cpp` both honour the flag.

### Known limitations

- **No air moves *onto* a non-default stop.** Launching off one works, because the launch area is the
  stop and its world places the mover. Landing on one does not: a jump toward the shaft from an ordinary
  corridor is swept in the default world, falls past the raised stop to the parked platform, and resolves
  to the default stop -- correct for the world it simulated. The fix is to sweep areas near a mover once
  per configuration, keeping only pairs that touch that mover's stops; such an edge would also need
  `BuildActivationDetours` extended past its walk-and-ride entry check, since `PlanWalker::Cross` already
  rejects a non-walk, non-ride entry into a gated stop.
- **A large ceiling-retracting door misclassifies.** A vertical door 48+ units in both axes that slides up
  into a recess becomes an elevator: its sweep is carved and the upper stop is unreachable inside the
  recess. A wasted area and a lost doorway, not a crash.
- **`test18_elevator` depends on its 2-unit flushness.** The raised top lands at nav 16 against corridor
  floors at nav 18, which is why the exits are walk seams. Change the `lip` and it becomes an air-move
  case.
- **Auto-return dwell is not enforced.** `rideWaitSeconds` records that the platform leaves after `wait`,
  but nothing prevents planning a top-side walk-off slower than the dwell. That is a runtime concern.
- **An unnamed `func_plat` cannot be summoned from outside.** `CollectActivatorsFor` only runs for a
  non-empty `targetname`, so a plat summoned by its implicit `CPlatTrigger` reports `activation none, 0
  activator(s)` and every seam into its raised stop is exit-only. `test55_plat` does not need entering (pit
  → board → ride → alight), but a plat parked at its *top* is unreachable from below: `topplat` (that map
  with a `targetname` patched in) has both stop areas and a ride-down edge, and nothing can call it.
  Modelling `CPlatTrigger` as a touch activator over the mover's own column would close it.

## water_volumes.cpp

`ExtractLiquidVolumes` finds every body of liquid, from two sources:

- **World liquid.** Hull-0 leaves of the world model with `CONTENTS_WATER`, `CONTENTS_SLIME` or
  `CONTENTS_LAVA` (hull 0 from `BuildHull0Clipnodes`, because the clip hulls have no liquid).
- **Brush entities.** Any brush entity whose `skin` is a liquid contents, or whose model contains a liquid
  leaf (taken as water), e.g. `func_water`. `trigger_*` entities never count: they are `SOLID_TRIGGER`,
  which `SV_LinkEdict` files under `trigger_edicts`, and `SV_AddLinksToPM_` builds physents (and so
  `PM_PointContents`) from `solid_edicts` only. dod_charlie's beach carries eight `trigger_multiple`
  mortar triggers with a stray `skin -3`; taken as water they sank the beach and left the Allied spawn a
  62-area island. A `func_water` or `func_door` with a `targetname` and a
  vertical movedir is a **moving** volume: it carries `travel` and `speed` from the door math, and
  `SF_DOOR_START_OPEN` (1) starts it at the moved position.

Pieces whose boxes touch within 1 unit are grouped into one body, and static bodies of the same contents
whose surfaces agree within 1 unit and whose boxes touch are merged. Water goes in `LiquidSet::water`,
slime and lava in `LiquidSet::hazard`. A moving body has two surface levels, `LowSurfaceZ` and
`HighSurfaceZ`.

**Slime and lava are floor deleters.** `DropLiquidHazardLeaves` drops every leaf whose lowest floor
centroid, lifted 18, is inside a hazard body, and `PruneLiquidHazardConnections` drops any connection whose
chord (sampled every 8 units, lifted 18) passes through one; teleports and elevator rides are exempt.
`--no-lethal` disables both, alongside the `trigger_hurt` prune.

## water_classifier.cpp

Runs after stand clearance. `SplitLeavesByWaterLevels` cuts each leaf along the XY footprint planes of
every water body over it whose top surface reaches 18 above the leaf bottom, so an area is uniformly in or
out of a pool. Each fragment is then tagged with its **governing** water volume and a `submergeZ`:

- `GoverningWaterVolume` looks up a floor point: a moving body that covers it wins, else the static body
  with the lowest surface. A static body whose bottom is more than `kStandHeight` (72) above the floor
  cannot touch a player standing there and is skipped, here and in `SplitLeavesByWaterLevels`; the probe is
  otherwise clamped into the body's z range, so dod_charlie's `func_mortar_field` with `skin -3` at z
  376..456 claimed 1118 leaves on the beach 400 units below it. `submergeZ` is the floor plus 18 (`kNavFloorToStandOrigin`), the standing
  origin -- the water level at which a standing bot is in the water.
- A **water column** (a floorless leaf kept by floor support because it sits inside water, capped at the
  surface) takes the body containing its bottom, with `submergeZ` at that bottom.

`BuildLeafWaterCeilings` marks a leaf `alwaysSubmerged` when even the body's low surface is above its
`submergeZ`, and gives it a `waterCeiling`: its ceiling faces below the surface (supported parts only) plus
a cap at the surface where the ceiling rises above it. A submerged area's "floor" for swimming is that
ceiling.

`MergeAreas` refuses a seam between different water volumes, or between an always-submerged leaf and a
dry one; two submerged leaves merge only when their water ceilings meet flush (`SeamIsFlush`).
`TagConnectionsByWater` stamps every walk and air-move connection whose lower endpoint is governed by a
water body with that body and a `submergeZ`; at runtime `ConnBlockedByWater` (`nav_format.h`) closes walk
and air-move edges while submerged and swim edges while not.

## floor_support.cpp

### SplitLeavesByFloorSupport

For each leaf:

1. Take every floor face (normal z > 0.7).
2. Probe 1 unit down from each against the world and blockers (`ComputeSupportedFloor`). A piece is
   carved only through blockers whose hull-expanded bounds it touches, and a blocker that removes nothing
   leaves the piece whole; without that, every blocker tree's planes re-split the piece. On dod_caen that
   stage was ~40s serial across 167 blockers.
3. Cut along the seams between supported and unsupported floor.
4. Fragments with supported floor become leaves. A floorless fragment inside water is kept as a water
   column, capped at the surface. The rest go to `AbsorbFloorlessPieces`.
5. Report split, absorbed, water columns, dropped.

**Every floor face, not just the lowest.** A leaf beside a ramp holds both the flat ground at its foot and
the ramp. Taking only the lowest face picks the ground, which is real floor everywhere, yields no cuts, and
leaves the ramp face untested however far it extends past the surface it belongs to.

**The probe is straight down, not along the face normal.** `PM_WalkMove` drops the hull vertically by
stepsize to step up, and `PM_AirMove` drops it vertically by 1 when falling; the probe matches that
wherever the floor is tilted. Probing along the normal would slide the patch 12.7 units sideways on a
slope and test solidity in the wrong place. Each probe step starts from the already-dropped patch.

**Cuts.** A supported patch is cut out only where it stops being supported: an edge shared with another
supported patch is a seam and stays uncut. Edge coverage is recorded per *stretch*, not per edge -- only
collinear segments cover anything -- because one edge is routinely both: the strip of upper floor beside a
ramp opening meets its neighbour along part of an edge and overhangs the ramp along the rest, and only the
overhanging part must be cut. An edge needs no cut when it wholly abuts a peer patch or runs along the
leaf's own floor boundary; the supported windings tile the floor exactly, so whatever lies past an
uncovered stretch is another class of floor by construction, with no probing. Two patches meeting along an
edge both want to cut there, so a plane enters the cut list once (`HasPlane`).

A floor face is supported where solid is directly under it and nowhere else. There is no "supported by
ground a step below": the space over lower ground is reunited with that ground by absorption, not by
inventing floor at the upper height.

### Floorless fragment absorption

The air above a staircase tread is routinely cut off at the *neighbouring* step's height: hull-3 expansion
leaves a step-shaped ceiling, and the clipnode plane that caps one tread runs straight over the next, so
the column above tread N gets a bottom face floating 8 units up. Deleting it as unsupported shreds
staircases: the tread is left as an 8-unit sliver with no standing room and no faces touching its
neighbours.

So a fragment with no supported floor is handed *down*. `AbsorbFloorlessPieces` looks for a leaf whose top
face the fragment rests on and whose union with it is convex, checked by comparing the merged volume with
the sum. If the fragment overhangs several steps, `SplitAgainstHostColumn` carves off the column directly
above one host -- clipping only by that host's side planes, and only by planes the fragment is not already
inside (see `polytope.cpp`) -- absorbs that, and returns the remainder to the pool. Whatever no leaf can take
is dropped.

Absorption is strictly downward: the shared facet must be the host's top and the fragment's bottom.
Sideways absorption looks reasonable and is badly wrong: a floorless band beside a pyramid's apex plateau
unions with it into a full-width phantom area with floor over 8 units of an 80-unit span.

### DropSkyFloorLeaves

Sky is solid in the clip hulls, so the inside of a skybox shell extracts as ordinary floor. Nothing
downstream objects: it is a real standing surface, a bot that walks off a platform lands on it, and that
fall is a real air move, so reachability keeps the whole shell. It is still the outside of the level.

The test is per leaf, on its lowest floor face's centroid, probed 2 units under the real surface
(`kDuckClipHullMinZ - 2`, since floor faces sit in the duck hull) and classified by the hull-0 world leaf
there. `CONTENTS_SKY` means the leaf stands on the shell and it goes. No trace, no volume list: the BSP
already answers. It runs at the leaf stage, before the lethal prune, for the reason given there:
`MergeAreas` would otherwise fuse shell floor with the real ledges above it.

Only the two synthetic skybox maps have any: test39_skybox (2 leaves) and test41_skydrop (3). On both
the shell was ~98.4% of the walkable footprint -- 8.05M of 8.19M sq units on test39 -- which the vantage
sampler paid for quadratically: 36k lattice points, 664M pair checks, 71s of a 71s run. Pruned it is 660
points, 217k pairs, and navgen finishes in 0.9s. All 19 HLDM maps report 0% sky floor, so on real maps the
prune is a no-op.

### DropLethalFloorLeaves

`trigger_hurt` with `dmg >= 100` is not an obstacle, it is a floor deleter. One tick takes a full health
bar, so a standing spot you can only reach by falling through one is not a place a bot can be -- and
nothing in the leaf pipeline notices, because a trigger brush is not solid in any hull.

Skipped: start-off (2), no-clients (8), and DoD's `DontHurtAllies` / `DontHurtAxis` (64 / 128). The DoD
pair is spawn protection that only kills the other team; treating it as lethal deletes the spawn exits
(dod_sturm reached 26 of 1767 areas). Negative `dmg` (healing) never qualifies.

The test is per leaf, on its lowest floor face's centroid. The floor is dead when it is inside a lethal
volume's XY footprint, below the volume's top, and a duck-hull trace straight up from it into the volume's
underside comes back clear. The upward trace keeps the rule narrow: a room under a lava pit has a ceiling,
the trace stops on it, and the room survives; an open column under a kill plane does not. Volumes are
extracted at `kHullDuck`, so a floor already inside the trigger reports `entry.z <= stand.z` and is dropped
without a trace.

**It has to run on leaves, not on areas.** A connection cut plus the unreachable-area prune does nothing
useful: on xen_dm it removed 550 connections and pruned nothing, because `MergeAreas` had already fused the
void-floor leaf with real ledges above it (area 309 spanned z −2286 to −1758 in one prism), so no void area
existed to prune.

**What it is worth.** xen_dm's whole footprint sits over a clip brush at z ≈ −2304 under a map-wide `dmg
10000` slab at −2160. That floor was 32.9M of 44.4M sq units of walkable footprint -- 74% of the map -- and
drove the vantage sampler quadratically: 221k lattice candidates instead of 53k. Dropping its 40 leaves takes
the footprint to 12.2M and the whole run to 3m34s; without it the run was abandoned at 575s unfinished.
On `pool_party` 616 of 2800 floor faces (29.5% of its footprint) stood *inside* a `dmg 200` trigger. Across
the 50 test maps and 19 HLDM maps only four have a qualifying volume: xen_dm (40 leaves), pool_party (88),
snark_pit (1), rocket_frenzy (1).

Healing volumes stay out of navgen entirely: they are not solid in any hull, so they carve and split
nothing, and the runtime finds them by edict scan (`ITEM_HEALVOLUME` in `doc/goals.md`). On `pool_party` the
`dmg 200` volume is the one lethal trigger navgen reports, and the `dmg -40` pool never appears.

### TagDriftLeaves

See "Drift Fields" under `area_merger.cpp`.

## stand_classifier.cpp

### SplitLeavesByStandClearance

For each leaf:

1. Probe the stand hull from every floor face, lifted `kStandProbeLift` (18) plus epsilon.
2. Compute the standable fraction over all floor faces together.
3. ≤1% or no floor → crouch-only; ≥99% → standable.
4. Otherwise cut along the standable/crouch boundaries (vertical planes through standable winding edges)
   and classify each fragment.

**Every floor face, not the lowest one** -- the same trap as floor support. Hull-3 expansion routinely
leaves the flat ground at a ramp's foot and the ramp in one convex leaf, so the lowest face is the ground,
which has all the headroom in the world; classifying off it reports the leaf standable, makes no cut, and
the ramp's high end keeps the ground's clearance. test51_pyraduck shows it: a 45° pyramid under a ceiling
48 above its apex, so the top 24 units of every face are crouch-only. Three faces came out right because
the BSP split their slope from the surrounding floor; the east face arrived as one leaf holding both, and
lost its crouch cap. With every floor face classified the four faces are identical -- same volumes, same
24-unit band.

**A cut must be a real stand/crouch boundary.** `OnWindingBoundary` tests a standable winding's edge
against that winding's own face; testing against another face mistakes the ramp's bottom edge (a real face
boundary) for a crouch boundary and halves the leaf for nothing. Cut planes are deduplicated with
`HasPlane`; coincident faces inflated test51's leaf volumes by up to 15%.

`ClassifyPolytope` probes every floor face of a fragment and calls it crouch if any face is crouch-only.
Cutting should make fragments uniform; where it does not quite, ducking where standing is allowed costs
nothing, while standing where it is not wedges the bot.

## area_merger.cpp

### MergeAreas

Union-find merge of leaves across walkable floor seams. Candidates are sorted by seam length, longest
first, and each tries to absorb as many neighbours as it can through the `sharedSeams` chain. Two modes:

**3D convex merge.** `MergeStaysConvex`: every vertex of B inside every plane of A and vice versa
(epsilon 0.5).

**Prism merge** (vertical extrusion), tried when the 3D test fails but the XY footprint union stays
convex:

- `MergedFloorStaysWalkable`: no internal step over 18 units between any pair of floor faces.
- `CoverageOk`: the group's footprint hull equals the union of its *member leaves'* footprints to within
  1 unit². Coverage is `Σ area(fp_i) − Σ_{i<j} overlap(fp_i, fp_j)`; by Bonferroni, truncating
  inclusion–exclusion at the pairwise terms is a **lower bound** on the union, so the test can only
  reject, never falsely accept, which lets it work at any group size without a polygon-union primitive. It
  iterates member leaves, not the groups' stored hulls, because hull-of-hull compounds across merges.
- The 1 unit² is floating-point slack, not a fudge factor. A tolerance proportional to area (0.1%) buys
  ~50 units² of unearned claim on a room-sized footprint, enough for the hull to chamfer across a pillar;
  that is what test49_pyrascew did.
- `FirstSlabConflict`: no leaf outside the group may overlap the merged footprint in XY *and* the merged
  slab in Z. Evaluated against *leaves*, never groups, so a later merge cannot invalidate an earlier prism.
  On a conflict the group absorbs the conflicting leaf's group if a candidate seam reaches it, and retries
  (a ramp closes as a whole).

**Growth past a non-monotone predicate.** Coverage is checked only at accept time, never as a reason to
abandon a pair, because it is *not monotone*: on a pyramid the ring of wedges plus the apex column is one
convex prism, yet every proper subset leaves a hole and fails. Validating after each addition can never
reach the valid group, in any order. When coverage fails, `FirstFootprintGap` asks which outside leaf
claims the uncovered part of the hull, and the group absorbs it, exactly as for slab conflicts. A gap
**nobody** claims is solid (a wall, the inside of a slope), and the merge is abandoned.

`FirstFootprintGap` is `FirstSlabConflict` without the Z-overlap requirement, and that is load-bearing: an
apex column sits *on top of* the frustum its wedges bound, so the two Z ranges touch at one plane and
`OverlapsZ` is false. Dropping the Z test is safe because `sharedSeams` still requires a walkable floor seam
into the group, which a leaf on an unrelated floor lacks. Keeping the finders separate leaves
`FirstSlabConflict`'s Z rule -- what stops stacked floors merging -- untouched. A leaf whose XY box misses
the merged footprint's is skipped before any winding math.

**Group cap.** `maxMergeGroup` (`--max-merge-group`, default `kDefaultMaxMergeGroup` = 64) bounds how many
leaf groups one merge may fuse. It is checked at the top of the growth loop, *before* absorbing, so `1`
refuses even the first pair; checked after absorbing, a 3D-convex pair would get through at `1`, since that
path accepts without reaching the cap. Raising the cap from 1 is the cheapest way to bisect a merge bug: it
bounds how much merging it takes to reproduce.

**The same tolerance validates the result.** `ValidateAreaFootprints` (`--dump-footprints`) re-measures
finished areas with `WalkableFootprintGapArea` and warns past the same 1 unit². Keep the two equal: a merge
test looser than the validator means the validator can never fire on what the merger accepted, which is how
the test49_pyrascew overclaim went unreported.

### Area geometry vs exported region

An area carries two descriptions of itself and they are not interchangeable:

- `Area::polytope` -- the true surface, multi-tread floors and all. Floor lookups, the step filter,
  air-move launch edges and the OBJ/SVG dumps read this.
- `Area::boundPlanes` -- the convex region written to the `.nav`. A convex area exports its own face
  planes; a prism exports its XY footprint hull plus a z-slab, a convex region the runtime's
  `ContainsPoint` tests unchanged. The slab check during merging is what makes the substitution sound.

**Anything asking "is this point in this area" must read `boundPlanes`.** A prism's faces include interior
risers whose halfspace intersection is empty, so a polytope test silently answers "no area" for every point
-- which is how `FindAreaStrict` (air-move landings) and spawn/target resolution once returned nothing and
gutted the graph.

### Seams, not portals

Merge adjacency comes from walkable floor seams (`CollectFloorSeams`), not from a global list of shared
faces between leaves. Portals couple adjacency to volume: deleting the floorless air column above a tread
also deletes the faces that made two treads portal-adjacent, and a flight stops merging though every walk
connection survives (raw portals collapsed 261 → 150 on test15_staircase while leaf count barely moved).
Seams carry the adjacency without that coupling, because `BuildBoundPlanes` derives a prism's region from the
walkable footprint rather than from faces. The face *cancellation* the convex path needs is computed per
candidate pair by `FindSharedFace` (opposed planes plus `IntersectCoplanarPolygons`). Seam-adjacent leaves
separated by a riser share no face at all; they take the prism path.

Two leaves whose floors meet on an XY line are not necessarily one open space: hull-3 expansion makes floor
edges meet at *every* vertical drop, an 8-unit step and a 200-unit cliff alike, because the top face expands
outward by 16 and the side wall expands the lower floor back by the same 16. The step test, not the seam,
makes a seam a merge candidate.

### Seam filtering

A seam is not a merge candidate when the two leaves differ in crouch, in passable (`passableIndex`), in
drift (by more than `kDriftMergeEpsilon`), in water volume, or in always-submerged state. A dry seam must
pass `SeamIsWalkable`, whose walkable spans cover the *whole* seam; a submerged one must be flush
(`SeamIsFlush`).

Full coverage is the point. A slope running alongside a floor meets it on one XY line but is steppable at
one end only, and "walkable anywhere" would fuse the two into an area whose footprint claims ground a bot
cannot reach. Refusing to merge on missing information is the safe direction, so unlike the connection
filter, "no floor on either side" counts as not walkable here.

A door or breakable never merges across `passableIndex`: it has to stay its own area to keep its
activation, and must not absorb a different one.

### WalkableSpansAcross

The stretch of segment `a + ab * t`, t in [0, 1], whose XY projection lies within `epsilon` of a face's
XY footprint. Windings are convex, so that is one interval, found by clipping the parameter range against
each edge's outward half-plane (an edge parallel to the segment has it wholly in or wholly out). The
epsilon makes the answer stable: a seam lies on a leaf boundary, so the segment routinely runs exactly
along a floor face's edge, and without slack whether the face counts would be floating-point luck.

Each floor face is a plane, so wherever the same pair of faces applies both heights are linear in t and so
is their difference; the applicable faces only change at the ends of those spans. The gap is therefore
piecewise linear with known breakpoints, and `|gap| <= maxStep` solves on each piece to a single
sub-interval -- nothing is sampled. On a piece that is one-sided over its whole stretch the closest approach
is at an end; where the floors cross, they meet exactly. The union of the sub-intervals is the exact stretch
of the seam a bot can step across. Where two faces abut, both are candidates along the shared stretch and a
point counts if *any* face pair clears the step -- the same existential question the caller asks, and no
tie broken by face iteration order.

A wedge closed by a sloped ceiling (the crawlspace under a ramp) ends where ceiling meets floor, and no
near-vertical plane records that, so the gate builder cannot see the end from the bound planes alone.
Reporting "no floor here" outside the real floor footprint keeps gates inside their area.

`valid` is false when no stretch has floor on both sides. That differs from "the step is large", and the
callers want opposite things from it, so they decide. The shallowest crossing is kept alongside the spans
only for `--dump-steps`; a nonzero 2D offset there means `FloorFaceUnder` fell back to its nearest face
instead of finding a containing one.

### After the merge

- `BuildWalkConnections` re-derives every seam on the final area polytopes.
- `FilterWalkConnectionsByStep` replaces each walk connection with its walkable runs: walkable end to end
  passes untouched, partially walkable is cut to the stretches that clear the step (one connection each,
  runs under `kMinSeamLength` dropped), walkable nowhere is dropped. This stops a slope beside a floor
  exporting a gate across the part of the seam 60 units up. A seam with no floor on both sides anywhere is
  kept: there is nothing to judge it on.
- `SnapConnectionsToFloor`: walk -- both endpoints onto side A's floor; air move -- launch only (the
  landing is the simulation result).
- `BakeConnectionGates` builds gates with the shared planner code and lifts endpoints to the true floor.

### Cost: ConnectionCostSeconds and ConnectionTravelSeconds

`ConnectionCostSeconds` is the seconds a bot loses on an edge **beyond running its length**: hanging in
the air on a jump, riding, teleporting. A* adds it to travel time, so it is real time and **not** a
distance multiplier; making it one would break door and air-move routing.

- plain walk -- 0
- air move, ladder, teleport -- `flightTime` plus a fall-damage penalty, with "lethal ⇒ `1e4`" as the
  never-route sentinel
- elevator -- `rideSeconds + rideWaitSeconds`

Door and breakable waits are **not** here. Charged on every connection touching a gated area, they would
count a through-route twice and skew `airmove_prune`'s Dijkstra near doors. `PlanWalker::Cross` adds
`openSeconds` to `m_extraTime` once, on entry; the activator detour costs its own travel time through
`m_committedLength`, which is the point of expressing cost as time.

`ConnectionTravelSeconds` is the running time, which the planner does not need (it measures the walk) but
a walker-less search does. Centroid to centroid over `kNavRunSpeed`, funnelled through the gate: the
straight line if it crosses the gate segment, else bent around the nearer gate endpoint, so an L-shaped area
is not priced as if the bot walked through the wall. Non-walk connections get only the ground legs to
`pointA` and from `pointB`; their flight, climb or ride is in `ConnectionCostSeconds` already. It goes in
the `.nav` as `travelSeconds` because it is fixed by the mesh and the funnel is not worth redoing per frame
(`doc/path_planning.md`).

`airmove_prune.cpp`'s `WalkDijkstra` keeps its own centroid-to-centroid measure; routing it through this
helper would change pruning on every map for no benefit.

### Drift Fields (conveyors and horizontal push)

`func_conveyor` and `trigger_push` shove a standing player by the same mechanism: they set
`pev->basevelocity`, which `pm_shared` adds to velocity for the move and subtracts afterwards, so world
speed is own speed plus field velocity and `pev->velocity` stays own-only. Travel time therefore depends on
direction, which is what drift captures.

The two entities differ in three ways that matter:

- A conveyor pushes only what **stands on** it; a push field acts on anything **inside** it, airborne
  included.
- A conveyor pushes iff `spawnflags & 3 == 0` (`SF_CONVEYOR_VISUAL` sets no `FL_CONVEYOR`;
  `SF_CONVEYOR_NOTSOLID` leaves nothing to stand on). That is **not** the carve predicate: `IsBakedBlocker`
  carves iff `spawnflags & 2 == 0`, so a visual-only conveyor is still solid. `ExtractDriftVolumes` tests the
  two flags separately so `--dump-drift` can say which one dropped a field; both constants live in
  `baked_blockers.h` beside the carve predicate.
- Overlapping push fields **sum** (`CTriggerPush::Touch` adds to the base velocity already applied), so a
  leaf's drift is a vector sum.

`ExtractDriftVolumes` (`blocking_entities.cpp`) takes conveyors and push fields whose *horizontal*
component reaches `kMinDriftSpeed` (30), which leaves vertical launch pads entirely to the air-move path.
Push fields with `SF_TRIG_PUSH_ONCE` (1) or `SF_TRIGGER_PUSH_START_OFF` (2) are skipped
(`kSfPushOnce`, `kSfPushStartOff`): a start-off field spawns `SOLID_NOT`, which `Bot_PushFields` skips at
runtime too, and snark_pit, rocket_frenzy and doublecross all have them.

**Tagging.** `TagDriftLeaves` (`floor_support.cpp`) runs after every leaf-splitting pass and before
`MergeAreas`, so no split has to propagate the tag. It probes the `LowestFloorFace` centroid -- 2 units down
for a conveyor (you must stand on it), 1 unit up for a push field (you must be inside it) -- against the
entity's own hull-3 clip tree, sums every claim and drops z. Conveyors need no leaf split, since the blocker
carve has already cut every leaf along their planes. Push fields are `SOLID_TRIGGER` and never carve, so a
leaf can straddle one; they get an explicit `SplitByPassableVolume` pass so an area is uniformly in or out.
`MergeAreas` then refuses a seam whose leaves disagree on drift by more than `kDriftMergeEpsilon`.

**The cost model.** With `ms` the mover's max ground speed, `v` the area's drift and `d` the unit direction
of travel, the bot steers to cancel the perpendicular part of `v` out of its own speed budget:

```
along = Dot(v, d)
s     = along + sqrt(max(ms^2 - (|v|^2 - along^2), 0))
```

Straight downstream that is `ms + |v|`. Across a gentle 100 u/s belt it is 304, a 5% penalty. Across a 500
u/s current it is 0, which is right: you cannot hold a line across a strong field.

Cost divides by `max(s, kMinDriftSpeed)`, so a sideways crossing of a violent field is roughly 10x dearer
but still available. **Blocking uses a looser test on purpose**: a direction is impassable only when
`ms + along < kMinDriftSpeed`, i.e. running flat out straight down `d` still loses ground. Blocking on `s`
would wall off the side exits of every fast field and disconnect the graph.

**Nothing is baked at a fixed speed.** `Bot_MaxSpeed` reads `pEdict->v.maxspeed` before `sv_maxspeed`, so
`ms` is per server *and* per bot. A baked decision fails in exactly the case that matters: a 280 u/s belt
bakes at `320 - 280 = 40` ("expensive but walkable") and is impossible at `sv_maxspeed 270`, where the bot
commits and stalls. So `ConnectionDriftTerm` stores a speed-independent `crossbot_nav::DriftTerm` and every
consumer calls `NavDrift::EvalDrift` with its own `maxSpeed` (`doc/path_planning.md`, "drift.h:
direction-aware cost").

The term splits the crossing at the connection's endpoints -- leg A is `centroid(areaA) -> pointA`, leg B
`pointB -> centroid(areaB)` -- and keeps, over the legs whose area drifts, the total `length`, the
length-weighted mean `along` and `perp`, and the `alongMin`/`alongMax` extremes. The mean prices the
crossing; the extremes decide passability, because a connection whose two legs drift in opposite directions
(subtransit's `*22` and `*23` meet head to head) must not average its way out of an impossible leg.

`WalkDijkstra` in `airmove_prune.cpp` does **not** see drift: it runs before the terms are computed. On a
map with a strong field it can prune an air move in favour of a walk route that goes upstream; only
subtransit and undertow are exposed.

`PruneUnreachableAreas` evaluates the term at navgen's own `--max-speed`, which `designMaxSpeed` records.
A server *slower* than the bake leaves areas the runtime will not route to, the safe direction; a *faster*
one over-prunes, and `--max-speed` is the answer. The runtime warning for a mismatch is
`BotManager::CheckDriftSpeeds` (`doc/movement.md`). `--no-drift` disables the feature.

## airmove_sampler.cpp

### PM_MoveSimulator

Wraps the shared `PmSim` core with navgen's trace function (world plus blocking volumes).
`SimulateAirborne` starts the simulation mid-flight with a given velocity, already flagged airborne, so the
first ground contact counts as a landing.

`OnLadder` is `PM_Ladder`: a point-content test of the origin against each `func_ladder`'s clipnode tree
for this hull, AABB-prefiltered. It must be the clipnode test, not the AABB: a slanted ladder's AABB is far
larger than the brush (test53_ladder4's model 3 is 160 units wide as a box and a thin slab in fact). `Fly`
ends a flight with no landing on the first airborne frame the origin is on a ladder: a `func_ladder` is
`SOLID_NOT` and stops no trace, but the engine switches the player to `PM_LadderMove` the moment they enter
one, and with no input they hang there. Ladders mount on walls, so this catches the flights that clip a wall
and slide down it. The check is skipped while on the ground, so a launch edge overlapping a ladder's
footprint is still sampled; only leaving the ground next to one is refused. `HullContext::ladders` carries
the volumes per hull; `BuildLadderConnections` leaves it null on its own simulator, because a dismount
starts inside the ladder it is leaving and would abort on frame one.

The planner's launch re-simulation cannot do the same: `JumpLandsIn` runs on `PlanTraceContext`'s trace
functions and the `.nav` carries no ladder volumes, so a launch from another point on the gate can still fly
through one. Closing that needs the volumes in the format.

### Coasting

A stepped frame costs two hull traces (`FlyMove`, then `CatagorizePosition`'s 2-unit ground probe), so a 1 s
flight is ~120 traces. `Fly` calls `PmSimulator::Coast` (`doc/path_planning.md`) before every `Step`, with
`ArcFn` → `SafeArcTime` as the arc function. The safe time is the minimum of:

- `TraceHullArcWithEntities` on the arc itself — walls, ceilings, floors;
- the same arc lowered by `kGroundTraceDist`. `CatagorizePosition` lands the player on a lip the hull clears by
  under 2 units without ever touching it; only the lowered arc sees that. Either trace starting solid (just left
  the ground) refuses the coast;
- `ArcEnterBox` into any ladder or push volume's AABB, because `OnLadder` and `PushAt` are per-frame point
  tests the coast would skip. AABB only, so the cap is early, never late; the exact clipnode test still runs in
  the stepped frames.

`Coast` rewinds one frame before that time and `Step` resumes, so every frame that touches anything is stepped
exactly as before. `peakZ` takes the analytic apex of a coasted span.

`TraceHullArc` is `HullCheck` with the segment replaced by `p(t) = p0 + v·t − ½g·t²`. Per node the plane
distance is `f0 + b·t + c·t²` (`b = n·v`, `c = −½g·n_z`; linear for walls), so a span can cross a plane twice
and the descent visits the near child, the far child, then the near child again. Each crossing gets the same
`CONTENTS_SOLID` far-child test, `kDistEpsilon` pull-back (`ε / |f'(t)|` in time) and 0.1-of-span back-off as
the segment version. Blockers are translation-only, so a blocker's arc is the same velocity from
`start − origin`; the cull is the arc's AABB (apex included) grown by 1, as for segments. The descent and
`HullCheck` itself live in `src/nav/hull_trace.h`, templated on vector and scalar type and taking a general
acceleration vector (a rotated brush entity sees gravity off its z axis); navgen instantiates them with
`Vec3`/`double`, the plugin's planner world (`doc/path_planning.md`, "The plan-time world") with `Vector`/`float`.

The stepped sim moves along chords of the parabola, at most `g·dt²/8` ≈ 0.028 units off it at 1/60 —
under `kDistEpsilon` — so the arc and the steps agree except on grazes of that size. A graze still diverges a
flight: the pull-back makes the engine's own result depend on where a chord happens to cross a plane, so a
corner the arc clears by 0.03 can stop a 1/60 chord, and the bounce from it decides the landing. Subdividing at
the exact root instead of the pulled-back point was measured and is no closer (bounce: 33 changed outcomes and
62 moved landings vs 31 and 52), so the descent keeps `HullCheck`'s rule.

`--check-arc-trace` flies every sample twice, coasted and fully stepped, and prints how many changed outcome
(landed, ladder grab or landing area), how many landed more than 1 unit apart, and the worst landing move and
flight-time difference. The stepped flight is the reference because it is what navgen did before coasting.

Measured 2026-10-20, default flags, 13 threads, coasted vs stepped:

| map | airmove simulation | navgen total | changed outcome |
|---|---|---|---|
| xen_dm (x0.4 + x1.0) | 4.3 s → 1.4 s | 34.7 s → 28.1 s | |
| bounce | 1.2 s → 0.6 s | 13.3 s → 12.3 s | 31 of 173,636 flights |
| dod_caen | 4.4 s → 2.4 s | 55.1 s → 49.9 s | |

The total drops by more than the simulation line because ladder connections (`ladder_connections.cpp`) and gate
re-simulation also fly through `Simulate`/`SimulateAirborne`. Connection counts moved by at most 4 (dod_caen 9751 → 9755).

### ProbeAirMoves

Runs **once per gravity multiplier** (see "Gravity"), stamping every connection it emits with that pass's
multiplier. Within one pass:

1. **Launch edges.** `FilterLaunchEdges` keeps floor-perimeter edges with void ahead (outward normal from
   the face centre). `PushLaunchEdges` adds the footprint boundary of any `trigger_push` over the area (see
   "Push fields").
2. **Samples.** Each edge gets `length * --airmove-samples` samples. Each draws a random point on the
   edge, a yaw within `±yawSpread` (90°) of outward, a commanded speed in `[0, maxSpeed]`, and for jumps a
   random 0-32 unit jump-back. Every sample is flown as a drop and as a jump, as the mode allows.
3. **Simulate** with `PmSim` until landing, wedge or timeout; keep it if it lands in another area that is
   not already connected by a non-air-move connection (`existingPairs`).
4. **Bin** by hull and kind: stand drop, stand jump, duck drop, duck jump. Within a bin the best
   trajectory per area pair is the lowest fall damage, then the shortest flight (`BetterTrajectory`).
5. **Reverse seeds.** Drops under the gravity-scaled `maxJumpHeight` seed a climb back (see "Reverse-jump
   seeds"), sampled as a second round.
6. **Emit.** Bins are visited in the order above, and for each pair the first bin whose cloud fits a gate
   (`FitGate`) and survives gate re-simulation wins: standing before ducking, drop before jump. The
   connection carries the gates, exit speed and yaw, jump-back distance, fall height, flight time, launch
   and landing positions, and `CONN_FLAG_NEEDS_CROUCH` when the duck hull won.

Samples from every edge that reached a pair land in one cloud. Which edge a sample left through does not
matter: the robust fit keeps the majority cluster and rejects the rest as outliers, so the gate settles on
the face the move is really about.

### Sweep modes

Three modes run over every area, each with its own hull, speed and z offsets:

- **stand** - stand hull; `probeZ`/`launchZ` = `kStandProbeLift` (18), `navZAdjust` = −18 to bring
  standing-origin positions back into nav space.
- **crouchjump** - duck hull at standing speed, `launchZ` = 18, jumps only.
- **duck** - duck hull at ducking speed (×0.333); all offsets 0, because duck space *is* nav space.

### Threading

Sampling is an enumerate pass and a sample pass, both binned by `work_pool.cpp`. The reverse-jump seeds are
a barrier between two such rounds: they come from the drop bins, so they cannot be enumerated until every
drop has landed.

- **Enumerate** -- one request per area (or per seeded area). Areas are Morton-sorted by centroid and
  binned with their floor perimeter vertex count as cost, known before any trace runs. Each worker runs
  `FilterLaunchEdges` for its areas into per-request slots, flattened in request order, so the work-item
  list and its ids do not depend on thread count.
- **Sample** -- one work item per launch edge, Morton-sorted by edge midpoint, cost `edgeSamples *
  variants`, tile size 32. Nearby edges land in one bin so their trajectories walk the same BSP subtrees,
  and summed cost is near equal across bins: on crossfire 5.1s against 3.3s, a 1.55x spread.

Nothing in the sweep is shared and mutable:

1. **Worlds are immutable.** Every distinct world -- the default plus one per placed elevator stop -- is
   composed once up front into its own `WorldConfig` with its own three simulators, and a work item
   carries a world index (see "Where the mover stands during sampling" under `elevators.cpp`).
2. **Each launch edge seeds its own generator** with `EdgeSeed(seed, phase, areaId, modeIndex,
   edgeIndex)`, a splitmix64 mix. Thread and bin index are deliberately *not* mixed in, so an edge draws the
   same stream whichever worker runs it. The gravity multiplier is not mixed in either, so each gravity pass
   re-runs the same streams.
3. **Bins are per worker** and merged afterwards. Each `AirmoveSample` carries the `(itemId, seq)` it came
   from (`seq` = sample × 2 + variant); the merge sorts each pair's cloud back into that order and re-folds
   `best` over the sorted cloud rather than merging per-worker bests. `FitGate` therefore sees the same
   sequence, in the same floating-point summation order, for any `--threads`.

On crossfire the simulation takes 5.4s threaded against 29.5s serial; enumeration is under 0.1s either way.

### Sampling noise

Connection counts move by a percent or two whenever the random streams change, because a pair whose cloud
is marginal can fall either side of `FitGate`'s minimum-sigma test. That is inherent to Monte Carlo
sampling: on crossfire per-edge seeding found 1391 air moves against a single shared stream's 1367 (+1.8%),
and across the 50 test maps the deltas were −2 to +2 with **reachability identical on every map**. Raising
`--airmove-samples` closes the gap (at 16 samples/unit the two agree on `test29_ladder3` and
`test23_precise`), and one stream swings by the same ±1 between sample rates on its own.

When comparing two navgen builds, compare reachable-area counts, not air-move connection counts.

### FitGate (robust segment fitting)

- Iteratively reweighted (Tukey biweight, 4 iterations).
- PCA on the weighted points gives the segment direction.
- Local regression (tricube kernel, bandwidth 0.5 × segment) at the ends gives yaw and speed.
- Gate endpoints are mean ± 1σ (σ at least 1.0), clamped to the sample range.
- Invalid when the cloud is too thin or tight. `BuildSampledGates` then drops the whole connection, not
  just its gate.

### Gate re-simulation

A fitted gate is a regression, not a sample: its endpoints and the yaw and speed interpolated along it were
never flown. The runtime launches from wherever the funnel crosses the gate with exactly that interpolated
yaw and speed (clamped to `maxSpeed`) and re-simulates it, so on a move with no margin the kept sample
lands and the gate does not.

So each air move is flown at `kGateProbes` (9) evenly spaced points along its exit gate, launched the way
the runtime does it and placed with the sampler's z convention (`launchZ - probeZ - navZAdjust`), in the
world and mode of the pair's samples. The gate shrinks to the longest contiguous run of points that land in
the destination, with yaw and speed re-interpolated at the new ends; fewer than two landing points drops the
connection. A gate that passes everywhere is left exactly as fitted, so a map with no marginal moves bakes
byte-identical. The summary line reports how many were dropped and shrunk by gate re-simulation.

The case that motivated it, on `dm_centralcore_v2_one`: a battery on a 64-unit ledge reachable only by
crouch-jumps. navgen's kept sample scraped over; the runtime launched from the gate end 3 units nearer, met
the ledge face 0.14 units low, and every bot that picked the battery ran a failing whole-mesh search (~1500
air-move simulations, 400-850ms).

### Landing clearance

`Fly` rejects a landing whose flight never rose `minLandClearance` (2 units) above the landing origin. The
model launches 1 unit above the floor (`lift`), so a crouch-jump that clears a 64-unit ledge by exactly its
apex is a unit higher than a real player gets, and at runtime in float it is a coin toss. Drops and jumps
across gaps clear by tens of units and are unaffected.

### Gravity

`trigger_gravity` copies its `pev->gravity` onto whoever touches it, and it sticks until respawn, so a bot
only ever runs at one of a small, statically known set of multipliers. `CollectGravityMultipliers` returns
1.0 plus one entry per distinct `trigger_gravity` value, capped at four (`kMaxGravityMultipliers`, with a
warning past that).

`ProbeAirMoves` runs once per multiplier with `physics.gravity` scaled and stamps `MergedConnection::gravity`
on what it emits. A map with no `trigger_gravity` gets one pass at 1.0. `xen_dm` has ten triggers, all
`0.4`, so it gets two passes and roughly twice the air moves -- intended, since the runtime only uses the set
matching the bot's current gravity ("Planning with gravity multipliers" in `doc/path_planning.md`).

Two things in a pass move with gravity instead of staying at their 800-tuned defaults:

- `existingPairs` is built from **non**-air-move connections only. Otherwise pass two would see pass one's
  air moves and suppress itself.
- The reverse-seed height is `params.maxJumpHeight - PlainJumpHeight(kGravity) +
  PlainJumpHeight(gravity)`: the default 64 (step height plus a plain jump) at gravity 800, 109 at half
  gravity. Left fixed, low gravity would never seed a climb back up a ledge it can now clearly reach.

`PruneRedundantLadders` counts only default-gravity air moves as an alternative, so a ladder is not pruned
for a jump that exists only under a `trigger_gravity`. Ladder mount reach (`ladder_connections.cpp`) uses
the default gravity only; ladders are not duplicated per multiplier. The SVG draws the per-gravity air-move
sets apart (`svg_debug.cpp`).

### Push fields

`trigger_push` hands the player a base velocity every frame it is touched. `ExtractPushVolumes` reads the
brush per hull like `ExtractLadderVolumes`, with `push = speed * movedir`; start-off and one-shot fields are
skipped (see "Drift Fields"). They are `SOLID_TRIGGER`: they never enter `blockers`, `standBlockers` or
`supportBlockers`, and never carve, split or support a leaf.

`PM_MoveSimulator::PushAt` sums the push of every field whose hull-expanded bounds contain the origin and
whose clipnodes classify it solid -- the point test `OnLadder` uses, and the one `SV_TouchLinks` uses to
decide a trigger touch. It reaches `PmSim` as a `PmPushFn`; `doc/path_planning.md` has what the simulator
does with it. A vertical push only acts on an **airborne** player.

**Launch seeding.** A push field on flat floor against a wall has no perimeter edge with void ahead, so the
perimeter sweep never launches there and push during flight would never fire. `PushLaunchEdges` clips the
four vertical sides of each field's footprint against the area's floor faces (2D Cyrus-Beck, endpoints
lifted onto the face plane) and emits each surviving segment as a `LaunchEdge` whose `outward` points
**into** the field. Launches are set back along `outward` and the yaw fan aims along it, which gives
launches just outside the field aimed inward: a bot running into the shaft and jumping.

Those edges are a seeded request, which forces jumps only and requires the launch to be inside the source
area. They set `LaunchEdge::noJumpBack`: the random 0-32 unit jump-back that helps an ordinary ledge jump is
wrong here, because it smears the launch cloud across a band where only the end nearest the field works,
and `FitGate` then produces a gate whose interpolated speed misses. Pinning the launch to the boundary makes
the fitted gate usable.

### Reverse-jump seeds

A drop under the gravity-scaled `maxJumpHeight` seeds a climb back the other way: `SeedEdgeFromLandings`
turns the drop's landing points into a synthetic launch edge in the destination area. It is swept like any
other edge (jumps only, with the same `±yawSpread` fan), but at `--airmove-reverse-density` (default 8,
`AirmoveSampleParams::reverseSeedDensity`) times `--airmove-samples`. Requests in this round carry
`kReversePhase`.

**Why denser.** A seed edge is only as long as the drop's landing spread, often 10 to 30 units, and the
climb it tests usually goes back through whatever narrow opening the drop came out of. At one sample per
unit, few of the fanned launches land, the cloud fails `FitGate`, and the climb is lost. dod_flash's
windows are the case: a 48x80 opening (16 units in hull space) with a sill 48 above the ledge outside.
The drop out was found, and the crouch-jump back in (`678 -> 675`) was not. At 8 samples per unit it is
found, as is the matching climb at the window at (-740,-1138). Raising `--airmove-samples` globally
finds the same climbs at 8x the cost everywhere.

**More samples can also lose a pair.** On `test54_window`, the climbs `1->6`, `1->7` and `1->15`
(pre-prune ids) come only from reverse seeds. With 15 samples, `1->7` fits a gate that lands. With 85,
the gate is wider, its interpolated yaw and speed miss, and re-simulation drops it. That cut reachability
from 20 areas to 14.

**So each pair falls back to the base-density cloud.** Every launch edge seeds its own generator, and the
sample loop draws in a fixed order, so the first `lround(length * --airmove-samples)` samples of a dense
edge are exactly the ones a density-1 run draws. Those are marked `AirmoveSample::base`. Emit tries the
full cloud first. If it does not fit or does not land, it retries with the base samples alone, which is
the density-1 cloud bit for bit. The result is a superset of both densities. At density 1 every sample
is base and the bake is unchanged. The summary line counts the rescues: `N fitted from the base-density
cloud`.

Measured over the 51 test maps plus dod_flash, density 8 against 1 (2026-10-27):
- **Reachability:** no map lost a reachable area; dod_flash went from 1557 to 1565.
- **Air moves:** five maps gained some (dod_flash 928 to 1209, `test30_gravity` 18 to 23,
  `test43_vent` 11 to 14, `test54_window` 79 to 82, `test6_crouchjump` 16 to 17), and none lost any.
- **Bake time:** the whole sweep went from 39.6 s to 42.8 s; dod_flash alone from 32.5 s to 34.6 s.

`--dump-airmove-pair <a> <b>` prints every gate attempt for one pre-prune pair: bin, cloud (full or
base), sample count, whether it fitted and whether it landed.

`LaunchEdge::yawRange`/`yawLo`/`yawHi` are unused. They were for a narrower arc on reverse seeds -- a climb
has to retrace its drop, so aim only along the drop's headings reversed -- which was designed and never
implemented. The reason it looked worth doing: `BuildSampledGates` drops the whole connection when
`FitGate` cannot establish lateral spread, so fanning wide at a narrow opening can lose a genuinely
traversable crossing. The seed density answers that case without narrowing the fan.

## ladder_connections.cpp

### BuildLadderConnections

Runs after `ProbeAirMoves` and before the prune passes, and emits `CONN_LADDER` connections for every
`func_ladder`. Running after the sampler matters: emitted first, ladders would suppress jumps through the
sampler's `existingPairs`, the opposite of the intended priority (see `PruneRedundantLadders`).

`PM_Ladder` is a **point-content test**, not a trace: is the player origin inside the ladder brush expanded
by the current hull? There is no "which face did I touch", so the whole pass works off the hull-3 clipnode
volume, never `LUMP_FACES` (not loaded, and empty for `func_ladder` once ZHLT nullifies the trigger
texture). Everything runs in hull 3, so no z-adjust is needed. Ducking enters only when the dismount area is
crouch-only: the connection then gets `CONN_FLAG_NEEDS_CROUCH` and the ducking climb speed.

### The volumes are carved out of the walkable leaves

`SubtractBoxVolumes` removes each ladder's hull-3 volume from every leaf it overlaps before areas are built.
A nav area is duck-origin space and the expanded volume is exactly the set of origins the engine puts on a
ladder, so after the carve **no nav area contains a ladder-capture point**. Routing past a ladder is then
structural: neither the planner nor `bot_avoid` can send the bot through a spot where `PM_PlayerMove` would
switch it to `MOVETYPE_FLY` and turn its forward input into a climb. Only the ladder state machine puts the
bot inside one.

The carve uses the ladder's **axis-aligned hull bounds, not its clipnode tree**, even though the tree is
exact. It is the cheap, robust choice: axis-aligned planes rarely trip `ClipPolytope`'s degenerate-face
failure, while test53_ladder4's slanted ladder tripped it catastrophically through the tree (total leaf
volume 94,346,287,097 against 25,747,456 uncarved, areas reaching `maxs 8192`). With `kMinSolidFaces`
guarding `ClipPolytope` the box carve is belt and braces. It is exact for a vertical ladder and conservative
for a slanted one (it removes walkable floor under the slope).

Because the mount lands inside the carved hole, `FindStandArea` steps outward from it in 4-unit steps up to
48, tracing down at each, and returns the first floor point inside a real area. That point, not the mount,
becomes `pointA` and both gate-A corners: it is where the bot stands to start the climb.

### Per ladder

1. **Extract the volume** - walk `model.headnode[kHullDuck]` for `CONTENTS_SOLID` leaves and build a
   polytope per leaf. Face normals from `BuildPolytopeFromChain` point *inward*, so the outward normal the
   player stands against is `-face.plane.normal`.
2. **Keep climb faces** - `|normal.z| <= kFloorNormalZ`. Near-horizontal caps are skipped, so the top cap
   is never a mount surface.
3. **Find a free probe on the face** (`FindFaceProbe`). Try the face centroid inset 2 units into the
   volume first, then sweep along `perp` in 8-unit steps out to the winding's extents, trying insets of 2, 1
   and 0.5 at each offset, and take the first point a duck hull fits in. A face with no free point is
   rejected, which buries the three faces of a wall-mounted ladder that face into the wall.

   The sweep is needed: a thin plate flush against a wall leaves a capture band a unit or two deep, and the
   centroid can miss it in both axes at once. test53_ladder4's shaft ladder is a 1-unit plate on the `y =
   0` wall of a shaft with walls at `x = 304` and `x = 384`. Its face centroid is `(320, 17, -24)`: `y = 15`
   after the inset is behind the wall for a duck hull, and `x = 320` is exactly on the hull-clip boundary of
   the `x = 304` wall. `(328, 16.5, -24)` -- one perp step over, half a unit in -- is free, and is the only
   reason that ladder has connections; without it the map loses 7 areas to the void prune and the target is
   unreachable.
4. **Trace down** along the face-projected down vector (a slanted ladder follows its own slope) to `2 *
   kStandProbeLift + PlainJumpHeight(gravity)` = 81 units below the expanded volume bottom. The bot is
   *standing* when it jumps for the rung, hence 18 + 18 + 45 rather than 45. The hit is the mount; no
   floor in reach, or a mount in no area, rejects the face.
5. **Trace up** from the mount to the expanded top, no overshoot. Whether the climb ended because the
   ladder ended or because a ceiling blocked it decides whether the forward dismount exists.

### Dismounts

Four ways off, scanned from the top down in 16-unit steps, with the standing reach height inserted in
order. Sides and jump-off apply at every height; **forward applies only at the top, and only when the
ladder ended** -- you cannot stop mid-ladder and step forward. test29_ladder3 is why the scan exists: its
ladders hit the ceiling, so no floor is reachable from the top and the ledge they serve is passed at
mid-height.

| Dismount | Start | Initial velocity | Sim yaw / speed |
| --- | --- | --- | --- |
| forward | top position | `(0, 0, +200)` | toward `-N`, `maxSpeed` |
| left / right | face end along `±perp` | `±perp * 200` | same direction, `maxSpeed` |
| jump off | column at that height | `N * 270` | toward `N`, speed 0 |

Working the `PM_LadderMove` algebra for a player facing the face, `lateral` is exactly zero and the result
is **pure +Z at 200**: you cannot move toward `-N` on a ladder, because that input *becomes* the climb.
Forward still reaches a ledge because the bot keeps rising ~25 units after leaving the volume and
air-accelerates sideways; `kAirAccelWishSpeedCap` buys ~15 units of drift, exactly what gets it from
test12_ladder's column at x≈354 onto the ledge at x≥368. Jump-off commands speed 0 so the simulation does
not air-accelerate the bot back into the ladder.

Each runs through `SimulateAirborne`, so a sideways step onto a ledge at the same height is a dismount, not
a slide.

Sampling is in hull 3 so a dismount into a crouch-only area is found, but a sample a *standing* player
cannot occupy is discarded unless the destination area is `NAV_AREA_CROUCH`. `StandFits` is a point test of
the sample lifted by `kStandProbeLift` against the hull-1 tree; rejections print `[standing hull does not
fit]` under `--dump-ladders`. The same test derives the climb's `reach`: step down from the true top until a
standing hull fits.

Per landing area there is **one** winner: lowest fall damage, then reversible (below), then nearest to that
area's centroid. A dismount landing back in the ladder's own bottom area is a **loopback** and produces
nothing.

### A sample that finds nothing is retried a sixteenth of a unit to either side

Every sample sits on the face centroid's lateral line, synthesised from brush geometry, so it can land
*exactly* on a hull-expanded clip plane -- it does whenever a ladder brush is exactly hull-wide (32 units)
and flush against a wall, because the brush centre and the wall's expanded face are the same number.
On-plane is the **front** side here and in the engine (`ClassifyPoint` and `HullCheck` send `>= 0` to
`children[0]`, as `SV_HullPointContents` and `SV_RecursiveHullCheck` do), and which side is front depends
only on how the axial plane is stored, so the same geometry fails or works by orientation alone.

test52_avoid has both. Ladder `*2` (y 0..32) is flush against the south wall; its centre `y = 16` is the
wall's expanded face with the open side in front, and it dismounts. Ladder `*1` (y 80..112) is flush against
the `y = 112` wall; its centre `y = 96` is that wall's expanded face with the *wall* in front, and the top
sample also sits on the pocket slab's `x = 656` face, whose front is open. The point passes `InSolid` only
because of the `x` plane; the forward dismount moves west, leaves it, is inside the wall on the first trace,
and falls back down the shaft: `[loopback]`, `no connections emitted`.

That agrees with the engine -- `PM_FlyMove` would block a player at exactly `y = 96` too -- but no player is
ever there: every engine trace stops `DIST_EPSILON` short of the plane it hits, so a real climber is at `y
<= 95.96875`. So when a sample produces no dismount, the whole sample (forward, sides and jump-off) is rerun
at `±kLadderLateralNudge` (1/16, twice `DIST_EPSILON`) along `perp`, `+` first, stopping at the first that
produces one. The nudge is far below anything geometric, so it cannot invent a dismount; it only moves the
start off a plane no real player can be on. `--dump-ladders` tags those lines `[nudged +perp]` / `[nudged
-perp]`: on test52_avoid `*1` loops back centred and at `+perp`, and at `-perp` reaches area 6 at (641,
95.9, 18). A sample that already works is never nudged, so every other test map bakes byte-identical.

### Two ladders are two connections

`emitted` deduplicates by area pair **per ladder entity**. Within one ladder it collapses what its faces and
pieces produce for a pair to the lowest-fall-damage one; across ladders it keeps both. One shared edge
cannot be routed around: the runtime `LadderBlocked` probe penalises the connection the bot is on
(`doc/path_planning.md`), and with a single connection that only makes the same ladder dearer -- on
test52_avoid, merged per pair, `*1` replaced `*2` outright by being processed first and tying on fall damage.
Parallel edges are fine for the planner, which walks connection lists and never looks one up by endpoints.

Of the twelve stock DM maps with ladders only snark_pit changes: its `*9` and `*10` are one wide ladder built
from two brushes, each keeps its own `148 <-> 147`, and `PruneRedundantLadders` keeps 2 of 5 instead of 1 --
the same climb twice, which costs nothing and lets a bot blocked on one half take the other.

### Reversibility, and why it decides the gates

Gate A is the mount. Gate B is **where a descending bot gets onto the ladder**, not in general where an
ascending one steps off. They coincide for a dismount that barely left the column and diverge for one that
flew, so `landingPos` carries the ascent's exit and gate B the descent's entry.

A dismount is **directly reversible** when it is not a jump-off and its landing lies inside the ladder's
hull-expanded volume, with `kStandProbeLift` of slack in z (a standing player's ladder volume reaches 18
units further each way in z than the hull-3 one). Gate B is then the landing: the bot steps back on where it
stepped off.

Otherwise the pass asks **can the bot walk off an edge of the landing area and fall onto the ladder?**
`EdgeDropReachesLadder` takes the landing area's drop edges from `FilterLaunchEdges`, samples each every 16
units, and simulates a plain run-off (no jump) at full, half and quarter speed against a `PM_MoveSimulator`
carrying **only this ladder**. A run that trips `grabbedLadder` makes the connection two-way, and gate B
becomes the ledge point it left from. Grabbing cancels fall damage, so the drop is free however far it falls.

This applies to every dismount kind. `PM_Ladder` is a point test, so the yaw the bot runs at does not decide
whether it catches the ladder; it only has to arrive. Each sample tries two yaws: toward the nearest point of
the ladder's volume in XY, and `climbYaw`. The first is what makes a **side** dismount reversible, where the
bot must come at the column from the side and the two yaws are 90° apart: test53_ladder4's `6 <-> 4` steps
off sideways to (428.7, 30, 19.2) and gets back by walking to (370, 30.8, 19) and running west at yaw 180;
a head-on run at `climbYaw` would run along the face and never into it.

Since the run-off yaw and the climb yaw then differ, the descent needs both. The climb yaw is `exitAngle`;
the run-off yaw goes in `gateAYaw`, which nothing else reads on a ladder. The planner hands them to the mount
waypoint as `ladderYaw` and `ladderEntryYaw`.

Candidates rank by **time to the grab**, not first hit. Edge order would otherwise decide: test53_ladder4's
`6 <-> 4` would resolve to a 161-unit fall down the side of the pit that clips the column near the bottom,
instead of the 18-unit step off the lip directly above. Shortest grab is the closest, safest, and least
sensitive to follower drift.

Both tests only go two-way when a simulation shows the bot ending up on the ladder, so neither invents a
route. What stays one-way is a landing the bot cannot fall back to the ladder from, which on the test maps is
always because the drop would have to go *upward*: test53_ladder4's `6 -> 9` and `4 -> 7` dismount onto the
pit floor at z −238, below the bottom of their ladder. test29_ladder3's `4 -> 2` is the other shape, a ledge
that does not overhang the column; its other two ladders are the drop case, which lets it descend into the
pit instead of taking a 256-unit fall for 10.5 damage.

### What the connection carries

No ladder-specific fields. `launchPos` is the mount, `landingPos` the dismount, `exitAngle` the yaw held to
climb, `exitSpeed` the climb speed, `flightTime`/`fallHeight` come from the winning dismount, and the climb
length is `|landingPos - launchPos|`. Gates are degenerate points: A at the mount, B at the descent entry,
which is why `CONN_LADDER` is exempt from `BuildConnectionGates` and the missing-gate counter. Gate B is the
only place the descent entry lives, so the planner reads it rather than `landingPos` when `fromId` is not
`areaA`, with `gateAYaw` as the run-off yaw.

Ladder connections count for `PruneUnreachableAreas` and A* expands them like any other connection, in both
graph builders (`main.cpp` and `src/nav/nav_path.cpp`). The planner turns one into a mount/dismount waypoint
pair (`doc/path_planning.md`); the climb itself is the follower's (`doc/movement.md`). `--dump-path` shows the
climb on test12_ladder, test25_ladder2, test29_ladder3 and test53_ladder4.

## teleport_connections.cpp

### BuildTeleportConnections

Runs after `BuildLadderConnections` and before the prunes, so a teleport can rescue areas nothing else
reaches and is still there when `PruneUnreachableAreas` runs. Emits one-way `CONN_TELEPORT` connections for
every usable `trigger_teleport`.

### What is skipped, and why

- **`SF_TRIGGER_NOCLIENTS` (spawnflag 2)** - `TeleportTouch` returns early for players.
- **A non-empty `master`** - `UTIL_IsMasterTriggered` gates the touch on a `multisource` whose state is
  whatever the map's logic has done to it, which navgen cannot know. Emitting the edge anyway would need
  runtime failure attribution to stop a bot walking repeatedly into a dead teleporter; the teleport branch
  stamps `connIndex`, but nothing spends it yet. This is why `rustmill` gets no teleport connections: all 48
  of its teleports are mastered (it rotates which rooms are live). `gasworks`, with 3 ungated teleports, gets
  12, one per area that can reach a trigger.
- **A `target` matching no entity, or a destination off the mesh.**

### The destination is found the way the engine finds it

`FIND_ENTITY_BY_TARGETNAME` takes the first `targetname` match *whatever its classname* --
`info_teleport_destination` has no special standing, and `test21_teleport` targets an `info_target`. The
landing origin is `dest.origin` with the feet offset removed and one unit added (`tmp.z -=
pOther->pev->mins.z; tmp.z++`), a **standing origin in world space**. `kLandingLift` drops it by
`kStandProbeLift` into nav space. Getting that wrong costs exactly 18 units and shows up as a
plausible-looking wrong drop, not a crash.

### The drop is real and it is priced

The engine clears `FL_ONGROUND` and zeroes velocity, so a destination hung above its floor is a fall.
`ResolveLanding` looks for an area whose `boundPlanes` contain the landing point, then falls back to the
highest area floor directly beneath it -- deliberately *not* `FindAreaContainingPoint`, whose nearest-centroid
fallback would invent an edge to an unrelated area. The drop feeds `flightTime` and `PmSim::FallDamageFor`,
and `ConnectionCostSeconds` prices `CONN_TELEPORT` like air moves and ladders, so a lethal drop takes the
`1e4` penalty and is never routed. A level destination costs 0.

### Source areas reuse FindServicePoints

"Which areas can walk into this brush, and where would the bot stand" is what `FindServicePoints` solves for
trigger activators, so a teleport builds an `ActivatorVolume` with `kind = ACT_TRIGGER` and hands over the
**raw** brush bounds; that function does its own expansion by each area's hull. One connection per source
area, deduplicated on the *ordered* `(from, to)` pair, keeping the lower fall damage.

It inherits one limit from `ServiceableArea`: gated areas (door, breakable) are never sources, so a teleport
trigger inside a doorway is missed.

### What the pruners do with them

Nothing. `PruneWalkableAirmoves` and `PruneRedundantLadders` neither judge teleports nor accept them as
alternatives, so a teleport is never dropped and never makes an air move look redundant. Conservative: a free
teleport arguably *should* retire a parallel air move, but that is a change to the prune.

## airmove_prune.cpp

### PruneWalkableAirmoves

Runs after `BuildTeleportConnections`. For each `CONN_AIRMOVE` it asks what the same two areas cost on foot
and drops the air move when walking is not much worse: `walk < kDefaultAirmoveWalkRatio * air`, ratio 1.2,
tunable with `--airmove-walk-ratio` (0 keeps every air move). `--dump-airmove-prune` prints each comparison.

Both sides are measured alike -- centroid to centroid at `maxSpeed`, plus `ConnectionCostSeconds` -- so the
air move pays for its `flightTime` and fall damage against a walk that pays only distance. The walk query is
a Dijkstra (`WalkDijkstra`) over **walk connections only**, memoised per source area.

Two properties make one pass safe. Reachability cannot change, because a connection is dropped only when a
route between the same endpoints already exists **in every direction the connection offered**. And the
result does not depend on visiting order, because the judging graph never contains the edge type being
judged, so dropping one cannot alter a later query.

Both halves of the first property need care, and getting either wrong silently orphans areas:

- **The graph is directed.** A one-way connection contributes one edge. Traversed backwards, two one-way
  drops *out* of test53_ladder4's crouch tunnel make a 64-unit climb look free, which deletes both ladders
  that are the only way *in*, and the tunnel and the room behind it (with the map's `info_target`) prune
  away as void.
- **A bidirectional connection is dropped only when the alternative works both ways**, so the query takes
  the worse of `A->B` and `B->A`.

**A jump the planner prefers is not automatically worth having.** A* ranks edges on crossing distance plus
`ConnectionCostSeconds` and cannot see what the approach corner will cost, so it picks jumps the profile then
rates slower than walking: on `test37_walkent` the 5→4 crouch-jump route came out at 3.5s against 3.2s for
walking round the pit, over an identical 888 units. That is the connection this pass drops (walk 0.81s vs
air 1.43s).

Most of what goes is stairs and slopes, where the sampler finds a jump between nearly every pair of treads:
`test15_staircase` 33 air moves to 7, `test50_stairs` 20 to 1, `test28_slopecase` 19 to 8, `test45_slopes2`
17 to 8. Areas, walk connections and route existence are untouched. The cost is real: routes are slower where
a discarded jump was a genuine shortcut that did not beat walking by the ratio, and the ratio is the knob if
those maps matter more than graph size.

One-way drops survive on the no-walk-route branch, which is most of what is left: 21 of `test37_walkent`'s 30
air moves fall into a pit with no way back out, and with no walk route the air move is load-bearing.

### PruneRedundantLadders

The same machinery, enforcing **walk > jump > ladder**. It judges `CONN_LADDER` against walk connections
*plus* the default-gravity air moves the first phase kept, and drops the ladder when `altSeconds <
kDefaultLadderRatio * ladder`, ratio 2.0, tunable with `--ladder-prune-ratio` (0 keeps every ladder). A
ladder gets the looser ratio because climbing is slow but reliable: within 2× of the alternative it is worth
keeping.

Both safety properties carry over because the graph is directed and bidirectional connections are checked
both ways; this phase is what exposed the directed-graph caveat above. On `test53_ladder4` it drops 2 of 6:
the degenerate 3-unit "climb" and a horizontal jump-off walking already covers. The two long climbs and the
two crouch-tunnel ladders stay, the latter because nothing else reaches the tunnel.

## swim_connections.cpp

`BuildSwimConnections` runs after floor snapping and water tagging and emits `CONN_SWIM`, each carrying its
water volume and a `submergeZ`.

**Water to water.** Two areas of the same water volume whose boxes touch with at least 8 units of overlap
and share a face get a two-way swim link. A swimmer hangs a hull under the ceiling the way a walker stands
one on the floor, with 2 units of slack (`kSwimCeilingSlack`) so a crossing on a seam plane does not touch
the solid the seam runs along. Across a shared wall the link is a window both areas fit through, hung under
the lower of the two ceilings -- the mirror of a walk gate on the shared floor. Stacked areas get no common
height, since that link exists to change depth. Reachability is a duck-hull trace through the window, or
point to point for stacked seams, whose endpoints already sit in their own areas.

**Dry to water.** For a dry neighbour, candidate edge points are tried until one has a clear line from the
dry floor out over the water and down to the swim height. The dry floor's height above the high surface
decides the link:

- at most `kWaterExitStep` (18) above: a **beach**, two-way;
- at most `kWaterJumpLedge` (34): a one-way **waterjump** out (`CONN_FLAG_WATERJUMP`) plus a one-way
  **dive** in;
- higher: the dive only.

`--dump-water` prints every link (`swim`, `beach`, `wjump`, `dive`) and every pair `walled` off by geometry.

## reachability.cpp

### FindSpawnAreas / FindTargetAreas

Spawns are `info_player_start`, `info_player_deathmatch`, `info_player_coop`, `info_player_teamspawn`,
`info_player_allies` and `info_player_axis`; targets are `info_target`. Each parses `origin` and finds the containing area (`boundPlanes`, ε = 1.0),
falling back to the nearest centroid.

**Targets are never seeds.** They exist to validate reachability, so they only resolve to areas; the prune
seeds from spawns alone.

### PruneLethalConnections

The route half of the `trigger_hurt` rule (`DropLethalFloorLeaves` removes the floors). It drops routes that
pass through a kill volume between two safe floors -- a jump across a lava sheet, a drop past a kill plane
onto a ledge below. The chord from `pointA` to `pointB` is sampled every 8 units and tested at the duck
origin and 36 above it, which together cover a standing bbox in nav space. `CONN_TELEPORT` and
`CONN_ELEVATOR` are exempt: neither travels through the space between its endpoints.

A chord is not a trajectory -- a jump arc bows above it -- so this is conservative for jumps on purpose: a
route whose straight line clips a 100+ damage volume is one where a small navigation error is fatal.

It is unexercised in practice: across the 50 test maps and 19 HLDM maps it cuts zero connections, because
`DropLethalFloorLeaves` already removes every floor those routes lead to. Without the leaf drop it cut 550 on
xen_dm.

### PruneUnreachableAreas

BFS from the spawn areas over the connection graph, honouring one-way connections and drift blocking at
`--max-speed`. Area ids are remapped to a compact 0..N-1 and connections filtered; everything that keys on an
area id runs after this.

## activation_detours.cpp

### Why precompute

An activator is rarely in the door's own area, and the route to it depends on *(door, which side you came
from)* -- static geometry, so it belongs in navgen. At search time it would be a sub-A* per A* expansion (the
walker is copied per node), and putting activation in the search state means revisit waves.

The stage runs after `PruneUnreachableAreas` (which renumbers area ids, so `fromAreaId` would otherwise be
garbage) and after `BakeConnectionGates` (the polyline anchors on a baked gate midpoint).

### FindServicePoints

Where the bot stands to work an activator. Gated areas are never service areas: you cannot stand inside a
closed door to press the button that opens it. Containment uses `boundPlanes`, never the polytope, since
areas can be non-convex prisms.

- **Trigger** -- touch is an AABB overlap of the player hull with the trigger's `absmin/absmax`, so the
  origins that touch it are `[act.mins − hullMaxs, act.maxs − hullMins]`. Each floor winding is clipped
  against that box's four vertical planes; a survivor over `kMinServiceArea` yields its centroid at
  player-origin height, with the origin z checked against the box's z span.
- **Button** -- GoldSrc `+USE` is `SV_FindEntityInSphere(pev->origin, 64)` against the brush model
  **centre**, then a facing dot test, with **no trace**: pressing a button through a thin wall works in
  vanilla HL. So the test is 3D distance from the player origin to `act.center` ≤ `kUseRange` (56, leaving
  margin), with no line-of-sight check -- one would reject the *common* case, since a flush-mounted button has
  its centre inside world solid. The range is 3D from `floorZ + 18`, so a button high on a wall is out of
  reach from directly beneath it.

A button sample is rejected when the player hull there would overlap the activator brush (`HullClearsBox`,
the same origin-space box the trigger case uses). `func_button` is not a blocking entity, so
`StandablePoint` alone returns points inside the button, and since samples rank by closeness to the centre,
that unreachable point always wins; the bot then jams against the brush short of the waypoint, `progressS`
never advances, and it stall-repaths forever.

### BuildActivationDetours

Per gated area D and per **directed walk** connection `X → D`, `PlanRoute` runs from the X-side gate midpoint
to each candidate service point and back (planned separately: gate string-pulling is not symmetric). The
cheapest pair becomes one closed polyline that starts and ends at that gate midpoint, with `actIndex`
marking the activation waypoint.

Non-walk entries into a gated area are rejected outright: the teleport, ladder and air-move branches of
`PlanWalker::Cross` return *before* the door check, so a jump landing inside a door area would emit no
activation waypoint.

**Bootstrapping needs no flag.** The planner rule is a lookup -- no detour for this neighbour ⇒ refuse the
edge -- so during the pass every gated area has an empty `detours` and seals itself. Results go to a side
buffer and are written back only after the last `PlanRoute` returns, so nothing depends on door order. That
is also the no-recursion rule: a button behind another button-door reads as blocked, and `--dump-activators`
says so.

The precompute graph is walk-only, so the stored polyline carries no air-move/ladder/teleport flags and
`traces` can be `nullptr`.

**Known limitation:** the detour starts at the door's own gate, so a bot walks up to the door, turns round to
the activator and comes back. Plausible, not optimal; diverging at the ideal point would need activation in
the search state.

## vantage_sampler.cpp

### What a vantage is

A sampled eye position with a visibility signature: where a bot standing there should watch, how exposed the
spot is, and how much of that exposure one facing covers. `BuildVantages` runs after `PruneUnreachableAreas`
and on every invocation unless `--no-vantage` is passed. Without the section bots fall back to no idle look
and unweighted senses (`doc/perception.md`, *Where the bot looks when it has no target*).

| Field | Meaning |
| --- | --- |
| `watch[]` | up to 4 `WatchDir{yaw, pitch, spread, amount, dist}`, `amount`-descending. `amount` is the fraction of everything that can see this spot arriving from that cone; `spread` the weighted RMS angular deviation inside it, in degrees; `dist` the mean length of the sightlines in it |
| `exposure` | fraction of all sampled vantages in the map with line of sight to this one |
| `visibility` | how visible a body standing here is against its background, 0 to 1 (see "Visibility") |

**It is the dominant cost of a run.** On crossfire the field took a run from 8.8s to 47.8s at `--threads 13`
when it first became a section: 38.7s of that was the field and 36.4s of *that* the pair loop. The full
50-map sweep in `generate_all_svgs.ps1` ran in 164s. `--no-vantage` exists for iterating on other stages.

### The internal datum is a direction histogram

Sampling accumulates a yaw × pitch histogram per vantage (64 × 8 by default), plus a parallel tally of
sightline lengths, and everything above is derived from them. The histogram is working memory, never output.
It is what makes the reduction well-defined: a fixed-length vector has a distance, a variable-length cluster
list does not.

Pitch is binned over ±45° with clamping rather than ±90°. Steeper sightlines are rare and their exact pitch
does not matter, and spending the bins near the horizon keeps the narrow cones the camp sampler seeds from
spanning a single bin.

### Grid sampling

The lattice is **world-aligned**, not area-aligned, so neighbouring areas agree on cell centres and dedup is
exact. Its spacing is derived from the walkable footprint so a map gets about `--vantage-points`
(`kDefaultVantagePoints`, 40000) samples, because pair cost is quadratic in the count: `grid = max(15,
sqrt(footprint / points))` (15 is `kDefaultVantageGrid`). Crossfire (7.4M sq units) keeps 15, dod_caen
(15.3M) samples at 19.5 and koth_one (38.2M) at 30.9, all near 40k vantages; a fixed 15 gave koth_one 170k
vantages and 14.3G pairs. `--vantage-grid n` pins it.

For each lattice point every walkable floor face of the area is tested with `Distance2DToWinding` and
contributes its own `FloorHeightAt` -- per face, not per area, because a prism area (stairs, a ramp) carries
several floor faces at different heights and each is a real standing position. Two candidates in one lattice
column within 32 units of z are the same spot and merge; a player cannot occupy two floors that close. An eye
point that classifies `CONTENTS_SOLID` in hull 0 is dropped.

**Eye height.** Nav z is the duck origin. With HLSDK's `VEC_VIEW.z = 28` and `VEC_DUCK_VIEW.z = 12`, that
gives `kDuckEyeLift = 12` (nav z + 12 = floor + 30) and `kStandEyeLift = 46` (floor + 64). A
`NAV_AREA_CROUCH` area uses the duck lift; everything else the mean, +29, because the bot may be in either
stance.

### Pair traces

Vantages are bucketed by hull-0 leaf and the pair loop walks them in leaf order, so each worker's
`VisRowReader` hits its one-row PVS cache on essentially every query. Leaf index is depth-first BSP order and
already spatially coherent, which is why the loop keeps it instead of Morton-sorting when it bins work across
threads. A pair that fails the PVS is skipped outright: on crossfire that removes two thirds of the 553M pairs
before any trace.

**World traces are batched per visleaf.** Each leaf's vantages (contiguous in leaf order) are cut into chunks
of at most `kVantageBatchChunk` (32), and every PVS-visible chunk pair is one `SegmentsBlockedBatch` (see
`clipnode_tree.cpp`). The chunk cap bounds the per-thread scratch, which grows with combos × tree depth. A
chunk owns its pairs with every later chunk, so its cost for thread binning is exactly the sum of its slots'
costs; inside one chunk only the pairs above the diagonal are new.

A world-clear pair is then tested against blockers: for any blocker whose box the segment crosses, one trace
against that blocker's own hull-0 tree (`BlockersOccluded`). The blocker set is `ExtractVisBlockers` -- the
*permanently* solid brush entities only, at hull 0 and without the movement hull's expansion. Doors and
breakables are excluded: they open, and a mesh that assumes them shut is wrong most of the time.

Visibility is symmetric, so one trace updates both endpoints' histograms. Every vantage represents the same
footprint cell by construction, so a uniform weight of 1 per pair is already area-proportional.

### Sight coverage

A traceline sees through `func_illusionary` and `.mdl` props and is stopped by glass -- both wrong for
sight. `ExtractSightCoverage` rasterizes the same models the runtime does (`src/nav/sight_coverage.cpp`; how
and why is `doc/perception.md`, "What the bot sees through"), from the entity lump instead of live edicts:
every `func_illusionary`, every permanently solid blocker whose `rendermode` is not normal (these then leave
the traced blocker set), and every `env_model` and `cycler_sprite` with a `.mdl`. Brush angles are 0, because
every one of those classes zeroes them in `Spawn`. `env_model` takes its sequence from `m_iszSequence_On` by
name, and spawnflag 2 drops it to the floor with a hull-0 trace. A sightline is blocked once `1 -
product(1 - alpha)` over the coverage boxes it crosses passes `kSightBlockOpacity` (0.5); `BlockersOccluded`
handles a coverage box in place of a hull trace, so the samplers need nothing else.

Wads and models are read relative to the BSP (`<bsp dir>/../<wad basename>`, `<bsp dir>/../<model path>`),
falling back to `<bsp dir>/../../valve/`. A missing wad or texture is drawn opaque, and every miss is a `note:`
line under `--dump-coverage`. On dod_forest the whole set (223 entities, 97 models, 1714 chunks) builds in
2.5s on all cores.

### Watch directions: peak extraction, not DBSCAN

DBSCAN on a sphere with a fixed `eps` degenerates in open areas: everything merges into one cluster. Instead,
on a working copy of the histogram (`ExtractWatchDirs`): seed at the max bin, mean-shift three passes over the
bins inside `--vantage-cone` (default 45°) of the current direction, record the cone's weight and spread,
divide the length tally by the count over exactly those bins for `dist`, zero the bins, repeat. Stop at 4
directions or when under 5% of the weight is left. This is the refinement `LookDirection` does in
`bot_sense.cpp`. Camping is not derived here; see `camp_sampler.cpp`.

### Reduction: the error metric must be scale-free

Greedy decimation, lowest interpolation error first, until `kept <= footprint / spacing²`
(`--vantage-spacing`, default 128). For each candidate the `--vantage-neighbors` (default 4) nearest
survivors are found, IDW-weighted by `1/(d²+1)`, and the candidate is scored on how well that interpolation
reproduces it. The only stop guard is never dropping below `neighbors + 1` vantages.

**The signature is split into shape and magnitude, and this is load-bearing.** Raw L1 distance between
un-normalized histograms is dominated by magnitude: an enclosed spot has a small vector, so its absolute
error is small however distinctive it is, and it is deleted first. On crossfire that kept a set averaging
exposure 0.306 against a population mean of 0.158 -- the open outdoor bowl kept, the bunker interiors
erased, exactly backwards. Normalizing each histogram to unit L1 and scoring exposure separately (weight
`kExposureErrorWeight` = 2 so the terms are comparable) brings the kept set to 0.133, slightly below the
population mean, and restores the interiors.

### Visibility: how dark the spot is to whoever sees it

After the watch directions, every candidate gets a `visibility` in [0, 1] from the map's lightmaps
(`Nav::LightMap`, shared with the plugin; see `doc/perception.md`, *Shadows*):

```
Lt          = light under the probe          Sample(eye, eye - 2048z)
Lb[d]       = background seen from d          Sample(eye, eye - dir(d) * 4096)
visibility  = sum amount[d] * LightVisibility(Lt, Lb[d]) / sum amount[d]
```

A watch direction points at the seers, so the background they see behind the probe is the first world face in
the opposite direction. Weighting by `amount` lets the side most of the map watches from decide. With no
watch directions, `Lb = Lt`. Light styles start as the map spawns them: a `light*` entity with `style >= 32`
and spawnflag 1 sets that style to `a`, its `pattern` otherwise; every other style is `m`.

- **The runtime multiplies it into exposure.** `BlendVantage` blends `exposure * visibility`, so everything
  that reads exposure (cover choice in `bot_combat.cpp`, careful routing through `conn.vantageExposure`)
  treats a dark spot as hidden. `maxExposure` stays the raw maximum.
- **The reduction scores the product too** (`exposureNorm`), so decimation keeps a dark pocket rather than
  interpolating it away from lit neighbours. Camp gating reads raw `exposure`.

The stats line reports `visibility mean`, or `(no lightmap)` when the BSP has none. Mean visibility was 0.932
on crossfire and 0.57-0.94 over the first six test maps.

### Nearest-neighbour queries

A uniform spatial hash over vantage positions, cell size `max(32, grid*2)`. The ring search **expands until
the k-th best distance beats the guaranteed radius** `r * cell`: a point outside the searched Chebyshev-r box
of cells is at least that far away. A fixed one-ring search silently returns wrong neighbours near cell
boundaries and as the kept set thins, which would corrupt every reduction decision;
`--vantage-verify-neighbors` cross-checks every query against brute force. It is expensive and timed
separately so it never inflates the reduce figure: at spacing 512 it added 290s to a 1.9s reduction.

### Where the time goes

Measured on crossfire at defaults (33266 vantages, 553M pairs), 6-core / 12-thread box, `--threads 13`
against `--threads 1`, with per-pair hull-0 traces (before batching and before the hull-check changes below):

| Phase | Serial | 13 threads |
| --- | --- | --- |
| Candidate grid | 0.0s | 0.0s |
| Solid rejection + leaf assignment | 0.0s | 0.0s |
| **Pair loop** | **276.2s** | **40.1s** |
| Derive (histogram to watch dirs) | 0.6s | 0.7s |
| Reduction (33266 down to 113) | 1.9s | 1.9s |
| Whole navgen run | 313.8s | 52.6s |

The pair loop is the entire cost and where all the parallelism went: **6.9x** on 6 physical cores. Of the
276.2s serial, hull-0 traces were 213.9s (184.8M traces at 1.16us); the rest is the PVS test and binning over
553M pairs.

Later changes, as whole-run wall time at default threads on the same box, alternating old and new binaries in
one sitting because run-to-run variance swallows anything under ~5%:

| Change | crossfire | boot_camp |
| --- | --- | --- |
| `/Ob2` on the navgen target | 59.0s to 46.0s | - |
| `SegmentBlocked` for visibility | 42.0s to 37.7s (−10%) | 133.9s to 113.7s (−15%) |
| Shared-origin chain cache | 40.8s to 34.5s (−15%) | 110.4s to 94.6s (−14%) |

The `SegmentBlocked` rows are means of four alternating pairs on crossfire and two on boot_camp; every new
run beat every old one. `.nav` output was byte-identical across both changes on all maps under `maps\` plus
crossfire and boot_camp. `SegmentsBlockedBatch` has since replaced per-pair traces in the pair loop (~58ns per
combo against ~550ns per `SegmentBlocked`, see `clipnode_tree.cpp`).

`traceSeconds` in the stats line is *summed across workers*, not wall time: CPU time, which also inflates
per trace under contention for memory bandwidth and shared L3 (1.16us serial to 2.06us at 13 threads with
per-pair traces). It is measured around each batch call, and kept because the trace-versus-PVS split is
what says whether traces are still the thing worth optimising.

**Why the tallies are atomic integers.** A pair `(i, j)` writes both `i`'s and `j`'s histogram, so no
partition of the loop gives a worker exclusive ownership of what it writes. Both tallies are flat
`vector<atomic<uint32_t>>` incremented with relaxed `fetch_add`: the count by 1, the length tally by the
truncated sightline length. Integer addition is associative, so unlike float accumulation the result is
bit-identical whatever order threads interleave in. (C++17 does not guarantee that value-initialising an
atomic zeroes it -- that is C++20's P0883 -- so the arrays are explicitly stored to 0 first.)

**Why the cost model counts PVS hits exactly.** `BuildWorkBins` needs a cost per slot, and `count - 1 - a`
alone bins badly: a tunnel vantage's PVS admits a handful of partners where a courtyard vantage's admits
thousands. Since every leaf's vantages occupy a contiguous slot range, the exact number of PVS-admitted
partners after slot `a` is a sum over the visible leaves with a higher index -- O(leaves²) for the whole
map, negligible. Cost is `(count - 1 - a) * kVantagePvsCost + traced * kVantageTraceCost` (1 and 18, the
measured per-trace to per-PVS-test ratio). A heuristic share of the leaf's whole row left the bins at a
2.7x spread; the exact count brought it to 1.29x (40.1s slowest, 31.1s fastest), which the stats line reports
so it stays checkable.

**The real levers are upstream.** The dominant term is N² in vantage count, which is why the lattice spacing
is derived. And vantage count is footprint over `grid`², so pairs go as **footprint squared**: junk floor is
paid for twice. xen_dm at 44.4M sq units would have needed 221k candidates and 2.5e10 pairs, about an hour;
with the void floor removed it is 12.2M sq units, 53k candidates, 1.4e9 pairs and 146s. If a map's vantage
stage looks absurd, read the `budget ... over N sq units` figure in the stats line before touching
`--vantage-grid`; `--dump-sky` says whether that N is real floor.

## camp_sampler.cpp

### Camp spots are a separate dataset, not a field

The vantage field answers "how seen is it around here" and "which way do sightlines run around here", both
properties of a neighbourhood, so the runtime blends the four nearest samples and gets a meaningful answer
anywhere. Camping is discrete -- stand *exactly* here, face *exactly* there -- and the midpoint between two
camping spots is usually not one. Camp spots are therefore their own reduced list with their own record
(`NavCampRecord`, `nav_format.h`):

| Field | Meaning |
| --- | --- |
| `pos` | the nav floor point, which is a ducking player's origin |
| `yaw`, `pitch` | one facing, shared by both stances |
| `standScore`, `standSpread`, `standDist` | from the standing eye, `pos.z + kStandEyeLift` (+46) |
| `crouchScore`, `crouchSpread`, `crouchDist` | from the crouching eye, `pos.z + kDuckEyeLift` (+12) |
| `score` | `max(standScore, crouchScore)`, what selection ranks on |
| `flags` | `CAMP_NO_STAND` when standing there is impossible |

The nav floor is the hull-3 centre, so it *is* a ducking origin: crouch eye = floor + 12 (the duck
`view_ofs`), standing origin = floor + 18, standing eye = floor + 18 + 28 = floor + 46. Both lifts come from
`vantage_sampler.h`.

`spread` is the RMS angle the targets inside the FOV sit off the axis, in degrees -- how wide an arc a camper
there must sweep. `dist` is their mean range. One spread answers "how concentrated is this" directly, which
four nested arc ratios per stance (FOVs 90 / 45 / 22.5 / 11.25) only approximated, and no arc carried a
distance. `BotCamp` reads all three: spread and dist decide the scope, the scores decide the stance
(`doc/goals.md`).

### The FOV metric is odds, not a fraction

For one 90° FOV (half-angle `kCampFovHalfDeg`, 45), count visible probe candidates inside the cone and
outside it, per stance:

```
score(stance)  = inside / max(outside, 1)
spread(stance) = sqrt(sum of angle^2 over inside / inside)     degrees off the axis
dist(stance)   = sum of range over inside / inside
score          = max(stand_score, crouch_score)
```

The odds form `f / (1 - f)` beats the fraction `f` because a fraction saturates exactly where camping
quality lives: 0.90, 0.95 and 0.99 look alike, but as odds they are 9, 19 and 99. `max(outside, 1)` rather
than `outside + 1` keeps a perfectly focused spot's score equal to its visible count, so "sees a lot, all in
front" outranks "sees one thing, technically in front". A large score means "you can see a lot without
turning your head"; tightness is reported separately as `spread`, so a wide overlook and a sniper's slot can
score alike and still be told apart.

**The dynamic range is enormous.** On crossfire the score runs into the hundreds, because `max(outside, 1)`
divides by 1 whenever nothing is behind you; the top spots are alcoves that see hundreds of targets with none
behind. That ranking is right, but see *Spacing does not control spread* for what the range costs.

**The minimum-visible gate is load-bearing.** A spot that sees a single target with nothing behind scores 1.0
and competes with real rooms once the odds form is applied. `--camp-min-visible` (default 32) removes it,
applied on `exposure * (count - 1)` before a candidate enters the queue. It counts probes at the
`kDefaultVantageGrid` lattice; a sparser lattice needs proportionally fewer to cover the same seen floor area,
so the effective value is `max(1, round(minVisible * (kDefaultVantageGrid / grid)²))`.

### Selection: cheap ordering, expensive confirmation

Camp spots are drawn from the **pre-reduction** probe candidates -- the full lattice, after the pair loop has
given each one an `exposure` and a `watch[]`, before decimation discards most of them. `BuildVantages` hands
that snapshot back through `candidatesOut`; no second lattice is sampled. The cheap data already in memory
decides *where to look*, and the expensive two-stance visibility test runs only around promising places:

```
gate    candidates seen by fewer than --camp-min-visible probes, plus water and elevator areas
score   preliminary = watch[0].amount / exposure
loop    pop the best remaining candidate
        hill-climb to the local maximum of the FOV score
        emit the summit; consume everything the climb consulted
        penalise survivors by min(1, d / spacing)
until   budget reached or the queue empties
```

`watch[0].amount / exposure` is main-direction strength over exposure: a high `amount` means "what sees me,
sees me from one place", a low `exposure` "not much sees me at all". Both are on every candidate already, so
the ordering is free. The budget is `kCampBudgetScale * footprint / --camp-spacing²` (scale 2), or exactly
`--camp-count`. The scale is 2 because objective maps split the budget into lanes (see *Objectives get balanced
lanes*): at scale 1 dod_avalanche's 108 spots left 19 per flag.

### Hill climbing, and why a climb cancels

The preliminary score is a proxy at one eye height from a bin histogram: it finds a promising *basin*, not the
best point in it. Each pop is a starting point: evaluate the candidate and its unvisited lattice neighbours,
step to the best, repeat. Neighbours are candidates within `grid * 1.5` in XY (reaching the diagonals) and 32
in z, so a climb follows a slope but never jumps between stacked floors. `--camp-climb-steps` (default 32)
caps one climb.

A step is taken only when the best neighbour beats the current point by `kCampMinGain` (2%). Without the
margin a climb crawls across a near-flat ridge one cell at a time: on xen_dm the mean climb was 6.5 steps and
112s; with 2% it is 3.8 steps and 59s for the same 182 spots.

**A climb whose ascent leads into a candidate an earlier climb consumed is cancelled.** It is re-walking
someone else's hill from lower down, and that climb already emitted the summit; stopping there would emit a
second spot partway up the same slope. The test is *ascent*, not adjacency: a consumed neighbour scoring lower
than where we stand means we are on our own summit with someone else's basin beside us, and that emits
normally.

**A cancelled climb still suppresses.** Suppression covers `spacing` around the points the winning climb
*walked*, roughly a dozen cells, while the basin draining into that summit is hundreds of cells wide. Without
suppression on cancel every pop in the untouched part of the basin climbs into the consumed summit and
cancels: on xen_dm 758 cancelled climbs against 182 emitted, 83% of all evaluation thrown away. Suppressing
on cancel drops it to 16.

Every score is cached, including for candidates that were only rejected neighbours, so a candidate is
evaluated at most once in the stage. *Cached* and *consumed* are different states: a score is cached by
whoever computed it, but a candidate is consumed only when a climb consulted it. That distinction is what lets
the worker pool exist.

### Objectives get balanced lanes

The runtime only picks overlook spots from camp spots (`BotTactics::Overlooks`, `doc/goals.md`), so every
objective needs camp spots that see it. `BuildCampObjectives` lists every objective at its map-start position,
whatever the runtime will later make of its state:

| game | objectives |
| --- | --- |
| CS | `func_bomb_target`, `info_bomb_target`, `func_hostage_rescue`, `info_hostage_rescue`, `func_vip_safetyzone`, `func_escapezone`; all `hostage_entity` as one; all `info_player_start` as one rescue objective when there are hostages and no rescue zone |
| DoD | `dod_control_point`, `dod_capture_area`, `dod_object`; `func_breakable` with spawnflag 64 when the map has any `dod_*` entity. A capture area whose `target` is a control point joins it, so a flag counts once. `dod_object`s sharing an `object_group` are one objective: dod_charlie has 21 satchels in group `tnt` |
| TFC | `item_tfgoal`; `info_tfgoal` / `i_t_g` with detpack activation, or touch activation that scores (`frags` or `increase_team1..4`) |

Each candidate gets a bitmask of the objectives its eye sees (stand eye where it can stand, else crouch eye),
computed once for every candidate on the worker pool before the queue is built. The test is the runtime's, with
the constants shared through `nav_format.h` (`kOverlookMargin`, `kOverlookReach`, `kOverlookSamples`,
`kOverlookChest`): a trace to the entity's point that runs clear or stops within 64 units of its box, or a clear
trace to the floor + 36 centroid of one of the 12 nearest areas within 1 s walk. The walk field is a reverse
Dijkstra over the merged connections priced as the `.nav` prices them. The mask is not written to the `.nav`.

**One queue per objective, plus one for the rest.** A queued candidate goes into the lane of every objective it
sees, or into the "none" lane if it sees nothing; each lane orders by the preliminary score as before. Every pop
takes the non-empty lane with the fewest picks (ties to the lower index, objectives before "none"). A kept spot
counts as a pick for every objective it sees, so a spot over two flags serves both. On dod_avalanche the six
lanes end at 42/42/42/42/41/41 of 216.

**A climb stays in its lane:** from objective k it only consults neighbours that see k, so it cannot walk off the
overlook. Whether a candidate sees anything is its tier (0 or 1); a "none" climb steps to a tier-1 neighbour
whatever the score, and suppression never lowers a candidate of a higher tier than the spot that suppresses it.

Two orderings lost. Sorting one queue by the objective count first gave all 108 avalanche slots to street spots
that see three or four flags (16010 of 31597 candidates see one), so the rooftop overlooks of one flag got none.
Using the count only locally, in climbs and suppression, kept 4 of 9 hand-picked market overlooks: the basins of
the others rank 4648 to 19487 of 31597 on the preliminary score, below where the budget runs out. With lanes and
the doubled budget, 7 of the 8 that have a floor get a spot within 71 units, 6 of them seeing the market. The
precompute is 0.1 s on avalanche. A map with no objectives has one lane.

### Spacing does not control spread

`--camp-spacing` (default 256) sets the budget and the radius of the soft penalty `min(1, d / spacing)`
applied to every survivor after a climb. The penalty is soft so a genuinely excellent spot 100 units from
another loses half its score but can still win, where a hard radius would delete it.

**In practice the budget is enforced and the spread is not.** On crossfire 96% of kept spots are closer to
their nearest neighbour than 256 (median 173, minimum 45). Suppression can only multiply by `[0, 1]`, and when
scores span 100x a spot 100 units from a claimed summit keeps 39% of a score already ten times its
neighbours', so it stays at the top of the queue. Camp spots cluster on the best hills rather than tiling the
map, and `--camp-spacing` behaves as a count knob. That follows from the rule, and the clusters are on real
camping ground; if tiling is wanted, the lever is a steeper falloff (`(d / spacing)^k`) or a hard radius, and
both change what the flag means.

### Cost

The two-stance test runs once per *evaluated* candidate against `footprint / 15²` targets at two eye heights.
With `E` the evaluations per emitted spot -- summit, path, rejected neighbours and everything spent on
cancelled climbs -- and the pair loop's `A² / 101250` pairs:

```
camp trace-pairs / vantage pair-loop pairs = 900 * E / spacing^2
```

`E` decides whether the stage is free or expensive, and geometry cannot predict it: it depends on how rugged
the campiness surface is. Measured at spacing 256 and budget scale 1:

| map | candidates | spots | E | cancelled | camp stage | share of run |
| --- | --- | --- | --- | --- | --- | --- |
| crossfire | 33266 | 114 | 11.5 | 0 | 8.2s | 13% of 62s |
| pool_party | 14998 | 51 | 8.5 | 0 | 2.7s | 4% of 68s |
| xen_dm | 53200 | 182 | 22.5 | 16 | 58.6s | 23% of 254s |

Cost is quadratic in the *inverse* of spacing: `--camp-spacing 64` is 16x this, not 4x. Watch the cancellation
count: one approaching the emitted count means the preliminary score keeps restarting inside claimed basins.

**World traces are batched per visleaf.** Targets are sorted by leaf (`targetOrder`, with their float eyes in
`targetEyes`) and grouped into single-leaf runs of at most `kCampBatchChunk` (256). An evaluation issues one
`SegmentsBlockedBatch` per PVS-visible group, with the candidate's stand and/or crouch eye as the starts, then
reads the results back in candidate order so the target lists keep their order. Blockers are traced per
world-clear pair (`BlockersOccluded`).

### Parallelism: a persistent pool that only fills a cache

`RunBins` spawns and joins `binCount - 1` threads per call -- fine for the one call each that covers the
air-move and vantage stages, ruinous once per evaluation. So threading goes **across evaluations**: a main
thread and a persistent `ThreadPool`.

The main thread runs the serial algorithm unchanged: it pops, climbs, decides and penalises, and asks the
cache whenever it needs a score. Workers only fill that cache; they never mark a candidate consumed, never
touch the queue, never decide anything. That is not stylistic: if being evaluated implied being consumed, how
far the workers happened to run would change which candidates remain, and the output would depend on thread
count and timing. `.nav` files are byte-identical at `--threads 1` and `13` because of it.

Most of the parallelism is not speculative: at each climb step every neighbour's score is needed before the
main thread can choose, so those are dispatched together. Measured batch width is only 3.5 of 13 threads,
because after the first step most neighbours are cached. Feeding idle workers neighbours-of-neighbours as
prefetch would recover some of that and is safe by construction (a wasted prefetch is one cached score nobody
reads); at 8-59s for the stage it is not yet worth the complexity.

Determinism also needs the priority queue to break score ties on candidate index, and the climb to break
neighbour ties on candidate index.

## work_pool.cpp

Static work-binning shared by the threaded stages:

- `DefaultThreadCount()` -- `hardware_concurrency() + 1`, or 4 when detection returns 0. The extra thread
  keeps a core busy while a worker stalls on a page fault or is descheduled; the bins are static, so there is
  nothing to steal.
- `MortonKey(point, cell)` -- Z-order code over the position quantised to `cell`, 21 bits per axis. Sorting by
  it puts spatially adjacent work next to each other.
- `BuildWorkBins(cost, order, tileSize, binCount)` -- chunks `order` into tiles of `tileSize` consecutive
  entries, sums each tile's cost, sorts tiles by cost descending, and assigns each to the currently lightest
  bin (**LPT**). Each bin's items come back in ascending tile order.
- `RunBins(binCount, body)` -- spawns `binCount - 1` threads and runs bin 0 on the caller. `binCount == 1`
  calls `body(0)` with no thread, so `--threads 1` is a genuinely serial reference. Each worker catches into an
  `exception_ptr` slot rethrown after the join: navgen throws `std::runtime_error` on malformed BSPs, and a
  throw crossing a thread boundary would otherwise `std::terminate`.
- `ThreadPool` -- the persistent pool `camp_sampler.cpp` uses.

Tiles are the compromise between the two things static binning wants. Whole-range partitioning gives perfect
locality and terrible balance (the pair loop's cost falls off linearly with the row index); round-robin over
single items balances perfectly and destroys locality. A tile is small enough that LPT has hundreds of pieces
to balance with, and large enough that the traces inside one reuse the same BSP subtrees.

**Output never depends on the thread count.** Every threaded stage keeps scheduling away from the result: the
vantage tallies are integer atomics, the air-move merge sorts samples back into a canonical order before
anything reads them, and camp workers only fill a cache. `--threads 1`, `3`, `7` and `13` produce
byte-identical `.nav` files on all 50 test maps and on crossfire.

## nav_file.cpp

`WriteNavFile` serializes the result in the `nav_format.h` layout. Every record is written raw with
`fwrite`, which is why each has explicit padding and a `static_assert` on its size: a field that shifts the
layout fails the build instead of silently corrupting nav files.

- **Area planes** come from `boundPlanes`, not `polytope.faces` (see "Area geometry vs exported region").
- **`connOffset`/`planeOffset`** are `uint16_t` into the global arrays and throw past 65535 via
  `ToU16Count`; the detour offsets are `uint32_t`.
- **Vantage `exposure` is raw**, the fraction of *all* sampled vantages that see the spot, before the
  reduction discards most of them. `maxExposure` rides in the header so the runtime can normalise against the
  map's most exposed spot; a pre-normalised value would mean nothing the moment `--vantage-spacing` changed.
- **Unused watch slots** are all-zero and the loader drops them on `amount == 0`, so the vantage record stays
  fixed-size and the section stays one `fwrite`.
- **`vantageSpacing`** is the header's copy of `--vantage-spacing`. The runtime uses it as its grid cell
  size, so it has to survive: recomputing it from the point cloud gives a different answer wherever the
  reduction spent its budget unevenly.
- **`designMaxSpeed`** is the `--max-speed` the mesh was sampled at. navgen never reads it back; it is the
  only way the runtime knows what speed the mesh assumes, and a bot faster than that has movement options no
  connection describes (`doc/movement.md`, `bot_opportunist.cpp`).
- **`actCenter`** on a detour is the activator brush model centre, the only thing the runtime cannot
  recompute: pressing needs the engine's own aim target, and the service point is where the bot *stands*, not
  what it looks at.

## nav_format.h

The on-disk format, version `kNavVersion` (26), magic `CBNV`. `nav_loader.cpp` rejects a version mismatch
outright, so a bump forces regeneration rather than loading a stale mesh.

Section order: header, areas, connections, area-connection index (one `uint32` per connection per area),
area planes (`normal[3]`, `dist`), detours, detour points, vantages, water volumes, camp spots, barriers.

| Record | Size | Fields |
| --- | --- | --- |
| `NavHeader` | 132 | magic, version, mapName[64], bspChecksum, flags, areaCount, connCount, areaConnIndexCount, areaPlaneCount, detourCount, detourPointCount, designMaxSpeed, vantageCount, vantageSpacing, maxExposure, waterVolumeCount, campCount, barrierCount |
| `NavAreaRecord` | 88 | id, mins/maxs/centroid[3], areaFlags, connOffset/Count, planeOffset/Count, activation, barrier, openSeconds, detourOffset/Count, waterVolume, barrierFloor, submergeZ, driftVelocity[3] |
| `NavConnectionRecord` | 172 | areaA/B, connType, connFlags, gateValid, costSeconds, travelSeconds, gateA/gateB[2][3], gateAYaw[2], gateASpeed[2], exitSpeed, exitAngle, jumpBackDist, fallHeight, flightTime, launchPos/landingPos[3], rideSeconds, rideSecondsMax, rideWaitSeconds, gravity, waterVolume, submergeZ, drift |
| `NavActivationDetourRecord` | 40 | fromAreaId, pointOffset/Count, actIndex, waitSeconds, detourSeconds, actCenter[3], actKind, flags |
| `NavDetourPointRecord` | 24 | pos[3], areaId, clearance, flags |
| `NavWaterVolumeRecord` | 48 | modelIndex, mins/maxs[3], surfaceZ, travel[3], flags |
| `NavVantageRecord` | 100 | pos[3], exposure, visibility, watch[4] |
| `NavWatchDir` | 20 | yaw, pitch, spread, amount, dist |
| `NavCampRecord` | 56 | pos[3], yaw, pitch, score, standScore/Spread/Dist, crouchScore/Spread/Dist, areaId, flags |
| `NavBarrierRecord` | 28 | modelIndex, mins/maxs[3] (the brush's raw world bounds, not hull-expanded) |

Field notes:

- `fallHeight` is the drop from apex to landing (air move: from the simulated landing speed and its gravity;
  teleport: the drop to the floor; ladder: the dismount hop's). `costSeconds` holds no fall term: each game
  prices falls at runtime ("Falls are priced per game" in `doc/path_planning.md`). Inside navgen,
  `MergedConnection` also keeps HL's `fallDamage`, which still ranks trajectories and dismounts and prices the
  prunes.
- `gravity` is the `pev->gravity` multiplier an air move was sampled at, 1.0 for every other type.
- `drift` is a `DriftTerm` (length, along, perp, alongMin, alongMax), speed-independent by design;
  `driftVelocity` is the area's conveyor plus push velocity in units/sec, used by the speed profile.
- `barrier` is an index into the barrier section or `kNoBarrier` (0xFFFF); warpath has 186 barriers, so it
  is 16 bits. `barrierFloor` names the barrier the area's floor rests on, closed while that barrier is open.
- `waterVolume` is an index into the water section or `kNoWaterVolume`; `submergeZ` is the water level at
  which the area or connection counts as submerged (`kNavSubmergeNever` when never). `WATERVOL_MOVING` marks
  a moving body.
- `NavWatchDir::dist` is the mean distance to everything visible in the cone, which the range bias reads
  through `VantageOpenDistance`.
- `NavCampRecord`: `CAMP_NO_STAND` means standing is impossible and the `stand*` fields are zero.

Enums:

- **AreaFlags**: `CROUCH` (1), `LADDER_TOP` (2), `WATER` (4), `DOOR` (128), `BREAKABLE` (256), `ELEVATOR`
  (512), `BARRIER` (1024).
- **ConnType**: `WALK` (0), `TELEPORT` (1), `AIRMOVE` (2), `LADDER` (3), `ELEVATOR` (4), `SWIM` (5). A ladder
  reuses the air-move fields (see "What the connection carries"); an elevator ride uses `launchPos` as the
  boarding point and `landingPos` as the arrival.
- **ConnFlags**: `ONE_WAY` (1), `NEEDS_CROUCH` (2), `JUMP` (4) -- left the ground under impulse, not
  derivable from `jumpBackDist` -- and `WATERJUMP` (8). There is no reverse one-way bit: a drift field blocks a
  direction only for a given max speed, so `NavDrift::EvalDrift` answers it per search.
- **ActivationKind**: `ACT_NONE` (0), `ACT_BUTTON` (1), `ACT_TRIGGER` (2), `ACT_ATTACK` (3).
- **GateValidFlags**: `VALID_A` (1), `VALID_B` (2), `CLAMPED_A` (4), `CLAMPED_B` (8).
- **CampFlags**: `CAMP_NO_STAND` (1), `CAMP_DEPLOY` (bit 31).

`ConnBlockedByWater` and `ConnAllowsDirection` are the two inline rules every consumer shares.

## nav_camp.h

### Camp spots are carried, not consumed

The loader lands camp spots on `NavMesh::camps` as a flat vector of `Nav::CampSpot`, which adds runtime-only
fields to the record. The consumer is `bot_camp.cpp` (`doc/goals.md`); `crossbot_camp_status` and
`--dump-camp-svg` are how you look at the data itself.

There is deliberately no spatial index. `FindAreaExact` is a linear scan too, camp spots are a hundred-odd
records, and the one pass over `mesh.camps` runs once per retarget, not per frame; the vantage field has a
CSR grid only because it is blended per bot at 10 Hz. On crossfire: 114 camps, and the pass keeps between 4
and 79 depending on where the bot is and how far off its target is.

## svg_debug.cpp

### WriteDebugSvg

The `--dump-svg` view of the final area graph:

- Areas are the 2D convex hull of their vertices (a prism is its footprint), painted lowest floor first so
  higher areas paint over.
- Connections: white walk, dashed red air move, green teleport. Air moves sampled under a `trigger_gravity`
  are drawn as their own set, so the per-gravity sets are distinguishable instead of overlapping.
- Gates: dashed purple (walk), dashed red (air move).
- Water: surface outlines with their level.
- The planned route is an orange polyline tinted red on slow stretches, with air-move segments overlaid
  (green simulated, red stored, yellow missed).
- The plan-time run-up simulations are drawn over the route they came from; where the two diverge is where
  the mover could not hold the planned line.
- The `--trace` bot trajectory is cyan under the route; targets are magenta crosshairs; the legend is
  auto-sized.

### WriteVantageSvg

A separate SVG (`--dump-vantage-svg`) rather than another overlay: the glyphs need the whole canvas. Areas are
flat grey at 0.35 opacity as a backdrop, in the same painter's order. Per vantage, inside one `<g>` so the
glyph shares a hover tooltip:

- **Watch wedges** -- an arc sector per watch direction, radius proportional to `dist` (how far a fight
  carries that way), half-width equal to `spread` (clamped to 4..80°), one colour per rank, lowest rank
  painted first so the main direction is on top.
- **The dot** -- a blue/yellow/red heatmap over `exposure / maxExposure`, so the most-seen floor in the map is
  red whatever the absolute scale.
- **The tooltip** -- id, position, raw and normalized exposure, and every watch direction's
  yaw/pitch/spread/amount/dist.

World Y is flipped (`x, -y`) like the main SVG, so wedge endpoints are computed in world space and flipped,
and the arc always uses `sweep-flag=1` because the flip turns world counter-clockwise into screen clockwise.

### WriteCampSvg

`--dump-camp-svg` draws one glyph per camp spot. The wedge is the winning stance's `spread` (clamped to
1.5..90°) around the facing, coloured by spread band, with radius proportional to its `dist`; the dot is the
score as a fraction of the map's best, with a heavier ring for crouch-only spots. The tooltip has the
facing, both stances' score/spread/dist and the climb length; the legend reports evaluations, climbs and
cancelled climbs.

## entity_parser.cpp

`ParseEntities` reads the entity lump (brace-delimited `"key" "value"` pairs) into a vector of `Entity`
maps.

## crc32.cpp

`BspChecksumLumps` is the CRC32 of the concatenated planes, clipnodes and models lumps, the standard GoldSrc
BSP checksum, stored in the header so the runtime (`LoadBspWorld`, `src/nav/bsp_world.cpp`) can
reject a nav built for a different BSP.
