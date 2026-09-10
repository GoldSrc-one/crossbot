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
- `--dump-sky` - classify every merged area's floor by the BSP leaf contents just below it, list the
  lethal `trigger_hurt` volumes, and report how much walkable footprint stands on sky, how much stands
  inside a kill volume, and the fifteen largest areas with their bounds. This is the flag that found
  xen_dm's void floor; `world contents empty under the floor` well above a few percent means the mesh
  is standing on something that is not world geometry
- `--dump-airmove` - air-move launch derivation: why no funnel launch was found, every re-simulation
  miss with its launch, yaw, speed and where it landed, every rescaled retry with the speed it
  corrected to and the wanted/travelled distances behind it, and every `retract` attempt with the
  run-up it needed against the run-up it had (see `doc/nav.md`)
- `--dump-planes <id>` - one final area's exact plane list (normal+dist) and face windings
- `--dump-obj <path>` - walkable floor of every area as OBJ mesh (+ MTL): crouch areas colored apart, planned route as ribbon
- `--dump-svg <path>` - top-down SVG of final area graph
- `--dump-vantage-svg <path>` - sample the visibility field and write it as its own top-down SVG: one glyph
  per vantage point with its watch directions and exposure. Tuned by `--vantage-grid`,
  `--vantage-spacing`, `--vantage-cone` and `--vantage-neighbors`; see `vantage_sampler.cpp` below
- `--vantage-verify-neighbors` - cross-check every nearest-neighbour query the reduction makes against a
  brute-force scan and report mismatches
- `--no-vantage` - skip the visibility field. The `.nav` is still valid and still v21, it just carries no
  vantage section, and bots on it lose the idle look and the cone weighting. This is the fast-iteration
  switch: the field is the single most expensive thing navgen does (see `vantage_sampler.cpp`)
- `--dump-camp-svg <path>` - camp spots as their own top-down SVG: one glyph per spot with its
  chosen facing, a wedge at the tightest arc still scoring at least 1, and the hill-climb path that
  found it. Cancelled climbs are drawn in a separate colour; see `camp_sampler.cpp` below
- `--camp-spacing <n>` - radius of the soft suppression a placed camp spot applies, and the divisor
  of the budget `footprint / spacing^2` (default 256). Cost is quadratic in its inverse: 64 is 16x
  the work, not 4x. Note it enforces the count but not the spread; see `camp_sampler.cpp`
- `--camp-count <n>` - override the derived budget with an exact number of camp spots
- `--camp-min-visible <n>` - probes a candidate must be seen by before it can become a camp spot
  (default 32). This gate is load-bearing, not a nicety: without it a spot that sees one thing scores
  a perfect ratio in every arc and outranks a real room
- `--camp-climb-steps <n>` - hard cap on one hill climb (default 32), the backstop for a plateau
- `--no-camp` - skip camp spots entirely. The `.nav` is still valid and still v21, it just carries
  no camp section. Implied by `--no-vantage`, which is what supplies the candidates
- `--dump-vis-probe "x y z x y z"` - what the visibility sampler sees between two world points: the BSP leaf
  each lands in, whether the PVS links them (both directions), and the hull-0 traceline result. Repeatable.
  This is the primitive for validating the PVS and the synthesized point hull against a known-good answer
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
- `--dump-teleports` - teleport discovery: every `trigger_teleport`, why it was skipped (`SF_TRIGGER_NOCLIENTS`,
  a `master`, no brush model, an unresolvable `target`, a destination off the mesh), where its destination
  lands with the drop and fall damage, and every area that can walk into it
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
- `--dump-drift` - every `func_conveyor` and horizontal `trigger_push` that can shove a player:
  classname, entity index, brush model, spawnflags, movedir, speed, hull-expanded bounds, and either
  "taken" or the reason it was skipped. The leaf tag count is printed unconditionally when any field
  survives (see `Drift Fields`)
- `--no-blockers` - skip ExtractBlockingVolumes, so no func_wall/func_wall_toggle/func_conveyor/
  func_healthcharger/func_recharge brush is carved out of the leaves (debug)
- `--no-passables` - skip ExtractPassableVolumes, so no func_door/func_door_rotating/func_breakable
  is split out as its own tagged leaf (debug). Doors already claimed as elevators are unaffected:
  they never reach the passable path
- `--no-floor-split` - keep leaves whose floor is not solid (debug)
- `--no-lethal` - keep floors and routes that only a fall through a lethal `trigger_hurt` can
  reach (debug: isolate what the lethal prune removes; see `floor_support.cpp`)
- `--no-drift` - ignore conveyor and push-field drift entirely, so every connection costs the same
  in both directions again (debug: isolate what the direction-aware costs change)
- `--no-sky-prune` - keep floors whose underside is sky (debug: isolate what the sky prune removes,
  e.g. the skybox shell a test map is built inside; see `floor_support.cpp`)
- `--max-merge-group <n>` - most leaf groups one merge may fuse (default 64). `1` blocks the first
  absorption, so nothing merges at all and each BSP hull-3 leaf becomes its own area — this replaces the
  old `--no-merge`, whose output it reproduces exactly
- `--threads <n>` - worker threads for air-move sampling and vantage pair checking. Defaults to one per
  logical processor plus one; `1` is a genuinely serial path with no thread spawned. Output does not
  depend on this value - see `work_pool.cpp`
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
- `PruneLethalConnections`: drops any route whose chord crosses a lethal `trigger_hurt` (see `reachability.cpp`)
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

### Drift Fields (conveyors and horizontal push)

`func_conveyor` and `trigger_push` shove a standing player by the same engine mechanism: they set
`pev->basevelocity`, which `pm_shared` adds to velocity for the move and subtracts back out afterwards, so
world speed is own speed plus field velocity and `pev->velocity` stays own-only. Travel time therefore
depends on which way you go, which is what `ExtractDriftVolumes` exists to capture.

The two entities differ in three ways that matter here:

- A conveyor pushes only what **stands on** it; a push field acts on anything **inside** it, airborne
  included.
- A conveyor pushes iff `spawnflags & 3 == 0` (`SF_CONVEYOR_VISUAL` sets no `FL_CONVEYOR`,
  `SF_CONVEYOR_NOTSOLID` means nothing can stand on it). That is **not** the carve predicate:
  `IsBakedBlocker` carves it iff `spawnflags & 2 == 0`, so a visual-only conveyor is still solid geometry.
  `ExtractDriftVolumes` tests the two flags separately so `--dump-drift` can say which one dropped a
  field; both constants live in `baked_blockers.h` next to the carve predicate they differ from.
- Overlapping push fields **sum** (`CTriggerPush::Touch` adds to any base velocity already applied), so a
  leaf's drift is a vector sum, not a table index.

`ExtractDriftVolumes` takes conveyors and push fields whose *horizontal* component reaches
`kMinDriftSpeed`, which is what leaves vertical launch pads entirely on the air-move path (`Push Fields`
above) — they are unchanged by any of this. `--dump-drift` prints every candidate with movedir, speed,
spawnflags, bounds, and either "taken" or the reason it was dropped.

`kSfPushStartOff` used to be 1. The SDK has `SF_TRIG_PUSH_ONCE = 1` and
`SF_TRIGGER_PUSH_START_OFF = 2`, so navgen was skipping one-shot pushes and treating start-disabled ones as
permanently on — snark_pit, rocket_frenzy and doublecross all have them, and the runtime disagreed because
`Bot_PushFields` skips `SOLID_NOT` entities. Both flags are skipped now.

**Tagging.** `TagDriftLeaves` runs after every leaf-splitting pass and before `MergeAreas`, so nothing has to
propagate the tag through a split. It probes the `LowestFloorFace` centroid: dropped 2 units for a conveyor
(you must be standing on it), lifted 1 unit for a push field (you must be inside it), classified against the
entity's own hull-3 clip tree — the probe `PruneLethalConnections` uses. Every claim is summed and the z
component dropped.

Conveyors need no leaf split: the blocker carve has already cut every leaf along their planes. Push fields
are `SOLID_TRIGGER` and never carve, so a leaf can straddle one; they get an explicit
`SplitByPassableVolume` pass so an area is uniformly inside or outside a field. `MergeAreas` then refuses any
seam whose two leaves disagree on drift by more than `kDriftMergeEpsilon`, exactly as it refuses one that
crosses `passableIndex`.

**The cost model.** With `ms` the mover's max ground speed, `v` the area's drift and `d` the unit direction
of travel, the bot steers to cancel the perpendicular part of `v`, spending its own speed budget on it:

```
along = Dot(v, d)
s     = along + sqrt(max(ms^2 - (|v|^2 - along^2), 0))
```

Straight downstream that is `ms + |v|`. Perpendicular to a gentle 100 u/s belt it is 304 — a 5% penalty.
Perpendicular to a 500 u/s current it is 0, which is correct: you cannot hold a line across a strong field.

Cost divides by `max(s, kMinDriftSpeed)`, so a sideways crossing of a violent field is roughly 10x expensive
but still available. **Blocking uses a looser test on purpose** — a direction is impassable only when
`ms + along < kMinDriftSpeed`, i.e. when running flat out straight down `d` still loses ground. Using `s` for
the block test would wall off the side exits of every fast field and disconnect the graph; using `ms + along`
blocks exactly the genuinely-upstream case.

**Nothing about this is baked at a fixed speed.** `Bot_MaxSpeed` reads `pEdict->v.maxspeed` before
`sv_maxspeed`, so `ms` is per-server *and* per-bot. Baking the decision fails in exactly the case that
matters: a 280 u/s belt bakes at `320 - 280 = 40` ("expensive but walkable") and is impossible at
`sv_maxspeed 270`, where the bot would commit to the edge and stall. So `ConnectionDriftTerm` stores a
speed-independent `crossbot_nav::DriftTerm` and every consumer calls `NavDrift::EvalDrift` with its own
`maxSpeed`. See `doc/nav.md`.

The term splits the crossing at the connection's own endpoints — leg A is `centroid(areaA) -> pointA`, leg B
is `pointB -> centroid(areaB)` — and keeps, over the legs whose area actually drifts, the total `length`,
the length-weighted mean `along` and `perp`, and the `alongMin`/`alongMax` extremes. The mean prices the
crossing; the extremes decide passability, because a connection whose two legs drift in opposite directions
(subtransit's `*22` and `*23` meet head to head) must not average its way out of an impossible leg.

`airmove_prune.cpp`'s `WalkDijkstra` does **not** see drift: it runs before the terms are computed, and
it already uses its own local centroid measure rather than `travelSeconds` for the same reason. On a map
with a strong field it can therefore prune an air move in favour of a walk route that goes upstream. Only
subtransit and undertow are exposed to this today.

`PruneUnreachableAreas` evaluates the term at navgen's own `--max-speed`, which is what
`header.designMaxSpeed` records. Running a server *slower* than the bake leaves areas in the mesh the runtime
will not route to, which is the safe direction; running it *faster* over-prunes, and `--max-speed` is the
answer. `--no-drift` disables the whole feature.

`BotManager::CheckDriftSpeeds` prints the warning when the two disagree, and it compares against the bot's
own cap from `Bot_MaxSpeed`, never `sv_maxspeed` — under CrossPlay there is no server-wide ground speed
to compare against, and the cvar read the engine default 320 on a server whose bots ran at 270, which is
exactly the case the warning exists to catch (`doc/bot.md`). One line per distinct speed, capped at
`kMaxDriftSpeeds`, because two bots in different hosted games legitimately differ. The check re-arms every
frame instead of firing once: a bot's cap is only known after its first `PM_Move`, so a one-shot check on
the first frame a bot exists would read the cold fallback. It costs a bounded scan of live bots against a
tiny seen-speed set, and only on a map that has drift areas at all.

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
Runs **once per gravity multiplier** (see "Gravity" below). Everything described here happens inside one
such pass, and every connection it emits is stamped with that pass's multiplier.

For each connection:
1. **Collect launch edges** - perimeter of floor faces (outward normal from face center), plus the
   footprint boundary of any `trigger_push` field over the area (see "Push Fields")
2. **Seed drops** - from each edge, fan yaws ±45° around outward, 1.7x maxSpeed run-up
3. **Seed jumps** - same edges, add jump impulse, fan yaws ±90°
4. **Seed reverse jumps (climbs)** - from landing spread of drops, edge along landings, outward toward ledge
5. **Simulate each sample** - PmSim until land/wedge/timeout; check landing in dest area
6. **Bin by hull and jump** - four bins: stand/duck hull × drop/jump
7. **Select best per bin** - prefer: jump > drop; shorter flight time; less fall damage
8. **Fit gates** - `FitGate` on launch/landing clouds (robust Tukey-weighted PCA segment, local yaw/speed regression at ends)
9. **Store on connection** - gates, exitSpeed/Yaw, jumpBackDist, fallDamage, flightTime, launchPos/landingPos

### Threading
The sampler is split into an enumerate pass and a sample pass, both binned by `work_pool.cpp`, with the
reverse-jump seeds forming a barrier between two such rounds (they are derived from the drop bins, so
they cannot be enumerated until every drop has landed).

- **Enumerate** — one work item per launch edge. Areas are Morton-sorted by centroid and binned with the
  perimeter vertex count as the cost, which is known before any trace runs. Each worker runs
  `FilterLaunchEdges` for its areas; results are written into per-request slots and flattened in request
  order, so the item list and its ids do not depend on the thread count.
- **Sample** — items Morton-sorted by edge midpoint, cost `edgeSamples * variants`, tile size 32. Nearby
  edges land in the same bin so their trajectories walk the same BSP subtrees, and summed cost is near
  equal across bins. On crossfire this holds 5.1s against 3.3s, a 1.55x spread.

Three pieces of shared mutable state had to go:

1. **The elevator world was mutated mid-sweep.** Each area with a non-default elevator stop used to
   rewrite the blocker vectors that the three `PM_MoveSimulator`s hold *by reference*. Now every distinct
   world — the default plus one per placed stop — is composed once up front into its own immutable
   `WorldConfig` with its own simulator trio, and a work item just carries a world index. This is a win
   serially too: the old path reallocated two blocker vectors twice per elevator area.
2. **One shared `std::mt19937`.** Each launch edge now seeds its own generator from
   `splitmix64(seed, phase, areaId, modeIndex, edgeIndex)` — deliberately *not* mixed with the thread or
   bin index, so an edge draws the same stream whichever worker runs it. The gravity multiplier is
   excluded from the mix, preserving the old behaviour where each gravity pass re-runs the same stream.
3. **The four bins accumulated across areas.** Each worker fills private bins that are merged afterwards.

Per-edge seeding resamples, so air-move output shifts relative to the single-stream version — see
"Sampling noise" below. Everything else about the result is fixed: each `AirmoveSample` carries the
`(workItemId, sampleIndex, variant)` it came from, the merge sorts every pair's cloud back into that
order, and `best` is re-folded over the sorted cloud rather than merged from per-worker bests. `FitGate`
therefore sees the same sequence, in the same floating-point summation order, for any `--threads`.

On crossfire the simulation goes from 29.5s to 5.4s. Launch-edge enumeration is under 0.1s either way.

### Sampling noise
Connection counts move by a percent or two when the random stream changes, because a pair whose sample
cloud is marginal can fall either side of `FitGate`'s minimum-sigma test. This is inherent to the Monte
Carlo sampling, not to any particular stream: on crossfire the per-edge-seeded build finds 1391 air-move
connections against the single-stream build's 1367 (+1.8%), and across the 50 test maps the deltas are
between -2 and +2 with **reachability identical on every map**. Raising `--airmove-samples` collapses the
gap — at 16 samples/unit the two agree on `test29_ladder3` and `test23_precise`, and the single-stream
build swings by the same ±1 between sample rates on its own.

If you are comparing two navgen builds, compare reachable-area counts, not air-move connection counts.

### FitGate (robust segment fitting)
- Iteratively reweighted (Tukey biweight, 4 iterations)
- PCA on weighted points → segment direction
- Local regression (tricube kernel, bandwidth = 0.5×segment) at segment ends for yaw/speed
- Gate endpoints = mean ± 1σ (min 1.0), clamped to sample range
- Returns invalid if cloud too thin/tight

### Gravity
`trigger_gravity` copies its own `pev->gravity` onto whoever touches it, and that sticks until the player
respawns. So a bot is only ever running at one of a small, statically known set of multipliers:
`CollectGravityMultipliers` returns 1.0 plus one entry per distinct `trigger_gravity` value (capped at four,
with a warning past that).

`ProbeAirMoves` runs once per multiplier with `physics.gravity` scaled accordingly, and stamps
`MergedConnection::gravity` on everything it emits. A map with no `trigger_gravity` gets exactly one pass at
1.0 and costs nothing extra. `xen_dm` has ten triggers, all `0.4`, so it gets two passes and roughly twice
the air moves — that is intended: the runtime filter is strict, and each gravity regime needs its own set.

Two things inside a pass move with gravity rather than staying at their 800-tuned defaults:
- `existingPairs` is built from **non**-air-move connections only. Without that, pass two would see pass
  one's air moves and suppress itself entirely.
- the reverse-seed gate is `params.maxJumpHeight - PlainJumpHeight(kGravity) + PlainJumpHeight(gravity)`,
  which is the default 64 at gravity 800 and 109 at half gravity. The 64 is step height plus a plain jump;
  leaving it fixed means low gravity never seeds a climb back up a ledge it can now clearly reach.

`PruneRedundantLadders` counts only default-gravity air moves as an alternative route, so a ladder is not
pruned because of a jump that exists solely under a `trigger_gravity`.

Ladder mount reach (`ladder_connections.cpp`) still uses a single gravity; ladders are not duplicated per
multiplier.

The `--dump-path` debug route is planned at the **lowest** multiplier the map offers, not at 1.0. A bot
holds exactly one multiplier at a time, and the lowest one unlocks the most air moves, so previewing there
keeps a map whose route only exists under a `trigger_gravity` from reading as unroutable offline. navgen
prints `debug path planned at gravity xN` when it is not 1.0.

### Push Fields
`trigger_push` hands the player a base velocity every frame it is touched. `ExtractPushVolumes` reads the
brush model per hull exactly like `ExtractLadderVolumes`, with `push = speed * movedir`; `START_OFF` fields
spawn `SOLID_NOT` and are skipped. They are `SOLID_TRIGGER`, so they never enter `blockers`,
`standBlockers` or `supportBlockers`, and never carve or split a leaf.

`PM_MoveSimulator::PushAt` sums the push of every field whose hull-expanded bounds contain the origin and
whose clipnodes classify it solid — the same point test `OnLadder` uses, and the same one `SV_TouchLinks`
uses to decide a trigger touch. It is handed to `PmSim` as a `PmPushFn`; see `doc/nav.md` for what the
simulator does with it, and note that a vertical push only acts on an **airborne** player.

**Launch seeding.** `FilterLaunchEdges` keeps only floor-perimeter edges with void ahead, so a push field
standing on flat floor against a wall produces no launch points at all and applying push during flight
would never fire. `PushLaunchEdges` clips the four vertical sides of each field's footprint against the
area's walkable floor faces (2D Cyrus-Beck, endpoints lifted onto the face plane) and emits each surviving
segment as a `LaunchEdge` whose `outward` points **into** the field. Because `runSweep` sets the launch back
along `outward` and aims the yaw fan along it, that gives launches just outside the field aimed inwards — a
bot running into the shaft and jumping.

These go through the existing `seed` parameter of `sweepArea`, which already forces jumps only and requires
the launch to be inside the source area. They also set `LaunchEdge::noJumpBack`: the random 0-32 unit
jump-back that helps an ordinary ledge jump is actively wrong here, because it smears the launch cloud
across a band where only the end nearest the field works, and `FitGate` then produces a gate whose
interpolated speed misses. Pinning the launch to the boundary is what makes the fitted gate usable.

### Reverse-Jump Seeds
A drop under the (gravity-scaled) `maxJumpHeight` seeds a climb back the other way: `SeedEdgeFromLandings`
turns the drop's landing points into a synthetic launch edge in the destination area, swept like any other
edge.

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

### A sample that finds nothing is retried a sixteenth of a unit to either side
Every sample sits on the face centroid's lateral line, and that line is synthesised from brush geometry, so
it can land *exactly* on a hull-expanded clip plane. It does whenever a ladder brush is exactly hull-wide
(32 units) and flush against a wall: the brush centre and the wall's hull-1/hull-3 face are then the same
number. On-plane is the **front** side both here and in the engine (`ClassifyPoint` and `HullCheck` take
`>= 0` to `children[0]`, exactly as `SV_HullPointContents` and `SV_RecursiveHullCheck` do), and which side
is front depends only on which way the axial plane is stored, so the same geometry fails or works by
orientation alone.

test52_avoid has both. Ladder `*2` (y 0..32) sits flush against the south wall, and its centre `y = 16` is
the wall's expanded face with the open side in front — it dismounts. Ladder `*1` (y 80..112) sits flush
against the `y = 112` wall, its centre `y = 96` is that wall's expanded face with the *wall* in front, and
the top sample also sits on the pocket slab's `x = 656` face, whose front is open. The point passes
`InSolid` only because of the `x` plane; the forward dismount moves west, leaves it, is inside the wall on
the first trace, and falls straight back down the shaft — `[loopback]`, `no connections emitted`.

That is not a disagreement with the engine. `PM_FlyMove` would block a player at exactly `y = 96` too, but
no player is ever there: every engine trace stops `DIST_EPSILON` short of the plane it hits, so a real
climber on that ladder is at `y <= 95.96875`. So when a sample produces no dismount, the whole sample —
forward, sides and jump-off — is rerun at `±kLadderLateralNudge` (1/16, twice `DIST_EPSILON`) along
`perp`, first `+` then `-`, stopping at the first that produces one. The nudge is far below anything
geometric, so it cannot invent a dismount; it only moves the start off a plane the real player cannot be
on. `--dump-ladders` tags those lines `[nudged +perp]` / `[nudged -perp]`: on test52_avoid `*1` loops back
centred and at `+perp`, and at `-perp` reaches area 6 at (641, 95.9, 18). A sample that already works is
never nudged, which is why every other test map's `.nav` is byte-identical with and without this.

### Two ladders are two connections
`emitted` deduplicates by area pair **per ladder entity**, not across the map. Within one ladder it still
collapses what its faces and pieces produce for the same pair to the lowest-fall-damage one. Across ladders
it used to do the same, so two separate ladders joining the same pair of areas became one connection —
and once `*1` dismounted on test52_avoid, it replaced `*2` outright, because it is processed first and
ties on fall damage. A single edge cannot be routed around: the bot's runtime `LadderBlocked` probe
penalises the connection it is on (see `doc/bot.md`), and with one connection that penalty only makes the
same ladder dearer. Parallel edges between one pair are fine for the planner, which walks connection lists
and never looks a connection up by its endpoints.

Of the twelve stock DM maps with ladders, only snark_pit changes: its `*9` and `*10` are one wide ladder
built from two adjacent brushes, each now keeps its own `148 <-> 147`, and `PruneRedundantLadders` leaves
2 of 5 where it left 1. That is the same physical climb twice, which costs nothing and lets a bot blocked on
one half take the other.

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

### DropSkyFloorLeaves
Sky is solid in the clip hulls, so the inside of a map's skybox shell extracts as ordinary walkable
floor. Nothing downstream objects: the shell is a genuine standing surface, a bot that walks off a
platform lands on it, and because that fall is a real air move the reachability prune keeps the whole
thing. It is still not a place a bot should ever be — it is the outside of the level.

The test is per leaf, on its lowest floor face's centroid, probed 2 units under the real surface
(`kDuckClipHullMinZ - 2`, since floor faces sit in the duck hull) and classified by the hull-0 world
leaf there. Contents `CONTENTS_SKY` means the leaf is standing on the shell, and it goes. There is no
trace and no volume list — the BSP already answers the question.

It runs at the leaf stage, immediately before the lethal prune, for the reason given below: `MergeAreas`
would otherwise fuse shell floor with the real ledges standing over it.

**What it is worth.** Only the two synthetic skybox maps have any: test39_skybox (2 leaves) and
test41_skydrop (3). In both, the shell was ~98.4% of the map's walkable footprint — 8.05M of 8.19M sq
units on test39 — which the vantage sampler pays for quadratically: 36k lattice points and 664M pair
checks, 71s of a 71s run. After the prune it is 660 points, 217k pairs, and navgen finishes in 0.9s.
All 19 HLDM maps report 0% sky floor, so the prune is a no-op on real maps; `--no-sky-prune` restores
the old behaviour for comparison.

### DropLethalFloorLeaves
`trigger_hurt` with `dmg >= 100` is not an obstacle, it is a floor deleter. One tick of it takes a full
health bar, so any standing spot you can only arrive at by falling through one is not a place a bot can
be — and, unlike a wall, nothing in the leaf pipeline notices it, because a trigger brush is not solid in
any hull.

The test is per leaf, on its lowest floor face's centroid. The floor is dead when it is inside a lethal
volume's xy footprint, below that volume's top, and a duck-hull trace straight up from the floor into the
volume's underside comes back clear. The upward trace is what keeps the rule narrow: a room under a lava
pit has a ceiling, the trace stops on it, and the room survives. An open column under a kill plane has
nothing in the way, and the floor at the bottom of it goes.

Volumes are extracted at `kHullDuck`, so a floor already inside the trigger reports `entry.z <= stand.z`
and is dropped without a trace. `SF_TRIGGER_HURT_START_OFF` and `SF_TRIGGER_HURT_NO_CLIENTS` triggers are
skipped; negative `dmg` (healing) never qualifies.

Healing volumes stay entirely out of navgen. They are not solid in any hull, so they carve nothing
and split nothing, and the runtime finds them by edict scan with no baked data at all — see
`ITEM_HEALVOLUME` in `doc/bot.md`. `pool_party` is the map to check this on: its `dmg 200` volume is
the one lethal trigger navgen reports, and its `dmg -40` pool never appears.

**Why the leaf stage and not reachability.** The obvious place for this is a connection cut plus the
existing unreachable-area prune, and that is what was tried first: it removed 550 connections on xen_dm
and pruned nothing. `MergeAreas` had already fused the void-floor leaf with real ledges above it — area
309 spanned z −2286 to −1758 in one prism — so by the time areas exist there is no longer a void area to
prune. The kill volume has to be applied while leaves are still separate.

**What it is worth.** On xen_dm the map's whole footprint sits over a clip brush at z ≈ −2304, under a
map-wide `dmg 10000` slab at −2160. That floor was 32.9M of the mesh's 44.4M sq units of walkable
footprint — 74% of the map — and it drove the vantage sampler quadratically: 221k lattice candidates
instead of 53k, and the pair loop goes with the square of that. Dropping 40 leaves takes the footprint to
12.2M and the whole navgen run to 3m34s; before it, the run was abandoned twice, once at 575s without
having finished. `pool_party` is the other big one: 616 of
its 2800 floor faces (29.5% of its footprint) stood *inside* a `dmg 200` trigger, and are now gone.

Across the 50 test maps and the 19 HLDM maps, only four have a qualifying volume: xen_dm (40 leaves),
pool_party (88), snark_pit (1) and rocket_frenzy (1). `--no-lethal` restores the old behaviour for
comparison.

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

## vantage_sampler.cpp

### What a vantage is
A sampled eye position carrying a visibility signature: where a bot standing there should watch, how exposed the spot is, and how much of that exposure a single facing covers. `BuildVantages` runs after `PruneUnreachableAreas` (which renumbers areas), and since the field became a `.nav` section it runs on **every** navgen invocation unless `--no-vantage` is passed.

**That is not free, and it is now the dominant cost of a run.** On crossfire the whole invocation went from 8.8s to 47.8s at `--threads 13`; 38.7s of that is the vantage field, and 36.4s of *that* is the pair loop. The full 50-map test sweep in `generate_all_svgs.ps1` runs in 164s. `--no-vantage` exists for iterating on unrelated navgen stages, and the `.nav` it writes is valid — bots simply fall back to no idle look and unweighted senses (`doc/bot.md`, *Where the bot looks when it has no target*).

| Field | Meaning |
| --- | --- |
| `watch[]` | up to 4 `WatchDir{yaw, pitch, spread, amount}`, `amount`-descending. `amount` is the fraction of everything that can see this spot arriving from that cone; `spread` is the weighted RMS angular deviation inside it, in degrees |
| `exposure` | fraction of all sampled vantages in the map with line of sight to this one |

### The internal datum is a direction histogram
Sampling accumulates a yaw x pitch weight histogram per vantage (64 x 8 by default), and everything above is derived from it. The histogram is working memory, never output. It is what makes the reduction well-defined: a fixed-length vector has a distance, a variable-length cluster list does not.

Pitch is binned over +/-45 degrees with clamping rather than the full +/-90. Sightlines steeper than that are rare and their exact pitch does not matter, and spending the bins near the horizon is what keeps the narrow cones the camp sampler seeds from spanning only a single bin.

### Grid sampling
The lattice is **world-aligned**, not area-aligned, so neighbouring areas agree on cell centres and dedup is exact. For each lattice point, every walkable floor face of the area is tested with `Distance2DToWinding` and contributes its own `FloorHeightAt` — per face, not per area, because a prism area (stairs, a ramp) carries several floor faces at different heights and each is a real standing position.

**Eye height.** Area z is in duck-hull-origin space, which is what `kStandProbeLift = kDuckClipHullMinZ - kHumanHullMinZ = 18` exists to undo. With HLSDK's `VEC_VIEW.z = 28` and `VEC_DUCK_VIEW.z = 12` that gives `kDuckEyeLift = 12` (area z + 12 = real floor + 30) and `kStandEyeLift = 46` (real floor + 64). A `NAV_AREA_CROUCH` area uses the duck lift; everything else uses the mean, +29, because the bot may be in either stance.

Two candidates in the same lattice column within 32 units of z are the same spot and merge — a player cannot occupy two floors that close together. Any eye point that classifies `CONTENTS_SOLID` in hull 0 is dropped.

### Pair traces
Vantages are bucketed by hull-0 leaf and the pair loop walks them in leaf order, so each worker's `VisRowReader` single-row PVS cache hits on essentially every query. Leaf index is depth-first BSP order, so leaf order is already spatially coherent — that is why the pair loop keeps it instead of re-sorting by Morton code when it bins work across threads, and why threading it changed no output at all. A pair that fails the PVS is skipped outright; on crossfire that removes two thirds of the 553M pairs before any tracing happens.

Each surviving pair costs one hull-0 traceline plus, for any blocker whose AABB the segment crosses, one trace against that blocker's own hull-0 tree. The blocker set is `ExtractVisBlockers` — the *permanently* solid brush entities only, at hull 0 and without the movement hull's bounds expansion. Doors and breakables are deliberately excluded: they open, and a mesh that assumes they are shut is wrong most of the time.

Visibility is symmetric, so one trace updates both endpoints' histograms — which is exactly why the histogram has to be atomic once the loop is threaded (see "Where the time goes"). Every vantage represents the same footprint cell by construction, so a uniform weight of 1 per pair is already area-proportional.

### Watch directions: peak extraction, not DBSCAN
DBSCAN on a sphere with a fixed `eps` degenerates in open areas — everything merges into one cluster. Instead, on a working copy of the histogram: seed at the max bin, mean-shift three passes over the bins inside `--vantage-cone` of the current direction, record the cone's weight/spread, zero those bins, repeat. Stop at 4 directions or when under 5% of the weight is left. This is the same refinement `LookDirection` already does in `bot_sense.cpp`.

Camping used to be a `focus[]` field measured here on the *original* histogram. It is now its own dataset; see `camp_sampler.cpp`.

### Reduction: the error metric must be scale-free
Greedy decimation, lowest interpolation error first, until `kept <= footprintArea / spacing^2`. For each candidate, the `--vantage-neighbors` nearest survivors are found, IDW-weighted by `1/(d^2+1)`, and the candidate is scored on how well that interpolation reproduces it.

**The signature is split into shape and magnitude, and this is load-bearing.** The first attempt scored raw L1 distance between un-normalized histograms, which is dominated by vector magnitude: an enclosed spot has a small vector, so its absolute error is small however distinctive it is, and it gets deleted first. On crossfire that kept a set averaging exposure 0.306 against a population average of 0.158 — it preserved the open outdoor bowl and erased the bunker interiors, exactly backwards for a tactical field. Normalizing each histogram to unit L1 and scoring exposure separately (weighted `kExposureErrorWeight = 2` so the two terms are comparable) brings the kept set to 0.133, slightly *below* the population mean, and restores the interior floors.

The only stop guard is never dropping below `neighbors + 1` vantages.

### Where the time goes
Measured on crossfire at defaults (33266 vantages, 553M pairs), on a 6-core / 12-thread box at
`--threads 13`. The serial column is the same binary at `--threads 1`.

| Phase | Serial | 13 threads |
| --- | --- | --- |
| Candidate grid | 0.0s | 0.0s |
| Solid rejection + leaf assignment | 0.0s | 0.0s |
| **Pair loop** | **276.2s** | **40.1s** |
| Derive (histogram to watch dirs and focus) | 0.6s | 0.7s |
| Reduction (33266 down to 113) | 1.9s | 1.9s |
| Whole navgen run, vantage stage included | 313.8s | 52.6s |

The pair loop is the entire cost and it is where all the parallelism went: **6.9x** on 6 physical cores.
Inside it, hull-0 tracelines are 213.9s of the 276.2s serial figure (184.8M traces at 1.16 us); the rest
is the PVS test and histogram binning over all 553M pairs.

`traceSeconds` is *summed across workers*, not wall time, so at 13 threads it reads 380.5s against a
40.1s wall — that is the CPU time, and the per-trace figure inflates from 1.16 us to 2.06 us because the
workers contend for memory bandwidth and share L3. It also carries its own measurement cost: two
`steady_clock` reads per traced pair, worth about 1.8s of the 40.1s (4.5%). It is kept because the
trace-versus-PVS split is what tells you the tracelines are the only thing worth optimising.

Those figures pre-date two later changes, both measured as whole-run wall time at default threads on the
same box, alternating the old and new binaries in one sitting because run-to-run variance here is large
enough to swallow anything under ~5%:

| Change | crossfire | boot_camp |
| --- | --- | --- |
| `/Ob2` on the navgen target | 59.0s to 46.0s | - |
| `SegmentBlocked` for visibility queries | 42.0s to 37.7s (-10%) | 133.9s to 113.7s (-15%) |
| Shared-origin chain cache | 40.8s to 34.5s (-15%) | 110.4s to 94.6s (-14%) |

The `SegmentBlocked` rows are means of four alternating pairs on crossfire and two on boot_camp; on both
maps every new run beat every old run, so the ranges do not overlap. `.nav` output is byte-identical
across the change on all 51 maps under `maps\` plus crossfire and boot_camp.

**Why the histogram is atomic.** A pair `(i, j)` writes *both* `hist[i]` and `hist[j]` — visibility is
symmetric and one trace updates both endpoints — so no partition of the outer loop gives a worker
exclusive ownership of what it writes. The histogram is therefore one flat
`vector<atomic<uint32_t>>` incremented with relaxed `fetch_add`. Weights are always exactly 1, so integer
counters lose nothing and, unlike float accumulation, give a bit-identical result whatever order the
threads interleave in. (C++17 does not guarantee value-initialising an atomic zeroes it — that is C++20's
P0883 — so the array is explicitly stored to 0 first.)

**Why the cost model counts PVS hits exactly.** Rows are binned by `BuildWorkBins`, which needs a cost per
row. `count - 1 - a` alone bins badly: a tunnel vantage's PVS admits a handful of partners where a
courtyard vantage's admits thousands. Because `order` is leaf-sorted, every leaf's vantages occupy a
contiguous slot range, so the exact number of PVS-admitted partners *after* slot `a` is a sum over the
visible leaves with a higher index — O(leaves²) to compute for the whole map, negligible. Cost is then
`(count - 1 - a) * 1 + tracedAfter * 18`, the 18 being the measured per-trace to per-PVS-test ratio. The
heuristic version (`pvsFrac` of the leaf's whole row) left the bins at a 2.7x spread; the exact count
brings it to 1.29x (40.1s slowest, 31.1s fastest), which the stats line reports so it stays checkable.

The `VisIndex` row cache had to move out into a per-worker `VisRowReader` — it was `mutable` state on a
shared object, i.e. a data race. Decompressing every row up front into one flat bitset was the
alternative and was rejected: that is `leafCount² / 8` bytes, fine at 8k leaves but 112 MB at 30k.

The dominant term is still N² in vantage count, so `--vantage-grid` remains the real lever: 20 instead of
15 gives about 18.7k vantages and roughly a third of the pairs.

The other lever is upstream, and it is the bigger one: vantage count is walkable footprint over
`gridSize`², so pairs go as **footprint squared**. A map with junk floor in it pays for that junk twice
over. xen_dm at 44.4M sq units of footprint would have needed 221k candidates and 2.5e10 pairs, about an
hour; with the void floor removed it is 12.2M sq units, 53k candidates, 1.4e9 pairs and 146s. If a map's
vantage stage looks absurd, read the `budget ... over N sq units` figure in the stats line before
touching `--vantage-grid` — `--dump-sky` will say whether that N is real floor.

Note `--vantage-verify-neighbors` is expensive and is timed separately so it never inflates the reduce
figure: at spacing 512 it added 290s of brute-force scanning to a 1.9s reduction.

### Nearest-neighbour queries
A uniform spatial hash over vantage positions, cell size `max(32, grid*2)`. The ring search **expands until the k-th best distance beats the guaranteed radius** `r * cell` — a point outside the searched Chebyshev-r box of cells is at least that far away. Stopping at a fixed one-ring silently returns wrong neighbours near cell boundaries and as the kept set thins out, which would quietly corrupt every reduction decision, so `--vantage-verify-neighbors` cross-checks every query against a brute-force scan and reports mismatches.

## camp_sampler.cpp

### Camp spots are a separate dataset, not a field
The vantage field answers "how seen is it around here" and "which way do sightlines run around here".
Both are properties of a neighbourhood, so the runtime blends them between the four nearest samples and
gets a meaningful answer anywhere. Camping is not like that. It is discrete -- stand *exactly* here, face
*exactly* there -- and the midpoint between two camping spots is usually not a camping spot. `focus[4]`
used to smear that discrete property across the continuous field and it never worked; nothing read it.
It is gone, and camp spots replace it as their own reduced list with their own record.

A camp spot is a floor position, one facing, and three numbers per stance:

| Field | Meaning |
| --- | --- |
| `pos` | the nav floor point, which is a ducking player's origin |
| `yaw`, `pitch` | one facing, shared by both stances |
| `standScore`, `standSpread`, `standDist` | from the standing eye, `pos.z + kStandEyeLift` (+46) |
| `crouchScore`, `crouchSpread`, `crouchDist` | from the crouching eye, `pos.z + kDuckEyeLift` (+12) |
| `score` | `max(standScore, crouchScore)`, what the selection ranks on |
| `flags` | `CAMP_NO_STAND` when standing there is impossible |

`spread` is the RMS angle the targets inside the FOV sit off the axis, in degrees -- how wide an arc a
camper there has to sweep. `dist` is their mean range. Both replace what used to be four nested arc
ratios per stance (FOVs 90 / 45 / 22.5 / 11.25): the nesting was only ever a crude way of asking "how
concentrated is this", which one spread answers directly, and no arc ever carried a distance at all.
`BotCamp` reads all three -- spread and dist decide the scope, the scores decide the stance
(`doc/bot.md`).

The nav floor plane is the hull-3 centre, so it *is* a ducking origin: crouch eye = floor + 12 (the duck
`view_ofs`), standing origin = floor + 18 and standing eye = floor + 18 + 28 = floor + 46. Both lifts
already existed in `vantage_sampler.h`; nothing new was measured to get them.

### The FOV metric is odds, not a fraction
For one 90-degree FOV -- half-angle `kCampFovHalfDeg` (45), the widest of the four nested arcs this
replaced -- count visible probe candidates inside the cone and outside it, per stance:

```
score(stance)  = inside / max(outside, 1)
spread(stance) = sqrt(sum of angle^2 over inside / inside)     degrees off the axis
dist(stance)   = sum of range over inside / inside
score          = max(stand_score, crouch_score)
```

The odds form `f / (1 - f)` is used instead of the fraction `f` because a fraction saturates exactly
where camping quality lives: 0.90, 0.95 and 0.99 are visually identical numbers, but as odds they are 9,
19 and 99. `max(outside, 1)` rather than `outside + 1` keeps a perfectly focused spot's score equal to
its visible count, so "sees a lot, all of it in front" outranks "sees one thing, technically in front".

A large score means "you can see a lot without turning your head", which is the primary camping virtue.
Tightness is no longer a bonus folded into that number: it is `spread`, reported separately, so a wide
overlook and a sniper's slot can score alike and still be told apart.

**The dynamic range this produces is enormous and that has consequences.** On crossfire the score runs
into the hundreds because `max(outside, 1)` divides by 1 whenever nothing at all is behind you. The top
spots are alcoves that see hundreds of targets with none behind. That is a real camping spot and the
ranking is right, but see *Spacing does not control spread* below for what the range costs.

**The minimum-visible gate is load-bearing.** A spot that sees a single target with nothing behind it
scores 1.0, beating nothing but competing with real rooms once the odds form is applied.
`--camp-min-visible` (default 32) removes it, applied on `exposure * (count - 1)` before a candidate ever
enters the queue.

### Selection: cheap ordering, expensive confirmation
Camp spots are drawn from the **pre-reduction** probe candidates -- the full 15-unit lattice, after the
pair loop has given each one an `exposure` and a `watch[]`, but before the decimation discards most of
them. `BuildVantages` hands that snapshot back through `candidatesOut`. No second lattice exists and
nothing new is sampled.

The whole point is that the cheap data already in memory decides *where to look*, and the expensive
two-stance visibility test runs only around the promising places:

```
gate    candidates seen by fewer than --camp-min-visible probes, plus water and elevator areas
score   preliminary = watch[0].amount / exposure
loop    pop the best remaining candidate
        hill-climb to the local maximum of the FOV score
        emit the summit; consume everything the climb consulted
        penalise survivors by min(1, d / spacing)
until   budget reached or the queue empties
```

`watch[0].amount / exposure` is main-direction strength over exposure: a high `amount` means "what sees
me, sees me from one place", a low `exposure` means "not much sees me at all". Both terms are already on
every candidate, so the ordering is free.

### Hill climbing, and why the climb cancels
The preliminary score is a proxy computed at one eye height from a bin histogram, so it finds a promising
*basin* but not the best point in it. Each pop is therefore a starting point: evaluate the candidate,
evaluate its unvisited lattice neighbours, step to the best, repeat. Neighbours are candidates within
`gridSize * 1.5` in XY -- which reaches the 21-unit diagonals -- and within 32 in z, so a climb follows a
slope but never jumps between floors stacked in the same column.

A step is only taken when the best neighbour beats where we stand by `kCampMinGain` (2%). Without that
margin a climb crawls sideways across a near-flat ridge one lattice cell at a time: on xen_dm the mean
climb was 6.5 steps and cost 112 s, and requiring 2% cut it to 3.8 steps and 59 s for the same 182 spots.

**A climb whose ascent leads into a candidate an earlier climb already consumed is cancelled outright.**
It is not finding a new spot, it is re-walking someone else's hill from lower down, and the earlier climb
already stood on that summit and emitted it. Stopping there instead would emit a second camp spot partway
up the same slope. Note the test is *ascent*, not adjacency: a consumed neighbour that scores lower than
where we stand means we are on our own summit with someone else's basin beside us, and that emits
normally.

Every score is cached, including for candidates that were only ever rejected neighbours, so a candidate
is evaluated at most once in the whole stage. *Cached* and *consumed* are different states, and the
distinction is what lets the worker pool exist: a score is cached by whoever computed it, but a candidate
is consumed only when a climb actually consulted it.

**Cancelled climbs must suppress, and this was learned the hard way.** The first implementation skipped
suppression on a cancelled climb, reasoning that the basin it merged into had already suppressed the
neighbourhood when it finished. That is false. Suppression only covers `spacing` around the points the
winning climb *walked*, roughly a dozen cells, whereas the basin of attraction that drains into that
summit is hundreds of cells wide. Every pop inside the untouched part of the basin started a climb,
ascended into the consumed summit and cancelled. On xen_dm that was **758 cancelled climbs against 182
emitted**, 83% of all evaluation work thrown away. Suppressing on cancel drops it to 16.

### Spacing does not control spread
`--camp-spacing` sets two things: the budget, `footprint / spacing^2`, and the radius of the soft penalty
`min(1, d / spacing)` applied to every survivor after a climb. The penalty is soft on purpose, so a
genuinely excellent spot 100 units from another excellent spot loses half its score but can still win
later, where a hard radius would delete it outright.

**In practice the budget is enforced and the spread is not.** On crossfire 96% of kept spots are closer
to their nearest neighbour than 256, with a median of 173 and a minimum of 45. The cause is the score
range: suppression can only ever multiply by a number in `[0, 1]`, and when scores span 100x, a spot 100
units from a claimed summit keeps 39% of a score that was already ten times its neighbours'. It stays at
the top of the queue regardless. The result is that camp spots cluster on the genuinely best hills rather
than tiling the map, and `--camp-spacing` behaves as a count knob.

That is a consequence of the specified rule, not a bug in it, and the output is still usable -- the
clusters are on real camping ground. If tiling is wanted instead, the lever is a steeper falloff
(`(d / spacing)^k`) or a hard radius, and both change what the flag means.

### Cost
The two-stance test runs once per *evaluated* candidate against `footprint / 15^2` targets at two eye
heights. Writing `E` for evaluations per emitted spot -- the summit, the path, the rejected neighbours
and everything spent on climbs that cancelled -- and comparing against the pair loop's `A^2 / 101250`:

```
camp trace-pairs / vantage pair-loop pairs = 900 * E / spacing^2
```

`E` is the number that decides whether this stage is free or expensive and it cannot be predicted from
geometry: it depends on how rugged the campiness surface is. Measured at the default 256 spacing:

| map | candidates | spots | E | cancelled | camp stage | share of run |
| --- | --- | --- | --- | --- | --- | --- |
| crossfire | 33266 | 114 | 11.5 | 0 | 8.2 s | 13% of 62 s |
| pool_party | 14998 | 51 | 8.5 | 0 | 2.7 s | 4% of 68 s |
| xen_dm | 53200 | 182 | 22.5 | 16 | 58.6 s | 23% of 254 s |

The knob is quadratic in the *inverse* of spacing, so `--camp-spacing 64` is 16x this cost, not 4x.
`--camp-climb-steps` (default 32) is the hard backstop if a plateau ever walks the map anyway. Watch the
cancellation count: a rate approaching the emitted count means the preliminary score keeps restarting
inside claimed basins, and that is evaluation spent for no spot.

### Parallelism: a persistent pool that only fills a cache
Splitting the inner target loop with `RunBins` is the wrong shape here. `RunBins` spawns and joins
`binCount - 1` threads per call, which is fine for the two calls that cover the whole air-move and
vantage stages but ruinous for one call per evaluation. So threading goes **across evaluations**, with a
main thread and a persistent `ThreadPool`.

The main thread runs the serial algorithm unchanged: it pops, climbs, decides and penalises, and asks the
cache whenever it needs a score. Workers only ever populate that cache. They never mark a candidate
consumed, never touch the priority queue, never decide anything. That is the constraint the scheme lives
under and it is not stylistic -- if being evaluated implied being consumed, how far the workers happened
to run would change which candidates remain available, and the output would depend on thread count and
timing. `.nav` files are byte-identical at `--threads 1` and `13` because of it.

Most of the parallelism is not speculative: at each climb step every neighbour's score is needed before
the main thread can choose, so those are dispatched together. Measured batch width is only 3.5 of 13
threads, because after the first step most neighbours are already cached and the batch shrinks. Feeding
idle workers neighbours-of-neighbours as pure prefetch hints would recover some of that; it is safe by
construction, since a wasted prefetch costs one cached score nobody reads. It has not been done because
the stage is 8-59 s and the complexity is not yet worth it.

Determinism has two other requirements: the priority queue breaks score ties on candidate index, and the
climb breaks neighbour ties on candidate index.

## work_pool.cpp

Shared static work-binning used by both threaded stages. Three pieces:

- `DefaultThreadCount()` — `hardware_concurrency() + 1`, falling back to 4 when detection returns 0. The
  extra thread is deliberate: it keeps a core busy while another worker stalls on a page fault or gets
  descheduled, and the bins are static so there is nothing to steal.
- `MortonKey(point, cell)` — Z-order code over the position quantised to `cell`, 21 bits per axis. Sorting
  by it puts spatially adjacent work next to each other in the list.
- `BuildWorkBins(cost, order, tileSize, binCount)` — chunks `order` into tiles of `tileSize` consecutive
  entries, sums each tile's cost, sorts tiles by cost descending, then assigns each to the currently
  lightest bin (**LPT**). Returns each bin's items back in ascending tile order.
- `RunBins(binCount, body)` — spawns `binCount - 1` threads and runs bin 0 on the caller. `binCount == 1`
  calls `body(0)` with no thread at all, so `--threads 1` is a genuinely serial reference path. Each
  worker catches into an `exception_ptr` slot that is rethrown after the join; navgen throws
  `std::runtime_error` on malformed BSPs and a throw crossing a thread boundary would otherwise
  `std::terminate`.

Tiles are the compromise between the two things static binning wants at once. Whole-range partitioning
gives perfect locality but terrible balance (the pair loop's cost falls off linearly with the row index);
round-robin over single items balances perfectly but destroys locality. A tile is small enough that LPT
has hundreds of pieces to balance with, and large enough that the traces inside one reuse the same BSP
subtrees.

**Output never depends on the thread count.** Both call sites are written so that thread scheduling
cannot reach the result: the vantage histogram accumulates into integer atomics (integer addition is
associative, unlike the float accumulation it replaced), and the air-move merge sorts samples back into a
canonical order before anything reads them. `--threads 1`, `3`, `7` and `13` produce byte-identical
`.nav` files on all 50 test maps and on crossfire.

## reachability.cpp

### FindSpawnAreas / FindTargetAreas
- Spawns: info_player_start, info_player_deathmatch, info_player_coop, info_player_teamspawn
- Targets: info_target
- Parses "origin" key, finds containing area (boundPlanes test, ε=1.0), falls back to nearest centroid

### PruneUnreachableAreas
BFS from seed areas over connection graph (respects one-way). Remaps area IDs to compact 0..N-1. Filters connections.

### PruneLethalConnections
The other half of the `trigger_hurt` rule (`DropLethalFloorLeaves` in `floor_support.cpp` is the first).
That one removes the *floors* a kill volume makes unusable; this one removes the *routes* that pass
through one between two floors that are both safe — a jump across a lava sheet, a drop past a kill plane
onto a ledge below it. The chord from `pointA` to `pointB` is sampled every 8 units and tested at the
duck origin and 36 above it, which together cover a standing bbox in area z-space. `CONN_TELEPORT` and
`CONN_ELEVATOR` are exempt: neither is traversed through the space between its endpoints.

A chord is not a trajectory — a jump arc bows above it — so this is conservative for jumps, and that is
deliberate. A route whose straight line clips a volume doing 100+ damage is one where a small navigation
error is fatal, and a bot is better off not knowing about it.

Be aware that it is unexercised in practice: across the 50 test maps and the 19 HLDM maps it cuts zero
connections, because `DropLethalFloorLeaves` upstream already removes every floor those routes lead to.
It fired 550 times on xen_dm only in the version where the leaf drop did not yet exist.

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

The recursion is written against a `HullTree` (a clipnode array plus a plane array), not against `BspFile`, so the same code serves the movement hulls out of `LUMP_CLIPNODES` and the point hull synthesized from `LUMP_NODES` — see `bsp_vis.cpp`. `WorldHull(bsp)` builds the clipnode-lump view, and the `BspFile` overloads of `ClassifyPoint`/`TraceHull` are one-line wrappers over it, so every existing caller is unchanged.

### SegmentBlocked, and why the traversal is a template

Visibility queries do not want a trace, they want a bit. `Occluded` (`vis_sampling.h`) reads exactly two
fields of the 112-byte `TraceResult` — `fraction` and `startsolid` — and on crossfire it runs 184.8M times,
so `RecursiveHullCheck` and `ClassifyPoint` between them were 53% of all navgen CPU.

`HullCheck<BoolOnly, Trace>` is therefore one function with two instantiations rather than two copies of an
epsilon-sensitive algorithm. `TraceHull` instantiates `<false, TraceResult>` and is unchanged in behaviour;
`SegmentBlocked` instantiates `<true, NullTrace>`, carries no state at all, and returns a `HullStep` of
`Continue` / `Stop` (blocked) / `Clear` instead of the old `bool`. For the full path `Continue`/`Stop` map
1:1 onto the old `true`/`false` and `Clear` never occurs.

What the boolean path actually saves is the **back-off loop**, which re-descends the whole tree from
`headnode` in a `while`. It can be skipped outright. `Occluded` is blocked iff
`fraction < 1.0f || startsolid`, and:

- A `true` return only ever originates at a leaf, so any `true` bottoms out at one.
- `fraction` is written only in the terminal block, which runs at most once per top-level trace.
- `startsolid` is set only on reaching a `CONTENTS_SOLID` leaf.
- In the terminal block `midf` is non-increasing: `frac` only decreases and `p2f >= p1f`.

Let the boolean path return `Stop` the moment it reaches a solid leaf. Then `allsolid` at the terminal block
is unreachable — it would mean a leaf was reached and none was non-solid, i.e. a solid leaf was reached,
which already returned — so `allsolid` drops out of the boolean path entirely. Reaching the terminal block
therefore implies `startsolid` is false, and the answer is exactly `finalFraction < 1.0f`. Since `midf` only
falls, `(float)midf < 1.0f` on entry means the final fraction is below 1 whatever the loop does: blocked, no
loop needed. Only when `(float)midf` rounds to exactly `1.0f` does the loop decide, and there it runs
verbatim.

Both of the loop's original exits set `fraction`/`endpos` from the current `midf`/`mid` and return, so the
`frac < 0.0` early return collapses to a `break` with no change in behaviour.

`--vis-probe` still prints a real `TraceHull` fraction, so it remains an independent cross-check that the
fast path and the full path agree.

### The shared-origin chain cache

Visibility traces come in fans: the vantage pair loop holds `vantages[i].pos` fixed for its whole inner
loop, and the camp sampler alternates two fixed eye points. On crossfire that is 206M boolean traces
issued from 33266 probes, and every one of them re-walked the same descent from the root.

In all three branches of `HullCheck` the **first** recursive call descends to the side `p1` is on:
both-front takes `children[0]` and `t1 >= 0` puts `p1` in front, both-back takes `children[1]`, and on a
split `side = (t1 < 0) ? 1 : 0` is `p1`'s side as well. Along that near chain `p1` and `p1f` are passed
through untouched -- only `p2`/`p2f` shrink to `mid`/`midf`. Three consequences:

- The near-chain node sequence is exactly the path `ClassifyPoint(hull, headnode, p1)` walks, so it is a
  function of the origin alone.
- Every trace walks it in full, because the recursion only bottoms out at `nodeIndex < 0`. The hit rate
  is 100%, not "usually".
- `t1` at each chain node, and the leaf `p1` lands in, are pure functions of the origin.

It is also most of the traversal. Counted directly: the chain is 15.11 of the 25.47 clipnodes a boolean
trace visits on crossfire (59%), and 14.47 of 25.03 on boot_camp (58%).

`HullOriginChain` caches it. Each `HullChainStep` holds the plane's `normal`/`dist` copied inline, the
memoized `t1`, and the far child -- 32 bytes, so walking the chain is one sequential read per node
instead of a random `clipnodes[n]` followed by a dependent random `planes[n.planenum]`, and the `t1`
half of the plane arithmetic disappears. `SegmentBlockedCached` replaces only the near descent, walking
the cached steps iteratively and pushing each split onto `chain.splits`; far-side subtrees still go
through the recursive `HullCheck<true>`, since they start mid-tree from a varying `mid`.

Unwinding the split stack top-down mirrors the recursion's returns exactly. Every branch after the near
call in `HullCheck` returns, so a non-`Continue` result propagates straight out of the loop, while a far
subtree that returns `Continue` lets the next shallower split take its turn.

The cache validates itself: the `SegmentBlocked` overload rebuilds whenever the clipnode array, headnode
or origin differ from what it holds, so no caller has to remember to clear it and a stale chain cannot
survive a probe change. `vantage_sampler` keeps one per worker, `camp_sampler` two per `SlotState`.
Blocker traces stay on the plain `SegmentBlocked` -- they are 0.12% of world traces on crossfire and
3.7% on boot_camp, not worth a keyed slot.

Arithmetic is preserved literally: `p1f` stays an explicit `0.0` term in `p1f + (p2f - p1f) * frac`
rather than being folded away, and the `kDistEpsilon` expressions are unchanged. `.nav` output is
byte-identical on all 51 maps under `maps\` plus crossfire and boot_camp, and on crossfire at
`--threads 1, 3, 7, 13`. On crossfire the vantage stage goes 25.5s to 22.5s with summed trace CPU
180.9s to 146.3s (0.98 us to 0.79 us per trace), and the camp climb 3.9s to 3.0s.

**Why there is a start leaf but no end leaf.** Knowing that the next trace begins in leaf L does not let
you skip anything: you still need `t1 = PlaneDiff(plane, newOrigin)` at every node above L to decide the
splits, which is precisely the work being saved. Harvesting those values on the way out of the previous
trace does not work either -- what the traversal computes along the path to the end leaf is
`PlaneDiff(plane, mid)` for assorted `mid`s, not for the endpoint, and computing the right ones costs the
same k plane evaluations. The airmove path is left out for the same reason: its one genuine shared origin
is `CatagorizePosition`'s ground probe and the *next* frame's `FlyMove` trace, both from an unmodified
`m_pos`, which is a 50% hit rate on a stage worth 3.8s of a 46s run.

## bsp_vis.cpp

### Hull 0 is synthesized, not loaded
`LUMP_CLIPNODES` only describes hulls 1-3, the three player-sized boxes. Line of sight is a *ray*, which is hull 0 — and hull 0 lives in `LUMP_NODES`, a different struct.

It does not need different traversal code, though. `dnode_t` and `dclipnode_t` have byte-identical `planenum` + `short children[2]` prefixes, and the engine exploits exactly this in `Mod_MakeHull0`: walk the node array, copy `planenum` and any non-negative child straight across, and replace each negative child (a leaf reference, encoded `-1 - leafIndex`) with that leaf's `contents`. The result is a clipnode array that `RecursiveHullCheck` traverses unmodified.

Only `CONTENTS_SOLID` blocks, which is the same rule the engine's `TRACE_LINE` uses — water and sky stay transparent to sight.

### PVS
`VisIndex` decompresses one PVS row at a time and caches it, because the pair loop in `vantage_sampler.cpp` visits sources in leaf order, so the cache hits on essentially every query. Decompressing the whole table up front would cost `numleafs * (numleafs+7)/8` bytes — over 100 MB on a large map — to save nothing.

Row length is `(leafCount + 7) >> 3` and the bit for leaf `L` is at index `L - 1`, matching the engine. Leaf 0 is the outside-the-map solid leaf and has no row; a `visofs` of -1, an empty `LUMP_VISIBILITY`, or an out-of-range leaf all fall back to "everything is visible", which is conservative: the traceline still has the final say, it just costs more of them. A map compiled without VIS reports that on stderr.

### LeafForPoint
A plain `dnode_t` descent from node 0, returning `-1 - nodeIndex` once the index goes negative. Note this walks nodes directly rather than the synthesized hull-0 clipnodes, because the synthesis throws leaf *identity* away and keeps only leaf *contents* — which is all tracing needs and all the PVS lookup cannot use.

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

## teleport_connections.cpp

### BuildTeleportConnections
Runs after `BuildLadderConnections` and before the prune passes, so a teleport can rescue areas nothing
else reaches and still be there when `PruneUnreachableAreas` runs. Emits one-way `CONN_TELEPORT`
connections for every `trigger_teleport`.

The runtime half of this predates the generator by a long way: `CONN_TELEPORT`, the connection record
fields, `PlanWalker::Cross`'s teleport branch and the gate-baking skips were all already in place with
nothing producing an edge. This file is the missing producer; it changed no format and bumped no version.

### What is skipped, and why
- **`SF_TRIGGER_NOCLIENTS` (spawnflag 2)** - `TeleportTouch` returns early for players, so no bot can ever
  use it.
- **A non-empty `master`** - `UTIL_IsMasterTriggered` gates the touch on a `multisource` whose state is
  whatever the map's logic has done to it at run time, and navgen cannot know that statically. Emitting the
  edge anyway would need runtime failure attribution to stop a bot walking repeatedly into a dead
  teleporter; the teleport branch now stamps `connIndex` but nothing spends it yet.

  **This is why `rustmill` gets zero teleport connections.** All 48 of its teleports are mastered — it
  rotates which rooms are live — so the map is unchanged by this stage. `gasworks`, whose 3 teleports are
  ungated, gets 12 connections (one per area that can reach a trigger).
- **A `target` matching no entity, or a destination that lands off the mesh.**

### The destination is found the way the engine finds it
`FIND_ENTITY_BY_TARGETNAME` walks the edict list and takes the first `targetname` match *whatever its
classname* — `info_teleport_destination` has no special standing, and `test21_teleport` in fact targets an
`info_target`. The landing origin is `dest.origin` with the feet offset removed and one unit added
(`tmp.z -= pOther->pev->mins.z; tmp.z++`), which is a **standing origin in world space**. Areas are hull-3
geometry, so `kLandingLift` drops it by `kStandProbeLift` to put it in the same duck-origin space the areas
live in. Getting that conversion wrong costs exactly 18 units and shows up as a plausible-looking wrong
drop, not as a crash.

### The drop is real and it is priced
The engine clears `FL_ONGROUND` and zeroes velocity, so a destination hung above its floor is a fall.
`ResolveLanding` first looks for an area whose `boundPlanes` contain the landing point, then falls back to
the highest area floor directly beneath it — deliberately *not* `FindAreaContainingPoint`, whose
nearest-centroid fallback (reachability.cpp) would happily invent an edge to an unrelated area. The drop
feeds `flightTime` and `PmSim::FallDamageFor`, and `ConnectionCostSeconds` now includes `CONN_TELEPORT` in
the same arm as air moves and ladders, so a lethal drop takes the `1e4` penalty and is never routed. A
level destination costs 0, which is the ordinary case.

### Source areas reuse FindServicePoints
Finding "which areas can walk into this brush volume, and where would the bot stand" is exactly the problem
`FindServicePoints` already solves for `trigger_multiple`/`trigger_once` activators, so a teleport builds an
`ActivatorVolume` with `kind = ACT_TRIGGER` and hands over the **raw** brush bounds — that function does its
own Minkowski expansion by each area's own hull. One connection per source area, deduped on the *ordered*
`(from, to)` pair, keeping the lower fall damage.

It inherits one limit from `ServiceableArea`: areas with `activation != ACT_NONE` (door, breakable) are never
sources, so a teleport trigger sitting inside a doorway is missed.

### What the pruners do with them
Nothing. `PruneWalkableAirmoves` and `PruneRedundantLadders` neither judge teleports nor accept them as
alternatives, so a teleport is never dropped and never makes an air move look redundant. Conservative: a
free teleport arguably *should* retire a parallel air move, but that is a change to the prune, not here.

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
Serializes MergeResult to crossbot_nav format (v22):
- Header: magic "CBNV", version 22, mapName[64], bspChecksum, counts (including `detourCount`,
  `detourPointCount` and `vantageCount`), `designMaxSpeed`, `vantageSpacing` and `maxExposure`
- Vantage records (96 B): `pos[3]`, `exposure`, and four `NavWatchDir{yaw, pitch, spread,
  amount, dist}`. Unused watch slots are all-zero and the loader drops them on `amount == 0`, so the
  record stays fixed-size and the section stays one `fwrite`
- `dist` is the mean length of the sightlines that fell in the cone. The pair loop already has the
  delta vector between every unoccluded pair; it used to bin the direction and throw the length away.
  A second per-bin tally sums the lengths beside the counts, and `ExtractWatchDirs` divides the two
  over exactly the bins it folds into each cone. No extra traces, and the tally is freed with the
  counts before the reduction's shape vectors are built
- `exposure` is left raw — the fraction of *all* sampled vantages that can see the spot, before the
  reduction throws most of them away. `maxExposure` rides in the header so the runtime can normalise
  against the map's most exposed spot; storing a pre-normalised value instead would make the number
  meaningless the moment `--vantage-spacing` changed
- `vantageSpacing` is the header's copy of `--vantage-spacing`. The runtime uses it as its grid cell
  size, so it is the one number that has to survive: recomputing it from the point cloud would give a
  different answer on a map where the reduction spent its budget unevenly
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

### Gravity multiplier visualization
Air moves sampled under a trigger_gravity only exist for a bot at that gravity. The two sets (default and trigger_gravity multipliers) are drawn separately so they are tellable apart instead of overlapping on the SVG.

### WriteVantageSvg
A second, separate SVG (`--dump-vantage-svg`), not another overlay on the area graph: the vantage glyphs need the whole canvas to be readable. Areas are drawn flat grey at 0.35 opacity purely as a map backdrop, in the same lowest-floor-first painter's order.

Per vantage, inside one `<g>` so the whole glyph shares a hover tooltip:
- **Watch wedges** — an arc sector per watch direction, radius proportional to `amount`, angular half-width equal to `spread` (clamped to 4..80 degrees), one colour per rank, painted lowest rank first so the main direction lands on top.
- **The dot** — filled from a blue/yellow/red heatmap over `exposure / maxExposure`, so the most-seen floor in the map is red regardless of the map's absolute scale.
- **The tooltip** — id, position, raw and normalized exposure, all four focus values, and every watch direction's yaw/pitch/spread/amount.

World Y is flipped (`x, -y`) like the main SVG, so wedge endpoints are computed in world space and flipped rather than reasoning about screen angles. The arc always uses `sweep-flag=1` because the flip turns world-counterclockwise into screen-clockwise.

## Trigger Forces in Sampling

### trigger_gravity handling

`trigger_gravity` copies its own `pev->gravity` onto whoever touches it, and that sticks, so the set of gravities a bot can ever be running at is the default plus one per distinct trigger. The sampler classifies each trigger and creates a separate gravity pass for each one.

Air moves are sampled once per gravity a trigger_gravity can put the bot in. The ones baked for a gravity we are not currently at simply do not exist for us at plan time. Air moves from an earlier gravity pass must not block the current one: each gravity gets its own set.

During the preview path (`--dump-path`), the planner samples at the lowest gravity multiplier the map hands out. The lowest gravity unlocks the most air moves, so sampling there ensures that a map whose route exists only under a trigger_gravity does not read as unroutable offline.

### trigger_push handling

`trigger_push` is `SOLID_TRIGGER`: it never blocks, carves or supports, it only shows up in the movement simulation and as a launch seed. The touch test uses the same point-content test of the player origin against the trigger's hull-expanded clipnodes that the engine uses in `SV_TouchLinks`, which is exactly what `OnLadder` does in the sampler.

### Launch seeding near trigger_push

A push field sitting on flat floor has no perimeter edge with void ahead, so the perimeter sweep never launches there. For such fields, the bot's own footprint boundary is seeded instead, aimed inwards at the field. This makes the seeding behave as if a bot were running into the shaft and jumping from the edge.

Cyrus-Beck clipping is used to find the side segment against the face winding in plan view for seeding.

### Height adjustment for gravity-modified jumps

The `64`-unit default launch height is step height plus a plain jump at gravity 800. Under a trigger_gravity, the bot climbs back up a taller drop, so the seeding gate has to move with it. The adjusted height is `2 * kStandProbeLift + PlainJumpHeight(gravity)` where gravity is the current multiplier being sampled.