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
spends it on landing. The teleport branch stamps it too, but nothing spends it yet: a teleport that
does not fire is not detected. Walk/door keep `-1` and are the remaining extension points.

## nav_mesh.h

### NavMesh struct
`planAreas` and `planConns` are the planner's view of the same graph, built once when the file is loaded. The area gates in particular are what the funnel routes through, and rebuilding them per repath would repeat the same plane intersections every time.

### Area struct
- `planes` - inward-facing bound planes (for prism areas, includes footprint vertical planes + z-slab)

Activation data lives only on `planAreas`, loaded straight from the area records. `Nav::Area` used to mirror
`doorActivation`/`doorActivatorPos`/`doorWaitTime`, which nothing outside the loader ever read.

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
outside, because the caller compares against the point it asked for. Read the plan's last waypoint
before believing a "stopped short" report.

`test21_teleport` used to be the standing example here: its `info_target` sits in a sealed pocket, so
the plan stopped 112 units short at `goal (448,336,32)` and the harness called it a failure. It is not
an example any more — navgen now builds the `trigger_teleport` edge into that pocket and the map
passes.

### Carefulness: routing away from exposure

`PlanPath` takes a `careful` factor in `[0, 1]`. It weights every edge's time by the vantage exposure
at that edge's crossing point:

```
edge score = time * (1 - careful) + time * exposure * careful
```

`careful = 0` is the plain fastest route and reproduces the pre-carefulness planner exactly — the
factor collapses to 1 and the score is the time. `careful = 1` drops the time term entirely and
minimizes exposure alone. It is **deliberately uncapped**: a bot with nothing to fight with should
take the covered way however long it is. `Repath` feeds it `Bot_Carefulness`, which is
`(1 - combat readiness)` scaled by `crossbot_careful` (`doc/bot.md`, "Carefulness").

**The exposure is cached on the connection, not blended during the search.** `nav_loader.cpp` resolves
`Connection::vantageExposure` once per map: `ConnectionCrossing` for the point, lifted to eye height
(`kVantageMeanEyeLift`, or `kVantageDuckEyeLift` when either endpoint is a crouch area — areas are in
duck-hull-origin space, so the raw crossing z is 29 units below where the probes were sampled), then
`BlendVantage` and a divide by `field.maxExposure`. Roughly 2200 blends on crossfire, none of them
allocating, so it is lost in the load.

Two traps live in that pass. `BlendVantage` returns `out.valid`, which is *`watchCount > 0`* — a spot
nothing looks at returns `false` carrying a perfectly good exposure, so gate on `blend.neighbors`
instead. And the fallback for an unresolvable edge is **1.0, not 0**: an unknown edge must collapse the
factor to 1 and behave like careful mode is off, never become free. A map built with `--no-vantage`
therefore plans identically at every `careful`. The normalized exposure is `blend.exposure / field.maxExposure`, clamped to `[0, 1]`.

**`g` stays true seconds.** `SearchNode` carries `score` *beside* `g`, and only `score` drives the
ordering, the dominance test and the open-list key. Nothing touches `PlanWalker`'s lengths or
`m_extraTime`, which is why the ETA needs no separate accounting at all: it is built by
`BuildPathProfile` from waypoint geometry (`doc/bot.md`), never from the search cost, so a careful
route reports the longer time it will actually take to walk.

**The heuristic has to shrink with it.** The factor can go below 1, so the straight-line estimate is
scaled by `1 - careful + careful * minExposure`, the lowest factor any edge can carry
(`NavMesh::minConnExposure`, also resolved at load). Without that scaling A* would overestimate and
quietly return suboptimal routes. Every edge is scaled by at least this bound `(1 - careful + careful * minExposure)`, so scaling the straight-line time estimate by the same bound keeps the heuristic admissible — it remains consistent with the search's own edge weighting. The cost is that the heuristic weakens as `careful` rises and the
search drifts toward Dijkstra — on crossfire one leg goes 45 expansions / 7ms at `careful 0` to 107
expansions / 21ms at `careful 1`. Bounded, but it is the number to watch.

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

**`g` stays true seconds.** `SearchNode` carries `score` beside `g`, and only `score` drives the
ordering, the dominance test and the open-list key. Nothing touches `PlanWalker`'s lengths or running cost,
which is why the ETA needs no separate accounting: it is built by `BuildPathProfile` from waypoint geometry,
never from the search cost, so a careful route correctly reports the longer time it will actually take to walk.
A node's `g` is the string-pulled travel time plus each connection's `costSeconds` — real seconds, unweighted
by exposure.

The edge cost scaling for exposure weighting only applies to `score`: `score = parent.score + (g - parent.g) * (1 - careful + careful * exposure)`. This separation lets the planner bound the true path length independently of the exposure weighting.

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

Two lookups that answer different questions, and the names say which. `FindAreaExact` asks *which area
contains this point* and is allowed to answer none. `FindAreaNearest` asks *which area is nearest* and is
a total function that essentially never fails. Picking the wrong one is a recurring bug — a strict lookup
where a forgiving one belongs makes a bot on a lip forget where it is, and a forgiving one where a strict
one belongs scores a maneuver against an area the bot never reached (`doc/bot.md`, `FinishLadder` and
`FinishElevatorRide`).

### AreaContainsPoint
The shared predicate both lookups are built from: AABB test with 4-unit epsilon, then exact plane test
(2-unit epsilon). Public because a caller sometimes knows *which* area it is asking about, and asking
directly is not the same question as sweeping the mesh. Areas abut on shared planes, so a point on a
seam is inside both within the plane epsilon and `FindAreaExact` returns whichever comes first in id
order — arbitrary. `FinishElevatorRide` tests its ride target with this first for exactly that reason
(`doc/bot.md`, "The target area wins a boundary tie"); the stop slab and the shaft below it share a
plane, and the sweep would always answer "shaft".

### FindAreaExact
`AreaContainsPoint` against every area, first hit wins. Null when nothing contains the point. Note the
point is tested in the mesh's own space: areas are built in hull 3, so their z range is duck-origin z,
and a caller holding a player's `v.origin` may need `BotNavZ` first. It usually does not matter,
because an ordinary area spans floor to ceiling and swallows an 18-unit offset — it matters wherever
areas are thin, which in practice means elevator stop slabs.

### FindAreaNearest
`FindAreaExact` first, then two fallbacks:
1. nearest centroid among areas whose AABB holds the point
2. global nearest centroid

### FindAreaById
Direct index lookup with fallback to linear search.

## nav_loader.cpp

### LoadNavFile
Loads and validates .nav file:
- Checks magic ("CBNV") and version (21)
- Reads area records, connection records, area-connection indices, area planes, activation detours and
  detour points, then the vantage section
- Verifies BSP checksum against live map
- Converts to runtime NavMesh with PlanArea/PlanConnection views
- Builds PlanArea boundPlanes from Area planes
- Builds PlanConnection from Connection (including gates, launch params)
- Converts `NavVantageRecord`s into `VantageField` and calls `BuildVantageGrid` once

The version check is a hard equality, not a floor, so a file from an older navgen is refused outright
rather than read with a shifted layout. Every record is `fread` straight into its struct, so any layout
change is a version bump and a full regeneration of every `.nav`.

## nav_camp.h

### Camp spots are carried, not consumed
navgen picks a reduced list of camp spots -- position, one facing, and four arc ratios per stance --
and the loader lands them on `NavMesh::camps` as a flat vector. The runtime consumer is
`bot_camp.cpp` (`doc/bot.md`, *bot_camp.cpp*); `crossbot_camp_status` and `--dump-camp-svg` on the
navgen side are still how you look at the data itself.

There is deliberately no spatial index, and the consumer that arrived did not need one. `FindAreaExact`
is still a linear scan and camp spots are a hundred-odd records, so the one pass over `mesh.camps` runs
once per retarget rather than per frame; the vantage field got a CSR grid only because it is blended
per bot at 10 Hz. Measured on crossfire: 114 camps, and the pass keeps between 4 and 79 of them
depending on where the bot is and how far off its target is.

### Why this is not part of the vantage field
The vantage field is a *continuous* signature: `exposure` and the watch cones are properties of a
neighbourhood, so IDW between the four nearest samples gives a meaningful answer anywhere. Camping is
discrete. The midpoint between two camping spots is usually not a camping spot, and the old `focus[4]`
was measured around each probe's own `watch[0]`, so blending two probes with different main directions
blended two numbers that were never about the same facing. `focus[]` is gone from the format.

## nav_vantage.cpp

### What the field is for
navgen samples, for every reduced vantage, up to four `WatchDir{yaw, pitch, spread, amount}` — the cones
that everything with line of sight to that spot arrives *through* — plus `exposure`
(`doc/navgen.md`, *vantage_sampler.cpp*). This is the runtime half: turn a bot's eye position into a
blended signature it can steer by. `bot_sense.cpp` refreshes one per bot every 0.1 s.

### The grid exists because there was no spatial index at all
`FindAreaExact` is a linear scan over every area, and nothing else in `src/nav/` indexes space. A blend
runs per bot at 10 Hz, so it gets its own CSR uniform grid: count pass, prefix sum, fill pass, one
allocation for `cellStart` and one for `cellItems` rather than a `vector` per cell. Cell size is the
header's `vantageSpacing`, floored at 32 and doubled until the cell count is within
`64 * vantages + 4096`, which keeps a long thin map from allocating a mostly-empty lattice.

`FindNearest` walks cells in expanding rings and stops once the fourth-best distance is inside the
searched ring's inner radius — the ring is a cube shell, so its inscribed radius is `ring * cell`, and
anything closer than that cannot be hiding in an unsearched cell.

### Blending: scalars interpolate, directions do not
`exposure` is IDW-weighted over the four nearest with `1 / (d^2 + 1)` — the same weight
and the same epsilon navgen's reduction scores its own decimation against, so the runtime reproduces the
interpolation the decimation assumed was possible.

Watch directions cannot be averaged that way: they are a variable-length list, and a fixed-length vector
is the only thing that has a distance. So they are merged angularly instead, with the same greedy
peak-and-absorb `ExtractWatchDirs` runs — take the heaviest entry, mean-shift once over everything
within 45 degrees, absorb them with summed weight and a weighted-RMS spread, repeat to at most four.
The difference is that navgen runs it over a 512-bin histogram and this runs it over at most 16 sparse
entries, each entering at `idw * amount`.

**Merged directions under 5% of the kept weight are dropped.** A far neighbour contributes a direction
whose weight rounds to nothing, and without the cut it survives as a fourth watch dir with
`amount 0.000` — which `ApplyVantageLook`'s nearest-to-path search would happily snap the bot's head
onto. The threshold is navgen's own `minClusterFraction`. Amounts are renormalised over what survives,
so `watch[0].amount` is always the largest and always meaningful as a denominator.

### Round-tripping is the correctness test
Blend at a stored vantage's exact position and the distance-0 neighbour dominates the IDW completely, so
the result must be that vantage's own record back. `crossbot_vantage_status <x y z>` exists to check
exactly that; anything else means the loader, the grid or the merge is wrong.

## astar_graph.h

Header only — there is no `astar_graph.cpp` any more. The search itself lives in `plan_search.cpp`;
what remains here is the two things both it and the graph builders need: `kNavRunSpeed`, and the
`Connection` view of an edge (endpoints, one-way flag, `costSeconds`, crossing point).

`costSeconds` is **additive time**, not the distance multiplier `traverseCost` used to be. That is what lets a
door contribute the seconds it takes to open; a multiplier had nowhere to put "wait 4 seconds". It already
includes an air move's `flightTime` and a door area's `openSeconds` (see `ConnectionCostSeconds` in
`area_merger.cpp`), so `PlanWalker` must not add either again — the search adds `costSeconds` once per
edge and the walker only measures string-pulled distance.

**`costSeconds` is a surcharge, not a travel cost, and a plain walk connection therefore costs zero.**
That is correct for the planner, which measures the walk itself, and a trap for anything that wants a
cheap search without a walker: summing `costSeconds` over a route prices every walk edge at 0, so the
"cheapest" area is whichever the heap happens to settle first. That is what `travelSeconds` is for — see
below.

### travelSeconds

A second per-connection cost, written by navgen (`ConnectionTravelSeconds` in `area_merger.cpp`) and used
by coarse searches that cannot afford a `PlanWalker`. It is the walk between the two area centroids over
`kNavRunSpeed`, funneled through the connection's gate: if the straight centroid-to-centroid line crosses
the gate segment it is that line, otherwise it bends around the nearer gate endpoint. Non-walk types get
the two ground legs only (`centroidA → pointA`, `pointB → centroidB`) because the flight, climb or ride
itself is already in `costSeconds`.

It is precomputed because it is a static property of the mesh and because the funnel correction is not
something a runtime caller should be redoing per frame.

**The planner must never add it.** `BuildGraph` deliberately does not: `PlanWalker` measures the real
string-pulled distance, and adding `travelSeconds` on top would charge every route twice. The only
consumer is `Nav::AreaCosts`, whose whole job is to answer "roughly how far is that area" without
planning. `travelSeconds` inherits the centroid model's known bias — it charges a route for visiting
every centroid, which overprices paths through many small areas — and that is acceptable for choosing a
*target*, not for choosing a *route*.

`Nav::AreaCosts` (`nav_path.cpp`) is the full Dijkstra over `travelSeconds + costSeconds +
runtimePenalty`, honouring `CONN_FLAG_ONE_WAY` exactly as `plan_search.cpp` does. It fills a
cost-per-area vector rather than stopping at the first hit, because the callers rank many candidates
against each other and an early exit only saves microseconds on a ~500-area map.

`GateAllowsCrossing` makes it honour the **static** half of the activation gating too, the half that does
not depend on the route taken: entering an `activation != ACT_NONE` area by anything other than a walk or a
ride is impossible, and so is entering it from a neighbour with no `PlanDetour`, a `blocked` one, or one with
no points. That is the same triple test `PlanWalker::Cross` and `PushActivationDetour` apply, so a Dijkstra
that skipped it would price areas the planner will always refuse. `crossfire`'s secret room is the case that
matters: `navgen` already records `detour area 21 <- 23 : BLOCKED (no route to any service point)` because the
`trigger_multiple` that opens `secret_door` sits *inside* the room, and the 12 items behind it read `11.8s
away` to the gather scan while every `Repath` to them failed. What stays outside this test is everything
funnel-dependent — an air move that needs a run-up this approach cannot give — because answering that needs
the walker, which is the whole reason phase 1 is a plain Dijkstra (`doc/bot.md`, "Selection is two searches").

Being seconds is also what lets `runtimePenalty` be added straight on top in `BuildGraph` (see
`nav.cpp`). Anything else fed into the cost model has to be in seconds too, or it will quietly outrank
everything.

`AreaCosts` takes the same `careful` factor as `PlanPath` and applies it the same way, to
`travelSeconds + costSeconds + runtimePenalty` as a whole. **Above 0 the values stop being seconds and
become scores**, which splits the two callers. `BotItems::SelectGoal` passes the bot's carefulness and
is fine with it — it only ever ranks candidates against each other. `BotOppo::RebuildCosts` must leave
it at 0, because `TryJump` subtracts two of these values and compares the difference against a real
simulated flight time (`doc/bot.md`); careful-scaled costs there would silently deflate every gain.
Reachability is unaffected either way: the factor is strictly positive, so nothing that was reachable
becomes unreachable.

### AreaCostsToGoal

The same Dijkstra run backwards: `AreaCosts(a)` prices every area *from* `a`, `AreaCostsToGoal(g)` prices
every area's cost *to* `g`. Both share `AreaCostsImpl`, which takes the direction as a flag, because the
only difference is which endpoint of each edge plays `from`.

Both edge tests have to flip with it, and neither is symmetric. `CONN_FLAG_ONE_WAY` means the edge is
legal only as `areaA -> areaB`, so the forward pass requires `c.areaA == current` and the reverse pass
requires `c.areaA == neighbor` — the reverse walk is looking for the *forward* edge that would have
arrived here. `GateAllowsCrossing` takes an ordered pair for the same reason: an activation-gated area is
enterable only by walk or ride and only from a neighbour with a live `PlanDetour`, and reversing the
search does not reverse the door.

It exists because a caller that has to price *many candidate positions against one fixed goal* would
otherwise run one search per candidate. One reverse pass turns every such question into an array index,
which is what makes a 4 Hz check affordable (`doc/bot.md`, `bot_opportunist.cpp`). Its values inherit
everything `travelSeconds` inherits: centroid-anchored, and priced at `kNavRunSpeed` rather than the
bot's real speed. They are plan-seconds, not real seconds, and mixing them with a real duration without
converting is the trap the opportunist section describes.

### drift.h: direction-aware cost

A `func_conveyor` or a horizontal `trigger_push` makes a connection cost different amounts in its two
directions, and sometimes makes one of them impossible. `doc/navgen.md` (Drift Fields) has how the fields
are found and how areas get their `driftVelocity`; this is what the searches do with it.

`crossbot_nav::DriftTerm` is five floats on `NavConnectionRecord`, and **carries no speed**: `length` (how
much of the crossing lies in drifting areas), the length-weighted mean `along` and `perp`, and the
`alongMin`/`alongMax` extremes. `NavDrift::EvalDrift(term, maxSpeed, forward)` turns it into
`{seconds, passable}` for one direction. `forward` means areaA -> areaB, and reversing it negates `along`
and swaps which extreme is the worst case.

This is deliberately evaluated per search rather than baked. `Bot_MaxSpeed` reads `pEdict->v.maxspeed`
before `sv_maxspeed`, so the speed is per-server and per-bot; a 280 u/s belt is walkable upstream at 320
and impossible at 270, and a mesh that had decided in advance would send a bot onto an edge it cannot
cross. `EvalDrift` returns `{0, true}` the moment `length` is zero, so on a map with no fields the whole
feature costs one float compare per edge.

`seconds` is a **correction on the existing `kNavRunSpeed` baseline**, not a replacement for it: negative
downstream, positive upstream, and bounded by the crossing's own baseline time. That bound is what keeps
`PlanWalker::Time()` monotonic, so the `std::max(next.Time(), state[current].g)` clamp in `PlanRoute` never
swallows a step.

Both searches consume it:

- `PlanRoute` (`plan_search.cpp`) skips an impassable direction and folds `seconds` into
  `AddCost(gc.costSeconds + drift.seconds)`, so it rides in `m_extraTime` alongside door waits and flight
  time. `PlanPath` passes `traces->physics.maxSpeed`, falling back to `mesh.designMaxSpeed`.
- `AreaCostsImpl` (`nav_path.cpp`) skips the same directions and adds `seconds` to
  `travelSeconds + costSeconds + runtimePenalty`, clamped at zero. `AreaCosts`/`AreaCostsToGoal` take a
  `maxSpeed` argument; the bot call sites pass `Bot_MaxSpeed(bot.pEdict)`.

The heuristic still divides by `kNavRunSpeed`, so a downstream run that beats 250 u/s makes `h` an
overestimate and optimality becomes best-effort. `PlanRoute` already re-opens closed nodes, so this is a
difference of degree from the bias `travelSeconds` already carries, not a new failure mode.

Blocking is expressed by `EvalDrift` returning `passable == false`, **not** by a connection flag. There is
no `ONE_WAY_BACK` bit: the answer depends on the speed of whoever is asking. `CONN_FLAG_ONE_WAY` still
means what it always did; `crossbot_nav::ConnAllowsDirection` is the shared test wherever the question
is "may I leave `from` along this connection". The sites asking a different question -- `airmove_prune`
testing whether an edge is bidirectional, `activation_detours` looking at entries into an area -- keep
their own inline check.

### Following a drifting path

`NavFollow::Config::driftVelocity` is the world-frame shove the bot is currently subject to.
`Bot_DriftVelocity` reads it live from the engine — `groundentity->v.movedir * speed` when the ground entity
has `FL_CONVEYOR`, plus the cached `Bot_PushFields` closure at the bot's origin — so a `Use`-reversed
conveyor or a switched-on field is handled without rebaking. The closure is cached on `CBot::pushFields` and
refreshed with each repath, because scanning every edict per frame per bot is not free.

`Track` does two things with it. The world-speed cap becomes `DriftSpeedAlong(drift, aimDir, maxSpeed)`
instead of `physics.maxSpeed`, so a bot riding a belt is allowed to exceed its own run speed. And the wanted
velocity handed to `SolveDrive` is `aimDir * target - drift`: `pev->velocity` is own-only, `SolveDrive`
solves in the own frame, and its clamp to `physics.maxSpeed` stays correct untouched. Without that
subtraction a bot crossing undertow's 420 u/s current is simply carried away from its own plan.

`BuildPathProfile`'s `NodeMaxSpeed` applies the same `DriftSpeedAlong` using the waypoint's outgoing segment
direction (the last waypoint uses its incoming one), which is where `PlanWaypoint::drift` — copied from
`PlanArea::drift` in `PlanPath` — is used. Corner radii, the forward/backward speed pass and `totalTime` all
follow from it, so `estimatedArrivalTime` stops being wrong by up to 2x on a belt.

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

An elevator ride contributes nothing to that integration and has to be added on top. Its two waypoints are
the mover's column centre at two heights, so `Dist2D` is zero, the segment is never pushed, and the whole
ride — travel *and* the departure stop's auto-return dwell — is invisible to every pass above. A trailing
loop adds `elevatorWaitTime + elevatorTravelTime` per ride waypoint. Door and button dwells have the same
gap and are still left to `kEtaSafetyMargin`; a ride cannot be, because on `test55_plat` it is twice the
walking time.

A ride's two waypoints share an XY, so its segment has zero length and the integration above cannot
see it at all. Add the time here or the ETA reads as a walk to the shaft and back out again.

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

**`done` means arrived, and only that.** The cross-track correction can cancel the tangent exactly — a
bot barred at a node by `sLimit` and standing beyond it along the tangent sees the chain point directly
behind, and the two unit vectors sum to zero. That used to fall through to `d.done`, which
`Bot_MoveNav` announces as reaching the goal and acts on by dropping it, wherever the bot happened to
be. `test18_elevator` hit it while barred at a lift, printing `reached goal` 312 units away and mid
shaft. A degenerate blend now keeps the raw tangent, and the surviving zero-direction case — which
needs an empty tangent, not a cancelled one — asks for a repath instead of claiming success.

The rejoin pull can cancel the tangent exactly -- a bot barred at a node and standing beyond it
along the tangent sees the chain point directly behind. Falling through to `done` there reports
an arrival wherever the bot happens to be, so keep the raw tangent and let the lateral test
decide whether this is worth a repath.

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
doubles friction by `sv_edgefriction`. `byRate` sidesteps the *stall*: the further the velocity is from
`V*`, the larger the commanded wish speed, so the bot saturates the frame and closes on `V*` from any
starting speed. See `doc/bot.md`, `RunAirmoveApproach`, for the dropoff this stalled.

What it does *not* do by itself is arrive at `V*`, and that is the next rule.

**`byRate` does not cover for an unmodelled friction — it converts it into a standing speed error.**
The formula is a deadbeat solve: it asks for the one wish speed whose `sv_accelerate * dt * wishspeed`
exactly replaces the drop, so its gain is set by whatever `f` the caller passed. Feed it a friction
lower than the engine's and it settles short, permanently. At the fixed point the engine takes
`stopSpeed * f_real * dt` per frame and returns at most `accelerate * dt * W`, so holding still needs
`W = stopSpeed * f_real / accelerate`; `byRate` produces that W at

```
v = V* - stopSpeed * (f_real - f_model) * dt
```

which for an unmodelled ledge doubling is a flat `V* - 400 * dt` at defaults — 4 units short at 100 fps,
6.7 at 60, whatever `V*` is. Measured on test53_ladder4's crouch jump: 66.8 / 66.4 / 66.2 against a
design 70.8, on frames of 10.0 / 11.0 / 11.6 ms.

The error exists only *below* the `stopSpeed * groundFriction * edgeFriction / accelerate` threshold
above — 80 at defaults. Above it `byCap` wins instead, `W = V*`, and a target that high is
self-sustaining against edge friction anyway, so the model error is invisible. That is why this hid for
so long: it needs a design speed under 80, and the one place that reliably produces one is a **crouched**
approach, capped at `maxspeed * kDuckingMultiplier` ≈ 90 and usually decelerating into a launch below
that.

`physics.edgeFriction` is therefore consumed by the follower, but not by `SolveDrive` itself, which stays
a pure function of the `PmPhysics` it is handed. `Bot_MoveNav` runs the probe and hands it physics whose
`groundFriction` is already doubled — see `BotDrivePhysics` in `doc/bot.md`. `BuildPathProfile` is
deliberately left alone: it is a whole-route model built once at plan time, and folding a momentary
ledge state into it would make design speeds depend on where the bot stood when it replanned.

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

## airmove_cache.cpp

### Why the planner caches whole simulations

`PlanWalker::JumpLandsIn` is the single most expensive thing the plugin does. Each call builds a
`PmSimulator` and steps it for up to `kAirmoveSimMaxTime`, and every step costs one engine `pfnTraceHull`
through `Bot_EngineTraceFn` -- 1 to 5 traces per step, roughly 2 typical, over 30 to 360 steps. Most of
those sims are *rejections*: A* relaxes an air move edge, the jump misses the destination area, the edge
is dropped. The same connection gets rejected over and over, by every bot, on every repath.

Measured on a 32-bot crossfire server before the cache: 18,000 hull traces per frame charged to
`ZONE_RP_PLAN`, 53% of all engine trace time. Plan cost tracks sim count almost exactly -- plans with no
sims average 0.5 ms, plans with 200+ sims average 27 ms and peak past 60 ms.

The unit that gets cached is the whole simulation, not the individual traces. A single trace is only
0.0008 ms; the win is in never running the 200 of them again.

### The key

A simulation's trajectory is determined by the state `sim.Begin` receives plus the physics the stepper
reads, so `AirmoveKey` is exactly that and nothing else:

- the **sim start origin** -- `launchFloor` lifted by `launchLift + kPlanLaunchLift`. Folding the lift in
  means `fromAreaId` drops out of the key: it only ever mattered through `PlanArea::crouch` picking that
  lift.
- `yaw`, `speed`, `jump`
- `crouch` (`needsCrouch`), which selects the duck vs stand hull and push function
- a hash of the four physics fields that vary *and* are read by the stepper: `maxSpeed`, `gravity`,
  `airAccelerate`, `maxVelocity`. `accelerate`, `groundFriction`, `stopSpeed` and `edgeFriction` are read
  by the profile and run-up maths, never by `PmSimulator`, so they are not in the key. `friction` is
  forced to 1 and `frameTime` is pinned (below).

`destAreaId` is deliberately **not** in the key. The cache stores the landing point and how the sim
terminated; `ApplyAirmoveResult` runs `PointInArea` against the destination afterwards. One cached
trajectory therefore answers the question for every destination anyone asks about it.

Keys are matched on exact float equality -- there is no quantisation and no tolerance. That was a
deliberate choice: it means a cache hit returns bit-for-bit what the simulation would have returned, so
the cache cannot change a plan. It works because the launch point, while continuous in principle, is
`NearestPointOnGate` of the funnel-pulled line against a fixed gate, and that clamps to the same values
far more often than it looks like it should. Measured hit rate is 94% on crossfire and 56% on xen_dm from a
4096-slot table.

### Pinned frame time

`JumpLandsIn` overrides `physics.frameTime` with `PmSim::kPlanSimFrameTime` (1/60) on its local copy.
Two reasons, and the caching one is the lesser:

1. `PlanTraceContext::physics.frameTime` is `Bot_FrameTime()`, the latched live server frame time. It
   jitters every frame, so without the pin no plan-time simulation would ever be repeatable and the cache
   could never hit.
2. navgen sampled every `CONN_AIRMOVE` at a fixed 1/60 (`AirmoveParams::frameTime`). The runtime sim was
   therefore integrating at a different step from the data it was validating against, and drifting further
   the more loaded the server got. Pinning makes plan-time agree with the baked connection data.

The pin is scoped to the simulation. `BuildPathProfile` and `AirmoveLaunchLead` keep the live frame time,
because those model how many real frames of acceleration the bot actually gets.

The pin is a no-op inside navgen, whose `PlanTraceContext.physics` already comes from
`PM_MoveSimulator::Physics()` at the sampler's 1/60.

### What may be cached, and the one case it gets wrong

`PmTrace` carries `hitEntity`, set by `Bot_EngineTraceFn` from `tr.pHit`. Three things make a hit count as
world rather than as an entity:

- `tr.pHit` is null or is edict 0 (worldspawn).
- it is a `SOLID_BSP` brush carrying `FL_WORLDBRUSH` — a brush the engine merged into the world, which can
  never move or be removed.
- `crossbot_nav::IsBakedBlocker` accepts it. This is the **same predicate navgen used to bake the entity
  into the mesh** in the first place (`baked_blockers.h`, called from both `navgen::IsBlockingEntity` and
  the runtime trace). `func_wall`, `func_healthcharger`, `func_recharge`, plus `func_wall_toggle` that does
  not start off and `func_conveyor` that is not flagged non-solid. The argument is that the nav data is
  *already* committed to these being solid — navgen carved the areas and sampled the air moves with them
  in place — so treating them as world at plan time is consistent with the mesh, not a new assumption.

  Keeping the two in one function matters. If navgen's list ever gains or loses a classname and the
  runtime does not follow, the planner starts caching against geometry the mesh does not agree about.

The trace is `ignore_monsters`, so players and monsters never appear in it at all. Doors, plats, trains and
breakables are what is left, and they are what makes a sim uncacheable.

`PmSimulator` ORs the flag across every trace it makes into `m_touchedEntity`, and also sets it when the
`trigger_push` lookup returns a non-zero vector — `Bot_PushFields` snapshots live entities and does a
`pfnTraceModel` per overlapping field, so a sim that entered a push field depends on entity state too.
`JumpLandsIn` inserts only when `TouchedEntity()` is false.

That rule is not quite "safe for the rest of the map", and it is worth being precise about why. It
guarantees the *recorded* trajectory did not depend on a moving entity. It does not guarantee no entity
will later move *into* that trajectory: a jump that flew over an empty shaft when it was recorded gives a
stale answer once the lift rises into the path.

`crossbot_airmove_verify 1` re-simulates every hit and compares. Measured mismatch rates, and **every
single mismatch on every run was this one case** — the fresh sim touched an entity, the cached one had
not. No mismatch has ever come from any other cause, which is what establishes that the key above is
sufficient for static geometry:

| map       | mismatches | of hits  | rate  |
|-----------|-----------:|---------:|------:|
| crossfire |        106 |  271,339 | 0.04% |
| xen_dm    |      2,519 |  254,147 | 0.99% |

Two things push that rate up, and both are deliberate choices made elsewhere in this design. Accepting
navgen's baked blockers means caching trajectories that touch brush entities, and the aging-LFU eviction
below keeps hot entries alive far longer than the old fixed-victim policy did — a longer-lived entry has
more time for the world to move underneath it. xen_dm is the worst case in the map set by a wide margin.

The divergence is left in. A wrong answer means the planner believes a jump clears when a moving platform
now blocks it; the bot attempts it, the follower reports `airmove aborted` and repaths — an existing,
routine path that already fires far more often from ordinary run-up speed shortfall. If it ever needs to
go lower, the levers are a staleness TTL on entries or refusing to cache trajectories whose bounds pass
near a mover; both buy freshness by giving back hit rate.

### Structure and eviction

Fixed 4096-slot table, `kAirmoveCacheSlots` in `airmove_cache.h`. It is a **compile-time constant, not a
cvar** — the table is a plain member array, so there is no allocation, no resize path, and the whole thing
is about 210 KB and stays in L2. A larger table wins a few points of hit rate but pays for it in locality
on every probe.

Lookup is a hash, never a scan: FNV-1a-style mix over the five float bit patterns, the physics hash and the
jump/crouch byte, masked to the table. Open addressing with a linear probe of up to `kAirmoveCacheProbe`
(8) slots, so every operation touches at most 8 entries in one or two cache lines. Lookup stops at the
first unused slot, which is sound because entries are never deleted individually — only wiped wholesale by
`Clear()` on map change.

Eviction only happens when all 8 probe slots are occupied by different keys. Each slot carries a saturating
`freq` (bumped on every hit) and a `stamp` (the global lookup tick at its last hit), and the victim is the
slot minimising

```
score = freq >> ((now - stamp) / kAirmoveAgeStep)     // 0 once the shift reaches kAirmoveMaxDecay
```

ties broken by the older stamp. This is aging LFU evaluated lazily: nothing decays on a timer, the decay
falls out of the comparison. It protects entries that are used a lot *and* entries used recently, while an
entry that was hot early and has gone cold loses its protection exponentially instead of squatting forever
the way plain LFU would. A freshly inserted entry starts at `freq = 1`, so it is the first thing thrown out
if it never gets a second hit — that is what keeps a burst of one-off launch positions from flushing the
working set.

The predecessor policy always clobbered slot `base`, which let two hot keys sharing a base ping-pong
forever. Measured at the same 4096 slots, on 2-minute 32-bot runs:

| map           | fixed victim   | aging LFU      |
|---------------|----------------|----------------|
| crossfire     | 86.2% / 47,021 | 93.6% / 13,300 |
| xen_dm        | 42.1% / 204,364| 55.7% / 119,702|
| rocket_frenzy | 31.3% / 245,222| 43.0% / 196,835|

(hit rate / evictions). For scale, crossfire at 4096 slots with this policy reaches 93.6%, against 95.1%
for a 65536-slot table using the old policy — a sixteenth of the memory for one and a half points.

On the harder maps the ceiling is not capacity and not eviction, it is `uncacheable`: sims refused because
they touched a real mover. Accepting navgen's baked blockers already cut that share of misses roughly in
half on xen_dm and rocket_frenzy. What is left is genuine door, plat and train geometry.

### Ownership

The cache instance is `g_airmoveCache` in `bot_move_nav.cpp`, attached to `PlanTraceContext::airmoveCache`
by `MakePlanTraces`. navgen leaves that pointer null, which is what keeps its multi-threaded sampler
race-free -- `waypoint_plan.cpp` is compiled into both targets. `Bot_ResetAirmoveCache()` is called from
both `BotManager::ServerActivate` and `ServerDeactivate`, mirroring `BotSense::ResetBoneCache()`.

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

**Activation detours.** Entering an area with `activation != ACT_NONE` looks up the precomputed detour for the
neighbour being entered from (`FindDetour`); a missing or blocked entry makes `Cross` return `false` and A*
routes around, so a route can no longer pass through a door whose activator is unreachable from that side. The
stored polyline begins and ends on the *same* gate midpoint the door waypoint is derived from — both sides call
`GateMidpoint`, and if they ever drift the unvalidated straight segment between them comes back. `FlushFunnelTo`
targets the polyline's first point, which is a point on the entering area's boundary; that is the ordinary case
(`UpdateFrontier` already aims at the last pending gate's midpoint), unlike the old code which aimed at a point
inside the button's solid brush. Only the polyline's interior is funnel-validated — the pull from `m_anchor` to
its first point is a straight 2D pull like every other, and can clip a wall in a prism area.

A gated area entered by a non-walk connection is refused. Previously the teleport, ladder and air-move branches
of `Cross` all returned before the door check, so an air move landing inside a door area emitted no activation
waypoint at all.

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

**Elevator:**
- Checked before the activation guard, because a stop area is gated *and* legitimately enterable — the guard
  otherwise rejects every non-walk edge into a gated area.
- When the destination stop is gated, `PushActivationDetour` emits the summon detour first (walk to the call
  button, press, come back). Returns **false** when that detour is missing or blocked — which is how "you
  cannot call this lift from up here" is expressed.
- Adds `m_extraTime += rideWaitSeconds`, then a counted boarding waypoint carrying
  `requiresElevatorRide` / `elevatorTravelTime` / `elevatorWaitTime`, then a free arrival waypoint so the
  shaft's height never enters the funnel. Anchor moves to the arrival point. The wait is copied onto the
  waypoint as well as charged to the search, because the runner's ride deadline has to allow for a
  platform that does not set off until its dwell elapses — see `doc/bot.md`.
- Walking *onto* a gated stop takes the ordinary door path instead: `needsStop` includes a gated elevator area,
  so the same detour runs and `doorWaitTime` (= the summon time) is charged.

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
- Refuses the edge outright when `conn.gravity` does not match `PlanTraceContext::gravityMultiplier`.
  Air moves are sampled once per gravity a `trigger_gravity` can put the bot in, and the ones baked for
  a gravity the bot is not currently at simply do not exist for it. `Nav::AreaCosts` applies the same
  filter so goal selection and route search agree about which jumps exist
- Pushes arrival gate (entry side only)
- If traces available: re-simulates from funnel's preferred launch point, through
  `PlanTraceContext::standPush`/`duckPush` so a launch crossing a `trigger_push` field is simulated the
  same way navgen sampled it
  - Computes funnel crossing on launch gate (exit side)
  - Interpolates yaw/speed from gateAYaw/gateASpeed
  - Re-simulates with PmSim; if lands in dest area, uses simulated params
  - On a miss, one retry at a rescaled speed — see "Rescaling a missed launch"
  - If the retry misses too, `Cross` returns false and the whole edge is refused — there is no fallback
    to the baked params, because a launch that does not land where the plan says is worse than no edge
    at all
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

**FloorPlaneOf** - finds bound plane with highest normal.z (most upward). That is the *flattest* upward
plane, not the highest one, so it is only good for asking whether an area has a floor at all — never for
a height.

**AreaFloorZ** - floor height at (x,y). An area is the intersection of its half-spaces, so every plane
with `normal.z > kWallNormalZ` is a lower bound on z there and the floor is the *maximum* over all of
them. Reading one plane instead is wrong for any area with more than one upward face, which is every
prism area: crossfire's 168 is a ramp plus a small flat patch, and asking `FloorPlaneOf` gave the patch's
z, -1838, for every point on a ramp that climbs to -1684. See "The clamp runs backwards along the route"
for what that cost.

**FloorsMeetAt** - true when two areas' floors agree at a seam point within a tolerance. Both areas must
have a floor plane at all; the heights themselves come from `AreaFloorZ`.

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
- `door`, `breakable`, `activation`, `doorWaitTime` - a door area makes the plan stop at it
- `detours` - one `PlanDetour` per walk neighbour of a gated area, precomputed by navgen. Each is a closed
  polyline of `PlanDetourPoint` starting and ending on that neighbour's gate midpoint, with `actIndex` marking
  the activation waypoint, plus `blocked` when no activator is reachable from that side. `FindDetour` scans
  linearly — a gated area has two to four entries

### PlanConnection
- `planeA`, `planeB` - bound plane indices for connection (baking only)
- `gateA`, `gateB` - shared face gates (built by BuildConnectionGates)
- `gateAYaw[2]`, `gateASpeed[2]` - launch params at gateA ends

### PlanWaypoint
- `point` - position at player height (duck origin + 18 for stand)
- Flags: requiresElevatorRide, elevatorTravelTime, elevatorWaitTime, requiresCrouch, requiresDoor, requiresBreak, requiresButtonUse, requiresTriggerTouch, activatorCenter, requiresTeleport, teleportDest, requiresLadder, ladderUp, ladderYaw, ladderEntryYaw, ladderSpeed, ladderExit, airmoveLaunch, requiresJump, launchSpeed, launchYaw, jumpBackDist, launchSimulated, launchSimFailed, launchSimFrom/To/Aim, targetAreaId
- `activatorCenter` is the activator brush model centre, copied from the detour. The bot aims at it to press:
  GoldSrc `PlayerUse` picks the entity whose centre gives the largest forward dot above `VIEW_FIELD_NARROW`,
  so the runtime cannot press without knowing where the brush is
- `requiresBreak` is serviced at runtime by `BotBreak` — the bot stops, draws its melee weapon and hits the
  brush until it dies. The plan's `openSeconds` (`health * 0.1`) is only the ETA estimate; the runtime
  decides for itself whether the entity is breakable at all, from `takedamage` and `health`. See `doc/bot.md`

### Activation at runtime
`src/bot/bot_move_nav.cpp` services activation waypoints; see `doc/navgen.md` for how the detour is built.

- **Barrier.** `NavFollow::Config` carries `sLimit` and `sFloor`, both arc lengths from
  `NavPlan::ProfileWaypointS`. They exist because a detour doubles back on itself: the outbound and return
  legs are collinear in XY, so `ProjectOntoProfile` cannot tell them apart by distance and resolves the tie
  in favour of the lower `s`. Without `sLimit` the bot's `s` jumps to the return leg and it never stands on
  the activator (worse on first acquisition, which searches the whole profile); without `sFloor` it can never
  leave the outbound leg afterwards and stalls in place. `sLimit` is the next unserviced activation *or
  pending elevator ride*, `sFloor` the last serviced one, both computed in one pass over the waypoints each
  frame. A ride is the same tie in its extreme form — its two waypoints share an XY exactly, so the
  segment between them has zero length and does not exist in the profile at all; see `doc/bot.md`
- **Press.** `RunActivation` takes over the frame inside `kActivationEngageRadius`, aims at `activatorCenter`
  in pitch and yaw, and emits a single `IN_USE` rising edge once the aim converges and the bot is within
  `kActivationUseRange` of the centre in 3D. One edge, not a held button — repeated presses would toggle a
  toggle-style button back off. It then holds for the door's `buttonWaitTime` before releasing the barrier
- **Triggers** need no press, only arrival, so they are serviced by proximity before `Track` runs
- **Memo.** Serviced activators are remembered by centre for `kActivationMemory` seconds. This is what stops a
  mid-detour repath from pressing twice (waypoint indices change, world state does not), and what makes a
  failed press retry on its own once the memo lapses

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
- `airmovesRetracted` - earlier launches whose speed was cut so their landing stops sitting on top of
  the next launch's run-up node. See "Landing on the next run-up".
- `airmovesRetractKept` - pairs where that pull was rejected — the two headings too far apart, or the
  slower jump no longer landing in its own area — so the earlier design speed was kept.
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
route arrives perpendicular to the launch heading, which is what produces the hairpin above. When the
route arrives by *air move* the node can land behind the bot outright — see "Landing on the next
run-up". The `AccelDistance` bound is charged at doubled friction whenever the launch sits at a ledge —
see "Edge friction on the approach".

**The clamp runs backwards along the route, not inside one area.** `PlanWalker::RunupDistance` steps
area by area: clamp inside the current one, and when that falls short of the offset, re-clamp with no
inset to reach the seam and continue in the area the route came from. An area's predecessor is read
straight off `m_pendingGates` — each walk connection pushes a gate pair carrying `insideArea`, and
`m_pendingGates` only ever holds plain-walk gates because every barrier flushes the funnel — so no
adjacency structure has to be plumbed into the walker. The chain stops at a real wall, at a seam the
next area's column does not actually contain, at a seam where the two floors are more than
`kPlanRunupSeamStep` apart, or at a crouch area, where running at `kDuckingMultiplier` would make the
extra distance worth less than the ducking costs.

The floor-continuity clause is there because `InAreaColumn` is a pure XY test — it skips every plane
with `|normal.z| > kWallNormalZ` — so on its own it happily walks a run-up back across a cliff. Crossfire
area 168 is a 26.57° ramp from the pit floor up to a ledge, sharing its column with the pit below it;
the run-up for the ramp-top air move was placed on the pit floor 150 units under the launch, and the bot
spent the rest of the round stalling and re-planning between a launch it could not walk to and the point
it kept falling back to. `FloorsMeetAt` compares `AreaFloorZ` on both sides of the seam and stops the
walk-back where they disagree by more than a step.

Confining it to one area silently capped the launch speed on thin ledges. `test54_window`'s area 3 is
a 16-unit-deep lip between the window and the pit; the 3→17 jump wants 26.7 units of run-up and got
3.2, so the bot reached 159 of a 267 design speed and aborted the jump every time. Extending back
through the window into area 2 gives the full offset and it now launches at 267 exactly.

Two invariants make the extension safe. The straight run-up→launch leg is not string-pulled, but every
step of the walk-back is clamped inside a convex area and each seam point is checked against the next
area's column, so the leg lies inside the union by construction. And the gates the run-up point now
sits *behind* are dropped before `FlushFunnelTo` — otherwise the pull would be forced out through a
gate the target is on the near side of, and come back. Those gates only constrained the leg the funnel
no longer draws.

The cost is that a run-up node absorbing a former gate corner also loses that corner's `clearance`,
falling back to the default 16 where the gate may have reported 32. That is the conservative
direction — a narrower corridor shrinks arcs rather than cutting them — and it is why `test5_jump`
trades two corners for one and reports one more `slowed`.

It is load-bearing for a second reason that has nothing to do with alignment. `Bot_MoveNav` hands
control to `RunAirmoveApproach` when the tracker's `wpIndex` reaches the launch waypoint, and
`wpIndex` comes from whichever profile segment the bot projects onto. `BuildPathProfile` drops
segments shorter than `kEpsLen`, and the funnel's last corner lands on the launch point exactly, so
without this node the segment ending at the launch is zero-length, gets dropped, and no segment ever
carries the launch's index. The bot then walks straight through the launch point and never jumps.

### Launch speed is clamped to the server
`candSpeed` comes out of the nav file's `gateASpeed`, which the sampler produced at navgen's assumed
`maxSpeed` — 320 by default. The server need not agree: `D:\hlds\valve\server.cfg` ships `sv_maxspeed 270`,
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

### Rescaling a missed launch
`Cross` interpolates yaw and speed from the gate ends at the fraction the funnel crosses, and that pair
was never flown by any sample — the gate endpoints sit at mean ± 1σ along the fitted segment and the
yaw/speed there come from a one-sided local regression, so at `t` 0 or 1 the planner is extrapolating.
An overshoot or undershoot of a few tens of units is routine, and refusing the edge on it loses genuine
connectivity.

So a miss gets exactly one retry, and the correction is one division: `want / conn.flightTime`, where
`want` is the destination centroid projected onto the candidate heading. `flightTime` is the air time
of the sampled trajectory the connection was built from — a flight that really did land in the
destination — so the division asks the only question that matters: how fast must the bot leave the gate
to cover `want` in the time the geometry allows.

Scaling `candSpeed` by `want / |end - exitPoint|` instead is the obvious correction, and it is the
*same number* whenever the miss ends at the destination's height, because there `|end - exitPoint|` is
`candSpeed * flightTime` and the two forms cancel. They diverge exactly where the assumption behind the
ratio — that air time barely depends on launch speed — breaks: when the missed flight ends somewhere
else vertically, its horizontal reach carries a different air time and the correction inherits it.
`test54_window`'s `1->2` is the extreme case: a 51-unit hop up onto a window sill whose first attempt
clears the sill, misses the 16-unit lip past it and falls 224 units into a pit — 192 units of travel
over 1.16 s. The ratio folds that fall in and answers 44 for a jump that needs 90. Dividing by the
connection's own 0.57 s answers 91, and it lands.

**The aim point is the destination centroid, not the funnel's own crossing of the arrival gate.** The
crossing is where the string pull wanted to enter, which is by construction the *nearest* corner of that
gate, and aiming a rescaled jump at the edge of the landing area is how you land just outside it. On
`test54_window` the `1->3` crossing asks for 207 and sails over the 16-unit lip into the pit; the
centroid asks for 152 and lands.

Nothing iterates. A second miss refuses the edge exactly as before, and the retry is skipped when `want`
is under `kPlanAirmoveRangeEps` or the correction moves the speed by less than `kPlanAirmoveRespeedEps`.
The rescaled speed is clamped to `physics.maxSpeed`. A connection whose `flightTime` is under
`kPlanAirmoveFlightEps` falls back to the old ratio, where that clamp is also what bounds the degenerate
case: a first sim that wedges immediately travels ~0 units and the ratio blows up, so it retries at full
speed and almost always misses again — one wasted simulation, never an unbounded one.

**Correcting downward composes with `Respeed`; correcting upward does not.** A retry that raises the
speed can assert a launch the run-up cannot deliver, and `Respeed` will then knock it back down and keep
the design speed if the slower jump misses. In practice the corrections that matter are reductions,
because the extrapolated gate speed is biased toward the cloud's interior mean and overshoots.

`test54_window` is the map this exists for. Its spawn room is a closed box whose only exit is a window
24 units up the wall, so navgen finds no launch edge there at all (`FilterLaunchEdges` needs void ahead
at running height) and every jump out of it is reverse-seeded from the landing spread of the jumps back
in — a 2D blob squashed into a segment, the worst case for the end regression. Both edges out of the
room miss, and area 1's only other exits drop into the pit, which cannot climb back to the sill level —
so until the retry corrects `1->2` to 91 and `1->3` to 152, the map has no route at all.

Across the 19 stock deathmatch maps the flight-time form converts 8 more retries from `landed elsewhere`
to `landed in area` (161 of 582 to 169 of 586) and moves three routes — `contamination` 28 waypoints to
25, `disposal` 11 connections to 10, `datacore` 14 to 16 — leaving every other route byte-identical. On
the test maps `test28_slopecase` trades four corners for a crouch jump (22 waypoints to 21) and
`test31_push` shifts a few waypoints; nothing else moves, and all 51 pass.

### Re-speeding a launch
The stored `exitSpeed` is what the air-move sampler launched at, not what the route can deliver. A
corner 30 units before the launch caps the bot well under it, and nothing used to notice: the plan
asserted the design speed and the jump was validated at a speed the bot would never reach.

`PlanWalker::Retract` runs first (see "Landing on the next run-up") and may lower an earlier launch;
the comparison below is against `PendingLaunch::designSpeed`, which `Retract` rewrites, so the two
passes compose instead of fighting.

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
`launchSimFailed` to reason about. It is back to never firing on the test maps, but it did fire while
the run-up was confined to one area: `test6_crouchjump` kept a 290 design speed the profile could not
reach. Extending the run-up backwards along the route fixed the cause rather than the symptom — the
same launch now re-speeds cleanly to 253 — so a kept design speed remains the signal that a launch is
asserting a speed the approach cannot deliver.

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

### Landing on the next run-up
When the barrier before an air move is *another* air move, the run-up node could be placed behind the
point where the bot lands. `RunupDistance` walks back through area geometry, and a previous landing is
not in `m_pendingGates` to stop it — that list holds plain-walk gates only, because every barrier
flushes the funnel. On `test54_window` at `--plan-speed 270` the node sat 14 units *behind* the
landing:

```
wp[ 3] landing  (582.0, 759.4, -1475.4) area 2
wp[ 4] corner   (585.6, 773.4, -1476.0) area 2     <- run-up node, behind the landing
wp[ 5] jump     (585.2, 746.9, -1476.0) area 17 JUMP launch 265
```

The plan credited 26.5 units of approach. The bot lands at wp[3] already moving at the previous launch
speed *toward* wp[5], so it cannot use them: it delivered 200 of 265, aborted, repathed and fell in
the pit.

**`Respeed` cannot see this.** `IsGap` covers launch, teleport and ladder waypoints but not landings,
so the node is not `Suppressed` — the ~170° reversal is arc-fitted, collapses to a pivot cap around
53, and `ProfileLaunchReach` then integrates forward over the whole 26.5 units and saturates at
`vExit`. The profile reads 265 of 265 and reports `0 re-speeded`, because it credits the reversal leg
as travel.

Two things fix it, and the first is what makes the second measurable.

**The offset is clamped at the anchor, but only across an air move.** `m_anchor` is where the bot
enters the current walk stretch, and after an air move that is the landing, so the run-up cannot be
placed before it. The clamp is a straight-line projection onto the launch heading, which is only valid
when the bot arrives *on* that heading — hence `m_anchorFromAirmove`. Applying it at every anchor
breaks curved approaches: on `test6_crouchjump` the route runs south from the spawn and then launches
north-east, so the projection of the start point is −194 and a legitimate run-up disappears.

**`PlanWalker::Retract` then buys the distance back.** For each adjacent launch pair shaped
`[launch][landing][run-up][launch]`, if the later launch's `needRunup` exceeds the `haveRunup` the
clamp left it, the *earlier* launch speed is cut by the same one-shot ratio the `Cross` rescale uses,
so its landing moves back along its own flight line. Both quantities are recorded on `PendingLaunch`
at `Cross` time, so the detection needs no profile: `needRunup` is `AccelDistance(0, launchSpeed)`
plus one lead, which is entry-speed independent and therefore has no feedback loop.

That last point is why the first attempt under-delivered. Sizing the pull from
`AccelDistance(reached, design)` looks right but is not: pulling the earlier launch back also lowers
the speed the bot carries across the gap into the approach, so more runway arrives with a slower
entry and the one-shot lands short.

**The run-up node moves with the landing.** `ProfileLaunchReach` measures only the final segment —
run-up node to launch — so room opened up *before* the node is invisible to it. `Retract` re-places
the node at `needRunup` back along the launch heading, re-floored in its own area and rejected if it
leaves that area's column. Landing and node move back by the same distance along the launch heading,
so their relative position is preserved and no new corner appears.

Guards, each keeping the design speed and counting `airmovesRetractKept`: a degenerate flight length,
an alignment below `kPlanAirmoveRetractAlign` (0.5) between the earlier flight and the later launch
heading — beyond 60° the pull is mostly lateral and the required distance blows up — and a
`JumpLandsIn` that lands neither in the earlier destination area nor in a walkable neighbour of it —
see below. Nothing is ever truncated.

`Retract` runs between `Finish` and `Respeed`, and writes `PendingLaunch::designSpeed` as well as the
waypoint. Without that write `Respeed` compares its reach against the *original* design speed and
raises the launch straight back up; with it, `Respeed` stays downward-only relative to the retracted
value and mops up whatever the pull could not reach.

#### Landing in a walk-connected neighbour
Two things stopped `Retract` from firing at all on the shape it was written for.

**The adjacency gate assumed a run-up node.** It required `later.index == prev.index + 3`, the
`[launch][landing][run-up][launch]` layout. But when the anchor clamp leaves `runupBack` under
`kPlanAirmoveRunupEps` no run-up node is pushed, the pair is `[launch][landing][launch]` and the gap
is 2 — exactly the worst case, skipped. Both shapes are accepted now, and the node-moving tail runs
only for the longer one. No waypoint is inserted for the short one: with no node the landing waypoint
itself starts the approach segment, which is the segment `ProfileLaunchReach` measures, so the pull
lengthens it directly.

**The pulled landing may leave the destination area.** Pulling the landing back is the whole point,
and on a thin destination it lands short of it. `test54_window`'s area 3 is a 16-unit lip; pulling the
`1→3` landing the 39 units the `3→17` launch needs puts it in area 2, and the plain `JumpLandsIn`
refuses. So a failed `JumpLandsIn` now gets a second look: `AreaContaining` finds where the mover
actually came to rest, and the landing is accepted there when the area is not the destination, is not
`crouch`, and `LegIsWalkable` confirms the bot can walk the straight leg from that point to the
launch.

That leg check is the load-bearing guard, not a formality. The landing moves along the *earlier
flight line* while the run-up runs along the *launch heading*, and `kPlanAirmoveRetractAlign` only
holds those within 60° of each other, so the landing can drift sideways off walkable ground while
still passing every other test. `LegIsWalkable` validates the leg the bot will actually walk.

**`ReachBack` is the walk-back the area graph knows, and it is not `RunupDistance`.** `RunupDistance`
chains areas off `m_pendingGates`, and that is right for `Cross`: the funnel's gate trail is exactly
the ground the route came over, and walking a run-up back past a barrier the route crossed — a door,
the route's own start — is not something the funnel can express. `Retract` has neither problem. It
runs after `Finish` on a settled route and only asks whether a straight leg is walkable, so it reads
walk adjacency directly: `PlanRoute` builds a per-area predecessor list from the `CONN_WALK`
connections and hands it to every `PlanWalker`. `ReachBack` keeps every guard `RunupDistance` uses —
`ClampTravelInsideArea` inside each convex area, the next area's column must contain the seam point,
`FloorsMeetAt` within `kPlanRunupSeamStep`, no crouch areas — plus a visited set, because the area
graph has cycles and a gate trail does not.

**The follower does not care which area the landing is in.** `AirmoveLandingMiss` measures against the
landing *waypoint*, and `haveMiss` is therefore always true on a planned air move, so the
`landedId != airmoveTargetArea` fallback never runs and the connection is never penalized for landing
in the neighbour. The launch waypoint keeps the connection's own destination in `targetAreaId`, which
is why the log reads `landed ... area 2 (wanted 3) miss 1.9 ... penalty 0.0`.

On `test54_window` the `1→3` launch retracts 142 → 74 and lands in area 2 at `(555.2, 784.9)`. The
`3→17` approach goes from 0.4 units to 39.0, `1 re-speed kept design` becomes `0`, and the launch
the bot delivers goes from 80% of its 270 design to 100%. It is the only map in the suite whose plan
moves — every other route is byte-identical, at both `--plan-speed` settings.

### Edge friction on the approach
Every run-up ends at a ledge — that is what an air move is — and GoldSrc charges for that. `PM_Friction`
traces `kEdgeProbeAhead` (16 units) ahead along the velocity and `kEdgeProbeDrop` (34) down; when that
trace comes back clear it multiplies ground friction by `sv_edgefriction`, normally 2. So the last 16
units before *any* launch run at friction 8, not 4.

The follower always knew this — `BotDrivePhysics` doubles `groundFriction` when `AtFrictionEdge` says
so — but doubling it there only corrects the throttle the bot asks for, not the physics the engine
applies. The planner did not know it at all: `MakePlanTraces` copies plain `Bot_Physics`, and both
`BuildPathProfile` and `PlanWalker::Cross` used `groundFriction * friction` neat. The plan therefore
sized every launch approach with half the friction the run-up actually sees.

It only bites when the run-up is short, because the shortfall is the whole cost of the last 16 units.
At `accel = 2700`, `stopSpeed = 100`, reaching 236 u/s from a standstill takes 13.7 units at friction 4
and **21.5** at friction 8. On `crossfire` the bot launches west out of area 32 — a box only 36 units
wide in x — so the run-up is 14 to 23 units, entirely inside the edge zone:

```
airmove aborted at (255.7,961.6,-1468.0) plan speed 236 actual 189 (80% of design) conn 1430
airmove aborted at (242.3,961.5,-1468.0) plan speed 237 actual 198 (84% of design) conn  866
```

14 units at friction 8 delivers ~198. The plan said 236, the follower demands 95%, and the launch
aborts. `Respeed` cannot save it because it sets `launchSpeed` to exactly the reach it measured, with
no margin, using the same friction that produced the wrong number.

`PlanWalker::LaunchAtEdge` runs the engine's probe from the launch floor along the launch heading,
with the hull the bot has on the run-up (the launch area's `crouch` flag). It is evaluated once per
launch in `Cross` and cached on `PendingLaunch::atEdge`, because both the run-up sizing and `Respeed`
need it and neither should re-trace. Probing from the launch point rather than along the approach is
the right read: it is the closest point to the drop, so a clear probe there means the whole final 16
units are inside the zone.

**The two users of that flag treat it differently on purpose.** `Cross` doubles the friction for the
whole of `needed`, past 16 units where the real friction is back to normal. That is deliberately
pessimistic, and it is safe because `needed` can only ever *lengthen* `runupBack` — `RunupDistance`
and the anchor clamp then cut it back to whatever the geometry actually offers. It never asserts a
speed. `ProfileLaunchReach` does assert one, so it integrates exactly: `AccelForward` over the plain
stretch, then `AccelForward` again over the edge stretch at doubled friction. Composing two forward
integrals needs no new solver and no inverse.

The edge stretch is `min(usable, edgeZone - lead)`, not `min(usable, edgeZone)`. The bot commits to the
jump one `AirmoveLaunchLead` *before* the waypoint, so the ground it covers inside the zone is shorter
than the zone by exactly that lead. Dropping the term over-charges every launch by a frame of travel.

Sweeping this changed 5 of 45 maps at each plan speed, and launch speeds moved in **both** directions.
A longer `needed` pushes the run-up node further back, and a longer approach can support a faster
launch even at doubled friction: `test6_crouchjump` goes 235 → 258 on its 2→x jump while
`test38_islands` goes 282 → 278 on its 4→x. The point is not that launches got slower, it is that the
number the plan asserts is now the number the ground delivers.

**Still optimistic:** `BuildPathProfile`'s backward braking and forward reachable passes carry one
global `profile.friction`, so `vReachEntry` — the speed handed to `ProfileLaunchReach` at the start of
the approach — is still computed without edge friction. That is second order here, because it feeds the
long haul rather than the last 16 units, and making the profile carry per-segment friction is a much
larger change than this one.

- `BuildAreaGates(PlanArea&)` - fills edgeGates from boundPlanes
- `BuildConnectionGates(areas, conns)` - builds shared face gates
- `ConnectionCrossing(areas, conn)` - crossing point, still stored on the graph edge
- `PlanWalker` - the entry point; `NavPlan::PlanRoute` drives it

## nav_format.h

### Nav File Format (version 21)
- Magic: "CBNV"
- Header (128 B): magic, version, mapName[64], bspChecksum, flags, counts (areas, conns, areaConnIndex,
  areaPlanes, detours, detourPoints), designMaxSpeed, vantageCount, vantageSpacing, maxExposure,
  waterVolumeCount, campCount
- AreaRecord: id, mins[3], maxs[3], centroid[3], areaFlags, connOffset, connCount, planeOffset, planeCount, activation, pad0[3], openSeconds, detourOffset, detourCount, waterVolume, pad1[3], submergeZ, driftVelocity[3]
- ConnectionRecord: areaA, areaB, connType, connFlags, gateValid, pad, costSeconds, travelSeconds, gateA[2][3], gateB[2][3], gateAYaw[2], gateASpeed[2], exitSpeed, exitAngle, jumpBackDist, fallDamage, flightTime, launchPos[3], landingPos[3], rideSeconds, rideSecondsMax, rideWaitSeconds, gravity, waterVolume, pad2[3], submergeZ, drift
- `gravity` is the `pev->gravity` multiplier the air move was sampled at (1.0 for every other
  connection type). A map with `trigger_gravity` carries one air move set per distinct multiplier, and
  the runtime uses only the set matching the bot's current gravity — see "Gravity" below
- `drift` is a `DriftTerm`: length, along, perp, alongMin, alongMax. Speed-independent by design;
  see "drift.h: direction-aware cost" above
- `driftVelocity` is the area's conveyor plus push-field velocity, used by the speed profile
- NavWatchDir (16 B): yaw, pitch, spread, amount
- NavVantageRecord (80 B): pos[3], exposure, watch[4]. It used to carry a `focus[4]` block; camping is
  now its own section and the field is gone
- NavCampRecord (64 B): pos[3], yaw, pitch, score, stand[4], crouch[4], areaId, flags. `pos` is the nav
  floor point, which is a ducking player's origin, so the standing eye is +46 and the crouching eye +12.
  `CAMP_NO_STAND` means standing there is impossible and `stand[]` is all zero
- ActivationDetourRecord: fromAreaId, pointOffset, pointCount, actIndex, waitSeconds, detourSeconds, actKind, flags, pad[2]
- DetourPointRecord: pos[3], areaId, clearance, flags, pad[3]

Section order on disk is header, areas, conns, areaConnIndex, areaPlanes, detours, detourPoints, vantages,
water volumes, camp spots. The detour
offsets are `uint32_t`; `connOffset`/`planeOffset` remain `uint16_t` into the global arrays and throw past
65535. Each record has explicit padding and a `static_assert` on its size, because they are written raw.

`nav_loader.cpp` rejects a version mismatch outright, so bumping forces regeneration rather than silently
loading a stale mesh. Area planes are the exported `boundPlanes`, which for a prism area is its footprint hull
plus a z-slab rather than its true faces.

### Enums
- AreaFlags: CROUCH(1), LADDER_TOP(2), WATER(4), DOOR(128), BREAKABLE(256), ELEVATOR(512)
- ConnType: WALK(0), TELEPORT(1), AIRMOVE(2), LADDER(3), ELEVATOR(4) - a ladder reuses the air-move fields
  rather than adding its own: launchPos is the mount, landingPos the dismount, exitAngle the climbing yaw,
  exitSpeed the climb speed, and flightTime/fallDamage belong to the dismount hop. An elevator ride uses
  launchPos as the boarding point and landingPos as the arrival point, and carries its own rideSeconds /
  rideWaitSeconds; rideSecondsMax equals rideSeconds today and is the hook for a swept-interval sampler
- ConnFlags: ONE_WAY(1), NEEDS_CROUCH(2), JUMP(4), WATERJUMP(8) - jump means left ground under impulse, not derivable from jumpBackDist. There is no reverse one-way bit: a drift field blocks a direction only for a given max speed, so that answer is computed per search by `NavDrift::EvalDrift` and never stored
- DoorActivation: TOUCH(0), BUTTON(1), TRIGGER(2)
- GateValidFlags: VALID_A(1), VALID_B(2), CLAMPED_A(4), CLAMPED_B(8)

## pm_sim.h

### PmPhysics
- `accelerate`, `groundFriction`, `stopSpeed` - sv_accelerate, sv_friction and sv_stopspeed. `PmSimulator` never reads them; they are carried for `BuildPathProfile` and `NavFollow::SolveDrive`, which are where ground physics is actually modelled.
- `edgeFriction` - sv_edgefriction. Not applied here: `BotDrivePhysics` folds it into `groundFriction` for the frames the bot is within a hull's reach of a drop, so `SolveDrive` sees one already-effective friction. See `NavFollow::SolveDrive`.
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

## Trigger Forces at Runtime

### Gravity multiplier in the nav format

The nav format stores a `gravityMultiplier` on each connection (`Connection::gravityMultiplier` in nav_format.h). This is the `pev->gravity` multiplier this air move was sampled at. For all other connection types (walk, ladder, elevator), it is 1.0.

Base velocity contributed by trigger_push fields overlapping the player origin is stored in units/sec.

### Planning with gravity multipliers

During planning, `gravityMultiplier` filters air moves the same way `PlanWalker::Cross` does, so goal selection and route search agree about which jumps exist. The filter is applied at the Dijkstra stage when selecting goals.

When the bot is at a different gravity than an air move was sampled for, that air move simply does not exist in the planner's view of the graph. Only the air moves matching the bot's current gravity are available for planning.

### Physics simulation with gravity

In `pm_sim.cpp`, `PM_AddCorrectGravity` folds the vertical push component into velocity as an acceleration and clears it from the base velocity, so only the horizontal remainder wraps `FlyMove` below.

The horizontal base velocity wraps the move the way `PM_AirMove` and `PM_WalkMove` do: it is added before the clip-and-slide loop, then pulled back out once the ground check has seen it. This ensures consistent handling across all movement types.