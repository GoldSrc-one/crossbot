# Mesh and Path Planning

The mesh the bots run on — areas, connections, lookups, the loader, the vantage field — and how a route
over it is built: the A\* search, its cost model, the walker that string-pulls and validates air moves
*during* the search, the speed profile, the air-move simulator and its cache. How the bot follows the
result is `doc/movement.md`; how navgen builds the mesh is `doc/navgen.md`.

## The mesh

### NavMesh, Area, Connection (`nav_mesh.h`)

- `NavMesh::planAreas` / `planConns` are the planner's view of the graph, built once at load. Rebuilding
  the area gates per repath would repeat the same plane intersections every time.
- `Area::planes` — inward-facing bound planes; for a prism area, the footprint's vertical planes plus a
  z-slab. Activation data lives only on `planAreas`, loaded straight from the area records.
- `Connection::gateA` / `gateB` — the funnel gates on each side, baked by navgen with the player radius
  already applied: inset at both ends and pushed up to that far into the owning area (`GateFromEdge`). A
  zero-length gate means the opening was too narrow to inset; its midpoint is the only way through.
  `gateValid` says which sides carry one; a teleport carries neither.

### Loading (`nav_loader.cpp`)

`LoadNavFile`:
- checks the magic `CBNV` and `kNavVersion` (`nav_format.h`, currently 26);
- reads area records, connection records, area-connection indices, area planes, activation detours and
  detour points, then the vantage section;
- verifies the BSP checksum against the live map;
- builds the `PlanArea` view (bound planes from the area planes) and the `PlanConnection` view (gates,
  launch params);
- converts the `NavVantageRecord`s into a `VantageField`, calls `BuildVantageGrid` once, and resolves the
  per-connection exposure and open distance (see Carefulness and Range bias below);
- falls back to `kNavDefaultDesignSpeed` (320) when the header's `designMaxSpeed` is unset.

The version check is a hard equality, not a floor: every record is `fread` straight into its struct, so
any layout change is a version bump and a regeneration of every `.nav`, and an older file is refused
rather than read with a shifted layout.

### Area lookups (`nav_query.cpp`)

Two lookups answer two different questions, and the names say which:

| Function | Question | Can fail? |
|---|---|---|
| `FindAreaExact` | which area *contains* this point | yes, returns null |
| `FindAreaNearest` | which area is *nearest* | essentially never |

Picking the wrong one is a recurring bug. A strict lookup where a forgiving one belongs makes a bot on a lip
forget where it is; a forgiving one where a strict one belongs scores a maneuver against an area the bot
never reached (`doc/movement.md`, the ladder, air-move and elevator verdicts).

- **`AreaContainsPoint`** — the predicate both are built from: AABB test with `kAabbEpsilon` (4), then the
  exact plane test with `kPlaneEpsilon` (2). Public because asking about one known area is not the same
  question as sweeping the mesh. Areas abut on shared planes, so a seam point is inside both and
  `FindAreaExact` returns whichever has the lower id. `FinishElevatorRide` asks its ride target first for
  exactly that reason: the stop slab and the shaft below it share a plane, and the sweep always answers
  "shaft".
- **`FindAreaExact`** — `AreaContainsPoint` over every area, first hit wins. The test is in mesh space:
  areas are built in hull 3, so their z range is duck-origin z, and a caller holding a player's
  `v.origin` may need `BotNavZ` first. An ordinary area spans floor to ceiling and swallows the 18-unit
  offset; it matters where areas are thin, which in practice means elevator stop slabs.
- **`FindAreaNearest`** — `FindAreaExact`, then the nearest centroid among areas whose AABB holds the
  point, then the global nearest centroid.
- **`FindAreaById`** — direct index, linear search as fallback.

## Vantage field (`nav_vantage.cpp`)

The runtime half of navgen's vantage sampling (`doc/navgen.md`, *vantage_sampler.cpp*). Each reduced
vantage stores up to four `WatchDir{yaw, pitch, spread, amount}` — the cones through which everything
with line of sight to that spot arrives — plus `exposure`. The runtime turns a bot's eye position into a
blended signature it can steer by; `bot_sense.cpp` refreshes one per bot every 0.1 s.

**The field has its own grid** because nothing else in `src/nav/` indexes space (`FindAreaExact` is a
linear scan) and a blend runs per bot at 10 Hz. It is a CSR uniform grid: count pass, prefix sum, fill
pass, one allocation each for `cellStart` and `cellItems`. Cell size is the header's `vantageSpacing`,
floored at 32 and doubled until the cell count is within `64 * vantages + 4096`, so a long thin map does
not allocate a mostly-empty lattice. `FindNearest` walks cube-shell rings outward and stops once the
fourth-best distance is inside the searched ring's inscribed radius, `ring * cell`.

**Scalars interpolate, directions merge.**
- `exposure` is IDW over the four nearest with weight `1 / (d^2 + 1)` — the same weight and epsilon navgen's
  decimation scored itself against, so the runtime reproduces the interpolation the decimation assumed.
- Watch directions are a variable-length list and have no distance, so they cannot be averaged. They are
  merged with the same greedy peak-and-absorb `ExtractWatchDirs` runs: take the heaviest entry, mean-shift
  once over everything within 45°, absorb those with summed weight and weighted-RMS spread, repeat up to
  four times. navgen runs it over a 512-bin histogram; this runs over at most 16 sparse entries, each
  entering at `idw * amount`.
- Merged directions under 5% of the kept weight (navgen's `minClusterFraction`) are dropped. Otherwise a
  far neighbour survives as a fourth watch dir with `amount 0.000`, and `ApplyVantageLook`'s
  nearest-to-path search would snap the head onto it. Amounts are renormalised over what survives, so
  `watch[0].amount` is always the largest and safe as a denominator.

**Round-tripping is the correctness test.** Blended at a stored vantage's exact position, the distance-0
neighbour dominates and the result must be that vantage's own record. `crossbot_vantage_status <x y z>`
checks this; anything else means the loader, the grid or the merge is wrong.

## Planning a route

### PathSearch and PlanPath (`nav_path.cpp`)

`Nav::PathSearch` is the NavMesh-shaped wrapper over `NavPlan::RouteSearch`: its constructor builds the
connection graph (`BuildGraph`), adds the bot's hurt costs, copies the trace context and the water surface
heights, and captures the detour team mask, so the search owns everything it reads except the mesh, which is
immutable while a map runs. `Step(deadline)` relaxes edges until the deadline passes and returns true when
the search is over; `Finish` reconstructs the plan. Both set the captured mask first
(`g_detourTeamMask` is `thread_local`). `PlanPath` is construct, `Step` with no deadline, `Finish`, and is
what navgen and the one-shot callers use. Searching and string-pulling are one pass, so there is no
graph-only search to call separately.

**Plans run on one worker thread (`plan_worker.cpp`).** `Repath` builds the `PathSearch` on the game
thread (the graph build reads live water and runtime penalties), wraps it in a `Nav::PlanJob` held by
`CBot::search`, and hands it to `SubmitPlan`; later calls poll `PlanJob::done`. The worker steps queued jobs
round-robin in 4 ms slices (`kPlanSlice`), so one 300-sim plan cannot hold up everyone else's, and on the
last slice runs `Finish` and `BuildPathProfile` too, leaving the game thread only to copy the result. The
queue holds `weak_ptr`s: a bot that drops its job (goal or mesh changed, `ResetPath`, respawn) cancels it, and
the worker frees it after at most the slice in progress. While it is pending `TryRepath` keeps the bot idling
in place and facing its look target. What the job snapshots at construction — graph costs, runtime
penalties, hurt costs, runtime teleports, the world snapshot, the team mask — stays as it was while it runs.
`FlushPlans` empties the queue and waits out the running slice; `BotManager::ServerActivate` and
`ServerDeactivate` call it before anything map-scoped (bots, mesh, airmove cache) is torn down, and
`Meta_Detach` joins the thread through `StopPlanWorker`.

The worker may only call engine-free code: every trace comes from the world snapshot ("The plan-time
world"), and nothing on the planner path logs. `kPlanBudgetMs` (6 ms) still caps what one frame spends
*starting* plans (`PlanBudgetSpent`), which is now just `MakePlanTraces` plus the graph build.

Measured 2026-10-20, 16 bots, 60 s, `-DCROSSBOT_PERF=ON`, interleaved, arc traces on, worker vs the same
search in 4 ms slices on the game thread:

| map | `rp_plan` on the game thread, ms/frame | frames ≥ 8 ms | p99 | plan latency avg / max |
|---|---|---|---|---|
| xen_dm | 0.22–0.25 → 0.08 | 119/106 → 39/41 | 9.8/9.5 → 6.6/6.9 ms | 11/10 → 13/15 ms, 138/105 → 60/61 ms |
| dod_avalanche | 0.27–0.30 → 0.08–0.11 | 360/348 → 181/127 | 12.8/13.7 → 12.1/10.0 ms | 19/26 → 17/20 ms, 164/358 → 75/76 ms |

Latency is submit to `done`. The average barely moves (plans are short now, "The plan-time world"); what the
worker removes is the tail, and the planning cost on the game thread.

**The plan ends at the goal clamped into its area.** `Repath` resolves the requested point to an area and
clamps it into that area's bounds; `PlanWalker::Finish` emits the clamped point as the `goal` waypoint. A
requested point outside the mesh therefore gives a plan that legitimately stops short, and the bot
reports "reached goal" there. That is not a planner bug, but to a caller comparing against the point it
asked for it looks exactly like one — read the plan's last waypoint before believing a "stopped short"
report.

### NavPlan::RouteSearch and PlanRoute (`plan_search.cpp`)

`RouteSearch` holds the A\* state (open list, per-area `SearchNode`s, walk adjacency) between `Step`
calls; `PlanRoute` runs it to completion in one call, so a time-sliced search and a one-shot search
return the same route. **The deadline is checked before every edge, not every expansion**, and a step can
stop halfway through an area's edges (`m_current`, `m_edge`) and resume there. An area on dod_avalanche
carries dozens of air-move edges, each a full simulation: with a per-expansion check one `Step` ran to
29 ms against a 6 ms slice. A step always relaxes at least one edge, so one simulation is the overrun's
floor. On the worker this is what keeps the round-robin fair.

A\* over the area graph where **each search node carries a `PlanWalker`** — the funnel state, the waypoints
emitted so far and the running cost. Expanding an edge copies the parent's walker and calls `Cross`, so the
string pull, the air-move simulation and the door/teleport waypoints all happen during the search.

Why not search first and string-pull after: A\* alone only knows a connection exists, while the pull
re-derives each launch point from the actual funnel, and re-simulating from there can miss the
destination. Two passes disagree, and the only repair is to truncate and re-search with the edge
blocked, repeatedly. With one funnel — the one the search built — the disagreement cannot arise.

What follows from that:
- **Cost is real.** `g` is the string-pulled travel time to the node's frontier (`PlanWalker::Time()`,
  pulled distance over `kNavRunSpeed` = 250) plus each connection's `costSeconds`. A centroid-to-centroid
  model charges every route for visiting every centroid and overprices routes through many small areas:
  on test50_stairs it picks a crouch-jump where walking up two areas is cheaper.
- **Recovery is free.** An air move whose launch cannot be simulated makes `Cross` return false, so the
  edge does not relax and the search continues. Nothing is truncated or retried.
  `PlannedRoute::airmovesRejected` counts these.
- **`h` is sharp.** It is the straight line from the node's funnel frontier to the goal *point*, which is
  why a crossfire search expands 165 of 503 areas. `g` and `h` share a reference point (`Frontier()`), so
  they are consistent. They are not strictly admissible — the real route need not cross at the gate
  midpoint — so a route can come back slightly suboptimal, but never missing.
- **The goal node's walker is the plan.** Reconstruction calls `Finish` on it and takes its waypoints;
  there is no replay pass, so the plan cannot drift from what the search costed. `Retract`, `Respeed` and
  `Relax` then run on it, in that order.

Two properties to know before changing it:
- **Nodes are keyed by area, not by `(area, incoming connection)`.** Two routes into one area have different
  funnels; the cheaper wins and the other is discarded. This is the standard Theta\*/TA\* approximation;
  keying by the pair would be exact but multiplies the node count by the average degree.
- **Reopening is allowed.** Funnel-derived `g` is path-dependent, so the closed-set argument does not hold
  and a node that receives a strictly better `g` is re-pushed. `g` is clamped to never fall below its
  parent's (`std::max(next.Time(), state[current].g)`), which keeps the parent chain acyclic; the
  reconstruction loop still carries a hop guard in case that reasoning breaks.

**`g` stays true seconds; `score` carries the weighting.** `SearchNode` carries `score` beside `g`, and only
`score` drives ordering, the dominance test and the open-list key:

```
score = parent.score + (g - parent.g) * (1 - careful + careful * exposure)
                                      * (1 - weight  + weight  * mismatch)
```

Nothing touches the walker's lengths or `m_extraTime`. The ETA is built by `BuildPathProfile` from waypoint
geometry, never from the search cost, so a careful route reports the longer time it will actually take.

#### Cost of searching

Every air-move edge relaxed runs a full `PmSim` re-simulation. On crossfire (503 areas, 773 walk and 1377
air-move connections) one `PlanRoute` expands 165 areas and runs 243 simulations, ~0.1 s in navgen against
BSP-hull traces. `--dump-path` prints both numbers on its `debug search:` line; `Repath` prints them with
the wall-clock time.

One early-out does most of the work and must not be deleted by accident:

```cpp
if (state[current].g + kPlanCostEpsilon >= state[neighbor].g)
    continue;
```

Since `g` never falls below the parent's, a parent already costing more than the neighbour's best cannot
improve it, so the edge is skipped *before* `Cross` pays for a simulation. It is exact, not an
approximation: it cuts crossfire from 402 simulations to 243 and the test maps from 207 to 144, with
byte-identical routes on all 52 maps.

Validating on *pop* instead would cut simulations to at most one per expanded node, but needs dead-edge
tracking, node resetting and reopening closed neighbours. Eager validation is correct by construction;
those are the numbers to beat if runtime cost ever justifies the trade.

#### What a plan guarantees

Every air move in a returned plan is **simulable**: the search re-simulated it from the funnel's own launch
point and watched it land in the destination area. A ladder gets no such check — navgen established its
mount and dismount against the real hulls, so the planner takes it at face value and only decides which
end it entered from.

A plan does **not** prove the bot can execute it. `PmSim` starts at the exact launch point, yaw and speed;
the follower arrives with whatever speed and heading it has, and `NavFollow::Track` derates speed off the
chain, so a bot rejoining late reaches the launch slow and undershoots.

The two failures look nothing alike in test results:
- **Planning failure** — shows in `--dump-path`: no route, a rejected connection, a plan ending somewhere
  unexpected.
- **Execution failure** — a correct plan the bot does not follow; only the runtime shows it.

A cost-model change can turn the second into a visible regression without any planning bug: a route the
follower handles badly gets cheaper, the search picks it, and pass rates drop while every plan stays valid.

### The cost model (`astar_graph.h`)

Header only; the search lives in `plan_search.cpp`. It holds `kNavRunSpeed` and the `Connection` view of an
edge: endpoints, one-way flag, `costSeconds`, crossing point.

**`costSeconds` is an additive surcharge in seconds.** Being time is what lets a door contribute the seconds
it takes to open. It already includes an air move's `flightTime` and a door area's `openSeconds`
(`ConnectionCostSeconds`, `area_merger.cpp`), so the search adds it once per edge and `PlanWalker` must not
add either again — the walker only measures string-pulled distance.

Consequences:
- **A plain walk connection costs zero.** Correct for the planner, which measures the walk itself, and a
  trap for any cheap search without a walker: summing `costSeconds` prices every walk at 0, so the
  "cheapest" area is whichever the heap settles first. That is what `travelSeconds` is for.
- **Everything fed into the cost must be seconds** — `runtimePenalty`, the hurt term, falls, drift — or it
  will quietly outrank everything else.

### travelSeconds

A second per-connection cost, written by navgen (`ConnectionTravelSeconds`, `area_merger.cpp`), for
coarse searches that cannot afford a walker. It is the walk between the two area centroids over
`kNavRunSpeed`, funneled through the gate: the straight centroid line if it crosses the gate segment,
otherwise bent around the nearer gate endpoint. Non-walk types get only the two ground legs
(`centroidA -> pointA`, `pointB -> centroidB`); the flight, climb or ride is already in `costSeconds`.
Precomputed because it is static and the funnel correction is not worth redoing per frame.

**The planner must never add it.** `BuildGraph` does not: `PlanWalker` measures the real distance and
adding this would charge twice. It inherits the centroid model's bias — overpricing routes through many
small areas — which is acceptable for choosing a *target*, not a *route*.

### AreaCosts and AreaCostsToGoal (`nav_path.cpp`)

- **`Nav::AreaCosts(a)`** — a full Dijkstra from `a` over `travelSeconds + costSeconds + runtimePenalty`
  (plus the fall, hurt and drift terms below), filling a cost-per-area vector. It never stops early: callers
  rank many candidates against each other, and on a ~500-area map an early exit saves microseconds.
- **`Nav::AreaCostsToGoal(g)`** — the same search backwards: every area's cost *to* `g`. One reverse pass
  turns "price many candidate positions against one goal" into array lookups, which is what makes the
  opportunist's 4 Hz check affordable (`doc/movement.md`, `bot_opportunist.cpp`).

Both share `AreaCostsImpl`, which takes the direction as a flag. Its values are plan-seconds:
centroid-anchored and priced at `kNavRunSpeed`, not at the bot's speed. Mixing them with a real duration
without converting is the trap the opportunist section describes.

They honour the planner's walker-free rules, and every edge test flips with the direction:
- **`CONN_FLAG_ONE_WAY`** means legal only as `areaA -> areaB`. The forward pass requires `c.areaA ==
  current`, the reverse pass `c.areaA == neighbor` — the reverse walk looks for the forward edge that would
  have arrived.
- **Static activation gating** (`GateAllowsCrossing`, which takes an ordered pair because reversing the
  search does not reverse the door). Entering an `activation != ACT_NONE` area by anything but a walk or a
  ride is refused, and so is entering it from a neighbour with no `PlanDetour`, a `blocked` one, or one
  with no points — the same triple test `PlanWalker::Cross` and `PushActivationDetour` apply. Without it
  the Dijkstra prices areas the planner always refuses: crossfire's secret room (navgen logs
  `detour area 21 <- 23 : BLOCKED` because the `trigger_multiple` that opens `secret_door` is inside the
  room) read its 12 items as `11.8s away` while every `Repath` to them failed. Funnel-dependent refusals,
  such as an air move needing a run-up this approach cannot give, stay outside: answering them needs the
  walker, which is why goal selection's first phase is a plain Dijkstra (`doc/goals.md`, "Selection is
  two searches").
- The team mask, gravity filter and drift directions, covered in their own sections.

**`careful` and `RangeBias` turn seconds into scores.** `AreaCosts` takes both and applies them as
`PlanPath` does, to the edge cost as a whole; the range term's direction is `from -> to`, so the reverse
sweep picks the other stored open distance. Above zero the values are scores, not seconds, which splits
the callers:
- goal selection (`BotGoal::Select`, `bot_goal.cpp`) passes the bot's carefulness and range bias — it only
  ranks candidates against each other;
- `BotOppo::RebuildCosts` must leave both at their zero defaults and add no hurt term, because `TryJump`
  subtracts two of these values and compares the difference with a real simulated flight time.

Reachability is unaffected either way: every factor is strictly positive.

### Carefulness: routing away from exposure

`PlanPath` takes a `careful` factor in `[0, 1]` that weights each edge's time by the vantage exposure at
its crossing:

```
edge score = time * (1 - careful) + time * exposure * careful
```

- `careful = 0` is the plain fastest route, exactly.
- `careful = 1` drops time and minimises exposure alone. **Deliberately uncapped:** a bot with nothing to
  fight with should take the covered way however long it is.
- `Repath` feeds it `Bot_Carefulness`: `(1 - combat readiness)` scaled by `crossbot_careful`
  (`doc/goals.md`, "Carefulness").

**Exposure is cached per connection at load.** `nav_loader.cpp` resolves `Connection::vantageExposure` once
per map: `ConnectionCrossing` for the point, lifted to eye height (`kVantageMeanEyeLift` 29, or
`kVantageDuckEyeLift` 12 when either endpoint is a crouch area — areas are in duck-origin space, so the raw
crossing z is 29 below where the probes were sampled), then `BlendVantage` and a divide by
`field.maxExposure`, clamped to `[0, 1]`. About 2200 blends on crossfire, none allocating.

Two traps in that pass:
- `BlendVantage`'s `out.valid` means `watchCount > 0`, so a spot nothing looks at returns false with a
  perfectly good exposure. Gate on `blend.neighbors`.
- An unresolvable edge falls back to **1.0, not 0**. Unknown must collapse the factor to 1 (careful off),
  never make the edge free. A map built with `--no-vantage` therefore plans identically at every `careful`.

**The heuristic shrinks with it.** The factor can go below 1, so `h` is scaled by
`1 - careful + careful * minExposure` (`NavMesh::minConnExposure`, resolved at load) — the lowest factor any
edge can carry, which keeps `h` admissible against the weighted edges. The price is a weaker heuristic as
`careful` rises: one crossfire leg goes from 45 expansions / 7 ms at `careful 0` to 107 / 21 ms at
`careful 1`. Bounded, but it is the number to watch.

### Range bias: routing to ground the weapon can use

`PlanPath` also takes a `NavPlan::RangeBias`; it multiplies with carefulness:

```
edge score = time * (1 - careful + careful * exposure)
                  * (1 - weight  + weight  * mismatch)
```

- **`RangeBias`** (`plan_search.h`) holds a weight, two distance thresholds and one bit for "this weapon
  wants long range". `Weight()` clamps to `[0, 1]` in the struct rather than at call sites, because above 1
  the multiplier goes negative — a negative edge cost in both searches.
- **`Mismatch`** is one lerp: `t` ramps 0 to 1 as the edge's open distance goes from `shortDist` to
  `longDist`; a close-range weapon takes `t`, a long-range one `1 - t`. Zero is ideal ground, one the worst.
  At `weight = 0.5` the multiplier spans `[0.5, 1]`, so a route on the right ground may take twice as long
  and still win.
- **`weight = 0`** (the default-constructed `RangeBias`) is the unbiased planner. `Repath` feeds
  `Bot_RangeBias`: combat readiness scaled by `kRangePref`, zero for a weapon with no opinion
  (`doc/goals.md`, "Range bias"). It mirrors carefulness: an unready bot avoids exposure, a ready one picks
  its terrain.

**Open distance is cached per connection, per direction.** The same load pass reuses its `VantageBlend` to
fill `Connection::openDistAB` and `openDistBA` with `VantageOpenDistance(blend, dir)`, `dir` being the XY
direction between the two centroids and its negation — two dot-product sums per connection. The fallback
is **0, not 1**: no vantage data reads as the shortest range. That is honest (nothing is visible from
there), and it gives every edge of a `--no-vantage` map the same distance, so the bias cancels.

**The heuristic shrinks again.** `hScale` is also multiplied by `RangeBias::BestScale()`, `1 - weight`, the
cheapest multiplier any edge can earn since `Mismatch >= 0`. Same argument and same cost as above.

**A weapon swap invalidates the route.** `bot.plannedRangeClass` records the range class the route was
planned for, and the nav update forces a repath when the live weapon's class differs — the class, not the
weapon id, so swapping between two close-range guns costs nothing. It is stamped only once a plan stands:
a failed search must not mark the route as planned, or the retry loses its trigger.

### Revenge: routing out of a watcher's sight

Carefulness and range bias *weight* edges; the revenge route (`doc/goals.md`, "Revenge") *excludes* them.
A bot approaching where its killer was must not be seen from where that enemy stood, so the search drops
every walk connection the enemy could watch rather than pricing it up — "we just need his back", not a
cheaper-but-covered compromise.

The excluded set is a `std::vector<bool>` over connections, built **on the plan worker thread, time-sliced**
like the search itself. `Repath` passes a `RevengeLos` (the killer's eye, forward, `cosFov`, radius) into
`RouteSearch` only when the leg's `Stop::revenge` is set; `BuildLosSlice` then fills the bitset inside
`Step`, before the A\* loop, yielding at the slice deadline exactly as edge relaxation does (it checks every
64 connections). For each `CONN_WALK` connection whose crossing (`NavAStar::Connection::crossing`, lifted 29
to eye height as the exposure sample is) is within `crossbot_revenge_los_radius` (2048) of the eye **and**
within `crossbot_revenge_fov` (120°) of the forward, one snapshot hull trace (`PlanTraceContext::standTrace`,
engine-free so it is legal on the worker — not an engine `TRACE_LINE`) decides it; a clear line marks the
connection blocked. `Relax` then skips a blocked connection, exactly as the drift `passable` test does.

- **It is a hard cut, so `h` is untouched.** Removing edges only raises real cost; the straight-line
  heuristic stays a lower bound, unlike the `careful`/`RangeBias` factors that shrink `hScale`.
- **No fallback.** If the exclusion leaves no route, that *is* the answer "there is no unseen way in", and
  the goal is dropped (`doc/goals.md`). The planner never retries without the cut.
- **The build is off the game thread, so the radius costs latency, not frame time.** It runs once per
  revenge leg. Measured on dod_avalanche (4226 walk connections, FOV 120°), worst case over spawn points and
  facings: `los_radius` 2048 traces up to ~1300 crossings (~13 ms of worker time), 1024 ~260 (~2 ms), 768
  ~115 (~0.4 ms). Because it is sliced on the worker, even the 2048 case never hitches the game thread — it
  only lengthens that one plan's latency and shares the worker fairly with other bots' plans. The radius is
  also the stealth zone (only sightlines near where the enemy stood matter), so 2048 is the default. A PVS
  pre-cull would cut the trace count further but needs the killer's PVS captured at death, since the edict
  is gone by the next life. `PlannedRoute::losTraced` / `losBlocked` ride back with the plan and
  `crossbot_revenge_debug` logs them; `crossbot_revenge_probe <x y z> <yaw>` measures the scan synchronously
  at any point for tuning, with no combat (`doc/diagnostics.md`).
- **navgen never passes a `RevengeLos`**, so `--dump-path` and offline route sweeps are unaffected, the same
  guarantee as `runtimePenalty`.

### Falls are priced per game

The `.nav` is shared by every game, and each hurts on landing differently: HL from 580 u/s, CS and DoD from
500, TFC from 650, Ricochet never (`doc/games.md`, "Fall damage follows mp_falldamage"). So navgen stores the
fall, not its price: `fallHeight` on air-move, ladder and teleport connections is the drop from apex to
landing, and the stored `costSeconds` carries no fall term (`ConnectionCostSeconds(c, false)`). A height
rather than a speed, so the file does not assume `sv_gravity`.

`PlanPath`, `AreaCosts` and `AreaCostsToGoal` take a `PmSim::FallRule` (default `kHlFall`). `FallCost` in
`BuildGraph` and `AreaCostsImpl` adds `FallPenaltySeconds` of the damage that rule gives the landing speed
`sqrt(2 · 800 · gravity · fallHeight)`: half a second per point, `1e4` once the damage reaches
`FallRule::health`. `Bot_FallDamage` fills `health` with the bot's current health, so a bot at 30 hp will not
plan a 35-point drop a healthy one takes; navgen and any caller passing a bare rule get 100. Stored costs
carry no fall, so the runtime only ever adds and `h` stays admissible.

navgen's own decisions still use HL's damage: it ranks trajectories and dismounts by it, and
`airmove_prune.cpp` and `plan_model.cpp` price connections with `ConnectionCostSeconds`' default `withFall`.
Only the file omits the term, so it moves no `--dump-path` route.

### drift.h: direction-aware cost

A `func_conveyor` or a horizontal `trigger_push` makes a connection cost different amounts in its two
directions, and can make one of them impossible. How the fields are found and areas get their
`driftVelocity` is `doc/navgen.md`, "Drift Fields"; this is what the searches do with it.

`crossbot_nav::DriftTerm` is five floats on `NavConnectionRecord` and **carries no speed**: `length` (how
much of the crossing lies in drifting areas), the length-weighted means `along` and `perp`, and the
extremes `alongMin`/`alongMax`. `NavDrift::EvalDrift(term, maxSpeed, forward)` turns it into
`{seconds, passable}` for one direction; `forward` means `areaA -> areaB`, and reversing negates `along`
and swaps which extreme is the worst case.

**It is evaluated per search, not baked**, because speed is per bot. `Bot_MaxSpeed` reads the cap the engine
settled on in `PM_Move_Post`, which under CrossPlay differs per hosted game; a 280 u/s belt is walkable
upstream at 320 and impossible at 270, and a mesh that decided in advance would send a bot onto an edge it
cannot cross. `EvalDrift` returns `{0, true}` the moment `length` is zero, so a map with no fields pays one
float compare per edge.

`seconds` is a **correction on the `kNavRunSpeed` baseline**: negative downstream, positive upstream, and
bounded by the crossing's own baseline time. The bound keeps `PlanWalker::Time()` monotonic, so the parent
clamp in `PlanRoute` never swallows a step.

Both searches consume it:
- `PlanRoute` skips an impassable direction and folds `seconds` into
  `AddCost(gc.costSeconds + drift.seconds)`, so it rides in `m_extraTime` with door waits and flight time.
  `PlanPath` passes `traces->physics.maxSpeed`, falling back to `mesh.designMaxSpeed`.
- `AreaCostsImpl` skips the same directions and adds `seconds` to its edge cost, clamped at zero. The bot's
  call sites pass `Bot_MaxSpeed(bot.pEdict)` as `maxSpeed`.

`h` still divides by `kNavRunSpeed`, so a downstream run faster than 250 u/s makes it overestimate and
optimality becomes best-effort. `PlanRoute` already reopens nodes, so this is a difference of degree from the
bias `travelSeconds` carries, not a new failure mode.

**Blocking is `passable == false`, not a connection flag.** There is no "one way back" bit, because the
answer depends on who is asking. `CONN_FLAG_ONE_WAY` means legal only as `areaA -> areaB`, and
`crossbot_nav::ConnAllowsDirection` is the shared test wherever the question is "may I leave `from` along
this connection". Sites asking something else — `airmove_prune` testing whether an edge is bidirectional,
`activation_detours` looking at entries into an area — keep their own inline check.

### Planning with gravity multipliers

A `trigger_gravity` sticks to whoever touches it until respawn, so a bot is only ever at one multiplier, and
navgen samples one air-move set per multiplier the map hands out (`doc/navgen.md`). `PlanWalker::Cross`
refuses an air move whose `conn.gravity` does not match `PlanTraceContext::gravityMultiplier`, and
`Nav::AreaCosts` applies the same filter, so goal selection and route search agree about which jumps exist.
An air move baked for another gravity simply does not exist for the bot. The live multiplier is
`pev->gravity`, read by `Bot_Physics` (`doc/movement.md`).

### Barriers

A `NAV_AREA_BARRIER` area is the inside of a wall the map removes or raises on its own: a named
`func_wall_toggle`, or a DoD explosives-only breakable (`doc/navgen.md`, "Barriers split too"). `LiveBlocks`
(`nav_path.cpp`) drops every connection with a barred area at either end, in `BuildGraph` and in the
`AreaCosts` Dijkstra, beside the live water rule. `Nav::AreaBarred` is two cases: the area is a barrier that
is closed, or its floor rests on a barrier (`barrierFloor`) that is open. Goal selection prunes by `AreaCosts`, so a goal behind a
closed barrier is unreachable before any plan is started, and becomes reachable the frame the wall goes.

`Nav::BarrierClosed` (`nav_barrier.cpp`) refreshes once per server frame. A barrier is closed while a live
entity with `pev->model` equal to its `*N` **and** `absmin`/`absmax` overlapping its baked bounds is not
`SOLID_NOT`; no such entity means open (a blown breakable is freed). The model index is the only key the BSP
lump and the live entity share, and the bounds test is what makes it safe: 35 brush models in installed maps
are shared by several entities (`hl_c5_b` `*174` by ten `func_button`s), an edict slot is reused after a
removal, and a `.ent` override can move a brush. The cached edict is re-checked every refresh and a rescan
of every edict runs at most once a second (`kRescanInterval`), so a removed brush does not cost a full scan
per frame on a map like warpath with 186 barriers.

`Nav::BarrierGeneration` counts state changes. `BotTactics` clears its per-spawn cost fields when it moves.
A bot already routed through a barrier that closes is not repathed on the spot: it stalls against the wall
and the ordinary stalled-repath path (`doc/movement.md`) plans around it or gives the goal up.

A barrier brush counts as a mover in the world snapshot (`WorldSolid::counts`), even though
`func_wall_toggle` carries `FL_WORLDBRUSH`, so an air-move sim that touches one is never cached: a result
cached while the wall stood would outlive it.

## Runtime writes to the mesh

### Runtime connection penalties

`PenalizeConnection` / `RewardConnection` / `ConnectionPenalty` (`nav.cpp`) are how the runtime learns where
movement fails. They move `Connection::runtimePenalty`, which `BuildGraph` adds to `costSeconds`: a failure
adds `kConnFailPenalty` (5 s), a success halves it and snaps it to zero under 0.1. A connection the bot
failed on gets more expensive, and the next plan routes around it if it can.

Deliberate properties:
- **Not a ban.** The edge stays in the graph because it may be the only way into an area. 5 s is far below
  the `1.0e4f` soft infinity `ConnectionCostSeconds` uses for lethal falls, so a penalized one-way drop
  still gets planned rather than stranding the bot.
- **Mesh-global and in memory.** One table shared by every bot, cleared when `Unload` frees the mesh. It is
  not in the file format: it is learned state, and navgen would overwrite it.
- **Offline planning never sees it.** It defaults to 0 and navgen builds its own graph, so `--dump-path` is
  unaffected; a route sweep diffing a change here must come out empty.
- **No exploration term.** The reward only fires on edges still being planned. Once a penalty makes an
  alternative win, nothing brings the edge back — acceptable, since an edge never planned is never paid
  for.

Attribution: `Cross` stamps `PlanWaypoint::connIndex` on the air-move launch waypoint,
`RunAirmoveApproach` latches it into `CBot::airmoveConnIndex`, and `RunAirmoveFlight` spends it on
landing. Ladder and ride runners latch their own index; walk and door waypoints carry `-1`, so a blocked
walk is charged through `Bot_PenalizeCurrentConnection`, which reads the route's connection list
(`doc/movement.md`). The teleport branch stamps `connIndex` too, and only a runtime teleporter spends it:
the TFC `pad` task penalizes its connection after 10 s on the pad. `crossbot_nav_penalties` lists the
table.

### Per-bot hurt penalties

`runtimePenalty` learns where movement fails; `Nav::HurtLog` (`nav_path.h`) learns where *this* bot gets
hurt and where its friends die. It is `CBot::hurt`, 32 entries of `{id, area, hp, time}`: `id` is a
connection, or an area when `area` is set.

- **What is charged.** `Bot_NoteHurt` runs first in `ThinkOneBot`, before the dead branch, so the frame the
  bot dies on counts. Any drop in `v.health` (clamped at 0, so overkill is not counted) goes onto
  `hurtConn`. Bullets, `trigger_hurt`, drowning and falls all count — including a planned drop that
  `FallCost` already priced, deliberately, since the landing may hurt more than the model says. Respawning
  raises health and is never damage.
- **Which connection.** `Bot_MoveNav` resolves `hurtConn` whenever the bot's area changes
  (`EnteredConnection`): the planned connection into that area when it is on `plannedAreaPath` (which
  covers air moves, ladders and rides whose mid-air areas are not adjacent to the landing); otherwise the
  first connection joining the previous area to the new one in that direction; otherwise `-1`. It runs
  every live frame, combat included, and tracks its own `hurtAreaId` rather than relying on
  `currentAreaId`. Respawn clears both: nothing led the bot into its spawn.
- **A friend's death is an area entry.** The frame a bot dies, `ThinkOneBot` hands it to
  `Bot_NoteFriendDeath` for every live bot that `Teams::Of` calls a friend. A friend whose
  `plannedAreaPath` still has the dead bot's last area (`hurtAreaId`) ahead of it adds 100 hp
  (`kFriendDeathHp`) on that area and resets its path, so the next plan weighs the kill zone. A charged
  connection alone is no deterrent here: a street is several areas wide, and the plan steps onto a
  parallel connection into the same street. Two cases are skipped:
  - The death area is the route's last area: every route to that goal ends there, so a repath would
    return the same route.
  - The friend is mid-maneuver (`Bot_ManeuverActive`): resetting drops a jump or ladder in flight. The
    entry is kept for the next plan.
  Only bot deaths are seen; nothing tracks a human's. Log: `X reroutes around area A where Y died, N hp
  there`.
- **An area entry is charged on its connections.** `HurtSeconds` adds half the entry's seconds to every
  connection in the area's `connIndices`, which lists both directions. A route through the area crosses
  two of them and pays the full charge; one that ends in it pays half.
- **Bookkeeping.** `Add` merges onto an existing entry with the same `id` and `area` and refreshes its `time`; a new connection takes a
  free slot or evicts the oldest. `Decay` forgets 1/3 hp per second (100 hp in five minutes) and frees
  entries at 0. It runs every frame and before every `Add`, so stored `hp` is always current and the
  planners need no clock — which keeps navgen clock-free.
- **Readers.** `PlanPath` from `Repath` adds `hp * 0.05` seconds to the connection's `costSeconds` after
  `BuildGraph`, so a death costs about 5 s. Every `AreaCosts`/`AreaCostsToGoal` call in `bot_goal.cpp` adds
  the same, so goal ranking avoids goals behind painful connections, and a flag where friends keep dying
  rolls lower until the entries decay. `BotOppo::RebuildCosts` must not (see
  above). Everything added is non-negative, so `h` stays admissible.

Unlike `runtimePenalty`: there is no reward, nothing is shared between bots beyond a friend's death on the
route, the log survives death, and it
dies with the map (`CBot()` in `ServerActivate`) because connection indices are per mesh. Damage does not
force a repath — the edge is already behind the bot. The `hurt N hp on conn C` log line is the only view of
the table. navgen never passes a log, so `--dump-path` is unaffected.

### Runtime teleport connections

`Nav::AddTeleport` / `SetTeleportWait` / `RemoveTeleport` / `IsAddedTeleport` (`nav.cpp`) add one-way
`CONN_TELEPORT` connections for things navgen never saw — today TFC engineer teleporters (`doc/games.md`,
`tfc_ability.cpp`). An added connection is an ordinary `Connection` + `PlanConnection` pair: `launchPos` the
entrance, `landingPos` the exit, `travelSeconds` 0.5, `costSeconds` the wait, flagged `CONN_FLAG_ONE_WAY`
and listed on **both** areas' `connIndices` (the reverse searches must find it from the exit side).
`PlanWalker::Cross`'s teleport branch turns it into the usual trigger + destination waypoint pair.

- **Indices are stable.** Slots are appended to `conns`/`planConns` and never erased; a removed slot is
  unlisted from its areas, masked out and reused by the next add. Nothing keeps a `Connection *` across
  frames, so appending is safe. Both lists clear on `LoadForMap`/`Unload`.
- **A team mask gates it.** `Connection::teamMask` (0 = everyone, which every baked connection is) is
  checked in `BuildGraph` and `AreaCostsImpl`. `ThinkOneBot` calls `Nav::SetPlanTeam(teamBit, carrier)`
  before `Bot_MoveNav`. Teleports get the team bit, or 0 while the bot carries an objective, because TFC
  refuses to teleport a player carrying a flagged goal item. A plan made outside a bot's think (console
  dumps, tactics fields computed for another bot) sees whatever mask was set last.
- **Activation detours carry one too.** `PlanDetour::teamMask` is runtime-only — never in the file, 0 on
  load — and stamped by `Nav::SetDetourTeams` from live TFC entities (`doc/games.md`, "Team-only doors,
  lifts and buttons"). `FindDetour` skips a detour whose mask misses `NavPlan::SetDetourTeamMask`, so
  `GateAllowsCrossing` and `PushActivationDetour` refuse the edge as if no detour existed. `SetPlanTeam`
  passes the team bit here **even for a carrier**: carrying a flag does not stop a TFC player opening their
  own doors. The mask defaults to `~0u` and navgen never sets it.
- **navgen never calls any of it**, so offline route sweeps are unchanged.

## The walker (`waypoint_plan.cpp`)

### NavPlan::PlanWalker

One partial route: the funnel state, the waypoints emitted so far, the running string-pulled length and the
accumulated `costSeconds`. It is deliberately **copyable**, because the search keeps one per node and
expanding an edge copies the parent's.

- `Reset(startPoint)` — begin at a point, clearing waypoints, pending gates and stats.
- `Cross(conn, connIndex, fromId, toId)` — advance over one connection. Returns **false** when the
  connection cannot be planned from here — an unsimulable air move, a gravity mismatch, a refused
  activation — and the search treats that as "this edge does not exist".
- `Finish(goalPoint, lastAreaId)` — flush the funnel to the goal and append the goal waypoint.
- `Retract()`, `Respeed()`, `Relax(startArea)` — post-passes on the winning walker, in that order; each has
  a section below.
- `Time()` — committed corners plus the pull to the current frontier, over `kNavRunSpeed`, plus the
  `AddCost` contributions. This is the search's `g`.
- `Frontier()` — the last pending gate's midpoint, or the anchor when the funnel is empty. This is what
  `h` measures from.

**Barriers.** Air moves, teleports, ladders, elevators, doors, buttons and triggers all flush the funnel and
re-anchor it at the waypoint they emit, so the string pull never crosses one. That is why a forward search
can resolve each air move exactly once, when it traverses it, and never revisit an earlier landing — and why
`m_pendingGates` only ever holds plain-walk gates.

### Connection types in Cross

**Walk (including doors).** A plain walk pushes both gates onto the funnel and continues — leaving area
first, then entering area. A door adds a button/trigger waypoint if needed, then the door waypoint
(`requiresDoor`, `doorWaitTime`).

**Activation detours.** Entering an area with `activation != ACT_NONE` looks up the precomputed detour for
the neighbour being entered from (`FindDetour`). A missing or blocked detour makes `Cross` return false and
A\* routes around, so a route never passes a door whose activator is unreachable from that side. A gated area
entered by any non-walk connection is refused outright. The stored polyline starts and ends on the *same*
gate midpoint the door waypoint is derived from — both sides call `GateMidpoint`, and if they ever drift the
unvalidated straight segment between them comes back. `FlushFunnelTo` targets the polyline's first point,
a point on the entering area's boundary, like any other flush. Only the polyline's interior is
funnel-validated; the pull from `m_anchor` to its first point is an ordinary straight 2D pull and can clip
a wall in a prism area.

**Elevator.**
- Checked *before* the activation guard, because a stop area is gated *and* legitimately enterable; the
  guard would otherwise reject every non-walk edge into it.
- When the destination stop is gated, `PushActivationDetour` emits the summon detour first (walk to the call
  button, press, come back). A missing or blocked detour returns false — which is how "you cannot call this
  lift from here" is expressed.
- Adds `rideWaitSeconds` to `m_extraTime`, then a counted boarding waypoint carrying
  `requiresElevatorRide` / `elevatorTravelTime` / `elevatorWaitTime`, then a free arrival waypoint so the
  shaft's height never enters the funnel. The anchor moves to the arrival point. The wait is copied onto the
  waypoint as well as charged to the search, because the runner's ride deadline must allow for a platform
  that does not set off until its dwell elapses (`doc/movement.md`, Elevators).
- Walking *onto* a gated stop takes the ordinary door path instead: `needsStop` includes a gated elevator
  area, so the same detour runs and `doorWaitTime` (the summon time) is charged.

**Teleport.** Flushes the funnel to `launchPos`, adds the trigger waypoint (`requiresTeleport`,
`teleportDest = landingPos`) and the destination waypoint, and moves the anchor to `landingPos`.

**Ladder.**
- Direction comes from the connection: `fromId == conn.areaA` means climbing, so the mount is `launchPos` and
  the exit `landingPos`. Descending, the exit is `launchPos` but the mount is **`gateB.left`, not
  `landingPos`** — where you land coming up is not where you get on going down (`doc/navgen.md`). When they
  differ, the mount is a ledge point the bot runs off to catch the ladder mid-fall. A one-way ladder never
  reaches the descending case, because `PlanRoute` refuses the reverse edge.
- The bot runs in at `ladderEntryYaw` (from the connection's `gateAYaw`) and holds `ladderYaw` once on the
  ladder. They are the same angle except for a side approach, where catching the column means arriving ~90°
  off the face normal.
- Flushes the funnel to the mount at player height for the area being left; adds the mount waypoint
  (`requiresLadder`, `ladderUp`, `ladderYaw`, `ladderEntryYaw`, `ladderSpeed`, `ladderExit`, target area,
  `connIndex`) and the exit waypoint; moves the anchor to the exit.
- The exit waypoint is not counted toward path length: the climb is vertical and its cost already arrived
  through `AddCost`, from `ConnectionCostSeconds`' ladder arm.
- No gate is pushed. Ladder gates are degenerate points at mount and dismount, so the mount is simply a
  hard waypoint, like a teleport trigger.

**Air move.**
- Refused outright when `conn.gravity` does not match `PlanTraceContext::gravityMultiplier` (see
  "Planning with gravity multipliers").
- Pushes the arrival gate (entry side only) and computes where the funnel crosses the launch gate.
- Interpolates yaw and speed from `gateAYaw`/`gateASpeed` at that fraction, clamps the speed to the server
  (see "Launch speed is clamped to the server"), and re-simulates with `JumpLandsIn` from that point,
  through `PlanTraceContext::standPush`/`duckPush` so a launch across a `trigger_push` is simulated the way
  navgen sampled it.
- On a miss, one retry at a rescaled speed ("Rescaling a missed launch"). If that misses too, `Cross` returns
  false. There is no fallback to the baked params: a launch that does not land where the plan says is worse
  than no edge.
- Pops the arrival gate, inserts the run-up node, flushes to the launch point, adds the launch waypoint
  (`airmoveLaunch`, `requiresJump`, `launchSpeed`/`launchYaw`, `jumpBackDist`, `requiresCrouch`, target area,
  `connIndex`) and the landing waypoint, and moves the anchor to the landing.

**Swim pricing is deliberately coarse.** Either endpoint under water prices the whole leg as a swim, which
over-charges the seam legs into and out of a pool. That is the direction to be wrong in: a land route that
loses to water should lose by a margin.

**Goal.** `Finish` flushes the funnel to the goal point and adds the final waypoint.

### Crouch belongs to the segment, not the corner

`requiresCrouch` on a waypoint means "be ducked while travelling *to* it", so it must be set on the waypoint
that ends every pulled segment passing through a crouch area. `FlushFunnelTo` walks the pending gates
alongside the pulled points: a corner on gate `i` ends a segment spanning gates `[previous corner's gate,
i]`, and if any of those is a crouch gate the corner is flagged. It returns whether the *tail* segment — the
one into the flush target — needs it, because that waypoint (run-up node, door, button, trigger, teleport
trigger, goal) is pushed by the caller, which ORs the flag with its own area's `crouch`.

Why: reading the flag off the gate a corner sits on loses it whenever the funnel runs straight through a
crouch gate without emitting a corner there — the normal case, since a gate only becomes a corner when the
funnel constricts. On `test45_slopes2` that walks the bot standing into the ceiling lip of crouch-only area
9, where it stalls; the follower's only other source of "duck" is the area it already stands in, which a
standing bot can never enter.

`PendingGate::crouch` is `fromArea.crouch || toArea.crouch`, so the flag lands one segment early on the way
in and one late on the way out. The early one is wanted: `PM_Duck` takes 0.4 s on the ground before
`FL_DUCKING` and the small hull appear, so a flag arriving exactly at the boundary would be too late. The
late one costs a segment held at `kDuckingMultiplier` speed.

### Gates and geometry

The funnel only pulls through gates; these build them and answer geometric questions about areas.

- **`BuildAreaGates`** — one inset gate (`edgeGates`) per bound wall plane.
- **`BuildConnectionGates`** — for a walk connection, finds the shared face and builds a gate on each side,
  falling back to the prebuilt `edgeGates`; air moves skip it and use their sampled gates.
  - A shared wall runs the whole boundary, but only the stretch where the two floors meet within a step is
    crossable — the rest is a cliff face. The face is narrowed to the longest steppable run, with each end
    pushed out to the exact crossing so the gate is not quantized away by the sampling spacing, and the
    function reports whether anything survived.
  - It is the same wall from either side, but only one side may have bound planes that close it: a wedge
    under a ramp is capped by its sloped ceiling, so its own edge line runs to infinity and `EdgeOfPlane`
    rejects it. The line is taken from whichever neighbour bounds it.
  - `PlanFloorFn` (`waypoint_plan.h`) samples an area's walkable floor at a 2D point, false where there is
    none. `PlanArea` models only plane floors, so navgen, which knows the real geometry, supplies it to keep
    gates off the cliff parts of a shared wall.
- **`GateFromEdge`** — builds a gate from an edge line: insets both ends by `kPlanClearance` (16), pushes it
  inward by up to 16, and clamps it to a point when too narrow. The push is capped at **half the area's
  depth** along `inward`, measured from each inset end to the far wall with `ClipLineByWalls`; an unbounded
  depth keeps the full push. Without the cap, an area thinner than 32 gets the gates of its two opposite
  connections crossed, and the funnel walks forward, back and forward to thread them: at de_dust2's mid
  double door (16-deep sliver 208) every bot drives into the jamb. Capped, both gates sit mid-sliver and the pull
  goes straight.
- **`ConnectionCrossing`** — the gate midpoint, or `launchPos` for a non-walk connection.
- **`GateFloorZ`** interpolates floor z along a gate from its endpoint heights; **`GateFraction`** is the
  parametric position along a stored gate (0 = left, 1 = right).
- **`NearestPointOnGate`** — closest point on a gate segment to the funnel polyline. **`CrossingOnGate`** —
  where the polyline crosses the gate plane, clamped to the gate span.
- **`LerpYaw`** — shortest angular interpolation across ±180.
- **`PointInArea`** — point against `boundPlanes` with epsilon. **`InAreaColumn`** — against the upright
  planes and the implied walls only, ignoring floor and ceiling.
- **`FloorPlaneOf`** — the bound plane with the highest `normal.z`. That is the *flattest* upward plane, not
  the highest, so it only answers whether an area has a floor at all — never a height.
- **`AreaFloorZ`** — floor height at (x, y). An area is the intersection of its half-spaces, so every plane
  with `normal.z > kWallNormalZ` (0.3) is a lower bound and the floor is the **maximum** over them. Reading
  one plane is wrong for any area with more than one upward face, which is every prism area: crossfire's
  area 168 is a ramp plus a flat patch, and `FloorPlaneOf` gives the patch's −1838 for a ramp climbing to
  −1684, which puts a run-up node on the pit floor (see "The run-up node"). A prism area's planes are its
  footprint plus a z-slab, so there it returns the slab bottom; a caller that needs the standing height on a
  prism traces down (`ResolveVolumeAreas`, `doc/goals.md`, "Healing volumes give one entity several candidates").
- **`FloorsMeetAt`** — two areas' floors agree at a seam point within a tolerance; both must have a floor
  plane, and the heights come from `AreaFloorZ`.
- **`EdgeOfPlane`** — a wall plane as a 2D segment, clipped by the area's other walls. **`ClipLineByWalls`**
  — clips a segment by an area's wall planes.

**Implied walls.** `ForEachImpliedWall`: an area's footprint is not fixed by its vertical planes alone. A
floor rising one way and a ceiling falling the other pinch the column shut where they meet — a wedge under a
ramp is capped by the ramp, a sliver between two pyramid faces is bounded that way at both ends. Eliminating
z from the half-space system recovers those bounds exactly: for a floor `f` and ceiling `c`, "the floor stays
under the ceiling" is one linear constraint in x and y, implied by the two planes, so it can only trim
overreach, never cut into the area. It iterates every floor/ceiling pair, skips pairs parallel in plan (they
never converge), and emits each as an inward-facing half-plane in x/y.

**A tilted wall is a ceiling.** "Ceiling" means every plane that is neither a floor (`normal.z >
kWallNormalZ`) nor truly upright (`|normal.z| <= kUprightNormalZ`, 0.001) — overhangs and steep slopes
included. `InAreaColumn` and `ClipLineByWalls` treat only upright planes as walls. `EdgeOfPlane` turns a
tilted connection plane into its line *at the floor*, by the same elimination against `FloorPlaneOf`; an area
with no floor plane keeps the plain reading at z = 0. Reading a tilted plane at z = 0 is off by
`normal.z * floorZ / |normal.xy|`, growing with the floor's distance from z = 0: on de_aztec an arch plane in
area 450 (floor z −206) read that way is a wall 46 units off its real foot, which cuts a 2.4-unit seam to
nothing and sends the string pull through solid. `--dump-path-between "x y z x y z"` plans such a leg offline.

### Corner clearance

`PlanWaypoint::clearance` is how far an arc may deviate from the vertex. `GateClearance` derives it from the
gate the corner came from — it is not stored on `PlanGate`, because gates round-trip through
`MergedConnection` and the `.nav` file field by field, and anything added to `PlanGate` is silently dropped
on load.

Gate width is not the budget. `GateFromEdge` already insets endpoints 16 from the wall, so the remaining
width is *extra* freedom: `clearance = min(16 + halfWidth, 32)`. A clamped gate is a point and gets
`kPlanClampedClearance` (6.63 = `16 * (sqrt(2) - 1)`, the diagonal slack at a right-angle corner of a
minimum-width corridor). Under-estimating stops the bot at every tight gate.

**That extra width is on the wrong side**, so `CornerClearance` in `FlushFunnelTo` clamps it. A funnel corner
sits on a gate endpoint and its arc bulges toward the inside of the turn, along `norm(w - u)`, where the room
is the 16-unit inset, not the half-width. It tries `GateClearance`, then `kPlanClearance`, then
`kPlanClampedClearance`, and keeps the first whose apex `p + bis * c` lies inside the corner's own area or
the gate's other area — column tests only, no traces. An apex in a *third* area (a fragmented open floor) is
refused too, costing only a slightly tighter arc. The profile's hull trace catches an over-estimate next to
a wall, but over a ledge a horizontal hull trace hits nothing, and without the clamp bots round 2fort's
catwalk corners into the pit. On the test maps the clamp changes the clearance at 77 corners, the radius at 3
(by 1–3 units), and no route time.

### Relaxing over walk-connected areas

`Relax` runs after the search: if the route goes a → b → c and the line a → c is clear, b is dropped — even
when that line crosses areas not on the A\* chain. From each waypoint it tries the *farthest* plain waypoint
first and steps back until a line is clear.

Why: the funnel only pulls through gates on the A\* chain, and every gate is inset 16 from both ends of its
seam whether that end is a wall or another seam. On a fragmented floor many seams are under 32 u, collapse to
point gates and pin the path to their midpoints — a chain of small jogs, and collinear duplicate corners in
corridors. The per-node walker also makes A\* path-dependent, so an area the straight line would cross can
lose the search. Extending forward and stopping at the first failure is not enough: on dust2's S-door the
short hops between the door's tight corners fail, while the long line from the last door corner to the goal
is clear.

- **Only plain runs.** A waypoint is plain when it has no `connIndex` and none of the door, break, button,
  trigger, teleport, ladder, elevator, launch or jump flags (crouch is allowed). Both ends and everything
  dropped must be plain, so special waypoints are never removed and the corner before one (the run-up node,
  the door approach) always survives as a run end.
- **Clear** means three `ReachBack` walks over the walk-successor graph all reach the end: the centre line,
  and two parallel lines `kPlanRelaxClearance` (12) to either side, each started by a perpendicular walk out
  of a. `ReachBack` only crosses seams where `FloorsMeetAt` holds, so ledges stop it. Every area crossed must
  be plain floor (no door, breakable, elevator, crouch, activation, drift or water), and the centre walk
  must end on a floor within `kPlanRunupSeamStep` (18) of c's height, so a storey overlapping in 2D does not
  count. The offset is just under 16 because a funnel corner sits exactly 16 from its wall corner and
  `ReachBack` keeps a 2-unit inset.
- **Start area of a waypoint.** A corner's `targetAreaId` is its gate's to-area, but the point may lie in the
  from-area, so `AreaOfWaypoint` takes the target if its column holds the point, otherwise the walk
  neighbour that does with the nearest floor.
- **It runs last**, after `Respeed`, because `Retract` and `Respeed` address waypoints by index through
  `m_launches` and erasing earlier would shift them. Relax never touches launch, landing or run-up
  waypoints, so the order changes nothing else.
- `areaPath`/`connPath` stay the A\* chain. A bot standing in a relaxed-in area finds no planned connection to
  penalise after a blocked detour; that is accepted. The ETA comes from the profile over the final
  waypoints, so it stays consistent.

It cannot fix a mesh whose seams really are ledges: on test14_terrain's hill neighbouring slope areas differ
by 24–33 u except in a few-unit window (`ClipLineByStep`), so the straight line would cross a ledge and the
jogs stay.

### Air moves at plan time

#### JumpLandsIn

Re-simulates one air move at plan time and accepts it only if the mover ends inside the destination area. It
starts **on the launch point**, at the given speed and the launch heading, and requests the jump on the
first frame.

Why not start some distance back to validate the run-up: a straight line drawn backwards from the launch is
not where the route comes from, so it runs through whatever is behind the launch and wedges. At 64 units back
it fails every air move on every test map. The approach is checked by the profile instead, which knows the
real route (the run-up node below).

`JumpLandsIn` pins `physics.frameTime` to `kPlanSimFrameTime` (1/60) on its local copy — see the airmove
cache for why.

#### Unplannable air moves

A launch `JumpLandsIn` cannot make — or one with no gate to funnel through, so nothing was simulated — makes
`Cross` return false. The search drops the edge and carries on; nothing is truncated or retried, and the
returned route has no unplannable air moves by construction.

The failure is route-dependent, not a property of the connection: the stored launch comes from
`ProbeAirMoves` sampling a whole edge, while the plan launches from wherever the funnel crosses it on *this*
route. On `test45_slopes2` the funnel pulls a drop's launch to the gate corner, from where the mover clears
the lip and lands on the ramp instead of the pit floor.

`--dump-airmove`'s `sim miss` lines are therefore not failures — they are the edges the search routed
around, and their count equals `rejected in search` (three on test45_slopes2, four each on test28_slopecase
and test15_staircase). A connection that appears there on *every* route is an air move `ProbeAirMoves`
should not have exported; chase it there.

#### The run-up node

A waypoint inserted back along the launch heading so the route's last leg into a launch is straight and
aligned. Its distance is `launchSpeed * kPlanAirmoveRunupTime` (0.1 s of travel, roughly six frames),
floored at `kPlanAirmoveRunupMin` (8) — proportional, because a fixed distance strands the node off the
route for slow drops (on `test15_staircase` two segments leave the mesh).

Bounds on that distance:
- **Below:** `AccelDistance(0, launchSpeed)` plus one `AirmoveLaunchLead`, the distance physics needs to reach
  the design speed from a standstill. It does not bind in the current speed range (0.1 s is larger from
  about 70 u/s up), but it keeps the travel-time guess from silently falling under the real requirement.
  It is charged at doubled friction when the launch sits at a ledge ("Edge friction on the approach").
- **Above:** `ClampTravelInsideArea` walks the source area's wall planes and shortens the offset so the
  node lands inside, `kLaunchInset` (2) clear of the wall. Placed blindly the node goes through the wall
  whenever the launch sits closer than the offset to the far side (on `test37_walkent`, 1.1 units into
  solid). The clamp limits length, not direction: when the route arrives perpendicular to the launch heading
  the node can still sit behind the bot, which produces a hairpin.

**The clamp runs backwards along the route, not inside one area.** `PlanWalker::RunupDistance` steps area by
area: clamp inside the current one and, when that falls short, re-clamp with no inset to the seam and
continue in the area the route came from. Predecessors come straight off `m_pendingGates` — each walk
connection pushes a gate pair carrying `insideArea`, and the list holds only plain-walk gates — so no
adjacency structure is plumbed into the walker. The chain stops at a real wall, at a seam the next area's
column does not contain, at a seam whose floors differ by more than `kPlanRunupSeamStep`, or at a crouch area
(running at `kDuckingMultiplier` makes extra distance worth less than the ducking costs).

- The **floor-continuity** stop exists because `InAreaColumn` is a pure XY test and happily walks a run-up
  back across a cliff: crossfire's area 168 is a 26.57° ramp sharing its column with the pit below, and
  without the stop the run-up for the ramp-top jump lands on the pit floor 150 units under the launch.
- Why not confine the run-up to one area: it silently caps launch speed on thin ledges. `test54_window`'s
  16-deep lip gives its jump 3.2 of the 26.7 units it wants and the bot reaches 159 of 267; through the
  window into the next area it launches at 267.
- Two invariants make it safe. Every step is clamped inside a convex area and each seam point checked
  against the next column, so the straight run-up → launch leg lies inside the union. And the gates the
  run-up point now sits *behind* are dropped before `FlushFunnelTo`, or the pull would be forced out through
  a gate on the near side and come back.
- Cost: a run-up node absorbing a former gate corner loses that corner's clearance and falls back to 16 —
  the conservative direction, which is why `test5_jump` trades two corners for one and reports one more
  `slowed`.

**The node is also what makes the bot jump at all.** `Bot_MoveNav` hands control to `RunAirmoveApproach`
when the tracker's `wpIndex` reaches the launch waypoint, and `wpIndex` comes from the profile segment the
bot projects onto. `BuildPathProfile` drops segments shorter than `kEpsLen`, and the funnel's last corner
lands exactly on the launch point, so without this node the segment ending at the launch is zero-length,
dropped, and no segment carries the launch's index — the bot walks through the launch and never jumps.

#### Launch speed is clamped to the server

`Cross` clamps the candidate speed to `physics.maxSpeed` before handing it to `JumpLandsIn`.

`candSpeed` comes from the nav file's `gateASpeed`, sampled at navgen's `--max-speed` (320 by default). The
bot flying it may be slower, and the number to compare against is not a cvar but what `PM_CheckParamters`
left in `pmove->maxspeed` — 270 for a valve player, whether or not `sv_maxspeed` says so. `Bot_Physics` reads
that live value into `PlanTraceContext::physics` (`doc/movement.md`, `Bot_MaxSpeed`).

This keeps a nav file server-independent: sampling stores the fastest jump the geometry allows, each server
asks whether *it* can make it, and an air move needing 315 on a 270 server fails `JumpLandsIn` and is
rejected like any other. No regen, no per-server nav.

The clamp has to live here because nothing downstream applies it. `PmSim::Begin` sets the launch velocity
**unclamped** — its `maxSpeed` bounds the air-accel wish speed in flight, not the launch — so an unclamped
speed simulates a jump the bot can never deliver. `RunAirmoveDrive` does clamp to `Bot_MaxSpeed`, so the bot
then chases an unreachable launch speed and aborts every attempt (unclamped, `test40_walljump` plans 315 and
drives 270). `Respeed` does not cover it either: when the slower re-simulation also misses it keeps the design speed
by design.

#### Rescaling a missed launch

A miss in `Cross` gets exactly one retry at speed `want / conn.flightTime`, where `want` is the destination
**centroid** projected onto the candidate heading and `flightTime` is the air time of the sampled trajectory
the connection was built from.

Why retry at all: the yaw/speed pair `Cross` interpolates at the funnel's crossing fraction was never flown
by any sample. The gate endpoints sit at mean ± 1σ along the fitted segment and their yaw/speed come from a
one-sided local regression, so at `t` 0 or 1 the planner is extrapolating, and overshooting or undershooting
by a few tens of units is routine. Refusing those edges loses real connectivity.

Why divide by the connection's flight time: it is the air time of a flight that really landed in the
destination, so the division asks how fast the bot must leave to cover `want` in the time the geometry
allows. The obvious alternative, scaling by `want / |end - exitPoint|`, is the *same number* whenever the miss
ends at the destination's height, and diverges when it ends elsewhere vertically. `test54_window`'s `1->2` is
the extreme case: the unscaled launch overshoots a 16-unit lip and falls 224 units into a pit; the ratio folds
that fall in and asks for 44, the flight-time form asks for 91 and lands.

Why the centroid and not the funnel's crossing of the arrival gate: that crossing is by construction the
*nearest* corner of the gate, and aiming a rescaled jump at the edge of the landing area is how you land just
outside it (`test54_window`'s `1->3`: 207 from the crossing sails over the lip, 152 from the centroid lands).

Bounds:
- Nothing iterates. A second miss refuses the edge.
- The retry is skipped when `want` is under `kPlanAirmoveRangeEps` or the correction moves the speed by less
  than `kPlanAirmoveRespeedEps`. The rescaled speed is clamped to `physics.maxSpeed`.
- A connection with `flightTime` under `kPlanAirmoveFlightEps` falls back to the distance ratio, where the
  clamp bounds the degenerate case: a first sim that wedges immediately travels ~0 units, so it retries at
  full speed — one wasted simulation, never an unbounded one.
- Correcting downward composes with `Respeed`; correcting upward does not (a raised speed can exceed what the
  run-up delivers, and `Respeed` then knocks it back). In practice the corrections that matter are
  reductions, because the extrapolated gate speed is biased toward the sample cloud's mean and overshoots.

`test54_window` is the map this exists for: its spawn room's only exit is a window 24 units up the wall, so
navgen finds no launch edge there (`FilterLaunchEdges` needs void ahead at running height) and every jump
out is reverse-seeded from the landings of the jumps back in — the worst case for the end regression. Without
the retry the map has no route. Across the 19 stock deathmatch maps the flight-time form lands 169 of 586
retries against the distance ratio's 161 of 582, and the two differ in three routes (`contamination`,
`disposal`, `datacore`); on the test maps only `test28_slopecase` and `test31_push` differ, and all 51 pass.

#### Re-speeding a launch

`PlanWalker::Respeed` re-simulates each launch at the speed the route can actually deliver. It builds a probe
`PlanProfile` over the finished waypoints; `ProfileLaunchReach` reads the reachable-speed pass on the segment
ending at each launch (arc fit, backward braking pass and acceleration integral folded into one number), and
if that falls short of `PendingLaunch::designSpeed` by more than `kPlanAirmoveRespeedEps`, the jump is
re-simulated at the reach and the landing waypoint moves to where it lands. The yaw needs no change: the last
leg is straight along the launch heading by construction.

Why: the stored `exitSpeed` is what the sampler launched at, not what the route delivers. A corner 30 units
before the launch caps the bot well under it, and without this pass the plan asserts a speed the bot never
reaches.

- **The reach is read one `AirmoveLaunchLead` short of the launch**, not at it. The bot commits to the jump a
  frame before the plane, so that is where its speed stops rising. Read at the launch point it saturates —
  `vExit` on that segment *is* the design speed — so the check could only fail if the whole segment fell
  short.
- **A slower jump that no longer lands keeps the design speed**, counted in `airmovesRespeedKept`, and the
  route is not truncated. A jump the bot may come up short on beats a plan that stops dead mid-map, and the
  follower has `launchSimFailed` to reason about. It fires on no test map; a kept design speed
  is the signal that a launch asserts a speed its approach cannot deliver.
- **One iteration only.** The run-up node's position depends on the design speed and the funnel was already
  pulled through it, so re-placing it from the re-speeded result would invalidate the geometry it was
  computed on.
- **Why not an incremental reach estimate inside the search?** It would let launches be simulated at the
  right speed first time, but would change launch speeds on every map to replace a pass that never fires.

Two profile rules make the reach meaningful, and both are required:
- the corner *into* the run-up node is not suppressed: it gets a real arc and speed cap, with its outward
  tangent clamped to half the run-up leg so the straight approach survives;
- `NodeMaxSpeed` does not apply `kDuckingMultiplier` at a launch waypoint, where `requiresCrouch` means duck
  *in flight*; capping the approach at 107 makes every crouch jump arrive at a third of its launch speed.

`Retract` runs before `Respeed` and may lower an earlier launch; because it rewrites
`PendingLaunch::designSpeed`, the two compose instead of fighting.

#### Landing on the next run-up

When the barrier before an air move is *another* air move, the run-up node can land behind where the bot
lands: `RunupDistance` walks back through area geometry, and a previous landing is not in `m_pendingGates` to
stop it. Unclamped, on `test54_window` at `--plan-speed 270`:

```
wp[ 3] landing  (582.0, 759.4, -1475.4) area 2
wp[ 4] corner   (585.6, 773.4, -1476.0) area 2     <- run-up node, behind the landing
wp[ 5] jump     (585.2, 746.9, -1476.0) area 17 JUMP launch 265
```

The plan credits 26.5 units of approach, but the bot lands already moving toward wp[5] and cannot use them;
it delivers 200 of 265 and aborts. `Respeed` cannot see this: `IsGap` covers launch, teleport and ladder
waypoints but not landings, so the ~170° reversal is arc-fitted to a pivot cap around 53 and
`ProfileLaunchReach` integrates over the whole 26.5 units and saturates.

Two fixes:

**The offset is clamped at the anchor, but only across an air move.** After an air move `m_anchor` is the
landing, so the run-up cannot start before it. The clamp projects onto the launch heading, which is only valid
when the bot arrives *on* that heading — hence `m_anchorFromAirmove`. Applied at every anchor it breaks curved
approaches (on `test6_crouchjump` the projection of the start point is −194 and a legitimate run-up vanishes).

**`PlanWalker::Retract` buys the distance back.** For each adjacent launch pair whose later launch's
`needRunup` exceeds the `haveRunup` the clamp left it, the *earlier* launch speed is cut by the same one-shot
ratio as the `Cross` rescale, moving its landing back along its own flight line.
- `needRunup` and `haveRunup` are recorded on `PendingLaunch` at `Cross` time, so detection needs no
  profile. `needRunup` is `AccelDistance(0, launchSpeed)` plus one lead — entry-speed independent, so there
  is no feedback loop. Sizing it from `AccelDistance(reached, design)` under-delivers: pulling the earlier
  launch back also lowers the speed carried into the approach.
- Both shapes are handled: `[launch][landing][run-up][launch]` and, when the anchor clamp left `runupBack`
  under `kPlanAirmoveRunupEps` and no node was pushed, `[launch][landing][launch]`. In the short shape the
  landing itself starts the measured segment, so the pull lengthens it directly.
- **The run-up node moves with the landing.** `ProfileLaunchReach` measures only node → launch, so room opened
  before the node is invisible to it. `Retract` re-places the node `needRunup` back along the launch heading,
  re-floored in its own area and rejected if it leaves that area's column. Landing and node move by the same
  distance, so no new corner appears.
- **The pulled landing may leave the destination area.** On a thin destination the pull lands short of it
  (`test54_window`'s 16-unit lip). A failed `JumpLandsIn` then gets a second look: `AreaContaining` finds
  where the mover came to rest, and the landing is accepted there when that area is not crouch and
  `LegIsWalkable` confirms the straight leg from it to the launch. That leg check is load-bearing: the
  landing moves along the *earlier flight line* while the run-up runs along the *launch heading*, so it can
  drift sideways off walkable ground while passing every other test.
- Guards that keep the design speed and count `airmovesRetractKept`: a degenerate flight length; alignment
  below `kPlanAirmoveRetractAlign` (0.5) between the earlier flight and the later heading — beyond 60° the
  pull is mostly lateral and the required distance blows up; and a re-simulation landing neither in the
  destination nor in an accepted neighbour. Nothing is ever truncated.
- `Retract` writes `PendingLaunch::designSpeed` as well as the waypoint, so `Respeed` stays downward-only
  relative to the retracted value instead of raising it straight back.

**`ReachBack` is not `RunupDistance`.** `RunupDistance` chains areas off `m_pendingGates`, which is right for
`Cross`: the gate trail is exactly the ground the route came over, and walking back past a barrier is not
expressible in the funnel. `Retract` runs on a settled route and only asks whether a straight leg is walkable,
so it reads walk adjacency directly: `PlanRoute` builds a per-area predecessor list from the `CONN_WALK`
connections and hands it to every walker. `ReachBack` keeps every `RunupDistance` guard (convex clamp, seam in
the next column, `FloorsMeetAt` within `kPlanRunupSeamStep`, no crouch) plus a visited set, because the area
graph has cycles and a gate trail does not.

**The follower does not care which area the landing is in.** `AirmoveLandingMiss` measures against the
landing *waypoint*, so the area fallback never runs and a landing in the neighbour is not penalized; the launch
keeps the connection's own destination in `targetAreaId`, hence log lines like `landed ... area 2 (wanted 3)
miss 1.9 ... penalty 0.0`.

On `test54_window` `Retract` cuts the `1→3` launch from 142 to 74, landing in area 2; the `3→17` approach gets
39.0 units instead of 0.4, and the bot delivers 100% of its 270 design instead of 80%. It is the only map whose
plan `Retract` changes.

#### Edge friction on the approach

Every run-up ends at a ledge, and `PM_Friction` charges for that: it traces `kEdgeProbeAhead` (16) ahead along
the velocity and `kEdgeProbeDrop` (34) down, and when that comes back clear multiplies ground friction by
`sv_edgefriction`, normally 2. So the last 16 units before any launch run at friction 8, not 4. The follower
models this (`Bot_DrivePhysics`, `doc/movement.md`), but that only corrects its throttle; `MakePlanTraces`
copies plain `Bot_Physics`, so the planner has to model the ledge itself or it sizes every approach with half
the real friction.

It bites when the run-up is short: at `accel = 2700`, `stopSpeed = 100`, reaching 236 u/s from rest takes
13.7 units at friction 4 and **21.5** at friction 8. Sized at friction 4, a crossfire launch out of a
36-unit-wide area gets 14 units of run-up, delivers ~198 of 236 and aborts at 80–84% of design.

`PlanWalker::LaunchAtEdge` runs the engine's probe from the launch floor along the launch heading, in the hull
the bot has on the run-up (the launch area's `crouch`), once per launch in `Cross`, cached on
`PendingLaunch::atEdge`. Probing at the launch is the right read: it is the closest point to the drop, so a
clear probe there means the whole final 16 units are in the zone.

The two users treat the flag differently on purpose:
- **`Cross` doubles friction for all of `needed`**, even past 16 units. Deliberately pessimistic and safe:
  `needed` can only *lengthen* `runupBack`, and `RunupDistance` and the anchor clamp then cut it to what the
  geometry offers. It never asserts a speed.
- **`ProfileLaunchReach` does assert one, so it integrates exactly:** `AccelForward` over the plain stretch,
  then again over the edge stretch at doubled friction. The edge stretch is `min(usable, edgeZone - lead)`,
  because the bot commits one `AirmoveLaunchLead` before the waypoint.

Modelling it moves launch speeds in both directions (5 of 45 maps at each plan speed; `test6_crouchjump` 235 →
258, `test38_islands` 282 → 278): a longer `needed` pushes the node back, and a longer approach can support a
faster launch even at doubled friction. Either way the plan asserts what the ground delivers.

**Still optimistic:** `BuildPathProfile` carries one global `profile.friction`, so the `vReachEntry` handed to
`ProfileLaunchReach` is computed without edge friction. Second order — it feeds the long haul, not the last
16 units — and per-segment friction in the profile is a much larger change.

### Data structures (`waypoint_plan.h`)

**`PlanArea`**
- `boundPlanes` — inward-facing planes (prism: footprint walls + z-slab); `edgeGates` — one inset gate per
  bound plane.
- `door`, `breakable`, `activation`, `doorWaitTime` — a door area makes the plan stop at it.
- `detours` — one `PlanDetour` per walk neighbour of a gated area, precomputed by navgen: a closed polyline of
  `PlanDetourPoint` starting and ending on that neighbour's gate midpoint, `actIndex` marking the activation
  waypoint, `blocked` when no activator is reachable from that side. `FindDetour` scans linearly; a gated area
  has two to four entries.

**`PlanConnection`** — `planeA`/`planeB` (bound plane indices, baking only), `gateA`/`gateB` (shared-face gates
from `BuildConnectionGates`), `gateAYaw[2]`/`gateASpeed[2]` (launch params at gateA's ends).

**`PlanWaypoint`**
- `point` — at player height (duck origin + 18 for standing).
- Flags and payload: `requiresElevatorRide`, `elevatorTravelTime`, `elevatorWaitTime`, `requiresCrouch`,
  `requiresDoor`, `requiresBreak`, `requiresButtonUse`, `requiresTriggerTouch`, `activatorCenter`,
  `requiresTeleport`, `teleportDest`, `requiresLadder`, `ladderUp`, `ladderYaw`, `ladderEntryYaw`,
  `ladderSpeed`, `ladderExit`, `airmoveLaunch`, `requiresJump`, `launchSpeed`, `launchYaw`, `jumpBackDist`,
  `launchSimulated`, `launchSimFailed`, `launchSimFrom`/`To`/`Aim`, `targetAreaId`, `connIndex`,
  `clearance`, `drift`.
- `activatorCenter` is the activator brush model's centre, copied from the detour. GoldSrc `PlayerUse` picks
  the entity whose centre gives the largest forward dot above `VIEW_FIELD_NARROW`, so the bot cannot press
  without knowing where the brush is.
- `requiresBreak` is serviced at runtime by `BotBreak` (`doc/movement.md`). The plan's `openSeconds`
  (`health * 0.1`) is only the ETA estimate; the runtime decides for itself whether the entity is breakable.

**Constants**
- `kPlanClearance = 16` — player hull radius; gates are inset this much at the ends and pushed up to this
  far off the connection plane (at most half the area's depth).
- `kPlanClampedClearance = 6.63` — corner budget at a clamped gate (see Corner clearance).
- `kPlanStandHeight = 18` — duck-hull origin to stand origin.

**`PlanTraceContext`** — `duckTrace`/`standTrace` for re-simulation, `duckPush`/`standPush`, `physics`,
`gravityMultiplier`, `airmoveCache`, `dumpAirmoves`. `collectRunupTraces` is vestigial: the steered-approach
simulator it fed is gone, nothing reads it, and nothing fills `PlanStats::runupTraces` (`--dump-svg` still
accepts them and draws nothing).

**`PlanStats`** — per walker, so they describe the kept route only; a walker copied down an abandoned branch
takes its counters with it.
- `airmovesSimulated` — launches re-simulated and accepted, i.e. the air moves on the plan.
- `airmovesUnplannable` — launches the simulation could not make; always 0 on the winning walker, since such
  a walker was discarded. `PlannedRoute::airmovesRejected` is the search-wide count.
- `airmovesRespeeded` / `airmovesRespeedKept` — launches re-simulated at the reachable speed, and those where
  that also missed so the design speed was kept.
- `airmovesRetracted` / `airmovesRetractKept` — earlier launches pulled back for the next run-up, and pairs
  where the pull was refused.
- `corners`, `gatesClamped` — funnel diagnostics.

## The funnel (`funnel_graph.cpp`)

`NavFunnel::StringPull` is the funnel algorithm on 2D gates:
- gates are oriented by travel direction (`MakeGate` orders endpoints so left/right match it, by cross
  product);
- it maintains the apex and the left/right bounds, and when the funnel constricts emits a corner at the tight
  bound and restarts from there;
- it returns the pulled points with gate indices, for corner attribution.

`PeekFunnel` is a dry-run pull to see where the route crosses the gates; `FlushFunnelTo` runs the pull and emits
the corners as waypoints.

**Coincident consecutive points are collapsed as they are pushed**, keeping the later one, so the goal always
ends the list with `kNoGate`. The last iteration feeds the goal in as both bounds, and without the collapse a
goal that becomes the tight bound is emitted twice — a zero-length step at the end of nearly every route. That
duplicate is not cosmetic: `BuildPathProfile` skips a corner whose adjacent segment is under `kEpsLen` before
assigning its speed cap, so it silently disables arc fitting and speed limiting at both corners touching it (on
`test5_jump` the 104° turn before the first launch gets no arc and a claimed 320, instead of an 11.7-unit arc
capped at 191).

## Speed profile (`path_profile.cpp`)

`BuildPathProfile` fits circular arcs to the waypoint polyline and computes a speed profile the engine can
actually hold. It runs at plan time, so navgen and the runtime produce identical geometry, inspectable offline
with `--dump-path`.

### The physics it derives from

Each ground frame the engine runs `PM_Friction` then `PM_WalkMove`. Acceleration scales with *wishspeed*, not
current speed, so at full drive it is the constant `A = sv_accelerate * maxspeed` (3200 at defaults). Friction
drag is `D(v) = sv_friction * max(v, sv_stopspeed)`.

- **Turning** at constant speed needs the wish direction tilted forward enough to cancel drag, leaving
  `a_lat(v) = sqrt(A^2 - (f*v)^2)` and `r = v^2 / a_lat(v)` — 34.9 at 320, not the drag-free `v^2/A = 32`.
  `SpeedForRadius` inverts this.
- **Braking** is coasting: `dv/dx = -f` exactly while `v > stopspeed`, so braking distance is linear (320 to 0
  costs 67.5 units). `BrakeBack` handles the sub-stopspeed branch separately.
- **Accelerating**: `AccelForward` is the exact solution of `dv/ds = (a - f·max(v, s0))/v`, bisected on the
  closed form — the mirror of `BrakeBack`. `AccelDistance` is its inverse use.

### Corner fitting

For deflection `theta` at a corner with clearance budget `C`:

```
R_geo = C*cos(theta/2) / (1 - cos(theta/2))     apex stays within C of the vertex
R_len = min(availIn, availOut) / tan(theta/2)   neighbouring arcs must not overlap
R     = min(R_geo, R_len)
```

- Three fit passes, because shrinking one arc frees length for its neighbours. With trace functions the arc
  is hull-traced entry → apex → exit and shrunk up to 3 times on a hit, which makes the clearance a prior rather
  than an assumption.
- Below `sv_accelerate * maxspeed * dt` (53.3, the one-frame velocity quantum) an arc is meaningless — the
  fitted radius is under a unit — so the corner emits no arc and the servo pivots through the vertex. Arcs
  only become visible above about 200.
- **Gaps get no arc.** `Suppressed(i)` is `IsGap(i) || IsGap(i - 1)`, where a gap is an air-move launch, a
  teleport or a ladder mount. A launch must be crossed on the launch heading and a landing is entered with
  whatever velocity the flight left. A ladder is a gap because the profile is planar and a climb is almost
  entirely vertical; its segment's cap is lowered to the connection's `ladderSpeed` (200, or 67 when the climb
  ends crouched) so the profile does not believe the bot leaves the ladder at 320. The segment's length is
  still the 2D dismount hop, so `totalTime` under-reports a climb; the planner's cost
  (`ConnectionCostSeconds`) is the honest one.
- The corner *into* a run-up node does get an arc, its outward tangent clamped to half the run-up leg, so the
  last stretch into the launch stays straight while the bot still rounds the turn before it.

### Speed passes

1. **Backward pass** — propagates each node cap through `BrakeBack`; the final waypoint is pinned to 0, launches
   are capped at `launchSpeed`, and entry speeds are also clamped by the cap of the node the segment starts
   from. `NodeMaxSpeed` applies `kDuckingMultiplier` for `requiresCrouch` (except at a launch waypoint) and the
   drift adjustment (`doc/movement.md`, Following a drifting path).
2. **Forward time pass** — integrates real speed against those ceilings to produce `totalTime`, which lands
   within 10% of measured arrival.
3. **Forward reach pass** — carries the reachable speed through `AccelForward` and stores it per segment as
   `vReachEntry`/`vReachExit`. A flight passes the launch speed straight through. `--dump-path` prints it as
   the `reach` column beside `v`, and the two disagreeing is the point: on `test37_walkent` the segment into the
   launch reads `v 320 -> 288` next to `reach 81 -> 288`, the 155° hairpin before it showing as a number.

**Ceilings steer, predictions validate, and they are separate fields.** `vEntry`/`vExit` are ceilings, not
predictions: a segment leaving an arc may report 320 while the bot is still accelerating. Feeding the reachable
ramp into `SegSpeedAt` instead breaks every map — at `s = 0` the ramp is 0, `SolveDrive` gets `wanted = 0`, and
the bot never leaves spawn — and it collapses the accelerate-then-brake trapezoid on long segments (ETAs rise 50%
on `test28_slopecase`). A prediction is a lower bound on nothing; the bot may arrive faster.

**Rides are added after the loop.** An elevator ride's two waypoints share an XY, so its segment has zero length
and is never pushed; the whole ride — travel and the departure stop's auto-return dwell — is invisible to every
pass. A trailing loop adds `elevatorWaitTime + elevatorTravelTime` per ride waypoint. Door and button dwells have
the same gap and are left to `kEtaSafetyMargin`; a ride cannot be, because on `test55_plat` it is twice the
walking time and the ETA-derived test timeout fails.

### Speed within a segment

`SegSpeedAt(seg, local)` is the single definition of "how fast at `local` units into this segment", used by the
forward pass and `SampleProfile`:

```
min(max(vEntry, vExit), BrakeBack(vExit, sLen - local))
```

Hold the ceiling until the physical brake curve for the exit speed drops below it, then follow the curve down.
`max(vEntry, vExit)` rather than `vEntry`, so an accelerating segment is not capped below its own exit; on an arc
the two are equal.

Why not lerp `vEntry` → `vExit`: braking takes 67.5 units from 320 whatever the segment length, so a lerp spreads
the deceleration over the whole segment (a 196-unit approach to a 191-cap corner tells the bot 238 halfway along
instead of 320 and a late brake). Measured ground speeds track the lerp, so the bot never reaches `maxSpeed` on a
straight. The forward pass compounds it: as the lerped limit falls toward zero, `v` hits its 1 u/s floor and the
tail of a segment ending at rest costs about `(sLen / maxSpeed) * ln(maxSpeed)` seconds, inflating `totalTime`
~1.5x.

### Sampling

- `SampleProfile(s)` returns point, tangent, speed, clearance and waypoint index at arclength `s`. Speed comes
  from `SegSpeedAt`, so the servo holds speed and brakes late; `PlanProfile` carries `friction`/`stopSpeed` so
  the sampler can evaluate the brake curve without the physics struct.
- `ProjectOntoProfile(pos, sMin, sMax)` finds the nearest arclength in a window and the unsigned lateral distance
  to it; it skips zero-length segments.
- `ProfileWaypointS` gives a waypoint's arclength, used for the follower's barriers (`doc/movement.md`).

## Air-move simulation (`pm_sim.h`, `pm_sim.cpp`)

### PmPhysics

- `accelerate`, `groundFriction`, `stopSpeed` — `sv_accelerate`, `sv_friction`, `sv_stopspeed`. `PmSimulator`
  never reads them; they are carried for `BuildPathProfile` and `NavFollow::SolveDrive`, where ground physics is
  modelled.
- `edgeFriction` — `sv_edgefriction`. Not applied here: `Bot_DrivePhysics` folds it into `groundFriction` for
  the frames the bot is at a drop (`doc/movement.md`, `NavFollow::SolveDrive`).
- `friction` — `pmove->friction`, the per-entity multiplier, not `sv_friction`.

### PmSimulator

Frame-by-frame HL player movement, **airborne only**: gravity, the pending jump impulse, `AirAccelerate`,
`FlyMove` (clip-hull movement), `CatagorizePosition`.

- `Begin(origin, yaw, speed, jump)` — starts on the ground with velocity from yaw and speed. The jump impulse is
  applied inside the first `Step`, so `OnGround()` is still true when `Begin` returns.
- `BeginAirborne(origin, velocity, yaw, speed)` — starts mid-flight with an explicit velocity, for a state that
  was measured rather than commanded (a ladder dismount, a long jump). Both delegate to a private `Reset`.
- `Step()` — advances one frame; returns false if wedged.
- `Coast(arc, maxTime)` — skips whole frames of free flight on the analytic parabola and returns how many.
  navgen's sampler (`doc/navgen.md`, "Coasting"), `JumpLandsIn` and `Bot_SimulateAirmoveArc` (which fills the
  skipped frames' points from the parabola) call it before every `Step`; a null `PmArcFn` makes it a no-op. It is exact
  because the stepper splits gravity half before and half after `FlyMove`, so frame-boundary positions lie on
  `p0 + v·t − ½g·t²`. It refuses unless airborne with no jump pending and `AirAccelerate` is a no-op
  (`vel·wishdir ≥ min(wishspeed, maxSpeed, 30)`; wishdir is horizontal and gravity vertical, so that holds for the
  whole flight), caps the flight where `CheckVelocity` would clamp `vz`, and stops one frame short of the first
  contact `arc` reports, so `Step` handles every touching frame as before. The parabola math is
  `src/nav/arc_math.h`, templated on the vector and scalar type so `PmSimulator` uses it as `float` and navgen's
  trace as `double`.
- Constants: jump impulse `kJumpImpulse` 268.328; bunny-hop cap 1.7 × maxSpeed; air-accel wish speed cap 30;
  ground normal threshold 0.7; ground trace distance 2. Falls: no damage below 580 u/s, fatal at 1024, damage
  `100 / (1024 - 580)` per unit between (HL's rule; `FallRule` generalizes it, see "Falls are priced per game").

There is no `PM_Friction` and no ground accelerator anywhere in it. That is the right shape for proving where a
launch lands, and the reason a plan can assert an air move the bot cannot begin: `JumpLandsIn` starts the mover
already at launch speed, so a speed the bot cannot reach on the ground looks the same as one it can. The approach
is checked by `BuildPathProfile` and executed by `NavFollow::SolveDrive`.

- **`AirAccelerate`** is `PM_AirAccelerate` with wish speed capped at 30, so it can only nudge a trajectory.
- **`FlyMove`**: reaching the "go along the crease" case with `i == numplanes` means no single plane could be
  slid along. The loop's early break has already stored the sliding velocity, so it must not fall into that case —
  testing `i != numplanes` instead zeroes the velocity of every successful slide.
- **Push fields.** `PM_AddCorrectGravity` folds the vertical component of a `trigger_push` into velocity as an
  acceleration and clears it from the base velocity, so only the horizontal remainder wraps `FlyMove`. That
  horizontal base velocity wraps the move as in `PM_AirMove`/`PM_WalkMove`: added before the clip-and-slide loop,
  removed once the ground check has seen it. The fields come from a `PmPushFn` — `Bot_PushFields` at runtime
  (`doc/movement.md`), the sampler's own at navgen time (`doc/navgen.md`).

## The plan-time world (`bsp_world.cpp`)

The planner's simulations never call the engine. `Nav::LoadBspWorld` reads the map's planes, clipnodes and
models (the same read that checksums the BSP against the `.nav`), and `NavMesh::world` keeps them for the map's
life. `Bot_WorldSnapshot` copies, once per server frame, what an `ignore_monsters` hull trace can hit, following
`SV_ClipToLinks`: every `SOLID_BSP` entity with a brush model (minus `FL_MONSTERCLIP` brushes and zero-size
entities), plus `MOVETYPE_PUSHSTEP` solids as boxes, with origin, angles, `absmin`/`absmax`, owner, whether a hit
counts as an entity (`counts`, the airmove cache's purity rule) and whether `BotBreak::IsTarget` accepts it
(`breakable`). It also keeps the `trigger_push` fields. The snapshot is an immutable `shared_ptr`, so a plan that
captured it reads a consistent world on any thread for as long as it runs.

- **`Nav::TraceWorld`** is `SV_Move`: the world's hull (`models[0].headnode[hull]`) first, then every snapshot
  solid whose `absmin`/`absmax` meets the move's box grown by the hull and 1 unit, each in its own frame
  (`origin` subtracted; rotated by `AngleVectors` when its angles are non-zero, the normal rotated back). A box
  solid is the engine's six-plane box hull. An entity replaces the result when it is all-solid, start-solid or
  nearer, and start-solid is sticky, as in the engine. The descent is `HullTrace::HullCheck`
  (`src/nav/hull_trace.h`), navgen's own, instantiated for `Vector`/`float`.
- **`TraceFilter`** carries the hull, the edict to skip (and what it owns) and `phaseBreakables`. The planner's
  traces (`MakePlanTraces`) phase breakables; the in-flight and avoidance sims do not.
- **`Nav::SafeArcTime`** is navgen's `SafeArcTime` over the snapshot: the arc through the world and the
  solids (a solid's arc is the same parabola in its frame, gravity rotated with it), the arc lowered by
  `kGroundTraceDist`, and `ArcEnterBox` into each push field's box. `Bot_WorldArcFn` wraps it as the
  `PmArcFn` for `Coast`.
- **`Nav::PushAt`** point-tests the origin against each field's clip hull in the sim's hull, the test
  `pfnTraceModel(origin, origin, hull, field)` makes.

Without a loaded mesh there is no world, and `Bot_WorldTraceFn` falls back to `pfnTraceHull`; the planner never
runs then.

**Agreement with the engine.** Measured 2026-10-20 with a compare harness (both traces on every planner ray, since
deleted), 16 bots:

| map | rays | disagree | what |
|---|---:|---:|---|
| xen_dm | 5.6M | ~30 | a `func_door` that moved between the snapshot and the ray, same frame |
| dod_avalanche | 1.0M | 0.15% fraction, 0.04% solid/entity | breakables the planner phases (expected); fixed spots on sloped world brushes where the engine stops 0.05–0.3 of a short move later and the descent backs off to 0 |

The sloped spots are not float error: the same descent in `double` disagrees at the same rays. It is the same
descent navgen builds the mesh with.

**Cost.** Per simulated airmove (cache misses), `rp_plan` ms, same runs as above, interleaved:

| map | engine trace | snapshot, stepped | snapshot + `Coast` |
|---|---:|---:|---:|
| xen_dm | 0.279 / 0.317 | 0.078 / 0.083 | 0.057 / 0.058 |
| dod_avalanche | 0.213 / 0.212 | 0.158 / 0.160 | 0.069 / 0.083 |

Why it wins where a per-ray port lost (`doc/testing.md`, "BSP tracer at runtime"): solids are
culled by box before any descent, the engine's per-call overhead is gone, and the arc replaces most rays. Coasting
changes 0.1–0.3% of runtime sims against fully stepped ones (xen_dm 17 outcomes and 33 landings moved > 1 unit of
18,838; dod_avalanche 8 and 15 of 12,580): one-frame landing differences, long multi-push flights that amplify
float noise, and corner grazes (`doc/navgen.md`, "Coasting").

## Airmove cache (`airmove_cache.cpp`)

### Why whole simulations are cached

`PlanWalker::JumpLandsIn` is the most expensive thing the planner does: each call steps a `PmSimulator` for up to
`kAirmoveSimMaxTime` (6 s) over 30 to 360 frames. Most of those sims are *rejections* — the same edge rejected over
and over, by every bot, on every repath. With engine traces, a 32-bot crossfire server without the cache charged
18,000 hull traces per frame to `ZONE_RP_PLAN`. Since the plan-time world and `Coast`, a miss costs 0.06–0.08 ms on
xen_dm and dod_avalanche, and it is paid on the worker. With the cache off entirely the game thread does not
notice (xen_dm p99 6.6/6.9 ms both ways; dod_avalanche frames ≥ 8 ms 159/183 vs 181/127) and plan latency stays
within run-to-run noise.

### The key

A trajectory is determined by what `sim.Begin` receives plus the physics the stepper reads, so `AirmoveKey` is
exactly that:
- the **sim start origin** — `launchFloor` lifted by `launchLift + kPlanLaunchLift`. Folding the lift in keeps
  `fromAreaId` out of the key; it would only matter through `PlanArea::crouch` choosing the lift;
- `yaw`, `speed`, `jump`;
- `crouch` (`needsCrouch`), which picks the duck or stand hull and push function;
- a hash of the physics fields the stepper reads and that vary: `maxSpeed`, `gravity`, `airAccelerate`,
  `maxVelocity`. `accelerate`, `groundFriction`, `stopSpeed` and `edgeFriction` are never read by `PmSimulator`;
  `friction` is forced to 1 and `frameTime` pinned.

`destAreaId` is **not** in the key. The cache stores the landing point and how the sim ended;
`ApplyAirmoveResult` runs `PointInArea` against the destination afterwards, so one cached trajectory answers for
every destination.

**Exact float equality, no quantisation.** A hit returns bit-for-bit what the simulation would, so the cache
cannot change a plan. It still hits because the launch point is `NearestPointOnGate` of the pulled line against a
fixed gate, which clamps to the same values far more often than it looks like it should: 94% on crossfire, 56% on
xen_dm from 4096 slots.

### Pinned frame time

`JumpLandsIn` overrides `physics.frameTime` with `PmSim::kPlanSimFrameTime` (1/60) on its local copy:
1. `PlanTraceContext::physics.frameTime` is `Bot_FrameTime()`, which jitters every frame, so without the pin no
   plan-time simulation would repeat and the cache could never hit.
2. More importantly, navgen samples every `CONN_AIRMOVE` at a fixed 1/60 (`AirmoveParams::frameTime`); integrating
   at the live step drifts from the data being validated, further the more loaded the server.

The pin is scoped to the simulation — `BuildPathProfile` and `AirmoveLaunchLead` keep the live frame time, because
they model how many real frames of acceleration the bot gets — and is a no-op in navgen, whose physics already come
from `PM_MoveSimulator::Physics()` at 1/60.

### What may be cached

`PmTrace::hitEntity`, set by `Nav::TraceWorld` from the `WorldSolid::counts` of the solid that decided the
result, marks a hit on something that can move. A hit counts as **world** when:
- it is the world hull (worldspawn);
- it is a `SOLID_BSP` brush with `FL_WORLDBRUSH` (merged into the world; can never move or be removed),
  except a barrier wall (`crossbot_nav::IsBarrierWall`), which `func_wall_toggle` makes removable;
- `crossbot_nav::IsBakedBlocker` accepts it — the **same predicate navgen uses to bake entities into the mesh**
  (`baked_blockers.h`, called from both `navgen::IsBlockingEntity` and the runtime trace): `func_wall`,
  `func_healthcharger`, `func_recharge`, an unnamed `func_wall_toggle` not starting off, `func_conveyor` not flagged
  non-solid. The mesh is already committed to these being solid, so treating them as world is consistent with
  it. Keeping it one function matters: if the two lists diverge, the planner caches against geometry the mesh
  disagrees about.

The snapshot holds what an `ignore_monsters` trace sees, so players and monsters never appear, and the planner
phases breakables. Doors, plats, trains and pushables are what is left, and they make a sim uncacheable. `PmSimulator` ORs the flag across every trace into `m_touchedEntity`,
and also sets it when the `trigger_push` lookup returns a non-zero vector (the push snapshot depends on live entity
state). `JumpLandsIn` inserts only when `TouchedEntity()` is false.

**The one case it gets wrong.** That rule guarantees the *recorded* trajectory did not depend on a mover; it does
not guarantee no mover later moves *into* it (a jump recorded over an empty shaft is stale once the lift rises).
`crossbot_airmove_verify 1` re-simulates every hit and compares. Every measured mismatch is this case, which is
what establishes the key is sufficient for static geometry:

| map       | mismatches | of hits  | rate  |
|-----------|-----------:|---------:|------:|
| crossfire |        106 |  271,339 | 0.04% |
| xen_dm    |      2,519 |  254,147 | 0.99% |

Two deliberate choices push the rate up: accepting baked blockers caches trajectories that touch brush entities,
and aging-LFU keeps hot entries alive longer, giving the world more time to move under them. The divergence is left
in: a wrong answer means the bot attempts a jump a platform now blocks, reports `airmove aborted` and repaths — a
routine path that already fires far more often from run-up shortfall. If it ever needs to go lower, the levers are
a staleness TTL or refusing trajectories whose bounds pass near a mover; both cost hit rate.

### Structure and eviction

- **Fixed table**, `kAirmoveCacheSlots` = 65536 in `airmove_cache.h` — a compile-time constant, not a cvar; the
  table is a plain member array with no allocation or resize path. About 3.3 MB, so it does not fit in L2. Why not
  4096 slots (210 KB): dm_centralcore_v2_one evicts 37k of 41k inserts with 10 bots, a failed whole-mesh search
  inserting 1–2k one-off sims per attempt.
- **Lookup is a hash, never a scan:** an FNV-1a-style mix over the five float bit patterns, the physics hash and the
  jump/crouch byte, masked to the table. Open addressing with a linear probe of up to `kAirmoveCacheProbe` (8)
  slots, one or two cache lines. Lookup stops at the first unused slot, which is sound because entries are only
  ever wiped wholesale by `Clear()`.
- **Eviction** happens only when all 8 probe slots hold other keys. Each slot carries a saturating `freq` (bumped on
  every hit) and a `stamp` (the global lookup tick at its last hit); the victim minimises

  ```
  score = freq >> ((now - stamp) / kAirmoveAgeStep)     // 0 once the shift reaches kAirmoveMaxDecay
  ```

  ties going to the older stamp. Aging LFU, evaluated lazily: it protects entries used a lot *and* entries used
  recently, while a once-hot entry loses protection exponentially instead of squatting as under plain LFU. A new
  entry starts at `freq = 1` and is the first thrown out if never hit again, so a burst of one-off launches cannot
  flush the working set.

Why not fixed-victim eviction (always clobber slot `base`): two hot keys sharing a base ping-pong forever.
At 4096 slots, 2-minute 32-bot runs (hit rate / evictions):

| map           | fixed victim    | aging LFU       |
|---------------|-----------------|-----------------|
| crossfire     | 86.2% / 47,021  | 93.6% / 13,300  |
| xen_dm        | 42.1% / 204,364 | 55.7% / 119,702 |
| rocket_frenzy | 31.3% / 245,222 | 43.0% / 196,835 |

For scale, crossfire at 4096 slots with aging LFU reaches 93.6% against 95.1% for 65536 slots with fixed victims.
On the harder maps the ceiling is neither capacity nor eviction but `uncacheable` sims that touched a real mover;
accepting baked blockers halves that share on xen_dm and rocket_frenzy.

### Ownership

The instance is `g_airmoveCache` in `bot_move_nav.cpp`, attached to `PlanTraceContext::airmoveCache` by
`MakePlanTraces`. Only the plan worker reads or writes it (every `JumpLandsIn` with traces runs there), so it has no
lock; the game thread only sets the verify flag and prints the counters. navgen leaves the pointer null, which keeps
its multi-threaded sampler race-free — `waypoint_plan.cpp` is compiled into both targets.
`Bot_ResetAirmoveCache()` clears it from `BotManager::ServerActivate` and `ServerDeactivate`, after `FlushPlans`. `crossbot_airmove_cache_stats` prints occupancy, hit rate,
inserts and evictions (`doc/console.md`).
