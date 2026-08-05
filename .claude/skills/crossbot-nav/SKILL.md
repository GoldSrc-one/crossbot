---
name: crossbot-nav
description: How CrossBot plans and follows a route (src/nav/, src/bot/) and how to work out why a bot does not reach its goal. Use this whenever work touches the planner or the follower — A* over the area graph, string pulling and funnels, air-move launch simulation, path profiles and speed, waypoint following, Repath, ETAs, path cost. Use it above all when a bot misbehaves at runtime: stops short, stalls in place, repaths in a loop, walks into a wall, misses a jump, takes a route that looks wrong, or reports no route at all — telling a mesh defect from a planning one from a steering one is the first move and this skill has that triage. Also use it before changing anything about path cost or the search, because a cost change silently re-ranks routes and only an end-to-end run can see the damage. Triggers on plan_search.cpp, waypoint_plan.cpp, path_profile.cpp, path_follower.cpp, PlanWalker, PlanRoute, JumpLandsIn, NavFollow::Track, crossbot_nav_status, test_nav_goal.ps1, --dump-path and --dump-airmove.
---

# nav: routing, following, and why the bot didn't get there

## Read these first, they are not duplicated here

- `AGENTS.md` — build (`& .\build.bat`, nothing else), house rules, the command lines.
- `doc/nav.md` — what each file in `src/nav/` does. The authority on behaviour: the search, the
  funnel, the profile, the follower, the physics constants.
- `doc/bot.md` — `Repath`, `Bot_MoveNav`, the air-move approach, and the field-by-field reading of
  `crossbot_nav_status`.
- `doc/testing.md` — what `test_nav_goal.ps1` and the sweeps actually do, and which sweep a given
  change needs.
- The `crossbot-navgen` skill — for when triage lands on the mesh rather than the route.

This skill is the part that isn't in any of them: the triage, and the traps. When you learn something
durable, put behaviour in `doc/nav.md` or `doc/bot.md` and technique here. Never in source comments —
they get deleted.

## The shape of the planner

**There is one pass and one funnel.** `NavPlan::PlanRoute` is A* where every search node carries a
`PlanWalker`: the funnel state, the waypoints emitted so far, the running cost. Expanding an edge
copies the parent's walker and calls `Cross`. The goal node's walker *is* the finished plan — nothing
is replayed afterwards, so the plan cannot disagree with what the search costed. There is no
graph-only `FindPath`, no separate `BuildWaypointPath`, and no retry loop.

Three consequences are worth holding in your head, because most confusion about this code comes from
assuming the old two-phase shape:

**Cost is seconds, everywhere.** `g` is string-pulled distance over `kNavRunSpeed` plus each
connection's `costSeconds`, which is *additive time* — door waits, elevator rides, air-move flight.
It is not a distance multiplier. Anything you add to the cost model has to be in seconds or it will
quietly outrank everything else.

**Discontinuities are funnel barriers.** Air moves, teleports, doors, buttons and triggers flush the
funnel before the crossing and re-anchor it after. So a string pull never crosses a landing, and a
forward search resolves each air move exactly once, when it traverses it. The natural worry — "the
string pull could later cut a corner through a landing I already planned" — cannot happen. If you add
a connection type where the bot's position jumps or is gated, it must flush too, or the funnel will
pull straight through it.

**A failed edge is just an edge that doesn't relax.** When `Cross` cannot simulate a launch it returns
false, the neighbour keeps its old cost, and the search carries on. That is the whole recovery
mechanism. `PlannedRoute::airmovesRejected` counts it, and a nonzero count is normal and healthy.

## First: mesh, route, or execution?

These three fail in completely different places and the fix for one is never the fix for another.
Work outward from the cheapest check.

**1. Does navgen plan it offline?**

```powershell
.\build\tools\navgen\navgen.exe maps\test45_slopes2.bsp --dump-path
```

`debug path:` gives the route and `debug search:` what it cost to find. No route at all, or a route
that ends somewhere unexpected, means the problem is upstream of the follower. A nonzero
`rejected in search` is *not* by itself a defect — it means the search refused a launch and routed
around it, which is the design working.

**2. If there is no route, is it the mesh?**

Almost always. Go to the `crossbot-navgen` skill — a missing connection, an area that fragmented, a
seam the step filter dropped. The planner can only search edges that exist. Before you go, check
one-way-ness: `--dump-conns` shows it, and a map dense in one-way drops (crossfire is 64% air moves,
mostly one-way) can genuinely have no route *back* along the way the bot came. That is a real
property of the map, not a bug to fix in the search.

**3. Does the *runtime* plan it?**

This is the step that gets skipped, and it is where several hours have gone. **`--dump-path` is not
the route the bot takes.** navgen plans from the first spawn to the `info_target`; the runtime plans
from wherever the bot is to whatever goal it was given, frequently the opposite direction. Get the
runtime's own answer:

```
crossbot_nav_status
```

`planned=` is the area sequence it chose and the waypoint list is the plan it is following. If those
are present and sane, planning is done and the rest is steering.

**4. Plan is right, bot doesn't follow it.**

Now you are in `path_follower.cpp` / `bot_move_nav.cpp`, and `doc/nav.md` describes the tracker's
projection, aim and velocity servo. Read the signatures below before touching anything.

## Reading a status line

`crossbot_nav_status` is the whole diagnosis if you read the pairs rather than the fields.
`doc/bot.md` lists them; these are the three patterns that actually recur.

**Stalled at full drive** — `vel≈(0,0,0)` with a large `move=(267,-42)`, `onground=1`, waypoint index
not advancing. The bot is pressing and not moving, so it is not lost, not idle and not mis-aimed. Two
different causes produce an identical line and you have to tell them apart:

- *Against geometry.* Look at where it is relative to the next waypoint — often only 10–20 units short —
  and what is between them. A hard stop on a plane shared by two areas usually means the plan is missing
  a flag for the crossing, not that the mesh is wrong.
- *Below the friction threshold.* `move` is a **command**, and the engine turns a command into a wish
  speed that friction can outrun. Under `sv_stopspeed` the drag is constant, so nothing moves until the
  wish speed clears `stopSpeed * friction * edgeFriction / accelerate` — 40 on open ground, 80 at a
  ledge. Everything that drives the bot must go through `NavFollow::SolveDrive`, which is built to
  sidestep exactly this; a crawl at 2–8 u/s means something bypassed it. `doc/nav.md` has the derivation.

Ducking hides both: `PM_Duck` scales the command by `kDuckingMultiplier` after the engine clamps it, so
a `move` of 270 is really a wish speed of 90.

**Thrashing** — `actual=` cycling, `[11,12,13,11,12,13,...]`. The bot is alternating between two routes
into the same dead end; the maneuver at the junction is what cannot be executed, and the route change
is a symptom. A failed air move now penalizes its connection, so a cycle should break after one lap —
`crossbot_nav_penalties` says whether it is learning, and `penalized=` on the `Replan` line says
whether the new route actually avoids the bad edge. A cycle that does *not* decay means either the
penalty never landed (check `conn -1` on the landing line — the launch waypoint carried no connection
index) or every remaining route is equally bad. `doc/bot.md` has the worked example.

**Short of the goal** — `hasGoal=0`, `waypoint=0/0`, and a distance the test calls a failure. This is
ambiguous: identical output for "arrived and cleared the goal" and "gave up". **Check the clamped goal
first.** `Repath` clamps the requested point into its area, and the plan's last waypoint is the clamped
point, so when the goal lies outside the mesh the bot arrives correctly at a point the caller still
counts as short. `--dump-path`'s final `goal` waypoint tells you immediately.

## Traps that have cost real time

**A `sim miss` in `--dump-airmove` is not a failure.** It is an edge the search refused and routed
around. The count matches `rejected in search` on the `debug path:` line exactly; if it ever doesn't,
*that* is worth investigating.

**Air-move span must not be charged twice.** `ConnectionCostSeconds` already folds `flightTime` into
`costSeconds`, and door `openSeconds` likewise. The walker pushes launch and landing waypoints with
`PushWaypoint(wp, false)` so the airborne hop never enters the string-pulled length. Adding
`flightTime` in the walker "because the jump takes time" double-charges it and makes the planner
avoid jumps for no reason. The correct total is `costSeconds` counted once per edge.

**A simulable plan is not an executable plan.** The search proves a `PmSim` run from the funnel's
launch point, at that yaw and speed, lands in the destination. It proves nothing about getting there:
`PmSim` models no ground physics at all, and `JumpLandsIn` starts the mover already at the launch
speed. So both halves of the approach are outside the proof — the follower has to *arrive* having
tracked an arc chain, with whatever speed it actually has (`NavFollow::Track` derates when off the
chain, so a late rejoin undershoots), and it has to be able to reach that speed on the ground in the
first place. Most "the planner made a bad jump" reports are one of these two.

**A flag only exists where the funnel put a waypoint.** Anything attached to a corner — crouch today,
whatever you add tomorrow — is silently dropped whenever the string pull runs straight past that gate,
and running straight past is the *normal* case: a gate becomes a corner only when the funnel
constricts. If a property describes a stretch of the route rather than a turn in it, it has to be
propagated across the segments the flush spans, and `FlushFunnelTo` has to hand the tail back to the
caller for the waypoint the caller pushes. `doc/nav.md` describes how crouch does it.

**A cost-model change re-ranks routes, and the sweep cannot see the damage.** Route changes land on
exactly the maps with interesting geometry, which are exactly the maps the follower is worst on. The
plan sweep will show a shorter, cheaper, entirely valid route while the bot fails end to end. Capture
an end-to-end baseline *before* the change, not after — after is too late, the old binary is gone, and
you are left unable to say whether anything regressed.

Runtime penalties are the exception, and the exception is worth knowing because it inverts the check.
`Connection::runtimePenalty` is 0 until a bot fails and navgen builds its own graph, so the route sweep
**must come out empty** — an empty diff is the evidence, not a formality, and a nonempty one means the
penalty leaked into offline planning.

**`test_nav_goal.ps1` prints its verdict with `Write-Host`.** `... | Select-String '=== RESULT'`
captures nothing — the line reaches the console but never the pipeline. Redirect the whole run to a
file and parse that. Its exit code is also easy to lose through a pipeline; parse the transcript.

**Building while a test holds HLDS fails only the copy step.** `build.bat` still produces
`build\crossbot_mm.dll`, so a rerun says `ninja: no work to do` and everything looks fine while the
deployed plugin is stale. When a runtime result contradicts the source, check the deploy line before
anything else.

## Sweeping a planner change

`doc/testing.md` says which sweep covers what and gives the commands. What it does not say is how to
act on the diff.

**Run the mesh sweep even though it must come out empty.** A change under `src/nav/` touches no
pipeline stage, so a nonempty diff means you changed something you did not intend to.

**Use the route diff to pick the end-to-end list.** The maps whose `wp[` lines changed are the only
ones whose runtime behaviour can have changed; the rest are byte-identical and cannot regress. Run
those through `test_nav_goal.ps1` plus one easy map as a control — much cheaper than the full suite,
and it finds the same regressions.

**That shortcut does not exist for a follower change**, because there is no diff. Pick the maps from
what the change touches instead: the air-move approach means every map with an air move, which is most
of them. There is no cheap version of this.

**Getting a *before* binary is the awkward part.** `git stash` on the file you edited will not do it
when the tree already has other uncommitted work in the same headers — the stashed version stops
compiling against the rest. Copy the files aside and revert your own hunks by hand, or accept that you
cannot answer the regression question and say so plainly rather than implying the change is clean.

**When the change is gated by a tuning constant, set the constant to its identity value instead.**
Zero for an added penalty, 1 for a multiplier: one edit, one rebuild, and the binary reproduces the old
behaviour exactly while keeping the new logging — which is usually what makes the old behaviour legible
in the first place. It answers the before/after question in two runs with no git surgery, and it also
proves the constant is the only thing doing the work. Setting `kConnFailPenalty` to 0 turned
test50_stairs from a 2 s pass back into a 12 s timeout looping conn 38 → conn 35 → conn 38, which no
amount of reading the diff would have shown. Put the constant back and rebuild before you do anything
else — a neutered binary that looks built is the same trap as a stale deploy.

A refactor that is meant to preserve behaviour should be checked *on its own*, before the behavioural
change goes on top. Extracting `PlanWalker` was verified byte-identical across all 52 maps before the
search was touched, which is the only reason the later diffs could be trusted.

## Judging whether a route got better

Fewer waypoints is not better. Shorter is not better. The cost model is seconds, so compare seconds:
`debug profile:` reports the modeled traversal time for the chosen route, and that is the number the
search was trying to minimise.

Even then, hold the conclusion loosely until the bot has run it. A route can be genuinely faster in
the model and still fail, because the model does not know what the follower is bad at. State what you
measured — plan time, plan shape — and do not upgrade it to "better" on offline evidence alone.
