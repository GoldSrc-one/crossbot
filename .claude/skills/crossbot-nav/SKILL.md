---
name: crossbot-nav
description: How CrossBot plans/follows routes (src/nav/, src/bot/) and why bots dont reach goals. Use for planner/follower work: A* over area graph, string pulling, air-move simulation, path profiles, waypoint following, Repath, ETAs. Use when bot misbehaves at runtime: stops short, stalls, repath loops, walks into walls, misses jumps, wrong route, or no route — triage (mesh vs plan vs steer) is first move. Also before changing path cost/search — cost change silently re-ranks routes. Triggers on plan_search.cpp, waypoint_plan.cpp, path_profile.cpp, path_follower.cpp, PlanWalker, PlanRoute, JumpLandsIn, NavFollow::Track, crossbot_nav_status, test_nav_goal.ps1, --dump-path, --dump-airmove.
---

# nav: routing, following, why bot didnt get there

## Read first (not duplicated here)
- AGENTS.md — build (& .\build.bat), house rules, commands
- doc/nav.md — src/nav/ behaviour authority (search, funnel, profile, follower, physics)
- doc/bot.md — Repath, Bot_MoveNav, air-move approach, crossbot_nav_status field guide
- doc/testing.md — what test_nav_goal.ps1 & sweeps do, which sweep a change needs
- crossbot-navgen skill — for mesh triage

This skill: triage & traps. Behaviour → doc/nav.md/doc/bot.md, technique here. No source comments.

## Planner shape (single pass)
One A* + funnel pass. NavPlan::PlanRoute carries PlanWalker (funnel state, waypoints, cost) per node. Expanding edge = copy walker + Cross. Goal walker is the plan — no replay. Three consequences: Cost = seconds everywhere — g = string-pulled distance / kNavRunSpeed + costSeconds (additive time: door waits, elevator, air-move). Not a distance multiplier. Discontinuities flush funnel — air moves, teleports, doors, buttons re-anchor funnel. String pull never crosses landing. Failed edge = no relax — Cross returns false, neighbour keeps old cost, search continues. airmovesRejected counts it; nonzero = normal.

## Triage: mesh, route, or execution?
Work outward from cheapest check:
1. Offline plan? navgen --dump-path → debug path: = route, debug search: = cost. No route or unexpected end = upstream of follower. rejected in search ≠ defect (search routed around).
2. If no route, mesh? Almost always. Go to crossbot-navgen — missing connection, fragmented area, dropped seam. Check one-way-ness in --dump-conns (crossfire 64% one-way air moves).
3. Runtime plan? --dump-path ≠ bots route. navgen plans spawn→target; runtime plans bot→goal (often opposite). Get runtime answer: crossbot_nav_status — planned= = area sequence, waypoints = plan following. If present & sane → steering issue (path_follower.cpp/bot_move_nav.cpp, see doc/nav.md).
4. Plan right, bot doesnt follow → doc/nav.md describes tracker projection, aim, velocity servo.

## Reading crossbot_nav_status (pairs, not fields)
- Stalled at full drive — vel≈(0,0,0) with large move, onground=1, waypoint stuck. Two causes: Against geometry (waypoint 10-20 units away, hard stop on shared plane → missing crossing flag); Below friction threshold — move is command, engine turns to wish speed. Under sv_stopspeed drag constant; nothing moves until wish > stopSpeed * friction * edgeFriction / accelerate (40 open, 80 at ledge). Must go through NavFollow::SolveDrive; 2-8 u/s crawl = bypassed it. Ducking hides both (PM_Duck scales after clamp).
- Thrashing — actual= cycling [11,12,13,11,12,13...]. Failed maneuver at junction, route change = symptom. Penalty should break after one lap — check penalized= on Replan line. Cycle not decaying = penalty never landed (launch wp had conn -1) or all routes equally bad.
- Short of goal — hasGoal=0, waypoint=0/0, test calls failure. Ambiguous: arrived vs gave up. Check clamped goal first — Repath clamps into area, plan ends at clamped point. --dump-path final goal waypoint tells you.

## Key traps
- sim miss in --dump-airmove ≠ failure — edge refused & routed around. Count = rejected in search.
- Air-move span not charged twice — costSeconds already includes flightTime; walker PushWaypoint(wp, false) excludes airborne hop from string-pulled length. Adding flightTime in walker double-charges.
- Simulable ≠ executable — PmSim starts at exact launch point/yaw/speed; follower must arrive tracking arc chain with whatever speed it has. Track derates off-chain → late rejoin undershoots. Most bad jump = approach or ground speed issue.
- Flag only at funnel waypoints — corner properties (crouch, future flags) dropped when string pull runs straight. Stretch properties must propagate across flush segments; FlushFunnelTo hands tail back.
- Cost-model change re-ranks routes; sweep blind — route changes land on maps follower is worst on. Capture end-to-end baseline before change. Runtime penalties exception: sweep must be empty (penalty only runtime, navgen builds own graph).
- test_nav_goal.ps1 verdict via Write-Host — Select-String captures nothing. Redirect to file & parse. Exit code lost in pipeline.
- Build while test holds HLDS fails only copy — build.bat produces build\crossbot_mm.dll, rerun says ninja: no work to do but deployed plugin stale. Check deploy line before trusting runtime.

## Sweeping planner change
- Mesh sweep must be empty — nonempty = unintended pipeline change.
- Route diff picks end-to-end list — maps with changed wp[ lines = only ones that can regress. Run those + one control through test_nav_goal.ps1.
- No shortcut for follower change — no waypoint diff. Pick maps from what change touches (air-move approach = most maps).
- Before binary = awkward — git stash fails with other uncommitted work. Copy files aside & revert hunks by hand, or admit cant answer regression.
- Tuning constant → set identity value (0 for penalty, 1 for multiplier): one edit, one rebuild, old behaviour exact + new logging. Proves constant is only driver. Put back before proceeding.
- Refactor meant to preserve behaviour = check alone first — PlanWalker extraction verified byte-identical across 52 maps before search touched.

## Judging route improvement
Fewer waypoints ≠ better. Shorter ≠ better. Compare seconds — debug profile: = modeled traversal time (what search minimises). Even then: model doesnt know follower weaknesses. State what measured (plan time, shape) — dont upgrade to better on offline evidence alone.
