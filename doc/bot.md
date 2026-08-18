# Bot System Comments

## bot_items.cpp

The first goal source the bot has. Everything else in `src/bot/` executes a goal; this decides what
the goal is. It runs only where `Bot_MoveNav` finds `hasGoal == false`, which is both "just arrived"
and "just gave up", so a gather goal never preempts one in flight. `crossbot_gather 0` turns it off
and restores the old stand-still-until-commanded behaviour — `test_nav_goal.ps1` sets it, for the
reason under "Why the nav sweep disables it" below.

### Selection is two searches, not one

Phase 1 is `Nav::AreaCosts`, a plain Dijkstra over the area graph that prices every area from the
bot's own. Phase 2 is the existing `Repath`, which does the real A* with air-move simulation to the
chosen item's point.

Doing it in one pass is the obvious design and it is wrong here. `PlanRoute` carries a full
`PlanWalker` — including its waypoint vector — on *every* node it touches, because the funnel state
is what makes an air move's feasibility depend on the route taken to reach it. A goal-directed plan
on crossfire expands ~165 areas; a goal-less one expands all ~500, and pays a `PmSim` run per
air-move edge on each. That is a third of a second to answer "which way do I walk", every time a bot
picks up anything.

The cost of splitting is that phase 1 can name an item phase 2 cannot reach — phase 1 does not know
that the air move into that area needs a run-up the bot cannot get. That is what
`BotItems::OnRepathFailed` absorbs: the item goes into the ignore table for 30 s and the next
selection picks something else. Without it the two phases disagree forever and the bot re-picks the
same unroutable item every frame.

The ignore table is for the disagreements that are *route-dependent*. A disagreement that is a static
property of the mesh has no business reaching it, because the 30 s timer then expires into the same failure
forever: `AreaCosts` applies the activation gating that does not need a walker (`doc/nav.md`,
`GateAllowsCrossing`), so an area behind a door the bot can never open from this side is priced
`kUnreachableSeconds` and `crossbot_gather_debug` prints `no route` instead of a cost. On `crossfire` that is
the 12-item secret cache in area 8 — and those 12 are the only items on the map that should read `no route`.

Phase 1 must charge for **`travelSeconds` as well as `costSeconds`**, and getting that wrong is not
a subtle bug. `Connection::costSeconds` is a surcharge — door waits, flight time, ride time — and it
is **zero on a plain walk connection** (see `doc/nav.md`, travelSeconds). A Dijkstra over
`costSeconds` alone therefore prices most of the map at 0 and "nearest" degenerates into whichever
area the heap happens to settle first, which is roughly area-index order. The symptom is a bot that
walks past the battery at its feet to fetch something across the map, and items in high-numbered
areas that are never chosen at all. `crossbot_gather_debug 1` prints the cost per candidate, which
is how that was found: every far item read `0.0s away`.

Candidates are then ranked by `AreaCosts[area] + |item − reference| / kNavRunSpeed`, where the
reference is the bot's own origin for its current area and the area centroid elsewhere. Ranking by
area alone leaves ties between every item in the winning area, and the tie in the bot's *own* area
is exactly the "it is right next to me" case that matters most.

### One ignore table, not two

"Respawning until T" and "Repath could not reach it, retry after T" are the same fact — *do not
consider this entity before T* — so `itemMemo` serves both. The `fullMemo` list is separate because
it is keyed by classname, not entity: being full of 9mm means every 9mm box is pointless, not just
the one that was touched.

Both are per-bot. `fullMemo` has to be, since it describes the bot's own inventory. `itemMemo` is
per-bot by choice: a bot only knows about pickups it made or saw, so two bots can and do walk at the
same battery, and neither has an accurate global respawn clock. That is the humanlike answer and it
matches the existing per-bot `activationMemo`.

`fullMemo` is cleared on respawn and `itemMemo` is kept — a fresh spawn has an empty inventory, but
the items the bot took are still on their respawn timers.

### Walking in on something that came back

`RefreshMemos` runs every frame and drops any *respawning* entry whose item is in the bot's current
area and available again: "it is already here, so it must have come back sooner than I thought". It
matters because the clock starts when the bot **noticed**, not when the item was taken — a bot that
sees an item is gone from across the room, or arrives to find it gone, starts a full 20 or 30 s from
that moment, and by then the real respawn may be seconds away.

Only entries marked `respawning` are eligible. The ignore entries written by outcome D and by
`OnRepathFailed` share the table and must **not** be cleared this way: the bot standing in the area
is precisely the situation that produced them, so clearing on arrival would reinstate the
full-framerate spin the ignore exists to break.

The memo stores the item's `kind` and `areaId` at the time it was written, so this is a handful of
integer compares per frame with no geometry — the alternative, resolving each remembered entity to
an area, is a linear scan of every area per entry.

Weapons never take this path and do not need to. `CBasePlayerItem::Respawn` creates a **new edict**
for the copy, so the handle in the memo goes stale, `CBotEntHandle::Get` returns null and the entry
is pruned; the respawned weapon is a fresh entity nobody has a memo for. `item_*` and `ammo_*`
reuse their edict and are what this actually serves.

### The four outcomes

A gather goal ends in exactly one of four ways, and three of them are not "the follower arrived".

**A — touched it.** The `pfnTouch` post hook. Because it is a *post* hook the game DLL has already
run its touch handler, so the entity's own state is the verdict: `SOLID_NOT`, `EF_NODRAW`,
`FL_KILLME` or freed means picked up, anything else means the bot could not use it and the classname
goes in `fullMemo`. `FL_KILLME` is the one that is easy to miss — `UTIL_Remove` only flags the
edict and the engine frees it at the end of the frame, so at post-touch time a removed item is
normally flagged rather than free.

**B — arrived and it was already gone.** No touch will ever fire for an item that is already
`SOLID_NOT`, so nothing would end the goal. Detected on `currentAreaId == goalItemArea`.

**C — saw from a distance that it was gone.** A trace from the entity to the bot's eye, ignoring the
entity, that comes back with `tr.pHit == bot.pEdict`. **The trace must use `dont_ignore_monsters`**
or it passes straight through the player and never reports it, which reads as "no line of sight" and
the bot walks the whole way for nothing. Throttled to 4 Hz.

**D — arrived, touched nothing, item still there.** Not a case anyone designs for, and the one that
bites: the bot arrives, no touch fires because the item is above it or on a lip, the goal clears,
selection picks the same item, the bot walks zero distance and arrives again — a full-framerate spin
with no stall detector to catch it, because the bot is exactly where it wanted to be. The item goes
into the ignore table for 30 s. Three bots on `test56_recharger` hit this within seconds, on the
`weaponbox` entities they dropped when they killed each other: those spawn in mid-air where the
player died and the bot cannot reach them.

### Resolving an item to an area

**`Nav::FindAreaForPoint` is the wrong function here.** It is deliberately forgiving — after the
exact half-space test it falls back to nearest-centroid-with-AABB and then to global nearest
centroid, so it essentially never returns null. An item inside a wall would snap to whatever area
happened to be closest and the bot would route confidently to nothing.
`Nav::FindAreaContainingPoint` is the strict half of it, added for this.

Area z-space is duck-origin based (navgen builds in hull 3) and an item's `v.origin` is at its base,
so the probe is `origin + 18` first and the raw origin as a fallback.

**Chargers have no origin at all.** They are brush entities: `v.origin` is `(0,0,0)` and the real
position is `(absmin + absmax) * 0.5`. That centre is inside the wall, so it never resolves to an
area either. The use spot is found by scanning areas whose bounds come within 96 units of the centre
and taking the closest point that is within 48 of it. 48 rather than HL's full 64-unit USE radius
because `NavFollow` stops within `goalRadius` (12) of the goal *horizontally* and can stop on the far
side, and 64 + 12 is out of range — the bot would stand there pressing USE at nothing until the
15 s timeout.

### Why the nav sweep disables it

`test_nav_goal.ps1` polls `crossbot_nav_status` every 2 s and treats `hasGoal=0` as arrival. Gather
re-arms `hasGoal` on the frame after arrival, roughly 20 ms later, so the poll never observes it and
**every map in the sweep times out**. `crossbot_gather 0` in the boot commands keeps the sweep
measuring what it measured before. Use `scripts/test_items.ps1` to exercise gathering instead.

### Reading crossbot_gather_debug

`crossbot_gather_debug 1` makes every selection print the whole scan: one line per item entity with
its resolved area and either its cost in seconds or why it was skipped (`taken`, `empty`,
`respawning for Ns`, `ignored for Ns`, `full for Ns`, `no nav area`, `no route`), with `PICKED` on
the winner. It is the only view that distinguishes the four ways an item can be invisible to a bot,
and the costs are what proves the phase-1 search is sane — if far items read `0.0s away`, the cost
model is broken, not the mesh.

It is deliberately verbose: ~130 lines per selection on crossfire, at up to 2 Hz per bot while a bot
has nothing to go for. Leave it off unless reading the log.

## bot_move.cpp

### Bot_DecomposeMove
PM_WalkMove and PM_AirMove both flatten forward/right and renormalize before use, so whatever pitch and roll the engine puts in pmove->angles drops out and the basis is just the yaw.

### Bot_NormalizeAngle
Wraps an angle into (-180, 180]. Lives here rather than in bot_move_nav.cpp so both movement
translation units share one copy.

### Bot_RunPlayerMove
The engine halves any usercmd longer than 50 ms and runs PM_PlayerMove once per half, so a
button held in a long command is re-read after the bot has already moved. IN_JUMP is the one
button where that matters: PM_LadderMove reads it with no oldbuttons gate and answers by
shoving the player 270 units off the ladder plane, so a jump that merely *starts* out of a
ladder's reach ejects the bot the moment a later sub-step brings it into contact.

Every actuation goes through here so a jump costs the same one frame of held button at any server
framerate. Below the split threshold this is exactly the single call it replaces.

**IN_DUCK and IN_JUMP in one command is a long jump, not a crouch jump.** `PM_Jump`
(`pm_shared.c:2554`) reads `cansuperjump` from the `slj` physinfo key — set by `CBasePlayer` the moment
the player picks up `item_longjump` — and if the command carries IN_DUCK with `flDuckTime > 0` and
`Length(velocity) > 50` it replaces the jump with `forward * 350 * 1.6` horizontally and
`sqrt(2*800*56)` up. `PM_Duck` sets `flDuckTime = 1000` on the press with the comment *"Use 1 second so
super long jump will work"*, so the very frame a crouch jump presses both buttons is inside the window.
`PmSim` models `kJumpImpulse` and the commanded speed, so nothing in the plan knows about 560 u/s: the
bot flies roughly twice as far as its arc and lands past the target area. Nothing was wrong on maps
without the module, which is why it only showed up once bots started gathering on `crossfire`.

So a command that carries IN_JUMP drops IN_DUCK, but **only while the bot holds the module** — the check
is one `pfnGetPhysicsKeyValue` call, and gating on it keeps every map without an `item_longjump` sending
byte-identical commands.

Delaying the duck by one frame costs nothing, and the reason is the call order. `PM_PlayerMove` runs
`PM_Duck` (2982) *before* `PM_Jump` (3038), and a duck pressed while still on the ground cannot finish
inside that frame — `PM_FinishDuck` needs either the 0.4 s `TIME_TO_DUCK` ramp to elapse or
`onground == -1`, and the jump that clears `onground` happens after `PM_Duck` has already returned. So
the launch frame is simulated with the standing hull whether or not IN_DUCK was in it, and the first
airborne frame finishes the duck either way: pressed on the launch frame it finishes because `bInDuck`
survives into the air, pressed one frame later it finishes on the press itself. The hull timeline
`JumpLandsIn` assumes — `duckTrace` for the whole arc — is met from the same frame as before. Jump
*height* never depended on the duck either; both branches of `PM_Jump` give the same 45-unit impulse.

## bot_move_nav.cpp

### Constants
- kFullDrive: Over maxspeed on purpose: PM_CheckParamters rescales the forward/side pair down to maxspeed, so this just means "full speed in exactly the commanded direction".
- kAimVelocitySpeed: Below this the velocity carries no usable heading, so the aim holds where it is rather than snapping about on numerical noise.
- kEdgeProbeAhead / kEdgeProbeDrop / kStandHullMinsZ / kDuckHullMinsZ: `PM_Friction`'s edge test, copied literally — 16 units ahead along the 3D velocity, dropped 34 from foot height. They are the engine's numbers, not tuning knobs; changing one desynchronises `AtFrictionEdge` from the friction it is trying to predict.

### WriteTraceLog
Writes bot trajectory trace to file specified by crossbot_trace_path cvar (defaults to valve/addons/crossbot/trace/nav_trace.txt).

### Repath
Finds path from current area to goal area using A* and builds waypoint plan.
- Clamps goal point into goal area bounds.
- On fresh attempt, calculates total path distance and estimated arrival time.
- Logs waypoint details including jump/crouch flags and target area IDs.

Two different log lines, and the distinction matters when reading a failure. A fresh path logs
`Repath` plus every waypoint; a mid-route re-plan logs the one-line `Replan`, because that is the one
that fires after a failed air move and it would otherwise be silent. Both carry `penalized=`, the
count of connections on the chosen route with a nonzero `runtimePenalty` — so `penalized=0` on the
replan after a failure means the penalty did its job and the new route avoids the bad edge.

The fresh dump also prints one `conn[i]=<index> <type> <a><->b cost= travel=` line per connection on
the route. A route used to be visible only as areas and waypoints, and neither says *which edge* the
plan took: two areas can be joined by a walk, a jump and a ladder at once, the waypoints for them look
alike, and the connection index is what every penalty, abort and `wanted <area>` message is keyed on.
Both of test53_ladder4's ledge failures were only legible once the alternation could be read as conn 15
against conn 26 rather than as two similar-looking waypoint lists.

### Bot_MoveNav
Main navigation-driven movement function. Repath triggers are: empty path, forced (air move landed
off-target), and stall (no progress for kRepathInterval).

An air move or ladder that finishes in the wrong area replans in the *same* frame, not the next one.
The forced-repath latch is read at the top of the frame, above the air-move and ladder blocks, so a
landing that arms it would otherwise spend a frame driving on a plan the bot has provably left — and
`trackState.started` is cleared on that same frame, which makes the reprojection global and can snap
`progressS` anywhere on the stale profile. Both blocks therefore re-check the latch through
`TryRepath` once their runner returns.

- Uses NavFollow::Track against the `PlanProfile` built at repath time (see doc/nav.md). Progress is
  arclength along the arc chain, so there is no per-waypoint arrival radius or advance loop, and
  waypoint 0 is no longer a special case — the chain starts at the bot's own position.
- Drive magnitude is the servo's `wishSpeed`, divided by `kDuckingMultiplier` when `FL_DUCKING`
  (the engine applies 0.333 after the maxspeed clamp) and then clamped to maxspeed. This replaced the
  constant `kFullDrive = 1000`, which meant the bot had no speed control at all: it entered every
  corner at full speed and walked past the goal until an arrival radius caught it.
- Stall detection is rebased on `trackState.progressS` advancing, not on the waypoint index changing —
  a bot correctly braking to a stop at a hard node would otherwise read as stalled. At 2 s the timer is
  short enough that a maneuver which suspends `Track` has to hold the clock open itself: `RunLadderMove`
  and `RunAirmoveFlight` both stamp `lastProgressTime` every frame, because no arclength accrues while
  they are driving. Without that a long fall (up to `kAirmoveMaxAirTime`, 6 s) trips the stall timer
  mid-air and repaths from an airborne origin. The cost of holding the clock open is that a flight which
  never ends is invisible to every other timer, which is why the flight bounds itself — see
  `RunAirmoveFlight`.
- `BotPhysics` reads sv_gravity, sv_airaccelerate, sv_accelerate, sv_friction, sv_stopspeed and
  sv_edgefriction, so plan-time physics match a non-default server. Only the first three were read
  before, which silently diverged the profile from the engine.
- Jump waypoints are commitments — not passed until the jump is made, and gated on on-ground,
  distance and facing tolerance, with duck added while rising.
- Aim leads the next waypoint rather than the current one, so the bot looks around corners; the move
  direction comes from the steered direction, not from where it is looking.

### RunAirmoveApproach / RunAirmoveDrive
The steering that takes over once the tracker's `wpIndex` reaches an air-move launch waypoint. It aims
`kAirmoveApproachLead` past the launch along the launch heading and commits when the bot is within one
frame of the launch plane **and is up to speed**.

The lead is `NavPlan::AirmoveLaunchLead` — `speed * frametime`, one frame of travel, shared with the
planner so both sides agree on where the bot stops accelerating. It used to carry a flat
`kAirmoveLaunchSlack` of 4 units on top, which bought nothing and cost every jump 4 units of run-up;
on `test37_walkent`, whose approach only has 19.5 units of straight, that was most of the margin.

**A launch under `kAirmoveLaunchSpeedTolerance` of its design speed is not taken.** `PmSim`,
`JumpLandsIn` and the sampler all begin their simulation *at* the design speed, so a launch delivered
short is a jump the plan never modelled. Nothing checked it, and the gap was systemic rather than
per-map — `qconsole.log` showed 222 planned against 146 delivered, repeatedly, and the launches that
matched their design speed were the ones that landed where they were aimed. `test37_walkent` crossed
the plane at 239 of 270 and fell 274 units into the pit it was jumping over.

When the bot reaches the plane still short, it penalizes the connection, requests a repath and brakes
(`RunAirmoveDrive` with drive 0 solves to full reverse thrust) instead of jumping. The bot pays one
approach to learn the edge is undeliverable, and the +5 s penalty routes the next plan around it —
the same penalize-and-repath loop a bad landing already used, moved earlier so the lesson costs an
abort rather than a fall.

**Before blaming the tolerance, check that the follower modelled the ledge it is standing on.**
test53_ladder4's crouch jump (conn 15, area 4 → 2) aborted at 94 % of design on *every* run, including
runs that got the full planned run-up, and 94 % misses `kAirmoveLaunchSpeedTolerance` by one point. The
tolerance was right and the profile was right: the bot arrived at the launch plane holding 66.4 against
a design 70.8, and held it — flat, frame after frame, not still decelerating. A servo that settles a
fixed distance short of its setpoint is a gain problem, not a margin problem.

The gain came from `SolveDrive` being handed `sv_friction` when the engine was applying
`sv_friction * sv_edgefriction`. `PM_Friction` probes 16 units ahead along the velocity, from foot
height, 34 units down, and doubles friction when that finds nothing — which is precisely the state
every air-move approach ends in, since it is walking at the lip it is about to launch from.
`doc/nav.md` derives the residual: a flat `V* - stopSpeed * (f_real - f_model) * dt`, invisible above a
design speed of 80 and unavoidable below it. Crouched approaches are capped near 90, so they are the
ones that live under the threshold.

`BotDrivePhysics` runs the same probe the engine runs — same 16/34 offsets, same hull, same
`fraction == 1` test — and doubles `groundFriction` when it misses, feeding that to every ground
`SolveDrive`: the tracker's, the air-move approach's, and the ladder, elevator, activation and charger
drives. `BotPhysics` stays untouched, so `MakePlanTraces` and `BuildPathProfile` keep planning against
open-ground friction. Do not merge the two: a plan whose design speeds depend on whether the bot
happened to be standing at a ledge when it replanned would move its own waypoints under it.

With the probe in, the same launch reads `actual yaw 105 speed 71 (100%)` and lands in area 2, and the
map stops taking the ladder detour it had been falling back to.

Two things about the drive are easy to get wrong and both stalled real routes:

- **The command is a target velocity, not a wish speed.** `RunAirmoveDrive` puts `drive` through
  `NavFollow::SolveDrive` while on the ground, exactly as `Track` does. Passing `launchSpeed` straight to
  `pfnRunPlayerMove` looks equivalent and is not: below `sv_stopspeed` the engine's friction drop is
  constant, so a raw wish speed under `stopSpeed * friction * edgeFriction / accelerate` moves the bot
  nowhere at all. At a ledge — which is where every dropoff launches from — `sv_edgefriction` doubles
  that to 80 at defaults. A dropoff planned at 57 pressed at full drive and crawled at 2 u/s until the
  stall timer repathed it, forever. `doc/nav.md` has the derivation.
- **Ducking scales the command, so the command has to be scaled back.** `PM_Duck` multiplies
  forwardmove/sidemove by `kDuckingMultiplier` *after* `PM_CheckParamters` has clamped them, so
  delivering wish speed `W` while `FL_DUCKING` means commanding `W / kDuckingMultiplier` and accepting
  that nothing above `maxspeed * kDuckingMultiplier` is reachable. `Bot_MoveNav` did this and
  `RunAirmoveDrive` did not, so every crouched air move was driven at a third of its planned speed.

`RunAirmoveFlight` keeps commanding the raw heading — no servo — because `PM_AirAccelerate` caps its wish
speed at 30 in flight and the trajectory is already committed.

**A landing outside `airmoveTargetArea` penalizes the connection, then forces a repath.**
`RunAirmoveFlight` resolves the landed area and calls `Nav::PenalizeConnection` on the index latched at
launch, or `RewardConnection` when it landed where it was aimed (see `doc/nav.md`). Without that, a
maneuver the follower cannot execute was an unbounded loop: both repath triggers re-plan from the
bot's current position with the full graph, nothing recorded the failure, and the new start position
could flip which route was cheapest — so the bot alternated between two routes into the same dead end
and `actual=` grew as `[11,12,13,11,12,13,...]` for the whole run.

test50_stairs is the map that motivated it, and it shows why this is not a planner or a mesh bug.
Jumping up a staircase is roughly a coin flip: a clipped riser eats all forward speed and throws the
mover sideways. The plan is valid and the sim proves the arc — but the follower launches from the same
point at the same yaw and speed every time, so an edge a human clears half the time fails every time
for the bot. With the penalty disabled the run is a clean 2-cycle, conn 38 → replan → conn 35 → replan
→ conn 38, until the test times out at 12 s; with it the bot penalizes both and walks up instead,
arriving in ~2 s.

An oscillating `actual=` list therefore now means the penalty has not caught up yet — one more failure
per bad edge — or that every remaining route is bad. It no longer means "no memory".

**The flight ends when the bot has settled, and `FL_ONGROUND` is not the only way to settle.** Two
states end an air move with the ground flag never set again: swimming, because `PM_CategorizePosition`
forces `onground = -1` at `waterlevel >= 2`, and a ladder, because `PM_LadderMove` puts the player on
`MOVETYPE_FLY`. An air move that ends in water or against a ladder therefore used to hold
`airmoveActive` until the flat 6 s `kAirmoveMaxAirTime`, and because `RunAirmoveFlight` stamps
`lastProgressTime` every frame the 2 s stall timer could not break it either — the bot swam or hung on
the yaw it launched at, and the only thing that ever repathed it was that timeout. `AirmoveSettled` tests
all three, and `AirmoveEndReason` names which one so the log line distinguishes `landed`,
`splashed down` and `caught a ladder`.

**The timeout is the connection's own flight time, not a flat 6 s.** `AirmoveTimeLimit` latches
`conn.flightTime * kAirmoveTimeFactor + kAirmoveTimeMargin` at launch (capped at `kAirmoveMaxAirTime`,
and the flat cap still applies when the connection carries no flight time), so a 0.4 s hop that goes
wrong is not worth six seconds of driving on a plan the bot has left. The limit is *suspended while the
bot is still falling* — `velocity.z < kAirmoveFallingZ` — because a bot in free fall is going to answer
the question itself in a moment, and a fall deeper than the plan modelled is exactly the case whose
verdict is worth waiting for. What the limit catches is the bot that is neither falling nor settled.

**A verdict is only scored when the bot settled *inside an area*.** The lookup is the strict
`Nav::FindAreaContainingPoint`, not `FindAreaForPoint`. The forgiving one cannot fail: with no containing
area it falls back to any area whose AABB holds the point, and failing that to the globally nearest
centroid — so a bot that ended in the void, in mid-air or on a lip outside the mesh was still scored, and
could be scored as the area it was aiming at. `FinishLadder` was moved off it for exactly this reason and
the air move was left behind. Settled-but-in-no-area is now `undecided`, like the timeout: neither
penalized nor rewarded, but repathed, because the plan is void either way.

**The verdict is the distance to the plan's landing waypoint; the area is the fallback.** Areas are large —
crossfire's area 66 is 36 × 388 units — so an area-only verdict passes a landing hundreds of units from the
modelled one, and at an area seam it fails a landing that was perfect: two of four "failures" in one
crossfire run were `miss 0.3` and `miss 0.5`, the bot exactly where the plan put it while the strict lookup
resolved the neighbour. So `failed` is `miss > kAirmoveLandingTolerance`, and the area comparison is used
only when there is no landing waypoint to measure against. `miss` is 3D against
`waypoints[waypointIndex]`, which after the launch's `++waypointIndex` *is* the plan's landing waypoint.

Two things about that measurement are easy to get wrong and both produced garbage before the sweep caught
them:

- **Measure against the plan's waypoint, never `conn.landingPos`.** The connection's landing is what
  navgen's sampler recorded for the edge; the planner re-derives the arc per route from the funnel's launch
  point. The two differ by 65 to 134 units on landings whose flight time matched plan to a hundredth and
  which landed in the wanted area — nine good connections penalized in one sweep.
- **Waypoint z is origin space, not `BotNavZ` space.** `BotNavZ` exists for elevator nav-z comparisons and
  subtracting `kPlanStandHeight` here put a systematic ~18-unit floor under every miss. The log settles it:
  landing waypoint `(-436.6, 841.9, -1626.2)` against an actual landing origin of `(-438.1, 843.5, -1628.0)`.

With both fixed the measurement is tight enough to set a threshold from: 0.2–8.2 units across the whole
`maps/` sweep, 0.2–14.0 across a two-bot crossfire run, and the failures in that same run at 93, 100 and
155. `kAirmoveLandingTolerance` is 48 — three times the worst good landing, half the cheapest bad one.

`airmoveOverrunLogged` prints one line — not one per frame — the first time a flight passes its planned
time by `kAirmoveOverrunGrace`, carrying `onground`, `waterlevel`, `movetype`, `FL_DUCKING`, velocity and
the resolved area. A flight that hangs is hard to reproduce and the exit line alone cannot say *which*
of those states it was stuck in.

A repath inside an air move's run-up leg makes this worse in a way worth recognising. `Repath` starts
the plan at the bot's own position and `Cross` then places the run-up node a fixed distance back along
the launch heading, so a bot that has already closed most of that distance gets a first waypoint
*behind* it and walks backwards to reach it. The plan is not wrong — it is the same plan it would
build from anywhere — but the bot arrives at the run-up node with no speed, which is the worst state
to start a launch that has to build speed at a ledge. Two waypoints at the same 2D point at the head
of the list in `crossbot_nav_status` are the tell.

### Ladders

`RunLadderApproach` walks the bot to the mount and latches `CBot::ladderActive`; `RunLadderMove` then
owns every frame until it finishes, exactly as the air move does. Eight rules make the difference between
a climb and the jump-on-jump-off loop test53_ladder4 used to run. A ninth is the descent aim, and a tenth
is how the attempt is scored; both are below.

**Drive from the plane the engine picked, not the face navgen planned.** `PM_LadderMove` chooses its
plane with `PM_TraceModel(origin -> ladder centre)` in the *point* hull, so the face is decided by where
the bot actually stands. When that disagrees with the planned face, `dot(velocity, N)` is ~0, `lateral`
collapses to the raw view vector, and the bot flies horizontally off the ladder at 200 u/s.
`ProbeLadderPlane` repeats the engine's own trace every frame and every aim is derived from its result;
`LadderPlane::why` records which of `no-entity` / `startsolid` / `allsolid` / `no-hit` / `degenerate`
rejected a probe. With a valid plane every motion is reachable with `IN_FORWARD` alone plus view angles —
climb faces `-N` at pitch 0, descent faces `-N` at pitch +89, sideways faces `±perp`. A degenerate plane
is not a failure to route around: velocity is then exactly `vpn * 200`, so the fallback simply aims pitch
∓89 and rides that.

**A ladder mount bars the profile, like a ride does.** Mount to exit is near-zero in XY, so
`NavFollow::Track` sails straight through it and `waypointIndex` reaches the exit while the bot is still
short of the volume. `RunLadderApproach` is then never called for the mount, and the moment the bot
drifts onto the ladder anyway the `ladderEject` rule sees no ladder waypoint in range and jumps it back
off at 270 u/s — forever. The mount therefore sets `NavFollow::Config::sLimit` while
`i >= bot.waypointIndex`, released by the `++bot.waypointIndex` that commits the climb.

**Never duck on a ladder while `FL_ONGROUND`.** `PM_CategorizePosition` treats the player as airborne
only above `velocity[2] > 180` (`pm_shared.c:1542`); below that it traces two units down and, if it finds
floor, snaps the origin back to it. A ducked climb is `MAX_CLIMB_SPEED * 0.333 = 67`, under the
threshold, so at the foot of a ladder the bot is re-glued to the floor every frame: correct plane,
correct aim, `vel=(0,0,67)`, and an origin that never leaves z 18-20. Only the two `[crouch]` ladders in
test53_ladder4 stalled, and only at their mounts. `RunLadderDrive` drops `IN_DUCK` while on a ladder and
on the ground; the bot detaches in one frame at 200 and re-ducks in the air, where the two-unit trace no
longer reaches. `PM_UnDuck` refuses to unduck into a ceiling, so a genuinely low mount degrades safely.

**Never hold `IN_JUMP` while `MOVETYPE_FLY`.** `PM_LadderMove` reads the jump button before anything
else and answers with `movetype = MOVETYPE_WALK; velocity = normal * 270` — one frame of jump ejects the
bot off the ladder at 270 u/s. The reach-up hop that grabs a dangling rung is therefore gated on
`!onLadder`, not just `FL_ONGROUND`: at the foot of a ladder that reaches the floor the bot is on the
ladder *and* on the ground at once, so the ungated version hopped on, ejected, walked back and repeated
forever. `avoid.jump` is suppressed the same way, and `RunAirmoveDrive` drops the button outright when
the mover is on a ladder, so a flight that ends on one cannot bounce off it.

**A ladder need not reach the floor, and reaching one costs exactly one frame of jump.** test53_ladder4's
`*2` brush stops 62 units above the floor of its shaft: hull 1 expands it to `z >= -212` and a standing
bot's origin is at -220, so no amount of walking touches it. `LadderOutOfReach` detects that, waits
`kLadderReachDwell`, and hops. What made the hop framerate-dependent was the engine re-simulating the
held button mid-flight — see `Bot_RunPlayerMove`, which is why the hop is now a 50 ms command and the
rest of the frame is jump-free. The contact sub-step then finds `IN_JUMP` clear, and the bot grabs at the
top of its arc instead of being thrown back down the shaft.

**Aim before driving.** `PM_LadderMove` builds its wish from the *buttons*, so `IN_MOVERIGHT` is a full
`±MAX_CLIMB_SPEED` sideways input no matter how small the `smove` that set it. A bot still turning toward
its target yaw is therefore commanding a large wrong-axis vector, and the ladder converts wrong-axis
input into the wrong motion: perp becomes pure horizontal at 200, into-the-face becomes +200 climb.
A descent that committed while still facing its run-up yaw climbed straight out of the top of the ladder
and walked off the ledge — `caught the ladder` then `fell off` 0.4 s later, 61 units sideways, twice per
run. Every on-ladder phase now zeroes `drive` while the yaw error exceeds `kLadderAimTolerance`; with no
buttons `PM_LadderMove` clears the velocity, so the bot simply hangs there and turns.

**Latch the dismount phase.** The height test that switches from climb yaw to dismount yaw sits at a
single z, so a bot hovering on it alternated targets every frame and the aim gate held `drive` at 0 for
both — a 12.7 s climb that should take 2 s. `ladderAtTop` is set once and never cleared before the next
commit.

**A climb approach aims past the mount.** `pointA` is deliberately outside the carved volume, so
steering *to* it stops the bot short of the capture region — for a ladder whose volume starts two units
beyond the stand point that is a permanent stall, jumping in place at full drive. The approach aims
`kLadderMountLead` further along `ladderYaw`, i.e. into the ladder, and lets the capture happen on the
way through.

**A descent approach aims past the mount and never turns back.** A climb has `ladderYaw` to steer by; a
descent has nothing equivalent, because the bot cannot grab the ladder from where it stands. `PM_Ladder`
is a *point* test of the origin against the brush's hull-3 volume, and where a ladder ends flush with the
ledge it serves, that volume tops out at exactly the walkable duck-origin z — test53_ladder4's `2 <-> 4`
ladder ends at 64, the ledge floor is 64, and both put the top of the volume at z 82 with the bot standing
on z 82. Nothing catches until the bot walks off the ledge and starts to fall, so the descent aim exists
to carry it over the edge, `kAirmoveApproachLead` beyond the mount along the line it is already on.

Deriving that line from `origin -> mount` inverts it the instant the bot passes the mount, and the guard
that swapped in `ladderEntryYaw` within eight units only moved the inversion to the edge of that ball:
the bot settled on the shell `|origin - mount| = 8` on the far side and hovered there at ~16 u/s, drifting
in and out, until `kLadderStallDist` gave up. It needed y 272 and the shell held it at y 270. The branch
is now chosen by the *signed* progress along `ladderEntryYaw` — `dot(toMount, entryDir) >
kLadderMountApproach` still steers through a mount genuinely ahead, level or behind drives on `entryDir`
alone — which is the projection `RunLadderApproach` already commits on. Every origin the old
`never caught` lines printed was 7.9-8.1 units from the mount; that number was the entire bug.

Because both ways out of that ledge failed, the failures alternated: the crouch jump aborted under speed
and penalized its connection, the next plan picked the ladder, the ladder never caught and penalized
*its* connection, and the plan came back to the jump. Nothing was oscillating in the planner — penalties
were working exactly as designed, on two edges that were both genuinely broken. A repath that keeps
flipping between two routes is worth reading as a symptom of that, not as a planner-stability problem.

**The verdict judges the maneuver, and only then the area.** `FinishLadder` decides whether the
connection earns a `RewardConnection` or a `PenalizeConnection`, and it used to decide entirely on
`FindAreaForPoint(origin)`. That function never returns NULL — after the exact half-space test it falls
back to nearest-centroid-with-AABB and then to plain nearest centroid — so it always names *an* area, and
where the ladder shaft is carved out of the mesh the nearest centroid is routinely the area the bot just
left. test53_ladder4's `2 <-> 4` descent ends at (442.7, 272.8, 19.7), inside no area at all: area 2
starts around y 280, and the answer came back as area 4, the ledge 62 units above. A descent that worked
was charged `kConnFailPenalty` every single time, and the next `Replan` reported `penalized=1` because
of it.

So the verdict now takes two inputs. `FinishLadder`'s `completed` argument says whether the maneuver
itself finished as designed — `reached the bottom of` and `dismounted off` pass true, `never caught`,
`fell off`, `timed out on` and `stalled on` pass false — and a maneuver that did not complete is a
failure whatever the geometry says. Only when it did complete does the area matter, and then it is read
with the strict `Nav::FindAreaContainingPoint`: the target area rewards, a *different* area penalizes,
and no area at all is `undecided` — neither rewarded nor penalized, because a bot still in the shaft is
not evidence of anything. The log line carries that word, so a run tells you which of the three happened.

Deferring the undecided case instead — hold the verdict, settle it once the bot is back on the ground
and strictly inside something — was tried and is worse. By the time the bot satisfies that test it has
walked on: test53_ladder4's `11 <-> 8` descent settled two areas later, in 13 rather than the 11 it was
promised, and penalized a ladder that had just worked. Deferring only moves the wrong answer. Nothing is
lost by declining to answer, either, because `RewardEnteredConnection` still decays the penalty when the
bot actually walks into the planned next area, and that is the reward path that matters.

`RunAirmoveFlight` scores its connection the same way and has the same forgiving lookup. It is far less
exposed — its verdict fires once `AirmoveSettled`, so the bot is on a floor, in water or on a ladder —
and the timeout, which is decided in mid-air, is the same trap and so declines to score at all.

**Stalls are measured in distance, not speed.** A bot wedged against the brush can hold a healthy
`vel=(0,0,67)` forever while its origin does not move, so the detector compares against
`ladderStallOrigin` and gives up after `kLadderStallDist`. Two escapes are allowed, each a push-off;
the third gives up on the connection and repaths.

### Ladder debugging

`bot_move_nav.cpp` carries a per-frame ladder trace behind `#ifdef CROSSBOT_LADDER_DEBUG`, off by
default. Turn it on with

```
build.bat -DCROSSBOT_LADDER_DEBUG=ON
```

(the value is cached in `build/CMakeCache.txt`, so plain `build.bat` keeps whatever was last set). It
prints to the server console, which the test harness already captures at
`nav_debug/logs/<map>_server.log`, and it is emitted only while the bot is in the ladder pipeline. The
macro compiles its arguments away when the flag is off.

Lines read `CrossBot: LDBG <bot> t=<time> <phase> key=val ...`, one per frame per phase:

| phase | what it shows |
| --- | --- |
| `approach` | the whole pre-commit state: mount, dist, remaining, launch lead, computed aim, `blocked`/`reachUp` |
| `past` | the approach handed back to the walker because the bot is already beyond the dismount height |
| `entry` | pre-catch, with all three "never caught" predicates evaluated separately |
| `state` | plane validity and reason, normal, chosen step, dismount dots, stall age and displacement |
| `aim` | target vs current angles, the error, and whether `drive` was gated by turning or commanded off |
| `drive` | `fmove`/`smove` and the decoded button names actually sent to `pfnRunPlayerMove` |
| `after` | position and velocity immediately after that call, so one frame's cause and effect sit together |
| `exit` | off-ladder, seeking the exit point |
| `eject` | the walker's `OnLadder` branch: which of the next three waypoints are ladder waypoints, and whether `ladderEject` fired |
| `walk` | the walker's own actuation whenever a ladder is involved — the only other place `IN_JUMP` can reach the bot |

`drive`/`after` and `eject`/`walk` between them cover every button the bot can be sent, which is what
makes "who pressed jump" answerable rather than a matter of reading gates.

### Elevators

`RunElevatorApproach` / `RunElevatorRide` / `FinishElevatorRide` are the ladder trio again, but the hard
part is not the driving — it is that **the ride is invisible to the profile**. The planner's boarding and
arrival waypoints are the mover's column centre at two heights (`elevators.cpp` builds both from
`centerX`/`centerY`), so they differ only in Z. `BuildPathProfile` measures `Dist2D`, gets zero, and never
pushes the segment; `ProjectOntoProfile` skips zero-length segments outright. The `gap` flag `IsGap` sets
for a ride lands on a segment that does not exist.

Three consequences, all of which had to be answered separately:

**The tracker walks through the ride.** With `projectAhead` of 64 units and both ends at the same XY, the
bot aims at the far-side corridor while standing at the near-side stop. `NavFollow::Config::sLimit` already
exists for the doubled-back activation detour, which is the same failure in weaker form, so a *pending*
ride sets the barrier too. Pending means the flag is still set: `FinishElevatorRide` clears
`requiresElevatorRide` on the bot's own copy of the waypoint rather than remembering an index, because
`SpliceDetour` rebuilds `bot.waypoints` with a shifted base and an index would not survive it. No memo is
needed the way activations need one — a repath from the far stop plans a route with no ride in it at all.

**Reprojection cannot be global.** The ladder can clear `trackState.started` when it finishes because its
two ends differ in XY. A ride's do not, so a global reprojection is free to resolve the tie in favour of
the *lower* `s` and put the bot back on the approach. `FinishElevatorRide` sets `trackState.progressS` to
the arrival waypoint's `ProfileWaypointS` by hand and leaves `started` true; the incremental window then
reaches at most `projectBack` behind, where the profile is 32 units away and cannot win the tie.

**`totalTime` under-reports by the whole ride.** The ETA integrates over segments, and there is no segment,
so `BuildPathProfile` adds `elevatorWaitTime + elevatorTravelTime` after the loop. Without it
`test55_plat`'s ETA is 3.8 s against a 4.96 s ride and `test_nav_goal.ps1` — which times out at twice the
bot's own ETA — cannot pass the map however well the bot behaves.

**The ride waypoint may own no profile segment at all**, so the approach cannot be dispatched off
`bot.waypointIndex` the way the air move and the ladder are. On `test18_elevator` the funnel corner
before the board point is the shaft centre at floor height, the board point is the same XY at the stop's
top surface, and *that* pair is zero-length too — the segment reaching the shaft carries the corner's
index and index 10 never appears in the profile. The runner is therefore driven by the barrier index the
`followCfg` loop already computed, exactly as `RunActivation` is driven by `activationWp`. Dispatching on
`waypointIndex` looked right on `test55_plat`, where the approach happens to be the ride's own segment,
and silently never boarded on test18.

Beyond that the runner is deliberately dull. It latches on `FL_ONGROUND` within `kElevatorBoardRadius`,
holds station with a `kElevatorHoldRadius` deadzone, never jumps, and aims at the waypoint *after* the
arrival one so the bot is already turned to walk off — `func_plat`'s `PLAT_DELAY` gives it three seconds.
`RunElevatorDrive` skips `SolveDrive` when drive is zero: the servo answers a residual velocity with a
full-rate reverse command, which is right on open ground and wrong on a platform edge, so friction does
the stopping instead. `lastProgressTime` is stamped every frame for the same reason the ladder stamps it,
and the 150-unit teleport reset at the top of `Bot_MoveNav` is skipped while riding, since a server hitch
would otherwise repath from mid-shaft where `FindAreaForPoint` finds nothing and the bot drops its goal.

The arrival test is `BotNavZ` against the arrival waypoint's own floor Z, directional with
`kElevatorArriveSlack`. The slack means the bot can step off up to 8 units short, and that is safe for a
reason worth stating: navgen only builds a walk seam off a stop when the step is under 18 units, so
anything within the slack is steppable by construction. On `test55_plat` the ride ends at nav 10 against
a stop at nav 18 and the bot walks out over the remainder.

The engage radius is pure XY distance to the board point, so a route that merely *passes* within 48 units
of a shaft it has not reached yet would hand the frame over early. Ordering is what protects this today:
the barrier sits at whichever comes first, so while a call button is unserviced `rideWp` is unset and the
runner is never called. On `test18_elevator` that matters concretely — the air move lands 40 units from
the board point, inside the radius, with the button still to press. If a map ever defeats that, the ride
times out, penalizes the connection and repaths; tying engagement to `progressS` reaching `sLimit` is the
tighter fix if one is needed.

`rideDeadline` is `wait + travel * kElevatorTimeMargin + kElevatorTimeGrace`; the
wait term matters because a ride *back* to the default stop does not begin until the platform's auto-return
dwell elapses. Timing out finishes the ride like any other failure — penalize the connection, repath — and
that is also what recovers a bot that walked to a boarding point whose platform someone else had ridden
away.

`bot_avoid.cpp` needed nothing: `RouteExemptsHit` already exempts every `SOLID_BSP` hit while a ride
waypoint is in the lookahead window, so the bot does not fight the platform on the way in, and the runner
returns before `BotAvoid::Update` while riding.

**What the two test maps actually cover.** `test55_plat` is the only one that exercises a sustained ride:
the bot boards the parked plat in the pit, `CPlatTrigger` fires under it, and it rides 248 units to the
ledge — the log reads `boarded ... (up)` then `ride reached the stop of after 4.18s ... area 4 (wanted 4)`
against a 4.96 s plan. Note it boards at nav −199, not the parked −230, because stepping onto the plat
starts it moving well before the bot reaches the column centre; the runner does not care, since it watches
the arrival height rather than counting from the board point. `test18_elevator` covers summon-then-board —
drop into the shaft, walk to the call button, press, board — but **not** the ride loop: the door travels
for 2.38 s while `RunActivation` holds its 2.4 s button wait, so the bot is already at the top when it
boards and the ride finishes in 0.00 s. That is correct behaviour, not a skipped test, but a map with a
call button *off* the platform is what would exercise waiting for a lift to arrive.

### BotNavZ
A waypoint's point is at player height in its own area, so the floor under it is the point less the
stand lift unless the plan wants the bot ducked -- the convention BotNavZ reads off the live stance.

### SolveDrive skip on zero drive
Zero drive means stand exactly still. SolveDrive answers a residual velocity with a full-rate reverse
command, which is the right servo answer on open ground and the wrong one on a platform edge, so the
solve is skipped and friction does the stopping.

### Elevator ride flag handling
The ride is spent. Clearing the flag is what lifts the sLimit barrier, and it travels with the
waypoint through SpliceDetour, which an index watermark would not.

Boarding and arrival share an XY, so a global reprojection is free to snap back below the
ride. Place progress on the far side by hand instead of clearing trackState.started, and
floor it there: the two segments meet at one point, and walking away from a lift you entered
and left on the same side is equidistant from both, which the earlier segment wins.

No arclength accrues while the platform does the work, so the 2 s stall timer would fire twice
over on a 5 s ride.

This is called for the whole stretch the ride bars, which is most of the path. Leave the walking
to the tracker and take the frame only once the platform is close enough to step onto.

Every arclength is about to change, and the rebuilt chain starts at the bot's own position, so
there is no pre-ride geometry left to guard against.

### Server hitch during ride
A server hitch during a ride can move the bot further in one frame than a teleport would, and resetting
mid-shaft repaths from a point that resolves to no area at all -- the bot then drops its goal.

### Pending ride bars profile
A pending ride bars the profile for the same reason an unserviced activator does, only harder: its two
waypoints share an XY, so the ride is a zero-length segment the tracker cannot see at all and would
otherwise walk straight through. A ride is "pending" until FinishElevatorRide clears its flag.

### Ladder mount bars profile
A ladder mount is the same shape -- mount to exit is near-zero in XY -- and is "pending" until
RunLadderApproach commits and steps the index past it. Letting the tracker sail through one is what
stranded test53_ladder4 at ladder *5: the index reached the exit waypoint while the bot was still
two units short of the volume, so the mount was never serviced, and the moment the bot drifted onto
the ladder anyway the ladderEject rule below saw no ladder waypoint in range and jumped it straight
back off at 270 u/s, forever.

### An air-move launch bars the profile too
Same shape again, and it was missing from the barrier list for a long time — `barrier` covered rides and
ladder mounts only. The gap segment starts *at* the launch point, so once the bot is within a unit or
two of it the nearest point on the chain flips to the far side of the hole and `wpIndex` becomes the
landing waypoint. `RunAirmoveApproach` is only entered when the *current* waypoint carries
`airmoveLaunch`, so a launch skipped that way is never taken at all: the bot keeps walking at a waypoint
on the other side of a gap and drops into it.

crossfire is where it showed. Area 66 is a ledge `x ∈ [140,176]`, area 47 is another at `x ∈ [240,276]`,
there is no walk connection between them — 66's only walk neighbours are 54 and 80 — and the bot fell at
`x = 204.7`, squarely in the 64 units of void, with `heading to wp 5/11` naming a waypoint inside area 47
and no launch line anywhere above it. Nothing else in the follower would catch that: `ProjectOntoProfile`,
`lateral` and the `done` test are all horizontal, so a 180-unit drop reads as zero deviation and only the
2 s stall timer eventually fires.

### Barrier driven by barrier index
Driven by the barrier index rather than bot.waypointIndex, because a ride waypoint need not own any
profile segment: on test18_elevator the funnel corner before the board point shares its XY, so the
segment that reaches the shaft carries the corner's index and the ride's index never appears at all.

### RunActivation ordering
Ahead of RunActivation, since a board point can sit inside the engage radius of a later button.

### RunChargerUse

`RunActivation` again, with two differences that matter. `IN_USE` is **held every frame** rather
than pressed once, because `CRecharge::Use` and `CWallHealth::Use` are `FCAP_CONTINUOUS_USE` and
transfer exactly one point per call — a single press buys one armor. And the exit condition is
measured, not timed: the runner watches `ChargerValue` and gives up `kChargerNoGainGrace` (0.5 s)
after it stops rising.

What stopped rising means depends on `pev->frame`, which is the charger's own empty flag (`Use` sets
it to 1 when `m_iJuice` hits zero, `Recharge` puts it back to 0). Frame 1 means the charger is
drained and the bot leaves without a memo entry — the live `frame` test in selection already filters
it, and unlike a pickup there is no guesswork about when it returns. Frame 0 with no gain means the
bot is full, and the classname goes in `fullMemo`.

**The charger's capacity is not the bot's ceiling.** `sk_suitcharger` is 30 on `test56_recharger`'s
skill level, so a bot at 0 armor drains the whole thing and leaves at 30, comes back after the 30 s
recharge, and needs four visits to reach 100. The run reads `0 -> 30 -> 60 -> 90 -> topped up at
100`. A runner that assumed one visit fills you up would have called the first three visits failures.

It is dispatched from inside the `follow.done` branch rather than from the waypoint loop, because a
charger is not a waypoint — the plan's goal is the use spot and the charger is off the mesh
entirely. `lastProgressTime` is stamped every frame for the same reason `RunActivation` stamps it:
no arclength accrues while standing still and the 2 s stall detector would otherwise repath.

## bot_avoid.cpp

Reactive obstacle handling, layered between the follower's steering decision and the engine call. The
nav mesh only knows world geometry and the brush entities navgen carves, so anything else solid — a
`cycler`, a monster, another player, a door that has swung out of its own doorway — is invisible to
the planner: it routes straight through and the follower drives into it and stalls. `BotAvoid::Update`
traces forward along the planned path each frame and answers with one of three moves: crouch-jump onto
the obstacle, duck under it, or go around it.

### Which entities get avoided

**`MOVETYPE_NONE` does not identify the static ones** — this is the obvious filter and it is a no-op.
Every solid brush entity in HL is `MOVETYPE_PUSH`, `func_wall` included (`dlls/bmodels.cpp`,
`doors.cpp`, `func_break.cpp`, `plats.cpp`). The brush entities that are `MOVETYPE_NONE`
(`func_illusionary`, `CMultiSource`) are all `SOLID_NOT`, so a trace never reaches them.

`FL_WORLDBRUSH` is the flag that does identify them. `CFuncWall::Spawn` sets it — "If it can't
move/go away, it's really part of the world" — `const.h` defines it as "Not moveable/removeable brush
entity", and the HLSDK's own node graph uses it for this same test. `func_conveyor` and
`func_wall_toggle` inherit it; the only other setter is unbreakable glass. It is also precisely the
set navgen carves as blockers, so skipping it avoids fighting the mesh over geometry the mesh already
encodes.

A hit is skipped when it is worldspawn, when `solid` is none of `SOLID_BSP`/`SOLID_BBOX`/
`SOLID_SLIDEBOX`, when it is a `SOLID_BSP` with `FL_WORLDBRUSH`, or when it is the passable the route
runs through (below). Everything else is avoided, which includes other players — bot-vs-bot is not
special-cased, it simply never qualifies for either maneuver.

**The door on the route is exempt, and identifying it needs no entity id.** `RouteExemptsHit` scans
the next `kAvoidRouteLookahead` waypoints for `requiresDoor`/`requiresBreak`, resolves
`targetAreaId`, and skips the hit when the area's bounds overlap the entity's `absmin`/`absmax`. That
works because navgen builds a door/breakable area *from the entity's own brush-model bounds*: a
closed door sits inside its area and overlaps, so the bot walks into it and touches it open; an
opened door has translated out of that volume, no longer overlaps, and is avoided like anything else.
This matters because `NavAreaRecord` stores no entity or model index — a direct edict↔area link does
not exist in the format. `requiresElevatorRide` in the same window exempts every `SOLID_BSP` hit, so
the bot does not fight a plat it is about to board.

### The three maneuvers

The forward trace runs in the hull the bot currently occupies (1 standing, 3 ducked) with
`dont_ignore_monsters`, over `speed * kAvoidLookaheadTime` clamped to
`[kAvoidLookaheadMin, kAvoidLookaheadMax]`. The existing `EngineTraceFn` cannot be reused for it: it
passes `ignore_monsters` and discards `tr.pHit`. Heights are measured against the bot's feet, so
`gap = absmin.z - feetZ` and `top = absmax.z - feetZ`.

**Jump-on is tried first, because it is the faster way past** — crawling costs `kDuckingMultiplier`
of speed for the whole traverse. It applies when `top` is above step height (below that the engine
steps up for free) and within `PlainJumpHeight(gravity) + 18` less a margin. The 18 is the feet lift
from ducking in the air; the jump fires when `dist <= speed * riseTime + kAvoidJumpLead`, where
`riseTime` is the smaller root of `top - 18 = kJumpImpulse*t - g*t²/2`. At a standstill against the
obstacle that degenerates to a fixed 8-unit trigger, which still fires.

**A ducked bot cannot crouch-jump.** From a ducked stance the origin rises 45 and the feet rise with
it — there is no 18-unit credit, because the hull is already small. So when the bot is ducked and
`top` exceeds a plain jump, the latch is cleared and `suppressPlanCrouch` is set instead of jumping;
unducking is instant where there is room and the jump fires a frame or two later. On `test52_avoid`
this is also what ends the crawl under the first can row in time for the jump onto the second, which
is only 64 units further on. `kAvoidLookaheadMin` is 64 rather than smaller for the same reason: a
ducked bot moves at 83 u/s, and a speed-proportional probe alone would not see the second row until
there was no room left to stand up.

**Crawl-under** applies otherwise, when the bot is standing and `gap` clears the 36-tall duck clip
hull with margin. The hold is `(dist + depth + 2*kPlanClearance) / duckSpeed * kAvoidCrawlPenalty`:
the requested `min(x,y)/speed*penalty`, plus the remaining approach and the bot's own 32-unit depth,
evaluated at ducked speed rather than max speed. `slowApproach` caps the drive at ducked speed while
the latch is set but `FL_DUCKING` has not appeared — `PM_Duck` takes 0.4 s on the ground before the
small hull exists, which is 100 units at full speed, and on test52 the bot only has 48 from spawn to
the first row. `kAvoidDuckMinHold` is 0.5 s for the same reason: releasing `IN_DUCK` part-way through
aborts the transition.

**The hold time does not have to be accurate**, in either direction, which is why nothing maintains
it. Too short is covered by the engine — `PM_UnDuck` traces the stand hull and returns without
clearing `FL_DUCKING` when it is `startsolid`, so a bot whose latch expired under an obstacle stays
ducked and keeps crawling, and the `drive /= kDuckingMultiplier` compensation keys off `FL_DUCKING`
rather than off the latch, so its speed stays right too. Too long is covered by the jump branch,
which clears the latch when it wants to stand. An earlier design carried a second "can I stand here"
probe to maintain the latch; it was solving a problem the engine already solves.

A plan-mandated crouch always wins: avoidance may add a duck and may drop *its own* duck, but it never
overrides `requiresCrouch` or `NAV_AREA_CROUCH`.

**Going around** is the third answer, taken when neither height predicate applies — a standing player,
a full-height crate, a door that has swung open. It searches for a route past the obstacle and splices
it into the front of the plan.

### The side search

The obstacle's `absmin`/`absmax` is expanded in x and y by `kAvoidHullHalf + kAvoidCornerMargin`, so
its four corners are positions the bot's own hull can occupy with two units to spare. Projected into a
frame where *forward* is the direction from the bot to the obstacle centre and *right* is the bot's
right, two corners fall on each side. The search then walks the box perimeter: from the current point,
trace to the first corner on the chosen side, and after each corner it reaches, try to trace straight
to the destination. Up to all four corners are used, so the route can wrap around a far face.

**Two corners is not enough,** which is why the walk exists. Two suffice only when the destination is
roughly opposite the approach; when it is off to the side the leg from the far corner re-enters the
expanded box and clips the obstacle it just cleared.

**Corners are not required to be standable.** The downward hull probe at a corner supplies its z, but
when it comes back `startsolid` — the corner is buried inside a neighbouring obstacle — the corner is
kept at the previous point's height and the leg is traced to it anyway. The leg then reports the
neighbour, and *that* is the useful outcome: the search recurses on the neighbour with the **final
destination** as its target, not the unreachable corner, starting from the point the blocked trace
began. On a cluster of touching obstacles this is the only thing that works; requiring standable
corners abandons the side, and merging touching obstacles into one box over-blocks whenever the
merged rectangle spans empty diagonal quadrants.

A leg blocked by *world* geometry does not fail the side — the walk skips that corner and tries the
next. A leg blocked by an already-visited obstacle fails, which with `kAvoidMaxDepth` and
`kAvoidMaxTraces` is what terminates the recursion. The visited set is restored to its entry state
before each side is tried, so the two sides never poison each other.

**Which side first.** If the obstacle is moving, go where it is not: `dot(MoverVelocityAt, right)`
above `kAvoidMoverSpeed` picks the opposite side and takes it without evaluating the other — an
entity sliding to the bot's left sends the bot right. Otherwise both sides are evaluated and the shorter
route wins, ties going right. Right-on-tie is the useful convention for two bots meeting head-on:
their `right` vectors point opposite ways in world space, so both stepping right passes them cleanly.

**A flat hull trace has no step-up,** so a stair nosing or a slope lip reads as a wall. When a leg is
blocked by non-qualifying geometry whose plane normal is floor-like (`z >= kAvoidFloorNormal`), the
same leg is retried raised by `kAvoidStepHeight` and accepted if that clears. This is the navgen idiom
from `airmove_sampler.cpp`.

### Where the detour rejoins the plan

Not at a waypoint. navgen cannot see a `cycler` or a player, so a planned waypoint is frequently
*inside* the obstacle — no route to it exists at all, and aiming at one is how the search fails
without any geometry being to blame. Instead the remaining planned polyline is sampled forward at
`kAvoidDestStep`, and the destination is the first sample that is both outside the expanded box and
passes the ground probe. The waypoint that sample belongs to becomes `detourReplaceUntil`; waypoints
before it are dropped, since they are the ones inside the obstacle.

The detour is capped at `kAvoidDetourMaxLength` of total path. **This cap must not be relative to the
distance to the rejoin point** — the rejoin point is deliberately close, so a relative cap rejects
exactly the long routes that a cluster legitimately requires. The five-can corner on `test52_avoid`
needs 432 units to clear a rejoin point 60 units away.

### Splicing

`BotAvoid::Update` returns the points; `SpliceDetour` in `bot_move_nav.cpp` owns the surgery, because
it already has `MakePlanTraces`. The waypoint list is rebuilt as
`[bot origin] + [detour points] + [plan from detourReplaceUntil]` with the index reset to 0, then
`BuildPathProfile`, a fresh `TrackState`, and `lastProgressS`/`lastProgressTime` reset — the same
sequence `Repath` performs. Splicing rather than running a second follower means the detour gets the
arc profile, the speed solve, the stall detector and the status dump for free.

**The list has to start at the bot's own origin.** A profile whose first segment begins at the far
corner leaves the bot over `Config::rejoinAbort` (96) lateral on frame one, and `follow.needRepath`
undoes the detour immediately. Dropping the already-traversed waypoints matters for the same reason
`Repath` does it: a from-scratch projection over a profile that still contains the route behind the
bot can snap backwards onto it.

Detour waypoints carry `BotAvoid::kDetourClearance` rather than the default 16. `BuildPathProfile`
sizes corner arcs from clearance, and a wide arc at a detour corner cuts back into the obstacle the
corner was placed to clear. `estimatedArrivalTime` is extended by the added length so the test's
ETA-derived timeout stays honest. The splice invalidates the `wp` pointer, so the frame ends with
`RunIdleMove` and a return.

### When no route exists

Treated like a failed air move: `Nav::PenalizeConnection` on the connection the bot is trying to
cross, then an immediate repath. `PlanWaypoint::connIndex` is only set on airmove launches, so the
connection is recovered from `plannedConnPath` (captured from `path.connPath` in `Repath`) at the
index where `plannedAreaPath` holds `currentAreaId`.

Airmove penalties are paired with `RewardConnection` on success and walk connections need that pairing
more, not less: the blocker may be a player who moves on a second later, and without the reward one
transient obstruction poisons a corridor for the rest of the map. `RewardEnteredConnection` fires when
`currentAreaId` advances to the next planned area over a connection that carries a penalty.
`kAvoidSearchCooldown` bounds the failure loop to one penalty per second when the map genuinely has
only one way through.

`crossbot_nav_avoid 0` disables the layer, which is how to A/B a suspected regression without
rebuilding.

### Obstacles that are leaving anyway

A `func_door_rotating` has `v.velocity == 0` — all of its motion is in `v.avelocity` — so without
help it reads as a stationary wall, gets penalised, and forces a repath over an obstacle that will
have removed itself within a second. `MoverVelocityAt` gives the linear velocity of a chosen point on
a mover, `v.velocity + omega x r`, and both the side ordering and the failure handling use it.

**`avelocity` is a rate on the angles triple `(pitch, yaw, roll)` in degrees per second, and the
mapping to world axes is not the identity.** From `AngleVectors`, `forward = (cp*cy, cp*sy, -sp)`, so
at yaw 0 a positive pitch swings `+X` toward `-Z`: pitch is rotation about **+Y**. `right` at
pitch = yaw = 0 is `(0, -cos r, -sin r)`, so roll swings `-Y` toward `-Z`: roll is about **+X**. Yaw
is about **+Z** — `avelocity (0,30,0)` is 30 degrees per second counterclockwise seen from above. So
`omega = (av.z, av.x, av.y)` in radians, and getting this wrong costs a sign, which is the whole
answer.

**`r` starts at the trace contact point, then is stretched to the model's own reach.** It cannot come
from `absmin`/`absmax`: whenever a `SOLID_BSP` entity's `angles` are non-zero, `SV_LinkEdict` throws
away the real bounds and publishes a cube of the largest half-extent found over all three axes,
centred on `v.origin`. `test9_door`'s door leaf is 18 x 66 x 98 and its box is measured at
`(318,78,-2)…(450,210,130)` — a 132-unit cube centred on its own hinge. Any radius taken from that is
meaningless, and a point on the wrong side of the hinge inverts the result.

Note the condition is `angles != 0`, not "is moving": a door that has finished opening and come to a
dead stop still sits at 90 degrees, so it keeps the cube permanently. Only a door closed in its spawn
orientation gets a tight box. That, not the swing, is why `PickDestination`'s lookahead is swamped
near an open rotating door. The contact point is a real position on the leaf, so its *direction* from the axis is right —
but its *distance* is whatever the probe happened to graze, and a probe that clips the leaf near the
hinge reports a few units per second for a door doing 100 degrees per second. Scaling `r` out to
`sqrt(max|mins.x|,|maxs.x|)^2 + (max|mins.y|,|maxs.y|)^2)` treats the leaf as the bar it is. The
magnitude can only be overestimated, and the sign — the thing being decided — never changes.

**The direction to dot against is the next waypoint, not `pathDir`.** `pathDir` is the follower's aim
point, which leads the bot around corner arcs and can already be pointing at the far side of a turn
while the bot still needs to travel the near leg. On `test9_door` the aim was `(0.16,0.99)` — north,
around the corner — while the door was sweeping `(+112,-24)` east out of the corridor the bot was
about to walk down. Dotting against the aim gave `-6` and the door was called a wall; dotting against
the next waypoint gives `+102` and it is correctly called receding.

Above `kAvoidMoverSpeed` a failed search sets `AVOID_FOLLOW` instead of `AVOID_BLOCKED`, and
`bot_move_nav.cpp` skips **both** the connection penalty and the forced repath, so the bot keeps
driving at its current waypoint and walks after the door. Three things bound that: the door reaches
its stop and `avelocity` goes to zero, at which point the next search penalises normally;
`kAvoidSearchCooldown` re-takes the verdict at most once a second; and the 2 s `kRepathInterval`
stall detector is the outer net if the classification is ever wrong. A stationary obstacle scores
exactly zero and keeps the old behaviour, which is what the `closing` field in the decision line is
for.

## bot_manager.cpp

### Death and respawn

`StartFrame` drives a dead bot with `IGameProfile::Respawn` instead of `Bot_MoveNav`, and it has to keep
doing so every frame: `SV_Physics` skips client edicts entirely, so a fake client's think only runs from
inside `pfnRunPlayerMove` (`SV_RunCmd` sets `v.button` from the usercmd, then `pfnPlayerPreThink`, and
`CBasePlayer::PreThink` dispatches to `PlayerDeathThink` whenever `deadflag >= DEAD_DYING`). Stop calling
`pfnRunPlayerMove` for a dead bot and the game DLL never gets another chance to respawn it.

**The button has to be released before it is pressed.** `PlayerDeathThink` gates the `DEAD_DEAD` →
`DEAD_RESPAWNABLE` transition on *no* button being held (`player.cpp`, "wait for all buttons released"),
and only the following call, with `deadflag` already `DEAD_RESPAWNABLE`, reaches `respawn()`. Holding
`IN_ATTACK` on every dead frame — which is what `CHLDMProfile::Respawn` used to do — therefore parks the
bot at `DEAD_DEAD` permanently; it also never reaches the `mp_forcerespawn` branch further down, so
nothing else rescues it. `Respawn` now sends zero buttons until `deadflag == DEAD_RESPAWNABLE` and
`IN_ATTACK` on that frame alone.

Nothing resets the bot's plan across a death. It does not need to: the respawn moves the bot far enough
that `Bot_MoveNav`'s `kTeleportJumpThreshold` guard calls `ResetPath` on the first live frame, and when
the spawn point is *close* to where the bot died the stale plan still starts within that same distance and
is followed correctly. `hasGoal`/`goalPoint` survive on purpose, so a respawned bot resumes walking to the
goal it was given.

`crossbot_kill` exists to make this testable from the dedicated-server console.

### Level changes drop every bot, and that is the engine doing it

`SV_InactivateClients` disconnects fake clients outright ("Dropping fakeclient on level change"), which
runs `pfnClientDisconnect` and so clears the `CBot` slot. Bots are therefore gone after `changelevel` or
`map` and have to be re-added with `crossbot_add`; this is not a plugin bug and there is no roster to
restore. Re-adding on the new map — and after a plain `kick` — was verified to produce a fully working
bot, so the reused client slot and its leftover `CBasePlayer` private data need no special handling.

The two commands tear down in opposite orders, which matters to anything that ever wants to hook this:
`changelevel` runs `SV_InactivateClients` (so `ClientDisconnect`) *before* `pfnServerDeactivate`, while
`map` goes through `Host_ShutdownServer`, which calls `pfnServerDeactivate` first and drops the clients
after.

### PrintNavStatus
Backs `crossbot_nav_status`, and is the only view of what the runtime planner actually decided —
`--dump-path` shows what navgen would plan for its own spawn/target pair, which is usually a different
pair and often the opposite direction. Prints:
- Origin, velocity, angles, move speeds
- Goal state, target waypoint
- Elapsed time vs ETA (with OVERDUE flag)
- Planned and actual area paths (truncated)
- Full waypoint list with jump/crouch flags

The pairs that carry the diagnosis:
- `vel` against `move` — `move` is the forward/side pair last handed to `pfnRunPlayerMove`. Large
  `move` with `vel=(0,0,0)` is the bot pressing at full drive and not moving: it is against geometry,
  not lost and not idle.
- `planned=` against `actual=` — planned is the route's area sequence, actual is the areas walked.
  Actual diverging once is a repath; actual cycling is the air-move loop described under
  `RunAirmoveApproach`.
- `waypoint=i/n` against the waypoint list — which maneuver it is stuck before. `n` also reveals a
  repath, since a fresh plan renumbers from the bot's position.
- `hasGoal=0` with `waypoint=0/0` is ambiguous by itself: it is both "arrived and cleared the goal"
  and "gave up". Distinguish with the distance to the goal, and remember the goal was clamped
  (see `doc/nav.md`, `PlanPath`).
- `item=` with `itemArea=`, `memo=`, `full=` — the gather goal: which entity the bot is after, the
  area it resolved to, and how many entries are in each memo. `item=-` with `hasGoal=1` is a
  commanded `crossbot_nav_goal`, not a gathered one. `charging=` appears only while the charger
  runner owns the frame, and carries the value it is watching.
- `avoid=` with `hit=` — the last obstacle decision (`duck`, `jump`, `side`, `blocked`) and the
  classname it was made against, plus `duckLeft` on the crawl latch, the `gap`/`top` the decision was
  made from, and `detour=` for how many spliced detour waypoints are still ahead. `avoid=none hit=-`
  on a bot stalled against something means the trace never qualified a hit, which is a different bug
  from a mistimed maneuver; `avoid=blocked` means it qualified and no route past it was found. Each
  new decision also prints its own line to the server console with distance, speed and trace count,
  which is what makes this tunable from one run — note those lines go to `qconsole.log`, not to the
  RCON transcript the test scripts capture.

## bot_weapons.cpp

What the bot is carrying, read straight out of `pvPrivateData` through the offsets `gamedata.cpp`
loaded. `BotWeapons::Refresh` runs from `BotManager::StartFrame` for every live bot, unconditionally
and with no caching: it is six short pointer chains, and a stamp check would cost more to reason about
than the walk costs to redo. `BotWeapons::Read` takes any player edict rather than a `CBot`, which is
what lets `crossbot_weapons` dump a human client's inventory with the same code the bots use.

### The four checks, and which one earns its keep

Every `CBasePlayerItem *` pulled out of `m_rgpPlayerItems` is a raw pointer into the game DLL's heap
with nothing vouching for it. Before any field is read: the pointer is non-null, its `pev` is non-null,
`pev->pContainingEntity` is a live edict with a plausible index, and — the one that matters —
**`edict->pvPrivateData` points back at the same object**. The first three would pass on almost any
plausible garbage; only the round trip out to the edict and back can fail on a pointer that is merely
*wrong*. That check is the whole reason `pev` at offset 4 is hardcoded instead of trusted.

A failure there is not recoverable and not transient — it means the offsets are wrong, so it goes
through `GameData::Fatal` like any other gamedata failure. A null pointer is different: that is the
normal chain terminator, and it just ends the walk.

The wide read is deliberate: `m_iClip` and friends live on `CBasePlayerWeapon`, past where a plain
`CBasePlayerItem` allocation would end. Half-Life has no non-weapon item in a player's inventory (the
suit is a `pev->weapons` bit, not an entity), so every chain entry is a weapon. A mod that adds one
would over-read, and that is the first thing to suspect if these numbers go strange on a new mod.

### Attack timers are stored raw

`nextPrimaryAttack`, `nextSecondaryAttack` and `nextAttack` are whatever was in memory, with no
normalisation, because there is no single right normalisation to apply. In retail Half-Life they are
**countdowns, not timestamps** — `UTIL_WeaponTimeBase()` returns `0.0` under `CLIENT_WEAPONS`, and the
player decrements them each frame and clamps at `-1`, so "ready" reads as `<= 0` and an idle weapon
sits at exactly `-1.00`. A handful of HLSDK paths write the absolute `gpGlobals->time + delay` form
anyway (`glock.cpp`'s empty-click being one), so a consumer that cares has to tolerate both. Deciding
that belongs to whatever fires the weapon, not to the reader.

### Ammo counts are indices, never names

`m_iPrimaryAmmoType` is an index into the player's `m_rgAmmo[32]`, assigned at runtime by
`CBasePlayer::GetAmmoIndex` in precache order. It is **not** stable across mods and must never be
hardcoded — on `crossfire` 9mm lands at 2 and rockets at 6, and nothing guarantees that elsewhere. It
also does not need to be: each weapon carries its own index, so `inv.ammo[w.primaryAmmoType]` is the
count without anyone knowing what the ammo is called. Resolving names would mean reaching
`CBasePlayerItem::AmmoInfoArray`, a static in `hl.dll` that the gamedata dump does not describe.

Non-zero `ammo[]` entries with no weapon using them are normal, not a bug: the gather goals pick up
ammo boxes for guns the bot does not have yet.

### The mask cross-check

`crossbot_weapons` ORs `1 << m_iId` over everything it walked and compares that against
`pev->weapons`, minus bit 31. `pev->weapons` is maintained on the engine-visible side of the entity, so
it is the one number in the dump that does not depend on a single offset being right — if the walk and
the mask agree, the offsets are good. Bit 31 is excluded because `WEAPON_SUIT` sets it and the suit is
not an inventory entity, so it can never appear in the walk.

## bot_move.h

### Bot_DecomposeMove
Splits a world-space direction into the forward/side speeds pfnRunPlayerMove takes, which the engine reads along the view axes -- so this is what lets the bot move somewhere other than where it is looking. viewYawDeg must be the yaw passed to pfnRunPlayerMove in that same call plus punchangle, because PM_CheckParamters builds the movement basis from exactly that.

PM_CheckParamters clamps the pair down to maxspeed before anything reads it, so a driveLength well over maxspeed just means "full speed exactly this way", and one below it passes through untouched and becomes the actual speed.