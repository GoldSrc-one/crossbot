# Goals: what the bot wants

What the bot wants and how it picks it: the goal tiers, detours, camp spots, and gathering.
Fighting interrupts all of this (`doc/combat.md`); walking toward it is `doc/movement.md`.

## bot_items.cpp

The lowest of the three goal tiers, and the only one on a stock HLDM map that is always populated.
`bot_goal.cpp` owns the tier decision and calls in here for candidates; this file knows about items
and nothing else. `crossbot_gather 0` turns it off and restores the old
stand-still-until-commanded behaviour — `test_nav_goal.ps1` sets it, for the reason under "Why the
nav sweep disables it" below.

`Collect` is the entry point: `ScanItems` plus the `SCAN_OK` filter, which used to be the front half
of a `SelectGoal` that also did the picking. The picking moved to `bot_goal.cpp` so every tier is
priced by one shared Dijkstra; what stayed is everything item-specific — the ignore table, the
respawn memo, `UpdateGoalItem`, and the charger and volume arrival behaviours.

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
forever: `AreaCosts` applies the activation gating that does not need a walker (`doc/path_planning.md`,
`GateAllowsCrossing`), so an area behind a door the bot can never open from this side is priced
`kUnreachableSeconds` and `crossbot_gather_debug` prints `no route` instead of a cost. On `crossfire` that is
the 12-item secret cache in area 8 — and those 12 are the only items on the map that should read `no route`.

Phase 1 must charge for **`travelSeconds` as well as `costSeconds`**, and getting that wrong is not
a subtle bug. `Connection::costSeconds` is a surcharge — door waits, flight time, ride time — and it
is **zero on a plain walk connection** (see `doc/path_planning.md`, travelSeconds). A Dijkstra over
`costSeconds` alone therefore prices most of the map at 0 and "nearest" degenerates into whichever
area the heap happens to settle first, which is roughly area-index order. The symptom is a bot that
walks past the battery at its feet to fetch something across the map, and items in high-numbered
areas that are never chosen at all. `crossbot_gather_debug 1` prints the cost per candidate, which
is how that was found: every far item read `0.0s away`.

Candidates are then ranked by `AreaCosts[area] + |item − reference| / kNavRunSpeed`, where the
reference is the bot's own origin for its current area and the area centroid elsewhere. Ranking by
area alone leaves ties between every item in the winning area, and the tie in the bot's *own* area
is exactly the "it is right next to me" case that matters most.

### Gathering never competes with fighting

Only items are candidates. An enemy used to be appended to the same scan as one more `Candidate`,
priced by the same Dijkstra and divided by an `EvaluateCombat` readiness, so a bot that saw someone
across the map planned a full A* route at them. That is gone: fighting is a state, not a goal, and
`BotCombat` takes movement away from the planner entirely for as long as it lasts (`doc/combat.md`, `bot_combat.h`). `SelectGoal` runs only when the bot is not in combat.

`crossbot_gather_debug` still prints the bot's readiness in the scan header, because `EvaluateCombat`
still drives carefulness.

### Carefulness

The same readiness drives *how* the bot travels, not only where. `Bot_Carefulness` returns
`(1 - readiness)` scaled by `crossbot_careful`, and both searches take it: `Repath` hands it to
`Nav::PlanPath` and `SelectGoal` hands it to `Nav::AreaCosts`. An armed, healthy bot routes for speed;
a bot with nothing to fight with routes for cover, however far around that takes it (`doc/path_planning.md`,
"Carefulness: routing away from exposure"). `BotOppo::RebuildCosts` is the one caller that must stay at
0 -- its gain test is a subtraction against real simulated seconds, so the exposure weighting must not
distort the cost values.

**It is sampled once, at repath time, and baked into the route.** A bot that arms itself halfway along
a timid route finishes that route; carefulness only refreshes on the repaths that already happen.

`crossbot_careful 0` disables the whole thing and restores pure fastest-route planning.
`crossbot_careful_pin` exists because readiness on a live bot does not reach the ends of its range --
a fresh spawn with a crowbar and a glock at full health sits around `0.35`, so the derived carefulness
never leaves the middle. Set it to a value in `[0,1]` to force one and sweep the range; `-1` (the
default) leaves it off. The `Repath` line reports `careful=`, `route=` (true seconds) and `score=`
(exposure-weighted) side by side, which is how you tell the factor actually reached the planner: at
`careful 0` the two are equal by construction.

### Range bias

Carefulness has a mirror. Every weapon is annotated `RANGE_CLOSE` or `RANGE_LONG` in `WeaponSpec`
(`doc/games.md`), and `Bot_RangeBias` turns the held weapon's class into a `NavPlan::RangeBias` whose
weight is `readiness` scaled by `crossbot_range_pref` -- the complement of carefulness, which scales
by `(1 - readiness)`. The two multiply, so neither is lost: a hurt, badly armed bot ignores weapon
range and just avoids exposed ground, while a healthy, well armed one shapes its route to suit what it
is holding. It goes to the same two searches, `Nav::PlanPath` from `Repath` and `Nav::AreaCosts` from
`SelectGoal`, and `BotOppo::RebuildCosts` leaves it at its zero default for the same reason it leaves
carefulness at 0.

What the route is weighed against is the connection's mean open sight distance in the direction it is
crossed (`doc/path_planning.md`, "Range bias: routing to ground the weapon can use"). A shotgun is happy below
`crossbot_range_short` (512), a crossbow wants above `crossbot_range_long` (1024), and in between the
penalty ramps. Both are cvars because the right numbers are per map: on crossfire the per-connection
open distance runs a mean of 530 and a max of 1584, so measure with `crossbot_vantage_status` before
trusting the defaults.

**Unlike carefulness, this one can invalidate a route in flight.** A route planned for a shotgun is
the wrong route once the crossbow comes out, so the nav update forces a repath when the live weapon's
class differs from `bot.plannedRangeClass`. Comparing the class rather than the weapon id means
swapping between two close-range guns costs nothing. The stamp happens only once the plan stands,
so a failed search does not consume the retry trigger.

`crossbot_range_pin` (-1) forces the weight the way `crossbot_careful_pin` does, and
`crossbot_range_force` (-1, else 1 = close / 2 = long) forces the class without holding the weapon.
Together they make an A/B of the same start and goal deterministic.

## Gathering: memos, outcomes, chargers and volumes

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

**`Nav::FindAreaNearest` is the wrong function here.** It is deliberately forgiving — after the
exact half-space test it falls back to nearest-centroid-with-AABB and then to global nearest
centroid, so it essentially never returns null. An item inside a wall would snap to whatever area
happened to be closest and the bot would route confidently to nothing.
`Nav::FindAreaExact` is the strict half of it, added for this. A healing volume takes a different
route again — see "Healing volumes give one entity several candidates" below.

Area z-space is duck-origin based (navgen builds in hull 3) and an item's `v.origin` is at its base,
so the probe is `origin + 18` first and the raw origin as a fallback.

**Both probes must also land at the item's height.** `FindAreaExact` only tests the xy half-spaces, so
an item on a shelf or a ledge resolves to the floor area under it and the bot routes to a point it
cannot touch. The area is now rejected unless the point clamped into it is within 72 units of the
item in z — one jump. Without that, four TFC demomen on crossover2 spent a whole five-minute run
cycling between six spawn-room pickups 110 units over their heads instead of walking to their
objective; the loop was visible as "reached ... without touching it" once per item per 30 s.

**Arrival does not mean touching.** The follower finishes a leg when the route is done, which can be
tens of units short of a small entity — on casbah, pickups were being abandoned from 60-120 units
away. A pickup that has not been touched on arrival gets up to 1.5 s of walking straight at the
item's centre before the bot gives up on it, which is how a bot finally touches backpacks sitting off
the nav area.

**Chargers have no origin at all.** They are brush entities: `v.origin` is `(0,0,0)` and the real
position is `(absmin + absmax) * 0.5`. That centre is inside the wall, so it never resolves to an
area either. The use spot is found by scanning areas whose bounds come within 96 units of the centre
and taking the closest point that is within 48 of it. 48 rather than HL's full 64-unit USE radius
because `NavFollow` stops within `goalRadius` (12) of the goal *horizontally* and can stop on the far
side, and 64 + 12 is out of range — the bot would stand there pressing USE at nothing until the
15 s timeout.

### Healing volumes give one entity several candidates

A `trigger_hurt` with negative `dmg` heals whoever stands in it, and maps use it as a pool —
`pool_party` has one at `dmg -40`, which is 40 hp/s. It is a gather goal like a charger, but the
geometry is the other way round. A charger is a fixture the bot walks *up to*, so one area beside it
is the answer. A pool is a room-sized brush the bot stands *inside*, so every area overlapping it is
a way in, and `ResolveVolumeAreas` emits a `Placement` for each — one `Candidate` per entrance,
sharing the entity. `ItemCost`'s Dijkstra field then picks the cheapest way in for free, instead of
this function guessing which side the bot will approach from. Capped at 8, nearest to the bot, so a
map-sized trigger cannot flood the scan.

**Its bounding box is not the pool.** This is the whole difficulty. A brush entity's `absmin`/`absmax`
is one box around every brush in the model, and `pool_party`'s heal volume is 674 x 733 x 80 units of
box holding basins that do not fill it. Worse, `SV_TouchLinks` does not settle for a box overlap
either — before calling `pfnTouch` it point-tests the *toucher's origin* against the trigger's clip
hull. A box test therefore fails twice over, and it fails quietly: the bot walks to a spot near the
pool, stands in it for the grace period, gains nothing, and blacklists every healing trigger on the
map for 120 s. Five of nine arrivals ended that way before `OriginInsideVolume`, which runs the
engine's own predicate through `pfnTraceModel` — the same call `Bot_PushFields` already uses on
`trigger_push`.

The probe point is where the bot's origin will *be*, not where the goal point is: area z-space is
duck-origin based, the bot stands unless the area is flagged `NAV_AREA_CROUCH`, so a standing probe
sits 18 units higher and uses hull 1 instead of hull 3. Skipping that lift is how the failures above
looked: spots at area z −768 and −780 sat under a pool whose top is −776, so the *duck* origin was
inside and the standing origin the bot actually arrived with was above it.

### RunHealVolume is RunChargerUse minus the aiming

No `IN_USE`, nothing to face — presence is the entire interaction. It reuses the charger fields on
`CBot` (`chargerActive`, `chargerStartTime`, `chargerLastGainTime`, `chargerLastValue`), with
`chargerPressed` meaning "has been inside" rather than "held USE", so `ManeuverActive` and the
`lastProgressTime` stamp keep working unchanged.

Two constants differ. The no-gain grace is 1.2 s, not 0.5: `HurtTouch` only re-applies every 0.5 s
(`pev->dmgtime`), so a working pool looks stalled for half a second at a time and the charger's grace
would abandon it mid-heal. And the timeout is split — `kHealApproachTimeout` (5 s) only runs while
the bot has never been inside, while `kHealTimeout` (30 s) is the hard stop, because a pool at
`dmg -5` legitimately takes twenty seconds to fill a bot and a single timer cannot serve both.

Because the view is not hijacked, the runner calls `BotSense::ApplyLook` before `Bot_UpdateFacing`
exactly as the idle branch does. A bot topping up in a pool keeps scanning the room and can track an
enemy while it heals, which a charger user cannot.

Drifting back out is not a useless pool: losing footing ends the goal with the short unreachable
timeout, not the 120 s classname ban. The runner drives at the resolved spot inside the volume,
not its centre, since a large pool's middle can sit inside geometry.

### Sated: asking before travelling, not on arrival

`ItemCost` prices goals on travel time alone, so nothing stopped a bot at full health from choosing a
charger — it walked there, gained nothing, and `NoteFull` blacklisted the classname for 120 s. That
was tolerable for chargers and much worse for pools, whose 120 s ban covers every healing trigger on
the map. `BotItems::Sated` compares `ChargerValue` against `ChargerFullValue` during the scan and
marks the candidate `SCAN_SATED`, printed as `no need`. It applies to chargers as well as pools;
this is the one place the change alters behaviour that already existed.

It is not health-need weighting. A hurt bot still has no extra pull toward health — a pool competes
with a shotgun on seconds of travel and nothing else.

### Why the nav sweep disables it

`test_nav_goal.ps1` polls `crossbot_nav_status` every 2 s and treats `hasGoal=0` as arrival. Gather
re-arms `hasGoal` on the frame after arrival, roughly 20 ms later, so the poll never observes it and
**every map in the sweep times out**. `crossbot_gather 0` in the boot commands keeps the sweep
measuring what it measured before. Use `scripts/test_items.ps1` to exercise gathering instead.

### Reading crossbot_gather_debug

`crossbot_gather_debug 1` makes every selection print the whole scan: one line per item entity with
its resolved area and either its cost in seconds or why it was skipped (`taken`, `empty`,
`respawning for Ns`, `ignored for Ns`, `full for Ns`, `no need`, `switched off`, `no nav area`,
`no route`), with `PICKED` on the winner. A healing volume is the one entity that prints several
lines, one per entrance, but only while it is selectable — a skipped one says the same thing from
every area, so it is reported once. It is the only view that distinguishes the four ways an item can be invisible to a bot,
and the costs are what proves the phase-1 search is sane — if far items read `0.0s away`, the cost
model is broken, not the mesh.

It is deliberately verbose: ~130 lines per selection on crossfire, at up to 2 Hz per bot while a bot
has nothing to go for. Leave it off unless reading the log.

## bot_goal.cpp

Owns *what the bot wants*, in three tiers: `GOAL_STRATEGIC` (`bot_strategic.cpp`), `GOAL_FUN` (`bot_fun.cpp`), `GOAL_GATHER`
(`bot_items.cpp`). It knows what none of them are. `Select` asks each in turn, highest first, and the
first tier with a surviving candidate wins outright — a bot with a tank to mount does not weigh it
against a shotgun, it goes to the tank.

`Select` still runs only where `Bot_MoveNav` finds `hasGoal == false`, so no tier preempts a goal in
flight. Re-tiering happens at arrival, not continuously; a higher tier appearing mid-route waits.

**One Dijkstra, shared.** `Nav::AreaCosts` runs once per selection and is handed to every tier, so
adding tiers does not add searches. The cost model is the one from "Selection is two searches, not
one" above, unchanged and non-negotiable: `travelSeconds + costSeconds`, plus the same
`|point − reference| / kNavRunSpeed` refinement so ties inside the bot's own area break sensibly.

**Gather picks the cheapest; fun and strategic pick at random, weighted toward closer.** The weight
is `1 / (cost + 2)^crossbot_goal_bias`. Nearest-first is right for items — the point of a battery is
that it is on the way — and wrong for gimmicks, because four bots that all deterministically pick the
nearest tank produce three bots queueing behind one. The `+ 2` floor keeps a goal the bot is standing
on from taking the entire weight.

**A profile can switch tiers off.** `CollectTier` returns nothing for a tier the bot's profile refuses
(`IGameProfile::AllowsGoalTier`, `doc/games.md`). That one gate covers the tier loop and the
pickup-priority promotion both. Nothing downstream needs its own check: fun behaviour only runs after a
fun goal mounted or boarded something, and camping needs a camp route, which `Select` only builds for
tiers above gather. Combat is a state, not a goal, and is unaffected. Ricochet admits `GOAL_GATHER`
alone.

### The distant target is sticky; only the leg is re-decided

`state.target` is the goal proper and is held across repaths. `state.stop` is where the current leg
ends, and it is what `bot.goalPoint` points at — `Repath` reads `goalPoint` and nothing else, so
changing legs is `goalPoint = …; Bot_ResetNavPath(bot)` and the existing A* does the rest.

No itinerary is ever materialised. `PickStop` runs at selection and again at each leg's end and
answers one question — where does *this* leg stop?

```
here = toGoal[botArea]
accept candidate c when   toGoal[c] < here                      strictly closer in
                    and   fromBot[c] + toGoal[c] - here <= crossbot_detour_budget
minimise fromBot[c]
```

`toGoal` is `Nav::AreaCostsToGoal`, a reverse Dijkstra that depends only on the target, so it is
computed once per *retarget* and cached in `state.toTargetCosts` with a `toTargetArea` guard — the
same shape as `bot.oppoCosts` / `oppoCostsGoalArea`. A repath therefore costs one `AreaCosts`, which
selection was already paying, not two.

`toGoal[c] < here` is what makes the walk monotone: every leg ends strictly closer to the target than
it started, so detours cannot chain forever. If they ever appear to, the reverse field is being
rebuilt against the wrong area.

A healthy transcript is one `wants` line followed by several `detours to` lines whose "left after"
figure decreases every time, then the arrival:

```
Player1 wants fun func_tankcontrols at (-368.0,-1576.0,-1284.0) in area 515, 18.5s away, 7 candidates
Player1 detours to item_healthkit  ... 0.7s out, 18.3s left after
Player1 detours to ammo_rpgclip    ... 1.7s out, 18.0s left after
Player1 detours to ammo_gaussclip  ... 3.1s out, 17.6s left after
Player1 detours to ammo_crossbow   ... 1.2s out, 17.5s left after
Player1 detours to ammo_rpgclip    ... 0.8s out, 17.1s left after
Player1 reached the tank func_tankcontrols
```

The failure signature is a fresh `wants` line where a `detours to` should be: the target is being
re-selected instead of held, and the bot oscillates between two goals forever without reaching
either.

### TargetValid must not read the bot's arrival state

This is the trap, and it is worth stating plainly because the code gives no hint.

`CommitStop` calls `BotItems::ForgetGoalItem` and `BotFun::Forget` on **every** leg, since the stop
it is about to commit may belong to a different tier than the last one. So while a bot is walking to
a gather detour on the way to a tank, `bot.fun` is empty — it holds the *stop*, not the target.

`TargetValid` therefore cannot ask `BotFun::Valid(bot)`, which reads `bot.fun.goal`. It asks
`BotFun::ValidFor(bot, ent, payload)` against the target's own entity instead. The version that read
per-bot state failed exactly this way: the fun target tested invalid the moment the first detour was
committed, `Advance` cleared it, and the bot re-selected from scratch on arrival at every single
item. It looked like weighted-random churn and it was a validity bug.

### Failure is per-leg

Phase-1 Dijkstra can still name a stop phase-2 A* cannot reach. On a **detour** that is not fatal:
`OnRepathFailed` logs `could not plan a route to the detour X, going straight on`, adds the entity to
`state.skipped`, and re-runs `PickStop` with the target intact. Only a repath failure against the
target itself goes through the owning tier's `OnRepathFailed` and kills the goal.

`BotCombat` clears the goal outright (`BotGoal::Clear`), target and cached reverse field included.
That is inherited behaviour — the gather tier did the same before there were tiers — and it means
that on a small map with several bots, fun goals get re-selected often. Fighting is not a goal.

## bot_camp.cpp

A bot walking to a goal that senses something **closing in on it** breaks off to a camp spot that is
still on the way, holds there facing the direction the spot was measured at, and resumes as soon as
nothing is getting closer any more. `NavMesh::camps` had been carried by the `.nav` since v21 with no
consumer (`doc/navgen.md`, `nav_camp.h`); this is the consumer.

Two constraints shaped it, and everything else follows from them:

- **A camp may never cost the bot ground on its target.** Same rule as the gather detour, same line of
  code: `if (in >= here) continue`.
- **The trigger cannot run a search.** Camping is a reflex. A Dijkstra on the frame a bot gets
  frightened is not a reflex.

### The route set: prepared once, tested for free afterwards

`BotGoal::Select` already holds both cost fields it needs when it picks a fun or strategic target --
the forward `AreaCosts` from the bot and the reverse `AreaCostsToGoal` to the target -- so `BuildRoute`
is called there and does one linear pass over `mesh.camps`, keeping what passes the gather-detour
acceptance test. `RefreshStrategic` rebuilds it when a moving strategic goal drags the field with it.
On crossfire that is 114 camps in, and between 4 and 79 kept.

Each survivor is flattened into a `RouteSpot` -- position, facing, flags, and `toGoal`, the
cost-to-target of its area. The forward cost is deliberately **not** kept: it was measured from where
the bot stood when the target was picked and is wrong a second later. `toGoal` is not, because it is a
property of the target, so the trigger can re-test "still on the way" against
`toTargetCosts[currentAreaId]` as the bot advances at the cost of two array reads. That is the whole
reason the set is prepared rather than searched for.

Gather targets get no route set, matching the rule that a gather target is its own stop and is never
detoured from. A bot sprinting for a health kit does not stop to camp.

### One tracker, two rules

"Something is getting closer" and "nothing has got closer for a second" are the same measurement, so
there is one running minimum rather than two mechanisms:

```
contact = BotSense::NearestLive(origin, crossbot_camp_radius, livingOnly)
if (contact && (!scared || dist < scareDist - 16))   scared = true, scareDist = dist, scareTime = now
if (scared && now - scareTime > 1.0)                 scared = false        // and stop camping
```

`NearestLive` has already applied the sense decay, so anything it returns is a contact the bot still
believes in, and **nothing further gates this on the slot's own age.** An age test was tried and
removed: footsteps arrive in bursts, and a window tight enough to mean anything flickered the tracker
on and off between them, several times a second. The cost of dropping it is that walking toward a
remembered position also reads as closing in -- which is the right answer for somewhere the bot heard
something a moment ago, and the 5s decay is what stops it being the answer forever.

Because the minimum resets with the scare, a contact that approaches, stops, and approaches again
frightens the bot twice. Because it is a minimum and not a last-known distance, an enemy that closes
and then retreats keeps the bot camped for exactly one second past its closest approach.

A visible teammate overrides the scare entirely:

```
friend = FriendInView(scare radius, live view cone)   // refreshed at most every 0.2s
if (friend)   scared = false                          // no commit, and a holder leaves
```

This is a hard override, not a distance tweak: backup in sight means the bot plays on. The check
runs only while a contact, a scare or a hold makes the answer matter; a quiet bot spends nothing.
It is gated on `Teams::Active`, so it is a no-op on deathmatch maps. It takes the closest live
teammate inside the scare radius and the bot's live view cone (90° when `v.fov` reads empty) and
spends a single traceline on that one friend, so an occluded closest friend masks a visible one
further out -- one trace was judged worth more than the corner case. A scoped holder keeps its
narrowed cone and so rarely calms down. The approach walk is not abandoned by this either; see below.

`Track` is called unconditionally, straight after `BotCombat::Update` and above every early return in
`Bot_MoveNav`. A gap in its sampling would read back as "nothing has come closer" the moment combat
handed control back.

### Committing, and the walk

The trigger picks the **nearest** qualifying spot within `crossbot_camp_reach`, which mirrors the
gather detour's `min out`. It is committed through `BotGoal::CommitCampStop` as an ordinary detour
stop carrying `Stop::camp` -- so the target survives it, the A* plans to it, `follow.done` detects the
arrival, and `OnRepathFailed`/`OnBlocked` already know to go straight on. The only new arrival case is
in `BotGoal::OnArrived`, which hands a camp stop to `BotCamp::OnArrived` instead of advancing past it.

**The reach radius is euclidean, so it cannot tell a camp across the room from a camp across a chasm.**
The planned route can. `CheckTravel` reads `followProfile.totalTime` on the frame after the commit and
abandons anything over 3s. On crossfire that fires on about a third of picks -- the map is vertical and
512 units of straight line is regularly a long way round.

**A stale scare does not abandon the approach.** This was the one thing the first live run got wrong:
applying the one-second rule to the walk as well as the hold meant the bot gave up on every camp it
ever picked, because a spot 350 units out is about a second away. The stale rule is about how long to
*stay*. Arriving somewhere it was going anyway costs the bot nothing, which is exactly what the "never
further from the target" filter buys.

### Holding

`BotCamp::Run` sits directly below `BotCombat::Run`, so an actual enemy always wins, and returns false
unless the bot has arrived. It parks the bot and refreshes `lastProgressTime` so the stall detector
does not read a bot holding a corner as a bot stuck on geometry. Then:

**The camp facing is the base, not an override.** `Run` writes the spot's yaw and pitch into
`desiredAngles` *before* calling `ApplyLook`, so `LOOK_SENSE` blends away from it toward whatever the
bot can hear. A camper that ignored the noise it was camping against would be worse than one that
never camped. With nothing to hear at all, the new `LOOK_CAMP` branch at the top of `ApplyVantageLook`
holds the measured facing -- ahead of the vantage blend, because the blend describes a neighbourhood
and the camp facing describes this exact spot.

**Stance is biased toward crouching:** duck unless standing sees more than 5% more
(`crouchScore >= 0.95 * standScore`), and always duck on `CAMP_NO_STAND`. A camper is a smaller and
steadier target, so the tie and the near-tie both go to crouch, and only a spot that genuinely needs
the extra 34 units of eye height gets the bot to stand up.

**Zoom is a property of the spot, not of the moment.** The spot's `spread` is the RMS angle its
visible targets sit off the measured facing, so twice it is the arc the camper actually has to sweep.
Zoom when that sweep fits inside the scope -- `spread * 2 <= zoomFov * crossbot_camp_zoom` -- and the
targets are far enough away for magnification to buy anything (`dist >= crossbot_camp_zoom_dist`).
`zoomFov` is `WeaponSpec::zoomFov` of whatever the bot holds when `BuildRoute` runs, so a bot with no
scope never plans to zoom. `BotFire::ZoomButtons` still owns the toggle -- it used to bail on
`!fire.aimValid`, which is precisely the camping case, and now asks `BotCamp::WantZoom` instead. The
camp module holds one opinion at a time and keeps holding it after it leaves, until the weapon agrees,
which is what gets the scope back down.

This replaced a nested-arc test (`arc[2] >= 0.9 * arc[0]` over FOVs 90/45/22.5/11.25). One spread says
the same thing in one number, and the arcs never carried a range at all, so the "is it far enough to
be worth a scope" half of the question could not be asked.

### On leaving

`Leave` (stale scare) and `Abandon` (unreachable, blocked, camping switched off) only reset camp state;
picking the next stop is `BotGoal::Advance`'s job at the call site, because `OnRepathFailed` and
`OnBlocked` already call it themselves. Either way a 5s cooldown stops a frightened bot chain-camping
its way across the map.

### Measured

120s on crossfire, six bots, `crossbot_gather 0` so every bot holds a detourable fun target: 63 camps
committed, 22 abandoned by the travel check, 26 reached and held, 21 ended by the stale rule at 1.0 to
2.3s each. The five that ended otherwise were taken over by combat, which is the correct outcome.

### cvars

`crossbot_camp` (1), `crossbot_camp_budget` (3s, the detour budget in `BuildRoute` -- tighter than
`crossbot_detour_budget`'s 6s because camping is a sidestep), `crossbot_camp_radius` (1024),
`crossbot_camp_reach` (512), `crossbot_camp_zoom` (0.85, the share of the zoomed FOV the sweep may
fill), `crossbot_camp_zoom_dist` (768, the range below which a scope buys nothing),
`crossbot_camp_debug` (0, gates the per-retarget route line and the scare transitions; the commit,
arrival and leave lines are plain `BOT_VLOG`).


## objectives.cpp

What a `cstrike`, `dod` or `tfc` map is *for*, read from the map's own entities. It replaced the
`bot_goal.sma` AMXX plugin for CrossBot's bots. The plugin keeps running for foxbot, Parabot and
marine_bot_dod, and `crossbot_objectives 0` hands CrossBot back to it, which is the A/B switch.

### Keyed on the map owner, not the bot

`Objectives::DetectMap` runs next to `Teams::DetectMap` and picks a reader from
`XPlay::GameDir(XPlay::MapOwner())`: `cstrike`/`czero`, `dod`, `tfc`, or none. Every other owner gets
no reader, the strategic tier sees only mapper-placed `bot_goal` entities, and bots play fun and gather
exactly as before. The reader cannot live on `IGameProfile` -- that is the *bot's* game, which is `valve`
on every one of these maps -- so `CollectStrategicGoals` is gone. The offsets it reads are the map
owner's too; see `doc/games.md`, "Objective offsets belong to the map, not the bot".

### Three tables, not N game modes

A reader fills three tables and decides nothing:

| table | CS | DoD | TFC |
| --- | --- | --- | --- |
| **Site**: act on a place | bomb target, rescue zone, VIP safety zone, escape zone, planted C4 | `dod_capture_area`, touchable `dod_control_point`, TNT `func_breakable` | touch `info_tfgoal` that scores, detpack `info_tfgoal` |
| **Payload**: a loose thing to carry | dropped C4 (`weaponbox` + `m_bIsBomb`), free hostage | `dod_object` | `item_tfgoal` |
| **Carrier**: a player bearing one | `m_bHasC4` (hidden from CT), `m_bIsVIP`, the player a hostage follows | `CObject::m_hOwner` | the item's `pev->owner` / `aiment` |

A site carries per-team attack and defend masks and an optional need, a payload id or group. A carrier
carries the id and group of what it bears. `BotStrategic::Intents` turns the tables into goals with one
set of rules, so a VIP, a bomb carrier, a hostage rescuer and a flag runner are the same thing to every
other bot:

| relation | intent | rank |
| --- | --- | --- |
| a detpack site I can blow (TFC demoman) | `attack` | 4 |
| I carry what a site needs, or the site is urgent | `deliver` / `attack` | 3 |
| a loose payload my team carries, under the shared cap; an urgent site I defend | `fetch` / `defend` | 2 |
| a site I attack that needs nothing, or whose need is not lying loose | `support` | 1 |
| a payload my team defends | `defend` | 1 |
| a site my team defends | `defend` | 0 |
| nothing at all | `roam` to a spawn | 0 |

There is no escort or hunt. A friendly carrier is not a target; it is the runner a `support` bot plans
around (`bot_tactics.cpp`). An enemy carrier is not known to anyone who has not sensed it. When a bot has
both rank-1 `support` and rank-1 `defend` intents, `BotStrategic::State::defender`, rolled once per life,
keeps one kind and drops the other, so a two-sided map splits its team by coin flip rather than by board.

`BotStrategic::Prioritize` keeps the highest rank present, `KeepCheapestPerEntity` keeps one entrance
per volume so a brush with eight entrances is not eight tickets, and the ordinary `1/(cost+2)^bias`
roulette picks among what is left. The hostage cap is the plugin's, `max(1, hostages / live teammates)`.
The rank rides in `Candidate::payload` above the kind, so the tier needs no new fields.

### Holding

What ends a native hold is the intent disappearing: `ValidFor` re-derives the intents and looks for the
same entity and kind, so the area being taken, the hostage following, or the flag being picked up all end
it with no special case. `attack`, `deliver`, `fetch` and `defend` also give up after 30 s and memo the
entity for 20 s. That rotates defenders between sites and stops an attacker waiting forever on a numcap-2
area it cannot take alone. `roam` memos its spawn for 60 s on arrival. Hostage `+USE` is pulsed at 0.2 s,
because a hostage wants a press edge; the planted C4 is held, because defusing is continuous use.

A point goal that is itself solid is placed short of it: 40 units toward the bot for a `+USE` fetch,
inside the 64-unit use range, and 96 for `defend` and `escort`, falling back to the entity's own point when
that spot resolves to no area. Aimed at the hostage's own origin, the route ended inside the hostage and a
CT on cs_assault logged `could not get past hostage_entity` five times before its first `works
hostage_entity`. `escort` and `hunt` also re-target only after the player has moved 160 units rather than
32, and a bot that is carrying something never escorts, which is what stopped two carriers on dod_escape
chasing each other and re-planning at 4 Hz.

### Measured facts the readers rest on

- **CS `pev->team` is the reverse of `CBasePlayer::m_iTeam`.** Every bot on `de_dust2` read
  `pev->team 2 m_iTeam 1`. The reader keys on `pev->team` (1 = CT, 2 = T), because that is the number
  `Teams::TeamOf` and every other gate use; ReGameDLL never writes `pev->team` for a player at all.
  `crossbot_list_objectives` prints both columns so this can be re-checked.
- **DoD's `point_can_allies_touch` key means the opposite.** A value above 0 sets `m_bAlliesCantTouch`
  (dod.c:118399), so the reader reads the bool, never the key. On `dod_avalanche` every point is
  cant-touch for both teams and all five objectives are capture areas.
- **A DoD TNT breakable's team bits are optional.** `dod.fgd` gives 16 Allies Only, 32 Axis Only and 64
  Object Cap Only. With neither team bit the charge belongs to whichever team can carry its
  `CBreakable::sz_Group` object, so the reader takes the attackers from the payloads and carriers of that
  group, and both teams only when no object matches. Reading the team bits alone dropped `dod_escape`'s
  `blow2` (`*178`, spawnflags 64, `axis_obj`), the charge that actually scores through `rebar` ->
  `dod_point_relay` -> `axis_flag`, while keeping the bridge charge beside it (`*204`, spawnflags 96).
- **A TFC goal's team can hide behind `teamcheck`.** `APMeetsCriteria` compares the activator's team
  against `GetTeamCheckTeam` (tfc.c:111208), which looks up an `info_tf_teamcheck` by targetname and
  reads *its* `team_no`; the goal's own `team_no` may be 0. avanti's church-hole detpack goal is
  `teamcheck "attackers"`, so reading `team_no` alone made every demoman on the map think the goal was
  theirs. The reader resolves `teamcheck` for sites and `owned_by_teamcheck` for payloads, falling back
  to the plain fields.
- **A detpack goal is a site with nowhere to stand.** `goal_activation & 2` goals are usually point
  entities inside the wall they open, so the ordinary "resolve the entity to a nav area" gives a point
  on the far side. Their sites are placed instead on every nav area within 1400 units that has a clear
  line to the goal — the same trace the blast itself does (`DetpackExplode` 22955) — and the cheapest
  reachable one wins. `Tables::detpackMap` records that the map has such goals at all, which is what
  stops a scared demoman from wasting the pack elsewhere.
- **A higher rank only preempts through `Outranked`.** A strategic goal is otherwise sticky until it
  dies or the hold times out, so a detpack site appearing after the bot took a flag would never be seen.
  `RefreshStrategic` drops the current goal when an intent at the override rank is available and not on
  cooldown, at most once every 10 s per bot. The cooldown matters: without it a bot whose override
  intent is unreachable dropped and re-picked its goal every refresh — 1657 times in one five-minute
  run before the limiter went in.
- **TFC `items_allowed` is a goal number, not a mask.** `APMeetsCriteria` (tfc.c:111822) passes only if
  the `item_tfgoal` with that `goal_no` has the activator as `pev->owner`; `has_item_from_group` is the
  same test on `group_no`. A touch goal fires only with `goal_activation & 1` and `goal_state != 1`
  (tfc.c:114591), and the reader mirrors the `if_goal_is_*` / `if_group_is_*` conditions against live
  `goal_state`. That polling is what replaces FoxBot's per-map scripts, which are almost entirely
  waypoint groups switched on broadcast strings.

### What it does not do

Rocket- and grenade-only DoD breakables (spawnflags 512, 256, 8) and `dod_point_relay` chains are not
read: a `valve` bot has neither, and a relay only says which point a breakable leads to. VIP detection only sees real `cstrike`
players. TFC VIP maps get their scoring escape goal but no civilian hunt.

## bot_strategic.cpp

The top tier. It has two producers: the native intents above, and `bot_goal` entities. With
`crossbot_objectives 1` on a map `objectives.cpp` reads, a `bot_goal` owned by a player is ignored and
only ownerless, mapper-placed ones still count; everywhere else both kinds are consumed as they always
were.

### The wire protocol

`bot_goal` is not a registered entity class anywhere. It is a bare `info_target` whose
`pev->classname` the plugin rewrites at runtime, so **only `entvars_t` fields carry meaning, and no
game context is needed to read one** — `doc/games.md` rule 3 does not apply, nothing here
touches private data.

| field | meaning |
| --- | --- |
| `v.classname` | `bot_goal` is live; `bot_goal_disabled` is one a map trigger switched off |
| `v.origin` | where to go |
| `v.owner` | non-null: the goal belongs to that player edict alone. Null: global, everyone's |
| `v.aiment` | optional entity to look at and press buttons on (a hostage, the C4, a map button) |
| `v.button` | bitmask of `IN_DUCK` / `IN_USE` to hold once close enough |

Two producers feed it. The plugin's `goal_think` runs at 1 Hz and creates one **owned** goal per
cross-game bot, which is exactly CrossBot's bots, moving its origin as the objective changes. A
mapper can also place a **global** one, in the `.map` or in a `maps/<name>.ent` entity-lump override;
those carry no owner and every bot sees them.

The distance rule is copied from the three sibling bots rather than invented:
`dist = max(len2D(dir), |dir.z|)` measured from the eye, gated on `dist <= 128` **or** a clear
traceline to the aiment; then `dist <= 128` enables `IN_DUCK` and `dist <= 64` enables `IN_USE`. The
aim point is the aiment's bbox centre, which is Parabot's refinement — a hostage's origin is at its
feet, and aiming there from two metres away misses the use trace.

### A goal that moves is a re-plan

`Select` bakes `target.point`, `target.areaId` and the reverse field `toTargetCosts` once. A per-bot
`bot_goal` moves every second, so all three go stale, and `BotGoal::State` therefore also carries
`targetOrigin` — the raw `v.origin` at bake time, which is **not** the same as the clamped nav
`target.point` and cannot be compared against it.

`BotGoal::RefreshStrategic` runs from `Bot_MoveNav` immediately before `Select`, at most every
0.25 s. While the entity has not moved further than `kGoalMoveRadius` (32 units) it does nothing.
Past that it re-resolves the area, refreshes `targetOrigin`, recomputes `AreaCostsToGoal`, re-runs
`PickStop` and resets the path. If the entity is gone, or is no longer a `bot_goal` this bot may have,
the goal is cleared outright and the next `Select` starts over.

### Arrival is a hold, not a touch

`BotStrategic::RunArrival` sits at the `follow.done` site next to `BotFun::RunArrival` and returns
true to keep holding, re-entered every think because `NavFollow::Track` keeps reporting `done`. It
faces the aiment, mirrors `v.button` into the bot's buttons, and stays. `v.button` is authoritative and
the plugin rewrites it every tick, so there is nothing to latch and nothing to pulse.

Standing still **is** the objective for a DOD capture area or a TFC dropoff, and the plugin is what
ends the hold: it moves or deletes the goal when the objective flips, and its roaming fallback
re-rolls once the bot is within 256 units. So for a `bot_goal` entity `RunArrival` returns false on
exactly two conditions — the entity died, or it moved further than `kGoalMoveRadius` — and the
ordinary `reached goal` path then re-selects against wherever it went. Native intents add two exits of
their own; see `objectives.cpp`, "Holding". Combat needs no special case: `BotCombat::Run` runs earlier
in `Bot_MoveNav` and clears the goal.

A repath failure or a hard block puts that goal on a 10 s per-bot memo rather than killing the tier,
so an unreachable objective lets fun and gather have a turn instead of trapping the bot in a
re-select loop against something it can never walk to.

## bot_tactics.cpp

Decides where a `support` or `defend` bot actually stands. There is no team board: a bot knows where its
friends are and nothing about what they intend, and every bot applies the same rule, so spreading out and
leapfrogging are what falls out of it rather than something coordinated.

### One primitive: a camp spot ahead of a runner

`BotTactics::Pick` runs from `PickStop` in `bot_goal.cpp`, before the gather detour, whenever the strategic
target is a `support` or `defend`. It picks a `mesh.camps` spot `c` that lies ahead of a *runner* on its way to
the target, and that the bot reaches first:

```
toTarget[c] < toTarget[runner]                                   ahead of the runner
fromRunner[c] + toTarget[c] - toTarget[runner] <= budget          on a near-ideal runner -> target path
clock + fromBot[c] + margin < fromRunner[c]                       the bot arrives first
not in the bot's spot memo, no friend within 96 units
weighted fit/(fromBot + 2)^crossbot_goal_bias
```

`fit` makes the spot suit the held weapon: `max(0.1, 1 - w * mismatch(dist))`, where `dist` is the camp's
`standDist` or `crouchDist` for the stance it will hold, and `w` and `mismatch` come from the routing range
bias (`Bot_RangeBias`, `crossbot_range_short/long/pref/pin/force`) without its readiness factor. A long
weapon favours far-seeing spots, a close one near ones; a weapon with no range class leaves `fit` at 1.

That is the gather detour's acceptance test with the runner in place of the bot. Nothing stores anyone's
path: an area is on a near-ideal path exactly when the two fields around it sum to the direct cost, so a
human teammate and a bot teammate are the same thing to it.

| job | runner | fallback when nothing qualifies |
| --- | --- | --- |
| `support` with a leader | the nearest friendly carrier whose payload the site needs (`BotStrategic::Leader`) | the site itself |
| `support` without one | a random live friend, weighted toward nearer | the site itself |
| `defend` | a random enemy spawn from `Objectives::Tables::spawns` | a spot within 4s of the target, else the target |

Every pick lands strictly closer to the target than its runner. A bot behind its friend jumps ahead of it; the
friend, picking in turn, jumps ahead again; and once no spot is left between a runner and the site, the bot
goes to the site. Leaderless attackers leapfrog in and converge instead of waiting on each other.

### The defender's clock is in its head

A defender assumes the enemy spawned when it did. `clock` is seconds since `BotStrategic::State::spawnTime`, so
the imagined enemy has already walked `clock` seconds of its path. Straight after spawning almost every spot on
that path passes the race and the defender ranges out along it; as the clock runs, only spots far from the
enemy spawn, which is near the site, still pass; past the enemy's travel time to the site nothing does and it
holds a spot near the site. The same rule serves CS rounds and DoD/TFC respawns.

`spawnTime` resets in `BotStrategic::OnRespawn` and in `NoteLife`, which runs every frame beside
`BotCamp::Track`. `NoteLife` covers what `OnRespawn` misses: the first spawn after `crossbot_add` (a bot starts
with `wasDead` false) and a CS new round, where survivors are moved to a spawn without dying. It resets when
the bot moved more than 128 units in one frame and landed within 64 of one of its own team's spawns.

### Fields

`toTarget` is `bot.goal.toTargetCosts`, already computed for the target. `fromBot` is the forward field
`Select` or `Advance` just ran. A friend runner costs one `Nav::AreaCosts` per pick; an enemy spawn's field is
cached per spawn area until `BotTactics::ForgetMap` at map change. Runner fields use default planner parameters:
the runner is assumed generic.

### Holding, re-picking, failing

The spot is committed by `BotCamp::CommitTimed` as an ordinary camp stop marked `timed`, so arrival, facing,
crouch and zoom are the camp module's. A timed hold ignores the scare and friend-in-view rules and ends on
its own clock: 0 to 10s for `support`, 5 to 15s for `defend`. Leaving calls `BotGoal::Advance`, which runs
`PickStop` and therefore `Pick` again against the same target, with a fresh runner.

Every picked spot goes into `BotCamp::State::memo` for 30s, so the next pick differs. A raced spot fails for
the rest of the life on either of two checks:
- **The planned route loses.** `CheckTravel` reads `followProfile.totalTime` on the frame after the commit, as
  it already does for scare camps. The comparison is made in one unit. The pick compared two field estimates,
  and the planned route runs well above its field estimate: on de_dust2, 5.3s planned against 3.0s
  estimated, and 12.7 against 6.5. Comparing the planned time with the runner's raw field time made
  nearly every support pick lose. The runner's estimate is scaled by the bot's own planned/estimated ratio,
  and the spot fails when `planned + clock + margin >= runner * planned / estimated`.
- **Something closes in on the bot before it arrives.**

A frightened bot skips the race entirely: `support` goes for the site, `defend` only considers spots near the
target. Near-target spots carry no race, so neither check abandons them. Before that, a defender
walking to one while a contact closed in abandoned it, re-picked the next near spot, and abandoned that too:
62 such aborts in four minutes, and the bot burned every spot around the site for the rest of its life.

Log lines: `supports X toward Y, camps at spot N (...)`, `guards Y against spawn N at spot M (...)`,
`falls back to spot N near Y`, `lost the race to spot N ... not trying it again this life`, and
`starts a life as an attacker|a defender`.

### cvars

`crossbot_tactics` (1; 0 keeps the intents but never picks a spot, so `support` walks to the site and `defend`
guards it -- the A/B lever), `crossbot_tactics_budget` (3s of path slack), `crossbot_tactics_margin` (1s the
bot must beat the runner by), `crossbot_tactics_debug` (0, adds the no-spot line).

## bot_fun.cpp

The middle tier: the map gimmicks that make an HLDM map what it is. Three kinds ship —
`FUN_SWITCH` (a button or trigger whose chain reaches something that hurts people), `FUN_TANK` (a
mounted gun) and `FUN_TRAIN` (a drivable `func_tracktrain`). Switches outrank the other two — see
below. `crossbot_fun 0` empties the tier, at which point `BotGoal::Select` reduces exactly to the old
gather-only behaviour.

**This is entirely a runtime feature. No nav format change, no `kNavVersion` bump, no navgen work.**
Everything a fun goal needs is either a plain `entvars_t` field or an offset already present in
`gamedata/valve-win.json` (`doc/games.md`, gamedata.cpp).

### Switches outrank everything else in the tier

`BotFun::Prioritize` is a hard filter, not a weight: if any surviving candidate is a `FUN_SWITCH`,
every tank and tram candidate is erased and the roulette runs over switches alone. Both switch
actions count, so crossfire's `trigger_multiple` nuke ranks with the buttons.

**It runs in `BotGoal::Select`, after `PruneUnreachable` and before the tier's emptiness test**
([bot_goal.cpp:321](../src/bot/bot_goal.cpp:321)). That position is the whole design:

- Filtering earlier — inside `BotFun::Collect` — cannot work. `Collect` is never handed the `costs`
  vector, so it cannot tell a reachable switch from an unreachable one, and a switch the bot has no
  route to would suppress every tank and leave the tier empty. The bot would fall through to gather
  and ignore a working tank standing next to it.
- Filtering after the emptiness test would be too late; the tier would already have been accepted.

The memo gate supplies the rest for free. A switch on cooldown emits no candidate at all, so
crossfire's tanks become selectable again during the nuke's `wait`, and undertow's 142 s button
simply empties the tier rather than blocking it.

**On stock maps this rule is currently a no-op**, which is worth knowing before debugging it. Only
crossfire and doublecross carry both a switch and tanks, and on both the switch is unroutable in the
mesh — crossfire's nuke has a nav area after `strike_ready_door` opens at t=180 but still prices as
unreachable, and doublecross's `fire_button_texture` at area 304 never survives the prune. Every
other fun map has exactly one kind. `crossbot_fun_debug` logs `fun priority kept N switch candidates,
dropped M others` whenever the filter actually bites; if that line never appears, the filter is not
broken, there is simply never more than one kind to choose between.

### The chain table

Built once per map into a file static, keyed on `gpGlobals->mapname`, cleared by `ResetMap` on both
`ServerActivate` and `ServerDeactivate`. It is pure map data, so it is not per bot.

From every `func_button` / `func_rot_button` / `trigger_multiple` / `trigger_once`, breadth-first
over `pev->target` → matching `targetname`, depth ≤ 6, cycle-guarded, passing through `trigger_relay`,
`multisource` and `multi_manager` and stopping anywhere else.

**`multi_manager` targets must be read from `m_iTargetName[0 .. m_cTargets)` through gamedata, never
from the map keys.** Worldcraft writes duplicate keys with `#N` suffixes — crossfire's `strike_pain`
and `strike_pain#1` are the on and off edges of the same 0.1 s kill window — and only the engine's
array is stripped and delay-sorted. A `multisource`'s *inputs* are a reverse edge: a button naming a
multisource is arming it, not firing through it.

A chain is interesting if it reaches a `trigger_hurt` with `dmg > 0`, a `func_water` (or a `func_door`
with `skin < 0`, which is the same class), an `env_explosion` without `SF_ENVEXPLOSION_NODAMAGE`, a
`func_mortar_field`, or a `monstermaker`. It is not interesting if the only terminals are
`func_door` / `func_plat` / `func_train` / `func_breakable` — navgen already models those as
activation detours, so crossfire's four `lift*` buttons stay navigation. `trigger_push` is excluded
for the same reason: mobility is navgen's business.

**Dangling targets are the norm, not an error.** crossfire has 8 and doublecross 4. A chain that
resolves to nothing yields no terminal, and that is precisely the filter that drops crossfire's four
inert tower buttons — the two-tower arming mechanic was cut and the names they fire do not exist —
and stalkyard's `armorall` gag button, whose only target is an `ambient_generic`.

One consequence to keep in mind: an `env_explosion` deletes itself after firing unless
`SF_ENVEXPLOSION_REPEATABLE`, so the table has to tolerate a terminal edict vanishing under it.

### Activation is three different actions

`CBaseButton::Spawn` decides which, and guessing wrong means a bot standing at a button pressing a
key that was never wired up:

| condition | how the bot activates it |
| --- | --- |
| `health > 0` | **shoot it** — `takedamage` strips `FCAP_IMPULSE_USE` |
| `spawnflags & 256` (`SF_BUTTON_TOUCH_ONLY`) | **walk into it** — `SetUse` is never installed |
| a trigger brush | **walk into it** |
| otherwise | **`+USE`** |

The `SHOOT` path hands off to `BotBreak`. No stock DM map has a shootable button whose chain is
interesting under the rule above, so it ships detected-but-unexercised; detecting it still matters,
because without it the bot would press `+USE` at a button that has no `SetUse` until it timed out.

The single most famous goal in HLDM is not a button at all: crossfire's nuke is a 16×14×19
`trigger_multiple` at `(0, -2219, -1846)`, `wait 60`. A detector keyed on `func_button` misses it.

### "Cannot be activated" is a real test, not a guess

A fun goal the bot cannot act on is not offered. For a `+USE` or shoot switch that is
`m_toggle_state == TS_AT_BOTTOM`; `TS_GOING_UP` / `TS_GOING_DOWN` is mid-move, and `TS_AT_TOP` is
waiting out `m_flWait` or, with `wait -1`, latched forever.

This has to come from gamedata. Undertow's flood button is `wait 142` **and** `spawnflags 1`
(`SF_BUTTON_DONTMOVE`): it does not move when pressed, so no geometric test can tell whether it is
waiting.

For a touch trigger it is `v.solid != SOLID_NOT` plus a hull-3 trace from the containing area's
stand point to the clamped point inside the volume. That is what makes crossfire's nuke correctly
report `no nav area` before t=180, while `strike_ready_door` still seals the bunker.

For a tank it is `spawnflags & SF_TANK_CANCONTROL (0x20)`, `m_pController == NULL`, and — if
`m_iszMaster` is set — the named `multisource`, read as `m_rgTriggered` against `m_iTotal`.
(`CMultiSource::m_iState` does not exist; `IsTriggered` reads those two.)

### UTIL_ClampVectorToBox is not a clamp

The one finding that cost the most time. `PlayerUse` tests

```c
vecLOS = UTIL_ClampVectorToBox( VecBModelOrigin(pev) - eye, pObject->pev->size * 0.5 );
flDot  = DotProduct( vecLOS, gpGlobals->v_forward );   // must exceed VIEW_FIELD_NARROW = 0.7
```

and `UTIL_ClampVectorToBox` (util.cpp:1439) **subtracts** the half-extent per axis, zeroes any axis
the eye is already inside, and returns the result **normalized**. It does not clamp anything. `flDot`
is therefore a true cosine measured toward the nearest *face* of the brush, not toward its centre.

Aiming at the box centre is the intuitive thing to do and it fails on tall brushes. Standing beside
stalkyard's 12×70×100 controls brush and looking at its centre is 54° down, cos 0.58, under the 0.7
gate — the bot pressed `+USE` correctly, forever, and got `common/wpn_denyselect.wav` every time,
with every gamedata read verifiably correct. Small brushes hide it, because centre and nearest-face
directions nearly coincide; undertow's button worked from the first try.

`RunArrival` therefore faces along `UseDir` — the same subtract-and-normalize — and re-checks the real
cosine against a 0.75 floor before pressing, rather than trusting the facing convergence alone.

Range is the engine's, too: `UTIL_FindEntityInSphere` measures **box distance from `pev->origin` to
`absmin`/`absmax`** (rehlds `pr_cmds.cpp:864`), from the feet, not the eye, radius 64. Centre
distance is the wrong number and is far too pessimistic on a long brush. `kFunUseRange` is 56, inside
the engine's 64.

### `+USE` is edge-triggered, and one press is easy to lose

`m_afButtonPressed = (m_afButtonLast ^ pev->button) & pev->button` (player.cpp:1849), so holding
`IN_USE` yields exactly one edge, ever. A single one-frame press is nonetheless unreliable, because
`PlayerUse` is reachable only from `ImpulseCommands` ← `ItemPostFrame`, which returns early on
`m_flNextAttack`. `RunArrival` pulses `IN_USE` on and off every 0.2 s instead, and `BotFire::Think`
gets a `BotFun::Pressing(bot)` early-out alongside the `BotBreak::Active` / `BotDeploy::Active` ones
so the weapon code cannot push `m_flNextAttack` out from under it.

### The tank

`func_tank` has no use caps at all; only the `func_tankcontrols` `SOLID_TRIGGER` brush does, and it
resolves its tank once at spawn from `target`. `TankOfControls` reads `CFuncTankControls::m_pTank`
and falls back to a name match on `strncmp(cls, "func_tank", 9)`.

Placement prefers standing *beside* the controls volume (`BotItems::StandArea`) over standing in the
middle of it, with volume areas within `kTankReachRange` as fallbacks. Standing at the centre of a
tall brush is the geometry that trips the dot test above.

Two mechanics shape the whole state machine:

- **`m_vecControllerUsePos` is the player's origin at the instant of mounting**, and the leash is
  `< 30` units, 3-D. A mounted bot must hold its feet still — `Run` sends zero move input. A mount
  immediately followed by an unexplained dismount is this, and nothing else.
- **`+USE` cannot dismount.** `ItemPostFrame` returns immediately while `m_pTank` is set
  (player.cpp:3432), so `PlayerUse`'s dismount branch is unreachable. The bot dismounts by walking
  more than 30 units away. There is no other mechanism.

Aiming is indirect. The tank slews its own `pev->angles` toward the controller's `v_angle` (pitch
inverted) at `m_yawRate` / `m_pitchRate` deg/s, clamped to `m_yawCenter ± m_yawRange` and
`m_pitchCenter ± m_pitchRange`, and bullets leave along the **tank's** angles, not the crosshair. So
`IN_ATTACK` is held only once the tank's own angles are within `kTankAimTolerance`; firing on the
crosshair sprays while the barrel is still swinging.

### Three ways off the tank, and only one of them is boredom

**Boredom.** No enemy seen for `kFunBoredom` (10 s) → dismount, record
`abandonReadiness = EvaluateCombat(...)` against *that* tank, and filter it out of `Collect` for this
bot until readiness rises above that mark. That is what sends the bot to resupply and come back
rather than remounting the same gun in the same state.

**Shot from outside the cone.** A mounted bot has no weapon and a barrel that only covers
`m_yawCenter ± m_yawRange`; an attacker outside that arc cannot be answered, so staying put is
standing still to be killed. `BotCombat::DamageSource` is read each frame while mounted — and it must
be read **before** `Bot_RunPlayerMove` for the frame — and a source outside the traversable cone, or
one whose position is unknown, dismounts at once. `BotCombat` picks the bot up on the next tick,
because that same damage is its entry condition.

This exit is deliberately **not** boredom: it records no `abandonReadiness` and leaves no readiness
gate, so the bot may come straight back once the fight is over. It takes only a short flat cooldown
on that tank, purely so a bot being spawn-camped from behind cannot loop mount → shot → dismount.
Damage from *inside* the cone is not a dismount at all — that is what the gun is for.

**The tank died, or the bot did.**

### What is deliberately not here

The bot does not reason about surviving its own gimmick. Pressing the crossfire nuke starts a 52 s
fuse and seals the bunker at t=20; the activator is inside the bunker, so a bot that presses and stays
put happens to live. doublecross poisons that same bunker for 115 s afterwards. Evacuation is real
work and belongs with strategic goals.

Nor do bots cooperate. rocket_frenzy needs three separate buttons armed before its launch button
unlocks; rustmill's bunker door needs two held at once. The `multisource` read makes a bot correctly
*skip* a locked button; it does not make two bots unlock one.

`func_mortar_field` with `m_fControl == 1` shells **the activator**, so pressing that switch bombs
the bot that pressed it. `m_fControl` is not in `gamedata/valve-win.json` and cannot be read the way
the others can. No stock DM map puts one behind a switch, so it is treated as interesting and left
alone.

`func_vehicle`, `func_tanklaser`, `func_tankrocket` and `momentary_rot_button` have zero occurrences
across all 19 DM maps, so none of them is handled. **No stock DM map has a `func_traincontrols`
either** — but that turns out not to matter, because a `func_tracktrain` is driven from its own deck,
not from a control panel. See `FUN_TRAIN` below.

After the first ride the subtransit trench is lethal on a ~25 s cycle and the mesh says nothing about
it. Modelling that needs time-varying area costs, which the nav format has no room for. This is the
same decision as the crossfire nuke: the bot does not reason about surviving its own gimmick, and a
bot that mistimes a catch and is crushed is an accepted outcome.

navgen ignores `func_tracktrain` entirely, so the parked tram is a solid brush the mesh does not know
about. It matters little on subtransit — the tram's underside sits 48 units above the trench floor, a
duck-height slot, and its deck is above the walkable ledge rather than in it. **If it is ever fixed,
the blocker must not go in `crossbot_nav::IsBakedBlocker`**: `TraceHitEntity`
([bot_move.cpp:128](../src/bot/bot_move.cpp:128)) reads that predicate as "permanent world geometry",
which stops being true the moment the tram is driven. It would need a navgen-only case resolving the
runtime origin from `target` → `path_track` origin rather than the editor `origin` key that
`ExtractBlockingVolumes` uses — on subtransit those differ by `(+64,+704,+496)`.

### FUN_TRAIN: the subtransit tram

**The tram does not start on its own, and it is not a crusher patrolling a route.** `startspeed` is
`0`, so `CFuncTrackTrain::Find` ends with `pev->speed = 0` and `Next()` returns on its first tick
without rescheduling its think. Nothing in the map targets `train`, and the `speed 0` keys on
`trainpath05`, `05.5` and `20.1` are inert — `Next()` skips zero speeds explicitly, and that branch is
gated on `SF_TRACKTRAIN_NOCONTROL` anyway. The 43-node loop is authored but never runs until a player
drives it. Once driven it circles forever: the track is closed, so it never reaches a dead end.

**Taking the controls is not the `PlayerUse` search loop.** `PlayerUse` handles trains *before* the
`FindEntityInSphere` sweep: if `m_afButtonPressed & IN_USE` and the player's `pev->groundentity` has
`FCAP_DIRECTIONAL_USE`, `FL_ONGROUND` is set, `IN_JUMP` is not held, and `OnControls(pev)` passes, the
player gets `PFLAG_ONTRAIN`. None of the `FUN_TANK` aim work applies — there is no cone, no 64-unit
radius, no `UTIL_ClampVectorToBox`. Stand on it, look anywhere, press `+USE`.

**`OnControls` is not "standing on it" — it is a box test on the origin.** `Spawn` caches
`m_controlMins = pev->mins` and `m_controlMaxs = pev->maxs` with `m_controlMaxs.z += 72`, and the test
is on the player's *origin* transformed into tram-local space. A bot resting on the deck **edge** has
the tram as its ground entity while its origin is up to half a hull outside that box: measured on
subtransit, `local.y = 80` against `y[-65,65]`, and `+USE` silently did nothing for the full timeout.
So the mount phase walks to the deck centre first and only pulses `+USE` once inside
`kTrainCentreRadius`. This is the single most confusing failure in the module, because every visible
precondition looks satisfied.

**Firing blocks `+USE`.** `PlayerUse` is reachable only through `ImpulseCommands()` ←
`ItemPostFrame()`, which returns early while `gpGlobals->time < m_flNextAttack`. A bot shooting on the
deck can never take the controls. `BotFun::Pressing` therefore reports true during `TRAIN_MOUNT` and
`TRAIN_RELEASE` so `BotFire` idles, and false during `TRAIN_DRIVE` and `TRAIN_RIDE` — acceleration
goes through `PreThink`, not `PlayerUse`, so the bot can fight while driving and while riding.

**Acceleration is four edge-triggered presses.** `PreThink` fires `Use(USE_SET, ±1)` on
`m_afButtonPressed & IN_FORWARD`, and the delta math steps `pev->speed` 0 → 100 → 200 → 300 → 400.
The `kFunPulse` alternation already used for `IN_USE` supplies the edges. Control drops the moment the
bot leaves the ground, strafes (`IN_MOVELEFT|IN_MOVERIGHT`), or presses `+USE` again, and
`pev->velocity` is zeroed every `PreThink` while it is held — so `TRAIN_DRIVE` and `TRAIN_RELEASE`
send zero `fmove`/`smove` and no jump.

**No new gamedata offsets.** Where `FUN_TANK` needed nine, the tram needs none: `FL_ONTRAIN` on
`pev->flags` says whether the bot holds the controls, `pev->groundentity` says whether it is aboard,
`pev->speed` is the signed current speed, `Spawn` caches the top speed in `pev->impulse`,
`pev->spawnflags` carries `SF_TRACKTRAIN_NOCONTROL`, and the track is the `path_track` chain walked
through plain `target`/`targetname`/`origin` keys.

### Board points come from the track, not from the tram

There is no attempt to predict a rendezvous. `BuildBoardPoints` walks the cached `path_track`
polyline once per map and, at each node, probes both sides for nav areas the bot could hop from. Three
rules decide, and each earned its place:

- The probe offsets start at **`halfWidth + kTrainSideClear`**, which is the tram's own half-width
  plus the bot's hull half-width plus a margin. Offsets measured from the deck centre put the board
  point inside the tram's footprint, and bots walked into the parked brush until `OnBlocked` retired
  the goal. Clearing only the tram's half-width is still not enough: the bot's hull is 32 wide, so at
  exactly `halfWidth` its near edge is flush with the car's side and a passing tram clips a bot that
  is standing still waiting for it.
- The stand height is **traced, never taken from the nav area**. `GroundBelow` fires a downward duck
  hull from the area point and reports the floor; `standZ` is that plus 36. An area's `centroid.z` is
  nowhere near its floor — on subtransit the centroid beside the track reads -128 where the floor is
  -192 — so every height comparison built on it is wrong by tens of units. This is the single most
  expensive mistake available here, because the numbers look plausible and invert the geometry: it
  makes the deck appear to be *below* the walkway when it is 25 units above it.
- The ground probe traces **straight down from the area point**, never from an offset above it. The
  area point is inside a nav area and therefore in open space; starting the trace 96 units higher
  puts it inside the tunnel ceiling, `fStartSolid` fires, and the whole board point is silently
  dropped. That symptom is a tram nobody ever boards, with no log line to say why.
- `deck.z - floorZ <= kTrainStepUp` and `floorZ - deck.z <= kTrainBoardRise` keep the point roughly
  level with the deck, both measured against the traced floor.

Note `point` stays the nav-space goal point from `ClampToArea` and is **not** overwritten with the
traced floor. It feeds `follow.done` and the air-route trace; moving it to the real floor drops the
trace to deck level, where the trench lip blocks it, and every board point beside the parked tram
disappears.

The deck itself is `DeckAt`, not `pev->origin`: the brush model is offset from its origin (on
subtransit by 64 units along the car) and the tram's own forward is the **reverse** of the travel
direction, because `Find` adds 180° to the angles it derives from the look-ahead.

### Start it, or catch it

The mode is chosen from the live `train->v.speed` on arrival, never from what was decided at
selection, so a bot that set out for a parked tram and finds it moving simply switches to catching.

**Stopped** → walk at the deck and hop on (`kTrainJumpGap` from the bbox, `IN_JUMP` only when below
deck height), then mount, accelerate to `pev->impulse`, release with one more `+USE`, and ride.

**Moving** → the bot is already trackside, so it stands still and computes
`ArcForward(ProjectOnTrack(tram), boardArc, sign(speed)) / |speed|`. If the tram is not due within
`kTrainCatchWindow` (3 s), the goal is banned by `abandonReadiness` alone — no `readyTime` — so the
bot goes to resupply and the tram becomes interesting again once it is better armed. If it is due, the
bot waits and hops.

The hop aims at the **cached trackside deck point**, not at the tram's current centre: driving at a
moving tram means running into its front face, and `Blocked` crushes for `dmg 1000` unless the victim
is already `FL_ONGROUND` **with the train as its ground entity**.

### The hop is a timed jump, and the deck is above the walkway

On subtransit the ledge floor beside the track is at -192 and the deck top at -167, so **boarding is
a 25-unit rise, not a drop**. That is worth stating because the opposite reading is easy to reach and
inverts the whole manoeuvre — see the `centroid.z` trap above.

`TryTrainHop` works entirely in time. From the traced geometry it computes the interval during which
a jump would leave the bot **at or above deck height**:

```
rise      = (boardDeck.z + 36) - origin.z
clearFrom = (kJumpImpulse - sqrt(kJumpImpulse^2 - 2*g*rise)) / g      -- clears the deck going up
clearTo   = (kJumpImpulse + sqrt(kJumpImpulse^2 - 2*g*rise)) / g      -- drops back below it
```

and jumps only when the tram's own `eta` falls inside `[clearFrom, clearTo]`. A rise beyond
`PmSim::PlainJumpHeight` is rejected outright. `eta` is used unmodified: `b.deck` is by construction
the deck's position when the tram's origin reaches `b.arc`, so `TrainEta` already *is* the deck
arrival time. Subtracting a deck-lead term on top of that double-counts and pushes the estimate below
the firing window, at which point the hop never triggers at all.

Two guards make the difference between boarding and being crushed:

- **The brink.** The bot never crosses `halfWidth + kTrainBrink` on foot. Beyond it the bot runs; at
  it the bot either jumps, or stands still and waits for the next pass. Air control is weak, so the
  run-up must happen outside the car's sweep and the bot must already be moving when it leaves the
  ground.
- **No proximity trigger.** An earlier version also fired on raw distance to the tram. That ignores
  geometry entirely and commits hopeless jumps, putting the bot airborne in the car's path — the
  worst possible state, since being off the ground is exactly what turns the nudge branch of
  `Blocked` into the `dmg 1000` branch.

`PmSim` is deliberately **not** used for this. It is perfectly usable at runtime — `BotOppo::TryJump`
([bot_opportunist.cpp:64](../src/bot/bot_opportunist.cpp:64)) predicts landings with `BeginAirborne`
and `Step` — but every trace it makes is a snapshot of the world *now*, and nothing in the repo
extrapolates geometry in time. Asked where the bot lands, it would answer "in the trench", which is
true at the moment of the query and false at the moment of landing. The tram is on rails at constant
speed, so its future position is closed-form and needs no simulation.

`RunArrival` only hands over the first frame. Everything after that runs from `BotFun::Run`, which is
called unconditionally — `RunArrival` fires only while `follow.done`, and the bot stops being at its
goal point the moment it steps toward the deck, so an arrival-driven state machine stalls silently
partway through boarding.

Riding is a normal player standing on a moving platform: zero move input, aim and fire from the usual
pipeline, and a nudge back toward the deck centre when drift exceeds `kTrainCentreRadius` (without it
bots slide off on the curves). Boredom is the tank's rule unchanged. Dismount looks each frame for a
nav area within `kTrainHopRange` of either side and no more than `kTrainSafeDrop` below, then jumps
onto it; `kTrainRideMax` forces a bail if nothing qualifies for a full minute.

