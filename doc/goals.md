# Goals: what the bot wants

What the bot wants and how it picks it: the three goal tiers, detours on the way, camp spots, and the
objective readers. Fighting interrupts all of this (`doc/combat.md`); walking to the chosen point is
`doc/movement.md`; the route itself is `doc/path_planning.md`.

## bot_goal.cpp

Owns *what the bot wants*, in three tiers, highest first:

| tier | candidates come from | pick within the tier |
| --- | --- | --- |
| `GOAL_STRATEGIC` | `bot_strategic.cpp`: objectives and `bot_goal` entities | weighted random |
| `GOAL_FUN` | `bot_fun.cpp`: map gimmicks | weighted random |
| `GOAL_GATHER` | `bot_items.cpp`: pickups, chargers, healing volumes | cheapest |

`Select` asks each tier in turn and the first tier with a surviving candidate wins outright: a bot with
a tank to mount does not weigh it against a shotgun. `bot_goal.cpp` knows nothing about what the
candidates are. Two exceptions to tier order:

- **A needed gather item goes first.** One whose worth beats its cost outranks every tier (see "Worth:
  need pulls"). The gather scan runs once per `Select`; the promoted, needed and gather lists are all cut
  from that one scan.
- **`roam` sits between fun and gather.** When the strategic tier's only survivors are `roam` (an
  objective map with nothing to do, such as every fy_ map), `Select` holds them back, tries fun with its
  promoted pickups, and roams only if that is empty; gather comes after roam. Without this a bot on
  fy_snow never gathered after its first pick and never mounted anything.

`Select` runs only when `Bot_MoveNav` finds `hasGoal == false` and the bot is not in combat, so no tier
preempts a goal in flight: a higher tier appearing mid-route waits for the arrival. Strategic goals have
their own preemption, `Outranked` (see `objectives.cpp`).

**Gather picks the cheapest; fun and strategic roll, weighted toward closer.** The weight is
`1 / (cost + 2)^kBias`, `kBias` 2. Nearest-first suits items, because a battery is worth taking only
when it is on the way. It is wrong for gimmicks: four bots that all pick the nearest tank make three
bots queueing behind one. The `+ 2` (`kGoalBiasFloor`) keeps a goal under the bot's feet from taking all
the weight.

Every game's bots pursue every tier. Camping needs a camp route, which `Select` builds only for tiers
above gather.

### Selection is two searches, not one

1. `Nav::AreaCosts`, a plain Dijkstra over the area graph, prices every area from the bot's own. It runs
   once per selection and is handed to every tier, so adding tiers adds no searches.
2. `Repath` runs the real A*, with air-move simulation, to the chosen point only.

**Why not one goal-less A*:** `PlanRoute` carries a full `PlanWalker`, waypoint vector included, on
every node it touches, because funnel state is what decides whether an air move is feasible. A
goal-directed plan on crossfire expands ~165 areas; a goal-less one expands all ~500 and runs a `PmSim`
per air-move edge on each. That is a third of a second every time a bot picks something up.

**The cost of splitting:** phase 1 can name a point phase 2 cannot reach, for example behind an air move
that needs a run-up the bot cannot get. The owning tier's `OnRepathFailed` absorbs it (for an item, the
unreachable ignore). Without that the two phases disagree forever and the bot re-picks the same
unroutable item every frame.

The ignore is only for *route-dependent* disagreements. `AreaCosts` applies the part of activation
gating that needs no walker (`GateAllowsCrossing`, `doc/path_planning.md`), so an area behind a door the
bot can never open from this side prices `kUnreachableSeconds` and `crossbot_gather_debug` prints
`no route`. A static failure that reached the ignore table would expire into the same failure forever.
On crossfire the 12-item secret cache in area 8 should be the only `no route` items.

**The cost is `travelSeconds + costSeconds`, never `costSeconds` alone.** `Connection::costSeconds` is a
surcharge (door waits, flight time, ride time) and is zero on a plain walk (`doc/path_planning.md`,
travelSeconds). Pricing by it alone makes most of the map cost 0, "nearest" degenerates into roughly
area-index order, and the bot walks past the battery at its feet to fetch something across the map. Far
items reading `0.0s away` in `crossbot_gather_debug` is this bug.

**Ties break on distance.** A candidate costs `AreaCosts[area] + |point − reference| / kNavRunSpeed`,
where the reference is the bot's own origin in its current area and the area centroid elsewhere. By
area alone every candidate in the winning area ties, and the tie in the bot's own area is exactly the
"it is right next to me" case.

### Carefulness

Combat readiness shapes *how* the bot travels as well as where. `Bot_Carefulness` returns
`(1 - readiness)` scaled by `crossbot_careful`, and both searches take it: `Repath` passes it to
`Nav::PlanPath`, `Select` to `Nav::AreaCosts`. An armed, healthy bot routes for speed; a bot with nothing
to fight with routes for cover however far round that is (`doc/path_planning.md`, "Carefulness: routing
away from exposure").

- **Sampled once per repath and baked into the route.** A bot that arms itself halfway along a timid
  route finishes that route.
- **`BotOppo::RebuildCosts` must stay at 0.** Its gain test subtracts real simulated seconds, which the
  exposure weighting would distort.
- **Live readiness stays mid-range.** A fresh spawn with crowbar and glock at full health reads about
  0.35, so carefulness rarely reaches either end.
- **`crossbot_careful 0`** restores pure fastest-route planning.
- The `Repath` log line prints `careful=`, `route=` (true seconds) and `score=` (exposure-weighted).
  At `careful 0` they are equal by construction, which is how to confirm the factor reached the planner.

`crossbot_gather_debug` prints readiness (`EvaluateCombat`) in its scan header for this reason.

### Range bias

The mirror of carefulness. Every weapon is `RANGE_CLOSE` or `RANGE_LONG` in its `WeaponSpec`
(`doc/games.md`), and `Bot_RangeBias` turns the held weapon's class into a `NavPlan::RangeBias` weighted
`readiness * kRangePref` (0.5). The two factors multiply, so neither is lost: a hurt, badly armed bot
ignores range and avoids exposed ground, while a healthy, armed one shapes its route to its weapon. It
goes to the same two searches, and `BotOppo::RebuildCosts` leaves it at zero for the same reason.

A route is weighed against each connection's mean open sight distance in the direction it is crossed
(`doc/path_planning.md`, "Range bias: routing to ground the weapon can use"). A close weapon is happy
below `RangeBias::shortDist` (512), a long one wants more than `RangeBias::longDist` (1024), and the
penalty ramps in between. Both live in `plan_search.h`, but the right values are per map: crossfire's
per-connection open distance has a mean of 530 and a max of 1584, so measure with
`crossbot_vantage_status` before trusting the defaults.

**A weapon change invalidates the route; a readiness change does not.** The nav update forces a repath
when the held weapon's class differs from `bot.plannedRangeClass`. Comparing the class, not the weapon
id, makes swapping between two close-range guns free. The stamp is written only once a plan stands, so
a failed search keeps the retry trigger.

### The distant target is sticky; only the leg is re-decided

`state.target` is the goal proper and survives repaths. `state.stop` is where the current leg ends and is
what `bot.goalPoint` holds. `Repath` reads `goalPoint` and nothing else, so changing legs is
`goalPoint = …; Bot_ResetNavPath(bot)`.

There is no itinerary. `PickStop` runs at selection and at each leg's end and answers one question:
where does *this* leg stop?

```
here = toGoal[botArea]
accept candidate c when   toGoal[c] < here                      strictly closer in
                    and   fromBot[c] + toGoal[c] - here <= kDetourBudget + worth[c]
minimise fromBot[c]
```

- `kDetourBudget` is 6 s; `worth` is "Worth: need pulls".
- `toGoal` is `Nav::AreaCostsToGoal`, a reverse Dijkstra that depends only on the target. It is computed
  once per retarget and cached in `state.toTargetCosts` behind a `toTargetArea` guard, the same shape as
  `bot.oppoCosts` / `oppoCostsGoalArea`. A repath therefore costs one `AreaCosts`, which selection
  already pays.
- `toGoal[c] < here` makes the walk monotone: every leg ends strictly closer to the target, so detours
  cannot chain forever. If they appear to, the reverse field is being built against the wrong area.

A healthy transcript is one `wants` line, then `detours to` lines whose "left after" figure falls every
time, then the arrival:

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
re-selected instead of held, and the bot oscillates between two goals without reaching either.

### TargetValid must not read the bot's arrival state

`CommitStop` calls `BotItems::ForgetGoalItem` and `BotFun::Forget` on **every** leg, because the next
stop may belong to a different tier. So while a bot walks to a gather detour on the way to a tank,
`bot.fun` holds the stop, not the target.

`TargetValid` therefore asks `BotFun::ValidFor(bot, ent, payload)` against the target's own entity, never
`BotFun::Valid(bot)`, which reads `bot.fun.goal`. Reading per-bot state invalidated the target at the
first detour, and the bot re-selected from scratch at every item; it looked like weighted-random churn.

### Failure is per-leg

- **A detour fails:** `OnRepathFailed` logs `could not plan a route to the detour X, going straight on`,
  adds the entity to `state.skipped`, and re-runs `PickStop` with the target intact.
- **The target fails:** the owning tier's `OnRepathFailed` runs and the goal dies.
- **Combat starts:** `BotCombat` clears the goal outright (`BotGoal::Clear`), target and cached reverse
  field included, so on a small map with several bots fun goals are re-selected often. The exception is
  a tank between the bot and its enemy (see "Taking a tank mid-fight").

### Revenge

A bot killed by an enemy it was tracking goes back, next life, to where that enemy was — approached so
the killer could not have seen it coming (`doc/path_planning.md`, "Revenge: routing out of a watcher's
sight"). `crossbot_revenge 1` by default.

The killer is latched on the death frame, not read later: `BotGoal::NoteDeath` runs in the `!wasDead`
block of `ThinkOneBot` (`doc/combat.md`, revenge latch), where `v.dmg_inflictor` is still valid. It
qualifies only when `BotCombat::Attacker(bot)` equals the tracked `BotCombat::Enemy` (or `sense.enemy`),
and stores three vectors on `CBot::revenge`: the enemy's last-seen origin (`sense.enemyPos`, the goal), the
killer's eye, and its forward (its view angles at the kill — the FOV the approach must stay out of). A
death that does not qualify clears the latch, so dying again cancels or replaces the revenge. The struct
survives respawn like the hurt log; `BotGoal::OnRespawn` promotes a pending latch to the live goal for the
new life. There is no time limit — the approach can legitimately take longer than any fixed deadline — so
the goal lives until it is reached, dropped, or replaced (below).

The destination is not always the enemy spot itself. The first time a life needs it, `ChooseRevenge` rolls
once over the spot plus every camp spot that could shoot it: reachable for the bot (not its current area),
`BotCamp::SpotAllowed`, its eye (`BotCamp::SpotEye`) within `crossbot_revenge_los_radius` of the killer's
eye and **outside the killer's FOV cone** (`crossbot_revenge_fov`), and with a clear `TRACE_LINE` from that
eye to the spot. Sightlines are symmetric, so a camp that sees the spot is in the killer's line of sight;
only the cone can make it unseen, which is why the cone is the test. Cheap filters run first and only
survivors are traced, once per life. Each candidate weighs `1 / (fromBot + 2)^Bias` — the same travel-time
weighting `PickWeighted` uses — so near shooting positions win most rolls. No candidate drops the revenge.
A picked camp is committed through `BotCamp::CommitSnipe` (weapon 0, so no snipe-weapon checks): the bot
holds there 4–10 s aimed at the enemy spot, then resumes; the spot itself stays pass-through.

Two modes, decided in `Select` by the tier that would otherwise win:

- **Primary** when the bot would only gather or roam (no objective to protect): the revenge destination replaces
  the target outright, ungated. Skipped while a *needed* gather item won — a bot out of ammo resupplies
  first rather than charging in unarmed.
- **Detour** when a fun or strategic goal won: the revenge point is injected in `PickStop` as a top-priority
  detour, but only through the camp along-the-way gate (`out + in - here <= crossbot_revenge_budget`, 10 s,
  the same test `BotCamp::BuildRoute` uses) and only while the target is not urgent
  (`!BotStrategic::Urgent`). Off the path by more than the budget, revenge waits; it may come onto the route
  later. **An urgent objective wins outright:** when a bomb is planted (or any rank ≥ `kRankDeliver` goal
  appears) the normal strategic preemption re-targets the bot to it, and the `Urgent` gate stops revenge
  re-injecting as a detour, so the bot defuses instead of swinging by the kill site.

Either way the stop carries `Stop::revenge`, which is what makes `Repath` plan the sneaky route. The goal
clears when the revenge stop ends — `Advance` clears it whether the bot arrived, finished the camp hold or
abandoned the camp, and when the revenge was the target itself it clears the goal too instead of
re-committing it — on a new death, or when **no unseen route exists**: with the hard sightline
exclusion a failed search means exactly that, so `OnRepathFailed` drops the revenge and the bot reverts to
its normal goal (`crossbot_revenge_debug` logs each of these).

## bot_items.cpp

The gather tier: the lowest of the three, and the only one a stock HLDM map always populates. `Collect`
is the entry point (`ScanItems` plus the `SCAN_OK` filter); the picking is `bot_goal.cpp`'s. This file
owns what is item-specific: the ignore table, the respawn memo, `UpdateGoalItem`, and the charger and
volume arrival behaviours.

- **Only items are candidates.** Fighting is a state, not a goal: `BotCombat` takes movement away from
  the planner for as long as it lasts (`doc/combat.md`, `bot_combat.h`). Enemies were once candidates
  priced by the same Dijkstra, which planned a full A* at anyone seen across the map.
- **`crossbot_gather 0`** turns the tier off, and the bot stands still until commanded. The nav sweep
  sets it (`doc/testing.md`, `scripts/test_nav_goal.ps1`); `scripts/test_items.ps1` exercises gathering.

### One ignore table, not two

- **`itemMemo`, per entity:** *do not consider this entity before T*. "Respawning until T" and
  "unreachable, retry after T" are that same fact, so one table serves both.
- **`fullMemo`, per classname:** being full of 9mm makes every 9mm box pointless, not just the one
  touched.

Both are per bot. `fullMemo` must be, since it describes the bot's inventory. `itemMemo` is per bot by
choice: a bot only knows about pickups it made or saw, so two bots can walk at the same battery. That is
the humanlike answer and matches the per-bot `activationMemo`. On respawn `fullMemo` clears and
`itemMemo` stays: the inventory is empty, but the taken items are still on their timers.

The durations are the profile's:

| memo | source | HLDM |
| --- | --- | --- |
| respawning | `IGameProfile::ItemRespawnIn` (default `ItemRespawnDelay`) | 20 s weapons and ammo, 30 s items and suit chargers, 60 s health chargers |
| unreachable or untouchable | `ItemUnreachableTimeout` | 30 s |
| full | `ItemFullTimeout` | 120 s |

TFC's `ItemRespawnIn` returns a pack's own `wait`, 1 to 60 s.

A pickup the bot **collects itself** whose respawn is under `kRecollectCooldown` (15 s) is held as an
*ignored* entry for that floor instead of a *respawning* one, so the re-selection cadence never drops
below 15 s. Re-selection otherwise has no per-item cooldown, so a `wait 1` item plus a cluster of them
(TFC's dustbowl resupply bags, with the needed override pulling the bot back to each in turn) round-robin
forever: the bot tops up and never leaves. The floor bites only on respawns shorter than itself; every
HLDM/CS/DoD pickup (20 s or more) keeps its plain `respawning` entry.

### Walking in on something that came back

`RefreshMemos` runs every frame and drops a *respawning* entry whose item is in the bot's current area
and available again, logging `finds X back already, forgetting it was taken`. The respawn clock starts
when the bot **noticed** the item gone, not when it was taken, so a bot that saw it gone from across the
room may be counting a full 20 or 30 s after the real respawn.

- **Only `respawning` entries qualify.** Entries from outcome D, `OnRepathFailed` and the recollect floor
  above share the table and must survive: standing in the area is what produced them, and clearing them on
  arrival restarts the spin they exist to break. A self-collected item has an accurate respawn clock (the
  bot took it where it stands), so the forgiveness this offers a stale, seen-from-afar clock is not needed.
- **No geometry.** The memo stores the item's `kind` and `areaId` when written, so the check is a few
  integer compares. Resolving each entity to an area would be a scan of every area per entry.
- **Weapons never need it.** `CBasePlayerItem::Respawn` creates a new edict, so the memo's handle goes
  stale, `CBotEntHandle::Get` returns null and the entry is pruned. `item_*` and `ammo_*` reuse their
  edict and are what this serves.

### The four outcomes

A gather goal ends in one of four ways, and three of them are not "the follower arrived".

- **A — touched it.** The `pfnTouch` post hook. The game's touch handler has already run, so the
  entity's state is the verdict: `SOLID_NOT`, `EF_NODRAW`, `FL_KILLME` or freed means picked up;
  anything else means the bot could not use it, and the classname goes in `fullMemo`. `FL_KILLME` is
  easy to miss: `UTIL_Remove` only flags the edict and the engine frees it at the end of the frame.
- **B — arrived and it was already gone.** No touch ever fires on a `SOLID_NOT` item, so arrival
  (`currentAreaId == goalItemArea`) ends the goal.
- **C — saw from a distance that it was gone.** At 4 Hz, `BotSight::Clarity` from the entity to the
  bot's eye, skipping the entity, comes back clear. It must use `dont_ignore_monsters`: otherwise the
  trace passes straight through the player, reads as no line of sight, and the bot walks the whole way
  for nothing.
- **D — arrived, touched nothing, item still there.** The item is above the bot or on a lip. Unhandled,
  the goal clears, selection re-picks the same item, and the bot "arrives" every frame with no stall
  detector to notice, because it is exactly where it wanted to be. The item goes into the ignore table
  (log: `reached X without touching it`). `weaponbox`es dropped mid-air where players died hit this
  within seconds on `test56_recharger`.

### Resolving an item to an area

- **`Nav::FindAreaExact`, never `Nav::FindAreaNearest`.** Nearest falls back to nearest centroid and
  essentially never returns null, so an item inside a wall snaps to some nearby area and the bot routes
  confidently to nothing. Healing volumes resolve differently (next section).
- **Probe at `origin + 18`, then the raw origin** (`kItemAreaLift`). Area z-space is duck-origin based
  (navgen builds in hull 3) and an item's origin is at its base.
- **The area must be at the item's height.** `FindAreaExact` tests only the xy half-spaces, so an item on
  a shelf resolves to the floor area below it. An area is rejected unless the point clamped into it is
  within 72 units of the item in z (one jump, `kItemReachHeight`). Without that, four TFC demomen on
  crossover2 cycled for five minutes between six pickups 110 units over their heads.
- **The goal point is the item's origin clamped into that area.** Only the point reaches the planner, and
  `Repath` re-resolves it with `FindAreaNearest`. A raw base origin sits 18 units below its area, so the
  exact test misses and the fallback takes whichever box it falls in: for a low ledge, the floor beside
  it. On fy_snow the CS bots walked into the 32-unit lip of the HE-grenade ledges forever instead of
  taking the jump the mesh has.
- **Arriving is not touching.** The follower can finish a leg tens of units short of a small entity (60-120
  on casbah). An untouched pickup gets up to 1.5 s (`kPickupApproachTime`) of walking straight at its
  centre before the bot gives up, which is how backpacks off the nav area get picked up.
- **Chargers have no origin.** They are brush entities: `v.origin` is `(0,0,0)` and the centre,
  `(absmin + absmax) * 0.5`, is inside the wall. The use spot is the closest point within 48 units of the
  centre (`kChargerUseRadius`) on any area whose bounds come within 96 (`kChargerSpotSearch`). 48, not
  HL's 64-unit USE radius, because `NavFollow` stops within `goalRadius` (12) of the goal horizontally and
  may stop on the far side; at 64 + 12 the bot presses USE at nothing until the 15 s timeout.

### Healing volumes give one entity several candidates

A `trigger_hurt` with negative `dmg` heals whoever stands in it; `pool_party` has one at `dmg -40`,
40 hp/s. A charger is a fixture the bot walks *up to*, so one area beside it is the answer. A pool is a
brush the bot stands *inside*, so every overlapping area is a way in: `ResolveVolumeAreas` emits a
`Placement`, and so a `Candidate`, per entrance, all sharing the entity, and `ItemCost`'s field picks the
cheapest way in. Capped at the 8 nearest the bot (`kHealAreaCandidates`) so a map-sized trigger cannot
flood the scan.

**"Inside" is the engine's own test, not the bounding box.** A brush entity's `absmin`/`absmax` is one
box around all its brushes: `pool_party`'s heal volume is 674 x 733 x 80 units of box around basins that
do not fill it. And `SV_TouchLinks` point-tests the *toucher's origin* against the trigger's clip hull
before calling `pfnTouch`. `OriginInsideVolume` runs that predicate through `pfnTraceModel`, the same test
`Nav::PushAt` makes on `trigger_push` clip hulls. A box test failed quietly: 5 of 9 arrivals stood near the pool,
gained nothing, and blacklisted every healing trigger on the map for 120 s.

**The probe is where the bot's origin will be**, built in two steps:

1. **Drop to the floor** with `Nav::AreaFloorZ`, the highest floor plane at that x/y. An area is a tall
   polytope bounded below by its sloped floor planes, and its centroid is not on the floor. de_dust2's A
   ramp (area 52, floor about z 18 at its east edge, top at z 302) has a centroid z of 124, which floated
   the probe into the bomb zone (z 96 to 192): Ts with the C4 "arrived" 90 units under the zone and held
   30 s without planting, every round.
   A prism area (a merged ramp or stairs) exports only its footprint walls and a z-slab, so its one floor
   plane is the slab bottom. A duck-hull trace down the column, from the area top capped at the volume top,
   gives the real floor and wins when it is higher. dod_charlie's midnest (`*11`) is reachable from the beach
   only through the ramp prisms 931 and 933 in front of the sandbags. Their slab floor of z 82 put the probe
   130 units under the volume (z 208 to 336), so only areas behind the sandbags counted, and the Allies
   priced midnest at 32 to 35 s from the beach via the far west flank instead of about 6 s up the ramp. The
   trace puts them at z 239 and 244. It costs one trace per area overlapping a volume, per goal pick.
2. **Lift 18 to a standing origin and test hull 1**, unless the area is `NAV_AREA_CROUCH` (then hull 3).
   Spots at area z −768 and −780 under a pool topped at −776 had the duck origin inside and the standing
   origin the bot actually arrived with above it.

### RunHealVolume is RunChargerUse minus the aiming

No `IN_USE` and nothing to face: presence is the interaction. It reuses the charger fields on `CBot`
(`chargerActive`, `chargerStartTime`, `chargerLastGainTime`, `chargerLastValue`), with `chargerPressed`
meaning "has been inside", so `ManeuverActive` and the `lastProgressTime` stamp work unchanged.

| | charger | healing volume | why it differs |
| --- | --- | --- | --- |
| no-gain grace | 0.5 s | 1.2 s (`kHealNoGainGrace`) | `HurtTouch` heals every 0.5 s (`pev->dmgtime`), so a working pool looks stalled for half a second at a time |
| timeout | 15 s | 5 s to get inside (`kHealApproachTimeout`), 30 s overall (`kHealTimeout`) | a `dmg -5` pool legitimately takes twenty seconds to fill a bot |

- **The view is not hijacked.** The runner calls `BotSense::ApplyLook` before `Bot_UpdateFacing`, as the
  idle branch does, so a bot in a pool keeps scanning the room and can track an enemy while it heals.
- **Drifting out is not a useless pool.** Losing footing ends the goal with the unreachable timeout, not
  the 120 s classname ban.
- **It drives at the resolved spot**, not the volume's centre, which in a large pool can be inside
  geometry.

### Sated: asking before travelling, not on arrival

Costs are travel time only, so without this check a bot at full health walks to a charger, gains nothing,
and blacklists the classname for 120 s; for a pool that ban covers every healing trigger on the map.
`BotItems::Sated` compares `ChargerValue` with `ChargerFullValue` during the scan and marks the candidate
`SCAN_SATED`, printed `no need`. It applies to chargers and pools alike.

Need weighting beyond that exists only for TFC packs ("Worth: need pulls"). Every other item competes on
seconds of travel alone.

### Every item is judged by the game that owns it

`ScanItems` walks every edict and classifies, tests (`ItemUsable`), prices (`ItemWorth`) and sated-checks
each one inside `XPlay::ScopedGame(JudgeGame(ent))`: the owner game's profile and offsets, or the bot's
own when the owner has no profile. `Classify`, `ChargerValue`, `ChargerFull` and the respawn delay go
through the same switch, so a valve bot sees an `info_tfgoal` as TFC sees it and a TFC bot sees a
`func_healthcharger` as valve sees it. The bot's own fields are read from its object in that game; for a
foreign player that is the copy CrossPlay keeps (CrossPlay's `FixProxyTeam` fills `team_no` there).
`XPlay::EnterGame` returns early for the game that is already current, since the scan enters one per edict.

What actually crosses games is not modelled. CrossPlay's Touch filter blocks some foreign touches, and
the pickups AMXX plugin (`goldsrc.one/scripting/pickups.sma`) translates foreign weapons, items, ammo,
and TFC pack ammo and grenades into the toucher's game. The bot simply touches the item and `OnTouch`
learns the result: vanished means picked up, stayed means `NoteFull`. On casbah, four valve bots for
four minutes: 24 detours to TFC packs, 19 pickups, zero `full of` lines.

### Worth: need pulls

`IGameProfile::ItemWorth(player, item, ammoNeed)` is how many seconds of extra travel the item is worth
to this player now. It is 0 for everything except TFC packs (`doc/games.md`), and `bot_goal.cpp` uses it
twice:

- **Detour:** the per-candidate budget is `kDetourBudget + worth`. The monotone `toGoal[c] < here` rule
  still holds.
- **Override:** in `Select`, a gather candidate whose worth beats its `CandCost` becomes the target ahead
  of the strategic tier, cheapest first, logged as `wants gather ... (needed)`. It is skipped while the
  bot is an `Objectives` carrier. It cannot loop: after the pickup need has dropped and the item is
  memoed as respawning.

`ammoNeed` is computed once per scan in the bot's own game: the fraction of its non-melee weapons that
`BotFire::WeaponArmed` calls dry. It is coarse, since a weapon with two rounds left counts as armed,
because no game exposes per-weapon maximum ammo generically. Refine it when a profile needs more than
"out".

### Reading crossbot_gather_debug

`crossbot_gather_debug 1` makes every selection print the whole scan: one line per item entity with its
resolved area and either its cost in seconds or why it was skipped:

| reason | meaning |
| --- | --- |
| `taken` / `empty` / `switched off` | gone: a pickup, a charger, a healing volume |
| `respawning for Ns`, `ignored for Ns` | `itemMemo` |
| `full for Ns` | `fullMemo` |
| `no need` | sated |
| `no nav area` | resolved to no area |
| `no route` | `AreaCosts` priced it unreachable |

The winner is marked `PICKED`. A healing volume prints one line per entrance while selectable, and once
when skipped, since a skipped one says the same thing from every area. This is the only view that tells
apart the ways an item can be invisible to a bot, and the costs prove phase 1 is sane: far items at
`0.0s away` mean the cost model is broken, not the mesh.

It is deliberately verbose, ~130 lines per selection on crossfire at up to 2 Hz per idle bot. Leave it
off unless reading the log.

## bot_fun.cpp

The middle tier: the map gimmicks that make an HLDM map what it is. Three kinds:

| kind | what |
| --- | --- |
| `FUN_SWITCH` | a button or trigger whose chain reaches something that hurts people |
| `FUN_TANK` | a mounted gun |
| `FUN_TRAIN` | a drivable `func_tracktrain` |

`crossbot_fun 0` empties the tier, and `BotGoal::Select` reduces exactly to gather-only.

This is entirely a runtime feature: no nav format change, no `kNavVersion` bump, no navgen work.
Everything a fun goal needs is a plain `entvars_t` field or an offset already in
`gamedata/valve-win.json` (`doc/games.md`, gamedata.cpp).

### Switches outrank everything else in the tier

`BotFun::Prioritize` is a hard filter, not a weight: if any surviving candidate is a `FUN_SWITCH`, every
tank and tram candidate is erased and the roll runs over switches alone. Both switch actions count, so
crossfire's `trigger_multiple` nuke ranks with the buttons.

It runs in `BotGoal::Select`, **after `PruneUnreachable` and before the tier's emptiness test**. That
position is the design:

- Inside `BotFun::Collect` it cannot work: `Collect` never sees the costs, so an unreachable switch
  would suppress every tank, empty the tier, and send the bot to gather past a working tank.
- After the emptiness test the tier has already been accepted.

The memo gate does the rest. A switch on cooldown emits no candidate, so crossfire's tanks come back
during the nuke's `wait`, and undertow's 142 s button empties the tier rather than blocking it.

**On stock maps the filter never bites.** Only crossfire and doublecross carry both a switch and tanks,
and on both the switch is unroutable: crossfire's nuke has a nav area once `strike_ready_door` opens at
t=180 but still prices unreachable, and doublecross's `fire_button_texture` (area 304) never survives the
prune. `crossbot_fun_debug` logs `fun priority kept N switch candidates, dropped M others` when it does
bite; that line never appearing is expected.

### The chain table

Built once per map into a file static keyed on `gpGlobals->mapname`, cleared by `ResetMap` on both
`ServerActivate` and `ServerDeactivate`. It is map data, not per bot.

From every `func_button`, `func_rot_button`, `trigger_multiple` and `trigger_once`, breadth-first over
`pev->target` → matching `targetname`, depth ≤ 6, cycle-guarded, passing through `trigger_relay`,
`multisource` and `multi_manager` and stopping anywhere else.

- **`multi_manager` targets come from `m_iTargetName[0 .. m_cTargets)` through gamedata, never from the
  map keys.** Worldcraft writes duplicate keys with `#N` suffixes (crossfire's `strike_pain` and
  `strike_pain#1` are the on and off edges of one 0.1 s kill window); only the engine's array is
  stripped and delay-sorted.
- **A `multisource`'s inputs are a reverse edge.** A button naming a multisource arms it; it does not
  fire through it.

A chain is **interesting** if it reaches:

- a `trigger_hurt` with `dmg > 0`,
- a `func_water` (or a `func_door` with `skin < 0`, the same class),
- an `env_explosion` without `SF_ENVEXPLOSION_NODAMAGE`,
- a `func_mortar_field`, or
- a `monstermaker`.

It is **not** interesting if its only terminals are `func_door`, `func_plat`, `func_train` or
`func_breakable`: navgen models those as activation detours, so crossfire's four `lift*` buttons stay
navigation. `trigger_push` is excluded for the same reason; mobility is navgen's business.

**Dangling targets are normal** (crossfire has 8, doublecross 4). A chain that resolves to nothing has no
terminal, which is what drops crossfire's four inert tower buttons (the two-tower arming mechanic was cut
and its names do not exist) and stalkyard's `armorall` gag button, whose only target is an
`ambient_generic`. An `env_explosion` deletes itself after firing unless `SF_ENVEXPLOSION_REPEATABLE`, so
the table tolerates a terminal edict vanishing.

### Activation is three different actions

`CBaseButton::Spawn` decides which, and guessing wrong leaves a bot pressing a key that was never wired:

| condition | how the bot activates it |
| --- | --- |
| `health > 0` | **shoot it**: `takedamage` strips `FCAP_IMPULSE_USE` |
| `spawnflags & 256` (`SF_BUTTON_TOUCH_ONLY`) | **walk into it**: `SetUse` is never installed |
| a trigger brush | **walk into it** |
| otherwise | **`+USE`** |

The shoot path hands off to `BotBreak`. No stock DM map has a shootable button with an interesting
chain, so it ships detected but unexercised; detection still matters, or the bot would press `+USE` at a
button with no `SetUse` until it timed out.

HLDM's most famous goal is not a button: crossfire's nuke is a 16×14×19 `trigger_multiple` at
`(0, -2219, -1846)`, `wait 60`. A detector keyed on `func_button` misses it.

### "Cannot be activated" is a real test, not a guess

A fun goal the bot cannot act on is not offered:

- **`+USE` or shoot switch:** only at `m_toggle_state == TS_AT_BOTTOM`. `TS_GOING_UP`/`TS_GOING_DOWN` is
  mid-move; `TS_AT_TOP` is waiting out `m_flWait`, or latched forever with `wait -1`. This must come from
  gamedata: undertow's flood button is `wait 142` **and** `SF_BUTTON_DONTMOVE`, so no geometric test can
  see that it is waiting.
- **Touch trigger:** `v.solid != SOLID_NOT`, plus a hull-3 trace from the containing area's stand point to
  the clamped point inside the volume. That is what makes crossfire's nuke read `no nav area` before t=180,
  while `strike_ready_door` still seals the bunker.
- **Tank:** `spawnflags & SF_TANK_CANCONTROL (0x20)`, `m_pController == NULL`, and, if `m_iszMaster` is
  set, the named `multisource` read as `m_rgTriggered` against `m_iTotal` (what `IsTriggered` reads;
  `CMultiSource::m_iState` does not exist).

### UTIL_ClampVectorToBox is not a clamp

`PlayerUse` tests

```c
vecLOS = UTIL_ClampVectorToBox( VecBModelOrigin(pev) - eye, pObject->pev->size * 0.5 );
flDot  = DotProduct( vecLOS, gpGlobals->v_forward );   // must exceed VIEW_FIELD_NARROW = 0.7
```

and `UTIL_ClampVectorToBox` (util.cpp:1439) **subtracts** the half-extent per axis, zeroes any axis the
eye is already inside, and returns the result **normalized**. `flDot` is therefore a cosine toward the
nearest *face* of the brush, not its centre.

Aiming at the centre fails on tall brushes and works on small ones, where the two directions nearly
coincide. Beside stalkyard's 12×70×100 controls brush the centre is 54° down, cos 0.58, under the 0.7
gate: the bot pressed `+USE` forever and got `common/wpn_denyselect.wav`, with every gamedata read
correct, while undertow's small button worked first time.

So `RunArrival` faces along `UseDir`, the same subtract-and-normalize, and re-checks the real cosine
against a 0.75 floor (`kUseDotFloor`) before pressing.

Range is the engine's too: `UTIL_FindEntityInSphere` measures **box distance from `pev->origin` to
`absmin`/`absmax`** (rehlds `pr_cmds.cpp:864`), from the feet, radius 64. Centre distance is far too
pessimistic on a long brush. `kFunUseRange` is 56, inside the engine's 64.

### `+USE` is edge-triggered, and one press is easy to lose

`m_afButtonPressed = (m_afButtonLast ^ pev->button) & pev->button` (player.cpp:1849), so holding `IN_USE`
yields one edge, ever. A one-frame press is unreliable too: `PlayerUse` is reached only from
`ImpulseCommands` ← `ItemPostFrame`, which returns early on `m_flNextAttack`. So `RunArrival` pulses
`IN_USE` every 0.2 s (`kFunPulse`), and `BotFire::Think` has a `BotFun::Pressing(bot)` early-out beside
the `BotBreak::Active` / `BotDeploy::Active` ones, so the weapon code cannot push `m_flNextAttack` out
from under it.

### The tank

`func_tank` has no use caps; only the `func_tankcontrols` `SOLID_TRIGGER` brush does, and it resolves its
tank once at spawn from `target`. `TankOfControls` reads `CFuncTankControls::m_pTank` and falls back to a
name match on `strncmp(cls, "func_tank", 9)`.

Placement prefers standing *beside* the controls volume (`BotItems::StandArea`), with volume areas within
`kTankReachRange` as fallbacks: the centre of a tall brush is the geometry that fails the dot test above.

Two mechanics shape the state machine:

- **The leash.** `m_vecControllerUsePos` is the player's origin at the instant of mounting, and the leash
  is `< 30` units in 3-D. A mounted bot sends zero move input. A mount immediately followed by an
  unexplained dismount is this and nothing else.
- **`+USE` cannot dismount.** `ItemPostFrame` returns at once while `m_pTank` is set (player.cpp:3432),
  so `PlayerUse`'s dismount branch is unreachable. The only way off is to walk more than 30 units away.

Aiming is indirect. The tank slews its own `pev->angles` toward the controller's `v_angle` (pitch
inverted) at `m_yawRate` / `m_pitchRate` deg/s, clamped to `m_yawCenter ± m_yawRange` and
`m_pitchCenter ± m_pitchRange`, and bullets leave along the **tank's** angles. So `IN_ATTACK` is held only
once the tank is within `kTankAimTolerance` (4°); firing on the crosshair sprays while the barrel swings.

### Three ways off the tank, and only one of them is boredom

- **Boredom.** No enemy seen for `kFunBoredom` (10 s): dismount and record
  `abandonReadiness = EvaluateCombat(...)` against that tank, which `Collect` then filters for this bot
  until readiness rises above the mark. That sends the bot to resupply and come back instead of
  remounting the same gun in the same state.
- **Shot from outside the cone.** A mounted bot has no weapon and a barrel limited to
  `m_yawCenter ± m_yawRange`, so an attacker outside it cannot be answered. `BotCombat::DamageSource` is
  read every mounted frame, **before** `Bot_RunPlayerMove`; a source outside the cone, or of unknown
  position, dismounts at once, and `BotCombat` picks the bot up next tick on the same damage. This exit
  records no `abandonReadiness`, only a short flat cooldown on that tank (`kFunTankShotCooldown`, 20 s),
  so the bot may return after the fight but cannot loop mount → shot → dismount while spawn-camped.
  Damage from *inside* the cone is what the gun is for.
- **The tank died, or the bot did.**

### Taking a tank mid-fight

`BotCombat::Run` asks `BotFun::CombatTank` before it drops the goal to fight. Every 0.5 s it scans the
`FUN_TANK` entries and keeps a use point from `CollectUsePoints` only when all three hold:

- it is closer to the bot than the enemy is (straight line),
- it lies toward the enemy: 2-D `dot(bot→point, bot→enemy) >= kCombatTankDot` (0.7, about 45°),
- the enemy's eye is inside the tank's traversable cone; otherwise the gun would sit aiming at its cone
  centre and get the bot shot.

The nearest survivor is committed straight as the target with `BotGoal::Commit` (no detours, no camp
route), `combatTank` is set, and combat returns false with `driving` cleared. `Bot_MoveNav` walks the
ordinary path, and `RunArrival` / `Run` mount and fire as for any fun goal.

Once committed the pursuit is held until the target turns invalid or `Forget` runs; re-testing every
frame flipped it as the bot's angle to the enemy changed. Failure needs no new code: a repath failure,
a block or the arrival timeout puts the usual cooldown on the controls entity, `combatTank` clears with
`Forget`, and the next `Run` drops the goal and fights on foot. Reachability is not searched at scan
time, since that would be a Dijkstra on the frame a fight starts and the repath failure answers it anyway.

Log: `X goes for the tank func_tank mid-fight, N out, <enemy> M out`.

### FUN_TRAIN: the subtransit tram

**The tram does not start on its own.** `startspeed` is 0, so `CFuncTrackTrain::Find` ends with
`pev->speed = 0` and `Next()` returns on its first tick without rescheduling. Nothing targets `train`,
and the `speed 0` keys on `trainpath05`, `05.5` and `20.1` are inert (`Next()` skips zero speeds, and that
branch is gated on `SF_TRACKTRAIN_NOCONTROL` anyway). The 43-node loop never runs until a player drives
it; once driven it circles forever, since the track is closed.

**Taking the controls is not the `PlayerUse` search loop.** `PlayerUse` handles trains first: on
`m_afButtonPressed & IN_USE`, if `pev->groundentity` has `FCAP_DIRECTIONAL_USE`, `FL_ONGROUND` is set,
`IN_JUMP` is not held and `OnControls(pev)` passes, the player gets `PFLAG_ONTRAIN`. No cone, no 64-unit
radius, no `UTIL_ClampVectorToBox`: stand on it, look anywhere, press `+USE`.

**`OnControls` is a box test on the origin, not "standing on it".** `Spawn` caches
`m_controlMins = pev->mins` and `m_controlMaxs = pev->maxs` with `m_controlMaxs.z += 72`, and tests the
player's *origin* in tram-local space. A bot on the deck **edge** has the tram as ground entity while its
origin is up to half a hull outside the box (measured: `local.y = 80` against `y[-65,65]`), and `+USE`
silently does nothing. So the mount phase walks to the deck centre and pulses `+USE` only inside
`kTrainCentreRadius`. This is the module's most confusing failure, because every visible precondition
looks satisfied.

**Firing blocks `+USE`**, through the same `ItemPostFrame` / `m_flNextAttack` early return as above.
`BotFun::Pressing` is true during `TRAIN_MOUNT` and `TRAIN_RELEASE` so `BotFire` idles, and false during
`TRAIN_DRIVE` and `TRAIN_RIDE`: acceleration goes through `PreThink`, not `PlayerUse`, so the bot can
fight while driving and riding.

**Acceleration is four edge-triggered presses.** `PreThink` fires `Use(USE_SET, ±1)` on
`m_afButtonPressed & IN_FORWARD`, stepping `pev->speed` 0 → 100 → 200 → 300 → 400; the `kFunPulse`
alternation supplies the edges. Control drops the moment the bot leaves the ground, strafes
(`IN_MOVELEFT|IN_MOVERIGHT`) or presses `+USE` again, and `pev->velocity` is zeroed every `PreThink`
while held, so `TRAIN_DRIVE` and `TRAIN_RELEASE` send zero `fmove`/`smove` and no jump.

**No gamedata offsets.** `FL_ONTRAIN` on `pev->flags` says whether the bot holds the controls,
`pev->groundentity` whether it is aboard, `pev->speed` is the signed speed, `Spawn` caches the top speed
in `pev->impulse`, `pev->spawnflags` carries `SF_TRACKTRAIN_NOCONTROL`, and the track is the `path_track`
chain through plain `target`/`targetname`/`origin` keys.

### Board points come from the track, not from the tram

No rendezvous is predicted. `BuildBoardPoints` walks the cached `path_track` polyline once per map and at
each node probes both sides for nav areas to hop from:

- **Probe offsets start at `halfWidth + kTrainSideClear`**: the tram's half-width plus the bot's hull
  half-width plus a margin. From the deck centre the board point lands inside the tram's footprint and
  bots walk into the parked brush until `OnBlocked` retires the goal; at exactly `halfWidth` the bot's
  32-unit hull is flush with the car and a passing tram clips a bot standing still.
- **The stand height is traced, never taken from the nav area.** `GroundBelow` fires a downward duck hull
  from the area point; `standZ` is the floor plus 36. A `centroid.z` is nowhere near the floor (on
  subtransit -128 against a floor of -192), and the numbers look plausible while inverting the geometry:
  the deck seems *below* the walkway when it is 25 units above it.
- **The ground probe starts at the area point, not above it.** The area point is in open space; 96 units
  higher is inside the tunnel ceiling, `fStartSolid` fires, and the board point is silently dropped. The
  symptom is a tram nobody boards, with no log line.
- `deck.z - floorZ <= kTrainStepUp` and `floorZ - deck.z <= kTrainBoardRise` keep the point roughly level
  with the deck, both against the traced floor.

`point` stays the nav-space goal point from `ClampToArea` and is **not** moved to the traced floor. It
feeds `follow.done` and the air-route trace; at floor level that trace drops to deck height, the trench
lip blocks it, and every board point beside the parked tram disappears.

The deck is `DeckAt`, not `pev->origin`: the brush model is offset from its origin (64 units along the
car on subtransit), and the tram's forward is the **reverse** of its travel, because `Find` adds 180° to
the angles it derives from the look-ahead.

### Start it, or catch it

The mode is chosen from the live `train->v.speed` on arrival, so a bot that set out for a parked tram and
finds it moving switches to catching.

- **Stopped:** walk at the deck and hop on (`kTrainJumpGap` from the bbox, `IN_JUMP` only when below deck
  height), mount, accelerate to `pev->impulse`, release with one more `+USE`, and ride.
- **Moving:** the bot is trackside, so it stands still and computes
  `ArcForward(ProjectOnTrack(tram), boardArc, sign(speed)) / |speed|`. If the tram is not due within
  `kTrainCatchWindow` (3 s) the goal is banned by `abandonReadiness` alone, with no `readyTime`, so the
  bot resupplies and the tram becomes interesting again once it is better armed. If it is due, the bot
  waits and hops.

The hop aims at the **cached trackside deck point**, not the tram's current centre: running at a moving
tram means hitting its front face, and `Blocked` crushes for `dmg 1000` unless the victim is already
`FL_ONGROUND` **with the train as ground entity**.

### The hop is a timed jump, and the deck is above the walkway

On subtransit the ledge floor is -192 and the deck top -167, so **boarding is a 25-unit rise**, not a
drop (see the `centroid.z` trap above). `TryTrainHop` computes, from the traced geometry, the interval in
which a jump leaves the bot at or above deck height:

```
rise      = (boardDeck.z + 36) - origin.z
clearFrom = (kJumpImpulse - sqrt(kJumpImpulse^2 - 2*g*rise)) / g      -- clears the deck going up
clearTo   = (kJumpImpulse + sqrt(kJumpImpulse^2 - 2*g*rise)) / g      -- drops back below it
```

and jumps only when the tram's `eta` is inside `[clearFrom, clearTo]`. A rise above
`PmSim::PlainJumpHeight` is rejected. `eta` is used unmodified: `b.deck` is by construction where the
deck is when the tram's origin reaches `b.arc`, so `TrainEta` already is the deck arrival time.
Subtracting a deck-lead term double-counts and puts `eta` below the window, and the hop never fires.

Two guards separate boarding from being crushed:

- **The brink.** The bot never crosses `halfWidth + kTrainBrink` on foot. Short of it the bot runs; at it
  the bot jumps or waits for the next pass. Air control is weak, so the run-up must happen outside the
  car's sweep.
- **No proximity trigger.** Firing on raw distance to the tram ignores geometry and commits hopeless
  jumps, leaving the bot airborne in the car's path, which is exactly what turns `Blocked`'s nudge into
  its `dmg 1000` branch.

**Why not `PmSim`:** it works at runtime (`BotOppo::TryJump` predicts landings with
`Bot_SimulateAirmoveArc`), but every trace is a snapshot of the world *now*, and asked where the bot
lands it would answer "in the trench", true at query time and false at landing. The tram is on rails at
constant speed, so its future position is closed-form.

`RunArrival` only hands over the first frame; everything after runs from `BotFun::Run`, called
unconditionally. `RunArrival` fires only while `follow.done`, and the bot stops being at its goal point
the moment it steps toward the deck, so an arrival-driven state machine stalls silently mid-boarding.

**Riding** is standing on a moving platform: zero move input, aim and fire as usual, and a nudge back to
the deck centre when drift exceeds `kTrainCentreRadius` (otherwise bots slide off on the curves). Boredom
is the tank's rule. Dismount looks each frame for a nav area within `kTrainHopRange` of either side and no
more than `kTrainSafeDrop` below and jumps onto it; `kTrainRideMax` (60 s) forces a bail if nothing
qualifies.

### What is deliberately not here

- **Surviving its own gimmick.** Pressing the crossfire nuke starts a 52 s fuse and seals the bunker at
  t=20; the activator is inside, so a bot that presses and stays put happens to live. doublecross then
  poisons that bunker for 115 s. After its first ride the subtransit trench is lethal on a ~25 s cycle
  and the mesh says nothing, because time-varying area costs have no room in the nav format; a bot that
  mistimes a catch and is crushed is accepted. Evacuation belongs with strategic goals.
- **Cooperation.** rocket_frenzy needs three buttons armed before its launch button unlocks; rustmill's
  bunker door needs two held at once. The `multisource` read makes a bot *skip* a locked button; it does
  not make two bots unlock one.
- **`func_mortar_field` with `m_fControl == 1`** shells the activator. `m_fControl` is not in
  `gamedata/valve-win.json`, and no stock DM map puts one behind a switch, so it is treated as interesting
  and left alone.
- **`func_vehicle`, `func_tanklaser`, `func_tankrocket`, `momentary_rot_button`**: zero occurrences in all
  19 DM maps. There is no `func_traincontrols` either, which does not matter: a `func_tracktrain` is
  driven from its own deck.
- **The parked tram in the mesh.** navgen ignores `func_tracktrain`, so the parked tram is a solid brush
  the mesh does not know about. On subtransit it matters little: the underside sits 48 units above the
  trench floor (a duck slot) and the deck is above the walkable ledge. If it is ever modelled, it must
  **not** go in `crossbot_nav::IsBakedBlocker`: `Bot_TraceHitEntity` reads that predicate as permanent
  world geometry, which stops being true once the tram is driven. It would need a navgen-only case that
  takes the runtime origin from `target` → `path_track` rather than the editor `origin` key
  `ExtractBlockingVolumes` uses; on subtransit those differ by `(+64,+704,+496)`.

## objectives.cpp

What a `cstrike`, `dod` or `tfc` map is *for*, read from the map's own entities. It replaced the
`bot_goal.sma` AMXX plugin for CrossBot's bots. The plugin still runs for foxbot, Parabot and
marine_bot_dod, and `crossbot_objectives 0` hands CrossBot back to it, which is the A/B switch.

### Keyed on the map owner, not the bot

`Objectives::DetectMap` runs next to `Teams::DetectMap` and picks a reader from
`XPlay::GameDir(XPlay::MapOwner())`: `cstrike`/`czero`, `dod`, `tfc`, or none. Any other owner gets no
reader, the strategic tier sees only mapper-placed `bot_goal` entities, and bots play fun and gather.

The reader cannot live on `IGameProfile`, which is the *bot's* game (`valve` on every one of these
maps). The offsets it reads are the map owner's too; see `doc/games.md`, "Objective offsets belong to the
map, not the bot".

### Three tables, not N game modes

A reader fills three tables and decides nothing:

| table | CS | DoD | TFC |
| --- | --- | --- | --- |
| **Site**: act on a place | bomb target, rescue zone, VIP safety zone, escape zone, planted C4 | `dod_capture_area`, touchable `dod_control_point`, TNT `func_breakable` | touch `info_tfgoal` that scores, detpack `info_tfgoal` |
| **Payload**: a loose thing to carry | dropped C4 (`weaponbox` + `m_bIsBomb`), free hostage | `dod_object` | `item_tfgoal` |
| **Carrier**: a player bearing one | `m_bHasC4` (hidden from CT), `m_bIsVIP`, the player a hostage follows | `CObject::m_hOwner` whose `CBasePlayer::m_pObject` is that object | the item's `pev->owner` / `aiment` |

A planted `dod_object` keeps its `m_hOwner` and `m_bCarried`, hidden, for the rest of the round; only the
player's `m_pObject` moves off it. Read from the object alone, a planter stayed a carrier and walked its
next `deliver` empty-handed (dod_charlie, 10 bots: all five Allies after their first plant, Player4 counted
twice).

A site carries per-team attack and defend masks and an optional need (a payload id or group). A carrier
carries the id and group of what it bears. `BotStrategic::Intents` turns the tables into goals with one
set of rules, so a VIP, a bomb carrier, a hostage rescuer and a flag runner are the same thing to every
other bot:

| relation | intent | rank |
| --- | --- | --- |
| a detpack site I can blow (TFC demoman) | `attack` | 4 |
| I carry what a site needs, or the site is urgent | `deliver` / `attack` | 3 |
| a loose payload my team carries, under the shared cap; an urgent site I defend | `fetch` / `defend` | 2 |
| a site I attack that needs nothing, or whose need is not lying loose | `support` | 1 |
| a payload my team defends | `defend` | 0 |
| a site my team defends | `defend` | 0 |
| nothing at all | `roam` to a spawn | 0 |

- **No escort or hunt.** A friendly carrier is the runner a `support` bot plans around
  (`bot_tactics.cpp`), not a target. An enemy carrier is unknown to anyone who has not sensed it.
- **Defend only when there is nothing to take.** Any rank-1 `support` intent removes every rank-0 and
  rank-1 `defend`, so a team pushes while any site is open to it and holds only what is left (dod_charlie's
  Axis, a CS CT). A per-life attacker/defender coin flip parked half of each team at its home flags:
  dod_avalanche, 24 bots, flags frozen from 94 s to 255 s with 13 to 16 bots standing, 1366 s of defend
  fallback camps in 240 s. A rank-2 fortify `defend` (`doc/games.md`) is above this rule.
- **Defend only what the enemy can reach, nearest the enemy first.** Each team has one cost field, a
  multi-source `Nav::AreaCosts` seeded from every one of its spawns and priced as that team
  (`Nav::ScopedPlanTeam`), cached in `bot_tactics.cpp` until `Nav::BarrierGeneration` moves.
  `BotTactics::EnemyField` is the minimum over the enemy teams' fields. `BotStrategic::Collect` reads it
  over **every** nav area inside a defended volume (`VolumeAreas(..., all)`), or the target's own area and
  its standoff for a point: a defend no area of which is reached is dropped, and the survivors roll on the
  enemy's time in place of the bot's own walk (`Candidate::rollCost`). `crossbot_strategic_debug` prints
  `weighs the defend X #N at Ns from the enemy` or `leaves the defend X #N alone, no enemy spawn reaches
  it`. When no defend is reachable at all (no enemy spawn on the mesh) every defend is kept and rolls on
  the bot's walk, the old rule.
  - **Not the capped entrance list.** Other volume goals offer only their 8 areas nearest the bot
    (`kHealAreaCandidates`, a cap on candidates); a defended volume offers all of them, and
    `PruneUnreachable` plus `KeepCheapestPerEntity` leave the defender the cheapest it can walk to. On
    dod_charlie the beach capture areas span the shingle: with the cap the Axis's 8 lay on the shingle
    itself, so the enemy test called every site unreachable, and once that was fixed the Axis could still
    walk to none of its own 8 and spent the time before the first blow on `fun func_button`.
  - A payload defend shares rank 0 with sites, and interchangeable payloads are one ticket: of the loose
    payloads a team defends that share a non-zero group and the same id, only the one nearest the bot
    becomes a `defend` intent. DoD gives a `dod_object` its group as its id, so a pile is one ticket; a
    TFC item's id is its own goal number, so flags sharing a `group_no` stay separate. A DoD
    `dod_object` is defended by the team that cannot carry it. Charlie's Allied satchel piles (20 in group
    `tnt`) sit beside the Allied spawn: one ticket per pile gave them 39 of 40 Axis defend picks against
    the capture areas, one per group 24 of 79 tactics picks (5 v 5, 240 s), and the cliffs overlooking the
    beach exit see the piles. Group 0 is no group, so hostages and the C4 stay one ticket each.
  - DoD spawn masters are not read (the master sits in private data CrossPlay's gamedata does not name),
    so an inactive forward spawn seeds its team's field too; charlie's Allied spawns have no master.
- **Picking.** `BotStrategic::Prioritize` keeps the highest rank present, `KeepCheapestPerEntity` keeps
  one entrance per volume (so a brush with eight entrances is not eight tickets), and the ordinary
  `1/(cost+2)^bias` roll picks among the rest. The hostage cap is the plugin's,
  `max(1, hostages / live teammates)`. The rank rides in `Candidate::payload` above the kind, so the tier
  needs no new fields.

### Holding

A native hold ends when its intent disappears: `ValidFor` re-derives the intents and looks for the same
entity and kind, so the area being taken, the hostage following, or the flag being picked up all end it
with no special case. On top of that:

- `attack`, `deliver`, `fetch` and `defend` give up after 30 s (`kHoldLimit`) and memo the entity for
  20 s, which rotates defenders between sites and stops an attacker waiting forever on a numcap-2 area it
  cannot take alone. None of the 30 s runs while the bot is frozen (`Bot_Frozen`, `doc/movement.md`),
  which is what a defuse is.
- `roam` memos its spawn for 60 s on arrival.
- Hostage `+USE` is pulsed at 0.2 s, because a hostage wants a press edge; the planted C4 is held,
  because defusing is continuous use.
- Planting is not a hold: `deliver` only walks the carrier onto the bomb site, and the CS profile's
  `AbilityRun` arms the bomb once the game's `SIGNAL_BOMB` bit is set (`doc/games.md`).

**A solid point goal is placed short of itself**: 40 units toward the bot for a `+USE` fetch
(`kUseStandoff`, inside the 64-unit use range) and 96 for `defend` (`kGuardStandoff`), falling back to the
entity's own point when that spot resolves to no area. Aimed at the hostage's own origin, the route ended
inside the hostage (`could not get past hostage_entity` before any `works hostage_entity`).

### Measured facts the readers rest on

- **CS `pev->team` is the reverse of `CBasePlayer::m_iTeam`.** Every bot on de_dust2 read `pev->team 2
  m_iTeam 1`. The reader keys on `pev->team` (1 = CT, 2 = T), the number `Teams::TeamOf` and every other
  gate use; ReGameDLL never writes `pev->team` for a player. `crossbot_list_objectives` prints both.
- **DoD's `point_can_allies_touch` key means the opposite.** A value above 0 sets `m_bAlliesCantTouch`
  (dod.c:118399), so the reader reads the bool, never the key. On dod_avalanche every point is
  cant-touch for both teams and all five objectives are capture areas.
- **A DoD capture area's own owner field is a cache.** `CAreaCapture::AreaTouch` (dod.c:116922) copies
  `m_nOwningTeam` from its `target` control point only when someone touches it, so on dod_charlie every
  area read neutral and the Axis had nothing to defend. `DodAreaOwner` reads the targeted
  `dod_control_point`'s `m_iTeam`, falling back to the cache. `AreaTouch` also returns at once while the
  area's `CBaseToggle::m_sMaster` is not triggered, so a closed master drops the site (charlie's shingle
  and wall areas target no point and only close their master once blown). An ownerless area only one team
  can take is defended by the other.
- **A DoD TNT breakable's team bits are optional.** `dod.fgd`: 16 Allies Only, 32 Axis Only, 64 Object
  Cap Only. With neither team bit the charge belongs to whichever team can carry its
  `CBreakable::sz_Group` object, so attackers come from the payloads and carriers of that group, and
  both teams only when no object matches. Reading the team bits alone dropped dod_escape's `blow2`
  (`*178`, spawnflags 64, `axis_obj`), the charge that actually scores through `rebar` →
  `dod_point_relay` → `axis_flag`, and kept the bridge charge beside it (`*204`, spawnflags 96).
- **A TFC goal's team can hide behind `teamcheck`.** `APMeetsCriteria` compares the activator's team with
  `GetTeamCheckTeam` (tfc.c:111208), which reads the `team_no` of the `info_tf_teamcheck` so named; the
  goal's own `team_no` may be 0. avanti's church-hole detpack goal is `teamcheck "attackers"`, so reading
  `team_no` alone made every demoman think it was theirs. The reader resolves `teamcheck` for sites and
  `owned_by_teamcheck` for payloads, falling back to the plain fields.
- **A detpack goal is a site with nowhere to stand.** `goal_activation & 2` goals are usually point
  entities inside the wall they open, so resolving the entity gives a point on the far side. Their sites
  are placed on every nav area within 1400 units with a clear line to the goal, the trace the blast itself
  does (`DetpackExplode` 22955), and the cheapest reachable one wins. `Tables::detpackMap` records that
  the map has such goals at all, which stops a scared demoman wasting the pack elsewhere.
- **A higher rank preempts only through `Outranked`.** An event that removes the current intent ends the
  goal through `ValidFor`; one that only adds a better intent leaves the old one valid, so a CT holding a
  bomb site never saw the planted C4 (rank 3 over 0). `RefreshStrategic` drops the current goal when any
  higher-ranked intent is available and not on cooldown, at most once every 10 s per bot
  (`kOverrideRetry`). Without the limit a bot whose better intent was unreachable re-picked 1657 times in
  five minutes.
- **TFC `items_allowed` is a goal number, not a mask.** `APMeetsCriteria` (tfc.c:111822) passes only if
  the `item_tfgoal` with that `goal_no` has the activator as `pev->owner`; `has_item_from_group` is the
  same test on `group_no`. A touch goal fires only with `goal_activation & 1` and `goal_state != 1`
  (tfc.c:114591), and the reader mirrors the `if_goal_is_*` / `if_group_is_*` conditions against live
  `goal_state`. That polling replaces FoxBot's per-map scripts, which are mostly waypoint groups switched
  on broadcast strings.

### What it does not do

- Rocket- and grenade-only DoD breakables (spawnflags 512, 256, 8) are not read: a `valve` bot has
  neither weapon. Nor are `dod_point_relay` chains, which only say which point a breakable leads to.
- VIP detection sees only real `cstrike` players. TFC VIP maps get their scoring escape goal but no
  civilian hunt.

## bot_strategic.cpp

The top tier, with two producers: the native intents above, and `bot_goal` entities. With
`crossbot_objectives 1` on a map `objectives.cpp` reads, a player-owned `bot_goal` is ignored and only
ownerless, mapper-placed ones count; everywhere else both kinds are consumed.

### The wire protocol

`bot_goal` is not a registered entity class. It is a bare `info_target` whose `pev->classname` the plugin
rewrites at runtime, so **only `entvars_t` fields carry meaning and no game context is needed to read
one**; `doc/games.md` rule 3 does not apply.

| field | meaning |
| --- | --- |
| `v.classname` | `bot_goal` is live; `bot_goal_disabled` is one a map trigger switched off |
| `v.origin` | where to go |
| `v.owner` | non-null: the goal belongs to that player alone. Null: global, everyone's |
| `v.aiment` | optional entity to look at and press buttons on (a hostage, the C4, a map button) |
| `v.button` | bitmask of `IN_DUCK` / `IN_USE` to hold once close enough |

The plugin's `goal_think` runs at 1 Hz and keeps one **owned** goal per cross-game bot (exactly
CrossBot's bots), moving its origin as the objective changes. A mapper can place a **global** one in the
`.map` or a `maps/<name>.ent` entity-lump override; it has no owner and every bot sees it.

The distance rule is the three sibling bots', not invented: `dist = max(len2D(dir), |dir.z|)` from the
eye, gated on `dist <= 128` **or** a clear traceline to the aiment; then `dist <= 128` enables `IN_DUCK`
and `dist <= 64` enables `IN_USE`. The aim point is the aiment's bbox centre (Parabot's refinement): a
hostage's origin is at its feet, and aiming there from two metres misses the use trace.

### A goal that moves is a re-plan

`Select` bakes `target.point`, `target.areaId` and the reverse field `toTargetCosts` once, and a per-bot
`bot_goal` moves every second. `BotGoal::State` therefore also carries `targetOrigin`, the raw `v.origin`
at bake time; it is **not** the clamped nav `target.point` and cannot be compared with it.

`BotGoal::RefreshStrategic` runs from `Bot_MoveNav` right before `BotCamp::Run`, at most every 0.25 s:

- Within `kGoalMoveRadius` (32 units) of `targetOrigin` it does nothing.
- Past that it re-resolves the area, refreshes `targetOrigin`, recomputes `AreaCostsToGoal`, re-runs
  `PickStop` and resets the path (log: `follows the strategic goal to ...`).
- If the entity is gone or no longer a `bot_goal` this bot may have, the goal is cleared and the next
  `Select` starts over.
- A hold skips the re-plan, as an arrival does, so a drifting goal does not pull a bot off its post.

Running before `BotCamp::Run` is what lets `Outranked` and `ValidFor` end a camp hold: `Clear` resets the
camp, so a CT on a 15 s defend post leaves the moment the C4 is planted. Placed before `Select`, which a
holding bot never reaches, it could not.

### Arrival is a hold, not a touch

`BotStrategic::RunArrival` sits at the `follow.done` site beside `BotFun::RunArrival` and returns true to
keep holding; `NavFollow::Track` keeps reporting `done`, so it is re-entered every think. It faces the
aiment, mirrors `v.button` into the bot's buttons, and stays. `v.button` is authoritative and rewritten
every tick, so nothing is latched or pulsed.

Standing still **is** the objective for a DoD capture area or a TFC dropoff, and the plugin ends the hold
by moving or deleting the goal when the objective flips; its roaming fallback re-rolls once the bot is
within 256 units. So for a `bot_goal` entity `RunArrival` returns false on exactly two conditions, the
entity died or moved further than `kGoalMoveRadius`, and the ordinary `reached goal` path re-selects.
Native intents add their own exits (`objectives.cpp`, "Holding"). Combat needs no special case:
`BotCombat::Run` runs earlier in `Bot_MoveNav` and clears the goal.

A repath failure or hard block puts the goal on a 10 s per-bot memo (`kGoalFailCooldown`) instead of
killing the tier, so an unreachable objective lets fun and gather have a turn instead of trapping the
bot in a re-select loop.

## bot_tactics.cpp

Decides where a `support` or `defend` bot actually stands. There is no team board: a bot knows where its
friends are and nothing of what they intend, and every bot applies the same rule, so spreading out and
leapfrogging fall out of it rather than being coordinated. `BotTactics::Pick` runs from `PickStop`,
before the gather detour, whenever the target is strategic:

| kind | tries, in order | without a camper gun | when nothing qualifies |
| --- | --- | --- | --- |
| `defend` | the runner rule with the enemy as runner, then an overlook (below) | the same | the target: the goal, or the nearest area to it the bot can reach |
| `support` | a sniper post, then the runner rule with a friend as runner, then an attack overlook | only the attack overlook, passed through | the site |
| `roam`, `heal` | a sniper post | nothing | the target |
| any other | a sniper post, then an attack overlook, neither while urgent | only the attack overlook, passed through | the target |

**A camper gun** (`BotFire::CampWeapon`) is a carried, usable weapon with a scope (`WeaponSpec::zoomFov >
0`), or anything the game lets the bot deploy (`IGameProfile::CanDeploy`: DoD's MG42, MG34, .30 cal, BAR
and Bren). Without one an attacker is a rusher: it takes no post and no support spot, and the attack
overlooks it still picks are holds of 0 s, so they only bend its way in. `RANGE_LONG` is not the test:
the Garand, the Kar, the AK47 and the M4A1 are all long-range and none of them is a reason to sit still.
Every timed hold (`BotCamp::CommitTimed`: tactics spots, posts, revenge camps) lasts twice as long
(`kCamperHold`) for a bot carrying a camper gun.

### One primitive: a camp spot ahead of a runner

The bot picks a `mesh.camps` spot `c` that lies ahead of a *runner* on its way to the target and that the
bot reaches first:

```
toTarget[c] < toTarget[runner]                                   ahead of the runner
toTarget[c] < toTarget[bot]                                       support: never behind the bot
fromRunner[c] + toTarget[c] - toTarget[runner] <= budget          on a near-ideal runner -> target path
clock + fromBot[c] + margin < fromRunner[c]                       the bot arrives first
not in the bot's spot memo, not BotCamp::SpotOccupied
weighted f(c) * fit / max(fromBot[c], 2), f = score/(1+score)       the spot's worth over the bot's walk
```

That is the gather detour's acceptance test with the runner in place of the bot. Nothing stores anyone's
path: an area is on a near-ideal path exactly when the two fields around it sum to the direct cost, so a
human teammate and a bot teammate are the same thing to it.

- **`fit` suits the spot to the held weapon:** `max(0.1, 1 - w * mismatch(dist))`, where `dist` is the
  camp's `standDist` or `crouchDist` for the stance it will hold, and `w` and `mismatch` come from the
  routing range bias (`Bot_RangeBias`: `kRangePref`, `RangeBias::shortDist/longDist`) without its
  readiness factor. A weapon with no range class leaves `fit` at 1.
- **A support's runner** is the nearest friendly carrier whose payload the site needs
  (`BotStrategic::Leader`), else a random live friend weighted toward nearer; `clock` is 0.
- **A defend's runner** is the enemy itself: `BotTactics::EnemyField`, so `fromRunner` is the walk from
  the nearest enemy spawn and `toTarget[runner]` is that field at the target. When the target is the
  approach area of a goal the defender cannot walk to, the enemy has no path to it and the rule offers
  nothing.
- **Picks weigh the spot, then the walk.** Every tactics roll (runner rule, overlook, sniper post) weights
  a spot `score * fit * crowd / max(fromBot, kWalkFloor)`, with `score` the navgen camp score
  (`doc/navgen.md`), `crowd = min(1, d / kCrowdRadius)` for `d` the distance from the spot to the nearest
  live friendly player (`kCrowdRadius` 1024), and `kWalkFloor` 2 s. A roll rather than the best, so
  friends who know nothing of each other's picks still spread; `crowd` reads only where friends stand,
  not where they are going.
  - Weighted by walk alone, `fit / (walk + 2)^2`, the spot did not matter: on dod_charlie the Axis rolled
    over 88 to 146 overlooks per pick, and two ledges navgen keeps for the beach (camp 0, score 23.8, and
    camp 6, 7.4) were offered usable 15 times each and taken once and never in 74 picks.
  - Weighted by the raw score alone the odds' tail took over: 7 of charlie's 978 camps score above 100
    (up to 406, median 2.25), and one of them got 11 of 66 Axis picks (5 v 5, 240 s). `crowd` is what
    spreads it: once a friend holds the spot, everyone else's weight for it falls.
  - Turning the odds into a fraction, `score / (1 + score)`, spread the picks almost evenly instead: 79
    Axis picks over 68 spots, none on the ledges (offered 22 and 32 times, all usable), because the walk
    then decided.
  - The walk stays in, linearly and floored at 2 s, so the spot a bot stands on does not win by being
    free: weighted by an unfloored walk, de_dust2's CT spawn, within the 3 s budget of the A site, took 8
    of 14 guard picks 50 to 81 units from `info_player_start`.
  - Sandbag spots take the score of the nearest navgen camp ("Sandbags and prone camps", below).

Every pick lands strictly closer to the target than its runner, and a support pick also closer than the
bot: without that a bot already ahead of its runner was sent back toward it. A bot behind its friend
jumps ahead; the friend, picking in turn, jumps ahead again; once no spot is left between a runner and the
site, the bot goes to the site. Leaderless attackers leapfrog in and converge instead of waiting on each
other.

### The defender's clock is in its head

A defender assumes the enemy spawned when it did. `clock` is seconds since
`BotStrategic::State::spawnTime`, so the imagined enemy has already walked `clock` seconds of its path.
Straight after spawning almost every spot on that path wins the race and the defender ranges out along
it; as the clock runs only spots far from the enemy spawn (near the site) still pass; past the enemy's
travel time nothing does and the defender goes to the target. The same rule serves CS rounds and DoD/TFC
respawns. `spawnTime` resets in `BotStrategic::OnRespawn` and in `NoteLife`, which runs every frame beside
`BotCamp::Track` and covers the first spawn after `crossbot_add`, where a bot starts with `wasDead` false.

### Fields

Every field the runner rule compares uses default planner parameters, so all are plain seconds.
`toTarget` and `fromBot` are two `Nav::AreaCosts` runs per pick, not the bot's own `toTargetCosts` and
forward field, and a friend runner costs one more. The per-team fields of "Defend only what the enemy can
reach" are cached here until the barrier state changes or `BotTactics::ForgetMap` runs at map change,
each priced as its team (`Nav::ScopedPlanTeam`): priced as the defender, a TFC spawn room behind the
enemy's own team door reads sealed and reaches nothing.

**Why not the bot's own fields:** they are not seconds. Carefulness scales each edge by
`1 - careful + careful * exposure` and the range bias scales it again: on de_dust2 `toTargetCosts` put
the enemy spawn 10.9 to 16.4 s from a site that the raw spawn field put at 18.8 to 21.5 s, a gap wider
than the 3 s budget. Sniper posts and overlooks compare only the bot's own `fromBot`, so they keep it.

### Holding, re-picking, failing

The spot is committed by `BotCamp::CommitTimed` as an ordinary camp stop marked `timed`, so arrival,
facing, crouch and zoom are the camp module's. A timed hold ignores the scare and friend-in-view rules and
ends on its own clock: 0 to 10 s for `support`, 5 to 15 s for `defend`, both doubled with a camper gun. Leaving calls `BotGoal::Advance`,
which runs `PickStop`, and so `Pick`, again against the same target with a fresh runner.

Every picked spot goes into `BotCamp::State::memo` for 30 s, so the next pick differs. A raced spot fails
for the rest of the life on either check:

- **The planned route loses.** `CheckTravel` reads `followProfile.totalTime` the frame after the commit,
  as for scare camps. Planned time runs well above the field estimate (de_dust2: 5.3 s planned against
  3.0 estimated, 12.7 against 6.5), so the runner's estimate is scaled by the bot's own planned/estimated
  ratio and the spot fails when `planned + clock + margin >= runner * planned / estimated`. Comparing raw
  field time made nearly every support pick lose.
- **Something closes in on the bot before it arrives.**

A frightened bot skips the race: `support` goes for the site, and `defend` takes an overlook or its
target. Neither carries a race, so neither check abandons them. (Abandoning raced spots while scared
burned every spot round the site for the rest of a life: 62 aborts in four minutes.)

Log lines: `supports X toward Y, camps at spot N (...)`, `guards Y at spot N (...)`, and `lost the race
to spot N ... not trying it again this life`.

### Overlooks: watching the goal from a camp spot

A `defend` takes an overlook when the runner rule finds no spot ahead of the enemy: meeting the enemy on
its way in comes first, watching the goal second. `BotTactics::Overlooks` builds the list the first time a
goal needs it. From every camp spot's eye (`BotCamp::SpotEye`) it traces:
1. To the goal's point. This passes if the trace runs clear or stops within 64 units (`kOverlookMargin`) of
   the goal's box (its brush bounds, or the point).
2. Failing that, to the floor-plus-36 centroid of each of the 12 areas nearest the goal by walk, within
   `kOverlookReach` (1 s). This passes only if the trace runs clear.

The constants live in `nav_format.h` because navgen runs the same test when it places camp spots and gives
every objective its share of them (`doc/navgen.md`, "Objectives get balanced lanes").

A spot that covers the ground in front of the goal counts even when a wall hides the goal itself.
dod_flash's British window at (-86,-276,194) faces the street south of FLASHVILLESTREET, while the flag
point is behind the jamb (the trace hits world at 14%).

**Breakables do not block the trace.** A hit on anything `BotBreak::IsTarget` accepts is skipped and the
trace goes on, up to 3 hops (`kOverlookBreakHops`). The first one is stored with the spot (`clearEnt`), and
`BotCamp::OnArrived` hands it to `BotBreak::Engage`, so the bot smashes the boards before it settles. The
same dod_flash window is boarded: every southward line hits a `func_breakable` 35 to 60 units out.

Each entry keeps the point it saw (`OverlookSpot::seen`). The hold faces that point, and `fit` is measured
to it.

The list is cached per entity index and point until `BotTactics::ForgetMap`; a goal that moves further
than the margin (a CS round's planted C4 on a reused edict) is traced again. Tracing on first use rather
than at map load is forced: a DoD area behind a closed master is not in the tables yet.

- **A goal the defender cannot walk to is still selectable.** When no area inside it is reachable,
  `BotStrategic::Collect` adds the nearest area to the goal that the bot can reach (`Approach`) as its
  placement, so selection keeps the goal and `Pick` then looks for overlooks from the bot's own field.
  dod_charlie's beach capture areas are reachable by the Allies and by no Axis bot until the shingle
  blows; without this the Axis had nothing to defend and spent that time on `fun func_button`. With
  overlooks they held 13 posts above the beach before the first blow, 1200 to 2100 units out
  (#43: 44 of 500 camp spots overlook it, #704: 41).
- **The pick** rolls over the overlook spots the bot can reach and `Usable` allows, weighted
  `score * fit * crowd / max(fromBot, 2)` like every tactics pick, with `fit` the routing range bias's
  mismatch at the spot's eye-to-goal distance.
- **A payload group is watched as a whole.** When the goal is a loose payload with two or more
  interchangeable ones (same group and id, as above: charlie's 20 `tnt` satchels), the weight is also multiplied by the share of the group's loose payloads
  the spot sees, by the direct test above, counting at least the one it qualified on: a spot over the
  whole pile keeps its weight, one that sees 1 of 20 keeps 1/20. Each payload's sight line from every
  camp spot is traced once per position (`SightOf`, cached by entity until it moves more than 64 units
  or `ForgetMap`), logged as `X #N is seen by N of M camp spots (... ms)`. The hold is `BotCamp::CommitSnipe` with weapon 0, so there is
  no race, no weapon switch and no closest-teammate rule, facing the seen point for 5 to 15 s. Log:
  `overlooks X from spot N (... away, ...s walk, fit ..., N spots[, breaking through])`. At first use the
  log reads `X #N is overlooked by N of M camp spots (N see it, N see within 1s of it, N through a
  breakable)`. With `crossbot_tactics_debug` every candidate logs `overlook spot N ... usable, ahead,
  breaks`.
- When no overlook spot is free either, the bot walks to its target and holds it (see "Holding").

**Attackers use the same overlooks on the way in.** In every game, an attacking intent (every kind but
`defend`, `roam` and `heal`, and never while `BotStrategic::Urgent`) whose runner rule found nothing rolls
over the goal's overlooks, keeping only spots with `toTarget[c] < toTarget[bot]` on plain seconds:
standing there leaves the bot closer to the goal than it is now. The walk to the spot is not limited, so
a spot reached round the far side of the goal counts.

The test makes it monotone. Each pick after a `0..kSupportHoldMax` hold lands nearer the goal,
and once no spot is left between the bot and the goal it walks onto the goal, as the runner rule's
leapfrog does. Approaching from overlooks spreads attackers over the angles a goal can be shot from,
instead of every life taking the shortest lane in. A camper holds each for 0 to 10 s (doubled); a rusher
holds 0 s, so it touches the spot and re-picks at once: the overlooks are its waypoints, and the log says
`passing` instead of `holding`.

**Any attacker may take a sandbag spot.** `Usable` normally keeps `CAMP_DEPLOY` spots for bots that can
deploy and that face the threat. On this path an open sandbag is enough: a rifleman stands behind it as
cover, and a gunner still deploys. On dod_flash the Axis spot 131 at (-118,-458,18) sees the flag and is
ahead from every Axis position, and it was refused only by that gate.

Two cases are excluded:
- Urgent intents (deliver, the planted C4, a detpack) must reach the spot itself.
- The closest teammate (`BotTactics::Closest`) skips it, so someone is always taking the goal.
  Weapon-0 holds have no in-hold `NowClosest` re-check, and the short hold bounds that.

Log: `approaches X from overlook spot N (... to the target from there, ... from here, ...)`.

### Sniper posts

A post is a camp spot that overlooks the target. Unlike every other camp it need not be on the way: a
dead end is fine, since the bot is there to shoot at the target. `Pick` tries a post first for any
strategic target that is not `BotStrategic::Urgent`, and falls through to the rules above when nothing
qualifies. `Urgent` is by rank (deliver and above), so the planted C4, an `attack` at deliver rank,
counts; a gate that named intent kinds let CTs with an AWP, a scout or a famas snipe a ticking bomb.

```
BotFire::CampWeapon                       the bot carries a scope or a deployable gun
BotFire::SnipeWeapon > 0                  best-valued RANGE_LONG weapon that still has ammo
!BotTactics::Closest                      a live friend is closer to the target, by toTarget
fromBot[c] <= toTarget[bot]               never walk further to the post than to the target
SpotDist(c) >= snipe_dist                 the spot looks far
snipe_dist <= d, d/2 <= SpotDist(c) <= 2d  d = eye to target point + 12; range matches the spot
angle(facing(c), eye -> target) <= 45     the target is inside the spot's measured 90-degree view
Usable (memo, no friend on it, sandbag rules), then Roll weighted score / max(fromBot, 2)
```

- **The closest teammate never snipes.** That is the guard against a team of snipers watching an
  objective nobody takes. It is re-checked every 0.5 s on the walk and in the hold (`BotCamp`
  `NowClosest`): a sniper that becomes closest abandons or leaves, and `Advance` re-runs `PickStop`, which
  now refuses the post. A lone bot is always closest, so the one-bot nav sweep never snipes.
- `BotCamp::CommitSnipe` is `CommitTimed` with no race, holding 5 to 15 s (10 to 30 with the camper gun
  every sniper has). It faces the target rather than
  the spot's measured axis, zooms whenever the weapon has a scope, and records the weapon; while the stop
  is a post, `IdleSelect` switches to that weapon and will not leave it. There is no line-of-sight trace:
  the facing and range match is the whole test.
- **`kSnipeDist` is 512, not `kZoomDist`'s 768.** `SpotDist` is a mean over everything visible and reads
  low (de_dust2: median 595, p90 847); at 768 each bombsite had 1 to 3 posts, at 512 it has 7 to 9.

Measured, 180 s, six bots: de_dust2 14 posts, 6 ended by `now the closest`; casbah (ten TFC bots) 22.
Log lines: `snipes X from spot N (... away, ... deg off its facing, ...s walk, ...s from the target),
holding ...s with <weapon>`, `... (it is now the closest to the target)`, and with debug
`finds no sniping post over X`.

### cvars

- `crossbot_tactics` (1): 0 keeps the intents but never picks a spot, so `support` walks to the site and
  `defend` guards it. The A/B lever.
- `crossbot_tactics_debug` (0): adds the no-spot and no-post lines, and one line per overlook candidate.
- `crossbot_snipe` (1): the posts' A/B lever.
- Constants in `bot_tactics.cpp`: `kBudget` (3 s of path slack), `kMargin` (1 s the bot must beat the
  runner by), `kWalkFloor` (2 s, the least walk a pick divides by), `kCrowdRadius` (1024, the friend
  distance past which a spot is not crowded), `kDeployBonus` (10: a sandbag's score ×10 for a gunner, ×1/10 for anyone else),
  `kSnipeDist` (512) and
  `kOverlookBreakHops` (3); in `bot_camp.cpp`, `kCamperHold` (2). In `nav_format.h`, shared with navgen:
  `kOverlookMargin` (64, how short of the goal's box a trace may stop), `kOverlookReach` (1 s),
  `kOverlookSamples` (12) and `kOverlookChest` (36), the areas near the goal an overlook may see instead.

## bot_camp.cpp

A bot walking to a goal that senses something **closing in on it** breaks off to a camp spot still on
the way, holds there facing the direction the spot was measured at, and resumes as soon as nothing is
getting closer. It is the consumer of `NavMesh::camps`, carried by the `.nav` since v21 (`doc/navgen.md`,
`nav_camp.h`).

Two constraints shape everything else:

- **A camp may never cost the bot ground on its target.** Same rule as the gather detour, same line:
  `if (in >= here) continue`.
- **The trigger cannot run a search.** Camping is a reflex, and a Dijkstra on the frame a bot is
  frightened is not.

### The route set: prepared once, tested for free afterwards

When `BotGoal::Select` picks a fun or strategic target it already holds both cost fields: the forward
`AreaCosts` from the bot and the reverse `AreaCostsToGoal` to the target. So `BuildRoute` is called there
and does one linear pass over `mesh.camps`, keeping what passes the gather-detour test with a 3 s budget
(`kBudget`; tighter than `kDetourBudget`'s 6 s because camping is a sidestep). `RefreshStrategic` rebuilds
it when a moving strategic goal drags the field. On crossfire that is 114 camps in and 4 to 79 kept.

Each survivor is flattened into a `RouteSpot`: position, facing, flags, and `toGoal`, the cost-to-target
of its area. The forward cost is **not** kept, since it was measured from where the bot stood at pick
time and is wrong a second later. `toGoal` is a property of the target, so the trigger can re-test "still
on the way" against `toTargetCosts[currentAreaId]` with two array reads. That is why the set is prepared
rather than searched.

**No route set, no scare reaction.** Gather targets get none: a gather target is its own stop and is
never detoured from, so a bot sprinting for a health kit does not stop to camp. Neither do urgent
strategic targets (`BotStrategic::Urgent`, rank deliver and above: a planted bomb, a carried flag, a
detpack run); otherwise a CT two seconds from a ticking bomb creeps toward a noise and camps while the
clock runs out. The charge below is gated on the route set too.

### One tracker, two rules

"Something is getting closer" and "nothing has got closer for a second" are one measurement, so there is
one running minimum:

```
contact = BotSense::NearestLive(origin, kScareRadius, livingOnly)
if (contact && (!scared || (contact.time > scareTime && dist < scareDist - 16)))
                                                     scared = true, scareDist = dist, scareTime = now
if (scared && now - scareTime > 1.0)                 scared = false        // and stop camping
```

- **No age gate on the start of a scare.** `NearestLive` has already applied the sense decay, so anything
  it returns is a contact the bot still believes in. An age window flickered the tracker on and off
  between footstep bursts several times a second.
- **A refresh needs new evidence**: `contact.time > scareTime`, the slot updated since the last refresh.
  Otherwise the bot's own walk toward an old sound shrank the distance and kept it scared until it got
  there, so every charge crept the full way (up to 1024 units, about eight seconds) and a bot routing past
  an old noise re-camped every cooldown. A scare now outlives its last *closer sound* by one second,
  whoever moved.
- Because the minimum resets with the scare, a contact that approaches, stops and approaches again
  frightens the bot twice. Because it is a minimum, an enemy that closes and retreats keeps the bot camped
  for one second past its closest approach.

**Charge or cover.** Once per scare, before looking for cover, the bot flips a coin weighted by combat
readiness (`EvaluateCombat`, 0..1). A win charges `scarePos` as a camp stop that never holds, creeping
with the profile's `CreepButtons()` (DoD `+duck`) or under its `CreepSpeed()` (CS 130, below the 150 at
which footsteps go silent); a profile offering neither never charges. A loss falls through to cover. The
charge ends on arrival or when the scare lapses.

CS creeps by scaling the move down to `CreepSpeed` in `Bot_RunPlayerMove`, where the buttons are added,
not by pressing `IN_RUN`: `+speed` walking is the client scaling its own `forwardmove` by
`cl_movespeedkey`, and ReGameDLL's server-side `IN_RUN` only ever speeds up (when `fuser3 > 0`).

**A visible teammate overrides the scare entirely:**

```
friend = FriendInView(scare radius, live view cone)   // refreshed at most every 0.2s
if (friend)   scared = false                          // no commit, and a holder leaves
```

Backup in sight means the bot plays on. The check runs only while a contact, scare or hold makes it
matter, and is gated on `Teams::Active`, so it is a no-op on deathmatch maps. It takes the closest live
teammate inside the scare radius and the bot's live view cone (90° when `v.fov` reads empty) and spends
one traceline on that friend, so an occluded closest friend masks a visible one further out; one trace was
judged worth more than that corner case. A scoped holder keeps its narrowed cone and rarely calms down.
The approach walk is not abandoned by it (see below).

`Track` is called unconditionally, straight after `BotCombat::Update` and above every early return in
`Bot_MoveNav`. A gap in its sampling would read as "nothing has come closer" the moment combat hands
control back.

### Committing, and the walk

The trigger picks the **nearest** qualifying spot within `kReachRadius` (512), mirroring the gather
detour's `min out`. It is committed through `BotGoal::CommitCampStop` as an ordinary detour stop carrying
`Stop::camp`: the target survives, the A* plans to it, `follow.done` detects arrival, and
`OnRepathFailed`/`OnBlocked` already go straight on. The one new arrival case is in `BotGoal::OnArrived`,
which hands a camp stop to `BotCamp::OnArrived` instead of advancing past it.

- **Abandon what is far by route.** The reach radius is euclidean and cannot tell a camp across the room
  from one across a chasm. `CheckTravel` reads `followProfile.totalTime` the frame after the commit and
  abandons anything over 3 s. On crossfire, a vertical map, that fires on about a third of picks.
- **A stale scare does not abandon the approach.** The one-second rule is about how long to *stay*. Applied
  to the walk too, it abandoned every camp, since a spot 350 units out is about a second away. Arriving
  somewhere on the way costs nothing, which is what the "never further from the target" filter buys.
- **One camper per spot.** `SpotOccupied` is the one test for both pickers and the walk: a live teammate
  within 96 units of the spot *and nearer to it than the bot*. The tie-break keeps two converging walkers
  from both giving up, and a holder at distance zero is never pushed off, since only `Consider`'s walk
  branch runs the check, never `Run`. A walker that loses abandons with "a teammate got there first" and
  clears its camp cooldown so it picks elsewhere at once: a timed spot re-rolls through `BotTactics::Pick`
  on `Advance`, a scare spot is re-picked by the selection loop next frame.

### Holding

`BotCamp::Run` sits directly below `BotCombat::Run`, so a real enemy always wins, and returns false until
the bot has arrived. It parks the bot and refreshes `lastProgressTime` so the stall detector does not read
a bot holding a corner as a stuck one.

- **The camp facing is the base, not an override.** `Run` writes the spot's yaw and pitch into
  `desiredAngles` *before* `ApplyLook`, so `LOOK_SENSE` blends from it toward whatever the bot hears: a
  camper ignoring the noise it camped against would be worse than none. With nothing to hear, the
  `LOOK_CAMP` branch at the top of `ApplyVantageLook` holds the measured facing, ahead of the vantage
  blend, which describes a neighbourhood where the camp facing describes this exact spot.
- **Stance leans to crouching:** duck unless standing sees more than 5% more
  (`crouchScore >= 0.95 * standScore`), and always on `CAMP_NO_STAND`. A crouched camper is smaller and
  steadier, so only a spot that genuinely needs the extra 34 units of eye height makes it stand.
- **Zoom is a property of the spot.** The spot's `spread` is the RMS angle of its visible targets off the
  measured facing, so twice it is the arc to sweep. Zoom when `spread * 2 <= zoomFov * kZoomShare` and
  the targets are far enough to magnify (`dist >= kZoomDist`). `zoomFov` is `WeaponSpec::zoomFov` of the
  weapon held when `BuildRoute` runs, so a bot with no scope never plans to zoom. `BotFire::ZoomButtons`
  owns the toggle and asks `BotCamp::WantZoom` whenever `fire.aimValid` is false, which is the usual state
  of a camping bot. The camp module keeps its last opinion after leaving, until the weapon agrees,
  which is what brings the scope back down. Rejected: a nested-arc test (`arc[2] >= 0.9 * arc[0]` over FOVs
  90/45/22.5/11.25), which carries no range and so cannot ask whether a scope is worth it.

### On leaving

`Leave` (stale scare) and `Abandon` (unreachable, blocked, camping switched off) only reset camp state.
Picking the next stop is `BotGoal::Advance`'s job at the call site, because `OnRepathFailed` and
`OnBlocked` already call it. Either way a 5 s cooldown (`kCampCooldown`) stops a frightened bot
chain-camping across the map.

### Measured

120 s on crossfire, six bots, `crossbot_gather 0` so every bot holds a detourable fun target: 63 camps
committed, 22 abandoned by the travel check, 26 reached and held, 21 ended by the stale rule after 1.0 to
2.3 s each. The other five were taken over by combat, the correct outcome.

### cvars

- `crossbot_camp` (-1, the difficulty tier; on below expert).
- `crossbot_camp_debug` (0): the per-retarget route line and the scare transitions. The commit, arrival and
  leave lines are plain `BOT_VLOG`.
- Constants in `bot_camp.cpp`: `kBudget` (3 s detour budget in `BuildRoute`), `kScareRadius` (1024),
  `kReachRadius` (512), `kZoomShare` (0.85, the share of the zoomed FOV the sweep may fill), `kZoomDist`
  (768, below which a scope buys nothing).

### Sandbags and prone camps

A DoD machine gunner fights best deployed, and a camp is where a bot stands still long enough to deploy,
so deploying rides on the camp instead of being a goal of its own. `IGameProfile::CanDeploy` is the only
game-specific question the camp code asks; DoD answers yes for anyone carrying a `Classify() == 0x100`
gun (.30cal, BAR, Bren, MG34, MG42).

**Sandbags become camp spots at map load.** `Objectives::ReadDod` lists each `dod_trigger_sandbag` with:

- `CSandBag::m_nYaw`, the brush `angles` (`InitTrigger` zeroes those, so `pev` cannot supply it),
- `m_nRange`, the `sandbag_range` key, default 15°,
- whether its master is open. Stock masters are `env_state` entities (charlie, kalt, zalec), on when
  `m_iState == 2`; a `~` prefix inverts that, as in `UTIL_IsMasterTriggered`; any other master class
  counts as open.

`BotCamp::AddSandbags`, right after the nav loads, appends one `Nav::CampSpot` per sandbag:

- at the nearest floor point inside the trigger (`BotItems::VolumeAreas`), facing `m_nYaw`,
- with `standScore` 1 and `crouchScore` 0, because the trigger refuses a ducking player,
- with `score` copied from the nearest navgen camp spot, since the two overlook the same ground and the
  tactics roll weighs by `score` (a fixed 1, below charlie's median 2.25, got the MG42s 1 sandbag in 23
  usable offers over 300 s of 10 v 10). The copy makes a sandbag as good as its neighbour; the tactics
  roll multiplies it by 10 (`kDeployBonus`) for a bot that can deploy, the only one it gives a bipod to,
  and divides it by 10 for anyone else, who may take an open bag only as cover on an attack overlook. At
  ×2 a usable bag averaged 3 times an ordinary candidate's weight for an MG42 and still drew 0 of 38
  picks among about 150 candidates (10 v 10, 300 s); at ×10 they took 5 of 42, about one in five of the
  26 usable offers (115 more were refused for facing away from the threat),
- with the runtime-only flag `CAMP_DEPLOY`, never written to a `.nav`,
- with the runtime-only flag `CAMP_DEPLOY`, never written to a `.nav`,
- with `arc = m_nRange + 45°` (the deployed traverse) and the trigger's `entIndex`.

On dod_avalanche that is all 12 of 12; every stock map has 6 to 46 sandbags.

**Only a gunner may pick one, and only facing the threat.** `SpotAllowed` drops `CAMP_DEPLOY` spots for
anyone who cannot deploy and sandbags whose master is shut; it is checked in `BuildRoute` and in
`BotTactics`' `Usable`. The facing rule is `|m_nYaw − threatYaw| ≤ arc`, with the threat direction by
picker:

- **Tactics `defend`** (overlook and runner rule): walk downhill in `BotTactics::EnemyField` from the
  spot's area, up to 16 areas or 512 units, and aim where the walk ends: that is where the enemy arrives
  from.
- **Tactics `support`:** the same walk down `toTargetCosts`, because the enemy is at the site.
- **Scare camps:** the contact that frightened the bot (`State::scarePos`). The route set is built before
  any contact exists, so this check waits until the trigger fires.

**At an ordinary spot a gunner deploys prone**, but only where standing sees no more than crouching
(`standScore <= crouchScore`): prone eyes are lower still, so only a spot that already favours a low
stance earns it. Elsewhere the gunner camps like anyone else.

The deploy kind travels on `RouteSpot::bipod` (`BIPOD_SANDBAG` / `BIPOD_PRONE`). A deploying spot never
ducks or zooms, and its hold time is unchanged (the timed 0-10 s / 5-15 s, or the stale-scare rule). The
pressing itself is `CDODProfile::MoveAdjust` (`doc/games.md`, `dod_profile.cpp`). A healthy run on
dod_avalanche logs `settles into its camp ... deploying on sandbags`, `deploys its gun on sandbags,
facing yaw 168`, `leaves its camp after 2.3s`, `folds its gun`, and the same for prone.
