# Bot System Comments

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

### Gathering never competes with fighting

Only items are candidates. An enemy used to be appended to the same scan as one more `Candidate`,
priced by the same Dijkstra and divided by an `EvaluateCombat` readiness, so a bot that saw someone
across the map planned a full A* route at them. That is gone: fighting is a state, not a goal, and
`BotCombat` takes movement away from the planner entirely for as long as it lasts (`bot_combat.h`
below). `SelectGoal` runs only when the bot is not in combat.

`crossbot_gather_debug` still prints the bot's readiness in the scan header, because `EvaluateCombat`
still drives carefulness.

### Carefulness

The same readiness drives *how* the bot travels, not only where. `Bot_Carefulness` returns
`(1 - readiness)` scaled by `crossbot_careful`, and both searches take it: `Repath` hands it to
`Nav::PlanPath` and `SelectGoal` hands it to `Nav::AreaCosts`. An armed, healthy bot routes for speed;
a bot with nothing to fight with routes for cover, however far around that takes it (`doc/nav.md`,
"Carefulness: routing away from exposure"). `BotOppo::RebuildCosts` is the one caller that must stay at
0 -- its gain test is a subtraction against real simulated seconds, so the exposure weighting must not
distort the cost values.

**It is sampled once, at repath time, and baked into the route.** A bot that arms itself halfway along
a timid route finishes that route; carefulness only refreshes on the repaths that already happen.

## bot_goal.cpp

Owns *what the bot wants*, in three tiers: `GOAL_STRATEGIC` (CS hostages, TFC flags, DOD areas — the
seam only; `CollectStrategicGoals` is a no-op for HLDM), `GOAL_FUN` (`bot_fun.cpp`), `GOAL_GATHER`
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

## bot_fun.cpp

The middle tier: the map gimmicks that make an HLDM map what it is. Two kinds ship —
`FUN_SWITCH` (a button or trigger whose chain reaches something that hurts people) and `FUN_TANK` (a
mounted gun). `crossbot_fun 0` empties the tier, at which point `BotGoal::Select` reduces exactly to
the old gather-only behaviour.

**This is entirely a runtime feature. No nav format change, no `kNavVersion` bump, no navgen work.**
Everything a fun goal needs is either a plain `entvars_t` field or an offset already present in
`gamedata/valve-win.json` (`doc/game.md`, gamedata.cpp).

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

The drivable-train tier is deferred, and the map survey argues for deferring it further: **no stock
DM map has a `func_traincontrols` at all**, and `func_vehicle`, `func_tanklaser`, `func_tankrocket`
and `momentary_rot_button` have zero occurrences across all 19 DM maps. subtransit's single
`func_tracktrain` runs its 43-node `path_track` loop autonomously and is a `dmg 1000` crusher, not a
vehicle.

## bot_move.h

### Frame time latching

A fake client receives no engine physics: `SV_Physics` skips every edict with index <= maxclients, so a bot advances only by the msec handed to `pfnRunPlayerMove`. The bot's movement clock is entirely our responsibility.

The engine's `pfnRunPlayerMove` does not restore `gpGlobals->frametime` after it executes: `SV_RunCmd` overwrites it with each command's msec and never restores it. After the first bot in the frame has moved, the global holds the previous bot's command length, not the frame time. Any bot code that runs between moves will read stale timing, so every movement calculation must latch the frame time before any bot runs.

`Bot_BeginFrame()` captures it at the top of `StartFrame`, before any bot has moved. `Bot_FrameTime()` and `Bot_FrameMsec()` return the latched value. Bot code must use these instead of `gpGlobals->frametime` for anything integrated per frame.

## bot_timing.h

### Movement clock diagnostics

A bot that receives less sim time per second than the wall clock spends looks like it is wading through honey -- both running and gravity appear slower. This module measures, per bot per second:

- **host frame time actually elapsed** (the budget)
- **msec issued** and over how many `pfnRunPlayerMove` calls
- **how much the engine actually executed** — `SV_RunCmd` silently drops commands while `host_client->ignorecmdtime > realtime` (the "clockwindow" speedhack guard), which is invisible from the game DLL unless you watch for `PlayerPostThink` not firing
- **the resulting drift** between bot sim time and real time, which is the same quantity `SV_CheckCmdTimes` compares against `clockwindow`

`crossbot_time_debug 1` enables per-second reports and a once-per-second frame dump showing what each bot in order saw as its frame time.

## hldm_profile.cpp

### Dead bot respawn

A dead bot's only physics tick is the respawn frame. `pfnRunPlayerMove` receives one frame's worth of msec (not a token 1ms), and leaves `gpGlobals->frametime` holding whatever msec it was given. Every later bot in the frame would inherit that stale timing if nothing restored it, so the respawn is a full frame's worth to avoid propagating old command msec into the next bot's calculations.

`crossbot_careful 0` disables the whole thing and restores pure fastest-route planning.
`crossbot_careful_pin` exists because readiness on a live bot does not reach the ends of its range --
a fresh spawn with a crowbar and a glock at full health sits around `0.35`, so the derived carefulness
never leaves the middle. Set it to a value in `[0,1]` to force one and sweep the range; `-1` (the
default) leaves it off. The `Repath` line reports `careful=`, `route=` (true seconds) and `score=`
(exposure-weighted) side by side, which is how you tell the factor actually reached the planner: at
`careful 0` the two are equal by construction.

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

## bot_aim.cpp

Everything the bot looks at goes through one function. `bot.desiredAngles` is the request, written by six
sites across `bot_move_nav.cpp` plus `BotSense::ApplyLook`; `Bot_UpdateFacing` is the only writer of `pev->v_angle` anywhere in the
plugin, and every drive branch calls it exactly once per frame and threads its return value into both
`Bot_DecomposeMove` and `pfnRunPlayerMove`. Aim is therefore fully decoupled from movement: the view can
point anywhere without disturbing where the bot walks, because the wish direction is world-space and gets
reprojected onto whatever yaw was actually commanded that frame.

### Why the flat turn rate had to go

The original controller stepped toward `desiredAngles` at a flat 300 deg/s, pitch and yaw clamped
independently. It turned at full speed from the first frame and stopped dead on the last one. Nothing
holding a mouse does that, and it was the single most obvious tell that a bot was not a player.

### Hand space, not angle space

The integrator state (`BotAim::State::yawVel`, `pitchVel`) is **not** angular velocity. It is hand velocity,
in degrees per second at a reference 90 degree FOV. Angles are converted in and out of it with
`s = fov / 90` (`pev->fov <= 0` means default, which is the normal case -- nothing else in the plugin reads
or writes `fov` today).

This is what makes zoom correct for free. A scope shrinking FOV from 90 to 20 means the same mouse movement
now covers 4.5x less angle, so the bot needs 4.5x more hand travel to make the same turn and gets there 4.5x
slower. Doing it in hand space means that when the scope snaps, hand velocity is untouched -- the hand keeps
its physical speed -- and the angular velocity drops on its own. Had the state been angular, the FOV change
would have been a velocity discontinuity needing its own special case.

### The control law

Per axis, time-optimal with a saturating acceleration:

    step     = aMax * dt
    envelope = sqrt(2 * aMax * |e| + step^2 / 4) - step / 2
    v_target = sign(e) * min( |e| / dt, envelope, kMaxHandSpeed )
    a        = clamp( (v_target - v) / dt, +/-aMax )
    v += a * dt ;  angle += v * s * dt

The envelope term is the whole model. It is the velocity of a body decelerating at `aMax` that arrives at
the target with zero velocity, so following it gives the accelerate-then-decelerate profile without any
explicit phase switching: far from target the required acceleration exceeds `aMax` and the clamp saturates,
near it the envelope takes over and brakes.

### Why the envelope carries a step correction

The obvious form is `sqrt(2 * aMax * |e|)`, the continuous-time optimum. Sampled at a finite frame rate it
overshoots badly, because it assumes braking begins the instant the envelope is touched when in fact it
begins up to a full frame later. Measured at 2880 deg/s^2, the plain form overshoots a 180 by 3.6 degrees at
100fps, 5.9 at 60fps and **12.4 at 30fps** -- and the framerate dependence is the real problem, because it
makes aim accuracy a function of server load. At the shipped 4500 it is worse still, since the error scales
with `aMax * dt`.

The `+ step^2 / 4` under the root and the `- step / 2` outside it invert the *discrete* stopping distance
rather than the continuous one. That drops overshoot below 0.7 degrees at every frame rate tested, and it
converges *faster* despite braking earlier (0.617s to 0.467s for a 180 at 60fps, at 2880), because none of the budget
is spent recovering from an overshoot. This matters beyond tidiness: deliberate over/undershoot is a planned
feature, and it has to be the only source of overshoot or it cannot be tuned.

The `|e| / dt` term is a dead-beat clamp. Capping the target velocity at "the speed that lands exactly this
frame" makes any error small enough to close in one step close in one step.

It is worth being precise about what it buys, because the obvious way to make the bot less surgical is to
delete it and that turns out not to work. The "bang-bang limit-cycles at roughly `aMax * dt^2`" claim
applies to the **plain** `sqrt(2 a |e|)` envelope. With the discrete correction above, the envelope
linearises near zero to `2e/dt` -- exactly twice the dead-beat speed -- so a step lands on `-e` and no
further. Squaring out `sqrt(2ae + s^2/4) - s/2 <= 2e/dt` with `s = a*dt` reduces to `0 <= 4e^2/dt^2`, which
holds for every `e > 0`, with equality only in the limit. So there is no limit cycle at all: dropping the
clamp gives a ring-down that decays quickly at first and stalls around `aMax * dt^2 / 8` -- 0.056 degrees at
100fps, 0.156 at 60, 1.41 at the `kMaxFrameTime` clamp. That is an order of magnitude finer than the hand
quantum at normal tick rates and three times coarser at 10 Hz: wrong in both directions, and frame-rate
coupled in a controller whose entire envelope design exists to not be. Coarseness comes from the quantum
below instead, and the clamp stays.

`kMaxHandSpeed` is a hand speed ceiling. It is near-inert at the current constants, since the largest
reachable error is 180 degrees and that only peaks at 720 deg/s, but it is the knob for a bot whose hand
should be slower than its acceleration allows.

### The hand has a resolution, and that is what zoom buys

A real hand on a real mouse cannot move an arbitrarily small amount, and cannot change speed by an
arbitrarily small amount. Two quantisations model that, both in hand space, both applied at the bottom of
`StepAxis`:

    velOut   = round(vel / impulse) * impulse       // commanded speed lives on a grid
    pending += velOut * dt
    counts   = trunc(pending / quantum)             // whole mouse counts only
    pending -= counts * quantum
    angle   += counts * quantum * s

`crossbot_aim_quantum` is degrees of hand travel per mouse count and `crossbot_aim_impulse` is the smallest
non-zero hand speed, in hand deg/s. Both cvars hold the value itself rather than a multiplier, and either at
0 switches its own quantisation off; both at 0 is the exact pre-quantum controller, which is what the
regression run in `test_aim_error.ps1` uses.

The default quantum is `atan(1/128)` = 0.4476 degrees: one unit of miss per 128 units of range. That is
deliberately chosen for its consequence at distance. Because both quantisations are in hand space, `s`
scales them the same way it scales `kMaxAccelYaw`:

| fov | angular quantum | 1 unit per | miss at 4096 units |
|---|---|---|---|
| 90 | 0.448 deg | 128u | +/- 32u, and a player is 32 wide |
| 20 (crossbow) | 0.0995 deg | 576u | +/- 7u |

**This is the reason zoom is worth anything to a bot.** Before it existed, `FovScale` priced zoom as pure
cost -- see "Zoom is a range decision" below -- and a bot that zoomed simply turned 4.5x slower for a mode
swap. Now the same 4.5x buys 4.5x finer placement. Measured on a 40 degree flick, ten samples each: mean
resting error 0.299 degrees at fov 90 against 0.052 at fov 20, for settle times of 0.180s and 0.393s. Peak
hand speed came out 414 and 891 deg/s -- the fov 20 figure matching the 890 of a 180 at fov 90, because a 40
degree turn through a 20 degree scope is the same 180 degrees of hand travel.

**Why the speed grid is applied to the output and not to the change in speed.** Quantising `dv` and keeping
`vel` itself on the grid is the tidier-looking form and it deadlocks. `dv` is clamped to `maxAccel * dt`
first, so as soon as `maxAccel * dt` drops below half an impulse every `dv` rounds to zero and the velocity
can never change again -- the view freezes solid. That threshold is `dt < impulse / (2 * maxAccel)`, i.e.
600fps on yaw and 480 on pitch at the defaults, and `sys_ticrate` goes to 1000. Leaving `vel` continuous and
snapping only the value that gets integrated has no such threshold.

**Why the commanded speed is floored and not merely snapped.** `round(vel / impulse)` returns zero whenever
`|vel| < impulse / 2`, and the dead-beat term turns a small error into a small velocity -- so without a
floor any error under `impulse * dt / 2` is invisible and the hand parks early. Measured, that put the
resting error at 0.065 degrees at 100fps: three times finer than the quantum intends and frame-rate coupled,
which is both of the things this design is trying to avoid. So the control law floors `limit` at `impulse`,
capped at `quantum / dt` so the floor can never demand more than one count per frame at a low tick rate.
The hand therefore never comes fully to rest: it dithers across the count it is nearest, roughly 20 Hz at
100fps, which is about where physiological hand tremor sits. `crossbot_aim_status` shows this as a hand
speed that idles at 21 deg/s rather than 0 -- `sqrt(15^2 + 15^2)`, both axes at the floor.

The tremor is not cosmetic. It is what makes long range *probabilistic*: `RayOnTarget` gates the trigger on
the ray actually entering the bounding box, so a wandering ray produces intermittent potshots at a distant
target rather than a clean yes or no that holds for the whole engagement.

**`kMaxQuantum` and `kMaxImpulse` are cvar clamp bounds, not physics.** They belong with `kMaxDrift`,
`kMaxDelay` and `kMaxErrorScale`, not with `kMaxAccelYaw` in the same block: acceleration keeps its ceiling
and the impulse adds a floor, opposite ends of one axis and both needed. `kMaxQuantum` is 5 degrees because
that is under `kLadderAimTolerance`, so no cvar value can leave a nav gate unlatchable; `kMaxImpulse` is 100
deg/s because past that the slowest possible tracking speed outruns most engagements.

Resting error is bounded by whichever quantisation binds, `max(quantum, impulse * dt) / 2` in hand space:
0.22 degrees at 100fps, 0.375 at the 20 Hz clamp. Both sit under `kTestSettleTolerance`, and the tremor
speed sits far under `kTestSettleSpeed`, so `crossbot_aim_test` still reports a settle. The nav gates have
more than 10x margin, and `crossbot_aim_drift` defaults to a degree of sine, comfortably above the quantum,
so idle drift still moves the view -- in steps now.

### Where the constants come from

`kMaxAccelYaw` is derived, not tuned. For a symmetric accelerate-then-decelerate turn covering angle `t` in
time `T`, `t = a * T^2 / 4`. Requiring a 180 in 0.4s gives `a = 4 * 180 / 0.16 = 4500 deg/s^2`, peaking at
900 deg/s halfway through. `kMaxAccelPitch` is 80% of that: vertical mouse travel is the awkward direction
for a human wrist. For reference, the CS zbot uses a hand-tuned 3000 deg/s^2 and peaks around 750 deg/s on
the same turn, so this bot's hand is somewhat quicker than a zbot's.

The first cut used 0.5s (2880 deg/s^2). It was raised to 0.4s because 0.5s measurably slowed
`test10_doorbutton` -- see below. `kMaxHandSpeed` is deliberately above the 900 deg/s peak so it stays a
ceiling rather than a clip; at exactly 900 it would bind at the top of every 180.

### The cost at the aim gates

Below about 80 degrees of error (the crossover, `4 * 300^2 / aMax`) the controller is *slower* than the old
flat 300 deg/s rate, because it spends the start of the turn accelerating. A 6 degree correction goes from
20ms to 73ms. The places that care are the two that **zero drive until the view has converged**: the ladder
gate (`kLadderAimTolerance`, 6 degrees) and the activation gate (`kActivationAimTolerance`, 8 degrees).

`test10_doorbutton` is the map that feels it. Isolated repeat runs, goal-arrival time:

| controller | 8.1s | 11.1s |
|---|---|---|
| old flat 300 deg/s | 8 of 8 | 0 |
| aMax 2880 (180 in 0.5s) | 0 | 2 of 2 |
| aMax 4500 (180 in 0.4s) | ~7 of 16 | ~9 of 16 |

**Read that table with the quantization in mind.** `test_nav_goal.ps1` polls for arrival on a grid -- the
observed times are 2.0, 5.0, 8.1, 11.1 -- so consecutive buckets are ~3s apart and a run that misses the
8.1s poll by a tenth of a second reports 11.1s. The real slowdown is well under a second; the table measures
which side of a threshold the bot lands on, not how much time it lost. Raising the acceleration moved it
back across that threshold about half the time.

Two things that look like they explain this and do **not**: every run on this map, on *every* controller
including the original, logs exactly one `mid-route repath (stalled)` at the route reversal in front of the
button and exactly one `avoid side func_door_rotating`. Neither is caused by the aim model, and neither
discriminates the fast runs from the slow ones.

For calibration when reading sweep numbers: `test45_slopes2` looked like a 5.2s-to-8.1s regression in the
parallel sweep and was not one -- the old controller measures 8.1s in isolation too, and the 5.2s came from
a run at `-MaxParallel 6`. The general rule for this suite: a single parallel-sweep number proves nothing,
and neither does a two-run isolated sample when the metric is bucketed. Repeat until the distribution is
visible.

### dt is clamped

The old first-order ramp could survive a raw `gpGlobals->frametime`: the worst a hitch did was let it snap
further than intended, bounded by the remaining error. A second-order integrator has memory, so a hitch
feeds a huge `dt` into both the acceleration and the position step and flings the view. `dt` is clamped to
[1/1000, 1/20], the same defensive range `Bot_Physics` uses.

### Pitch clamp and velocity reset

Pitch is clamped to +/-89 to avoid gimbal lock, and `pitchVel` is zeroed when it hits the stop -- otherwise
velocity winds up against the limit and the view stays pinned for several frames after the target comes
back into range. `TeleportAllBots` zeroes both velocities alongside the `v_angle` reset for the same reason.
Everything else is covered by `bot = CBot()`, since `BotAim::State` is default-initialized.

### crossbot_aim_test

`crossbot_aim_test <bot> <yawDelta> [pitchDelta]` commands a flick relative to the bot's current view and
prints the time to settle within 0.5 degrees plus the peak hand speed. It exists because there is otherwise
no way to check the 180-in-0.5s claim against the running server.

The test angles are an **override**, not a write to `desiredAngles`: navigation rewrites `desiredAngles`
every frame, so a plain write would be stomped before the next frame. The override releases itself after 3
seconds and the bot goes back to whatever it was looking at.

Expected, from an offline simulation of the same step function at the shipped constants: `180 0` settles to
within 0.5 degrees in 0.37-0.38s at a peak of 900 deg/s hand speed, and `0 90` in about 0.30s at a peak near
540-560. The spread is the frame rate -- lower fps settles slower. Peak hand speed is the more stable number
and the better thing to check: it is set by the constants, not the sampling, and 900 for a 180 is the
derivation above closing on itself.

### The reaction delay queue

Everything above is the motor half: how fast the hand can move once it decides to. The queue is the
perception half: the bot acts on what it saw `crossbot_aim_delay` seconds ago, never on the present frame.
Default 0.25s, clamped to [0, 0.5]. Human simple visual reaction is roughly 200-250ms; 0.5s is
choice-reaction territory and reads as sluggish, so the clamp is a usable range rather than a safety rail.

`BotAim::State` carries a 256-slot ring of `{ time, point }`. 256 rather than 128 because 128 slots at
250fps only spans 0.512s, right on the boundary of the maximum delay.

**Slots hold a world point, not angles**, and the write extrapolates only the *target*:

    push { now, targetPoint + targetVelocity * delay }

The push is conditional: that predicted point is traced first, and a lead that has already crossed into
geometry is dropped rather than queued. See `### The lead point is traced before it is queued` under
`## bot_sense.cpp`.

The read takes the newest slot older than `now - delay`, linearly interpolates it with the next newer one
so the served delay is exact regardless of frame rate rather than quantized to a frame, and only then
converts to angles -- from the bot's **actual current eye**, in `ApplyLook`.

The extrapolation is the entire point. At time `T` the bot records where the target will be at `T + delay`.
At `T + delay` it reads that back. If the target moved at constant velocity over the window the prediction
was exact, and the bot is aimed exactly at it despite acting on quarter-second-old information. Only
*acceleration* -- a target changing direction -- produces error. That is the human signature being modelled:
a good player tracks steady movement perfectly and gets beaten by the change of direction, not by the speed.

The bot's own motion is deliberately **not** extrapolated, and neither is its aiming velocity. Where the bot
is standing is known exactly at read time, so predicting it would replace a fact with a guess; and turning
the view does not move the eye, so it never enters the geometry at all.

### The two queue edges are the behaviour, not error handling

**Underflow.** When a target has just been acquired the ring holds nothing older than `delay`, so the read
fails and the queue writes nothing -- the bot keeps whatever `desiredAngles` the navigation branch set. It
therefore sits perfectly still for a full reaction time and *then* flicks. The obvious alternative, falling
back to the oldest slot, would snap the head to truth the instant a target appears, which is exactly zero
reaction time. The silent quarter second before anything moves is the most human artifact the whole system
produces, and it costs nothing.

**Drain.** On losing line of sight the bot stops *writing* but keeps *reading*. It therefore continues
tracking a target that has already left view, for exactly `delay`, and then stops -- because its input is
`delay` stale and it does not know yet. Also free: it falls out of the ring rather than being coded.

The ring is cleared on acquiring a target, on teleport, and on respawn (the last via `bot = CBot()`).
Clearing on acquire is what makes the reaction time apply to every reacquisition, not just the first.

**The drain did not actually work until firing needed it.** `UpdateVision` compared the new target
against the old and cleared on any difference, and *losing* a target is a difference — so the ring was
wiped on the same frame line of sight broke and there was no drain at all, only the suspicion fallback
in `ApplyLook`. The test is now `best && best != sense.target.Get()`: clear when a different live
target is acquired, never when one is lost. Reacquiring the same enemy after a gap still clears,
because the handle was set to null in between, so the reaction delay is still paid every time.
Nothing above this paragraph changed — the behaviour it described is simply true now.

### What feeds the queue

`BotSense::Update` does, once per frame, from the slot in `Bot_MoveNav` that `UpdateGaze` used to hold.
See `## bot_sense.cpp` below for the whole producer. The contract the queue cares about is one
`PushDelaySlot` per frame while a target is held *and its lead point comes back clear*, the target
extrapolated forward by `crossbot_aim_delay`, and `ClearDelayQueue` on acquiring a *different* target.

The one thing that did change is what point gets pushed. It is no longer the target's eye but a randomly
chosen hitbox bone, re-picked every frame. The queue therefore stores points that jitter over the enemy's
body, and the acceleration-limited controller in `Bot_UpdateFacing` is what averages them back down -- it
is already a low-pass filter, so no explicit smoothing was added. The visible result is a head that settles
near body centre while the underlying samples wander, which is both what a human does and what makes a
partially-hidden enemy still trackable.

`ClearDelayQueue` keys on the target *entity*, not the bone. Keying it on the sample point would clear the
ring every frame and destroy the reaction delay entirely.

### Slots also hold velocity

Beside the point, and interpolated the same way. The reader is `BotFire`, which needs the target's
velocity to lead a projectile, and needs it as **perceived** rather than as it is right now — a lead
computed from live velocity would give a bot with a quarter-second reaction time perfect prediction.
`ReadDelayed` takes it as an optional out pointer, so `ApplyLook`, which only wants the point, is
unchanged. See `## bot_fire.cpp` for what the composition means.

### Why slots hold points and not angles

Slots held `{ time, yaw, pitch }` first, which forced the write to guess where the *bot itself* would be one
reaction time later and bake that guess into the stored angles. The guess only holds while the bot moves at
constant velocity. A bot that jumps, gets stopped by `BotAvoid`, rides an elevator or repaths inside the
delay window breaks it, and the recalled angles are then right about *where the target is* and wrong about
*where the bot is looking from*. That is parallax error, worst at close range where a small positional error
subtends a large angle -- which is exactly the range where aim decides fights. It is also unattributable: it
lands in `err` mixed together with genuine target misprediction, so neither number can be read cleanly.

Storing the point deletes that term rather than modelling it. The bot's eye enters once, at read time, as
its true present value, so there is nothing left to mispredict. Where the prediction would have been right
the two forms are identical; where it would have been wrong, only the angle form was wrong. The target-side
extrapolation -- the half that actually models human lead -- is untouched.

The cost is a `LosAngles` call moved from the write to the read, which is the same call either way, and a
three-float lerp in place of a two-float one that also had to take yaw the short way round. The one real
consequence is that `LosAngles` can now fail at *read* time, when the recalled point sits on the bot's own
eye. `ApplyLook` falls through to the suspicion blend there and leaves `servedDelay` at zero rather than
claiming a serve it did not make.

### crossbot_aim_status

Prints, per bot: the sense target, which bone it is currently sampling, ring occupancy, how much history
the ring holds, the delay actually served, hand speed, and `err` -- the angle between the bot's real
`v_angle` and the true present-time angles to its target.

`err` is the number that decides whether any of this works, and it is why the command exists at all.
Spectating cannot distinguish "correctly delayed" from "delayed and also wrong": both look like a head
lagging behind a target. `err` separates them. At `crossbot_aim_delay 0` it is the controller's own tracking
error with no queue involved, which is the baseline. At 0.25 it should still be flat whenever the
target is moving at constant velocity, and spike only on direction reversals. A large `err` under steady
target motion means the extrapolation or the read index is wrong, not that the delay is working.

**`err` no longer bottoms out at zero.** `AnglesToTarget` measures against the target's eye, while the bot
is aiming at a randomly chosen hitbox bone. The difference between those two points is a standing floor,
and it is a floor that *grows as the range closes* -- the same bone spread subtends a wider angle up close,
and every bone sits below the eye, so the residual is almost pure negative pitch. At the point-blank range
`test_sense.ps1` converges the bots to, it is tens of degrees. That floor is the bone sampling working, not
a defect, and it is why `err` is no longer a number to read absolutely. Read the shape across the delay
sweep instead: the thing that would still be a real extrapolation bug is `err` *trending* with
`crossbot_aim_delay` at a fixed range.

`scripts/test_sense.ps1` drives exactly this: two bots on `crossfire`, sense on, sweeping the delay while
polling the command. The nav sweep cannot substitute for it -- `test_nav_goal.ps1` spawns a single bot, and
with line-of-sight gating a lone bot never has a target, so the sweep can only ever prove the feature is
inert when switched off.

Measured on `crossfire`, both bots converged on one spawn and idling, drift off:

| `crossbot_aim_delay` | served | ring |
|---|---|---|
| 0.000 | 0.000 | fills to 256/256 |
| 0.250 | 0.250 | fills to 256/256 |
| 0.500 | 0.500 | fills to 256/256 |

`served` tracking the request exactly is the interpolated read working: without the lerp it would quantize
to whole frames and drift with the frame rate. A `served` of 0.000 at a nonzero delay is not a defect
either -- it is the underflow silence, caught in the second or so after a re-acquire while the ring refills.

`err` is deliberately absent from that table. The script converges the bots to hold line of sight, and that
is also where the hitbox-bone spread subtends its widest angle: every bone sits below the eye that
`AnglesToTarget` measures against, so err runs to tens of degrees and is almost pure negative pitch. That is
geometry, not tracking error.

**What the static phase does not prove.** Both bots are *stationary*, so `velocity * delay` is zero and the
extrapolation is never exercised -- it proves the ring, the read index and the interpolation, not the
prediction. Constant-velocity tracking has to be checked by hand, by joining and strafing at a steady speed;
see `doc/testing.md`. Roaming bots are not a substitute: `crossfire` has no stretch long enough to hold a
constant velocity, and the roam phase mixes genuine target acceleration with the gate suppression pointing
the bot at a ladder. err there has been seen anywhere between 2 and 48 degrees across runs, with no way to
attribute any of it -- it is not a number to compare against a previous run.

### The brake multiplier, and why b<1 is not really an undershoot

The controller in `StepAxis` is time-optimal, so it arrives with zero velocity every time. That is the one
thing a hand never does. Phase 3 makes it miss on purpose with a single number: `brake`, multiplying the
deceleration envelope and nothing else.

```
envelope = (sqrt(2*a*m + 0.25*s^2) - 0.5*s) * brake
```

The dead-beat `m/dt` term is deliberately left alone. It is what closes an error small enough to close this
frame, and multiplying it too would make tiny errors miss forever instead of terminating.

`brake > 1` lets the axis carry more speed than it can shed, so it arrives moving and sails past. The
overshoot is exact: deceleration starts at remaining `v^2/(2*a*b^2)` but stopping needs `v^2/(2*a)`, so it
overshoots by `v^2/(2*a) * (1 - 1/b^2)`. For a large flick the peak is `v^2/(2*a) ~ E/2`, which makes the
overshoot a fixed *fraction* of the movement rather than a fixed angle -- constant relative error, which is
the Fitts' law signature, and it falls out of the algebra rather than being imposed.

`brake < 1` is the asymmetric case and the name is a slight lie. Decelerating early does not leave the axis
short of the target: solving `dm/dt = -b*sqrt(2*a*m)` gives `sqrt(m)` falling linearly to zero in
`sqrt(2*E/a)/b`, so it still converges, just `1/b` times slower. What you see is a soft landing, not a
discrete undershoot and correction. A real undershoot needs the *aim point* to be wrong, not the braking,
and that is an offset model rather than this one. The soft landing is worth having anyway, so it stays.

### Sampled once per flick, not driven by a sine

The original sketch modulated the error with a sine. That was dropped: anything periodic reads as robotic
once you watch it for a minute, which is the exact failure the model exists to avoid.

`UpdateBrake` resamples both axes when the combined error magnitude is above `kFlickMinError` (3 degrees)
*and* grew by more than `kFlickGrowth` (0.5) since last frame. During a converging movement the error falls
monotonically, so nothing retriggers. Two things make it grow: a genuinely new target, and the bot's own
overshoot flipping the error sign and pushing it back up through the threshold. So the correction movement
gets its own independent brake for free, and the visible result is one ballistic movement followed by one or
two smaller ones -- again, not imposed anywhere.

The chain terminates on its own. At 60fps the dead-beat term dominates below `2*a*b^2*dt^2` ~ 2.5 degrees,
so a correction smaller than that closes in a single frame no matter what brake was drawn, and there is no
limit cycle at the end of a flick.

Both axes are sampled independently on the same trigger. That is deliberate: a shared brake would make every
miss land exactly along the movement's own line, and humans miss to the side too.

`crossbot_aim_status` prints the live `brake` pair, and **a frozen value there is the correct reading, not a
stuck one.** A bot idling on a stationary target never lets the error grow past 3 degrees, so it never
resamples and holds whatever it drew on the last real flick — which is exactly what the static phase of
`test_sense.ps1` shows. Watch it change during the roam phase instead.

### The RNG is per bot and seeded from the entity index

`RandomUnit` is a plain LCG on `BotAim::State`, not `RANDOM_FLOAT`. Two reasons. It does not consume the
shared engine RNG, which the game uses for weapon spread. More importantly it is seeded from
`pfnIndexOfEdict` alone, so a given bot draws the same sequence on every run of a map -- the nav sweep stays
reproducible, which matters more here than usual, because the measurement discipline this work needed
(`doc/testing.md`) already has enough sources of run-to-run noise without aim error adding another.

`kBrakeSpread` is 0.06, scaled by `crossbot_aim_error`. Sampling is uniform and symmetric about 1. Motor
control literature says the primary submovement usually undershoots rather than overshooting, so biasing the
sample low is the obvious next tuning knob; it is not done because there is no measurement here that could
justify a particular bias.

### Idle micro-drift

A bot holding still with a settled view is visibly a statue. `Bot_UpdateFacing` adds a slow sine to the
*local* copy of the desired angles -- `crossbot_aim_drift` degrees of yaw at 1.7 rad/s, 60% of that in pitch
at 1.19 rad/s, per-bot phases drawn at seed time. The two rates are incommensurate, so the pattern does not
visibly repeat.

It is added to a local copy, not to `bot.desiredAngles`, so `RunActivation` and `RunChargerUse` keep
comparing the view against the same angles they wrote and their gates are untouched. The ladder gate at
`kLadderAimTolerance` compares against its own locally computed `aimYaw`, so drift does show up there as
steady-state error -- 1 degree of it, against a 6 degree tolerance.

**Drift is suppressed while `testActive`.** This is not optional. `crossbot_aim_test` settles on a 0.5 degree
tolerance and the drift amplitude is 1 degree, so with drift running the test could never settle at all.

### The settle gate had to learn about overshoot

`UpdateTest` previously reported settle on the first frame inside `kTestSettleTolerance`. With overshoot in
play, a view flying through the target at 200+ deg/s on its way past can land inside that band for one frame
and report a settle that never happened. It now also requires hand speed below `kTestSettleSpeed` (150
deg/s), which is above the ~50 deg/s the dead-beat term produces on genuine arrival at typical frame rates
and well below flythrough speed.

This is a threshold, not a proof: an overshoot small enough to cross the target slowly can still slip under
it. Read the spread across many flicks rather than any single number, which is what
`scripts/test_aim_error.ps1` exists to produce.

### What the error actually measures out at

45/45 on the nav sweep at the shipped defaults (`crossbot_aim_error 1`, `crossbot_aim_drift 1`), so the
gate-latch cost the phase 1 controller introduced is not compounded by the error model. Eight flicks per
cell, `crossfire`, one bot:

| `crossbot_aim_error` | flick | settle min | avg | max | spread | peak |
|---|---|---|---|---|---|---|
| 0 | 180 0 | 0.308 | 0.349 | 0.372 | 0.064s | 895 deg/s |
| 0 | 0 90 | 0.255 | 0.266 | 0.297 | 0.042s | 563 deg/s |
| 1 | 180 0 | 0.296 | 0.398 | 0.543 | 0.247s | 908 deg/s |
| 1 | 0 90 | 0.245 | 0.285 | 0.354 | 0.109s | 557 deg/s |

The `crossbot_aim_error 0` rows are the regression check: brake forced to exactly 1, and the numbers land on
the phase 1 measurements (0.37s at 899 deg/s for a 180, 0.28s at 557 for 90 of pitch). The residual 0.06s
spread there is frame-phase jitter, not the model.

At scale 1 the distribution opens up and, importantly, opens up *asymmetrically*: the mean moves up 0.05s
but the max moves up 0.17s. That long tail is the overshoot-and-correct case, and it is the shape that was
wanted. The minimum going slightly *below* the scale-0 minimum is not an anomaly -- a small `brake > 1`
carries more speed in and lands with an overshoot under the dead-beat threshold, so it arrives faster than
time-optimal-with-a-full-stop does.

## bot_sense.cpp

What the bot knows about the world and how it found out. Three inputs -- vision, heard sounds and playback
events, and damage taken -- feed one per-bot ring of suspicions, and two outputs come back out: an aim
target pushed into the delay queue, and a look direction for when there is no target.

`crossbot_sense` gates the whole thing. `crossbot_sense_decay` (default 5s) is the only other knob; the
view cone (90 degrees) and vision range (4096) are constants, because they are geometry rather than policy.

### The engine hooks were dead code

`GetEngineFunctions` and `GetEngineFunctions_Post` had existed in `engine_hooks.cpp` since the beginning,
correctly written, and Metamod had never once called them: `meta_api.cpp` left slots 7 and 8 of
`META_FUNCTIONS` at `NULL`. Filling a table entry alone does nothing. If a sound hook ever appears to be
silent, check those two slots first.

Both tables are now `memset` plus assignment by member name. They used to be ~160 bare `NULL,` entries
each, positional against a 158-member struct; `pfnEmitSound` is member 29 and `pfnPlaybackEvent` is member
123, and counting to those by hand is a defect waiting to happen.

Both hooks are in the **post** table and return `MRES_IGNORED`. Post for the same reason item detection is
post: a pre hook fires even when another plugin returns `MRES_SUPERCEDE` and the sound never actually
plays, which would record something that did not happen.

The hooks fan out through `BotManager::OnSound` / `OnEvent`, which loop `g_bots` inline. The work per bot
is a distance compare and a ring push, so there is no deferred sound queue -- unlike foxbot, whose
`BotSoundSense` does waypoint route-distance queries inside the engine callback.

### Sounds carry no position, and the origin is not v.origin

`pfnEmitSound` passes an entity and nothing else spatial. The engine derives the position as the emitter's
bbox centre, so `EntitySoundOrigin` does the same: `origin + (mins + maxs) * 0.5`. For brush entities --
doors, lifts, breakables -- `v.origin` is `(0,0,0)` and using it would put every door sound at the map
origin.

Audible radius follows the engine's own spatialization, which is inaudible past a 1000-unit nominal clip
distance scaled by attenuation:

    attenuation <= 0  ->  audible everywhere (ATTN_NONE)
    otherwise         ->  radius = 1000 * volume / attenuation

`ATTN_NORM` (0.8) at full volume is about 1250 units, `ATTN_STATIC` (1.25) is 800, `ATTN_IDLE` (2.0) is
500. GoldSrc does not occlude sound, so a plain distance test is faithful and no trace is needed.

`sample` is never inspected. Two filters replace it, both chosen off a dump of what actually arrives on
`crossfire`: drop `volume <= 0`, and drop `common/null.wav`. Between them those catch every stop-sound and
silence command, which were a quarter of all traffic. **`CHAN_STATIC` is deliberately kept** even though it
is nominally the looping-ambient channel -- in practice HL brush entities emit door and lift sounds on it,
and those are among the few genuinely interesting world sounds on a deathmatch map.

### Footsteps arrive through pmove, not through pfnEmitSound

Hooking `pfnEmitSound` sees zero footsteps, and the reason is not that footsteps are client-side. They are
emitted server-side like anything else -- they just take a different road into the engine.

There are two entry points into `SV_StartSound`. `pfnEmitSound` is one of them, and it is the only one the
`enginefuncs_t` table can reach. The other is `PM_SV_PlaySound`, which the engine hangs off
`pmove->PM_PlaySound` and pmove calls directly. Everything pmove emits -- footsteps, ladder steps, water
wade and slosh, jump landings, fall grunts -- goes down that second road and is invisible to the
enginefuncs hook.

`PM_PlayStepSound` does have a `!pmove->runfuncs` early-out, but it never trips on a server: `SV_SetupMove`
sets `runfuncs = TRUE` unconditionally, immediately before calling `pfnPM_Move`.

So the hook goes on the callback itself. `pfnPM_Move` is hooked both pre and post: pre saves
`ppmove->PM_PlaySound` and swaps in a trampoline, post puts the original back. The trampoline calls the
original first and then feeds `BotManager::OnSound` with the moving player's edict, so footsteps land in
the suspicion list as ordinary `SUSP_SOUND` entries and every existing filter and the attenuation-radius
test apply unchanged.

Three things make this the only workable shape:

- **The engine reassigns `PM_PlaySound` on every `SV_RunCmd`**, so there is no such thing as a persistent
  install. Inside `pfnPM_Move` is the only place the swap can live. (jk_botti installs "permanently" and
  restores at unload; because of the reassignment its pre-hook is in fact re-installing every single call,
  which is the same thing written less obviously.)
- **The identity guard `ppmove->PM_PlaySound != PM_PlaySoundHook` is load-bearing.** Without it a double
  install saves the trampoline as its own original and the first footstep recurses until the stack dies.
- **`ppmove->player_index` is 0-based** -- the edict index is `+ 1`. Getting this wrong misattributes every
  footstep to the neighbouring player, which looks entirely plausible and is entirely wrong.

The install is skipped when `crossbot_sense` is 0, so that path stays free; the post restore is guarded on
the identity check instead, so flipping the cvar mid-frame cannot strand the pointer.

`pm_shared` never calls `PM_PlaybackEventFull`, so there is no matching blind spot on the event side --
`pfnPlaybackEvent` really is the only road for events.

### What footsteps are worth, without a single tuning constant

The volume pmove passes is already graded, and the existing `1000 * volume / attenuation` radius turns it
straight into a hearing distance. Everything below is read out of `pm_shared.c` and confirmed against a
live dump of 85 step sounds from two bots on `crossfire`:

| Sound | Volume | Audible at (ATTN_NORM) |
|---|---|---|
| Running step, concrete/metal/tile/grate/slosh | 0.50 | ~625 |
| Running step, dirt | 0.55 | ~690 |
| Running step, vent | 0.70 | ~875 |
| Ladder step | 0.35 | ~440 |
| Wading into water | 0.65-1.0 | ~810-1250 |
| Jump launch, and landing by fall speed | 0.5 / 0.85 / 1.0 | ~625-1250 |
| Fall-pain grunt (`CHAN_VOICE`, hard landing only) | 1.0 | ~1250 |

Two engine rules come along for free and are worth knowing because they are behaviour, not detail:

- **Walking is silent in multiplayer.** Any step with horizontal speed `<= 220` is dropped outright. This
  is the real rule that foxbot approximates by hand with a velocity threshold, having had no way to reach
  it. It also means a bot that wants to move quietly already has a mechanism -- move slower -- and nothing
  in `bot_move_nav.cpp` uses it yet.
- **`sv_footsteps 0` disables the whole sense** from outside the plugin, since the gate is inside
  `PM_PlayStepSound`. If footsteps ever appear to have stopped working, check that first.

A bot does not hear its own steps: `OnSound` already skips the emitting entity. Note that this means the
nav sweep, which spawns a single bot, is completely unaffected by any of this.

### Events come from pInvoker, never from the origin argument

`pfnPlaybackEvent`'s `origin` is `(0,0,0)` for essentially all weapon fire -- the engine fills it in from
`pInvoker->v.origin` when `FEVENT_ORIGIN` is unset. Reading the argument would put every gunshot at the map
origin.

Filters: `pInvoker` must be a live player, and `FEV_HOSTONLY` is dropped as per-client viewmodel cosmetics.
`FEV_CLIENT` needs no filter, as those never reach the server. There is no radius test -- a gunshot in HL
is `LOUD_GUN_VOLUME` class and is meant to carry.

There is deliberately no `pfnPrecacheEvent` index-to-name table. jk_botti keeps one for per-weapon
loudness; a player-invoked non-`FEV_HOSTONLY` event is essentially always weapon fire in HLDM, and the
table is per-map bookkeeping for a refinement nothing uses yet.

### Damage must be read before Bot_RunPlayerMove

The trigger is `pev->dmg_take != 0`, and the position is `EntitySoundOrigin(pev->dmg_inflictor)`.

The ordering is the whole trick. `dmg_take` is zeroed by `CBasePlayer::UpdateClientData`, which runs inside
`PlayerPostThink` -- and for a fakeclient the only thing that runs `PlayerPostThink` is the bot's own
`pfnRunPlayerMove`, the last action of its think. So the field accumulates over exactly the window from the
previous frame's `RunPlayerMove` to this frame's `BotSense::Update`, and other players' bullets always land
inside that window because they are fired during *their* think. Move the read after `Bot_RunPlayerMove` and
it reads zero essentially always.

`dmg_inflictor` is never cleared by anything, so on its own it answers "who hit me" and not "was I hit". It
is only meaningful read in the same breath as a non-zero `dmg_take`.

The blind spot is damage taken during the bot's own pmove -- fall damage, walking into a `trigger_hurt` --
which `UpdateClientData` clears before the next update sees it. Those are self- and world-inflicted and the
inflictor filter discards them anyway.

### Vision samples a hitbox bone, not the eye

Per player, in this order: range, then a 3D cone test, then a bone position, then the trace. The order is
cost -- `pfnGetBonePosition` runs a full `SV_StudioSetupBones` on every single call, one skeleton setup per
bone, so it goes last and only for candidates that already passed the cheap tests.

The cone is a genuine 3D dot against `v_forward`, not foxbot's yaw-only `FInViewCone`, which lets a bot see
targets directly above it. Half-angle is `fov / 2`, so a symmetric cone against the *horizontal* FOV --
slightly generous vertically, since HL's vertical FOV at 4:3 is nearer 74 degrees. Accepted rather than
building a real frustum.

The trace is `dont_ignore_monsters`, and that is mandatory, not stylistic: with `ignore_monsters` the trace
passes straight through players and `tr.pHit == ent` can never be true.

The sample point is a bone drawn at random from the set of bones that carry a hitbox. Sampling all of
`0..numbones` instead would include helper bones -- the `Bip01` root sits at the feet, and weapon and blend
bones sit off the body -- which drag the averaged aim point down and make visibility flicker on floor
geometry. Every bone that owns a hitbox is a point that can actually be shot.

Two engine caveats. `pfnGetBonePosition`'s `rgflAngles` out-parameter is **never written**; it is passed
`nullptr` here and only the origin is ever read. That is also why hitbox *centres* are not used, since
transforming `bbmin`/`bbmax` into world space would need the bone's rotation. And stock GoldSrc only
bounds-checks `iBone` under `#ifdef REHLDS_FIXES`, so an out-of-range index is an out-of-bounds read on
vanilla HLDS -- the bone set is built from `numhitboxes` and every index checked against `numbones`.

### The lead point is traced before it is queued

The queue write pushes `bestPoint + targetVelocity * delay` -- where the target is predicted to be one
reaction time from now. For a long time nothing checked that this predicted point was anywhere a bullet
could reach. At the default 0.25s and full run speed the lead runs to about 80 units, so it crosses into a
wall roughly a whole reaction time before the target does, and a bot leading someone into cover spent the
entire drain window aimed at brickwork.

So the lead is traced, from the bot's real current eye, and the slot is pushed only if it comes back clear.
A blocked lead pushes nothing at all.

**It gates the queue write and nothing else.** The per-candidate trace above still runs against the raw bone
point, so `SUSP_SIGHT` and target selection are untouched. Pointing that one trace at the lead instead --
the obvious version -- would let an occluded lead null out `best`, and when it returned the next frame
`best != sense.target.Get()` would fire `ClearDelayQueue` and wipe the whole ring. With 80-unit leads and a
hitbox bone redrawn at random every frame, that flickers at every doorway and railing, and each flicker
costs a full reaction delay of silence. Gating only the write also costs one trace per bot per frame rather
than one per candidate, because the write happens once, for the winner.

**The flags differ from the perception trace, deliberately.** This one is `ignore_monsters`; the one above
is `dont_ignore_monsters` and has to be, or `tr.pHit == ent` could never be true. The two ask different
questions. Perception asks *can I see this player*. The lead trace asks *is this predicted point inside or
behind world geometry* -- a teammate, or the target's own body, standing between the eye and a point 80
units past it must not suppress the lead, and under `dont_ignore_monsters` the target itself would be
`tr.pHit` most of the time. Brush entities still block, which is right: a closed door is a wall. It is the
same flag and the same question as the fire-side trace in `BotFire::Buttons`.

**The trace is skipped when the lead is the sample point**, which covers both a stationary target and
`crossbot_aim_delay 0`. That is not only an optimisation -- it is what makes `test_sense.ps1`'s static phase
provably unchanged, since two motionless bots produce no lead, which keeps that phase usable as a regression
check for everything else in the queue.

Nothing is pushed in place of a vetoed lead. The ring drains, and if the block outlasts `delay` the read
underflows and `ApplyLook` falls through to the suspicion blend, which points at the last `SUSP_SIGHT`
entry -- the target's real last-seen position, at the corner. `ReadDelayed` needs no help with the gap: it
interpolates on time, so a hole in the write history is just a longer span, and the served point slides
across it smoothly when writes resume rather than jumping.

Both `crossbot_aim_status` and `crossbot_sense_status` report `leadblk`, a per-bot count of vetoes since
spawn. A count rather than a current-frame flag because a two-second poll would almost never catch the flag
set, and the thing worth verifying is that the veto fires at all.

### The bone cache and when it flushes

`g_boneSets` is a file-static array of 64 entries, map-scoped rather than per-bot. 64 rather than one per
player model so the same cache serves monster models if perception ever extends past players; the whole
table is under 3 KB.

The key is `v.modelindex`, not the header pointer. Modelindex is stable for the life of a map, and a player
changing model with `setinfo model` lands on a different modelindex and therefore a different slot.

It flushes:

- on `ServerActivate` -- the only flush actually required, since modelindex-to-model mapping is rebuilt per
  map and every entry is meaningless afterwards;
- on `ServerDeactivate`, so nothing survives a level change;
- per entry, opportunistically: each lookup compares the stored `studiohdr_t*` against the current
  `pfnGetModelPtr`, and rebuilds in place if they differ. GoldSrc's model cache can free and reload studio
  extradata under memory pressure, and this makes that self-healing rather than something to detect.

Never on a timer and never per bot -- the set of bones a model has does not change while it is loaded.

### One entry per entity, and why it stopped being a ring

The list used to append. Sound, event and damage entries each took a fresh slot, and only `SUSP_SIGHT` was
deduplicated per entity. The reasoning was that an enemy firing while moving leaves a trail of positions,
which is real information about their direction of travel -- and it is, but the price was that **the
trail outvoted the head**. A visible enemy is one entry; the same enemy running away lays down a footstep
entry every ~0.3s, so within the 5-second window they hold fifteen votes at fifteen different points and
the weighted mean is dragged backwards along the way they came. An elevator cycling in a shaft did the
same thing for free. Footsteps did not create this defect, but they made it constant.

Now every entity gets exactly one slot. `Push` looks for a live handle matching the emitter, overwrites it
in place if it finds one, appends if there is room, and otherwise reuses the entry with the smallest
`time`. Nothing can vote twice no matter how much noise it makes, and the direction the bot faces stops
being a function of emission rate.

The dedup key is a `CBotEntHandle`, not an entity index. That matters at exactly one moment: a breakable
is destroyed and something else is later allocated into the same edict slot. Under an index it would
inherit the old entry and its stale position; the handle's serial number makes `Is` return false across
that boundary, so the new entity gets a new slot and the orphan is unmatchable and ages out.

`head` is gone, and that deletion is forced rather than tidy. Every read used to walk backwards from
`head` and `break` at the first zero-weight entry, which was only ever valid because appending kept the
ring in time order. Overwriting a slot's `time` in place destroys that order, so a `break` would silently
truncate the list at the first stale entry -- the one failure mode here that looks like it works. Every
scan now runs `[0, count)` and uses `continue`. With one entry per entity the live set is a handful, so
scanning the array costs less than the ring ever did.

`count` is a high-water mark of distinct entities seen, not a queue depth. Eviction only begins after 256
distinct entities, and since decayed and orphaned entries are by definition the oldest, it reclaims those
long before it touches anything live.

### Where the bot looks when it has no target

Reading an entry goes through `LiveWeight`, and every scan in the file uses it -- tier selection,
accumulation, both refine passes, `LiveCount`, `Freshest`. One choke point is the design: a second
liveness test that disagreed with the first is how a tier gets selected and then contributes nothing.

`LiveWeight` drops an entry whose entity is gone, drops a player or monster that is dead, and otherwise
returns `max(decay - age, 0) / decay`. The handle check is what makes gibs and destroyed breakables stop
steering the bot; the dead check is what stops it staring at a corpse for the five seconds after a kill.

**The dead check is scoped to players and monsters on purpose.** `IsDead` is
`deadflag != DEAD_NO || health <= 0`, and a `func_door` or `func_train` has `health == 0`. Applying it to
`KIND_OTHER` would silently discard every piece of map machinery from the list and leave the fallback tier
permanently empty -- and nothing in the test suite would say so, because the player tier would still work.

Then the vote picks **one tier and ignores the rest**: players if any are live, else monsters, else
everything else. The enum order is the priority order so this is a `min` and not a switch. The point is
that a player entry decayed to weight 0.1 still beats a door at 1.0 -- the bot glances at the fading
contact instead of snapping to the lift. That is why the tier filter runs before `bestWeight` rather than
after it.

### Nothing is averaged: the winner is one entry, chosen by cone

Within the winning tier every live entry is scored

    score = LiveWeight(entry) * ConeWeight(vantage, dirToEntry)

and the **single highest scorer wins outright**. There is no vector sum, no mean-shift refinement and no
degenerate-mean fallback any more -- all three existed to make averaging survive threats on opposite
sides, and an argmax does not have that problem to solve. `authority` is still the winner's recency
weight alone, so the cone data decides *which* entry the bot looks at and never how hard it commits to
looking: a fresh contact from an unwatched direction still commands a full turn.

`ConeWeight` reads the bot's blended vantage signature (`nav_vantage.cpp`), which says which cones
everything with line of sight to this spot arrives through:

    rel  = watch[k].amount / watch[0].amount        // watch is amount-descending, so rel in (0,1]
    best = max over k of  rel * max(0, dot(dir, watchDir[k]))
    return kVantageConeFloor + (1 - kVantageConeFloor) * best

**Normalising against the strongest cone, not the sum, is what keeps the scale honest.** A dead-on hit on
the main cone scores 1.0 whether the spot has one cone or four. Dividing by the sum would cap a four-cone
spot at 0.25 and the floor would swamp every real hit on it.

`kVantageConeFloor = 0.15` is the whole reason this is a bias and not a blindfold. An entry the map says
nothing ever arrives from still keeps 15% of its recency weight, so a close, fresh threat from an
unwatched direction outscores a stale one down the main sightline and the bot turns. Without the floor a
bot could be structurally incapable of reacting to someone shooting it from a direction navgen never saw
a sightline through.

The point is the wall-staring case: a noise behind the bot in a direction with no sightline used to drag
the averaged mean into the wall between it and everything else. Now it simply loses.

With no vantage data -- an old map, `--no-vantage`, or `crossbot_vantage 0` -- `ConeWeight` returns 1 and
this degenerates to "the freshest live entry in the winning tier wins", which is the old behaviour minus
the averaging.

`ApplyLook` then blends, rather than overriding:

    desiredAngles.yaw = base.yaw + shortWay(want.yaw - base.yaw) * authority
    desiredAngles.pitch = base.pitch + (want.pitch - base.pitch) * authority

`base` is whatever `desiredAngles` already holds -- path facing at the walk site, the last commanded angles
at the idle sites. At `authority == 0` the write is a no-op and the behaviour is bit-identical to having no
sense system at all, which is what keeps the nav sweep honest. As the freshest suspicion ages, the head
drifts back onto the path by itself instead of snapping.

An aim target always wins: if `ReadDelayed` serves a point, `ApplyLook` aims at it and returns before any
of the above.

### When there is no suspicion at all, the vantage field drives the head

`ApplyLook`'s `authority == 0` branch used to level the pitch and leave the yaw alone. It now calls
`ApplyVantageLook`, which splits on whether a path is running:

- **No path** -- write `watch[0]`'s yaw and pitch. That is the direction most of the sightlines into this
  spot arrive from, so a bot standing still is watching the way it is most likely to be shot from.
- **Path running** -- `desiredAngles.y` at this point is the path yaw, freshly written from the next
  waypoint in `bot_move_nav.cpp`. Pick the watch direction closest to it in yaw; snap to it if it is
  within `kVantagePathSnap = 60` degrees, otherwise keep the path yaw and level the pitch. So a bot
  rounding a corner is already looking down the corridor rather than at its own next waypoint.

**The 60-degree clamp is not cosmetic: yaw is the movement basis.** `Bot_DecomposeMove` flattens to yaw
alone, so an unbounded snap turns every corridor into a sideways strafe. The clamp buys the head-turn
without giving up forward drive.

**Both branches write absolutely, not through the `authority` blend, and that is deliberate** -- see the
ratchet note below. `base` is the previous frame's own output at the idle call sites, so a blend there
compounds and the head freezes wherever it drifted to. An absolute write cannot ratchet.

The blend itself is refreshed once every `kVantageInterval = 0.1` s per bot from the bot's eye position,
not per frame; the field is a smooth spatial function and a bot does not move far in 100 ms.

`crossbot_vantage 0` skips the refresh entirely, so the blend is empty, `ConeWeight` returns 1 and this
branch falls through to the old `desiredAngles.x = 0`. That switches off everything the vantage field
does, which is what the nav-sweep A/B needs -- but it is **not** a full revert to the pre-field runtime,
because the argmax replaced the averaging unconditionally and does not depend on the cvar.

**Senses outrank the vantage look, and on a busy map that matters more than it sounds.** The tier vote
falls through to `KIND_OTHER`, and map machinery -- doors, chargers, item respawns -- pushes `SUSP_SOUND`
entries constantly. A live `KIND_OTHER` entry at weight 0.1 still takes the head away from the vantage
direction, because it is a real sense and this is the fallback. `crossbot_vantage_status` reports which
branch actually won per bot (`look=sense` / `vantage-idle` / `vantage-path` / `vantage-offpath` /
`gated`), which is the only reliable way to tell them apart from outside.

### The gate veto survived from the gaze scaffold

`ApplyLook` refuses entirely when any waypoint in `waypointIndex .. +2` needs a ladder, an airmove launch,
an elevator ride, a button use or a trigger touch. Without that lookahead the bot arrives at a gate with
its head turned and pays the full `kLadderAimTolerance` / `kActivationAimTolerance` latch cost, which is the
controller's one real cost.

It is also still **opt-in**, applied at five sites in `bot_move_nav.cpp` and never on the branches where
the view drives the movement. Opt-in fails safe -- a missed site means no looking around there. Opt-out
would fail dangerous, since a missed site breaks a ladder.

### Two gaps worth knowing about

**A bot with nothing in its cone and no suspicion used to never turn.** This is now solved, and the
solution is the Parabot one rather than the foxbot one. Parabot scored the four compass directions per
map cell by how much of the map is visible that way, weighted by where players historically died, then
swept the head around the winner. navgen's vantage field is that measurement done properly -- an actual
traced visibility signature per sampled eye position rather than four compass buckets -- and
`ApplyVantageLook` points the head at its `watch[0]`. Foxbot's cheap version (random yaw every 1-3s,
rejected unless a 200-unit trace is clear) was the alternative and is not needed.

What is *not* here is a sweep. The head goes to the main watch direction and stays there; it does not
cycle through `watch[1..3]`, so a stationary bot is now watching the right way but still watching one way.
The data to sweep over is already loaded -- `watch[]` carries up to four directions with their `amount`
and `spread` -- so this is a scheduling question, not a data one.

Footsteps narrow the remaining gap. A running enemy is heard at ~625 units regardless of facing, so a bot
turns round on its own as soon as anyone moves nearby. The uncovered case is the enemy who is stationary
or walking, since below 220 units/s they emit nothing at all -- and a bot facing its main sightline is
much better positioned for that case than one facing a wall.

**Pitch is levelled, and the suspicion pitch is clamped, for a reason.** When `ApplyLook` finds no target,
no live suspicion and no usable vantage direction it writes `desiredAngles.x = 0` rather than leaving the
angles alone -- and the vantage branches sidestep the same trap by writing absolutely. Without that the
blend ratchets: `base` is the previous frame's output, so each frame moves further toward the suspicion and
nothing ever moves it back — when the authority decays to zero the head simply freezes wherever it got to.
A door sound in a shaft below the bot was enough to leave two bots permanently staring at the floor at
about 70 degrees of pitch, which also put each other outside their view cones so neither could ever
reacquire. Levelling is safe at the walk site because navigation always commands pitch 0 there; pitch is
purely a look concern. The `kLookPitchLimit` clamp of 45 degrees bounds how far a very close suspicion can
crank the head in the first place, and does not apply to a real aim target, which needs the full range to
aim at someone below.

### crossbot_sense_status

Per bot: the aim target and the bone currently sampled, the remembered enemy and how stale that
sighting is, live suspicion count broken out by kind, which tier won the vote, the freshest entry with
its source and age and position, the look direction with its authority and how many entries survived
the cone refinement, and cumulative push counts per source.

`target=` against `enemy=` is the pair that shows the memory doing its job. They agree while the bot
can see someone; `target=- enemy=Sense2 enemyAge=1.8s` is the whole point of the feature, and
`enemy=-` returning after `enemyAge` passes `crossbot_sense_decay` is the record expiring on schedule.

`kept` is now simply the live entry count in the winning tier, not a mean-shift survivor count -- the
direction is one entry's, so there is nothing to survive.

`kinds` against `tier` is the second pair, and it is the one assertion worth automating: `tier` must read
`PLAYER` whenever `kinds=p:` is nonzero. `test_sense.ps1` checks exactly that on every poll.

`fresh=` is the freshest entry *overall*, not the freshest in the winning tier, so `tier=PLAYER` next to
`fresh=SOUND ent=148` is correct and not a contradiction -- a door was heard more recently than the
player, and lost anyway.

`pushed=` exists because dedup destroyed the old way of proving a source fires. With one entry per entity
an enemy's `DAMAGE` entry is overwritten by their own `SIGHT` entry inside the same `BotSense::Update`,
so `fresh=` reads `SIGHT` almost always and `DAMAGE` essentially never. `test_fire.ps1` used to sample
this command four times per poll to work around that; the counters are cumulative, so one poll cannot
miss a source that fired between polls, and the sampling loop is gone.

### crossbot_vantage_status

`crossbot_vantage_status` with no arguments reports the loaded field (point count, spacing, resolved
grid cell and dimensions, `maxExposure`, and the cvar) and then, per bot, the blend at its own eye:
neighbours used, distance to the nearest, `exposure` raw and as a fraction of the map maximum,
`focus[4]`, every merged watch direction, and a drive line giving `view=` (actual `v_angle`),
`desired=` (what the look code commanded this frame), `look=` (which branch produced it) and the
waypoint index.

`crossbot_vantage_status <x> <y> <z>` blends at a world point instead. **This is the round-trip test:**
at a stored vantage's exact position the distance-0 neighbour dominates the IDW, so the output must be
that vantage's own record back -- watch yaws, pitches, spreads and normalised amounts all matching. It is
the cheapest way to prove the loader, the grid and the angular merge are all correct at once.

`desired=` next to the watch list is what makes the look behaviour checkable from outside. On a
`look=vantage-path` sample the commanded yaw *and* pitch equal one of the listed watch directions
exactly, because the branch writes the direction rather than blending toward it. Two caveats when reading
it: the blend printed here is recomputed at print time, so on a moving bot it can be one sample ahead of
the one `ApplyLook` used, and `look=vantage-offpath` prints pitch 0 with the path yaw, which is the
60-degree clamp refusing the snap rather than a missing match.

### The enemy handle outlives the sight of it

`sense.target` is not memory. `UpdateVision` writes `sense.target.Set(best)` unconditionally, `best`
included when it is null, so the handle drops the frame line of sight breaks. That is correct for what
it feeds -- aim and the trigger -- and useless for deciding where to walk.

`sense.enemy` is the memory. It is written from the same site as `target` whenever a target is visible
and survives the loss: handle, position, and the time it was recorded. It is dropped only when the
entity is gone, when the entity is dead, or when the record is older than `crossbot_sense_decay` --
the same three tests `LiveWeight` applies to a suspicion, because it is the same question.

**The position stored is `v.origin`, not `bestPoint`.** `bestPoint` is a randomly chosen hitbox bone
and can be a hand held out over a ledge; `v.origin` is at the feet, which is what `FindAreaExact` can
resolve to a nav area. A bone-derived goal resolves to no area, or to the wrong one, in exactly the
cases where the enemy is somewhere interesting.

**It is deliberately refreshed only by sight.** Sound, gunfire and damage from that same enemy keep
updating its `Suspicion` slot and keep steering the head, but they do not move the remembered
position. So the bot walks to where it last *saw* someone rather than where it last heard a footstep,
and the goal holds still while it walks there.

### Sense reports, it does not decide

`sense.enemy` used to reach forward into navigation: `ForgetEnemy` cleared `bot.goalIsEnemy` and reset
the path, and a sight-loss edge armed a `rescore` flag that `Bot_MoveNav` consumed by dropping the
goal. Both are gone. Sense answers questions and owns nothing downstream; `BotCombat` reads the
answers and makes every transition (`bot_combat.cpp` below).

`Senses(bot, ent)` is the one question the table could not answer before: is there still *any* live
entry pointing at this entity? Sight, sound, damage and events all count, so a bot that can hear its
enemy has not lost it. Entries expire on `crossbot_sense_decay` in the same sweep every other reader
uses, which is what makes this "have I lost every trace of it" rather than "did I ever see it".

`bot.sense` is not covered by anything that resets on death, so `BotSense::OnRespawn` clears the enemy
alongside `BotItems::OnRespawn`, `BotFire::Reset` and `BotCombat::OnRespawn`.

## bot_combat.cpp

### Combat is a state, not a goal

The question "is this bot fighting someone?" used to have five different answers living in five
files: `sense.enemy` aged on `crossbot_sense_decay`, `bot_fire`'s own 3s `kCombatMemory` on
`targetTime`, `bot_deploy`'s `enemy || target`, `bot_items`' readiness-gated enemy candidate, and the
`sense.rescore` pulse. Nothing owned the transition, and `BotSense::ForgetEnemy` reached forward into
navigation to reset the path -- the wrong direction of dependency.

`BotCombat` is the one owner. `Update` runs immediately after `BotSense::Update` and makes every
entry and exit; `Run` is the behaviour, and it is what stops the path follower.

The old behaviour was a **chase**: the last-seen position competed in `SelectGoal` and won a full A*
route across the map. Replacing it with a local shuffle is the point of the whole change -- a bot in a
firefight should be sidestepping, not commuting.

### Two ways in, five ways out

Entry is sight (`sense.target`, which is line of sight *this frame*) or damage from a player.

`DamageSource` is the damage half, and its job is almost entirely rejection. `pev->dmg_inflictor` is
not the attacker: for a bullet it is the player, but for a rocket or a grenade it is the projectile,
which carries the shooter in `pev->owner` -- so the inflictor is resolved through `owner` when it is
not itself a player. Everything the bot should *not* pick a fight with falls out of that same
resolution rather than needing a list: falling, drowning and crush damage are inflicted by worldspawn
at edict index 0, a `trigger_hurt` is a brush entity no player owns, and a rocket the bot fired at its
own feet resolves back to the bot.

Exit, checked every frame:

| Rule | Reason string |
| --- | --- |
| `enemy.Get()` is null | `the enemy handle went stale` |
| `IGameProfile::IsDead` | `the enemy died` |
| `BotSense::Senses` finds nothing | `lost every trace of the enemy` |
| 5s without line of sight | `went too long without line of sight` |
| the bot died | `respawned`, from `BotManager` |

The last two overlap on stock settings: suspicion entries expire on `crossbot_sense_decay`, which
also defaults to 5, so either rule may be the one that logs a given exit. Raising the cvar makes the
sight rule authoritative; lowering it makes `Senses` fire first. That is deliberate -- the sight rule
is a fixed promise about *this* behaviour, not a second name for the sense decay.

**A new opponent retargets rather than re-entering.** Exiting and re-entering would throw away the
probe and re-drop a goal the bot had already given up. It is the same fight.

**Damage entry stamps `lastSightTime` even though the bot has seen nothing.** Shot in the back, the
bot gets the same 5s everyone else gets to turn around and find whoever did it; if it never does, the
sight rule ends the state on schedule.

### The takeover waits for maneuvers, the state does not

`Run` returns false while `Bot_ManeuverActive` -- airmove, ladder, elevator ride, activation, charger,
break, deploy. Those runners live below goal selection and steer along the waypoint list, so calling
`ClearGoalItem` mid-flight would drop a bot into whatever is under it. The state is still entered on
time, and `driving` records whether the movement half has actually started.

The first frame `Run` does drive, it calls `BotItems::ClearGoalItem` once. That is what "path
following is terminated" means concretely: goal cleared, `hasGoal` false, waypoints gone. On exit,
`nextSelectTime` is zeroed so gathering picks something new on the next frame instead of idling out
the retry interval.

### The shuffle: one random direction, judged on exposure alone

Every `kProbeInterval` (0.7s), or on arrival at the last probe, the bot picks a random yaw, resolves a
standable point that way, and compares its `exposure` to the exposure where it stands. Lower wins. If
the probe is no better, the bot goes the *opposite* way instead -- unscored, by design: the algorithm
is a random walk biased toward cover, not a search.

**`focus[]` is not part of the score and cannot be.** It measures the fraction of the visible set
inside a cone around `watch[0]` (`doc/navgen.md`), so it describes one facing rather than the spot.
A high `focus[3]` says "if you look *there*, you cover nearly every sightline in" -- a fine thing to
know once you have chosen a spot, and not a way to choose between spots. `exposure` is the only field
that scores a position independent of where the bot is looking.

**The enemy is deliberately not consulted.** No line-of-sight trace from the probe to whoever the bot
is fighting. The bot can shuffle itself out of the fight, and when it does, the 5s rule ends the state
and it goes back to gathering -- which is a reasonable outcome, not a failure.

`hereExposure` is read from `bot.sense.vantage`, the blend `RefreshVantage` already maintains every
0.1s. Only the probe costs a `BlendVantage`.

### Local means the current area and its walk neighbours

`ResolveSpot` accepts the area the bot occupies and any area joined to it by a `CONN_WALK` connection,
honouring `CONN_FLAG_ONE_WAY`. Jumps, ladders, lifts and teleports are route-building, which is
exactly what combat drops. Containment is `Nav::AreaContainsPoint` -- areas merge into non-convex
prisms, so the bound-plane test is required and an AABB clamp would be wrong.

**A single full-length step is not enough.** Measured on crossfire, a plain `origin + dir * 160`
resolved to nothing **73%** of the time: the map's areas are routinely narrower than the step, and a
walk neighbour at a different height fails the z test because the probe is placed at that area's
centroid z. The direction was fine; the distance was not. `ResolveAlongRay` retries the same ray at
0.7, 0.45 and 0.25 of the step, so `walled off` now means the bot really is against something rather
than merely that it aimed long.

**With no vantage field** -- a nav built `--no-vantage`, or `crossbot_vantage 0` -- there is nothing
to compare, so the bot takes the random direction unscored and the reason reads `no vantage`. Moving
unpredictably beats standing still.

## bot_fire.cpp

The bot could see, aim and describe every gun it carried long before it could pull a trigger. This is
the piece that closes the loop, and it does it for **instant-hit weapons only** — crowbar, glock,
python, MP5, shotgun, gauss primary, egon. Everything else is skipped rather than mishandled.

Two entry points, and the split between them is the design:

- `BotFire::Think` is **policy**. It runs once per frame from `Bot_MoveNav`, immediately after
  `BotSense::Update`, and decides which weapon and which fire mode the bot is trying to use. It has
  side effects — it can issue a weapon-select command — so it needs a single known slot in the frame.
- `BotFire::Buttons` is the **trigger**. It is pure, it runs inside `Bot_RunPlayerMove`, and it
  answers one question: given the angles about to be sent to the engine this instant, does the shot
  hit?

### Running dry has to release the trigger

The single most important fact about `CBasePlayerWeapon::ItemPostFrame`: the auto-reload lives in the
`else if ( !(button & (IN_ATTACK|IN_ATTACK2)) )` branch. **A bot that holds `IN_ATTACK` through an
empty clip clicks forever and never reloads.** Every ammo check in this file exists to release the
button rather than to save a wasted press.

`IN_RELOAD` is honoured directly by the same dispatcher, so the bot reloads deliberately instead of
falling through to the idle branch. That matters: the idle branch also runs `GetNextBestWeapon`, which
would change the weapon out from under the choice `Think` just made.

`wantReload` is set in `Think` and consumed in `Buttons` ahead of every other test. It is deliberately
cleared while `m_fInReload` is set, so the sequence is one frame of `IN_RELOAD`, then nothing at all
until the reload completes — which is exactly what a released trigger looks like to `ItemPostFrame`.

### Ready means <= 0

`CanAttack` under `CLIENT_WEAPONS` is `attack_time <= 0.0`, and `CBasePlayer::ItemPostFrame` gates
everything on `m_flNextAttack > 0`. That settles the question `bot_weapons.cpp` left open: in retail
Half-Life these are countdowns, not timestamps, and `BotInventory::nextAttack > 0` means nothing will
fire this frame no matter what the buttons say. `Buttons` returns 0 there rather than pressing into a
closed gate.

### The fire ray is v_angle + punchangle, and that is burst fire for free

Bullets travel along `pev->v_angle + pev->punchangle`, not `v_angle`. Testing the fire ray with
punchangle folded in means the bot sprays, recoil walks the muzzle off the target, the ray leaves the
box, the trigger releases, punchangle decays and it fires again. **There is no burst timer in this
file and none is needed.** It also means the bot never compensates for its own recoil, which is the
top item still outstanding in `bot_aim.cpp`'s deferred list; here that gap produces the right
behaviour by accident and should not be "fixed" without replacing it with something.

`IN_ATTACK2` short-circuits `IN_ATTACK` in the dispatcher — the secondary test is an `if` and the
primary is its `else if` — so `Buttons` returns one bit or the other and never both.

### The box is centred on the served aim point, not on the enemy

`Buttons` builds a box the size of the target's own `maxs - mins` around **the point `Think` published
this frame** — the served aim point for hitscan, the solved lead point for a projectile — and fires
when the ray enters it. The enemy's extents are the tolerance: a bigger target is easier to justify
shooting at, which is the whole of the error budget.

Centring it on the served point rather than the enemy's live origin is the interesting choice. It
makes the test ask *is my hand where my intent is*, and three things fall out of that:

- A bot mid-flick is not yet pointing where it means to, so it holds fire. No separate settle timer.
- A bot whose reaction delay has it tracking a stale position fires at that stale position and
  **misses**, the way a human does. Aiming error stops being cosmetic.
- Line-of-sight loss needs no special case. The queue drains for `crossbot_aim_delay` after the
  target is gone and the bot keeps tracking the place it last believed them to be for exactly that
  long, then stops when `ReadDelayed` runs dry. Whether any of those frames become *shots* is the
  world trace below, and increasingly they do not: `UpdateVision` no longer queues a lead point that
  has already crossed into geometry, so the drain now mostly runs out over open ground rather than
  into a wall.

That drain only works because `BotSense::UpdateVision` clears the queue on acquiring a *different*
live target and not on losing one — see `bot_sense.cpp`.

The served point is a randomly chosen hitbox bone, so a box centred on a foot reaches a full body
height below the enemy. That is generous, and it is generosity being spent deliberately rather than
by oversight.

A second trace, `ignore_monsters` from the eye out to `range`, rejects the shot if world geometry
stops the ray before the box entry. Without it a bot happily fires through a wall at someone it can
still hear.

None of this applies to an arcing mode, which has no straight ray to test. That path is below.

### Supported is the incremental frontier

```cpp
(mode.fire == FIRE_HOLD || mode.fire == FIRE_CHARGE || mode.fire == FIRE_TAP) &&
mode.toggle == TOGGLE_NONE &&
(mode.aim == AIM_TARGET || (mode.aim == AIM_LEAD && mode.speed > 0)) &&
(homing || arc || mode.gravity == 0) &&
(arc || !thrown) &&
(mode.quirks & (WPN_Q_HIDDEN_STATE |
                ((homing || thrown || guided) ? 0 : WPN_Q_INHERIT_VEL))) == 0
```

A predicate over the spec, deliberately **not** a list of weapon ids. Turning the next group on is
relaxing a clause here; it is not editing a table.

**The frontier is now three doors wide**, and all three are the same feature. What is left out is a
`WeaponToggle` (the two zooms and the RPG laser), an aim of `AIM_NONE` or `AIM_SURFACE` (satchel
primary, tripmine), and `WPN_Q_HIDDEN_STATE` (both satchel modes, RPG secondary). Between them that is
the **deployables** and nothing else: things placed rather than fired, which need a path that runs
with no target at all.

`arc` is `gravity > 0 && splash > 0 && !homing` — a mode whose flight the bot solves and then checks
by simulating. `thrown` is `WPN_Q_THROW_ARC` and `guided` is `WPN_Q_KEEP_AIMING`. `arc` is a local
predicate rather than a quirk because it is derivable, and `weapon_spec.h`'s one criterion for a field
is that a consumer cannot recover it from the others; the other two are quirks because nothing else in
the row implies them.

`arc` admits two modes that need different solves. **MP5 secondary** is a fixed-speed parabola with a
contact fuse. **Handgrenade primary** is a parabola whose speed and direction are one parameter and
which inherits the thrower's velocity, and `thrown` is what routes it to the coupled solve below.
`arc && !thrown` gets its own name, `ArcDetonates`, and it is the predicate the uphill veto turns on.

**`WPN_Q_INHERIT_VEL` is now the only quirk in the reject mask that is conditional**, and the
condition is "does anything account for it". Three things do. A `thrown` mode solves it exactly — the
launch is a vector triangle, not a longer iteration. A `homing` mode performs no solve, so inherited
velocity rotates a launch vector nobody aimed. A `guided` mode has the same exemption for a different
reason: the RPG adds only `v_forward * dot(playerVel, v_forward)`, a change to launch *speed* rather
than direction, and a rocket that steers at the crosshair does not care. What is left is the satchel,
which `WPN_Q_HIDDEN_STATE` holds out anyway.

`splash` no longer rejects anything. It used to reject any non-arc mode outright, which is what kept
the RPG out, and it has been replaced by a gate that runs for every mode with `splash > 0` — see the
self-damage rule below. Self-damage is still a decision rather than a tolerance; it is just decided
per shot instead of per weapon.

`ModeUsable` adds the conditions the spec exists to answer: `range` (which is what keeps the crowbar
holstered past 32 units and the shotgun and egon past 2048), the `ammo` pool (the one field that
genuinely cannot be derived at runtime — secondary fire does *not* imply secondary ammo),
`WPN_Q_NO_UNDERWATER` against `waterlevel == 3`, the uphill veto below, and for `AIM_LEAD` a
flight-time ceiling.

That ceiling applies to homing modes too, where it is really a **distance**: `dist / speed >
kMaxFlightTime` with nothing solved is just `dist > speed * 1.5`, which parks the hornet past 900
units and the snark past 300. Both are about right for weapons you throw rather than aim, and it costs
no new field to say so.

Ammo is judged by `ShotCost`, which is `ammoPerShot` for everything discrete and the **whole spin-up**
for a `FIRE_CHARGE` mode: 16 uranium for a full gauss charge, not 1. A bot with less than that keeps
gauss primary, which is the right fallback and needs no special case.

Ammo is judged differently at the two levels, on purpose. For *selection*, a clip weapon with an empty
clip but a non-empty reserve is still usable — otherwise the bot throws away a good gun the instant it
needs reloading. For *firing*, the clip has to actually hold `ShotCost`.

### The ballistic solve, the sim, and the two gates

An arcing mode gets a different treatment end to end. `RayOnTarget` is not used at all — a
ray-versus-box test on a parabola is meaningless.

`SolveArc` is closed form. For horizontal range `d`, height difference `h`, launch speed `v` and
effective gravity `g`:

```
disc       = v^4 - g * (g*d*d + 2*h*v*v)
tan(theta) = (v*v - sqrt(disc)) / (g*d)
flight     = d / (v * cos(theta))
```

`disc < 0` is out of range, and it hands back a real ceiling instead of a guessed one: flat ground at
800 u/s under `kGravity * 0.5` gives `v^2/g`, or 1600 units. The **low** arc is always taken; the high
root is a mortar shot that spends three seconds in the air against a target that moves. `SolveArcLead`
wraps it in the same four fixed passes as `SolveLead` — solve, take the flight time, re-predict.

`SimulateArc` then steps that parabola at `kArcStep` with a traceline per segment. It exists for the
one question the closed form cannot answer, **what does the shell hit first**, and it traces
`dont_ignore_monsters` on purpose: `ExplodeTouch` fires on players too, so a shell that would hit the
enemy in the chest has to register there rather than sailing through to the wall behind. At 0.1s and
800 u/s each segment is 80 units, so a full flight is fifteen traces. The cost of that coarseness is
that a chord can cut a corner the shell would really have clipped, and the shot is then wasted — the
cheap failure of the two.

Reproducing the engine's integration exactly is not worth it. `SV_Physics_Toss` subtracts a whole
`g * frametime` before moving, so it is semi-implicit Euler and the parabola is out by `g*dt*t/2` —
about two units over a second, against a 250-unit splash radius.

Two gates decide the trigger, and both are in **units**, not degrees:

- **Near enough to hurt.** The impact must be within `splash * kSplashAccept` of the target's box,
  measured to the nearest point on it. Measuring to the box surface rather than its centre is what
  sampling at `mins.z` means in practice — a shell landing at their feet scores zero, which is right,
  because `RadiusDamage` falls off from the blast and a floor burst at the feet is a full hit.
- **Closer to them than to me.** `|impact - targetPoint| < |impact - eye|`. It needs no constant, it
  scales with the weapon, and it is the symmetric statement of "I win this exchange". It is also
  deliberately a little suicidal, which was the call: a bot that lingers doing nothing is worse than
  one that takes a graze.

The second gate is `SplashSafe`, and it is **not** an arc feature. It runs for every mode with
`splash > 0`, which is what let `Supported` stop rejecting splash outright and admit the RPG. For a
direct-fire splash weapon the impact is the ray's, not the parabola's: `RayOnTarget` already computes
both the box entry distance and the traced wall distance, so the impact is `eye + dir * entry` when
the shot reaches the enemy and the wall when it does not. No extra trace, and the refusal reads
`splash-self` rather than `arc-self` so the two paths stay tellable apart.

At close range the rule is knowingly generous. A bot that rockets someone 200 units away takes about
20 damage from a 100-damage, 250-radius blast and wins the exchange; that is the trade, and it is the
same trade the arc gate makes.

Both run twice per frame, in the two places for the two different reasons. `Think` runs them along the
**ideal** launch direction and treats a failure as *mode unusable*, so the bot picks another weapon
instead of standing in front of a wall holding a grenade launcher — the same reason `SolveLead`
already lives there. `Buttons` runs them along the **live** view direction, which lags behind the
solve by the reaction delay and by finite hand speed, and that is what decides the press.

`fire.aimPoint` and `fire.targetPoint` part company here for the first time. `aimPoint` is where to
*look* — `eye + launchDir * dist`, well above the enemy — and it reaches the view through
`BotFire::AimPoint` with no extra plumbing. `targetPoint` is where the enemy actually is, and the
splash gate needs it because the aim point is no longer a claim about them.

`miss` and `tol` in `crossbot_fire_status` switch to units for an arc: the gap from the impact to the
target box, against `splash * kSplashAccept`.

### The uphill veto, and what it was a proxy for

A mode with `gravity > 0` **that the bot does not solve an arc for** is rejected when the served point
sits more than `kUphillSlack` above the bot's eye. Throwing something that falls at an enemy standing
above you gets it back: a snark lands short and walks home, a grenade bounces down the step it failed
to clear.

`kUphillSlack` is 16 units, and the number comes from the hitbox rather than from taste. A standing
player's eye is 28 above the origin and the top of the hull is 36, so the highest bone the sense system
can serve on a *level* target is about 8 above the bot's own eye. 16 clears that with room and still
vetoes anything standing on a step, which is 18.

It lives in `ModeUsable`, not in `Buttons`, so the bot picks a different weapon rather than standing
there holding a snark it will not throw. `uphillVetoes` counts rejections, not situations — the
predicate runs per weapon per mode per frame — so read it as a witness that the rule fires at all,
which is the only thing `test_fire.ps1` asks of it.

**The exemption is for arcs that stop where they land, not for arcs.** Applied to a grenade launcher
the veto would be backwards — elevating is exactly what the solve is for — so it was first narrowed to
"anything the bot solves an arc for". That was too wide by one weapon, and the handgrenade is the
weapon.

The rule was always about projectiles that *persist after landing*, which is a property of the fuse
rather than of gravity. `ShootContact` gives the MP5 grenade `ExplodeTouch`, so the simulated impact
**is** the explosion and the sim genuinely knows where it ends up. `ShootTimed` gives the handgrenade
`BounceTouch` and `friction 0.8`, so the impact is only where it *lands*; it then rolls for whatever
remains of the 2.5-second fuse and explodes somewhere downhill. So the predicate is `ArcDetonates`,
`arc && !thrown`, and it names the thing rather than approximating it:

> The veto applies to projectiles whose resting place the sim does not predict — a snark that walks, a
> grenade that rolls.

Two weapons carry it now. The snark is the one that can never lose it, because it is homing and
nothing is ever solved for it, which is why `uphillVetoes` must stay non-zero in the pinned snark
phase: it is the only witness that narrowing the rule did not switch it off. The 9mmAR phase is the
witness for the other side — `uphill` must stay **zero** there, or the narrowing caught the contact
grenade it was written to exempt.

The residual roll is not modelled and is not going to be. Simulating `BounceTouch`, the 0.8 friction
and the ONGROUND damping is a bigger job than the whole solve, and 250 units of splash forgives more
than it would buy. It is the honest cost of throwing immediately instead of cooking.

### The lead solve, and why four passes with no convergence test

`AIM_LEAD` needs the point where the target will be when the projectile arrives, and the flight time
depends on that point, which depends on the flight time. `SolveLead` runs the obvious fixed point:

```
p = servedPoint
repeat 4:
    t = |p - eye| / speed
    p = servedPoint + servedVelocity * t
```

Four passes, unconditionally, and **no convergence test** — testing costs about what the pass it
saves costs. The map is a contraction whenever the target is slower than the projectile, which every
admitted mode satisfies by a wide margin against a 320 u/s player, so four passes land far inside the
target's own hitbox and the residual cannot change any decision downstream.

`kMaxFlightTime` (1.5s) rejects the solve, and `ModeUsable` pre-rejects any mode whose straight-line
`dist / speed` already exceeds it. That is a **plausibility limit on prediction, not a lifetime
model**: extrapolating a player who is free to turn 1.5 seconds ahead is fiction whatever the
projectile's real range is. It happens to park the tracking hornet past roughly 900 units, which is
where a bot should be choosing a different gun anyway.

`WPN_Q_SLOW_IN_WATER` halves `speed` at `waterlevel == 3`, matching `CCrossbow::FireBolt`, which
tests the *player's* waterlevel and not the bolt's.

### The queue carries velocity, so the lead is as stale as the perception

`BotAim::DelaySlot` stores the target's velocity beside the point, interpolated the same way, and
`SolveLead` uses the *served* velocity rather than the target's live `pev->velocity`.

This is the whole reason the lead is honest. Reading the live velocity would let a bot with a 250ms
reaction time predict a target's future perfectly — reaction delay would cost it position error but
never *prediction* error, which is backwards. With the served velocity, a target that changed
direction inside the delay window is led along the direction it was last seen going, and the shot
misses. The composition is

    perceived position + perceived velocity * (reaction delay + flight time)

since `UpdateVision` already folds `velocity * delay` into the point it queues. Both terms come from
the same stale sample, which is what a human has.

### Weapon choice drives the aim, and the call order already allowed it

`Think` publishes `aimPoint` — the served point for `AIM_TARGET`, the solved lead for `AIM_LEAD` —
and `BotSense::ApplyLook` prefers it over its own `ReadDelayed`. So the bot *looks* where the weapon
in its hands needs it to look, and `Buttons` tests the ray against the same value rather than reading
the queue a second time.

This was expected to need restructuring and did not. `Bot_MoveNav` already runs `BotSense::Update`,
then `BotFire::Think`, and only then reaches any `ApplyLook` call site — so the weapon decision was
already sitting between perception and aiming. No pass was added and nothing was reordered.

For `AIM_TARGET` the published point *is* the served point, so hitscan aiming is unchanged by
construction, which is what makes the existing duel phase a regression check on this.

### WPN_Q_HOMING replaces the solve with a cone

A hornet and a snark both steer themselves, so the bot's job is to point roughly at the enemy and let
go. `SolveLead` is skipped entirely, the published aim point *is* the served point, and the tolerance
becomes a fixed `kHomingCone` of 20 degrees.

The cone is spent through the same `RayHitsBox` as everything else — the half-extents are raised to
`dist * tan(20°)` on each axis, which is an angular cone written in the box the function already takes.
No second code path, and `miss`/`tol` in `crossbot_fire_status` stay comparable across every weapon:
`tol` simply reports 20 instead of whatever the target subtends.

This is a **simplification**, not an extra feature, and it is worth being clear about what was
discarded. The previous version solved the lead and then widened the box to the union of the present
and predicted boxes. That was more code for a worse model: a RED hornet accelerates from 300 while
turning, so the 600 in the table is a curve flattened to a float, and solving a lead against a number
that fictional buys precision that was never there. Pointing at the target and trusting the hornet is
both shorter and more honest.

`flight=` reads zero for these modes for the same reason — nothing was solved, so there is no flight
time to report, and printing the unsolved `dist / speed` would be inventing one.

Hornetgun **secondary** keeps the solve. Its ORANGE dart flies straight at 1200 and never tracks, so
one weapon exercises both paths, which is exactly what makes the pinned hornetgun phase a useful test.

### The coupled throw solve

A `WPN_Q_THROW_ARC` mode does not have a launch direction and a launch speed. It has **one**
parameter, and `IGameProfile::ThrowLaunch` is the map from it: the pitch the bot is holding goes in,
the elevation and speed the grenade leaves at come out. The five coefficients behind that live in
`CHLDMProfile` (see `doc/game.md`), so nothing here knows a number that belongs to a weapon.

Which means `SolveArc`'s closed form does not apply — it assumes a fixed `v` and solves for the angle,
and here `v` is a function of the angle. What replaces it is a scan, and the scan is cheap because
each sample is closed form in the *other* unknown.

Fix a candidate pitch. `ThrowLaunch` gives `(e, s)`, so the horizontal launch speed is `sh = s cos e`.
The grenade also inherits the thrower's velocity, so with `flat` the 2D vector to the target, `pvH` the
thrower's horizontal velocity and `u = 1/t`, the horizontal problem is one equation:

```
|flat * u - pvH| = sh
=>  A u^2 - 2 B u + C = 0,   A = |flat|^2,  B = flat . pvH,  C = |pvH|^2 - sh^2
```

A quadratic. Take the larger positive root — the shorter flight — and the rest falls out:

```
t     = 1/u
dirH  = (flat * u - pvH) / sh          unit by construction
err   = (sz + pvz) t - g t^2 / 2 - h
```

That is the **vector triangle** `WPN_Q_INHERIT_VEL` always needed, and it is solved rather than
approximated. `dirH` is the yaw the bot must hold so that launch *plus drift* arrives on target, which
for a bot strafing sideways is not the yaw pointing at the enemy. Its magnitude is `sh` by
construction, so there is nothing to normalise and nothing to check.

`err` is unimodal in elevation — it rises to the range envelope at `tan e = v^2 / (g d)` and falls
after — so scanning **down** the pitch range (which is up the elevation range) and taking the first
upward zero crossing finds the low arc. `kThrowScan` samples bracket it, `kThrowBisect` halvings
refine it. Samples where the quadratic has no positive root are simply invalid and skipped; the
crossing test runs between consecutive *valid* samples, which is what makes the far end of the range
degrade into "no solution" rather than into a wrong one.

The bracket ends are named `under` and `over` for the sign of `err`, not for their magnitude: pitch is
down-positive, so the `over` end is numerically the *smaller* of the two. Reading them as an ordered
interval is the one way to misread this function.

The muzzle is `eye + launchDir * 16` and so depends on the answer, and the target moves during the
flight. Both fold into the same four fixed passes `SolveArcLead` already used: re-predict the target,
re-place the muzzle from the previous pass's launch direction, solve again. Measured against a static
target the residual is under 0.03 units at every range it accepts.

The published `aimPoint` is built from the **pre-remap** pitch, not the launch elevation. The bot is
looking somewhere the grenade is not going, by design, because that is what the game's own remap does
to a player. `ArcOnTarget` undoes nothing either: it reads the live pitch, runs it through
`ThrowLaunch`, and builds the launch direction from `-elevation`. Nowhere is the remap inverted.

Reach is about 550 units, and the ceiling is `kMaxFlightTime` rather than the law — the throw itself
tops out near 625 on flat ground. `ModeUsable`'s `dist / speed > kMaxFlightTime` gate at the row's
speed of 400 prunes at 600, so there is a 50-unit band where selection says yes and the solve says no
and the bot idles half a second before re-picking. That is left alone rather than papered over with a
`range` value copied out of a bot-side constant.

### The three commits, and why they are one enum

`fire.commit` is the answer to "I have started something I must not walk away from". It replaced a
lone `charging` bool when a second and third weapon needed the same shape:

| | act | latch | release | cap |
|---|---|---|---|---|
| `COMMIT_CHARGE` | press and hold | — | the shot, or the aim gate | `chargeMax - kChargeSafety` |
| `COMMIT_THROW` | press one frame | `m_flStartThrow != 0` | it returns to 0 | `kTapTimeout` |
| `COMMIT_GUIDE` | press **on a ready weapon** | `m_cActiveRockets > 0` | it returns to 0 | `kGuideMax` |

The two new ones share a structure the charge does not need: the bot **cannot see its own action
immediately**. `CHandGrenade::PrimaryAttack` pulls the pin and returns; the grenade leaves half a
second later, from `WeaponIdle`. `CRpg::PrimaryAttack` creates the rocket inside
`Bot_RunPlayerMove`, so the count only reads back on the next frame. So `commitSeen` latches the frame
the game first shows the thing existing, and the release is `commitSeen && !live` — never `!live`
alone, which would fire on the gap between the button and the effect.

`COMMIT_THROW` also refuses on `m_flStartThrow` directly, not only on its own state, and that is not
belt and braces. A primed grenade whose target walks out of the 550-unit reach makes the solve fail,
which drops the mode and clears the commit — and a bot that then pressed again would skip
`WeaponIdle` for that frame and **cook a grenade it has already pulled the pin on**. Reading the
weapon's own flag closes it, and picks up the free case of a bot that inherits somebody else's primed
grenade off the floor.

`COMMIT_GUIDE` is entered only when `nextPrimaryAttack <= 0`, and that condition is load-bearing.
`Buttons` presses a `FIRE_HOLD` weapon every frame it likes the shot, which is how the engine's
level-triggered dispatcher is meant to be driven — the game fires when it is ready and ignores the
rest. But `CRpg::Reload` pushes `m_flNextPrimaryAttack` half a second forward on **every** frame the
reload button is held, so the moment the reload finishes there is a window where the clip is full, the
player's own `m_flNextAttack` has expired and the weapon's has not. A press there launches nothing. It
cost one phantom six-second window per reload before the gate went in, which showed up as `guides`
running six ahead of `shots` on a pinned RPG.

All three caps are valves rather than models. `kTapTimeout` covers a press that never took.
`kGuideMax` is six seconds, about 12,000 units of rocket, and it exists because **the SDK leaks the
counter**: `CRpgRocket::FollowThink` calls `Detonate()` directly on the self-detonation path without
decrementing, and `RocketTouch` only decrements while `m_pLauncher` is alive. Without the cap a leak
would pin a bot to an RPG it can neither fire nor reload, permanently.

`commitCaps` counts the two new ones and **not** the charge, which releases inside `ChargeButtons`
before the shared lifecycle sees it and reports itself as `why=charge-timeout` instead. That is an
asymmetry rather than a design: the gauss assertion still reads a sampled reason string where the
other two read a counter, and folding the charge into `commitCaps` would make all three uniform.

Everything a commit does, `holding` already did for the charge: it bypasses `ModeUsable`, pins the
mode order to one entry, and survives the no-target bail. Only the reason string and the release
condition differ.

### Guiding is press nothing, and that is the hard part

`WPN_Q_KEEP_AIMING` looks like it should mean "keep the crosshair there". It also has to mean **press
no button at all**, and that is the non-obvious half.

`CRpg::UpdateSpot` traces the player's view 8192 units and moves the `laser_spot` to the endpoint, and
the rocket steers at whichever spot it can see. But `UpdateSpot` is only called from `PrimaryAttack`
and `WeaponIdle`, and `ItemPostFrame` reaches `WeaponIdle` solely through its "no fire buttons down"
branch — which `IN_RELOAD` takes a different path around entirely. So a bot that reloads its
just-emptied RPG **freezes the laser where it last was** and the rocket flies to a stale point.

So `COMMIT_GUIDE` returns 0 from `Buttons` with `why=guiding` before the reload and empty-clip
branches, and `Think` leaves `wantReload` false for as long as it lasts. The reload happens after,
which is also what `CRpg::Reload` itself wants: it refuses outright while `m_cActiveRockets` and
`m_fSpotActive` are both set.

Guiding is also the only commit that keeps its aim through a lost target. `Think` skips `ClearAim` for
it, so the crosshair holds the last known position and the rocket goes there. A charge that loses its
target reports `why=carrying` and drops the aim, because a gauss shot at nothing is worth nothing; a
rocket already in the air is worth steering at where they were.

### The charge is carried, not timed

`FIRE_CHARGE` is the gauss secondary: hold `IN_ATTACK2` to spin up, and **the release is the shot** —
`CGauss::WeaponIdle` calls `StartFire()` once no fire button is down, which `ItemPostFrame` only
reaches through its "no fire buttons" branch.

The obvious implementation is a timer: press, wait `chargeFull`, release. It is wrong, and the SDK
says so. `SecondaryAttack` sets `m_flNextAmmoBurn = 1000` the moment the charge fills, and the burn is
gated on `!= 1000`, so **holding a charged gauss costs nothing**. A timer would throw away a paid-for
charge every time the bot's aim was off at the wrong instant. So the machine releases on the *shot*:

- not charging → press, unconditionally, before any aim test
- `held < chargeFull` → hold; a partial release fires for `200 * held / 1.5` damage and is never worth
  choosing
- charged → hold until the ray passes the tolerance test, then release. `why=charged` is the bot
  standing there with a full gauss waiting for a shot
- `held >= chargeMax - kChargeSafety` → release regardless. Past 10s the SDK zaps the player for 50 and
  costs a second, and `kChargeSafety` (0.5s) is the margin. `test_fire.ps1` **fails** if this ever
  fires, because reaching it means the release logic stopped working, not that the bot was patient

A held charge outranks everything else in the file. `Think` checks `fire.charging` **before** the
no-target bail, so a bot that loses its target keeps the weapon, keeps the charge, reports
`why=carrying` and walks around with it until someone reappears. It also bypasses `ModeUsable` while
holding — the ammo was already spent, so "can you afford this shot" is no longer a question — and
pins the mode order to one entry so selection cannot switch modes underneath it.

`Buttons` treats "no aim point" as *hold*, never as release. That asymmetry is the whole feature.

### Zoom is a range decision, because FovScale already prices it

Zoomed crossbow primary in multiplayer is `FireSniperBolt()` — a hitscan trace for a flat 120 — so
zooming does not sharpen the same shot, it swaps `AIM_LEAD` for `AIM_TARGET`. `ModeOf` gains a
`zoomed` argument and the `(zoomed && spec->primaryZoomed.fire) ? ... : spec->primary` one-liner from
`doc/game.md` is finally reachable instead of aspirational.

**Zooming costs turn rate and buys precision, and `FovScale` prices both.** It divides hand speed by
`fov / 90`, so at the crossbow's fov 20 the bot's peak turn rate falls from roughly 450°/s to 100°/s —
and it divides the hand quantum by the same factor, so the bot places the crosshair 4.5x finer. See
"The hand has a resolution" above for the measurements. That is a genuine trade rather than a pure
cost, so it gets a real rule: zoom above `kZoomIn` (500 units), unzoom below `kZoomOut` (300). A 320
u/s target crossing at 500 units sweeps about 37°/s and is comfortably trackable zoomed; at 200 units
it is not. The band rather than a single threshold matters because each toggle costs a frame and 1.0s
of `m_flNextSecondaryAttack`.

The thresholds predate the quantum and were set purely on the tracking cost. They are now pricing a
benefit as well, and have not been retuned for it — if a bot should be zooming further out than 500
units, that is the number to move.

Which weapons zoom is decided **by data, not by name**: only those with a populated `primaryZoomed`.
That admits the crossbow and excludes the python, whose zoom changes no mode and would cost a bot a
quarter of its turn rate for nothing.

`FIRE_TAP` is the whole of the press rule — press only when the live state differs from the wanted
one. `pev->fov` is read every frame, so a single press stops itself the frame after it lands; without
that, holding the button would flip the zoom every 1.0s forever. Toggles are never *fire modes*:
`Supported` still rejects them, and the press is a step before the fire decision that returns
`IN_ATTACK2` alone and shoots nothing that frame. `IN_ATTACK2` short-circuits `IN_ATTACK` in
`ItemPostFrame` anyway, so there was never a frame to share.

### Selection spends the weapon the bot has used least

Every `(weapon, mode)` pair carries an accumulator of seconds. It lives in `BotFire::Usage` on `CBot`
rather than in `BotFire::State`, because `Reset` assigns a fresh `State` on every respawn and the whole
point of the tally is that it outlives a life — a bot that spent the last three lives on the mp5 should
still reach for something else after the fourth.

**It counts time on target, not shots.** `Buttons` adds `frametime` to the held pair the moment
`onTarget` comes back true, which is downstream of every readiness gate — reloading, `nextAttack`, an
empty clip and the zoom toggle have all already returned by then. So the number is "seconds this
weapon spent pointed at somebody with the trigger available", which is what a weapon costs the bot in a
fight; a slow weapon is not punished for firing fewer rounds and a fast one is not rewarded for spraying.
Rounds fired would have measured the weapon's cycle time instead of the bot's attention.

Zoomed variants fold into primary. `BotFire::Mode` has no zoom member — zoom is a range decision made
inside the mode, not a mode of its own — so `primaryZoomed` accrues where `primary` does.

`Buttons` can run twice in one frame (the jump split in `Bot_RunPlayerMove`), so accrual is stamped
with `gpGlobals->time` and the second call is skipped. Without the stamp a jumping bot would rate its
weapon twice as expensive as a walking one.

### Three triggers, and only one of them is new to combat

1. **The held weapon has no usable mode.** Unchanged, and still the common case: Half-Life auto-switches
   to whatever it considers best on pickup, which for a roaming bot is regularly an RPG.
2. **The clip is dry, in combat.** `AmmoReady` grew a `strict` flag. The strict pass demands the rounds
   be in the clip *now*; the loose pass keeps the old behaviour of counting reserve ammo, because a
   weapon that can be reloaded is still a weapon. `Think` runs keep-strict, select-strict, keep-loose,
   select-loose in that order, so a loaded weapon in the inventory always beats reloading the empty one
   in hand, and reloading under fire only happens when nothing else is loaded. `why=empty-switch` is
   that path.
3. **Every 3 s out of combat.** Out of combat means no target this frame and none seen for
   `kCombatMemory` (3 s, off `sense.targetTime`, which `UpdateVision` stamps every frame a target is
   visible). The memory window is what stops a bot from fumbling with its inventory the instant an
   enemy steps behind a pillar.

There is deliberately no in-combat rotation timer. Switching weapons mid-fight for variety's sake
costs a deploy and reads as indecision; the two combat triggers are both "this weapon stopped working".

### Out of combat the bot reloads first, then rotates

`ItemPostFrame` auto-reloads only at `m_iClip == 0`, so every partial clip in the inventory stays
partial until somebody presses the key — and it only runs for the active item, so the bot has to hold
the weapon while the reload lands. Hence the idle pass picks the largest reloadable deficit, selects
that weapon, and `Buttons` presses `IN_RELOAD` from a branch above the `mode == MODE_NONE` return
(`fire.idleReloadId`). A weapon that still needs rounds wins the next tick outright, so an in-progress
reload is never abandoned halfway.

Only when nothing needs rounds does the least-used pair win, with a random tie-break through the
existing per-bot RNG. **Melee is excluded from the ranking** and reachable only as the last resort when
no other pair is usable — a crowbar with zero seconds on it would otherwise win every idle tick for
free, since it never runs out of ammo.

**Out of combat the held weapon wins ties.** The bot switches only when its best held pair has strictly
more seconds than the best candidate. Usage accrues only in combat, so without that rule everything the
bot could rotate to sits at zero forever and an idle bot cycles its whole inventory every three seconds.
The exception is a held melee weapon, which loses ties on purpose: a bot idling with a crowbar and a
loaded shotgun in the bag should be holding the shotgun.

Out of combat there is no target, so range, uphill and flight time cannot be evaluated at all. That
split is why `ModeUsable` became `ModeViable` (`Supported`, the pin, `WPN_Q_NO_UNDERWATER`, ammo) plus
the geometry gates layered on top; the idle path calls the target-free half.

### The mode survives the switch now

`fire.pendingMode` rides along with `fire.pendingId`, and the keep-loop tries the pending mode first on
the frame the switch lands. Previously the mode was thrown away and the loop restarted at
`MODE_PRIMARY`, so a weapon whose primary worked never reached its secondary by choice: the gauss never
charged, the glock never used its fast fire, and mode variety was entirely an accident of range.

The active weapon is excluded from the choice list when the keep-loop already tried both of its modes.
It failed for a reason — usually a blocked arc solve rather than ammo — and letting the ranking re-pick
it would spend a 0.5 s `kSelectInterval` per frame to select the weapon already in hand.

`kSelectInterval` still throttles combat re-selection to 2 Hz so that a select the game refuses cannot
become a per-frame command loop, and completion is still observed rather than assumed:
`BotWeapons::Refresh` reports `activeIndex` every frame, and the frame that issues the command reports
`mode=none` and tries again next frame. Being throttled no longer blocks the loose keep, so a bot
waiting out the interval keeps firing what it has instead of standing idle.

`crossbot_weapon_usage` prints the table. `crossbot_fire_pin` is still the only lever that reaches
selection deterministically, because in combat the rules above are all "the weapon stopped working" and
a pinned weapon is the only way to make that happen on demand.

### Why the trigger lives inside Bot_RunPlayerMove

`Bot_RunPlayerMove` is the one funnel every drive branch passes through, and it is the only place that
has the angles actually being commanded this frame. Putting the call there gets firing into walking,
ladders, elevator rides, airmoves and idling at once, instead of seven copies of the decision — and it
gets the exact ray the engine is about to use, rather than one aim-controller step stale.

It takes `CBot &` instead of `edict_t *` for that reason alone.

The jump split means a held `IN_ATTACK` can span two `pfnRunPlayerMove` calls in one frame. That is
harmless, and it is why nothing here depends on button *edges* — which is just as well, since
`ITEM_FLAG_NOAUTOFIRE` does not exist in this SDK and no weapon reads `m_afButtonPressed`.

### The shot counter counts the game DLL, not the plugin

`shots` increments when the active weapon's primary or secondary attack timer goes from ready to busy
**on a frame after the bot actually pressed**. The press gate is not decoration: deploying a weapon
and `GetNextBestWeapon`'s empty-weapon switch both arm `m_flNextPrimaryAttack` too, and a counter
without it reads several shots per weapon pickup with firing disabled entirely. The observation is
still the game's own timer — the gate only decides attribution.

`leadShots` is the same observation gated on the press having been a *solved-lead* one, carried on the
press flag rather than read back afterwards — by the time the timer moves, selection may already have
chosen something else. It is separate from `shots` because a bot keeps its spawn glock throughout a
test and would otherwise let a completely dead lead solve pass on hitscan alone. Homing presses do not
count toward it, which is what makes `leadshots < shots` a usable assertion on the pinned hornetgun:
equality would mean the cone never replaced the solve.

**The charged gauss needs a different witness entirely.** `CGauss::StartFire` moves no attack timer —
not `m_flNextPrimaryAttack`, not `m_flNextSecondaryAttack`, not the player's `m_flNextAttack` — so the
observation every other weapon relies on simply does not happen. `chargeShots` counts the falling edge
of `BotWeapon::chargeState`, which is `m_fInAttack` read straight off the weapon object, and that is
still the game DLL's own state rather than the plugin's intention. The 10-second overcharge would also
register as a discharge that fired nothing; `kChargeSafety` is what keeps the bot from ever reaching
it, and the test asserts the safety release never fires.

`arcShots` rides the same press flag as `leadShots` and needs no special witness, because an arcing
weapon does move its attack timer. It is worth counting separately anyway: `arcshots == shots` on the
pinned MP5 is what proves nothing reached the trigger through the ordinary ray path, and the test says
so — the first run of it caught a gauss charge carried in from the previous phase, which is
`ModeUsable` being bypassed by design and not by accident.

**The handgrenade needs a different witness for the same reason the gauss does.**
`CHandGrenade::PrimaryAttack` moves no attack timer either — it only pulls the pin — and the throw
happens half a second later inside `WeaponIdle`. So `throwShots` counts the falling edge of
`BotWeapon::throwStart`, which is `m_flStartThrow` read straight off the weapon object, exactly as
`chargeShots` reads `m_fInAttack`. The press frame deliberately sets no press flag at all, so there is
no path by which one throw could be counted twice.

`guides` counts the rising edge of `COMMIT_GUIDE`, so it is one per rocket launched. `commitCaps`
counts commits released by their cap instead of by the game's own state, and it is the one to assert
on: it has exactly the standing `charge-timeout` has on the gauss. Non-zero there means a pin was
pulled that never reached the weapon, or `m_cActiveRockets` never came back to zero.

`reloads` counts the rising edge of `wantReload`, because the state itself lasts one frame and is
invisible to any poll. `selfVetoes` counts refusals to fire into a blast nearer the bot than the
enemy, and like `uphillVetoes` it is a per-frame predicate, so read it as a yes/no. It is the only
refusal witness that survives a two-second poll — the `why=` string samples one frame and misses
almost everything it describes — which is why the arc phases assert on the counter and treat the
reason strings as a bonus.

Every counter is per life. `BotFire::Reset` runs on respawn from `BotManager::StartFrame`, so a drop
in any of them is a death, which is how `test_fire.ps1` counts deaths without reading the scoreboard.

### What is not here

**The deployables are no longer here, and `Supported` never learned them.** The tripmine and the
satchel are placed by `bot_deploy.cpp`, which owns the frame the way `BotBreak` does, and `BotFire`
stands down for it exactly as it does for a break. `Supported` still rejects `AIM_NONE`,
`AIM_SURFACE`, `WeaponToggle` and `WPN_Q_HIDDEN_STATE`, and that is correct: the diagnosis below was
right, and the answer was a second decision layer rather than a wider gate.

What they shared is why neither belonged in `BotFire`: **both fire with no target at all**, and
`Think` returns `no-target` several branches before any mode is considered. They wanted a decision
layer that runs on the map rather than on an enemy, plus a memory of what the bot has already placed.
Both now exist in `bot_deploy.cpp` — the map layer is the nav mesh's own connection gates, and the
memory is a per-bot connection memo.

`CRpg::m_fSpotActive` still stays out of `GameOffsets`, and the reason is no longer the per-class rule
— `rpg_cActiveRockets` broke that deliberately and safely, gated by the quirk. It stays out because
nothing would read it: the laser is on from `CRpg::Spawn` and the bot is forbidden from pressing the
button that turns it off. `m_chargeReady` was in the same position until the satchel needed it; it is
requested now, and it cost the offset rule nothing, exactly as predicted.

Also still absent: weapon preference weights, and recoil compensation. The aim override, the toggle
step, the commit machine and both arc solves exist now, and none of the above needs a fifth one.

## bot_move.cpp

### Bot_DecomposeMove
PM_WalkMove and PM_AirMove both flatten forward/right and renormalize before use, so whatever pitch and roll the engine puts in pmove->angles drops out and the basis is just the yaw.

### Bot_NormalizeAngle
Wraps an angle into (-180, 180]. Lives here rather than in bot_move_nav.cpp so both movement
translation units share one copy.

### Bot_BeginFrame and Bot_FrameTime
**`gpGlobals->frametime` is not the frame time once a bot has moved.** `PF_RunPlayerMove_I` calls
`SV_RunCmd`, and `SV_RunCmd` ends with `gGlobalVariables.frametime = ucmd->msec * 0.001` before
`pfnPlayerPostThink` (`sv_user.cpp:1055`). It never restores it. The engine only puts the real value back
at the top of `SV_Frame` and after each `SV_ExecuteClientMessage` in `SV_ReadPackets` -- nothing
restores it between two bots inside one `pfnStartFrame`. So the second bot in the loop reads the *first
bot's command length*, the third reads the second's, and so on down the array.

While every bot asks for the same msec that is self-sustaining and invisible. It stops being invisible
the moment one bot in the middle of the array issues a short command:

```
CrossBot TIME chain (frame 10.04ms): bot1[saw 10ms]=10ms ... bot6[saw 10ms]=10ms
  bot7(dead)[saw 10ms]=1ms bot8[saw 1ms]=1ms bot9[saw 1ms]=1ms ... bot12[saw 1ms]=1ms
```

One dead bot pressing `+attack` to respawn used to hand the engine a token 1 ms command, and every bot
after it in `g_bots` then ran at 1 ms of simulated time per 10 ms frame -- a tenth of real speed, for as
long as that one bot stayed dead. Movement *and* gravity slow together, because both are integrated
against the same msec, which is what makes it look like the bot is wading through honey rather than
like a bot that is lagging or stuck. With a dozen bots someone is nearly always dead, so nearly always
somebody is in it. The 50 ms first half of the jump split and the engine's own halving of commands over
50 ms are the same bug at lower server framerates.

`Bot_BeginFrame` latches `gpGlobals->frametime` at the top of `BotManager::StartFrame`, before any bot
has moved, and `Bot_FrameTime` hands that value out for the rest of the frame. Every per-frame
integration in the bot -- `Bot_FrameMsec`, the aim filter's `dt`, `AirmoveLaunchLead`, the weapon-usage
accumulator -- reads the latch. `gpGlobals->frametime` must not be read directly from bot code.

`crossbot_time_debug 1` prints the per-second budget-vs-issued report and the frame chain above; see
`bot_timing.cpp`.

### Bot_RunPlayerMove
The engine halves any usercmd longer than 50 ms and runs PM_PlayerMove once per half, so a
button held in a long command is re-read after the bot has already moved. IN_JUMP is the one
button where that matters: PM_LadderMove reads it with no oldbuttons gate and answers by
shoving the player 270 units off the ladder plane, so a jump that merely *starts* out of a
ladder's reach ejects the bot the moment a later sub-step brings it into contact.

Every actuation goes through here so a jump costs the same one frame of held button at any server
framerate. Below the split threshold this is exactly the single call it replaces.

It takes `CBot &` rather than `edict_t *` because it is also where the fire button is added. Being the
one funnel every drive branch passes through is exactly what firing wants, and this is the only place
that holds the angles actually being commanded this frame — see `bot_fire.cpp`, *Why the trigger lives
inside Bot_RunPlayerMove*. A fire button is held, never edged, so the jump split spanning it across
two sub-commands changes nothing.

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

### Airmove simulation cache

`MakePlanTraces` attaches the process-wide `g_airmoveCache` to every `PlanTraceContext` it builds, so all
plan-time `JumpLandsIn` calls share one table. The cache itself, its key, the world-only rule that decides
what may be stored and the sizing measurements all live in `doc/nav.md` under `airmove_cache.cpp`; what
belongs here is the wiring.

`Bot_ResetAirmoveCache()` clears the table. It is called from both `BotManager::ServerActivate` and
`ServerDeactivate`, alongside `BotSense::ResetBoneCache()` — that pair is the only per-map clear site in
the plugin, and anything holding map-scoped state has to hook it. There is no sizing cvar: the table is a
fixed member array sized by `kAirmoveCacheSlots`, so changing it means a rebuild. `MakePlanTraces` reads
`crossbot_airmove_verify` into `SetVerify` on every plan.

`Bot_EngineTraceFn` fills `PmTrace::hitEntity` for this: `tr.pHit` non-null, `pfnIndexOfEdict` non-zero,
not a `SOLID_BSP` brush flagged `FL_WORLDBRUSH`, and not accepted by `crossbot_nav::IsBakedBlocker`. The
first two are the `HitQualifies` idiom from `bot_avoid.cpp` minus the solid-class filter, which the
`ignore_monsters` flag on the trace already covers. The last is the shared predicate in
`nav/baked_blockers.h` that navgen uses to decide what gets baked into the mesh as permanent geometry —
runtime and navgen must agree on that list, which is why it is one function and not two.

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
- `Bot_Physics` reads sv_airaccelerate, sv_accelerate, sv_friction, sv_stopspeed, sv_edgefriction and
  sv_maxvelocity, so plan-time physics match a non-default server. Only the first three were read
  before, which silently diverged the profile from the engine. It does *not* read `sv_gravity` — the
  800 is `PmSim::kGravity` (see `doc/game.md`) — but it does multiply by `pev->gravity`, which is what
  `trigger_gravity` writes onto the player.
- `Bot_PushFields` snapshots the live `trigger_push` entities into a `PmSim::PmPushFn` once per plan,
  so `JumpLandsIn` simulates a launch through a push field the same way navgen sampled it. A field
  that is `SOLID_NOT` (START_OFF, or toggled off) is skipped; the next repath picks up the change.
- `Bot_DriftVelocity` is the steering counterpart: the world-frame shove acting on the bot right now,
  read live as `groundentity->v.movedir * speed` when the ground entity carries `FL_CONVEYOR`, plus the
  cached push closure at the bot's origin. It goes into `NavFollow::Config::driftVelocity`, which is what
  stops a bot on a conveyor or in a current being carried off its own plan — see `doc/nav.md`. The closure
  lives on `CBot::pushFields` and is refreshed with each repath, because scanning every edict per bot per
  frame is not free.
- `sv_maxvelocity` matters only where a push field is involved: `bounce`'s pods push at 3800 u/s
  squared, which runs the simulated velocity into the engine's clamp within a second.
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
`Nav::FindAreaExact`, not `FindAreaNearest`. The forgiving one cannot fail: with no containing
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
`FindAreaNearest(origin)`. That function never returns NULL — after the exact half-space test it falls
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
with the strict `Nav::FindAreaExact`: the target area rewards, a *different* area penalizes,
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

**A ride is only scored when the bot settled *inside an area*.** `FinishElevatorRide` grades its
connection exactly as `FinishLadder` and the air move do, and for a long time it was the odd one out: it
read the landing with `FindAreaNearest`, which never fails. An elevator column is precisely where two
areas overlap in AABB, and a bot standing on the mover's brush is routinely outside every area's exact
planes, so a ride that ended on the wrong floor snapped onto a plausible neighbour and a ride that never
arrived earned `RewardConnection`. It uses `FindAreaExact` now, with the same three-way verdict the other
two use: the target area rewards, a different area penalizes, and no area at all is `undecided` —
neither, because a bot the mesh cannot place is not evidence about the connection. Undecided still
repaths, since the bot no longer knows where it stands relative to the plan.

**But the exact lookup has to be handed a constructed point, not `pEdict->v.origin`.** Switching to
`FindAreaExact` alone made every *correct* arrival report `undecided` and repath, for two independent
reasons that each hide the other:

- **The origin is in stand space, the mesh is in duck space.** Areas are built in hull 3, so their z
  range is duck-origin z — `BotNavZ` exists for exactly this conversion. That is harmless for an
  ordinary area, which spans floor to ceiling (`test18_elevator` area 1 is `z 18..110`, and a standing
  origin at 36 is well inside), and fatal for an elevator stop, which is a carved slab two units tall:
  `test18_elevator` area 7 is `16..18`, and crossfire's top stops 11, 154 and 188 are `-1520..-1518`.
  A standing bot's origin sits 18 units above that, past `kAabbEpsilon`, so the raw origin can
  *never* resolve a top stop. On crossfire the bot stands at `-1500`, `BotNavZ` is `-1518`, and the
  slab is `-1520..-1518`.
- **The ride ends up to `kElevatorArriveSlack` short.** On `test18_elevator` it ends at nav z 8.6
  against a stop at 16, and a two-unit slab does not reach down 7.4 units even with the AABB epsilon.

So the probe is the bot's XY at `rideArriveZ` whenever the residual is inside the slack, and at
`BotNavZ` otherwise. The snap condition is the arrival predicate reused: the slack that is allowed to
*end* the ride early is the slack that is allowed to *judge* it at the stop. A ride that stalled
mid-shaft is hundreds of units from `rideArriveZ`, gets no snap, resolves to the shaft area and is
penalized exactly as before. `test55_plat` is the map that pins both halves down: it resolves today
only by luck, because its stop area is `18..110` and the raw origin 28.5 happens to fall inside, while
its nav z at ride end is 10.5 and needs the snap to reach 18.

**The target area wins a boundary tie.** Stop slabs abut the shaft exactly — `test18_elevator` area 6
ends at `z 16` and area 7 begins at `16` — so a probe at the stop is inside both within
`kPlaneEpsilon`, and `FindAreaExact` returns whichever comes first in id order, which is the shaft.
`FinishElevatorRide` therefore asks `Nav::AreaContainsPoint(*target, probe)` first and only falls back
to the sweep when the answer is no. That is also the more honest question: the verdict being sought is
"did I arrive at the stop I wanted", not "which of two overlapping areas wins the iteration". A ride
that ended on the wrong floor is nowhere near the target in z, fails the test, and is penalized by the
fallback.

The same raw-origin probe is still used by `RunAirmoveFlight` and `FinishLadder`. Neither has a
reproducible failure: crossfire's two-unit stops carry only walk and elevator connections, no air
moves, and ladder tops are ordinary volumetric areas. The `undecided` lines those two still emit
(`test25_ladder2`, `test53_ladder4`) are genuine — a descent that ends mid-drop is in no area.

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
would otherwise repath from mid-shaft where `FindAreaNearest` finds nothing and the bot drops its goal.

The arrival test is `BotNavZ` against the arrival waypoint's own floor Z, directional with
`kElevatorArriveSlack`. The slack means the bot can step off up to 8 units short, and that is safe for a
reason worth stating: navgen only builds a walk seam off a stop when the step is under 18 units, so
anything within the slack is steppable by construction. On `test55_plat` the ride ends at nav 10 against
a stop at nav 18 and the bot walks out over the remainder. That same slack is what the landing
verdict snaps its probe by, above.

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

## bot_opportunist.cpp

The nav mesh is generated once at a fixed design speed and every connection's `costSeconds`,
`flightTime` and `exitSpeed` is baked at it. A bot that is *faster* than that — `sv_maxspeed` raised, or
holding `item_longjump`, which replaces a jump with `forward * 560` and `sqrt(2*800*56)` up — has a
movement option no plan on that mesh can contain. This checks, four times a second, whether taking it
right now beats walking the plan, and if so hops off-plan and repaths from wherever it lands. It is not a
planner change: the plan stays what it is and the bot occasionally short-circuits it. Can be disabled with
`crossbot_oppo_jump = 0`.

### Why the design speed had to go in the nav file

The trigger is "the bot is faster than the mesh assumes", and until version 16 nothing recorded what the
mesh assumed — navgen's `--maxspeed` was a command-line option that vanished after the run. The obvious
substitute, `kNavRunSpeed`, is the wrong number and fails loudly: it is 250, so a bot at the stock 320
already clears `1.25 x` and the check would arm on every map. `NavHeader::designMaxSpeed` is the number
navgen actually built with. The loader falls back to `kNavDefaultDesignSpeed` if it reads as zero, which
only happens for a file this field predates — and the version check rejects those anyway.

### One reverse search, not one per candidate

The comparison needs the cost-to-goal of two different positions, and needs it again every tick.
`Nav::AreaCostsToGoal` prices *every* area to the goal in one Dijkstra (`doc/nav.md`), so both lookups
are array indexes and the per-tick cost is a hull-traced `PmSim` run and nothing else.

The field is cached per bot and rebuilt when the goal area changes **or on any successful `Repath`**. The
second condition is the one that is easy to miss: `runtimePenalty` mutates edge costs between repaths
through the penalize/reward loop, so a field built before a penalty landed prices a route the planner has
already given up on. Rebuilding is cheap — this is the search with no `PlanWalker` and no `PmSim`, the
one `BotItems::SelectGoal` already runs at up to 2 Hz per bot.

### Comparing plan-seconds against real seconds

`AreaCostsToGoal` values are centroid-anchored and priced at `kNavRunSpeed`; `PmSimulator::Time()` is
real seconds. Subtracting one from the other directly is wrong in a way that always errs the same
direction: both cost terms are inflated relative to what the bot really achieves on the ground, the
flight is not, so the difference between them looks bigger than the flight it is being weighed against
and the bot jumps for gains that do not exist. The flight is converted into plan-seconds instead:

```
gain = C[here] - C[landing] - flightTime * (maxSpeed / kNavRunSpeed)
```

and has to clear `kMinGainSeconds` to fire, which absorbs the centroid noise both terms carry.

**The ratio is `maxSpeed`, not the bot's current speed**, and that distinction is worth more than it
looks. A long jump overwrites horizontal velocity with a flat 560 whatever the bot was doing, so a
launch at 68 u/s and a launch at 270 u/s produce the *same* 0.8 s flight — but scaling by instantaneous
speed charged the first one 0.22 plan-seconds and the second 0.86, making a jump look nearly free
precisely when the bot was too slow to be jumping. `maxSpeed` is what the cost field is being compared
against: the field prices ground travel at `kNavRunSpeed` while the bot actually cruises at `maxSpeed`,
and that ratio is a property of the map's physics, not of this frame.

No sub-area `|origin - centroid|` term is added to either side. Both endpoints price to the same goal, so
the centroid offsets largely cancel; the item-ranking formula in `bot_items.cpp` does add one, but it is
ranking *targets* against each other, not two routes to one target, and copying it here would charge the
bot for walking back to its own centroid. A landing in the area it started in is rejected outright rather
than priced: the two costs are then identical by construction and the flight is pure loss.

### Simulating a long jump without teaching PmSim about it

`PmSim` models `kJumpImpulse` and the commanded speed and knows nothing about the module. It does not
need to: `BeginAirborne` takes an explicit launch velocity, so the caller builds the one `PM_Jump` would
have produced — `kLongJumpSpeed` horizontally, `kLongJumpImpulse` up. That impulse is `sqrt(2*800*56)`
with the 800 **hardcoded in `pm_shared.c`**, not `sv_gravity`; a map with altered gravity changes the
arc's fall but not its launch. The arc is traced with the duck hull throughout, the same timeline
`JumpLandsIn` assumes and the same one the engine produces (see "IN_DUCK and IN_JUMP in one command").

The loop has to notice the bot leaving the ground before it can notice it landing. `Begin(..., jump)`
returns with `OnGround()` still true — the impulse is applied inside the first `Step` — so a
`while (!OnGround())` never runs a single frame. It tracks an `airborne` flag instead, and a sim that
never gets airborne is rejected rather than treated as an instant landing.

### The long jump leaves along the view, not along velocity

The plain jump and the long jump take their direction from different places, and the difference is not
cosmetic. `PM_Jump` keeps horizontal velocity for an ordinary jump, so simulating one from
`atan2(vel.y, vel.x)` is exact. The module branch instead *overwrites* it:

```c
for (i = 0; i < 2; i++)
    pmove->velocity[i] = pmove->forward[i] * PLAYER_LONGJUMP_SPEED * 1.6;
```

`pmove->forward` is `AngleVectors(cmd.viewangles + punchangle)` (`PM_CheckParamters`, then
`PM_PlayerMove`), so the whole horizontal launch is *where the bot is looking* and the velocity it had is
used for nothing but the `Length(velocity) > 50` gate. The comment sitting above that branch in
`pm_shared.c` — "UNDONE -- note this should be based on forward angles, not current velocity" — reads
like the opposite of what the code does, and simulating the velocity yaw makes the bot predict a landing
area it then flies nowhere near: navigation aim points at `waypointIndex + 1` and `ApplyLook` can drag it
onto an enemy, so the two yaws routinely disagree by tens of degrees.

`forward` is also not flattened. `forward[0..1]` is `cos(pitch) * (cos(yaw), sin(yaw))`, so pitch scales
the horizontal launch while `velocity[2]` stays the flat `sqrt(2*800*56)` — a bot looking 30° down long
jumps at 485, not 560.

**The launch frame is one `Bot_UpdateFacing` step away from the angles the check reads.** `TryJump` runs
before the follower's aim code and the command is issued by `RunAirmoveDrive` right after, which sets
`desiredAngles` to the launch yaw — so the yaw error is nulled, but residual `aim.yawVel` and drift still
move the view during that step, and pitch is dragged toward 0. Both are handled by refusing to launch
rather than by modelling them: `kMaxLaunchYawRate` skips the tick while the aim is slewing (it retries in
250 ms), and `kMaxLaunchPitch` keeps the `cos(pitch)` term inside a couple of percent of whatever the
step leaves behind. The oppo log prints the launch yaw and the velocity yaw side by side, so a hop that
misses can be read as "wrong prediction" or "aim moved" without guessing.

### Letting the command through, and what happens after

`Bot_RunPlayerMove` strips `IN_DUCK` from any `IN_JUMP` command while `slj` is set, which is exactly the
long jump this wants. The strip is now gated on an `allowLongJump` argument defaulting to false, so every
other call site sends byte-identical commands and only the opportunistic launch frame opts in.

The hop reuses the airmove flight machinery with `airmoveConnIndex = -1` and `airmoveOpportunistic` set.
That flag does three things at landing: it suppresses `AirmoveLandingMiss`, which measures distance to the
plan's next waypoint and is meaningless for a jump that deliberately left the plan; it forces a repath
unconditionally, since the landing is off-plan by construction; and a landing in an area other than the
predicted one starts a cooldown. Without that cooldown a bot whose sim disagrees with the engine retries
from the same spot at 4 Hz forever. There is no connection to grade, so the reward/penalty branch is
skipped — it already keys on `connIndex >= 0`.

## bot_avoid.cpp

Reactive obstacle handling, layered between the follower's steering decision and the engine call. The
nav mesh only knows world geometry and the brush entities navgen carves, so anything else solid — a
`cycler`, a monster, another player, a door that has swung out of its own doorway — is invisible to
the planner: it routes straight through and the follower drives into it and stalls. `BotAvoid::Update`
traces forward along the planned path each frame and answers with one of four moves: crouch-jump onto
the obstacle, duck under it, go around it, or — when none of those work and the thing can be destroyed
— break it. `BotAvoid` only ever *decides* to break; `bot_break.cpp` does it.

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

**The goal item is exempt too, and that one does need the edict.** `RouteExemptsHit` returns true for
`bot.goalItem`, before the `SOLID_BSP` test, because avoiding the thing you are deliberately walking up
to is never right. In practice it only fires for chargers — every other item is `SOLID_TRIGGER` and
never qualifies in the first place — and it only matters for the last stride, which is exactly where a
32-wide hull grazes the brush it is about to press `+use` against. Without it that graze costs a
forced repath, and the charger case turns out to be the one where a forced repath cannot help (below).

Note the carve set and the `FL_WORLDBRUSH` set are close but not identical. `func_wall`,
`func_conveyor` and `func_wall_toggle` are both carved by navgen and flagged, so the two filters agree.
`func_healthcharger` and `func_recharge` are carved but **not** flagged — `CWallHealth::Spawn` never
sets it — so the mesh routes around them while a trace still sees them. That asymmetry is deliberate:
the plan keeps its 16 units of clearance, and a bot that has drifted off the plan into one still gets
the detour search rather than pushing at it.

### Breaking through

Breaking is decided in two places, and they answer different questions.

**On the route.** The plan deliberately goes through the breakable: navgen split it into its own
`NAV_AREA_BREAKABLE` and the planner emitted a `requiresBreak` waypoint and charged `openSeconds` for
it. That is exactly the case `RouteExemptsHit` already skipped, so the avoid chain never ran at all and
the bot simply walked into the brush and stalled. `RouteExemptsHit` now reports *why* it matched
through a `wantsBreak` out-param, and a hit exempted for `requiresBreak` goes straight to
`BotBreak::Engage`. No duck, no jump, no side search — the planner has already decided and already paid.

**Off the route.** An unplanned breakable in the way, which is the ordinary avoidance path. Here
breaking is the *last* resort, after `canJump`, `canDuck` and `RunSideSearch` have all failed:
`TakeBreakTarget` sits on both non-receding failure paths in `RunSideSearch`, ahead of the
`searchFailed` / `blockedUntil` assignment. That ordering matters in both directions. A crate the bot
can hop onto gets hopped onto rather than smashed; and a breakable it genuinely cannot get past no
longer reaches `PenalizeCurrentConnection` and `BotItems::OnBlocked`, because it is about to stop being
in the way. A receding mover never gets here at all — `!recedes` guards both call sites.

Neither path sets `searchFailed`, so `avoid=break` is the only kind that survives the end-of-frame
stickiness test on its own terms rather than through `detourExpire` / `nextSearchTime`.

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
cross, then an immediate repath — **but only when the penalty actually lands.**
`PenalizeCurrentConnection` returns whether it found a connection to charge, and the repath is forced
only on true. When the plan is a single area there is no connection on the route, the penalty is a
no-op, and the replan is guaranteed to return the identical plan; forcing it is not merely wasted work
but actively harmful, because `TryRepath` resets `lastProgressTime` and the stall detector below can
then never fire. See "When there is nothing to reroute". `PlanWaypoint::connIndex` is only set on airmove launches, so the
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

### When there is nothing to reroute

A blocked search used to be able to livelock the bot outright, and the failure is worth understanding
because nothing about it looks wrong in isolation. On crossfire a bot printed this once a second,
forever, from a position that never changed:

```
2 avoid blocked by func_healthcharger at (464.1,368.0,-1500.0) no clear rejoin point closing 0 traces 0
2 mid-route repath (forced) at (464.1,368.0,-1500.0) wp 1/2
2 Replan from=(464.1,368.0,-1500.0) waypoints=2 areas=1 eta=0.4s | penalized=0
```

Read the three numbers together. `traces 0` says `PickDestination` gave up before probing any
geometry, which happens when every sample of the remaining path lies inside the obstacle's box padded
by `kAvoidHullHalf + kAvoidCornerMargin` — here the charger's padded box was `x 430…474, y 366…434`
and it contained the bot's own position and its goal. `areas=1` says the plan is a single area, so
`plannedConnPath` holds nothing `PenalizeCurrentConnection` can charge, which `penalized=0` confirms.
And the forced repath therefore rebuilt the same two waypoints. One second later, all of it again.

The trap is that the forced repath was also **suppressing the recovery**. `TryRepath` resets
`lastProgressTime`, the avoid failure repeats every `kAvoidSearchCooldown` (1 s), and the stall
detector needs `kRepathInterval` (2 s) of no progress — so the loop kept its own safety net
permanently one second from firing. A useless repath is not a no-op.

Two changes close it. The repath is conditional on the penalty landing (above), so `lastProgressTime`
is allowed to age. And when the stall does fire while the bot is still reporting blocked, the bot
gives up on the goal rather than repathing into the same plan: `BotItems::OnBlocked` prints what
stopped it, `NoteIgnored`s the item for `ItemUnreachableTimeout`, and clears the goal so `SelectGoal`
picks something else next frame. It is the same escalation `OnRepathFailed` already used for "no route
exists at all", which is the honest description of this situation too.

**The "still blocked" test uses its own timestamp, not `state.lastKind`.** The sticky window on
`lastKind` expires exactly at `nextSearchTime`, which with a 1 s cooldown and a 2 s stall is exactly
when the stall lands — a coin flip on frame timing. `state.blockedUntil`, set to
`now + 2 * kAvoidSearchCooldown` on every non-receding failure, covers the gap by construction.

A receding mover never reaches any of this: `hitRecedes` short-circuits before the penalty, so a bot
walking after a swinging door neither penalises nor gives up.

The root cause of that particular loop was upstream, in navgen — the charger was not in the mesh at
all, so the goal point was inside it. `doc/navgen.md` has that half. This section is about why the
runtime could not save itself from a bad goal, which is a property worth keeping fixed independently.

### Tripmine beams, which no trace can see

A tripmine beam is a `CBeam`, not geometry: it is `SOLID_NOT`, it has no bounding box, and the forward
hull trace goes straight through it. The mine detonates on a `FTRACE_SIMPLEBOX` traceline it casts
itself, from its own origin along its own normal, so what the bot has to avoid is a **line segment**
that nothing in the engine will report as a hit.

**The registry is shared and refreshed at 4 Hz**, in `BotManager::StartFrame` rather than per bot per
frame — sixteen bots times `maxEntities` every frame is the expensive way to learn something that
changes only when a mine is placed or destroyed. One entry per armed `monster_tripmine`: handle,
origin, and the beam end from a single 2048-unit traceline, mirroring `CTripmineGrenade::MakeBeam`.

Armed is `solid == SOLID_BBOX`, which the mine only becomes after its 2.5 s powerup — before that
there is no beam, and a bot that walks through the space is in no danger.

**The direction has to be un-negated.** The mine's angles come from `UTIL_VecToAngles(planeNormal)`,
whose pitch convention is the opposite of a view angle's, and `CTripmineGrenade::Spawn` recovers the
normal with `UTIL_MakeAimVectors`, which negates pitch before `MAKE_VECTORS`. The registry does the
same. Skipping that negation gives a beam that is mirrored vertically and mostly still plausible.

### The beam as an obstacle

`Probe` is the single choke point for "is this leg blocked, and by what", so teaching *it* about beams
makes `Solve`, `SolveAround` and the destination search beam-aware together. It marches the hull AABB
along the leg in 8-unit steps and slab-clips each beam segment against it; whichever of the solid hit
and the beam is nearer wins, and the beam's mine edict is reported as `tr.pHit` so the visited set and
every message keep working unchanged.

`SolveAround` and `PickDestination` take explicit bounds now, because the mine's own 16-unit box is
not the obstacle — the **beam's** bounding box is, and for a wall mount that box spans the doorway.

**The mine body and its beam must be one obstacle.** A ground-mounted mine sits 8 units off the floor,
so the hull trace hits the *body* first, and the body alone reads as a 16-tall box: under step height,
so no jump; no gap, so no duck; and `BotBreak::IsTarget` rejects `MOVETYPE_FLY`, so no break either.
The bot stood in a doorway repeating `blocked by monster_tripmine` forever. `BotDeploy::BeamFor` maps
a hit edict back to its beam, and both the forward trace and `Probe` use it, so hitting the body is
the same event as hitting the beam.

### Two predicates change, and both follow from the beam having no thickness

**The step-height floor is dropped.** `canJump` normally requires `top > kAvoidStepHeight`, because
below that the engine steps up for free. It does no such thing for a beam — a 12-unit-high beam trips
exactly like a 60-unit one — so for a beam the lower bound goes and only the reachability ceiling
remains.

**`depth` is the bot's own 32, not `min(box.x, box.y)`.** A wall-to-wall beam's AABB spans the
corridor, and feeding that width into the crawl-hold would ask the bot to stay ducked for the length
of the room.

The three outcomes then fall out of the existing ladder, with `gap` and `top` taken from the z range
of the beam *inside* the hull rather than from a box:

- a beam 40 or more above the feet clears the 36-tall duck hull, so it is **crawled under**;
- a beam within `PlainJumpHeight + 18` is **jumped**;
- a beam above 72 is not an obstacle at all and never enters the ladder;
- a **vertical** beam spans the whole column, so `gap` is 0 and `top` is 72, neither applies, and the
  side search **walks around** it. `RunSideSearch` already handles a thin tall column; for a
  horizontal beam it correctly fails, because the beam runs wall to wall.

Measured on crossfire with bot-placed mines: beams from a standing placement report `gap 61 top 69`
and are ducked; beams from a ducked placement report `gap 27 top 35` and are jumped. The 8-unit spread
between gap and top is `2 * kAvoidBeamPad`.

**When nothing works, shoot it.** Where the side search fails, `TakeMineTarget` sets `mineTarget`
instead of `breakTarget`, and `Bot_MoveNav` hands it to `BotBreak::EngageStandoff` at
`kMineSafeRadius`. `TakeBreakTarget` cannot serve here: it requires `IsTarget`, which excludes
`MOVETYPE_FLY` deliberately, and a crowbar at melee range against a 150-damage blast is not a plan.

**Own mines are not exempt, and must not be.** A tripmine's beam-break test traces from the mine, and
`pev->owner` is the wall it is stuck to, so the player who placed it breaks the beam like anyone else.
There is no owner filter anywhere in this path.

**Placed satchels get no code at all.** A live `monster_satchel` is 8 units tall, well under the
engine's 18-unit step, so the bot walks over it and `HitQualifies` is left alone.

## bot_break.cpp

Owns the frame while the bot is destroying something, the same way `RunElevatorRide` and
`RunChargerUse` do: aim, drive, buttons and `Bot_RunPlayerMove` all come from here, and `Bot_MoveNav`
returns. It is called *before* the progress/stall/repath block so a repath can never interrupt a swing,
and it refreshes `lastProgressTime` every frame so the 2 s stall detector and `BotItems::OnBlocked`
stay off its back — without that the bot would throw away the goal it is two swings from reaching.

### What counts as breakable

`IsTarget` is `SOLID_BSP` or `SOLID_BBOX`, `MOVETYPE_PUSH` or `MOVETYPE_PUSHSTEP`, not a player or
monster, `takedamage != DAMAGE_NO`, and `0 < health <= kBreakMaxHealth`. There is deliberately no
classname test: `takedamage` and `health` are the properties that actually decide it, and dropping the
classname picks up mod breakables for free. Nothing that is meant to be avoided qualifies — `func_wall`,
the chargers and the doors are all `DAMAGE_NO`.

`takedamage` is the exact inverse of `SF_BREAK_TRIGGER_ONLY`: `CBreakable::Spawn` sets `DAMAGE_NO` for
that flag and `DAMAGE_YES` otherwise, which is the same distinction navgen makes with
`kSfBreakTriggerOnly`.

**The movetype test is what makes `SOLID_BBOX` safe, and `SOLID_BBOX` is not optional.**
`func_pushable` spawns `SOLID_BBOX`, and with `SF_PUSH_BREAKABLE` (128) it runs `CBreakable::Spawn`
first, so it is `DAMAGE_YES` and the map author has explicitly said it may be smashed. Excluding it
wedges the bot: `test13_break` has two, and a bot that has just broken through the wall in front of
them meets them with `no clear rejoin point closing 0 traces 0` — the padded box contains the bot's own
position, exactly the shape of the charger livelock. But `SOLID_BBOX` alone would also admit live
satchel charges, which spawn `DAMAGE_YES` with `health` 1. `MOVETYPE_PUSH`/`MOVETYPE_PUSHSTEP` is the
line between a brush entity and a thrown prop: grenades and satchels are `MOVETYPE_BOUNCE`, tripmines
`MOVETYPE_FLY`, monsters `MOVETYPE_STEP`.

`health <= 100` is the request, and the numbers back it: `CBreakable::TakeDamage` doubles `DMG_CLUB`,
so a crowbar lands `2 * plrDmgCrowbar` = 50 per swing, always the full-damage branch under
`IsMultiplayer()`. A qualifying target dies in at most two swings, which is why no interaction with
combat is needed — the whole engagement is under a second.

**`takedamage != DAMAGE_NO` is necessary but not sufficient, and nothing can make it sufficient.**
`TakeDamage` early-returns on `!IsBreakable()`, so `matUnbreakableGlass` keeps `DAMAGE_YES` and
silently absorbs everything. `m_Material` is private data with no gamedata offset, so this cannot be
tested up front. It is caught instead by watching `pev->health` fail to drop: after the first swing,
`kBreakNoDamageGrace` seconds without the health moving aborts the attempt.

### Giving up

`Abort` fires on the no-damage grace or on `kBreakTimeout`, and does three things beyond restoring the
weapon. It records the entity in `refused` for `kBreakRefuseTime` so the next frame does not re-engage
the same brush. It calls `Bot_PenalizeCurrentConnection`, so the connection through the breakable gets
more expensive and the planner routes around it. And it forces a repath to spend that penalty
immediately. This is the whole answer to breakables navgen cannot classify offline — unbreakable glass
and anything over 100 health are discovered once, at runtime, and then avoided.

### The weapon

`IGameProfile::MeleeWeapon()` names it; HLDM returns the crowbar. If the bot is not carrying it the
active weapon is used instead and the bot shoots the thing, which is worse for ammo but never wrong.
The switch is `FakeCmd::Run(pEdict, spec->classname)` rate-limited to `kBreakSelectInterval`, the same
mechanism and the same interval `BotFire::Think` uses. `prevWeaponId` is captured at `Engage` and
redrawn on both `Finish` and `Abort`; if the weapon is gone by then nothing happens and `BotFire`
re-picks within half a second anyway.

**`BotFire::Think` early-outs while a break is active**, and it has to. `Bot_RunPlayerMove`
unconditionally ORs in `BotFire::Buttons`, and `Think` re-rolls a weapon at random every half second
whenever it has a target — it would fight the melee switch for as long as the bot could see anyone.
Idling it leaves `fire.mode == MODE_NONE`, so `Buttons` returns 0 and `BotBreak` is the only source of
`IN_ATTACK`.

### Aim and reach

`CCrowbar::Swing` traces from `GetGunPosition()` along `v_forward` for 32 units, so reach is measured
**eye to surface**, not eye to centre. The aim point is the eye clamped into the entity's
`absmin`/`absmax` and then nudged `kBreakNudge` toward the centre, so the traceline ends inside the
brush rather than grazing its face. A bot whose eye is already inside the box — which happens, the
hull overlaps before the trace does — falls back to aiming at the centre.

### The standoff variant

`EngageStandoff(bot, ent, minRange)` is the same machine with the approach inverted, and it exists for
tripmine beams (`bot_deploy.cpp`). It shares `Run`, the weapon save/restore, the `lastProgressTime`
hold and the `Abort` path; the differences are four.

**It bypasses `IsTarget`.** A tripmine is `MOVETYPE_FLY`, which `IsTarget` excludes on purpose — that
exclusion is what keeps live satchels out of the melee path, and widening it would undo that. The
standoff caller names its target explicitly instead.

**`inRange` inverts.** Melee wants `los <= kBreakReach`; standoff wants `los >= standoff`, and the
drive direction flips with it, so `moveDir` is negated and the bot backs away along the line it came
in on. `kMineSafeRadius` is 375 because that is where a 150-damage blast reaches zero — the bot is
retreating out of its own explosion, not merely out of reach.

**A corridor shorter than the standoff must not deadlock.** `kBreakRetreatTime` (4 s) forces `inRange`
true once the retreat has had its chance, so a bot backed against a wall shoots anyway and eats some
blast. That is strictly better than standing there. On `test2_doorway` it is the only case — the map
is smaller than 375 units end to end, so every observed standoff fires on the timer and still kills
the mine in one shot.

**The weapon choice inverts too.** `WantWeaponId` returns the melee weapon for a break; for a standoff
it returns the active weapon unless that *is* the melee weapon, in which case it takes the first
non-melee weapon carried. A crowbar at 375 units hits nothing.

A tripmine is `health == 1` and `takedamage == DAMAGE_YES`, so one hit anywhere destroys it, and
`Killed` -> `DelayDeathThink` -> `Explode` is the expected outcome rather than a failure. The
no-damage grace still applies and still means what it means: shots that are not landing.

## bot_deploy.cpp

Placing tripmines and satchels, and detonating satchels. It owns the frame the way `BotBreak` does —
aim, drive, buttons and `Bot_RunPlayerMove` all come from here and `Bot_MoveNav` returns — and
`BotFire::Think` idles with `why=deploying` while it runs, for the same reason it idles during a
break: `Buttons` would otherwise fight the weapon switch every half second.

It is called from `Bot_MoveNav` immediately after `BotBreak::Run`, before goal selection.
`Bot_ManeuverActive` includes `deploy.active`, which is what stops `TryStart` re-entering and what
keeps `BotCombat::Run` from tearing the mine placement out from under it.

### Why the deployables never fit in BotFire

`BotFire::Think` returns `no-target` several branches before any mode is considered, and both
deployables fire with **no target at all**. "What is not here" named this: they want a decision layer
that runs on the map rather than on an enemy. That layer is this file. The `Supported` gate was never
widened — `AIM_SURFACE`, `AIM_NONE` and `WPN_Q_HIDDEN_STATE` are still rejected there, and nothing
about firing changed.

### The damage rule is one number, and it comes from the SDK

`sk_plr_tripmine` and `sk_plr_satchel` are both 150. `CGrenade::Explode` calls
`RadiusDamage(pev->dmg, pev->dmg * 2.5)`, so the radius is 375 and the falloff is `150/375 = 0.4` per
unit: damage at distance `d` is `150 - 0.4d`. Two constants fall out and both are load-bearing.

- **`kDeployKillRadius` is 125** — where the blast still does 100. This is the whole meaning of "a gate
  small enough that anyone in it dies", and `Covers` applies it directly rather than through a width
  threshold, sampling the gate cross-section at three points along the span and three body-centre
  heights and requiring every sample within 125 *with line of sight to the mount*.
- **`kMineSafeRadius` is 375** — where it does nothing. That is the standoff range in `bot_break.cpp`.

### The mesh already knows where the chokepoints are

Every walk connection carries `gateA`/`gateB`, a two-endpoint floor segment per side, and they survive
the `.nav` round trip. **No format change and no map regeneration was needed for any of this.** The
gate supplies the crossing point and the beam direction; it does not supply a usable width.

**The stored gate is inset twice, and the second inset is easy to miss.** navgen builds areas by
carving the *hull 3* clipnode tree, so area boundaries already sit a player radius inside the real
walls; `GateFromEdge` then insets each end by `kPlanClearance` again. So the true opening is
`|left - right| + 4 * kPlanClearance`, not `+ 2`. Getting this wrong is invisible — every number is
still plausible — which is why the physical measurement below is what the code actually trusts.

### A short gate is not a chokepoint

The first version accepted a gate on `|left - right|` alone and found **1238 of them on crossfire**.
Two mesh seams across an open floor produce a short gate for the same reason two areas are small, and
a mine there traps nobody. Worse, the ceiling/ground fallback rescued every one of them: there is
always a floor below you.

`EvaluateGate` measures the opening instead. From the eye at the gate midpoint it traces both ways
along the gate direction; **both sides must hit something** within `kDeployReach`, and
`opening = distA + distB` must be at most `kDeployOpeningMax` (144). Crossfire drops to 99, and the
opening histogram is 96 and 128 — Half-Life's two standard doorway widths, which is the sanity check
that matters more than the count.

### The gate direction, and the clamped case that had none

For an ordinary gate the direction is `normalize(right - left)`. A **clamped** gate — one whose raw
opening was under 32, so `GateFromEdge` collapsed it to a single point — has `left == right` and no
direction at all.

Guessing it from the two areas' centroids is wrong often enough to matter: on `test2_doorway` every
gate is clamped, and the centroid guess measured the openings diagonally as 113 when they are 64, then
rejected all four for `no-surface`. **The far side's gate midpoint is the right source.** `gateA` and
`gateB` are the two sides of the same portal, so `gateB.mid - gateA.mid` is the crossing normal by
construction, and the gate direction is its horizontal perpendicular. Centroids remain the last
fallback. This was not only a count fix — it fixed the measurement, and it raised every map
(`stalkyard` 93 -> 114, `datacore` 91 -> 118).

### Where the mine goes, and why the wall comes first

`CTripmine::PrimaryAttack` traces 128 units from the gun position along `v_forward`, spawns
`monster_tripmine` at `tr.vecEndPos + tr.vecPlaneNormal * 8`, and sets its angles from the plane
normal — so **the beam runs back out along the surface normal**. A beam that crosses a doorway
therefore needs a mount on a *side* wall, one whose normal lies in the gate plane. That is what
"a wall with normal parallel to the gate" means, and `Probe` enforces it: the hit's normal must be
within `kDeployNormalTol` (~20 degrees) of facing back down the trace.

Preference is wall, then ceiling, then ground, and the vertical two are gated behind
`opening <= kDeployVerticalMax` (48). The gate is the point: a vertical beam is a line, and in
anything wider than about a player's width people simply walk past it. Forced open, ground mounts in
crossfire's 128-wide doorways were stepped around without the avoidance system ever engaging.

The bot stands **at the gate midpoint**, and `EvaluateGate` probes from exactly that point. Evaluating
from anywhere else stops the pre-scan predicting what the shot will do.

### Stance is rolled, and that is what makes the avoidance visible

Standing, the eye is 64 above the floor; ducked it is 30 — `VEC_DUCK_VIEW` is 12 above a duck origin
that sits 18 up. The mine mounts at eye height, so the beam does too, and the two stances land on
opposite sides of every avoidance predicate: a beam at 64 is crawled under, a beam at 30 is jumped.
`TryStart` flips a per-bot coin. Without it every horizontal beam is a crawl-under and the jump branch
never runs outside a test map.

Nav areas are in duck-origin space, so the gate's own z is `floor + 18` and the eye offsets from it
are 46 and 12 rather than 64 and 30.

### The phases, and the one that is not optional

`APPROACH` -> `SELECT` -> `AIM` -> `FIRE` -> `VERIFY`, plus `DETONATE` entered from its own trigger.
The chassis is `RunActivation` almost verbatim, including the three details that are easy to lose:
`lastProgressTime` is refreshed every frame or the 2 s stall detector repaths mid-placement; the press
is gated on the angles `Bot_UpdateFacing` *returned* rather than the ones requested, because the aim
controller is rate-limited; and the approach drive is `NavFollow::SolveDrive` + `Bot_DecomposeMove`
straight at the spot. Straight-line drive is safe because an area is a convex prism and the gate
midpoint lies on its boundary.

`VERIFY` asks a different question per weapon, and the obvious one is wrong for the satchel. A
tripmine is confirmed by a `monster_tripmine` within 48 of the predicted mount. A satchel is **not**:
it leaves at 274 u/s and is 80 units away before the 0.3 s verify delay is up. It is confirmed by
`m_chargeReady == 1`, which is the game's own record that a satchel is deployed.

**The memo is not optional.** Every exit writes `Memo { connIndex, expireTime }`, success or failure,
and `FindBest` honours it. Without it the bot re-picks the gate it just failed at, walks zero
distance, and spins at full framerate exactly where it wants to be — the failure mode described for
`BotItems::OnArrived`, and no stall detector catches this one either.

### Satchels are dropped, not lobbed

The satchel shares the whole chassis with the surface search skipped: the spot is the gate midpoint,
the coverage test runs from the floor rest point, and the press is `IN_ATTACK2`.
`CSatchel::SecondaryAttack` throws for any `m_chargeReady != 2`, so the throw needs no state knowledge
— only the detonation does. Tripmine is preferred; a satchel is placed only when there is no tripmine
ammo, and `CSatchel::AddDuplicate` refuses new satchels while any are deployed, so the live group is
self-limiting.

### Detonation, and the contact that was the bot's own satchel

The trigger runs every frame in `TryStart`, ahead of the placement throttle: for each live
`monster_satchel` the bot owns, fire when a live suspicion sits within 375 of it **and** closer to it
than the bot is, out of combat and with no fire commit open.

`BotSense` had no spatial query — `Freshest` and `LiveCounts` have no position filter — so
`NearestLive` was added beside them, running the same `[0, count)` scan through the file-static
`LiveWeight`, keeping the file's one-liveness-test rule.

**`livingOnly` is the whole feature.** Without it the first run detonated 181 times off 7 satchels,
every one reporting `contact 2 away`: `monster_satchel` plays `weapons/g_bounce*.wav` when it lands,
`BotSense::Hear` pushes a suspicion for the emitter, and the bot blew up its own charge because it
heard it hit the floor. `NearestLive` now skips `KIND_OTHER` and skips any entity owned by the bot.
"An entity near the satchel" has to mean something that can be killed.

**The press must be confirmed, not assumed.** The same run counted a detonation on the frame it
pressed and then re-fired forever. `DETONATE` now presses only when `m_chargeReady == 1` and both the
player's and the weapon's own attack timers are clear, then waits `kDeployVerifyDelay` and calls it
done only if `m_chargeReady` has left 1.

### crossbot_deploy_surface

`0` is off; `1`/`2`/`3` pin wall/ceiling/ground and bypass the `kDeployVerticalMax` gate. It exists for
the same reason `crossbot_fire_pin` does: vertical mounts are rare by design — a 48-unit opening is
uncommon, and across six stock maps the unpinned scan finds none — so the side-search and standoff
branches are unreachable in a normal run and would ship untested. Pinned to ground on `test2_doorway`
they are reached every time.

### crossbot_list_gates and crossbot_deploy_status

`crossbot_list_gates` dumps every area's walk gates with the mesh width, the measured opening, the
chosen surface, the mount point and the rejection reason. It prints a gate that reached the surface or
coverage stage even when it failed, and stays quiet about ones filtered earlier, so the interesting
rejections are visible without burying them.

`crossbot_deploy_status` is per bot: phase, target connection, measured opening, surface, stance,
`placed`, `dets`, `aborts`, memo size and `why`. Unlike the fire counters these are **not** per life —
`OnRespawn` clears the memo and the active placement but leaves the tallies alone.

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
- `tier=` with `stopTier=`, `detour=`, `target=` and `targetArea=` — the goal hierarchy. `tier` is the
  distant target's tier, `stopTier` the current leg's, and they differ exactly when `detour=1`: a bot
  reading `tier=fun stopTier=gather detour=1` is on its way to a gimmick and picking something up en
  route, which is the intended shape and not a confusion. `tier` changing while `detour=1` holds is
  the broken-stickiness signature (`bot_goal.cpp`).
- `fun=` with `funKind=`, `funAct=`, `mounted=` and `shots=` — present only while the fun tier owns
  the goal. `mounted=1` with a nonzero `move=` is the 30-unit leash about to break.
- `combat=` with `enemy=`, `enemyAge=` and `ready=` — whether `BotCombat` owns the bot, who sense
  remembers, how stale the sighting is, and the bot's `EvaluateCombat` score. `combat=0` next to a
  live `enemy=` is normal and not a contradiction: sense remembers someone the bot never saw well
  enough to fight, or combat already exited on one of its own rules while the suspicion lingers.
  `ready=` no longer selects anything — it survives because carefulness still routes by it.
  `enemyAge=` above `crossbot_sense_decay` should never be seen; the record is dropped there.
  `crossbot_combat_status` is the fuller view.
- `breaking=` with `health=`, `swings=` and `for=` — present only while `BotBreak` owns the frame.
  `health` not falling across two samples while `swings` climbs is the unbreakable-glass signature; the
  no-damage grace will abort it a moment later.
- `avoid=` with `hit=` — the last obstacle decision (`duck`, `jump`, `side`, `blocked`, `break`) and the
  classname it was made against, plus `duckLeft` on the crawl latch, the `gap`/`top` the decision was
  made from, and `detour=` for how many spliced detour waypoints are still ahead. `avoid=none hit=-`
  on a bot stalled against something means the trace never qualified a hit, which is a different bug
  from a mistimed maneuver; `avoid=blocked` means it qualified and no route past it was found. Each
  new decision also prints its own line to the server console with distance, speed and trace count,
  which is what makes this tunable from one run — note those lines go to `qconsole.log`, not to the
  RCON transcript the test scripts capture.

## bot_perf.cpp

A nested wall-clock zone profiler, compiled out unless `CROSSBOT_PERF` is defined. It exists because
neither of the obvious alternatives answers the question "which frame was slow and why":

**A sampling profiler cannot see into the plugin.** AMD uProf attributes work by module fine --
`swds.dll` 76%, `crossbot_mm.dll` 7%, `ntoskrnl` 7% at 32 bots on crossfire -- but the call graph
dead-ends. `swds.dll` ships without symbols and FPO unwinding fails there, so every engine leaf hangs
off one address in `hlds.exe`'s main loop and none of the engine time attributes back to the plugin
call that caused it. Since most of what crossbot costs *is* engine time it asked for, the module split
alone reads as "the engine is slow".

**CPU sampling under-counts stalls.** A blocking `WriteFile` on the server's stdout is off-CPU, so it
barely samples, and console logging measured at a fifth of all crossbot time was nearly invisible to
uProf. Wall-clock zones charge it.

`Enter`/`Leave` push a `StackEntry` and subtract child time from the parent, so a zone reports both
inclusive and self time; `BOTPERF_SCOPE(ZONE_X)` is the RAII form and is what most call sites use.
Depth is capped at 24 with no dynamic allocation on the timing path.

### Trace attribution needs the plugin's own table, not a metamod hook

Metamod's `GetEngineFunctions` hooks the *game DLL's* calls into the engine. A plugin's `g_engfuncs` is
a plain `memcpy` of the real engine table taken in `GiveFnptrsToDll` (`h_export.cpp`), so crossbot's own
`TRACE_LINE` never passes through metamod at all. The first version of this counted zero traces inside
`sn_los`, a zone that is nothing but traces.

`InstallTraceWrappers` fixes it by saving the four trace pointers out of the plugin's own `g_engfuncs`
and overwriting them with wrappers. The metamod hooks in `engine_hooks.cpp` stay -- they catch hl.dll's
traces, which is how the `runmove` zone accounts for the engine's own movement traces -- and the two
paths cannot double count because they are different function tables.

A trace is charged to whichever zone is on top of the stack when it starts, along with its ray length
and whether it clips entities, which is what makes a per-purpose breakdown possible. Only the outermost
trace is timed (`g_traceDepth`), so a wrapper calling into another cannot double count.

### Reporting

`crossbot_perf <seconds>` starts a window and reports every N seconds; `crossbot_perf_spike <ms>` logs
any frame over that threshold with its worst bot and its four heaviest zones, capped at 8 per window so
a bad window cannot itself become the bottleneck. The window keeps every frame's duration for
percentiles and a full zone snapshot of the single worst frame. Both cvars only exist in a
`CROSSBOT_PERF` build.

## bot_log.cpp

One function, `BotLog::Verbosef`, reached only through the `BOT_VLOG` macro in `bot_log.h`. With
`CROSSBOT_VERBOSE` defined the macro forwards to it; without, it expands to `((void)0)` and the
compiler drops the call, the arguments and every expression that only fed them.

### Why the logging is compiled out rather than gated at runtime

A `SERVER_PRINT` on this harness costs roughly 700us -- the server's stdout is a redirected pipe and
the write blocks (`profile_hlds.ps1` in `doc/testing.md` measures this). `Repath` emitted one print per
connection *and* one per waypoint, so a single 20-waypoint route paid about 40 of them. That showed up
in the profiler as the `rp_log` zone at 11--15ms per call, and it was the largest single contributor to
the worst frames: one 115ms spike on crossfire was `rp_log=56.01ms` over 5 calls, another 105ms frame
was `rp_log=45.85ms` over 3.

A cvar would not have fixed it. The cost is in the `snprintf` fan-out and the blocking write, both of
which happen before any runtime check could help unless every call site were individually guarded --
which is what the macro does, for free, at compile time. Measured at 32 bots, 60s windows, interleaved
in one sitting:

| map | verbose | frames | p99 | max |
| --- | --- | ---: | ---: | ---: |
| crossfire | on | 926 | 186.6ms | 273.9ms |
| crossfire | off | 1108 | 119.3ms | 171.6ms |
| crossfire | on | 796 | 209.7ms | 293.2ms |
| xen_dm | on | 608 | 403.9ms | 614.9ms |
| xen_dm | off | 1010 | 259.7ms | 473.3ms |

`rp_log` falls to 0.0ms and stops appearing in any spike breakdown.

### What is verbose and what is not

Verbose means **per-bot narration**: what a bot is doing and why. All of `bot_move_nav.cpp` bar two
exceptions, the item narration in `bot_items.cpp`, and the touch dump in `bot_manager.cpp`.

Four categories deliberately stay unconditional, because something other than a human reading the
console depends on them:

- **Console command output.** Everything in `commands.cpp`, and the `Print*` handlers in
  `bot_manager.cpp`, `bot_items.cpp` and `bot_deploy.cpp` that they call. These are answers to an rcon
  question; a build that answered nothing would be broken, and they cost nothing when nobody asks.
- **Server lifecycle and fatal errors.** `gamedata.cpp`, `AddBot`, `ServerActivate`, `nav_loader.cpp`.
- **Event lines the test suite asserts on.** `reached goal` in `bot_move_nav.cpp` is
  `nav_target_test.ps1`'s primary pass signal; `test_deploy.ps1` counts `placed a mine`, `detonated a
  satchel`, `avoid jump`, `is shooting`, `destroyed`, `blocked by` and `reached goal`. That is why all
  of `bot_avoid.cpp`, `bot_break.cpp` and `bot_deploy.cpp` stay as they are. They are event-level, not
  per-frame, so they were never the problem.
- **`WriteTraceLog`.** Once per goal, and `test_nav_goal.ps1` reads the file it names.

The split is not "cheap vs expensive" but **"who reads it"**: a human debugging one bot, or a script.

### The flag defaults on

```
build.bat -DCROSSBOT_VERBOSE=OFF
```

turns it off; the value caches in `build/CMakeCache.txt`, so plain `build.bat` keeps whatever was last
set and `-DCROSSBOT_VERBOSE=ON` puts it back. CMake prints `crossbot: verbose bot logging DISABLED`
when it is off, which is the only way to tell from the build output.

On by default is the deliberate choice: these lines are how nav work gets debugged, and most of
`doc/nav.md` and this file quote them. The flag exists so a profiling or saturated-server run can drop
them, not so the normal build goes quiet. Pair it with `-DCROSSBOT_PERF=ON` for a profiling build.

Two cvars follow the flag rather than surviving it: `crossbot_gather_debug` (both the item scan dump
and the touch dump) reports disabled in a non-verbose build instead of doing the work and printing
nothing.

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

`bot_fire.cpp` is that consumer and it decided: retail Half-Life is the countdown case, so ready is
`<= 0` and `nextAttack > 0` means nothing will fire this frame. It never needs the absolute form,
because the only weapon it presses is one it is already holding.

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

Which of a weapon's two modes those numbers belong to, and what the bot is supposed to do with the
button, is not here — that is `weapon_spec.h` in `doc/game.md`. This file answers "what am I holding",
that one answers "how is it operated".

## bot_move.h

### Bot_DecomposeMove
Splits a world-space direction into the forward/side speeds pfnRunPlayerMove takes, which the engine reads along the view axes -- so this is what lets the bot move somewhere other than where it is looking. viewYawDeg must be the yaw passed to pfnRunPlayerMove in that same call plus punchangle, because PM_CheckParamters builds the movement basis from exactly that.

PM_CheckParamters clamps the pair down to maxspeed before anything reads it, so a driveLength well over maxspeed just means "full speed exactly this way", and one below it passes through untouched and becomes the actual speed.

## Trigger Forces: gravity and push

### trigger_gravity

A brush entity (`CBaseTrigger`, classname `trigger_gravity`) that changes the player's gravity multiplier when touched. The bot's `pev->gravity` field stores this multiplier as of the last touch. It persists until the next respawn, and it decides which air moves in the mesh are usable.

When the bot spawns, the game DLL resets `pev->gravity` to 0 (default). Any trigger_gravity has to be touched again to take effect. Because `trigger_gravity` sticks to whoever touches it, a bot only ever runs at one of these multipliers at a time — the default (gravity 800) plus one per distinct trigger_gravity on the map.

This affects routing: `trigger_gravity` multipliers filter air moves the same way `PlanWalker::Cross` does during planning, so goal selection and route search agree about which jumps exist. Only default-gravity air moves count as an alternative route during pruning; a jump that exists solely at some `trigger_gravity` value is not a route the bot can generally rely on.

During navgen sampling, a full air move set is sampled per multiplier found on the map. At plan time, only the air moves baked for the gravity we are currently at exist for us. This means the runtime's route search honors the gravity the bot is actually under.

### trigger_push

A brush entity (`CBaseTrigger`, classname `trigger_push`) that provides a base velocity to the player for as long as it is being touched. Unlike trigger_gravity, it does not persist after leaving the field.

`trigger_push` hands the player a base velocity that gets added to the movement simulation. The bot snapshots the live fields once per plan; a plan runs long enough that a toggled field is caught by the next repath.

In navigation, `trigger_push` is `SOLID_TRIGGER`: it never blocks, carves or supports the nav mesh. It only shows up in the movement simulation during air move sampling and as a launch seed.

### Physics integration

The base velocity contributed by trigger_push fields overlapping the bot's origin is added at plan time. In the physics simulation (`pm_sim.cpp`), `PM_AddCorrectGravity` folds the vertical push component into velocity as an acceleration and clears it from the base velocity, so only the horizontal remainder wraps `FlyMove`. The horizontal base velocity wraps the move the way `PM_AirMove` and `PM_WalkMove` do: added before the clip-and-slide loop, pulled back out once the ground check has seen it.

During navgen air move sampling, the trigger touch test uses the same point-test of the player origin against trigger hull-expanded clipnodes that the engine uses in `SV_TouchLinks`.

### START_OFF fields

A trigger_push with the `START_OFF` spawnflag (1024) spawns as `SOLID_NOT`, so nothing touches it until something turns it on via trigger.

### Push field seeding

A push field sitting on flat floor has no perimeter edge with void ahead, so the perimeter sweep never launches there. For these fields, the bot's own footprint boundary is seeded instead, aimed inwards at the field. This accounts for a bot running into the shaft and jumping.

Push-field seeds have to launch at the boundary: backing up first only spends the boost.

### Preview path at lowest gravity

A bot only ever holds one gravity multiplier. The lowest one the map hands out is the one that unlocks the most air moves. During offline path preview (`--dump-path`), navgen samples at the lowest gravity multiplier found, so a map whose route exists only under a trigger_gravity does not read as unroutable offline.
