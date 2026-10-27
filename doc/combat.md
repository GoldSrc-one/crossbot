# Combat: fighting and firing

How a bot fights: the combat state, the fire chain that picks a weapon and pulls the trigger, TFC
off-hand grenades, and the inventory reader all of it stands on. Enemies come from
`doc/perception.md`, what combat interrupts is `doc/goals.md`, and what the weapon table's fields mean
is `doc/games.md`, "weapon_spec.h".

| file | owns |
|---|---|
| `src/bot/bot_combat.cpp` | `BotCombat`: entering and leaving combat, the shuffle, the ambush and duel flags |
| `src/bot/bot_fire.cpp` | `BotFire`: weapon selection, aiming solves, the trigger, the brace |
| `src/bot/bot_grenade.cpp` | `BotGrenade`: TFC off-hand grenades |
| `src/bot/bot_weapons.cpp` | `BotWeapons`: reading the inventory out of private data |

**Frame order.** `Bot_MoveNav` runs `BotSense::Update`, then `BotFire::Think`, then
`BotCombat::Update`, then the maneuver bail, then the frame owners in order (ability, break, deploy,
fun, `BotCombat::Run`, `BotCamp::Run`). Inside `Bot_RunPlayerMove` come `BotFire::Buttons`,
`BotGrenade::Run` and `BotFire::Brace`. Weapon choice therefore already sits between perception and
aiming: `Think` publishes an aim point that every later `ApplyLook` prefers.

## The combat state (bot_combat.cpp)

`BotCombat` is the single owner of "is this bot fighting someone". `Update` makes every entry and
exit; `Run` is the behaviour, and it is what stops the path follower. No other file decides whether a
bot is fighting, and perception never resets the path.

A bot in a fight does a local shuffle, not a chase. The last-seen position is not a goal; a fighting
bot should be sidestepping, not commuting across the map.

### One way in, six ways out

**Entry is sight only** (`sense.target`, line of sight this frame).

Damage does not enter combat. It pushes a `SUSP_DAMAGE` suspicion, which skips the vantage cone and
wins the look at full authority (`doc/perception.md`). The bot turns, the attacker fills the notice
meter quickly (being near the crosshair), and the fight opens on sight with `EnemySees`. Entering on
damage skipped noticing and reaction entirely.

`DamageSource` still decides who hit us, for the `shot at us` log, the maneuver bail, the tank
dismount and (as `Attacker`) the teammate-hurt suspicion. Its job is mostly rejection:

- `pev->dmg_inflictor` is the player for a bullet but the projectile for a rocket or grenade, so a
  non-player inflictor is resolved through its `pev->owner`.
- That resolution drops everything not worth fighting by itself: falling, drowning and crush damage
  come from worldspawn (edict 0), a `trigger_hurt` is a brush no player owns, and a rocket fired at the
  bot's own feet resolves to the bot.
- A non-player inflictor is accepted when `MonsterEnemy` names this bot: a snark bite counts like a
  bullet, with no owner hop since the snark is the inflictor.
- `Teams::Attackable` is checked on both the monster branch and the resolved player. Friendly fire is
  off on every team mod, so it should never fire, but the bail and the tank dismount act on it without
  vision and must not rest on a server setting nothing here reads.

`Attacker` is also read one last time on the death frame, by `BotGoal::NoteDeath`, to seed the revenge
goal (`doc/goals.md`, "Revenge"): `dmg_inflictor`/`dmg_take` still hold the killing blow in the `!wasDead`
block of `ThinkOneBot`, so the killer, its eye and its view angles are captured there — later the edict may
be gone. Revenge only latches when that killer is the enemy the bot was already tracking (`Enemy`).

**Exits**, checked every frame:

| Rule | Reason string |
| --- | --- |
| `enemy.Get()` is null | `the enemy handle went stale` |
| `IGameProfile::IsDead` | `the enemy died` |
| the enemy is on our team | `the enemy is on our team` |
| a monster enemy's `m_hEnemy` is no longer us (ignored for an enemy-team monster, matching `MonsterCandidate`) | `the monster stopped hunting us` |
| `BotSense::Senses` finds nothing | `lost every trace of the enemy` |
| 5 s without line of sight (`kSightTimeout`) | `went too long without line of sight` |
| the bot died | `respawned`, from `BotManager` |

- The team exit exists because combat persists on `Senses` (sound, damage, events), not on sight;
  hiding a teammate from vision alone would leave a fight open for the whole decay window if someone
  changed team mid-fight. It is checked before the monster exit.
- The monster exit exists because a snark re-targets about every two seconds; without it the bot would
  duel something that lost interest for the full sight timeout.
- On stock settings the `Senses` and sight rules overlap, since suspicions expire on
  `crossbot_sense_decay`, also 5 s. That is deliberate: the sight rule is a fixed promise about combat,
  not a second name for the sense decay. Raising the cvar makes the sight rule authoritative.

**A new opponent retargets rather than re-entering.** Exiting and re-entering would discard the probe
and re-drop a goal the bot already gave up; it is the same fight.

**Monsters have already noticed us.** `EnemySees` returns true for any non-`FL_CLIENT` enemy: a
monster whose `m_hEnemy` is us knows where we are, so `aware` is 1 and there is no ambush. It also
must not read `v.v_angle` or `v.fov` off a monster.

### The takeover waits for maneuvers

`Run` returns false while `Bot_ManeuverActive` (airmove, ladder, elevator ride, activation, charger,
break, deploy). Those runners steer along the waypoint list below goal selection, and clearing the goal
mid-flight would drop the bot into whatever is under it. The state is still entered on time;
`driving` records whether the movement half has started. `Run` also yields with a reason when
`BotDiff::CombatInterrupt()` is off (`doc/difficulty.md`) or when `BotFun::CombatTank` takes the bot to
a gun.

Two maneuvers are cut short instead, and only when the bot is actually hit: a frame where
`CombatInterrupt()` holds and `DamageSource` names an attacker calls `Bot_BailManeuver` from
`Bot_MoveNav`, whether or not combat is active (a bot shot on a ladder may have seen nobody). A ladder
ejects unless the drop would be fatal (`doc/movement.md`, "Ladders"), and a charger is walked away from
(`doc/movement.md`, "RunChargerUse"). Airmoves, rides and activations still wait: there is nowhere to
eject to mid-flight. Being seen is not enough; a two-second climb is worth finishing when nobody is
shooting yet. The call lives in `Bot_MoveNav` so combat never learns what a ladder is.

The first frame `Run` drives, it calls `BotGoal::Clear` once: goal cleared, waypoints gone. On exit,
`bot.nextSelectTime` is zeroed so goal selection picks something new next frame instead of idling out
the retry interval.

### The shuffle: one random direction, judged on exposure alone

Every `kProbeInterval` (0.7 s), or on arriving at the last probe, the bot picks a random yaw, resolves a
standable point that way, and compares its `exposure` with the exposure where it stands. Lower wins. If
the probe is no better, the bot goes the opposite way, unscored. It is a random walk biased toward
cover, not a search.

- **`exposure` is used because it is the one vantage field independent of facing.** Facing-dependent
  measurements belong to camping, which reads its own position-plus-facing dataset (`NavMesh::camps`,
  `doc/navgen.md`). The two never run together: `BotCamp::Run` sits below `BotCombat::Run`, so a bot
  with an enemy shuffles and a bot that only suspects one camps.
- **The enemy is not consulted.** No sight trace from the probe to the opponent. If the bot shuffles
  out of the fight, the 5 s rule ends it and the bot goes back to its goals, which is fine.
- `hereExposure` comes from `bot.sense.vantage`, which `RefreshVantage` keeps every 0.1 s; only the
  probe costs a `BlendVantage`.
- **With no vantage data** (a `.nav` built `--no-vantage`, or `crossbot_vantage 0`) the bot takes the
  random direction unscored with reason `no vantage`. Moving unpredictably beats standing still.

**Local means the current area and its walk neighbours.** `ResolveSpot` accepts the bot's area and any
area joined by a `CONN_WALK` connection, honouring `CONN_FLAG_ONE_WAY`. Jumps, ladders, lifts and
teleports are route-building, which combat drops. Containment is `Nav::AreaContainsPoint`, because
merged areas are non-convex prisms and an AABB clamp would be wrong.

A single full-length step is not enough: on crossfire a plain `origin + dir * 160` resolved to nothing
73% of the time, because areas are narrower than the step and a neighbour at another height fails the
z test (the probe sits at that area's centroid z). `ResolveAlongRay` retries the ray at 0.7, 0.45 and
0.25 of the step, so `walled off` means the bot really is against something.

### Whether the enemy has noticed

`enemyAware` says whether the opponent has seen the bot, so an encounter with an oblivious enemy plays
differently from being shot in the back. It is nearly free: combat is entered only on sight, so the
sightline is already known clear, and "does he see me?" is a dot product against the enemy's forward
vector. No new traceline, which matters with vision budgeted at two traces per bot per frame. The
reciprocal is assumed (vision traces eye to a hitbox bone, `EnemySees` reasons eye to eye); a wrong
guess costs one badly chosen opening weapon.

- Use `pfnAngleVectors`, not `MAKE_VECTORS`: the macro writes `gpGlobals->v_forward`, which a caller
  may still be holding.
- A nonzero `v.fov` means zoomed, so the cone is narrower than the 90° default. Counter-Strike's
  unzoomed `v.fov` is 90, which every cone that falls back to 90 for zero already handles.

**The flag latches on**: nobody unsees someone. It is set at three sites:

| Site | Value |
| --- | --- |
| sight entry | `EnemySees` |
| **retarget** | recomputed; that branch rebinds `st.enemy` without `Enter`, so a flag written only in `Enter` would inherit the last opponent's verdict |
| every later frame with sight, or on damage | `EnemySees`, or `true` |

`SetAware` is the only writer of the true edge, and it zeroes `fire.nextSelectTime`. Selection is
throttled to `kSelectInterval`, so without that a bot caught mid-ambush holds a crowbar for half a
second at someone already aiming back. That one line is the "switch to something reasonable once he
sees you" rule.

### The ambush: a restricted pick

While the enemy is unaware, `BotFire` opens with melee if it can and a throwable otherwise. It is the
ordinary ranking pointed at a smaller set: `PickBest` ranks by value then least carry time with a
reservoir draw on ties, so restricted to ambush weapons it gives "a random throwable weighted by
rarity" (the grenade and the snark tie at 0.1). Melee needs no range constant: `BuildChoices` only
offers the crowbar inside its `range` of 32.

`IGameProfile::AmbushWeapon(AmbushKind)` names the weapons; it must be a profile hook because nothing in
`WeaponSpec` separates a snark from a hornet gun (`doc/games.md`, "AmbushWeapon").

**Three passes, in order:**

1. `PASS_REACH`: melee with a finite `range`. The grapple's range 0 means unlimited, so `BuildChoices`
   offers it at every distance; "any melee" would open every ambush with it.
2. `PASS_THROWN`: grenade, snark, penguin; on CS the HE, flashbang and smoke. A mode is rejected when
   `dist < m.splash`, because no `WeaponMode` has a minimum range and `ModeUsable` only gates the
   maximum; without it the bot draws a grenade at knife range and never throws. The same gap exists for
   the RPG and the MP5 grenade on the ordinary path (not fixed).
3. `PASS_LAST`: `AMBUSH_GRAPPLE` alone, no distance or splash gate.

**The attempt runs before `TryKeep`.** `TryKeep` returns as soon as the held weapon works, so a bot with
a loaded MP5 would never reach a picker placed in `PickChoice`. Three guards, each load-bearing:

- not while `holding`, because the commit in flight may be a primed grenade;
- not when an ambush weapon is already drawn, or it re-selects every frame and never throws;
- not once the enemy knows.

**Many ambushes decline themselves, correctly.** The uphill veto refuses gravity modes at targets more
than 16 above the eye, and the flight-time gate caps the grenade near 600 units and the snark near
300. `PickAmbush` then returns -1 and ordinary selection runs: "blast him with whatever is in hand".

### Mirroring an opponent who draws melee

If the enemy has melee out and stands in the bot's own area, the bot draws melee and duels: close,
swing, back off. Neither flag latches; the duel ends the moment the opponent draws a gun.

- **`enemyMelee`** is `IsMeleeWeapon(BotWeapons::ActiveWeaponId(enemy))`, a set (OP4 has a knife,
  pipewrench and grapple). `ActiveWeaponId` is two unvalidated derefs (`m_pActiveItem`, then `m_iId`)
  returning 0 for "no idea". The full `BotWeapons::Read` would validate every slot through
  `ItemEdict` and call `GameData::Fatal` on a bad pointer, which is right once a frame for the bot's own
  inventory and wrong for a question this narrow. The read is gated on `FL_CLIENT`, so monsters never
  duel. `crossbot_weapons` cross-checks it against the full walk's `activeIndex` and prints
  match/MISMATCH.
- It refreshes only on frames the bot can see the enemy, the same restraint as `EnemySees`; between
  sightings the last verdict stands, bounded by `kSightTimeout`.
- **`enemyInArea`** asks "is the enemy in my area" rather than "which area is the enemy in":
  `FindAreaById` plus one `AreaContainsPoint` is O(1), while `FindAreaNearest` is up to three linear
  passes over the mesh. It tests XY against the area's centroid z (nav areas are duck-origin z, as in
  `TestArea`) plus a 72-unit z window, so someone on a walkway overhead does not count.

Both edges of the combined predicate zero `fire.nextSelectTime`, so the bot neither holds a shotgun at
a charging crowbar nor a crowbar at someone who just drew an MP5.

Selection uses `PASS_MELEE` (an identity test against `MeleeWeapon()`) with `BuildChoices(...,
geometry=false)`, which goes through `ModeViable` instead of `ModeUsable`: the duel wants a weapon
usable after walking, so the range gate is dropped while the pin and ammo checks stay. The block sits
after the ambush and before `TryKeep`; at contact range the two agree.

**The duel replaces the shuffle**, the one place combat moves toward the enemy. `haveProbe` is dropped
on entry so a stale probe does not resurface afterwards. It runs only with melee actually in hand, so a
failed selection cannot leave a bot charging bare-handed:

| Condition | Movement |
| --- | --- |
| swung within `kDuelBackoff` (0.4 s) | away from the enemy |
| `dist > range * 0.8` | toward the enemy |
| otherwise | stand and swing |

- The reach is the held weapon's `primary.range`. Range 0 (the grapple) skips the charge row, so a
  grapple duel only swings and backs off.
- 0.4 s is a little under the crowbar's refire. In practice the bot fires as soon as it is inside 32,
  which puts it straight into backoff, so "stand and swing" is the rarest state.
- Steering goes through `ResolveSpot` (areas are non-convex); when it fails, the probe gets the frame.
  Everything below the direction choice is the shared drive (`SolveDrive`, `Bot_DecomposeMove`, the
  crouch flag), and aim is untouched, so the retreat is a backpedal.
- A swing is known from `fire.lastShotTime`, set on the rising edge of the weapon's refire timer beside
  every `++fire.shots`.
- Two mirroring bots never back down; only death and `kSightTimeout` end it. If that reads badly, add a
  duel timeout rather than changing the mirror rule.

## Firing (bot_fire.cpp)

Two entry points, split by design:

- **`BotFire::Think` is policy.** Once per frame from `Bot_MoveNav`, it decides which weapon and mode
  the bot is trying to use and publishes the aim point. It can issue a weapon-select command, so it
  needs one known slot in the frame.
- **`BotFire::Buttons` is the trigger.** Pure, inside `Bot_RunPlayerMove`, it answers: given the angles
  being sent to the engine this instant, does the shot land? Before that,
  `IGameProfile::FireToggle(bot, firing)` may claim the frame for an `IN_ATTACK2` of the game's own,
  gated on the player and weapon both being able to act (the CS shield, the DoD bazooka).

**Why the trigger lives in `Bot_RunPlayerMove`.** It is the one funnel every drive branch passes
through and the only place with the angles actually commanded this frame, so firing works while
walking, on ladders, on lifts, mid-airmove and idle, against the exact ray the engine will use. That
is why it takes `CBot &`. The jump split can send a held `IN_ATTACK` through two `pfnRunPlayerMove`
calls in one frame, which is harmless because nothing here depends on button edges.

### Rules the game's dispatcher imposes

From `CBasePlayerWeapon::ItemPostFrame` and `CBasePlayer::ItemPostFrame`:

- **Running dry must release the trigger.** Auto-reload lives in the "no fire buttons down" branch, so
  a bot holding `IN_ATTACK` on an empty clip clicks forever. Every ammo check exists to release the
  button.
- **Reload deliberately.** `IN_RELOAD` is honoured directly, so the bot does not fall into the idle
  branch, which also runs `GetNextBestWeapon` and would swap the weapon `Think` chose. `wantReload` is
  set in `Think` and consumed first in `Buttons`, and is cleared while `m_fInReload` is set: one frame of
  `IN_RELOAD`, then nothing until the reload completes.
- `Bot_RunPlayerMove` strips `IN_RELOAD` on a frame that holds `IN_USE`, so a bot never reloads while
  defusing or working a use goal; `wantReload` and `idleReloadId` stay set and the reload goes out
  after. The engine's own auto-reload at 0 rounds still happens.
- **Ready means `<= 0`.** Retail Half-Life uses `CLIENT_WEAPONS`, so attack timers are countdowns
  (see "Attack timers are stored raw" below). `BotInventory::nextAttack > 0` means nothing fires
  this frame, and `Buttons` returns 0 rather than pressing into a closed gate.
- **`IN_ATTACK2` short-circuits `IN_ATTACK`.** The secondary test is an `if`, the primary its
  `else if`, so `Buttons` returns one bit or the other, never both.
- Every Half-Life weapon is mechanically full auto (`doc/games.md`, "Why FIRE_TAP is bot intent").

### The fire ray and the target box

Bullets travel along `v_angle + punchangle`, and the test uses the same ray. So the bot sprays, recoil
walks the muzzle off the target, the trigger releases, punchangle decays and it fires again: **burst
fire for free, with no burst timer.** The bot never compensates for its own recoil; this gap produces
the right behaviour and should not be "fixed" without replacing it.

`Buttons` builds a box the size of the target's `maxs - mins` around **the point `Think` published this
frame** (the served aim point for hitscan, the solved lead for a projectile) and fires when the ray
enters it. The enemy's size is the whole error budget. Centring on the served point instead of the
live origin asks "is my hand where my intent is":

- A bot mid-flick is not yet pointing where it means to, so it holds fire. No settle timer.
- A bot tracking a stale position because of its reaction delay fires there and misses, like a human.
- Losing sight needs no special case: the queue drains for `crossbot_aim_delay` and the bot keeps
  tracking the last believed position, then stops when `ReadDelayed` runs dry. `UpdateVision` does not
  queue a lead point that has crossed into geometry, so the drain mostly runs out over open ground.
  This relies on `BotSense::UpdateVision` clearing the queue on acquiring a *different* target, not on
  losing one.

The served point is a random hitbox bone, so a box centred on a foot reaches a body height below the
enemy. That generosity is deliberate.

A second trace, `ignore_monsters` from the eye out to `range`, refuses the shot if world geometry stops
the ray before the box entry, so the bot does not fire through a wall at someone it can hear.
`RayOnTarget` returns a `RayVeto`: `RAY_HIT`, `RAY_WIDE` (not converged), `RAY_BLOCKED`, `RAY_SELF`.

**Glass.** Vision sees through translucent brushes (`doc/perception.md`, "What the bot sees through").
If what stops the ray is a breakable that is not `kRenderNormal`, a second trace skipping that pane
decides, and a clear ray counts as a hit with `why=glass`, since bullets break glass. Unbreakable glass
stays `blocked`.

### Wall-banging

Under `crossbot_wallbang` (expert), a blocked verdict gets one appeal:
`IGameProfile::Penetrates(bot, id, eye, dir, entry)`. If the round still does damage at the box entry,
the ray counts as a hit (`why=wallbang`), and a shot that leaves counts in `wallbangs=`. The base
profile answers no.

- **The target is the prediction, never the enemy.** After sight breaks, `BotAim::ReadPredicted` serves
  the dead-reckoned point for up to 1 s on an engagement that rolled `predictLatched`
  (`doc/difficulty.md`). YaPB (`isPenetrableObstacle`) aims at the live `v.origin`, which is x-ray;
  here an enemy who stops behind the wall is missed. A heard enemy never seen is not shot at.
- **Both overrides replay the game's own loop with engine traces** instead of modelling thickness.
  Every heuristic tried elsewhere (YaPB has three, all walking `PointContents`) gets concrete or thin
  boards wrong.
- **CS** (`CCSProfile::Penetrates`) replays ReGameDLL `FireBullets3`. Per hit, the texture's material
  scales penetration power, cumulatively, and sets the damage modifier; the next segment starts
  `power` units past the hit, is half as long and costs one penetration; damage falls off by
  `rangeMod^(d/500)` per segment. A trace starting inside a thick wall hits again at once and burns a
  penetration, which is why a replay is exact and a formula is not. Material comes from
  `TRACE_TEXTURE` and `sound/materials.txt` (loaded as CrossPlay serves it: `crossplay/` first, then
  the game dir; 16 characters; unknown textures are concrete). `CS_Bullet` carries count, power,
  distance, damage, falloff and range; only rifles, snipers, the M249 and the deagle penetrate more
  than once. The shot needs `kWallbangMinDamage` (10) left: one concrete wall halves a rifle (AK 36 to
  18), and a floor of 20 refused every rifle wallbang on fy_snow.
- **HLDM gauss** (`CHLDMProfile::Penetrates`, secondary only) replays `CGauss::Fire`: the first wall
  must be hit at `n >= 0.5` or the beam reflects; the exit is found the SDK's way (forward trace from 8
  units in, then a trace back); the wall must be thinner than the charge damage and leave more than 10.
  The shot counts if the exit blast (`left * 1.75`, multiplayer) reaches the box entry or the beam
  continues to it unobstructed. Damage is `BotGauss::ChargeDamage`.

### The CS spread gate

Recoil is covered by the punchangle ray. CS spread also grows per shot, and `SpreadHold` covers that.
On `RAY_HIT` the bot asks `IGameProfile::SpreadWide` whether the next shot's cone fits:
`limit = spreadFit * half / dist`, with `half` half the target's smaller horizontal size. If not, it
holds until `ready + U(spreadEarly, spreadLate)` (`doc/difficulty.md`), and the hold turns sticky fire
off, because the cone only recovers with the button up. Only the CS profile implements it.

What the gate relies on, from ReGameDLL (`wpn_shared/*.cpp`, `weapons.cpp`, `cbase.cpp`):

- Each axis is offset by `t * spread`, `t` the sum of two uniform draws on [-0.5, 0.5], so shots bunch
  in the middle. With a cone K times the target, one axis lands inside with probability
  `1 - (1 - 1/K)^2`: 100% at K=1, 89% at 1.5, 75% at 2.
- A shot uses the `m_flAccuracy` the previous shot left; the new value is stored after.
- **Rifles and SMGs** (`spread = a + b * acc`): `acc` follows `m_iShotsFired`. With the button up,
  `ItemPostFrame` caps the count at 15, waits 0.4 s, then removes one shot per 22.5 ms; at zero,
  `REGAMEDLL_FIXES` restores base `acc`. The cone snaps back all at once:
  `ready = now + 0.4 + 0.0225 * min(shots, 15)`. Waiting part of that gains nothing.
- **Pistols** (`spread = a + b * (1 - acc)`): each shot moves `acc` by `-(T - gap) * r`, clamped to
  [floor, base]. Any gap under T drains toward the floor; T or more keeps base. The gate never holds
  while the floor cone fits; otherwise it holds each shot until `m_flLastFire + T` (0.275-0.4 s).
- When even the resting cone is too wide (rifles above their move threshold, anything airborne) the
  gate stays open and the bot sprays.

Coefficients live in `kSpreads` (`cs_weapons.cpp`) from each weapon's `PrimaryAttack`, with branches
chosen in the weapon's order. USP and M4A1 use unsilenced numbers and the Glock semi-auto, because the
bot never toggles them. Shotguns and the knife have no entry. The four scoped rifles have speed-only rows
with `B` 0, so this gate never holds for them (see "Speed-gated brace").

Once a hold ends the bot fires whatever the cone; the gate can close again only after the clip drops
(`spreadClip`), so a bot that stopped waiting early (easy 37.5% of the time, normal 25%) fires one
sloppy shot instead of being locked back into waiting. A `spreadUntil` more than `kSpreadMaxWait`
ahead is a previous map's clock and is ignored.

Shots per burst at 1024 u against a 32 u target, standing:

| weapon | expert K=1 | hard K=1.25 | normal K=1.5 | easy K=2 |
|---|---|---|---|---|
| AK-47 | 4 | 5 | 5 | 6 |
| M4A1 | 5 | 5 | 6 | whole mag |
| Galil | 3 | 4 | 4 | 5 |
| FAMAS / AUG / SG552 | 5 | 6 | whole mag | whole mag |
| M249 | 3 | 4 | 5 | whole mag |
| MP5 | 1 | 3 | 6 | whole mag |
| P90 | 1 | 1 | 4 | 7 |

Reference bots: ZBot (`cs_bot_weapon.cpp`) uses fixed random pauses (sprays under 400 u, 0.15-0.5 s
beyond, 0.3-0.7 s past 800 u); YaPB (`needToPauseFiring`) pauses 0.38-0.55 s when recoil at the
target's distance passes a few units. Neither reads `m_flAccuracy`.

### Supported: which modes the fire path drives

```cpp
(mode.fire == FIRE_HOLD || mode.fire == FIRE_CHARGE || mode.fire == FIRE_TAP) &&
mode.toggle == TOGGLE_NONE &&
(mode.aim == AIM_TARGET || (mode.aim == AIM_LEAD && mode.speed > 0)) &&
(homing || arc || mode.gravity == 0) &&
(arc || !thrown) &&
(mode.quirks & (WPN_Q_HIDDEN_STATE |
                ((homing || thrown || guided) ? 0 : WPN_Q_INHERIT_VEL))) == 0
```

A predicate over the spec, not a list of weapon ids: enabling a group of weapons is relaxing a clause.
`arc` is `gravity > 0 && splash > 0 && !homing` (derivable, so not a quirk); `thrown` is
`WPN_Q_THROW_ARC`; `guided` is `WPN_Q_KEEP_AIMING`. `ArcDetonates` is `arc && !thrown`.

- **What is left out is the deployables and nothing else**: a `WeaponToggle` (zooms, the RPG laser),
  `AIM_NONE`/`AIM_SURFACE` (satchel primary, tripmine) and `WPN_Q_HIDDEN_STATE` (both satchel modes,
  RPG secondary). They fire with no target at all, so they belong to `bot_deploy.cpp`, a decision
  layer that runs on the map's gates and remembers what it placed. `BotFire` stands down for it as for
  a break. Zoom is pressed as a step before the fire decision, never as a fire mode.
- **`WPN_Q_INHERIT_VEL` is rejected only when nothing accounts for it.** A thrown mode solves it
  exactly (a vector triangle); a homing mode performs no solve; a guided rocket adds only
  `v_forward * dot(playerVel, v_forward)`, a speed change, and steers anyway; a plain arc solves in the
  thrower's frame. What remains is the satchel, held out by `WPN_Q_HIDDEN_STATE`.
- **`splash` rejects nothing.** Self-damage is decided per shot by `SplashSafe`, below.

### ModeViable, ModeUsable and ammo

`ModeViable` is the target-free half: `Supported`, the pin, `IGameProfile::ModeAvailable`,
`WPN_Q_NO_UNDERWATER` against `waterlevel == 3`, and ammo. `ModeUsable` adds geometry: `range` (the
crowbar stays holstered past 32, shotgun and egon past 2048), the uphill veto, and for `AIM_LEAD` a
flight-time ceiling. Out of combat there is no target, so the idle path calls only `ModeViable`.

The flight-time ceiling is `dist / speed > kMaxFlightTime` (1.5 s). For homing modes, where nothing is
solved, it is effectively a distance: the hornet stops past 900 units, the snark past 300.

Ammo is judged by `ShotCost`: `ammoPerShot` for discrete fire, the whole spin-up for `FIRE_CHARGE`
(about 16 uranium for a full gauss charge; the gauss policy uses less, `doc/difficulty.md`). A bot with
less keeps gauss primary. Selection and firing judge ammo differently: a clip weapon with an empty clip
and a full reserve is still selectable, but firing needs `ShotCost` in the clip.

### Leading a projectile

`AIM_LEAD` needs where the target will be when the projectile arrives, which depends on the flight
time, which depends on that point. `SolveLead` runs the fixed point:

```
p = servedPoint
repeat 4:
    t = |p - eye| / speed
    p = servedPoint + servedVelocity * t
```

Four passes, no convergence test: testing costs about what a pass saves. The map contracts whenever the
target is slower than the projectile, which every admitted mode is by a wide margin against a 320 u/s
player, so the residual is far inside the hitbox. `kMaxFlightTime` is a plausibility limit on
prediction, not a projectile lifetime: extrapolating a player 1.5 s ahead is fiction.
`WPN_Q_SLOW_IN_WATER` halves `speed` at `waterlevel == 3`, matching `CCrossbow::FireBolt`, which tests
the player's waterlevel.

**The lead is as stale as the perception.** `BotAim::DelaySlot` stores the target's velocity beside the
point, and `SolveLead` uses that served velocity, not the live `pev->velocity`. Reading the live value
would make reaction delay cost position error but never prediction error. The composition is
`perceived position + perceived velocity * (reaction delay + flight time)`, since `UpdateVision`
already folds `velocity * delay` into the queued point; both terms come from the same stale sample.

**Weapon choice drives the aim.** `Think` publishes `aimPoint` (the served point for `AIM_TARGET`, the
solved lead for `AIM_LEAD`) and `BotSense::ApplyLook` prefers it over its own `ReadDelayed`, so the bot
looks where the weapon needs. `Buttons` tests against the same value. For `AIM_TARGET` the published
point is the served point, so hitscan aim is unchanged by construction.

**Homing modes skip the solve.** For `WPN_Q_HOMING` (hornet primary, snark), `SolveLead` is skipped,
the aim point is the served point, and the tolerance is a fixed `kHomingCone` of 20°, written into
`RayHitsBox` by raising the half-extents to `dist * tan(20°)`. `tol` in `crossbot_fire_status` reports
20 and `flight=` reads 0, since nothing was solved. Solving a lead for a hornet that accelerates from
300 while turning would buy precision that is not there. Hornet secondary flies straight at 1200 and
keeps the solve, so one weapon exercises both paths.

`AimBase` shifts the predicted point before any solve. `WPN_Q_AIM_FEET` drops it to the target's feet
(origin z plus `mins.z`, measured against the sensed point so the offset holds for any bone): the
displacer only teleports on a direct hit, and the damage is the splash. `WPN_Q_SPINUP` pushes the point
ahead by `chargeFull` seconds of target velocity.

### Arcs: the solve, the sim and the two gates

An arcing mode skips `RayOnTarget`; a ray-versus-box test means nothing on a parabola.

`SolveArc` is closed form for horizontal range `d`, height difference `h`, speed `v`, effective
gravity `g`:

```
disc       = v^4 - g * (g*d*d + 2*h*v*v)
tan(theta) = (v*v - sqrt(disc)) / (g*d)
flight     = d / (v * cos(theta))
```

`disc < 0` is out of range, a real ceiling (flat ground, 800 u/s under `kGravity * 0.5`: `v^2/g` =
1600 units). The low arc is always taken; the high root spends seconds in the air. `SolveArcLead` wraps
it in the same four passes as `SolveLead`.

`SimulateArc` steps that parabola at `kArcStep` (0.1 s) with a traceline per segment, answering what
the closed form cannot: **what does the shell hit first**. It traces `dont_ignore_monsters`, because
`ExplodeTouch` fires on players too. At 800 u/s a segment is 80 units, about fifteen traces per flight;
a chord can cut a corner the shell would clip, which wastes the shot, the cheap failure. Matching the
engine's semi-implicit Euler (`SV_Physics_Toss`) exactly would move the impact about two units over a
second, against a 250 splash.

Two gates decide the trigger, both in units:

- **Near enough to hurt.** The impact must be within `splash * kSplashAccept` (0.5) of the nearest
  point on the target's box. A shell at their feet scores zero distance, which is right: a floor burst
  at the feet is a full hit.
- **Closer to them than to me** (`SplashSafe`): `|impact - targetPoint| < |impact - eye|`. No
  constant, scales with the weapon, and deliberately a little suicidal: a bot that lingers is worse
  than one that takes a graze. Rocketing someone 200 units away costs about 20 damage from a 100-damage,
  250-radius blast.

`SplashSafe` runs for **every** mode with `splash > 0`, not just arcs, which is what lets `Supported` admit the RPG.
For direct fire the impact is the ray's: `RayOnTarget` already has the box entry and wall distances,
so no extra trace. It reports `splash-self` there and `arc-self` on the arc path.

Both gates run twice per frame. `Think` runs them along the **ideal** launch direction and treats
failure as "mode unusable", so the bot picks another weapon instead of facing a wall with a grenade
launcher. `Buttons` runs them along the **live** view, which lags by the reaction delay and finite hand
speed, and that decides the press.

`fire.aimPoint` (where to look: `eye + launchDir * dist`, well above the enemy) and `fire.targetPoint`
(where the enemy is, for the splash gate) differ for arcs. `miss` and `tol` in `crossbot_fire_status`
switch to units: impact-to-box gap against `splash * kSplashAccept`.

**Arcs with inherited velocity** (the spore grenade: `800 * forward + player velocity`, gravity 1) are
solved in the thrower's frame: the projectile leaves at `dir * speed`, the target moves at
`vel - botVel`. The world impact is the answer plus `botVel * flight`, and the simulator gets
`dir * speed + botVel`. `ArcOnTarget` adds the same drift. The MP5 grenade has no `INHERIT_VEL` and
takes the plain path.

### The uphill veto

A gravity mode the bot does **not** know the resting place of is refused when the served point is more
than `kUphillSlack` (16) above the bot's eye: a snark lands short and walks home, a grenade bounces down
the step it failed to clear. 16 comes from the hitbox: eye is 28 above origin and the hull top 36, so
the highest bone on a level target is about 8 above the bot's eye; 16 clears that and still vetoes a
step (18).

The exemption is `ArcDetonates`: arcs that stop where they land. The MP5 grenade's `ShootContact`
gives it `ExplodeTouch`, so the simulated impact is the explosion. The hand grenade's `ShootTimed`
gives it `BounceTouch` and friction 0.8, so the impact is only where it lands before rolling. The
residual roll is not modelled; 250 units of splash forgives more than simulating `BounceTouch` would
buy, and it is the cost of throwing instead of cooking.

It lives in `ModeUsable`, so the bot switches weapon instead of holding a snark it will not throw.
`uphillVetoes` counts rejections per weapon per mode per frame, so read it as a witness: it must stay
non-zero in `test_fire.ps1`'s pinned snark phase and zero in the 9mmAR phase.

### The coupled throw solve

A `WPN_Q_THROW_ARC` mode has one launch parameter: `IGameProfile::ThrowLaunch` maps the held pitch to
launch elevation and speed (`doc/games.md`, "The throw law"). `SolveArc` assumes a fixed `v`, so this
is a scan, cheap because each sample is closed form in the other unknown.

For a candidate pitch, `ThrowLaunch` gives `(e, s)` and the horizontal speed `sh = s cos e`. With `flat`
the 2D vector to the target, `pvH` the thrower's horizontal velocity and `u = 1/t`:

```
|flat * u - pvH| = sh
=>  A u^2 - 2 B u + C = 0,   A = |flat|^2,  B = flat . pvH,  C = |pvH|^2 - sh^2
t     = 1/u            (larger positive root: the shorter flight)
dirH  = (flat * u - pvH) / sh          unit by construction
err   = (sz + pvz) t - g t^2 / 2 - h
```

That solves the `WPN_Q_INHERIT_VEL` vector triangle exactly: `dirH` is the yaw that makes launch plus
drift arrive, which for a strafing bot is not the yaw at the enemy.

`err` is unimodal in elevation, so scanning down the pitch range (up the elevation range) and taking
the first upward zero crossing finds the low arc: `kThrowScan` samples bracket it, `kThrowBisect`
halvings refine it. Samples with no positive root are skipped and the crossing test runs between valid
samples, so the far end degrades into "no solution", not a wrong one. The bracket ends are named
`under` and `over` for the sign of `err`; pitch is down-positive, so `over` is the numerically smaller
pitch.

The muzzle (`eye + launchDir * 16`) depends on the answer and the target moves, so both fold into the
four passes `SolveArcLead` uses. Against a static target the residual is under 0.03 units.

The published `aimPoint` is built from the **pre-remap** pitch: the bot looks where the game's remap
says to, not where the grenade goes. `ArcOnTarget` reads the live pitch through `ThrowLaunch`.
Nowhere is the remap inverted.

Reach is about 550 units, set by `kMaxFlightTime` (the throw itself tops out near 625 on flat ground).
`ModeUsable` at the row's speed of 400 prunes at 600, so in a 50-unit band selection says yes, the
solve says no, and the bot idles half a second before re-picking. That is left alone rather than
copying a bot-side constant into `range`.

### The sticky trigger

A human with an MP5 does not blip the trigger every time the crosshair crosses a hitbox edge. Modes
with `WPN_Q_STICKY` (`doc/games.md`, "WPN_Q_STICKY") may keep the trigger down while
`gpGlobals->time - bot.sense.targetTime <= kStickyWindow` (0.5 s). `sense.targetTime` is stamped every
frame the target passes `TraceVisible`, so it is "last seen". The window sits inside the 3 s
`kCombatMemory` by construction.

The window outlives the aim queue, so it takes two mechanisms. `ReadDelayed` goes dry once the newest
sample is older than `crossbot_aim_delay`:

```
0 ───────────── 0.25s ───────────── 0.5s
  haveTarget true      haveTarget false      window closed
  aim point served     mode would be         ClearAim + Idle
  → relax the box      dropped by Idle
    test in Buttons    → hold it in Think
```

- **Before the boundary, `Buttons` overrides `RAY_WIDE` alone.** That is the crosshair not converged,
  exactly what a spraying human looks like. `RAY_BLOCKED` is positive knowledge the shot cannot arrive,
  and `RAY_SELF` cannot happen (every sticky mode has `splash == 0`). The cost: `RAY_WIDE` returns
  before the wall trace, so a bot facing a near wall when it loses its target sprays it for up to half a
  second. All sticky modes are cheap per shot.
- **After the boundary, `Think` keeps the state alive.** The no-target branch tests sticky between the
  holding case and the idle case and **skips `ClearAim`**, as `COMMIT_GUIDE` does: `aimPoint` stays on
  the last known position and `fire.mode`/`fire.weaponId` keep last frame's choice, so `StickyHold`
  needs no new state; it requires the active weapon to still match. The ordinary `Buttons` path then
  fires (`why=firing`) or the `RAY_WIDE` override presses. Like `holding`, the branch bypasses
  `ModeUsable`, and it clears itself when the window closes.

### The commits

`fire.commit` answers "I started something I must not walk away from":

| | act | latch | release | cap |
|---|---|---|---|---|
| `COMMIT_CHARGE` | press and hold | — | the shot, or the aim gate | `chargeMax - kChargeSafety` |
| `COMMIT_THROW` | press one frame | `m_flStartThrow != 0` | it returns to 0 | `kTapTimeout` (1.5 s) |
| `COMMIT_GUIDE` | press on a ready weapon | `m_cActiveRockets > 0` | it returns to 0 | `kGuideMax` (6 s) |
| `COMMIT_SPINUP` | press one frame | — (time only) | `chargeFull + kSpinupSlack` elapsed | the same |

While a commit is live it bypasses `ModeUsable`, pins the mode order to one entry and survives the
no-target bail.

- **The game shows the action late.** The hand grenade leaves half a second after the press, from
  `WeaponIdle`; the RPG's rocket is created inside `Bot_RunPlayerMove` and reads back next frame. So
  `commitSeen` latches the first frame the thing exists, and release is `commitSeen && !live`, never
  `!live` alone.
- **`COMMIT_THROW` opens only for a `WPN_Q_THROW_ARC` mode.** Its latch is `m_flStartThrow`. Any other
  `FIRE_TAP` mode (Ricochet's disc leaves on the press) just presses; a commit there could never latch
  and would run to the cap every time.
- **`COMMIT_THROW` also refuses while `m_flStartThrow` is set.** If a primed grenade's target walks out
  of reach, the solve fails and the commit clears; pressing again would skip `WeaponIdle` that frame and
  cook a grenade whose pin is already pulled. Reading the weapon's flag also covers a primed grenade
  picked up off the floor.
- **`COMMIT_THROW` and `COMMIT_GUIDE` open only when the weapon's own attack timer has expired**, not
  just the player's `m_flNextAttack`. A press the weapon rejects never latches and burns the whole cap.
  The grenade sets `m_flNextPrimaryAttack = 0.5` after the throw while `m_flStartThrow` already reads
  zero; `CRpg::Reload` pushes `m_flNextPrimaryAttack` forward every frame reload is held, leaving a
  window where the clip is full and the player timer expired but the weapon's is not.
- **`COMMIT_SPINUP`** is the displacer: the ball leaves about a second after the press along the view
  at that moment, with nothing observable at launch, so the commit is timed. Buttons stay released, the
  aim keeps tracking through a lost target (like guiding), and `AimBase` leads by `chargeFull` on top
  of the flight time.
- **The caps are valves, not models.** `kTapTimeout` covers a press that never took. `kGuideMax` is
  about 12,000 units of rocket and exists because the SDK leaks the rocket counter:
  `CRpgRocket::FollowThink` calls `Detonate()` without decrementing, and `RocketTouch` only decrements
  while `m_pLauncher` lives. Without it a leak pins a bot to an RPG it can neither fire nor reload.

`commitCaps` counts the throw and guide caps but not the charge, which releases inside `ChargeButtons`
and reports `why=charge-timeout` instead. Folding it in would make the three uniform.

**Guiding presses nothing.** `CRpg::UpdateSpot` moves the `laser_spot` to where the view points, but is
only called from `PrimaryAttack` and `WeaponIdle`, which `IN_RELOAD` routes around. A bot that reloads
its just-emptied RPG freezes the laser and the rocket flies to a stale point. So `COMMIT_GUIDE` returns 0
from `Buttons` with `why=guiding` ahead of the reload and empty-clip branches, and `Think` keeps
`wantReload` false while it lasts (`CRpg::Reload` refuses anyway while a rocket and the spot are live).
Guiding keeps its aim through a lost target (`Think` skips `ClearAim`): a rocket in the air is worth
steering at where they were.

**The charge is carried, not timed.** For the gauss secondary, releasing is the shot (`WeaponIdle`
calls `StartFire()` once no fire button is down). A timer would be wrong: `SecondaryAttack` sets
`m_flNextAmmoBurn = 1000` at full charge, so holding is free and a timer would throw away paid-for
charges. The machine:

- not charging: press, before any aim test;
- `held < chargeFull`: hold (a partial release does `200 * held / 1.5` and is never chosen);
- charged: hold until the ray passes the tolerance test, then release (`why=charged` while waiting);
- `held >= chargeMax - kChargeSafety`: release regardless. Past 10 s the SDK zaps the holder for 50;
  `kChargeSafety` is 0.5 s. `test_fire.ps1` fails if this fires, since it means the release logic
  stopped working.

A live charge is checked before the no-target bail, so a bot that loses its target keeps the charge
(`why=carrying`) until someone reappears or the cap, where the carry is dropped. That no-target drop
is not counted in `capped`, which stays the "release broke with a target in view" signal; with a target
in view `ChargeButtons` owns the timeout. `Buttons` treats "no aim point" as hold, never release. The
same carry cap releases a TFC sniper, which crawls at 80 u/s while it aims.

### Zoom

Zoom is a range decision. Zoomed crossbow primary in multiplayer is `FireSniperBolt()`, a hitscan for a
flat 120, so zooming swaps `AIM_LEAD` for `AIM_TARGET` (`ModeOf(spec, mode, zoomed)`). Which weapons
zoom is decided by data: only those with a populated `primaryZoomed`. The python's zoom changes no mode
and would cost turn rate for nothing.

`FovScale` divides hand speed by `fov / 90`, so at the crossbow's fov 20 the peak turn rate falls from
about 450°/s to 100°/s. The bot's hand has no resolution limit, so zoom buys only what the game gives
(the hitscan mode, a scope's accuracy). The rule is set on tracking cost: zoom above `kZoomIn` (500),
unzoom below `kZoomOut` (300). A 320 u/s target crossing at 500 sweeps about 37°/s. The band matters
because each toggle costs a frame and 1.0 s of `m_flNextSecondaryAttack`. The expert gauss policy
overrides this with a scope-cone rule (`doc/difficulty.md`, "At expert the scope is the shot").

The press rule is `FIRE_TAP`: press only when the live state differs from the wanted one. `pev->fov` is
read every frame, so a press stops itself once it lands. The toggle step returns `IN_ATTACK2` alone and
shoots nothing that frame.

**CS's AWP and Scout unzoom for the bolt cycle and rezoom themselves.** A zoomed shot sets `m_iFOV` 90
and `m_bResumeZoom`; `ItemPostFrame` restores the zoom once `m_flNextPrimaryAttack` runs out, before
reading buttons. The AWP also expires its secondary timer that frame, so a bot seeing fov 90 would press
`IN_ATTACK2` on top of the restore and unzoom. Two fov-only rules close it: `ZoomButtons` never presses
on `cycleEnd` (the first frame the primary timer reads ready after being busy), and `Buttons` holds fire
as `zoom-wait` while it wants in but the toggle cannot be pressed yet. Wanting out while zoomed still
fires.

### Selection: the best gun it has, and the rarest of those

Two numbers rank a weapon, in order:

- **Score** is `WeaponSpec::value` (`doc/games.md`, "value"). Only candidates at the single highest
  score survive; a 0.3 never competes with a 0.5.
- **Carry time**: seconds the weapon has been in the inventory, per weapon id in `BotFire::Carry`
  (`bot.weaponCarry`). Among the survivors, **least carried wins**: the gauss a bot rarely finds beats
  the crossbow it has lugged for two minutes, and a fresh pickup beats everything.

Then one of the winner's usable modes is drawn at random through the per-bot RNG, once at select time;
`fire.pendingMode` carries it through the switch so it does not flap. Zoomed variants are never
candidates.

`AccrueCarry` runs once per bot per frame from `BotManager::StartFrame`, right after
`BotWeapons::Refresh`, charging `Bot_FrameTime()` to each carried weapon. Dead bots skip it, and the
single call site means no double accrual from the jump split. The table lives on `CBot`, not in
`BotFire::State`, because `Reset` replaces the state on every respawn: the inventory resets to the spawn
kit each death while the tally survives, so the crowbar and glock climb life after life and rare guns
stay near zero. That divergence is the rarity signal. Only `bot = CBot()` (map change, disconnect)
clears it. Rejected: counting seconds on target per weapon, which only accrues in combat and cannot
tell "never fired" from "never picked up".

**Three triggers.** There is no in-combat rotation; switching mid-fight for variety costs a deploy and
reads as indecision.

1. **The held weapon has no usable mode.** The common case: Half-Life auto-switches on pickup, often to
   an RPG.
2. **The clip is dry, in combat.** `AmmoReady` has a `strict` flag. `Think` runs keep-strict,
   select-strict, keep-loose, select-loose, so a loaded weapon in the bag beats reloading the one in
   hand (`why=empty-switch`), and reloading under fire happens only when nothing else is loaded.
3. **Every 3 s out of combat.** Out of combat means no target this frame and none for `kCombatMemory`
   (3 s), so a bot does not fumble its inventory the moment an enemy steps behind a pillar.

**Out of combat, reload first, then rotate.** Auto-reload only fires at `m_iClip == 0` and only for the
active item, so the idle pass selects the weapon with the largest reloadable deficit and `Buttons`
presses `IN_RELOAD` from a branch above the `MODE_NONE` return (`fire.idleReloadId`). A weapon still
short wins the next tick outright, so a reload is never abandoned. This pass is not score-filtered;
topping up is tidying, and carry time breaks ties.

Only then does score-then-rarity run. **Melee is excluded** except as the last resort: at spawn the
crowbar and glock both score 0.1, and the melee pass is what stops the bot settling on the crowbar.
**The held weapon wins ties**: the bot switches only if the winner beats it on score, then on carry.
Without that, two same-score weapons with converging carry times flap every 3 s. A held melee weapon
loses ties on purpose. Because score is stable, an idle bot with a gauss and a glock settles on the
gauss and stays; the tests assert where a bot ends up.

**The mode survives the switch.** `fire.pendingMode` rides with `fire.pendingId`, and the keep loop
tries it first on the frame the switch lands; otherwise a weapon whose primary worked never reached its
secondary by choice. The active weapon is left out of the choice list when the keep loop already tried
both its modes (usually a blocked arc solve), or the ranking would spend `kSelectInterval` re-picking it.

`kSelectInterval` (0.5 s) throttles combat re-selection so a refused select cannot become a per-frame
command loop. Completion is observed, not assumed: `BotWeapons::Refresh` reports `activeIndex` every
frame, and the issuing frame reports `mode=none`. Being throttled does not block the loose keep, so a
waiting bot keeps firing what it has.

The ambush and the duel reuse `PickBest` on restricted sets ahead of `TryKeep`, described above.
`crossbot_weapon_carry` prints each weapon as `carry/score`. `crossbot_fire_pin` is the only lever that
reaches selection deterministically.

### Brace: stop and duck to fire

`WPN_Q_BRACE` marks guns whose cone depends on the shooter's stance or speed: the OP4 M249, DoD rifles
and the FG42, the TFC sniper rifle and most CS guns.

`BotFire::Brace` runs in `Bot_RunPlayerMove` right after `Buttons`. When it answers yes, the move is
zeroed, the four move bits cleared, and `IN_DUCK` added if ducking is allowed and the profile does not
speed-gate. It answers yes only when:

- the bot wants to fire: a mode is chosen, the aim is valid, and the mode carries the quirk;
- it is on the ground, not on a ladder (`MOVETYPE_FLY`), not swimming;
- nothing asked for a jump this frame;
- the weapon will be ready by the time the bot can stop: not reloading, and the larger of the mode's
  timer and the player's `m_flNextAttack` is at most `NavPlan::StopTime`.

The readiness rule stops flicker. `StopTime` is HL ground friction (exponential above `stopSpeed`,
linear below): `ln(v/stop)/f + 1/f`, about 0.54 s from 320 u/s. A stopped bot's stop time is 0, so any
real wait (a reload, a slow cycle) releases it and it brakes again when the wait falls under its stop
time. The M249's 0.067 s cycle keeps it braced through a burst; its 3.78 s reload lets it move.

Ducking needs a clear line from the ducked eye: a trace from 34 units lower (or the current eye if
already ducked) to `fire.targetPoint`, cached 0.1 s. Blocked means stop but stay standing, so waist-high
cover does not blind the bot. `crossbot_fire_status` reports `bracing=1` (ducked) or `2` (standing) and
a per-life `braces=` count.

**OP4's M249 reads button bits, not velocity**: `IN_DUCK` 3°, any of `IN_FORWARD/BACK/MOVELEFT/
MOVERIGHT` 15°, otherwise 6°. The cone is read from `pev->button` when the shot leaves, while movement
comes from `cmd.forwardmove`/`sidemove`. So under `crossbot_brace_moving` (expert) the bot keeps its
movement values on the firing frames, clears only the four move bits and taps `IN_DUCK` for the 3°
cone. A one-frame tap never reaches `FL_DUCKING`, so it costs no speed. This applies only in OP4; the
TFC sniper rifle (no charge above 50 u/s), CS and DoD are velocity-based and keep the stop at every
tier.

### Speed-gated brace: CS and DoD

`IGameProfile::BraceSpeed` turns "stop" into "get under a speed": it returns the largest 2-D speed the
next shot may leave at, or `FLT_MAX` for run and gun. OP4 and TFC do not implement it. When it
answers:

- `Brace` brakes only to that cap: the drive is scaled to at most `0.9 * cap` (0 still means stop), and
  readiness uses `StopTime(v) - StopTime(cap)`, exact for HL friction.
- `Buttons` holds the trigger while speed is over the cap (`why=brace`, counted in `bracewaits=`).
  Without it a weapon already ready fires on the first braced frame at run speed.
- `BraceCreep`: in combat, holding a weapon whose primary carries the quirk and not holding a camp
  spot, the bot uses the camp creep for the whole fight (`CreepSpeed` 130 in CS, a walk;
  `CreepButtons` `IN_DUCK` in DoD), so there is less speed to shed.
- A speed-gated brace never ducks (`bracing=2`). With the creep, a CS bot is under its cap and the
  brace only answers yes on the ready frame, so a duck would be a one-frame bob per shot that buys
  nothing: CS reads `FL_DUCKING`, which takes about 0.4 s of held duck. DoD already crouches through
  `CreepButtons`.

**DoD** always answers 45: every rifle `PrimaryAttack` adds its penalty above 45 u/s 3-D, and on the
ground 2-D is the same.

**CS** answers from `kSpreads` and the target, with the spread gate's `limit` (`SpreadLimit`). If the
cone the next shot would get *moving* (`moveA + moveB * acc` for rifles, `* (1 - acc)` for pistols,
live `m_flAccuracy`, plus the unzoomed penalty) fits, the bot runs and guns; otherwise the cap is the
weapon's move threshold, since ReGameDLL uses the moving cone above `velocity.Length2D() > T`.

| weapons | T |
|---|---|
| AK47, M4A1, Galil, FAMAS, SG552, AUG, M249 | 140 |
| P90, Scout | 170 |
| every pistol, G3SG1, SG550 | 0 |
| AWP | 10 |
| MP5, TMP, MAC10, UMP45, shotguns | no speed term, no quirk |

- The walk is 130, under 140, so a walking rifle bot never brakes; the brake matters for pistols, the
  auto-snipers and the AWP.
- At base accuracy against a 32 u target the moving cone fits out to about AK 296 / M4 327 / M249 250 /
  P90 696 / Deagle 640 units at expert, scaled by `spreadFit` for other tiers. A fully sprayed AK's
  moving cone (0.1275) fits only to about 125 u, so a bot that opened fire on the run brakes mid-burst.
- The sniper rows carry only the moving tier, with `B` 0. The AWP really has two tiers (0.25 above
  140, 0.1 above 10); the row keeps 0.25 at 10, because 0.1 only fits within about 160 u. G3SG1 and
  SG550 standing cones depend on time since the last shot and are not modelled.
- `crossbot_spread_fit 0` makes CS answer `FLT_MAX`: one switch turns off both the spread gate and the
  CS brace.

### Throwing at a heard enemy

A bot that hears an enemy it cannot see throws a grenade at where that enemy will be, if the blast
hurts them more than it hurts the bot. Both throw paths do it (`BotFire` for thrown weapons,
`BotGrenade` for TFC off-hands); `crossbot_grenade 0` turns both off.

- **The target is `BotSense::HeardEnemy`**: the freshest suspicion whose latest source is a sound or
  playback event, at most 1 s old, on a player or a monster hunting us, and `Attackable`. A sighting
  overwrites the source, so an enemy last seen is never treated as heard. The prediction moves the
  heard position along the enemy's horizontal velocity by the sound's age (a small cheat: footsteps
  only give a rough heading), the lead solve adds flight time, and Z is dropped so a jump does not
  send the grenade into the ceiling.
- **"Hurts them more" is `BotFire::HarmFavors`**, HL's `RadiusDamage` shape: each side takes
  `1 - d/splash`, zero if the trace from the blast (lifted 1 unit) is blocked. The enemy's harm must be
  above zero and above the bot's. The blast point is the arc's first impact (`ArcImpactHits` for
  weapons, `SolveOffhand(..., &impact)` for TFC). Failures count in `harmveto=` and report `arc-harm`.
  Homing throwables (snark, penguin) have splash 0 and take the `RayOnTarget` path instead. Flashbangs
  are judged against the body centre, not the eyes; change it there if flashes misjudge.
- **Weapons: thrown ambush weapons only, no blind gunfire.** `HeardThrow` runs first in `Think`'s
  no-target branch. A thrown ambush weapon in hand (not the grapple) goes through `TryKeep`; otherwise
  `PickBest(PASS_THROWN)` picks one that must pass `TryKeep` as a dry run before
  `SelectWeapon("heard")`. If nothing works, the ordinary no-target code runs.
- **One throw, then 4-9 s of silence.** Each shot while `fire.heard` is set adds to `heard=` and starts
  `nextHeardTime`, so a stream of footsteps does not empty the stack. `Idle` clears `heard`.

### The throw intent: deciding to throw is the commitment

Once a bot decides to throw, it switches, aims at the point it decided on, and throws. If it cannot, it
switches back to a gun and does not try again that fight. Rejected: re-checking every frame against a
fresh heard point. The point jumps with each footstep, the harm test flips frame to frame, the aim
never settles, and the bot holds the grenade for 3-6 s until idle selection takes it away (measured in
a 3v3 on fy_iceworld: 2 throws in 11 heard holds).

- **The decision.** Callers with a target point (`heard`, `ambush`, `select`, `empty-switch`) pass it
  to `SelectWeapon`. For a `WPN_Q_THROW_ARC` mode it runs a `TryKeep` dry run on that point: pass
  switches and latches `throwId`, `throwPoint`, `throwVel`, `throwHeard`; fail refuses the switch,
  keeping the select throttle. A bot never draws a grenade it has not decided to throw. A passing dry
  run on a grenade already in hand latches the same intent.
- **The execution.** `ThrowIntent` runs in `Think` before the no-target branch, and while it is live
  nothing else selects a weapon. Every frame, including the deploy (`why=drawing`, which gives the view
  the whole deploy to turn), it runs `TryKeep` on the live target if visible, else the latched point.
  The harm test is not re-run (`fire.heard` is false during the intent); the arc gap test and
  `SplashSafe` still apply. With a pin pulled the throw cannot be undone, so the bot keeps it
  (`why=throw-blocked`) and the game throws it. `Brace` holds the bot still for the whole intent, so it
  does not walk out of the latched arc during the 0.75 s CS deploy.
- **The end.** `CountShots` sees `throwStart` fall to 0, clears the intent, and for a heard intent adds
  `heard=` and starts the cooldown.
- **No clock.** The only wait is the grenade's own deploy. `DropIntent` runs on the first frame, with no
  pin pulled, where the grenade is not active (the switch is synchronous, so it failed), `TryKeep`
  finds no solution, or the grenade was ready last frame (`throwReady`) and `Buttons` did not pull the
  pin (typically an unsettled view; the bot does not wait for it).
- **A drop** counts in `dropped=`, starts `nextHeardTime`, selects the best weapon for the range
  skipping the grenade (`why=throw-dropped`), and if `BotCombat` is active bans grenades until that
  combat ends. `ThrowBanned` compares the stored `combat.startTime`, so the ban lifts on `Exit` or a
  fresh `Enter`; while it holds, `ChoiceEligible` rejects throw modes and `TryKeep` skips them unless a
  pin is pulled. Outside combat the heard cooldown alone stops the next draw. `Idle` clears the intent,
  so a break, deploy, ability or tank still takes the frame.

**Team-unsafe throwables** carry `WPN_Q_TEAM_UNSAFE`: the CS flashbang and smoke, the snark, the
penguin, and TFC concussion and gas (added to the off-hand mode in `OffhandRead`).
`BotFire::FriendCloser` rejects them when any live `REL_FRIEND` player is nearer the target than the
bot. It applies to every target, seen or heard: in `ChoiceEligible` for all passes, in `TryKeep` when
not committed, and in `AttackClear` for TFC. In deathmatch `Teams::Of` never returns `REL_FRIEND`.
Caltrops are `OFFHAND_SCARED` and never reach the attack path.

**CS flash and smoke are throwable.** They copy the HE grenade's arc with ReGameDLL's gravity 0.5 (HE
is 0.55, same speed), splash 350, as `AMBUSH_FLASH`/`AMBUSH_SMOKE`. Value 0, so selection only picks
them with no gun. 350 is a harm radius, not the flash radius, and doubles as the minimum throw range.

### The shot counters

Every counter is per life: `BotFire::Reset` runs on respawn from `BotManager::StartFrame`, so a drop in
any counter is a death, which is how `test_fire.ps1` counts deaths. They are what the tests assert on;
the sampled `why=` string sees one frame and misses most of what it describes.

| counter | witness |
|---|---|
| `shots` | the active weapon's attack timer goes ready to busy on a frame after the bot pressed. The press gate matters: deploys and `GetNextBestWeapon` arm the timer too. `TapButtons` sets the press flag (except for throws), or no semi-auto shot is counted |
| `leadShots`, `arcShots`, `stickyShots` | the same edge, gated on the press having been a solved lead / arc / sticky press, carried on the press flag since selection may change before the timer moves. Homing presses are not lead shots, so `leadshots < shots` on a pinned hornetgun proves the cone replaced the solve; `arcshots == shots` on a pinned MP5 proves nothing went through the ray path |
| `chargeShots` | falling edge of `BotWeapon::chargeState` (`m_fInAttack`), because `CGauss::StartFire` moves no attack timer |
| `throwShots` | falling edge of `BotWeapon::throwStart` (`m_flStartThrow`), because the hand grenade's press moves no timer; the press frame sets no press flag, so a throw is never counted twice |
| `guides` | rising edge of `COMMIT_GUIDE`: one per rocket |
| `commitCaps` | commits released by their cap; non-zero means a pin never reached the weapon or `m_cActiveRockets` never returned to 0 |
| `reloads` | rising edge of `wantReload`, a one-frame state |
| `uphillVetoes`, `selfVetoes`, `aimVetoes` | per-frame refusal predicates; read as yes/no |
| `targetAims` | frames that published an `AIM_TARGET` aim; how a pinned crossbow proves `primaryZoomed` was used |

Blind spot: `CGauss::PrimaryAttack` moves no weapon timer (only `m_flTimeWeaponIdle` and the player's
`m_flNextAttack`), so gauss primary shots, sticky or not, are never counted. The sticky test asserts on
the duel, where every bot carries a glock.

### What is not here

- **Deployables** (tripmine, satchel) are `bot_deploy.cpp`'s; see "Supported" above and
  `doc/games.md`, "The deployables".
- **`CRpg::m_fSpotActive`** is not read: the laser is on from `CRpg::Spawn` and the bot never presses the
  button that turns it off (`doc/games.md`).
- **Recoil compensation** and weapon preference weights do not exist.

## Off-hand grenades (bot_grenade.cpp)

TFC grenades are not weapons: `+gren1`/`+gren2` prime one without touching `m_pActiveItem` and
`-gren1`/`-gren2` release it, so the bot keeps firing its gun through the cook. Everything
game-specific is behind `OffhandRead` (per slot: `OffhandUse`, type, count, pin, fuse, range gate, the
ballistic `WeaponMode`), `OffhandCommand` and `OffhandPrimed`; every other profile returns nothing, so
the module is inert outside TFC. `BotGrenade::Run` runs in `Bot_RunPlayerMove` because the throw needs
the frame's view angles.

**What the game does** (`CTFPrimeGrenade`, tfc.c 101597-101947). Priming sets `tfstate |= 1` and a
think at +pin (0.8 s); the grenade explodes at prime + pin + prime time (3.0 s; caltrops 0.5 s), in
hand if still held. Release only sets `tfstate |= 0x400`; the throw happens at the next think (never
before +0.8 s) from the player's **origin** along `v_angle`: 600 forward + 200±10 up ± 10 right,
gravity 0.81, no inherited velocity. That is the grenade launcher's launch, so `ThrowLaunch` is
`DMC_ThrowLaunch`, and `BotFire::SolveOffhand` wraps `SolveThrowLead` to return a look direction and
flight time.

**Deciding.** Every 0.25 s with nothing cooking, the first slot that fits is primed with a 35% chance
(a failed roll waits 1 s; a finished throw waits 4-9 s):

- `OFFHAND_ATTACK` (frag, conc, nail, MIRV, napalm, gas, EMP): the sense target seen in the last 0.3 s,
  or else a heard enemy, aimed at its feet (origin + mins.z + 8) with its velocity, within
  [minRange, 650] and with no teammate within 300 of it. minRange is 250 for frag/napalm, 300 for
  conc/nail/gas/EMP, 350 for MIRV. 650 is about the arc's reach: 2fort's sightings are 1800-2200 out,
  so grenades are rare there and common on casbah.
- `OFFHAND_SCARED` (caltrops): when `BotCamp` is scared, toward the nearest live suspicion, at 60% of
  the way clamped to 200-300 (a caltrop scatters after about 0.5 s of flight), trace-clipped. Never
  with a teammate within 400 of the bot or the point.

**Cooking is a difficulty flag** (`crossbot_grenade_cook`, hard and expert). Easy and normal release as
soon as they have a solution, so their grenades arrive with most of the fuse left. While primed the shot
is re-solved every frame; a cooking attack grenade releases once fuse left is at most flight + 0.3 s;
a caltrop releases at once. If the target falls inside minRange or a friend walks near it, there is no
shot. At 1 s left the grenade is forced out toward the most open of eight directions at pitch -20
(`OpenLook`), never at a stale aim, which would kill bots with their own frags. The release angles are
held until the grenade has left (`primeTime + pin`, or the primed bit clearing, plus 0.1 s), and while
they differ from the planned angles `Bot_RunPlayerMove` re-decomposes `fmove`/`smove` for the new yaw
so the bot keeps walking where it was going.

Measured on casbah, 12 bots, 5 minutes: 55 throws (37 forced), 10 grenade kills, no grenade suicides.
The forced share is high because targets rarely stay in sight for a 3 s cook.

## The inventory (bot_weapons.cpp)

What the bot carries, read from private data through the offsets `gamedata.cpp` loaded
(`doc/games.md`, "Gamedata"). `BotWeapons::Refresh` runs from `BotManager::StartFrame` for every live
bot with no caching: it is a few short pointer chains, and a stamp check would cost more to reason
about. `BotWeapons::Read` takes any player edict, so `crossbot_weapons` dumps a human's inventory with
the same code. A profile may fill the inventory itself through `IGameProfile::ReadInventory` (DMC,
TFC, DoD); the shared item walk is `BotWeapons::ReadItems`.

### Validating every item pointer

Each `CBasePlayerItem *` from `m_rgpPlayerItems` is a raw heap pointer. Before any field is read: the
pointer is non-null, its `pev` is non-null, `pev->pContainingEntity` is a live edict with a plausible
index, and **`edict->pvPrivateData` points back at the same object**. Only that round trip can fail on
a pointer that is merely wrong, and it is why `pev` at offset 4 is hardcoded rather than trusted.

A failed check means the offsets are wrong, so it goes through `GameData::Fatal`. A null pointer is the
normal chain terminator and just ends the walk.

The read is wide on purpose: `m_iClip` and friends live on `CBasePlayerWeapon`, past the end of a plain
`CBasePlayerItem`. Half-Life has no non-weapon inventory item (the suit is a `pev->weapons` bit), so
every entry is a weapon. A mod that adds one would over-read; suspect that first if numbers go strange.

### Attack timers are stored raw

`nextPrimaryAttack`, `nextSecondaryAttack` and `nextAttack` are stored unnormalised. In retail
Half-Life they are **countdowns**: `UTIL_WeaponTimeBase()` is 0 under `CLIENT_WEAPONS`, and the player
decrements them and clamps at -1, so ready is `<= 0` and an idle weapon reads exactly -1.00. A few SDK
paths write the absolute `gpGlobals->time + delay` form anyway (`glock.cpp`'s empty click), so the
consumer decides. `bot_fire.cpp` decided on countdowns; it only presses the weapon it holds.

### Ammo counts are indices, never names

`m_iPrimaryAmmoType` indexes the player's `m_rgAmmo[32]`, assigned at runtime by
`CBasePlayer::GetAmmoIndex` in precache order. It is not stable across mods and must never be
hardcoded (on crossfire 9mm is 2 and rockets 6). It does not need to be: `inv.ammo[w.primaryAmmoType]`
is the count without knowing the ammo's name, which would need `CBasePlayerItem::AmmoInfoArray`, a
static the gamedata does not describe. Non-zero `ammo[]` entries with no weapon are normal: bots pick
up ammo for guns they do not have yet.

### The mask cross-check

`crossbot_weapons` ORs `1 << m_iId` over the walk and compares it with `pev->weapons` minus bit 31.
`pev->weapons` is engine-side, so it is the one number that does not depend on an offset; if walk and
mask agree, the offsets are good. Bit 31 is `WEAPON_SUIT`, never an inventory entity. DMC prints
`MISMATCH` on every bot, because its `pev->weapons` is meaningless (`doc/games.md`, "Deathmatch
Classic").

What the modes mean and how the button is driven is `weapon_spec.h` in `doc/games.md`: this file
answers "what am I holding", that one "how is it operated".
