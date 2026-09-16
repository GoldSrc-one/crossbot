# Combat: fighting and firing

The combat state, the fire chain, and the inventory it reads.
Enemies come from `doc/perception.md`; what it interrupts is `doc/goals.md`.

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

### Two ways in, six ways out

Entry is sight (`sense.target`, which is line of sight *this frame*) or damage from a player.

`DamageSource` is the damage half, and its job is almost entirely rejection. `pev->dmg_inflictor` is
not the attacker: for a bullet it is the player, but for a rocket or a grenade it is the projectile,
which carries the shooter in `pev->owner` -- so the inflictor is resolved through `owner` when it is
not itself a player. Everything the bot should *not* pick a fight with falls out of that same
resolution rather than needing a list: falling, drowning and crush damage are inflicted by worldspawn
at edict index 0, a `trigger_hurt` is a brush entity no player owns, and a rocket the bot fired at its
own feet resolves back to the bot.

`Teams::Attackable` is checked on both halves of that resolution: on the monster branch, and on the
player the projectile resolved to. It should never fire -- friendly fire is off on every mod that
turns teams on, so a teammate cannot set `dmg_take` at all -- but this is the one entry into combat
that does not come through vision, and leaving it ungated would mean the whole feature rested on a
server setting nothing here reads.

The exit list gained one for the same reason. Combat persists on `BotSense::Senses` -- sound, damage
and events -- not on sight, so dropping a teammate out of *vision* would leave an already-open fight
running for the whole decay window if either side changed team mid-fight. `the enemy is on our team`
is checked before the monster exit, and the monster exit now ignores `m_hEnemy` for an enemy-team
monster, matching `MonsterCandidate`.

Exit, checked every frame:

| Rule | Reason string |
| --- | --- |
| `enemy.Get()` is null | `the enemy handle went stale` |
| `IGameProfile::IsDead` | `the enemy died` |
| a monster enemy no longer names us | `the monster stopped hunting us` |
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

**A hunting monster is an attacker like any other.** `DamageSource` accepts an inflictor that is not a
player when `MonsterEnemy` names this bot -- a snark bite reaches combat the same way a bullet does, and
does not need the `owner` fallback, since the snark *is* the inflictor. `EnemySees` returns true outright
for a non-`FL_CLIENT` enemy: a monster whose `m_hEnemy` is us has already noticed us by definition, so
`aware` is always 1 and there is no ambush to open with. It also must not read `v.v_angle` or `v.fov` off
a monster, which are not view angles there.

That adds the sixth exit: **`the monster stopped hunting us`**, when the enemy is a monster whose
`m_hEnemy` is no longer this bot. A snark re-targets roughly every two seconds and will pick someone else;
without this rule the bot would keep duelling something that had lost interest until the sight timeout,
which is 5s of shooting at a threat that is now someone else's problem.

### The takeover waits for maneuvers, the state does not

`Run` returns false while `Bot_ManeuverActive` -- airmove, ladder, elevator ride, activation, charger,
break, deploy. Those runners live below goal selection in the frame and steer along the waypoint list, so calling
`ClearGoalItem` mid-flight would drop a bot into whatever is under it. The state is still entered on
time, and `driving` records whether the movement half has actually started.

Two of those maneuvers are cut short instead of waited out, and only when the bot is actually hit:
a frame where combat is active, `BotDiff::CombatInterrupt()` holds and `DamageSource` names an
attacker calls `Bot_BailManeuver` from `Bot_MoveNav`, right after `Update`. A ladder ejects unless the
drop would be fatal (see `### Ladders`), and a charger is walked away from (see `### RunChargerUse`).
Airmoves, rides and activations still wait — there is nowhere to eject to mid-flight, and a ride is
the only way off its platform. Being *seen* is not enough: a climb that finishes in two seconds is
still worth finishing when nobody is shooting yet. The call lives in `Bot_MoveNav` rather than in
`Update` so combat never learns what a ladder is, and the frame order stays readable in one place.

The first frame `Run` does drive, it calls `BotItems::ClearGoalItem` once. That is what "path
following is terminated" means concretely: goal cleared, `hasGoal` false, waypoints gone. On exit,
`nextSelectTime` is zeroed so gathering picks something new on the next frame instead of idling out
the retry interval.

### The shuffle: one random direction, judged on exposure alone

Every `kProbeInterval` (0.7s), or on arrival at the last probe, the bot picks a random yaw, resolves a
standable point that way, and compares its `exposure` to the exposure where it stands. Lower wins. If
the probe is no better, the bot goes the *opposite* way instead -- unscored, by design: the algorithm
is a random walk biased toward cover, not a search.

**`exposure` is the only field here that scores a position independent of where the bot is looking,**
which is why the shuffle uses it alone. This used to be argued against a `focus[]` field on the vantage:
it measured the fraction of the visible set inside a cone around `watch[0]`, so it described one facing
rather than the spot, and blending it between probes with different main directions was meaningless.
That objection stands and the field is gone. Camping is now a separate discrete dataset -- `NavMesh::camps`,
each a position *paired with* the facing it was measured at (`doc/navgen.md`, *camp_sampler.cpp*) -- which is
what makes the measurement mean something. `bot_camp.cpp` reads it now; the shuffle is unchanged, and
the two never run together -- `BotCamp::Run` sits below `BotCombat::Run` in `Bot_MoveNav`, so a bot
that has an enemy shuffles and a bot that only *suspects* one camps.

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

### Whether the enemy has noticed, and what the bot does about it

Combat knew *that* a fight had started and never *how*. A bot rounding a corner behind an oblivious
opponent played the encounter exactly like one that had just been shot in the back: highest-`value`
loaded gun, open fire.

`enemyAware` is that missing fact, and it is nearly free. Combat is entered either on sight or on
damage, and **the bot already knows the sightline is clear** on a sight entry -- that is how
`TraceVisible` set `sense.target` in the first place. So "does he see me?" is a dot product against
the enemy's own forward vector and nothing else. No new traceline, which matters when vision is
deliberately budgeted to two per bot per frame.

The reciprocal is assumed rather than tested, and the two rays are not literally the same one:
`UpdateVision` clears bot-eye to a hitbox *bone*, `EnemySees` reasons eye to eye. They are near
enough. Getting it wrong costs one badly chosen opening weapon, not a broken state.

`pfnAngleVectors`, not `MAKE_VECTORS`, for the reason `bot_sense.cpp` gives at its own gaze term: the
macro writes `gpGlobals->v_forward`, which a caller above may still be holding. A nonzero `v.fov`
means the player is **zoomed**, so it is a narrower cone than the 90 degree default, not a wider one.

**The flag latches on.** It is evaluated at entry and re-tested every frame it is still false, and
once true it stays true -- nobody unsees someone. Four sites set it, and the third is the one that is
easy to miss:

| Site | Value |
| --- | --- |
| sight entry | `EnemySees` |
| damage entry | `true` -- they shot us, no cone test improves on that |
| **retarget** | recomputed; that branch rebinds `st.enemy` *without* going through `Enter`, so a flag written only in `Enter` would hand the new opponent the last one's verdict |
| every later frame with sight, or on damage | `EnemySees`, or `true` |

`SetAware` is the only writer of the true edge, and it does one more thing: it zeroes
`fire.nextSelectTime`. Selection is throttled to `kSelectInterval`, so without that a bot caught
mid-ambush spends half a second holding a crowbar at someone already aiming back. That single line
*is* the "switch to something reasonable once he sees you" rule; the rest of it was already there.

### The ambush is a restricted pick, not a new policy

While `Ambushing`, `BotFire` opens with melee if it can and a throwable otherwise.

Almost all of this is existing machinery pointed at a smaller set. `PickBest` already ranks by best
`value` then *least carry time* with a reservoir draw on ties, so restricting it to the profile's
ambush weapons gives "a random throwable weighted by rarity" with no second scoring rule -- the hand
grenade and the snark are both worth 0.1, tie on value, and the least-carried one wins. Likewise
melee needs no range constant of its own: `BuildChoices` only offers the crowbar when `dist` is
inside its `range` of 32, so "in melee range" is already computed.

`IGameProfile::AmbushWeapon(AmbushKind)` names them, beside `MeleeWeapon` and `DeployWeapon`. It has
to be a profile hook rather than a quirk test: `WPN_Q_THROW_ARC` does identify the hand grenade
uniquely, but **nothing distinguishes the snark from the hornet gun** -- both are `WPN_Q_HOMING`, and
the snark is deliberately not `IsArc`.

**The attempt happens before `TryKeep`, and that placement is the whole trick.** `PickChoice` is the
obvious home and it is wrong: `TryKeep` returns the moment the held weapon still works, so a bot
holding a loaded MP5 beside an oblivious enemy would never reach the picker. Three guards on the
block, each load-bearing -- not while `holding`, because a commit in flight may be a *primed* grenade
and switching away leaves it cooking; not when an ambush weapon is already drawn, or it re-selects
every frame and `TryKeep` never gets to throw it; and not once the enemy knows.

**One thing `BuildChoices` does not check is a minimum range.** `ModeUsable` gates the maximum only,
and no `WeaponMode` has a minimum-range field -- self-blast is caught much later, at press time, by
`SplashSafe`. Left alone the bot would draw a grenade at knife range and then never throw it. So the
throwable pass rejects a mode when `dist < m.splash`. The same gap exists for the RPG and the MP5
contact grenade on the ordinary path; widening the fix to those is a separate change.

**Expect a good share of ambushes to decline themselves, correctly.** The uphill veto refuses any
`gravity > 0` non-detonating mode at a target more than 16 units above the eye, which covers *both*
the grenade and the snark, and the `AIM_LEAD` flight-time gate caps the grenade near 600 units and the
snark near 300. When nothing survives, `PickAmbush` returns -1 and the frame falls through to ordinary
selection -- which is exactly "blast him with whatever is in hand", written by not writing it.

**Three passes, in order: reach, thrown, last.** `PASS_REACH` is melee with a finite `range`, which is
the old contact pass; it had to stop being "any melee" once the grapple joined the melee set, because
its range is 0 and `BuildChoices` offers it at every distance -- the ambush would have opened with it
every time. `PASS_THROWN` is the grenade, snark and penguin. `PASS_LAST` is `AMBUSH_GRAPPLE` alone,
with no distance or splash gate: out of grenades but holding a grapple, the bot draws it at any range
and fires the tongue.

### Mirroring an opponent who draws melee

If the enemy has melee out, the bot draws melee too and fights the duel properly: close, swing, back
off, close again. Two flags carry it, and neither latches -- unlike `enemyAware`, a duel is over the
moment the opponent draws a gun.

`enemyMelee` is `IsMeleeWeapon(BotWeapons::ActiveWeaponId(enemy))` -- a set, since OP4 has a knife,
a pipewrench and a grapple beside the crowbar (`doc/games.md`, `gearbox_profile.cpp`). That reader is new and
deliberately not `BotWeapons::Read`: the full walk exists to build an inventory, and it validates
every slot through `ItemEdict`, which calls `GameData::Fatal` on a bad pointer. That is a reasonable
trade once a frame for a bot's own inventory and an unreasonable one for a question this narrow, so
`ActiveWeaponId` is two derefs -- `m_pActiveItem`, then `m_iId` -- with no validation, returning 0 for
"no idea". `m_pActiveItem` is a `CBasePlayer` offset, so the read is gated on `FL_CLIENT`; monsters
never duel. `crossbot_weapons` cross-checks the shortcut against the full walk's own `activeIndex`
and prints match/MISMATCH, in the same spirit as the existing `pev->weapons` mask check.

**The flag is only refreshed on a frame the bot can see them.** Reading private data works through a
wall, but a human reads the crowbar off someone's hands, and that is the same restraint `EnemySees`
already observes. Between sightings the last verdict stands, which `kSightTimeout` bounds at 5s.

`enemyInArea` asks whether the enemy stands in **the bot's own area** -- deliberately that way round.
"Which area is the enemy in" would be `FindAreaNearest`, which is up to three full linear passes over
every area in the mesh and has no spatial index; "is the enemy in *this* one" is `FindAreaById`, an
O(1) direct index, plus a single `AreaContainsPoint`. It uses `TestArea`'s idiom for `TestArea`'s
reason -- nav areas are duck-origin z-space, so the honest test is XY against the area's own centroid
z -- and then a 72-unit z window, because XY containment alone matches someone on a walkway directly
overhead who cannot be charged.

Both edges of the combined predicate zero `fire.nextSelectTime`. Entering, so the bot does not spend
`kSelectInterval` holding a shotgun at a charging crowbar; leaving, so it does not spend it holding a
crowbar at someone who just drew an MP5. `SetAware` already does this in one direction.

Selection is the ambush block's shape with one difference. `PASS_MELEE` already exists and is a pure
identity test against `MeleeWeapon()`, so `PickBest` on that pass is the whole picker -- but
`BuildChoices` is called with `geometry=false`, routing through `ModeViable` instead of `ModeUsable`.
That drops the range gate while keeping the pin and ammo checks. The ambush wants a weapon usable
*now*; the duel wants one usable *after walking*, and gating on range here would refuse the crowbar at
every distance the bot still has to close. The block sits after the ambush and before `TryKeep`: an
unaware enemy is the better opportunity, and `PickAmbush` tries melee first anyway, so at contact
range the two agree.

### The duel is a scoped reintroduction of approach movement

`BotCombat` deleted the chase on purpose, and the shuffle above scores spots on exposure alone with
the enemy explicitly not an input. The duel is the one exception, and it *replaces* the probe rather
than blending with it -- `haveProbe` is dropped on entry, or a stale `probePoint` resurfaces the
moment the duel ends.

It is reachable only with melee **actually in hand** against a melee opponent in the same area. The
weapon in hand is the condition rather than the intent to hold it, so a failed selection cannot leave
a bot charging bare-handed. Three states:

| Condition | Movement |
| --- | --- |
| swung within `kDuelBackoff` (0.4s) | away from the enemy |
| `dist > range * 0.8` | toward the enemy |
| otherwise | stand still and swing |

The reach comes from the *held* melee spec's own `primary.range`, never from a constant here. A range
of 0 is the grapple's "unlimited": the charging row is skipped, so a grapple duel only swings and backs
off, and the tongue does the closing. 0.4s is a
little under the crowbar's own refire, so the bot is back in range for the next swing. In practice
`swinging` is the rarest of the three: the bot fires as soon as it is inside 32 units, which puts it
straight into the backoff window.

Steering goes through `ResolveSpot`, not at the raw origin. Areas merge into **non-convex** prisms, so
a straight line between two points in one area can still leave it; when `ResolveSpot` fails there is
nowhere safe to charge and the probe gets the frame back. Everything below the direction choice is
untouched -- `SolveDrive`, the ducking divisor, `Bot_DecomposeMove`, the deadzone, the crouch flag.
Negating `moveDir` to retreat is what `bot_break.cpp` already does for its standoff.

Aim is untouched too, which is what turns the retreat into a backpedal rather than a turn-and-run.

**Knowing a swing happened.** There is no per-shot callback anywhere; `fire.pressed` is a one-frame
flag that `CountShots` consumes on the following frame. The one corroborated signal is the rising edge
on the weapon's refire countdown, and that closes the loop for instant-hit weapons specifically --
the crowbar named first. `fire.lastShotTime` is set there, beside every `++fire.shots`.

**Two mirroring bots never back down.** Both draw crowbars and both charge, which is the intended
outcome, but `kSightTimeout` and death are then the only exits. If that reads badly in play the fix is
a duel timeout, not a change to the mirror rule.

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

### The sticky trigger spans two mechanisms, because the window outlives the aim queue

A bot that releases the trigger the instant `RayOnTarget` fails stutters in a way no human does. A
player holding an MP5 does not blip the trigger every time the crosshair crosses the edge of a hitbox,
and does not stop the moment someone steps behind a doorframe. `WPN_Q_STICKY` (see `doc/games.md` for
which seven modes carry it and why it is not a `cycleTime` field) says the trigger may stay down.

The condition is `gpGlobals->time - bot.sense.targetTime <= kStickyWindow`, half a second.
`sense.targetTime` is stamped every frame the target passes `TraceVisible`, so it is exactly "last
seen" and not "last aimed at". **The "am I in combat" half of that needs no separate test**: combat is
already `now - targetTime <= kCombatMemory`, three seconds, so half a second is inside it by
construction.

What makes this two changes rather than one is that the trigger has to survive a boundary. `ReadDelayed`
returns false once the newest queued sample is older than `crossbot_aim_delay`, so from the last
visible frame:

```
0 ───────────── 0.25s ───────────── 0.5s
  haveTarget true      haveTarget false      window closed
  aim point served     mode would be         ClearAim + Idle
  → relax the box      dropped by Idle       (today's behaviour)
    test in Buttons    → hold it in Think
```

**Before the boundary, `Buttons` relaxes the box test.** `RayOnTarget` had three different refusals
behind one `bool`, so it now returns a `RayVeto` — `RAY_HIT`, `RAY_WIDE`, `RAY_BLOCKED`, `RAY_SELF` —
and sticky overrides **`RAY_WIDE` alone**:

- `RAY_WIDE` is "the crosshair has not converged", which is precisely what a spraying human looks like.
- `RAY_BLOCKED` is the one case where the bot has positive knowledge the shot cannot arrive. Spending
  ammo against positive knowledge is a bug, not humanness.
- `RAY_SELF` cannot happen here at all: **every sticky mode has `splash == 0`**.

The honest cost is that a sticky press runs no wall trace, because `RAY_WIDE` returns before
`RayOnTarget` traces anything. A bot that loses its target while facing a near wall sprays it for up to
half a second. All seven modes are cheap-per-shot hitscan, beam or hornet, so the waste is bounded, and
letting `RAY_BLOCKED` through as well is a one-line change if it ever reads better.

**After the boundary, `Think` keeps the state alive.** The no-target branch used to be "not holding →
`ClearAim` + `Idle`", and `Idle` drops `fire.mode`, which sends `Buttons` to `IdleReloadButton`. It now
tests sticky between the holding case and the idle case, and **skipping `ClearAim` is the whole
trick** — the same trick `COMMIT_GUIDE` already uses. `aimValid` stays true, `aimPoint` stays parked on
the last known position, and `BotFire::AimPoint` keeps serving it with no new plumbing. `fire.mode` and
`fire.weaponId` still hold last frame's selection, so `StickyHold` needs no new state to know which
mode to test; it looks the spec up and requires the active weapon to still match.

That second half needs **no second special case in `Buttons`**. Once the state is alive the ordinary
path runs against the stale aim point: crosshair still parked on it and the bot fires with
`why=firing`, crosshair drifted off and the `RAY_WIDE` override presses anyway.

Like `holding`, the sticky branch returns before the selection loop, so it bypasses `ModeUsable` for
the same reason and the same half second. It self-clears when the window closes.

`stickyShots` rides `fire.pressedSticky` through `CountShots` exactly as `leadShots` rides
`pressedLead`, so it is a real per-life shot count rather than a frame tally — and, as ever, the thing
the test asserts on, never `why=sticky`.

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
`CHLDMProfile` (see `doc/games.md`), so nothing here knows a number that belongs to a weapon.

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
| `COMMIT_SPINUP` | press one frame | — (time only) | `chargeFull + kSpinupSlack` elapsed | the same |

`COMMIT_SPINUP` is the displacer's: its ball leaves about a second after the press, aimed along the
view at that moment. Nothing observable marks the launch, so the commit is purely timed. While it runs
the buttons stay released, the aim keeps tracking (no `ClearAim` when the target drops out, the same
exception as guiding), and `AimBase` leads the target by `chargeFull` on top of the flight time.

The two new ones share a structure the charge does not need: the bot **cannot see its own action
immediately**. `CHandGrenade::PrimaryAttack` pulls the pin and returns; the grenade leaves half a
second later, from `WeaponIdle`. `CRpg::PrimaryAttack` creates the rocket inside
`Bot_RunPlayerMove`, so the count only reads back on the next frame. So `commitSeen` latches the frame
the game first shows the thing existing, and the release is `commitSeen && !live` — never `!live`
alone, which would fire on the gap between the button and the effect.

**`COMMIT_THROW` opens only for a `WPN_Q_THROW_ARC` mode.** Its whole reason is the delayed grenade,
and its latch is `m_flStartThrow`. Any other `FIRE_TAP` mode — Ricochet's disc, whose throw leaves on
the press — just presses. Before that gate a disc throw opened a commit that could never latch (no
`m_flStartThrow` exists there), held the mode unchecked as `why=primed`, and ran to `kTapTimeout` every
time.

`COMMIT_THROW` also refuses on `m_flStartThrow` directly, not only on its own state, and that is not
belt and braces. A primed grenade whose target walks out of the 550-unit reach makes the solve fail,
which drops the mode and clears the commit — and a bot that then pressed again would skip
`WeaponIdle` for that frame and **cook a grenade it has already pulled the pin on**. Reading the
weapon's own flag closes it, and picks up the free case of a bot that inherits somebody else's primed
grenade off the floor.

**Both `COMMIT_THROW` and `COMMIT_GUIDE` are entered only when the weapon's own attack timer has
expired, and that condition is load-bearing.** `Buttons` checks the *player's* `m_flNextAttack`
(`inv.nextAttack`) before it reaches either, and that is a different timer from the weapon's. A commit
opened on a press the weapon rejected can never latch `commitSeen`, so it runs to its cap and wastes
the whole valve.

The grenade proves it. `CHandGrenade::WeaponIdle` sets `m_flNextPrimaryAttack = 0.5` *after* the throw
(`handgrenade.cpp:178`), while `m_flStartThrow` goes back to zero at the same moment. So the guards
that read pin state — `commit == COMMIT_THROW`, `throwStart != 0` — both clear half a second before
the weapon will accept another press, and a bot with a target still in range presses into that gap.
`ItemPostFrame` skips `PrimaryAttack`, no pin is pulled, and 1.5s later `kTapTimeout` fires. It cost
six capped commits in a 60-second pinned phase before the timer check went in.

`COMMIT_GUIDE` had the same hole for a different reason.
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
`doc/games.md` is finally reachable instead of aspirational.

**Zooming costs turn rate and buys precision, and `FovScale` prices both.** It divides hand speed by
`fov / 90`, so at the crossbow's fov 20 the bot's peak turn rate falls from roughly 450°/s to 100°/s —
and it divides the hand quantum by the same factor, so the bot places the crosshair 4.5x finer. See
"The hand has a resolution" in `doc/perception.md` for the measurements. That is a genuine trade rather than a pure
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

### Selection takes the best gun it has, and the rarest of those

One caller reuses this ranking on a restricted set rather than adding a rule of its own: the ambush
pick in `bot_combat.cpp` above runs `PickBest` over melee only, then over the profile's ambush
weapons only. It is attempted *before* `TryKeep` rather than inside `PickChoice`, because `TryKeep`
returns early whenever the held weapon still works.

Two numbers rank a weapon, applied in that order.

**Score** is `WeaponSpec::value`, the static ladder already in the weapon table (`doc/games.md`,
"`value` is the one field that fails the criterion on purpose"): 0.5 for gauss, egon and crossbow, 0.3
for RPG and mp5, 0.2 for shotgun, hornetgun and python, 0.1 for everything else. Selection keeps only
the candidates at the single highest score present and discards the rest outright. There is no
tolerance band: a 0.3 never competes with a 0.5.

**Carry time** is seconds the weapon has been in the bot's inventory, one float per weapon id in
`BotFire::Carry` on `CBot`. Among the survivors of the score filter, **least carried wins** — that is
what "rare to him" means. The gauss a bot almost never finds beats the crossbow it has been lugging
around for two minutes, and a weapon just picked up beats everything.

`AccrueCarry` runs once per bot per frame from `BotManager::StartFrame`, immediately after
`BotWeapons::Refresh` — the one place the inventory is known fresh — and charges `Bot_FrameTime()` to
every carried weapon. Dead bots `continue` before that point, so the clock only runs while the bot is
alive, which is what "has the weapon" means. That single call site is also why there is no
double-accrual stamp: the hazard was `Buttons` running twice per frame in the jump split, and
`StartFrame` is not part of `Bot_RunPlayerMove`.

The table lives on `CBot` rather than in `BotFire::State`, because `Reset` assigns a fresh `State` on
every respawn and the tally has to outlive a life. The inventory does not outlive one — it resets to
the spawn kit on every death — and the gap between the two is the point: the crowbar and glock climb
life after life while a gun the bot rarely finds stays near zero. That divergence *is* the rarity
signal. Only `bot = CBot()` clears it — map change, disconnect, deactivate.

Mode does not enter the ranking. "Time in inventory" is the same number for a weapon's primary and its
secondary, so once a weapon wins, one of its usable modes is drawn at random through the per-bot RNG.
The draw happens once at select time and `fire.pendingMode` carries it through the switch, so it does
not flap frame to frame. Zoomed variants never appear as candidates at all: `BotFire::Mode` has no zoom
member, because zoom is a range decision made inside a mode rather than a mode of its own.

**What this replaced, and why.** The first cut of selection counted seconds on target per
`(weapon, mode)` pair and spent the least-used one. The metric was unreliable. It accrued only in
combat, downstream of every readiness gate, so it measured how often the bot happened to be in a fight
while holding a gun rather than anything about the gun — and a weapon carried for ten minutes and never
fired read exactly like one never picked up. Carry time is observable every frame, needs no fight to
mean something, and answers the question the humanness goal actually asks: is this weapon novel to this
bot.

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

The reload pass is deliberately **not** score-filtered. Topping up a magazine is inventory tidying, not
a decision about which gun to fight with, and a bot that only ever reloaded its best weapon would leave
the rest of the bag empty for the moment that weapon runs dry. Carry time breaks ties between equal
deficits, for no stronger reason than that it is the number already to hand.

Only when nothing needs rounds does the score-then-rarity ranking run. **Melee is excluded from it**
and reachable only as the last resort when no other candidate is usable. Score nearly handles this on
its own — the crowbar's 0.1 loses to anything better — but at spawn the crowbar and the glock are both
0.1, and the melee pass is what stops the bot settling on the crowbar there.

**Out of combat the held weapon wins ties**, now lexicographically: the bot switches only when the
winner strictly beats what it holds on score first, then on carry. Equal score and equal-or-greater
carry means stay put. Without it two same-score weapons whose carry times are converging make an idle
bot flap between them every three seconds. A held melee weapon is exempt and loses ties on purpose: a
bot idling with a crowbar and a loaded shotgun in the bag should be holding the shotgun.

In practice this rule fires far less than its predecessor did, because score is stable where usage
drifted. An idle bot with a gauss and a glock settles on the gauss and stops re-evaluating, instead of
alternating on every tick — which is the visible behaviour change from the rewrite, and the reason the
test suite now asserts where a bot *ends up* rather than that it did not churn.

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

`crossbot_weapon_carry` prints the table, each weapon as `carry/score`. `crossbot_fire_pin` is still the only lever that reaches
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

### Brace: stop and duck to fire

`WPN_Q_BRACE` is for guns whose cone depends on the shooter's stance. In OP4 that is only the M249, and
it reads the command's *button bits*, not velocity: `IN_DUCK` 3°, any of `IN_FORWARD/BACK/MOVELEFT/
MOVERIGHT` 15°, otherwise 6°. The combat drive sets those bits whenever it strafes, so a strafing bot
used to spray at 15°. CS and DoD weapons are the expected next users.

`BotFire::Brace` is called from `Bot_RunPlayerMove` right after `Buttons`, for the same reason the
trigger lives there: it is the one funnel every movement path goes through, and `Think` has already
decided this frame's weapon and mode. When it answers yes, the move is zeroed, the four move bits
are cleared, and `IN_DUCK` is added if ducking is allowed.

It answers yes only when all of these hold:

- the bot wants to fire: a mode is chosen, the aim is valid, and the mode carries the quirk;
- it is on the ground, not on a ladder (`MOVETYPE_FLY`) and not swimming;
- nothing asked for a jump this frame;
- the weapon will be ready by the time the bot can stop: not reloading, and the larger of the mode's
  timer and the player's `m_flNextAttack` is at most `NavPlan::StopTime(speed2D)`.

**The readiness rule is what stops it flickering.** `StopTime` is HL ground friction, above
`stopSpeed` exponential and below it linear: `ln(v/stop)/f + 1/f`, about 0.54 s from 320 u/s. A
stopped bot has a stopping time of 0, so any real wait (a reload, a slow cycle) releases it, and it
brakes again exactly when the remaining wait falls under its current stopping time. The M249's 0.067 s
cycle keeps it braced through a burst; its 3.78 s reload lets it move.

**Ducking needs a clear line from the ducked eye.** A trace from the eye 34 units lower (or the current
eye, if already ducked) to `fire.targetPoint` decides it, cached for 0.1 s. Blocked means the bot still
stops but stays standing, so waist-high cover does not blind it. `crossbot_fire_status` reports
`bracing=1` (ducked) or `2` (standing) and a per-life `braces=` count; `crossbot_brace 0` turns it off
for A/B runs.

### Aiming at the feet, and throwing from a moving frame

`AimBase` shifts the predicted point before any solve. `WPN_Q_AIM_FEET` drops it to the target's
feet, the height of its origin plus `mins.z`, measured against the sensed point so the offset holds
for whichever bone was sampled. The displacer needs this because a direct hit only teleports the
target. `WPN_Q_SPINUP` pushes the point ahead by `chargeFull` seconds of target velocity.

A plain arc with `WPN_Q_INHERIT_VEL` (the spore grenade: `800 * forward + player velocity`, gravity 1)
is solved in the thrower's frame. There the projectile leaves at `dir * speed`, and the target moves at
`vel - botVel`, so `SolveArcLead` is called with that relative velocity. The world impact is the
answer plus `botVel * flight`, and the launch handed to the simulator is `dir * speed + botVel`.
Gravity is the same in both frames, so the solve itself is untouched. `ArcOnTarget` adds the same
drift. The MP5 grenade carries no `INHERIT_VEL` and takes the old path unchanged.

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

`stickyShots` is the same press-flag mechanism as `leadShots` and `arcShots`, and it inherits their
one blind spot: **`CGauss::PrimaryAttack` moves no weapon timer**, only `m_flTimeWeaponIdle` and the
player's `m_flNextAttack`, so the `fired` edge never sees a gauss primary shot and never has. Gauss
primary is sticky and its sticky shots do not reach the counter. That is why the test asserts on the
duel, where every bot carries a glock, rather than on a pinned gauss.

`reloads` counts the rising edge of `wantReload`, because the state itself lasts one frame and is
invisible to any poll. `selfVetoes` counts refusals to fire into a blast nearer the bot than the
enemy, and like `uphillVetoes` it is a per-frame predicate, so read it as a yes/no. It is the only
refusal witness that survives a two-second poll — the `why=` string samples one frame and misses
almost everything it describes — which is why the arc phases assert on the counter and treat the
reason strings as a bonus. `aimVetoes` is the same shape for the aim gate, and `targetAims` counts
frames that published an `AIM_TARGET` aim, which is how a pinned crossbow proves `primaryZoomed`
was selected when the sampled `aim=` field can only witness it by luck.

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
button, is not here — that is `weapon_spec.h` in `doc/games.md`. This file answers "what am I holding",
that one answers "how is it operated".
