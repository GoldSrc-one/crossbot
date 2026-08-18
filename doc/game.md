# Game System Comments

## game_profile.h

### IGameProfile
The mod-specific half of the bot. Everything a bot needs to know that is not geometry lives behind
this interface, so `src/bot/` never names a Half-Life classname or constant. `CHLDMProfile` is the
only implementation today and `GameProfile_Detect` returns a function-local static.

`g_pGameProfile` is a real global (defined in `bot_manager.cpp`) rather than a `BotManager` accessor
because it is read on nearly every item decision; an accessor bought nothing and cost a call.

`GetWeaponSpec(id)` is the one method that answers with a struct rather than a scalar, because what it
describes is a table rather than a policy — see `weapon_spec.h` below. It takes the raw `m_iId` the
inventory walk produces, so the caller needs no weapon vocabulary of its own, and returns null for any
id this game has no weapon for.

### ItemKind
The classification the gather goals key off. `ITEM_WEAPON`/`ITEM_AMMO`/`ITEM_PICKUP` are touched to
collect; `ITEM_HEALTHCHARGER`/`ITEM_RECHARGE` are used with `IN_USE` and never touched at all, which
is why `ItemKindIsCharger` exists rather than a comparison at each site — the two families share
almost no code path.

## hldm_profile.cpp

### ClassifyItem
Prefix match on `weapon_`, `ammo_`, `item_`, plus exact `weaponbox`, `func_healthcharger` and
`func_recharge`. Prefix matching rather than a table of every HLDM classname is what lets the bot
pick up mod items it has never heard of; the cost is that `SelectGoal` has to walk the whole edict
array instead of using `FIND_ENTITY_BY_STRING`, since that engine call takes an exact name.

**The recharger's classname is `func_recharge`, not `func_recharger`.** The wrong spelling silently
classifies every HEV charger as `ITEM_NONE` and the bot ignores them all.

`item_` catches `item_suit`, `item_antidote` and `item_security` as well. They are harmless: the bot
touches one, nothing happens, and the full list keeps it away for 60 s.

### The respawn constants
20 s weapons and ammo, 30 s items, 30 s HEV charger, 60 s health charger. These are HLDM's own
`WEAPON_RESPAWN_TIME`, `AMMO_RESPAWN_TIME`, `ITEM_RESPAWN_TIME` and the deathmatch branches of
`FlHEVChargerRechargeTime` / `FlHealthChargerRechargeTime`. They are a *prediction*, not a
measurement — nothing tells the plugin when an item actually rematerialised, so a bot that guesses
short simply walks up to an empty spot and learns from it (see `doc/bot.md`, case B).

### ChargerValue / ChargerFullValue
The charger runner needs one number to watch and one ceiling to stop at, and which `pev` field that
is depends on the charger. Health chargers read `v.health`, HEV chargers read `v.armorvalue`; both
cap at 100. Keeping this behind the profile is what stops `bot_move_nav.cpp` from having to know
which is which.

## gamedata.cpp

The bot's only way into the game DLL's own data. `edict_t::v` (`entvars_t`) is a shared struct the
engine defines, so the plugin can read it directly; everything else a `CBasePlayer` knows — the
inventory, clips, ammo counts, attack timers — lives behind `edict->pvPrivateData`, which is an opaque
blob of a class the plugin has no header for. `gamedata/valve-win.json` is a dump of that class layout
for retail `hl.dll`, and `GameOffsets` is the handful of byte offsets out of it that the bot needs.

### Offsets are absolute, not per-class

The JSON flattens the inheritance chain: `CBasePlayerItem::m_pPlayer` is 112, `CBasePlayer::m_pActiveItem`
is 1228, and `CBaseMonster::m_flNextAttack` is 592. Every number is already an offset from the start of
the object, so reading a base-class field out of a derived object needs no arithmetic — a player object
is read at 592 for `m_flNextAttack` exactly as a monster would be. That is why `GameOffsets` is a flat
struct of ints and not a per-class hierarchy.

`CBaseEntity::pev` at 4 is the one offset the plugin hardcodes rather than trusting, because it is the
offset it needs in order to check that any other offset was right (see `bot_weapons.cpp`). The loader
still reads it from the file and refuses to run if the file disagrees.

### Only fields common to every weapon

The requested set stops at `CBasePlayerItem` / `CBasePlayerWeapon` / `CBasePlayer` / `CBaseMonster`.
Per-weapon-class state — the Gauss charge, the Egon fire mode, the RPG's live rockets, `CShotgun`'s own
`m_fInReload` shadowing the base one — is deliberately absent. Adding it means reading a field whose
existence depends on which weapon the pointer happens to be, and none of it is needed to answer "what
am I holding and can it shoot".

### Every failure is fatal

A wrong offset is a wild read into another object's memory, which corrupts silently and crashes
somewhere unrelated. So there is no degraded mode and no validity flag anywhere in this feature: a
missing file, unparseable JSON, an absent field, or a `pev` that is not 4 all go through
`GameData::Fatal`, which prints the reason and takes the server down. A deployment that cannot describe
its own game DLL is a bug to fix before the server runs, not a condition to survive.

### Loaded at GameDLLInit, keyed by gamedir

`GameData::Load` runs from `GameDLLInit` (`dll_hooks.cpp`), not from `ServerActivate`: offsets cannot
change while `hl.dll` stays mapped, so it parses once per DLL load rather than once per map, and it
guards itself with a file-static flag.

It is keyed by gamedir — `<gamedir>/addons/crossbot/gamedata/<gamedir>-<win|linux>.json` — and not by
`IGameProfile`, for two reasons. `CHLDMProfile::Detect` reads `gpGlobals->deathmatch_`, which the
engine only fills in once it spawns a server, so profile detection cannot run this early and stays
where it is. And the gamedir is the better key anyway: these offsets describe a *game binary*, and the
gamedir is what selects which binary the engine loaded. `IGameProfile` gains nothing from this file.

## ent_handle.h

### CBotEntHandle
`EHANDLE` from `hlsdk/dlls/cbase.cpp`, retargeted at `edict_t` because the plugin has no
`CBaseEntity`. The serial number is the whole point: the respawn memo outlives the entity it names
by up to a minute, and edict slots are recycled, so a raw pointer would eventually resolve to an
unrelated entity that happened to reuse the slot.

**`Get()` checks `ent->free` as well as the serial number**, which the HLSDK original does not. It
does not need to — a `CBaseEntity` never outlives its edict — but a memo entry does, and a freed
edict keeps its old serial number until the slot is reused.

### CBotPlayerNameOf / CBotIsLivePlayer

Two one-liners in `ent_handle.cpp`, for walking `pfnPEntityOfEntIndex(1..maxClients)`. They lived as
statics in `commands.cpp` until the aim gaze scaffold needed the same scan from `bot_aim.cpp`; they are
here rather than duplicated because a second copy of "what counts as a live player" is exactly the kind
of thing that silently diverges.

`CBotClassOf` is declared alongside them but defined in `hldm_profile.cpp`, which is a wart — nothing
about it is mod-specific.

## weapon_spec.h

What the bot needs in order to *operate* a gun, as data rather than code. `src/bot/` never names a
Half-Life weapon: it asks `IGameProfile::GetWeaponSpec(id)` with the `m_iId` that `BotWeapons::Read`
already pulled out of the inventory, and gets back a `WeaponSpec` describing both firing modes. The
enums here carry no Half-Life constants, which is what lets `bot/` include them.

### The one criterion for a field

**Does it tell the bot how to operate this gun?** Facts the bot can already observe at runtime, and
facts that only matter when *choosing* between guns, are deliberately absent. The rejected-fields list
below is part of the design, not an oversight — several of those look obviously useful and are not.

Operating a gun decomposes into three independent questions, one enum each: how to drive the button
(`WeaponFire`), where to point (`WeaponAim`), and what persistent state the press flips
(`WeaponToggle`).

### Why FIRE_TAP is bot intent, not an SDK requirement

There is exactly one `ItemPostFrame` in the HLSDK (`dlls/weapons.cpp`). No weapon overrides it, no
weapon reads `m_afButtonPressed`, and `ITEM_FLAG_NOAUTOFIRE` does not exist in this SDK at all. The
dispatcher is level-triggered on `pev->button`, so **every Half-Life weapon is mechanically full
auto** — the python only feels semi-auto because its cycle time is 0.75s.

So `FIRE_TAP` never means "the weapon requires a release". It means the bot must not hold this button,
and the reason is always the same: holding a toggle flips it again every cycle. Anyone comparing this
table against the SDK will find no mechanism behind `FIRE_TAP` and be tempted to delete it. It is not
redundant.

`FIRE_CHARGE` is the inverse: releasing is what fires. `ItemPostFrame` only reaches `WeaponIdle()`
through its "no fire buttons down" branch, and that is where the gauss discharges and the handgrenade
leaves the hand. Both need the button held first and then released deliberately.

**`IN_ATTACK2` short-circuits `IN_ATTACK`** in that same dispatcher — the secondary test is an `if`
and the primary is its `else if`. A bot that sets both bits fires only secondary, every time.

### AIM_TARGET vs AIM_LEAD is "is the travel time zero"

Not "what comes out of the muzzle". A crowbar, a shotgun and an egon beam all produce one instruction
— point at the target — so melee, hitscan and beam are a single value. The crowbar's reach is `range`;
the egon's continuous damage is `FIRE_HOLD`.

Homing weapons are `AIM_LEAD`, not `AIM_TARGET`. A hornet crosses 300 units in about a second, a
RED-type hornet has to slow down to turn, and an MP hornet expires after 3.5s — so the correction it
makes for a lead the bot did not apply costs flight time it may not have. Homing makes the lead
*forgiving*; it does not make it unnecessary, and a tolerance is not a different instruction.

The RPG is the exception that stays `AIM_TARGET`. Its rocket chases the nearest visible `laser_spot`,
which sits wherever the bot is pointing right now, so it is tracked rather than led. "Keep pointing
after you fire" is a duration, not a place, so it is `WPN_Q_KEEP_AIMING`.

### gravity is effective, not pev->gravity

`pev->gravity` is a multiplier on `sv_gravity` — 0.5 is 400 u/s squared at the default 800 — and the
table does not pre-multiply it, because `sv_gravity` is a cvar.

**The table deliberately disagrees with the SDK in three rows.** The crossbow bolt sets
`pev->gravity = 0.5` *and* `pev->movetype = MOVETYPE_FLY`; MOVETYPE_FLY does not apply gravity, so the
bolt flies dead straight and that 0.5 only takes effect after `BoltTouch` mutates the movetype. The
hornet is the same, both modes. All three carry gravity 0 here. Copying `pev->gravity` verbatim would
make the bot aim low with the flattest-shooting weapon in the game.

Three more shapes a single `speed` float cannot hold, all recorded as quirks:

- The handgrenade's throw speed is pitch-dependent: `flVel = (90 - angThrow.x) * 4` capped at 500,
  over a pitch remapped so that looking straight down still throws. Level aim gives the 400 in the
  table; `WPN_Q_THROW_ARC` says that number is only the level-aim case.
- The crossbow bolt halves to 1000 underwater (`WPN_Q_SLOW_IN_WATER`).
- The RPG launches at 250 on `MOVETYPE_BOUNCE` pitched up 30 degrees, lobs for 0.4s, then ignites and
  accelerates toward a 2000 cap while homing. It is guided, so it is never led, and 250 is only ever
  the muzzle figure.

**Velocity inheritance is inconsistent** and changes the launch vector: the handgrenade, satchel,
snark and RPG all add the player's own velocity, and the MP5 grenade explicitly does not. Hence
`WPN_Q_INHERIT_VEL` rather than a blanket assumption either way.

### Why ammo is a field when the inventory already has counts

`BotWeapon` carries `primaryAmmoType`, `secondaryAmmoType` and the counts, so `WeaponAmmo` looks
derivable. It is not. "Secondary fire spends secondary ammo" is false for the shotgun (spends the
clip), the crossbow (both modes spend bolts) and the gauss (both spend uranium). Without this field
the bot cannot answer whether pressing IN_ATTACK2 will do anything at all.

`ammoPerShot` is 1 everywhere except shotgun secondary (2 from the clip) and gauss primary (2
uranium). It exists so a bot holding one shell does not press IN_ATTACK2 forever against a weapon that
will never fire.

### primaryZoomed, and why zoom is not a fire mode

Zoomed crossbow primary in **multiplayer** does not fire a bolt — it calls `FireSniperBolt()`, a plain
traceline for a flat 120 damage. Same button, same weapon, and the aim requirement flips from
`AIM_LEAD` to `AIM_TARGET`. That is the whole reason zoom is a `WeaponToggle` with its own mode slot
rather than a `bool`: it has to be able to change what the *primary* does.

`primaryZoomed.fire == FIRE_NONE` means "no override", so the consumer stays one line and no weapon is
special-cased in `bot/`:

    const WeaponMode &m = (zoomed && spec->primaryZoomed.fire) ? spec->primaryZoomed : spec->primary;

Zoom state is free to read: both toggles write `m_pPlayer->pev->fov`, which is engine-side, so no
gamedata offset is involved. `pev->fov != 0` is the whole test — which is also why the specific FOV
values (20 crossbow, 40 python) are *not* stored.

### The two weapons the bot cannot drive

`WPN_Q_HIDDEN_STATE` marks a mode whose behaviour depends on private state no offset exposes:

- **Satchel primary** is three different actions on `CSatchel::m_chargeReady` — throw at 0,
  radio-detonate at 1, inert at 2.
- **RPG secondary** toggles `CRpg::m_fSpotActive`, which also suppresses reloading while rockets are
  in flight.

Both classes are already described in `gamedata/valve-win.json`, so adding the two offsets is cheap —
but `gamedata.cpp` excludes per-weapon-class state on purpose (see "Only fields common to every
weapon" above), so that is a deliberate decision and not a thing to do in passing. Until then, a bot
must treat these two modes as unpredictable rather than guess.

### Rejected fields

Each of these is a true fact about the weapon that fails the criterion above. They are listed so they
do not get re-proposed:

| Rejected | Why |
|---|---|
| `MELEE` / `HITSCAN` / `BEAM` as separate aim values | One instruction between them. Reach is `range`, sustained fire is `FIRE_HOLD`. |
| `HOMING` as its own aim value | Still has flight time, still wants leading. It is `AIM_LEAD` with a wider tolerance. |
| `cycleTime` | The live `nextPrimaryAttack` on the weapon in hand is authoritative. A table value only helps rank guns you are *not* holding, which is selection. |
| `zoomFov` | `pev->fov != 0` answers "am I zoomed". 20 vs 40 changes no handling decision. |
| a `needsSurface` quirk | Redundant with `AIM_SURFACE`. |
| an `exhaustible` quirk | The inventory walk runs every frame; the bot sees the weapon disappear. |
| a `regenAmmo` quirk | Same — the bot sees the hornet count climb. Whether to wait or switch is selection. |
| a `selfDamage` quirk | Redundant with `splash > 0`. |
| a `selfPush` quirk | The gauss jump is a movement capability, not a firing instruction. |

Spread cones, pellet counts, damage values and projectile lifetimes are likewise not fields. Some are
noted below because they were expensive to establish, but they belong to the accuracy and selection
layers when those land.

## hldm_weapons.cpp

A flat `WeaponSpec[16]` indexed by `m_iId`, and nothing else. Indexing by the id the inventory walk
already produces makes the lookup free and keeps every string compare out of it.

**The table carries multiplayer values throughout**, because `CHLDMProfile` is a deathmatch profile
and several weapons differ:

| | multiplayer | single player |
|---|---|---|
| gauss full charge | 1.5s | 4s |
| MP5 cone | 3 degrees | 6 degrees |
| shotgun pellets | 4 / 8 | 6 / 12 |
| egon ammo drain | 1 per 0.2s | 1 per 0.1s |
| hornet lifetime | 3.5s | 5.0s |
| python zoom | works | returns immediately |
| crossbow zoomed primary | hitscan | still a bolt |

The MP5 cone is worth care: the branch guarded by `if (!IsMultiplayer())` takes
`VECTOR_CONE_6DEGREES`, so multiplayer gets the *narrower* 3 degrees. The comments inside that
function are swapped relative to the code.

`splash` radii come from `RadiusDamage`, which uses `flDamage * 2.5`. With retail `skill.cfg` that is
250 for the RPG, the M203 grenade and the handgrenade (100 damage each), and 375 for the satchel
(150).

### ammoPerShot means per press, except where there is no press

For every weapon that fires discretely, `ammoPerShot` is what one trigger pull costs. The egon and the
charging gauss have no discrete shot: the egon burns 1 uranium every 0.2s for as long as the button is
held, and the gauss burns 1 every 0.1s while spinning up. Both carry 1, which is the per-tick cost, not
a per-press one. A consumer that budgets ammo before committing has to multiply by the time it intends
to hold, and `chargeFull` is the only hint the table gives about that duration.

### Two holes, both real

`WEAPON_CHAINGUN` is id 5 in the SDK header and has no entity in retail Half-Life, so slot 5 is empty.
Slot 0 is `WEAPON_NONE`. `HLDM_WeaponSpec` returns null for both, which is how the console dump ends
up showing 14 weapons across 16 slots — and why `pev->weapons` on a fully equipped player reads
`0xffde`, with bit 5 clear.

### The lookup validates its own index

`HLDM_WeaponSpec` returns null when `spec.id != id`, not merely when the row is empty. A flat array
indexed by id has exactly one interesting failure — a row in the wrong slot — and that failure would
otherwise hand back a confidently wrong weapon. `crossbot_weaponspec` with no argument walks the raw
slots instead and prints `TABLE BROKEN` for any mismatch, so the condition is visible rather than
merely survived.

The other half of that check is `crossbot_weaponspec <name>`, which compares each spec's `classname`
against `CBotClassOf` on the live weapon entity. Getting an id wrong is the one mistake this file can
make, and a live join is the only thing that catches it.
