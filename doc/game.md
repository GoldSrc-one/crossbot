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

`ThrowLaunch(mode, viewPitch, elevation, speed)` is here for the one thing a table of floats could not
hold: a weapon whose launch direction and launch speed are the *same* parameter. It maps the pitch the
bot is holding to what leaves the hand, and returns false for any mode without `WPN_Q_THROW_ARC`. It
maps that way round rather than the reverse on purpose — the pitch is what the bot commands and what
the fire gate re-measures, so nothing anywhere has to invert the game's own remap. See
`WPN_Q_THROW_ARC` under `weapon_spec.h`.


`DeployWeapon(kind)` and `DeployEntity(kind)` are the two halves of the same rule `MeleeWeapon()`
follows: `src/bot/` must not name `weapon_tripmine` or `monster_satchel`. `DeployKind` is
`DEPLOY_TRIPMINE` / `DEPLOY_SATCHEL`, and the pair answers "which weapon places one" and "what does a
placed one look like in the world". `bot_deploy.cpp` needs both — the first to draw and fire, the
second to check whether a gate is already mined, to confirm a placement landed, and to find the
bot's own live charges — and `bot_avoid.cpp` needs the second to recognise a beam's owner. A mod with
no deployables returns 0 and null and the whole feature idles.

`AmmoEntity(weaponId, pool)` is the mapping from a weapon and one of its ammo pools to the entity
classname that refills it, and it is here for the same reason `ThrowLaunch` is: `src/bot/` must not
name `ammo_ARgrenades`. It exists because the pin now equips (see `bot_fire.cpp` in `doc/bot.md`), and
"give this bot ammo for the mode it is pinned to" cannot be answered from `WeaponSpec` — the table
records which *pool* a mode spends, not which pickup fills it, and the two are not one-to-one. It
returns null where no entity fills that pool, which is the crowbar and the hornetgun's self-regenerating
hive.

Three rows in `CHLDMProfile`'s table are the reason it is a table and not a prefix rule: MP5 secondary
takes `ammo_ARgrenades` while its clip takes `ammo_9mmAR`; the gauss and the egon share uranium under
two different classnames; and the handgrenade, snark, tripmine and satchel are their own ammo, so the
mapping returns the weapon classname back.
`EvaluateCombat(pEdict, inv)` scores how ready a bot is to fight, in `[0,1]`, and is the only input
that decides whether an enemy or an item wins a goal selection — see `doc/bot.md`, "Readiness is what
makes the comparison mean anything". It is mod policy rather than geometry for the obvious reason: what
counts as well-armed is entirely a property of the game's weapon set.

It is the one profile method that takes a `BotInventory`. `game_profile.h` forward-declares the struct
rather than including `bot_weapons.h`, so the header still names nothing from `src/bot/`; only the
implementation does. Passing the inventory rather than re-reading it keeps the per-frame walk in
`BotWeapons::Refresh` the single reader of the game DLL's private data.

`CHLDMProfile::EvaluateCombat` is survivability times firepower, a product rather than a weighted sum,
so a bot at death's door with a full arsenal and a healthy bot with nothing loaded both score near
zero — either half alone is enough to make hunting a bad idea:

    survive = (health + armorvalue) / (maxHealth + maxArmor)
    arms    = sum of WeaponSpec::value over every armed weapon, clamped to 1
    result  = survive * arms

`maxHealth` reads `pev->max_health` and falls back to 100; there is no pev field for maximum armor, so
it is a constant. "Armed" means holding a weapon with a mode that can actually deal damage and has the
ammo to do it — `BotFire::WeaponArmed`, which reuses the existing `AmmoReady` rather than restating the
clip and reserve rules. A mode with `AIM_NONE` is skipped there, which is what stops a dry python
counting as armed on the strength of its zoom toggle.

### ItemKind
The classification the gather goals key off. `ITEM_WEAPON`/`ITEM_AMMO`/`ITEM_PICKUP` are touched to
collect; `ITEM_HEALTHCHARGER`/`ITEM_RECHARGE` are used with `IN_USE` and never touched at all, which
is why `ItemKindIsCharger` exists rather than a comparison at each site — the two families share
almost no code path. `ITEM_HEALVOLUME` is a third family: a healing `trigger_hurt` the bot only has
to stand in.

Three predicates rather than one, because the families group differently at different sites.
`ItemKindIsCharger` still means "aim at it and hold `IN_USE`". `ItemKindIsBrush` means "read the
absbox midpoint, `pev->origin` is `0 0 0`" and covers chargers and volumes. `ItemKindUsesValue`
means "there is a `ChargerValue` to watch instead of a touch to wait for", and is what `OnTouch`,
`OnArrived` and `UpdateGoalItem` branch on. `OnTouch` is the one that had to change: a trigger is
`SOLID_TRIGGER` and really does fire `pfnTouch` on the bot, so the old charger-only test would have
ended the goal the instant the bot waded in.

## hldm_profile.cpp

### ClassifyItem
Prefix match on `weapon_`, `ammo_`, `item_`, plus exact `weaponbox`, `func_healthcharger`,
`func_recharge` and `trigger_hurt`. Prefix matching rather than a table of every HLDM classname is what lets the bot
pick up mod items it has never heard of; the cost is that `SelectGoal` has to walk the whole edict
array instead of using `FIND_ENTITY_BY_STRING`, since that engine call takes an exact name.

**The recharger's classname is `func_recharge`, not `func_recharger`.** The wrong spelling silently
classifies every HEV charger as `ITEM_NONE` and the bot ignores them all.

`item_` catches `item_suit`, `item_antidote` and `item_security` as well. They are harmless: the bot
touches one, nothing happens, and the full list keeps it away for 60 s.

**Why it takes an edict and not a classname.** A `trigger_hurt` is a healing pool or a death trap
depending on the sign of `pev->dmg`, and `pool_party` ships one of each (`dmg -40` and `dmg 200`).
Nothing in the classname separates them, so the signature had to widen; `CHLDMProfile` is the only
implementation and every call site already held the edict. `IsHealingHurt` wants `dmg < 0` and no
`SF_TRIGGER_HURT_NO_CLIENTS`. It deliberately does not test `SF_TRIGGER_HURT_START_OFF`: that flag
only spawns the trigger `SOLID_NOT`, and a trigger with a targetname can be toggled either way
mid-map, so whether the pool is *running* is a live question — `BotItems::IsVolumeInactive` asks it
at scan time — and not a fixed property of the entity.

### The respawn constants
20 s weapons and ammo, 30 s items, 30 s HEV charger, 60 s health charger. These are HLDM's own
`WEAPON_RESPAWN_TIME`, `AMMO_RESPAWN_TIME`, `ITEM_RESPAWN_TIME` and the deathmatch branches of
`FlHEVChargerRechargeTime` / `FlHealthChargerRechargeTime`. They are a *prediction*, not a
measurement — nothing tells the plugin when an item actually rematerialised, so a bot that guesses
short simply walks up to an empty spot and learns from it (see `doc/bot.md`, case B).

### ChargerValue / ChargerFullValue
The charger runner needs one number to watch and one ceiling to stop at, and which `pev` field that
is depends on the charger. HEV chargers read `v.armorvalue`, health chargers and healing pools read
`v.health`; all cap at 100. Keeping this behind the profile is what stops `bot_move_nav.cpp` from
having to know which is which. 100 happens to be the right ceiling for a pool for a different
reason: `CBaseEntity::TakeHealth` refuses to heal past `pev->max_health`, which HLDM leaves at 100.

These two are also now asked *before* travelling and not only on arrival — see `BotItems::Sated` in
`doc/bot.md`.

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

The rule is that the requested set stops at `CBaseEntity` / `CBasePlayerItem` / `CBasePlayerWeapon` /
`CBasePlayer` / `CBaseMonster`. Genuinely per-weapon-class state — `CShotgun`'s own `m_fInReload`
shadowing the base one, for instance — is absent, because reading it means reading a field whose
existence depends on which weapon the pointer happens to be.

**Two entries look like they break it. One does not, and one does.**

`ent_fInAttack` and `ent_flStartThrow` do not. The gauss charge state reads as `CGauss::m_fInAttack`
everywhere in the SDK — `DEFINE_FIELD( CGauss, m_fInAttack, ... )` says so — and the handgrenade's pin
timer reads as `CHandGrenade::m_flStartThrow`, but both declarations are on **`CBaseEntity`**
(`dlls/cbase.h`), where the SDK parks the grenade, satchel and egon prediction state so the client can
share the layout. Every weapon object has both slots; only a couple of weapons ever write them. So the
offsets are valid on any item pointer the walk produces, and reading them needs no knowledge of which
weapon it is.

Both exist for the same reason: the thing they describe is invisible to every other counter.
`CGauss::StartFire` moves no attack timer at all. `CHandGrenade::PrimaryAttack` moves no attack timer
either — it only pulls the pin, and the grenade leaves half a second later from `WeaponIdle`, so
`m_flStartThrow` falling back to zero is the only observation of the throw there is (see
`doc/bot.md`, `bot_fire.cpp`).

`rpg_cActiveRockets` genuinely breaks the rule. `CRpg::m_cActiveRockets` is at 196 and
`CBasePlayerWeapon` ends at 188, so reading it off a crowbar is a heap over-read past the end of a
smaller allocation. It is here because `WPN_Q_KEEP_AIMING` needs to know when the rocket is gone, and
nothing on a base class knows.

What makes it safe is that **the table gates the read**: `bot_weapons.cpp` consults
`IGameProfile::GetWeaponSpec` and only peeks at the offset when the spec for that weapon id carries
`WPN_Q_KEEP_AIMING`. The quirk is the assertion that this pointer is that weapon, which is the same
discipline every other weapon decision uses. The standing cost is that two differently-keyed files
have to agree about which weapon that is — the gamedata JSON is keyed by gamedir, the quirk by
`IGameProfile::Detect` — so a profile that moves the quirk to another weapon has to move the offset
with it.

The rule is otherwise unchanged, and this is the only exception. `CRpg::m_fSpotActive` sits four bytes
below the counter and is *not* requested: the same gate would make it safe, but see "The deployables"
below for why nothing wants it.

### The fun-goal offsets, and why entvars_t is not enough

The fun goal tier (`doc/bot.md`, `bot_fun.cpp`) added a block of offsets that have nothing to do with
weapons. They are all on brush entities, not the player, and every one of them exists because the
question it answers is *not* visible in `entvars_t`:

| offset | class | what it answers |
| --- | --- | --- |
| `plr_pTank` | `CBasePlayer::m_pTank` | is this bot on a tank right now |
| `tog_toggleState` | `CBaseToggle::m_toggle_state` | is this button pressed, waiting, or mid-move |
| `tog_flWait` | `CBaseToggle::m_flWait` | how long before it can be pressed again |
| `btn_fStayPushed` | `CBaseButton::m_fStayPushed` | is it a one-shot latch |
| `tank_pController` | `CFuncTank::m_pController` | is the gun free |
| `tank_vecControllerUsePos` | `CFuncTank` | the 30-unit leash origin |
| `tank_yaw/pitch Center,Range,Rate` | `CFuncTank` | the cone the barrel can cover, and how fast |
| `tank_fireRate` | `CFuncTank` | rounds per second |
| `tank_iszMaster` | `CFuncTank` | which `multisource` unlocks it |
| `tankctl_pTank` | `CFuncTankControls::m_pTank` | which gun this control brush drives |
| `mm_cTargets`, `mm_iTargetName` | `CMultiManager` | what this manager actually fires |
| `ms_rgTriggered`, `ms_iTotal` | `CMultiSource` | is the master satisfied |

Two of these are load-bearing in a way that is easy to miss.

**`tog_toggleState`.** Undertow's flood button is `wait 142` *and* `spawnflags 1`
(`SF_BUTTON_DONTMOVE`) — it does not move when pressed, so there is no geometric or `pev` observation
that distinguishes "ready" from "waiting out 142 seconds". Without this offset the bot walks back to
a dead button every time it repaths.

**`mm_cTargets` / `mm_iTargetName`.** Worldcraft writes duplicate `multi_manager` keys with `#N`
suffixes, so parsing the map entity lump gives `strike_pain` and `strike_pain#1` as two different
targets when they are the on and off edges of one 0.1 s kill window. The engine's array is already
stripped and delay-sorted; the map text is not.

**`CMultiSource::m_iState` does not exist** and cannot be requested. `CMultiSource::IsTriggered`
(buttons.cpp:214) reads `m_rgTriggered` against `m_iTotal`, so those are the two offsets, and
`MasterSatisfied` mirrors that function rather than inventing a state test.

These all obey the base-class rule above without an exception: every one is read off a pointer whose
classname the caller has already matched, and none of them is a derived field read through a base
pointer.

### The tram needs no offsets at all

`FUN_TRAIN` (`doc/bot.md`, bot_fun.cpp) drives subtransit's `func_tracktrain` and adds **nothing** to
`GameOffsets` or `kRequests`. Worth stating explicitly, because the natural assumption from
`FUN_TANK` — which needed nine — is that a second vehicle needs its own set, and adding fatal request
entries for fields that turn out to be reachable through `entvars_t` is a self-inflicted crash on any
deployment whose `gamedata/valve-win.json` predates the change.

| what the tram needs | where it already is |
| --- | --- |
| does the bot hold the controls | `pev->flags & FL_ONTRAIN`, set by `CBasePlayer::PreThink` |
| is the bot aboard | `pev->groundentity` |
| current speed and direction | `pev->speed`, signed |
| top speed | `pev->impulse`, cached by `CFuncTrackTrain::Spawn` as `(int)m_speed` |
| controllable at all | `pev->spawnflags & SF_TRACKTRAIN_NOCONTROL` |
| the route | the `path_track` chain, walked through `target` / `targetname` / `origin` |

`CFuncTrackTrain::m_ppath` and `m_controlMins`/`m_controlMaxs` are the fields one would reach for and
are deliberately **not** requested. The path is rebuilt from the entity lump instead, and the control
box is reconstructed from `pev->mins`/`pev->maxs` with `maxs.z += 72`, exactly as `Spawn` computes it.

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

## fake_cmd.cpp

Making a fake client issue a console command. `FakeCmd::Run(player, "weapon_9mmAR")` is the whole
interface, and today it has exactly one caller: `BotFire::Think` changing weapons.

### Why a client command is the only correct route

`client.cpp:553` — any command whose name begins with `weapon_` goes to
`CBasePlayer::SelectItem(pcmd)`. That is the only path that runs `Holster()` on the old weapon and
`Deploy()` on the new one, sets `m_flNextAttack` to the deploy time, and updates the client's weapon
state. Nothing else in the plugin's reach does.

The two alternatives are both wrong rather than merely inconvenient. `pfnClientCommand` sends a
command *to* a client, and a fake client has no client to send it to. Writing
`CBasePlayer::m_pActiveItem` through `GameOffsets` — cheap, since the offset would be one line —
skips Holster and Deploy entirely: the old weapon is never told it was put away, the new one never
deploys, and `m_flNextAttack` never gets set, so the bot would fire through a switch that a human
cannot.

### The hook is flag-gated and inert everywhere else

The game DLL reads its command through `CMD_ARGV` / `CMD_ARGC` / `CMD_ARGS`, which are engine
functions. So `Run` tokenises the string, raises a module-local flag, calls `MDLL_ClientCommand`, and
lowers the flag; three Metamod **pre**-hooks on `pfnCmd_Argv` / `pfnCmd_Argc` / `pfnCmd_Args` return
`MRES_SUPERCEDE` with the fake tokens while the flag is up and `MRES_IGNORED` otherwise.

These three are on the hot path for every real command from every real client on the server, so the
gating is not an optimisation. They are live only inside the single synchronous `MDLL_ClientCommand`
call and are a plain passthrough at every other instant. `Run` refuses to re-enter for the same
reason.

This is also why `engine_hooks.cpp` now installs anything at all in the **pre** table, which until
this change was memset to zero and returned.

### The classname is case sensitive

`SelectItem` compares with `FClassnameIs`, which is `strcmp`. `weapon_9mmAR` works and `weapon_9mmar`
silently selects nothing — no error, no return value, the bot simply keeps holding what it had. The
classnames in `hldm_weapons.cpp` are the exact SDK spellings and `crossbot_weaponspec <name>` is what
proves it, by joining each spec against `CBotClassOf` on the live weapon entity.

For the same reason `Run` **drops** a command that would not fit its buffer rather than truncating
it: a truncated classname is indistinguishable from a working one at the call site.

`Run` returns whether it dispatched, not whether the game did anything. Nothing downstream should
trust it as confirmation — `BotFire` watches `BotInventory::activeIndex` change instead.

## give_item.cpp

`GiveItem::Give(player, classname)` is the game's own `give`, four calls long: `pfnCreateNamedEntity`,
origin set to the player's, `SF_NORESPAWN` raised, `pfnSpawn`, `pfnTouch`. There is no
`GiveNamedItem` anywhere in the plugin and there should not be — that is a `CBasePlayer` method behind
`pvPrivateData` with no offset for it, whereas spawn-and-touch goes through the DLL API Metamod
already hands us and works for any classname the mod defines, ammo boxes included.

It lives in its own file rather than in `commands.cpp` because it now has two callers with opposite
needs: `crossbot_equip` announces every give on the console, and the pin's auto-equip fires every half
second for six bots and must announce nothing. So `Give` returns a bool and prints nothing, and the
console command keeps its own message at the call site.

`SF_NORESPAWN` is `1 << 30` and is set so the spawned pickup does not become a permanent extra item on
the map. It is defined locally rather than pulled from the SDK header because the plugin does not
include `weapons.h`.

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

### value is the one field that fails the criterion on purpose

`value` says nothing about how to operate the gun. By the rule above it belongs nowhere near this
struct, and `cycleTime` was rejected in as many words for being useful only to "rank guns you are
*not* holding, which is selection".

It is here anyway because it answers a question no runtime observation can. It has two readers, and
they want the same number for different reasons:

- `EvaluateCombat` asks how dangerous the bot is *in total*, to decide whether to go looking for a
  fight — see `doc/game.md` above and `doc/bot.md`, "Readiness is what makes the comparison mean
  anything". That is a property of the whole inventory.
- `BotFire`'s weapon selection asks which of the guns in the bag is worth holding. That is ranking guns
  you are *not* holding, which is precisely what the rejected list said this table would not do.

The second reader is a deliberate reversal, made when selection moved to score-then-rarity (see
`doc/bot.md`, "Selection takes the best gun it has, and the rarest of those"). The earlier position was
that the live inventory could answer selection on its own. It cannot: everything observable at runtime
— clip, reserve, cycle time, range — says whether a weapon *works*, and nothing says whether it is
worth holding. A bot with a full glock and a full gauss sees two working weapons and no reason to
prefer either. Desirability is a judgement about the mod, which is what this table is for, and the only
honest place to record what one weapon contributes is next to the weapon. The alternative is a second
table keyed by `m_iId` sitting beside this one, which is the same data with an extra way to fall out of
sync.

The numbers are a coarse ladder, not damage figures: 0.5 for gauss, egon and crossbow, 0.3 for RPG and
MP5, 0.2 for shotgun, hornetgun and python, 0.1 for everything else. They sum to 3.3 across the table,
so `arms` saturates at two good weapons rather than requiring the full arsenal — deliberate, because a
bot holding gauss and egon is already as willing to hunt as it will ever be, and a sum that only
reached 1.0 with every weapon would mean bots essentially never chased.

The coarseness serves the second reader too, and there it is load-bearing rather than merely
sufficient. Selection filters to the single best score and breaks the remainder on rarity, so a ladder
of unique values would turn the choice into a lookup: one weapon would always win and the bot would
never vary. The ties are what leave rarity room to act — three weapons share 0.5 and six share 0.1, so
in most inventories the score filter narrows the field without deciding it. Adding precision to these
numbers would make bots *less* varied, not more accurate.

### Why FIRE_TAP is bot intent, not an SDK requirement

There is exactly one `ItemPostFrame` in the HLSDK (`dlls/weapons.cpp`). No weapon overrides it, no
weapon reads `m_afButtonPressed`, and `ITEM_FLAG_NOAUTOFIRE` does not exist in this SDK at all. The
dispatcher is level-triggered on `pev->button`, so **every Half-Life weapon is mechanically full
auto** — the python only feels semi-auto because its cycle time is 0.75s.

So `FIRE_TAP` never means "the weapon requires a release". It means the bot must not hold this button.
There are **two** reasons a bot must not, and the table uses the same value for both:

- Holding a **toggle** flips it again every cycle. Python and crossbow secondary, RPG secondary.
- Holding a **handgrenade** cooks it. `PrimaryAttack` pulls the pin and returns; the grenade only
  leaves from `WeaponIdle`, which `ItemPostFrame` reaches solely through its "no fire buttons down"
  branch. Hold the button and it never throws at all — and past three seconds `ShootTimed` gets a fuse
  under 0.1s, **zeroes the velocity**, and drops it at your feet.

Anyone comparing this table against the SDK will find no mechanism behind `FIRE_TAP` and be tempted to
delete it. It is not redundant.

`FIRE_CHARGE` is the inverse: releasing is what fires, deliberately, after a hold that is worth
something. `ItemPostFrame` reaches `WeaponIdle()` the same way, but only the gauss has a charge that
grows while held. The handgrenade *looks* like the same shape and is not, because its hold buys
nothing and costs everything; it is `FIRE_TAP`, and the half-second between the press and the throw is
the engine's, not the bot's.

**`IN_ATTACK2` short-circuits `IN_ATTACK`** in that same dispatcher — the secondary test is an `if`
and the primary is its `else if`. A bot that sets both bits fires only secondary, every time.

### AIM_TARGET vs AIM_LEAD is "is the travel time zero"

Not "what comes out of the muzzle". A crowbar, a shotgun and an egon beam all produce one instruction
— point at the target — so melee, hitscan and beam are a single value. The crowbar's reach is `range`;
the egon's continuous damage is `FIRE_HOLD`.

Homing weapons are `AIM_LEAD`, not `AIM_TARGET`, because the flight time is real and a consumer may
want it. What they are *not* is led: `WPN_Q_HOMING` says the projectile steers itself, and
`bot_fire.cpp` spends that by skipping the solve entirely (see below). The aim value records the
physics; the quirk records what a consumer should do about it, and here they disagree on purpose.

The RPG is the exception that stays `AIM_TARGET`. Its rocket chases the nearest visible `laser_spot`,
which sits wherever the bot is pointing right now, so it is tracked rather than led. "Keep pointing
after you fire" is a duration, not a place, so it is `WPN_Q_KEEP_AIMING`.

### gravity is effective, not pev->gravity

`pev->gravity` is a multiplier on `sv_gravity` — 0.5 is 400 u/s squared at 800 — and the table does
not pre-multiply it, because the multiplier is what belongs to the weapon.

The 800 is now `PmSim::kGravity`, a constant. `sv_gravity` used to be read once, in `Bot_Physics`, and
nothing reads it any more; `sv_stepsize` never was, and has always been the hardcoded 18 in
`navgen::kStepHeight` and its siblings. Both are treated as settings nobody moves. Anyone who moves
them gets a bot that jumps and lobs by the wrong numbers, which is a trade taken deliberately.

**Two rows in this table are parabolas, and they are not the same parabola**, which matters because a
consumer reading `gravity != 0` as "ballistic" would still get half of them wrong:

- **MP5 secondary** is the real one. `CGrenade::ShootContact` at `forward * 800`, `pev->gravity = 0.5`,
  `ExplodeTouch` on anything it meets, and explicitly *no* inherited velocity — the SDK comment says
  so. An elevation solve describes it exactly, and `bot_fire.cpp` solves it.
- **Handgrenade** is the other real one, but on a launch that is one parameter rather than two — see
  `WPN_Q_THROW_ARC` below — and it **bounces** rather than detonating, on a fuse that started at the
  pin pull. A solve finds where it lands; where it explodes is one to two seconds of rolling later.
- **Satchel** is not a projectile. `CSatchel::Throw` spawns a `monster_satchel` that lands and waits;
  the damage comes from a *later* radio press.
- **RPG carries no gravity in this table at all any more**, and the row used to say `250 / 0.5`. Both
  numbers were true for 0.4 seconds and misleading for the rest, so the row now reads `speed 2000,
  gravity 0`. What the launch really does is worth writing down, because it is the reason no clearance
  check follows from it:

  `CRpgRocket::Spawn` tilts the launch 30 degrees above the aim and throws at 250 on `MOVETYPE_BOUNCE`
  with `pev->gravity = 0.5`. For a level aim that is `vz = 250 sin 30 = 125` against `g = 400`, so the
  hump is `125^2 / 800 ~= 19.5` units and it is over in 0.31s. The muzzle sits at
  `GetGunPosition() + up * -8`, about `z + 20` on a 72-unit player, so **the whole excursion peaks
  below head height**. A rocket cannot bury itself in a ceiling a player can stand under, which is why
  `bot_fire.cpp` treats the RPG as plain guided direct fire and never simulates the lob.

  Then `IgniteThink` switches to `MOVETYPE_FLY` and `FollowThink` accelerates it toward 2000 while
  steering at the laser spot. **It does not then run out.** The doc used to claim it self-detonates
  below 1500 u/s about 1500 units out; the decay is `velocity * 0.2 + target * speed * 0.798`, and
  `0.2 + 0.798 = 0.998`, so an aligned rocket loses 0.2% per 0.1s tick and needs some fourteen seconds
  — around 28,000 units — to fall under 1500. `SV_Physics_Bounce` skips `SV_AddGravity` for
  `MOVETYPE_FLY` and applies no air friction, so nothing else slows it either. The rocket cruises
  until it hits geometry, and `range 0` in the row is literal.

Damage falls off linearly from the blast to zero at `splash`, and **the thrower is not excluded** —
`CGrenade::Explode` clears `pev->owner` before calling `RadiusDamage`. That is why a bot needs a rule
about where its own shell lands, not merely where the enemy is.

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
- The RPG's 2000 is a cap it reaches about 0.9s after leaving the tube, not a muzzle speed — see the
  launch breakdown above. It is guided, so it is never led, and no consumer of `speed` needs the curve.

**The throw law is five numbers and it is not in this table.** Direction and speed are one parameter:

    angThrow.x = -10 + view.x * ((view.x < 0) ? 80 : 100) / 90
    flVel      = min(500, (90 - angThrow.x) * 4)

Five coefficients, one weapon. They live behind `IGameProfile::ThrowLaunch`, which maps the pitch the
bot is holding to the elevation and speed the grenade leaves at, and `CHLDMProfile` implements it. The
alternative was five floats on all 42 rows, or five magic numbers in `src/bot/`; the interface that
already hides every other game-specific fact was the right shelf. `WPN_Q_THROW_ARC` is what selects
it, so a mode without the quirk gets `false` and no consumer has to know which weapon it is.

**Velocity inheritance is inconsistent** and changes the launch vector: the handgrenade, satchel,
snark and RPG all add the player's own velocity, and the MP5 grenade explicitly does not. Hence
`WPN_Q_INHERIT_VEL` rather than a blanket assumption either way.

### WPN_Q_HOMING means the thing aims itself

Hornetgun primary and the snark carry it. The RED hornet launches at 300, accelerates, and steers
toward its target every think — so the 600 in the `speed` column is an effective figure for a
projectile whose real speed is a curve, and no single float could be more honest than that.

The quirk exists because a consumer cannot recover "this thing corrects itself" from any other field,
and what follows from it is not a wider tolerance but **no lead at all**. `bot_fire.cpp` skips the
solve, aims at the target, and fires when the target is anywhere inside a wide cone. See a target,
click, hope. That is what these weapons are for, and it is why `speed` and `gravity` stop mattering
for them: no trajectory was computed, so nothing can bend one.

The **snark is not a projectile in the first place**. `CSqueak::PrimaryAttack` traces 20 to 64 units
ahead, spawns a `monster_snark` at the trace endpoint, throws it at `forward * 200 + player velocity`,
and the thing hunts on its own legs. The `200` and the `0.5` describe the toss, not a flight. Worse
for anyone tempted to lead it: 200 u/s is *slower* than a 320 u/s player, so the usual lead iteration
is not a contraction and diverges rather than merely erring. `WPN_Q_HOMING` is the only honest row
for it.

Hornetgun **secondary** does not carry it: the ORANGE dart flies straight at 1200 and never tracks,
so it is led like a crossbow bolt. The two modes of one weapon differing here is the reason this is a
mode-level quirk.

What the quirk deliberately does **not** excuse is `gravity`, which still earns the uphill veto in
`bot_fire.cpp`: a thrown snark aimed above the thrower's eye lands short and walks back, which is a
fact about where you are standing rather than about the solve.

`splash` used to reject a homing mode outright, and no longer rejects anything. Self-damage is now a
gate rather than a category: every mode with `splash > 0` has its predicted impact tested against
"closer to them than to me", so a weapon that steers itself is judged on where it lands like any
other. The snark's splash is 0 regardless.

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

### The deployables, and the one toggle that needs no code

`WPN_Q_HIDDEN_STATE` marks a mode whose behaviour depends on private state, and after the handgrenade
and RPG work it is the last door in `BotFire::Supported` other than a `WeaponToggle` and an aim of
`AIM_NONE` or `AIM_SURFACE`. Three modes carry it, for two different reasons.

**The satchel, both modes.** Primary is three different actions on `m_chargeReady` — throw at 0,
radio-detonate at 1, inert at 2 — and *secondary branches on the same field*, which is why it carries
the quirk too rather than being held out by luck. Every branch of primary ends in `CSatchel::Throw()`,
and what that throws is a `monster_satchel`: a **mine** that lands, waits, and only hurts anyone when
the radio is pressed later. Throwing without being able to press the radio is worse than not throwing,
so the state and the throw are one feature and neither ships alone.

`m_chargeReady` is **not** a `CSatchel` field. It is on `CBaseEntity` alongside `m_flStartThrow`,
`m_fInAttack` and `m_fireState`, so reading it cost the offset rule nothing — it is requested now,
as `ent_chargeReady`, because `bot_deploy.cpp` needs it. `BotWeapons::Read` fills it only for a mode
carrying `WPN_Q_HIDDEN_STATE`, the same gate `rpg_cActiveRockets` sits behind, so the satchel is the
only weapon it is ever read from.

It is read for two different questions and both need the real field rather than a guess. The
**throw** is confirmed by `m_chargeReady == 1`, because a satchel is 80 units downrange before any
position check could find it. The **detonation** is refused unless `m_chargeReady == 1`, because
primary means three different things depending on it, and a bot that guessed wrong would throw
another satchel when it meant to blow one up. Inferring the state from live `monster_satchel`
entities is close but not equal: `AddToPlayer` resets the field to 0 on pickup while the old charges
are still in the world and still owned by the same edict.

`WPN_Q_HIDDEN_STATE` still rejects both satchel modes in `BotFire::Supported`, and that has not
changed. The quirk means "this mode's behaviour depends on private state", which is exactly why the
firing path must not touch it; `bot_deploy.cpp` reads the state instead and drives the weapon itself.

**RPG secondary** toggles `CRpg::m_fSpotActive`, and here the quirk earns its keep by keeping the bot
away from a button it must never press. `CRpg::Spawn` sets `m_fSpotActive = 1`, so **the laser is on
for the whole life of every RPG a bot holds**, and secondary fire would only turn it off. `TOGGLE_LASER`
is therefore handled entirely by never pressing it: `Supported` rejects `toggle != TOGGLE_NONE`, and
`WPN_Q_HIDDEN_STATE` says the same thing a second way.

That is why `m_fSpotActive` is not in `GameOffsets` even though `rpg_cActiveRockets` four bytes above
it now is. The gate that makes the counter safe would make the flag safe. Knowing the flag would only
let the bot press a button it is forbidden to press, to restore a state that was never lost. The one
case it would help — a human toggles the spot off and a bot picks the weapon up — leaves the rocket
flying along its launch aim, which mostly still arrives.

### Rejected fields

Each of these is a true fact about the weapon that fails the criterion above. They are listed so they
do not get re-proposed:

| Rejected | Why |
|---|---|
| `MELEE` / `HITSCAN` / `BEAM` as separate aim values | One instruction between them. Reach is `range`, sustained fire is `FIRE_HOLD`. |
| `HOMING` as its own aim value | The flight time is still real and still worth recording, so the aim value stays `AIM_LEAD`. What a homing weapon needs is a *handling* difference — do not solve, point and click — and that is `WPN_Q_HOMING`. |
| `cycleTime` | The live `nextPrimaryAttack` on the weapon in hand is authoritative. A table value only helps rank guns you are *not* holding, and `value` already carries that ranking at the coarseness selection needs — a second, finer axis would only give the two something to disagree about. |
| `zoomFov` | `pev->fov != 0` answers "am I zoomed". 20 vs 40 changes no handling decision. |
| a `needsSurface` quirk | Redundant with `AIM_SURFACE`. |
| an `exhaustible` quirk | The inventory walk runs every frame; the bot sees the weapon disappear. |
| a `regenAmmo` quirk | Same — the bot sees the hornet count climb. Whether to wait or switch is selection. |
| a `selfDamage` quirk | Redundant with `splash > 0`. |
| a `selfPush` quirk | The gauss jump is a movement capability, not a firing instruction. |
| the throw-law coefficients | Five floats on every one of 42 rows to describe one weapon's pitch-to-speed curve. `WPN_Q_THROW_ARC` names the law and `IGameProfile::ThrowLaunch` implements it. |
| a `guided` quirk | Redundant with `WPN_Q_KEEP_AIMING`. The reason you must keep pointing *is* that the projectile follows where you point. |

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

The gauss spin-up has the same shape and is worth the same care. `CGauss::SecondaryAttack` burns one
uranium every **0.1s in multiplayer** and 0.3s in single player, so a 1.5s full charge costs 16 — one
to start the spin plus fifteen — and not the 6 the single-player figure implies. `ammoPerShot` is 1
because that is the per-tick cost; a consumer that wants a full charge multiplies. Burning stops dead
at full charge (`m_flNextAmmoBurn = 1000`), so **holding a charged gauss is free**, which is the whole
reason a bot can carry one between fights.

`splash` radii come from `RadiusDamage`, which uses `flDamage * 2.5`. With retail `skill.cfg` that is
250 for the RPG, the M203 grenade and the handgrenade (100 damage each), and 375 for the satchel
(150).

MP5 **secondary** carries `WPN_Q_NO_UNDERWATER` even though primary does not. `CMP5::SecondaryAttack`
tests `waterlevel == 3` before anything else and plays the empty sound; primary fires happily. The
quirk was missing from the row until the mode became reachable, which is the usual way a table entry
gets checked — nothing reads a field until something drives it.

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
