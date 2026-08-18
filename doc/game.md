# Game System Comments

## game_profile.h

### IGameProfile
The mod-specific half of the bot. Everything a bot needs to know that is not geometry lives behind
this interface, so `src/bot/` never names a Half-Life classname or constant. `CHLDMProfile` is the
only implementation today and `GameProfile_Detect` returns a function-local static.

`g_pGameProfile` is a real global (defined in `bot_manager.cpp`) rather than a `BotManager` accessor
because it is read on nearly every item decision; an accessor bought nothing and cost a call.

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
