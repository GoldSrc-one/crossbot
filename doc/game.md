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

## ent_handle.h

### CBotEntHandle
`EHANDLE` from `hlsdk/dlls/cbase.cpp`, retargeted at `edict_t` because the plugin has no
`CBaseEntity`. The serial number is the whole point: the respawn memo outlives the entity it names
by up to a minute, and edict slots are recycled, so a raw pointer would eventually resolve to an
unrelated entity that happened to reuse the slot.

**`Get()` checks `ent->free` as well as the serial number**, which the HLSDK original does not. It
does not need to — a `CBaseEntity` never outlives its edict — but a memo entry does, and a freed
edict keeps its old serial number until the slot is reused.
