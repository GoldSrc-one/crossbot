# Games

CrossPlay loads several real game DLLs into one server, and everything per-game (profiles, offsets,
teams, models, weapons) hangs off the game a bot belongs to. This file owns:

- the CrossPlay seam: game contexts and the rules for touching another game's data;
- `IGameProfile` and each game's profile (`src/game/<game>/`);
- teams, gamedata offsets, and the small helpers in `src/game/`;
- the weapon table format (`weapon_spec.h`) and the Half-Life table.

How the bot fires a weapon is `doc/combat.md`; this file says what the table means.

## CrossPlay hosting

`src/game/crossplay.h`, `src/game/crossplay.cpp`

CrossPlay is a hyper-game: it loads several real game DLLs into one HLDS process and multiplexes them
onto a single entity pool.

```
hlds -console -game crossplay -xgame1 valve -sgame1 valve +map crossfire
```

- `-game crossplay` is the hyper-game directory, `D:\hlds\crossplay\`.
- `-xgameN <dir>` is a game DLL for CrossPlay to load. They become `gGameDirs[N-1]`, counted by
  `gNumGameDlls`, up to 8.
- `-sgameN <dir>` adds the same mod to the engine's content search path, so its maps, models and
  sounds resolve.

The load chain is engine → `crossplay/addons/metamod/metamod.dll` → `plugins.ini` →
`crossplay/dlls/crossplay.dll` → each xgame's `liblist.gam` → `hl.dll` and friends. **CrossBot is a
plugin of the top-level Metamod**, so it sits above CrossPlay: its hooks run with CrossPlay's
`gCaller == -1`, and its `g_engfuncs` is the raw engine table, not CrossPlay's hooked one.

Registering the plugin (a `win32 addons/crossbot/crossbot_mm.dll` line in
`crossplay/addons/metamod/plugins.ini`) is an operator step. Nothing in this repo writes that file;
`test_server_start.ps1` warns when the line is missing.

### The data model

CrossPlay keeps one `edict_t` pool but **one private-data object per game DLL per edict**:

| Symbol | Meaning |
| --- | --- |
| `gNumGameDlls`, `gGameDirs[game]` | which games are loaded, and their directory names |
| `gEntityOwners[index]` | which game created edict `index`; `-1` for none |
| `gEntityObjects[game][index]` | that game's `CBaseEntity *` for edict `index` |
| `GameContext(game, cb, ctx)` | run `cb` with `game` as the caller |

- `gEntityOwners[0]` is the world, so it names the game the **map** belongs to.
- `gEntityObjects` is a full matrix: when a game allocates an entity, CrossPlay builds same-classname
  proxies in every other loaded game, so a `valve` bot can look at a `cstrike` entity through
  `valve`'s eyes. A game with no such class gets null, the designed soft failure; every read is
  null-checked.
- Entering a context rewrites `pvPrivateData` on every live edict and swaps `gpGlobals->teamplay`, so
  it is expensive. Entering the context that is already current is a free early return, which is what
  makes `MDLL_*` calls safe from inside a bot's own context.

### Two halves of "which game"

- `XPlay::EnterGame(i)` installs **CrossBot's** view: `g_offsets = g_gameOffsets[i]` and
  `g_pGameProfile = g_profiles[i]`. Cheap, no CrossPlay involvement. `XPlay::Current()` reports it,
  and `CBotPrivateData(ent)` reads `gEntityObjects[Current()][ent]`.
- `XPlay::InGame(i, lambda)` installs **the engine's** view for the call, through `GameContext`. Only
  needed when calling into a game DLL, or an engine function that dispatches into one. It is a
  template over a stateless thunk, so it takes a lambda:

```cpp
edict_t *pEdict = nullptr;
XPlay::InGame(iGame, [&] { pEdict = CREATE_FAKE_CLIENT(botName); });
```

Only `crossplay.cpp` includes `crossplay_api.h`: that header declares `maxGameDlls` inside
`extern "C"`, which multiply-defines it across translation units. `crossplay.h` exposes
`XPlay::kMaxGames` and static-asserts the two agree.

### Where CrossBot enters a context

- **`BotManager::StartFrame`**: one `EnterGame` + `InGame` per bot around `ThinkOneBot`, covering the
  whole AI (respawn, `BotWeapons::Refresh`, `Bot_MoveNav`, `FakeCmd::Run`, `pfnRunPlayerMove`). Monster
  refresh runs once per frame in the map owner's context. `IGameProfile::RefreshWorld` runs once a
  second per loaded game in that game's context.
- **`BotManager::AddBot`**, around `CREATE_FAKE_CLIENT`. Mandatory: CrossPlay's `CreateFakeClient`
  ReHLDS hook `DebugError`s on `gCaller == -1`, and it stamps the new client's owner.
  `MDLL_ClientConnect` / `MDLL_ClientPutInServer` stay outside, since CrossPlay enters its own. So does
  `ApplyAddArg` (a plain engine call), which runs before `ClientConnect` so the game reads a finished
  userinfo buffer, as for a real client.
- **`BotManager::Touch`, `OnSound`, `OnEvent`, `KillAllBots`**: per resolved bot.
- **`GiveItem::Give` / `Spawn`**, around `pfnCreateNamedEntity`, so CrossPlay stamps the owner at
  allocation. `Give` takes the game from the receiving player.
- **Console commands that read private data**: `ScopedGame` in `commands.cpp`, keyed by the bot's
  `gameDll` or `XPlay::OwnerOf(player)` (see below).

**Hooks hand back the game they found.** `Touch`, `OnSound` and `OnEvent` fire inside a bot's think
as readily as outside it (`GiveItem` plays a pickup sound, `pfnRunPlayerMove` fires a gun). Leaving the
game on the way out would null `g_pGameProfile` for the rest of the enclosing think (a crash with one
game loaded) or run it against another game's profile (silent with two). Both `ScopedGame` structs
save `XPlay::Current()` and restore it through `EnterGame`; `EnterGame(-1)` falls through to
`LeaveGame`, so a hook entered from no game leaves none. The one bare `LeaveGame` is the end of
`StartFrame`, which never nests. Copying a `ScopedGame` is deleted, since two destructors would
restore twice.

`ServerActivate` and `ServerDeactivate` need no context; they call `XPlay::DetectProfiles()` and
`XPlay::ForgetProfiles()`.

### Console dumps need a game context

A console command runs at `gCaller == -1` with no game entered: `g_pGameProfile` is null and
`CBotPrivateData` returns null for every entity. Bot code never sees that state, so anything a dump
reuses from the bot side meets it for the first time there. Without a context there are two failure
shapes:

- **Crash.** `crossbot_list_items` reaches `BotItems::PrintCandidates`, which would call
  `g_pGameProfile->ClassifyItem` through a null vtable. HLDS prints nothing and reloads the level, which
  looks like a bad console line, not a missing context. `PrintCandidates` therefore also refuses to run
  without a profile.
- **Silent and cached.** `ChainTerminal` null-checks the profile and returns false for every switch, so
  a `crossbot_list_fun` before the first bot connects would leave `BuildTable`'s per-map table empty of
  switch and trigger goals for the rest of the map (on crossfire, no nuke). `crossbot_list_gates` would
  never show `mined`, because `GateHasMine` returns false.

So `ListMesh` in `commands.cpp` resolves both halves for every mesh-backed dump (the nav mesh, then a
playable game: the map owner if playable, otherwise the first that is) and each command enters it with
`ScopedGame` before calling bot code, as `PrintMonsters` does. `crossbot_list_spawns` and
`crossbot_list_targets` read only `pev` fields and need neither. Console line length limits are in
`doc/console.md`.

### Rules

1. **Never write `gEntityOwners`.** CrossPlay stamps it from `gCaller` in `PvAllocEntPrivateData` and
   the `CreateFakeClient` hook. Reading is fine.
2. **Never read `edict->pvPrivateData` directly.** Outside a context it is garbage (`0xFFFFFFFF` in
   CrossPlay debug builds). Use `CBotPrivateData(ent)`, or `XPlay::PrivateData(ent, game)` when the
   entity's own game is the right lens (as `CBotIsLivePlayer` does).
3. **Never edit anything under `D:\hlds\crossplay\` except `addons\crossbot\`.**
4. **Never test or set `FL_FAKECLIENT`.** ReGameDLL's `CBasePlayer::Spawn` (`player.cpp:5725`) does
   `pev->flags &= FL_PROXY; pev->flags |= FL_CLIENT;`, wiping `FL_FAKECLIENT` on every spawn and round
   restart. HL keeps it, which is why it looks dependable until a cstrike bot dies once. Use
   `BotManager::FindBot(ent)` to ask whether an edict is our bot; `FL_CLIENT` is safe and is what
   separates players from monsters in `bot_sense.cpp` and `bot_combat.cpp`.

### Reading CrossPlay's complaints

`crossplay.lib` links a **Debug** `crossplay.dll`, so these fire loudly on stderr (`D:\hlds\hlds.err`)
and each names a broken rule:

| Message | Meaning |
| --- | --- |
| `Bad caller!` | an engine call that needs a context was made at `gCaller == -1` |
| `Entity without owner!` | an entity reached a dispatch with `gEntityOwners == -1` |
| `Owner mismatch!` | two games claim the same edict |
| `Wrong private data pointer on enter/exit!` | someone wrote `pvPrivateData` behind CrossPlay's back |

### Per-game state

- `gamedata.cpp` holds `g_gameOffsets[kMaxGames]` plus a loaded flag per game, filled by
  `GameData::LoadAll()` at `GameDLLInit`. `GameData::Activate(i)` copies one into the `g_offsets`
  every peek site reads.
- `crossplay.cpp` holds `g_profiles[kMaxGames]`, filled by `XPlay::DetectProfiles()` at
  `ServerActivate` through `GameProfile_Detect(gamedir)`. `g_pGameProfile` is the active one.
- A game is **playable** (`XPlay::Supported`) only with both. `crossbot_add` refuses anything else and
  `XPlay::PickRandomSupported()` draws only from that set: `valve`, `ricochet`, `gearbox`, `dmc`,
  `tfc`, `cstrike`/`czero` (one profile), `dod`, `rewolf` and `ts`.
- Each `CBot` carries `gameDll`, set once in `AddBot`; it is the key to all of the above.

Profiles live one directory per game: `src/game/hldm/`, `ricochet/`, `gearbox/`, `dmc/`, `tfc/`,
`cstrike/`, `dod/`, `rewolf/`, `ts/`. `src/game/game_profile.cpp` holds `GameProfile_Detect`, the only file that names
them all, returning function-local statics. Anything keyed off the **map's** game rather than the bot's
(teams, objectives, gamedata) stays in `src/game/` itself.

## The profile interface (game_profile.h)

`IGameProfile` is the mod-specific half of the bot. Everything that is not geometry lives behind it,
so `src/bot/` never names a Half-Life classname or constant. `CHLDMProfile` is the base
implementation and every other profile derives from it, overriding what its game changes.
`g_pGameProfile` is a plain global (defined in `bot_manager.cpp`) because it is read on nearly every
item decision; an accessor would cost a call for nothing.

### Weapons

- **`GetWeaponSpec(id)`** returns the `WeaponSpec` for the raw `m_iId` the inventory walk produces, or
  null. It answers with a struct because it describes a table, not a policy (see "The weapon table").
- **`WeaponSlotCount()` / `WeaponSlot(i)`** walk the raw table, empty and broken rows included, for the
  places that need every weapon: `ItemPreferred` and `crossbot_weaponspec`. Every caller goes through
  the virtuals so a derived profile never silently sees HL's table.
- **`ItemSlots()`** is the number of `m_rgpPlayerItems` buckets: HL 6 (`MAX_ITEM_TYPES`), OP4 7, Gunman 11. It is a
  profile value because the gamedata JSON carries no array sizes.
- **`ReadInventory(player, inv)`** fills `BotInventory`. HLDM runs the shared item walk
  (`BotWeapons::ReadItems`); DMC, TFC and DoD override it.
- **`SelectWeapon(player, spec)`** switches weapons. HLDM sends `weapon_<name>` through `FakeCmd`; DMC
  drives `CmdStart`.
- **`ModeAvailable(player, spec, mode)`** is a per-player veto on a firing mode that depends on state the
  table cannot describe (Ricochet's hard disc, DoD's undeployed MGs). `BotFire::ModeViable` asks it
  right before `AmmoReady`, so a refused mode is never selected or pressed.
- **`ThrowLaunch(mode, viewPitch, elevation, speed)`** holds what a table of floats cannot: a weapon
  whose launch direction and speed are one parameter. It maps the held pitch to what leaves the hand
  and returns false for any mode without `WPN_Q_THROW_ARC`. It maps that way round on purpose: the
  pitch is what the bot commands and what the fire gate re-measures, so nothing inverts the game's
  remap. See "The throw law".
- **`AmmoEntity(weaponId, pool)`** maps a weapon and ammo pool to the classname that refills it, so
  `src/bot/` never names `ammo_ARgrenades`. The pin's auto-equip needs it (`doc/combat.md`), and
  `WeaponSpec` records which pool a mode spends, not which pickup fills it. Null where nothing fills the
  pool (the crowbar, the hornetgun's regenerating hive). HLDM's table is why it is a table: the MP5
  clip takes `ammo_9mmAR` and its grenade `ammo_ARgrenades`; gauss and egon share uranium under two
  classnames; the hand grenade, snark, tripmine and satchel are their own ammo.

### Weapon roles

`src/bot/` must not name a weapon, so each role it needs is a profile answer:

- **`IsMeleeWeapon(id)`** is the set; **`MeleeWeapon()`** the one to draw (the breaker uses it). HLDM's
  set is the crowbar; OP4 and TFC have several (see their sections).
- **`AmbushWeapon(kind)`** answers what to open with against someone unaware. `AmbushKind` is
  `AMBUSH_GRENADE`, `AMBUSH_SNARK`, `AMBUSH_PENGUIN`, `AMBUSH_FLASH`, `AMBUSH_SMOKE` (the thrown group)
  and `AMBUSH_GRAPPLE` (last resort); a mod returns 0 for kinds it lacks. It must be a named role:
  `WPN_Q_THROW_ARC` picks out the hand grenade, but nothing in `WeaponSpec` separates a snark from a
  hornet gun (both `WPN_Q_HOMING`, and the snark is deliberately not an arc). A throwable-creature
  quirk would be a type tag, which the table rejects.
- **`DeployWeapon(kind)` / `DeployEntity(kind)`** (`DEPLOY_TRIPMINE`, `DEPLOY_SATCHEL`) answer "which
  weapon places one" and "what a placed one looks like". `bot_deploy.cpp` needs both (to fire, to check
  a gate is mined, to confirm a placement, to find its own charges) and `bot_avoid.cpp` needs the
  second to recognise a beam's owner. A mod with no deployables returns 0 and null and the feature
  idles.

### Policy

- **`EvaluateCombat(pEdict, inv)`** scores combat readiness in [0,1], the input that decides whether an
  enemy or an item wins goal selection (`doc/goals.md`, "Carefulness"). It is the one method taking a
  `BotInventory`; `game_profile.h` forward-declares it so the header names nothing from `src/bot/`, and
  passing it keeps `BotWeapons::Refresh` the single reader of weapon private data.
  `CHLDMProfile::EvaluateCombat` is a product, so a dying bot with a full arsenal and a healthy bot with
  nothing loaded both score near zero:

      survive = (health + armorvalue) / (maxHealth + maxArmor)
      arms    = sum of WeaponSpec::value over every armed weapon, clamped to 1
      result  = survive * arms

  `maxHealth` is `pev->max_health` or 100; there is no pev field for maximum armour, so it is a
  constant. "Armed" is `BotFire::WeaponArmed`: a mode that deals damage and has the ammo, reusing
  `AmmoReady`. `AIM_NONE` modes are skipped, so a dry python does not count on its zoom toggle.
- **`MonsterEnemy(ent)`** answers "who is this monster hunting", as an edict or null. It is how
  `src/bot/` gates monster engagement without knowing what a `CBaseMonster` is; see "mon_hEnemy".
- **`FallDamage()`** is a `PmSim::FallRule`: the safe speed and damage per u/s above it. `kHlFall` is
  HL's 580 and 100/444; `kNoFall` never hurts. `Bot_FallDamage` hands it to the planners, which price
  each fall from the `.nav` height (`doc/path_planning.md`, "Falls are priced per game"), and to the
  runtime `FallDamageFor` vetoes in `bot_opportunist.cpp`, `bot_gauss.cpp` and the ladder bail. From
  each game's `FlPlayerFallDamage`:

  | game | safe u/s | damage per u/s | 100 hp lethal at |
  |---|---|---|---|
  | HL, OP4, Gunman (`mp_falldamage 1`, set by CrossPlay's `server.cfg`) | 580 | 0.225 | 1024 |
  | CS (ReGameDLL; `mp_falldamage 0` is none) | 500 | 0.208 | 980 |
  | DoD (dod.c 35803) | 500 | 0.25 | 900 |
  | TFC (tfc.c 38681) | 650 | 0.267 | 1024 |
  | Ricochet; DMC under `mp_falldamage 0` | none | | |

  HL's own `mp_falldamage 0` is a flat 10 per landing and is not modelled.

### Joining and spawning

- **`PreThink(edict)`** runs first in every think and may own the bot (team and class joining, TFC's
  feign). Returning true skips the rest.
- **`Respawn(edict)`** presses whatever revives a dead player; round- and wave-based games send an
  empty move.
- **`IsDead(edict)`** is the asking bot's view of anyone. HLDM's treats an observer (`iuser1 > 0`) as
  dead too (see DoD).
- **`ApplyAddArg(iGame, pEdict, arg, out, size)`** owns the optional third `crossbot_add` argument. The
  command maps `*` and empty to null and passes the rest untouched, because its meaning is per mod:
  HLDM reads a player model (`player_models.cpp`), the team mods a team or class. It runs once on the
  fresh fake client after `CREATE_FAKE_CLIENT` and before `MDLL_ClientConnect`, and writes back what it
  settled on for the `added ... as '...'` line. Null means pick a default, at random where sensible.

### Client-side behaviour

DoD moves into the client what other mods do on the server. Six hooks default to the old behaviour:

- `Zoomed(edict)`: default `0 < pev->fov < 90`; DoD reads `m_iFOV`.
- `AdjustView(bot, dt, pitch, yaw)` edits the angles `Bot_UpdateFacing` is about to write (recoil,
  scope sway, the deploy clamp).
- `MoveAdjust(bot, fmove, smove, buttons)` runs inside `Bot_RunPlayerMove` after the brace (sprint, the
  bipod).
- `MaxSpeed(edict, engineMax)` is applied at the end of `Bot_MaxSpeed`.
- `CanDeploy(edict)` decides who may take a sandbag or prone camp.
- `BlindSeconds(fade, hold, scale)` turns a blinding ScreenFade into blind time.

### Other game hooks

All default to doing nothing; the game sections say who implements them.

- Items: `ClassifyItem`, `ItemUsable`, `ItemWorth`, `ItemPreferred`, `ItemRespawnDelay`,
  `ItemRespawnIn`, `ChargerValue`, `ChargerFullValue` (`doc/goals.md` owns how gathering uses them).
- `AbilityRun` (TFC class abilities, CS planting, DoD ammo drops and bayonets).
- TFC: `RefreshWorld`, `Disguised`, `HoldFire`, `NoteSound`, `Patients`, `Fortifies`, `CanDetpack`,
  `OffhandRead` / `OffhandCommand` / `OffhandPrimed`.
- DMC: `Cloaked`.
- CS and DoD firing: `FireToggle`, `SpreadWide`, `BraceSpeed`, `Penetrates` (also HLDM's gauss),
  `CreepSpeed`, `CreepButtons` (`doc/combat.md`).

### ItemKind

The classification the gather goals key off:

- `ITEM_WEAPON`, `ITEM_AMMO`, `ITEM_PICKUP` are touched to collect.
- `ITEM_HEALTHCHARGER`, `ITEM_RECHARGE` are used with `IN_USE` and never touched.
- `ITEM_HEALVOLUME` is a healing `trigger_hurt` the bot only has to stand in.
- `ITEM_STATION` is a TFC dispenser: touched, then stood at while it refills.

The families group differently at different sites, hence several predicates instead of comparisons:

- `ItemKindIsCharger`: aim at it and hold `IN_USE`.
- `ItemKindIsBrush`: chargers and volumes; read the absbox midpoint, because `pev->origin` is 0 0 0.
- `ItemKindUsesValue`: chargers, volumes and stations; there is a `ChargerValue` to watch instead of a
  touch to wait for. `OnTouch`, `OnArrived` and `UpdateGoalItem` branch on it. A trigger is
  `SOLID_TRIGGER` and really fires `pfnTouch` on the bot, so a charger-only test would end the goal the
  instant the bot waded in.

## Teams (teams.cpp)

The friend/foe half of teamplay and nothing else: no roles, objectives, coordination or team joining.
It exists so a bot does not aim at, chase or blow a satchel under someone it should not fight.
Friendly fire is off on the mods covered, so this is about behaviour that would look broken, not
damage.

**Teams are a property of the map.** `DetectMap()` reads `XPlay::GameDir(XPlay::MapOwner())` (the game
owning worldspawn) and matches `cstrike`, `czero`, `tfc` and `dod`. It cannot key off the bot's game (a
`valve` bot on a cs map has no team mod of its own) nor `gpGlobals->teamplay` (CrossPlay swaps it to
the entered game's value). It runs at `ServerActivate`, right after `XPlay::DetectProfiles()`, with
`ForgetMap()` at `ServerDeactivate`. Its one printed line names the map's game and whether teams came
on; look for it first when bots misbehave on a team map.

**The team is read live, never cached.** `TeamOf(ent)` is `ent->v.team`, falling back to
`ent->v.owner->v.team`. The owner hop makes a rocket, grenade or thrown snark inherit its thrower's
team, and is what makes the satchel gate work. A game may reassign a bot at any time (balance, forced
join, respawn on the other side), so nothing is stored on `CBot`. `edict_t::v` is shared by every
CrossPlay game, so teams across games are one number space and no read needs a context.

**Zero is unknown, and unknown is never attacked.** `Of(self, ent)` is `REL_ENEMY` only when both teams
are non-zero and differ, `REL_FRIEND` when they match, `REL_UNKNOWN` otherwise. `Attackable`, the
predicate every gate calls, is `REL_ENEMY` or teams not active for this bot.

**`Active(self)` is false when the bot's own team is zero, and that fallback is load-bearing.** A mod
that never fills `pev->team` would otherwise make every bot match every other and the server would go
pacifist. With it, such a mod means "teams do nothing", not "bots do nothing", and
`crossbot_teams_status` says which. It also makes every gate a no-op on deathmatch maps.

**What the mods do, measured.** `cstrike` fills `pev->team` on a `valve` fake client without being
asked: bots on `cs_assault` came up `team=1`, and within a minute the mod moved three of six to
`team=2` (hence the live read). `tfc` and `dod` report teams on for their own maps. The first minute of
a round can be all-friends, correctly: bots join one side before the mod balances them. Nothing here
joins a team; the TFC, CS and DoD profiles join themselves. `crossbot_teams 0` restores free-for-all on
a team map.

**Where the gates are**, all in `src/bot/` (`doc/perception.md`, `doc/combat.md`):
`Candidate` and `MonsterCandidate` in `bot_sense.cpp`, `DamageSource` and the team exit in
`bot_combat.cpp`, and `NearestLive`'s `livingOnly` filter (no satchel under a teammate). Everything
else that shoots reads `sense.target` or `sense.enemy` and is gated by the first two.

## Half-Life (hldm_profile.cpp)

### ClassifyItem

Prefix match on `weapon_`, `ammo_`, `item_`, plus exact `weaponbox`, `func_healthcharger`,
`func_recharge` and `trigger_hurt`. Prefixes let the bot pick up mod items it has never heard of; the
cost is that the item scan walks the whole edict array, since `FIND_ENTITY_BY_STRING` takes an exact
name.

- **The recharger's classname is `func_recharge`, not `func_recharger`.** The wrong spelling silently
  makes every HEV charger `ITEM_NONE`.
- `item_` also catches `item_suit`, `item_antidote`, `item_security`. Harmless: the bot touches one,
  nothing happens, and the full list keeps it away for 60 s.
- **Foreign items are classified by their owner's profile** (`doc/goals.md`, "Every item is judged by
  the game that owns it"). The pickups AMXX plugin (`scripting/pickups.sma`, an `FM_Touch` hook ahead of
  CrossPlay's Touch filter) translates foreign weapons, items and ammo into the toucher's game, so the
  bot just tries the touch.
- **`IsArmouryArmor`** makes a CS armoury holding kevlar or an assault suit a preferred `ITEM_PICKUP`
  (Ricochet shares it). With owner-context classification a real CS armoury reaches `CCSProfile`, so
  this only matters when cstrike has no profile. Armour is told apart by model (`models/w_kevlar.mdl`,
  `models/w_assault.mdl`, set by `CArmoury::Spawn` from `m_iItem` 16/17), so no CS offset is read. An
  emptied armoury is `EF_NODRAW` and drops out of the scan.
- **It takes an edict, not a classname**, because a `trigger_hurt` is a healing pool or a death trap
  depending on the sign of `pev->dmg`; `pool_party` ships one of each (`dmg -40`, `dmg 200`).
  `IsHealingHurt` wants `dmg < 0` and no `SF_TRIGGER_HURT_NO_CLIENTS`. It does not test
  `SF_TRIGGER_HURT_START_OFF`: that only spawns the trigger `SOLID_NOT`, and a named trigger can be
  toggled mid-map, so whether the pool runs is a live question (`BotItems::IsVolumeInactive`).

### Respawn times and chargers

20 s weapons and ammo, 30 s items, 30 s HEV charger, 60 s health charger: HLDM's `WEAPON_RESPAWN_TIME`,
`AMMO_RESPAWN_TIME`, `ITEM_RESPAWN_TIME` and the deathmatch branches of `FlHEVChargerRechargeTime` /
`FlHealthChargerRechargeTime`. They are a prediction: nothing tells the plugin when an item
rematerialised, so a bot that guesses short walks up to an empty spot and learns (`doc/goals.md`).

`ChargerValue` / `ChargerFullValue` give the charger loop one number to watch and a ceiling, so
`bot_move_nav.cpp` never knows which field: HEV chargers read `v.armorvalue`, health chargers and pools
read `v.health`, all capped at 100. For a pool 100 is right because `CBaseEntity::TakeHealth` refuses
to heal past `pev->max_health`, which HLDM leaves at 100. They are also asked before travelling
(`BotItems::Sated`, `doc/goals.md`).

## Ricochet (ricochet_profile.cpp)

`CRicochetProfile` overrides only what Discwar changes. Under CrossPlay a Ricochet player runs
Half-Life's pmove on whatever map is up, so jumping, ducking and the long jump all work even though
Ricochet's own `PM_Jump` and `PM_Duck` return early; everything HLDM-shaped is inherited.

**One weapon, one row.** `weapon_disc` is id 1 and always held. Ammo is `m_rgAmmo[disc]`, three at most,
and a thrown disc comes back after up to seven bounces, so nothing predicts when; `AmmoReady` just
waits.

- **Secondary** is the decapitator, the only throw that kills. `SecondaryAttack` needs all three discs
  and spends them all: `AMMO_PRIMARY`, `ammoPerShot 3`, `FIRE_TAP`, `AIM_LEAD` at 1000 u/s, no gravity.
- **Primary** throws one disc and **does no damage**, only pushes. It is in the row for the hard-disc
  powerup, and `ModeAvailable` refuses it unless `CBasePlayer::m_iPowerups & POW_HARD` (4), read through
  `plr_iPowerups`. A hard disc always decapitates; `GivePowerup` refills `m_iPowerupDiscs` to three and
  the flag clears when they run out, so the flag alone answers "hard discs left".
- `POW_FAST` lets secondary fire with one disc; the row ignores it.
- **Melee-only throws the disc.** `MeleeOnlyAllows` passes it, so `crossbot_melee_only` still lets
  Ricochet bots fight. `IsMeleeWeapon` stays false: a melee disc would start duels (`DuelDrive` reads
  range 0 as "swinging" and stands still) and send other games' bots charging a Ricochet player with a
  crowbar.

**Never combat-ready.** `EvaluateCombat` is always 0, so carefulness is 1: a Ricochet bot always routes
for cover and the range bias is off.

**Gathers `item_*` and CS armour only.** All goal tiers are on: gather alone would leave bots standing on
`fy_snow`, where `roam` is the only goal. `ClassifyItem` is the `item_` prefix or a CS kevlar/assault
armoury; on a Ricochet map that is `item_powerup*`, respawning after `DISC_POWERUP_RESPAWN_TIME` (10 s).
A powerup waiting to respawn is `EF_NODRAW`.

**No fall damage.** `CBasePlayer::PostThink` has the fall-damage call commented out: `kNoFall`.

## Opposing Force (gearbox_profile.cpp)

`CGearboxProfile` is OP4 **deathmatch**: HLDM with ten more weapons, the same crowbar-and-glock spawn,
and the HL weapons (ids 1-15) with SDK timings. CTF (`op4ctf_*`: `m_iTeamNum` teams, flags, a
pipewrench/eagle/grapple loadout) is not covered.

**The ids come from retail, not the reconstructed source.** `halflife-op4-updated` numbers the new
weapons 16-25; retail `opfor.dll` leaves **21 unused** and runs 22-26 (a live `crossbot_weaponspec`
printed `MISMATCH` past the displacer; jk_botti and Parabot agree). `gearbox_weapons.cpp` has 27 slots:
0-15 fall through to `HLDM_WeaponSpec`, 21 is empty.

| id | weapon | value | range | handling |
|---|---|---|---|---|
| 16 | grapple | 0.1 | close | hold, lead 2000, no ammo, sticky; melee and last-resort ambush |
| 17 | eagle | 0.3 | long | hold, target, clip 7; secondary is the laser, `WPN_Q_LASER_ON` |
| 18 | pipewrench | 0.1 | close | the crowbar row |
| 19 | m249 | 0.3 | close | hold, target, clip 50, sticky, `WPN_Q_BRACE` |
| 20 | displacer | 0.4 | close | tap, lead 500, 20 uranium, splash 300, `WPN_Q_SPINUP` + `WPN_Q_AIM_FEET` |
| 22 | shockrifle | 0.2 | close | hold, lead 2000, self-charging pool, sticky, no underwater |
| 23 | sporelauncher | 0.3 | long | primary lead 2400 splash 200; secondary a lob at 800, gravity 1, inherits velocity |
| 24 | sniperrifle | 0.5 | long | hold, target, clip 5, zoom 18 (zoomed row is the same shot) |
| 25 | knife | 0.1 | close | the crowbar row |
| 26 | penguin | 0.2 | close | the snark row plus splash 250 (it explodes with hand-grenade damage) |

Left out on purpose: the pipewrench's charged swing and the knife's backstab (both crowbar skins); the
displacer's secondary (teleports the user, not an attack); the grapple's movement use (only sticks to
`xeno_grapple` textures, which would be navgen work). The shock rifle underwater is a radius blast that
kills its holder, hence `WPN_Q_NO_UNDERWATER`.

**Melee is a set.** `IsMeleeWeapon` is crowbar, knife, pipewrench and grapple, so an enemy holding a
knife starts a duel. The grapple is full melee with range 0 (unlimited), so `DuelDrive` never charges
with it and the ambush's `PASS_REACH` skips it. `MeleeWeapon()` stays the crowbar.

**Ambush order.** `AMBUSH_PENGUIN` joins the thrown group; `AMBUSH_GRAPPLE` is `PASS_LAST`, taken with no
distance or splash gate when nothing else fits (`doc/combat.md`, "The ambush").

**Expert pickups.** `ItemPreferred` adds `weapon_displacer` and `weapon_sniperrifle`. A displacer
pickup carries 40 uranium, the gauss and egon pool, so it counts even for a bot that has one.

The OP4 weapon quirks are under "The OP4 quirks"; `eagle_iLaserActive` under "Gamedata".

## Deathmatch Classic (dmc_profile.cpp)

`CDMCProfile` inherits respawn, `IsDead`, player models and item classification from HLDM and replaces
everything about weapons, because DMC keeps no weapon entities.

### The inventory is four ints and a bitmask

Every DMC player carries one dummy `weapon_quakegun` (`m_iId` 2, the glock's id) that fires whatever the
player's state says. The real state is on `CBasePlayer`: `m_iQuakeItems` holds `IT_*` bits (weapons in
bits 0-7: axe, shotgun, super shotgun, nailgun, super nailgun, grenade launcher, rocket launcher,
lightning gun), `m_iQuakeWeapon` is the bit in hand, and ammo is `m_iAmmoShells` / `Nails` / `Rockets` /
`Cells`. `pev->weapons` is meaningless, which is why `crossbot_weapons` prints `MISMATCH` on every DMC
bot.

The item walk would report a glock, so `CDMCProfile::ReadInventory` emits one `BotWeapon` per set bit:
no edict, no clip, `primaryAmmoType` 0-3 indexing a synthetic shells/nails/rockets/cells `ammo[]`. The
player's `m_flNextAttack` is copied onto the held weapon's `nextPrimaryAttack` so the shot counter sees
a timer rise. `BotWeapons::ActiveWeaponId` takes the same path. The six fields are optional gamedata
requests; a DMC profile without them is `GameData::Fatal`.

**The weapon id is the bit plus one**: DMC's own `usercmd.weaponselect` number (1 axe .. 8 lightning),
keeping 0 as "none".

Checked live one give at a time: each pickup sets the expected bit and adds the SDK's ammo (super
shotgun +5 shells, nailgun +30 nails, launchers +5 rockets, lightning +15 cells). **A death wipes
`m_iQuakeItems`**, and a backpack carries the killer's current weapon, so an arsenal after a fight need
not resemble what was given.

### Switching is weaponselect

DMC comments out impulses 1-8 and has no `weapon_` command; the only selector is `CmdStart` reading
`usercmd.weaponselect` into `W_ChangeWeapon`, which refuses a weapon the player lacks or has no ammo
for. `pfnRunPlayerMove` has no field for it, so `CDMCProfile::SelectWeapon` calls `MDLL_CmdStart` /
`MDLL_CmdEnd` with an otherwise empty usercmd (Parabot's route). The switch is instant.

### The table

No clips, no secondary modes. Shotguns and the lightning gun (range 600, 0.1 s per cell) are
`AIM_TARGET`; nails and rockets fly at 1000 and are led; the rocket launcher has `WPN_Q_AIM_FEET`.
Splash is `Q_RadiusDamage`'s reach, damage + 40 = 160. Both shotguns spend one shell per press, since
the super shotgun fires a single shot on one shell.

**The grenade launcher is `WPN_Q_THROW_ARC`**: its launch is `forward * 600 + up * 200`, leaving 18.4°
above the aim at 632.5 u/s, which only `ThrowLaunch` (`DMC_ThrowLaunch`) can express. It is `FIRE_HOLD`
because the tap path's throw commit waits for a hand grenade's `m_flStartThrow`, which DMC never sets.
It does not inherit velocity (only modes with `WPN_Q_INHERIT_VEL` do). The grenade spawns at
`pev->origin`, not the eye; the solve ignores those 18 units.

### The rest

- **Fall damage follows `mp_falldamage`.** DMC's default 0 is a flat 5 per landing, so `kNoFall`; a
  non-zero cvar uses the HL formula, `kHlFall`.
- **Load `valve` first: `-xgame1 valve -xgame2 dmc`.** CrossPlay's `PM_Move` routes a ducking DMC player
  (`noDuck`) to a move donor, and its donor search never increments. With `valve` as game 0 the first
  check succeeds; with `dmc` alone or first the server spins forever the first time a bot crouch-jumps.
  That loop is CrossPlay's to fix.
- **The ring makes a player 80% invisible.** `CDMCProfile::Cloaked` reads `renderfx ==
  kRenderFxGlowShell` with `renderamt == 5`, or `rendermode == kRenderTransColor`. It is not
  `Disguised`: the player is still a target and only sight is rolled (`doc/perception.md`, cloaked
  enemies). Damage and sound find him as usual.
- **Known gaps.** `mp_weaponstay` defaults to 1, so touching an owned weapon does nothing and only the
  item timeouts teach that; quad and pentagram are not read; DMC's jump is 295 u/s against HL's ~268,
  so plans are conservative; `WPN_Q_NO_UNDERWATER` tests `waterlevel == 3` but the lightning gun
  discharges at `> 1`.

## Gunman Chronicles (rewolf_profile.cpp)

`CRewolfProfile` is Gunman deathmatch (`rewolf`, `gunman.dll`, November 2000): an HL SDK 2.x game with
its own weapons, a weapon-customization system, and absolute weapon timers. Everything below was read out
of `gunman.dll` and checked live on `takeoff`; sandbot's Gunman bot was the starting point, not the source.

**The gamedata is the matcher's.** No PDB or symbol file exists, so `rewolf-win.json` comes from
`GamedataMatcher` against the HL oracle (Valve class names only). Every field the bot reads is backed by
gunman's own save tables or was checked in code (`DefaultReload` for `m_fInReload`, `m_iClip`,
`m_flNextAttack`; `AddPlayerItem` / `RemovePlayerItem` / `GiveAmmo` for `m_rgpPlayerItems` 1152,
`m_pActiveItem` 1196, `m_rgAmmo` 1212). `CBaseEntity` is 32 bytes, so `m_fInAttack`, `m_flStartThrow` and
`m_chargeReady` do not exist. There are 11 item buckets (`MAX_ITEM_TYPES` loops stop at 11).

### Timers are absolute

Every weapon's `UseDecrement()` returns 0, and `m_flNextAttack` is compared against `gpGlobals->time`, so
gunman stores `m_flNextPrimaryAttack`, `m_flNextSecondaryAttack` and `m_flNextAttack` as absolute times.
`ReadInventory` runs the shared item walk and subtracts `gpGlobals->time`, which gives the bot the same
relative "ready at <= 0" timers as HL. The gauss pistol's pulse sets only the player's `m_flNextAttack`, so
the held weapon's primary timer is the larger of the two, as in DMC, and the shot counter sees it rise.

### Customization is a console command

`cust_<slider><value>` calls a virtual on the **active** weapon with the slider digit and the value;
`custupdate` calls another. The client's menu only sends these commands, so a bot can too. The table
describes one setup per weapon, and `PreThink` sends that setup (`Rewolf_WeaponSetup`) whenever the
active weapon object changes, forgetting it on death:

| weapon | command | effect |
|---|---|---|
| gausspistol | `cust_11` | fire type 1, pulse: hitscan to 8192, 1 uranium, 0.1 s (0.25 s after every third shot) |
| shotgun | `cust_22` | 2 shells a shot, 8 pellets, traced to 2048 |
| dml | `cust_23` | flight path 3, a straight rocket at 900 u/s |

The other gauss fire types: 2 is a `gauss_bolt` projectile at 1450 u/s for 10 uranium; 3 is a 5° spray
at 2000 u/s; 4 is the sniper, which needs 20 uranium and a `cust_2GaussPistolSniper` item. The DML's
default flight path 1 follows the shooter's laser spot and 2 homes; detonation 1 is on impact, 3 a 2 s
bounce, 4 a wall-mounted trip charge.

**Upgrades are inventory items.** `cust_*` entities (`cust_1GaussPistolFireType` id 31,
`cust_2MinigunCooled` 23, `cust_2ShotgunShells` 32 ...) sit in item bucket 0. Spawn hands out fists, the
gauss pistol and the minigun coolers. The table has no rows for them, so the bot ignores them, and ids
above 31 overflow `pev->weapons`: `crossbot_weapons` prints `MISMATCH` on every Gunman bot.

### The table

Aliases rename themselves in `Spawn`, so the table uses the real classnames: `weapon_mechagun` is
`weapon_minigun`, `weapon_pistol` is `weapon_gausspistol`, `weapon_mule` is `weapon_dml`,
`weapon_polarisblade` is `weapon_beamgun`, and `weapon_chemgun` is `weapon_SPchemicalgun` (the maps place
the latter).

| id | weapon | value | range | handling |
|---|---|---|---|---|
| 1 | fists | 0.1 | close | the crowbar row (32 u trace); melee |
| 14 | beamgun | 0.3 | close | hold, 1 battery, range 512, sticky, no underwater |
| 16 | dml | 0.4 | long | hold, lead 900, clip 2, splash 250, `WPN_Q_AIM_FEET` |
| 17 | shotgun | 0.2 | close | hold, target, 2 buckshot, range 2048 |
| 19 | SPchemicalgun | 0.2 | close | hold, lead 850, gravity 0.5, 1 acid, splash 150 |
| 28 | minigun | 0.3 | close | hold, target, 1 9mm, sticky |
| 30 | gausspistol | 0.2 | long | hold, target, 1 uranium, sticky |

- **The beamgun aims itself.** It takes the nearest living client or monster in a ±512 box
  (`UTIL_EntitiesInBox`), hence range 512.
- **The chem gun's speed is its pressure slider**, 400 + 150 × pressure; the default 3 gives 850. The
  shot also shoves the shooter.
- **Splash radii are estimates**: 250 for the DML (`dmg` 100 at HL's 2.5×) and 150 for the chem gun
  were not measured.
- **Left out:** `weapon_dmlGrenade` (id 13, comes with the DML and spends its missiles) and
  `weapon_aicore` (7). Neither is on the multiplayer maps.

`AmmoEntity` maps each weapon to its pickup, checked against each `ammo_*` entity's `AddAmmo`.
`ItemPreferred` adds `item_armor`, `weapon_dml` and `weapon_minigun`. There are no ambush or deployable
weapons, and fists are melee.

## The Specialists (ts_profile.cpp)

`CTSProfile` is The Specialists 3.0 deathmatch (`ts`, `mp.dll`): an HL SDK 2.3 MP game that, like DMC,
keeps no weapon entities. Everything below was read out of `ts_i686.so` (gcc 2.96 symbols, no field
names) and checked live on `ts_bikini`. TS's own bot (`addbot`) is in `mp.dll` only, and the Linux
build says "Bots only in listen servers".

### The inventory is one gun with 38 slots

Every TS player carries one `weapon_tsgun` (`CTSGun`, `m_iId` 16) that fires whichever TS weapon is
selected. The gamedata has no names for its members, so `ts-manual.json` names the three the bot reads
(Windows offsets; Linux is +16):

| field | offset | what |
|---|---|---|
| `CTSGun::m_iWeapon` | 184 | the selected TS weapon id, 0 = kung fu |
| `CTSGun::m_rgAmmo` | 196 | `int[16]` reserve per ammo type |
| `CTSGun::m_rgStatus` | 256 | 38 weapon states of 56 bytes: byte +4 owned, int +8 rounds in the magazine |

`ReadInventory` emits one `BotWeapon` per owned id, plus kung fu, which is always there. Its id is 38,
because 0 means "none" to the bot; every other id is TS's own (the AMXX `TSW_*` numbering). The gun's
`m_flNextPrimaryAttack` and `m_fInReload` go to the selected weapon only. The timers are relative
(`UseDecrement` returns 1). The magazine count was seen above the table's clip after a few kills (an
AUG at 56, a PDW at 76); the cause was not traced. `pev->weapons` is meaningless, so
`crossbot_weapons` prints `MISMATCH`.

### Joining is the buy commands

A joining player is an observer until the VGUI spawn menu sends its loadout and `respawn`. The menu
only sends console commands (`ClientCommand`), so the bot does too. `Respawn` sends them once a second
while dead, and presses attack as HLDM does:

- `rst_cfg` clears the loadout.
- `tkw <id>_<attachments> ...` adds weapons (`AddItemToArmament`). For grenades and knives the second
  number is a count.
- `tka <ammotype>_<rounds> ...` sets the reserve (`AddAmmoToArmament`, a set, not an add).
- `respawn` spawns, once the respawn timer has run out.

`TS_Loadout` picks one of eight primary + pistol pairs by player index, each with its reserve and no
attachments, weighing 35-55. Three of them (AK-47 + MK23, AUG + Glock 18, PDW + Glock 22) were seen
bought live on the default cvars; the others were not.

### Switching is weapon_<id>

`weapon_<n>` calls `CTSGun::SelectWeapon(n)`, which refuses a weapon the player does not own;
`weapon_0` is kung fu. Other commands the bot does not use: `firemode`, `drop`, `usepowerup`,
`lastinv`.

### The table

Every gun is hitscan, `AMMO_CLIP`, one round a shot, and `FIRE_TAP`. TS has per-weapon fire modes
(the status byte at +6). Semi-auto modes fire only on a fresh press (`m_afButtonPressed`), and a tap
also drives full auto at its rate. Clip and ammo type come from `s_weapon_info`, which the
`Write_<weapon>` functions fill at startup (clip at +4, ammo type +8, fire interval +0x18, damage
+0x1c, kind +0x20, weight +0xd0, price +0xd8).

- **Values** scale by class: pistols 0.1-0.2, SMGs 0.25-0.3, shotguns 0.25-0.35, rifles 0.4, the
  M60 0.45, the Barrett 0.5. Rifles, the Desert Eagle, the Raging Bull and the Contender are long range.
- **Shotgun ranges** follow the pellet spread: 1024 for the M3 and Mossberg, 768 for the SPAS and
  USAS, 384 for the sawed-off.
- **Melee** is kung fu (48 u) and the katana (64 u); `MeleeWeapon` is kung fu.
- **Left out:** the M61 grenade (24), the throwing knives (25, 35) and the C4 (29). The bot never buys
  them.

### The rest

- **Ground weapons and ammo are used, not touched** (`CTSGroundWeapon::Use`), so `ClassifyItem`
  ignores `ts_groundweapon`, `ts_groundammo` and every `weapon_*` / `ammo_*`. `ts_powerup` is touched
  and is `ITEM_PICKUP`, preferred along with health kits.
- **No zoom, attachments, stunts or slow motion.** Secondary attack is a melee bash on most guns; it is
  not in the table.
- **Fall damage** is assumed to be HL's (`kHlFall`); TS's `FlPlayerFallDamage` was not read.

## Team Fortress Classic (tfc_profile.cpp, tfc_ability.cpp)

TFC is the HL SDK with QuakeC Team Fortress 2.x ported in. The inventory is a normal HL item chain
(`m_rgpPlayerItems`, `m_pActiveItem`), while everything TF (ammo, grenades, `tfstate`, disguise,
buildings, goal keys) is a QuakeC field **on `CBaseEntity`** (`tfstate`, `ammo_shells`/`nails`/
`cells`/`rockets`, `no_grenades_1/2`, `tp_grenades_1/2`, `is_building`, `is_feigning`, `team_no`...).
Class and team are mirrored into `pev->playerclass` and `pev->team`. `CTFCProfile::Detect` does not
check `deathmatch`, and neither does `CCSProfile::Detect`.

References are into the decompiled `tfc.c` (`D:\projects\libs\tfc.c`). **Its offsets are not the live
DLL's**: every field sits exactly 16 bytes past `tfc-win.json` (`ammo_shells` 228 vs 212, `team_no` 312
vs 296, `m_flTouchableAt` 1164 vs 1148). Match fields by name, never by offset. All TF fields are
optional gamedata requests, and so is `CFuncTank::m_iszMaster`, which `tfc-win.json` lacks
(`bot_fun.cpp` skips a negative offset).

### Joining

`ClientPutInServer` leaves the player with no team or class, and CrossPlay's `joinTeam` feature holds
the spawn until `pev->playerclass` is set. `PreThink` owns the bot until both are set: `jointeam 5`
(auto-assign to the smallest team with room) while `pev->team` is 0, then the class command, each 1.2 s
after the last because `TeamSet` and `ChangeClass` refuse inside `m_fNextTeamOrClassChange` (1 s). The
menus only prompt; the commands work without them. The check runs every think, so a bot moved back to
team 0 or stripped of its class just rejoins. While joining the bot sends an empty move and is marked
`wasDead`, so the usual respawn resets run at first spawn.

The third `crossbot_add` argument is a class: `scout sniper soldier demoman medic hwguy pyro spy
engineer civilian randompc`; anything else, `*` or nothing is `randompc`. The choice lives in a
per-edict-index table, since the profile is a shared static.

- **`randompc` re-rolls every spawn** (`ChangeClass` 75766, `Spawn` 74997-75229): it sets
  `tfstate |= 8` and each spawn draws a class 1-9 that is not the previous one, not in the map's
  `illegalclasses`, and not spy under `spy_off`. `pev->playerclass` holds the real class (seen live:
  a `randompc` pyro came back from `crossbot_kill` as a scout).
- A class command while classed only sets `nextpc` for the next spawn unless the userinfo key
  `hud_classautokill` is set. The bot never changes class alive, so it does not set it.

### Inventory and the table

`ReadInventory` runs the shared item walk, then overwrites `ammo[primaryAmmoType]` from the entity
field that weapon draws on (`TFC_AmmoField`). `SendAmmoUpdate` (71073) only copies those fields into
`m_rgAmmo` on a client data update, so for a fake client `m_rgAmmo` is stale. The sniper rifle has no
`m_fInAttack`; its aiming state is `tfstate & 0x800`, reported as the rifle's `chargeState`.

| id | classname | notes |
| --- | --- | --- |
| 3, 4, 5, 23 | medikit, spanner, axe, knife | melee, `IsMeleeWeapon`; `MeleeWeapon` is the axe |
| 6 | sniperrifle | `FIRE_CHARGE`, `WPN_Q_BRACE`, charge 2 s, carry cap 12 s; zoom 20 on secondary (`CTFSniperRifle::SecondaryAttack` sets fov 20), zoomed row is the same shot |
| 7 | autorifle | hitscan, sticky |
| 8, 9 | shotgun, supershotgun | clips 8 and 16; the super shotgun spends 2 |
| 10, 11 | ng, superng | nails at 1000, no gravity (`MOVETYPE_FLYMISSILE`); super spends 2 |
| 12, 22 | gl, pl | clip 6, `WPN_Q_THROW_ARC`, DMC's launch (600 forward + 200 up) |
| 13 | flamethrower | bursts at 600, range 300 |
| 14, 15 | rpg, ic | rockets at 900 and 600, `WPN_Q_AIM_FEET`; the RPG has a clip of 4 |
| 17 | ac | hitscan, sticky |
| 20, 21 | tranq, railgun | nails at 1500; the tranq is `RANGE_NONE` (its dart slows for 15 s and barely hurts, so it prefers no ground and never makes a sniper) |

Speeds come from each projectile's `Spawn` (103660-104076), clip sizes and ammo names from each
`GetItemInfo`. The pipebomb launcher is worth almost nothing (0.02) because its bombs only go off on
`detpipe`. `ThrowLaunch` is `DMC_ThrowLaunch`.

- **The sniper rifle fires on release, and only when aimed.** `ItemPostFrame` (144056) starts aiming only
  at horizontal speed 50 or less, slows the player to 80, grows damage from 50 at 50/s up to 400, and
  fires on release if on the ground (in the air it prints `#Sniper_cantfire`). `WPN_Q_BRACE` stops the
  bot first.
- **The assault cannon does not brace.** Its cone is the constant 0.1 passed to `FireBullets`
  (`CTFAssaultC::Fire` 138102), with no stance or velocity term. Its slowdown is state-based: the
  wind-up (`PrimaryAttack` 138185) sets `tfstate |= 0x800`, `TeamFortress_SetSpeed` (74241) caps
  `maxspeed` at 80, and only `WeaponIdle`, `WindDown` or `Holster` clears it after the spin times out.
  Releasing between shots gives nothing back.

### Class abilities

`IGameProfile::AbilityRun` is called from `Bot_MoveNav` right after `BotCamp::Track`, before break,
deploy, fun, combat and camp; returning true owns the frame. Per-bot state is the generic
`BotAbility::State` (`src/bot/bot_ability.h`), and everything TF lives in `tfc_ability.cpp`. While a
task runs, `BotFire::Think` idles with `why=ability` and `Bot_ManeuverActive` is true. Tasks drive in a
straight line (`Drive`, the deploy approach), so each starts only for a target in sight and near.
`crossbot_ability_status` prints the task, the last `why` and per-life counters.

| class | what | trigger | commands / buttons |
| --- | --- | --- | --- |
| medic | `heal` | a friend under 95% health in the medic's own nav area, no enemy engaged; ends when the patient leaves the area and is 64+ away | medikit, attack within 64 |
| engineer | `build` | holding a camp spot: sentry (130 cells), else dispenser (100), else exit teleporter (entrance exists, 125). Near its own spawn with sentry and dispenser up: entrance (125) | `build 2` / `build 1` / `build 5` / `build 4` |
| engineer | `repair` | a building of its own team (`team_no`; its own only when teams are off) below `max_health`, or a sentry not on `sentry3.mdl` with 130 cells; in sight within 700 | spanner, attack within 64 |
| spy | disguise | teams on, `is_undercover == 0` (2 in progress, 1 done), not engaged, not carrying a goal, every 5 s at most | `disguise_enemy <1-7,9>` |
| all but medic | call | leaving combat below 50% health, teams on, not feigning | `saveme` |
| spy | feign | damage taken, chance `1 - health/max`, on the ground, no grenade cooking | `feign` |
| spy | `stab` | hard/expert only (`backstab`): the sense target within 300 and facing away (dot < -0.3) | knife, attack within 56; ends if the victim turns (dot > 0.3) |
| scout | `touch` | a player that looks friendly, within 800, in the view cone, closer to an enemy suspicion than to any other friendly-looking player | drives into it; an enemy scout's touch strips a disguise |
| scout | `disarm` | an enemy `detpack` in sight within 1000 | drives into it; a scout's touch defuses it and scores a frag |
| demoman | `detpack` | see "Detpacks" | `detstart N`, then a planned retreat |
| demoman | `pipes` | `BotCamp` scared, fewer than 6 own pipes, no friend within 400 of the aim point | pipe launcher, arc-solved toward 70% of the way to the contact (max 500) |
| demoman | detonate | an own `tf_gl_pipebomb` 150+ from the bot has a live suspicion within 120 and no friend | `detpipe` |
| all | `pad` | the route's next teleport waypoint is a runtime teleporter within 64 | see "Teleporters" |

Grenades are in `doc/combat.md`, "Off-hand grenades". **Not done**: dismantling a sentry the objective
moved away from, conc jumping, rocket jumping.

**A medic answers `saveme`.** `saveme` plays `speech/saveme1.wav`/`saveme2.wav` through `EmitSound`, so
`BotManager::OnSound` sees every call and hands it to `IGameProfile::NoteSound`, which stamps the
caller. For 30 s, while the caller is a live friend below max health, `IGameProfile::Patients` lists it
and `BotStrategic::Intents` gives a medic carrying no payload a rank-4 (override) `KIND_HEAL` intent.
The goal follows the patient and preempts a held goal (`RefreshStrategic` checks `Outranked` before its
arrival early-out). In the patient's area, the `heal` task takes over; a healed or dead patient drops
out of `Patients`, invalidating the goal.

**Building.** The yaw is the camp spot's watch yaw (exit behind, dispenser beside), turned to the first
of eight offsets with a clear 72-unit hull trace, since TFC places the building 64 ahead. The bot stands
still, faces it, sends `build N` once settled, and expects `is_building` within 1 s; a refusal (no
metal, no room, already has one) waits 10 s.

**A sentry faces backwards unless turned.** Every building is created at
`AngleMod(owner->pev->angles.y + 180)` (`CreateSentryBase` 83984), so a sentry built facing the camp
yaw sweeps `yaw ± 50` at the engineer (`CTFSentrygunBase::Finished` 83816). The gun's `pev->angles`
never change; the sweep is `m_iRightBound`/`m_iLeftBound`, and `rotatesentry` adds 45° to both,
`rotatesentry180` 180°. Once `is_building` clears, the task sends one `rotatesentry180` plus
`rotatesentry` × k, with `45k = (180 − offset) mod 360` for the room-trace offset (a multiple of 45).
Measured on casbah: sweep centre 136 and 138 for a camp yaw of 137.

**An engineer works on any building of its team.** `CTFSpanner::AxeHit` (139525) calls `EngineerUse`
for anything `IsAlly` accepts, with no owner check, so `StartRepair` matches `team_no`, not
`real_owner`.

**Attacking engineers fortify what their team will have to hold.** `IGameProfile::Fortifies` is true
for an engineer with no sentry; `BotStrategic::Intents` then gives it a rank-2 `KIND_DEFEND` intent on
every site its team attacks that needs a payload
delivered: a CS rescue point, a bomb site, a TFC capture point. `StartBuild` refuses a sentry at a camp
spot when such a site exists and the current strategic goal is not a defend, keeping sentries off the
attack path and out of scared route camps. Maps with touch captures that need nothing keep the old
behaviour.

**Feigning is owned by `PreThink`.** A feigning spy has `deadflag == 5`, which `IsDead` reports as dead,
so other bots ignore it and `ThinkOneBot` would run the respawn path. `CTFCProfile::PreThink` runs
first: while feigning it holds an empty move and sends `feign` again (stand up) after 3 s once no enemy
within 500 is looking at it (dot > 0.8, line of sight), or after 12 s regardless.

### Teleporters

`RefreshWorld` (once a second, TFC context) pairs every `building_teleporter` entrance
(`CTFTeleporter::m_iType` 4) with its owner's exit (5), both `m_iState > 0`, and keeps one runtime
connection per pair through `Nav::AddTeleport` (`doc/path_planning.md`), with `SetTeleportWait` 0
while the entrance is ready (`m_iState == 2`) and 3 s otherwise. Gone pairs are removed; the team mask
is the owner's team.

The game only sends a player standing on the entrance at exactly zero velocity (`TeleporterTouch`
136005). So `pad` drives on at a distance-capped speed, brakes against its own velocity and holds. It
ends when the teleport waypoint leaves the route (a teleport resets the path), when the connection is
unlinked, or after 10 s, which also penalises the connection.

**It gives up the moment the entrance refuses.** A ready entrance moves to `m_iState == 3` on the touch
that accepts the player, so once the bot stands on it (`groundentity`) under speed 1, the state has
0.3 s to reach 3. If not (recharging, exit gone, wrong team) the task ends "not teleported", penalises
the connection and repaths, instead of standing for 10 s. `RouteExemptsHit` in `bot_avoid.cpp` exempts
a solid within 48 of a route teleport waypoint, or the avoid layer would steer around the pad.

### Spies

**A disguised spy is invisible to bots and holds its fire.** Neither TFC nor CrossPlay hides a spy: the
disguise only rewrites the client's `model`, `topcolor`, `bottomcolor` (`TeamFortress_SetSkin` 74390);
`pev->team` and `pev->playerclass` keep the truth and nothing sets `FL_NOTARGET` (TFC's sentries skip a
spy whose `undercover_team` is theirs, 84230). So `IGameProfile::Disguised` is asked inside
`BotSense::NoTarget` in the owner game's context, covering the sight scan, the damage source and the
keep-or-drop test for watchers from any game. TFC answers yes for `is_undercover == 1` with an
`undercover_team` not his own and no goal item. Firing drops the disguise in-game (131453).

`IGameProfile::HoldFire` keeps a disguised spy out of combat (`BotCombat::Update` exits "disguised",
`BotFire::Think` idles), so he walks his route instead of blowing the disguise; being hurt in the last
2 s lifts the hold. Every tier disguises; for easy and normal it is just a way into enemy territory,
since only hard and expert backstab.

Spies need no goal help: `item_tfgoal_touch` (114802) and `tfgoal_touch` ignore `is_undercover`, and
goals with `goal_result & 0x10` strip the disguise themselves (114309). The bot only stops
re-disguising while it carries something.

### Detpacks

`detstart N` clamps the fuse to 5-50 s, freezes the demoman for 3 s while arming
(`TeamFortress_SetDetpack` 23303, `tfstate |= 0x10000`), and refuses within 65 units of a player or
building, in the air, or with a pack already out. The blast (`DetpackExplode` 22955) triggers every
`info_tfgoal` with `goal_activation & 2` within 1500 units it can trace to, and hurts everything within
700 for `1400 − d`. An enemy scout defuses it by touch (22731).

`ReadTfc` turns those goals into sites flagged `detpack` (`doc/goals.md`), and a demoman who can set one
(`IGameProfile::CanDetpack`: class 4, `ammo_detpack >= 1`, not arming) gets them above everything else.
The task waits until the site is within 1400 and either in sight or the bot is on its planned spot,
arms, holds still for 3 s, and walks a planned route to an area at least 800 away for the rest of the
fuse, shooting as it goes. The fuse is the difficulty: easy and normal take 50 s, hard and expert 5 s
when something is closing in and 10 s otherwise. On a map with no detpack goal a scared demoman may
drop one where he stands, timed so the contact arrives as it goes off (`scareDist` over his own
`maxspeed`; the enemy's speed is unknowable); objective maps never spend the pack that way.

### Backpacks and dispensers

A TFC resupply pack is an `info_tfgoal` that a touch hands to the activating player. The chain is
`tfgoal_touch` 114581 → `ActivateDoResults` 114430 → `APMeetsCriteria` 111796 → `DoResults` 113212 →
`IsAffectedBy` 108305 → `Apply_Results` 114047 → `CheckClassStats` 77810. `IsPack` mirrors it:

- `goal_activation & 1`, and not `& 4` (which inverts the criteria);
- `goal_effects & 3`, so the player or the player's team is affected (`& 8` alone means everyone but
  the player);
- no `goal_result` bit `0x4` (ends the round), `0x20` (force respawn) or `0x40` (deletes the engineer's
  buildings);
- `frags <= 0`, no `increase_team`, nothing negative;
- every criteria field zero (`teamcheck`, the six `if_goal/group_is_*`, `if_item_has(nt)_moved`,
  `has(nt)_item_from_group`, `items_allowed`). No shipped pack uses them, and a pack whose criteria fail
  is not consumed, which would make `NoteFull` ignore every `info_tfgoal`;
- and it gives something.

`item_tfgoal` is not an item: flags are strategic payloads. Brush packs are left out (always `EF_NODRAW`
at origin 0; the only one on the 20 shipped maps is koth_one's 3-hp "cap hp"). A dump of the ~300 packs
shows point `info_tfgoal`s with `g_a=1 g_e=1`, typically 25 health and 50 armour, `wait` 1-60 s,
`team_no` 0, 1 or 2.

`PackAdmits` covers the per-player part: rejects `goal_state` 1 (taken), 3 (removed), 4 (delayed) and
`EF_NODRAW`; compares `team_no` with the player's TFC `team_no` (correct for a foreign player's copy
too); compares `playerclass` with 3 for a foreign player, because CrossPlay's Touch fakes soldier.

`PackGain` is the pack's worth in "full refills", turned into seconds by `kNeedWorth` (20) for
`ItemWorth` (`doc/goals.md`, "Worth: need pulls"):

```
1.0  × healthGain/maxHealth × (1 + missing health fraction)
0.5  × armourGain/maxarmor
0.5  × best ammoGain/max over the pools of weapons in pev->weapons (TFC_AmmoField; spanner = cells)
0.25 × best grenadeGain/cap per type held (2 for nail and MIRV, 3 for conc and caltrop, 4 otherwise)
0.5  if a demoman with no detpack and the pack gives one
```

`xGain = min(give, max − have)`. For a foreign player the TFC fields on its copy are not its real ammo,
so ammo and grenades use the scan's `ammoNeed` and armour a max of 100. `ItemUsable` is
`PackAdmits && PackGain > 0`. Per-type grenade caps matter: a flat cap of 4 sends soldiers holding two
nail grenades after grenade-only packs.

A dispenser is a station, `ITEM_STATION`: `DispenserTouch` (24095) starts a thinker that tops the
toucher up 0.2 s later and then once a second while within 24 units, giving ammo and armour but no
health. The charger loop in `bot_move_nav.cpp` drives it with no `IN_USE`: "in range" is boxes
touching, then the bot stands until the value stops climbing. `ChargerValue` is the percentage of ammo
and armour capacity carried, and `ItemUsable` keeps a bot to its own team's dispensers even though TFC
would refill an enemy.

### Discarded ammo

`discard` (`TeamFortress_Discard` 107478) zeroes the ammo pools the class has no weapon for, spawns a
`CWeaponBox` named `tf_ammobox` and throws it along `v_forward * 400 + v_up * 200` from the player's
**current `v_angle`**, with `pev->enemy` = the dropper, `m_flTouchableAt` = now + 2 and a 30 s life.
With nothing owed no box spawns, so the bot does not check first. The engineer's branch also converts
spare cells into shells, nails and rockets.

`TASK_DISCARD` is the class-agnostic fallback after the per-class starters in `TFCAbility::Run`. The
target is the nearest live friend in the bot's nav area with a different `playerclass` and the same
`XPlay::OwnerOf` (a foreign ally has no meaningful `playerclass`). The bot turns until `AimedAt` agrees
with the real `v_angle`, since that angle is the throw, then sends `discard`. The 60 s cooldown starts
with the attempt, success or not, or a bot whose aim never settles would stand 1.5 s of every 2; it
resets on respawn (`BotAbility::Reset`).

`CWeaponBox::Touch` (131692) pays out through `GiveTFAmmo` from the box's `ammo_*` fields, the same
`CBaseEntity` fields a pack uses. Touch refuses the toucher while the box is in the air, before
`m_flTouchableAt`, if not a live client, or while it holds `IN_ATTACK`. A refused touch would make
`NoteFull` ignore every `tf_ammobox`, so `ClassifyItem` reports a box only once it is `FL_ONGROUND` and
past `CWeaponBox::m_flTouchableAt` (CrossPlay's gamedata). Touch has no team check, so `ItemUsable`
has none either; it refuses only the dropper's own box (`pev->enemy`) and asks `NeedsAmmo`. The
`IN_ATTACK` refusal is not guarded: a bot touching a box mid-fight still trips `NoteFull`.

### Team-only doors, lifts and buttons

TFC runs `APMeetsCriteria` (111796) on nearly everything a player activates: `MultiTouch`, buttons,
`CBaseDoor::DoorActivate`, plat triggers, teleports. A `team_no` (or `teamcheck`) on the activator or on
the door it fires refuses every other team. 2fort's spawn and battlement doors and both lifts are
opened by `trigger_multiple` with `team_no`; cz2, dustbowl, flagrun, push, ravelin, well, crossover2 and
badlands have more (badlands puts the team on the door).

navgen knows none of this; the runtime fills it in. `Objectives::ReadTfc` → `StampDetourTeams`
(`objectives.cpp`) runs on every objectives refresh: it walks `trigger_multiple`, `trigger_once`,
`func_button` and `func_rot_button`, takes the activator's team, ANDs in the team of whatever its
`target` names, and calls `Nav::SetDetourTeams(CenterOf(ent), mask)`, stamping every detour whose
`actCenter` is within 2 units. `func_door` and `func_door_rotating` stamp their own team (an unnamed
use-only door is its own activator). Unrestricted entities stamp 0, so a `teamcheck` flipping back
clears the mask. `teamcheck` is read live, not baked into the `.nav`, because avanti, juno and magelli
swap attack and defence at run time. `crossbot_list_objectives` prints every team-gated detour; the
planner side is in `doc/path_planning.md` (team mask).

Known limits, none hit by a shipped map (checked by scanning every `target` in `D:\hlds\tfc\maps`):

- The door's team is read through one direct `target`; a relay or `multi_manager` in between hides it.
- One activator per detour: navgen keeps the cheapest service point, whatever its team.
- Disjoint teams (a team-1 trigger firing a team-2 door) AND to 0, "everyone".
- Unnamed team *touch* doors (rock2 has 8, hunted 5) are `ACT_NONE` in navgen, so there is no detour to
  stamp and every team plans through them.

## Counter-Strike (cs_profile.cpp)

`CCSProfile` serves `cstrike` and `czero`. `D:\hlds\cstrike\dlls\mp.dll` is a symlink to the ReGameDLL
build, so `regamedll/dlls/` is the source to read; `czero` runs the retail DLL. The map side (bomb
sites, hostages, VIP, the planted C4, carriers) is `Objectives::ReadCs` (`doc/goals.md`); the profile
lets a bot be a CS player. Out of scope: buying, money, radio.

### Joining answers whichever menu is open

`PreThink` owns the bot until `m_iJoiningState` is `JOINED` (0), and every 1.2 s answers the menu
`m_iMenu` shows: `jointeam <n>` on `Menu_ChooseTeam`/`Menu_IGChooseTeam` (1/2), `joinclass <1-4>` on
`Menu_ChooseAppearance` (3), nothing otherwise. `READINGLTEXT` (the MOTD) gets an `IN_ATTACK` edge. Both
commands work with VGUI menus on (client.cpp:3353, 3384).

**Never send `jointeam` before the game asks.** Sent during `SHOWLTEXT`, it sets the team and the
appearance menu, then `JoiningThink` moves on to `PICKINGTEAM` and resets the menu to
`Menu_ChooseTeam`; every `joinclass` then returns `#Command_Not_Available` and the bot never spawns. Re-sending `jointeam`
for the current team is allowed ("switch to the same team just to choose a new appearance"), which is
what makes following the menu safe.

- The third `crossbot_add` argument is the team: `t`, `ct`, or `*`/nothing for auto (5). After three
  refused `jointeam`s (team full, `mp_limitteams`) the bot asks for auto. The model is random 1-4,
  valid on 1.6 and CZ.
- ReGameDLL never writes `pev->team`. CrossPlay's `FixProxyTeam` mirrors `m_iTeam` into it, reversed
  (T 1→2, CT 2→1), for native and foreign players alike, so `Teams::TeamOf` and `ReadCs` need nothing
  new. `crossbot_list_objectives` prints both columns.
- `Respawn` is an empty move: CS is round-based, and `IN_ATTACK` only cycles the spectator camera.

### The table

`cs_weapons.cpp`, 31 rows indexed by `m_iId` (`weapontype.h:31-65`), `ItemSlots` 6.

- Semi-automatics (pistols, scout, awp, g3sg1, sg550) are `FIRE_TAP`; rifles, SMGs and the m249
  `FIRE_HOLD` + `WPN_Q_STICKY`. Rifles, snipers and the m249 are `RANGE_LONG`, the rest `RANGE_CLOSE`.
- `hegrenade` is `WPN_Q_THROW_ARC` and the `AMBUSH_GRENADE`; `ThrowLaunch` is the HL law with CS's rate
  6 and cap 750 (`wpn_hegrenade.cpp:184-194`), gravity 0.55 (`ShootTimed2`). `flashbang` and
  `smokegrenade` copy its arc at gravity 0.5 (`doc/combat.md`, "Throwing at a heard enemy"); `c4` has
  value 0 and no modes.
- Spread coefficients (`kSpreads`) and penetration (`CS_Bullet`) are described in `doc/combat.md`.
- Scopes are `TOGGLE_ZOOM` and `Zoomed` is `fov != 0 && fov < 90`, because CS's unzoomed `pev->fov` is
  90. aug and sg552 are one-stage (`zoomFov` 55). awp, scout, g3sg1 and sg550 cycle 90→40→10→90, so
  `zoomFov` is the first stage and unzooming takes two presses (the loop presses again while
  `fov < 90`, one extra `m_flNextSecondaryAttack` at stage two). Not measured live; a real two-stage
  state is the upgrade if bots are seen crawling at 150 u/s.

### One weapon per slot

CS refuses a second weapon for a filled slot, so `ItemUsable` refuses it first, or a bot with a rifle
walks to every rifle on the map. The item's weapon id comes from its classname, from
`CArmoury::m_iItem` (the `ArmouryItemPack` order, `weapons.h:79`), or from the first item in
`CWeaponBox::m_rgpPlayerItems`, and is vetoed when the bot's `m_rgpPlayerItems[slot]` is occupied
(primary 1, pistol 2, knife 3). Grenades are capped instead: HE 1, flash 2, smoke 1 against
`m_rgAmmo`. The VIP may take loose pistols only, never a weaponbox or armoury (`weapons.cpp:620`,
2695); the bomb weaponbox is vetoed for CT, who would chase it forever. Only entities the bot's own game
owns are read. On `fy_pool_day` an armed bot's scan flips the armoury rifles to `no need` after its
first pickup. On objective maps this rarely matters: the strategic tier wins over gathering, so an
unarmed CT on de_dust2 defends rather than fetches.

### The shield

A shield blocks primaries (`m_bOwnsShield`), and dual elites block the shield (`CWShield::Touch`).
Raised is `m_bShieldDrawn`, toggled by `IN_ATTACK2` on any shield-capable weapon. `FireToggle` returns
`IN_ATTACK2` whenever the shield disagrees with "in combat and not firing": up while `BotCombat::Active`
and no mode is chosen, down once one is, and down when combat ends, because a raised shield caps the
walk at 180 instead of 250. It is gated on the player's `m_flNextAttack` too: a freshly deployed pistol
holds it at 0.75 s, and an ungated toggle presses about 70 times into that window. Seen live on fy_iceworld:
no press before `entered combat`, then raise, lower to shoot, raise between shots, lower after `left
combat`.

### Planting

`AbilityRun` starts `BotAbility::TASK_PLANT` when the bot carries C4, is on the ground, has no target,
and `m_signals` (`CUnifiedSignals`, state at +4) has `SIGNAL_BOMB`. It selects `weapon_c4`, holds
`IN_ATTACK|IN_DUCK` with no movement, and stops on success (C4 gone, `plants` counted in
`crossbot_ability_status`), on a target (retry in 3 s), on leaving the zone, or after 5 s. The
`KIND_DELIVER` intent walks the carrier onto the site. Defusing needs no profile code: the planted C4 is
an urgent `IN_USE|IN_DUCK` site. Both seen on de_dust2 (`Planted_The_Bomb` → `Target_Bombed`, and →
`Begin_Bomb_Defuse_With_Kit` → `Bomb_Defused`). Arming, defusing and freeze time pin `maxspeed` to 1
(`Bot_Frozen`, `doc/movement.md`).

### Testing notes

- `equip.amxx` hands every spawn a random rifle; `amxx pause equip.amxx` gives empty-handed bots.
- A hostage killed on cs_assault was crossfire; the shooter only engaged players.
- `czero` is not verified live: with the retail DLL CrossPlay aborts at map load with `NULL CS
  gamerules!` in `CheckWinConditions` in every layout tried (valve+czero, valve+cstrike+czero, cstrike
  on the content path only).

## Day of Defeat (dod_profile.cpp)

`CDODProfile`. The server source is the decompiled Linux `dod.so` (v1.2f) at `D:\projects\libs\dod.c`;
the client, which holds recoil, sway and the deploy clamp, is `client-dod.c`. Line numbers are into
`dod.c` unless marked client. The map side (capture areas, TNT, spawns) is `Objectives::ReadDod`.
CrossPlay runs DoD as `noTank` and never writes the `CFuncTank` controller fields to `dod-win.json`, so
those requests are optional (`bot_fun.cpp` skips a negative offset).

### Joining, and waiting for the wave

The third `crossbot_add` argument is the class: `garand carbine tommy grease spring bar 30cal bazooka
enfield sten enfields bren piat` (Allies), `k98 k43 mp40 mp44 k98s fg42 fg42_s mg34 mg42 pschreck`
(Axis), or `random` (default). The class names the team, so `PreThink` sends `jointeam 1|2` then
`cls_<name>`; `random` sends `jointeam 5` and `cls_random`. Commands repeat every 1.2 s until
`pev->team` is 1 or 2 and `m_iNextClass != -1` (the gate CrossPlay's `joinTeam` holds the spawn on).
After three tries the class becomes `cls_random`, which covers a British class on a US map, a para class
on a non-para map and class limits without parsing `info_doddetect`.

Respawn is in waves (`RespawnAllies`/`RespawnAxis` 39629); between death and the wave the player is an
observer (`iuser1 > 0`) and `Respawn` is an empty move. That observer has `deadflag == DEAD_NO` and
positive health, so the `iuser1` test lives in `CHLDMProfile::IsDead`, not a DoD override:
`g_pGameProfile` is the *asking* bot's profile, and an HL or CS bot must see a DoD teammate in the
chase cam (which sits on its target's origin) as dead (`doc/movement.md`, "Teammate spacing").

`ApplyAddArg` also sets `cl_autoreload 0` in the userinfo (a missing key means on, 93072), so the bot's
own reload is the only one.

### The table

`dod_weapons.cpp` has 34 rows. Ids 1-31 are `m_iId`; 32 and 33 are synthetic: the scoped Enfield and
scoped FG42 are the plain weapon with `m_iWeaponState & 1` (39409), so `ReadInventory` remaps them and
the rows keep the real classname for selection.

- **Semi-automatics and bolt rifles are `FIRE_TAP`**: DoD latches `m_fInAttack` until release.
- **Rifles carry `WPN_Q_BRACE`.** Every `PrimaryAttack` adds `accuracy_penalty` to the cone whenever 3-D
  speed is over 45, with no stance term and nothing growing per shot (so the CS spread gate does not
  apply). Crouch-walking (73) and prone crawling (50) are both over 45, so bracing must slow the bot.
  `BraceSpeed` answers 45, and in combat the bot crouches throughout (`BraceCreep`), so it has 73 to
  shed rather than a run (`doc/combat.md`, "Speed-gated brace").
- **The .30cal, MG34 and MG42 are refused by `ModeAvailable` unless deployed** (or a camp is about to
  deploy them): undeployed they spread 0.2 and cost 15 stamina a shot, so a moving gunner uses its
  pistol.
- **Rockets lead at 1300 u/s with no gravity**; the real 0.1 made the arc solver refuse every shot.
  `FireToggle` shoulders the launcher (`IN_ATTACK2`, `m_iWeaponState & 1`) while firing it and lowers it
  otherwise. Firing unshouldered costs a 3 s penalty; shouldered movement is capped at 50.
- **Grenades are a `WPN_Q_THROW_ARC` tap**, thrown on release with a fixed 5 s fuse; `ThrowLaunch` is the
  HL law with cap 600. The hand grenade is `AMBUSH_GRENADE` and the stick grenade `AMBUSH_SNARK` (the
  kinds are only a list of thrown ambush ids).
- **`chargeState` is forced to 0**, because DoD's `m_fInAttack` is the semi-auto latch. `throwStart`
  reads `CDoDGrenade::m_flStartThrow`, since the `CBaseEntity` field of that name is shadowed.

`MeleeWeapon` is the American knife; Axis carry a spade or German knife, so `bot_break.cpp` falls back
to any carried `IsMeleeWeapon` id.

### Recoil, sway and the deploy clamp are client-side

The server never kicks the view: bullets go along `v_angle + punchangle` and firing leaves punchangle
alone. The client moves the real view angles, so the profile does too, through `AdjustView`, just
before `Bot_UpdateFacing` writes `v_angle`:

- **Kick** (client `CHud::GetWeaponRecoilAmount` 40317, `PopRecoil` 40474). A shot is a drop in the
  active clip seen in `ReadInventory` (zoom and bipod toggles also move the timers, so timers are not
  used). Each shot replaces the pending kick, spread over 0.1 s: pitch up by the table value, yaw a
  quarter of it (1.3 for the BAR and Bren) × U(0.8, 1.1) with random sign. The multiplier is 0 with a
  bipod deployed, ×0.25 prone (except the three MGs) and ×0.5 ducking. Nothing recovers it
  automatically: the aim controller pulls back at hand speed, and the fire ray releases the trigger
  while the muzzle is off target.
- **Scope sway** (client `CL_AdjustAngles` 64017): while `m_iFOV` is under 90, pitch changes by
  2·dt·cos(2t)·A and yaw by dt·cos(t)·A, A = 0.5 standing, 0.2 ducking, 0.1 prone.
- **Deploy clamp** (client `IN_MouseMove` 37776): once deployed, yaw stays within ±45° and pitch ±20° of
  the deploy angles, and deploy pitch is no higher than −10. The server enforces neither.

`Zoomed` reads `m_iFOV`; DoD only mirrors it into `pev->fov` inside `UpdateClientData`.

A non-lethal headshot sends the victim a ScreenFade (alpha 200, 3 s fade after a 2 s hold;
`TraceAttack` hitgroup 1, 86590) with the same engine message as every other game, so `BotDaze` blinds
the bot as for a CS flash. `BlindSeconds` adds the hold to the usual third of the fade, about 3 s with
no target and no firing (`doc/perception.md`). `crossbot_daze_test <bot> flash 200 3 2` checks it
deterministically; a real headshot fade has not been caught live, because bots aim at the body.

### Walking, sprinting and deploying

The server clamps speed only to `maxspeed` (600 × stamina factor, 310 seen live). The **client** picks
forward speed: `min(stamina, 100) + 120` walking, `+ 230` with `IN_RUN` (client `DoDGetSpeed` 63489).
Without `MaxSpeed` a bot would run at 310 always. `MoveAdjust` sprints (`IN_RUN`) when nothing is
engaged, the bot is moving forward on the ground and stamina (`pev->fuser4`) has reached 90, and stops
below 70. Sprinting drains 30/s, walking regains 10/s, and jump height falls under 60.

`MoveAdjust` also owns the bipod, because it runs inside every `Bot_RunPlayerMove` while the camp's
timers keep running. While `BotCamp::HeldBipod` says the held camp spot wants a deploy:

1. Stop and select the MG.
2. On a sandbag spot, press `IN_ATTACK2` once the zone is armed (`vuser1.x == 1`).
3. On a prone spot, send `sprone`, then press `IN_ATTACK2` once `iuser3 == 1` and the view is within
   20° of the spot's yaw (or an enemy is in sight).
4. Give up after 5 s.

When nothing wants it any more, it presses `IN_ATTACK2` until folded and sends `sprone` until standing,
holding the bot still. A deployed MG never auto-reloads (134306), so `wantReload` is its only reload.
marine_bot says undeployed MGs cannot reload, but `CBipodWeapon::Reload` (139260) has no such gate (not
tested live). Spot selection is `doc/goals.md`, "Sandbags and prone camps".

### Gathering and dropping ammo

`CDoDTeamPlay::CanHavePlayerItem` (36940) refuses a second weapon of a group, and every primary is one
group, so the rifle slot holds one gun; the pistol cannot be dropped. `CWeaponBox::Touch` (134576) takes
what fits and all the ammo. So `ClassifyItem` keeps HLDM's `weapon_*`/`weaponbox` and `ammo_*` and drops
the rest; `ItemUsable` accepts a weapon only for a bot with no primary (`DOD_IsPrimary` against
`pev->weapons`), and ammo only of the bot's team (`CBasePlayerAmmo::DefaultTouch` 134668) and not its
own drop. `ItemPreferred` is every weapon, so a bot that lost its primary re-arms first.

Every soldier spawns with one generic ammo box (`m_nGenericAmmoCount = 1`, 87086). `dropammo`
(`DropGenericAmmo` 90619) throws it 200 u/s along `pev->angles` as `ammo_generic_american`, `_british`
or `_german`, with `pev->team` set and the dropper as `owner`. Every half second a calm bot on the
ground looks for the nearest live friendly DoD player in its nav area (`TASK_DISCARD`), turns to them
with a level view and sends `dropammo` once within 4°. That sets `nextDiscard` a million seconds out
(cleared by `BotAbility::Reset` at respawn), so it happens once per life; an attempt that times out
after 1.5 s may retry after 10 s.

### Bayonet and rifle butt

`RifleMeleeAttack` (31963) is `IN_ATTACK2` on the Garand (5, butt), K98 (10, bayonet), K43 (24, butt)
and Enfield (25, butt, unscoped only): 60 damage along a 64 u trace from the eye. It is used like the
TFC spy's knife, behind the `backstab` tier flag (`doc/difficulty.md`): when the target is an enemy
within 300 u facing away (dot ≤ −0.3) and one of those rifles is out, `TASK_STAB` sprints at the
target's centre (`MoveAdjust` keeps sprint on) and presses `IN_ATTACK2` inside 56 u once aimed within
4°. It stops when the victim dies, turns (dot > 0.3), gets 400 u away, the weapon changes, or after 4 s.
SMG and MG classes do not switch to their knife for it.

**Known gaps.** Grenade pickup and throw-back (`+use` on a live `grenade`, 59525), prone for riflemen,
the BAR/Bren/FG42 bipod outside camps, MG42 heat. Grenade throws have not been seen live.

## Gamedata (gamedata.cpp)

The bot's only way into a game DLL's own data. `edict_t::v` (`entvars_t`) is engine-defined and read
directly; everything else (inventory, clips, ammo, attack timers) lives in private data of a class the
plugin has no header for. `crossplay/gamedata/<gamedir>-<win|linux>.json` dumps those class layouts,
and `GameOffsets` holds the byte offsets the bot needs. Under CrossPlay reads go through
`CBotPrivateData(ent)` (see "CrossPlay hosting"); nothing dereferences `pvPrivateData` directly.

### The format

A class is `{ "baseClass": ..., "fields": { "m_iClip": 104, ... }, "virtualFunctions": ... }`, and a
field's value is its offset and nothing else. `LoadOne` reads only that shape; an older dump that
wrapped fields as `{ "offset", "size", "type" }` fails every required field.

**Offsets are absolute.** The JSON flattens inheritance (`CBasePlayerItem::m_pPlayer` 112,
`CBasePlayer::m_pActiveItem` 1228, `CBaseMonster::m_flNextAttack` 592), so a base-class field reads at
the same offset off any derived object. `GameOffsets` is therefore a flat struct of ints.

`CBaseEntity::pev` at 4 is hardcoded, because it is what checks every other offset (the round trip in
`doc/combat.md`, "Validating every item pointer"). The loader still reads it and refuses a file that
disagrees.

### Loading

`GameData::LoadAll` runs from `GameDLLInit` (`dll_hooks.cpp`): offsets cannot change while a DLL stays
mapped, and CrossPlay has loaded its games by then. It reads each loaded game's file relative to the
HLDS working directory (`pfnGetGameDir()` returns the hyper-game outside a context). **That directory is
CrossPlay's**; nothing here writes it. Results land in `g_gameOffsets[game]`, and
`GameData::Activate(game)` copies one into `g_offsets`.

Keying by gamedir, not profile, is required: the offsets describe a binary, which the gamedir selects,
while `CHLDMProfile::Detect` reads `gpGlobals->deathmatch`, which the engine only fills in once a
server spawns, so profile detection stays at `ServerActivate`.

**A bad read is fatal; a missing file is not.** A wrong offset is a wild read that corrupts silently and
crashes elsewhere, so once the bot uses an offset there is no degraded mode: the consistency checks in
`bot_weapons.cpp` go through `GameData::Fatal`, which prints the reason and takes the server down.
Loading is softer, because CrossPlay routinely loads games CrossBot cannot play: `LoadOne` returns a
reason (unreadable file, bad JSON, missing required field, `pev` not 4), `LoadAll` prints it and leaves
that game unloaded, `XPlay::Supported` is false and `crossbot_add` refuses it by name.

### Which fields may be requested

**Base-class fields, plus class-specific fields read only after the caller has proved the class.** The
general set stops at `CBaseEntity` / `CBasePlayerItem` / `CBasePlayerWeapon` / `CBasePlayer` /
`CBaseMonster` / `CBaseToggle`. Per-weapon state (`CShotgun`'s own `m_fInReload` shadowing the base
one) is absent, because reading it means reading a field whose existence depends on which weapon the
pointer is. Class-specific requests (`CWeaponBox`, `CArmoury`, `CTFTeleporter`, `CFuncTank`,
`CMultiManager`, `CMultiSource`, `CDoDGrenade`) are read only off entities whose classname the caller
matched.

Two that look like exceptions:

- **`ent_fInAttack` and `ent_flStartThrow` are not.** The gauss charge state (`CGauss::m_fInAttack`)
  and the hand grenade's pin timer (`CHandGrenade::m_flStartThrow`) are declared on **`CBaseEntity`**
  (`dlls/cbase.h`), where the SDK parks prediction state for the client. Every weapon object has both
  slots, so they are valid on any item pointer. They exist because nothing else observes those actions:
  `CGauss::StartFire` moves no timer, and the grenade leaves from `WeaponIdle` half a second after
  `PrimaryAttack`, so `m_flStartThrow` returning to zero is the only sign of the throw. `ent_chargeReady`
  (`m_chargeReady`, the satchel's state) is the same kind.
- **`rpg_cActiveRockets` really breaks the rule.** `CRpg::m_cActiveRockets` is at 196 and
  `CBasePlayerWeapon` ends at 188, so reading it off a crowbar over-reads a smaller allocation. It is
  needed because `WPN_Q_KEEP_AIMING` has to know when the rocket is gone. **The table gates the read**:
  `bot_weapons.cpp` peeks it only for a weapon whose spec carries `WPN_Q_KEEP_AIMING`, the same
  discipline as every other weapon decision. The cost: the gamedata (keyed by gamedir) and the quirk
  (keyed by profile) must agree which weapon that is. `eagle_iLaserActive` (`CEagle::m_iLaserActive`,
  an `int` at 200 in retail `opfor.dll`; the reconstructed source has a `bool` there) is gated the same
  way, by `WPN_Q_LASER_ON`; `m_chargeReady` is filled only for a `WPN_Q_HIDDEN_STATE` mode.

`CRpg::m_fSpotActive`, four bytes below the rocket counter, is not requested even though the same gate
would make it safe: see "The deployables".

### Optional fields

A `Request` can be optional; a missing optional field stays -1 and every reader tests it first. Fields
that exist only in some games are optional: Half-Life's client-weapon state on `CBaseEntity`
(`m_fInAttack`, `m_flStartThrow`, `m_chargeReady`, absent in Ricochet), `rpg_cActiveRockets`,
`plr_iPowerups` (Ricochet), `eagle_iLaserActive` (OP4), the DMC `m_iQuake*` fields, all TFC fields, the
CS and DoD player fields (`m_iTeam`, `m_iJoiningState`, `m_iMenu`, `m_signals`, `m_iFOV`,
`m_flAccuracy`...), and the `CFuncTank` controller fields (DoD runs `noTank`).

**`ricochet-win.json` was completed by hand.** The dump lacked `CBaseEntity::pev` (4),
`CBasePlayerWeapon::m_fInReload` (116), `CBaseMonster::m_flNextAttack` (232) and `m_hEnemy` (236), and
`CBasePlayer::m_iPowerups` (1196). They were computed from the Ricochet SDK headers and checked against
anchors in the dump: `CBaseToggle` ending at 192, `CEnvExplosion::m_iMagnitude` 284,
`CBasePlayer::m_iTargetVolume` 292, and `m_rgpPlayerItems` 804 + 6 × 4 = `m_pActiveItem` 828.
`m_iPowerups` was walked field by field from `m_rgAmmo` (840). **A regenerated file loses these five**;
the first four are required, so Ricochet would then report no gamedata.

### mon_hEnemy

`CBaseMonster::m_hEnemy` (244 in valve) is who a monster is hunting, and the whole gate on monster
engagement: a bot fights a monster if and only if this names the bot (`doc/perception.md`). It is an
`EHANDLE` (`{ edict_t *m_pent; int m_serialnumber; }`), so the read is two peeks, and the serial must
match `m_pent->serialnumber` or the handle is stale (the `CBotEntHandle::Get` rule). Everything in valve
with `FL_MONSTER` is a `CBaseMonster` (snarks, tripmines and satchels via `CGrenade`), so `FL_MONSTER`
plus non-null private data makes the peek legal, and the serial check catches a mod that breaks that.

- **The read is `IGameProfile::MonsterEnemy`**, because only the mod knows the answer is a field at 244.
  The bot compares the result with its own edict.
- **It reads through the monster's own game.** It takes `XPlay::OwnerOf(ent)` and peeks
  `XPlay::PrivateData(ent, owner)` at `GameData::MonsterEnemyOffset(owner)`, ignoring `CBotPrivateData`
  and `g_offsets`. Through the bot's lens a foreign monster is a proxy, and a game without the class has
  a null one: on `cs_monsters` cstrike bots read `enemy=-1` for every valve and gearbox monster at
  cstrike's offset and walked past monsters shooting at them. The returned edict is engine-global.
  `test_monster.ps1 -Phase teams` is the witness.

### Fun-goal offsets

The fun tier (`doc/goals.md`, `bot_fun.cpp`) reads brush-entity state that `entvars_t` does not show:

| offset | class | answers |
| --- | --- | --- |
| `plr_pTank` | `CBasePlayer::m_pTank` | is this bot on a tank |
| `tog_toggleState` | `CBaseToggle::m_toggle_state` | is this button pressed, waiting, or moving |
| `tog_flWait` | `CBaseToggle::m_flWait` | how long until it can be pressed again |
| `btn_fStayPushed` | `CBaseButton::m_fStayPushed` | is it a one-shot latch |
| `tank_pController` | `CFuncTank::m_pController` | is the gun free |
| `tank_vecControllerUsePos` | `CFuncTank` | the 30-unit leash origin |
| `tank_yaw/pitch Center,Range,Rate` | `CFuncTank` | the cone the barrel covers, and how fast |
| `tank_fireRate` | `CFuncTank` | rounds per second |
| `tank_iszMaster` | `CFuncTank` | which `multisource` unlocks it |
| `tankctl_pTank` | `CFuncTankControls::m_pTank` | which gun this control brush drives |
| `mm_cTargets`, `mm_iTargetName` | `CMultiManager` | what this manager fires |
| `ms_rgTriggered`, `ms_iTotal` | `CMultiSource` | is the master satisfied |

- **`tog_toggleState`**: undertow's flood button is `wait 142` and `SF_BUTTON_DONTMOVE`, so nothing
  geometric or in `pev` distinguishes ready from waiting; without it the bot walks back to a dead
  button on every repath.
- **`mm_cTargets` / `mm_iTargetName`**: Worldcraft writes duplicate `multi_manager` keys with `#N`
  suffixes, so the entity lump shows `strike_pain` and `strike_pain#1` as two targets when they are the
  on and off edges of one 0.1 s kill window. The engine's array is stripped and delay-sorted.
- **`CMultiSource::m_iState` does not exist.** `CMultiSource::IsTriggered` (buttons.cpp:214) compares
  `m_rgTriggered` against `m_iTotal`, and `MasterSatisfied` mirrors it.

**The tram needs no offsets.** `FUN_TRAIN` drives subtransit's `func_tracktrain` with nothing added to
`GameOffsets`, unlike `FUN_TANK`. Adding required requests for fields reachable through `entvars_t`
would crash every deployment whose gamedata predates them.

| the tram needs | where it is |
| --- | --- |
| does the bot hold the controls | `pev->flags & FL_ONTRAIN`, set by `CBasePlayer::PreThink` |
| is the bot aboard | `pev->groundentity` |
| current speed and direction | `pev->speed`, signed |
| top speed | `pev->impulse`, cached by `CFuncTrackTrain::Spawn` as `(int)m_speed` |
| controllable at all | `pev->spawnflags & SF_TRACKTRAIN_NOCONTROL` |
| the route | the `path_track` chain, walked through `target` / `targetname` / `origin` |

`CFuncTrackTrain::m_ppath` and `m_controlMins`/`m_controlMaxs` are deliberately not requested: the path
comes from the entity lump, and the control box is `pev->mins`/`pev->maxs` with `maxs.z += 72`, as
`Spawn` computes it.

### Objective offsets belong to the map

`GameData::LoadMap(gamedir, MapOffsets &)` reads a second request list per game (`kCsRequests`,
`kDodRequests`, `kTfcRequests`) for the **map owner** only, from `Objectives::DetectMap` at
`ServerActivate`. They describe the map's entities (a `CAreaCapture`, a `CHostage`), so they are read
through `XPlay::PrivateData(ent, mapOwner)`, never `g_offsets`, which belongs to whichever bot is
thinking. A missing field turns objectives off with the reason and path printed.

Two layouts to know before adding to these lists: `m_hTargetEnt` and `m_hOwner` are `EHANDLE`s, read
with the serial check; `sz_RequiredObjectGroup` and `sz_Group` (on `CObject` and DoD's `CBreakable`)
are inline `char[256]`, not `string_t`, copied out with a length bound. DoD's `CBasePlayer::m_pObject`
is a raw `CBaseEntity *`, compared against the object's private data, not an `EHANDLE`.

## Helpers in src/game/

### player_models.cpp

Which player models a game has installed, for HLDM's third `crossbot_add` argument. `PickRandom`,
`Installed`, `PrintList` and `Forget` are the interface; `CHLDMProfile::ApplyAddArg` is the only caller.

A model is a directory under `models/player` holding a `.mdl` of the same name
(`models/player/gordon/gordon.mdl`), and an `fopen` of that path is the whole test. It checks existence,
stands in for a directory test (`readdir`'s `d_type` is `DT_UNKNOWN` on plenty of filesystems), and keeps
CrossPlay's own `crossplay/models/player/` out: its 43 entries are per-game remaps
(`gordon/gordon_cs4.mdl`, `gordon_dod5.mdl`) with no `gordon.mdl`.

`XPlay::FindMapBsp` does not probe like this: it asks the engine's `IFileSystem::GetLocalPath` for
`maps/<map>.bsp`, because CrossPlay also mounts each game's `crossplay.smc` `alias` dir (`cstrike` mounts
`czero`), so `de_fastline_cz` loads from `czero/maps/` with no `czero` game loaded. Probing the game dirs
is only the fallback when `filesystem_stdio` is not found.

A game with no `models/player` falls back to `valve`'s. DMC is why: `dmc/models/` has one flat
`player.mdl`, and CrossPlay's `crossplay.smc` names HL models in its `dmc` block. Paths are bare gamedir
names relative to the HLDS working directory. The scan is cached per game and dropped in
`ServerDeactivate`, so models copied in while running appear on the next map.

**A bot not wearing what the console said is CrossPlay, not this code.** `crossplay.smc` has a per-game
whitelist (`games/valve/teams/DEATHMATCH` is `barney;gordon;helmet;zombie;gman`) and clamps anything
else; valve has 14 models installed, so most picks are pulled back to five. Reading the whitelist
instead was rejected: scanning keeps CrossBot independent of CrossPlay's config and works on plain
HLDS.

### fake_cmd.cpp

Makes a fake client issue a console command. `FakeCmd::Run(player, "weapon_9mmAR")` is the interface;
its one caller is `CHLDMProfile::SelectWeapon`.

**A client command is the only correct route.** Any command starting `weapon_` goes to
`CBasePlayer::SelectItem` (`client.cpp:553`), the only path that runs `Holster()` on the old weapon and
`Deploy()` on the new, sets `m_flNextAttack` to the deploy time and updates client weapon state.
`pfnClientCommand` sends a command *to* a client, which a fake client does not have. Writing
`m_pActiveItem` through an offset would skip Holster and Deploy and let the bot fire through a switch a
human cannot.

**The hook is flag-gated.** The game reads its command through the engine's `CMD_ARGV` / `CMD_ARGC` /
`CMD_ARGS`. `Run` tokenises the string, raises a module-local flag, calls `MDLL_ClientCommand` and lowers
it; three Metamod pre-hooks on `pfnCmd_Argv` / `pfnCmd_Argc` / `pfnCmd_Args` (in `engine_hooks.cpp`)
return `MRES_SUPERCEDE` with the fake tokens while the flag is up and `MRES_IGNORED` otherwise. They sit
on the hot path of every real command, so they are live only inside that one synchronous call, and
`Run` refuses to re-enter.

**Classnames are case sensitive.** `SelectItem` compares with `FClassnameIs` (`strcmp`):
`weapon_9mmAR` works, `weapon_9mmar` silently selects nothing. The table uses exact SDK spellings, and
`crossbot_weaponspec <name>` proves them against the live entities. For the same reason `Run` drops a
command too long for its buffer rather than truncating it. `Run` returns whether it dispatched, not
whether the game acted; `BotFire` watches `BotInventory::activeIndex` instead.

### give_item.cpp

`GiveItem::Give(player, classname)` is the game's own `give`: `pfnCreateNamedEntity`, origin set to the
player's, `SF_NORESPAWN` raised, `pfnSpawn`, `pfnTouch`. There is no `GiveNamedItem`: that is a
`CBasePlayer` method with no offset, while spawn-and-touch uses the DLL API and works for any classname,
ammo included. `SF_NORESPAWN` (`1 << 30`, defined locally since the plugin does not include
`weapons.h`) stops the pickup becoming a permanent extra item.

It is its own file because its two callers differ: `crossbot_equip` announces every give, and the pin's
auto-equip fires every half second for several bots and must announce nothing. So `Give` returns a bool
and prints nothing.

`GiveItem::Spawn(classname, origin)` is the same without `pfnTouch`: it leaves the entity in the world
and returns the edict, for `crossbot_spawn` and the monster test's live `monster_snark`s
(`doc/testing.md`). It checks `pent->free` after `pfnSpawn`, since an entity may remove itself during
spawn. It takes no owner on purpose: a snark reads `pev->owner` into `m_hOwner` at spawn and treats that
player differently, which is wrong for a test that wants it to hunt whoever is nearest.

### ent_handle.h

**`CBotEntHandle`** is the SDK's `EHANDLE` retargeted at `edict_t`. The serial number is the point: the
respawn memo outlives its entity by up to a minute and edict slots are recycled, so a raw pointer would
eventually name an unrelated entity. **`Get()` also checks `ent->free`**, which the SDK does not need (a
`CBaseEntity` never outlives its edict) but a memo does: a freed edict keeps its serial until reuse.

`CBotPlayerNameOf`, `CBotTargetNameOf` and `CBotIsLivePlayer` (`ent_handle.cpp`) walk
`pfnPEntityOfEntIndex(1..maxClients)`, shared so "what counts as a live player" has one definition. Use
`CBotTargetNameOf` in any line that can name a non-player: it returns the netname for a client and the
classname otherwise, where `CBotPlayerNameOf` prints `?`. `CBotClassOf` is declared here but defined in
`hldm_profile.cpp`, a wart, since nothing about it is mod-specific.

## The weapon table (weapon_spec.h)

What the bot needs to **operate** a gun, as data. `src/bot/` never names a weapon: it asks
`IGameProfile::GetWeaponSpec(id)` and gets a `WeaponSpec` describing the modes. The enums carry no
Half-Life constants, so `src/bot/` may include them.

```
WeaponSpec: id, classname, clipSize, value, rangeClass, zoomFov, primary, primaryZoomed, secondary
WeaponMode: fire, aim, toggle, ammo, ammoPerShot, speed, gravity, range, splash,
            chargeFull, chargeMax, quirks
```

**The one criterion for a field: does it tell the bot how to operate this gun?** Facts observable at
runtime, and facts that only matter when choosing between guns, are left out; the rejected list at the
end is part of the design. Operating decomposes into three questions, one enum each: how to drive the
button (`WeaponFire`), where to point (`WeaponAim`), what persistent state the press flips
(`WeaponToggle`).

### value, rangeClass and zoomFov fail the criterion on purpose

None says how to operate the gun. Each is a judgement about the mod that no runtime observation can
supply, and it belongs next to the weapon rather than in a second table keyed by `m_iId` that could
drift out of sync.

**`value`** has two readers:

- `EvaluateCombat`: how dangerous the bot is in total (`doc/goals.md`, "Carefulness").
- Weapon selection: which gun in the bag is worth holding (`doc/combat.md`, "Selection"). Clip,
  reserve, cycle time and range say whether a weapon *works*, never whether it is worth holding; a bot
  with a full glock and a full gauss would see no reason to prefer either.

HLDM's ladder: 0.5 gauss, egon, crossbow; 0.3 RPG and MP5; 0.2 shotgun, hornetgun, python; 0.1
everything else. It sums to 3.3, so `arms` saturates at about two good weapons; a sum reaching 1 only
with the full arsenal would mean bots almost never hunt. **The coarseness is load-bearing.** Selection
keeps only the top score and breaks the rest on rarity, so unique values would make one weapon always
win. Three weapons share 0.5 and six share 0.1, leaving rarity room to act. More precision would make
bots less varied, not more accurate.

**`rangeClass`** is `RANGE_CLOSE`, `RANGE_LONG`, or `RANGE_NONE` (no opinion: empty slots, Ricochet's
disc, TFC's tranq). `Bot_RangeBias` turns it into a routing weight, so a shotgun accepts tighter ground
and a crossbow the reverse (`doc/goals.md`, "Range bias"). Its two thresholds are cvars, because what
counts as long is a property of the map.

**`zoomFov`** is the scope's fov (40 python, 20 crossbow, 0 without `TOGGLE_ZOOM`). The toggle says a zoom
exists; this says how tight, so `BotCamp` can check whether a camp spot's arc fits in the scope. A
non-zero `zoomFov` also makes the weapon a camper gun (`BotFire::CampWeapon`, `doc/goals.md`). It is
on `WeaponSpec`, not `WeaponMode`, because a weapon has one scope.

### WeaponFire: FIRE_TAP is bot intent

There is one `ItemPostFrame` in the HLSDK. No weapon overrides it or reads `m_afButtonPressed`, and
`ITEM_FLAG_NOAUTOFIRE` does not exist in this SDK. The dispatcher is level-triggered on `pev->button`,
so **every Half-Life weapon is mechanically full auto**; the python only feels semi-auto because its
cycle is 0.75 s. (`IN_ATTACK2` also short-circuits `IN_ATTACK` there; see `doc/combat.md`.)

So `FIRE_TAP` means "the bot must not hold this button", for two reasons:

- Holding a **toggle** flips it again every cycle (python and crossbow secondary, RPG secondary).
- Holding a **hand grenade** cooks it. `PrimaryAttack` pulls the pin; the grenade leaves from
  `WeaponIdle`, reached only through the "no fire buttons down" branch. Held, it never throws, and past
  three seconds `ShootTimed` gets a fuse under 0.1 s, **zeroes the velocity** and drops it at your feet.

It is not redundant with anything in the SDK; do not delete it. Other games' semi-automatics are taps
for their own reasons (DoD latches `m_fInAttack`; CS pistols).

**`FIRE_HOLD` is not the opposite**: it only means "may hold", which is true of the shotgun, python,
crossbow, RPG and snark alike. What separates spraying from placing shots is cycle time, recorded as
`WPN_Q_STICKY`.

**`FIRE_CHARGE`** fires on release after a hold that is worth something; only the gauss charge grows
while held. The hand grenade looks like the same shape but its hold buys nothing and costs everything,
so it is `FIRE_TAP`, and the half second between press and throw is the engine's.

### WeaponAim: AIM_TARGET vs AIM_LEAD is "is the travel time zero"

Not "what leaves the muzzle". A crowbar, a shotgun and an egon beam all mean "point at the target", so
melee, hitscan and beam are one value; reach is `range`, continuous damage is `FIRE_HOLD`.

- **Homing weapons are `AIM_LEAD`**, because the flight time is real; `WPN_Q_HOMING` then tells the
  consumer not to lead. The aim value records physics, the quirk records handling.
- **The RPG stays `AIM_TARGET`.** Its rocket chases the `laser_spot` wherever the bot points now, so it
  is tracked, not led. "Keep pointing after you fire" is `WPN_Q_KEEP_AIMING`.
- `AIM_SURFACE` is a placement (tripmine); `AIM_NONE` is no aim at all (satchel primary, zoom-only
  modes).

### gravity is effective, not pev->gravity

`pev->gravity` multiplies `sv_gravity` (0.5 is 400 u/s² at 800); the table stores the multiplier, which
belongs to the weapon. The 800 is `PmSim::kGravity`; `sv_gravity` and `sv_stepsize` (the hardcoded 18
in `navgen::kStepHeight`) are treated as settings nobody moves. Moving them breaks jumps and lobs, a
trade taken deliberately.

**Two rows are parabolas, and not the same parabola:**

- **MP5 secondary**: `CGrenade::ShootContact` at `forward * 800`, `pev->gravity = 0.5`, `ExplodeTouch`
  on anything, no inherited velocity. An elevation solve describes it exactly.
- **Hand grenade**: a launch where direction and speed are one parameter (`WPN_Q_THROW_ARC`), and it
  **bounces**, on a fuse started at the pin pull. A solve finds where it lands; it explodes one to two
  seconds of rolling later.

Not parabolas:

- **Satchel**: `CSatchel::Throw` spawns a `monster_satchel` that lands and waits for a later radio press.
- **RPG** (`speed 2000, gravity 0`). `CRpgRocket::Spawn` tilts the launch 30° above the aim and throws
  at 250 on `MOVETYPE_BOUNCE` with gravity 0.5: for a level aim `vz = 125` against `g = 400`, a hump of
  about 19.5 units over 0.31 s from a muzzle at about `z + 20`, **peaking below head height**, so it
  cannot bury itself in a ceiling a player stands under, and `bot_fire.cpp` treats it as guided direct
  fire. Then `IgniteThink` switches to `MOVETYPE_FLY` and `FollowThink` accelerates toward 2000 while
  steering at the spot. **It does not run out**: the decay is `velocity * 0.2 + target * speed * 0.798`,
  0.2% per 0.1 s tick, about 14 s (28,000 units) to fall below 1500, and `MOVETYPE_FLY` gets no gravity
  or friction. `range 0` is literal.

**The table deliberately disagrees with the SDK in three rows.** The crossbow bolt sets
`pev->gravity = 0.5` and `MOVETYPE_FLY`, which ignores gravity until `BoltTouch` changes the movetype;
the hornet is the same in both modes. All three carry gravity 0; copying `pev->gravity` would make the
bot aim low with the flattest-shooting weapon in the game.

Damage falls off linearly to zero at `splash`, and **the thrower is not excluded**: `CGrenade::Explode`
clears `pev->owner` before `RadiusDamage`. Hence the bot's "closer to them than to me" rule.

Shapes a single `speed` cannot hold, all quirks:

- The hand grenade's speed depends on pitch (below); 400 is the level-aim case.
- The crossbow bolt halves to 1000 underwater (`WPN_Q_SLOW_IN_WATER`).
- The RPG's 2000 is a cap reached about 0.9 s after launch; it is guided, so nothing needs the curve.
- Velocity inheritance is inconsistent: the hand grenade, satchel, snark and RPG add the player's
  velocity, the MP5 grenade explicitly does not. Hence `WPN_Q_INHERIT_VEL`.

### The throw law

Direction and speed are one parameter:

    angThrow.x = -10 + view.x * ((view.x < 0) ? 80 : 100) / 90
    flVel      = min(500, (90 - angThrow.x) * 4)

Five coefficients for one weapon. They live behind `IGameProfile::ThrowLaunch` (`CHLDMProfile`;
CS changes the rate and cap, DMC and TFC have their own launcher law). The alternative was five floats
on every row or magic numbers in `src/bot/`. `WPN_Q_THROW_ARC` selects it, so no consumer needs to know
which weapon it is. The solve is `doc/combat.md`, "The coupled throw solve".

### WPN_Q_HOMING: the thing aims itself

Hornetgun primary and the snark. The red hornet launches at 300, accelerates and steers every think,
so its 600 is an effective figure for a curve. What follows is not a wider tolerance but **no lead at
all**: `bot_fire.cpp` aims at the target and fires inside a wide cone, so `speed` and `gravity` stop
mattering for the aim.

The **snark is not a projectile**: `CSqueak::PrimaryAttack` traces 20-64 units ahead, spawns a
`monster_snark` there, tosses it at `forward * 200 + player velocity`, and it hunts on its own legs. 200
u/s is slower than a 320 u/s player, so a lead iteration would diverge rather than merely err.

Hornet **secondary** does not carry it: the orange dart flies straight at 1200 and is led like a bolt.
That two modes of one weapon differ is why quirks are per mode.

The quirk does not excuse `gravity`, which still earns the uphill veto: a snark thrown above the eye
lands short and walks back. `splash` rejects no mode; every `splash > 0` mode is judged on where it
lands. The snark's splash is 0.

### WPN_Q_STICKY: a cycle time without storing one

Every weapon is full auto, so "is this a sprayer" is a question of cycle speed, and HL's ladder has a
cliff:

| mode | cycle | |
|---|---|---|
| MP5 primary, hornetgun secondary | 0.10 s | sticky |
| gauss primary, glock secondary | 0.20 s | sticky |
| egon primary, hornetgun primary | 0.25 s | sticky |
| glock primary | 0.30 s | sticky |
| shotgun, python, crossbow primary | 0.75 s | not |
| MP5 secondary, RPG, shotgun secondary | 1.0 s+ | not |

`WPN_Q_STICKY` marks the seven HL modes at or under 0.30 s (other games mark their automatics); it lets
`bot_fire.cpp` keep the trigger down through frames the crosshair is off target (`doc/combat.md`, "The
sticky trigger"). It is a quirk, not a `cycleTime` field: the live `nextPrimaryAttack` is
authoritative, and what the bit records is the *judgement* that 0.30 is where spraying stops being
sensible. A number compared against a threshold hidden in `src/bot/` would bury the judgement and
imply false precision.

Gauss primary sets no weapon timer (only `m_flTimeWeaponIdle` and the player's `m_flNextAttack`), so
its 0.20 is the player timer's. Hornetgun primary is also homing, whose 20° cone passes almost every
frame, so sticky is nearly a no-op there; it is set because the table describes the weapon, not the
code path.

### Ammo fields

`WeaponAmmo` looks derivable from the inventory's ammo types and is not: "secondary fire spends
secondary ammo" is false for the shotgun (spends the clip), crossbow (both modes spend bolts) and gauss
(both spend uranium). Without it the bot cannot tell whether `IN_ATTACK2` will do anything.

`ammoPerShot` is what one press costs: 1 everywhere except shotgun secondary (2 from the clip) and
gauss primary (2 uranium), so a bot with one shell does not press secondary forever. The egon and the
charging gauss have no discrete shot: the egon burns 1 every 0.2 s while held, the gauss 1 every 0.1 s
while spinning up, and both carry the per-tick cost of 1. A consumer budgeting ammo multiplies by the
hold time; `chargeFull` is the only hint (`ShotCost` in `doc/combat.md`).

### primaryZoomed: zoom is not a fire mode

Zoomed crossbow primary in **multiplayer** calls `FireSniperBolt()`, a traceline for a flat 120: same
button, but the aim flips from `AIM_LEAD` to `AIM_TARGET`. So zoom is a `WeaponToggle` with its own mode
slot rather than a bool. `primaryZoomed.fire == FIRE_NONE` means no override, keeping the consumer one
line:

    const WeaponMode &m = (zoomed && spec->primaryZoomed.fire) ? spec->primaryZoomed : spec->primary;

Zoom state comes from `IGameProfile::Zoomed`, which reads engine-side `pev->fov` by default, so no
offset is needed.

### The deployables

`WPN_Q_HIDDEN_STATE` marks a mode whose behaviour depends on private state; `BotFire::Supported`
rejects it, and `bot_deploy.cpp` drives those weapons itself.

**The satchel, both modes.** Primary is three actions on `m_chargeReady`: throw at 0, radio-detonate at
1, inert at 2; secondary branches on the same field. What it throws is a mine that only hurts when the
radio is pressed later, so the state and the throw are one feature. `m_chargeReady` is on `CBaseEntity`
beside `m_flStartThrow`, `m_fInAttack` and `m_fireState`, requested as `ent_chargeReady` and filled only
for a `WPN_Q_HIDDEN_STATE` mode. `bot_deploy.cpp` needs the real field twice: the **throw** is confirmed
by `m_chargeReady == 1` (the satchel is 80 units downrange before a position check could find it), and
**detonation** is refused unless it is 1, or the bot would throw a second satchel meaning to blow the
first. Inferring it from live `monster_satchel`s is close but wrong: `AddToPlayer` resets the field on
pickup while old charges are still out and owned by the same edict.

**RPG secondary** toggles `CRpg::m_fSpotActive`, and the quirk keeps the bot off a button it must never
press. `CRpg::Spawn` sets the spot on, so **the laser is on for the life of every RPG**, and secondary
would only turn it off. `TOGGLE_LASER` is handled by never pressing (`Supported` rejects toggles too).
That is why `m_fSpotActive` is not in `GameOffsets`: the bot could only use it to press a forbidden
button. If a human turns the spot off and a bot picks the weapon up, the rocket flies along its launch
aim, which mostly still arrives.

### The OP4 quirks

Each says how to operate one weapon; none is a type tag.

- **`WPN_Q_LASER_ON`** (Eagle secondary): keep this toggle on. The Eagle laser starts **off**, and on is
  the mode worth having (spread 0.001 instead of 0.1, at a 0.5 s cycle instead of 0.22). The primary row
  describes laser-on, so it is not sticky. `Buttons` presses `IN_ATTACK2` while the gated
  `eagle_iLaserActive` reads 0, once per weapon object, since holstering keeps the flag.
- **`WPN_Q_SPINUP`** (displacer primary): the shot leaves `chargeFull` seconds after the press, aimed
  wherever the player looks then. `ShotCost` only multiplies for `FIRE_CHARGE`, so the 20-uranium cost is
  untouched. `WPN_Q_KEEP_AIMING` could not be reused: it gates a read of `CRpg::m_cActiveRockets` that
  would land in unrelated memory on a `CDisplacer`.
- **`WPN_Q_AIM_FEET`** (displacer primary, also DMC and TFC rockets): aim at the ground under the
  target. A direct displacer hit only teleports (`CDisplacerBall::BallTouch`); the 250 is splash.
- **`WPN_Q_BRACE`** (M249 primary, and other games' stance- or speed-sensitive guns): stop and duck to
  fire (`doc/combat.md`, "Brace").

`WPN_Q_INHERIT_VEL` on a plain arc is solved in the thrower's frame (the spore grenade); `WPN_Q_TEAM_UNSAFE`
marks throwables that hurt or blind friends (`doc/combat.md`).

### Rejected fields

True facts that fail the criterion, listed so they are not re-proposed:

| Rejected | Why |
|---|---|
| `MELEE` / `HITSCAN` / `BEAM` aim values | One instruction between them. Reach is `range`, sustained fire is `FIRE_HOLD`. |
| `HOMING` aim value | The flight time is real, so the aim stays `AIM_LEAD`; the handling difference is `WPN_Q_HOMING`. |
| `cycleTime` | The live `nextPrimaryAttack` is authoritative. Ranking unheld guns is `value`'s job at the coarseness selection needs, and the sticky trigger needs the judgement about the cliff, which is `WPN_Q_STICKY`. Raised twice, rejected twice. |
| a `needsSurface` quirk | Redundant with `AIM_SURFACE`. |
| an `exhaustible` quirk | The inventory walk runs every frame; the bot sees the weapon disappear. |
| a `regenAmmo` quirk | The bot sees the hornet count climb; waiting or switching is selection. |
| a `selfDamage` quirk | Redundant with `splash > 0`. |
| a `selfPush` quirk | The gauss jump is movement, modelled in `bot_gauss.cpp`. |
| throw-law coefficients | Five floats on every row for one weapon's curve; `WPN_Q_THROW_ARC` + `ThrowLaunch`. |
| a `guided` quirk | Redundant with `WPN_Q_KEEP_AIMING`: you keep pointing because the projectile follows. |

Spread cones, pellet counts, damage and projectile lifetimes are not fields either; some are noted
below because they were expensive to establish. `zoomFov` is a field despite failing the criterion,
for the camp scope check (see above).

## The Half-Life table (hldm_weapons.cpp)

`kHLDMWeapons[16]` indexed by `m_iId`, so the lookup is free and needs no string compares.

**It carries multiplayer values throughout**, because `CHLDMProfile` is a deathmatch profile:

| | multiplayer | single player |
|---|---|---|
| gauss full charge | 1.5 s | 4 s |
| MP5 cone | 3° | 6° |
| shotgun pellets | 4 / 8 | 6 / 12 |
| egon ammo drain | 1 per 0.2 s | 1 per 0.1 s |
| hornet lifetime | 3.5 s | 5.0 s |
| python zoom | works | returns immediately |
| crossbow zoomed primary | hitscan | still a bolt |

- **The MP5 cone**: the `if (!IsMultiplayer())` branch takes `VECTOR_CONE_6DEGREES`, so multiplayer
  gets the *narrower* 3°. The SDK's comments in that function are swapped relative to the code.
- **The gauss spin-up** burns one uranium every 0.1 s in multiplayer (0.3 s single player), so a full
  1.5 s charge costs 16 (one to start plus fifteen), not 6. Burning stops at full charge
  (`m_flNextAmmoBurn = 1000`), so **holding a charged gauss is free**, which is why a bot can carry one.
- **Splash** is `RadiusDamage`'s `flDamage * 2.5`: with retail `skill.cfg`, 250 for the RPG, M203
  grenade and hand grenade (100 damage), 375 for the satchel (150).
- **MP5 secondary carries `WPN_Q_NO_UNDERWATER`**, primary does not: `CMP5::SecondaryAttack` tests
  `waterlevel == 3` first and plays the empty sound. Nothing checks a field until something drives it.

**Two holes, both real.** `WEAPON_CHAINGUN` is id 5 with no entity in retail, and slot 0 is
`WEAPON_NONE`. `HLDM_WeaponSpec` returns null for both, so the dump shows 14 weapons in 16 slots and a
fully equipped player's `pev->weapons` reads `0xffde`, bit 5 clear.

**The lookup validates its own index.** `HLDM_WeaponSpec` returns null when `spec.id != id`, not only
for an empty row: a row in the wrong slot would otherwise return a confidently wrong weapon.
`crossbot_weaponspec` with no argument walks the raw slots and prints `TABLE BROKEN` on any mismatch;
`crossbot_weaponspec <name>` compares each spec's `classname` against `CBotClassOf` on the live weapon
entity, the only thing that catches a wrong id.
