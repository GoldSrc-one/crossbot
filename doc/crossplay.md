# CrossPlay

`src/game/crossplay.h`, `src/game/crossplay.cpp`

CrossBot no longer runs on a server with one game DLL. It runs on **CrossPlay**, a hyper-game that
loads several real game DLLs into one HLDS process and multiplexes them onto a single entity pool.
Everything in this file is about the seam between the two.

## The server it lives in

```
hlds -console -game crossplay -xgame1 valve -sgame1 valve +map crossfire
```

- `-game crossplay` is the hyper-game directory, `D:\hlds\crossplay\`.
- `-xgameN <dir>` is a real game DLL for CrossPlay to load. These become `gGameDirs[N-1]`, and
  `gNumGameDlls` counts them. Up to 8.
- `-sgameN <dir>` adds the same mod to the engine's content search path, so its maps, models and
  sounds resolve.

The load chain is `engine -> crossplay/addons/metamod/metamod.dll (real Metamod) -> plugins.ini ->
crossplay/dlls/crossplay.dll -> per-xgame liblist.gam -> hl.dll and friends`. **CrossBot is a plugin
of that top-level Metamod**, so it sits *above* CrossPlay: its hooks always run with CrossPlay's
`gCaller == -1`, and its `g_engfuncs` is the raw engine table rather than CrossPlay's hooked one.

That placement is not a preference. `CrossPlay::GameContext` wraps the *non-nested* caller-context
macro, which raises `Caller change not allowed!` if a context is already active. A plugin loaded
below CrossPlay (in a per-game `addons/metamod/plugins.ini`) would already be inside one.

Registering the plugin — a `win32 addons/crossbot/crossbot_mm.dll` line in
`crossplay/addons/metamod/plugins.ini` — is an operator step. Nothing in this repo writes that file;
`test_server_start.ps1` warns when the line is absent.

## The data model

CrossPlay keeps one `edict_t` pool but **one private-data object per game DLL per edict**:

| Symbol | Meaning |
| --- | --- |
| `gNumGameDlls`, `gGameDirs[game]` | which games are loaded, and their directory names |
| `gEntityOwners[index]` | which game created edict `index`; `-1` for none |
| `gEntityObjects[game][index]` | that game's `CBaseEntity *` for edict `index` |
| `GameContext(game, cb, ctx)` | run `cb` with `game` as the caller |

`gEntityOwners[0]` is the world, so it names the game the **map** belongs to. `gEntityObjects` is a
full matrix, not a diagonal: when a game allocates an entity's private data, CrossPlay builds proxy
objects of the same classname in every other loaded game, so a `valve` bot can look at a `cstrike`
entity through `valve`'s eyes. A game with no such class gets a null there — that is the designed
soft failure, and every read is null-checked.

Entering a context is expensive: it rewrites `pvPrivateData` on *every* live edict and swaps
`gpGlobals->teamplay`. Entering the context that is already current is free (an early return), which
is what makes `MDLL_*` calls safe from inside a bot's own context — CrossPlay resolves the same owner
and its own context enter becomes a no-op.

## The two halves of "which game"

CrossBot separates them deliberately.

- `XPlay::EnterGame(i)` installs *CrossBot's* view: `g_offsets = g_gameOffsets[i]` and
  `g_pGameProfile = g_profiles[i]`. Cheap, no CrossPlay involvement. `XPlay::Current()` reports it,
  and `CBotPrivateData(ent)` reads `gEntityObjects[Current()][ent]`.
- `XPlay::InGame(i, lambda)` installs *the engine's* view for the duration of the call, via
  `GameContext`. Only needed when calling into a game DLL or an engine function that dispatches into
  one.

`InGame` is a template over a stateless thunk, so it takes a lambda:

```cpp
edict_t *pEdict = nullptr;
XPlay::InGame(iGame, [&] { pEdict = CREATE_FAKE_CLIENT(botName); });
```

Only `crossplay.cpp` includes `crossplay_api.h`. That header declares `maxGameDlls` inside
`extern "C"`, which gives it external linkage and multiply-defines it across translation units;
`crossplay.h` exposes `XPlay::kMaxGames` instead and static-asserts the two agree.

## Where CrossBot enters a context

- `BotManager::StartFrame` — one `EnterGame` + `InGame` per bot around `ThinkOneBot`, which covers
  the whole AI: respawn, `BotWeapons::Refresh`, `Bot_MoveNav` and everything under it, including
  `FakeCmd::Run` (weapon selection, via `MDLL_ClientCommand`) and `pfnRunPlayerMove`. Monster refresh
  runs once per frame in the **map owner's** context instead.
- `BotManager::AddBot` — around `CREATE_FAKE_CLIENT`. Mandatory: CrossPlay's `CreateFakeClient`
  ReHLDS hook `DebugError`s on `gCaller == -1`, and it is what stamps the new client's owner.
  `MDLL_ClientConnect` / `MDLL_ClientPutInServer` stay outside; CrossPlay enters its own.
- `BotManager::Touch`, `OnSound`, `OnEvent`, `KillAllBots` — per resolved bot.
- `GiveItem::Give` / `Spawn` — around `pfnCreateNamedEntity`, so CrossPlay stamps the owner as the
  entity's private data is allocated. `Give` takes the game from the receiving player.
- Console commands that read private data — `ScopedGame` in `commands.cpp`, keyed by the bot's
  `gameDll` or `XPlay::OwnerOf(player)`.

`ServerActivate` and `ServerDeactivate` need no context; they call `XPlay::DetectProfiles()` and
`XPlay::ForgetProfiles()`.

## Rules

1. **Never nest `InGame`.** One context at a time, entered from `gCaller == -1`.
2. **Never write `gEntityOwners`.** CrossPlay owns it; it stamps ownership from `gCaller` in
   `PvAllocEntPrivateData` and in the `CreateFakeClient` hook. Reading it is fine.
3. **Never read `edict->pvPrivateData` directly.** Outside a context it is garbage (`0xFFFFFFFF` in
   CrossPlay debug builds). Use `CBotPrivateData(ent)`, or `XPlay::PrivateData(ent, game)` when the
   entity's own game is the right lens — that is what `CBotIsLivePlayer` does.
4. **Never edit anything under `D:\hlds\crossplay\` except `addons\crossbot\`.**

## Reading CrossPlay's complaints

`crossplay.lib` symlinks to a **Debug** build, so these fire loudly on stderr (`D:\hlds\hlds.err`) and
each names a broken rule:

| Message | What it means |
| --- | --- |
| `Caller change not allowed!` | a context was entered while another was active — rule 1 |
| `Bad caller!` | an engine call that needs a context was made at `gCaller == -1` |
| `Entity without owner!` | an entity reached a dispatch with `gEntityOwners == -1` |
| `Owner mismatch!` | two games claim the same edict |
| `Wrong private data pointer on enter/exit!` | someone wrote `pvPrivateData` behind CrossPlay's back |

## Per-game state in CrossBot

`GameOffsets` and `IGameProfile` were both single globals keyed to "the" mod. They are now arrays.

- `gamedata.cpp` holds `g_gameOffsets[kMaxGames]` plus a per-game loaded flag, filled by
  `GameData::LoadAll()` at `GameDLLInit` from `crossplay/gamedata/<gamedir>-win.json`.
  `GameData::Activate(i)` copies one into the `g_offsets` every peek site still reads.
- `crossplay.cpp` holds `g_profiles[kMaxGames]`, filled by `XPlay::DetectProfiles()` at
  `ServerActivate` via `GameProfile_Detect(gamedir)`. `g_pGameProfile` is the active one.
- A game is **playable** (`XPlay::Supported`) only with both. `crossbot_add` refuses anything else and
  `XPlay::PickRandomSupported()` draws only from that set. Today only `valve` qualifies, because
  `CHLDMProfile::Detect` matches that directory alone.

Each `CBot` carries `gameDll`. It is set once in `AddBot` and is the key to everything above.
