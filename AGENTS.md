# CrossBot

CrossBot is a Metamod plugin for GoldSrc (Half-Life 1) that adds navigation-mesh-driven bots. The runtime plugin (`crossbot_mm.dll`) and an offline nav-graph compiler (`navgen.exe`) share the planning/steering code verbatim via `src/nav/`.

It runs inside **CrossPlay**, a hyper-game that loads several real game DLLs into one HLDS at once, so
every bot belongs to a game and the plugin talks to game state through `crossplay_api.h` — see
`doc/games.md` before touching anything that reads private data or calls into a game DLL.

How the systems actually work lives in `doc/`. This file is build, commands and house rules only. If you find yourself explaining behaviour here, it belongs in `doc/`.

## Code Style

- **Main source: Use HLSDK style C++, use std if necessary.**
- **Use floats in the bot code, use doubles in navgen, in shared code use floats.**
- **Tools and scripts: Use PowerShell! Don't use CMD! Use Powershell! Don't use Python! Use powershell! Don't use Node.JS! Use powerShell! Don't use Perl! Use POWERSHELL!**
- **Comments: Do not add comments. All comments will be deleted! If you cannot escape you irresistable urge to write comments, put them in doc/ instead.**
- **Keep the size of AGENTS.md and all SKILL.md files to be between 50 to 100 lines. Don't duplicate what is already in doc/**
- **Edit files with the Edit tool. Never `perl -i` or `sed -i`: they leave `.bak` files and can truncate the file.**

## Writing doc/

- A doc says how the system works **now**: what it does, then why, then the numbers that justify it.
- No diaries. No "used to", "the first attempt", "once broke", and no dates except on a measurement. A
  fixed bug becomes the rule it taught plus one line of why.
- Keep what is expensive to rediscover: measured numbers (with map and settings), rejected alternatives
  and why they lost, invariants the code relies on.
- Every function, file, flag and cvar named must exist. Renaming code updates the doc in the same change.
- One owner per fact: the doc of the system it belongs to. Everywhere else links to it.
- Change the section in place. Never append a paragraph that contradicts an older one.

## Important Notes

- **DO NOT include target areas (info_target) as seed areas in navgen.** The targets are for debugging/validation only. The navgen must discover reachability through normal connectivity (walk, jump, elevator, teleport, launch edges). Using targets as seeds would artificially keep unreachable areas.
- **Debug output is always a command-line flag.** Never gate diagnostics behind environment variables.
  Every debug dump is a `--dump-*` flag on navgen, so a run is reproducible from its command line alone.
- **Never touch anything that belongs to CrossPlay.** Not the code behind `external/crossplay/`
  (machine-local symlinks into the CrossPlay project; `crossplay_api.h` is read-only to us), not its
  deployed data (`D:\hlds\crossplay\` — `liblist.gam`, `server.cfg`, `mapcycle.txt`, `crossplay.smc`,
  `gamedata\`, `addons\metamod\`), and not the `gEntityOwners` array. CrossBot writes exactly one place:
  `D:\hlds\crossplay\addons\crossbot\`. Registering the plugin in `addons/metamod/plugins.ini` is a
  manual, operator-owned step; the scripts warn when it is missing and never edit it.
- **What the plugin reads at runtime**: `crossplay/gamedata/<gamedir>-<win|linux>.json`, one per loaded
  game (**CrossPlay's, not ours**), plus `nav/<map>.nav` and `trace/` under `crossplay/addons/crossbot/`.
  Missing or wrong gamedata disables that game's bots with a printed reason; a bad offset **read** still
  kills the server on purpose — see `doc/games.md`.
- **Coordinates (GoldSrc / Hammer):** X = East, Y = North, Z = Up (right-handed). The SVG debug view
  flips Y (screen Y down), so world Y+ = North = SVG Y-.

## Build & Commands

Visual Studio 2022 (x86 build tools) + Windows 10 SDK, CMake 3.20+, Ninja.

### Build
```powershell
& .\build.bat
```

Just do `& .\build.bat`, you are in powershell, so just do that. No fucking bullshit with cmd /c or trying to build it yourself. Enough of that shit!

GoldSrc only loads a 32-bit plugin and navgen is 64-bit, so each CMake configure builds one target:
`build\crossbot_mm.dll` (x86) and `build-x64\tools\navgen\navgen.exe` (x64). `build.bat` builds **both**,
each in its own compiler environment, then copies `crossbot_mm.dll` into `D:\hlds\crossplay\addons\crossbot\`.
`crossbot_mm` links a **Debug** `crossplay.lib`, so CrossPlay's asserts name any context rule you break.

`build.bat standalone` builds a CrossPlay-free plugin into `build-standalone\` and deploys nothing; that
variant installs only under `D:\hlds\<game>\addons\crossbot\` (`doc/games.md`, "Standalone build").

The copy fails while a server holds the DLL, and a rerun reports `ninja: no work to do` yet still
deploys. **Check for the `build.bat: deployed crossbot_mm.dll` line before believing any runtime result.**

### Run navgen
```powershell
.\build-x64\tools\navgen\navgen.exe maps\test1_simple.bsp -o out.nav
```
No arguments gives the authoritative flag list; `doc/navgen.md` says what each dump means. Test maps
live in `maps/`. `.\scripts\generate_all_svgs.ps1` runs navgen over all of them into `nav_debug/`.

### Run the server by hand
```powershell
D:\hlds\hlds.exe -console -game crossplay -xgame1 valve -sgame1 valve +map crossfire
```
`-xgameN` is what CrossPlay loads as `gGameDirs[N-1]`; `-sgameN` is the engine's content search path for
the same mod. Add a bot with `crossbot_add <gamedir> [name] [model|team]` — see `doc/console.md`.
**Before adding bots by hand, run `amxx pause crossbot_menu.amxx`.** That AMXX plugin keeps a `bots_max`
quota: it kicks hand-added bots and adds and kicks bots of other games on its own.

### Tests
```powershell
.\scripts\test_nav_goal.ps1 -Map maps\test1_simple.bsp
.\scripts\test_nav_goal_all.ps1
```
The script prints only the verdict; the plugin's output goes to `nav_debug\logs\<map>_server.log`. Tests
drive the live `D:\hlds` install (maps into `valve\maps\`, files under `crossplay\addons\crossbot\`).
Raising `-MaxParallel` past 6 changes bot behaviour. `test_fire.ps1` covers combat, `test_deploy.ps1`
tripmines/satchels; the nav sweep sees neither.

**Never judge a change on one map.** Which sweep a change needs, including the mesh and route sweeps, is
in `doc/testing.md`.
