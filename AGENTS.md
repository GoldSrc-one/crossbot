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

## Important Notes

- **DO NOT include target areas (info_target) as seed areas in navgen.** The targets are for debugging/validation only. The navgen must discover reachability through normal connectivity (walk, jump, elevator, teleport, launch edges). Using targets as seeds would artificially keep unreachable areas.

- **Debug output is always a command-line flag.** Never gate diagnostics behind environment variables.
  Every debug dump is a `--dump-*` flag on navgen, so a run is reproducible from its command line alone
  and shows up in `--help`.

- **Never touch anything that belongs to CrossPlay.** Not the code behind `external/crossplay/`
  (machine-local symlinks into the CrossPlay project; `crossplay_api.h` is read-only to us), not its
  deployed data (`D:\hlds\crossplay\` — `liblist.gam`, `server.cfg`, `mapcycle.txt`, `crossplay.smc`,
  `gamedata\`, `addons\metamod\`), and not the `gEntityOwners` array. CrossBot writes exactly one place:
  `D:\hlds\crossplay\addons\crossbot\`. Registering the plugin in `addons/metamod/plugins.ini` is a
  manual, operator-owned step; the scripts warn when it is missing and never edit it.

- **What the plugin reads at runtime**: `crossplay/gamedata/<gamedir>-<win|linux>.json`, one per loaded
  game (**CrossPlay's, not ours**), plus `nav/<map>.nav` and `trace/` under
  `crossplay/addons/crossbot/`, which `build.bat` creates. A game with missing or wrong gamedata is
  disabled for bots with a printed reason; a bad offset **read** still kills the server on purpose —
  see `doc/games.md`, `gamedata.cpp`.

- **Coordinates (GoldSrc / Hammer):** X = East, Y = North, Z = Up (right-handed). The SVG debug view
  flips Y (screen Y down), so world Y+ = North = SVG Y-.

## Build & Commands

Visual Studio 2022 (x86 build tools) + Windows 10 SDK, CMake 3.20+, Ninja.

### Build
```powershell
& .\build.bat
```

Just do `& .\build.bat`, you are in powershell, so just do that. No fucking bullshit with cmd /c or trying to build it yourself. Enough of that shit!

It builds **both** targets and then copies `crossbot_mm.dll` into `D:\hlds\crossplay\addons\crossbot\`.
Build them together — the plugin once sat broken for several commits because only `navgen` was built.
`crossbot_mm` links against `external/crossplay/crossplay.lib`; that symlink points at a **Debug** build
of `crossplay.dll`, so CrossPlay's own asserts are live and will name any context rule you break.

The copy fails if the file is locked, which means a server is still up. Only the *copy* fails: a rerun
after the server exits reports `ninja: no work to do` and still deploys, so until that rerun the
deployed plugin is stale while everything looks fine. **Check for the `build.bat: deployed
crossbot_mm.dll` line before believing any runtime result.** Output lands in
`build-x64\tools\navgen\navgen.exe` and `build\crossbot_mm.dll`.

### Run navgen
```powershell
.\build-x64\tools\navgen\navgen.exe maps\test1_simple.bsp -o out.nav
```
No arguments gives the authoritative flag list; `doc/navgen.md` says what each dump means. Test maps
live in `maps/`. `.\scripts\generate_all_svgs.ps1` runs navgen over all of them into `nav_debug/` and
regenerates the debug gallery's `index.html`.

### Run the server by hand
```powershell
D:\hlds\hlds.exe -console -game crossplay -xgame1 valve -sgame1 valve +map crossfire
```
`-xgameN` is what CrossPlay loads as `gGameDirs[N-1]`; `-sgameN` is the engine's content search path for
the same mod. Add a bot with `crossbot_add <gamedir> [name] [model|team]` — see `doc/console.md`.

### Automatic nav goal test (full HLDS run)
```powershell
.\scripts\test_nav_goal.ps1 -Map maps\test1_simple.bsp
```

The script's own output is the pass/fail verdict only. Everything the plugin printed goes to
`nav_debug\logs\<map>_server.log` for that run (and, shared across runs, `D:\hlds\qconsole.log`). The
tests drive the live `D:\hlds` install: they add maps to `valve\maps\`, write under
`crossplay\addons\crossbot\`, and touch nothing else.

### Sweep every map
```powershell
.\scripts\test_nav_goal_all.ps1
```
Per-map transcripts in `nav_debug\logs\`. Raising `-MaxParallel` past the default 6 loads the box enough
to change bot behaviour — see `doc/testing.md` before trading that for wall clock.

`.\scripts\test_fire.ps1` covers combat (`-Full` for the rarer cases) and `.\scripts\test_deploy.ps1`
covers tripmines, satchels and beam avoidance (`-Phase` gates/tripmine/standoff/satchel). The nav sweep
sees neither: one bot per server means no target, and it runs with `crossbot_deploy 0`.

### Sweep the mesh, then the routes
```powershell
Get-ChildItem maps\*.bsp | ForEach-Object { $n=$_.Name
    & .\build-x64\tools\navgen\navgen.exe $_.FullName --dump-path 2>&1 |
        Select-String '^  wp' | ForEach-Object { "${n}: $_" } } | Set-Content before_wp.txt
```

**Never judge a change on one map.** What each sweep covers and which one a given change needs is in
`doc/testing.md`. `scripts/nav_target_test.ps1` greps qconsole.log for
`CrossBot: <name> reached goal`.
