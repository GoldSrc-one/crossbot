# CrossBot

CrossBot is a Metamod plugin for GoldSrc (Half-Life 1) that adds navigation-mesh-driven bots. The runtime plugin (`crossbot_mm.dll`) and an offline nav-graph compiler (`navgen.exe`) share the planning/steering code verbatim via `src/nav/`.

How the systems actually work lives in `doc/`, one file per system, keyed by source file: `doc/navgen.md`,
`doc/nav.md`, `doc/bot.md`, `doc/game.md`, `doc/console.md`, plus `doc/testing.md` for the scripts. This
file is build, commands and house rules only. If you find yourself explaining behaviour here, it belongs
in `doc/`.

## Code Style

- **Main source: Use HLSDK style C++, use std if necessary.**
- **Use floats in the bot code, use doubles in navgen, in shared code use floats.**
- **Tools and scripts: Use PowerShell! Don't use CMD! Use Powershell! Don't use Python! Use powershell! Don't use Node.JS!**
- **Comments: Do not add comments. All comments will be deleted! If you cannot escape you irresistable urge to write comments, put them in doc/ instead.**
- **Keep the size of AGENTS.md and all SKILL.md files to be between 50 to 100 lines. Don't duplicate what is already in doc/**

## Important Notes

- **DO NOT include target areas (info_target) as seed areas in navgen.** The targets are for debugging/validation only. The navgen must discover reachability through normal connectivity (walk, jump, elevator, teleport, launch edges). Using targets as seeds would artificially keep unreachable areas.

- **Debug output is always a command-line flag.** Never gate diagnostics behind environment variables.
  Every debug dump is a `--dump-*` flag on navgen, so a run is reproducible from its command line alone
  and shows up in `--help`.

- **Everything the plugin reads at runtime lives under `<gamedir>/addons/crossbot/`** next to the DLL:
  `gamedata/<gamedir>-<win|linux>.json` (member offsets into the game DLL's private data), `nav/<map>.nav`,
  and `trace/`. `build.bat` creates all three and deploys `gamedata/` from the repo. Missing or wrong
  gamedata **kills the server on purpose** — see `doc/game.md`, `gamedata.cpp`.

- **Coordinate conventions (GoldSrc / Hammer):**
  - X = East, Y = North, Z = Up (right-handed)
  - SVG debug view: Y is flipped (screen Y down), so world Y+ = North = SVG Y-

## Build & Commands

### Prerequisites
- Visual Studio 2022 (x86 build tools) + Windows 10 SDK
- CMake 3.20+, Ninja

### Build
```powershell
& .\build.bat
```

Just do `& .\build.bat`, you are in powershell, so just do that. No fucking bullshit with cmd /c or trying to build it yourself. Enough of that shit!

It builds **both** targets and then copies `crossbot_mm.dll` into `hlds\valve\addons\crossbot\`. Build
them together — the plugin once sat broken for several commits because only `navgen` was built.

The copy fails if the file is locked, which means a server is still up. Only the *copy* fails: a rerun
after the server exits reports `ninja: no work to do` and still deploys, so until that rerun the
deployed plugin is stale while everything looks fine. **Check for the `build.bat: deployed
crossbot_mm.dll` line before believing any runtime result.**

### Output
- `navgen.exe` → `build\tools\navgen\navgen.exe`
- `crossbot_mm.dll` → `build\crossbot_mm.dll`

### Run navgen
```powershell
.\build\tools\navgen\navgen.exe maps\test1_simple.bsp -o out.nav
```

Run `navgen.exe` with no arguments for the authoritative flag list; `doc/navgen.md` says what each dump
means and how to read it. Test maps live in `maps/`.

### Batch generate SVGs for all maps
```powershell
.\scripts\generate_all_svgs.ps1
```
Runs navgen on every `.bsp` in `maps/`, writes `.svg` and `.nav` files to `nav_debug/`, and regenerates
`nav_debug/index.html` for the debug gallery.

### Automatic nav goal test (full HLDS run)
```powershell
.\scripts\test_nav_goal.ps1 -Map maps\test1_simple.bsp
```

The script's own output is the pass/fail verdict only. Everything the plugin printed goes to
`nav_debug\logs\<map>_server.log` for that run (and, shared across runs, `hlds\qconsole.log`).

### Sweep every map
```powershell
.\scripts\test_nav_goal_all.ps1
```
Per-map transcripts in `nav_debug\logs\`. Raising `-MaxParallel` past the default 6 loads the box enough
to change bot behaviour — see `doc/testing.md` before trading that for wall clock.

### Test combat
```powershell
.\scripts\test_fire.ps1
```
The nav sweep cannot see firing -- one bot per server means no target. `-Full` for the rarer cases.

### Test deployables
```powershell
.\scripts\test_deploy.ps1
```
Tripmines, satchels and beam avoidance. The nav sweep runs with `crossbot_deploy 0` -- mines change
every route. `-Phase` runs one of gates/tripmine/standoff/satchel.

### Sweep the mesh, then the routes
```powershell
& .\.claude\skills\crossbot-navgen\scripts\sweep.ps1 -Out before.txt
```
```powershell
Get-ChildItem maps\*.bsp | ForEach-Object { $n=$_.Name
    & .\build\tools\navgen\navgen.exe $_.FullName --dump-path 2>&1 |
        Select-String '^  wp' | ForEach-Object { "${n}: $_" } } | Set-Content before_wp.txt
```

**Never judge a change on one map.** What each sweep covers, which one a given change needs, and the
options and behaviour of `test_nav_goal.ps1` are in `doc/testing.md`. The `crossbot-nav` and
`crossbot-navgen` skills have the technique for reading the diffs.

- `scripts/nav_target_test.ps1` greps qconsole.log for "CrossBot: <name> reached goal" lines
