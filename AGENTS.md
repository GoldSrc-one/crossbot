# CrossBot

CrossBot is a Metamod plugin for GoldSrc (Half-Life 1) that adds navigation-mesh-driven bots. The runtime plugin (`crossbot_mm.dll`) and an offline nav-graph compiler (`navgen.exe`) share the planning/steering code verbatim via `src/nav/`.

## Build & Commands

### Prerequisites
- Visual Studio 2022 (x86 build tools) + Windows 10 SDK
- CMake 3.20+, Ninja

### Build
```powershell
& .\build.bat
```

Just do `& .\build.bat`, you are in powershell, so just do that. No fucking bullshit with cmd /c or trying to build it yourself. Enough of that shit!

`build.bat` calls vcvarsall and runs `cmake --build build --target navgen`.

### Output
- `navgen.exe` → `build\tools\navgen\navgen.exe`
- `crossbot_mm.dll` → `build\crossbot_mm.dll`

### Run navgen
```powershell
cd d:\projects\trash\crossbot
.\build\tools\navgen\navgen.exe maps\test1_simple.bsp -o out.nav
```

### Debug SVG (offline inspection)
```powershell
.\build\tools\navgen\navgen.exe maps\test35_smoothwall.bsp --dump-svg test35_smoothwall.svg
```
Add `--trace <nav_trace.txt>` to overlay a recorded bot trajectory (written by the runtime plugin when a goal ends).

### Batch generate SVGs for all maps
```powershell
.\scripts\generate_all_svgs.ps1
```
Runs navgen on every `.bsp` in `maps/`, writes `.svg` and `.nav` files directly to `nav_debug/`, and regenerates `nav_debug/index.html` for the debug gallery.

### Test on a test map
```powershell
.\build\tools\navgen\navgen.exe maps\test1_simple.bsp --dump-areas --dump-svg test1_simple.svg
```
Test maps live in `maps/`. See `--dump-leaves`, `--dump-tactical`, `--dump-planes <id>`, `--dump-obj <out.obj>` for other debug outputs.

### Automatic nav goal test (full HLDS run)
```powershell
.\scripts\test_nav_goal.ps1 -Map maps\test1_simple.bsp
```
Spins up HLDS on an auto-assigned port (29015-29100), adds a bot, auto-selects a goal (farthest spawn pair or `info_target`), drives the bot to the goal, collects the movement trace, runs `navgen --dump-svg --trace` to produce `nav_debug/<map>.svg` with the recorded trajectory overlay, generates the `.nav` file, and shuts down HLDS. Exits 0 on goal reached, 1 on timeout/failure.

**Parallel-safe**: multiple instances can run simultaneously on different maps — each instance uses a unique port, map-specific trace file (`valve/crossbot_data/nav_trace_<map>.txt`), map-specific config (`_crossbot_autotest_<map>.cfg`), and map-specific output files (`nav_debug/<map>.svg`, `<map>.nav`, `nav_trace_<map>.txt`). The `crossbot_trace_path` cvar isolates trace logs per instance. Spawn/target discovery and goal detection use RCON instead of parsing shared `qconsole.log`.

Options:
- `-Timeout <seconds>` — absolute safety cap (default 60)
- `-TimeoutMultiplier <float>` — multiplier on bot's reported ETA (default 2.0); test fails at min(ETA × multiplier, Timeout)
- `-Port <port>` — HLDS port (default: auto-assign 29015-29100)
- `-NavGenPath <path>` — navgen.exe location (default `build\tools\navgen\navgen.exe`)
- `-HLDSDir <path>` — HLDS install dir (default `D:\projects\trash\crossbot\hlds`)
- `-OutputDir <path>` — output directory for SVG/NAV (default `nav_debug`)

### Run parallel tests
```powershell
# Sequential (default)
.\scripts\test_nav_goal.ps1 -Map maps\test1_simple.bsp
.\scripts\test_nav_goal.ps1 -Map maps\test16_float.bsp

# Parallel via background jobs
Start-Job { .\scripts\test_nav_goal.ps1 -Map maps\test1_simple.bsp }
Start-Job { .\scripts\test_nav_goal.ps1 -Map maps\test16_float.bsp }
Get-Job | Wait-Job | Receive-Job
```

## Code Style

- **Main source: Use HLSDK style C++, use std if necessary**
- **Tools and scripts: Use PowerShell! Don't use CMD! Use Powershell! Don't use Python! Use powershell! Don't use Node.JS!**
- **Comments: only when they contain information not present in the code itself. Even then, keep brief (one sentence). No restating variable names, no section banners, no decorative dividers.**

## Testing

- `scripts/nav_target_test.ps1` greps qconsole.log for "CrossBot: <name> reached goal" lines (a short goal can complete within one rcon poll interval)
- Debug visualization: `NavDebug::Draw()` emits `TE_BEAMPOINTS` temp entities; toggled via `crossbot_nav_debug_path` / `crossbot_nav_debug_area`