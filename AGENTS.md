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
Areas are flat grey; areas flagged `NAV_AREA_CROUCH` get a light blue tint. Connections are white (walk), dashed red (air move) and green (teleport).

### Prism areas

`MergeAreas` merges leaves whose union is 3D-convex, and also leaves whose union is only a **vertical prism**: a
convex X-Y footprint extruded over a z-slab. That is what lets a staircase or a ramp become one area instead of
one area per step. A merge is allowed into a prism when the union's X-Y footprint stays convex, the merged floor
has no internal step taller than `kStepHeight`, and no leaf outside the group lives inside the resulting slab.
The slab test is evaluated against *leaves*, never groups, so an earlier prism can never be invalidated by a
later merge. When the slab test does fail, the merge absorbs the conflicting group if a candidate portal reaches
it and retries — a ramp only closes as a whole, so every pairwise step along it conflicts with the neighbour the
finished area would have swallowed anyway.

`Area::polytope` keeps the true multi-tread surface, so floor lookups, the step filter and air-move sampling all
see real treads. `Area::boundPlanes` is the convex region written to the `.nav` file: the polytope's own planes
for a convex area, the footprint's vertical planes plus a z-slab for a prism. **Any point-in-area test must use
`boundPlanes`** — a prism's raw face list is not a convex halfspace set, and intersecting it yields nothing.
`--dump-areas` marks these areas `prism`, and `--validate-areas` judges every area by its exported region. Areas are drawn as the 2D convex hull of their vertices, which for a prism area is exactly its footprint.

The SVG overlays the A* route from the first spawn point to the `info_target` origin (falling back to the farthest area's centroid when there is no usable target) as a single orange polyline, with a green dot at the start and a red dot at the goal. The endpoints are the real entity origins, not the containing areas' centroids; the goal is clamped into its area's bounds the same way the runtime's `Repath` clamps it. The route is computed with the same `NavAStar::FindPath`, traverse costs and `NavPlan::BuildWaypointPlan` funnel the runtime uses, so it shows the string-pulled waypoints a bot would actually follow — straight across open areas, hugging corners through corridors — not the raw portal endpoints.

Add `--dump-conns` to print every final connection with its type, flags, endpoints and air-move parameters.

### Prism areas

An area is normally a convex polytope, and the nav file exports its face planes verbatim. Stairs and
ramp/floor junctions are not convex in 3D, so `MergeAreas` also accepts a merge when the union's **X-Y
footprint stays convex**, its **z-slab is free of every leaf outside the group**, and its **floor stays
walkable** (no internal step over 18 units). Such an area is flagged `prism`: `Area::polytope` keeps the true
multi-tread surface for floor lookups, air-move launches and the OBJ/SVG dumps, while `Area::boundPlanes`
exports the footprint hull plus a z-slab — a convex region the runtime's `ContainsPoint` can test unchanged.
The z-slab check is what makes the two interchangeable for point lookup.

Anything in navgen that asks "is this point in this area" must use `boundPlanes`, never `polytope.faces`: for a
prism the faces include interior risers, and their halfspace intersection is empty.

`--validate-areas` reports the prism count, plus any area that is non-convex, overlaps another, or contains an
unclimbable internal floor step. All three must be zero.

`--dump-planes <id>` prints the exported bound planes first, then the true face windings.

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
Spins up HLDS on an auto-assigned port (29015-29100), adds a bot, auto-selects a goal (farthest spawn pair or `info_target`), drives the bot to the goal, collects the movement trace, runs `navgen --dump-svg` to produce `nav_debug/<map>.svg`, generates the `.nav` file, and shuts down HLDS. Exits 0 on goal reached, 1 on timeout/failure.

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
- **Comments: Do not add comments. All comments will be deleted!**

## Testing

- `scripts/nav_target_test.ps1` greps qconsole.log for "CrossBot: <name> reached goal" lines (a short goal can complete within one rcon poll interval)
- Debug visualization: `NavDebug::Draw()` emits `TE_BEAMPOINTS` temp entities; toggled via `crossbot_nav_debug_path` / `crossbot_nav_debug_area`

## Important Notes

- **DO NOT include target areas (info_target) as seed areas in navgen.** The targets are for debugging/validation only. The navgen must discover reachability through normal connectivity (walk, jump, elevator, teleport, launch edges). Using targets as seeds would artificially keep unreachable areas.

- **Coordinate conventions (GoldSrc / Hammer):**
  - X = East, Y = North, Z = Up (right-handed)
  - SVG debug view: Y is flipped (screen Y down), so world Y+ = North = SVG Y-
