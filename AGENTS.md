# CrossBot

CrossBot is a Metamod plugin for GoldSrc (Half-Life 1) that adds navigation-mesh-driven bots. The runtime plugin (`crossbot_mm.dll`) and an offline nav-graph compiler (`navgen.exe`) share the planning/steering code verbatim via `src/nav/`.

## Code Style

- **Main source: Use HLSDK style C++, use std if necessary**
- **Tools and scripts: Use PowerShell! Don't use CMD! Use Powershell! Don't use Python! Use powershell! Don't use Node.JS!**
- **Comments: Do not add comments. All comments will be deleted! If you cannot escape you irresistable urge to write comments, put them in doc/ instead.**

## Important Notes

- **DO NOT include target areas (info_target) as seed areas in navgen.** The targets are for debugging/validation only. The navgen must discover reachability through normal connectivity (walk, jump, elevator, teleport, launch edges). Using targets as seeds would artificially keep unreachable areas.

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

`build.bat` calls vcvarsall and runs `cmake --build build`, which builds **both** targets. Build
them together — the plugin once sat broken for several commits because only `navgen` was built.

It then copies `crossbot_mm.dll` into `hlds\valve\addons\crossbot\`, so `test_nav_goal.ps1` always
runs the DLL that was just built rather than whatever was deployed last. The copy is skipped when
`hlds\` is absent, and fails the build if the file is locked — that means a server is still up.

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
Areas are flat grey; `NAV_AREA_CROUCH` gets a light blue tint, `NAV_AREA_DOOR` gold and `NAV_AREA_BREAKABLE` terracotta. Connections are white (walk), dashed red (air move) and green (teleport).

Add `--trace <file>` to overlay a recorded bot trajectory — one `x y z` per line, exactly what the
plugin's `crossbot_trace_path` writes — as a cyan polyline under the orange planned route. Where
the bot tracked the plan the orange covers the cyan; any excursion shows as bare cyan. An empty or
missing file draws nothing rather than failing, which is what `test_nav_goal.ps1` relies on when a
run produces no trace.

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

The slab test runs against *leaves*, never groups, so leaves never move and an earlier prism cannot be
invalidated by a later merge. When it fails, the merge **absorbs** the conflicting group if a candidate portal
reaches it, then retries: a ramp only becomes conflict-free as a whole, so every pairwise step along one trips
over the neighbour the finished area would have swallowed anyway.

Anything in navgen that asks "is this point in this area" must use `boundPlanes`, never `polytope.faces`: for a
prism the faces include interior risers, and their halfspace intersection is empty.

`--dump-planes <id>` prints the exported bound planes first, then the true face windings.

### Doors and breakables

`func_wall` and `func_detail` are permanently solid, so they are **carved** out of the world leaves. A
`func_door`, `func_door_rotating` or `func_breakable` is not: it **splits** the leaf it sits in and the interior
piece survives as its own area, tagged `NAV_AREA_DOOR` or `NAV_AREA_BREAKABLE`. Both halves come out of the same
clip planes, so their shared faces are exactly coplanar and `BuildPortals` connects them with no extra linking —
the same property prism merging relies on. A merge never crosses `ExtractedLeaf::passableIndex`, so a door keeps
its own area and never absorbs a different one.

How the bot gets through is decided from the entity, not at runtime:

- **No `targetname` and not trigger-only** → `DOOR_ACTIVATION_TOUCH`: walk straight through.
- **Named** → the activator is resolved through `func_button` (chasing `trigger_relay` and `multi_manager`) or
  `trigger_multiple`, giving `DOOR_ACTIVATION_BUTTON` / `_TRIGGER` and a `doorActivatorPos` at the activator's
  model centre. The bot detours there first.
- **Named with no activator navgen can reach** → treated as a wall and carved, so whatever is behind it prunes
  out. That is deliberate: pruning then reports honestly that there is no way through.
- `SF_DOOR_START_OPEN` and `SF_DOOR_PASSABLE` doors never occupy their brush space at all, so they are neither
  carved nor split.

Which passes see a door as solid:

- **Floor support** does — a door brush still holds up whatever rests on it.
- **The leaf carve, the stand-clearance test and air-move sampling** do not. The bot opens or breaks a door, so
  a jump may pass straight through one; leaving doors in would also classify every door interior as crouch-only.

In the SVG a door area is gold and a breakable area terracotta, taking precedence over the crouch tint so a
crouch-height vent still reads as breakable.

`--dump-areas` prints the flag, activation, activator position and time, e.g.
`door(button at (4.0, 112.0, 48.0) 0.94s)`.

### Path cost is seconds

`NavConnectionRecord::costSeconds` is the time an edge costs **beyond running its length**, and
`NavAStar::FindPath` computes `distance / kNavRunSpeed + costSeconds`, with the heuristic in the same units. It
is not a distance multiplier. A door contributes its opening time (`travel / speed` for a slider,
`distance / speed` for a rotator, from the real SDK formulas), a breakable `health * 0.1` as a guess at how long
the bot spends shooting it, and an air move its flight time plus a fall-damage penalty. Button and trigger doors
need no extra penalty — the detour to the activator already costs its own travel time.

### Debug output is always a command-line flag

Never gate diagnostics behind environment variables. Every debug dump is a `--dump-*` flag on
navgen, so a run is reproducible from its command line alone and shows up in `--help`.

- `--dump-steps` — every walk connection the step filter drops. The filter samples the portal span
  and keeps the connection if *any* point on it is walkable, so what is printed is the shallowest
  crossing found: area pair, where it is, both floor heights, the step, and each chosen floor
  face's normal z and 2D offset from the query point. A nonzero offset means `FloorFaceUnder` fell
  back to its nearest face instead of finding a containing one.
- `--dump-support` — every leaf fragment dropped as unsupported: floor centroid, floor normal,
  total supported area over all of the fragment's floor faces, fragment volume.
- `--dump-portals` — raw leaf-to-leaf portals with midpoints, before merging.
- `--dump-path` — the waypoint plan: each waypoint's kind, position, area, flags and speed.
- `--dump-airmove` — air-move launch derivation: why no funnel launch was found, and every
  re-simulation miss with its launch, yaw, speed and where it actually landed.

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


## Testing

- `scripts/nav_target_test.ps1` greps qconsole.log for "CrossBot: <name> reached goal" lines (a short goal can complete within one rcon poll interval)
- Debug visualization: `NavDebug::Draw()` emits `TE_BEAMPOINTS` temp entities; toggled via `crossbot_nav_debug_path` / `crossbot_nav_debug_area`
