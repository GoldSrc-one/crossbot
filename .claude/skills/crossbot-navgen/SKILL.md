---
name: crossbot-navgen
description: Geometry invariants & debugging playbook for navgen (src/navgen/, navgen.exe). Use when nav mesh wrong: phantom/missing areas, fragmented stairs/slopes, missing connections, gateless/ill-formed warnings, bad SVG/OBJ. Also when bot has no route or air moves rejected — hand off to crossbot-nav once mesh clear. Trigger on test map names or dump flags.
---

# navgen: geometry & debugging

## Read first (not duplicated here)
- AGENTS.md — build (& .\build.bat), house rules, commands
- doc/testing.md — sweeps & test_nav_goal.ps1
- doc/navgen.md — stage-by-stage behaviour authority

This skill: geometry invariants & debugging technique. Behaviour → doc/navgen.md, invariants here. No source comments.

## Core invariants (see doc/navgen.md for derivation)
- Hull-3 leaves = world space inset 16 X, 16 Y, 18 Z (exact only axis-aligned)
- Tilted face offset = 16*nx² + 16*ny² + 18*nz² — read --dump-planes, dont derive
- Leaf space = duck hull origin space; headroom via hull-1 probe separately
- All gen in hull 3; hull 1 = can stand predicate only
- Floor edges meet exactly at every vertical drop (expansion symmetry)
- Walls never on seams — shared edge = one-sided step/drop only
- Prism areas = non-convex fallback; overclaiming is the bug (coverage ≤1 unit²)
- Merge deletes connections — walk/air falling with areas = healthy
- Plane normals inward (p·n ≥ d); floor normal.z > 0.7
- boundPlanes for point-in-area, not polytope (prism polytope empty intersection)
- Face walkable flag load-bearing — gates footprints, seams, air-move edges

## Mesh vs route triage
.\build\tools\navgen\navgen.exe maps\<map>.bsp --dump-path
- no route from area A to B → mesh problem (this skill)
- Otherwise → route exists, bot fails to follow → crossbot-nav skill
- sim miss = planner declining edge (not failure); count = rejected in search

## Debugging ladder (widest lens inward)
1. Reproduce on smallest testNN_* map
2. --no-floor-split --dump-leaves — raw clipnode leaves
3. --max-merge-group 1 — one area/leaf; defect survives = merge innocent
4. --dump-support — unsupported fragments dropped
5. --dump-steps — walk connections cut/dropped with diagnostics
6. --dump-svg / --dump-obj / --dump-planes <id> — visual inspection

## Key traps
- --dump-path labels last waypoint goal positionally — verify coords vs info_target
- Never head-limit --dump-conns|steps — filter headers: Select-String ^  (walk|air|teleport)
- Area IDs renumbered by PruneUnreachableAreas — --no-prune makes them agree
- Single leaf = multiple floor faces — iterate IsFloorFace, never LowestFloorFace
- ClipPolytope drops walkable & adds cross-section face unconditionally
- Stage contracts unwritten — check downstream assumptions before changing
- Debug output = --dump-* flag only (house rule)

## Sweeps (always before/after pipeline changes)
& .\claude\skills\crossbot-navgen\scripts\sweep.ps1 -Out before.txt; # change, rebuild; & .\claude\skills\crossbot-navgen\scripts\sweep.ps1 -Out after.txt; Compare-Object (Get-Content before.txt) (Get-Content after.txt) -SyncWindow 60
Read whole diff: fewer areas + preserved connections = win; walk→air = loss; nogate/illformed/noroute = regression.
Mesh sweep empty for src/nav/ changes — use plan sweep: Get-ChildItem maps\*.bsp | ForEach-Object { "$($_.Name): " + ((& .\build\tools\navgen\navgen.exe $_.FullName --dump-path 2>&1 | Select-String ^debug path: area|^debug search:) -join " | ") } | Set-Content before_plan.txt; Also diff ^  wp per map — catches coordinate changes summary lines hide.
