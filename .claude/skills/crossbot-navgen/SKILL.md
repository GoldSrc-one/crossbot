---
name: crossbot-navgen
description: Geometry invariants and the debugging playbook for CrossBot's navgen nav-mesh compiler (src/navgen/, navgen.exe). Use this whenever work touches navgen — areas, leaves, floor seams, walk/air-move connections, merging, hull-3 geometry, .nav output — and especially when a nav mesh comes out wrong: phantom or missing areas, a staircase or slope that fragments, connections that should exist and don't, gateless-connection or ill-formed-footprint warnings, or an SVG/OBJ dump that looks off. Also use it when a bot has no route from spawn to target at all, or an air move gets rejected on every route, since both usually mean a missing or badly sampled connection and this skill has the mesh-versus-route triage — hand off to the `crossbot-nav` skill once the mesh is cleared and the question becomes planning or steering. Also use it before changing any navgen pipeline stage, since the stages have unwritten contracts that are easy to break silently. Triggers even when the user just names a test map (test15_staircase, test49_pyrascew, crossfire) or a dump flag (--dump-support, --dump-areas, --no-floor-split, --dump-path).
---

# navgen: geometry and debugging

## Read these first, they are not duplicated here

- `AGENTS.md` — build (`& .\build.bat`, nothing else), house rules, the command lines.
- `doc/testing.md` — what the sweeps and `test_nav_goal.ps1` do, and which sweep a change needs.
- `doc/navgen.md` — what each source file does, stage by stage. The authority on behaviour.

This skill is the part that isn't in either: the geometry you would otherwise re-derive, and how to
find out why a mesh is wrong. When you learn something durable about navgen, put behaviour in
`doc/navgen.md` and invariants or debugging technique here. Never in source comments — they get deleted.

## The hull-3 mental model

Everything navgen sees is the **duck clip hull**, `kDuckClipHull` = mins (-16,-16,-18), maxs (16,16,18)
(`src/navgen/hull_defs.h`). GoldSrc bakes that hull into the clipnode tree by expanding every solid
brush outward by it, so:

> **A hull-3 leaf is world empty space inset by 16 in X, 16 in Y, and 18 in Z, on every side.**

Check it on `maps/test1_simple.bsp`: the room is 0..512 in X with floor z=0 and ceiling z=128; the
leaves come out x 16..496, z 18..110. If a coordinate ever looks 16 or 18 off from the `.map`, this is
why — it isn't a bug.

**That rule is exact only for axis-aligned faces.** A tilted face is pushed out along its normal by

```
offset = 16*nx² + 16*ny² + 18*nz²
```

which is the displacement vector `(16*nx, 16*ny, 18*nz)` projected back onto `n` — *not* the box
support function `16|nx| + 16|ny| + 18|nz|` you would get from a true Minkowski sum. The difference is
large and always in the same direction: for a 45° slope the real offset is **17.0**, the support
function says 24.04. Measured at three angles against the brush planes in the `.map` —
test51_pyraduck's 45° face 17.00, test45_slopes2's 2:1 ramp 17.60, test49_pyrascew's two 1.25:1 faces
17.22 — every one matches the formula to three decimals while the support function is 5.7 to 7.0 out.
The offset can never leave the 16..18 band, and it collapses to plain 16 for any vertical plane and 18
for a floor or ceiling, which is why the axial rule reads as if it were general.

The practical upshot: **do not hand-derive where a slope's expanded surface sits.** Read the plane off
`--dump-planes <id>` and solve from that. Deriving it from the inset rule puts the surface ~7 units off
on a 45° face, which is more than enough to land a threshold in the wrong place. (This is also the
familiar GoldSrc bug where players clip into slopes: the expanded solid is *smaller* than the hull
sweeping the brush would be.)

Two consequences worth internalising:

**Leaf space is the space of the duck hull's *origin*, not of the player's body.** A leaf floor at
z=18 means the origin rests 18 above ground. Areas do not describe headroom; standing is decided
separately by `SplitLeavesByStandClearance` probing hull 1, and arrives at the merger as
`isCrouchLeaf`. Do not reason about whether a bot fits from a leaf's z-extent.

When you do need to predict that boundary to check a crouch classification, it falls out of the probe:
the stand hull is tested at `floor + kStandProbeLift` (18) and reaches `kHumanHullMaxZ` (36) above
that, so

> **a leaf floor is crouch-only where `floorZ > leafCeilingZ - 36`**, both in leaf space.

test51_pyraduck's bands start at 105.9 under a 142 ceiling; test45_slopes2's ramp goes crouch at -62.1
under a -26 ceiling. Combined with the tilted-plane offset above, that is enough to predict exactly
where a slope should change classification — which is how you tell a real misclassification from a
surprising but correct one.

**Everything is generated in hull 3, hull 1 is only a predicate.** Air moves, floors, seams — all
hull-3. Hull 1 answers one question: can you stand here.

## What expansion does to real geometry

These are the facts that decide how stairs, slopes and ledges come out. They follow from the *axial*
inset above; they are not empirical accidents. Anything involving a tilted face needs the offset
formula instead.

**Floor edges meet exactly at every vertical drop.** A block's top face expands outward laterally by
16, so the upper walkable surface extends 16 *past* the block's side. The same block's side wall
expands by 16, pushing the lower floor 16 back from it. Both land on the identical XY line. This holds
for an 8-unit step and a 200-unit cliff alike. It is why floor seams exist at all — and why a seam on
its own tells you nothing about whether a bot can cross. `FilterWalkConnectionsByStep` drops 199 and
trims 4 of test15_staircase's 242 candidate seams; those are drops, not bugs. (Treat that count as a
rough gauge, not a fixture — it moves whenever an upstream stage changes. Re-read it from the run.)

**A wall can never sit on a seam.** A wall of thickness *t* with floor on both sides erodes each floor
16 back from its own face, so the two floor regions end up *t*+32 apart in XY and cannot share an edge.
So if two walkable floors do share an edge, whatever separates them is one-sided — a step or a drop —
and its height is the only thing that matters. That is exactly what `WalkableSpansAcross` measures.

**An expanded staircase is still a staircase.** Expansion does not ramp it. Treads keep their depth
and risers their height; the profile just shifts. So the air above a staircase arrives as a stack of
step-shaped leaves, and the clipnode plane capping one tread runs straight over the next one — which
is why columns above treads routinely have a bottom face floating one riser above the actual ground.
That is normal input, not corruption, and `AbsorbFloorlessPieces` is what reunites them.

**Slopes and stairs often become non-convex prism areas.** An `Area` with `prism = true` has a
multi-level floor; its bound planes are the walkable-footprint outline plus a z-slab, not its own face
planes. Prism is the *fallback*: the merger tries a 3D convex merge first and only extrudes a footprint
when that fails, so whether a given ramp lands on the prism path depends on how the BSP happened to
split it. Do not read the presence or absence of `prism` as a defect on its own — the same ramp can
merge convexly on one map and as a prism on another.

**A prism being non-convex is never the bug — overclaiming is.** The invariant is that the exported
footprint hull covers *only* space the area's own floors cover, to within 1 unit². So when a merge looks
wrong, do not measure convexity; measure the gap. Read the leaves' walkable footprints out of
`--max-merge-group 1 --dump-planes <id>` (floor faces are the ones with `normal.z > 0.7`), take their
convex hull, and compare its area against the sum of the parts. A hull edge that no leaf face justifies —
typically a chamfer cutting a corner off — is the tell. That is exactly how test49_pyrascew's apex column
was found being swallowed by 32.56 units², which is half of its 8.07×8.07 cross-section. `doc/navgen.md`
has the coverage rule and why the tolerance is absolute rather than a fraction of the footprint.

**Merged areas legitimately delete connections.** A merge turns inter-area connections into intra-area
space, so `walk` and `air` counts falling alongside `areas` is the healthy pattern, not a loss. Judge a
merge change by `nogate` / `illformed` / `noroute` staying flat and by `walk` not converting into `air`
between areas that did *not* merge. Beware reading a per-area diff across two builds by id:
`PruneUnreachableAreas` renumbers, so a regrouping shifts every id after it and makes an ordinary diff
look like carnage.

## Conventions that are easy to get backwards

**Plane normals point inward.** A region is `{p : p·n >= d}`. A leaf's *floor* has `normal.z > 0`
(`kFloorNormalZ` = 0.7); its *ceiling* has `normal.z < 0`. Getting this backwards produces code that
silently does nothing, which is hard to spot — verify direction with `--dump-planes <id>` before
trusting a sign.

**`Area::polytope` and `Area::boundPlanes` are not interchangeable.** Any "is this point in this area"
test reads `boundPlanes`. A prism's `polytope.faces` include interior risers whose halfspace
intersection is *empty*, so testing against them answers "no area" everywhere. `doc/navgen.md` records
that this bug ate connectivity twice.

**A face's `walkable` flag is load-bearing.** `IsFloorFace` gates on it, and footprints, seams and
air-move launch edges all flow from `IsFloorFace`. A floor face that is geometrically fine but
unsupported is marked `walkable = false` and must stay that way.

## First: is it the mesh, or the route?

"The bot never reaches the target" reads exactly like a connectivity bug and usually is not one. Establish
which half is broken before opening the ladder below. One command settles it:

```powershell
.\build\tools\navgen\navgen.exe maps\<map>.bsp --dump-path
```

`no route from area A to area B` is a mesh problem — the ladder below is the right tool. Anything else
means the planner found a route, and a bot that still fails is failing to *follow* it, which is
`path_follower.cpp` / `bot_move_nav.cpp` territory, not navgen's.

There is no longer a third case where the route exists but the plan gets truncated. `NavPlan::PlanRoute`
string-pulls and simulates air moves *inside* the A\* search, so an air move it cannot steer across never
enters the route — the edge simply does not relax and the search continues. The `debug path:` line counts
those as `rejected in search`, and `--dump-airmove` prints each one with its launch, yaw, speed and where
it actually landed. **A `sim miss` is not a failure.** It is the planner declining an edge, and its count
should equal `rejected in search`.

What it *is* evidence of: a connection rejected on every route is an air move `ProbeAirMoves` should not
have exported. The stored launch comes from sampling a whole edge, while the plan launches from wherever
the funnel crosses that edge — usually a gate corner. That is a navgen bug even though it surfaces in the
planner, and it belongs in `ProbeAirMoves`.

Route behaviour lives in `doc/nav.md`, not `doc/navgen.md`, and the `crossbot-nav` skill carries the rest
of this triage — how to read `crossbot_nav_status`, how to tell a steering failure from a planning one,
and why `--dump-path` is not the route the bot actually takes. Switch to it as soon as the verdict above
comes back "not the mesh". Sending a planner bug through the mesh ladder below costs an hour and finds
nothing — the leaves, areas, seams and step filter will all be innocent.

## Diagnosing a bad mesh

Work from the widest lens inward. Each step tells you which *stage* is at fault, which is far more
valuable than staring at the final output.

1. **Reproduce on the smallest map that shows it.** `maps/` has 52 maps built for exactly this;
   prefer a `testNN_*` over crossfire. If none isolates the defect, say so — the user builds new test
   maps readily, and a minimal repro is usually worth asking for.

2. **Is it there before the floor split?** `--no-floor-split --dump-leaves` shows the raw clipnode
   leaves. If the geometry is already wrong here, the problem is in extraction or the BSP, not in any
   later stage. This one command has repeatedly turned "mysterious phantom area" into "oh, that's just
   the leaf".

3. **Is it the merge?** `--max-merge-group 1` gives one area per leaf. If the defect survives, merging is
   innocent. Raising the cap (`2`, `3`, ...) then bisects *how much* merging it takes to reproduce, which
   is usually faster than reading the growth loop. `--no-prune` keeps unreachable areas.

4. **Which fragments got thrown away?** `--dump-support` prints every leaf fragment dropped as
   unsupported, with floor centroid, normal and volume. Deleted volume is a common root cause of
   "these two areas won't merge" — a stage further down lost the faces or floors it needed.

5. **Which connections got refused, and why?** `--dump-steps` prints each walk connection the step
   filter cut or dropped: the area pair, the shallowest crossing, both floor heights, and each floor
   face's normal z plus its 2D offset from the query point. A nonzero offset means `FloorFaceUnder`
   fell back to its nearest face rather than finding a containing one — usually the real story.

6. **Look at it.** `--dump-svg <path>` for the top-down graph, `--dump-obj <path>` for the 3D floors,
   `--dump-planes <id>` for one area's exact bound planes and face windings.

Run `navgen.exe` with no arguments for the current flag list, and prefer it over a list from memory or
from `doc/navgen.md`. Keep it honest: the usage text has itself gone stale before — four working
`--dump-*` flags went unlisted for a while, so "not in `--help`" briefly meant "does not exist". If you
add a flag, add its usage line in the same edit; if a flag you expect is missing, grep the parser in
`main.cpp` before concluding it is gone.

## Traps that have cost real time

**`--dump-path` labels its last waypoint `goal` whichever waypoint that is.** The label is positional, not
semantic, so a route that gave up 400 units short still prints a confident `goal` line — and the plan looks
finished. Always check that waypoint's coordinates against the `info_target` origin in the `.map` rather
than trusting the word. This is what hid the truncation on test45_slopes2, test28_slopecase and
test15_staircase back when the planner could still truncate; the coordinate check is what caught it, and
it is still the only thing that proves a route arrived.

**Never head-limit a `--dump-conns` or `--dump-steps` dump.** Each connection prints a header line plus two
gate lines, so `Select-Object -First 60` shows about twenty of them — and the one you are looking for is
usually below the cut. Truncating that dump made a present-and-healthy walk connection look missing and sent
this session hand-deriving ramp geometry for a seam that was fine. Filter to the header lines instead, which
is both complete and shorter:

```powershell
.\build\tools\navgen\navgen.exe maps\<map>.bsp --dump-conns 2>&1 | Select-String '^  (walk|air|teleport)'
```

**Area ids are renumbered by `PruneUnreachableAreas`.** `--dump-areas` and the SVG show *post*-prune
ids; leaf dumps and anything earlier use pre-prune numbering. The same visual area has two different
ids depending on where you looked. When the user names an id, establish which side of the prune they
mean before analysing anything — a whole diagnosis can land on the wrong object. `--no-prune` makes
them agree.

**One leaf routinely has several floor faces, and picking one is a bug.** Hull-3 expansion leaves the
flat ground at a ramp's foot and the ramp itself inside a *single convex* leaf — the space above both
is just an intersection of two halfspaces, so the BSP has no reason to split them. `LowestFloorFace`
then returns the flat ground, which has all the headroom and all the support in the world, and any
stage that classifies off it silently passes the ramp. This has now shipped twice: once in
`SplitLeavesByFloorSupport`, once in `SplitLeavesByStandClearance`, where it cost test51_pyraduck the
crouch cap on one pyramid face while the other three came out right purely because the BSP happened to
split their slopes away from the floor. `LowestFloorFace` is for asking "how low is this leaf", never
for "what is this leaf's floor" — for that, iterate `IsFloorFace` over every face. The symptom to
watch for is an asymmetric result on a symmetric map: identical geometry classified differently is
almost always one stage looking at one face.

**`ClipPolytope` drops `Face::walkable` unless you pass it through**, and it appends its cross-section
face *unconditionally* — so clipping by a plane the polytope already has a face on leaves two
coincident faces and a `Polytope::Volume()` that is quietly wrong. When clipping against another
polytope's planes, skip the ones it is already fully inside. A volume-equality check is the cheapest
way to catch this class of bug, and it is worth adding to any new geometry predicate.

**Stage contracts are unwritten and easy to break.** Before changing a stage, check what later stages
assume about its output: whether every leaf still has a walkable floor, whether ids are still indices,
whether faces are still shared exactly between neighbours. Splitting or deleting geometry to fix one
symptom regularly breaks a downstream stage that depended on it.

**Debug output is a `--dump-*` flag, never an environment variable** — so a run is reproducible from
its command line. This is a house rule, and it also just makes the tool usable.

## Sweep before and after any pipeline change

Never judge a navgen change on one map. This skill's own `scripts/sweep.ps1` — bundled here, *not* in
the repo's `scripts/` directory, so use the full path below — runs every map in `maps/` and records
leaf and area counts, walk and air-move connection counts, and warning counts:

```powershell
& .\.claude\skills\crossbot-navgen\scripts\sweep.ps1 -Out before.txt
# make the change, rebuild
& .\.claude\skills\crossbot-navgen\scripts\sweep.ps1 -Out after.txt
Compare-Object (Get-Content before.txt) (Get-Content after.txt) -SyncWindow 60
```

Read the diff as a whole. Fewer areas with connections preserved is usually a win; connections turning
from walk into air is usually a loss. Treat `nogate`, `illformed` and `noroute` as regressions
wherever they appear on a map that was previously clean.

Improving one map while quietly wrecking five others is the normal failure mode here, and the sweep is
the only thing that catches it.

**The sweep measures the mesh, not the route.** `sweep.ps1` runs navgen bare, and `BuildDebugPath` only
runs when a `--dump-path` / `--dump-obj` / `--dump-svg` / `--dump-airmove` flag asks for it — so a change
under `src/nav/` (the search, the funnel, `PlanWalker`, the profile) produces an *empty* sweep diff no
matter what it did to routing. Sweep those over the plan instead:

```powershell
Get-ChildItem maps\*.bsp | ForEach-Object {
    "$($_.Name): " + ((& .\build\tools\navgen\navgen.exe $_.FullName --dump-path 2>&1 |
        Select-String '^debug path: area|^debug search:') -join ' | ')
} | Set-Content before_plan.txt
```

Read it for waypoint counts collapsing relative to connection counts, `rejected in search` rising, any map
flipping to `no route`, and — on the `debug search:` line — expansions or simulations climbing, which is
the search doing more work for the same answer. Both sweeps are needed for a change that touches a
pipeline stage *and* the shared planner.

**The summary line hides coordinate changes.** Counts can match exactly while every waypoint moved, so a
change that reroutes without changing the shape of the plan looks like a no-op. Dump `'^  wp'` per map and
diff that too — it is what proved the `PlanWalker` extraction was behaviour-preserving across all 52 maps.
