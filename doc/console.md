# Console

`commands.cpp` registers every server command in `Register()`, and `dll_hooks.cpp` registers the cvars
(all `FCVAR_SERVER`). Commands are thin wrappers: the work lives in `BotManager` or `Nav`, and each one is
explained in the doc of the system it reports on. This page is the index and says how to read each dump.

## Output

Everything prints through `SERVER_PRINT`, so it lands on the HLDS console and in `D:\hlds\qconsole.log`,
**not** on the stdout of a script that sent the command over RCON.

**No console line may reach 1024 characters.** The dedicated server converts each line to wide characters
in `CTextConsoleWin32::PrintRaw`, which holds a `WCHAR unicodeStr[1024]` and guards it with
`if (nSize > sizeof(unicodeStr))`: a count of *characters* against a count of *bytes*. A line of 1024
characters or more passes the guard and overruns the buffer by up to 2 KB. hlds does not crash; it stops
using CPU behind an invisible `Run-Time Check Failure #2 - Stack around the variable 'outputStr' was
corrupted` box and looks hung until killed. That code is CrossPlay's, so the limit is ours to respect:

- Every fixed print buffer in the tree is 1024 bytes or smaller.
- A dump built in a `std::string` (`PrintNavStatus`, whose `waypoints=[...]` grows with the route) goes
  out through `PrintLong`, which splits at a comma or space below 1000 characters. A long strategic route
  on `dod_flash` froze the server on `crossbot_nav_status` without it. Any new dump that concatenates
  per-waypoint or per-area text goes through `PrintLong` too.

## Bots and navigation

- `crossbot_add [gamedir|*] [name] [model|team]` — add a bot to a loaded game.
  - The first argument is always the game directory. `*` or nothing picks a random playable one, and an
    unknown name lists what is loaded. A game is playable only with both gamedata and a game profile
    (`doc/games.md`).
  - With no name, one is picked at random from `addons/crossbot/names.txt` (deployed by `build.bat`), with
    `crossbot_name_prefix` in front.
  - The third argument belongs to the game's profile: a player model in deathmatch; a class for `tfc`
    (`scout` .. `engineer`, `civilian`, `randompc`); a team for `cstrike`/`czero` (`t`, `ct`, `*` auto); a
    class for `dod` (`garand` .. `piat` Allies, `k98` .. `pschreck` Axis, `random`), which also picks the
    team. `*` or nothing lets the profile choose: a random installed model in deathmatch, `randompc` in
    TFC (a new class every spawn).
  - A model the scan did not find is a warning, not a refusal: it prints the installed list and uses the
    name anyway, because models inside a `.pak` are invisible to the scan. Where the list comes from and
    why CrossPlay may clamp the result: `doc/games.md`, `player_models.cpp`.
  - The standalone build has one game, so it drops the first argument: `crossbot_add [name] [model|team]`
    (`doc/games.md`, "Standalone build").
- `crossbot_nav_goal <x> <y> <z>` — send all bots to a point.
- `crossbot_revenge_probe <x> <y> <z> <yaw>` — run the revenge sightline scan (`doc/path_planning.md`,
  "Revenge") from that eye and facing against the loaded mesh, printing walk connections, traces and
  blocked count with the build time. Deterministic and combat-free, for tuning `crossbot_revenge_los_radius`
  and `crossbot_revenge_fov` on a big map.
- `crossbot_teleport <x> <y> <z> [bot]` — teleport all bots, or the named one.
- `crossbot_kill` — suicide every bot; the only way to exercise the respawn path from the console.
- `crossbot_nav_status` — the runtime planner's own answer, and how to read it: `doc/manager.md`,
  `PrintNavStatus`.
- `crossbot_waypoints` — the full waypoint list per bot.
- `crossbot_nav_penalties` — connections with a nonzero `runtimePenalty`, and the total. The only view of
  what the bots have learned on this map (`doc/path_planning.md`, Runtime connection penalties):
  everything else a repath rebuilds, but the penalty table dies with the map.
- `crossbot_airmove_cache_stats` — the airmove cache's occupancy, hit rate, inserts, evictions and, under
  `crossbot_airmove_verify 1`, verified and `MISMATCH` counts (`doc/testing.md`, "Airmove cache:
  correctness and tuning").

## Map inventories

- `crossbot_list_spawns` / `crossbot_list_targets` — entity positions, for picking a goal by hand. Spawns
  cover every game's spawn classname (`doc/manager.md`, Death and respawn).
- `crossbot_list_items` — every pickup and charger with its kind, resolved nav area, `frame`, `solid` and
  state (`doc/goals.md`, `bot_items.cpp`). **Run it first on any map where bots seem to ignore
  something:** an item whose area reads `none` is invisible to the gather goals, which is a mesh gap, not a
  bot bug.
- `crossbot_list_fun` — the fun tier's whole inventory: kind (`switch`/`tank`/`tram`), action
  (`use`/`touch`/`shoot`), brush centre, every resolved nav area, the chain terminal it fires, and why it
  was skipped (`no nav area`, `taken or locked`, `not ready`) or `ready`. Ends with a count. See
  `doc/goals.md`, `bot_fun.cpp`.
  - It is the first thing to run when bots ignore a map's gimmick. An entity **absent** was never
    detected (its chain does not reach an interesting terminal, or dangles); one listed as `no nav area`
    was detected and is unreachable.
  - `ready` means detected, interesting, and resolved to a nav area. It does **not** mean routable:
    reachability is priced per bot in `BotGoal::Select`. Listed `ready` but never in a `wants fun` line
    is a routing gap, not a detection gap.
  - Negative controls: `datacore` must print `0 fun entities`; on `crossfire` the nuke trigger reads
    `no nav area` until `strike_ready_door` opens at t=180, and then `ready` while still pricing as
    unreachable.
  - A `tram` row also prints the tram's `speed` and its computed `deck` point. With `crossbot_fun_debug 1`
    each board point follows as a `tram board area` line: stand position, deck position as the tram
    passes that node, and arc length along the track. Board points come from the track, so the list is
    identical whether the tram is parked or doing 400; if it changes as the tram moves, the deck is being
    read from `pev->origin` instead of `DeckAt`. A board point whose stand z is far from the deck z is a
    walkway the bot cannot get down from.
- `crossbot_list_goals` — every `bot_goal` entity with its nav area, owning player (`everyone` for a
  map-placed one), `aiment` and the buttons it asks for. Ends with a count. Zero on a `valve` map is
  correct: the entity exists only where `bot_goal.sma` runs (`cstrike`, `tfc`, `dod`). With
  `crossbot_objectives 1` on a map the objectives module reads, bots ignore every owned entity listed here
  (`doc/goals.md`, `bot_strategic.cpp`).
- `crossbot_list_objectives` — the map's objective tables. First the status line: which reader, or why
  there is none (`off for 'tfc' -- no offset for ...` names the file). Then one line per site, payload
  and carrier with team masks, need id / group and player count, the DoD object group names, one line per
  live player with `pev->team` (and `m_iTeam` on a cstrike map), and every bot's intents with kind and
  rank. Works with no bots. **On a cstrike map read the two team columns first:** they are inverted, and
  the reader keys on `pev->team`. See `doc/goals.md`, `objectives.cpp`.
- `crossbot_list_gates` — every walk gate that could hold a mine: mesh width, *measured* opening, chosen
  surface, mount point, rejection reason. Gates that failed only at the surface or coverage stage are
  printed too, so near misses show. Read the openings before the count: they should be doorway widths
  (96, 128); hundreds of gates across open floor means the opening measurement is not running. See
  `doc/movement.md`, `bot_deploy.cpp`.
- `crossbot_list_barriers` — every barrier in the loaded mesh: index, brush model, `CLOSED`/`open`, how many areas
  it owns, its baked bounds, and the entity it resolved to (classname, edict, targetname) or `gone`. Then the
  total and how many are closed. See `doc/path_planning.md`, "Barriers".
- `crossbot_monsters` — one line per `FL_MONSTER` edict: classname, flags, health, `takedamage`, solid
  type, box centre and the enemy its `m_hEnemy` names; then per bot the distance, whether it is in that
  bot's view cone, and whether a live traceline to the box centre reaches it, with `clarity` and
  `(coverage)` when a see-through model was crossed. The trailing summary prints the resolved
  `mon_hEnemy` offset, which separates "the bot ignores snarks" from "there are no snarks" from "the
  offset is wrong". It refreshes the shared monster list first, so it works with no bots. After the
  monsters it lists every sight occluder (entity, model, solid type, chunk count, render mode and amount,
  world box) and whether the coverage build has finished.
- `crossbot_usermsgs` — every user message the game DLLs registered, by id, name and size, with `<- daze`
  on `ScreenFade` and `Concuss`. Run it before testing flashes or concussions: an id of 0 means nothing
  downstream can fire. Ids come from a `pfnRegUserMsg` hook, so a plugin loaded by `meta load` after the
  game DLLs registered lists the names but marks none (`doc/perception.md`, `bot_daze.cpp`).

## Status dumps

- `crossbot_teams_status` — **run this first on any cs/tfc/dod map.** The map's game
  (`gEntityOwners[0]`), whether that makes it a team game, `crossbot_teams` and the verdict; then per
  live player its owning `game`, `pev->team`, `hp`, `deadflag`, `movetype`, `iuser1` (non-zero is an
  observer), `origin` and the nearest spawn entity as `classname/distance/team` (the spawn's `pev->team`;
  TFC keeps a spawn's team in `team_no`, so it reads 0 there), and, per bot, how that bot reads the
  player (`friend` / `enemy` / `unknown`) and whether it may shoot it (`shoot=1`). Polled right after a
  round restart or a respawn, the spawn column checks which side's spawns each game's players use.
  Everything team-related rests on the mod filling `pev->team` for a fake client another game DLL
  created; all-zero teams means bots behave exactly as on a deathmatch map.
- `crossbot_difficulty_status` — the resolved tier and every row of the difficulty ladder with its
  effective value and whether it came from the tier or a pinned cvar. Every difficulty-derived cvar
  defaults to -1 ("tier"), so a stale `crossbot_aim_delay 0.25` left behind by a test script is otherwise
  invisible and overrides the difficulty for the rest of the session. See `doc/difficulty.md`.
- `crossbot_sense_status` — per bot (`doc/perception.md`, `bot_sense.cpp`):
  - `team`, aim target and bone.
  - `notice=<count>:<name>:<progress>/<need>s` — live notice slots, then the highest-progress one.
  - `light=<Lt>/<Lb>/<vis>` for the last traced entity, and `dark`: frames it was dropped as unlit
    (`doc/perception.md`, "Shadows").
  - `clar` — sight clarity of the last seen target. `cov=<partial>/<blocked>/<traces>` — cumulative
    perception traces that crossed a see-through or see-blocking model, and how many still saw through or
    were stopped ("What the bot sees through").
  - `cands` — players that passed range and the view cone; `keep` — `inc` when the incumbent enemy held,
    `new` for a fresh weighted-random pick, `none` when nothing was visible. A duel reads `inc` nearly
    every frame; a run of `new` means the lock is broken.
  - Live suspicion count with `kinds` by player/monster/other, the `tier` that won the look vote, and
    `fresh`: the freshest entry overall (not within the winning tier, so it can name an entity the vote
    ignored) with source, age and position. `tier` must read `PLAYER` whenever `kinds=p:` is nonzero.
  - `hunt` — live entries that are monsters hunting this bot. They vote at the player tier, so
    `tier=PLAYER` with `kinds=p:0 hunt=1` is correct.
  - The look direction with its authority and how many entries survived the cone refinement, and
    `pushed`: cumulative per-source push counts.
  - `mon` — the monster vision path: `list` (the shared per-frame monster list), `seen` (considered by
    this bot), `mine` (naming it as their enemy), `trc` (tracelines spent), `hit` (landed). `mine:0` with
    `list` nonzero: nothing hunts this bot. `mine` nonzero with `hit:0`: hunted, never in line of sight.
- `crossbot_aim_status` — per bot: sense target, bone sampled, ring occupancy, delay served, hand speed,
  `seen`/`perr` (the bot's delayed belief about its own view, and how far it is from the real view plus
  punch), and `err`, the angle between the real view and the true present-time angles to the target.
  `err` separates "correctly delayed" from "delayed and also wrong". It is measured against the target
  eye while the bot aims at a random hitbox bone, so a couple of degrees is its floor: read its trend
  across delays, not its value (`doc/perception.md`, `bot_aim.cpp`).
- `crossbot_vantage_status [x y z]` — the loaded vantage field, then the blend at each bot's eye or at the
  given point (the round-trip test). Exposure is already multiplied by the blended `visibility`, printed
  beside it (`doc/perception.md`, "crossbot_vantage_status").
- `crossbot_camp_status [x y z]` — the camp spot count and `crossbot_camp`'s effective value, then the
  nearest camp spot to each bot or the point: position, area, facing, score, and stand and crouch score,
  spread and distance (`doc/goals.md`, `bot_camp.cpp`).
- `crossbot_combat_status` — the combat cvars, then per bot: who it fights, for how long, how long unseen,
  whether combat drives movement (`driving=`), the current probe's exposure against the exposure where the
  bot stands, and the duel flags `melee=`/`inarea=` (`doc/testing.md`, `bot_combat.cpp`).
- `crossbot_daze_status` — per bot: seconds of blindness left, the alpha, fade and hold of the last
  accepted flash, `flashes`, the current `conc` byte and how long it has been non-zero, `updates`
  received during this concussion, `concs`, and the `sway` being added to pitch and yaw. Counters are per
  life.
- `crossbot_ability_status` — per bot, the TFC class ability state (`tfc_ability.cpp`): the running `task`
  (`heal`, `repair`, `build`, `stab`, `pipes`, `pad`, `touch`, `disarm`, `detpack`, `none`), the last
  `why`, the class, and counters that survive respawn: `heals`, `repairs`, `builds`, `buildfails`,
  `disguises` (commands sent), `feigns`, `stabs`, `pipes` (bursts started), `dets`, `pads`, `padfails` (a
  pad that would not send), `touches`, `uncovers` (spies unmasked), `disarms` (enemy detpacks defused),
  `detpacks` (set), `calls` (`saveme` sent).
- `crossbot_grenade_status` — per bot, the TFC off-hand grenade state (`bot_grenade.cpp`): `slot` being
  cooked (0 none), `why`, `held` seconds, the last solved `flight`, and counters `primes`, `throws`,
  `forced` (released at 1 s left without a shot, toward the most open direction), `refused` (the game
  never set the primed bit), `scared` (caltrop throws), `heard` (throws at an enemy heard but not seen).
  `slots=type:count,...` is what the profile read last; `sightings`/`rejects` count attack decisions and
  how many failed the range or friend gate; `dist`/`min` are the last and smallest target distance.
  Counters survive respawn.
- `crossbot_deploy_status` — per bot: phase, target connection, measured opening, surface, stance,
  `placed`, `dets`, `aborts`, `selfhits`, `escapes`, memo size and `why`. These tallies are **not** per
  life: `OnRespawn` clears the memo and any placement in flight but keeps the tallies, so a drop is a bug,
  not a death (`doc/movement.md`, `bot_deploy.cpp`).

### crossbot_fire_status

Per bot: the weapon held, whether it is `zoom`ed, the fire mode chosen, why the bot is or is not
shooting, live clip and ammo. See `doc/combat.md`, `bot_fire.cpp`.

- **The aim gate in two numbers:** `miss`/`tol`, the angle from the fire ray to the aim point against the
  tolerance allowed there. `aim` is `target`, `lead`, `homing` or `arc`. For `homing`, `tol` is a fixed
  20 degree cone. **For `arc`, `miss` and `tol` are units, not degrees:** the gap from the simulated
  impact to the target's box, against the allowed splash radius.
- `flight` — solved projectile flight time in ms; zero for hitscan **and for homing**, which solves
  nothing.
- `commit` — what the bot has started and will not walk away from: `none`, `charge` (gauss spinning up),
  `throw` (handgrenade pin pulled, waiting for the engine to let go) or `guide` (rocket in the air), with
  `for` seconds in it, or -1. `capped` counts commits released by their safety valve instead of the
  game's own state; non-zero is a fault, like `charge-timeout` on the gauss.
- Throw intent (`doc/combat.md`, "The throw intent"): `intent=` is the weapon id of a live intent or 0;
  `dropped=` counts intents given up (no longer solvable, or ready but the pin was never pulled);
  `banned=1` means grenades are off until this combat ends.
- Heard enemies (`doc/combat.md`, "Throwing at a heard enemy"): `heard=` counts throws at an enemy heard
  but not seen; `harmveto=` counts heard arcs refused because the blast would hurt the bot as much as the
  enemy or more.
- `wallbangs=` and `gaussjumps=` (`doc/combat.md`, "Wall-banging"); `spreadwaits=` and `bracewaits=`
  (`doc/combat.md`, "The CS spread gate").
- `why` values worth knowing: `primed`, `guiding` (the two commits, seen from the trigger),
  `splash-self` (the direct-fire twin of `arc-self`, refusing a rocket rather than a shell), `arc-harm`,
  `heard` (switching to a throwable for a heard throw), `drawing` (grenade deploy), `throw-blocked` (a
  pulled pin whose throw no longer solves), `throw-dropped` (switching back to a gun), `wallbang`, and
  `sticky` (trigger held through a frame the crosshair is off target, deliberate on a `WPN_Q_STICKY`
  weapon, from either side of the aim-queue boundary).
- **Per-life counters**, zeroed on respawn so a drop is a death: `shots`, `leadshots`, `chargeshots`,
  `arcshots`, `throwshots`, `guides`, `stickyshots`, `uphill`, `selfveto`, `switches`, `reloads`,
  `emptyswitch`, `idleswitch`, `idlereload`, `heard`, `harmveto`, `ambush`, `dropped`.
  - `emptyswitch` — fights the bot left a dry clip in rather than reloading. `idleswitch` —
    out-of-combat rotations. `idlereload` — `IN_RELOAD` presses with no target (several per shotgun
    refill, one per magazine elsewhere).
  - `throwshots` and `chargeshots` are counted from the game DLL's state, because neither weapon moves an
    attack timer when the bot presses. `stickyshots` is the subset of `shots` that happened only because
    the trigger stayed down off target; it never counts a gauss primary, which moves no timer.
    `guides` is one per rocket launched.
  - `uphill` and `selfveto` count rejections, not situations (both predicates run per weapon per mode
    per frame): read them as yes/no.

### Weapons

- `crossbot_weapons [name]` — the inventory: every weapon with its `m_iId`, clip, ammo indices and carried
  counts, both attack timers, `*` on the active one. It ends with the walked weapon mask against
  `pev->weapons`, **the only self-check the offsets have**: read it before believing any other number
  (`doc/combat.md`, `bot_weapons.cpp`). In a game with `plr_iPowerups` (Ricochet) it also prints
  `powerups 0x<m_iPowerups>, +4 = <m_iPowerupDiscs>`: 0 normally, `0x4` and 3 right after
  `crossbot_equip item_powerup_hard`, which checks the hand-filled offset.
- `crossbot_weaponspec [name]` — how each gun is operated. With no argument, per loaded game every
  game-file value (slots, melee, ambush, deploy, respawn, fall, throw, radio; preferred items for the
  HLDM-class games), then the whole table: per weapon id the classname, clip size, `value`, range class
  and scope FOV, per firing mode its button pattern, aim requirement, toggle, ammo pool and ballistics,
  and its refill entities. The range class bends routing toward ground
  the gun can use (`doc/goals.md`, "Range bias"); the scope FOV decides whether a camp spot is worth
  zooming into. With a player name it joins that player's weapons against their specs and flags any
  classname that disagrees, the one way the id-keyed table can be wrong (`doc/games.md`, `weapon_spec.h`).
- `crossbot_weapon_carry [name]` — per bot: the weapon held, then `classname=carry/score` for every weapon
  it has held, and `carried`, how many. Carry is seconds in the inventory while alive; score is
  `WeaponSpec::value`. That is all of selection: highest score wins, lowest carry breaks the tie. Read the
  *spread*, not the count (every listed weapon has carry > 0 by construction): all-equal times mean every
  slot is being charged instead of the carried ones, and a held weapon that is neither top score nor least
  carried at that score means the ranking is not applied. Survives respawn; reports bots only
  (`doc/combat.md`, `bot_fire.cpp`).

## Test tools

- `crossbot_equip <classname> [name]` — spawn an item at the target's feet and touch them with it, as the
  game's own `give` does. Any classname, ammo boxes included; default target is every bot. It reaches
  weapon states that gathering would take a whole match to produce.
- `crossbot_spawn <classname> [name] [distance]` — spawn a loose entity in front of the target, traced out
  to `distance` (default 96) so it lands short of a wall. Default target is every bot. `test_monster.ps1`
  uses it for live `monster_snark`s: bots rarely throw them, because the snark is only reachable through
  the ambush path and the uphill veto refuses the throw almost everywhere on crossfire.
- `crossbot_fire_pin <classname|off> [primary|secondary] [zoom|nozoom|any]` — restrict weapon selection to
  one classname, optionally one mode. With no argument it reports all three settings.
  - Needed because in a fight a bot re-selects only when its weapon stops working, and out-of-combat
    rotation never lands on the one a test wants. Without the classname the projectile path is
    unreachable from the console; without the mode, the gauss charge.
  - The third argument overrides `ZoomButtons`, which zooms in at 500 units and out at 300. Fights shorter
    than that barely zoom, leaving the 120-damage zoomed crossbow bolt effectively untested; `zoom` and
    `nozoom` force the state, `any` restores the range rule.
  - **The pin also equips.** A bot without the pinned weapon is given it, and one whose pinned mode is out
    of ammo is given ammo for it (`IGameProfile::AmmoEntity`, `doc/games.md`). Only when the mode is
    *unusable*, never on a timer, so reloads and clip depletion stay real and only starvation is removed;
    otherwise an RPG run gets one rocket per bot per life.
- `crossbot_aim_test <bot> <yawDelta> [pitchDelta] [fov]` — command a flick relative to the current view
  and print settle time and peak hand speed; the only check on the 180-in-0.4s derivation. Overrides
  navigation aim for 3 s. Two seconds in it prints the **rest error**, the miss still standing once the
  view has stopped, which must read 0. `fov` writes `pev->fov` first (0 = engine default) so a script can
  sweep it (`doc/perception.md`, `bot_aim.cpp`).
- `crossbot_daze_test <bot|*> flash <alpha> [duration] [hold]` and `crossbot_daze_test <bot|*> conc
  <scale>` — feed a synthetic `ScreenFade` (white, flags 0; duration and hold default to CS's point-blank
  12 and 2.67) or `Concuss` byte straight into the handlers, as if the wire had delivered it. Exercises
  everything downstream of the message hook without a CS or TFC client. `conc 0` clears.
- `crossbot_sight <x1> <y1> <z1> <x2> <y2> <z2>` — one sightline: the plain traceline's fraction and what
  it hit, then `BotSight::Clarity` for the same segment, `covered` when a see-through or see-blocking
  model took part. Flip `crossbot_coverage` around it for an A/B (`doc/perception.md`, "What the bot sees
  through").

## Cvars

A cvar marked **tier** defaults to -1 and takes its value from `crossbot_difficulty`; any value `>= 0`
pins it. `crossbot_difficulty_status` shows which is which.

### Debugging

- `crossbot_verbose` — per-bot narration (`BOT_VLOG`), default 0. Master switch: every `crossbot_*_debug`
  prints through the same macro, so those need it on. Registered only in a `CROSSBOT_VERBOSE` build (the
  default); an OFF build compiles the lines out and has no cvar. `test_server_start.ps1` sets it to 1, and
  `-RequireVerbose` refuses to run without it.
- `crossbot_trace_path` — where `WriteTraceLog` writes the bot trajectory (verbose builds only; empty,
  the default, disables it).
- `crossbot_fun_debug` — per-frame arrival diagnostics at a switch or tank, the tram board-point dump,
  and every bot death with its distance to the tram and its tram phase (the only way to tell bots crushed
  attempting a catch from bots run over while walking).
- `crossbot_strategic_debug` — one line per `bot_goal` per selection: taken, on its cooldown memo, or not
  placeable in a nav area; and one per native objective that could not be placed.
- `crossbot_tactics_debug` — also log a supporter finding no spot ahead of its runner, or a sniper no
  post. Default 0.
- `crossbot_gather_debug` — every gather selection: each item, its area, its cost or skip reason, the
  bot's readiness in the header, `ENEMY` on the enemy row. Default 0; verbose enough to turn back off.
- `crossbot_combat_debug` — one line per probe (shuffle direction, resolved area, exposure there against
  here), and the frame an enemy notices the bot, which ends an ambush.
- `crossbot_camp_debug` — the per-retarget camp route line and the scare transitions. Default 0.
- `crossbot_deploy_debug` — tripmine forensics, default 0. An audit line per mine spawned (beam length
  against measured opening, both jambs, whether the placer stands in its own beam), a residency line (how
  long the bot took to clear the beam against the 2.5 s arming clock), and an attribution line on every
  death. In `KILLED BY ITS OWN MINE`, `onbeam=1` means the bot walked into its own laser (a bug);
  `onbeam=0` means it was in the blast when something else tripped the mine (the game working). See
  `doc/movement.md`, `bot_deploy.cpp`.
- `crossbot_time_debug` — movement-clock diagnostics, default 0. Once a second: the server's frame
  budget, then per bot the msec issued against it, how much the engine actually ran (`SV_RunCmd` silently
  drops commands while the `clockwindow` guard is armed, visible only as `PlayerPostThink` not firing),
  the drift between bot time and real time, and a `chain` line with the whole bot loop in order. Less sim
  time per second than wall clock is the "honey mode" symptom: slow running and slow gravity together
  (`doc/movement.md`, *Bot_BeginFrame and Bot_FrameTime*).
- `crossbot_airmove_verify` — re-simulate every airmove cache hit and compare. Default 0; a correctness
  harness that costs the whole speedup (`doc/testing.md`).
- `crossbot_perf` / `crossbot_perf_spike` — the zone profiler: report window in seconds (default 0, off)
  and the frame time in ms that logs a spike (default 5). **Only in a `-DCROSSBOT_PERF=ON` build**
  (`doc/diagnostics.md`, `profile_hlds.ps1`).
- `crossbot_hitch` / `crossbot_hitch_report` / `crossbot_hitch_stack` — the StartFrame hitch timer (ms
  threshold, default 50), its percentile report (seconds, default 0) and the stall stack sampler (ms,
  default 0). **Only in a `-DCROSSBOT_PERF=ON` build** (`doc/diagnostics.md`).

### A/B switches

Each one turns a whole feature off, so a run can tell "this feature is wrong" from "something under it is".

- `crossbot_nav_avoid` — the obstacle avoidance layer. Default 1.
- `crossbot_beam_jump` — crouch-jumping tripmine beams too low to crawl under. Default 1. With 0 the bot reroutes, then shoots; `test_deploy.ps1 -Phase reroute` runs that way. See `doc/movement.md`, "A low beam is crouch-jumped".
- `crossbot_gather` — item gathering goals. Default 1. `test_nav_goal.ps1` sets 0: a bot that picks a new
  goal a frame after arriving never shows the harness the `hasGoal=0` it polls for.
- `crossbot_fun` — the fun goal tier. Default 1. With it off `BotGoal::Select` is gather-only. **The nav
  sweep sets it to 0** with `crossbot_gather 0`, so route behaviour stays comparable across transcripts.
- `crossbot_strategic` — whether bots follow `bot_goal` entities at all. Default 1. Off, a bot on a
  `cstrike`, `tfc` or `dod` map falls back to fun and gather: "the objective logic is wrong" versus "the
  navigation is wrong".
- `crossbot_objectives` — native objectives. Default 1. On a `cstrike`/`czero`, `dod` or `tfc` map whose
  objective offsets loaded, bots take strategic goals from `objectives.cpp` and ignore the `bot_goal`
  entities the AMXX plugin publishes; 0 restores the plugin's goals. A map with no reader or missing
  offsets behaves the same either way.
- `crossbot_tactics` — camp-spot tactics for `support` and `defend` goals. Default 1; 0 keeps the intents
  but never picks a spot (`doc/goals.md`, `bot_tactics.cpp`).
- `crossbot_snipe` — sniper posts: a bot with a long-range weapon that is not its team's closest to the
  target may hold a camp spot overlooking it, dead ends included. Default 1; also off under
  `crossbot_tactics 0`.
- `crossbot_teams` — friend/foe gating. Default 1, inert unless the map belongs to `cstrike`, `tfc` or
  `dod`. On, a bot never sights, chases, retaliates against or detonates a satchel on anything not a
  confirmed enemy; off, every gate says "attackable". On a `valve` map both settings must be
  indistinguishable (`doc/games.md`, `teams.cpp`).
- `crossbot_sense` — the whole sense system: vision, heard sounds and events, damage, and the suspicion
  list that drives the idle look. Default 1, held there by a 45/45 nav sweep at both settings
  (`doc/perception.md`, `bot_sense.cpp`).
- `crossbot_combat` — the combat state. Default 1. A bot that sees an enemy (or is shot) drops its goal
  and repositions locally until the enemy dies, vanishes, stops being sensed, or goes 5 s unseen. Off, it
  keeps gathering through a firefight but still shoots (aim and trigger are `crossbot_fire`'s). Inert
  without `crossbot_sense`. Not affected by `crossbot_gather 0`: combat is not a goal.
- `crossbot_fire` — whether bots shoot. Default 1. Off also stops weapon switching and the out-of-combat
  reload, the same decision made without a target. Inert without `crossbot_sense`, and in the nav sweep
  either way, since a lone bot never acquires a target (`doc/combat.md`, `bot_fire.cpp`).
- `crossbot_grenade` — whether bots prime and throw off-hand grenades (TFC). Default 1.
- `crossbot_deploy` — whether bots place tripmines and satchels. Default 1. **The nav sweep sets 0**: a
  mine changes every route through the doorway it blocks, and the per-map waypoint diffs need repeatable
  routes. Off, the beam registry stays empty and every deploy branch in `bot_avoid.cpp` early-outs, so the
  sweep clears a deploy change for movement regressions and says nothing about deployment
  (`doc/movement.md`, `bot_deploy.cpp`).
- `crossbot_deploy_surface` — pin the mount surface: 0 off, 1 wall, 2 ceiling, 3 ground. Default 0. A pin
  also bypasses the bound that keeps ceiling and ground mounts to gaps under 48 units. It exists for the
  reason `crossbot_fire_pin` does: openings that narrow are rare enough that six stock maps have none, so
  the vertical beam's side search and standoff shot would otherwise go untested. Not for play.
- `crossbot_coverage` — 1 (default) lets vision see through translucent brushes and be blocked by
  nonsolid foliage and props, by their rasterized coverage; 0 is the plain traceline
  (`doc/perception.md`, "What the bot sees through").
- `crossbot_vantage` — the baked vantage field that steers an idle bot's look. Default 1; 0 invalidates
  the blend without regenerating the nav (`doc/perception.md`).
- `crossbot_careful` — scale on carefulness, the `(1 - readiness)` exposure weight in route planning,
  clamped [0, 1]. Default 1; 0 is pure fastest-route planning (`doc/goals.md`, "Carefulness").
- `crossbot_melee_only` — every bot fights with melee only: no guns, off-hand grenades or deployables.
  Default 0. The brain's per-bot `melee_only` rule applied server-wide through
  `BotDirective::MeleeOnly`. A weapon passes when `IGameProfile::MeleeOnlyAllows` says so: the game's
  melee weapons, plus Ricochet's disc (`doc/games.md`, "Ricochet").

### Difficulty

The ladder and every tier value are in `doc/difficulty.md`.

- `crossbot_difficulty` — the tier, 0-3 (easy/normal/hard/expert), default 1. One global setting,
  modelled on the CS bot's `cv_bot_difficulty`.
- `crossbot_aim_delay` — reaction delay in seconds, clamped [0, 0.5]. **Tier:** a per-engagement roll
  inside the tier's band; `>= 0` pins a fixed value. Inert without `crossbot_sense`.
- `crossbot_notice` — how long a seen enemy takes to notice: `dist * tan(offAxis) * scale / 1000` seconds,
  so 1 means 45 degrees at 1000 units takes 1 s. **Tier** (1.5 / 1 / 0.75 / 0.5); 0 is instant
  (`doc/perception.md`, "Notice delay").
- `crossbot_aim_gain` — spread of the motor gain mismatch rolled per flick, clamped [0, 0.5]. **Tier**
  (0.25 easy down to 0.05 expert). The hand moves `step * gain` with gain in `[1 - s, 1 + s/2]` while the
  efference copy records `step`, so the miss is seen one reaction time later and corrected. Active only
  while a target is held or `crossbot_aim_test` runs. 0 is the exact controller, the baseline
  `scripts/test_aim_error.ps1` measures against (`doc/perception.md`, "The efference copy is wrong on
  purpose").
- `crossbot_sense_decay` — suspicion weight and decay window in seconds, clamped [0.1, 60], default 5.
  Also the blend authority: a suspicion this old has zero pull on the view.
- `crossbot_ambush` — open on an enemy who has not noticed the bot with melee at contact range, otherwise
  a grenade or a snark. **Tier** (on below expert); 0 always opens with the best gun, and the awareness
  verdict is still logged, which makes it the A/B. Inert without `crossbot_combat`. An ambush weapon is
  only *offered*: if none is usable at that range, ordinary selection runs.
- `crossbot_camp` — camping at the map's camp spots. **Tier** (on below expert); 0 forces off, 1 on
  (`doc/goals.md`, `bot_camp.cpp`).
- `crossbot_wallbang` — **tier** flag: shoot a lost target through a wall the round gets through
  (`doc/combat.md`, "Wall-banging").
- `crossbot_spread_fit` — CS spread gate: the largest cone allowed, as a multiple of the target's half
  width. **Tier** (2 / 1.5 / 1.25 / 1); 0 turns the gate and the CS speed brace off (`doc/combat.md`,
  "The CS spread gate").
- `crossbot_oppo_interval` — retry throttle for opportunistic jumping (always on). **Tier**; easy halves
  it to 0.5 s.
- `crossbot_grenade_cook`, `crossbot_backstab`, `crossbot_brace_moving`, `crossbot_turn_scale`,
  `crossbot_back_check`, `crossbot_head_pct`, `crossbot_predict_pct`, `crossbot_combat_interrupt`,
  `crossbot_pickup_priority`, `crossbot_gauss`, `crossbot_bunnyhop` — **tier**; each is described in
  `doc/difficulty.md`, "cvars".

### Other

- `crossbot_name_prefix` — text put in front of every added bot's name, e.g. `[BOT] `. Default empty.
  Read at `crossbot_add`, so bots already in the game keep their names.

## Brain and orders

How the brain works is `doc/brain.md`.

- `crossbot_do <bot|all> <tool> [json]` — run one brain tool on a bot exactly as the LLM would: `act`,
  `rules`, `alert`, `release`, `say`, `say_team`, `radio`. The JSON is the tool's arguments; a top-level
  `"speaker":"<player>"` sets who `me` and `aim` mean. Examples:
  - `crossbot_do Wing act {"steps":[{"do":"follow","target":"Lead","seconds":60}]}`
  - `{"do":"fire","item":"weapon_hegrenade","target":"site:B"}`
  - `{"do":"ability","ability":"disguise_enemy","arg":"soldier"}`
  - `{"do":"hold","target":"aim","stance":"prone"}`
- `crossbot_directive_status` — per bot: position, current step and expiry, rules, alert, last step
  result, done/failed counts.
- `crossbot_brain_status` — config, humans, and per team brain: running, in flight, jobs, LLM calls, tool
  calls, errors, tokens, latencies, last verdict, last calls; then the ignore list.
- `crossbot_brain_log [n]` — the last n feed events (default 30) and the learned message/event ids.
- `crossbot_brain_memory [n]` — the newest memory notes.
- `crossbot_brain_reload` — reread `addons/crossbot/brain.json`.
- `crossbot_brain_ignore <name> [seconds]` — ignore a player, default 120 s.
- `crossbot_brain_unignore [name]` — forget one ignored player, or all.
- `crossbot_voice_say <bot> <text>` — the bot says the text on voice chat, and its own packets are fed
  back through the capture path as if a player had sent them, so the transcript reaches the brain as that
  bot's voice line: a full TTS → Opus → Steam packet → decode → STT round trip without a client.
- `crossbot_voice_tone <bot> [seconds] [hz]` — a sine tone (default 3 s, 440 Hz) on voice chat, no TTS.
  Prints a per-client report (active, live, human, hears, engine listening bit, datagram fill) and later
  `voice sent N frame(s) ... to X listener(s)`.
- `crossbot_voice_echo <bot|off>` — replay every human voice packet byte for byte from that bot's slot.
  Hearing yourself proves routing; the first packet prints its size and codec opcode.

Cvars:

- `crossbot_brain` — 0 turns the brain off without touching `brain.json`. Default 1.
- `crossbot_brain_debug` — 1 prints every JEV state, LLM context and raw answer. Default 0.
- `crossbot_brain_trace` — 1 appends every JEV and LLM exchange (request body, response, status, seconds;
  never the key) as a JSON line to `addons/crossbot/trace/brain_<map>.jsonl`, every speech-to-text and
  text-to-speech call to `voice_<map>.jsonl`, and saves each heard utterance as
  `voice_<map>_<userid>_<n>.wav`. Default 0.
- `crossbot_brain_test_human <name>` — treat that bot as a human, for tests. Default empty.
