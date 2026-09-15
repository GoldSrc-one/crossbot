# Console System Documentation

`commands.cpp` registers every server command in `Register()`. They are thin wrappers — the work lives
in `BotManager` or `Nav`, and each one is documented where that work is.

- `crossbot_add [gamedir|*] [name] [model|team]` — add a bot to a loaded game. The first argument is
  always the game directory; `*` or nothing picks a random playable one, and an unknown name lists
  what is loaded. A game is playable only with both gamedata and a game profile — see
  `doc/games.md`. The third argument belongs to that game's profile rather than to this command:
  in deathmatch it is a player model, and in a team mod it will be a team. `*` or nothing means the
  profile chooses — a random installed model in deathmatch. A model the scan did not find is a
  warning rather than a refusal: it prints the installed list and uses the name anyway, because
  models living inside a `.pak` are invisible to the scan. See `doc/games.md`, `player_models.cpp`,
  for where the list comes from and why CrossPlay may clamp the result.
- `crossbot_nav_goal <x> <y> <z>` — send all bots to a point
- `crossbot_teleport <x> <y> <z>` — teleport all bots
- `crossbot_kill` — suicide every bot; the only way to exercise the respawn path from the console
- `crossbot_nav_status` — the runtime planner's own answer; see `doc/manager.md`, `PrintNavStatus`
- `crossbot_waypoints` — full waypoint list per bot
- `crossbot_nav_penalties` — connections with a nonzero `runtimePenalty`, and the total
- `crossbot_list_spawns` / `crossbot_list_targets` — entity positions, for picking a goal by hand
- `crossbot_list_items` — every pickup and charger with its kind, resolved nav area, `frame`, `solid`
  and state; see `doc/goals.md`, `bot_items.cpp`
- `crossbot_list_fun` — the fun goal tier's whole inventory for this map: kind
  (`switch`/`tank`/`tram`), action (`use`/`touch`/`shoot`), brush centre, every resolved nav area, the
  chain terminal it fires, and why it was skipped (`no nav area`, `taken or locked`, `not ready`) or
  `ready`. Ends with a count. A `tram` row also prints the tram's current `speed` and its computed
  `deck` point, and with `crossbot_fun_debug 1` follows it with one `tram board area` line per board
  point giving the stand position, the deck position as the tram passes that node, and its arc length
  along the track; see `doc/goals.md`, `bot_fun.cpp`. `crossbot_fun_debug 1` also logs every bot death
  with its distance to the tram and its tram phase, which is the only way to separate bots crushed
  attempting a catch from bots run over while walking
- `crossbot_list_goals` — every `bot_goal` entity in the world with its resolved nav area, the
  player it belongs to (or `everyone` for a map-placed one), its `aiment` and the buttons it asks
  for. Ends with a count. Zero on a `valve` map is correct: the entity only exists where the
  `bot_goal.sma` plugin is running, which is on `cstrike`, `tfc` and `dod` maps; see
  `doc/goals.md`, `bot_strategic.cpp`
- `crossbot_list_gates` — every walk gate that could hold a mine: the mesh width, the *measured*
  opening, the chosen surface, the mount point and the rejection reason. A gate that failed only at
  the surface or coverage stage is printed too, so the near misses are visible; ones filtered earlier
  are not. Read the openings before the count — they should be doorway widths (96, 128), and a map
  reporting hundreds of gates across open floor means the opening measurement is not running; see
  `doc/movement.md`, `bot_deploy.cpp`
- `crossbot_deploy_status` — per bot: phase, target connection, measured opening, surface, stance,
  `placed`, `dets`, `aborts`, memo size and `why`. Unlike the `crossbot_fire_status` counters these
  are **not** per life — respawn clears the memo and any placement in flight but leaves the tallies
  alone, so a drop in them is a bug rather than a death; see `doc/movement.md`, `bot_deploy.cpp`
- `crossbot_weapons [name]` — the inventory snapshot: every weapon with its `m_iId`, clip, ammo indices
  and carried counts, both attack timers, and a `*` on the active one; see `doc/combat.md`, `bot_weapons.cpp`.
  In a game with `plr_iPowerups` (Ricochet) it also prints `powerups 0x<m_iPowerups>, +4 = <m_iPowerupDiscs>`,
  which is the live check that the hand-filled offset is right: 0 normally, `0x4` and 3 right after
  `crossbot_equip item_powerup_hard`
- `crossbot_weaponspec [name]` — how each gun is operated. With no argument, dumps the whole
  weapon table: per weapon id, the classname, clip size, `value`, range class and scope FOV, then one
  line per firing mode with its button pattern, aim requirement, toggle, ammo pool and ballistics. The
  range class is what bends routing toward ground the gun can use (`doc/goals.md`, "Range bias") and the
  scope FOV is what decides whether a camp spot is worth zooming into. With a player name, joins that
  player's carried weapons against their specs and flags any classname that disagrees, which is the
  one way the id-keyed table can be wrong; see `doc/games.md`, `weapon_spec.h`
- `crossbot_equip <classname> [name]` — spawn an item at the target's feet and touch them with it,
  which is what the game's own `give` does. Default target is every bot
- `crossbot_spawn <classname> [name] [distance]` — spawn a loose entity in front of the target instead of
  in its hands, traced out to `distance` (default 96) so it lands short of a wall rather than inside one.
  Default target is every bot. This is how `test_monster.ps1` produces live `monster_snark`s: letting the
  bots throw them does not work, because the snark is only reachable through the ambush path and the
  uphill veto refuses the throw almost everywhere on crossfire
- `crossbot_aim_test <bot> <yawDelta> [pitchDelta] [fov]` — command a flick relative to the bot's current view
  and print settle time and peak hand speed; the only check on the 180-in-0.4s derivation. Overrides
  navigation aim for 3 seconds. Two seconds in it also prints a **rest error**: the miss still standing once
  the view has stopped, which is the number the hand quantum sets and the one that should shrink with zoom.
  The optional 4th argument writes `pev->fov` before the flick (0 meaning the engine default) so a script
  can sweep that; see `doc/perception.md`, `bot_aim.cpp`
- `crossbot_aim_status` — per bot: sense target, bone sampled, ring occupancy, delay served, hand speed,
  and `err`, the angle between the real view and the true present-time angles to the target. `err` is the
  one number that separates "correctly delayed" from "delayed and also wrong", but it no longer bottoms out
  at zero: it is measured against the target eye while the bot aims at a random hitbox bone, so a couple of
  degrees is the floor. Read its trend across delays, not its value; see `doc/perception.md`, `bot_aim.cpp`
- `crossbot_teams_status` — the map's game (`gEntityOwners[0]`), whether that makes it a team game,
  the `crossbot_teams` value and the verdict; then one line per live player with its `pev->team` and,
  per bot, how that bot reads it (`friend` / `enemy` / `unknown`) and whether it may shoot it
  (`shoot=1`). **Run this first on any cs/tfc/dod map.** Every feature below rests on the mod actually
  filling `pev->team` for a fake client another game DLL created, and this is the only thing that says
  whether it does: all-zero teams means teams idle and bots behave exactly as on a deathmatch map
- `crossbot_usermsgs` — every user message the loaded game DLLs registered, by id, name and size,
  with `<- daze` on `ScreenFade` and `Concuss`. Run it before testing flashes or concussions: if
  either id reads 0, nothing downstream can fire. The ids come from a `pfnRegUserMsg` hook, so a
  plugin brought in by `meta load` after the game DLLs registered lists the names but marks none;
  see `doc/perception.md`, `bot_daze.cpp`
- `crossbot_daze_status` — per bot: seconds of blindness left, the alpha, fade and hold of the last
  accepted flash, `flashes`, the current `conc` byte and how long it has been non-zero, `updates`
  received during this concussion, `concs`, and the `sway` being added to pitch and yaw right now.
  The counters are per life
- `crossbot_daze_test <bot|*> flash <alpha> [duration] [hold]` and
  `crossbot_daze_test <bot|*> conc <scale>` — feed a synthetic `ScreenFade` (white, flags 0; duration
  and hold default to CS's point-blank 12 and 2.67) or `Concuss` byte straight into the handlers, as
  if the wire had delivered it. Exercises everything downstream of the message hook without a CS or
  TFC client. `conc 0` clears
- `crossbot_sense_status` — per bot: the bot's own `team`, aim target and bone, `cands`/`keep` for the
  vision pick, live suspicion count with a `kinds` breakdown
  by player/monster/other, the `tier` that won the look vote, the freshest entry with source, age and
  position, the look direction with its authority and how many entries survived the cone refinement, and
  `pushed` — cumulative per-source push counts. `tier` must read `PLAYER` whenever `kinds=p:` is nonzero.
  `cands` is how many players passed range and the view cone — i.e. how many tracelines the old
  every-candidate scan would have spent, against the 1-2 now actually spent. `keep` is `inc` when the
  incumbent enemy held, `new` for a fresh weighted-random pick, `none` when nothing was visible; a duel
  should read `inc` on nearly every frame, and a run of `new` means the lock is broken.
  `fresh` is the freshest entry overall rather than within the winning tier, so it can name an entity the
  vote ignored. `hunt` is how many live entries are monsters hunting this bot — those vote at the player
  tier, so `tier=PLAYER` with `kinds=p:0 hunt=1` is correct and not a contradiction. `mon` is the monster
  vision path in five numbers: `list` is the shared per-frame monster list, `seen` how many of those this
  bot considered, `mine` how many named it as their enemy, `trc` tracelines spent on monsters and `hit`
  how many landed. `mine:0` with `list` nonzero means nothing is hunting this bot; `mine` nonzero with
  `hit:0` means it is hunted but never has line of sight; see `doc/perception.md`, `bot_sense.cpp`
- `crossbot_monsters` — one line per `FL_MONSTER` edict on the map with its classname, flags, health,
  `takedamage`, solid type, box centre and the enemy its `m_hEnemy` names, then one indented line per bot
  giving distance, whether the monster is in that bot's view cone, and whether a live traceline to the box
  centre reaches it. This is the command that separates "the bot ignores snarks" from "there are no
  snarks" and from "the offset is wrong": the trailing summary prints the resolved `mon_hEnemy` offset.
  It refreshes the shared monster list before printing, so it is safe with no bots on the server
- `crossbot_fire_status` — per bot: the weapon held, whether it is `zoom`ed, the fire mode chosen, why
  the bot is or is not shooting, live clip and ammo, and `miss`/`tol` — the angle from the fire ray to
  the aim point against the tolerance allowed there, which is the whole aim gate in two numbers. `aim`
  is `target`, `lead`, `homing` or `arc`; for `homing`, `tol` is the fixed 20 degree cone rather than
  anything the target subtends. **For `arc` both `miss` and `tol` are units, not degrees** — the gap
  from the simulated impact point to the target's box, against the splash radius allowed there — so
  do not compare them against the other three. `flight` is the solved projectile flight time in
  milliseconds, zero for hitscan **and for homing**, which solves nothing.

  `commit` is what the bot has started and will not walk away from — `none`, `charge` (gauss spinning
  up), `throw` (handgrenade pin pulled, waiting for the engine to let go of it) or `guide` (rocket in
  the air) — and `for` is how many seconds it has been in it, or -1. `why=primed` and `why=guiding`
  are the two commits seen from the trigger's side, and `why=splash-self` is the direct-fire twin of
`arc-self` — the same rule, refusing a rocket rather than a shell. `why=sticky` is the trigger being
held through a frame the crosshair is off the target, which on a `WPN_Q_STICKY` weapon is deliberate;
it appears from both sides of the aim-queue boundary and means the same thing on each.

  `shots`, `leadshots`, `chargeshots`, `arcshots`, `throwshots`, `guides`, `stickyshots`, `uphill`,
  `selfveto`,
  `switches`, `reloads`, `emptyswitch`, `idleswitch` and `idlereload` are **per life**: respawn zeroes
  them, so a drop is a death. The carry table that drives selection is the one thing that does *not*
  reset — read it with `crossbot_weapon_carry`. `emptyswitch` counts fights the bot left a dry clip in
  rather than reloading it, `idleswitch` counts out-of-combat rotations, and `idlereload` counts
  `IN_RELOAD` presses issued with no target — several per shotgun refill, one per magazine elsewhere.
  `throwshots` and `chargeshots` are counted from the game DLL's own state rather than from an attack timer,
  because neither weapon moves one when the bot presses. `guides` is one per rocket launched, and
  `capped` counts commits released by their safety valve rather than by the game's own state —
  non-zero is a fault, with the same standing `charge-timeout` has on the gauss. `stickyshots` is the
  subset of `shots` that only happened because the trigger stayed down off-target; it never counts a
  gauss primary shot, because that weapon moves no attack timer for the counter to see. `uphill` and `selfveto` count rejections rather than
  situations — both predicates run per weapon per mode per frame — so read them as a yes/no; see
  `doc/combat.md`, `bot_fire.cpp`
- `crossbot_weapon_carry [name]` — per bot: the weapon held, then `classname=carry/score` for every
  weapon the bot has ever held, and `carried` — how many that is. Carry is seconds the weapon has spent
  in the inventory while the bot was alive; score is the weapon's `WeaponSpec::value`. Together they
  are the whole of selection: highest score wins, lowest carry breaks the tie. Every weapon listed has
  a non-zero carry by construction, so the count proves nothing — what is worth reading is the
  *spread*. All-equal times mean the walk is charging every slot rather than the carried ones, and a
  bot whose held weapon is neither the top score nor the least carried of that score means the ranking
  is not being applied. Unlike the other counters it survives respawn, and unlike `crossbot_weapons`
  it can only report bots; see `doc/combat.md`, `bot_fire.cpp`
- `crossbot_fire_pin <classname|off> [primary|secondary] [zoom|nozoom]` — restrict weapon selection to
  one classname, and optionally one mode, making everything else unusable. Only useful for testing, and
  necessary there because in a fight a bot only re-selects when the weapon it holds stops working — it
  keeps any weapon with a usable mode and a loaded clip, so one that spawned with a glock will not
  reach for a crossbow while an enemy is in front of it. Out-of-combat rotation does move the held
  weapon around, but on its own schedule and never onto the one a test wants. Without the classname the
  projectile path is unreachable from the console; without the mode the gauss charge is. With no
  argument it reports all three.

  The third argument overrides `ZoomButtons`, which otherwise picks the zoom state by range — in at
  500 units, out at 300. On a map whose fights are shorter than that the bot barely zooms, so the
  zoomed crossbow shot is nearly unreachable by chance: an unforced run produced 12 zoomed presses
  and not one hit, leaving the 120 damage sniper bolt untested. `zoom` and `nozoom` force the state
  so the two can be exercised separately; `any` restores the range rule.

  **The pin also equips.** While one is set, a bot that does not carry the pinned weapon is given it,
  and one that carries it with the pinned mode out of ammo is given ammo for that mode
  (`IGameProfile::AmmoEntity`, see `doc/games.md`). It gives only when the mode is *unusable*, never on
  a timer, so reloading and clip depletion stay real and only permanent starvation is removed. Before
  this, an RPG run reached one rocket per bot per life.

## crossbot_difficulty_status

Prints the resolved tier and every row of the difficulty ladder with its effective value and
whether it came from the tier or from a pinned cvar. This is the only way to audit the ladder from
a live server: every difficulty-derived cvar defaults to -1 and reads as "tier", so a stale
`crossbot_aim_delay 0.25` left behind by a test script is otherwise invisible and silently
overrides the difficulty for the rest of the session. See `doc/difficulty.md`, `bot_difficulty.cpp`.

The ladder itself is one global setting, `crossbot_difficulty` 0-3 (easy/normal/hard/expert),
modelled on the CS bot's `cv_bot_difficulty`. Normal is the behaviour that predates it.

The dedicated server converts every console line to wide characters through
`CTextConsoleWin32::PrintRaw`, which holds a `WCHAR unicodeStr[1024]` and guards it with
`if (nSize > sizeof(unicodeStr))` — a count of *characters* tested against a count of *bytes*. Any
line of 1024 characters or more therefore passes the guard and overruns the buffer by up to 2 KB.
The symptom is not a crash: hlds stops consuming CPU and sits on an invisible modal
`Run-Time Check Failure #2 - Stack around the variable 'outputStr' was corrupted` box, so the
server looks hung and only a kill clears it. That file belongs to CrossPlay and is read-only to us,
so the limit is ours to respect.

Every fixed print buffer in the tree is therefore 1024 bytes or smaller, which caps a line at 1023
characters. The one dump built in a `std::string` rather than a fixed buffer, `PrintNavStatus`, is
unbounded by nature — its `waypoints=[...]` block grows with the route — and goes out through
`PrintLong`, which splits at a comma or a space below 1000 characters and prints the pieces. It found
the bug: a bot with a long strategic route on `dod_flash` froze the server on the first
`crossbot_nav_status`. Any new dump that concatenates per-waypoint or per-area text belongs behind
the same helper.

`crossbot_nav_penalties` is the only view of what the bots have learned this map (see `doc/path_planning.md`,
Runtime connection penalties). Everything else is derived state that a repath would rebuild; the
penalty table is not, and it dies with the map.

`crossbot_list_items` answers a question no bot-side log can: an item whose area reads `none` is one
the gather goals cannot see at all, and that is a mesh gap rather than a bot bug. Run it first on any
map where the bots seem to ignore something.

`crossbot_list_fun` is the same idea one tier up, and it is the *first* thing to run when bots ignore
a map's gimmick, because it separates the two failures that look identical from the bot side: an
entity absent from the list was never detected (its chain does not reach an interesting terminal, or
it dangles), while one that is listed but reads `no nav area` was detected and is unreachable. It is
also the negative control — on `datacore` it must print `0 fun entities`, and on `crossfire` the nuke
trigger must read `no nav area` until `strike_ready_door` opens at t=180.

Note `ready` on this dump means "detected, chain is interesting, and it resolved to a nav area". It
does **not** mean the bots can route to it: reachability is priced per bot in `BotGoal::Select`, and
crossfire's nuke reads `ready` after t=180 while still pricing as unreachable. An entity that is
listed `ready` and never appears in a `wants fun` line is a routing gap, not a detection gap.

For the subtransit tram the useful reading is the `deck` point against the board-point dump. Board
points are derived from the track, not from where the tram happens to be, so the list is identical
whether the tram is parked or doing 400 — if it changes when the tram moves, the deck geometry is
being taken from `pev->origin` instead of `DeckAt`. A board point whose stand z is far from the deck z
is the signature of a walkway the bot cannot actually get down from.

`crossbot_weapons` ends with a line comparing the walked weapon mask against `pev->weapons`. That is
the only self-check the offsets have, so read it before believing any other number on the dump — see
`doc/combat.md`, `bot_weapons.cpp`. `crossbot_equip` exists to reach a weapon state that gathering would
take a whole match to produce, and it takes any entity classname, ammo boxes included.

Cvars, all `FCVAR_SERVER`, registered in `dll_hooks.cpp`:

- `crossbot_verbose` — **read-only marker, not a switch.** Registered only when the plugin was built
  with `CROSSBOT_VERBOSE` (the default), the same way `crossbot_perf` marks a `CROSSBOT_PERF` build.
  Setting it does nothing: the narration is compiled out, not branched on. It exists so a script can
  tell the two builds apart before it starts parsing per-bot log lines —
  `test_server_start.ps1 -RequireVerbose` probes it and refuses to run without it
- `crossbot_bsptrace` / `crossbot_bsptrace_blockers` — **only exist in a `-DCROSSBOT_BSPTRACE=ON`
  build**, which is not the default. A/B the in-process BSP hull tracer against the engine's
  `pfnTraceHull` on the planner's airmove traces; read with `crossbot_bsptrace_stats`. Mode 1 still
  returns the engine's answer, so it measures without changing behaviour. See `doc/testing.md` for the
  numbers and why it is off
- `crossbot_trace_path` — where `WriteTraceLog` puts the bot trajectory (verbose builds only)
- `crossbot_nav_avoid` — obstacle avoidance layer, 0 to A/B a suspected regression without rebuilding
- `crossbot_nav_break` — breaking `func_breakable` out of the way. Default 1. With it off the bot falls
  back to the pre-`BotBreak` behaviour: it avoids the breakable, or gives up on the goal behind it
- `crossbot_gather` — item gathering goals. Default 1. `test_nav_goal.ps1` sets it to 0, because a bot
  that picks a new goal a frame after arriving never shows the harness the `hasGoal=0` it polls for
- `crossbot_fun` — the fun goal tier. Default 1. With it off `BotGoal::Select` reduces exactly to the
  old gather-only behaviour, which is what makes it the regression control: **the nav sweep sets it
  to 0** alongside `crossbot_gather 0`, so route behaviour stays comparable against transcripts taken
  before the tier existed
- `crossbot_fun_debug` — per-frame arrival diagnostics while a bot is at a switch or mounting a tank
- `crossbot_strategic` — the strategic goal tier, which is whether bots follow `bot_goal`
  entities at all. Default 1. With it off the top tier is empty and a bot on a `cstrike`, `tfc`
  or `dod` map falls back to fun and gather, which is the control that separates "the objective
  logic is wrong" from "the navigation is wrong"
- `crossbot_strategic_debug` — one line per `bot_goal` per selection saying whether it was
  taken, left on its cooldown memo, or could not be placed in a nav area
- `crossbot_goal_bias` — exponent in the fun/strategic weight `1 / (cost + 2)^bias`. Default 2. 0 is
  uniform random across candidates, which is how you tell a selection problem from a distance-weighting
  one; large values collapse to nearest-first. Does not affect gather, which is always cheapest-first
- `crossbot_detour_budget` — seconds of extra travel a bot will accept to pick something up on the way
  to a distant goal. Default 6. 0 disables detours entirely without disabling the tier, so a bot walks
  straight at its target; see `doc/goals.md`, `bot_goal.cpp`
- `crossbot_teams` — friend/foe gating. Default 1, and inert unless the map belongs to `cstrike`,
  `tfc` or `dod`. With it on a bot never sights, chases, retaliates against or detonates a satchel on
  anything that is not a confirmed enemy; with it off every gate returns "attackable" and the bot
  behaves as it always has. It is the A/B switch for the whole feature — on a `valve` map the two
  settings must be indistinguishable. See `doc/games.md`, `teams.cpp`
- `crossbot_combat` — the combat state. Default 1. With it on, a bot that sees an enemy (or is shot
  by one) drops whatever it was walking to and repositions locally instead, until the enemy dies,
  vanishes, stops being sensed, or goes 5s unseen. With it off the bot never enters the state and
  keeps gathering through a firefight — it still shoots, since aim and the trigger are
  `crossbot_fire`'s. Inert without `crossbot_sense`, which is what supplies the sighting. Unlike the
  `crossbot_chase` it replaces, it is **not** disabled by `crossbot_gather 0`: combat is no longer a
  goal, so it does not live inside `SelectGoal`
- `crossbot_combat_debug` — one line per probe: which way the bot decided to shuffle, the area it
  resolved to, and the exposure there against the exposure where it stands. Also logs the frame an
  enemy notices the bot, which is what ends an ambush
- `crossbot_ambush` — open on an enemy who has not noticed the bot yet with melee at contact range,
  otherwise a grenade or a snark. Default 1. Off, the bot always opens with its best gun, which is
  what it did before the flag existed — useful as the A/B, since the awareness verdict is still
  logged either way. Inert without `crossbot_combat`, which owns the flag, and it decides nothing on
  its own: an ambush weapon is only ever *offered*, and if none is usable at that range the ordinary
  selection runs untouched
- `crossbot_duel` — mirror an opponent who has melee out: draw melee too, charge until in reach, back
  off after each swing. Default 1. Gated on the enemy standing in the bot's *own* nav area, so how far
  a duel can start from is the map's business, not a tunable. Off, the bot fights a crowbar opponent
  with whatever it would otherwise have picked, and the `melee=`/`inarea=` flags are still reported so
  the A/B is readable. Inert without `crossbot_combat`. Note that a bot holding melee out of range is
  the *point* here rather than a bug: the charge is what closes the gap
- `crossbot_gather_debug` — dump every gather selection: each item entity, its area, and its cost or the
  reason it was skipped, plus the bot's readiness in the header and an `ENEMY` marker on the enemy row.
  Default 0, and verbose enough to be worth turning back off; see `doc/goals.md`
- `crossbot_oppo_jump` — opportunistic jumping. Default -1, following `crossbot_difficulty`,
  which enables it at every tier; 0 forces it off and 1 forces it on. The retry throttle is
  `crossbot_oppo_interval`, which easy halves to 0.5s
- `crossbot_time_debug` — movement-clock diagnostics. Default 0. Once a second prints the server's own
  frame budget, then one line per bot: msec issued versus that budget, how much of it the engine
  actually ran (`SV_RunCmd` silently drops commands while the `clockwindow` guard is armed, which is
  only visible as `PlayerPostThink` not firing), the resulting drift between bot time and real time, and
  a `chain` line showing the whole bot loop in order with the msec each one saw and issued. A bot
  getting less sim time per second than the wall clock is the "honey mode" symptom -- slow running and
  slow gravity together; see `doc/movement.md`, *Bot_BeginFrame and Bot_FrameTime*
- `crossbot_aim_delay` — reaction delay in seconds, clamped [0, 0.5]. Default -1, meaning the
  delay comes from `crossbot_difficulty` as a per-engagement roll inside that tier's band. Any value
  `>= 0` pins it to a fixed point value instead. Inert unless `crossbot_sense` is on, since vision is
  what feeds the queue
- `crossbot_sense` — the whole sense system: vision, heard sounds and events, damage, and the suspicion
  list that drives where the bot looks when it has no target. Default 1, held there by a 45/45 nav sweep
  at both settings; see `doc/perception.md`, `bot_sense.cpp`
- `crossbot_sense_decay` — suspicion weight and decay window in seconds, clamped [0.1, 60]. Default 5.
  Doubles as the blend authority: a suspicion this old has zero pull on the view
- `crossbot_aim_error` — scale on the intentional over/undershoot, clamped [0, 4]. Default -1,
  following `crossbot_difficulty` (2.0 easy down to 0.35 expert). At 0 the brake
  is forced to exactly 1 and the controller is time-optimal again, which is the regression baseline
  `scripts/test_aim_error.ps1` measures against. Above 1 is for seeing the effect, not for playing against
- `crossbot_fire` — whether bots shoot at all. Default 1. Inert without `crossbot_sense`, since firing
  needs a target and an aim queue; and inert in the nav sweep either way, because a lone bot never
  acquires one. Turning it off also stops weapon *switching* and the out-of-combat reload, which are
  the same decision made without a target; see
  `doc/combat.md`, `bot_fire.cpp`
- `crossbot_daze_debug` — 1 logs every intercepted `ScreenFade` with its raw wire fields and verdict,
  and the onset and clear of each concussion; 2 adds every intermediate `Concuss` byte and one line
  per user message addressed to any bot. Default 0. Verbose builds only
- `crossbot_blind_scale` — how long a flash blinds, as a fraction of the fade duration on the wire.
  Default 0.33, ReGameDLL's own ratio for its bots
- `crossbot_conc_gain` — multiplier on the `Concuss` byte before it becomes sway. Default 1, which at
  TFC's start value of 100 is ±30 degrees on pitch and yaw
- `crossbot_aim_drift` — idle micro-drift amplitude in degrees, clamped [0, 5]. Default 1. Suppressed while
  `crossbot_aim_test` is running, because 1 degree of drift against a 0.5 degree settle tolerance would mean
  the test never settles at all
- `crossbot_aim_quantum` — degrees of hand travel per mouse count, clamped [0, 5]. Default 0.4476, which is
  `atan(1/128)`: one unit of miss per 128 units of range at fov 90, so 32 units at 4096. This is the value
  itself, not a scale. It is in **hand** space, so the angular quantum is `value * fov / 90` and zooming
  buys precision — that is the whole point of it. 0 disables
- `crossbot_aim_impulse` — smallest non-zero hand speed in deg/s, clamped [0, 100]. Default 15. The hand
  cannot creep more slowly than this, so it never fully stops: it dithers across the nearest mouse count and
  `crossbot_aim_status` shows an idle hand speed of 21 rather than 0. 0 disables

- `crossbot_deploy` — whether bots place tripmines and satchels. Default 1. **The nav sweep sets it to
  0**, the same way it sets `crossbot_gather 0`: a mine changes every route through the doorway it
  blocks, and the sweep's per-map waypoint diffs depend on the route being repeatable. With it off the
  beam registry stays empty and every branch this feature added to `bot_avoid.cpp` early-outs, so a
  sweep clears a deployable change for movement regressions and says nothing about deployment itself;
  see `doc/movement.md`, `bot_deploy.cpp`
- `crossbot_deploy_debug` — per-placement and per-death tripmine forensics. Default 0. Prints an
  audit line for every mine actually spawned (real beam length against the measured opening, both
  jambs, and whether the placer is standing in its own beam), a residency line saying how long the
  bot took to clear that beam against the 2.5 s arming clock, and an attribution line on every death.
  `KILLED BY ITS OWN MINE` carries `onbeam`, which is the number that matters: `1` means the bot
  walked into its own laser and is a bug, `0` means it was inside the blast when something else
  tripped the mine and is the game working; see `doc/movement.md`, `bot_deploy.cpp`
- `crossbot_deploy_surface` — pin the mount surface: 0 off, 1 wall, 2 ceiling, 3 ground. Default 0.
  A pin also bypasses the opening bound that normally keeps ceiling and ground mounts to gaps under 48
  units. It exists for the reason `crossbot_fire_pin` does: a vertical beam is what reaches the side
  search and the standoff shot, and openings that narrow are rare enough that an unpinned scan across
  six stock maps finds none, so those branches would ship untested. Not for playing against — a
  vertical beam in a wide doorway is walked past without anyone noticing it

Output from all of these is `SERVER_PRINT`, so it goes to the HLDS console and `D:\hlds\qconsole.log` —
**not** to the stdout of whatever script sent the command over RCON.

