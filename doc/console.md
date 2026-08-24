# Console System Documentation

`commands.cpp` registers every server command in `Register()`. They are thin wrappers — the work lives
in `BotManager` or `Nav`, and each one is documented where that work is.

- `crossbot_add [name]` — add a bot
- `crossbot_nav_goal <x> <y> <z>` — send all bots to a point
- `crossbot_teleport <x> <y> <z>` — teleport all bots
- `crossbot_kill` — suicide every bot; the only way to exercise the respawn path from the console
- `crossbot_nav_status` — the runtime planner's own answer; see `doc/bot.md`, `PrintNavStatus`
- `crossbot_waypoints` — full waypoint list per bot
- `crossbot_nav_penalties` — connections with a nonzero `runtimePenalty`, and the total
- `crossbot_list_spawns` / `crossbot_list_targets` — entity positions, for picking a goal by hand
- `crossbot_list_items` — every pickup and charger with its kind, resolved nav area, `frame`, `solid`
  and state; see `doc/bot.md`, `bot_items.cpp`
- `crossbot_weapons [name]` — the inventory snapshot: every weapon with its `m_iId`, clip, ammo indices
  and carried counts, both attack timers, and a `*` on the active one; see `doc/bot.md`,
  `bot_weapons.cpp`
- `crossbot_weaponspec [name]` — how each gun is operated. With no argument, dumps the whole
  weapon table: per weapon id, the classname and clip size, then one line per firing mode with its
  button pattern, aim requirement, toggle, ammo pool and ballistics. With a player name, joins that
  player's carried weapons against their specs and flags any classname that disagrees, which is the
  one way the id-keyed table can be wrong; see `doc/game.md`, `weapon_spec.h`
- `crossbot_equip <classname> [name]` — spawn an item at the target's feet and touch them with it,
  which is what the game's own `give` does. Default target is every bot
- `crossbot_aim_test <bot> <yawDelta> [pitchDelta] [fov]` — command a flick relative to the bot's current view
  and print settle time and peak hand speed; the only check on the 180-in-0.4s derivation. Overrides
  navigation aim for 3 seconds. Two seconds in it also prints a **rest error**: the miss still standing once
  the view has stopped, which is the number the hand quantum sets and the one that should shrink with zoom.
  The optional 4th argument writes `pev->fov` before the flick (0 meaning the engine default) so a script
  can sweep that; see `doc/bot.md`, `bot_aim.cpp`
- `crossbot_aim_status` — per bot: sense target, bone sampled, ring occupancy, delay served, hand speed,
  and `err`, the angle between the real view and the true present-time angles to the target. `err` is the
  one number that separates "correctly delayed" from "delayed and also wrong", but it no longer bottoms out
  at zero: it is measured against the target eye while the bot aims at a random hitbox bone, so a couple of
  degrees is the floor. Read its trend across delays, not its value; see `doc/bot.md`, `bot_aim.cpp`
- `crossbot_sense_status` — per bot: aim target and bone, live suspicion count with a `kinds` breakdown
  by player/monster/other, the `tier` that won the look vote, the freshest entry with source, age and
  position, the look direction with its authority and how many entries survived the cone refinement, and
  `pushed` — cumulative per-source push counts. `tier` must read `PLAYER` whenever `kinds=p:` is nonzero.
  `fresh` is the freshest entry overall rather than within the winning tier, so it can name an entity the
  vote ignored; see `doc/bot.md`, `bot_sense.cpp`
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
`arc-self` — the same rule, refusing a rocket rather than a shell.

  `shots`, `leadshots`, `chargeshots`, `arcshots`, `throwshots`, `guides`, `uphill`, `selfveto`,
  `switches` and `reloads` are **per life**: respawn zeroes them, so a drop is a death. `throwshots`
  and `chargeshots` are counted from the game DLL's own state rather than from an attack timer,
  because neither weapon moves one when the bot presses. `guides` is one per rocket launched, and
  `capped` counts commits released by their safety valve rather than by the game's own state —
  non-zero is a fault, with the same standing `charge-timeout` has on the gauss. `uphill` and `selfveto` count rejections rather than
  situations — both predicates run per weapon per mode per frame — so read them as a yes/no; see
  `doc/bot.md`, `bot_fire.cpp`
- `crossbot_fire_pin <classname|off> [primary|secondary]` — restrict weapon selection to one
  classname, and optionally one mode, making everything else unusable. Only useful for testing, and
  necessary there for two layers of the same reason. A bot keeps any weapon that still has a usable
  mode, so one that spawned with a glock never voluntarily picks up anything else; and the random pick
  discards the mode it chose, so a weapon whose primary works never reaches its secondary. Without the
  classname the projectile path is unreachable from the console; without the mode the gauss charge is.
  With no argument it reports both.

  **The pin also equips.** While one is set, a bot that does not carry the pinned weapon is given it,
  and one that carries it with the pinned mode out of ammo is given ammo for that mode
  (`IGameProfile::AmmoEntity`, see `doc/game.md`). It gives only when the mode is *unusable*, never on
  a timer, so reloading and clip depletion stay real and only permanent starvation is removed — which
  is what a DPS measurement needs. Before this, an RPG phase measured one rocket per bot per life.

- `crossbot_dps_status` — rows written to the DPS log so far, split by kind, plus the running gaze
  and damage totals. The one line `scripts/measure_dps.ps1` polls between phases. Exists only in a
  `CROSSBOT_DPS` build; see `doc/bot.md`, `bot_dps.cpp`

`crossbot_nav_penalties` is the only view of what the bots have learned this map (see `doc/nav.md`,
Runtime connection penalties). Everything else is derived state that a repath would rebuild; the
penalty table is not, and it dies with the map.

`crossbot_list_items` answers a question no bot-side log can: an item whose area reads `none` is one
the gather goals cannot see at all, and that is a mesh gap rather than a bot bug. Run it first on any
map where the bots seem to ignore something.

`crossbot_weapons` ends with a line comparing the walked weapon mask against `pev->weapons`. That is
the only self-check the offsets have, so read it before believing any other number on the dump — see
`doc/bot.md`, `bot_weapons.cpp`. `crossbot_equip` exists to reach a weapon state that gathering would
take a whole match to produce, and it takes any entity classname, ammo boxes included.

Cvars, all `FCVAR_SERVER`, registered in `dll_hooks.cpp`:

- `crossbot_trace_path` — where `WriteTraceLog` puts the bot trajectory
- `crossbot_nav_avoid` — obstacle avoidance layer, 0 to A/B a suspected regression without rebuilding
- `crossbot_nav_break` — breaking `func_breakable` out of the way. Default 1. With it off the bot falls
  back to the pre-`BotBreak` behaviour: it avoids the breakable, or gives up on the goal behind it
- `crossbot_gather` — item gathering goals. Default 1. `test_nav_goal.ps1` sets it to 0, because a bot
  that picks a new goal a frame after arriving never shows the harness the `hasGoal=0` it polls for
- `crossbot_gather_debug` — dump every gather selection: each item entity, its area, and its cost or the
  reason it was skipped. Default 0, and verbose enough to be worth turning back off; see `doc/bot.md`
- `crossbot_oppo_jump` — opportunistic jumping. Default 1
- `crossbot_aim_delay` — reaction delay in seconds, clamped [0, 0.5]. Default 0.3. Inert unless
  `crossbot_sense` is on, since vision is what feeds the queue
- `crossbot_sense` — the whole sense system: vision, heard sounds and events, damage, and the suspicion
  list that drives where the bot looks when it has no target. Default 1, held there by a 45/45 nav sweep
  at both settings; see `doc/bot.md`, `bot_sense.cpp`
- `crossbot_sense_decay` — suspicion weight and decay window in seconds, clamped [0.1, 60]. Default 5.
  Doubles as the blend authority: a suspicion this old has zero pull on the view
- `crossbot_aim_error` — scale on the intentional over/undershoot, clamped [0, 4]. Default 1. At 0 the brake
  is forced to exactly 1 and the controller is time-optimal again, which is the regression baseline
  `scripts/test_aim_error.ps1` measures against. Above 1 is for seeing the effect, not for playing against
- `crossbot_fire` — whether bots shoot at all. Default 1. Inert without `crossbot_sense`, since firing
  needs a target and an aim queue; and inert in the nav sweep either way, because a lone bot never
  acquires one. Turning it off also stops weapon *switching*, which is the same decision; see
  `doc/bot.md`, `bot_fire.cpp`
- `crossbot_aim_drift` — idle micro-drift amplitude in degrees, clamped [0, 5]. Default 1. Suppressed while
  `crossbot_aim_test` is running, because 1 degree of drift against a 0.5 degree settle tolerance would mean
  the test never settles at all
- `crossbot_dps` — the DPS measurement layer: gaze clocks, shot records and the damage rows the
  `CBasePlayer::TakeDamage` hook feeds. Default **0**, because it writes a file. Turning it on
  mid-map is fine; the gaze counters start from that moment. Both this and `crossbot_dps_log` exist
  only in a build configured with `-DCROSSBOT_DPS=ON` — a normal build has no such cvars, and no
  hook
- `crossbot_dps_log` — where the raw CSV goes, resolved against the HLDS working directory like
  `crossbot_trace_path`. Opened in **append** mode, so re-running a sweep accumulates samples rather
  than replacing them. Every parallel server needs its own path

- `crossbot_aim_quantum` — degrees of hand travel per mouse count, clamped [0, 5]. Default 0.4476, which is
  `atan(1/128)`: one unit of miss per 128 units of range at fov 90, so 32 units at 4096. This is the value
  itself, not a scale. It is in **hand** space, so the angular quantum is `value * fov / 90` and zooming
  buys precision — that is the whole point of it. 0 disables
- `crossbot_aim_impulse` — smallest non-zero hand speed in deg/s, clamped [0, 100]. Default 15. The hand
  cannot creep more slowly than this, so it never fully stops: it dithers across the nearest mouse count and
  `crossbot_aim_status` shows an idle hand speed of 21 rather than 0. 0 disables

Output from all of these is `SERVER_PRINT`, so it goes to the HLDS console and `hlds/qconsole.log` —
**not** to the stdout of whatever script sent the command over RCON.
