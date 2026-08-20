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
- `crossbot_aim_test <bot> <yawDelta> [pitchDelta]` — command a flick relative to the bot's current view
  and print settle time and peak hand speed; the only check on the 180-in-0.4s derivation. Overrides
  navigation aim for 3 seconds; see `doc/bot.md`, `bot_aim.cpp`
- `crossbot_aim_status` — per bot: sense target, bone sampled, ring occupancy, delay served, hand speed,
  and `err`, the angle between the real view and the true present-time angles to the target. `err` is the
  one number that separates "correctly delayed" from "delayed and also wrong", but it no longer bottoms out
  at zero: it is measured against the target eye while the bot aims at a random hitbox bone, so a couple of
  degrees is the floor. Read its trend across delays, not its value; see `doc/bot.md`, `bot_aim.cpp`
- `crossbot_sense_status` — per bot: aim target and bone, live suspicion count, the freshest entry with
  source, age and position, and the look direction with its authority and how many entries survived the
  cone refinement; see `doc/bot.md`, `bot_sense.cpp`
- `crossbot_fire_status` — per bot: the weapon held, the fire mode chosen, why the bot is or is not
  shooting, live clip and ammo, and `miss`/`tol` — the angle from the fire ray to the served aim point
  against the angle the target's own size subtends there, which is the whole tolerance model in two
  numbers. `shots`, `switches` and `reloads` are **per life**: respawn zeroes them, so a drop is a
  death; see `doc/bot.md`, `bot_fire.cpp`

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

Output from all of these is `SERVER_PRINT`, so it goes to the HLDS console and `hlds/qconsole.log` —
**not** to the stdout of whatever script sent the command over RCON.
