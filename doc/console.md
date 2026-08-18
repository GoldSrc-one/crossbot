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
- `crossbot_equip <classname> [name]` — spawn an item at the target's feet and touch them with it,
  which is what the game's own `give` does. Default target is every bot

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

Output from all of these is `SERVER_PRINT`, so it goes to the HLDS console and `hlds/qconsole.log` —
**not** to the stdout of whatever script sent the command over RCON.
