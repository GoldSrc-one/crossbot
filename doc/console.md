# Console System Documentation

`commands.cpp` registers every server command in `Register()`. They are thin wrappers — the work lives
in `BotManager` or `Nav`, and each one is documented where that work is.

- `crossbot_add [name]` — add a bot
- `crossbot_nav_goal <x> <y> <z>` — send all bots to a point
- `crossbot_teleport <x> <y> <z>` — teleport all bots
- `crossbot_nav_status` — the runtime planner's own answer; see `doc/bot.md`, `PrintNavStatus`
- `crossbot_waypoints` — full waypoint list per bot
- `crossbot_nav_penalties` — connections with a nonzero `runtimePenalty`, and the total
- `crossbot_list_spawns` / `crossbot_list_targets` — entity positions, for picking a goal by hand

`crossbot_nav_penalties` is the only view of what the bots have learned this map (see `doc/nav.md`,
Runtime connection penalties). Everything else is derived state that a repath would rebuild; the
penalty table is not, and it dies with the map.

Output from all of these is `SERVER_PRINT`, so it goes to the HLDS console and `hlds/qconsole.log` —
**not** to the stdout of whatever script sent the command over RCON.
