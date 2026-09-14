# Manager

The manager that spawns and buries bots: death and respawn, level changes, mesh load and unload, and the status dumps.
Movement is `doc/movement.md`; difficulty is `doc/difficulty.md`.

## bot_manager.cpp

### Death and respawn

`StartFrame` drives a dead bot with `IGameProfile::Respawn` instead of `Bot_MoveNav`, and it has to keep
doing so every frame: `SV_Physics` skips client edicts entirely, so a fake client's think only runs from
inside `pfnRunPlayerMove` (`SV_RunCmd` sets `v.button` from the usercmd, then `pfnPlayerPreThink`, and
`CBasePlayer::PreThink` dispatches to `PlayerDeathThink` whenever `deadflag >= DEAD_DYING`). Stop calling
`pfnRunPlayerMove` for a dead bot and the game DLL never gets another chance to respawn it.

**The button has to be released before it is pressed.** `PlayerDeathThink` gates the `DEAD_DEAD` →
`DEAD_RESPAWNABLE` transition on *no* button being held (`player.cpp`, "wait for all buttons released"),
and only the following call, with `deadflag` already `DEAD_RESPAWNABLE`, reaches `respawn()`. Holding
`IN_ATTACK` on every dead frame — which is what `CHLDMProfile::Respawn` used to do — therefore parks the
bot at `DEAD_DEAD` permanently; it also never reaches the `mp_forcerespawn` branch further down, so
nothing else rescues it. `Respawn` now sends zero buttons until `deadflag == DEAD_RESPAWNABLE` and
`IN_ATTACK` on that frame alone.

Nothing resets the bot's plan across a death. It does not need to: the respawn moves the bot far enough
that `Bot_MoveNav`'s `kTeleportJumpThreshold` guard calls `ResetPath` on the first live frame, and when
the spawn point is *close* to where the bot died the stale plan still starts within that same distance and
is followed correctly. `hasGoal`/`goalPoint` survive on purpose, so a respawned bot resumes walking to the
goal it was given.

`crossbot_kill` exists to make this testable from the dedicated-server console.

A dead bot's only physics tick is the respawn frame. `pfnRunPlayerMove` receives one frame's worth of msec (not a token 1ms), and leaves `gpGlobals->frametime` holding whatever msec it was given. Every later bot in the frame would inherit that stale timing if nothing restored it, so the respawn is a full frame's worth to avoid propagating old command msec into the next bot's calculations.

### Level changes drop every bot, and that is the engine doing it

`SV_InactivateClients` disconnects fake clients outright ("Dropping fakeclient on level change"), which
runs `pfnClientDisconnect` and so clears the `CBot` slot. Bots are therefore gone after `changelevel` or
`map` and have to be re-added with `crossbot_add`; this is not a plugin bug and there is no roster to
restore. Re-adding on the new map — and after a plain `kick` — was verified to produce a fully working
bot, so the reused client slot and its leftover `CBasePlayer` private data need no special handling.

The two commands tear down in opposite orders, which matters to anything that ever wants to hook this:
`changelevel` runs `SV_InactivateClients` (so `ClientDisconnect`) *before* `pfnServerDeactivate`, while
`map` goes through `Host_ShutdownServer`, which calls `pfnServerDeactivate` first and drops the clients
after.

### PrintNavStatus
Backs `crossbot_nav_status`, and is the only view of what the runtime planner actually decided —
`--dump-path` shows what navgen would plan for its own spawn/target pair, which is usually a different
pair and often the opposite direction. Prints:
- Origin, velocity, angles, move speeds
- Goal state, target waypoint
- Elapsed time vs ETA (with OVERDUE flag)
- Planned and actual area paths (truncated)
- Full waypoint list with jump/crouch flags

The pairs that carry the diagnosis:
- `vel` against `move` — `move` is the forward/side pair last handed to `pfnRunPlayerMove`. Large
  `move` with `vel=(0,0,0)` is the bot pressing at full drive and not moving: it is against geometry,
  not lost and not idle.
- `planned=` against `actual=` — planned is the route's area sequence, actual is the areas walked.
  Actual diverging once is a repath; actual cycling is the air-move loop described under
  `RunAirmoveApproach`.
- `waypoint=i/n` against the waypoint list — which maneuver it is stuck before. `n` also reveals a
  repath, since a fresh plan renumbers from the bot's position.
- `hasGoal=0` with `waypoint=0/0` is ambiguous by itself: it is both "arrived and cleared the goal"
  and "gave up". Distinguish with the distance to the goal, and remember the goal was clamped
  (see `doc/path_planning.md`, `PlanPath`).
- `item=` with `itemArea=`, `memo=`, `full=` — the gather goal: which entity the bot is after, the
  area it resolved to, and how many entries are in each memo. `item=-` with `hasGoal=1` is a
  commanded `crossbot_nav_goal`, not a gathered one. `charging=` appears only while the charger
  runner owns the frame, and carries the value it is watching.
- `tier=` with `stopTier=`, `detour=`, `target=` and `targetArea=` — the goal hierarchy. `tier` is the
  distant target's tier, `stopTier` the current leg's, and they differ exactly when `detour=1`: a bot
  reading `tier=fun stopTier=gather detour=1` is on its way to a gimmick and picking something up en
  route, which is the intended shape and not a confusion. `tier` changing while `detour=1` holds is
  the broken-stickiness signature (`bot_goal.cpp`).
- `fun=` with `funKind=`, `funAct=`, `mounted=` and `shots=` — present only while the fun tier owns
  the goal. `mounted=1` with a nonzero `move=` is the 30-unit leash about to break.
- `combat=` with `enemy=`, `enemyAge=` and `ready=` — whether `BotCombat` owns the bot, who sense
  remembers, how stale the sighting is, and the bot's `EvaluateCombat` score. `combat=0` next to a
  live `enemy=` is normal and not a contradiction: sense remembers someone the bot never saw well
  enough to fight, or combat already exited on one of its own rules while the suspicion lingers.
  `ready=` no longer selects anything — it survives because carefulness still routes by it.
  `enemyAge=` above `crossbot_sense_decay` should never be seen; the record is dropped there.
  `crossbot_combat_status` is the fuller view.
- `breaking=` with `health=`, `swings=` and `for=` — present only while `BotBreak` owns the frame.
  `health` not falling across two samples while `swings` climbs is the unbreakable-glass signature; the
  no-damage grace will abort it a moment later.
- `avoid=` with `hit=` — the last obstacle decision (`duck`, `jump`, `side`, `blocked`, `break`) and the
  classname it was made against, plus `duckLeft` on the crawl latch, the `gap`/`top` the decision was
  made from, and `detour=` for how many spliced detour waypoints are still ahead. `avoid=none hit=-`
  on a bot stalled against something means the trace never qualified a hit, which is a different bug
  from a mistimed maneuver; `avoid=blocked` means it qualified and no route past it was found. Each
  new decision also prints its own line to the server console with distance, speed and trace count,
  which is what makes this tunable from one run — note those lines go to `qconsole.log`, not to the
  RCON transcript the test scripts capture.

## nav.cpp

### Nav namespace
Global mesh management:
- `LoadForMap(mapName)` - loads nav file for map
- `Unload()` - releases mesh
- `IsLoaded()` - checks if mesh loaded
- `Get()` - returns pointer to NavMesh
