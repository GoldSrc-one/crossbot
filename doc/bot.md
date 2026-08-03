# Bot System Comments

## bot_move.cpp

### Bot_DecomposeMove
PM_WalkMove and PM_AirMove both flatten forward/right and renormalize before use, so whatever pitch and roll the engine puts in pmove->angles drops out and the basis is just the yaw.

### Bot_FrameMsec
Frame time converted to milliseconds, clamped to [1, 100].

### Bot_NormalizeAngle
Wraps an angle into (-180, 180]. Lives here rather than in bot_move_nav.cpp so both movement
translation units share one copy.

## bot_move_nav.cpp

### Constants
- kFullDrive: Over maxspeed on purpose: PM_CheckParamters rescales the forward/side pair down to maxspeed, so this just means "full speed in exactly the commanded direction".
- kAimVelocitySpeed: Below this the velocity carries no usable heading, so the aim holds where it is rather than snapping about on numerical noise.
- kTeleportJumpThreshold: Distance threshold to detect teleport/jump and reset path.
- kRepathInterval: Interval for forced repath when stalled.
- kEtaSafetyMargin: Safety margin multiplier for ETA calculation.

### ResetPath
Clears all path state: waypoints, indices, timers, progress tracking, visited areas.

### WriteTraceLog
Writes bot trajectory trace to file specified by crossbot_trace_path cvar (defaults to valve/crossbot_data/nav_trace.txt).

### Repath
Finds path from current area to goal area using A* and builds waypoint plan.
- Clamps goal point into goal area bounds.
- On fresh attempt, calculates total path distance and estimated arrival time.
- Logs waypoint details including jump/crouch flags and target area IDs.

### Bot_MoveNav
Main navigation-driven movement function:
- Detects teleportation via origin jump and resets path if needed.
- Updates current area ID.
- If no goal, runs idle move.
- Triggers repath on empty path, forced repath (airmove landed off-target), or stall (no progress for kRepathInterval).
- Tracks visited areas for debug.
- Logs position to trace log (capped at 100k points).
- On waypoint completion: logs goal reached, writes trace, clears goal.
- Uses NavFollow::Steer for path following with funnel string-pulling.
- Handles jump waypoints as commitments - not passed until jump is made.
- Steers toward next waypoint for aiming (looking around corners).
- Computes move direction from the steered direction.
- Sets desired angles toward next waypoint or velocity direction.
- Decomposes move into forward/side speeds for RunPlayerMove.
- Sets movement buttons (forward/back, left/right, duck, jump).
- For jumps: checks on-ground, distance, facing tolerance; adds duck while rising.

## bot_manager.cpp

### AppendAreaList
Helper to format area ID lists for debug output, truncating to 24 entries with ellipsis.

### PrintNavStatus
Prints comprehensive bot status including:
- Origin, velocity, angles, move speeds
- Goal state, target waypoint
- Elapsed time vs ETA (with OVERDUE flag)
- Planned and actual area paths (truncated)
- Full waypoint list with jump/crouch flags

### PrintWaypoints
Prints detailed waypoint list for each bot.

## bot.h

### CBot struct
- Last forward/side pair handed to pfnRunPlayerMove, for crossbot_nav_status.

## bot_move.h

### Bot_DecomposeMove
Splits a world-space direction into the forward/side speeds pfnRunPlayerMove takes, which the engine reads along the view axes -- so this is what lets the bot move somewhere other than where it is looking. viewYawDeg must be the yaw passed to pfnRunPlayerMove in that same call plus punchangle, because PM_CheckParamters builds the movement basis from exactly that.

PM_CheckParamters clamps the pair down to maxspeed before anything reads it, so a driveLength well over maxspeed just means "full speed exactly this way", and one below it passes through untouched and becomes the actual speed.