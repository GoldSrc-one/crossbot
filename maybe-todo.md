doors, plats, trains... move them to their other positions (time) and run airmove sampling out of them too, note the time into the move. For reverse jump edges, note the time too.

LADDERS
Planner done -- A* expands CONN_LADDER in both builders, PlanWalker::Cross emits a mount waypoint (requiresLadder, ladderUp, ladderYaw, ladderSpeed, ladderExit) plus an exit waypoint, and path_profile treats the climb as a gap. --dump-path shows the climb on test12/test25/test53.

Follower done -- RunLadderApproach/RunLadderMove drive the climb, descent and all four dismounts; test12/test25/test29/test53 all pass end to end. The volumes are carved out of the walkable leaves in navgen, so no nav area contains a ladder-capture point and routing past one is structural. See doc/bot.md "Ladders" and doc/navgen.md.

Ladder volumes touching each other are still separate func_ladder entities and are never merged, so a ladder built from two brushes gets two independent mounts.

The planner's launch re-simulation is not ladder-aware. The sampler now ends a flight that enters a func_ladder (PM_MoveSimulator::OnLadder), but JumpLandsIn only has PlanTraceContext's trace functions and the .nav carries no ladder volumes, so a re-sim launched from a different point on the gate can still fly through a ladder and claim a landing the bot will never reach. Needs the volumes in the nav format.

sidemove sampling + yawrate in airmove ;-)

buttons and activators for doors and breakables -- the path finder should have a parameter "should I try to find the activators?" which would be ON by default. When the path finder would hit a door or a breakable that needs activation and this param would be OFF, the connection through that obstacle wouldn't be considered; if it were ON, a new, recursive path find would be launched from the starting point (or last activator position) to the activator position. If that path couldn't be found, the connection through the obstacle wouldn't be considered. If such path would be found, the path finding would continue from the sub-path end (the activator position).
Alternative solution -- let the activations be a part of the search state. If during the search an activator would be found, this would allow for re-visit of previsouly visited areas. And the doors and breakables would be considered unpassable (if they require activation and are not already open or broken) unless the correct activation happened on the path. It could be like a... 128 bitfield? That should be enough to cover all possible activations in a map (distinct by name, mapped to bits).

integrate namegiver
rename to goldclanker, .nav to .gcn
