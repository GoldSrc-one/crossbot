#pragma once

#include "bot.h"

// Drives one bot's movement using the loaded nav mesh: picks a random
// reachable area as a goal if the bot doesn't have one yet (the
// wander-equivalent default the plan calls for -- real goal selection is
// future combat/behavior AI), repaths and follows the funneled waypoint
// polyline via pfnRunPlayerMove, and special-cases ladder climbing using
// the engine's own on-ladder signal (pEdict->v.movetype == MOVETYPE_FLY)
// rather than trying to precompute which stretch of the waypoint list is
// "the ladder part" from static nav data. Only called when Nav::IsLoaded()
// (see Bot_Move's dispatch in bot_move.cpp).
void Bot_MoveNav(CBot &bot);

// Clears all per-attempt nav-path state (waypoints, jump/elevator/obstacle
// tracking, and the planned/visited area-path diagnostics). Exposed so
// BotManager::SetGoalForAllBots (crossbot_nav_goal) can reuse the same reset
// a fresh goal assignment already gets internally, instead of the caller
// hand-rolling a partial clear that drifts out of sync with it over time.
void Bot_ResetNavPath(CBot &bot);
