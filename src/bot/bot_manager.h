#pragma once

#include <extdll.h>

struct CBot;

// Bot lifecycle, called from the metamod dll hooks in src/meta/dll_hooks.cpp
// and from the crossbot_* console commands in src/console/commands.cpp.
namespace BotManager
{
	void ServerActivate(edict_t *pEdictList, int edictCount, int clientMax);
	void ServerDeactivate();
	void ClientPutInServer(edict_t *pEntity);
	void ClientDisconnect(edict_t *pEntity);
	void StartFrame();

	// Creates one fake-client bot with the given name. Returns false if no
	// game profile was detected, no slot is free, or the engine/gamedll
	// refused the connection.
	bool AddBot(const char *name);

	// Kicks one connected bot (if any). Returns false if none are connected.
	bool RemoveBot();

	int GetBotCount();

	// Dev/testing hook for crossbot_nav_goal (src/console/commands.cpp):
	// forces every connected bot's nav goal to `point`, dropping whatever
	// path it was already following so it repaths immediately. Returns
	// false if no bots are connected. Has no effect on a bot currently
	// falling back to wander (no .nav file loaded for this map).
	bool SetGoalForAllBots(const Vector &point);

	// Dev/testing hook for crossbot_teleport: instantly relocates every
	// connected bot to `point` and zeroes its velocity. Bot_MoveNav's
	// existing teleport-jump detection (kTeleportJumpThreshold in
	// bot_move_nav.cpp) picks up the discontinuity on its own next frame
	// and drops any in-progress path, so no separate path-reset call is
	// needed here. Meant for jumping straight to a specific problem area
	// during live testing instead of waiting for a bot to walk there from
	// spawn every time. Returns false if no bots are connected.
	bool TeleportAllBots(const Vector &point);

	// Dev/testing hook for crossbot_nav_status: logs each connected bot's
	// current origin and waypoint progress via ALERT(at_logged, ...), so
	// path-following behavior can be observed in qconsole.log over time.
	void PrintNavStatus();

	// Local-avoidance hook (src/nav/nav_avoid.cpp, Milestone 2d): copies the
	// edict of every other in-use bot (excluding `self`) into `outEdicts`,
	// up to `maxCount` entries. Returns the number written. Bounded by
	// kMaxBots (16) so a fixed-size stack array at the call site is enough --
	// no allocation needed.
	int GetOtherBotEdicts(edict_t *self, edict_t **outEdicts, int maxCount);

	// Read-only debug hook (src/nav/nav_debug.cpp): copies pointers to every
	// in-use bot's full state, including its current waypoint path, into
	// outBots (up to maxCount entries). Returns the number written. Unlike
	// GetOtherBotEdicts this exposes the whole CBot, not just its edict,
	// since path-beam drawing needs the waypoint list itself.
	int GetBotsForDebug(const CBot **outBots, int maxCount);
}
