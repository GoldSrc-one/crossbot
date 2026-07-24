#pragma once

#include <extdll.h>

struct CBot;

// Bot lifecycle, called from metamod dll hooks (src/meta/dll_hooks.cpp) and crossbot_* console commands (src/console/commands.cpp).
namespace BotManager
{
	void ServerActivate(edict_t *pEdictList, int edictCount, int clientMax);
	void ServerDeactivate();
	void ClientPutInServer(edict_t *pEntity);
	void ClientDisconnect(edict_t *pEntity);
	void StartFrame();

	// Creates one fake-client bot with given name. Returns false if no game profile detected, no slot free, or engine/gamedll refused connection.
	bool AddBot(const char *name);

	// Kicks one connected bot (if any). Returns false if none connected.
	bool RemoveBot();

	int GetBotCount();

	// Dev/testing hook for crossbot_nav_goal: forces every connected bot's nav goal to `point`, dropping current path so it repaths immediately. Returns false if no bots connected. No effect on bot falling back to wander (no .nav file).
	bool SetGoalForAllBots(const Vector &point);

	// Dev/testing hook for crossbot_teleport: instantly relocates every connected bot to `point` and zeroes velocity. Bot_MoveNav's teleport-jump detection (kTeleportJumpThreshold in bot_move_nav.cpp) picks up discontinuity next frame and drops in-progress path. Meant for jumping to specific problem area during live testing. Returns false if no bots connected.
	bool TeleportAllBots(const Vector &point);

	// Dev/testing hook for crossbot_nav_status: logs each connected bot's origin and waypoint progress via ALERT(at_logged, ...), so path-following behavior observable in qconsole.log over time.
	void PrintNavStatus();

	// Local-avoidance hook (nav_avoid.cpp, Milestone 2d): copies edict of every other in-use bot (excluding `self`) into `outEdicts`, up to `maxCount` entries. Returns number written. Bounded by kMaxBots (16) so fixed-size stack array at call site is enough -- no allocation.
	int GetOtherBotEdicts(edict_t *self, edict_t **outEdicts, int maxCount);

	// Read-only debug hook (nav_debug.cpp): copies pointers to every in-use bot's full state (including waypoint path) into outBots (up to maxCount entries). Returns number written. Unlike GetOtherBotEdicts this exposes whole CBot, not just edict, since path-beam drawing needs waypoint list itself.
	int GetBotsForDebug(const CBot **outBots, int maxCount);
}
