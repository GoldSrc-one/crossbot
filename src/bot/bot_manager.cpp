#include <extdll.h>
#include <meta_api.h>

#include "bot_manager.h"
#include "bot.h"
#include "bot_move.h"
#include "bot_move_nav.h"
#include "../game/game_profile.h"
#include "../nav/nav.h"
#include "../nav/nav_debug.h"

#include <string>

namespace
{
	constexpr int kMaxBots = 16;

	CBot g_bots[kMaxBots];
	IGameProfile *g_pGameProfile = nullptr;
	int g_botSerial = 0;

	CBot *FindBotByEdict(edict_t *pEdict)
	{
		for (auto &bot : g_bots)
		{
			if (bot.inUse && bot.pEdict == pEdict)
				return &bot;
		}
		return nullptr;
	}

	CBot *FindFreeSlot()
	{
		for (auto &bot : g_bots)
		{
			if (!bot.inUse)
				return &bot;
		}
		return nullptr;
	}
}

namespace BotManager
{

void ServerActivate(edict_t *pEdictList, int edictCount, int clientMax)
{
	(void)pEdictList;
	(void)edictCount;
	(void)clientMax;

	for (auto &bot : g_bots)
		bot = CBot();

	g_pGameProfile = GameProfile_Detect();

	// SERVER_PRINT (pfnServerPrint), not ALERT(at_logged, ...): at_logged routes to server's *.log file (sv_logfile), not console -- confirmed hard way debugging this milestone's live test (every ALERT(at_logged, ...) was silently invisible in qconsole.log even with `developer 2`, while pfnServerPrint always showed up). Dev-facing status needs to show up where someone's actually looking.
	char msg[256];
	if (g_pGameProfile)
		snprintf(msg, sizeof(msg), "CrossBot: detected game profile '%s'.\n", g_pGameProfile->GetName());
	else
		snprintf(msg, sizeof(msg), "CrossBot: no known game profile matched this mod; bots disabled.\n");
	SERVER_PRINT(msg);

	const char *mapName = g_engfuncs.pfnSzFromIndex(gpGlobals->mapname);
	Nav::LoadForMap(mapName);

	NavDebug::Precache();
}

void ServerDeactivate()
{
	for (auto &bot : g_bots)
		bot = CBot();

	g_pGameProfile = nullptr;
	Nav::Unload();
}

void ClientPutInServer(edict_t *pEntity)
{
	// Bots added directly by AddBot() below (via MDLL_ macros bypassing this metamod chain), so this only fires for human players joining through normal engine path. No behavior needed for this milestone; extension point for future auto-fill/team-balance logic.
	(void)pEntity;
}

void ClientDisconnect(edict_t *pEntity)
{
	CBot *pBot = FindBotByEdict(pEntity);
	if (pBot)
		*pBot = CBot();
}

void StartFrame()
{
	NavDebug::Draw();

	if (!g_pGameProfile)
		return;

	for (auto &bot : g_bots)
	{
		if (!bot.inUse)
			continue;

		if (g_pGameProfile->IsDead(bot.pEdict))
			g_pGameProfile->Respawn(bot.pEdict);
		else
			Bot_Move(bot);
	}
}

bool AddBot(const char *name)
{
	if (!g_pGameProfile)
	{
		SERVER_PRINT("CrossBot: cannot add bot, no game profile detected.\n");
		return false;
	}

	CBot *pSlot = FindFreeSlot();
	if (!pSlot)
	{
		char msg[64];
		snprintf(msg, sizeof(msg), "CrossBot: cannot add bot, all %d slots in use.\n", kMaxBots);
		SERVER_PRINT(msg);
		return false;
	}

	char botName[32];
	if (name && name[0])
		snprintf(botName, sizeof(botName), "%s", name);
	else
		snprintf(botName, sizeof(botName), "CrossBot%d", ++g_botSerial);

	edict_t *pEdict = CREATE_FAKE_CLIENT(botName);
	if (!pEdict)
	{
		SERVER_PRINT("CrossBot: engine refused to create a fake client.\n");
		return false;
	}

	// Bots don't go through engine's normal client-connect path, so we replay it ourselves. MDLL_* talks directly to gamedll, bypassing metamod's hook chain (see meta_api.h).
		char rejectReason[128] = {};
	MDLL_ClientConnect(pEdict, botName, "127.0.0.1", rejectReason);

	if (rejectReason[0])
	{
		char msg[192];
		snprintf(msg, sizeof(msg), "CrossBot: game rejected bot connection: %s\n", rejectReason);
		SERVER_PRINT(msg);
		return false;
	}

	MDLL_ClientPutInServer(pEdict);
	pEdict->v.flags |= FL_CLIENT | FL_FAKECLIENT;

	pSlot->inUse = true;
	pSlot->pEdict = pEdict;
	snprintf(pSlot->name, sizeof(pSlot->name), "%s", botName);
	pSlot->nextDirChangeTime = 0.f;

	return true;
}

bool RemoveBot()
{
	for (auto &bot : g_bots)
	{
		if (!bot.inUse)
			continue;

		char cmd[64];
		snprintf(cmd, sizeof(cmd), "kick \"%s\"\n", bot.name);
		SERVER_COMMAND(cmd);
		SERVER_EXECUTE();
		return true;
	}
	return false;
}

int GetBotCount()
{
	int count = 0;
	for (auto &bot : g_bots)
	{
		if (bot.inUse)
			++count;
	}
	return count;
}

bool SetGoalForAllBots(const Vector &point)
{
	bool any = false;
	for (auto &bot : g_bots)
	{
		if (!bot.inUse)
			continue;
		bot.goalPoint = point;
		bot.hasGoal = true;
		Bot_ResetNavPath(bot); // also clears elevator/jump/obstacle state and ETA/area-trace diagnostics
		bot.traceLog.clear();  // fresh origin recording for this goal (see CBot::traceLog)
		any = true;
	}
	return any;
}

bool TeleportAllBots(const Vector &point)
{
	bool any = false;
	for (auto &bot : g_bots)
	{
		if (!bot.inUse)
			continue;
		// SET_ORIGIN (not raw v.origin write) so engine properly relinks edict's collision/PVS position -- direct struct write leaves stale spatial-partition data until next physics step touches it.
		SET_ORIGIN(bot.pEdict, point);
		bot.pEdict->v.velocity = Vector(0.f, 0.f, 0.f);
		any = true;
	}
	return any;
}

namespace
{
	// Appends "<label>=[a,b,c,...]" to msg, capped at kMaxShown entries so long corridor can't blow past qconsole's line-length handling. Planned paths shown from *front* (what's still ahead); visited-history from *back* (where bot ended up) -- see fromTail param.
	void AppendAreaList(std::string &msg, const char *label, const std::vector<uint32_t> &ids, bool fromTail)
	{
		constexpr size_t kMaxShown = 24;
		msg += " ";
		msg += label;
		msg += "=[";

		size_t begin = 0;
		size_t end = ids.size();
		bool truncated = ids.size() > kMaxShown;
		if (truncated)
		{
			if (fromTail)
				begin = ids.size() - kMaxShown;
			else
				end = kMaxShown;
		}

		if (truncated && fromTail)
			msg += "...,";
		char idBuf[16];
		for (size_t i = begin; i < end; ++i)
		{
			snprintf(idBuf, sizeof(idBuf), "%u", ids[i]);
			msg += idBuf;
			if (i + 1 < end)
				msg += ",";
		}
		if (truncated && !fromTail)
			msg += ",...";
		msg += "]";
	}
}

void PrintNavStatus()
{
	for (auto &bot : g_bots)
	{
		if (!bot.inUse)
			continue;
		Vector origin = bot.pEdict->v.origin;
		Vector vel = bot.pEdict->v.velocity;
		Vector angles = bot.pEdict->v.v_angle;
		char targetBuf[72] = "(none)";
		if (bot.waypointIndex < bot.waypoints.size())
		{
			const Nav::Waypoint &wp = bot.waypoints[bot.waypointIndex];
			snprintf(targetBuf, sizeof(targetBuf), "(%.0f,%.0f,%.0f)%s", wp.point.x, wp.point.y, wp.point.z,
				wp.requiresJump ? "[JUMP]" : wp.requiresElevatorRide ? "[ELEVATOR]" : "");
		}

		bool onGround = (bot.pEdict->v.flags & FL_ONGROUND) != 0;

		char line[400];
		snprintf(line, sizeof(line),
			"CrossBot: %s origin=(%.0f,%.0f,%.0f) vel=(%.0f,%.0f,%.0f) angles=(%.0f,%.0f) hasGoal=%d "
			"goal=(%.0f,%.0f,%.0f) target=%s waypoint=%zu/%zu movetype=%d area=%d onground=%d",
			bot.name, origin.x, origin.y, origin.z, vel.x, vel.y, vel.z, angles.x, angles.y,
			bot.hasGoal ? 1 : 0, bot.goalPoint.x, bot.goalPoint.y, bot.goalPoint.z, targetBuf,
			bot.waypointIndex, bot.waypoints.size(), bot.pEdict->v.movetype, bot.currentAreaId, onGround ? 1 : 0);
		std::string msg = line;

		// ETA only means anything once a real path has actually been found
		// for the current goal (estimatedArrivalTime stays 0 until then).
		if (bot.hasGoal && bot.estimatedArrivalTime > 0.f)
		{
			float elapsed = gpGlobals->time - bot.pathStartTime;
			float budget = bot.estimatedArrivalTime - bot.pathStartTime;
			bool overdue = gpGlobals->time > bot.estimatedArrivalTime;
			snprintf(line, sizeof(line), " elapsed=%.1fs/eta=%.1fs%s", elapsed, budget, overdue ? " OVERDUE" : "");
			msg += line;
		}

		AppendAreaList(msg, "planned", bot.plannedAreaPath, false);
		AppendAreaList(msg, "actual", bot.visitedAreaPath, true);

		msg += "\n";
		SERVER_PRINT(msg.c_str());
	}
}

int GetOtherBotEdicts(edict_t *self, edict_t **outEdicts, int maxCount)
{
	int count = 0;
	for (auto &bot : g_bots)
	{
		if (count >= maxCount)
			break;
		if (!bot.inUse || bot.pEdict == self)
			continue;
		outEdicts[count++] = bot.pEdict;
	}
	return count;
}

int GetBotsForDebug(const CBot **outBots, int maxCount)
{
	int count = 0;
	for (auto &bot : g_bots)
	{
		if (count >= maxCount)
			break;
		if (!bot.inUse)
			continue;
		outBots[count++] = &bot;
	}
	return count;
}

} // namespace BotManager
