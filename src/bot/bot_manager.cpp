#include <extdll.h>
#include <meta_api.h>

#include "bot_manager.h"
#include "bot.h"
#include "bot_move.h"
#include "../game/game_profile.h"
#include "../nav/nav.h"

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

	// SERVER_PRINT (pfnServerPrint), not ALERT(at_logged, ...): at_logged
	// routes to the server's *.log file (sv_logfile), not the console --
	// confirmed the hard way debugging this milestone's live test (every
	// ALERT(at_logged, ...) call was silently invisible in qconsole.log
	// even with `developer 2`, while pfnServerPrint always showed up).
	// Dev-facing status like this needs to reliably show up wherever
	// someone's actually looking.
	char msg[256];
	if (g_pGameProfile)
		snprintf(msg, sizeof(msg), "CrossBot: detected game profile '%s'.\n", g_pGameProfile->GetName());
	else
		snprintf(msg, sizeof(msg), "CrossBot: no known game profile matched this mod; bots disabled.\n");
	SERVER_PRINT(msg);

	const char *mapName = g_engfuncs.pfnSzFromIndex(gpGlobals->mapname);
	Nav::LoadForMap(mapName);
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
	// Bots are put in the server directly by AddBot() below (via the
	// MDLL_ macros, which bypass this metamod chain), so this only fires
	// for human players joining through the normal engine path. No
	// behavior needed for this milestone; kept as an extension point for
	// future auto-fill / team-balance logic.
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

	// Bots don't go through the engine's normal client-connect path, so we
	// have to replay it ourselves. MDLL_* talks directly to the gamedll,
	// bypassing metamod's hook chain (see meta_api.h).
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
		bot.waypoints.clear();
		bot.waypointIndex = 0;
		any = true;
	}
	return any;
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
				wp.requiresJump ? "[JUMP]" : "");
		}
		char msg[400];
		snprintf(msg, sizeof(msg),
			"CrossBot: %s origin=(%.0f,%.0f,%.0f) vel=(%.0f,%.0f,%.0f) angles=(%.0f,%.0f) hasGoal=%d "
			"goal=(%.0f,%.0f,%.0f) target=%s waypoint=%zu/%zu movetype=%d\n",
			bot.name, origin.x, origin.y, origin.z, vel.x, vel.y, vel.z, angles.x, angles.y,
			bot.hasGoal ? 1 : 0, bot.goalPoint.x, bot.goalPoint.y, bot.goalPoint.z, targetBuf,
			bot.waypointIndex, bot.waypoints.size(), bot.pEdict->v.movetype);
		SERVER_PRINT(msg);
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

} // namespace BotManager
