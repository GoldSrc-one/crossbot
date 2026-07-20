#include <extdll.h>
#include <meta_api.h>

#include "bot_manager.h"
#include "bot.h"
#include "bot_move.h"
#include "../game/game_profile.h"

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

	if (g_pGameProfile)
		ALERT(at_logged, "CrossBot: detected game profile '%s'.\n", g_pGameProfile->GetName());
	else
		ALERT(at_logged, "CrossBot: no known game profile matched this mod; bots disabled.\n");
}

void ServerDeactivate()
{
	for (auto &bot : g_bots)
		bot = CBot();

	g_pGameProfile = nullptr;
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
		ALERT(at_logged, "CrossBot: cannot add bot, no game profile detected.\n");
		return false;
	}

	CBot *pSlot = FindFreeSlot();
	if (!pSlot)
	{
		ALERT(at_logged, "CrossBot: cannot add bot, all %d slots in use.\n", kMaxBots);
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
		ALERT(at_logged, "CrossBot: engine refused to create a fake client.\n");
		return false;
	}

	// Bots don't go through the engine's normal client-connect path, so we
	// have to replay it ourselves. MDLL_* talks directly to the gamedll,
	// bypassing metamod's hook chain (see meta_api.h).
	char rejectReason[128] = {};
	MDLL_ClientConnect(pEdict, botName, "127.0.0.1", rejectReason);

	if (rejectReason[0])
	{
		ALERT(at_logged, "CrossBot: game rejected bot connection: %s\n", rejectReason);
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

} // namespace BotManager
