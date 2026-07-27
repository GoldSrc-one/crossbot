#pragma once

#include <extdll.h>

struct CBot;

namespace BotManager
{
	void ServerActivate(edict_t *pEdictList, int edictCount, int clientMax);
	void ServerDeactivate();
	void ClientPutInServer(edict_t *pEntity);
	void ClientDisconnect(edict_t *pEntity);
	void StartFrame();

	bool AddBot(const char *name);

	bool RemoveBot();

	int GetBotCount();

	bool SetGoalForAllBots(const Vector &point);

	bool TeleportAllBots(const Vector &point);

	void PrintNavStatus();

		void PrintWaypoints();

	int GetOtherBotEdicts(edict_t *self, edict_t **outEdicts, int maxCount);

	int GetBotsForDebug(const CBot **outBots, int maxCount);
}

