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

	CBot *FindBot(edict_t *pEdict);

	void Touch(edict_t *pentTouched, edict_t *pentOther);

	bool AddBot(const char *name);

	bool SetGoalForAllBots(const Vector &point);

	bool TeleportAllBots(const Vector &point);

	bool KillAllBots();

	void PrintNavStatus();

	void PrintWaypoints();
}

