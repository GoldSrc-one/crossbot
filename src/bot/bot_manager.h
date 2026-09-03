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

	void PrintAimStatus();

	void PrintSenseStatus();

	void PrintVantageStatus(const Vector *at);

	void PrintCombatStatus();

	void PrintFireStatus();

	void PrintDeployStatus();

	void PrintWeaponCarry();

	void OnSound(edict_t *src, int channel, const char *sample, float volume, float attenuation);

	void OnEvent(int flags, const edict_t *pInvoker);

	void PrintWaypoints();
}

