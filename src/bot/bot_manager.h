#pragma once

#include <extdll.h>

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
}
