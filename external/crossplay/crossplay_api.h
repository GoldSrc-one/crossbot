#pragma once

#ifndef MAX_QPATH
#define MAX_QPATH 64
#endif

#ifndef MAX_EDICTS
#define MAX_EDICTS 2048
#endif

#ifndef CROSSPLAY_API
#define CROSSPLAY_API __declspec(dllimport)
#pragma comment( lib, "crossplay" )
#define CROSSPLAY_NAMESPACE namespace CrossPlay
#else
#define CROSSPLAY_NAMESPACE
#endif

extern "C" CROSSPLAY_NAMESPACE {
	const int maxGameDlls = 8;
	CROSSPLAY_API extern int gNumGameDlls;
	CROSSPLAY_API extern char gGameDirs[maxGameDlls][MAX_QPATH];
	CROSSPLAY_API extern int gEntityOwners[MAX_EDICTS];
	class CBaseEntity;
	CROSSPLAY_API extern CBaseEntity* gEntityObjects[maxGameDlls][MAX_EDICTS];

	CROSSPLAY_API void GameContext(int iGameDll, void (*callback)(void* callbackContext), void* callbackContext);
}