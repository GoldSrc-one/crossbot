#include <extdll.h>
#include <meta_api.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>

#if defined(_WIN32)
#include <windows.h>
#endif
#include <FileSystem.h>

#include "crossplay.h"
#ifndef CROSSBOT_STANDALONE
#include "crossplay_api.h"
#endif
#include "gamedata.h"
#include "game_profile.h"

namespace
{

#ifndef CROSSBOT_STANDALONE
static_assert(XPlay::kMaxGames == CrossPlay::maxGameDlls, "XPlay::kMaxGames must match CrossPlay::maxGameDlls");
#endif

int g_current = -1;
IGameProfile *g_profiles[XPlay::kMaxGames] = {};

IFileSystem *EngineFileSystem()
{
#if defined(_WIN32)
	HMODULE module = GetModuleHandleA("filesystem_stdio.dll");
#else
	void *module = dlopen("filesystem_stdio.so", RTLD_NOW | RTLD_NOLOAD);
#endif
	if (!module)
		return nullptr;

	auto createInterface = reinterpret_cast<CreateInterfaceFn>(GetProcAddress(module, CREATEINTERFACE_PROCNAME));
	if (!createInterface)
		return nullptr;

	return static_cast<IFileSystem *>(createInterface(FILESYSTEM_INTERFACE_VERSION, nullptr));
}

}

namespace XPlay
{

#ifdef CROSSBOT_STANDALONE

int NumGames()
{
	return 1;
}

const char *GameDir(int iGame)
{
	return iGame == 0 ? GET_GAME_INFO(PLID, GINFO_NAME) : "?";
}

int MapOwner()
{
	return 0;
}

int OwnerOf(const edict_t *ent)
{
	return ent && !ent->free && ent->pvPrivateData ? 0 : -1;
}

void *PrivateData(const edict_t *ent, int iGame)
{
	return ent && !ent->free && iGame == 0 ? ent->pvPrivateData : nullptr;
}

void RunInGame(int iGame, void (*callback)(void *context), void *context)
{
	if (iGame == 0)
		callback(context);
}

#else

int NumGames()
{
	int n = CrossPlay::gNumGameDlls;
	if (n < 0)
		return 0;
	return n > XPlay::kMaxGames ? XPlay::kMaxGames : n;
}

const char *GameDir(int iGame)
{
	if (iGame < 0 || iGame >= NumGames())
		return "?";
	return CrossPlay::gGameDirs[iGame];
}

int MapOwner()
{
	return CrossPlay::gEntityOwners[0];
}

int OwnerOf(const edict_t *ent)
{
	if (!ent)
		return -1;

	int index = g_engfuncs.pfnIndexOfEdict(ent);
	if (index < 0 || index >= MAX_EDICTS)
		return -1;

	return CrossPlay::gEntityOwners[index];
}

void *PrivateData(const edict_t *ent, int iGame)
{
	if (!ent || ent->free || iGame < 0 || iGame >= NumGames())
		return nullptr;

	int index = g_engfuncs.pfnIndexOfEdict(ent);
	if (index < 0 || index >= MAX_EDICTS)
		return nullptr;

	return CrossPlay::gEntityObjects[iGame][index];
}

void RunInGame(int iGame, void (*callback)(void *context), void *context)
{
	if (iGame < 0 || iGame >= NumGames())
		return;

	CrossPlay::GameContext(iGame, callback, context);
}

#endif

int FindGame(const char *dir)
{
	if (!dir || !dir[0])
		return -1;

	for (int i = 0; i < NumGames(); ++i)
	{
		if (CB_STRICMP(GameDir(i), dir) == 0)
			return i;
	}
	return -1;
}

const char *HyperDir()
{
	static char hyperdir[256];
	if (!hyperdir[0])
		g_engfuncs.pfnGetGameDir(hyperdir);

	return hyperdir;
}

bool FindMapBsp(const char *mapName, char *out, unsigned int size)
{
	char relPath[256];
	snprintf(relPath, sizeof(relPath), "maps/%s.bsp", mapName);
	IFileSystem *fileSystem = EngineFileSystem();
	if (fileSystem && fileSystem->GetLocalPath(relPath, out, static_cast<int>(size)))
		return true;

	const char *dirs[XPlay::kMaxGames + 2];
	int count = 0;

	int owner = MapOwner();
	if (owner >= 0 && owner < NumGames())
		dirs[count++] = GameDir(owner);

	for (int i = 0; i < NumGames(); ++i)
	{
		if (i != owner)
			dirs[count++] = GameDir(i);
	}

	dirs[count++] = HyperDir();

	for (int i = 0; i < count; ++i)
	{
		snprintf(out, size, "%s/maps/%s.bsp", dirs[i], mapName);

		FILE *f = fopen(out, "rb");
		if (f)
		{
			fclose(f);
			return true;
		}
	}

	snprintf(out, size, "%s/maps/%s.bsp", dirs[0], mapName);
	return false;
}

int Current()
{
	return g_current;
}

void EnterGame(int iGame)
{
	if (iGame < 0 || iGame >= NumGames())
	{
		LeaveGame();
		return;
	}
	if (iGame == g_current && g_pGameProfile == g_profiles[iGame])
		return;

	g_current = iGame;
	GameData::Activate(iGame);
	g_pGameProfile = g_profiles[iGame];
}

void LeaveGame()
{
	g_current = -1;
	g_pGameProfile = nullptr;
}

bool Supported(int iGame)
{
	if (iGame < 0 || iGame >= NumGames())
		return false;

	return GameData::Loaded(iGame) && g_profiles[iGame] != nullptr;
}

int PickRandomSupported()
{
	int supported[XPlay::kMaxGames];
	int count = 0;

	for (int i = 0; i < NumGames(); ++i)
	{
		if (Supported(i))
			supported[count++] = i;
	}

	if (!count)
		return -1;

	return supported[RANDOM_LONG(0, count - 1)];
}

IGameProfile *ProfileFor(int iGame)
{
	if (iGame < 0 || iGame >= NumGames())
		return nullptr;

	return g_profiles[iGame];
}

void DetectProfiles()
{
	for (int i = 0; i < XPlay::kMaxGames; ++i)
		g_profiles[i] = nullptr;

	for (int i = 0; i < NumGames(); ++i)
		g_profiles[i] = GameProfile_Detect(GameDir(i));

	char msg[256];
	for (int i = 0; i < NumGames(); ++i)
	{
		if (Supported(i))
			snprintf(msg, sizeof(msg), "CrossBot: game #%d '%s' ready (profile '%s').\n",
				i, GameDir(i), g_profiles[i]->GetName());
		else if (!g_profiles[i])
			snprintf(msg, sizeof(msg), "CrossBot: game #%d '%s' has no game profile; no bots there.\n",
				i, GameDir(i));
		else
			snprintf(msg, sizeof(msg), "CrossBot: game #%d '%s' has no usable gamedata; no bots there.\n",
				i, GameDir(i));
		SERVER_PRINT(msg);
	}
}

void ForgetProfiles()
{
	for (int i = 0; i < XPlay::kMaxGames; ++i)
		g_profiles[i] = nullptr;

	LeaveGame();
}

}
