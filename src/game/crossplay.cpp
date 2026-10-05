#include <extdll.h>
#include <meta_api.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "crossplay.h"
#include "crossplay_api.h"
#include "gamedata.h"
#include "game_profile.h"

namespace
{

static_assert(XPlay::kMaxGames == CrossPlay::maxGameDlls, "XPlay::kMaxGames must match CrossPlay::maxGameDlls");

int g_current = -1;
IGameProfile *g_profiles[XPlay::kMaxGames] = {};

}

namespace XPlay
{

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

int FindGame(const char *dir)
{
	if (!dir || !dir[0])
		return -1;

	for (int i = 0; i < NumGames(); ++i)
	{
		if (CB_STRICMP(CrossPlay::gGameDirs[i], dir) == 0)
			return i;
	}
	return -1;
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

const char *HyperDir()
{
	static char hyperdir[256];
	if (!hyperdir[0])
		g_engfuncs.pfnGetGameDir(hyperdir);

	return hyperdir;
}

bool FindMapBsp(const char *mapName, char *out, unsigned int size)
{
	const char *dirs[XPlay::kMaxGames + 2];
	int count = 0;

	int owner = MapOwner();
	if (owner >= 0 && owner < NumGames())
		dirs[count++] = CrossPlay::gGameDirs[owner];

	for (int i = 0; i < NumGames(); ++i)
	{
		if (i != owner)
			dirs[count++] = CrossPlay::gGameDirs[i];
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

void RunInGame(int iGame, void (*callback)(void *context), void *context)
{
	if (iGame < 0 || iGame >= NumGames())
		return;

	CrossPlay::GameContext(iGame, callback, context);
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
		g_profiles[i] = GameProfile_Detect(CrossPlay::gGameDirs[i]);

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
