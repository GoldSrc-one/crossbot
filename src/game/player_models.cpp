#include <extdll.h>
#include <meta_api.h>

#include <cstdio>
#include <cstring>

#if defined(_WIN32)
#include <windows.h>
#else
#include <dirent.h>
#endif

#include "player_models.h"
#include "crossplay.h"

namespace
{

constexpr int kMaxModels = 64;
constexpr unsigned int kMaxNameLen = 32;

// The game every game borrows from when it has no player models of its own.
const char *const kFallbackDir = "valve";

struct ModelList
{
	bool built = false;
	int count = 0;
	char names[kMaxModels][kMaxNameLen] = {};
};

ModelList g_lists[XPlay::kMaxGames];

// models/player/<name>/<name>.mdl, probed the way FindMapBsp probes for a bsp. This doubles as the
// is-it-a-directory test -- readdir's d_type is DT_UNKNOWN on plenty of filesystems, and a plain
// file named "gordon" simply fails to open as "gordon/gordon.mdl".
bool HasModelFile(const char *root, const char *name)
{
	char path[512];
	snprintf(path, sizeof(path), "%s/models/player/%s/%s.mdl", root, name, name);

	FILE *f = fopen(path, "rb");
	if (!f)
		return false;

	fclose(f);
	return true;
}

void Add(ModelList &list, const char *root, const char *name)
{
	if (list.count >= kMaxModels)
		return;

	if (!name[0] || name[0] == '.' || strlen(name) >= kMaxNameLen)
		return;

	if (!HasModelFile(root, name))
		return;

	snprintf(list.names[list.count], kMaxNameLen, "%s", name);
	++list.count;
}

void ScanDir(ModelList &list, const char *root)
{
	char pattern[512];

#if defined(_WIN32)
	snprintf(pattern, sizeof(pattern), "%s/models/player/*", root);

	WIN32_FIND_DATAA find;
	HANDLE h = FindFirstFileA(pattern, &find);
	if (h == INVALID_HANDLE_VALUE)
		return;

	do
	{
		Add(list, root, find.cFileName);
	} while (list.count < kMaxModels && FindNextFileA(h, &find));

	FindClose(h);
#else
	snprintf(pattern, sizeof(pattern), "%s/models/player", root);

	DIR *dir = opendir(pattern);
	if (!dir)
		return;

	while (list.count < kMaxModels)
	{
		struct dirent *entry = readdir(dir);
		if (!entry)
			break;

		Add(list, root, entry->d_name);
	}

	closedir(dir);
#endif
}

const ModelList &ListFor(int iGame)
{
	static ModelList empty;
	if (iGame < 0 || iGame >= XPlay::kMaxGames)
		return empty;

	ModelList &list = g_lists[iGame];
	if (list.built)
		return list;

	list.built = true;

	const char *dir = XPlay::GameDir(iGame);
	if (dir && dir[0])
		ScanDir(list, dir);

	// dmc and anything else Quake-shaped lands here: no models/player of its own, so it wears
	// Half-Life's. CrossPlay expects exactly that -- its dmc model whitelist names valve models.
	if (list.count == 0 && (!dir || CB_STRICMP(dir, kFallbackDir) != 0))
		ScanDir(list, kFallbackDir);

	return list;
}

}

namespace PlayerModels
{

bool PickRandom(int iGame, char *out, unsigned int size)
{
	if (!out || size == 0)
		return false;

	out[0] = 0;

	const ModelList &list = ListFor(iGame);
	if (list.count == 0)
		return false;

	snprintf(out, size, "%s", list.names[RANDOM_LONG(0, list.count - 1)]);
	return true;
}

bool Installed(int iGame, const char *name)
{
	if (!name || !name[0])
		return false;

	const ModelList &list = ListFor(iGame);
	for (int i = 0; i < list.count; ++i)
	{
		if (CB_STRICMP(list.names[i], name) == 0)
			return true;
	}

	return false;
}

void PrintList(int iGame)
{
	const ModelList &list = ListFor(iGame);
	if (list.count == 0)
	{
		SERVER_PRINT("  (none installed)\n");
		return;
	}

	char line[64];
	for (int i = 0; i < list.count; ++i)
	{
		snprintf(line, sizeof(line), "  %s\n", list.names[i]);
		SERVER_PRINT(line);
	}
}

void Forget()
{
	for (ModelList &list : g_lists)
	{
		list.built = false;
		list.count = 0;
	}
}

}
