#include <extdll.h>
#include <meta_api.h>

#include <cstdio>
#include <cstring>

#include "teams.h"
#include "crossplay.h"

namespace
{

const char *const kTeamGameDirs[] = { "cstrike", "czero", "tfc", "dod" };

char g_mapGameDir[64] = "?";
bool g_mapTeamplay = false;

bool IsTeamGameDir(const char *dir)
{
	if (!dir || !dir[0])
		return false;

	for (const char *known : kTeamGameDirs)
	{
		if (CB_STRICMP(dir, known) == 0)
			return true;
	}
	return false;
}

}

namespace Teams
{

void DetectMap()
{
	const char *dir = XPlay::GameDir(XPlay::MapOwner());
	snprintf(g_mapGameDir, sizeof(g_mapGameDir), "%s", dir ? dir : "?");
	g_mapTeamplay = IsTeamGameDir(g_mapGameDir);

	char msg[160];
	snprintf(msg, sizeof(msg), "CrossBot: map belongs to '%s'; teams are %s.\n",
		g_mapGameDir, g_mapTeamplay ? "on" : "off");
	SERVER_PRINT(msg);
}

void ForgetMap()
{
	snprintf(g_mapGameDir, sizeof(g_mapGameDir), "?");
	g_mapTeamplay = false;
}

bool MapTeamplay()
{
	return g_mapTeamplay;
}

const char *MapGameDir()
{
	return g_mapGameDir;
}

bool Enabled()
{
	if (!g_mapTeamplay)
		return false;

	cvar_t *cvar = g_engfuncs.pfnCVarGetPointer("crossbot_teams");
	return !cvar || cvar->value != 0.f;
}

int TeamOf(const edict_t *ent)
{
	if (!ent || ent->free)
		return 0;

	if (ent->v.team != 0)
		return ent->v.team;

	const edict_t *owner = ent->v.owner;
	if (!owner || owner->free || owner == ent)
		return 0;

	return owner->v.team;
}

bool Active(const edict_t *self)
{
	return Enabled() && TeamOf(self) != 0;
}

Relation Of(const edict_t *self, const edict_t *ent)
{
	if (!ent || ent->free || !Enabled())
		return REL_UNKNOWN;

	int mine = TeamOf(self);
	if (mine == 0)
		return REL_UNKNOWN;

	int theirs = TeamOf(ent);
	if (theirs == 0)
		return REL_UNKNOWN;

	return (theirs == mine) ? REL_FRIEND : REL_ENEMY;
}

bool Attackable(const edict_t *self, const edict_t *ent)
{
	if (!Active(self))
		return true;

	return Of(self, ent) == REL_ENEMY;
}

const char *RelationName(Relation rel)
{
	switch (rel)
	{
	case REL_FRIEND: return "friend";
	case REL_ENEMY: return "enemy";
	default: return "unknown";
	}
}

}
