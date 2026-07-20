#include <extdll.h>
#include <meta_api.h>
#include <in_buttons.h>

#include "hldm_profile.h"

#if defined(_WIN32)
#define CB_STRICMP _stricmp
#else
#define CB_STRICMP strcasecmp
#endif

const char *CHLDMProfile::GetName() const
{
	return "HLDM";
}

unsigned int CHLDMProfile::GetFlags() const
{
	return GAME_DEATHMATCH;
}

bool CHLDMProfile::Detect() const
{
	if (!gpGlobals->deathmatch_)
		return false;

	char gamedir[256] = { 0 };
	g_engfuncs.pfnGetGameDir(gamedir);
	return CB_STRICMP(gamedir, "valve") == 0;
}

bool CHLDMProfile::IsDead(edict_t *pEdict) const
{
	return pEdict->v.deadflag != DEAD_NO || pEdict->v.health <= 0.0f;
}

void CHLDMProfile::Respawn(edict_t *pEdict) const
{
	// Standard GoldSrc convention: CBasePlayer::PlayerDeathThink() respawns
	// a dead player once the fire button is pressed.
	g_engfuncs.pfnRunPlayerMove(pEdict, pEdict->v.v_angle, 0.f, 0.f, 0.f, IN_ATTACK, 0, 1);
}

IGameProfile *GameProfile_Detect()
{
	static CHLDMProfile hldmProfile;

	if (hldmProfile.Detect())
		return &hldmProfile;

	return NULL;
}
