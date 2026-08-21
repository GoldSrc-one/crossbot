#include <extdll.h>
#include <meta_api.h>
#include <in_buttons.h>

#include <string.h>

#include "ent_handle.h"
#include "hldm_profile.h"
#include "hldm_weapons.h"

#if defined(_WIN32)
#define CB_STRICMP _stricmp
#else
#define CB_STRICMP strcasecmp
#endif

namespace
{
	constexpr float kHLDMWeaponRespawn = 20.f;
	constexpr float kHLDMAmmoRespawn = 20.f;
	constexpr float kHLDMItemRespawn = 30.f;
	constexpr float kHLDMRechargeDelay = 30.f;
	constexpr float kHLDMHealthChargerDelay = 60.f;
	constexpr float kHLDMFullTimeout = 120.f;
	constexpr float kHLDMUnreachableTimeout = 30.f;
	constexpr float kHLDMMaxNormal = 100.f;

	constexpr float kHLDMThrowTilt = 10.f;
	constexpr float kHLDMThrowRate = 4.f;
	constexpr float kHLDMThrowSpeedMax = 500.f;
	constexpr float kHLDMThrowPitchLimit = 89.f;

	bool HasPrefix(const char *s, const char *prefix)
	{
		return strncmp(s, prefix, strlen(prefix)) == 0;
	}
}

const char *CBotClassOf(const edict_t *ent)
{
	if (!ent || !gpGlobals->pStringBase)
		return "?";
	const char *s = gpGlobals->pStringBase + static_cast<unsigned int>(ent->v.classname);
	return (s && s[0]) ? s : "?";
}

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
	unsigned short buttons = (pEdict->v.deadflag == DEAD_RESPAWNABLE) ? IN_ATTACK : 0;

	g_engfuncs.pfnRunPlayerMove(pEdict, pEdict->v.v_angle, 0.f, 0.f, 0.f, buttons, 0, 1);
}

ItemKind CHLDMProfile::ClassifyItem(const char *classname) const
{
	if (!classname || !classname[0])
		return ITEM_NONE;

	if (HasPrefix(classname, "weapon_") || strcmp(classname, "weaponbox") == 0)
		return ITEM_WEAPON;
	if (HasPrefix(classname, "ammo_"))
		return ITEM_AMMO;
	if (HasPrefix(classname, "item_"))
		return ITEM_PICKUP;
	if (strcmp(classname, "func_healthcharger") == 0)
		return ITEM_HEALTHCHARGER;
	if (strcmp(classname, "func_recharge") == 0)
		return ITEM_RECHARGE;

	return ITEM_NONE;
}

float CHLDMProfile::ItemRespawnDelay(ItemKind kind) const
{
	switch (kind)
	{
	case ITEM_WEAPON:         return kHLDMWeaponRespawn;
	case ITEM_AMMO:           return kHLDMAmmoRespawn;
	case ITEM_PICKUP:         return kHLDMItemRespawn;
	case ITEM_RECHARGE:       return kHLDMRechargeDelay;
	case ITEM_HEALTHCHARGER:  return kHLDMHealthChargerDelay;
	default:                  return kHLDMItemRespawn;
	}
}

float CHLDMProfile::ItemFullTimeout() const
{
	return kHLDMFullTimeout;
}

float CHLDMProfile::ItemUnreachableTimeout() const
{
	return kHLDMUnreachableTimeout;
}

float CHLDMProfile::ChargerFullValue(ItemKind kind) const
{
	(void)kind;
	return kHLDMMaxNormal;
}

float CHLDMProfile::ChargerValue(ItemKind kind, edict_t *pEdict) const
{
	if (!pEdict)
		return 0.f;
	return (kind == ITEM_HEALTHCHARGER) ? pEdict->v.health : pEdict->v.armorvalue;
}

const WeaponSpec *CHLDMProfile::GetWeaponSpec(int id) const
{
	return HLDM_WeaponSpec(id);
}

bool CHLDMProfile::ThrowLaunch(const WeaponMode &mode, float viewPitch, float &elevation,
	float &speed) const
{
	if ((mode.quirks & WPN_Q_THROW_ARC) == 0)
		return false;
	if (viewPitch < -kHLDMThrowPitchLimit || viewPitch > kHLDMThrowPitchLimit)
		return false;

	float slope = (viewPitch < 0.f) ? (90.f - kHLDMThrowTilt) : (90.f + kHLDMThrowTilt);
	float pitch = -kHLDMThrowTilt + viewPitch * slope / 90.f;

	float v = (90.f - pitch) * kHLDMThrowRate;
	if (v > kHLDMThrowSpeedMax)
		v = kHLDMThrowSpeedMax;
	if (v <= 0.f)
		return false;

	elevation = -pitch;
	speed = v;
	return true;
}

IGameProfile *GameProfile_Detect()
{
	static CHLDMProfile hldmProfile;

	if (hldmProfile.Detect())
		return &hldmProfile;

	return NULL;
}

