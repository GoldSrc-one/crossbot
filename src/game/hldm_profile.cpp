#include <extdll.h>
#include <meta_api.h>
#include <algorithm>
#include <commonmacros.h>
#include <in_buttons.h>

#include <stdio.h>
#include <string.h>

#include "../bot/bot_fire.h"
#include "../bot/bot_move.h"
#include "../bot/bot_timing.h"
#include "../bot/bot_weapons.h"
#include "crossplay.h"
#include "ent_handle.h"
#include "gamedata.h"
#include "hldm_profile.h"
#include "hldm_weapons.h"
#include "player_models.h"
#include "ricochet_profile.h"

namespace
{
	constexpr int kEnvExplosionNoDamage = 1;

	constexpr float kHLDMWeaponRespawn = 20.f;
	constexpr float kHLDMAmmoRespawn = 20.f;
	constexpr float kHLDMItemRespawn = 30.f;
	constexpr float kHLDMRechargeDelay = 30.f;
	constexpr float kHLDMHealthChargerDelay = 60.f;
	constexpr float kHLDMFullTimeout = 120.f;
	constexpr float kHLDMUnreachableTimeout = 30.f;
	constexpr float kHLDMMaxNormal = 100.f;

	constexpr int kSfHurtNoClients = 8;

	constexpr float kHLDMThrowTilt = 10.f;
	constexpr float kHLDMThrowRate = 4.f;
	constexpr float kHLDMThrowSpeedMax = 500.f;
	constexpr float kHLDMThrowPitchLimit = 89.f;

	bool HasPrefix(const char *s, const char *prefix)
	{
		return strncmp(s, prefix, strlen(prefix)) == 0;
	}

	bool IsHealingHurt(edict_t *ent)
	{
		if (ent->v.dmg >= 0.f)
			return false;
		return (ent->v.spawnflags & kSfHurtNoClients) == 0;
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

bool CHLDMProfile::Detect(const char *gamedir) const
{
	if (!gpGlobals->deathmatch_)
		return false;

	return gamedir && CB_STRICMP(gamedir, "valve") == 0;
}

void CHLDMProfile::ApplyAddArg(int iGame, edict_t *pEdict, const char *arg, char *out,
	unsigned int size) const
{
	if (!out || size == 0)
		return;

	out[0] = 0;

	if (arg && arg[0])
	{
		snprintf(out, size, "%s", arg);

		if (!PlayerModels::Installed(iGame, out))
		{
			char msg[192];
			snprintf(msg, sizeof(msg),
				"CrossBot: model '%s' is not installed for '%s' -- using it anyway. Installed:\n",
				out, XPlay::GameDir(iGame));
			SERVER_PRINT(msg);
			PlayerModels::PrintList(iGame);
		}
	}
	else if (!PlayerModels::PickRandom(iGame, out, size))
	{
		return;
	}

	g_engfuncs.pfnSetClientKeyValue(g_engfuncs.pfnIndexOfEdict(pEdict),
		g_engfuncs.pfnGetInfoKeyBuffer(pEdict), "model", out);
}

bool CHLDMProfile::IsDead(edict_t *pEdict) const
{
	return pEdict->v.deadflag != DEAD_NO || pEdict->v.health <= 0.0f;
}

edict_t *CHLDMProfile::MonsterEnemy(edict_t *ent) const
{
	if (!ent || ent->free || !CBotPrivateData(ent))
		return nullptr;
	if ((ent->v.flags & FL_MONSTER) == 0)
		return nullptr;
	if (g_offsets.mon_hEnemy < 0)
		return nullptr;

	edict_t *foe = GameData::Peek<edict_t *>(CBotPrivateData(ent), g_offsets.mon_hEnemy);
	if (!foe || foe->free)
		return nullptr;

	int serial = GameData::Peek<int>(CBotPrivateData(ent), g_offsets.mon_hEnemy + 4);
	if (foe->serialnumber != serial)
		return nullptr;

	return foe;
}

float CHLDMProfile::EvaluateCombat(edict_t *pEdict, const BotInventory &inv) const
{
	if (!pEdict)
		return 0.f;

	float maxHealth = (pEdict->v.max_health > 0.f) ? pEdict->v.max_health : kHLDMMaxNormal;
	float pool = maxHealth + kHLDMMaxNormal;
	float survive = (pEdict->v.health + pEdict->v.armorvalue) / pool;
	survive = std::clamp(survive, 0.f, 1.f);

	float arms = 0.f;
	for (int i = 0; i < inv.count; ++i)
	{
		const BotWeapon &w = inv.weapons[i];
		const WeaponSpec *spec = HLDM_WeaponSpec(w.id);
		if (!spec || spec->value <= 0.f)
			continue;
		if (BotFire::WeaponArmed(inv, w))
			arms += spec->value;
	}

	return survive * std::clamp(arms, 0.f, 1.f);
}

bool CHLDMProfile::FallDamage() const
{
	return true;
}

bool CHLDMProfile::AllowsGoalTier(BotGoal::Tier) const
{
	return true;
}

void CHLDMProfile::Respawn(edict_t *pEdict) const
{
	unsigned short buttons = (pEdict->v.deadflag == DEAD_RESPAWNABLE) ? IN_ATTACK : 0;

	byte msec = Bot_FrameMsec();
	BotTiming::BeginMove(pEdict, msec);
	g_engfuncs.pfnRunPlayerMove(pEdict, pEdict->v.v_angle, 0.f, 0.f, 0.f, buttons, 0, msec);
	BotTiming::EndMove(pEdict);
}

ItemKind CHLDMProfile::ClassifyItem(edict_t *ent) const
{
	if (!ent || ent->free)
		return ITEM_NONE;

	const char *classname = CBotClassOf(ent);
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
	if (strcmp(classname, "trigger_hurt") == 0)
		return IsHealingHurt(ent) ? ITEM_HEALVOLUME : ITEM_NONE;

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
	return (kind == ITEM_RECHARGE) ? pEdict->v.armorvalue : pEdict->v.health;
}

const WeaponSpec *CHLDMProfile::GetWeaponSpec(int id) const
{
	return HLDM_WeaponSpec(id);
}

bool CHLDMProfile::ModeAvailable(edict_t *, const WeaponSpec &, const WeaponMode &) const
{
	return true;
}

bool CHLDMProfile::ItemPreferred(edict_t *ent, ItemKind kind) const
{
	if (ItemKindIsCharger(kind) || ItemKindIsVolume(kind))
		return true;

	const char *classname = CBotClassOf(ent);
	if (!classname || !classname[0])
		return false;

	if (strcmp(classname, "item_healthkit") == 0 || strcmp(classname, "item_battery") == 0 ||
		strcmp(classname, "item_longjump") == 0)
		return true;

	static const char *kPreferred[] = { "weapon_gauss", "weapon_crossbow", "weapon_egon" };

	for (size_t i = 0; i < ARRAYSIZE(kPreferred); ++i)
	{
		if (strcmp(classname, kPreferred[i]) == 0)
			return true;

		for (int slot = 0; slot < kHLDMWeaponSlots; ++slot)
		{
			const WeaponSpec *spec = HLDM_WeaponSlot(slot);
			if (!spec || !spec->classname || strcmp(spec->classname, kPreferred[i]) != 0)
				continue;

			for (int pool = AMMO_CLIP; pool <= AMMO_SECONDARY; ++pool)
			{
				const char *ammo = AmmoEntity(spec->id, static_cast<WeaponAmmo>(pool));
				if (ammo && strcmp(classname, ammo) == 0)
					return true;
			}
		}
	}

	return false;
}

int CHLDMProfile::MeleeWeapon() const
{
	return kHLDMMelee;
}

int CHLDMProfile::AmbushWeapon(AmbushKind kind) const
{
	switch (kind)
	{
	case AMBUSH_GRENADE: return kHLDMGrenade;
	case AMBUSH_SNARK: return kHLDMSnark;
	default: return 0;
	}
}

int CHLDMProfile::DeployWeapon(DeployKind kind) const
{
	switch (kind)
	{
	case DEPLOY_TRIPMINE: return kHLDMTripmine;
	case DEPLOY_SATCHEL: return kHLDMSatchel;
	default: return 0;
	}
}

const char *CHLDMProfile::DeployEntity(DeployKind kind) const
{
	switch (kind)
	{
	case DEPLOY_TRIPMINE: return "monster_tripmine";
	case DEPLOY_SATCHEL: return "monster_satchel";
	default: return NULL;
	}
}

const char *CHLDMProfile::AmmoEntity(int weaponId, WeaponAmmo pool) const
{
	const WeaponSpec *spec = HLDM_WeaponSpec(weaponId);
	if (!spec || !spec->classname || pool == AMMO_NONE)
		return NULL;

	static const struct
	{
		const char *weapon;
		const char *clip;
		const char *primary;
		const char *secondary;
	} kAmmo[] = {
		{ "weapon_9mmhandgun",  "ammo_9mmclip",   NULL,               NULL },
		{ "weapon_357",         "ammo_357",       NULL,               NULL },
		{ "weapon_9mmAR",       "ammo_9mmAR",     NULL,               "ammo_ARgrenades" },
		{ "weapon_crossbow",    "ammo_crossbow",  NULL,               NULL },
		{ "weapon_shotgun",     "ammo_buckshot",  NULL,               NULL },
		{ "weapon_rpg",         "ammo_rpgclip",   NULL,               NULL },
		{ "weapon_gauss",       NULL,             "ammo_gaussclip",   NULL },
		{ "weapon_egon",        NULL,             "ammo_egonclip",    NULL },
		{ "weapon_handgrenade", NULL,             "weapon_handgrenade", NULL },
		{ "weapon_snark",       NULL,             "weapon_snark",     NULL },
		{ "weapon_tripmine",    NULL,             "weapon_tripmine",  NULL },
		{ "weapon_satchel",     NULL,             "weapon_satchel",   "weapon_satchel" },
	};

	for (size_t i = 0; i < ARRAYSIZE(kAmmo); ++i)
	{
		if (strcmp(kAmmo[i].weapon, spec->classname) != 0)
			continue;
		if (pool == AMMO_CLIP)
			return kAmmo[i].clip;
		if (pool == AMMO_PRIMARY)
			return kAmmo[i].primary;
		return kAmmo[i].secondary;
	}

	return NULL;
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

bool CHLDMProfile::IsFunTerminal(edict_t *ent) const
{
	if (!ent || ent->free)
		return false;

	const char *cls = STRING(ent->v.classname);
	if (!cls || !cls[0])
		return false;

	if (strcmp(cls, "trigger_hurt") == 0)
		return ent->v.dmg > 0.f;

	if (strcmp(cls, "func_water") == 0)
		return true;

	if (strcmp(cls, "func_door") == 0)
		return ent->v.skin < 0;

	if (strcmp(cls, "env_explosion") == 0)
		return (static_cast<int>(ent->v.spawnflags) & kEnvExplosionNoDamage) == 0;

	if (strcmp(cls, "func_mortar_field") == 0)
		return true;

	if (strncmp(cls, "monstermaker", 12) == 0)
		return true;

	return false;
}

IGameProfile *GameProfile_Detect(const char *gamedir)
{
	static CHLDMProfile hldmProfile;
	static CRicochetProfile ricochetProfile;

	if (hldmProfile.Detect(gamedir))
		return &hldmProfile;
	if (ricochetProfile.Detect(gamedir))
		return &ricochetProfile;

	return NULL;
}

