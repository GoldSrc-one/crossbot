#include <extdll.h>
#include <meta_api.h>
#include <commonmacros.h>
#include <usercmd.h>

#include <math.h>
#include <string.h>

#include "../../bot/bot_move.h"
#include "../../bot/bot_weapons.h"
#include "../crossplay.h"
#include "../ent_handle.h"
#include "../gamedata.h"
#include "dmc_profile.h"

namespace
{
	enum DMCWeaponId
	{
		DMC_AXE              = 1,
		DMC_SHOTGUN          = 2,
		DMC_SUPERSHOTGUN     = 3,
		DMC_NAILGUN          = 4,
		DMC_SUPERNAILGUN     = 5,
		DMC_GRENADELAUNCHER  = 6,
		DMC_ROCKETLAUNCHER   = 7,
		DMC_LIGHTNING        = 8,
		DMC_WEAPON_SLOTS     = 9,
	};

	enum DMCAmmoPool
	{
		DMC_POOL_SHELLS = 0,
		DMC_POOL_NAILS,
		DMC_POOL_ROCKETS,
		DMC_POOL_CELLS,
	};

	constexpr float kDMCWeaponRespawn = 30.f;
	constexpr float kDMCAmmoRespawn = 30.f;
	constexpr float kDMCItemRespawn = 20.f;

	constexpr float kDMCGrenadeForward = 600.f;
	constexpr float kDMCGrenadeUp = 200.f;
	constexpr float kDMCThrowPitchLimit = 89.f;
	constexpr float kDMCDegrees = 57.29578f;

	const WeaponMode kNoMode = {};

	const WeaponSpec kDMCWeapons[DMC_WEAPON_SLOTS] = {
		{},
		{ DMC_AXE, "weapon_axe", -1, 0.f, RANGE_CLOSE, 0.f,
			{ FIRE_HOLD, AIM_TARGET, TOGGLE_NONE, AMMO_NONE, 0, 0.f, 0.f, 64.f, 0.f, 0.f, 0.f, WPN_Q_NONE },
			kNoMode,
			kNoMode },
		{ DMC_SHOTGUN, "weapon_shotgun", -1, 0.1f, RANGE_LONG, 0.f,
			{ FIRE_HOLD, AIM_TARGET, TOGGLE_NONE, AMMO_PRIMARY, 1, 0.f, 0.f, 2048.f, 0.f, 0.f, 0.f, WPN_Q_NONE },
			kNoMode,
			kNoMode },
		{ DMC_SUPERSHOTGUN, "weapon_supershotgun", -1, 0.2f, RANGE_CLOSE, 0.f,
			{ FIRE_HOLD, AIM_TARGET, TOGGLE_NONE, AMMO_PRIMARY, 1, 0.f, 0.f, 2048.f, 0.f, 0.f, 0.f, WPN_Q_NONE },
			kNoMode,
			kNoMode },
		{ DMC_NAILGUN, "weapon_nailgun", -1, 0.2f, RANGE_CLOSE, 0.f,
			{ FIRE_HOLD, AIM_LEAD, TOGGLE_NONE, AMMO_PRIMARY, 1, 1000.f, 0.f, 0.f, 0.f, 0.f, 0.f, WPN_Q_STICKY },
			kNoMode,
			kNoMode },
		{ DMC_SUPERNAILGUN, "weapon_supernailgun", -1, 0.3f, RANGE_CLOSE, 0.f,
			{ FIRE_HOLD, AIM_LEAD, TOGGLE_NONE, AMMO_PRIMARY, 1, 1000.f, 0.f, 0.f, 0.f, 0.f, 0.f, WPN_Q_STICKY },
			kNoMode,
			kNoMode },
		{ DMC_GRENADELAUNCHER, "weapon_grenadelauncher", -1, 0.3f, RANGE_CLOSE, 0.f,
			{ FIRE_HOLD, AIM_LEAD, TOGGLE_NONE, AMMO_PRIMARY, 1, 632.5f, 1.f, 0.f, 160.f, 0.f, 0.f, WPN_Q_THROW_ARC },
			kNoMode,
			kNoMode },
		{ DMC_ROCKETLAUNCHER, "weapon_rocketlauncher", -1, 0.5f, RANGE_LONG, 0.f,
			{ FIRE_HOLD, AIM_LEAD, TOGGLE_NONE, AMMO_PRIMARY, 1, 1000.f, 0.f, 0.f, 160.f, 0.f, 0.f, WPN_Q_AIM_FEET },
			kNoMode,
			kNoMode },
		{ DMC_LIGHTNING, "weapon_lightning", -1, 0.5f, RANGE_CLOSE, 0.f,
			{ FIRE_HOLD, AIM_TARGET, TOGGLE_NONE, AMMO_PRIMARY, 1, 0.f, 0.f, 600.f, 0.f, 0.f, 0.f, WPN_Q_STICKY | WPN_Q_NO_UNDERWATER },
			kNoMode,
			kNoMode },
	};

	int PoolOf(int id)
	{
		switch (id)
		{
		case DMC_SHOTGUN:
		case DMC_SUPERSHOTGUN:     return DMC_POOL_SHELLS;
		case DMC_NAILGUN:
		case DMC_SUPERNAILGUN:     return DMC_POOL_NAILS;
		case DMC_GRENADELAUNCHER:
		case DMC_ROCKETLAUNCHER:   return DMC_POOL_ROCKETS;
		case DMC_LIGHTNING:        return DMC_POOL_CELLS;
		default:                   return -1;
		}
	}

	bool HasQuakeOffsets()
	{
		return g_offsets.plr_iQuakeItems >= 0 && g_offsets.plr_iQuakeWeapon >= 0 &&
			g_offsets.plr_iAmmoShells >= 0 && g_offsets.plr_iAmmoNails >= 0 &&
			g_offsets.plr_iAmmoRockets >= 0 && g_offsets.plr_iAmmoCells >= 0;
	}
}

const char *CDMCProfile::GetName() const
{
	return "DMC";
}

bool CDMCProfile::Detect(const char *gamedir) const
{
	if (!gpGlobals->deathmatch_)
		return false;

	return gamedir && CB_STRICMP(gamedir, "dmc") == 0;
}

bool CDMCProfile::FallDamage() const
{
	return CVAR_GET_FLOAT("mp_falldamage") != 0.f;
}

bool CDMCProfile::Disguised(edict_t *pEdict) const
{
	constexpr int kInvisibility = 1 << 19;
	void *priv = CBotPrivateData(pEdict);
	return priv && g_offsets.plr_iQuakeItems >= 0 &&
		(GameData::Peek<int>(priv, g_offsets.plr_iQuakeItems) & kInvisibility) != 0;
}

float CDMCProfile::ItemRespawnDelay(ItemKind kind) const
{
	switch (kind)
	{
	case ITEM_WEAPON:  return kDMCWeaponRespawn;
	case ITEM_AMMO:    return kDMCAmmoRespawn;
	case ITEM_PICKUP:  return kDMCItemRespawn;
	default:           return CHLDMProfile::ItemRespawnDelay(kind);
	}
}

bool CDMCProfile::ItemPreferred(edict_t *ent, ItemKind kind) const
{
	if (ItemKindIsCharger(kind) || ItemKindIsVolume(kind))
		return true;

	const char *classname = CBotClassOf(ent);
	if (!classname || !classname[0])
		return false;

	if (strcmp(classname, "item_artifact_super_damage") == 0 || strcmp(classname, "item_armor2") == 0 ||
		strcmp(classname, "item_armor3") == 0 || strcmp(classname, "item_armorInv") == 0)
		return true;

	static const char *const kPreferred[] = { "weapon_rocketlauncher", "weapon_lightning", "weapon_supernailgun" };

	return PreferredWeaponOrAmmo(classname, kPreferred, ARRAYSIZE(kPreferred));
}

const WeaponSpec *CDMCProfile::GetWeaponSpec(int id) const
{
	if (id <= 0 || id >= DMC_WEAPON_SLOTS)
		return nullptr;

	const WeaponSpec &spec = kDMCWeapons[id];
	return spec.id == id ? &spec : nullptr;
}

int CDMCProfile::WeaponSlotCount() const
{
	return DMC_WEAPON_SLOTS;
}

const WeaponSpec *CDMCProfile::WeaponSlot(int slot) const
{
	if (slot < 0 || slot >= DMC_WEAPON_SLOTS)
		return nullptr;

	return kDMCWeapons[slot].classname ? &kDMCWeapons[slot] : nullptr;
}

bool CDMCProfile::ReadInventory(edict_t *player, BotInventory &out) const
{
	if (!HasQuakeOffsets())
		GameData::Fatal("DMC profile is active but the gamedata lacks the CBasePlayer quake fields");

	void *priv = CBotPrivateData(player);
	if (!priv)
		return true;

	out.nextAttack = GameData::Peek<float>(priv, g_offsets.mon_flNextAttack);
	out.ammo[DMC_POOL_SHELLS] = GameData::Peek<int>(priv, g_offsets.plr_iAmmoShells);
	out.ammo[DMC_POOL_NAILS] = GameData::Peek<int>(priv, g_offsets.plr_iAmmoNails);
	out.ammo[DMC_POOL_ROCKETS] = GameData::Peek<int>(priv, g_offsets.plr_iAmmoRockets);
	out.ammo[DMC_POOL_CELLS] = GameData::Peek<int>(priv, g_offsets.plr_iAmmoCells);

	int items = GameData::Peek<int>(priv, g_offsets.plr_iQuakeItems);
	int held = GameData::Peek<int>(priv, g_offsets.plr_iQuakeWeapon);

	for (int id = DMC_AXE; id < DMC_WEAPON_SLOTS && out.count < kMaxBotWeapons; ++id)
	{
		int bit = 1 << (id - 1);
		if ((items & bit) == 0)
			continue;

		BotWeapon &w = out.weapons[out.count];
		w.id = id;
		w.primaryAmmoType = PoolOf(id);
		if (held == bit)
		{
			w.nextPrimaryAttack = out.nextAttack;
			out.activeIndex = out.count;
		}
		++out.count;
	}

	return true;
}

bool CDMCProfile::SelectWeapon(edict_t *player, const WeaponSpec &spec) const
{
	if (!player || player->free || spec.id <= 0 || spec.id >= DMC_WEAPON_SLOTS)
		return false;

	usercmd_t cmd;
	memset(&cmd, 0, sizeof(cmd));
	cmd.msec = Bot_FrameMsec();
	cmd.viewangles = player->v.v_angle;
	cmd.weaponselect = static_cast<byte>(spec.id);

	MDLL_CmdStart(player, &cmd, 0);
	MDLL_CmdEnd(player);
	return true;
}

const char *CDMCProfile::AmmoEntity(int weaponId, WeaponAmmo pool) const
{
	if (pool != AMMO_PRIMARY)
		return nullptr;

	switch (PoolOf(weaponId))
	{
	case DMC_POOL_SHELLS:  return "item_shells";
	case DMC_POOL_NAILS:   return "item_spikes";
	case DMC_POOL_ROCKETS: return "item_rockets";
	case DMC_POOL_CELLS:   return "item_cells";
	default:               return nullptr;
	}
}

int CDMCProfile::MeleeWeapon() const
{
	return DMC_AXE;
}

int CDMCProfile::AmbushWeapon(AmbushKind) const
{
	return 0;
}

int CDMCProfile::DeployWeapon(DeployKind) const
{
	return 0;
}

const char *CDMCProfile::DeployEntity(DeployKind) const
{
	return nullptr;
}

bool CDMCProfile::ThrowLaunch(const WeaponMode &mode, float viewPitch, float &elevation,
	float &speed) const
{
	if ((mode.quirks & WPN_Q_THROW_ARC) == 0)
		return false;

	return DMC_ThrowLaunch(viewPitch, elevation, speed);
}

bool DMC_ThrowLaunch(float viewPitch, float &elevation, float &speed)
{
	if (viewPitch < -kDMCThrowPitchLimit || viewPitch > kDMCThrowPitchLimit)
		return false;

	float lift = atan2f(kDMCGrenadeUp, kDMCGrenadeForward) * kDMCDegrees;
	elevation = -viewPitch + lift;
	if (elevation >= 90.f)
		return false;

	speed = sqrtf(kDMCGrenadeForward * kDMCGrenadeForward + kDMCGrenadeUp * kDMCGrenadeUp);
	return true;
}
