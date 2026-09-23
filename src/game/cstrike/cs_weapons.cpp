#include <string.h>

#include "cs_weapons.h"

namespace
{
	const WeaponMode kNoMode = {};

	const WeaponMode kAuto = { FIRE_HOLD, AIM_TARGET, TOGGLE_NONE, AMMO_CLIP, 1, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, WPN_Q_STICKY };
	const WeaponMode kSemi = { FIRE_TAP, AIM_TARGET, TOGGLE_NONE, AMMO_CLIP, 1, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, WPN_Q_NONE };
	const WeaponMode kShotgun = { FIRE_HOLD, AIM_TARGET, TOGGLE_NONE, AMMO_CLIP, 1, 0.f, 0.f, 2048.f, 0.f, 0.f, 0.f, WPN_Q_NONE };
	const WeaponMode kZoom = { FIRE_TAP, AIM_NONE, TOGGLE_ZOOM, AMMO_NONE, 0, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, WPN_Q_NONE };
	const WeaponMode kKnife = { FIRE_HOLD, AIM_TARGET, TOGGLE_NONE, AMMO_NONE, 0, 0.f, 0.f, 48.f, 0.f, 0.f, 0.f, WPN_Q_NONE };
	const WeaponMode kHeGrenade = { FIRE_TAP, AIM_LEAD, TOGGLE_NONE, AMMO_PRIMARY, 1, 750.f, 0.55f, 0.f, 350.f, 0.f, 0.f, WPN_Q_THROW_ARC | WPN_Q_INHERIT_VEL };

	const WeaponSpec kCSWeapons[kCSWeaponSlots] = {
		{},
		{ CS_P228, "weapon_p228", 13, 0.12f, RANGE_CLOSE, 0.f, kSemi, kNoMode, kNoMode },
		{},
		{ CS_SCOUT, "weapon_scout", 10, 0.35f, RANGE_LONG, 40.f, kSemi, kSemi, kZoom },
		{ CS_HEGRENADE, "weapon_hegrenade", -1, 0.1f, RANGE_CLOSE, 0.f, kHeGrenade, kNoMode, kNoMode },
		{ CS_XM1014, "weapon_xm1014", 7, 0.3f, RANGE_CLOSE, 0.f, kShotgun, kNoMode, kNoMode },
		{ CS_C4, "weapon_c4", -1, 0.f, RANGE_NONE, 0.f, kNoMode, kNoMode, kNoMode },
		{ CS_MAC10, "weapon_mac10", 30, 0.25f, RANGE_CLOSE, 0.f, kAuto, kNoMode, kNoMode },
		{ CS_AUG, "weapon_aug", 30, 0.4f, RANGE_LONG, 55.f, kAuto, kAuto, kZoom },
		{ CS_SMOKEGRENADE, "weapon_smokegrenade", -1, 0.f, RANGE_NONE, 0.f, kNoMode, kNoMode, kNoMode },
		{ CS_ELITE, "weapon_elite", 30, 0.15f, RANGE_CLOSE, 0.f, kSemi, kNoMode, kNoMode },
		{ CS_FIVESEVEN, "weapon_fiveseven", 20, 0.12f, RANGE_CLOSE, 0.f, kSemi, kNoMode, kNoMode },
		{ CS_UMP45, "weapon_ump45", 25, 0.25f, RANGE_CLOSE, 0.f, kAuto, kNoMode, kNoMode },
		{ CS_SG550, "weapon_sg550", 30, 0.45f, RANGE_LONG, 40.f, kSemi, kSemi, kZoom },
		{ CS_GALIL, "weapon_galil", 35, 0.35f, RANGE_LONG, 0.f, kAuto, kNoMode, kNoMode },
		{ CS_FAMAS, "weapon_famas", 25, 0.35f, RANGE_LONG, 0.f, kAuto, kNoMode, kNoMode },
		{ CS_USP, "weapon_usp", 12, 0.12f, RANGE_CLOSE, 0.f, kSemi, kNoMode, kNoMode },
		{ CS_GLOCK18, "weapon_glock18", 20, 0.1f, RANGE_CLOSE, 0.f, kSemi, kNoMode, kNoMode },
		{ CS_AWP, "weapon_awp", 10, 0.5f, RANGE_LONG, 40.f, kSemi, kSemi, kZoom },
		{ CS_MP5NAVY, "weapon_mp5navy", 30, 0.25f, RANGE_CLOSE, 0.f, kAuto, kNoMode, kNoMode },
		{ CS_M249, "weapon_m249", 100, 0.4f, RANGE_LONG, 0.f, kAuto, kNoMode, kNoMode },
		{ CS_M3, "weapon_m3", 8, 0.3f, RANGE_CLOSE, 0.f, kShotgun, kNoMode, kNoMode },
		{ CS_M4A1, "weapon_m4a1", 30, 0.4f, RANGE_LONG, 0.f, kAuto, kNoMode, kNoMode },
		{ CS_TMP, "weapon_tmp", 30, 0.22f, RANGE_CLOSE, 0.f, kAuto, kNoMode, kNoMode },
		{ CS_G3SG1, "weapon_g3sg1", 20, 0.45f, RANGE_LONG, 40.f, kSemi, kSemi, kZoom },
		{ CS_FLASHBANG, "weapon_flashbang", -1, 0.f, RANGE_NONE, 0.f, kNoMode, kNoMode, kNoMode },
		{ CS_DEAGLE, "weapon_deagle", 7, 0.2f, RANGE_CLOSE, 0.f, kSemi, kNoMode, kNoMode },
		{ CS_SG552, "weapon_sg552", 30, 0.4f, RANGE_LONG, 55.f, kAuto, kAuto, kZoom },
		{ CS_AK47, "weapon_ak47", 30, 0.4f, RANGE_LONG, 0.f, kAuto, kNoMode, kNoMode },
		{ CS_KNIFE, "weapon_knife", -1, 0.05f, RANGE_CLOSE, 0.f, kKnife, kNoMode, kNoMode },
		{ CS_P90, "weapon_p90", 50, 0.3f, RANGE_CLOSE, 0.f, kAuto, kNoMode, kNoMode },
	};

	const int kArmoury[] = {
		CS_MP5NAVY, CS_TMP, CS_P90, CS_MAC10, CS_AK47, CS_SG552, CS_M4A1, CS_AUG, CS_SCOUT, CS_G3SG1,
		CS_AWP, CS_M3, CS_XM1014, CS_M249, CS_FLASHBANG, CS_HEGRENADE, 0, 0, CS_SMOKEGRENADE, CS_SHIELD,
		CS_FAMAS, CS_SG550, CS_GALIL, CS_UMP45, CS_GLOCK18, CS_USP, CS_ELITE, CS_FIVESEVEN, CS_P228,
		CS_DEAGLE,
	};

	constexpr int kArmouryKevlar = 16;
	constexpr int kArmouryAssault = 17;

	const CSSpread kSpreads[] = {
		{ CS_AK47,      140.f, 0.f, 0.0275f, 0.04f,  0.07f,   0.f, 0.0275f, 0.2f,  0.f,    0.f   },
		{ CS_M4A1,      140.f, 0.f, 0.025f,  0.035f, 0.07f,   0.f, 0.025f,  0.2f,  0.f,    0.f   },
		{ CS_GALIL,     140.f, 0.f, 0.0375f, 0.04f,  0.07f,   0.f, 0.0375f, 0.2f,  0.f,    0.f   },
		{ CS_FAMAS,     140.f, 0.f, 0.02f,   0.03f,  0.07f,   0.f, 0.02f,   0.2f,  0.f,    0.f   },
		{ CS_SG552,     140.f, 0.f, 0.02f,   0.035f, 0.075f,  0.f, 0.02f,   0.2f,  0.f,    0.f   },
		{ CS_AUG,       140.f, 0.f, 0.02f,   0.035f, 0.07f,   0.f, 0.02f,   0.2f,  0.f,    0.f   },
		{ CS_M249,      140.f, 0.f, 0.03f,   0.045f, 0.095f,  0.f, 0.03f,   0.2f,  0.f,    0.f   },
		{ CS_P90,       170.f, 0.f, 0.045f,  0.f,    0.115f,  0.f, 0.045f,  0.2f,  0.f,    0.f   },
		{ CS_MP5NAVY,   0.f,   0.f, 0.04f,   0.f,    0.04f,   0.f, 0.04f,   0.f,   0.f,    0.f   },
		{ CS_TMP,       0.f,   0.f, 0.03f,   0.f,    0.03f,   0.f, 0.03f,   0.2f,  0.f,    0.f   },
		{ CS_MAC10,     0.f,   0.f, 0.03f,   0.f,    0.03f,   0.f, 0.03f,   0.15f, 0.f,    0.f   },
		{ CS_UMP45,     0.f,   0.f, 0.04f,   0.f,    0.04f,   0.f, 0.04f,   0.f,   0.f,    0.f   },
		{ CS_DEAGLE,    0.f,   0.f, 0.13f,   0.f,    0.25f,   0.f, 0.115f,  0.9f,  0.4f,   0.55f },
		{ CS_USP,       0.f,   0.f, 0.1f,    0.f,    0.225f,  0.f, 0.08f,   0.92f, 0.3f,   0.6f  },
		{ CS_GLOCK18,   0.f,   0.f, 0.1f,    0.f,    0.165f,  0.f, 0.075f,  0.9f,  0.325f, 0.6f  },
		{ CS_P228,      0.f,   0.f, 0.15f,   0.f,    0.255f,  0.f, 0.075f,  0.9f,  0.325f, 0.6f  },
		{ CS_FIVESEVEN, 0.f,   0.f, 0.15f,   0.f,    0.255f,  0.f, 0.075f,  0.92f, 0.275f, 0.725f },
		{ CS_ELITE,     0.f,   0.f, 0.1f,    0.f,    0.175f,  0.f, 0.08f,   0.88f, 0.325f, 0.55f },
	};

	const CSBullet kBullets[] = {
		{ CS_AK47,  2, 39, 5000.f, 36,  0.98f,  8192.f },
		{ CS_M4A1,  2, 35, 4000.f, 32,  0.97f,  8192.f },
		{ CS_GALIL, 2, 35, 4000.f, 30,  0.98f,  8192.f },
		{ CS_FAMAS, 2, 35, 4000.f, 30,  0.96f,  8192.f },
		{ CS_SG552, 2, 35, 4000.f, 33,  0.955f, 8192.f },
		{ CS_AUG,   2, 35, 4000.f, 32,  0.96f,  8192.f },
		{ CS_M249,  2, 35, 4000.f, 32,  0.97f,  8192.f },
		{ CS_SG550, 2, 35, 4000.f, 70,  0.98f,  8192.f },
		{ CS_SCOUT, 3, 39, 5000.f, 75,  0.98f,  8192.f },
		{ CS_G3SG1, 3, 39, 5000.f, 80,  0.98f,  8192.f },
		{ CS_AWP,   3, 45, 8000.f, 115, 0.99f,  8192.f },
		{ CS_DEAGLE, 2, 30, 1000.f, 54, 0.81f,  4096.f },
	};
}

const CSSpread *CS_Spread(int id)
{
	for (const CSSpread &s : kSpreads)
	{
		if (s.id == id)
			return &s;
	}
	return nullptr;
}

const CSBullet *CS_Bullet(int id)
{
	for (const CSBullet &b : kBullets)
	{
		if (b.id == id)
			return &b;
	}
	return nullptr;
}

const WeaponSpec *CS_WeaponSpec(int id)
{
	if (id <= 0 || id >= kCSWeaponSlots)
		return nullptr;

	const WeaponSpec &spec = kCSWeapons[id];
	if (!spec.classname || spec.id != id)
		return nullptr;

	return &spec;
}

const WeaponSpec *CS_WeaponSlot(int slot)
{
	if (slot < 0 || slot >= kCSWeaponSlots)
		return nullptr;

	return &kCSWeapons[slot];
}

int CS_WeaponId(const char *classname)
{
	if (!classname)
		return 0;
	if (strcmp(classname, "weapon_shield") == 0)
		return CS_SHIELD;

	for (int id = 1; id < kCSWeaponSlots; ++id)
	{
		const WeaponSpec &spec = kCSWeapons[id];
		if (spec.classname && strcmp(spec.classname, classname) == 0)
			return id;
	}
	return 0;
}

int CS_ArmouryWeapon(int item)
{
	if (item < 0 || item >= static_cast<int>(sizeof(kArmoury) / sizeof(kArmoury[0])))
		return 0;
	return kArmoury[item];
}

bool CS_ArmouryArmor(int item)
{
	return item == kArmouryKevlar || item == kArmouryAssault;
}

CSInventorySlot CS_InventorySlot(int id)
{
	switch (id)
	{
	case CS_P228:
	case CS_ELITE:
	case CS_FIVESEVEN:
	case CS_USP:
	case CS_GLOCK18:
	case CS_DEAGLE:
		return CS_SLOT_PISTOL;
	case CS_KNIFE:
		return CS_SLOT_KNIFE;
	case CS_HEGRENADE:
	case CS_SMOKEGRENADE:
	case CS_FLASHBANG:
		return CS_SLOT_GRENADE;
	case CS_C4:
		return CS_SLOT_C4;
	case 0:
		return CS_SLOT_NONE;
	default:
		return CS_SLOT_PRIMARY;
	}
}

int CS_GrenadeMax(int id)
{
	return id == CS_FLASHBANG ? 2 : 1;
}
