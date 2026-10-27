#include <stddef.h>

#include "rewolf_weapons.h"

namespace
{
	enum RewolfWeaponId
	{
		RW_FISTS         = 1,
		RW_BEAMGUN       = 14,
		RW_DML           = 16,
		RW_SHOTGUN       = 17,
		RW_CHEMGUN       = 19,
		RW_MINIGUN       = 28,
		RW_GAUSSPISTOL   = 30,
	};

	const WeaponMode kNoMode = {};

	const WeaponSpec kRewolfWeapons[] = {
		{ RW_FISTS, "weapon_fists", -1, 0.1f, RANGE_CLOSE, 0.f,
			{ FIRE_HOLD, AIM_TARGET, TOGGLE_NONE, AMMO_NONE, 0, 0.f, 0.f, 32.f, 0.f, 0.f, 0.f, WPN_Q_NONE },
			kNoMode,
			kNoMode },
		{ RW_BEAMGUN, "weapon_beamgun", -1, 0.3f, RANGE_CLOSE, 0.f,
			{ FIRE_HOLD, AIM_TARGET, TOGGLE_NONE, AMMO_PRIMARY, 1, 0.f, 0.f, 512.f, 0.f, 0.f, 0.f, WPN_Q_NO_UNDERWATER | WPN_Q_STICKY },
			kNoMode,
			kNoMode },
		{ RW_DML, "weapon_dml", 2, 0.4f, RANGE_LONG, 0.f,
			{ FIRE_HOLD, AIM_LEAD, TOGGLE_NONE, AMMO_CLIP, 1, 900.f, 0.f, 0.f, 250.f, 0.f, 0.f, WPN_Q_AIM_FEET },
			kNoMode,
			kNoMode },
		{ RW_SHOTGUN, "weapon_shotgun", -1, 0.2f, RANGE_CLOSE, 0.f,
			{ FIRE_HOLD, AIM_TARGET, TOGGLE_NONE, AMMO_PRIMARY, 2, 0.f, 0.f, 2048.f, 0.f, 0.f, 0.f, WPN_Q_NONE },
			kNoMode,
			kNoMode },
		{ RW_CHEMGUN, "weapon_SPchemicalgun", -1, 0.2f, RANGE_CLOSE, 0.f,
			{ FIRE_HOLD, AIM_LEAD, TOGGLE_NONE, AMMO_PRIMARY, 1, 850.f, 0.5f, 0.f, 150.f, 0.f, 0.f, WPN_Q_NONE },
			kNoMode,
			kNoMode },
		{ RW_MINIGUN, "weapon_minigun", -1, 0.3f, RANGE_CLOSE, 0.f,
			{ FIRE_HOLD, AIM_TARGET, TOGGLE_NONE, AMMO_PRIMARY, 1, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, WPN_Q_STICKY },
			kNoMode,
			kNoMode },
		{ RW_GAUSSPISTOL, "weapon_gausspistol", -1, 0.2f, RANGE_LONG, 0.f,
			{ FIRE_HOLD, AIM_TARGET, TOGGLE_NONE, AMMO_PRIMARY, 1, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, WPN_Q_STICKY },
			kNoMode,
			kNoMode },
	};
}

const WeaponSpec *Rewolf_WeaponSlot(int slot)
{
	for (size_t i = 0; i < sizeof(kRewolfWeapons) / sizeof(kRewolfWeapons[0]); ++i)
	{
		if (kRewolfWeapons[i].id == slot)
			return &kRewolfWeapons[i];
	}

	return nullptr;
}

const WeaponSpec *Rewolf_WeaponSpec(int id)
{
	return Rewolf_WeaponSlot(id);
}

const char *Rewolf_WeaponSetup(int id)
{
	switch (id)
	{
	case RW_GAUSSPISTOL: return "cust_11";
	case RW_SHOTGUN: return "cust_22";
	case RW_DML: return "cust_23";
	default: return nullptr;
	}
}
