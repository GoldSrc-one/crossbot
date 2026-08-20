#include "hldm_weapons.h"

namespace
{
	enum HLDMWeaponId
	{
		HLDM_CROWBAR     = 1,
		HLDM_GLOCK       = 2,
		HLDM_PYTHON      = 3,
		HLDM_MP5         = 4,
		HLDM_CROSSBOW    = 6,
		HLDM_SHOTGUN     = 7,
		HLDM_RPG         = 8,
		HLDM_GAUSS       = 9,
		HLDM_EGON        = 10,
		HLDM_HORNETGUN   = 11,
		HLDM_HANDGRENADE = 12,
		HLDM_TRIPMINE    = 13,
		HLDM_SATCHEL     = 14,
		HLDM_SNARK       = 15,
	};

	const WeaponMode kNoMode = {};

	const WeaponSpec kHLDMWeapons[kHLDMWeaponSlots] = {
		{},
		{ HLDM_CROWBAR, "weapon_crowbar", -1,
			{ FIRE_HOLD, AIM_TARGET, TOGGLE_NONE, AMMO_NONE, 0, 0.f, 0.f, 32.f, 0.f, 0.f, 0.f, WPN_Q_NONE },
			kNoMode,
			kNoMode },
		{ HLDM_GLOCK, "weapon_9mmhandgun", 17,
			{ FIRE_HOLD, AIM_TARGET, TOGGLE_NONE, AMMO_CLIP, 1, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, WPN_Q_NONE },
			kNoMode,
			{ FIRE_HOLD, AIM_TARGET, TOGGLE_NONE, AMMO_CLIP, 1, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, WPN_Q_NONE } },
		{ HLDM_PYTHON, "weapon_357", 6,
			{ FIRE_HOLD, AIM_TARGET, TOGGLE_NONE, AMMO_CLIP, 1, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, WPN_Q_NO_UNDERWATER },
			kNoMode,
			{ FIRE_TAP, AIM_NONE, TOGGLE_ZOOM, AMMO_NONE, 0, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, WPN_Q_NONE } },
		{ HLDM_MP5, "weapon_9mmAR", 50,
			{ FIRE_HOLD, AIM_TARGET, TOGGLE_NONE, AMMO_CLIP, 1, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, WPN_Q_NONE },
			kNoMode,
			{ FIRE_HOLD, AIM_LEAD, TOGGLE_NONE, AMMO_SECONDARY, 1, 800.f, 0.5f, 0.f, 250.f, 0.f, 0.f, WPN_Q_NONE } },
		{},
		{ HLDM_CROSSBOW, "weapon_crossbow", 5,
			{ FIRE_HOLD, AIM_LEAD, TOGGLE_NONE, AMMO_CLIP, 1, 2000.f, 0.f, 0.f, 0.f, 0.f, 0.f, WPN_Q_SLOW_IN_WATER },
			{ FIRE_HOLD, AIM_TARGET, TOGGLE_NONE, AMMO_CLIP, 1, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, WPN_Q_NONE },
			{ FIRE_TAP, AIM_NONE, TOGGLE_ZOOM, AMMO_NONE, 0, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, WPN_Q_NONE } },
		{ HLDM_SHOTGUN, "weapon_shotgun", 8,
			{ FIRE_HOLD, AIM_TARGET, TOGGLE_NONE, AMMO_CLIP, 1, 0.f, 0.f, 2048.f, 0.f, 0.f, 0.f, WPN_Q_NONE },
			kNoMode,
			{ FIRE_HOLD, AIM_TARGET, TOGGLE_NONE, AMMO_CLIP, 2, 0.f, 0.f, 2048.f, 0.f, 0.f, 0.f, WPN_Q_NONE } },
		{ HLDM_RPG, "weapon_rpg", 1,
			{ FIRE_HOLD, AIM_TARGET, TOGGLE_NONE, AMMO_CLIP, 1, 250.f, 0.5f, 0.f, 250.f, 0.f, 0.f, WPN_Q_KEEP_AIMING | WPN_Q_INHERIT_VEL },
			kNoMode,
			{ FIRE_TAP, AIM_NONE, TOGGLE_LASER, AMMO_NONE, 0, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, WPN_Q_HIDDEN_STATE } },
		{ HLDM_GAUSS, "weapon_gauss", -1,
			{ FIRE_HOLD, AIM_TARGET, TOGGLE_NONE, AMMO_PRIMARY, 2, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, WPN_Q_NO_UNDERWATER },
			kNoMode,
			{ FIRE_CHARGE, AIM_TARGET, TOGGLE_NONE, AMMO_PRIMARY, 1, 0.f, 0.f, 0.f, 0.f, 1.5f, 10.f, WPN_Q_NO_UNDERWATER } },
		{ HLDM_EGON, "weapon_egon", -1,
			{ FIRE_HOLD, AIM_TARGET, TOGGLE_NONE, AMMO_PRIMARY, 1, 0.f, 0.f, 2048.f, 0.f, 0.f, 0.f, WPN_Q_NONE },
			kNoMode,
			kNoMode },
		{ HLDM_HORNETGUN, "weapon_hornetgun", -1,
			{ FIRE_HOLD, AIM_LEAD, TOGGLE_NONE, AMMO_PRIMARY, 1, 600.f, 0.f, 0.f, 0.f, 0.f, 0.f, WPN_Q_HOMING },
			kNoMode,
			{ FIRE_HOLD, AIM_LEAD, TOGGLE_NONE, AMMO_PRIMARY, 1, 1200.f, 0.f, 0.f, 0.f, 0.f, 0.f, WPN_Q_NONE } },
		{ HLDM_HANDGRENADE, "weapon_handgrenade", -1,
			{ FIRE_CHARGE, AIM_LEAD, TOGGLE_NONE, AMMO_PRIMARY, 1, 400.f, 0.5f, 0.f, 250.f, 0.f, 3.f, WPN_Q_THROW_ARC | WPN_Q_INHERIT_VEL },
			kNoMode,
			kNoMode },
		{ HLDM_TRIPMINE, "weapon_tripmine", -1,
			{ FIRE_TAP, AIM_SURFACE, TOGGLE_NONE, AMMO_PRIMARY, 1, 0.f, 0.f, 128.f, 0.f, 0.f, 0.f, WPN_Q_NONE },
			kNoMode,
			kNoMode },
		{ HLDM_SATCHEL, "weapon_satchel", -1,
			{ FIRE_TAP, AIM_NONE, TOGGLE_NONE, AMMO_NONE, 0, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, WPN_Q_HIDDEN_STATE },
			kNoMode,
			{ FIRE_TAP, AIM_LEAD, TOGGLE_NONE, AMMO_PRIMARY, 1, 274.f, 0.5f, 0.f, 375.f, 0.f, 0.f, WPN_Q_INHERIT_VEL } },
		{ HLDM_SNARK, "weapon_snark", -1,
			{ FIRE_HOLD, AIM_LEAD, TOGGLE_NONE, AMMO_PRIMARY, 1, 200.f, 0.5f, 0.f, 0.f, 0.f, 0.f, WPN_Q_INHERIT_VEL },
			kNoMode,
			kNoMode },
	};
}

const WeaponSpec *HLDM_WeaponSpec(int id)
{
	if (id <= 0 || id >= kHLDMWeaponSlots)
		return nullptr;

	const WeaponSpec &spec = kHLDMWeapons[id];
	if (!spec.classname || spec.id != id)
		return nullptr;

	return &spec;
}

const WeaponSpec *HLDM_WeaponSlot(int slot)
{
	if (slot < 0 || slot >= kHLDMWeaponSlots)
		return nullptr;

	return &kHLDMWeapons[slot];
}
