#include "gearbox_weapons.h"
#include "../hldm/hldm_weapons.h"

namespace
{
	enum GearboxWeaponId
	{
		OP4_GRAPPLE       = 16,
		OP4_EAGLE         = 17,
		OP4_PIPEWRENCH    = 18,
		OP4_M249          = 19,
		OP4_DISPLACER     = 20,
		OP4_SHOCKRIFLE    = 22,
		OP4_SPORELAUNCHER = 23,
		OP4_SNIPERRIFLE   = 24,
		OP4_KNIFE         = 25,
		OP4_PENGUIN       = 26,
	};

	const WeaponMode kNoMode = {};

	const WeaponMode kSwing = { FIRE_HOLD, AIM_TARGET, TOGGLE_NONE, AMMO_NONE, 0, 0.f, 0.f, 32.f, 0.f, 0.f, 0.f, WPN_Q_NONE };

	const WeaponMode kSniperShot = { FIRE_HOLD, AIM_TARGET, TOGGLE_NONE, AMMO_CLIP, 1, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, WPN_Q_NO_UNDERWATER };

	const WeaponSpec kGearboxWeapons[kGearboxWeaponSlots - kHLDMWeaponSlots] = {
		{ OP4_GRAPPLE, "weapon_grapple", -1, 0.1f, RANGE_CLOSE, 0.f,
			{ FIRE_HOLD, AIM_LEAD, TOGGLE_NONE, AMMO_NONE, 0, 2000.f, 0.f, 0.f, 0.f, 0.f, 0.f, WPN_Q_STICKY },
			kNoMode,
			kNoMode },
		{ OP4_EAGLE, "weapon_eagle", 7, 0.3f, RANGE_LONG, 0.f,
			{ FIRE_HOLD, AIM_TARGET, TOGGLE_NONE, AMMO_CLIP, 1, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, WPN_Q_NO_UNDERWATER },
			kNoMode,
			{ FIRE_TAP, AIM_NONE, TOGGLE_LASER, AMMO_NONE, 0, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, WPN_Q_HIDDEN_STATE | WPN_Q_LASER_ON } },
		{ OP4_PIPEWRENCH, "weapon_pipewrench", -1, 0.1f, RANGE_CLOSE, 0.f,
			kSwing,
			kNoMode,
			kNoMode },
		{ OP4_M249, "weapon_m249", 50, 0.3f, RANGE_CLOSE, 0.f,
			{ FIRE_HOLD, AIM_TARGET, TOGGLE_NONE, AMMO_CLIP, 1, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, WPN_Q_NO_UNDERWATER | WPN_Q_STICKY | WPN_Q_BRACE },
			kNoMode,
			kNoMode },
		{ OP4_DISPLACER, "weapon_displacer", -1, 0.4f, RANGE_CLOSE, 0.f,
			{ FIRE_TAP, AIM_LEAD, TOGGLE_NONE, AMMO_PRIMARY, 20, 500.f, 0.f, 0.f, 300.f, 1.f, 0.f, WPN_Q_SPINUP | WPN_Q_AIM_FEET },
			kNoMode,
			kNoMode },
		{},
		{ OP4_SHOCKRIFLE, "weapon_shockrifle", -1, 0.2f, RANGE_CLOSE, 0.f,
			{ FIRE_HOLD, AIM_LEAD, TOGGLE_NONE, AMMO_PRIMARY, 1, 2000.f, 0.f, 0.f, 0.f, 0.f, 0.f, WPN_Q_NO_UNDERWATER | WPN_Q_STICKY },
			kNoMode,
			kNoMode },
		{ OP4_SPORELAUNCHER, "weapon_sporelauncher", 5, 0.3f, RANGE_LONG, 0.f,
			{ FIRE_HOLD, AIM_LEAD, TOGGLE_NONE, AMMO_CLIP, 1, 2400.f, 0.f, 0.f, 200.f, 0.f, 0.f, WPN_Q_NONE },
			kNoMode,
			{ FIRE_HOLD, AIM_LEAD, TOGGLE_NONE, AMMO_CLIP, 1, 800.f, 1.f, 0.f, 200.f, 0.f, 0.f, WPN_Q_INHERIT_VEL } },
		{ OP4_SNIPERRIFLE, "weapon_sniperrifle", 5, 0.5f, RANGE_LONG, 18.f,
			kSniperShot,
			kSniperShot,
			{ FIRE_TAP, AIM_NONE, TOGGLE_ZOOM, AMMO_NONE, 0, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, WPN_Q_NONE } },
		{ OP4_KNIFE, "weapon_knife", -1, 0.1f, RANGE_CLOSE, 0.f,
			kSwing,
			kNoMode,
			kNoMode },
		{ OP4_PENGUIN, "weapon_penguin", -1, 0.2f, RANGE_CLOSE, 0.f,
			{ FIRE_HOLD, AIM_LEAD, TOGGLE_NONE, AMMO_PRIMARY, 1, 200.f, 0.5f, 0.f, 250.f, 0.f, 0.f, WPN_Q_INHERIT_VEL | WPN_Q_HOMING | WPN_Q_TEAM_UNSAFE },
			kNoMode,
			kNoMode },
	};
}

const WeaponSpec *Gearbox_WeaponSlot(int slot)
{
	if (slot < kHLDMWeaponSlots)
		return HLDM_WeaponSlot(slot);
	if (slot >= kGearboxWeaponSlots)
		return nullptr;

	return &kGearboxWeapons[slot - kHLDMWeaponSlots];
}

const WeaponSpec *Gearbox_WeaponSpec(int id)
{
	if (id < kHLDMWeaponSlots)
		return HLDM_WeaponSpec(id);

	const WeaponSpec *spec = Gearbox_WeaponSlot(id);
	if (!spec || !spec->classname || spec->id != id)
		return nullptr;

	return spec;
}
