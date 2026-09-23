#include "tfc_weapons.h"

namespace
{
	const WeaponMode kNoMode = {};

	const WeaponMode kMelee = { FIRE_HOLD, AIM_TARGET, TOGGLE_NONE, AMMO_NONE, 0, 0.f, 0.f, 32.f, 0.f, 0.f, 0.f, WPN_Q_NONE };

	const WeaponSpec kTFCWeapons[kTFCWeaponSlots] = {
		{},
		{},
		{},
		{ TFC_MEDIKIT, "tf_weapon_medikit", -1, 0.05f, RANGE_CLOSE, 0.f, kMelee, kNoMode, kNoMode },
		{ TFC_SPANNER, "tf_weapon_spanner", -1, 0.05f, RANGE_CLOSE, 0.f, kMelee, kNoMode, kNoMode },
		{ TFC_AXE, "tf_weapon_axe", -1, 0.05f, RANGE_CLOSE, 0.f, kMelee, kNoMode, kNoMode },
		{ TFC_SNIPERRIFLE, "tf_weapon_sniperrifle", -1, 0.5f, RANGE_LONG, 0.f,
			{ FIRE_CHARGE, AIM_TARGET, TOGGLE_NONE, AMMO_PRIMARY, 1, 0.f, 0.f, 0.f, 0.f, 2.f, 12.f, WPN_Q_BRACE },
			kNoMode,
			kNoMode },
		{ TFC_AUTORIFLE, "tf_weapon_autorifle", -1, 0.3f, RANGE_LONG, 0.f,
			{ FIRE_HOLD, AIM_TARGET, TOGGLE_NONE, AMMO_PRIMARY, 1, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, WPN_Q_STICKY },
			kNoMode,
			kNoMode },
		{ TFC_SHOTGUN, "tf_weapon_shotgun", 8, 0.15f, RANGE_CLOSE, 0.f,
			{ FIRE_HOLD, AIM_TARGET, TOGGLE_NONE, AMMO_CLIP, 1, 0.f, 0.f, 2048.f, 0.f, 0.f, 0.f, WPN_Q_NONE },
			kNoMode,
			kNoMode },
		{ TFC_SUPERSHOTGUN, "tf_weapon_supershotgun", 16, 0.25f, RANGE_CLOSE, 0.f,
			{ FIRE_HOLD, AIM_TARGET, TOGGLE_NONE, AMMO_CLIP, 2, 0.f, 0.f, 2048.f, 0.f, 0.f, 0.f, WPN_Q_NONE },
			kNoMode,
			kNoMode },
		{ TFC_NAILGUN, "tf_weapon_ng", -1, 0.3f, RANGE_CLOSE, 0.f,
			{ FIRE_HOLD, AIM_LEAD, TOGGLE_NONE, AMMO_PRIMARY, 1, 1000.f, 0.f, 0.f, 0.f, 0.f, 0.f, WPN_Q_STICKY },
			kNoMode,
			kNoMode },
		{ TFC_SUPERNAILGUN, "tf_weapon_superng", -1, 0.45f, RANGE_CLOSE, 0.f,
			{ FIRE_HOLD, AIM_LEAD, TOGGLE_NONE, AMMO_PRIMARY, 2, 1000.f, 0.f, 0.f, 0.f, 0.f, 0.f, WPN_Q_STICKY },
			kNoMode,
			kNoMode },
		{ TFC_GL, "tf_weapon_gl", 6, 0.4f, RANGE_CLOSE, 0.f,
			{ FIRE_HOLD, AIM_LEAD, TOGGLE_NONE, AMMO_CLIP, 1, 632.5f, 1.f, 0.f, 160.f, 0.f, 0.f, WPN_Q_THROW_ARC },
			kNoMode,
			kNoMode },
		{ TFC_FLAMETHROWER, "tf_weapon_flamethrower", -1, 0.45f, RANGE_CLOSE, 0.f,
			{ FIRE_HOLD, AIM_LEAD, TOGGLE_NONE, AMMO_PRIMARY, 1, 600.f, 0.f, 300.f, 0.f, 0.f, 0.f, WPN_Q_STICKY | WPN_Q_NO_UNDERWATER },
			kNoMode,
			kNoMode },
		{ TFC_RPG, "tf_weapon_rpg", 4, 0.6f, RANGE_LONG, 0.f,
			{ FIRE_HOLD, AIM_LEAD, TOGGLE_NONE, AMMO_CLIP, 1, 900.f, 0.f, 0.f, 160.f, 0.f, 0.f, WPN_Q_AIM_FEET },
			kNoMode,
			kNoMode },
		{ TFC_IC, "tf_weapon_ic", -1, 0.55f, RANGE_CLOSE, 0.f,
			{ FIRE_HOLD, AIM_LEAD, TOGGLE_NONE, AMMO_PRIMARY, 1, 600.f, 0.f, 0.f, 160.f, 0.f, 0.f, WPN_Q_AIM_FEET },
			kNoMode,
			kNoMode },
		{},
		{ TFC_AC, "tf_weapon_ac", -1, 0.7f, RANGE_CLOSE, 0.f,
			{ FIRE_HOLD, AIM_TARGET, TOGGLE_NONE, AMMO_PRIMARY, 1, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, WPN_Q_STICKY },
			kNoMode,
			kNoMode },
		{},
		{},
		{ TFC_TRANQ, "tf_weapon_tranq", -1, 0.2f, RANGE_LONG, 0.f,
			{ FIRE_HOLD, AIM_LEAD, TOGGLE_NONE, AMMO_PRIMARY, 1, 1500.f, 0.f, 0.f, 0.f, 0.f, 0.f, WPN_Q_NONE },
			kNoMode,
			kNoMode },
		{ TFC_RAILGUN, "tf_weapon_railgun", -1, 0.25f, RANGE_LONG, 0.f,
			{ FIRE_HOLD, AIM_LEAD, TOGGLE_NONE, AMMO_PRIMARY, 1, 1500.f, 0.f, 0.f, 0.f, 0.f, 0.f, WPN_Q_NONE },
			kNoMode,
			kNoMode },
		{ TFC_PL, "tf_weapon_pl", 6, 0.02f, RANGE_CLOSE, 0.f,
			{ FIRE_HOLD, AIM_LEAD, TOGGLE_NONE, AMMO_CLIP, 1, 632.5f, 1.f, 0.f, 160.f, 0.f, 0.f, WPN_Q_THROW_ARC },
			kNoMode,
			kNoMode },
		{ TFC_KNIFE, "tf_weapon_knife", -1, 0.1f, RANGE_CLOSE, 0.f, kMelee, kNoMode, kNoMode },
	};
}

const WeaponSpec *TFC_WeaponSpec(int id)
{
	if (id <= 0 || id >= kTFCWeaponSlots)
		return nullptr;

	const WeaponSpec &spec = kTFCWeapons[id];
	if (!spec.classname || spec.id != id)
		return nullptr;

	return &spec;
}

const WeaponSpec *TFC_WeaponSlot(int slot)
{
	if (slot < 0 || slot >= kTFCWeaponSlots)
		return nullptr;

	return &kTFCWeapons[slot];
}

TFCAmmoField TFC_AmmoField(int id)
{
	switch (id)
	{
	case TFC_SNIPERRIFLE:
	case TFC_AUTORIFLE:
	case TFC_SHOTGUN:
	case TFC_SUPERSHOTGUN:
	case TFC_AC:
	case TFC_TRANQ:
		return TFC_FIELD_SHELLS;
	case TFC_NAILGUN:
	case TFC_SUPERNAILGUN:
	case TFC_RAILGUN:
		return TFC_FIELD_NAILS;
	case TFC_SPANNER:
	case TFC_FLAMETHROWER:
		return TFC_FIELD_CELLS;
	case TFC_GL:
	case TFC_RPG:
	case TFC_IC:
	case TFC_PL:
		return TFC_FIELD_ROCKETS;
	default:
		return TFC_FIELD_NONE;
	}
}
