#pragma once

#include "../weapon_spec.h"

constexpr int kTFCWeaponSlots = 24;

constexpr int kTFCItemSlots = 6;

enum TFCWeaponId
{
	TFC_MEDIKIT      = 3,
	TFC_SPANNER      = 4,
	TFC_AXE          = 5,
	TFC_SNIPERRIFLE  = 6,
	TFC_AUTORIFLE    = 7,
	TFC_SHOTGUN      = 8,
	TFC_SUPERSHOTGUN = 9,
	TFC_NAILGUN      = 10,
	TFC_SUPERNAILGUN = 11,
	TFC_GL           = 12,
	TFC_FLAMETHROWER = 13,
	TFC_RPG          = 14,
	TFC_IC           = 15,
	TFC_AC           = 17,
	TFC_TRANQ        = 20,
	TFC_RAILGUN      = 21,
	TFC_PL           = 22,
	TFC_KNIFE        = 23,
};

enum TFCAmmoField
{
	TFC_FIELD_NONE = -1,
	TFC_FIELD_SHELLS = 0,
	TFC_FIELD_NAILS,
	TFC_FIELD_CELLS,
	TFC_FIELD_ROCKETS,
};

const WeaponSpec *TFC_WeaponSpec(int id);

const WeaponSpec *TFC_WeaponSlot(int slot);

TFCAmmoField TFC_AmmoField(int id);
