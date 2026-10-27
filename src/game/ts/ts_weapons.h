#pragma once

#include "../weapon_spec.h"

constexpr int kTSWeaponSlots = 39;

constexpr int kTSKungFu = 38;
constexpr int kTSKatana = 34;

const WeaponSpec *TS_WeaponSpec(int id);

const WeaponSpec *TS_WeaponSlot(int slot);

int TS_AmmoType(int id);

struct TSLoadout
{
	const char *weapons;
	const char *ammo;
};

const TSLoadout &TS_Loadout(int index);
