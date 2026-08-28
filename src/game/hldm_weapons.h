#pragma once

#include "weapon_spec.h"

constexpr int kHLDMWeaponSlots = 16;

constexpr int kHLDMMelee = 1;

constexpr int kHLDMTripmine = 13;
constexpr int kHLDMSatchel = 14;

const WeaponSpec *HLDM_WeaponSpec(int id);

const WeaponSpec *HLDM_WeaponSlot(int slot);
