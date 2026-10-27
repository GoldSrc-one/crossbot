#pragma once

#include "../weapon_spec.h"

constexpr int kRewolfWeaponSlots = 31;

constexpr int kRewolfItemSlots = 11;

constexpr int kRewolfFists = 1;

const WeaponSpec *Rewolf_WeaponSpec(int id);

const WeaponSpec *Rewolf_WeaponSlot(int slot);

const char *Rewolf_WeaponSetup(int id);
