#pragma once

#include "../weapon_spec.h"

constexpr int kGearboxWeaponSlots = 27;

constexpr int kGearboxItemSlots = 7;

constexpr int kGearboxGrapple = 16;
constexpr int kGearboxPipewrench = 18;
constexpr int kGearboxKnife = 25;
constexpr int kGearboxPenguin = 26;

const WeaponSpec *Gearbox_WeaponSpec(int id);

const WeaponSpec *Gearbox_WeaponSlot(int slot);
