#pragma once

#include "../hldm/hldm_profile.h"

class CGearboxProfile : public CHLDMProfile
{
public:
	const char *GetName() const override;
	bool Detect(const char *gamedir) const override;

	const WeaponSpec *GetWeaponSpec(int id) const override;
	int WeaponSlotCount() const override;
	const WeaponSpec *WeaponSlot(int slot) const override;
	int ItemSlots() const override;

	const char *AmmoEntity(int weaponId, WeaponAmmo pool) const override;
	bool ItemPreferred(edict_t *ent, ItemKind kind) const override;

	bool IsMeleeWeapon(int id) const override;
	int AmbushWeapon(AmbushKind kind) const override;
};
