#pragma once

#include "../hldm/hldm_profile.h"

class CTSProfile : public CHLDMProfile
{
public:
	void Respawn(edict_t *pEdict) const override;

	ItemKind ClassifyItem(edict_t *ent) const override;
	bool ItemPreferred(edict_t *ent, ItemKind kind) const override;

	const WeaponSpec *GetWeaponSpec(int id) const override;
	int WeaponSlotCount() const override;
	const WeaponSpec *WeaponSlot(int slot) const override;
	bool ReadInventory(edict_t *player, BotInventory &out) const override;
	bool SelectWeapon(edict_t *player, const WeaponSpec &spec) const override;

	const char *AmmoEntity(int weaponId, WeaponAmmo pool) const override;
};
