#pragma once

#include "../hldm/hldm_profile.h"

bool DMC_ThrowLaunch(float viewPitch, float &elevation, float &speed);

class CDMCProfile : public CHLDMProfile
{
public:
	bool PreThink(edict_t *pEdict) const override;
	bool Cloaked(edict_t *pEdict) const override;

	bool ItemPreferred(edict_t *ent, ItemKind kind) const override;

	const WeaponSpec *GetWeaponSpec(int id) const override;
	int WeaponSlotCount() const override;
	const WeaponSpec *WeaponSlot(int slot) const override;
	bool ReadInventory(edict_t *player, BotInventory &out) const override;
	bool SelectWeapon(edict_t *player, const WeaponSpec &spec) const override;
	const char *AmmoEntity(int weaponId, WeaponAmmo pool) const override;
};
