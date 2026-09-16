#pragma once

#include "../hldm/hldm_profile.h"

class CDMCProfile : public CHLDMProfile
{
public:
	const char *GetName() const override;
	bool Detect(const char *gamedir) const override;
	bool FallDamage() const override;

	float ItemRespawnDelay(ItemKind kind) const override;
	bool ItemPreferred(edict_t *ent, ItemKind kind) const override;

	const WeaponSpec *GetWeaponSpec(int id) const override;
	int WeaponSlotCount() const override;
	const WeaponSpec *WeaponSlot(int slot) const override;
	bool ReadInventory(edict_t *player, BotInventory &out) const override;
	bool SelectWeapon(edict_t *player, const WeaponSpec &spec) const override;
	const char *AmmoEntity(int weaponId, WeaponAmmo pool) const override;

	int MeleeWeapon() const override;
	int AmbushWeapon(AmbushKind kind) const override;
	int DeployWeapon(DeployKind kind) const override;
	const char *DeployEntity(DeployKind kind) const override;

	bool ThrowLaunch(const WeaponMode &mode, float viewPitch, float &elevation,
		float &speed) const override;
};
