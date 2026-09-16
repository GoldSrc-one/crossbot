#pragma once

#include "../hldm/hldm_profile.h"

class CRicochetProfile : public CHLDMProfile
{
public:
	const char *GetName() const override;
	bool Detect(const char *gamedir) const override;
	float EvaluateCombat(edict_t *pEdict, const BotInventory &inv) const override;
	bool FallDamage() const override;
	bool AllowsGoalTier(BotGoal::Tier tier) const override;

	ItemKind ClassifyItem(edict_t *ent) const override;
	float ItemRespawnDelay(ItemKind kind) const override;
	bool ItemPreferred(edict_t *ent, ItemKind kind) const override;

	const WeaponSpec *GetWeaponSpec(int id) const override;
	int WeaponSlotCount() const override;
	const WeaponSpec *WeaponSlot(int slot) const override;
	bool ModeAvailable(edict_t *player, const WeaponSpec &spec, const WeaponMode &mode) const override;
	const char *AmmoEntity(int weaponId, WeaponAmmo pool) const override;

	int MeleeWeapon() const override;
	int AmbushWeapon(AmbushKind kind) const override;
	int DeployWeapon(DeployKind kind) const override;
	const char *DeployEntity(DeployKind kind) const override;
};
