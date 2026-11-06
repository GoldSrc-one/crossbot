#pragma once

#include "../hldm/hldm_profile.h"

class CRicochetProfile : public CHLDMProfile
{
public:
	float EvaluateCombat(edict_t *pEdict, const BotInventory &inv) const override;
	ItemKind ClassifyItem(edict_t *ent) const override;
	bool ItemPreferred(edict_t *ent, ItemKind kind) const override;

	const WeaponSpec *GetWeaponSpec(int id) const override;
	int WeaponSlotCount() const override;
	const WeaponSpec *WeaponSlot(int slot) const override;
	bool ModeAvailable(edict_t *player, const WeaponSpec &spec, const WeaponMode &mode) const override;
	bool MeleeOnlyAllows(int id) const override;
};
