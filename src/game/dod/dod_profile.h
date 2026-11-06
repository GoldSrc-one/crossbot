#pragma once

#include "../hldm/hldm_profile.h"

class CDODProfile : public CHLDMProfile
{
public:
	float EvaluateCombat(edict_t *pEdict, const BotInventory &inv) const override;
	bool PreThink(edict_t *pEdict) const override;
	bool AbilityRun(CBot &bot, const Nav::NavMesh &mesh) const override;
	int Abilities(const AbilityInfo *&list) const override;
	unsigned short FireToggle(const CBot &bot, bool firing) const override;
	bool Zoomed(edict_t *pEdict) const override;
	void AdjustView(CBot &bot, float dt, float &pitch, float &yaw) const override;
	void MoveAdjust(CBot &bot, float &fmove, float &smove, unsigned short &buttons) const override;
	unsigned short CreepButtons() const override { return IN_DUCK; }
	bool BraceSpeed(edict_t *, int, void *, float, float &speed) const override { speed = 45.f; return true; }
	float MaxSpeed(edict_t *pEdict, float engineMax) const override;
	bool CanDeploy(edict_t *pEdict) const override;
	bool CanProne(edict_t *pEdict) const override;
	float BlindSeconds(float fadeSec, float holdSec, float scale) const override;
	bool ItemUsable(edict_t *player, edict_t *item) const override;

	ItemKind ClassifyItem(edict_t *ent) const override;

	const WeaponSpec *GetWeaponSpec(int id) const override;
	int WeaponSlotCount() const override;
	const WeaponSpec *WeaponSlot(int slot) const override;
	bool ReadInventory(edict_t *player, BotInventory &out) const override;
	bool ModeAvailable(edict_t *player, const WeaponSpec &spec, const WeaponMode &mode) const override;

	const char *AmmoEntity(int weaponId, WeaponAmmo pool) const override;
	bool ItemPreferred(edict_t *ent, ItemKind kind) const override;

	void ApplyAddArg(int iGame, edict_t *pEdict, const char *arg, char *out,
		unsigned int size) const override;
};
