#pragma once

#include "../hldm/hldm_profile.h"

class CTFCProfile : public CHLDMProfile
{
public:
	const char *GetName() const override;
	bool Detect(const char *gamedir) const override;
	PmSim::FallRule FallDamage() const override;

	bool PreThink(edict_t *pEdict) const override;

	bool OffhandRead(edict_t *pEdict, int slot, OffhandInfo &out) const override;
	const char *OffhandCommand(int slot, bool prime) const override;
	bool OffhandPrimed(edict_t *pEdict) const override;
	const char *HazardEntity() const override;
	bool AbilityRun(CBot &bot, const Nav::NavMesh &mesh) const override;
	void RefreshWorld() const override;
	bool Disguised(edict_t *pEdict) const override;
	bool HoldFire(const CBot &bot) const override;
	bool ItemUsable(edict_t *player, edict_t *item) const override;
	bool Fortifies(edict_t *pEdict) const override;
	bool CanDetpack(edict_t *pEdict) const override;
	void NoteSound(edict_t *src, const char *sample) const override;
	void Patients(edict_t *medic, std::vector<edict_t *> &out) const override;

	ItemKind ClassifyItem(edict_t *ent) const override;
	float ChargerFullValue(ItemKind kind) const override;
	float ChargerValue(ItemKind kind, edict_t *pEdict) const override;

	const WeaponSpec *GetWeaponSpec(int id) const override;
	int WeaponSlotCount() const override;
	const WeaponSpec *WeaponSlot(int slot) const override;
	int ItemSlots() const override;
	bool ReadInventory(edict_t *player, BotInventory &out) const override;

	int MeleeWeapon() const override;
	bool IsMeleeWeapon(int id) const override;
	int AmbushWeapon(AmbushKind kind) const override;
	int DeployWeapon(DeployKind kind) const override;
	const char *DeployEntity(DeployKind kind) const override;

	bool ThrowLaunch(const WeaponMode &mode, float viewPitch, float &elevation,
		float &speed) const override;

	void ApplyAddArg(int iGame, edict_t *pEdict, const char *arg, char *out,
		unsigned int size) const override;
};
