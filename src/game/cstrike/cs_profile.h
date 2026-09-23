#pragma once

#include "../hldm/hldm_profile.h"

class CCSProfile : public CHLDMProfile
{
public:
	const char *GetName() const override;
	bool Detect(const char *gamedir) const override;

	void Respawn(edict_t *pEdict) const override;
	bool PreThink(edict_t *pEdict) const override;
	bool AbilityRun(CBot &bot, const Nav::NavMesh &mesh) const override;
	unsigned short FireToggle(const CBot &bot, bool firing) const override;
	bool SpreadWide(edict_t *player, int id, void *weapon, float limit, float &ready) const override;
	bool ItemUsable(edict_t *player, edict_t *item) const override;
	bool Penetrates(const CBot &bot, int id, const Vector &eye, const Vector &dir,
		float entry) const override;

	ItemKind ClassifyItem(edict_t *ent) const override;

	const WeaponSpec *GetWeaponSpec(int id) const override;
	int WeaponSlotCount() const override;
	const WeaponSpec *WeaponSlot(int slot) const override;
	int ItemSlots() const override;

	bool ItemPreferred(edict_t *ent, ItemKind kind) const override;

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
