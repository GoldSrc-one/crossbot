#pragma once

#include "game_profile.h"

class CHLDMProfile : public IGameProfile
{
public:
	const char *GetName() const override;
	unsigned int GetFlags() const override;

	bool Detect() const override;
	bool IsDead(edict_t *pEdict) const override;
	float EvaluateCombat(edict_t *pEdict, const BotInventory &inv) const override;
	void Respawn(edict_t *pEdict) const override;

	ItemKind ClassifyItem(edict_t *ent) const override;
	float ItemRespawnDelay(ItemKind kind) const override;
	float ItemFullTimeout() const override;
	float ItemUnreachableTimeout() const override;
	float ChargerFullValue(ItemKind kind) const override;
	float ChargerValue(ItemKind kind, edict_t *pEdict) const override;

	const WeaponSpec *GetWeaponSpec(int id) const override;

	const char *AmmoEntity(int weaponId, WeaponAmmo pool) const override;

	int MeleeWeapon() const override;

	int AmbushWeapon(AmbushKind kind) const override;

	int DeployWeapon(DeployKind kind) const override;

	const char *DeployEntity(DeployKind kind) const override;

	bool ThrowLaunch(const WeaponMode &mode, float viewPitch, float &elevation,
		float &speed) const override;

	bool IsFunTerminal(edict_t *ent) const override;

	void CollectStrategicGoals(const CBot &bot, const Nav::NavMesh &mesh,
		std::vector<BotGoal::Candidate> &out) const override;
};
