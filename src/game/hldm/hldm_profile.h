#pragma once

#include "../game_profile.h"

class CHLDMProfile : public IGameProfile
{
public:
	const char *GetName() const override;
	unsigned int GetFlags() const override;

	bool Detect(const char *gamedir) const override;
	bool IsDead(edict_t *pEdict) const override;
	edict_t *MonsterEnemy(edict_t *ent) const override;
	float EvaluateCombat(edict_t *pEdict, const BotInventory &inv) const override;
	bool FallDamage() const override;
	bool AllowsGoalTier(BotGoal::Tier tier) const override;
	void Respawn(edict_t *pEdict) const override;

	ItemKind ClassifyItem(edict_t *ent) const override;
	float ItemRespawnDelay(ItemKind kind) const override;
	float ItemFullTimeout() const override;
	float ItemUnreachableTimeout() const override;
	float ChargerFullValue(ItemKind kind) const override;
	float ChargerValue(ItemKind kind, edict_t *pEdict) const override;

	const WeaponSpec *GetWeaponSpec(int id) const override;
	int WeaponSlotCount() const override;
	const WeaponSpec *WeaponSlot(int slot) const override;
	int ItemSlots() const override;
	bool ReadInventory(edict_t *player, BotInventory &out) const override;
	bool SelectWeapon(edict_t *player, const WeaponSpec &spec) const override;
	bool ModeAvailable(edict_t *player, const WeaponSpec &spec, const WeaponMode &mode) const override;

	const char *AmmoEntity(int weaponId, WeaponAmmo pool) const override;

	bool ItemPreferred(edict_t *ent, ItemKind kind) const override;

	int MeleeWeapon() const override;

	bool IsMeleeWeapon(int id) const override;

	int AmbushWeapon(AmbushKind kind) const override;

	int DeployWeapon(DeployKind kind) const override;

	const char *DeployEntity(DeployKind kind) const override;

	bool ThrowLaunch(const WeaponMode &mode, float viewPitch, float &elevation,
		float &speed) const override;

	bool IsFunTerminal(edict_t *ent) const override;

	void ApplyAddArg(int iGame, edict_t *pEdict, const char *arg, char *out,
		unsigned int size) const override;

protected:
	bool PreferredWeaponOrAmmo(const char *classname, const char *const *weapons,
		size_t count) const;
};
