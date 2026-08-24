#pragma once

#include "game_profile.h"

class CHLDMProfile : public IGameProfile
{
public:
	const char *GetName() const override;
	unsigned int GetFlags() const override;

	bool Detect() const override;
	bool IsDead(edict_t *pEdict) const override;
	void Respawn(edict_t *pEdict) const override;

	ItemKind ClassifyItem(const char *classname) const override;
	float ItemRespawnDelay(ItemKind kind) const override;
	float ItemFullTimeout() const override;
	float ItemUnreachableTimeout() const override;
	float ChargerFullValue(ItemKind kind) const override;
	float ChargerValue(ItemKind kind, edict_t *pEdict) const override;

	const WeaponSpec *GetWeaponSpec(int id) const override;

	const char *AmmoEntity(int weaponId, WeaponAmmo pool) const override;

	bool ThrowLaunch(const WeaponMode &mode, float viewPitch, float &elevation,
		float &speed) const override;
};
