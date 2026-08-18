#pragma once

#include <extdll.h>

#include "weapon_spec.h"

enum GameFlags : unsigned int
{
	GAME_NONE       = 0,
	GAME_DEATHMATCH = 1 << 0,
	GAME_TEAMPLAY   = 1 << 1,
};

enum ItemKind
{
	ITEM_NONE = 0,
	ITEM_WEAPON,
	ITEM_AMMO,
	ITEM_PICKUP,
	ITEM_HEALTHCHARGER,
	ITEM_RECHARGE,
};

inline bool ItemKindIsCharger(ItemKind kind)
{
	return kind == ITEM_HEALTHCHARGER || kind == ITEM_RECHARGE;
}

class IGameProfile
{
public:
	virtual ~IGameProfile() {}

	virtual const char *GetName() const = 0;
	virtual unsigned int GetFlags() const = 0;

	virtual bool Detect() const = 0;

	virtual bool IsDead(edict_t *pEdict) const = 0;

	virtual void Respawn(edict_t *pEdict) const = 0;

	virtual ItemKind ClassifyItem(const char *classname) const = 0;

	virtual float ItemRespawnDelay(ItemKind kind) const = 0;

	virtual float ItemFullTimeout() const = 0;

	virtual float ItemUnreachableTimeout() const = 0;

	virtual float ChargerFullValue(ItemKind kind) const = 0;

	virtual float ChargerValue(ItemKind kind, edict_t *pEdict) const = 0;

	virtual const WeaponSpec *GetWeaponSpec(int id) const = 0;
};

IGameProfile *GameProfile_Detect();

extern IGameProfile *g_pGameProfile;
