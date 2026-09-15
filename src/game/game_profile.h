#pragma once

#include <extdll.h>

#include <vector>

#include "../bot/bot_goal.h"
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
	ITEM_HEALVOLUME,
};

inline bool ItemKindIsCharger(ItemKind kind)
{
	return kind == ITEM_HEALTHCHARGER || kind == ITEM_RECHARGE;
}

inline bool ItemKindIsVolume(ItemKind kind)
{
	return kind == ITEM_HEALVOLUME;
}

inline bool ItemKindIsBrush(ItemKind kind)
{
	return ItemKindIsCharger(kind) || ItemKindIsVolume(kind);
}

inline bool ItemKindUsesValue(ItemKind kind)
{
	return ItemKindIsCharger(kind) || ItemKindIsVolume(kind);
}

enum DeployKind
{
	DEPLOY_TRIPMINE = 0,
	DEPLOY_SATCHEL,
	DEPLOY_COUNT,
};

enum AmbushKind
{
	AMBUSH_GRENADE = 0,
	AMBUSH_SNARK,
	AMBUSH_COUNT,
};

struct BotInventory;

class IGameProfile
{
public:
	virtual ~IGameProfile() {}

	virtual const char *GetName() const = 0;
	virtual unsigned int GetFlags() const = 0;

	virtual bool Detect(const char *gamedir) const = 0;

	virtual bool IsDead(edict_t *pEdict) const = 0;

	virtual edict_t *MonsterEnemy(edict_t *ent) const = 0;

	virtual float EvaluateCombat(edict_t *pEdict, const BotInventory &inv) const = 0;

	virtual bool FallDamage() const = 0;

	virtual bool AllowsGoalTier(BotGoal::Tier tier) const = 0;

	virtual void Respawn(edict_t *pEdict) const = 0;

	virtual ItemKind ClassifyItem(edict_t *ent) const = 0;

	virtual float ItemRespawnDelay(ItemKind kind) const = 0;

	virtual float ItemFullTimeout() const = 0;

	virtual float ItemUnreachableTimeout() const = 0;

	virtual float ChargerFullValue(ItemKind kind) const = 0;

	virtual float ChargerValue(ItemKind kind, edict_t *pEdict) const = 0;

	virtual const WeaponSpec *GetWeaponSpec(int id) const = 0;

	virtual bool ModeAvailable(edict_t *player, const WeaponSpec &spec, const WeaponMode &mode) const = 0;

	virtual const char *AmmoEntity(int weaponId, WeaponAmmo pool) const = 0;

	virtual bool ItemPreferred(edict_t *ent, ItemKind kind) const = 0;

	virtual int MeleeWeapon() const = 0;

	virtual int AmbushWeapon(AmbushKind kind) const = 0;

	virtual int DeployWeapon(DeployKind kind) const = 0;

	virtual const char *DeployEntity(DeployKind kind) const = 0;

	virtual bool ThrowLaunch(const WeaponMode &mode, float viewPitch, float &elevation,
		float &speed) const = 0;

	virtual bool IsFunTerminal(edict_t *ent) const = 0;

	virtual void ApplyAddArg(int iGame, edict_t *pEdict, const char *arg, char *out,
		unsigned int size) const = 0;
};

IGameProfile *GameProfile_Detect(const char *gamedir);

extern IGameProfile *g_pGameProfile;
