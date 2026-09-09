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

// Weapons worth opening with against someone who has not noticed the bot yet. A role rather than a
// type: nothing in WeaponSpec distinguishes a snark from a hornet gun (both are WPN_Q_HOMING), so
// the profile has to name them the way it already names the melee and deploy weapons.
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

	virtual void Respawn(edict_t *pEdict) const = 0;

	virtual ItemKind ClassifyItem(edict_t *ent) const = 0;

	virtual float ItemRespawnDelay(ItemKind kind) const = 0;

	virtual float ItemFullTimeout() const = 0;

	virtual float ItemUnreachableTimeout() const = 0;

	virtual float ChargerFullValue(ItemKind kind) const = 0;

	virtual float ChargerValue(ItemKind kind, edict_t *pEdict) const = 0;

	virtual const WeaponSpec *GetWeaponSpec(int id) const = 0;

	virtual const char *AmmoEntity(int weaponId, WeaponAmmo pool) const = 0;

	virtual bool ItemPreferred(edict_t *ent, ItemKind kind) const = 0;

	virtual int MeleeWeapon() const = 0;

	virtual int AmbushWeapon(AmbushKind kind) const = 0;

	virtual int DeployWeapon(DeployKind kind) const = 0;

	virtual const char *DeployEntity(DeployKind kind) const = 0;

	virtual bool ThrowLaunch(const WeaponMode &mode, float viewPitch, float &elevation,
		float &speed) const = 0;

	virtual bool IsFunTerminal(edict_t *ent) const = 0;

	virtual void CollectStrategicGoals(const CBot &bot, const Nav::NavMesh &mesh,
		std::vector<BotGoal::Candidate> &out) const = 0;

	// The optional third crossbot_add argument, whatever this game makes of it: a player model in
	// deathmatch, a team in a mod that has them. Called once on a fresh fake client, before
	// MDLL_ClientConnect, so the game DLL reads a finished userinfo buffer at connect. arg is null
	// when the operator gave none -- the profile picks its own default then, at random where that
	// makes sense -- and out receives what it settled on, for the console line.
	virtual void ApplyAddArg(int iGame, edict_t *pEdict, const char *arg, char *out,
		unsigned int size) const = 0;
};

IGameProfile *GameProfile_Detect(const char *gamedir);

extern IGameProfile *g_pGameProfile;
