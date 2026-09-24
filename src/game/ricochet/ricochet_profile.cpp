#include <extdll.h>
#include <meta_api.h>

#include <string.h>

#include "../crossplay.h"
#include "../ent_handle.h"
#include "../gamedata.h"
#include "ricochet_profile.h"

#if defined(_WIN32)
#define CB_STRICMP _stricmp
#else
#define CB_STRICMP strcasecmp
#endif

namespace
{
	constexpr int kRicochetDisc = 1;
	constexpr int kRicochetMaxDiscs = 3;
	constexpr float kRicochetDiscSpeed = 1000.f;
	constexpr float kRicochetPowerupRespawn = 10.f;
	constexpr int kRicochetPowHard = 1 << 2;

	const WeaponMode kNoMode = {};

	const WeaponSpec kRicochetDiscSpec = { kRicochetDisc, "weapon_disc", -1, 0.f, RANGE_NONE, 0.f,
		{ FIRE_TAP, AIM_LEAD, TOGGLE_NONE, AMMO_PRIMARY, 1, kRicochetDiscSpeed, 0.f, 0.f, 0.f, 0.f, 0.f, WPN_Q_NONE },
		kNoMode,
		{ FIRE_TAP, AIM_LEAD, TOGGLE_NONE, AMMO_PRIMARY, kRicochetMaxDiscs, kRicochetDiscSpeed, 0.f, 0.f, 0.f, 0.f, 0.f, WPN_Q_NONE } };
}

const char *CRicochetProfile::GetName() const
{
	return "Ricochet";
}

bool CRicochetProfile::Detect(const char *gamedir) const
{
	if (!gpGlobals->deathmatch_)
		return false;

	return gamedir && CB_STRICMP(gamedir, "ricochet") == 0;
}

float CRicochetProfile::EvaluateCombat(edict_t *, const BotInventory &) const
{
	return 0.f;
}

PmSim::FallRule CRicochetProfile::FallDamage() const
{
	return PmSim::kNoFall;
}

bool CRicochetProfile::AllowsGoalTier(BotGoal::Tier tier) const
{
	return tier == BotGoal::GOAL_GATHER;
}

ItemKind CRicochetProfile::ClassifyItem(edict_t *ent) const
{
	if (!ent || ent->free)
		return ITEM_NONE;

	return strncmp(CBotClassOf(ent), "item_", 5) == 0 ? ITEM_PICKUP : ITEM_NONE;
}

float CRicochetProfile::ItemRespawnDelay(ItemKind) const
{
	return kRicochetPowerupRespawn;
}

bool CRicochetProfile::ItemPreferred(edict_t *, ItemKind kind) const
{
	return kind == ITEM_PICKUP;
}

const WeaponSpec *CRicochetProfile::GetWeaponSpec(int id) const
{
	return id == kRicochetDisc ? &kRicochetDiscSpec : nullptr;
}

int CRicochetProfile::WeaponSlotCount() const
{
	return kRicochetDisc + 1;
}

const WeaponSpec *CRicochetProfile::WeaponSlot(int slot) const
{
	return GetWeaponSpec(slot);
}

bool CRicochetProfile::ModeAvailable(edict_t *player, const WeaponSpec &spec, const WeaponMode &mode) const
{
	if (&mode != &spec.primary)
		return true;

	if (!player || g_offsets.plr_iPowerups < 0 || !CBotPrivateData(player))
		return false;

	return (GameData::Peek<int>(CBotPrivateData(player), g_offsets.plr_iPowerups) & kRicochetPowHard) != 0;
}

const char *CRicochetProfile::AmmoEntity(int, WeaponAmmo) const
{
	return nullptr;
}

int CRicochetProfile::MeleeWeapon() const
{
	return 0;
}

int CRicochetProfile::AmbushWeapon(AmbushKind) const
{
	return 0;
}

int CRicochetProfile::DeployWeapon(DeployKind) const
{
	return 0;
}

const char *CRicochetProfile::DeployEntity(DeployKind) const
{
	return nullptr;
}
