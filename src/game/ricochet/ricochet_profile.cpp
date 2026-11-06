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
	constexpr int kRicochetPowHard = 1 << 2;

	const WeaponMode kNoMode = {};

	const WeaponSpec kRicochetDiscSpec = { kRicochetDisc, "weapon_disc", -1, 0.f, RANGE_NONE, 0.f,
		{ FIRE_TAP, AIM_LEAD, TOGGLE_NONE, AMMO_PRIMARY, 1, kRicochetDiscSpeed, 0.f, 0.f, 0.f, 0.f, 0.f, WPN_Q_NONE },
		kNoMode,
		{ FIRE_TAP, AIM_LEAD, TOGGLE_NONE, AMMO_PRIMARY, kRicochetMaxDiscs, kRicochetDiscSpeed, 0.f, 0.f, 0.f, 0.f, 0.f, WPN_Q_NONE } };
}

float CRicochetProfile::EvaluateCombat(edict_t *, const BotInventory &) const
{
	return 0.f;
}

ItemKind CRicochetProfile::ClassifyItem(edict_t *ent) const
{
	if (!ent || ent->free)
		return ITEM_NONE;

	return strncmp(CBotClassOf(ent), "item_", 5) == 0 || IsArmouryArmor(ent) ? ITEM_PICKUP : ITEM_NONE;
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

bool CRicochetProfile::MeleeOnlyAllows(int id) const
{
	return id == kRicochetDisc;
}

