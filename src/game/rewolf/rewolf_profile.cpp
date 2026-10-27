#include <extdll.h>
#include <meta_api.h>
#include <commonmacros.h>

#include <algorithm>
#include <string.h>

#include "../../bot/bot_weapons.h"
#include "../crossplay.h"
#include "../ent_handle.h"
#include "../fake_cmd.h"
#include "../gamedata.h"
#include "rewolf_profile.h"
#include "rewolf_weapons.h"

namespace
{
	constexpr int kMaxPlayers = 32;

	void *g_configured[kMaxPlayers + 1];
}

const char *CRewolfProfile::GetName() const
{
	return "Gunman";
}

bool CRewolfProfile::Detect(const char *gamedir) const
{
	if (!gpGlobals->deathmatch_)
		return false;

	return gamedir && CB_STRICMP(gamedir, "rewolf") == 0;
}

bool CRewolfProfile::PreThink(edict_t *pEdict) const
{
	int index = ENTINDEX(pEdict);
	if (index <= 0 || index > kMaxPlayers)
		return false;

	void *priv = CBotPrivateData(pEdict);
	if (!priv || IsDead(pEdict))
	{
		g_configured[index] = nullptr;
		return false;
	}

	void *active = GameData::Peek<void *>(priv, g_offsets.plr_pActiveItem);
	if (!active || active == g_configured[index])
		return false;

	g_configured[index] = active;

	const char *setup = Rewolf_WeaponSetup(GameData::Peek<int>(active, g_offsets.item_iId));
	if (setup)
		FakeCmd::Run(pEdict, setup);

	return false;
}

const WeaponSpec *CRewolfProfile::GetWeaponSpec(int id) const
{
	return Rewolf_WeaponSpec(id);
}

int CRewolfProfile::WeaponSlotCount() const
{
	return kRewolfWeaponSlots;
}

const WeaponSpec *CRewolfProfile::WeaponSlot(int slot) const
{
	return Rewolf_WeaponSlot(slot);
}

int CRewolfProfile::ItemSlots() const
{
	return kRewolfItemSlots;
}

bool CRewolfProfile::ReadInventory(edict_t *player, BotInventory &out) const
{
	BotWeapons::ReadItems(player, out);

	float now = gpGlobals->time;
	out.nextAttack -= now;
	for (int i = 0; i < out.count; ++i)
	{
		BotWeapon &w = out.weapons[i];
		w.nextPrimaryAttack -= now;
		w.nextSecondaryAttack -= now;
		if (i == out.activeIndex)
			w.nextPrimaryAttack = std::max(w.nextPrimaryAttack, out.nextAttack);
	}

	return true;
}

const char *CRewolfProfile::AmmoEntity(int weaponId, WeaponAmmo pool) const
{
	const WeaponSpec *spec = GetWeaponSpec(weaponId);
	if (!spec || !spec->classname || pool == AMMO_NONE)
		return NULL;

	static const struct
	{
		const char *weapon;
		const char *clip;
		const char *primary;
	} kAmmo[] = {
		{ "weapon_gausspistol",   NULL,           "ammo_gaussclip" },
		{ "weapon_shotgun",       NULL,           "ammo_buckshot" },
		{ "weapon_minigun",       NULL,           "ammo_minigunClip" },
		{ "weapon_beamgun",       NULL,           "ammo_beamgunclip" },
		{ "weapon_dml",           "ammo_dmlclip", NULL },
		{ "weapon_SPchemicalgun", NULL,           "ammo_chemical" },
	};

	for (size_t i = 0; i < ARRAYSIZE(kAmmo); ++i)
	{
		if (strcmp(kAmmo[i].weapon, spec->classname) != 0)
			continue;
		if (pool == AMMO_CLIP)
			return kAmmo[i].clip;
		if (pool == AMMO_PRIMARY)
			return kAmmo[i].primary;
		return NULL;
	}

	return NULL;
}

bool CRewolfProfile::ItemPreferred(edict_t *ent, ItemKind kind) const
{
	if (ItemKindIsCharger(kind) || ItemKindIsVolume(kind))
		return true;

	const char *classname = CBotClassOf(ent);
	if (!classname || !classname[0])
		return false;

	if (strcmp(classname, "item_healthkit") == 0 || strcmp(classname, "item_armor") == 0)
		return true;

	static const char *const kPreferred[] = { "weapon_dml", "weapon_minigun" };

	return PreferredWeaponOrAmmo(classname, kPreferred, ARRAYSIZE(kPreferred));
}

int CRewolfProfile::MeleeWeapon() const
{
	return kRewolfFists;
}

int CRewolfProfile::AmbushWeapon(AmbushKind) const
{
	return 0;
}

int CRewolfProfile::DeployWeapon(DeployKind) const
{
	return 0;
}

const char *CRewolfProfile::DeployEntity(DeployKind) const
{
	return NULL;
}
