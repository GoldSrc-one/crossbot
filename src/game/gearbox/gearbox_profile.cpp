#include <extdll.h>
#include <meta_api.h>
#include <commonmacros.h>

#include <string.h>

#include "../crossplay.h"
#include "../ent_handle.h"
#include "../hldm/hldm_weapons.h"
#include "gearbox_profile.h"
#include "gearbox_weapons.h"

const char *CGearboxProfile::GetName() const
{
	return "Gearbox";
}

bool CGearboxProfile::Detect(const char *gamedir) const
{
	if (!gpGlobals->deathmatch_)
		return false;

	return gamedir && CB_STRICMP(gamedir, "gearbox") == 0;
}

const WeaponSpec *CGearboxProfile::GetWeaponSpec(int id) const
{
	return Gearbox_WeaponSpec(id);
}

int CGearboxProfile::WeaponSlotCount() const
{
	return kGearboxWeaponSlots;
}

const WeaponSpec *CGearboxProfile::WeaponSlot(int slot) const
{
	return Gearbox_WeaponSlot(slot);
}

int CGearboxProfile::ItemSlots() const
{
	return kGearboxItemSlots;
}

const char *CGearboxProfile::AmmoEntity(int weaponId, WeaponAmmo pool) const
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
		{ "weapon_eagle",         "ammo_357",   NULL },
		{ "weapon_m249",          "ammo_556",   NULL },
		{ "weapon_sniperrifle",   "ammo_762",   NULL },
		{ "weapon_sporelauncher", "ammo_spore", NULL },
		{ "weapon_displacer",     NULL,         "ammo_gaussclip" },
		{ "weapon_penguin",       NULL,         "weapon_penguin" },
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

	return CHLDMProfile::AmmoEntity(weaponId, pool);
}

bool CGearboxProfile::ItemPreferred(edict_t *ent, ItemKind kind) const
{
	if (CHLDMProfile::ItemPreferred(ent, kind))
		return true;

	const char *classname = CBotClassOf(ent);
	if (!classname || !classname[0])
		return false;

	static const char *const kPreferred[] = { "weapon_displacer", "weapon_sniperrifle" };

	return PreferredWeaponOrAmmo(classname, kPreferred, ARRAYSIZE(kPreferred));
}

bool CGearboxProfile::IsMeleeWeapon(int id) const
{
	return id == kHLDMMelee || id == kGearboxPipewrench || id == kGearboxKnife ||
		id == kGearboxGrapple;
}

int CGearboxProfile::AmbushWeapon(AmbushKind kind) const
{
	switch (kind)
	{
	case AMBUSH_PENGUIN: return kGearboxPenguin;
	case AMBUSH_GRAPPLE: return kGearboxGrapple;
	default: return CHLDMProfile::AmbushWeapon(kind);
	}
}
