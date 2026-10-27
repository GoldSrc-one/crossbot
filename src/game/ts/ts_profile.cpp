#include <extdll.h>
#include <meta_api.h>
#include <commonmacros.h>

#include <stdio.h>
#include <string.h>

#include "../../bot/bot_weapons.h"
#include "../crossplay.h"
#include "../ent_handle.h"
#include "../fake_cmd.h"
#include "../gamedata.h"
#include "ts_profile.h"
#include "ts_weapons.h"

namespace
{
	constexpr int kMaxPlayers = 32;
	constexpr int kTSGunId = 16;
	constexpr int kTSAmmoTypes = 16;
	constexpr int kTSStatusSize = 56;
	constexpr int kTSStatusOwned = 4;
	constexpr int kTSStatusClip = 8;
	constexpr float kTSSpawnRetry = 1.f;

	float g_nextSpawn[kMaxPlayers + 1];

	bool HasTSOffsets()
	{
		return g_offsets.tsgun_iWeapon >= 0 && g_offsets.tsgun_rgAmmo >= 0 && g_offsets.tsgun_rgStatus >= 0;
	}
}

const char *CTSProfile::GetName() const
{
	return "TS";
}

bool CTSProfile::Detect(const char *gamedir) const
{
	if (!gpGlobals->deathmatch_)
		return false;

	return gamedir && CB_STRICMP(gamedir, "ts") == 0;
}

void CTSProfile::Respawn(edict_t *pEdict) const
{
	CHLDMProfile::Respawn(pEdict);

	int index = ENTINDEX(pEdict);
	if (index <= 0 || index > kMaxPlayers)
		return;

	float now = gpGlobals->time;
	if (g_nextSpawn[index] > now && g_nextSpawn[index] - now <= kTSSpawnRetry)
		return;
	g_nextSpawn[index] = now + kTSSpawnRetry;

	FakeCmd::Run(pEdict, "rst_cfg");
	const TSLoadout &loadout = TS_Loadout(index);
	FakeCmd::Run(pEdict, loadout.weapons);
	FakeCmd::Run(pEdict, loadout.ammo);
	FakeCmd::Run(pEdict, "respawn");
}

ItemKind CTSProfile::ClassifyItem(edict_t *ent) const
{
	if (!ent || ent->free)
		return ITEM_NONE;

	const char *classname = CBotClassOf(ent);
	if (strcmp(classname, "ts_powerup") == 0)
		return ITEM_PICKUP;
	if (strncmp(classname, "ts_", 3) == 0 || strncmp(classname, "weapon_", 7) == 0 ||
		strncmp(classname, "ammo_", 5) == 0 || strcmp(classname, "weaponbox") == 0)
		return ITEM_NONE;

	return CHLDMProfile::ClassifyItem(ent);
}

bool CTSProfile::ItemPreferred(edict_t *ent, ItemKind kind) const
{
	if (ItemKindIsCharger(kind) || ItemKindIsVolume(kind))
		return true;

	const char *classname = CBotClassOf(ent);
	return strcmp(classname, "ts_powerup") == 0 || strcmp(classname, "item_healthkit") == 0 ||
		strcmp(classname, "item_battery") == 0;
}

const WeaponSpec *CTSProfile::GetWeaponSpec(int id) const
{
	return TS_WeaponSpec(id);
}

int CTSProfile::WeaponSlotCount() const
{
	return kTSWeaponSlots;
}

const WeaponSpec *CTSProfile::WeaponSlot(int slot) const
{
	return TS_WeaponSlot(slot);
}

bool CTSProfile::ReadInventory(edict_t *player, BotInventory &out) const
{
	if (!HasTSOffsets())
		GameData::Fatal("TS profile is active but the gamedata lacks the CTSGun fields");

	void *priv = CBotPrivateData(player);
	if (!priv)
		return true;

	out.nextAttack = GameData::Peek<float>(priv, g_offsets.mon_flNextAttack);

	void *gun = GameData::Peek<void *>(priv, g_offsets.plr_pActiveItem);
	if (!gun || GameData::Peek<int>(gun, g_offsets.item_iId) != kTSGunId)
		return true;

	for (int i = 0; i < kTSAmmoTypes; ++i)
		out.ammo[i] = GameData::Peek<int>(gun, g_offsets.tsgun_rgAmmo + i * 4);

	int current = GameData::Peek<int>(gun, g_offsets.tsgun_iWeapon);

	for (int id = 1; id < kTSWeaponSlots && out.count < kMaxBotWeapons; ++id)
	{
		bool kungFu = id == kTSKungFu;
		int status = g_offsets.tsgun_rgStatus + id * kTSStatusSize;
		if (!kungFu && GameData::Peek<unsigned char>(gun, status + kTSStatusOwned) == 0)
			continue;

		BotWeapon &w = out.weapons[out.count];
		w.priv = gun;
		w.id = id;
		w.clip = kungFu ? -1 : GameData::Peek<int>(gun, status + kTSStatusClip);
		w.primaryAmmoType = TS_AmmoType(id);

		if (id == current || (kungFu && current == 0))
		{
			w.nextPrimaryAttack = GameData::Peek<float>(gun, g_offsets.wpn_flNextPrimaryAttack);
			w.nextSecondaryAttack = GameData::Peek<float>(gun, g_offsets.wpn_flNextSecondaryAttack);
			w.reloading = GameData::Peek<int>(gun, g_offsets.wpn_fInReload) != 0;
			out.activeIndex = out.count;
		}

		++out.count;
	}

	return true;
}

bool CTSProfile::SelectWeapon(edict_t *player, const WeaponSpec &spec) const
{
	char cmd[32];
	snprintf(cmd, sizeof(cmd), "weapon_%d", spec.id == kTSKungFu ? 0 : spec.id);
	return FakeCmd::Run(player, cmd);
}

const char *CTSProfile::AmmoEntity(int, WeaponAmmo) const
{
	return NULL;
}

int CTSProfile::MeleeWeapon() const
{
	return kTSKungFu;
}

bool CTSProfile::IsMeleeWeapon(int id) const
{
	return id == kTSKungFu || id == kTSKatana;
}

int CTSProfile::AmbushWeapon(AmbushKind) const
{
	return 0;
}

int CTSProfile::DeployWeapon(DeployKind) const
{
	return 0;
}

const char *CTSProfile::DeployEntity(DeployKind) const
{
	return NULL;
}
