#include <extdll.h>
#include <meta_api.h>

#include "bot.h"
#include "bot_weapons.h"
#include "../game/game_profile.h"
#include "../game/gamedata.h"

namespace
{

template <typename T>
T Peek(void *base, int offset)
{
	return *reinterpret_cast<T *>(reinterpret_cast<char *>(base) + offset);
}

const char *PlayerName(edict_t *player)
{
	if (!player || !gpGlobals->pStringBase)
		return "?";
	const char *s = gpGlobals->pStringBase + static_cast<unsigned int>(player->v.netname);
	return (s && s[0]) ? s : "?";
}

edict_t *ItemEdict(edict_t *player, void *item, int slot)
{
	entvars_t *pev = Peek<entvars_t *>(item, g_offsets.pev);
	if (!pev)
		GameData::Fatal("inventory slot %d of %s holds an item with a null pev", slot, PlayerName(player));

	edict_t *ent = pev->pContainingEntity;
	if (!ent || ent->free)
		GameData::Fatal("inventory slot %d of %s holds an item with no live edict", slot, PlayerName(player));

	int index = g_engfuncs.pfnIndexOfEdict(ent);
	if (index < 1 || index >= gpGlobals->maxEntities)
		GameData::Fatal("inventory slot %d of %s holds an item whose edict index is %d", slot, PlayerName(player), index);

	if (CBotPrivateData(ent) != item)
		GameData::Fatal("inventory slot %d of %s holds an item its own edict #%d does not point back at", slot, PlayerName(player), index);

	return ent;
}

}

namespace BotWeapons
{

void Read(edict_t *player, BotInventory &out)
{
	out = BotInventory();

	if (!player || player->free || !CBotPrivateData(player))
		return;

	if (g_pGameProfile && g_pGameProfile->ReadInventory(player, out))
		return;

	void *priv = CBotPrivateData(player);

	out.nextAttack = Peek<float>(priv, g_offsets.mon_flNextAttack);

	const int *ammo = reinterpret_cast<const int *>(reinterpret_cast<char *>(priv) + g_offsets.plr_rgAmmo);
	for (int i = 0; i < MAX_AMMO_SLOTS; ++i)
		out.ammo[i] = ammo[i];

	void *active = Peek<void *>(priv, g_offsets.plr_pActiveItem);
	void *const *items = reinterpret_cast<void *const *>(reinterpret_cast<char *>(priv) + g_offsets.plr_rgpPlayerItems);

	int slots = g_pGameProfile ? g_pGameProfile->ItemSlots() : MAX_ITEM_TYPES;
	for (int slot = 0; slot < slots; ++slot)
	{
		void *item = items[slot];
		for (int guard = 0; item && guard < kMaxBotWeapons; ++guard)
		{
			edict_t *ent = ItemEdict(player, item, slot);

			if (out.count >= kMaxBotWeapons)
				return;

			BotWeapon &w = out.weapons[out.count];
			w.ent = ent;
			w.priv = item;
			w.id = Peek<int>(item, g_offsets.item_iId);
			w.clip = Peek<int>(item, g_offsets.wpn_iClip);
			w.primaryAmmoType = Peek<int>(item, g_offsets.wpn_iPrimaryAmmoType);
			w.secondaryAmmoType = Peek<int>(item, g_offsets.wpn_iSecondaryAmmoType);
			w.nextPrimaryAttack = Peek<float>(item, g_offsets.wpn_flNextPrimaryAttack);
			w.nextSecondaryAttack = Peek<float>(item, g_offsets.wpn_flNextSecondaryAttack);
			w.reloading = Peek<int>(item, g_offsets.wpn_fInReload) != 0;
			if (g_offsets.ent_fInAttack >= 0)
				w.chargeState = Peek<int>(item, g_offsets.ent_fInAttack);
			if (g_offsets.ent_flStartThrow >= 0)
				w.throwStart = Peek<float>(item, g_offsets.ent_flStartThrow);

			const WeaponSpec *spec = g_pGameProfile ? g_pGameProfile->GetWeaponSpec(w.id) : nullptr;
			if (spec && (spec->primary.quirks & WPN_Q_KEEP_AIMING) != 0 && g_offsets.rpg_cActiveRockets >= 0)
				w.guidedCount = Peek<int>(item, g_offsets.rpg_cActiveRockets);
			if (spec && (spec->primary.quirks & WPN_Q_HIDDEN_STATE) != 0 && g_offsets.ent_chargeReady >= 0)
				w.chargeReady = Peek<int>(item, g_offsets.ent_chargeReady);
			if (spec && (spec->secondary.quirks & WPN_Q_LASER_ON) != 0 && g_offsets.eagle_iLaserActive >= 0)
				w.laserActive = Peek<int>(item, g_offsets.eagle_iLaserActive);

			if (item == active)
				out.activeIndex = out.count;

			++out.count;
			item = Peek<void *>(item, g_offsets.item_pNext);
		}
	}
}

int ActiveWeaponId(edict_t *player)
{
	if (!player || player->free || !CBotPrivateData(player))
		return 0;

	BotInventory own;
	if (g_pGameProfile && g_pGameProfile->ReadInventory(player, own))
		return (own.activeIndex >= 0 && own.activeIndex < own.count) ? own.weapons[own.activeIndex].id : 0;

	void *item = Peek<void *>(CBotPrivateData(player), g_offsets.plr_pActiveItem);
	if (!item)
		return 0;

	return Peek<int>(item, g_offsets.item_iId);
}

void Refresh(CBot &bot)
{
	Read(bot.pEdict, bot.inventory);
}

}
