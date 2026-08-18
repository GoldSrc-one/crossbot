#include <extdll.h>
#include <meta_api.h>

#include "bot.h"
#include "bot_weapons.h"
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

	if (ent->pvPrivateData != item)
		GameData::Fatal("inventory slot %d of %s holds an item its own edict #%d does not point back at", slot, PlayerName(player), index);

	return ent;
}

}

namespace BotWeapons
{

void Read(edict_t *player, BotInventory &out)
{
	out = BotInventory();

	if (!player || player->free || !player->pvPrivateData)
		return;

	void *priv = player->pvPrivateData;

	out.nextAttack = Peek<float>(priv, g_offsets.mon_flNextAttack);

	const int *ammo = reinterpret_cast<const int *>(reinterpret_cast<char *>(priv) + g_offsets.plr_rgAmmo);
	for (int i = 0; i < MAX_AMMO_SLOTS; ++i)
		out.ammo[i] = ammo[i];

	void *active = Peek<void *>(priv, g_offsets.plr_pActiveItem);
	void *const *items = reinterpret_cast<void *const *>(reinterpret_cast<char *>(priv) + g_offsets.plr_rgpPlayerItems);

	for (int slot = 0; slot < MAX_ITEM_TYPES; ++slot)
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

			if (item == active)
				out.activeIndex = out.count;

			++out.count;
			item = Peek<void *>(item, g_offsets.item_pNext);
		}
	}
}

void Refresh(CBot &bot)
{
	Read(bot.pEdict, bot.inventory);
}

}
