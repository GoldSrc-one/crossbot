#pragma once

struct GameOffsets
{
	int pev;

	int ent_fInAttack;
	int ent_flStartThrow;

	int item_pPlayer;
	int item_pNext;
	int item_iId;

	int wpn_iClip;
	int wpn_iPrimaryAmmoType;
	int wpn_iSecondaryAmmoType;
	int wpn_fInReload;
	int wpn_flNextPrimaryAttack;
	int wpn_flNextSecondaryAttack;
	int wpn_flTimeWeaponIdle;

	int rpg_cActiveRockets;

	int mon_flNextAttack;

	int plr_pActiveItem;
	int plr_rgpPlayerItems;
	int plr_rgAmmo;
};

extern GameOffsets g_offsets;

namespace GameData
{
	template <typename T>
	inline T Peek(void *base, int offset)
	{
		return *reinterpret_cast<T *>(reinterpret_cast<char *>(base) + offset);
	}

	void Load();

	void Fatal(const char *fmt, ...);
}
