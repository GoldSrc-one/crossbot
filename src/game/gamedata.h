#pragma once

struct GameOffsets
{
	int pev;

	int ent_fInAttack;
	int ent_flStartThrow;
	int ent_chargeReady;

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
	int mon_hEnemy;

	int plr_pActiveItem;
	int plr_rgpPlayerItems;
	int plr_rgAmmo;
	int plr_pTank;

	int tog_toggleState;
	int tog_flWait;

	int btn_fStayPushed;

	int tank_pController;
	int tank_vecControllerUsePos;
	int tank_yawCenter;
	int tank_yawRange;
	int tank_yawRate;
	int tank_pitchCenter;
	int tank_pitchRange;
	int tank_pitchRate;
	int tank_fireRate;
	int tank_iszMaster;

	int tankctl_pTank;

	int mm_cTargets;
	int mm_iTargetName;

	int ms_rgTriggered;
	int ms_iTotal;
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
