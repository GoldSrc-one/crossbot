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
	int eagle_iLaserActive;

	int mon_flNextAttack;
	int mon_hEnemy;

	int plr_pActiveItem;
	int plr_rgpPlayerItems;
	int plr_rgAmmo;
	int plr_pTank;
	int plr_iPowerups;
	int plr_iQuakeItems;
	int plr_iQuakeWeapon;
	int plr_iAmmoShells;
	int plr_iAmmoNails;
	int plr_iAmmoRockets;
	int plr_iAmmoCells;

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

struct MapOffsets
{
	int cs_plrHasC4;
	int cs_plrIsVIP;
	int cs_plrTeam;
	int cs_monTargetEnt;
	int cs_grnIsC4;
	int cs_boxIsBomb;

	int dod_cpTeam;
	int dod_cpAlliesCantTouch;
	int dod_cpAxisCantTouch;
	int dod_acOwningTeam;
	int dod_acAlliesNumCap;
	int dod_acAxisNumCap;
	int dod_acAlliesCanCap;
	int dod_acAxisCanCap;
	int dod_acRequiresObject;
	int dod_acRequiredGroup;
	int dod_objTeam;
	int dod_objGroup;
	int dod_objOwner;
	int dod_brkGroup;

	int tfc_teamNo;
	int tfc_goalNo;
	int tfc_groupNo;
	int tfc_goalState;
	int tfc_ownedBy;
	int tfc_goalActivation;
	int tfc_itemsAllowed;
	int tfc_hasItemFromGroup;
	int tfc_ifGoalActive;
	int tfc_ifGoalInactive;
	int tfc_ifGoalRemoved;
	int tfc_ifGroupActive;
	int tfc_ifGroupInactive;
	int tfc_ifGroupRemoved;
	int tfc_increaseTeam;
};

extern GameOffsets g_offsets;

namespace GameData
{
	const char *LoadMap(const char *gamedir, MapOffsets &out);

	template <typename T>
	inline T Peek(void *base, int offset)
	{
		return *reinterpret_cast<T *>(reinterpret_cast<char *>(base) + offset);
	}

	void LoadAll();

	bool Loaded(int iGame);

	void Activate(int iGame);

	void Fatal(const char *fmt, ...);
}
