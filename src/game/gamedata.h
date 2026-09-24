#pragma once

struct GameOffsets
{
	int pev;

	int ent_fInAttack;
	int ent_flStartThrow;
	int ent_chargeReady;
	int ent_tfState;
	int ent_noGrenades1;
	int ent_noGrenades2;
	int ent_tpGrenades1;
	int ent_tpGrenades2;
	int ent_isBuilding;
	int ent_isFeigning;
	int ent_isUndercover;
	int ent_hasSentry;
	int ent_hasDispenser;
	int ent_hasEntryTele;
	int ent_hasExitTele;
	int ent_realOwner;
	int tele_iType;
	int tele_iState;
	int ent_ammoShells;
	int ent_ammoNails;
	int ent_ammoCells;
	int ent_ammoRockets;
	int ent_teamNo;
	int ent_undercoverTeam;
	int ent_ammoDetpack;
	int ent_isDetpacking;
	int ent_maxammoShells;
	int ent_maxammoNails;
	int ent_maxammoCells;
	int ent_maxammoRockets;
	int ent_maxarmor;
	int ent_goalActivation;
	int ent_goalState;
	int ent_itemsAllowed;
	int ent_increaseTeam;

	int box_flTouchableAt;
	int box_rgpPlayerItems;
	int box_bIsBomb;

	int arm_iItem;

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
	int wpn_flAccuracy;
	int wpn_flLastFire;
	int wpn_iShotsFired;
	int wpn_iWeaponState;
	int gren_flStartThrow;

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
	int plr_iTeam;
	int plr_iJoiningState;
	int plr_iMenu;
	int plr_signals;
	int plr_bIsVIP;
	int plr_bShieldDrawn;
	int plr_bOwnsShield;
	int plr_iNextClass;
	int plr_iFOV;

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
	int dod_sbYaw;
	int dod_sbRange;
	int dod_sbMaster;
	int dod_esState;

	int tfc_teamNo;
	int tfc_goalNo;
	int tfc_groupNo;
	int tfc_goalState;
	int tfc_ownedBy;
	int tfc_teamcheck;
	int tfc_ownedByTeamcheck;
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
