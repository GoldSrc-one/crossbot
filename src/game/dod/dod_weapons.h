#pragma once

#include "../weapon_spec.h"

constexpr int kDODWeaponSlots = 34;

constexpr int kDODItemSlots = 6;

enum DODWeaponId
{
	DOD_AMERKNIFE      = 1,
	DOD_GERKNIFE       = 2,
	DOD_COLT           = 3,
	DOD_LUGER          = 4,
	DOD_GARAND         = 5,
	DOD_SCOPEDKAR      = 6,
	DOD_THOMPSON       = 7,
	DOD_MP44           = 8,
	DOD_SPRING         = 9,
	DOD_KAR            = 10,
	DOD_BAR            = 11,
	DOD_MP40           = 12,
	DOD_HANDGRENADE    = 13,
	DOD_STICKGRENADE   = 14,
	DOD_STICKGRENADE_EX = 15,
	DOD_HANDGRENADE_EX = 16,
	DOD_MG42           = 17,
	DOD_30CAL          = 18,
	DOD_SPADE          = 19,
	DOD_M1CARBINE      = 20,
	DOD_MG34           = 21,
	DOD_GREASEGUN      = 22,
	DOD_FG42           = 23,
	DOD_K43            = 24,
	DOD_ENFIELD        = 25,
	DOD_STEN           = 26,
	DOD_BREN           = 27,
	DOD_WEBLEY         = 28,
	DOD_BAZOOKA        = 29,
	DOD_PSCHRECK       = 30,
	DOD_PIAT           = 31,
	DOD_SCOPEDENFIELD  = 32,
	DOD_SCOPEDFG42     = 33,
};

struct DODRecoil
{
	float pitch;
	float yaw;
};

const WeaponSpec *DOD_WeaponSpec(int id);

const WeaponSpec *DOD_WeaponSlot(int slot);

DODRecoil DOD_Recoil(int id);

bool DOD_IsRocket(int id);

bool DOD_IsSandbagGun(int id);

bool DOD_NeedsDeploy(int id);

bool DOD_IsPrimary(int id);

bool DOD_RifleMelee(int id);
