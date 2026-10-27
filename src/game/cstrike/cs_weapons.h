#pragma once

#include "../weapon_spec.h"

constexpr int kCSWeaponSlots = 31;

constexpr int kCSItemSlots = 6;

enum CSWeaponId
{
	CS_P228         = 1,
	CS_SCOUT        = 3,
	CS_HEGRENADE    = 4,
	CS_XM1014       = 5,
	CS_C4           = 6,
	CS_MAC10        = 7,
	CS_AUG          = 8,
	CS_SMOKEGRENADE = 9,
	CS_ELITE        = 10,
	CS_FIVESEVEN    = 11,
	CS_UMP45        = 12,
	CS_SG550        = 13,
	CS_GALIL        = 14,
	CS_FAMAS        = 15,
	CS_USP          = 16,
	CS_GLOCK18      = 17,
	CS_AWP          = 18,
	CS_MP5NAVY      = 19,
	CS_M249         = 20,
	CS_M3           = 21,
	CS_M4A1         = 22,
	CS_TMP          = 23,
	CS_G3SG1        = 24,
	CS_FLASHBANG    = 25,
	CS_DEAGLE       = 26,
	CS_SG552        = 27,
	CS_AK47         = 28,
	CS_KNIFE        = 29,
	CS_P90          = 30,
	CS_SHIELD       = 99,
};

enum CSInventorySlot
{
	CS_SLOT_NONE = 0,
	CS_SLOT_PRIMARY,
	CS_SLOT_PISTOL,
	CS_SLOT_KNIFE,
	CS_SLOT_GRENADE,
	CS_SLOT_C4,
};

struct CSSpread
{
	int id;
	float moveSpeed;
	float standA, standB;
	float moveA, moveB;
	float duckA, duckB;
	float base;
	float recover;
	float floor;
	float unzoomed;
};

const CSSpread *CS_Spread(int id);

struct CSBullet
{
	int id;
	int penetration;
	int power;
	float distance;
	int damage;
	float rangeMod;
	float range;
};

const CSBullet *CS_Bullet(int id);

const WeaponSpec *CS_WeaponSpec(int id);

const WeaponSpec *CS_WeaponSlot(int slot);

int CS_WeaponId(const char *classname);

int CS_ArmouryWeapon(int item);

bool CS_ArmouryArmor(int item);

CSInventorySlot CS_InventorySlot(int id);

int CS_GrenadeMax(int id);
