#pragma once

#include <extdll.h>

#include <cdll_dll.h>

struct CBot;

constexpr int kMaxBotWeapons = 32;

struct BotWeapon
{
	edict_t *ent = nullptr;
	void *priv = nullptr;
	int id = 0;
	int clip = -1;
	int primaryAmmoType = -1;
	int secondaryAmmoType = -1;
	float nextPrimaryAttack = 0.f;
	float nextSecondaryAttack = 0.f;
	bool reloading = false;
	int chargeState = 0;
	float throwStart = 0.f;
	int guidedCount = 0;
	int chargeReady = 0;
	int laserActive = -1;
};

struct BotInventory
{
	int count = 0;
	BotWeapon weapons[kMaxBotWeapons];
	int activeIndex = -1;
	float nextAttack = 0.f;
	int ammo[MAX_AMMO_SLOTS] = {};
};

namespace BotWeapons
{
	void Read(edict_t *player, BotInventory &out);

	int ActiveWeaponId(edict_t *player);

	void Refresh(CBot &bot);
}
