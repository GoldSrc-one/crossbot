#include <stddef.h>

#include "ts_weapons.h"

namespace
{
	const WeaponMode kNoMode = {};

	constexpr WeaponMode Gun(float range)
	{
		return { FIRE_TAP, AIM_TARGET, TOGGLE_NONE, AMMO_CLIP, 1, 0.f, 0.f, range, 0.f, 0.f, 0.f, WPN_Q_NONE };
	}

	constexpr WeaponMode Melee(float range)
	{
		return { FIRE_TAP, AIM_TARGET, TOGGLE_NONE, AMMO_NONE, 0, 0.f, 0.f, range, 0.f, 0.f, 0.f, WPN_Q_NONE };
	}

	struct TSWeapon
	{
		WeaponSpec spec;
		int ammoType;
	};

	const TSWeapon kTSWeapons[kTSWeaponSlots] = {
		{ {}, -1 },
		{ { 1, "glock18", 17, 0.15f, RANGE_CLOSE, 0.f, Gun(0.f), kNoMode, kNoMode }, 1 },
		{ { 2, "beretta", 15, 0.15f, RANGE_CLOSE, 0.f, Gun(0.f), kNoMode, kNoMode }, 1 },
		{ { 3, "uzi", 32, 0.25f, RANGE_CLOSE, 0.f, Gun(0.f), kNoMode, kNoMode }, 1 },
		{ { 4, "benellim3", 7, 0.3f, RANGE_CLOSE, 0.f, Gun(1024.f), kNoMode, kNoMode }, 3 },
		{ { 5, "m4", 30, 0.4f, RANGE_LONG, 0.f, Gun(0.f), kNoMode, kNoMode }, 4 },
		{ { 6, "mp5sd", 30, 0.3f, RANGE_CLOSE, 0.f, Gun(0.f), kNoMode, kNoMode }, 1 },
		{ { 7, "mp5k", 30, 0.25f, RANGE_CLOSE, 0.f, Gun(0.f), kNoMode, kNoMode }, 1 },
		{ { 8, "beretta_akimbo", 30, 0.2f, RANGE_CLOSE, 0.f, Gun(0.f), kNoMode, kNoMode }, 1 },
		{ { 9, "mk23", 12, 0.15f, RANGE_CLOSE, 0.f, Gun(0.f), kNoMode, kNoMode }, 2 },
		{ { 10, "mk23_akimbo", 24, 0.2f, RANGE_CLOSE, 0.f, Gun(0.f), kNoMode, kNoMode }, 2 },
		{ { 11, "usas12", 20, 0.35f, RANGE_CLOSE, 0.f, Gun(768.f), kNoMode, kNoMode }, 3 },
		{ { 12, "desert", 7, 0.2f, RANGE_LONG, 0.f, Gun(0.f), kNoMode, kNoMode }, 5 },
		{ { 13, "ak47", 30, 0.4f, RANGE_LONG, 0.f, Gun(0.f), kNoMode, kNoMode }, 6 },
		{ { 14, "fnh", 20, 0.15f, RANGE_CLOSE, 0.f, Gun(0.f), kNoMode, kNoMode }, 7 },
		{ { 15, "aug", 30, 0.4f, RANGE_LONG, 0.f, Gun(0.f), kNoMode, kNoMode }, 4 },
		{ { 16, "uzi_akimbo", 64, 0.3f, RANGE_CLOSE, 0.f, Gun(0.f), kNoMode, kNoMode }, 1 },
		{ { 17, "skorpion", 20, 0.2f, RANGE_CLOSE, 0.f, Gun(0.f), kNoMode, kNoMode }, 12 },
		{ { 18, "barrett", 5, 0.5f, RANGE_LONG, 0.f, Gun(0.f), kNoMode, kNoMode }, 8 },
		{ { 19, "pdw", 40, 0.3f, RANGE_CLOSE, 0.f, Gun(0.f), kNoMode, kNoMode }, 7 },
		{ { 20, "spas12", 8, 0.3f, RANGE_CLOSE, 0.f, Gun(768.f), kNoMode, kNoMode }, 3 },
		{ { 21, "gold", 16, 0.2f, RANGE_CLOSE, 0.f, Gun(0.f), kNoMode, kNoMode }, 2 },
		{ { 22, "glock22", 15, 0.15f, RANGE_CLOSE, 0.f, Gun(0.f), kNoMode, kNoMode }, 9 },
		{ { 23, "ump", 25, 0.3f, RANGE_CLOSE, 0.f, Gun(0.f), kNoMode, kNoMode }, 2 },
		{ {}, 0 },
		{ {}, 0 },
		{ { 26, "mossberg", 8, 0.3f, RANGE_CLOSE, 0.f, Gun(1024.f), kNoMode, kNoMode }, 3 },
		{ { 27, "m16", 18, 0.4f, RANGE_LONG, 0.f, Gun(0.f), kNoMode, kNoMode }, 4 },
		{ { 28, "ruger", 17, 0.1f, RANGE_CLOSE, 0.f, Gun(0.f), kNoMode, kNoMode }, 10 },
		{ {}, -1 },
		{ { 30, "fnh_akimbo", 40, 0.2f, RANGE_CLOSE, 0.f, Gun(0.f), kNoMode, kNoMode }, 7 },
		{ { 31, "bull", 5, 0.25f, RANGE_LONG, 0.f, Gun(0.f), kNoMode, kNoMode }, 11 },
		{ { 32, "m60", 100, 0.45f, RANGE_CLOSE, 0.f, Gun(0.f), kNoMode, kNoMode }, 4 },
		{ { 33, "sawedoff", 2, 0.25f, RANGE_CLOSE, 0.f, Gun(384.f), kNoMode, kNoMode }, 3 },
		{ { kTSKatana, "katana", -1, 0.1f, RANGE_CLOSE, 0.f, Melee(64.f), kNoMode, kNoMode }, 0 },
		{ {}, 0 },
		{ { 36, "contender", 1, 0.3f, RANGE_LONG, 0.f, Gun(0.f), kNoMode, kNoMode }, 13 },
		{ { 37, "skorpion_akimbo", 40, 0.25f, RANGE_CLOSE, 0.f, Gun(0.f), kNoMode, kNoMode }, 12 },
		{ { kTSKungFu, "kungfu", -1, 0.05f, RANGE_CLOSE, 0.f, Melee(48.f), kNoMode, kNoMode }, -1 },
	};

	const TSLoadout kLoadouts[] = {
		{ "tkw 5_0 12_0", "tka 4_120 5_35" },
		{ "tkw 13_0 9_0", "tka 6_120 2_48" },
		{ "tkw 15_0 1_0", "tka 4_120 1_68" },
		{ "tkw 19_0 22_0", "tka 7_160 9_60" },
		{ "tkw 4_0 12_0", "tka 3_40 5_35" },
		{ "tkw 6_0 2_0", "tka 1_180" },
		{ "tkw 27_0 14_0", "tka 4_90 7_80" },
		{ "tkw 26_0 9_0", "tka 3_40 2_48" },
	};
}

const WeaponSpec *TS_WeaponSlot(int slot)
{
	if (slot < 0 || slot >= kTSWeaponSlots)
		return nullptr;

	return &kTSWeapons[slot].spec;
}

const WeaponSpec *TS_WeaponSpec(int id)
{
	const WeaponSpec *spec = TS_WeaponSlot(id);
	return (spec && spec->classname && spec->id == id) ? spec : nullptr;
}

int TS_AmmoType(int id)
{
	return TS_WeaponSpec(id) ? kTSWeapons[id].ammoType : -1;
}

const TSLoadout &TS_Loadout(int index)
{
	constexpr int count = sizeof(kLoadouts) / sizeof(kLoadouts[0]);
	return kLoadouts[((index % count) + count) % count];
}
