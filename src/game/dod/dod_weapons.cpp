#include "dod_weapons.h"

namespace
{
	const WeaponMode kNoMode = {};

	const WeaponMode kMelee = { FIRE_HOLD, AIM_TARGET, TOGGLE_NONE, AMMO_NONE, 0, 0.f, 0.f, 40.f, 0.f, 0.f, 0.f, WPN_Q_NONE };
	const WeaponMode kPistol = { FIRE_TAP, AIM_TARGET, TOGGLE_NONE, AMMO_CLIP, 1, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, WPN_Q_NONE };
	const WeaponMode kRifle = { FIRE_TAP, AIM_TARGET, TOGGLE_NONE, AMMO_CLIP, 1, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, WPN_Q_BRACE };
	const WeaponMode kAuto = { FIRE_HOLD, AIM_TARGET, TOGGLE_NONE, AMMO_CLIP, 1, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, WPN_Q_STICKY };
	const WeaponMode kAutoBrace = { FIRE_HOLD, AIM_TARGET, TOGGLE_NONE, AMMO_CLIP, 1, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, WPN_Q_STICKY | WPN_Q_BRACE };
	const WeaponMode kMG42 = { FIRE_HOLD, AIM_TARGET, TOGGLE_NONE, AMMO_CLIP, 2, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, WPN_Q_STICKY };
	const WeaponMode kZoom = { FIRE_TAP, AIM_NONE, TOGGLE_ZOOM, AMMO_NONE, 0, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, WPN_Q_NONE };
	const WeaponMode kGrenade = { FIRE_TAP, AIM_LEAD, TOGGLE_NONE, AMMO_PRIMARY, 1, 600.f, 0.5f, 0.f, 300.f, 0.f, 0.f, WPN_Q_THROW_ARC | WPN_Q_INHERIT_VEL };
	const WeaponMode kLiveGrenade = { FIRE_TAP, AIM_LEAD, TOGGLE_NONE, AMMO_NONE, 0, 600.f, 0.5f, 0.f, 300.f, 0.f, 0.f, WPN_Q_THROW_ARC | WPN_Q_INHERIT_VEL };
	const WeaponMode kRocket = { FIRE_TAP, AIM_LEAD, TOGGLE_NONE, AMMO_CLIP, 1, 1300.f, 0.f, 0.f, 160.f, 0.f, 0.f, WPN_Q_AIM_FEET | WPN_Q_NO_UNDERWATER };

	const WeaponSpec kDODWeapons[kDODWeaponSlots] = {
		{},
		{ DOD_AMERKNIFE, "weapon_amerknife", -1, 0.05f, RANGE_CLOSE, 0.f, kMelee, kNoMode, kNoMode },
		{ DOD_GERKNIFE, "weapon_gerknife", -1, 0.05f, RANGE_CLOSE, 0.f, kMelee, kNoMode, kNoMode },
		{ DOD_COLT, "weapon_colt", 7, 0.15f, RANGE_CLOSE, 0.f, kPistol, kNoMode, kNoMode },
		{ DOD_LUGER, "weapon_luger", 8, 0.15f, RANGE_CLOSE, 0.f, kPistol, kNoMode, kNoMode },
		{ DOD_GARAND, "weapon_garand", 8, 0.45f, RANGE_LONG, 0.f, kRifle, kNoMode, kNoMode },
		{ DOD_SCOPEDKAR, "weapon_scopedkar", 5, 0.5f, RANGE_LONG, 20.f, kRifle, kRifle, kZoom },
		{ DOD_THOMPSON, "weapon_thompson", 30, 0.4f, RANGE_CLOSE, 0.f, kAuto, kNoMode, kNoMode },
		{ DOD_MP44, "weapon_mp44", 30, 0.45f, RANGE_CLOSE, 0.f, kAuto, kNoMode, kNoMode },
		{ DOD_SPRING, "weapon_spring", 5, 0.5f, RANGE_LONG, 20.f, kRifle, kRifle, kZoom },
		{ DOD_KAR, "weapon_kar", 5, 0.4f, RANGE_LONG, 0.f, kRifle, kNoMode, kNoMode },
		{ DOD_BAR, "weapon_bar", 20, 0.5f, RANGE_LONG, 0.f, kAuto, kNoMode, kNoMode },
		{ DOD_MP40, "weapon_mp40", 30, 0.4f, RANGE_CLOSE, 0.f, kAuto, kNoMode, kNoMode },
		{ DOD_HANDGRENADE, "weapon_handgrenade", -1, 0.1f, RANGE_CLOSE, 0.f, kGrenade, kNoMode, kNoMode },
		{ DOD_STICKGRENADE, "weapon_stickgrenade", -1, 0.1f, RANGE_CLOSE, 0.f, kGrenade, kNoMode, kNoMode },
		{ DOD_STICKGRENADE_EX, "weapon_stickgrenade_ex", -1, 0.1f, RANGE_CLOSE, 0.f, kLiveGrenade, kNoMode, kNoMode },
		{ DOD_HANDGRENADE_EX, "weapon_handgrenade_ex", -1, 0.1f, RANGE_CLOSE, 0.f, kLiveGrenade, kNoMode, kNoMode },
		{ DOD_MG42, "weapon_mg42", 250, 0.6f, RANGE_LONG, 0.f, kMG42, kNoMode, kNoMode },
		{ DOD_30CAL, "weapon_30cal", 150, 0.6f, RANGE_LONG, 0.f, kAuto, kNoMode, kNoMode },
		{ DOD_SPADE, "weapon_spade", -1, 0.05f, RANGE_CLOSE, 0.f, kMelee, kNoMode, kNoMode },
		{ DOD_M1CARBINE, "weapon_m1carbine", 15, 0.4f, RANGE_LONG, 0.f, kRifle, kNoMode, kNoMode },
		{ DOD_MG34, "weapon_mg34", 75, 0.6f, RANGE_LONG, 0.f, kAuto, kNoMode, kNoMode },
		{ DOD_GREASEGUN, "weapon_greasegun", 30, 0.35f, RANGE_CLOSE, 0.f, kAuto, kNoMode, kNoMode },
		{ DOD_FG42, "weapon_fg42", 20, 0.5f, RANGE_LONG, 0.f, kAuto, kNoMode, kNoMode },
		{ DOD_K43, "weapon_k43", 10, 0.45f, RANGE_LONG, 0.f, kRifle, kNoMode, kNoMode },
		{ DOD_ENFIELD, "weapon_enfield", 5, 0.4f, RANGE_LONG, 0.f, kRifle, kNoMode, kNoMode },
		{ DOD_STEN, "weapon_sten", 30, 0.4f, RANGE_CLOSE, 0.f, kAuto, kNoMode, kNoMode },
		{ DOD_BREN, "weapon_bren", 30, 0.5f, RANGE_LONG, 0.f, kAuto, kNoMode, kNoMode },
		{ DOD_WEBLEY, "weapon_webley", 6, 0.15f, RANGE_CLOSE, 0.f, kPistol, kNoMode, kNoMode },
		{ DOD_BAZOOKA, "weapon_bazooka", 1, 0.35f, RANGE_LONG, 0.f, kRocket, kNoMode, kNoMode },
		{ DOD_PSCHRECK, "weapon_pschreck", 1, 0.35f, RANGE_LONG, 0.f, kRocket, kNoMode, kNoMode },
		{ DOD_PIAT, "weapon_piat", 1, 0.35f, RANGE_LONG, 0.f, kRocket, kNoMode, kNoMode },
		{ DOD_SCOPEDENFIELD, "weapon_enfield", 5, 0.5f, RANGE_LONG, 20.f, kRifle, kRifle, kZoom },
		{ DOD_SCOPEDFG42, "weapon_fg42", 20, 0.5f, RANGE_LONG, 20.f, kAutoBrace, kAutoBrace, kZoom },
	};
}

const WeaponSpec *DOD_WeaponSpec(int id)
{
	if (id <= 0 || id >= kDODWeaponSlots)
		return nullptr;

	const WeaponSpec &spec = kDODWeapons[id];
	if (!spec.classname || spec.id != id)
		return nullptr;

	return &spec;
}

const WeaponSpec *DOD_WeaponSlot(int slot)
{
	if (slot < 0 || slot >= kDODWeaponSlots)
		return nullptr;

	return &kDODWeapons[slot];
}

DODRecoil DOD_Recoil(int id)
{
	switch (id)
	{
	case DOD_COLT:
	case DOD_LUGER:
	case DOD_M1CARBINE:
	case DOD_WEBLEY:
		return { 1.4f, 0.35f };
	case DOD_GARAND:
	case DOD_KAR:
	case DOD_ENFIELD:
		return { 8.f, 2.f };
	case DOD_THOMPSON:
	case DOD_GREASEGUN:
		return { 2.15f, 0.5375f };
	case DOD_MP44:
		return { 5.f, 1.25f };
	case DOD_SPRING:
		return { 5.6f, 1.4f };
	case DOD_BAR:
	case DOD_BREN:
		return { 6.72f, 1.3f };
	case DOD_MP40:
	case DOD_STEN:
		return { 2.2f, 0.55f };
	case DOD_MG42:
	case DOD_30CAL:
	case DOD_MG34:
		return { 20.f, 5.f };
	case DOD_FG42:
	case DOD_SCOPEDFG42:
		return { 5.3f, 1.325f };
	case DOD_K43:
		return { 7.f, 1.75f };
	case DOD_SCOPEDKAR:
	case DOD_SCOPEDENFIELD:
		return { 6.f, 1.5f };
	case DOD_BAZOOKA:
	case DOD_PSCHRECK:
	case DOD_PIAT:
		return { 10.f, 2.5f };
	default:
		return { 0.f, 0.f };
	}
}

bool DOD_IsRocket(int id)
{
	return id == DOD_BAZOOKA || id == DOD_PSCHRECK || id == DOD_PIAT;
}

bool DOD_IsSandbagGun(int id)
{
	return id == DOD_30CAL || id == DOD_BAR || id == DOD_BREN || id == DOD_MG34 || id == DOD_MG42;
}

bool DOD_NeedsDeploy(int id)
{
	return id == DOD_30CAL || id == DOD_MG34 || id == DOD_MG42;
}

bool DOD_IsPrimary(int id)
{
	if (id < DOD_GARAND || id > DOD_PIAT)
		return false;
	return !(id >= DOD_HANDGRENADE && id <= DOD_HANDGRENADE_EX) && id != DOD_SPADE && id != DOD_WEBLEY;
}

bool DOD_RifleMelee(int id)
{
	return id == DOD_GARAND || id == DOD_KAR || id == DOD_K43 || id == DOD_ENFIELD;
}
