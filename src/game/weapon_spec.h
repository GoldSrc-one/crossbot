#pragma once

enum WeaponFire
{
	FIRE_NONE = 0,
	FIRE_HOLD,
	FIRE_TAP,
	FIRE_CHARGE,
};

enum WeaponAim
{
	AIM_NONE = 0,
	AIM_TARGET,
	AIM_LEAD,
	AIM_SURFACE,
};

enum WeaponToggle
{
	TOGGLE_NONE = 0,
	TOGGLE_ZOOM,
	TOGGLE_LASER,
};

enum WeaponAmmo
{
	AMMO_NONE = 0,
	AMMO_CLIP,
	AMMO_PRIMARY,
	AMMO_SECONDARY,
};

enum WeaponQuirks : unsigned int
{
	WPN_Q_NONE          = 0,
	WPN_Q_HIDDEN_STATE  = 1u << 0,
	WPN_Q_NO_UNDERWATER = 1u << 1,
	WPN_Q_SLOW_IN_WATER = 1u << 2,
	WPN_Q_THROW_ARC     = 1u << 3,
	WPN_Q_INHERIT_VEL   = 1u << 4,
	WPN_Q_KEEP_AIMING   = 1u << 5,
	WPN_Q_HOMING        = 1u << 6,
	WPN_Q_STICKY        = 1u << 7,
};

struct WeaponMode
{
	WeaponFire fire;
	WeaponAim aim;
	WeaponToggle toggle;
	WeaponAmmo ammo;
	int ammoPerShot;
	float speed;
	float gravity;
	float range;
	float splash;
	float chargeFull;
	float chargeMax;
	unsigned int quirks;
};

struct WeaponSpec
{
	int id;
	const char *classname;
	int clipSize;
	float value;
	WeaponMode primary;
	WeaponMode primaryZoomed;
	WeaponMode secondary;
};

const char *WeaponFireName(WeaponFire fire);
const char *WeaponAimName(WeaponAim aim);
const char *WeaponToggleName(WeaponToggle toggle);
const char *WeaponAmmoName(WeaponAmmo ammo);
