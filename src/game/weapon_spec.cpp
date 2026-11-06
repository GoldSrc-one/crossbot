#include "weapon_spec.h"

const WeaponQuirkName kWeaponQuirkNames[] = {
	{ WPN_Q_HIDDEN_STATE,  "hiddenstate" },
	{ WPN_Q_NO_UNDERWATER, "nounderwater" },
	{ WPN_Q_SLOW_IN_WATER, "slowinwater" },
	{ WPN_Q_THROW_ARC,     "throwarc" },
	{ WPN_Q_INHERIT_VEL,   "inheritvel" },
	{ WPN_Q_KEEP_AIMING,   "keepaiming" },
	{ WPN_Q_STICKY,        "sticky" },
	{ WPN_Q_HOMING,        "homing" },
	{ WPN_Q_LASER_ON,      "laseron" },
	{ WPN_Q_SPINUP,        "spinup" },
	{ WPN_Q_AIM_FEET,      "aimfeet" },
	{ WPN_Q_BRACE,         "brace" },
	{ WPN_Q_TEAM_UNSAFE,   "teamunsafe" },
};

const int kWeaponQuirkNameCount = static_cast<int>(sizeof(kWeaponQuirkNames) / sizeof(kWeaponQuirkNames[0]));

const char *WeaponFireName(WeaponFire fire)
{
	switch (fire)
	{
	case FIRE_HOLD:   return "hold";
	case FIRE_TAP:    return "tap";
	case FIRE_CHARGE: return "charge";
	default:          return "none";
	}
}

const char *WeaponAimName(WeaponAim aim)
{
	switch (aim)
	{
	case AIM_TARGET:  return "target";
	case AIM_LEAD:    return "lead";
	case AIM_SURFACE: return "surface";
	default:          return "none";
	}
}

const char *WeaponToggleName(WeaponToggle toggle)
{
	switch (toggle)
	{
	case TOGGLE_ZOOM:  return "zoom";
	case TOGGLE_LASER: return "laser";
	default:           return "none";
	}
}

const char *WeaponAmmoName(WeaponAmmo ammo)
{
	switch (ammo)
	{
	case AMMO_CLIP:      return "clip";
	case AMMO_PRIMARY:   return "prim";
	case AMMO_SECONDARY: return "sec";
	default:             return "none";
	}
}

const char *WeaponRangeName(WeaponRangeClass range)
{
	switch (range)
	{
	case RANGE_CLOSE: return "close";
	case RANGE_LONG:  return "long";
	default:          return "none";
	}
}
