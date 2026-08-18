#include "weapon_spec.h"

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
