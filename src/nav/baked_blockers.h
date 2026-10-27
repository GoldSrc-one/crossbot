#pragma once

#include <string.h>

namespace crossbot_nav
{

constexpr int kSfWallStartOff = 1;
constexpr int kSfConveyorVisual = 1;
constexpr int kSfConveyorNotSolid = 2;

inline bool IsBarrierWall(const char *classname, bool named)
{
	return named && classname && strcmp(classname, "func_wall_toggle") == 0;
}

inline bool IsBakedBlocker(const char *classname, int spawnflags, bool named)
{
	if (!classname)
		return false;

	if (strcmp(classname, "func_wall") == 0 || strcmp(classname, "func_healthcharger") == 0 ||
		strcmp(classname, "func_recharge") == 0)
		return true;

	if (strcmp(classname, "func_wall_toggle") == 0)
		return !named && (spawnflags & kSfWallStartOff) == 0;

	if (strcmp(classname, "func_conveyor") == 0)
		return (spawnflags & kSfConveyorNotSolid) == 0;

	return false;
}

}
