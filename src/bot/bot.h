#pragma once

#include <extdll.h>

// A single bot's state. Movement is a placeholder wall-avoiding random walk
// for this milestone; real pathfinding is a future extension.
struct CBot
{
	bool inUse = false;
	edict_t *pEdict = nullptr;
	char name[32] = {};

	Vector moveDir = Vector(1.f, 0.f, 0.f);
	float nextDirChangeTime = 0.f;
};
