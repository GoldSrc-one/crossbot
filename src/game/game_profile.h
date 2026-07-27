#pragma once

#include <extdll.h>

enum GameFlags : unsigned int
{
	GAME_NONE       = 0,
	GAME_DEATHMATCH = 1 << 0,
	GAME_TEAMPLAY   = 1 << 1,
};

class IGameProfile
{
public:
	virtual ~IGameProfile() {}

	virtual const char *GetName() const = 0;
	virtual unsigned int GetFlags() const = 0;

	virtual bool Detect() const = 0;

	virtual bool IsDead(edict_t *pEdict) const = 0;

	virtual void Respawn(edict_t *pEdict) const = 0;
};

IGameProfile *GameProfile_Detect();

