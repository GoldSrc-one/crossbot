#pragma once

#include <extdll.h>

// Bitmask of gamerule traits, extended per title as more games are
// supported. Only GAME_DEATHMATCH is meaningful for this milestone (HLDM).
enum GameFlags : unsigned int
{
	GAME_NONE       = 0,
	GAME_DEATHMATCH = 1 << 0, // free-for-all deathmatch rules
	GAME_TEAMPLAY   = 1 << 1, // team-based rules (CTF, hostage rescue, ...)
};

// Per-mod knowledge: how to tell this mod is running, and how to keep a bot
// alive in it. Weapon/nav knowledge is a future extension of this interface.
class IGameProfile
{
public:
	virtual ~IGameProfile() {}

	virtual const char *GetName() const = 0;
	virtual unsigned int GetFlags() const = 0;

	// True if this profile matches the mod the server is currently running.
	virtual bool Detect() const = 0;

	// True once pEdict needs to be respawned.
	virtual bool IsDead(edict_t *pEdict) const = 0;

	// Puts a dead player entity back into play.
	virtual void Respawn(edict_t *pEdict) const = 0;
};

// Returns the profile matching the currently running mod, or NULL if none
// of the known profiles match (the bot manager should stay idle then).
IGameProfile *GameProfile_Detect();
