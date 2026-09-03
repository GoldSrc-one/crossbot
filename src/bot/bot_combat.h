#pragma once

#include <extdll.h>

#include "../game/ent_handle.h"

struct CBot;

namespace Nav
{
	struct NavMesh;
}

namespace BotCombat
{

struct State
{
	bool active = false;
	bool driving = false;

	CBotEntHandle enemy;
	float startTime = 0.f;
	float lastSightTime = 0.f;

	// Has this enemy noticed the bot? Latches on true and is re-tested every frame it stays
	// false, so a bot that opens an ambush stops ambushing the moment it is spotted.
	bool enemyAware = false;

	bool haveProbe = false;
	Vector probePoint;
	float probeExposure = 0.f;
	float hereExposure = 0.f;
	int probeArea = -1;
	float nextProbeTime = 0.f;

	unsigned int rngState = 0;
	bool rngSeeded = false;

	const char *lastReason = "idle";
};

bool Enabled();

bool DebugEnabled();

bool AmbushEnabled();

void Update(CBot &bot);

bool Run(CBot &bot, const Nav::NavMesh &mesh);

void Exit(CBot &bot, const char *why);

void OnRespawn(CBot &bot);

bool Active(const CBot &bot);

bool Ambushing(const CBot &bot);

edict_t *Enemy(const CBot &bot);

edict_t *DamageSource(edict_t *pEdict);

}
