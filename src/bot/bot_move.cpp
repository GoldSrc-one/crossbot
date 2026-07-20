#include <extdll.h>
#include <meta_api.h>
#include <in_buttons.h>

#include "bot_move.h"

namespace
{
	constexpr float kPi = 3.14159265f;
	constexpr float kMoveSpeed = 250.f;
	constexpr float kDirChangeInterval = 3.f;
	constexpr float kTraceDistance = 64.f;

	Vector RandomDir()
	{
		float yaw = (rand() % 360) * (kPi / 180.f);
		return Vector(cosf(yaw), sinf(yaw), 0.f);
	}
}

void Bot_Move(CBot &bot)
{
	edict_t *pEdict = bot.pEdict;

	if (bot.nextDirChangeTime <= gpGlobals->time)
	{
		bot.moveDir = RandomDir();
		bot.nextDirChangeTime = gpGlobals->time + kDirChangeInterval;
	}

	// Wall avoidance: trace ahead, and if the path is blocked, pick a new
	// random direction right away instead of walking into it.
	Vector start = pEdict->v.origin;
	Vector end = start + bot.moveDir * kTraceDistance;

	TraceResult tr;
	g_engfuncs.pfnTraceLine(start, end, ignore_monsters, pEdict, &tr);

	if (tr.flFraction < 1.0f)
	{
		bot.moveDir = RandomDir();
		bot.nextDirChangeTime = gpGlobals->time + kDirChangeInterval;
	}

	Vector angles(0.f, atan2f(bot.moveDir.y, bot.moveDir.x) * (180.f / kPi), 0.f);
	pEdict->v.v_angle = angles;
	pEdict->v.angles = angles;

	g_engfuncs.pfnRunPlayerMove(pEdict, angles, kMoveSpeed, 0.f, 0.f, IN_FORWARD, 0, 25);
}
