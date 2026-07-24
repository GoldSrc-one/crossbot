#include <extdll.h>
#include <meta_api.h>
#include <in_buttons.h>

#include "bot_move.h"
#include "bot_move_nav.h"
#include "../nav/nav.h"

namespace
{
	constexpr float kPi = 3.14159265f;
	constexpr float kMoveSpeed = 250.f;
	constexpr float kDirChangeInterval = 3.f;
	constexpr float kTraceDistance = 64.f;
	constexpr float kMaxTurnRatePerSec = 300.f; // deg/sec -- fast but not instant, avoids classic snap-to-target bot tell

	float NormalizeAngle(float a)
	{
		while (a > 180.f) a -= 360.f;
		while (a < -180.f) a += 360.f;
		return a;
	}

	Vector RandomDir()
	{
		float yaw = (rand() % 360) * (kPi / 180.f);
		return Vector(cosf(yaw), sinf(yaw), 0.f);
	}

	void Bot_MoveWander(CBot &bot)
	{
		edict_t *pEdict = bot.pEdict;

		if (bot.nextDirChangeTime <= gpGlobals->time)
		{
			bot.moveDir = RandomDir();
			bot.nextDirChangeTime = gpGlobals->time + kDirChangeInterval;
		}

		// Wall avoidance: trace ahead, pick new random direction immediately if blocked.
		Vector start = pEdict->v.origin;
		Vector end = start + bot.moveDir * kTraceDistance;

		TraceResult tr;
		g_engfuncs.pfnTraceLine(start, end, ignore_monsters, pEdict, &tr);

		if (tr.flFraction < 1.0f)
		{
			bot.moveDir = RandomDir();
			bot.nextDirChangeTime = gpGlobals->time + kDirChangeInterval;
		}

		bot.desiredAngles = Vector(0.f, atan2f(bot.moveDir.y, bot.moveDir.x) * (180.f / kPi), 0.f);

		Vector angles = Bot_UpdateFacing(bot);
		g_engfuncs.pfnRunPlayerMove(pEdict, angles, kMoveSpeed, 0.f, 0.f, IN_FORWARD, 0, Bot_FrameMsec());
	}
}

byte Bot_FrameMsec()
{
	// Must reflect real elapsed time, not a guessed constant -- all reference bots derive from gpGlobals->frametime. Confirmed the hard way: fixed msec overstating ~10ms tick caused pfnRunPlayerMove to silently produce zero displacement, no error -- GoldSrc's timing bookkeeping rejects fake client claiming more elapsed time per move than actually passed.
	int msec = static_cast<int>(gpGlobals->frametime * 1000.f);
	if (msec < 1)
		msec = 1;
	else if (msec > 100)
		msec = 100;
	return static_cast<byte>(msec);
}

Vector Bot_UpdateFacing(CBot &bot)
{
	edict_t *pEdict = bot.pEdict;
	Vector current = pEdict->v.v_angle;

	float yawDelta = NormalizeAngle(bot.desiredAngles.y - current.y);
	float pitchDelta = NormalizeAngle(bot.desiredAngles.x - current.x);

	float maxStep = kMaxTurnRatePerSec * gpGlobals->frametime;

	float yawStep = (yawDelta > maxStep) ? maxStep : (yawDelta < -maxStep ? -maxStep : yawDelta);
	float pitchStep = (pitchDelta > maxStep) ? maxStep : (pitchDelta < -maxStep ? -maxStep : pitchDelta);

	Vector smoothed(NormalizeAngle(current.x + pitchStep), NormalizeAngle(current.y + yawStep), 0.f);

	pEdict->v.v_angle = smoothed;
	// v.angles (model's visible orientation, sent to other clients) uses different pitch convention: player model has narrow aim_pitch bone-controller range and expects opposite sign, so raw view pitch must be negated and scaled down, not copied as-is. Purely cosmetic third-person fix (v_angle drives movement via pfnRunPlayerMove).
	pEdict->v.angles = Vector(-smoothed.x / 3.f, smoothed.y, 0.f);

	return smoothed;
}

void Bot_Move(CBot &bot)
{
	if (Nav::IsLoaded())
		Bot_MoveNav(bot);
	else
		Bot_MoveWander(bot);
}
