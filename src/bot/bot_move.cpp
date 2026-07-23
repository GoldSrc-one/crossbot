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
	constexpr float kMaxTurnRatePerSec = 300.f; // deg/sec -- fast but not instant, avoids the classic snap-to-target bot tell

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

		bot.desiredAngles = Vector(0.f, atan2f(bot.moveDir.y, bot.moveDir.x) * (180.f / kPi), 0.f);

		Vector angles = Bot_UpdateFacing(bot);
		g_engfuncs.pfnRunPlayerMove(pEdict, angles, kMoveSpeed, 0.f, 0.f, IN_FORWARD, 0, Bot_FrameMsec());
	}
}

byte Bot_FrameMsec()
{
	// See the header comment: this must reflect real elapsed time, not a
	// guessed constant -- confirmed against FoxBot/jk_botti/Parabot, which
	// all derive it from gpGlobals->frametime (or an equivalent self-tracked
	// delta) rather than hardcoding it, and confirmed the hard way here: a
	// fixed msec that overstates the server's real ~10ms tick caused
	// pfnRunPlayerMove to silently produce zero displacement, frame after
	// frame, with no error -- GoldSrc's own timing bookkeeping appears to
	// reject a fake client that consistently claims more elapsed time per
	// move than actually passed.
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
	// v.angles (the model's visible orientation, sent to other clients --
	// distinct from v_angle, the real aim direction used for movement) uses
	// a different pitch convention than v_angle: GoldSrc's player model only
	// has a narrow aim_pitch bone-controller range and expects the opposite
	// sign, so the raw view pitch has to be negated and scaled down, not
	// copied as-is. Confirmed against reference bots -- foxbot's
	// BotChangePitch (bot_navigate.cpp) does `angles.x = v_angle.x / 3`
	// after negating v_angle.x, and yapb's Bot::updateBodyAngles (vision.cpp)
	// does `angles.x = -v_angle.x * (1/3)` directly. This is a purely
	// cosmetic third-person fix (v_angle, not v.angles, drives actual
	// movement via pfnRunPlayerMove), so it doesn't change bot behavior.
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
