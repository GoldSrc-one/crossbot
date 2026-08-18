#include <extdll.h>
#include <meta_api.h>

#include "bot_move.h"
#include "../nav/nav_math.h"

namespace
{
	constexpr float kMaxTurnRatePerSec = 300.f;

	constexpr byte kMaxJumpCommandMsec = 50;

	bool HasLongJumpModule(edict_t *pEdict)
	{
		const char *slj = g_engfuncs.pfnGetPhysicsKeyValue(pEdict, "slj");
		return slj && atoi(slj) == 1;
	}
}

float Bot_NormalizeAngle(float a)
{
	while (a > 180.f) a -= 360.f;
	while (a < -180.f) a += 360.f;
	return a;
}

byte Bot_FrameMsec()
{

	int msec = static_cast<int>(gpGlobals->frametime * 1000.f);
	if (msec < 1)
		msec = 1;
	else if (msec > 100)
		msec = 100;
	return static_cast<byte>(msec);
}

void Bot_RunPlayerMove(edict_t *pEdict, const Vector &angles, float fmove, float smove,
	unsigned short buttons)
{
	byte msec = Bot_FrameMsec();

	if ((buttons & (IN_JUMP | IN_DUCK)) == (IN_JUMP | IN_DUCK) && HasLongJumpModule(pEdict))
		buttons &= ~IN_DUCK;

	if ((buttons & IN_JUMP) != 0 && msec > kMaxJumpCommandMsec)
	{
		g_engfuncs.pfnRunPlayerMove(pEdict, angles, fmove, smove, 0.f, buttons, 0,
			kMaxJumpCommandMsec);
		buttons &= ~IN_JUMP;
		msec -= kMaxJumpCommandMsec;
	}

	g_engfuncs.pfnRunPlayerMove(pEdict, angles, fmove, smove, 0.f, buttons, 0, msec);
}

Vector Bot_UpdateFacing(CBot &bot)
{
	edict_t *pEdict = bot.pEdict;
	Vector current = pEdict->v.v_angle;

	float yawDelta = Bot_NormalizeAngle(bot.desiredAngles.y - current.y);
	float pitchDelta = Bot_NormalizeAngle(bot.desiredAngles.x - current.x);

	float maxStep = kMaxTurnRatePerSec * gpGlobals->frametime;

	float yawStep = (yawDelta > maxStep) ? maxStep : (yawDelta < -maxStep ? -maxStep : yawDelta);
	float pitchStep = (pitchDelta > maxStep) ? maxStep : (pitchDelta < -maxStep ? -maxStep : pitchDelta);

	Vector smoothed(Bot_NormalizeAngle(current.x + pitchStep), Bot_NormalizeAngle(current.y + yawStep), 0.f);

	pEdict->v.v_angle = smoothed;

	pEdict->v.angles = Vector(-smoothed.x / 3.f, smoothed.y, 0.f);

	return smoothed;
}

void Bot_DecomposeMove(float viewYawDeg, const Vector &worldDir, float driveLength, float &forwardMove,
	float &sideMove)
{
	forwardMove = 0.f;
	sideMove = 0.f;

	float len = sqrtf(worldDir.x * worldDir.x + worldDir.y * worldDir.y);
	if (len < 0.0001f)
		return;

	float nx = worldDir.x / len, ny = worldDir.y / len;

	float yawRad = viewYawDeg * (NavMath::kPiF / 180.f);
	float cy = cosf(yawRad), sy = sinf(yawRad);

	forwardMove = (nx * cy + ny * sy) * driveLength;
	sideMove = (nx * sy - ny * cy) * driveLength;
}
