#include <extdll.h>
#include <meta_api.h>

#include "bot_move.h"
#include "bot_move_nav.h"

namespace
{
	constexpr float kPi = 3.14159265f;
	constexpr float kMaxTurnRatePerSec = 300.f;

	float NormalizeAngle(float a)
	{
		while (a > 180.f) a -= 360.f;
		while (a < -180.f) a += 360.f;
		return a;
	}
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

	// PM_WalkMove and PM_AirMove both flatten forward/right and renormalize before use, so whatever
	// pitch and roll the engine puts in pmove->angles drops out and the basis is just the yaw.
	float yawRad = viewYawDeg * (kPi / 180.f);
	float cy = cosf(yawRad), sy = sinf(yawRad);

	forwardMove = (nx * cy + ny * sy) * driveLength;
	sideMove = (nx * sy - ny * cy) * driveLength;
}

void Bot_Move(CBot &bot)
{
	Bot_MoveNav(bot);
}
