#include <extdll.h>
#include <meta_api.h>

#include "bot_aim.h"
#include "bot.h"
#include "bot_move.h"

namespace
{
	constexpr float kReferenceFov = 90.f;

	constexpr float kMaxAccelYaw = 4500.f;
	constexpr float kMaxAccelPitch = 3600.f;
	constexpr float kMaxHandSpeed = 1200.f;

	constexpr float kPitchLimit = 89.f;

	constexpr float kMinFrameTime = 1.f / 1000.f;
	constexpr float kMaxFrameTime = 1.f / 20.f;

	constexpr float kTestSettleTolerance = 0.5f;
	constexpr float kTestTimeout = 3.f;

	float FovScale(edict_t *pEdict)
	{
		float fov = pEdict->v.fov;
		if (fov <= 0.f)
			fov = kReferenceFov;
		return fov / kReferenceFov;
	}

	float StepAxis(float errAngle, float fovScale, float maxAccel, float dt, float &vel)
	{
		float errHand = errAngle / fovScale;
		float mag = fabsf(errHand);

		float limit = mag / dt;

		float step = maxAccel * dt;
		float envelope = sqrtf(2.f * maxAccel * mag + 0.25f * step * step) - 0.5f * step;
		if (envelope < limit)
			limit = envelope;
		if (limit > kMaxHandSpeed)
			limit = kMaxHandSpeed;

		float target = (errHand < 0.f) ? -limit : limit;

		float accel = (target - vel) / dt;
		if (accel > maxAccel)
			accel = maxAccel;
		else if (accel < -maxAccel)
			accel = -maxAccel;

		vel += accel * dt;
		return vel * fovScale * dt;
	}

	void UpdateTest(CBot &bot, const Vector &angles)
	{
		BotAim::State &aim = bot.aim;

		float speed = BotAim::ViewSpeed(bot);
		if (speed > aim.testPeakSpeed)
			aim.testPeakSpeed = speed;

		if (aim.testSettled)
			return;

		float yawErr = fabsf(Bot_NormalizeAngle(aim.testAngles.y - angles.y));
		float pitchErr = fabsf(Bot_NormalizeAngle(aim.testAngles.x - angles.x));
		if (yawErr > kTestSettleTolerance || pitchErr > kTestSettleTolerance)
			return;

		aim.testSettled = true;

		float fov = bot.pEdict->v.fov;
		if (fov <= 0.f)
			fov = kReferenceFov;

		char msg[192];
		snprintf(msg, sizeof(msg),
			"CrossBot: %s aim test settled in %.3fs, peak %.0f deg/s hand, fov %.0f\n",
			bot.name, gpGlobals->time - aim.testStart, aim.testPeakSpeed, fov);
		SERVER_PRINT(msg);
	}
}

float BotAim::ViewSpeed(const CBot &bot)
{
	const BotAim::State &aim = bot.aim;
	return sqrtf(aim.yawVel * aim.yawVel + aim.pitchVel * aim.pitchVel);
}

void BotAim::BeginTest(CBot &bot, float yawDelta, float pitchDelta)
{
	BotAim::State &aim = bot.aim;
	const Vector &current = bot.pEdict->v.v_angle;

	float pitch = current.x + pitchDelta;
	if (pitch > kPitchLimit)
		pitch = kPitchLimit;
	else if (pitch < -kPitchLimit)
		pitch = -kPitchLimit;

	aim.testAngles = Vector(pitch, Bot_NormalizeAngle(current.y + yawDelta), 0.f);
	aim.testActive = true;
	aim.testSettled = false;
	aim.testStart = gpGlobals->time;
	aim.testUntil = gpGlobals->time + kTestTimeout;
	aim.testPeakSpeed = 0.f;
}

Vector Bot_UpdateFacing(CBot &bot)
{
	edict_t *pEdict = bot.pEdict;
	BotAim::State &aim = bot.aim;
	Vector current = pEdict->v.v_angle;

	if (aim.testActive && gpGlobals->time > aim.testUntil)
		aim.testActive = false;

	const Vector &desired = aim.testActive ? aim.testAngles : bot.desiredAngles;

	float dt = gpGlobals->frametime;
	if (dt < kMinFrameTime)
		dt = kMinFrameTime;
	else if (dt > kMaxFrameTime)
		dt = kMaxFrameTime;

	float fovScale = FovScale(pEdict);

	float yawErr = Bot_NormalizeAngle(desired.y - current.y);
	float pitchErr = Bot_NormalizeAngle(desired.x - current.x);

	float yaw = Bot_NormalizeAngle(current.y + StepAxis(yawErr, fovScale, kMaxAccelYaw, dt, aim.yawVel));
	float pitch = current.x + StepAxis(pitchErr, fovScale, kMaxAccelPitch, dt, aim.pitchVel);

	if (pitch > kPitchLimit)
	{
		pitch = kPitchLimit;
		aim.pitchVel = 0.f;
	}
	else if (pitch < -kPitchLimit)
	{
		pitch = -kPitchLimit;
		aim.pitchVel = 0.f;
	}

	Vector smoothed(pitch, yaw, 0.f);

	pEdict->v.v_angle = smoothed;

	pEdict->v.angles = Vector(-smoothed.x / 3.f, smoothed.y, 0.f);

	if (aim.testActive)
		UpdateTest(bot, smoothed);

	return smoothed;
}
