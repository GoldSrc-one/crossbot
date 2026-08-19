#include <extdll.h>
#include <meta_api.h>

#include <algorithm>

#include "bot_aim.h"
#include "bot.h"
#include "bot_move.h"
#include "../game/game_profile.h"
#include "../nav/nav_math.h"

namespace
{
	using NavMath::kPiF;

	constexpr float kReferenceFov = 90.f;

	constexpr float kMaxAccelYaw = 4500.f;
	constexpr float kMaxAccelPitch = 3600.f;
	constexpr float kMaxHandSpeed = 1200.f;

	constexpr float kPitchLimit = 89.f;

	constexpr float kMinFrameTime = 1.f / 1000.f;
	constexpr float kMaxFrameTime = 1.f / 20.f;

	constexpr float kTestSettleTolerance = 0.5f;
	constexpr float kTestSettleSpeed = 150.f;
	constexpr float kTestTimeout = 3.f;

	constexpr float kBrakeSpread = 0.06f;
	constexpr float kFlickMinError = 3.f;
	constexpr float kFlickGrowth = 0.5f;
	constexpr float kDefaultErrorScale = 1.f;
	constexpr float kMaxErrorScale = 4.f;

	constexpr float kDefaultDrift = 1.f;
	constexpr float kMaxDrift = 5.f;
	constexpr float kDriftYawRate = 1.7f;
	constexpr float kDriftPitchRate = 1.19f;
	constexpr float kDriftPitchScale = 0.6f;

	constexpr float kDefaultDelay = 0.25f;

	constexpr float kDegenerateLos = 0.0001f;

	float FovScale(edict_t *pEdict)
	{
		float fov = pEdict->v.fov;
		if (fov <= 0.f)
			fov = kReferenceFov;
		return fov / kReferenceFov;
	}

	float RandomUnit(BotAim::State &aim)
	{
		aim.rngState = aim.rngState * 1664525u + 1013904223u;
		return static_cast<float>((aim.rngState >> 8) & 0xFFFFFFu) / 16777216.f;
	}

	float RandomSigned(BotAim::State &aim)
	{
		return RandomUnit(aim) * 2.f - 1.f;
	}

	void SeedAim(CBot &bot)
	{
		BotAim::State &aim = bot.aim;
		if (aim.rngSeeded)
			return;

		aim.rngSeeded = true;
		aim.rngState = static_cast<unsigned int>(
			g_engfuncs.pfnIndexOfEdict(bot.pEdict)) * 2654435761u + 1u;
		aim.driftPhaseYaw = RandomUnit(aim) * 2.f * kPiF;
		aim.driftPhasePitch = RandomUnit(aim) * 2.f * kPiF;
	}

	void UpdateBrake(CBot &bot, float yawErr, float pitchErr)
	{
		BotAim::State &aim = bot.aim;

		float mag = sqrtf(yawErr * yawErr + pitchErr * pitchErr);
		float spread = kBrakeSpread * BotAim::ErrorScale();

		if (spread <= 0.f)
		{
			aim.brakeYaw = 1.f;
			aim.brakePitch = 1.f;
		}
		else if (mag > kFlickMinError && mag > aim.flickMag + kFlickGrowth)
		{
			aim.brakeYaw = 1.f + RandomSigned(aim) * spread;
			aim.brakePitch = 1.f + RandomSigned(aim) * spread;
		}

		aim.flickMag = mag;
	}

	float StepAxis(float errAngle, float fovScale, float maxAccel, float brake, float dt, float &vel)
	{
		float errHand = errAngle / fovScale;
		float mag = fabsf(errHand);

		float limit = mag / dt;

		float step = maxAccel * dt;
		float envelope = (sqrtf(2.f * maxAccel * mag + 0.25f * step * step) - 0.5f * step) * brake;
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
		if (speed > kTestSettleSpeed)
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

bool BotAim::LosAngles(const Vector &from, const Vector &to, float &yaw, float &pitch)
{
	Vector los = to - from;
	float flat = sqrtf(los.x * los.x + los.y * los.y);
	if (flat < kDegenerateLos && fabsf(los.z) < kDegenerateLos)
		return false;

	yaw = atan2f(los.y, los.x) * (180.f / kPiF);
	pitch = -atan2f(los.z, flat) * (180.f / kPiF);
	return true;
}

void BotAim::PushDelaySlot(CBot &bot, const Vector &point)
{
	BotAim::State &aim = bot.aim;

	BotAim::DelaySlot &slot = aim.delaySlots[aim.delayHead];
	slot.time = gpGlobals->time;
	slot.point = point;

	aim.delayHead = (aim.delayHead + 1) % BotAim::kDelaySlots;
	if (aim.delayCount < BotAim::kDelaySlots)
		++aim.delayCount;
}

bool BotAim::ReadDelayed(const CBot &bot, float delay, Vector &point)
{
	const BotAim::State &aim = bot.aim;
	if (aim.delayCount == 0)
		return false;

	float want = gpGlobals->time - delay;

	size_t newer = (aim.delayHead + BotAim::kDelaySlots - 1) % BotAim::kDelaySlots;
	if (aim.delaySlots[newer].time < want)
		return false;

	for (size_t back = 0; back + 1 < aim.delayCount; ++back)
	{
		size_t older = (newer + BotAim::kDelaySlots - 1) % BotAim::kDelaySlots;
		const BotAim::DelaySlot &a = aim.delaySlots[older];
		if (a.time > want)
		{
			newer = older;
			continue;
		}

		const BotAim::DelaySlot &b = aim.delaySlots[newer];
		float span = b.time - a.time;
		float f = (span > 1e-6f) ? (want - a.time) / span : 0.f;
		if (f < 0.f)
			f = 0.f;
		else if (f > 1.f)
			f = 1.f;

		point = a.point + (b.point - a.point) * f;
		return true;
	}

	return false;
}

float BotAim::PitchLimit()
{
	return kPitchLimit;
}

float BotAim::ViewSpeed(const CBot &bot)
{
	const BotAim::State &aim = bot.aim;
	return sqrtf(aim.yawVel * aim.yawVel + aim.pitchVel * aim.pitchVel);
}

float BotAim::Delay()
{
	cvar_t *cvar = g_engfuncs.pfnCVarGetPointer("crossbot_aim_delay");
	float delay = cvar ? cvar->value : kDefaultDelay;
	if (delay < 0.f)
		delay = 0.f;
	else if (delay > kMaxDelay)
		delay = kMaxDelay;
	return delay;
}

float BotAim::ErrorScale()
{
	cvar_t *cvar = g_engfuncs.pfnCVarGetPointer("crossbot_aim_error");
	float scale = cvar ? cvar->value : kDefaultErrorScale;
	if (scale < 0.f)
		scale = 0.f;
	else if (scale > kMaxErrorScale)
		scale = kMaxErrorScale;
	return scale;
}

float BotAim::DriftAmplitude()
{
	cvar_t *cvar = g_engfuncs.pfnCVarGetPointer("crossbot_aim_drift");
	float amp = cvar ? cvar->value : kDefaultDrift;
	if (amp < 0.f)
		amp = 0.f;
	else if (amp > kMaxDrift)
		amp = kMaxDrift;
	return amp;
}

void BotAim::ClearDelayQueue(CBot &bot)
{
	bot.aim.delayHead = 0;
	bot.aim.delayCount = 0;
}

bool BotAim::AnglesToTarget(const CBot &bot, edict_t *target, float &yaw, float &pitch)
{
	if (!target)
		return false;

	return LosAngles(bot.pEdict->v.origin + bot.pEdict->v.view_ofs,
		target->v.origin + target->v.view_ofs, yaw, pitch);
}

float BotAim::QueueAge(const CBot &bot)
{
	const BotAim::State &aim = bot.aim;
	if (aim.delayCount == 0)
		return 0.f;

	size_t oldest = (aim.delayHead + kDelaySlots - aim.delayCount) % kDelaySlots;
	return gpGlobals->time - aim.delaySlots[oldest].time;
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

	SeedAim(bot);

	if (aim.testActive && gpGlobals->time > aim.testUntil)
		aim.testActive = false;

	Vector desired = aim.testActive ? aim.testAngles : bot.desiredAngles;

	if (!aim.testActive)
	{
		float amp = BotAim::DriftAmplitude();
		if (amp > 0.f)
		{
			desired.y += amp * sinf(gpGlobals->time * kDriftYawRate + aim.driftPhaseYaw);
			desired.x += amp * kDriftPitchScale *
				sinf(gpGlobals->time * kDriftPitchRate + aim.driftPhasePitch);
		}
	}

	float dt = gpGlobals->frametime;
	if (dt < kMinFrameTime)
		dt = kMinFrameTime;
	else if (dt > kMaxFrameTime)
		dt = kMaxFrameTime;

	float fovScale = FovScale(pEdict);

	float yawErr = Bot_NormalizeAngle(desired.y - current.y);
	float pitchErr = Bot_NormalizeAngle(desired.x - current.x);

	UpdateBrake(bot, yawErr, pitchErr);

	float yaw = Bot_NormalizeAngle(current.y +
		StepAxis(yawErr, fovScale, kMaxAccelYaw, aim.brakeYaw, dt, aim.yawVel));
	float pitch = current.x +
		StepAxis(pitchErr, fovScale, kMaxAccelPitch, aim.brakePitch, dt, aim.pitchVel);

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
