#include <extdll.h>
#include <meta_api.h>

#include <algorithm>

#include "bot_aim.h"
#include "bot_difficulty.h"
#include "bot.h"
#include "bot_combat.h"
#include "bot_move.h"
#include "bot_sense.h"
#include "../game/game_profile.h"
#include "../nav/nav_math.h"

namespace
{
	using NavMath::kPiF;

	constexpr float kReferenceFov = 90.f;

	constexpr float kMaxAccelYaw = 3000.f;
	constexpr float kMaxAccelPitch = 2000.f;
	constexpr float kMaxHandSpeed = 520.f;

	constexpr float kPitchLimit = 89.f;

	constexpr float kMinFrameTime = 1.f / 1000.f;
	constexpr float kMaxFrameTime = 1.f / 20.f;

	constexpr float kTestSettleTolerance = 0.5f;
	constexpr float kTestSettleSpeed = 150.f;
	constexpr float kTestTimeout = 3.f;
	constexpr float kTestRestDelay = 2.f;

	constexpr float kFlickMinError = 3.f;
	constexpr float kFlickGrowth = 0.5f;
	constexpr float kMaxGainSpread = 0.5f;
	constexpr float kGainOvershoot = 0.5f;

	constexpr float kDegenerateLos = 0.0001f;

	constexpr float kPredictHorizon = 1.f;

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

	void SeedAim(CBot &bot)
	{
		BotAim::State &aim = bot.aim;
		if (aim.rngSeeded)
			return;

		aim.rngSeeded = true;
		aim.rngState = static_cast<unsigned int>(
			g_engfuncs.pfnIndexOfEdict(bot.pEdict)) * 2654435761u + 1u;
	}

	void UpdateGain(CBot &bot, float yawErr, float pitchErr)
	{
		BotAim::State &aim = bot.aim;

		float mag = sqrtf(yawErr * yawErr + pitchErr * pitchErr);
		bool flick = mag > kFlickMinError && mag > aim.flickMag + kFlickGrowth;
		aim.flickMag = mag;

		float gain = (aim.testActive || bot.sense.target.Get()) ? BotAim::GainSpread() : 0.f;
		if (gain <= 0.f)
		{
			aim.gainYaw = 1.f;
			aim.gainPitch = 1.f;
		}
		else if (flick)
		{
			aim.gainYaw = 1.f + gain * (RandomUnit(aim) * (1.f + kGainOvershoot) - 1.f);
			aim.gainPitch = 1.f + gain * (RandomUnit(aim) * (1.f + kGainOvershoot) - 1.f);
		}
	}

	float StepAxis(float errAngle, float fovScale, float maxAccel, float handSpeed, float dt, float &vel)
	{
		float errHand = errAngle / fovScale;
		float mag = fabsf(errHand);

		float limit = mag / dt;

		float step = maxAccel * dt;
		float envelope = sqrtf(2.f * maxAccel * mag + 0.25f * step * step) - 0.5f * step;
		if (envelope < limit)
			limit = envelope;
		if (limit > handSpeed)
			limit = handSpeed;

		float target = (errHand < 0.f) ? -limit : limit;

		float accel = (target - vel) / dt;
		if (accel > maxAccel)
			accel = maxAccel;
		else if (accel < -maxAccel)
			accel = -maxAccel;

		vel += accel * dt;

		return vel * fovScale * dt;
	}

	template <typename Slot>
	Slot &PushSlot(Slot *slots, size_t &head, size_t &count)
	{
		Slot &slot = slots[head];
		slot.time = gpGlobals->time;

		head = (head + 1) % BotAim::kDelaySlots;
		if (count < BotAim::kDelaySlots)
			++count;
		return slot;
	}

	template <typename Slot>
	bool Bracket(const Slot *slots, size_t head, size_t count, float want, size_t &older,
		size_t &newer, float &f)
	{
		newer = (head + BotAim::kDelaySlots - 1) % BotAim::kDelaySlots;
		older = newer;
		f = 0.f;
		if (slots[newer].time < want)
			return false;

		for (size_t back = 0; back + 1 < count; ++back)
		{
			older = (newer + BotAim::kDelaySlots - 1) % BotAim::kDelaySlots;
			if (slots[older].time > want)
			{
				newer = older;
				continue;
			}

			float span = slots[newer].time - slots[older].time;
			f = (span > 1e-6f) ? (want - slots[older].time) / span : 0.f;
			if (f < 0.f)
				f = 0.f;
			else if (f > 1.f)
				f = 1.f;
			return true;
		}

		older = newer;
		return false;
	}

	void ReadView(const BotAim::State &aim, float delay, Vector &off, Vector &punch)
	{
		size_t older = 0, newer = 0;
		float f = 0.f;
		Bracket(aim.viewSlots, aim.viewHead, aim.viewCount, gpGlobals->time - delay, older, newer, f);

		const BotAim::ViewSlot &a = aim.viewSlots[older];
		const BotAim::ViewSlot &b = aim.viewSlots[newer];
		off = Vector(a.off.x + (b.off.x - a.off.x) * f,
			Bot_NormalizeAngle(a.off.y + Bot_NormalizeAngle(b.off.y - a.off.y) * f), 0.f);
		punch = a.punch + (b.punch - a.punch) * f;
	}

	void UpdateTest(CBot &bot, const Vector &angles)
	{
		BotAim::State &aim = bot.aim;

		float speed = BotAim::ViewSpeed(bot);
		if (speed > aim.testPeakSpeed)
			aim.testPeakSpeed = speed;

		float yawErr = Bot_NormalizeAngle(aim.testAngles.y - angles.y);
		float pitchErr = Bot_NormalizeAngle(aim.testAngles.x - angles.x);

		float fov = bot.pEdict->v.fov;
		if (fov <= 0.f)
			fov = kReferenceFov;

		if (!aim.testRested && gpGlobals->time - aim.testStart >= kTestRestDelay)
		{
			aim.testRested = true;

			char msg[192];
			snprintf(msg, sizeof(msg),
				"CrossBot: %s aim test rest err %.3f deg (yaw %+.3f pitch %+.3f), fov %.0f\n",
				bot.name, sqrtf(yawErr * yawErr + pitchErr * pitchErr), yawErr, pitchErr, fov);
			SERVER_PRINT(msg);
		}

		if (aim.testSettled)
			return;

		if (fabsf(yawErr) > kTestSettleTolerance || fabsf(pitchErr) > kTestSettleTolerance)
			return;
		if (speed > kTestSettleSpeed)
			return;

		aim.testSettled = true;

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

void BotAim::PushDelaySlot(CBot &bot, const Vector &point, const Vector &vel)
{
	BotAim::State &aim = bot.aim;

	BotAim::DelaySlot &slot = PushSlot(aim.delaySlots, aim.delayHead, aim.delayCount);
	slot.point = point;
	slot.vel = vel;
}

bool BotAim::ReadDelayed(const CBot &bot, float delay, Vector &point, Vector *vel)
{
	const BotAim::State &aim = bot.aim;
	if (aim.delayCount == 0)
		return false;

	size_t older = 0, newer = 0;
	float f = 0.f;
	if (!Bracket(aim.delaySlots, aim.delayHead, aim.delayCount, gpGlobals->time - delay, older, newer, f))
		return false;

	const BotAim::DelaySlot &a = aim.delaySlots[older];
	const BotAim::DelaySlot &b = aim.delaySlots[newer];
	point = a.point + (b.point - a.point) * f;
	if (vel)
		*vel = a.vel + (b.vel - a.vel) * f;
	return true;
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

void BotAim::RollDelay(CBot &bot)
{
	SeedAim(bot);

	float lo = 0.f, hi = 0.f;
	BotDiff::AimDelay(lo, hi);

	BotAim::State &aim = bot.aim;
	aim.engagementDelay = lo + RandomUnit(aim) * (hi - lo);
	aim.delayRolled = true;
}

float BotAim::Delay(const CBot &bot)
{
	float delay = bot.aim.engagementDelay;
	if (!bot.aim.delayRolled)
	{
		float lo = 0.f, hi = 0.f;
		BotDiff::AimDelay(lo, hi);
		delay = 0.5f * (lo + hi);
	}

	if (delay < 0.f)
		delay = 0.f;
	else if (delay > kMaxDelay)
		delay = kMaxDelay;
	return delay;
}

float BotAim::GainSpread()
{
	float spread = BotDiff::AimGain();
	if (spread < 0.f)
		spread = 0.f;
	else if (spread > kMaxGainSpread)
		spread = kMaxGainSpread;
	return spread;
}

void BotAim::ClearDelayQueue(CBot &bot)

{
	bot.aim.delayHead = 0;
	bot.aim.delayCount = 0;
}

void BotAim::ClearViewQueue(CBot &bot)
{
	bot.aim.viewHead = 0;
	bot.aim.viewCount = 0;
}

float BotAim::RandomRoll(CBot &bot)
{
	SeedAim(bot);
	return RandomUnit(bot.aim);
}

bool BotAim::ReadPredicted(CBot &bot, float delay, Vector &point, Vector *vel)
{
	bot.aim.predictedNow = false;

	if (ReadDelayed(bot, delay, point, vel))
		return true;

	BotAim::State &aim = bot.aim;
	if (aim.delayCount == 0 || !bot.sense.predictLatched || !bot.combat.active)
		return false;

	size_t newest = (aim.delayHead + BotAim::kDelaySlots - 1) % BotAim::kDelaySlots;
	const BotAim::DelaySlot &slot = aim.delaySlots[newest];

	float ahead = gpGlobals->time - delay - slot.time;
	if (ahead <= 0.f || ahead > kPredictHorizon)
		return false;

	point = slot.point + slot.vel * ahead;
	if (vel)
		*vel = slot.vel;

	bot.aim.predictedNow = true;
	++bot.aim.predicts;
	return true;
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
	aim.testRested = false;
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
		BotDaze::Sway(bot, desired);

	float dt = Bot_FrameTime();
	if (dt < kMinFrameTime)
		dt = kMinFrameTime;
	else if (dt > kMaxFrameTime)
		dt = kMaxFrameTime;

	float fovScale = FovScale(pEdict);

	BotAim::ViewSlot &slot = PushSlot(aim.viewSlots, aim.viewHead, aim.viewCount);
	slot.off = Vector(current.x - aim.own.x, Bot_NormalizeAngle(current.y - aim.own.y), 0.f);
	slot.punch = pEdict->v.punchangle;

	Vector off, punch;
	ReadView(aim, BotAim::Delay(bot), off, punch);
	Vector seen(aim.own.x + off.x, Bot_NormalizeAngle(aim.own.y + off.y), 0.f);

	float yawErr = Bot_NormalizeAngle(desired.y - seen.y);
	float pitchErr = Bot_NormalizeAngle(desired.x - seen.x);

	UpdateGain(bot, yawErr, pitchErr);

	float turn = BotDiff::TurnScale();
	if (turn < 0.f)
		turn = 0.f;
	float handSpeed = kMaxHandSpeed * turn;

	float yawStep = StepAxis(yawErr, fovScale, kMaxAccelYaw * turn, handSpeed, dt, aim.yawVel);
	float yaw = Bot_NormalizeAngle(current.y + yawStep * aim.gainYaw);
	float pitchStep = StepAxis(pitchErr, fovScale, kMaxAccelPitch * turn, handSpeed, dt, aim.pitchVel);
	float pitch = current.x + pitchStep * aim.gainPitch;

	if (pitch > kPitchLimit || pitch < -kPitchLimit)
	{
		pitch = (pitch > 0.f) ? kPitchLimit : -kPitchLimit;
		aim.pitchVel = 0.f;
		pitchStep = pitch - current.x;
	}

	aim.own = Vector(aim.own.x + pitchStep, Bot_NormalizeAngle(aim.own.y + yawStep), 0.f);
	aim.perceived = Vector(seen.x + pitchStep, Bot_NormalizeAngle(seen.y + yawStep), 0.f);
	aim.perceivedPunch = punch;

	if (g_pGameProfile)
	{
		g_pGameProfile->AdjustView(bot, dt, pitch, yaw);
		yaw = Bot_NormalizeAngle(yaw);
	}

	Vector smoothed(pitch, yaw, 0.f);

	pEdict->v.v_angle = smoothed;

	pEdict->v.angles = Vector(-smoothed.x / 3.f, smoothed.y, 0.f);

	if (aim.testActive)
		UpdateTest(bot, smoothed);

	return smoothed;
}
