#include <extdll.h>
#include <meta_api.h>

#include <cmath>

#include "bot_gauss.h"
#include "bot.h"
#include "bot_aim.h"
#include "bot_difficulty.h"
#include "bot_fire.h"
#include "bot_log.h"
#include "bot_move.h"
#include "bot_move_nav.h"
#include "bot_opportunist.h"
#include "../game/game_profile.h"
#include "../nav/astar_graph.h"
#include "../nav/nav.h"
#include "../nav/nav_math.h"
#include "../nav/nav_path.h"
#include "../nav/nav_query.h"
#include "../nav/pm_sim.h"

namespace BotGauss
{

namespace
{
	constexpr int kHullDuck = 3;

	constexpr float kPushPerDamage = 5.f;
	constexpr float kPolicyDamage = 100.f;

	constexpr float kPitchMin = 40.f;
	constexpr float kPitchMax = 65.f;

	constexpr float kProbeInterval = 0.1f;
	constexpr float kProbeWindow = 2.f;
	constexpr int kProbeDirs = 4;

	constexpr float kAimTolerance = 6.f;
	constexpr float kMinGainSeconds = 0.35f;
	constexpr float kMaxSimSeconds = 6.f;

	float RandomUnit(CBot &bot)
	{
		return BotAim::RandomRoll(bot);
	}

	bool HoldingCharge(const CBot &bot)
	{
		const BotInventory &inv = bot.inventory;
		if (inv.activeIndex < 0 || inv.activeIndex >= inv.count)
			return false;

		const BotWeapon &w = inv.weapons[inv.activeIndex];
		if (!g_pGameProfile)
			return false;

		const WeaponSpec *spec = g_pGameProfile->GetWeaponSpec(w.id);
		if (!spec || spec->secondary.fire != FIRE_CHARGE)
			return false;

		return bot.fire.commit == BotFire::COMMIT_CHARGE &&
			gpGlobals->time - bot.fire.commitStart >= spec->secondary.chargeFull * 0.5f;
	}

	bool Evaluate(CBot &bot, const Nav::NavMesh &mesh, float yaw, float pitch, float &gainOut)
	{
		edict_t *pEdict = bot.pEdict;

		float yawRad = yaw * (NavMath::kPiF / 180.f);
		float pitchRad = pitch * (NavMath::kPiF / 180.f);

		Vector aim(cosf(pitchRad) * cosf(yawRad), cosf(pitchRad) * sinf(yawRad), -sinf(pitchRad));

		float push = kPolicyDamage * kPushPerDamage;
		Vector launch = pEdict->v.velocity - aim * push;

		float launchYaw = atan2f(launch.y, launch.x) * (180.f / NavMath::kPiF);
		float launchSpeed = sqrtf(launch.x * launch.x + launch.y * launch.y);

		Bot_SimulateAirmoveArc(pEdict, pEdict->v.origin, &launch, launchYaw, launchSpeed, true,
			kHullDuck, 0.f, kMaxSimSeconds, bot.airmoveArc);

		if (!bot.airmoveArc.landed)
			return false;
		if (PmSim::FallDamageFor(bot.airmoveArc.fallSpeed) > 0.f)
			return false;

		const Vector &landing = bot.airmoveArc.points.back();
		const Nav::Area *landedArea = Nav::FindAreaExact(mesh, landing);
		if (!landedArea)
			return false;

		uint32_t here = static_cast<uint32_t>(bot.currentAreaId);
		uint32_t land = landedArea->id;
		if (land == here)
			return false;
		if (here >= bot.oppoCosts.size() || land >= bot.oppoCosts.size())
			return false;
		if (bot.oppoCosts[here] >= Nav::kUnreachableSeconds ||
			bot.oppoCosts[land] >= Nav::kUnreachableSeconds)
			return false;

		PmSim::PmPhysics physics = Bot_Physics(pEdict);
		float flight = bot.airmoveArc.flightTime * (physics.maxSpeed / NavAStar::kNavRunSpeed);

		gainOut = bot.oppoCosts[here] - bot.oppoCosts[land] - flight;
		return gainOut > kMinGainSeconds;
	}
}

bool Enabled()
{
	return BotDiff::GaussPolicy();
}

void Reset(CBot &bot)
{
	bot.gaussAiming = false;
	bot.gaussWindowStart = 0.f;
	bot.gaussNextCheck = 0.f;
}

bool Facing(const CBot &bot, float &yawOut, float &pitchOut)
{
	if (!bot.gaussAiming)
		return false;

	yawOut = bot.gaussYaw;
	pitchOut = bot.gaussPitch;
	return true;
}

void Update(CBot &bot, const Nav::NavMesh &mesh)
{
	edict_t *pEdict = bot.pEdict;

	if (!Enabled() || !HoldingCharge(bot) || !bot.hasGoal || bot.currentAreaId < 0)
	{
		Reset(bot);
		return;
	}

	Vector point;
	if (BotFire::AimPoint(bot, point))
	{
		Reset(bot);
		return;
	}

	if ((pEdict->v.flags & FL_ONGROUND) == 0)
		return;
	if (bot.airmoveActive || bot.ladderActive || bot.rideActive || bot.activationActive)
		return;

	if (bot.gaussWindowStart == 0.f)
		bot.gaussWindowStart = gpGlobals->time;

	if (bot.gaussAiming)
	{
		float dYaw = Bot_NormalizeAngle(bot.gaussYaw - pEdict->v.v_angle.y);
		float dPitch = Bot_NormalizeAngle(bot.gaussPitch - pEdict->v.v_angle.x);
		if (sqrtf(dYaw * dYaw + dPitch * dPitch) <= kAimTolerance)
		{
			bot.fire.releaseCharge = true;
			BOT_VLOG("CrossBot: %s gauss jump yaw %.0f pitch %.0f\n", bot.name, bot.gaussYaw,
				bot.gaussPitch);
			Reset(bot);
		}
		return;
	}

	if (gpGlobals->time - bot.gaussWindowStart > kProbeWindow)
	{
		bot.fire.releaseCharge = true;
		Reset(bot);
		return;
	}

	if (gpGlobals->time < bot.gaussNextCheck)
		return;

	bot.gaussNextCheck = gpGlobals->time + kProbeInterval;

	if (!BotOppo::RebuildCosts(bot, mesh))
		return;

	float bestGain = 0.f;
	bool have = false;
	float bestYaw = 0.f, bestPitch = 0.f;

	for (int i = 0; i < kProbeDirs; ++i)
	{
		float yaw = RandomUnit(bot) * 360.f - 180.f;
		float pitch = kPitchMin + RandomUnit(bot) * (kPitchMax - kPitchMin);

		float gain = 0.f;
		if (!Evaluate(bot, mesh, yaw, pitch, gain))
			continue;

		if (!have || gain > bestGain)
		{
			have = true;
			bestGain = gain;
			bestYaw = yaw;
			bestPitch = pitch;
		}
	}

	if (!have)
		return;

	bot.gaussAiming = true;
	bot.gaussYaw = bestYaw;
	bot.gaussPitch = bestPitch;
}

}
