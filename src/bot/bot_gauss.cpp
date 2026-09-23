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
#include "bot_sense.h"
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
	constexpr int kHullHuman = 1;

	constexpr float kPushPerDamage = 5.f;
	constexpr float kPolicyDamage = 100.f;
	constexpr float kMaxDamage = 200.f;
	constexpr float kFullChargeSeconds = 1.5f;

	constexpr float kPitchMin = 40.f;
	constexpr float kPitchMax = 65.f;

	constexpr float kProbeInterval = 0.1f;
	constexpr float kProbeWindow = 2.f;
	constexpr int kProbeDirs = 4;
	constexpr float kOverchargeSlack = 1.f;

	constexpr float kAimTolerance = 6.f;
	constexpr float kMinGainSeconds = 0.35f;
	constexpr float kMaxSimSeconds = 6.f;

	constexpr float kPredictMinAir = 0.15f;
	constexpr float kPredictTimeout = 8.f;

	float RandomUnit(CBot &bot)
	{
		return BotAim::RandomRoll(bot);
	}

	const WeaponMode *HeldCharge(const CBot &bot)
	{
		const BotInventory &inv = bot.inventory;
		if (inv.activeIndex < 0 || inv.activeIndex >= inv.count)
			return nullptr;

		const BotWeapon &w = inv.weapons[inv.activeIndex];
		if (!g_pGameProfile)
			return nullptr;

		const WeaponSpec *spec = g_pGameProfile->GetWeaponSpec(w.id);
		if (!spec || spec->secondary.fire != FIRE_CHARGE)
			return nullptr;

		return bot.fire.commit == BotFire::COMMIT_CHARGE ? &spec->secondary : nullptr;
	}

	bool NearOvercharge(const CBot &bot, const WeaponMode &m)
	{
		return m.chargeMax > 0.f &&
			gpGlobals->time - bot.fire.commitStart >= m.chargeMax - kProbeWindow - kOverchargeSlack;
	}

	void CheckLanding(CBot &bot)
	{
		if (!bot.gaussPredictPending)
			return;

		edict_t *pEdict = bot.pEdict;
		float airTime = gpGlobals->time - bot.gaussPredictTime;

		if ((pEdict->v.flags & FL_ONGROUND) == 0)
		{
			if (airTime > kPredictTimeout)
				bot.gaussPredictPending = false;
			return;
		}

		if (airTime < kPredictMinAir)
			return;

		const Vector &at = pEdict->v.origin;
		Vector miss(at.x - bot.gaussPredicted.x, at.y - bot.gaussPredicted.y, 0.f);

		BOT_VLOG("CrossBot: %s gauss jump landed at (%.1f,%.1f,%.1f) after %.2fs, predicted "
			"(%.1f,%.1f,%.1f) after %.2fs, miss %.1f flat %.1f vertical\n", bot.name,
			at.x, at.y, at.z, airTime, bot.gaussPredicted.x, bot.gaussPredicted.y,
			bot.gaussPredicted.z, bot.gaussPredictFlight, miss.Length(),
			at.z - bot.gaussPredicted.z);

		bot.gaussPredictPending = false;
	}

	bool Evaluate(CBot &bot, const Nav::NavMesh &mesh, float yaw, float pitch, float damage,
		float &gainOut, Vector &landingOut, float &flightOut)
	{
		edict_t *pEdict = bot.pEdict;

		float yawRad = yaw * (NavMath::kPiF / 180.f);
		float pitchRad = pitch * (NavMath::kPiF / 180.f);

		Vector aim(cosf(pitchRad) * cosf(yawRad), cosf(pitchRad) * sinf(yawRad), -sinf(pitchRad));

		float push = damage * kPushPerDamage;
		Vector launch = pEdict->v.velocity - aim * push;

		float launchYaw = atan2f(launch.y, launch.x) * (180.f / NavMath::kPiF);
		float launchSpeed = sqrtf(launch.x * launch.x + launch.y * launch.y);

		Bot_SimulateAirmoveArc(pEdict, pEdict->v.origin, &launch, launchYaw, launchSpeed, true,
			kHullHuman, 0.f, kMaxSimSeconds, bot.probeArc);

		if (!bot.probeArc.landed)
			return false;
		if (Bot_FallDamage(bot) && PmSim::FallDamageFor(bot.probeArc.fallSpeed) > 0.f)
			return false;

		const Vector &landing = bot.probeArc.points.back();
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
		float flight = bot.probeArc.flightTime * (physics.maxSpeed / NavAStar::kNavRunSpeed);

		landingOut = landing;
		flightOut = bot.probeArc.flightTime;

		gainOut = bot.oppoCosts[here] - bot.oppoCosts[land] - flight;
		return gainOut > kMinGainSeconds;
	}
}

bool Enabled()
{
	return BotDiff::GaussPolicy();
}

float ChargeDamage(const CBot &bot)
{
	float held = gpGlobals->time - bot.fire.commitStart;
	float damage = kMaxDamage * (held / kFullChargeSeconds);
	if (damage < 0.f)
		damage = 0.f;
	else if (damage > kMaxDamage)
		damage = kMaxDamage;
	return damage;
}

bool Scared(const CBot &bot)
{
	int counts[BotSense::KIND_COUNT];
	BotSense::LiveCounts(bot, counts);
	return counts[BotSense::KIND_PLAYER] + counts[BotSense::KIND_MONSTER] > 0;
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

	CheckLanding(bot);

	const WeaponMode *charge = HeldCharge(bot);
	if (!Enabled() || !charge || !bot.hasGoal || bot.currentAreaId < 0)
	{
		Reset(bot);
		return;
	}

	Vector point;
	if (BotFire::AimPoint(bot, point) || (Scared(bot) && !NearOvercharge(bot, *charge)))
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
		if (sqrtf(dYaw * dYaw + dPitch * dPitch) > kAimTolerance)
			return;

		float gain = 0.f, flight = 0.f;
		Vector landing;
		float damage = ChargeDamage(bot);
		if (damage < kPolicyDamage)
			return;

		if (!BotOppo::RebuildCosts(bot, mesh) ||
			!Evaluate(bot, mesh, bot.gaussYaw, bot.gaussPitch, damage, gain, landing, flight))
		{
			bot.gaussAiming = false;
			return;
		}

		bot.fire.releaseCharge = true;
		bot.gaussPredicted = landing;
		bot.gaussPredictFlight = flight;
		bot.gaussPredictTime = gpGlobals->time;
		bot.gaussPredictPending = true;

		BOT_VLOG("CrossBot: %s gauss jump yaw %.0f pitch %.0f from (%.1f,%.1f,%.1f) at %.0f, "
			"%.0f damage, %.1fs gain, lands (%.1f,%.1f,%.1f) after %.2fs\n", bot.name, bot.gaussYaw,
			bot.gaussPitch, pEdict->v.origin.x, pEdict->v.origin.y, pEdict->v.origin.z,
			sqrtf(pEdict->v.velocity.x * pEdict->v.velocity.x +
				pEdict->v.velocity.y * pEdict->v.velocity.y),
			damage, gain, landing.x, landing.y, landing.z, flight);

		Reset(bot);
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

		float gain = 0.f, flight = 0.f;
		Vector landing;
		if (!Evaluate(bot, mesh, yaw, pitch, kPolicyDamage, gain, landing, flight))
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
