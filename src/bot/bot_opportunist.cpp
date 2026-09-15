#include <extdll.h>
#include <meta_api.h>

#include <cmath>
#include <cstdio>

#include "bot_opportunist.h"
#include "bot_log.h"
#include "bot_difficulty.h"
#include "bot_move.h"
#include "bot_move_nav.h"
#include "../nav/astar_graph.h"
#include "../nav/nav.h"
#include "../nav/nav_math.h"
#include "../nav/nav_path.h"
#include "../nav/nav_query.h"
#include "../nav/pm_sim.h"

namespace BotOppo
{

namespace
{
	constexpr int kHullDuck = 3;

	constexpr float kMinGainSeconds = 0.35f;
	constexpr float kSpeedTriggerRatio = 1.25f;
	constexpr float kMinLaunchSpeed = 60.f;
	constexpr float kMaxSimSeconds = 6.f;
	constexpr float kMaxLaunchYawRate = 30.f;
	constexpr float kMaxLaunchPitch = 20.f;

	constexpr float kBunnyInterval = 0.1f;
	constexpr float kBunnySpeedRatio = 0.75f;
	constexpr float kBunnyCorridor = 64.f;
	constexpr float kBunnyMinProgress = 96.f;
	constexpr float kBunnySimSeconds = 3.f;
	constexpr float kBunnyLookahead = 256.f;

	float Speed2D(const Vector &v)
	{
		return sqrtf(v.x * v.x + v.y * v.y);
	}

	float SegmentDist2D(const Vector &p, const Vector &a, const Vector &b, float &alongOut)
	{
		Vector ab(b.x - a.x, b.y - a.y, 0.f);
		Vector ap(p.x - a.x, p.y - a.y, 0.f);

		float len2 = ab.x * ab.x + ab.y * ab.y;
		float t = (len2 > 0.f) ? ((ap.x * ab.x + ap.y * ab.y) / len2) : 0.f;
		if (t < 0.f)
			t = 0.f;
		else if (t > 1.f)
			t = 1.f;

		alongOut = sqrtf(len2) * t;

		Vector off(ap.x - ab.x * t, ap.y - ab.y * t, 0.f);
		return sqrtf(off.x * off.x + off.y * off.y);
	}

	bool Refuse(CBot &bot, int reason)
	{
		++bot.bunnyRefuse[reason];
		bot.bunnyNextCheck = gpGlobals->time + kBunnyInterval;
		return false;
	}

	bool RebuildCostsImpl(CBot &bot, const Nav::NavMesh &mesh)
	{
		const Nav::Area *goalArea = Nav::FindAreaNearest(mesh, bot.goalPoint);
		if (!goalArea)
			return false;

		if (bot.oppoCostsGoalArea == static_cast<int>(goalArea->id) &&
			bot.oppoCosts.size() == mesh.areas.size())
			return true;

		Nav::AreaCostsToGoal(mesh, goalArea->id, bot.oppoCosts, Bot_GravityMultiplier(bot.pEdict),
			0.f, Bot_MaxSpeed(bot.pEdict), NavPlan::RangeBias(), Bot_FallDamage(bot));
		bot.oppoCostsGoalArea = static_cast<int>(goalArea->id);
		return true;
	}
}

bool RebuildCosts(CBot &bot, const Nav::NavMesh &mesh)
{
	return RebuildCostsImpl(bot, mesh);
}

bool Enabled()
{
	float pin = BotDiff::Pin("crossbot_oppo_jump");
	return pin >= 0.f ? pin != 0.f : true;
}

void OnRepath(CBot &bot)
{
	bot.oppoCostsGoalArea = -1;
}

bool WantBunnyHop(CBot &bot, const Nav::NavMesh &mesh)
{
	edict_t *pEdict = bot.pEdict;

	if (!BotDiff::BunnyHop())
		return false;
	if ((pEdict->v.flags & FL_ONGROUND) == 0)
		return false;
	if (bot.airmoveActive || bot.ladderActive || bot.rideActive || bot.activationActive)
		return false;
	if (bot.waypointIndex >= bot.waypoints.size())
		return false;
	if (gpGlobals->time < bot.bunnyNextCheck)
	{
		++bot.bunnyRefuse[0];
		return false;
	}

	PmSim::PmPhysics physics = Bot_Physics(pEdict);
	const Vector &vel = pEdict->v.velocity;
	float speed = Speed2D(vel);

	if (speed < kBunnySpeedRatio * physics.maxSpeed)
	{
		++bot.bunnyRefuse[1];
		return false;
	}

	float ahead = 0.f;
	Vector walk = pEdict->v.origin;
	for (size_t i = bot.waypointIndex; i < bot.waypoints.size() && ahead < kBunnyLookahead; ++i)
	{
		const Nav::Waypoint &wp = bot.waypoints[i];
		if (wp.airmoveLaunch || wp.requiresLadder || wp.requiresCrouch ||
			wp.requiresElevatorRide || wp.requiresButtonUse || wp.requiresTriggerTouch ||
			wp.requiresTeleport || wp.requiresBreak || wp.requiresDoor)
			return Refuse(bot, 2);

		Vector step(wp.point.x - walk.x, wp.point.y - walk.y, 0.f);
		ahead += step.Length();
		walk = wp.point;
	}

	float yaw = atan2f(vel.y, vel.x) * (180.f / NavMath::kPiF);

	Bot_SimulateAirmoveArc(pEdict, pEdict->v.origin, nullptr, yaw, speed, true, kHullDuck, 0.f,
		kBunnySimSeconds, bot.probeArc);

	if (!bot.probeArc.landed)
		return Refuse(bot, 3);
	if (Bot_FallDamage(bot) && PmSim::FallDamageFor(bot.probeArc.fallSpeed) > 0.f)
		return Refuse(bot, 3);

	const Vector &landing = bot.probeArc.points.back();

	float miss = kBunnyCorridor + 1.f;
	float progress = 0.f;
	float walked = 0.f;
	Vector prev = pEdict->v.origin;
	for (size_t i = bot.waypointIndex; i < bot.waypoints.size(); ++i)
	{
		float along = 0.f;
		float d = SegmentDist2D(landing, prev, bot.waypoints[i].point, along);
		if (d < miss)
		{
			miss = d;
			progress = walked + along;
		}

		Vector leg(bot.waypoints[i].point.x - prev.x, bot.waypoints[i].point.y - prev.y, 0.f);
		walked += leg.Length();
		prev = bot.waypoints[i].point;
	}

	if (miss > kBunnyCorridor || progress < kBunnyMinProgress)
		return Refuse(bot, 4);

	++bot.bunnyHops;
	bot.bunnyAirborne = true;

	BOT_VLOG("CrossBot: %s bunnyhops at %.0f of %.0f from (%.1f,%.1f,%.1f), lands "
		"(%.1f,%.1f,%.1f) %.1f off the path, %.0f along it, after %.2fs\n", bot.name, speed,
		physics.maxSpeed, pEdict->v.origin.x, pEdict->v.origin.y, pEdict->v.origin.z,
		landing.x, landing.y, landing.z, miss, progress, bot.probeArc.flightTime);

	return true;
}

bool TryJump(CBot &bot, const Nav::NavMesh &mesh)
{
	edict_t *pEdict = bot.pEdict;

	if (!Enabled() || !bot.hasGoal)
		return false;
	if (gpGlobals->time < bot.oppoNextCheck)
		return false;
	if (bot.airmoveActive || bot.ladderActive || bot.rideActive || bot.activationActive)
		return false;
	if ((pEdict->v.flags & FL_ONGROUND) == 0)
		return false;
	if (bot.currentAreaId < 0)
		return false;
	if (bot.waypointIndex < bot.waypoints.size() && bot.waypoints[bot.waypointIndex].airmoveLaunch)
		return false;

	bot.oppoNextCheck = gpGlobals->time + BotDiff::OppoInterval();

	const Vector &vel = pEdict->v.velocity;
	float speed = Speed2D(vel);
	if (speed < kMinLaunchSpeed)
		return false;

	bool longJump = Bot_HasLongJumpModule(pEdict);
	if (!longJump && speed < kSpeedTriggerRatio * mesh.designMaxSpeed)
		return false;

	if (longJump && (fabsf(bot.aim.yawVel) > kMaxLaunchYawRate ||
		fabsf(pEdict->v.v_angle.x) > kMaxLaunchPitch))
		return false;

	if (!RebuildCostsImpl(bot, mesh))
		return false;

	uint32_t here = static_cast<uint32_t>(bot.currentAreaId);
	if (here >= bot.oppoCosts.size() || bot.oppoCosts[here] >= Nav::kUnreachableSeconds)
		return false;

	PmSim::PmPhysics physics = Bot_Physics(pEdict);

	float velYaw = atan2f(vel.y, vel.x) * (180.f / NavMath::kPiF);
	float yaw = velYaw;

	Vector launchVel;
	const Vector *airborneVel = nullptr;

	if (longJump)
	{
		float pitch = pEdict->v.v_angle.x + pEdict->v.punchangle.x;
		yaw = Bot_NormalizeAngle(pEdict->v.v_angle.y + pEdict->v.punchangle.y);

		float horizontal = PmSim::kLongJumpSpeed * cosf(pitch * (NavMath::kPiF / 180.f));
		Vector fwd = Bot_YawDir(yaw);
		launchVel = Vector(fwd.x * horizontal, fwd.y * horizontal, PmSim::kLongJumpImpulse);
		airborneVel = &launchVel;
	}

	Bot_SimulateAirmoveArc(pEdict, pEdict->v.origin, airborneVel, yaw,
		longJump ? physics.maxSpeed : speed, true, kHullDuck, 0.f, kMaxSimSeconds, bot.airmoveArc);

	if (!bot.airmoveArc.landed)
		return false;
	if (Bot_FallDamage(bot) && PmSim::FallDamageFor(bot.airmoveArc.fallSpeed) > 0.f)
		return false;

	const Vector &simLanding = bot.airmoveArc.points.back();
	const Nav::Area *landedArea = Nav::FindAreaExact(mesh, simLanding);
	if (!landedArea || landedArea->id == here)
		return false;

	uint32_t land = landedArea->id;
	if (land >= bot.oppoCosts.size() || bot.oppoCosts[land] >= Nav::kUnreachableSeconds)
		return false;

	float flight = bot.airmoveArc.flightTime * (physics.maxSpeed / NavAStar::kNavRunSpeed);
	float gain = bot.oppoCosts[here] - bot.oppoCosts[land] - flight;
	if (gain <= kMinGainSeconds)
		return false;

	bot.airmoveActive = true;
	bot.airmoveOpportunistic = true;
	bot.airmoveYaw = yaw;
	bot.airmoveSpeed = physics.maxSpeed;
	bot.airmoveCrouch = longJump;
	bot.airmoveStartTime = gpGlobals->time;
	bot.airmoveTargetArea = static_cast<int>(land);
	bot.airmoveConnIndex = -1;
	bot.airmovePlannedTime = bot.airmoveArc.flightTime;
	bot.airmoveMaxTime = kMaxSimSeconds;
	bot.airmoveOverrunLogged = false;
	bot.airmoveStrafeSide = 1.f;
	bot.airmovePeakError = 0.f;
	bot.airmoveCorrectFrames = 0;

	BOT_VLOG("CrossBot: %s oppo jump at (%.1f,%.1f,%.1f) yaw %.0f (vel %.0f) speed %.0f kind %s | "
		"here %.2fs land %.2fs flight %.2fs (%.2fs real) gain %.2fs -> area %u\n",
		bot.name, pEdict->v.origin.x, pEdict->v.origin.y, pEdict->v.origin.z, yaw, velYaw, speed,
		longJump ? "lj" : "jump", bot.oppoCosts[here], bot.oppoCosts[land], flight,
		bot.airmoveArc.flightTime, gain,
		land);

	if (!Bot_AirmoveCorrectEnabled())
		bot.airmoveArc.Clear();

	return true;
}

}
