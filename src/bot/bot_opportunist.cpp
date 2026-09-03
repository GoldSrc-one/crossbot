#include <extdll.h>
#include <meta_api.h>

#include <cmath>
#include <cstdio>

#include "bot_opportunist.h"
#include "bot_move.h"
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

	constexpr float kCheckInterval = 0.25f;
	constexpr float kMinGainSeconds = 0.35f;
	constexpr float kSpeedTriggerRatio = 1.25f;
	constexpr float kMinLaunchSpeed = 60.f;
	constexpr float kMaxSimSeconds = 6.f;
	constexpr float kMaxLaunchYawRate = 30.f;
	constexpr float kMaxLaunchPitch = 20.f;

	float Speed2D(const Vector &v)
	{
		return sqrtf(v.x * v.x + v.y * v.y);
	}

	bool RebuildCosts(CBot &bot, const Nav::NavMesh &mesh)
	{
		const Nav::Area *goalArea = Nav::FindAreaNearest(mesh, bot.goalPoint);
		if (!goalArea)
			return false;

		if (bot.oppoCostsGoalArea == static_cast<int>(goalArea->id) &&
			bot.oppoCosts.size() == mesh.areas.size())
			return true;

		Nav::AreaCostsToGoal(mesh, goalArea->id, bot.oppoCosts, Bot_GravityMultiplier(bot.pEdict),
			0.f, Bot_MaxSpeed(bot.pEdict));
		bot.oppoCostsGoalArea = static_cast<int>(goalArea->id);
		return true;
	}
}

bool Enabled()
{
	cvar_t *cvar = g_engfuncs.pfnCVarGetPointer("crossbot_oppo_jump");
	return cvar && cvar->value != 0.f;
}

void OnRepath(CBot &bot)
{
	bot.oppoCostsGoalArea = -1;
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

	bot.oppoNextCheck = gpGlobals->time + kCheckInterval;

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

	if (!RebuildCosts(bot, mesh))
		return false;

	uint32_t here = static_cast<uint32_t>(bot.currentAreaId);
	if (here >= bot.oppoCosts.size() || bot.oppoCosts[here] >= Nav::kUnreachableSeconds)
		return false;

	PmSim::PmPhysics physics = Bot_Physics(pEdict);
	PmSim::PmSimulator sim(Bot_EngineTraceFn(pEdict, kHullDuck), physics);

	float velYaw = atan2f(vel.y, vel.x) * (180.f / NavMath::kPiF);
	float yaw = velYaw;

	if (longJump)
	{
		float pitch = pEdict->v.v_angle.x + pEdict->v.punchangle.x;
		yaw = Bot_NormalizeAngle(pEdict->v.v_angle.y + pEdict->v.punchangle.y);

		float horizontal = PmSim::kLongJumpSpeed * cosf(pitch * (NavMath::kPiF / 180.f));
		Vector fwd = Bot_YawDir(yaw);
		Vector launchVel(fwd.x * horizontal, fwd.y * horizontal, PmSim::kLongJumpImpulse);
		sim.BeginAirborne(pEdict->v.origin, launchVel, yaw, physics.maxSpeed);
	}
	else
	{
		sim.Begin(pEdict->v.origin, yaw, speed, true);
	}

	bool airborne = false;
	while (sim.Time() < kMaxSimSeconds)
	{
		if (!sim.Step())
			break;
		if (!sim.OnGround())
		{
			airborne = true;
			continue;
		}
		break;
	}

	if (!airborne || !sim.OnGround())
		return false;
	if (PmSim::FallDamageFor(sim.FallSpeed()) > 0.f)
		return false;

	const Nav::Area *landedArea = Nav::FindAreaExact(mesh, sim.Origin());
	if (!landedArea || landedArea->id == here)
		return false;

	uint32_t land = landedArea->id;
	if (land >= bot.oppoCosts.size() || bot.oppoCosts[land] >= Nav::kUnreachableSeconds)
		return false;

	float flight = sim.Time() * (physics.maxSpeed / NavAStar::kNavRunSpeed);
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
	bot.airmovePlannedTime = sim.Time();
	bot.airmoveMaxTime = kMaxSimSeconds;
	bot.airmoveOverrunLogged = false;

	char msg[320];
	snprintf(msg, sizeof(msg),
		"CrossBot: %s oppo jump at (%.1f,%.1f,%.1f) yaw %.0f (vel %.0f) speed %.0f kind %s | "
		"here %.2fs land %.2fs flight %.2fs (%.2fs real) gain %.2fs -> area %u\n",
		bot.name, pEdict->v.origin.x, pEdict->v.origin.y, pEdict->v.origin.z, yaw, velYaw, speed,
		longJump ? "lj" : "jump", bot.oppoCosts[here], bot.oppoCosts[land], flight, sim.Time(), gain,
		land);
	SERVER_PRINT(msg);

	return true;
}

}
