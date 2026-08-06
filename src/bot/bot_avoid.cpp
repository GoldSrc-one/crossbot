#include <extdll.h>
#include <meta_api.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

#include "bot_avoid.h"
#include "bot.h"
#include "../nav/nav_mesh.h"
#include "../nav/nav_query.h"
#include "../nav/waypoint_plan.h"

namespace
{
	constexpr int kHullHuman = 1;
	constexpr int kHullDuck = 3;

	constexpr float kDuckHullHalf = NavPlan::kPlanStandHeight;
	constexpr float kStandHullHalf = kDuckHullHalf * 2.f;
	constexpr float kDuckClipHeight = kDuckHullHalf * 2.f;
	constexpr float kAvoidStepHeight = 18.f;

	constexpr float kAvoidLookaheadTime = 0.5f;
	constexpr float kAvoidLookaheadMin = 64.f;
	constexpr float kAvoidLookaheadMax = 160.f;

	constexpr float kAvoidDuckClearance = kDuckClipHeight + 4.f;
	constexpr float kAvoidCrawlPenalty = 1.25f;
	constexpr float kAvoidDuckMinHold = 0.5f;
	constexpr float kAvoidDuckMaxHold = 4.f;

	constexpr float kAvoidJumpMargin = 4.f;
	constexpr float kAvoidJumpLead = 8.f;
	constexpr float kAvoidJumpCooldown = 0.2f;

	constexpr size_t kAvoidRouteLookahead = 3;

	const char *ClassOf(edict_t *ent)
	{
		if (!ent || !gpGlobals->pStringBase)
			return "?";
		const char *s = gpGlobals->pStringBase + static_cast<unsigned int>(ent->v.classname);
		return (s && s[0]) ? s : "?";
	}

	float Length2D(const Vector &v)
	{
		return sqrtf(v.x * v.x + v.y * v.y);
	}

	bool AabbOverlap(const Vector &aMin, const Vector &aMax, const Vector &bMin, const Vector &bMax)
	{
		return aMin.x <= bMax.x && aMax.x >= bMin.x
			&& aMin.y <= bMax.y && aMax.y >= bMin.y
			&& aMin.z <= bMax.z && aMax.z >= bMin.z;
	}

	float RiseTime(float rise, float gravity)
	{
		if (rise <= 0.f)
			return 0.f;
		if (gravity <= 1.f)
			return -1.f;
		float disc = PmSim::kJumpImpulse * PmSim::kJumpImpulse - 2.f * gravity * rise;
		if (disc < 0.f)
			return -1.f;
		return (PmSim::kJumpImpulse - sqrtf(disc)) / gravity;
	}

	bool RouteExemptsHit(const CBot &bot, const Nav::NavMesh &mesh, edict_t *hit, int solid)
	{
		if (solid != SOLID_BSP)
			return false;

		size_t end = std::min(bot.waypoints.size(), bot.waypointIndex + kAvoidRouteLookahead);
		for (size_t i = bot.waypointIndex; i < end; ++i)
		{
			const Nav::Waypoint &wp = bot.waypoints[i];
			if (wp.requiresElevatorRide)
				return true;
			if (!wp.requiresDoor && !wp.requiresBreak)
				continue;
			if (wp.targetAreaId < 0)
				continue;

			const Nav::Area *area = Nav::FindAreaById(mesh, static_cast<uint32_t>(wp.targetAreaId));
			if (!area)
				continue;
			if (AabbOverlap(area->mins, area->maxs, hit->v.absmin, hit->v.absmax))
				return true;
		}
		return false;
	}

	bool HitQualifies(const CBot &bot, const Nav::NavMesh &mesh, edict_t *hit)
	{
		if (!hit)
			return false;
		if (g_engfuncs.pfnIndexOfEdict(hit) == 0)
			return false;

		int solid = hit->v.solid;
		if (solid != SOLID_BSP && solid != SOLID_BBOX && solid != SOLID_SLIDEBOX)
			return false;
		if (solid == SOLID_BSP && (hit->v.flags & FL_WORLDBRUSH) != 0)
			return false;

		return !RouteExemptsHit(bot, mesh, hit, solid);
	}

	void Record(BotAvoid::State &state, BotAvoid::Kind kind, edict_t *hit, float gap, float top)
	{
		state.lastKind = kind;
		state.lastGap = gap;
		state.lastTop = top;
		strncpy(state.lastHit, ClassOf(hit), sizeof(state.lastHit) - 1);
		state.lastHit[sizeof(state.lastHit) - 1] = '\0';
	}

	void Announce(const CBot &bot, const char *what, edict_t *hit, float gap, float top, float dist,
		float speed, float hold)
	{
		const Vector &origin = bot.pEdict->v.origin;
		char msg[256];
		snprintf(msg, sizeof(msg),
			"CrossBot: %s avoid %s %s at (%.1f,%.1f,%.1f) gap %.0f top %.0f dist %.0f speed %.0f "
			"hold %.2f\n",
			bot.name, what, ClassOf(hit), origin.x, origin.y, origin.z, gap, top, dist, speed, hold);
		SERVER_PRINT(msg);
	}
}

namespace BotAvoid
{

const char *KindName(Kind kind)
{
	switch (kind)
	{
	case AVOID_DUCK: return "duck";
	case AVOID_JUMP: return "jump";
	default: return "none";
	}
}

bool Enabled()
{
	cvar_t *cvar = g_engfuncs.pfnCVarGetPointer("crossbot_nav_avoid");
	return !cvar || cvar->value != 0.f;
}

Decision Update(CBot &bot, const Nav::NavMesh &mesh, const Vector &pathDir,
	const PmSim::PmPhysics &physics)
{
	Decision out;

	State &state = bot.avoid;
	edict_t *pEdict = bot.pEdict;
	float now = gpGlobals->time;

	bool onGround = (pEdict->v.flags & FL_ONGROUND) != 0;
	bool ducking = (pEdict->v.flags & FL_DUCKING) != 0;

	if (state.jumpArmed)
	{
		if (onGround && now >= state.jumpCooldownUntil)
			state.jumpArmed = false;
		else
			out.duck = true;
	}

	const Vector &origin = pEdict->v.origin;
	const Vector &vel = pEdict->v.velocity;
	float speed = Length2D(vel);

	if (onGround && !state.jumpArmed && (pathDir.x != 0.f || pathDir.y != 0.f))
	{
		int hull = ducking ? kHullDuck : kHullHuman;
		float feetZ = origin.z - (ducking ? kDuckHullHalf : kStandHullHalf);

		float lookahead = speed * kAvoidLookaheadTime;
		lookahead = std::max(kAvoidLookaheadMin, std::min(kAvoidLookaheadMax, lookahead));

		TraceResult tr;
		Vector end = origin + pathDir * lookahead;
		g_engfuncs.pfnTraceHull(origin, end, dont_ignore_monsters, hull, pEdict, &tr);

		if (tr.flFraction < 1.f && HitQualifies(bot, mesh, tr.pHit))
		{
			edict_t *hit = tr.pHit;
			float dist = Length2D(tr.vecEndPos - origin);
			float gap = hit->v.absmin.z - feetZ;
			float top = hit->v.absmax.z - feetZ;
			float depth = std::min(hit->v.absmax.x - hit->v.absmin.x,
				hit->v.absmax.y - hit->v.absmin.y);

			float duckSpeed = physics.maxSpeed * PmSim::kDuckingMultiplier;
			if (duckSpeed < 1.f)
				duckSpeed = 1.f;

			float plainJump = PmSim::PlainJumpHeight(physics.gravity);
			float crouchJump = plainJump + kDuckHullHalf;

			if (top > kAvoidStepHeight && top <= crouchJump - kAvoidJumpMargin)
			{
				if (ducking && top > plainJump)
				{
					state.duckUntil = 0.f;
					out.duck = false;
					out.suppressPlanCrouch = true;
					Record(state, AVOID_JUMP, hit, gap, top);
				}
				else
				{
					float riseTime = RiseTime(top - kDuckHullHalf, physics.gravity);
					if (riseTime >= 0.f && now >= state.jumpCooldownUntil
						&& dist <= speed * riseTime + kAvoidJumpLead)
					{
						float hold = riseTime
							+ (depth + 2.f * NavPlan::kPlanClearance) / duckSpeed;

						out.jump = true;
						state.jumpArmed = true;
						state.jumpCooldownUntil = now + kAvoidJumpCooldown;
						state.duckUntil = std::max(state.duckUntil, now + hold);

						Announce(bot, "jump", hit, gap, top, dist, speed, hold);
					}
					Record(state, AVOID_JUMP, hit, gap, top);
				}
			}
			else if (!ducking && gap >= kAvoidDuckClearance)
			{
				float hold = (dist + depth + 2.f * NavPlan::kPlanClearance) / duckSpeed
					* kAvoidCrawlPenalty;
				hold = std::max(kAvoidDuckMinHold, std::min(kAvoidDuckMaxHold, hold));

				if (state.duckUntil <= now)
					Announce(bot, "duck", hit, gap, top, dist, speed, hold);

				state.duckUntil = std::max(state.duckUntil, now + hold);
				Record(state, AVOID_DUCK, hit, gap, top);
			}
		}
	}

	if (now < state.duckUntil)
		out.duck = true;
	if (out.duck)
		out.suppressPlanCrouch = false;

	out.slowApproach = out.duck && !ducking;

	if (!out.duck && !out.jump && !out.suppressPlanCrouch && !state.jumpArmed)
	{
		state.lastKind = AVOID_NONE;
		state.lastHit[0] = '\0';
	}

	return out;
}

}
