#include <extdll.h>
#include <meta_api.h>
#include <in_buttons.h>
#include <algorithm>

#include "bot_move_nav.h"
#include "bot_move.h"
#include "../nav/nav.h"
#include "../nav/nav_avoid.h"
#include "../nav/nav_query.h"
#include "../nav/path_astar.h"
#include "../nav/path_follower.h"
#include "../nav/path_funnel.h"

namespace
{
		constexpr float kPi = 3.14159265f;
		constexpr float kMoveSpeed = 250.f;

		constexpr float kWaypointReachedDistance = 24.f;

		constexpr float kJumpTriggerDistance = 100.f;
		constexpr float kJumpFacingToleranceDeg = 20.f;

		constexpr float kDoorWaitTriggerDistance = 64.f;

		constexpr int kHullHuman = 1;
		constexpr int kHullPoint = 0;

		constexpr float kDodgeFloorProbeDepth = 90.f;

		constexpr float kObstacleTraceDistance = 48.f;

		constexpr float kObstacleStuckRepathDelay = 1.5f;

	float NormalizeAngle(float a)
	{
		while (a > 180.f) a -= 360.f;
		while (a < -180.f) a += 360.f;
		return a;
	}

		Vector FindClearDirection(edict_t *pEdict, const Vector &dir2D, float traceDistance)
	{
		static constexpr float kCandidateAngles[] = {0.f, 45.f, -45.f, 90.f, -90.f, 135.f, -135.f, 180.f};

		Vector best = dir2D;
		float bestFraction = -1.f;

		for (float deg : kCandidateAngles)
		{
			float rad = deg * (kPi / 180.f);
			float cs = cosf(rad), sn = sinf(rad);
			Vector candidate(dir2D.x * cs - dir2D.y * sn, dir2D.x * sn + dir2D.y * cs, 0.f);

			TraceResult tr;
			Vector traceEnd = pEdict->v.origin + candidate * traceDistance;
			g_engfuncs.pfnTraceHull(pEdict->v.origin, traceEnd, dont_ignore_monsters, kHullHuman, pEdict, &tr);

				Vector reach = pEdict->v.origin + candidate * (traceDistance * tr.flFraction);
			TraceResult down;
			g_engfuncs.pfnTraceHull(reach, reach - Vector(0.f, 0.f, kDodgeFloorProbeDepth), ignore_monsters,
				kHullPoint, pEdict, &down);
			if (down.flFraction >= 1.0f)
				continue;

			if (tr.flFraction >= 1.0f)
				return candidate;

			if (tr.flFraction > bestFraction)
			{
				bestFraction = tr.flFraction;
				best = candidate;
			}
		}

		return best;
	}

		constexpr float kTeleportJumpThreshold = 150.f;

		constexpr float kRepathInterval = 10.f;

		constexpr float kEtaSafetyMargin = 1.6f;

	void ResetPath(CBot &bot)
	{
		bot.waypoints.clear();
		bot.waypointIndex = 0;
		bot.elevatorReadyTime = 0.f;
		bot.doorWaitUntil = 0.f;
		bot.obstacleBlockedSince = 0.f;
		bot.jumpAttempted = false;
		bot.plannedAreaPath.clear();
		bot.visitedAreaPath.clear();
		bot.pathStartTime = 0.f;
		bot.estimatedArrivalTime = 0.f;
		bot.lastProgressTime = 0.f;
		bot.lastSeenWaypointIndex = 0;
		bot.nextRepathTime = 0.f;
	}

	void WriteTraceLog(CBot &bot)
	{
		if (bot.traceLog.empty())
			return;
		cvar_t *cvar = g_engfuncs.pfnCVarGetPointer("crossbot_trace_path");
		const char *path = (cvar && cvar->string && cvar->string[0]) ? cvar->string : "valve/crossbot_data/nav_trace.txt";
		FILE *f = fopen(path, "w");
		if (f)
		{
			for (const Vector &p : bot.traceLog)
				fprintf(f, "%.1f %.1f %.1f\n", p.x, p.y, p.z);
			fclose(f);
			char msg[128];
			snprintf(msg, sizeof(msg), "CrossBot: wrote nav_trace.txt (%zu points) to %s\n", bot.traceLog.size(), path);
			SERVER_PRINT(msg);
		}
		else
		{
			char msg[256];
			snprintf(msg, sizeof(msg), "CrossBot: FAILED to open trace file: %s\n", path);
			SERVER_PRINT(msg);
		}
		bot.traceLog.clear();
	}

	bool Repath(CBot &bot, const Nav::NavMesh &mesh, edict_t *pEdict)
	{

		bool freshAttempt = bot.waypoints.empty();

		const Nav::Area *startArea = Nav::FindAreaForPoint(mesh, pEdict->v.origin);
		const Nav::Area *goalArea = Nav::FindAreaForPoint(mesh, bot.goalPoint);
		if (!startArea || !goalArea)
			return false;

			Vector clampedGoal = bot.goalPoint;
			clampedGoal.x = std::max(goalArea->mins.x, std::min(goalArea->maxs.x, clampedGoal.x));
			clampedGoal.y = std::max(goalArea->mins.y, std::min(goalArea->maxs.y, clampedGoal.y));
			clampedGoal.z = std::max(goalArea->mins.z, std::min(goalArea->maxs.z, clampedGoal.z));

			Nav::PathResult path = Nav::FindPath(mesh, startArea->id, goalArea->id);
			if (!path.found)
				return false;

			bot.waypoints = Nav::BuildWaypointPath(mesh, path, pEdict->v.origin, clampedGoal);
		bot.waypointIndex = 0;
		bot.plannedAreaPath = path.areaPath;

		if (freshAttempt)
		{
			float totalDist = 0.f, totalElevatorTime = 0.f;
			Vector prev = pEdict->v.origin;
			for (const Nav::Waypoint &wp : bot.waypoints)
			{
				totalDist += (wp.point - prev).Length();
				prev = wp.point;
				if (wp.requiresElevatorRide)
					totalElevatorTime += wp.elevatorTravelTime;
			}
			bot.pathStartTime = gpGlobals->time;
			bot.estimatedArrivalTime = gpGlobals->time + totalDist / kMoveSpeed * kEtaSafetyMargin + totalElevatorTime;
			bot.visitedAreaPath.clear();
			bot.visitedAreaPath.push_back(startArea->id);

					char wpMsg[512];
					snprintf(wpMsg, sizeof(wpMsg), "CrossBot: Repath start=(%.1f,%.1f,%.1f) goal=(%.1f,%.1f,%.1f) waypoints=%zu\n",
						pEdict->v.origin.x, pEdict->v.origin.y, pEdict->v.origin.z,
						clampedGoal.x, clampedGoal.y, clampedGoal.z, bot.waypoints.size());
					SERVER_PRINT(wpMsg);
					for (size_t i = 0; i < bot.waypoints.size(); ++i)
					{
						const Nav::Waypoint &wp = bot.waypoints[i];
						snprintf(wpMsg, sizeof(wpMsg),
							"CrossBot: wp[%zu]=(%.1f,%.1f,%.1f) jump=%d elev=%d tele=%d launch=%d drop=%d btn=%d trig=%d door=%d crouch=%d targArea=%d maxSpeed=%.1f\n",
							i, wp.point.x, wp.point.y, wp.point.z,
							wp.requiresJump, wp.requiresElevatorRide, wp.requiresTeleport,
							wp.requiresLaunch, wp.requiresDrop, wp.requiresButtonUse, wp.requiresTriggerTouch,
							wp.requiresDoor, wp.requiresCrouch, wp.targetAreaId, wp.maxSpeed);
						SERVER_PRINT(wpMsg);
					}
				}
		return true;
	}
}

void Bot_ResetNavPath(CBot &bot)
{
	ResetPath(bot);
}

void Bot_MoveNav(CBot &bot)
{
	edict_t *pEdict = bot.pEdict;
	const Nav::NavMesh *mesh = Nav::Get();
	if (!mesh)
		return;

	if (bot.hasLastOrigin && (pEdict->v.origin - bot.lastOrigin).Length() > kTeleportJumpThreshold)
		ResetPath(bot);
	bot.lastOrigin = pEdict->v.origin;
	bot.hasLastOrigin = true;

	const Nav::Area *curArea = Nav::FindAreaForPoint(*mesh, pEdict->v.origin);
	bot.currentAreaId = curArea ? static_cast<int>(curArea->id) : -1;

	if (!bot.hasGoal)
		{

			Vector angles = Bot_UpdateFacing(bot);
		g_engfuncs.pfnRunPlayerMove(pEdict, angles, 0.f, 0.f, 0.f, 0, 0, Bot_FrameMsec());
		return;
	}

		bool hasPendingActivation = false;
	for (size_t wi = bot.waypointIndex; wi < bot.waypoints.size(); ++wi)
	{
		if (bot.waypoints[wi].requiresButtonUse || bot.waypoints[wi].requiresTriggerTouch)
		{
			hasPendingActivation = true;
			break;
		}
	}

	if (static_cast<int>(bot.waypointIndex) != bot.lastSeenWaypointIndex)
	{
		bot.lastSeenWaypointIndex = static_cast<int>(bot.waypointIndex);
		bot.lastProgressTime = gpGlobals->time;
	}

		bool forcedRepath = bot.nextRepathTime > 0.f && bot.nextRepathTime <= gpGlobals->time;
	bool stalled = !bot.waypoints.empty() && bot.lastProgressTime > 0.f &&
		gpGlobals->time - bot.lastProgressTime > kRepathInterval;
	if (bot.elevatorReadyTime <= 0.f && bot.doorWaitUntil <= 0.f && !hasPendingActivation &&
		(bot.waypoints.empty() || forcedRepath || stalled))
	{
		if (!bot.waypoints.empty())
		{

			char msg[160];
			snprintf(msg, sizeof(msg), "CrossBot: %s mid-route repath (%s) at (%.1f,%.1f,%.1f) wp %zu/%zu\n",
				bot.name, forcedRepath ? "obstacle-forced" : "stalled", pEdict->v.origin.x, pEdict->v.origin.y,
				pEdict->v.origin.z, bot.waypointIndex, bot.waypoints.size());
			SERVER_PRINT(msg);
		}
		if (!Repath(bot, *mesh, pEdict))
		{

			WriteTraceLog(bot);
			bot.hasGoal = false;
			return;
		}
		bot.nextRepathTime = 0.f;
		bot.lastProgressTime = gpGlobals->time;
		bot.lastSeenWaypointIndex = static_cast<int>(bot.waypointIndex);
	}

	if (curArea && (bot.visitedAreaPath.empty() || bot.visitedAreaPath.back() != curArea->id))
		bot.visitedAreaPath.push_back(curArea->id);

	if (bot.traceLog.size() < 100000)
		bot.traceLog.push_back(pEdict->v.origin);

	if (bot.waypointIndex >= bot.waypoints.size())
	{

		char msg[128];
		snprintf(msg, sizeof(msg), "CrossBot: %s reached goal (%.1f,%.1f,%.1f)\n", bot.name, bot.goalPoint.x,
			bot.goalPoint.y, bot.goalPoint.z);
		SERVER_PRINT(msg);

		WriteTraceLog(bot);
		bot.hasGoal = false;
		ResetPath(bot);
		return;
	}

	Nav::Waypoint *wp = &bot.waypoints[bot.waypointIndex];
			Vector toTarget = wp->point - pEdict->v.origin;
		float horizDist = sqrtf(toTarget.x * toTarget.x + toTarget.y * toTarget.y);

			if (bot.waypointIndex == 0 && horizDist < 16.f)
			{
				++bot.waypointIndex;
				if (bot.waypointIndex >= bot.waypoints.size())
				{
					bot.hasGoal = false;
					ResetPath(bot);
					return;
				}

				wp = &bot.waypoints[bot.waypointIndex];
				toTarget = wp->point - pEdict->v.origin;
				horizDist = sqrtf(toTarget.x * toTarget.x + toTarget.y * toTarget.y);
			}

					while (bot.waypointIndex + 1 < bot.waypoints.size())
					{
						const Nav::Waypoint &currentWp = bot.waypoints[bot.waypointIndex];
						bool isCommitmentWp = currentWp.requiresJump || currentWp.requiresLadder ||
							currentWp.requiresElevatorRide || currentWp.requiresTeleport ||
							currentWp.requiresLaunch || currentWp.requiresDrop ||
							currentWp.requiresButtonUse || currentWp.requiresTriggerTouch ||
							currentWp.requiresDoor;

						bool actionDone = false;
						if (currentWp.requiresJump && bot.jumpAttempted) actionDone = true;
						if (currentWp.requiresLadder) actionDone = false;
						if (currentWp.requiresElevatorRide && bot.elevatorReadyTime > 0.f) actionDone = true;
						if (currentWp.requiresTeleport) actionDone = true;
						if (currentWp.requiresLaunch) actionDone = true;
						if (currentWp.requiresDrop && !(pEdict->v.flags & FL_ONGROUND)) actionDone = true;
						if (currentWp.requiresButtonUse && bot.doorWaitUntil > 0.f) actionDone = true;
						if (currentWp.requiresTriggerTouch && bot.doorWaitUntil > 0.f) actionDone = true;
						if (currentWp.requiresDoor && bot.doorWaitUntil > 0.f) actionDone = true;

						if (isCommitmentWp && !actionDone)
							break;

						NavFollow::Decision d = NavFollow::Steer(bot.waypoints, bot.waypointIndex, pEdict->v.origin);
						if (!d.advance)
							break;
						++bot.waypointIndex;
					}

					wp = &bot.waypoints[bot.waypointIndex];
					toTarget = wp->point - pEdict->v.origin;
					horizDist = sqrtf(toTarget.x * toTarget.x + toTarget.y * toTarget.y);

		{
			bool onGroundNow = (pEdict->v.flags & FL_ONGROUND) != 0;
			if (onGroundNow)
			{
				bot.fallTelemetryLogged = false;
			}
			else if (!bot.fallTelemetryLogged && pEdict->v.velocity.z < -120.f && !wp->requiresJump)
			{
				char fallMsg[192];
				snprintf(fallMsg, sizeof(fallMsg),
					"CrossBot: %s falling at (%.1f,%.1f,%.1f) heading to wp %zu/%zu (%.1f,%.1f,%.1f)\n", bot.name,
					pEdict->v.origin.x, pEdict->v.origin.y, pEdict->v.origin.z, bot.waypointIndex,
									bot.waypoints.size(), wp->point.x, wp->point.y, wp->point.z);
				SERVER_PRINT(fallMsg);
				bot.fallTelemetryLogged = true;
			}
		}

		if (wp->requiresElevatorRide)
		{

			if (horizDist >= kWaypointReachedDistance)
			{
				bot.desiredAngles = Vector(0.f, atan2f(toTarget.y, toTarget.x) * (180.f / kPi), 0.f);
				Vector angles = Bot_UpdateFacing(bot);
				g_engfuncs.pfnRunPlayerMove(pEdict, angles, kMoveSpeed, 0.f, 0.f, IN_FORWARD, 0, Bot_FrameMsec());
				return;
			}

			if (bot.elevatorReadyTime <= 0.f)
						bot.elevatorReadyTime = gpGlobals->time + wp->elevatorTravelTime;

			Vector angles = Bot_UpdateFacing(bot);
			g_engfuncs.pfnRunPlayerMove(pEdict, angles, 0.f, 0.f, 0.f, 0, 0, Bot_FrameMsec());

			if (gpGlobals->time >= bot.elevatorReadyTime)
			{
				bot.elevatorReadyTime = 0.f;
				++bot.waypointIndex;
			}
			return;
		}

	if (wp->requiresTeleport)
			{

				if (horizDist >= kWaypointReachedDistance)
				{
					bot.desiredAngles = Vector(0.f, atan2f(toTarget.y, toTarget.x) * (180.f / kPi), 0.f);
					Vector angles = Bot_UpdateFacing(bot);
					g_engfuncs.pfnRunPlayerMove(pEdict, angles, kMoveSpeed, 0.f, 0.f, IN_FORWARD, 0, Bot_FrameMsec());
					return;
				}

				++bot.waypointIndex;
				return;
			}

		if (wp->requiresLaunch)
			{

				if (horizDist >= kWaypointReachedDistance)
				{
					bot.desiredAngles = Vector(0.f, atan2f(toTarget.y, toTarget.x) * (180.f / kPi), 0.f);
					Vector angles = Bot_UpdateFacing(bot);
					g_engfuncs.pfnRunPlayerMove(pEdict, angles, kMoveSpeed, 0.f, 0.f, IN_FORWARD, 0, Bot_FrameMsec());
					return;
				}

				Vector angles = Bot_UpdateFacing(bot);
				g_engfuncs.pfnRunPlayerMove(pEdict, angles, 0.f, 0.f, 0.f, 0, 0, Bot_FrameMsec());

				const Nav::Area *actualArea = Nav::FindAreaForPoint(*mesh, pEdict->v.origin);
				if (actualArea && actualArea->id == static_cast<uint32_t>(wp->targetAreaId))
				{
					++bot.waypointIndex;
				}
				return;
			}

		if (wp->requiresButtonUse)
			{
				if (horizDist >= kWaypointReachedDistance)
				{

					bot.desiredAngles = Vector(0.f, atan2f(toTarget.y, toTarget.x) * (180.f / kPi), 0.f);
					Vector angles = Bot_UpdateFacing(bot);
					g_engfuncs.pfnRunPlayerMove(pEdict, angles, kMoveSpeed, 0.f, 0.f, IN_FORWARD, 0, Bot_FrameMsec());
					return;
				}

				bot.desiredAngles = Vector(0.f, atan2f(toTarget.y, toTarget.x) * (180.f / kPi), 0.f);
				Vector angles = Bot_UpdateFacing(bot);
				g_engfuncs.pfnRunPlayerMove(pEdict, angles, 0.f, 0.f, 0.f, IN_USE, 0, Bot_FrameMsec());

				if (bot.doorWaitUntil <= 0.f)
					bot.doorWaitUntil = gpGlobals->time + wp->buttonWaitTime;

		if (gpGlobals->time >= bot.doorWaitUntil)
		{
			bot.doorWaitUntil = 0.f;
			++bot.waypointIndex;
		}
		return;
	}

	if (wp->requiresDoor)
	{
		if (horizDist >= kDoorWaitTriggerDistance)
		{

					bot.desiredAngles = Vector(0.f, atan2f(toTarget.y, toTarget.x) * (180.f / kPi), 0.f);
					Vector angles = Bot_UpdateFacing(bot);
					g_engfuncs.pfnRunPlayerMove(pEdict, angles, kMoveSpeed, 0.f, 0.f, IN_FORWARD, 0, Bot_FrameMsec());
					return;
				}

				if (bot.doorWaitUntil <= 0.f)
									bot.doorWaitUntil = gpGlobals->time + wp->doorWaitTime;

		Vector angles = Bot_UpdateFacing(bot);
		g_engfuncs.pfnRunPlayerMove(pEdict, angles, 0.f, 0.f, 0.f, 0, 0, Bot_FrameMsec());

		if (gpGlobals->time >= bot.doorWaitUntil)
		{
			bot.doorWaitUntil = 0.f;
			++bot.waypointIndex;
		}
		return;
	}

	bool onLadder = (pEdict->v.movetype == MOVETYPE_FLY);
			bool needsJump = !onLadder && wp->requiresJump;
			bool needsDrop = !onLadder && wp->requiresDrop;

		if (needsJump)
		{

			bool onGround = (pEdict->v.flags & FL_ONGROUND) != 0;
				bool inTargetArea = wp->targetAreaId < 0;
			if (!inTargetArea)
			{
				const Nav::Area *actualArea = Nav::FindAreaForPoint(*mesh, pEdict->v.origin);
					inTargetArea = actualArea && actualArea->id == static_cast<uint32_t>(wp->targetAreaId);
			}
			if (bot.jumpAttempted && onGround && inTargetArea)
			{
				bot.jumpAttempted = false;
				++bot.waypointIndex;
				return;
			}
		}

		if (needsDrop)
			{

				bool onGround = (pEdict->v.flags & FL_ONGROUND) != 0;
				if (!onGround)
				{
					++bot.waypointIndex;
					return;
				}
			}

				NavFollow::Decision steer;
				while (true)
				{
					steer = NavFollow::Steer(bot.waypoints, bot.waypointIndex, pEdict->v.origin);
					if (!steer.advance || bot.waypointIndex + 1 >= bot.waypoints.size())
						break;

					const Nav::Waypoint &currentWp = bot.waypoints[bot.waypointIndex];
					if (currentWp.requiresDrop)
					{
						bool onGround = (pEdict->v.flags & FL_ONGROUND) != 0;
						if (onGround)
							break;
					}
					++bot.waypointIndex;
				}
			if ((!needsJump && !needsDrop) && steer.advance)
			{

				return;
			}

		bool committingToJump = needsJump && horizDist < kJumpTriggerDistance;
		bool committingToDrop = needsDrop && horizDist < kJumpTriggerDistance;

		if (onLadder)
	{

		float climbPitch = (toTarget.z > 0.f) ? -60.f : 60.f;
		bot.desiredAngles = Vector(climbPitch, atan2f(toTarget.y, toTarget.x) * (180.f / kPi), 0.f);
	}
	else if (committingToJump || committingToDrop)
	{

		bot.desiredAngles = Vector(0.f, atan2f(toTarget.y, toTarget.x) * (180.f / kPi), 0.f);
	}
	else
	{

		Vector aimTarget = steer.aimPoint - pEdict->v.origin;

				Vector steerDir = NavAvoid::Deflect(bot, aimTarget);

				float dirLen2D = sqrtf(steerDir.x * steerDir.x + steerDir.y * steerDir.y);
				if (dirLen2D > 1.f)
				{
					Vector dir2D(steerDir.x / dirLen2D, steerDir.y / dirLen2D, 0.f);
					Vector traceEnd = pEdict->v.origin + dir2D * kObstacleTraceDistance;

					TraceResult tr;
					g_engfuncs.pfnTraceHull(pEdict->v.origin, traceEnd, dont_ignore_monsters, kHullHuman, pEdict, &tr);

					bool hitDynamicObstacle = tr.flFraction < 1.0f && tr.pHit &&
						g_engfuncs.pfnIndexOfEdict(tr.pHit) != 0;

			if (hitDynamicObstacle)
			{
				if (bot.obstacleBlockedSince <= 0.f)
					bot.obstacleBlockedSince = gpGlobals->time;

				steerDir = FindClearDirection(pEdict, dir2D, kObstacleTraceDistance);

				if (gpGlobals->time - bot.obstacleBlockedSince > kObstacleStuckRepathDelay)
					bot.nextRepathTime = gpGlobals->time;
			}
			else
			{
				bot.obstacleBlockedSince = 0.f;
			}
		}

		bot.desiredAngles = Vector(0.f, atan2f(steerDir.y, steerDir.x) * (180.f / kPi), 0.f);
	}

	Vector angles = Bot_UpdateFacing(bot);
	unsigned short buttons = IN_FORWARD;

		if (wp->requiresCrouch || (curArea && (curArea->areaFlags & crossbot_nav::NAV_AREA_CROUCH)))
	{
		buttons |= IN_DUCK;
	}

	if (needsJump)
	{
		bool onGround = (pEdict->v.flags & FL_ONGROUND) != 0;
		float facingError = fabsf(NormalizeAngle(angles.y - bot.desiredAngles.y));
		if (onGround && horizDist < kJumpTriggerDistance && facingError < kJumpFacingToleranceDeg)
		{
			buttons |= IN_JUMP;
			bot.jumpAttempted = true;
		}

			if (pEdict->v.velocity.z > 0.f)
				buttons |= IN_DUCK;
	}

		float moveSpeed = (!needsJump && !onLadder) ? steer.baseSpeed : kMoveSpeed;
	if (!onLadder)
	{
		float headingError = fabsf(NormalizeAngle(angles.y - bot.desiredAngles.y));
		moveSpeed = NavFollow::ScaleSpeedByHeadingError(moveSpeed, headingError);
	}

	g_engfuncs.pfnRunPlayerMove(pEdict, angles, moveSpeed, 0.f, 0.f, buttons, 0, Bot_FrameMsec());

		if (bot.waypointIndex == 1 && horizDist > 100.f && horizDist < 200.f)
		{
			static float lastDebugTime = 0.f;
			if (gpGlobals->time - lastDebugTime > 1.f)
			{
				lastDebugTime = gpGlobals->time;
				char dbg[256];
				snprintf(dbg, sizeof(dbg), "CrossBot: dbg pos=(%.1f,%.1f,%.1f) vel=(%.1f,%.1f,%.1f) moveSpeed=%.1f headingErr=%.1f dist=%.1f\n",
					pEdict->v.origin.x, pEdict->v.origin.y, pEdict->v.origin.z,
					pEdict->v.velocity.x, pEdict->v.velocity.y, pEdict->v.velocity.z,
					moveSpeed, fabsf(NormalizeAngle(angles.y - bot.desiredAngles.y)), horizDist);
				SERVER_PRINT(dbg);
			}
		}
}

