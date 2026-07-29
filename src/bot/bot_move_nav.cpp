#include <extdll.h>
#include <meta_api.h>
#include <in_buttons.h>
#include <algorithm>

#include "bot_move_nav.h"
#include "bot_move.h"
#include "../nav/nav.h"
#include "../nav/nav_path.h"
#include "../nav/nav_query.h"
#include "../nav/path_follower.h"

namespace
{
		constexpr float kPi = 3.14159265f;
		constexpr float kMoveSpeed = 250.f;

		// Over maxspeed on purpose: PM_CheckParamters rescales the forward/side pair down to
		// maxspeed, so this just means "full speed in exactly the commanded direction".
		constexpr float kFullDrive = 1000.f;

		// Below this the velocity carries no usable heading, so the aim holds where it is rather
		// than snapping about on numerical noise.
		constexpr float kAimVelocitySpeed = 20.f;

		constexpr float kJumpTriggerDistance = 100.f;
		constexpr float kJumpFacingToleranceDeg = 20.f;

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

	// Every path that stands the bot still, so the reported move pair never lingers from the last
	// frame it was actually walking.
	void RunIdleMove(CBot &bot, const Vector &angles)
	{
		bot.moveForward = 0.f;
		bot.moveSide = 0.f;
		g_engfuncs.pfnRunPlayerMove(bot.pEdict, angles, 0.f, 0.f, 0.f, 0, 0, Bot_FrameMsec());
	}

	void ResetPath(CBot &bot)
	{
		bot.waypoints.clear();
		bot.waypointIndex = 0;
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
			float totalDist = 0.f;
			Vector prev = pEdict->v.origin;
			for (const Nav::Waypoint &wp : bot.waypoints)
			{
				totalDist += (wp.point - prev).Length();
				prev = wp.point;
			}
			bot.pathStartTime = gpGlobals->time;
			bot.estimatedArrivalTime = gpGlobals->time + totalDist / kMoveSpeed * kEtaSafetyMargin;
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
							"CrossBot: wp[%zu]=(%.1f,%.1f,%.1f) jump=%d crouch=%d targArea=%d\n",
							i, wp.point.x, wp.point.y, wp.point.z,
							wp.requiresJump, wp.requiresCrouch, wp.targetAreaId);
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
		RunIdleMove(bot, angles);
		return;
	}

	if (static_cast<int>(bot.waypointIndex) != bot.lastSeenWaypointIndex)
	{
		bot.lastSeenWaypointIndex = static_cast<int>(bot.waypointIndex);
		bot.lastProgressTime = gpGlobals->time;
	}

		bool forcedRepath = bot.nextRepathTime > 0.f && bot.nextRepathTime <= gpGlobals->time;
	bool stalled = !bot.waypoints.empty() && bot.lastProgressTime > 0.f &&
		gpGlobals->time - bot.lastProgressTime > kRepathInterval;
	if (bot.waypoints.empty() || forcedRepath || stalled)
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

					// Steer reports advance for a corner passed and, on the final waypoint, for
					// having arrived. Consuming it in both cases is what ends a route: once the
					// index runs off the end, the arrival check at the top of the next frame fires.
					NavFollow::Config followCfg;
					if (pEdict->v.maxspeed > 1.f)
						followCfg.maxSpeed = pEdict->v.maxspeed;

					NavFollow::Decision steer;
					while (true)
					{
						steer = NavFollow::Steer(bot.waypoints, bot.waypointIndex, pEdict->v.origin,
							pEdict->v.velocity, followCfg);

						// A jump waypoint is a commitment: it is not passed until the jump is made.
						if (bot.waypoints[bot.waypointIndex].requiresJump && !bot.jumpAttempted)
							break;
						if (!steer.advance)
							break;

						++bot.waypointIndex;
						if (bot.waypointIndex >= bot.waypoints.size())
							break;
					}

					if (bot.waypointIndex >= bot.waypoints.size())
					{
						Vector angles = Bot_UpdateFacing(bot);
						RunIdleMove(bot, angles);
						return;
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

		bool needsJump = wp->requiresJump;

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

		bool committingToJump = needsJump && horizDist < kJumpTriggerDistance;

		// Where the feet go. Independent of where the bot looks -- the two only coincide by accident.
		Vector moveDir;
		if (committingToJump)
	{

		moveDir = Vector(toTarget.x, toTarget.y, 0.f);
	}
	else
	{

		// Already a unit direction, and already braked if the bot is inside a corner.
		Vector steerDir = steer.moveDir;

				if (steerDir.x != 0.f || steerDir.y != 0.f)
				{
					Vector dir2D = steerDir;
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

		moveDir = Vector(steerDir.x, steerDir.y, 0.f);
	}

		// Where the bot looks: at the node it is heading for next, so it reads as a player looking
		// round the corner rather than one watching its own feet. Past the last node there is
		// nothing ahead to look at, so it settles on wherever it is actually drifting.
		size_t aimIndex = bot.waypointIndex + 1;
		if (aimIndex < bot.waypoints.size())
		{
			Vector toAim = bot.waypoints[aimIndex].point - pEdict->v.origin;
			if (toAim.x * toAim.x + toAim.y * toAim.y > 1.f)
				bot.desiredAngles = Vector(0.f, atan2f(toAim.y, toAim.x) * (180.f / kPi), 0.f);
		}
		else
		{
			const Vector &vel = pEdict->v.velocity;
			if (sqrtf(vel.x * vel.x + vel.y * vel.y) > kAimVelocitySpeed)
				bot.desiredAngles = Vector(0.f, atan2f(vel.y, vel.x) * (180.f / kPi), 0.f);
		}

	Vector angles = Bot_UpdateFacing(bot);

	float fmove = 0.f, smove = 0.f;
	Bot_DecomposeMove(angles.y + pEdict->v.punchangle.y, moveDir, kFullDrive, fmove, smove);
	bot.moveForward = fmove;
	bot.moveSide = smove;

		// Movement rides on the speeds, not the buttons, but nothing watching a bot should see it
		// claim to walk forward while it strafes.
	unsigned short buttons = 0;
	if (fmove > 1.f) buttons |= IN_FORWARD;
	else if (fmove < -1.f) buttons |= IN_BACK;
	if (smove > 1.f) buttons |= IN_MOVERIGHT;
	else if (smove < -1.f) buttons |= IN_MOVELEFT;

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

	g_engfuncs.pfnRunPlayerMove(pEdict, angles, fmove, smove, 0.f, buttons, 0, Bot_FrameMsec());
}
