#include <extdll.h>
#include <meta_api.h>
#include <in_buttons.h>
#include <algorithm>
#include <chrono>

#include "bot_move_nav.h"
#include "bot_move.h"
#include "bot_avoid.h"
#include "../nav/nav.h"
#include "../nav/nav_math.h"
#include "../nav/nav_path.h"
#include "../nav/nav_query.h"
#include "../nav/path_follower.h"

namespace
{
	using NavMath::kPiF;
	using NavMath::Norm2D;

	constexpr float kMoveSpeed = 250.f;
	constexpr float kFullDrive = 1000.f;
	constexpr float kAimVelocitySpeed = 20.f;
	constexpr int kHullHuman = 1;
	constexpr int kHullDuck = 3;

	constexpr float kAirmoveApproachLead = 32.f;
	constexpr float kAirmoveLaunchSlack = 4.f;
	constexpr float kAirmoveMinAirTime = 0.1f;
	constexpr float kAirmoveMaxAirTime = 6.f;

	Vector YawDir(float yawDeg)
	{
		float r = yawDeg * (kPiF / 180.f);
		return Vector(cosf(r), sinf(r), 0.f);
	}

	PmSim::PmTraceFn EngineTraceFn(edict_t *ignore, int hull)
	{
		return [ignore, hull](const Vector &start, const Vector &end)
		{
			TraceResult tr;
			g_engfuncs.pfnTraceHull(start, end, ignore_monsters, hull, ignore, &tr);

			PmSim::PmTrace out;
			out.endpos = tr.vecEndPos;
			out.planeNormal = tr.vecPlaneNormal;
			out.fraction = tr.flFraction;
			out.allsolid = tr.fAllSolid != 0;
			out.startsolid = tr.fStartSolid != 0;
			return out;
		};
	}

	float BotMaxSpeed(edict_t *pEdict)
	{
		float maxspeed = pEdict->v.maxspeed;
		if (maxspeed <= 1.f)
			maxspeed = g_engfuncs.pfnCVarGetFloat("sv_maxspeed");
		if (maxspeed <= 1.f)
			maxspeed = kMoveSpeed;
		return maxspeed;
	}

	PmSim::PmPhysics BotPhysics(edict_t *pEdict)
	{
		PmSim::PmPhysics physics;

		float maxspeed = BotMaxSpeed(pEdict);

		float gravity = g_engfuncs.pfnCVarGetFloat("sv_gravity");
		if (gravity <= 1.f)
			gravity = 800.f;
		if (pEdict->v.gravity > 0.f)
			gravity *= pEdict->v.gravity;

		float airAccel = g_engfuncs.pfnCVarGetFloat("sv_airaccelerate");
		if (airAccel <= 0.f)
			airAccel = 10.f;

		float accel = g_engfuncs.pfnCVarGetFloat("sv_accelerate");
		if (accel <= 0.f)
			accel = 10.f;

		float groundFriction = g_engfuncs.pfnCVarGetFloat("sv_friction");
		if (groundFriction <= 0.f)
			groundFriction = 4.f;

		float stopSpeed = g_engfuncs.pfnCVarGetFloat("sv_stopspeed");
		if (stopSpeed <= 0.f)
			stopSpeed = 100.f;

		float edgeFriction = g_engfuncs.pfnCVarGetFloat("sv_edgefriction");
		if (edgeFriction <= 0.f)
			edgeFriction = 2.f;

		float frameTime = gpGlobals->frametime;
		if (frameTime < 1.f / 128.f || frameTime > 1.f / 20.f)
			frameTime = 1.f / 60.f;

		physics.maxSpeed = maxspeed;
		physics.gravity = gravity;
		physics.airAccelerate = airAccel;
		physics.accelerate = accel;
		physics.groundFriction = groundFriction;
		physics.stopSpeed = stopSpeed;
		physics.edgeFriction = edgeFriction;
		physics.friction = 1.f;
		physics.frameTime = frameTime;
		return physics;
	}

	NavPlan::PlanTraceContext MakePlanTraces(edict_t *pEdict)
	{
		NavPlan::PlanTraceContext traces;
		traces.standTrace = EngineTraceFn(pEdict, kHullHuman);
		traces.duckTrace = EngineTraceFn(pEdict, kHullDuck);
		traces.physics = BotPhysics(pEdict);
		return traces;
	}

	constexpr float kTeleportJumpThreshold = 150.f;
	constexpr float kRepathInterval = 10.f;
	constexpr float kEtaSafetyMargin = 1.6f;
	constexpr float kProgressEpsilon = 8.f;
	constexpr float kMoveButtonDeadzone = 0.01f;

	void RunIdleMove(CBot &bot, const Vector &angles)
	{
		bot.moveForward = 0.f;
		bot.moveSide = 0.f;
		g_engfuncs.pfnRunPlayerMove(bot.pEdict, angles, 0.f, 0.f, 0.f, 0, 0, Bot_FrameMsec());
	}

	void RunAirmoveDrive(CBot &bot, const Vector &moveDir, float drive, float aimYaw, bool crouch, bool jump)
	{
		edict_t *pEdict = bot.pEdict;

		bot.desiredAngles = Vector(0.f, aimYaw, 0.f);
		Vector angles = Bot_UpdateFacing(bot);

		Vector wishDir = moveDir;
		if ((pEdict->v.flags & FL_ONGROUND) != 0)
		{
			NavFollow::Drive solved =
				NavFollow::SolveDrive(moveDir * drive, pEdict->v.velocity, BotPhysics(pEdict));
			if (solved.wishSpeed > 0.f)
			{
				wishDir = solved.wishDir;
				drive = solved.wishSpeed;
			}
		}

		if ((pEdict->v.flags & FL_DUCKING) != 0)
			drive /= PmSim::kDuckingMultiplier;
		float maxSpeed = BotMaxSpeed(pEdict);
		if (drive > maxSpeed)
			drive = maxSpeed;

		float fmove = 0.f, smove = 0.f;
		Bot_DecomposeMove(angles.y + pEdict->v.punchangle.y, wishDir, drive, fmove, smove);
		bot.moveForward = fmove;
		bot.moveSide = smove;

		unsigned short buttons = 0;
		if (fmove > 1.f) buttons |= IN_FORWARD;
		else if (fmove < -1.f) buttons |= IN_BACK;
		if (smove > 1.f) buttons |= IN_MOVERIGHT;
		else if (smove < -1.f) buttons |= IN_MOVELEFT;
		if (crouch) buttons |= IN_DUCK;
		if (jump) buttons |= IN_JUMP;

		g_engfuncs.pfnRunPlayerMove(pEdict, angles, fmove, smove, 0.f, buttons, 0, Bot_FrameMsec());
	}

	void RunAirmoveApproach(CBot &bot, const Nav::Waypoint &wp)
	{
		edict_t *pEdict = bot.pEdict;
		const Vector &origin = pEdict->v.origin;
		const Vector &vel = pEdict->v.velocity;

		Vector launchDir = YawDir(wp.launchYaw);
		Vector toLaunch(wp.point.x - origin.x, wp.point.y - origin.y, 0.f);
		float remaining = toLaunch.x * launchDir.x + toLaunch.y * launchDir.y;

		float speed = sqrtf(vel.x * vel.x + vel.y * vel.y);
		bool onGround = (pEdict->v.flags & FL_ONGROUND) != 0;

		float drive = (wp.launchSpeed > 1.f) ? wp.launchSpeed : kFullDrive;

		bool atLaunchPlane = remaining <= speed * gpGlobals->frametime + kAirmoveLaunchSlack;

		bool launching = onGround && atLaunchPlane;
		if (!wp.requiresJump && !onGround && remaining <= kAirmoveApproachLead)
			launching = true;

		if (launching)
		{
			bot.airmoveActive = true;
			bot.airmoveYaw = wp.launchYaw;
			bot.airmoveSpeed = drive;
			bot.airmoveCrouch = wp.requiresCrouch;
			bot.airmoveStartTime = gpGlobals->time;
			bot.airmoveTargetArea = wp.targetAreaId;
			bot.airmoveConnIndex = wp.connIndex;
			++bot.waypointIndex;

			Vector offTrack(toLaunch.x - launchDir.x * remaining,
				toLaunch.y - launchDir.y * remaining, 0.f);

			char msg[256];
			snprintf(msg, sizeof(msg),
				"CrossBot: %s airmove launch at (%.1f,%.1f,%.1f) plan yaw %.0f speed %.0f "
				"jump %d crouch %d | actual yaw %.0f speed %.0f lateral %.1f\n",
				bot.name, origin.x, origin.y, origin.z, wp.launchYaw, drive, wp.requiresJump ? 1 : 0,
				wp.requiresCrouch ? 1 : 0,
				(speed > kAimVelocitySpeed) ? atan2f(vel.y, vel.x) * (180.f / kPiF) : wp.launchYaw, speed,
				sqrtf(offTrack.x * offTrack.x + offTrack.y * offTrack.y));
			SERVER_PRINT(msg);

			RunAirmoveDrive(bot, launchDir, drive, wp.launchYaw, wp.requiresCrouch,
				wp.requiresJump && onGround);
			return;
		}

		Vector aim = wp.point + launchDir * kAirmoveApproachLead;
		Vector moveDir = Norm2D(aim - origin);
		if (moveDir.x == 0.f && moveDir.y == 0.f)
			moveDir = launchDir;

		RunAirmoveDrive(bot, moveDir, drive, wp.launchYaw, wp.requiresCrouch && !wp.requiresJump, false);
	}

	bool RunAirmoveFlight(CBot &bot, const Nav::NavMesh &mesh)
	{
		edict_t *pEdict = bot.pEdict;
		float airTime = gpGlobals->time - bot.airmoveStartTime;
		bool onGround = (pEdict->v.flags & FL_ONGROUND) != 0;

		if ((onGround && airTime >= kAirmoveMinAirTime) || airTime >= kAirmoveMaxAirTime)
		{
			bot.airmoveActive = false;

			const Nav::Area *landedArea = Nav::FindAreaForPoint(mesh, pEdict->v.origin);
			int landedId = landedArea ? static_cast<int>(landedArea->id) : -1;

			bool failed = (bot.airmoveTargetArea >= 0 && landedId != bot.airmoveTargetArea);

			if (bot.airmoveConnIndex >= 0)
			{
				uint32_t connIndex = static_cast<uint32_t>(bot.airmoveConnIndex);
				if (failed)
					Nav::PenalizeConnection(connIndex);
				else
					Nav::RewardConnection(connIndex);
			}

			char msg[288];
			snprintf(msg, sizeof(msg),
				"CrossBot: %s airmove %s after %.2fs at (%.1f,%.1f,%.1f) area %d (wanted %d) "
				"conn %d penalty %.1f\n", bot.name,
				onGround ? "landed" : "timed out", airTime, pEdict->v.origin.x, pEdict->v.origin.y,
				pEdict->v.origin.z, landedId, bot.airmoveTargetArea, bot.airmoveConnIndex,
				(bot.airmoveConnIndex >= 0)
					? Nav::ConnectionPenalty(static_cast<uint32_t>(bot.airmoveConnIndex))
					: 0.f);
			SERVER_PRINT(msg);

			if (failed)
				bot.nextRepathTime = gpGlobals->time;

			return false;
		}

		RunAirmoveDrive(bot, YawDir(bot.airmoveYaw), bot.airmoveSpeed, bot.airmoveYaw, bot.airmoveCrouch,
			false);
		return true;
	}

	void ResetPath(CBot &bot)
	{
		bot.waypoints.clear();
		bot.waypointIndex = 0;
		bot.followProfile = NavPlan::PlanProfile();
		bot.trackState = NavFollow::TrackState();
		bot.airmoveActive = false;
		bot.airmoveTargetArea = -1;
		bot.airmoveConnIndex = -1;
		bot.avoid = BotAvoid::State();
		bot.plannedAreaPath.clear();
		bot.plannedConnPath.clear();
		bot.visitedAreaPath.clear();
		bot.pathStartTime = 0.f;
		bot.estimatedArrivalTime = 0.f;
		bot.lastProgressTime = 0.f;
		bot.lastProgressS = 0.f;
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

	void SpliceDetour(CBot &bot, edict_t *pEdict, const BotAvoid::Decision &avoid,
		const PmSim::PmPhysics &physics)
	{
		size_t from = std::min(avoid.detourReplaceUntil, bot.waypoints.size());

		std::vector<Nav::Waypoint> rebuilt;
		rebuilt.reserve(1 + avoid.detourPoints.size() + (bot.waypoints.size() - from));

		Nav::Waypoint here;
		here.point = pEdict->v.origin;
		rebuilt.push_back(here);

		for (const Vector &p : avoid.detourPoints)
		{
			Nav::Waypoint wp;
			wp.point = p;
			wp.clearance = BotAvoid::kDetourClearance;
			rebuilt.push_back(wp);
		}
		for (size_t i = from; i < bot.waypoints.size(); ++i)
			rebuilt.push_back(bot.waypoints[i]);

		Vector rejoin = (from < bot.waypoints.size()) ? bot.waypoints[from].point
			: avoid.detourPoints.back();
		float detourLen = 0.f;
		for (size_t i = 1; i < rebuilt.size() && i <= avoid.detourPoints.size() + 1; ++i)
		{
			const Vector &a = rebuilt[i - 1].point;
			const Vector &b = rebuilt[i].point;
			detourLen += sqrtf((b.x - a.x) * (b.x - a.x) + (b.y - a.y) * (b.y - a.y));
		}
		Vector straight = rejoin - pEdict->v.origin;
		float extra = detourLen - sqrtf(straight.x * straight.x + straight.y * straight.y);

		bot.waypoints = std::move(rebuilt);
		bot.waypointIndex = 0;
		bot.airmoveActive = false;

		NavPlan::PlanTraceContext traces = MakePlanTraces(pEdict);
		bot.followProfile = NavPlan::BuildPathProfile(bot.waypoints, traces.physics, &traces);
		bot.trackState = NavFollow::TrackState();
		bot.lastProgressS = 0.f;
		bot.lastProgressTime = gpGlobals->time;

		bot.avoid.detourEnd = 1 + avoid.detourPoints.size();

		if (extra > 0.f && physics.maxSpeed > 1.f && bot.estimatedArrivalTime > 0.f)
			bot.estimatedArrivalTime += extra / physics.maxSpeed;
	}

	void PenalizeCurrentConnection(CBot &bot)
	{
		if (bot.currentAreaId < 0)
			return;

		uint32_t cur = static_cast<uint32_t>(bot.currentAreaId);
		size_t areaSteps = bot.plannedAreaPath.empty() ? 0 : bot.plannedAreaPath.size() - 1;
		size_t count = std::min(bot.plannedConnPath.size(), areaSteps);
		for (size_t i = 0; i < count; ++i)
		{
			if (bot.plannedAreaPath[i] != cur)
				continue;

			Nav::PenalizeConnection(bot.plannedConnPath[i]);

			char msg[192];
			snprintf(msg, sizeof(msg),
				"CrossBot: %s penalizing conn %u (area %u->%u) after a blocked detour, penalty %.1f\n",
				bot.name, bot.plannedConnPath[i], cur, bot.plannedAreaPath[i + 1],
				Nav::ConnectionPenalty(bot.plannedConnPath[i]));
			SERVER_PRINT(msg);
			return;
		}
	}

	void RewardEnteredConnection(CBot &bot, uint32_t enteredId)
	{
		if (bot.visitedAreaPath.empty())
			return;

		uint32_t prev = bot.visitedAreaPath.back();
		size_t areaSteps = bot.plannedAreaPath.empty() ? 0 : bot.plannedAreaPath.size() - 1;
		size_t count = std::min(bot.plannedConnPath.size(), areaSteps);
		for (size_t i = 0; i < count; ++i)
		{
			if (bot.plannedAreaPath[i] != prev || bot.plannedAreaPath[i + 1] != enteredId)
				continue;
			if (Nav::ConnectionPenalty(bot.plannedConnPath[i]) > 0.f)
				Nav::RewardConnection(bot.plannedConnPath[i]);
			return;
		}
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

		NavPlan::PlanTraceContext traces = MakePlanTraces(pEdict);
		NavPlan::PlanStats planStats;
		std::chrono::steady_clock::time_point planBegin = std::chrono::steady_clock::now();
		Nav::PlannedPath path = Nav::PlanPath(mesh, startArea->id, goalArea->id, pEdict->v.origin,
			clampedGoal, &traces, &planStats);
		float planMs = std::chrono::duration<float, std::milli>(
			std::chrono::steady_clock::now() - planBegin).count();
		if (!path.found)
			return false;

		bot.waypoints = std::move(path.waypoints);
		bot.waypointIndex = 0;
		bot.airmoveActive = false;
		bot.plannedAreaPath = path.areaPath;
		bot.plannedConnPath = path.connPath;
		bot.avoid.detourEnd = 0;
		bot.avoid.detourHit = 0;
		bot.avoid.detourExpire = 0.f;

		bot.followProfile = NavPlan::BuildPathProfile(bot.waypoints, traces.physics, &traces);
		bot.trackState = NavFollow::TrackState();
		bot.lastProgressS = 0.f;

		size_t penalizedOnRoute = 0;
		for (uint32_t connIndex : path.connPath)
			if (Nav::ConnectionPenalty(connIndex) > 0.f)
				++penalizedOnRoute;

		if (freshAttempt)
		{
			bot.pathStartTime = gpGlobals->time;
			bot.estimatedArrivalTime = gpGlobals->time + bot.followProfile.totalTime * kEtaSafetyMargin;
			bot.visitedAreaPath.clear();
			bot.visitedAreaPath.push_back(startArea->id);

			char wpMsg[512];
			snprintf(wpMsg, sizeof(wpMsg),
				"CrossBot: Repath start=(%.1f,%.1f,%.1f) goal=(%.1f,%.1f,%.1f) waypoints=%zu "
				"airmoves=%zu simulated/%zu respeeded/%zu kept/%zu rejected | search %zu areas/%zu sims "
				"in %.1fms | profile segs=%zu len=%.0f "
				"eta=%.1fs arcs=%zu slowed=%zu shrunk=%zu | penalized=%zu\n",
				pEdict->v.origin.x, pEdict->v.origin.y, pEdict->v.origin.z,
				clampedGoal.x, clampedGoal.y, clampedGoal.z, bot.waypoints.size(),
				planStats.airmovesSimulated, planStats.airmovesRespeeded, planStats.airmovesRespeedKept,
				path.airmovesRejected, path.expansions, path.airmovesTried, planMs,
				bot.followProfile.segs.size(), bot.followProfile.totalLength,
				bot.followProfile.totalTime, bot.followProfile.cornersArced,
				bot.followProfile.cornersSlowed, bot.followProfile.cornersShrunk, penalizedOnRoute);
			SERVER_PRINT(wpMsg);
			for (size_t i = 0; i < bot.waypoints.size(); ++i)
			{
				const Nav::Waypoint &wp = bot.waypoints[i];
				if (wp.airmoveLaunch)
				{
					snprintf(wpMsg, sizeof(wpMsg),
						"CrossBot: wp[%zu]=(%.1f,%.1f,%.1f) AIRMOVE jump=%d crouch=%d yaw=%.0f speed=%.0f "
						"sim=%d targArea=%d\n",
						i, wp.point.x, wp.point.y, wp.point.z, wp.requiresJump, wp.requiresCrouch,
						wp.launchYaw, wp.launchSpeed, wp.launchSimulated, wp.targetAreaId);
				}
				else
				{
					snprintf(wpMsg, sizeof(wpMsg),
						"CrossBot: wp[%zu]=(%.1f,%.1f,%.1f) jump=%d crouch=%d targArea=%d\n",
						i, wp.point.x, wp.point.y, wp.point.z,
						wp.requiresJump, wp.requiresCrouch, wp.targetAreaId);
				}
				SERVER_PRINT(wpMsg);
			}
		}
		else
		{
			char wpMsg[256];
			snprintf(wpMsg, sizeof(wpMsg),
				"CrossBot: Replan from=(%.1f,%.1f,%.1f) waypoints=%zu areas=%zu eta=%.1fs | penalized=%zu\n",
				pEdict->v.origin.x, pEdict->v.origin.y, pEdict->v.origin.z, bot.waypoints.size(),
				path.areaPath.size(), bot.followProfile.totalTime, penalizedOnRoute);
			SERVER_PRINT(wpMsg);
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

	if (bot.trackState.progressS > bot.lastProgressS + kProgressEpsilon)
	{
		bot.lastProgressS = bot.trackState.progressS;
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
				bot.name, forcedRepath ? "forced" : "stalled", pEdict->v.origin.x, pEdict->v.origin.y,
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
		bot.lastProgressS = 0.f;
	}

	if (curArea && (bot.visitedAreaPath.empty() || bot.visitedAreaPath.back() != curArea->id))
	{
		RewardEnteredConnection(bot, curArea->id);
		bot.visitedAreaPath.push_back(curArea->id);
	}

	if (bot.traceLog.size() < 100000)
		bot.traceLog.push_back(pEdict->v.origin);

	if (bot.airmoveActive)
	{
		if (RunAirmoveFlight(bot, *mesh))
			return;
		bot.trackState.started = false;
	}

	PmSim::PmPhysics physics = BotPhysics(pEdict);

	NavFollow::Decision follow = NavFollow::Track(bot.followProfile, pEdict->v.origin,
		pEdict->v.velocity, physics, bot.trackState);

	if (follow.wpIndex < bot.waypoints.size())
		bot.waypointIndex = follow.wpIndex;

	if (follow.done)
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

	if (follow.needRepath && bot.nextRepathTime <= 0.f)
	{
		char msg[192];
		snprintf(msg, sizeof(msg),
			"CrossBot: %s strayed %.1f from the plan at (%.1f,%.1f,%.1f), repathing\n", bot.name,
			follow.lateral, pEdict->v.origin.x, pEdict->v.origin.y, pEdict->v.origin.z);
		SERVER_PRINT(msg);
		bot.nextRepathTime = gpGlobals->time;
	}

	if (bot.waypointIndex >= bot.waypoints.size())
	{
		Vector angles = Bot_UpdateFacing(bot);
		RunIdleMove(bot, angles);
		return;
	}

	Nav::Waypoint *wp = &bot.waypoints[bot.waypointIndex];

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

	if (wp->airmoveLaunch)
	{
		RunAirmoveApproach(bot, *wp);
		return;
	}

	Vector moveDir(follow.wishDir.x, follow.wishDir.y, 0.f);

	size_t aimIndex = bot.waypointIndex + 1;
	if (aimIndex < bot.waypoints.size())
	{
		Vector toAim = bot.waypoints[aimIndex].point - pEdict->v.origin;
		if (toAim.x * toAim.x + toAim.y * toAim.y > 1.f)
			bot.desiredAngles = Vector(0.f, atan2f(toAim.y, toAim.x) * (180.f / kPiF), 0.f);
	}
	else
	{
		const Vector &vel = pEdict->v.velocity;
		if (sqrtf(vel.x * vel.x + vel.y * vel.y) > kAimVelocitySpeed)
			bot.desiredAngles = Vector(0.f, atan2f(vel.y, vel.x) * (180.f / kPiF), 0.f);
	}

	Vector angles = Bot_UpdateFacing(bot);

	BotAvoid::Decision avoid;
	if (BotAvoid::Enabled())
	{
		Vector pathDir = Norm2D(follow.aimPoint - pEdict->v.origin);
		if (pathDir.x == 0.f && pathDir.y == 0.f)
			pathDir = moveDir;
		avoid = BotAvoid::Update(bot, *mesh, pathDir, physics);
	}

	if (!avoid.detourPoints.empty())
	{
		SpliceDetour(bot, pEdict, avoid, physics);
		RunIdleMove(bot, angles);
		return;
	}

	if (avoid.searchFailed && !avoid.hitRecedes)
	{
		PenalizeCurrentConnection(bot);
		bot.nextRepathTime = gpGlobals->time;
	}

	bool planCrouch = wp->requiresCrouch
		|| (curArea && (curArea->areaFlags & crossbot_nav::NAV_AREA_CROUCH));
	bool crouch = planCrouch || avoid.duck;
	if (avoid.suppressPlanCrouch && !planCrouch)
		crouch = false;

	float drive = follow.wishSpeed;
	if ((pEdict->v.flags & FL_DUCKING) != 0)
		drive /= PmSim::kDuckingMultiplier;
	if (drive > physics.maxSpeed)
		drive = physics.maxSpeed;
	if (avoid.slowApproach)
		drive = std::min(drive, physics.maxSpeed * PmSim::kDuckingMultiplier);

	float fmove = 0.f, smove = 0.f;
	Bot_DecomposeMove(angles.y + pEdict->v.punchangle.y, moveDir, drive, fmove, smove);
	bot.moveForward = fmove;
	bot.moveSide = smove;

	unsigned short buttons = 0;
	if (fmove > kMoveButtonDeadzone) buttons |= IN_FORWARD;
	else if (fmove < -kMoveButtonDeadzone) buttons |= IN_BACK;
	if (smove > kMoveButtonDeadzone) buttons |= IN_MOVERIGHT;
	else if (smove < -kMoveButtonDeadzone) buttons |= IN_MOVELEFT;

	if (crouch)
		buttons |= IN_DUCK;
	if (avoid.jump)
		buttons |= IN_JUMP;

	g_engfuncs.pfnRunPlayerMove(pEdict, angles, fmove, smove, 0.f, buttons, 0, Bot_FrameMsec());
}
