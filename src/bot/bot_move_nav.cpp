#include <extdll.h>
#include <meta_api.h>
#include <in_buttons.h>
#include <algorithm>
#include <chrono>
#include <cstdarg>

#include "bot_move_nav.h"
#include "bot_move.h"
#include "bot_aim.h"
#include "bot_fire.h"
#include "bot_sense.h"
#include "bot_avoid.h"
#include "bot_break.h"
#include "bot_combat.h"
#include "bot_deploy.h"
#include "bot_fun.h"
#include "bot_goal.h"
#include "bot_items.h"
#include "bot_opportunist.h"
#include "bot_log.h"
#include "bot_perf.h"
#include "../game/game_profile.h"
#include "../nav/nav.h"
#include "../nav/nav_math.h"
#include "../nav/nav_path.h"
#include "../nav/nav_query.h"
#include "../nav/nav_water.h"
#include "../nav/path_follower.h"

namespace
{
	using NavMath::kPiF;
	using NavMath::Norm2D;

	int AreaIdOf(const Nav::Area *area)
	{
		return area ? static_cast<int>(area->id) : -1;
	}

	constexpr float kFullDrive = 1000.f;
	constexpr float kAimVelocitySpeed = 20.f;
	constexpr int kHullPoint = 0;
	constexpr int kHullHuman = 1;
	constexpr int kHullDuck = 3;

	constexpr float kAirmoveApproachLead = 32.f;
	constexpr float kAirmoveMinAirTime = 0.1f;
	constexpr float kAirmoveMaxAirTime = 6.f;

	NavPlan::AirmoveCache g_airmoveCache;
	constexpr float kAirmoveTimeFactor = 2.f;
	constexpr float kAirmoveTimeMargin = 1.f;
	constexpr float kAirmoveOverrunGrace = 0.5f;
	constexpr float kAirmoveFallingZ = -60.f;
	constexpr float kAirmoveLandingTolerance = 48.f;
	constexpr int kWaterlevelSwimming = 2;

	constexpr float kStandHullMinsZ = -36.f;
	constexpr float kDuckHullMinsZ = -18.f;

	constexpr float kLadderCommitRadius = 48.f;
	constexpr float kLadderMountRadius = 32.f;
	constexpr float kLadderMountLead = 32.f;
	constexpr float kLadderMountApproach = 8.f;
	constexpr float kLadderEntryTimeout = 3.f;
	constexpr float kLadderEntryDrift = 64.f;
	constexpr float kLadderReachDwell = 0.3f;
	constexpr float kLadderMaxTime = 25.f;
	constexpr float kLadderDownPitch = 89.f;
	constexpr float kLadderUpPitch = -89.f;
	constexpr float kLadderAimTolerance = 6.f;
	constexpr float kLadderDismountSlack = 2.f;
	constexpr float kLadderPushOffDot = -0.7f;
	constexpr float kLadderMinAirTime = 0.1f;
	constexpr float kLadderStallDist = 8.f;
	constexpr float kLadderStallTime = 0.5f;
	constexpr int kLadderStallEscapes = 2;

	constexpr float kElevatorEngageRadius = 48.f;
	constexpr float kElevatorBoardRadius = 12.f;
	constexpr float kElevatorHoldRadius = 6.f;
	constexpr float kElevatorArriveSlack = 8.f;
	constexpr float kElevatorTimeMargin = 2.f;
	constexpr float kElevatorTimeGrace = 3.f;

	bool AtFrictionEdge(edict_t *pEdict)
	{
		if ((pEdict->v.flags & FL_ONGROUND) == 0)
			return false;

		const Vector &vel = pEdict->v.velocity;
		float speed = vel.Length();
		if (speed < 0.1f)
			return false;

		bool ducking = (pEdict->v.flags & FL_DUCKING) != 0;
		const Vector &origin = pEdict->v.origin;
		Vector start(origin.x + vel.x / speed * PmSim::kEdgeProbeAhead,
			origin.y + vel.y / speed * PmSim::kEdgeProbeAhead,
			origin.z + (ducking ? kDuckHullMinsZ : kStandHullMinsZ));
		Vector stop(start.x, start.y, start.z - PmSim::kEdgeProbeDrop);

		TraceResult tr;
		g_engfuncs.pfnTraceHull(start, stop, dont_ignore_monsters, ducking ? kHullDuck : kHullHuman,
			pEdict, &tr);
		return tr.flFraction == 1.f;
	}

	NavPlan::PlanTraceContext MakePlanTraces(edict_t *pEdict)
	{
		BOTPERF_SCOPE(ZONE_PLANTRACES);
		NavPlan::PlanTraceContext traces;
		traces.standTrace = Bot_EngineTraceFn(pEdict, kHullHuman);
		traces.duckTrace = Bot_EngineTraceFn(pEdict, kHullDuck);
		{
			BOTPERF_SCOPE(ZONE_PUSHFIELDS);
			traces.standPush = Bot_PushFields(kHullHuman);
			traces.duckPush = Bot_PushFields(kHullDuck);
		}
		traces.physics = Bot_Physics(pEdict);
		traces.gravityMultiplier = Bot_GravityMultiplier(pEdict);
		if (const Nav::NavMesh *mesh = Nav::Get())
			traces.waterSurfaces = &Nav::WaterSurfaces(*mesh);
		g_airmoveCache.SetVerify(g_engfuncs.pfnCVarGetFloat("crossbot_airmove_verify") > 0.f);
		traces.airmoveCache = &g_airmoveCache;
		return traces;
	}

	constexpr float kActivationEngageRadius = 40.f;
	constexpr float kActivationStandRadius = 8.f;
	constexpr float kActivationUseRange = 56.f;
	constexpr float kActivationAimTolerance = 8.f;
	constexpr float kActivationMinHold = 0.5f;
	constexpr float kActivationMaxHold = 3.f;
	constexpr float kActivationTimeout = 6.f;
	constexpr float kActivationMemory = 15.f;
	constexpr float kChargerUseRange = 64.f;
	constexpr float kChargerNoGainGrace = 0.5f;
	constexpr float kChargerTimeout = 15.f;
	// HurtTouch only re-applies every 0.5s (pev->dmgtime), so a healing pool looks stalled for
	// half a second at a time even while it is working.
	constexpr float kHealNoGainGrace = 1.2f;
	// Split in two: a slow pool can legitimately take half a minute to fill the bot, but if the
	// bot never gets in at all there is nothing to wait for.
	constexpr float kHealApproachTimeout = 5.f;
	constexpr float kHealTimeout = 30.f;
	constexpr float kActivationSameRadius = 8.f;
	constexpr float kTriggerTouchRadius = 20.f;
	constexpr float kTriggerTouchHeight = 40.f;

	constexpr float kTeleportJumpThreshold = 150.f;

	constexpr float kMineSafeRadius = 375.f;
	constexpr float kRepathInterval = 2.f;
	constexpr float kEtaSafetyMargin = 1.6f;
	constexpr float kProgressEpsilon = 8.f;
	constexpr float kMoveButtonDeadzone = 0.01f;

	void RunIdleMove(CBot &bot, const Vector &angles)
	{
		bot.moveForward = 0.f;
		bot.moveSide = 0.f;
		Bot_RunPlayerMove(bot, angles, 0.f, 0.f, 0);
	}

	bool OnLadder(edict_t *pEdict)
	{
		return pEdict->v.movetype == MOVETYPE_FLY;
	}

	struct LadderPlane
	{
		bool valid = false;
		Vector normal;
		Vector perp;
		const char *why = "not-probed";
		float fraction = 0.f;
	};

#ifdef CROSSBOT_LADDER_DEBUG
	void LadderDebugf(const CBot &bot, const char *phase, const char *fmt, ...)
	{
		char body[768];
		va_list args;
		va_start(args, fmt);
		vsnprintf(body, sizeof(body), fmt, args);
		va_end(args);

		char line[900];
		snprintf(line, sizeof(line), "CrossBot: LDBG %s t=%.2f %-8s %s\n", bot.name, gpGlobals->time,
			phase, body);
		SERVER_PRINT(line);
	}

	const char *LadderButtonNames(unsigned short buttons)
	{
		static char text[64];
		text[0] = 0;
		struct { unsigned short bit; const char *name; } kNames[] = {
			{IN_FORWARD, "FWD"}, {IN_BACK, "BACK"}, {IN_MOVELEFT, "LEFT"}, {IN_MOVERIGHT, "RIGHT"},
			{IN_DUCK, "DUCK"}, {IN_JUMP, "JUMP"}, {IN_USE, "USE"}
		};
		for (const auto &entry : kNames)
		{
			if ((buttons & entry.bit) == 0)
				continue;
			if (text[0])
				strncat(text, "|", sizeof(text) - strlen(text) - 1);
			strncat(text, entry.name, sizeof(text) - strlen(text) - 1);
		}
		if (!text[0])
			strncpy(text, "-", sizeof(text));
		return text;
	}

	#define LADDER_DBG(bot, phase, ...) LadderDebugf(bot, phase, __VA_ARGS__)
#else
	#define LADDER_DBG(bot, phase, ...) ((void)0)
#endif

	edict_t *FindLadderEntity(edict_t *pEdict)
	{
		const Vector &origin = pEdict->v.origin;
		float halfZ = ((pEdict->v.flags & FL_DUCKING) != 0) ? 18.f : 36.f;

		edict_t *best = nullptr;
		float bestDist = 0.f;
		edict_t *ent = nullptr;
		while (true)
		{
			ent = FIND_ENTITY_BY_STRING(ent, "classname", "func_ladder");
			if (!ent || g_engfuncs.pfnIndexOfEdict(ent) == 0)
				break;

			if (origin.x < ent->v.absmin.x - 16.f || origin.x > ent->v.absmax.x + 16.f ||
				origin.y < ent->v.absmin.y - 16.f || origin.y > ent->v.absmax.y + 16.f ||
				origin.z < ent->v.absmin.z - halfZ || origin.z > ent->v.absmax.z + halfZ)
				continue;

			Vector center = (ent->v.absmin + ent->v.absmax) * 0.5f;
			float dist = (center - origin).Length();
			if (!best || dist < bestDist)
			{
				best = ent;
				bestDist = dist;
			}
		}

		return best;
	}

	edict_t *NearestLadderEntity(const Vector &point)
	{
		edict_t *best = nullptr;
		float bestDist = 0.f;
		edict_t *ent = nullptr;
		while (true)
		{
			ent = FIND_ENTITY_BY_STRING(ent, "classname", "func_ladder");
			if (!ent || g_engfuncs.pfnIndexOfEdict(ent) == 0)
				break;

			float dist = ((ent->v.absmin + ent->v.absmax) * 0.5f - point).Length();
			if (!best || dist < bestDist)
			{
				best = ent;
				bestDist = dist;
			}
		}

		return best;
	}

	bool LadderOutOfReach(edict_t *pEdict, const Vector &mount)
	{
		edict_t *ent = NearestLadderEntity(mount);
		if (!ent)
			return false;

		float halfZ = ((pEdict->v.flags & FL_DUCKING) != 0) ? 18.f : 36.f;
		return pEdict->v.origin.z < ent->v.absmin.z - halfZ;
	}

	LadderPlane ProbeLadderPlane(edict_t *pEdict)
	{
		LadderPlane plane;

		edict_t *ent = FindLadderEntity(pEdict);
		if (!ent)
		{
			plane.why = "no-entity";
			return plane;
		}

		Vector center = (ent->v.absmin + ent->v.absmax) * 0.5f;

		TraceResult tr;
		g_engfuncs.pfnTraceModel(pEdict->v.origin, center, kHullPoint, ent, &tr);
		plane.fraction = tr.flFraction;
		plane.normal = tr.vecPlaneNormal;

		if (tr.fStartSolid != 0)
		{
			plane.why = "startsolid";
			return plane;
		}
		if (tr.fAllSolid != 0)
		{
			plane.why = "allsolid";
			return plane;
		}
		if (tr.flFraction == 1.f)
		{
			plane.why = "no-hit";
			return plane;
		}

		Vector perp(-tr.vecPlaneNormal.y, tr.vecPlaneNormal.x, 0.f);
		float perpLen = perp.Length();
		if (perpLen < 0.1f)
		{
			plane.why = "degenerate";
			return plane;
		}

		plane.perp = perp * (1.f / perpLen);
		plane.valid = true;
		plane.why = "ok";
		return plane;
	}

	float BotNavZ(edict_t *pEdict)
	{
		return pEdict->v.origin.z -
			(((pEdict->v.flags & FL_DUCKING) != 0) ? 0.f : NavPlan::kPlanStandHeight);
	}

	float AirmoveLandingMiss(const CBot &bot, edict_t *pEdict)
	{
		if (bot.waypointIndex >= bot.waypoints.size())
			return -1.f;

		return (pEdict->v.origin - bot.waypoints[bot.waypointIndex].point).Length();
	}

	bool AirmoveSettled(edict_t *pEdict)
	{
		if ((pEdict->v.flags & FL_ONGROUND) != 0)
			return true;
		if (pEdict->v.waterlevel >= kWaterlevelSwimming)
			return true;
		return OnLadder(pEdict);
	}

	const char *AirmoveEndReason(edict_t *pEdict)
	{
		if ((pEdict->v.flags & FL_ONGROUND) != 0)
			return "landed";
		if (pEdict->v.waterlevel >= kWaterlevelSwimming)
			return "splashed down";
		if (OnLadder(pEdict))
			return "caught a ladder";
		return "timed out";
	}

	float AirmovePlannedTime(const Nav::NavMesh &mesh, int connIndex)
	{
		if (connIndex < 0 || static_cast<size_t>(connIndex) >= mesh.conns.size())
			return 0.f;
		return mesh.conns[static_cast<size_t>(connIndex)].flightTime;
	}

	float AirmoveTimeLimit(float plannedTime)
	{
		if (plannedTime <= 0.f)
			return kAirmoveMaxAirTime;
		return std::min(plannedTime * kAirmoveTimeFactor + kAirmoveTimeMargin, kAirmoveMaxAirTime);
	}

	void RunAirmoveDrive(CBot &bot, const Vector &moveDir, float drive, float aimYaw, bool crouch, bool jump,
		bool allowLongJump = false, const Vector *aimAngles = nullptr, unsigned short extraButtons = 0)
	{
		edict_t *pEdict = bot.pEdict;

		bot.desiredAngles = aimAngles ? *aimAngles : Vector(0.f, aimYaw, 0.f);
		Vector angles = Bot_UpdateFacing(bot);

		Vector wishDir = moveDir;
		if ((pEdict->v.flags & FL_ONGROUND) != 0)
		{
			NavFollow::Drive solved =
				NavFollow::SolveDrive(moveDir * drive, pEdict->v.velocity, Bot_DrivePhysics(pEdict));
			if (solved.wishSpeed > 0.f)
			{
				wishDir = solved.wishDir;
				drive = solved.wishSpeed;
			}
		}

		if ((pEdict->v.flags & FL_DUCKING) != 0)
			drive /= PmSim::kDuckingMultiplier;
		float maxSpeed = Bot_MaxSpeed(pEdict);
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
		if (jump && !OnLadder(pEdict)) buttons |= IN_JUMP;
		buttons |= extraButtons;

		Bot_RunPlayerMove(bot, angles, fmove, smove, buttons, allowLongJump);
	}

	void RunAirmoveApproach(CBot &bot, const Nav::NavMesh &mesh, const Nav::Waypoint &wp)
	{
		edict_t *pEdict = bot.pEdict;
		const Vector &origin = pEdict->v.origin;
		const Vector &vel = pEdict->v.velocity;

		Vector launchDir = Bot_YawDir(wp.launchYaw);
		Vector toLaunch(wp.point.x - origin.x, wp.point.y - origin.y, 0.f);
		float remaining = toLaunch.x * launchDir.x + toLaunch.y * launchDir.y;

		float speed = sqrtf(vel.x * vel.x + vel.y * vel.y);
		bool onGround = (pEdict->v.flags & FL_ONGROUND) != 0;

		float drive = (wp.launchSpeed > 1.f) ? wp.launchSpeed : kFullDrive;

		bool atLaunchPlane = remaining <= NavPlan::AirmoveLaunchLead(speed, Bot_FrameTime());
		bool upToSpeed = (wp.launchSpeed <= 1.f) ||
			(speed >= wp.launchSpeed * NavPlan::kAirmoveLaunchSpeedTolerance);

		bool launching = onGround && atLaunchPlane && upToSpeed;
		if (!wp.requiresJump && !onGround && remaining <= kAirmoveApproachLead)
			launching = true;

		if (onGround && atLaunchPlane && !upToSpeed)
		{
			if (wp.connIndex >= 0)
				Nav::PenalizeConnection(static_cast<uint32_t>(wp.connIndex));

			BOT_VLOG("CrossBot: %s airmove aborted at (%.1f,%.1f,%.1f) plan speed %.0f actual %.0f "
				"(%.0f%% of design) conn %d, repathing\n",
				bot.name, origin.x, origin.y, origin.z, drive, speed,
				(drive > 1.f) ? (100.f * speed / drive) : 0.f, wp.connIndex);

			bot.nextRepathTime = gpGlobals->time;
			RunAirmoveDrive(bot, launchDir, 0.f, wp.launchYaw, false, false);
			return;
		}

		if (launching)
		{
			bot.airmoveActive = true;
			bot.airmoveYaw = wp.launchYaw;
			bot.airmoveSpeed = drive;
			bot.airmoveCrouch = wp.requiresCrouch;
			bot.airmoveStartTime = gpGlobals->time;
			bot.airmoveTargetArea = wp.targetAreaId;
			bot.airmoveConnIndex = wp.connIndex;
			bot.airmovePlannedTime = AirmovePlannedTime(mesh, wp.connIndex);
			bot.airmoveMaxTime = AirmoveTimeLimit(bot.airmovePlannedTime);
			bot.airmoveOverrunLogged = false;
			++bot.waypointIndex;

			Vector offTrack(toLaunch.x - launchDir.x * remaining,
				toLaunch.y - launchDir.y * remaining, 0.f);

			BOT_VLOG("CrossBot: %s airmove launch at (%.1f,%.1f,%.1f) plan yaw %.0f speed %.0f "
				"jump %d crouch %d | actual yaw %.0f speed %.0f (%.0f%%) lateral %.1f early %.1f\n",
				bot.name, origin.x, origin.y, origin.z, wp.launchYaw, drive, wp.requiresJump ? 1 : 0,
				wp.requiresCrouch ? 1 : 0,
				(speed > kAimVelocitySpeed) ? atan2f(vel.y, vel.x) * (180.f / kPiF) : wp.launchYaw, speed,
				(drive > 1.f) ? (100.f * speed / drive) : 0.f,
				sqrtf(offTrack.x * offTrack.x + offTrack.y * offTrack.y), remaining);

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

	edict_t *BreakablePlugging(const Nav::Area &area)
	{
		for (int i = 1; i < gpGlobals->maxEntities; ++i)
		{
			edict_t *ent = g_engfuncs.pfnPEntityOfEntIndex(i);
			if (!BotBreak::IsTarget(ent))
				continue;
			if (ent->v.absmax.x < area.mins.x || ent->v.absmin.x > area.maxs.x)
				continue;
			if (ent->v.absmax.y < area.mins.y || ent->v.absmin.y > area.maxs.y)
				continue;
			if (ent->v.absmax.z < area.mins.z || ent->v.absmin.z > area.maxs.z)
				continue;
			return ent;
		}

		return NULL;
	}

	edict_t *AirmovePlug(CBot &bot, const Nav::NavMesh &mesh)
	{
		if (!BotBreak::Enabled() || bot.airmoveTargetArea < 0)
			return NULL;

		const Nav::Area *area = Nav::FindAreaById(mesh, static_cast<uint32_t>(bot.airmoveTargetArea));
		if (!area || (area->areaFlags & crossbot_nav::NAV_AREA_BREAKABLE) == 0)
			return NULL;

		return BreakablePlugging(*area);
	}

	bool BreakBlockedAirmove(CBot &bot, const Nav::NavMesh &mesh)
	{
		edict_t *plug = AirmovePlug(bot, mesh);
		if (!plug || !BotBreak::Engage(bot, plug))
			return false;

		BOT_VLOG("CrossBot: %s cannot land in area %d, it is plugged by %s\n", bot.name,
			bot.airmoveTargetArea, CBotClassOf(plug));

		return true;
	}

	bool RunAirmoveFlight(CBot &bot, const Nav::NavMesh &mesh)
	{
		edict_t *pEdict = bot.pEdict;
		float airTime = gpGlobals->time - bot.airmoveStartTime;
		bool settled = AirmoveSettled(pEdict);

		bot.lastProgressTime = gpGlobals->time;

		bool falling = pEdict->v.velocity.z < kAirmoveFallingZ;
		bool overdue = airTime >= bot.airmoveMaxTime && !falling;

		if ((settled && airTime >= kAirmoveMinAirTime) || overdue || airTime >= kAirmoveMaxAirTime)
		{
			bot.airmoveActive = false;
			bool oppo = bot.airmoveOpportunistic;
			bot.airmoveOpportunistic = false;

			const Nav::Area *landedArea = Nav::FindAreaExact(mesh, pEdict->v.origin);
			int landedId = landedArea ? static_cast<int>(landedArea->id) : -1;
			float miss = oppo ? -1.f : AirmoveLandingMiss(bot, pEdict);

			bool haveMiss = miss >= 0.f;
			bool decided = settled && (haveMiss || landedArea != nullptr);
			bool failed = decided && (haveMiss
				? miss > kAirmoveLandingTolerance
				: (bot.airmoveTargetArea >= 0 && landedId != bot.airmoveTargetArea));

			if (decided && bot.airmoveConnIndex >= 0)
			{
				uint32_t connIndex = static_cast<uint32_t>(bot.airmoveConnIndex);
				if (failed)
					Nav::PenalizeConnection(connIndex);
				else
					Nav::RewardConnection(connIndex);
			}

			BOT_VLOG("CrossBot: %s %s %s after %.2fs (planned %.2fs limit %.2fs) at (%.1f,%.1f,%.1f) "
				"area %d (wanted %d) miss %.1f conn %d penalty %.1f%s\n", bot.name,
				oppo ? "oppo" : "airmove", AirmoveEndReason(pEdict), airTime,
				bot.airmovePlannedTime, bot.airmoveMaxTime,
				pEdict->v.origin.x, pEdict->v.origin.y, pEdict->v.origin.z, landedId,
				bot.airmoveTargetArea, miss, bot.airmoveConnIndex,
				(bot.airmoveConnIndex >= 0)
					? Nav::ConnectionPenalty(static_cast<uint32_t>(bot.airmoveConnIndex))
					: 0.f,
				decided ? "" : " undecided");

			if (failed && BreakBlockedAirmove(bot, mesh))
				return false;

			if (oppo || failed || !decided)
				bot.nextRepathTime = gpGlobals->time;

			return false;
		}

		if (!bot.airmoveOverrunLogged && bot.airmovePlannedTime > 0.f &&
			airTime > bot.airmovePlannedTime + kAirmoveOverrunGrace)
		{
			bot.airmoveOverrunLogged = true;

			BOT_VLOG("CrossBot: %s airmove overrunning at %.2fs (planned %.2fs limit %.2fs) at "
				"(%.1f,%.1f,%.1f) vel=(%.0f,%.0f,%.0f) onground=%d water=%d movetype=%d duck=%d "
				"area %d (wanted %d) conn %d\n", bot.name, airTime, bot.airmovePlannedTime,
				bot.airmoveMaxTime, pEdict->v.origin.x, pEdict->v.origin.y, pEdict->v.origin.z,
				pEdict->v.velocity.x, pEdict->v.velocity.y, pEdict->v.velocity.z,
				(pEdict->v.flags & FL_ONGROUND) != 0 ? 1 : 0, pEdict->v.waterlevel,
				pEdict->v.movetype, (pEdict->v.flags & FL_DUCKING) != 0 ? 1 : 0,
				AreaIdOf(Nav::FindAreaNearest(mesh, pEdict->v.origin)), bot.airmoveTargetArea,
				bot.airmoveConnIndex);
		}

		Vector breakAngles;
		unsigned short breakButtons = 0;
		const Vector *breakAim = nullptr;

		if (edict_t *plug = AirmovePlug(bot, mesh))
		{
			bool swing = false;
			if (BotBreak::AimSwing(bot, plug, breakAngles, swing))
			{
				breakAim = &breakAngles;
				if (swing)
				{
					breakButtons |= IN_ATTACK;

					BOT_VLOG("CrossBot: %s swings at %s mid-flight, it plugs area %d\n", bot.name,
						CBotClassOf(plug), bot.airmoveTargetArea);
				}
			}
		}

		RunAirmoveDrive(bot, Bot_YawDir(bot.airmoveYaw), bot.airmoveSpeed, bot.airmoveYaw, bot.airmoveCrouch,
			false, false, breakAim, breakButtons);
		return true;
	}

	void RunLadderDrive(CBot &bot, float drive, float aimYaw, float aimPitch, bool crouch, bool jump)
	{
		edict_t *pEdict = bot.pEdict;

		bot.desiredAngles = Vector(aimPitch, aimYaw, 0.f);
		Vector angles = Bot_UpdateFacing(bot);

		float fmove = 0.f, smove = 0.f;

		if (crouch && OnLadder(pEdict) && (pEdict->v.flags & FL_ONGROUND) != 0)
			crouch = false;

		if (OnLadder(pEdict))
		{
			fmove = (drive > 1.f) ? Bot_MaxSpeed(pEdict) : 0.f;
		}
		else
		{
			Vector wishDir = Bot_YawDir(aimYaw);
			if ((pEdict->v.flags & FL_ONGROUND) != 0)
			{
				NavFollow::Drive solved =
					NavFollow::SolveDrive(wishDir * drive, pEdict->v.velocity, Bot_DrivePhysics(pEdict));
				if (solved.wishSpeed > 0.f)
				{
					wishDir = solved.wishDir;
					drive = solved.wishSpeed;
				}
			}

			if ((pEdict->v.flags & FL_DUCKING) != 0)
				drive /= PmSim::kDuckingMultiplier;
			float maxSpeed = Bot_MaxSpeed(pEdict);
			if (drive > maxSpeed)
				drive = maxSpeed;

			Bot_DecomposeMove(angles.y + pEdict->v.punchangle.y, wishDir, drive, fmove, smove);
		}

		bot.moveForward = fmove;
		bot.moveSide = smove;

		unsigned short buttons = 0;
		if (fmove > 1.f) buttons |= IN_FORWARD;
		else if (fmove < -1.f) buttons |= IN_BACK;
		if (smove > 1.f) buttons |= IN_MOVERIGHT;
		else if (smove < -1.f) buttons |= IN_MOVELEFT;
		if (crouch) buttons |= IN_DUCK;
		if (jump) buttons |= IN_JUMP;

		LADDER_DBG(bot, "drive",
			"fly=%d grnd=%d duck=%d fmove=%.0f smove=%.0f btn=%s ang=(%.1f,%.1f) want=(%.1f,%.1f) "
			"org=(%.1f,%.1f,%.1f) vel=(%.0f,%.0f,%.0f)",
			OnLadder(pEdict) ? 1 : 0, (pEdict->v.flags & FL_ONGROUND) != 0 ? 1 : 0,
			(pEdict->v.flags & FL_DUCKING) != 0 ? 1 : 0, fmove, smove, LadderButtonNames(buttons),
			angles.x, angles.y, aimPitch, aimYaw,
			pEdict->v.origin.x, pEdict->v.origin.y, pEdict->v.origin.z,
			pEdict->v.velocity.x, pEdict->v.velocity.y, pEdict->v.velocity.z);

		Bot_RunPlayerMove(bot, angles, fmove, smove, buttons);

		LADDER_DBG(bot, "after",
			"fly=%d grnd=%d duck=%d org=(%.1f,%.1f,%.1f) vel=(%.0f,%.0f,%.0f)",
			OnLadder(pEdict) ? 1 : 0, (pEdict->v.flags & FL_ONGROUND) != 0 ? 1 : 0,
			(pEdict->v.flags & FL_DUCKING) != 0 ? 1 : 0,
			pEdict->v.origin.x, pEdict->v.origin.y, pEdict->v.origin.z,
			pEdict->v.velocity.x, pEdict->v.velocity.y, pEdict->v.velocity.z);
	}

	enum LadderStep
	{
		kLadderUp,
		kLadderDown,
		kLadderSideMinus,
		kLadderSidePlus,
		kLadderPushOff
	};

	struct LadderAim
	{
		float yaw = 0.f;
		float pitch = 0.f;
		bool jump = false;
		bool drive = false;
		bool gated = false;
	};

	const char *LadderStepName(int step)
	{
		switch (step)
		{
		case kLadderUp: return "up";
		case kLadderDown: return "down";
		case kLadderSideMinus: return "side-";
		case kLadderSidePlus: return "side+";
		case kLadderPushOff: return "pushoff";
		default: return "?";
		}
	}

	LadderAim ResolveLadderAim(const CBot &bot, const LadderPlane &plane, int step, float fallbackYaw)
	{
		float aimYaw = fallbackYaw;
		float aimPitch = 0.f;
		float drive = kFullDrive;
		bool jump = false;

		if (plane.valid)
		{
			float intoYaw = atan2f(-plane.normal.y, -plane.normal.x) * (180.f / kPiF);
			float perpYaw = atan2f(plane.perp.y, plane.perp.x) * (180.f / kPiF);

			if (step == kLadderUp)
				aimYaw = intoYaw;
			else if (step == kLadderDown)
			{
				aimYaw = intoYaw;
				aimPitch = kLadderDownPitch;
			}
			else if (step == kLadderSidePlus)
				aimYaw = perpYaw;
			else if (step == kLadderSideMinus)
				aimYaw = perpYaw + 180.f;
			else if (step == kLadderPushOff)
			{
				aimYaw = intoYaw;
				drive = 0.f;
				jump = true;
			}
		}
		else
		{
			if (step == kLadderUp)
			{
				aimYaw = bot.pEdict->v.v_angle.y;
				aimPitch = kLadderUpPitch;
			}
			else if (step == kLadderDown)
			{
				aimYaw = bot.pEdict->v.v_angle.y;
				aimPitch = kLadderDownPitch;
			}
			else if (step == kLadderPushOff)
			{
				drive = 0.f;
				jump = true;
			}
		}

		float yawErr = Bot_NormalizeAngle(aimYaw - bot.pEdict->v.v_angle.y);
		float pitchErr = Bot_NormalizeAngle(aimPitch - bot.pEdict->v.v_angle.x);
		bool gated = !jump &&
			(fabsf(yawErr) > kLadderAimTolerance || fabsf(pitchErr) > kLadderAimTolerance);
		if (gated)
			drive = 0.f;

		LadderAim aim;
		aim.yaw = aimYaw;
		aim.pitch = aimPitch;
		aim.jump = jump;
		aim.drive = drive > 1.f;
		aim.gated = gated;

		LADDER_DBG(bot, "aim",
			"step=%s plane=%s n=(%.2f,%.2f,%.2f) perp=(%.2f,%.2f) want=(%.1f,%.1f) cur=(%.1f,%.1f) "
			"err=(%.1f,%.1f) gated=%d drive=%d jump=%d",
			LadderStepName(step), plane.why, plane.normal.x, plane.normal.y, plane.normal.z,
			plane.perp.x, plane.perp.y, aimPitch, aimYaw, bot.pEdict->v.v_angle.x,
			bot.pEdict->v.v_angle.y, pitchErr, yawErr, gated ? 1 : 0, aim.drive ? 1 : 0,
			jump ? 1 : 0);

		return aim;
	}

	Vector LadderMountAim(const Vector &origin, const Vector &mount, bool up, float climbYaw,
		float entryYaw)
	{
		if (up)
			return mount + Bot_YawDir(climbYaw) * kLadderMountLead;

		Vector entryDir = Bot_YawDir(entryYaw);
		Vector toMount(mount.x - origin.x, mount.y - origin.y, 0.f);
		float ahead = toMount.x * entryDir.x + toMount.y * entryDir.y;
		Vector dir = (ahead > kLadderMountApproach)
			? (toMount * (1.f / toMount.Length())) : entryDir;
		return mount + dir * kAirmoveApproachLead;
	}

	void RunLadderSeek(CBot &bot, const Vector &target, float drive, float pitch, bool crouch)
	{
		edict_t *pEdict = bot.pEdict;
		Vector to(target.x - pEdict->v.origin.x, target.y - pEdict->v.origin.y, 0.f);
		float len = to.Length();
		float yaw = (len > 1.f) ? atan2f(to.y, to.x) * (180.f / kPiF) : pEdict->v.v_angle.y;
		RunLadderDrive(bot, drive, yaw, pitch, crouch, false);
	}

	bool FinishLadder(CBot &bot, const Nav::NavMesh &mesh, const char *why, bool completed)
	{
		edict_t *pEdict = bot.pEdict;
		bot.ladderActive = false;

		const Nav::Area *landedArea = Nav::FindAreaExact(mesh, pEdict->v.origin);
		int landedId = landedArea ? static_cast<int>(landedArea->id) : -1;
		bool failed = !completed ||
			(landedId >= 0 && bot.ladderTargetArea >= 0 && landedId != bot.ladderTargetArea);
		bool undecided = !failed && bot.ladderTargetArea >= 0 && landedId < 0;

		if (!undecided && bot.ladderConnIndex >= 0)
		{
			uint32_t connIndex = static_cast<uint32_t>(bot.ladderConnIndex);
			if (failed)
				Nav::PenalizeConnection(connIndex);
			else
				Nav::RewardConnection(connIndex);
		}

		BOT_VLOG("CrossBot: %s ladder %s %s after %.2fs at (%.1f,%.1f,%.1f) area %d (wanted %d) conn %d %s\n",
			bot.name, bot.ladderUp ? "climb" : "descent", why, gpGlobals->time - bot.ladderStartTime,
			pEdict->v.origin.x, pEdict->v.origin.y, pEdict->v.origin.z, landedId, bot.ladderTargetArea,
			bot.ladderConnIndex, undecided ? "undecided" : (failed ? "failed" : "ok"));

		if (failed)
			bot.nextRepathTime = gpGlobals->time;

		return false;
	}

	int LadderDismountStep(const CBot &bot, const LadderPlane &plane)
	{
		if (bot.ladderJumpOff)
			return kLadderPushOff;
		if (!plane.valid)
			return kLadderUp;

		Vector d = Bot_YawDir(bot.ladderDismountYaw);
		float intoDot = -(d.x * plane.normal.x + d.y * plane.normal.y);
		float perpDot = d.x * plane.perp.x + d.y * plane.perp.y;

		if (intoDot <= kLadderPushOffDot)
			return kLadderPushOff;
		if (intoDot >= fabsf(perpDot))
			return kLadderUp;
		return (perpDot > 0.f) ? kLadderSidePlus : kLadderSideMinus;
	}

	bool RunLadderMove(CBot &bot, const Nav::NavMesh &mesh)
	{
		edict_t *pEdict = bot.pEdict;
		bool onLadder = OnLadder(pEdict);
		bool onGround = (pEdict->v.flags & FL_ONGROUND) != 0;
		float elapsed = gpGlobals->time - bot.ladderStartTime;
		float navZ = BotNavZ(pEdict);

		bot.lastProgressTime = gpGlobals->time;

		if (elapsed >= kLadderMaxTime)
			return FinishLadder(bot, mesh, "timed out on", false);

		if (!bot.ladderGrabbed)
		{
			if (!onLadder)
			{
				Vector toMount(bot.ladderMountPoint.x - pEdict->v.origin.x,
					bot.ladderMountPoint.y - pEdict->v.origin.y, 0.f);

				LADDER_DBG(bot, "entry",
					"up=%d elapsed=%.2f toMount=%.1f disp=%.1f stallAge=%.2f "
					"timeout=%d drift=%d stall=%d org=(%.1f,%.1f,%.1f)",
					bot.ladderUp ? 1 : 0, elapsed, toMount.Length(),
					(pEdict->v.origin - bot.ladderStallOrigin).Length(),
					gpGlobals->time - bot.ladderMoveTime,
					elapsed >= kLadderEntryTimeout ? 1 : 0,
					toMount.Length() > kLadderCommitRadius + kLadderEntryDrift ? 1 : 0,
					gpGlobals->time - bot.ladderMoveTime >= kLadderStallTime ? 1 : 0,
					pEdict->v.origin.x, pEdict->v.origin.y, pEdict->v.origin.z);

				if ((pEdict->v.origin - bot.ladderStallOrigin).Length() >= kLadderStallDist)
				{
					bot.ladderStallOrigin = pEdict->v.origin;
					bot.ladderMoveTime = gpGlobals->time;
				}

				if (elapsed >= kLadderEntryTimeout ||
					toMount.Length() > kLadderCommitRadius + kLadderEntryDrift ||
					gpGlobals->time - bot.ladderMoveTime >= kLadderStallTime)
					return FinishLadder(bot, mesh, "never caught", false);

				Vector aim = LadderMountAim(pEdict->v.origin, bot.ladderMountPoint, bot.ladderUp,
					bot.ladderYaw, bot.ladderEntryYaw);
				float entryDrive = (bot.ladderEntrySpeed > 1.f) ? bot.ladderEntrySpeed : kFullDrive;
				RunLadderSeek(bot, aim, entryDrive, bot.ladderUp ? 0.f : kLadderDownPitch,
					bot.ladderCrouch);
				return true;
			}

			bot.ladderGrabbed = true;
			bot.ladderMoveTime = gpGlobals->time;

			BOT_VLOG("CrossBot: %s caught the ladder at (%.1f,%.1f,%.1f) after %.2fs\n",
				bot.name, pEdict->v.origin.x, pEdict->v.origin.y, pEdict->v.origin.z, elapsed);
		}

		if (!bot.ladderUp)
		{
			if (navZ <= bot.ladderDismountZ + kLadderDismountSlack)
				return FinishLadder(bot, mesh, "reached the bottom of", true);
			if (!onLadder && onGround)
				return FinishLadder(bot, mesh, "fell off", false);
		}
		else if (!onLadder)
		{
			if (bot.ladderLeftTime <= 0.f)
				bot.ladderLeftTime = gpGlobals->time;

			if (onGround && gpGlobals->time - bot.ladderLeftTime >= kLadderMinAirTime)
				return FinishLadder(bot, mesh, "dismounted off", true);
		}

		if (!onLadder)
		{
			LADDER_DBG(bot, "exit",
				"up=%d grnd=%d navZ=%.1f dz=%.1f leftFor=%.2f exit=(%.1f,%.1f,%.1f)",
				bot.ladderUp ? 1 : 0, onGround ? 1 : 0, navZ, bot.ladderDismountZ,
				(bot.ladderLeftTime > 0.f) ? gpGlobals->time - bot.ladderLeftTime : 0.f,
				bot.ladderExitPoint.x, bot.ladderExitPoint.y, bot.ladderExitPoint.z);

			RunLadderSeek(bot, bot.ladderExitPoint,
				bot.ladderJumpOff ? 0.f : Bot_MaxSpeed(pEdict),
				bot.ladderUp ? 0.f : kLadderDownPitch, bot.ladderCrouch);
			return true;
		}

		LadderPlane plane = ProbeLadderPlane(pEdict);

		int step = kLadderDown;
		if (bot.ladderUp)
		{
			if (bot.ladderAtTop || navZ >= bot.ladderDismountZ - kLadderDismountSlack)
			{
				bot.ladderAtTop = true;
				step = LadderDismountStep(bot, plane);
			}
			else
			{
				step = kLadderUp;
			}
		}

		LADDER_DBG(bot, "state",
			"up=%d fly=%d grnd=%d navZ=%.1f dz=%.1f atTop=%d plane=%s frac=%.3f "
			"n=(%.2f,%.2f,%.2f) step=%s dmYaw=%.0f jumpOff=%d stalls=%d stallAge=%.2f disp=%.1f",
			bot.ladderUp ? 1 : 0, onLadder ? 1 : 0, onGround ? 1 : 0, navZ, bot.ladderDismountZ,
			bot.ladderAtTop ? 1 : 0, plane.why, plane.fraction, plane.normal.x, plane.normal.y,
			plane.normal.z, LadderStepName(step), bot.ladderDismountYaw, bot.ladderJumpOff ? 1 : 0,
			bot.ladderStalls, gpGlobals->time - bot.ladderMoveTime,
			(pEdict->v.origin - bot.ladderStallOrigin).Length());

		float fallbackYaw = (step == kLadderUp || step == kLadderDown)
			? bot.ladderYaw : bot.ladderDismountYaw;
		LadderAim aim = ResolveLadderAim(bot, plane, step, fallbackYaw);

		if (!aim.drive || aim.jump ||
			(pEdict->v.origin - bot.ladderStallOrigin).Length() >= kLadderStallDist)
		{
			bot.ladderStallOrigin = pEdict->v.origin;
			bot.ladderMoveTime = gpGlobals->time;
		}
		else if (gpGlobals->time - bot.ladderMoveTime >= kLadderStallTime)
		{
			if (bot.ladderStalls >= kLadderStallEscapes)
				return FinishLadder(bot, mesh, "stalled on", false);

			++bot.ladderStalls;
			bot.ladderMoveTime = gpGlobals->time;
			aim = ResolveLadderAim(bot, plane, kLadderPushOff, bot.ladderDismountYaw);

			BOT_VLOG("CrossBot: %s wedged on the ladder at (%.1f,%.1f,%.1f), pushing off\n", bot.name,
				pEdict->v.origin.x, pEdict->v.origin.y, pEdict->v.origin.z);
		}

		RunLadderDrive(bot, aim.drive ? kFullDrive : 0.f, aim.yaw, aim.pitch, bot.ladderCrouch,
			aim.jump);
		return true;
	}

	bool RunLadderApproach(CBot &bot, const Nav::Waypoint &wp)
	{
		edict_t *pEdict = bot.pEdict;
		const Vector &origin = pEdict->v.origin;
		const Vector &vel = pEdict->v.velocity;

		float navZ = BotNavZ(pEdict);
		if (wp.ladderUp ? (navZ >= wp.ladderDismount.z - kLadderDismountSlack)
			: (navZ <= wp.ladderDismount.z + kLadderDismountSlack))
		{
			LADDER_DBG(bot, "past", "up=%d navZ=%.1f dz=%.1f handing back to the walker",
				wp.ladderUp ? 1 : 0, navZ, wp.ladderDismount.z);
			return false;
		}

		Vector entryDir = Bot_YawDir(wp.ladderEntryYaw);
		Vector toMount(wp.point.x - origin.x, wp.point.y - origin.y, 0.f);
		float remaining = toMount.x * entryDir.x + toMount.y * entryDir.y;
		float dist = sqrtf(toMount.x * toMount.x + toMount.y * toMount.y);
		float speed = sqrtf(vel.x * vel.x + vel.y * vel.y);

		bool onLadder = OnLadder(pEdict);
		bool committed = wp.ladderUp
			? (onLadder && dist <= kLadderMountRadius)
			: (onLadder || (dist <= kLadderCommitRadius &&
				remaining <= NavPlan::AirmoveLaunchLead(speed, Bot_FrameTime())));

		if (committed)
		{
			bot.ladderActive = true;
			bot.ladderGrabbed = false;
			bot.ladderAtTop = false;
			bot.ladderUp = wp.ladderUp;
			bot.ladderJumpOff = wp.ladderJumpOff;
			bot.ladderCrouch = wp.requiresCrouch;
			bot.ladderYaw = wp.ladderYaw;
			bot.ladderEntryYaw = wp.ladderEntryYaw;
			bot.ladderEntrySpeed = wp.ladderEntrySpeed;
			bot.ladderSpeed = wp.ladderSpeed;
			bot.ladderDismountYaw = wp.ladderDismountYaw;
			bot.ladderDismountZ = wp.ladderDismount.z;
			bot.ladderMountPoint = wp.point;
			bot.ladderExitPoint = wp.ladderExit;
			bot.ladderStartTime = gpGlobals->time;
			bot.ladderLeftTime = 0.f;
			bot.ladderMoveTime = gpGlobals->time;
			bot.ladderStallOrigin = origin;
			bot.ladderStalls = 0;
			bot.ladderTargetArea = wp.targetAreaId;
			bot.ladderConnIndex = wp.connIndex;
			++bot.waypointIndex;

			BOT_VLOG("CrossBot: %s ladder %s at (%.1f,%.1f,%.1f) entry yaw %.0f speed %.0f | climb yaw %.0f "
				"speed %.0f | off z %.1f yaw %.0f jump %d | targArea %d conn %d\n",
				bot.name, wp.ladderUp ? "climb" : "descent", origin.x, origin.y, origin.z,
				wp.ladderEntryYaw, wp.ladderEntrySpeed, wp.ladderYaw, wp.ladderSpeed,
				wp.ladderDismount.z, wp.ladderDismountYaw, wp.ladderJumpOff ? 1 : 0, wp.targetAreaId,
				wp.connIndex);

			RunLadderMove(bot, *Nav::Get());
			return true;
		}

		Vector aim = LadderMountAim(origin, wp.point, wp.ladderUp, wp.ladderYaw, wp.ladderEntryYaw);
		Vector moveDir = Norm2D(aim - origin);
		float aimYaw = (moveDir.x != 0.f || moveDir.y != 0.f)
			? atan2f(moveDir.y, moveDir.x) * (180.f / kPiF) : wp.ladderEntryYaw;

		bool blocked = wp.ladderUp && !onLadder && dist <= kLadderMountRadius &&
			(pEdict->v.flags & FL_ONGROUND) != 0 && LadderOutOfReach(pEdict, wp.point);
		if (!blocked)
			bot.ladderReachTime = 0.f;
		else if (bot.ladderReachTime <= 0.f)
			bot.ladderReachTime = gpGlobals->time;

		bool reachUp = blocked && gpGlobals->time - bot.ladderReachTime >= kLadderReachDwell;
		if (reachUp)
			bot.ladderReachTime = 0.f;

		LADDER_DBG(bot, "approach",
			"wp=%zu up=%d crouch=%d jumpOff=%d org=(%.1f,%.1f,%.1f) navZ=%.1f vel=(%.0f,%.0f,%.0f) "
			"mt=%d fly=%d grnd=%d duck=%d mount=(%.1f,%.1f,%.1f) dz=%.1f dist=%.1f rem=%.1f lead=%.1f "
			"aim=(%.1f,%.1f) aimYaw=%.1f curYaw=%.1f blocked=%d reachAge=%.2f reachUp=%d",
			bot.waypointIndex, wp.ladderUp ? 1 : 0, wp.requiresCrouch ? 1 : 0,
			wp.ladderJumpOff ? 1 : 0, origin.x, origin.y, origin.z, navZ, vel.x, vel.y, vel.z,
			pEdict->v.movetype, onLadder ? 1 : 0, (pEdict->v.flags & FL_ONGROUND) != 0 ? 1 : 0,
			(pEdict->v.flags & FL_DUCKING) != 0 ? 1 : 0, wp.point.x, wp.point.y, wp.point.z,
			wp.ladderDismount.z, dist, remaining,
			NavPlan::AirmoveLaunchLead(speed, Bot_FrameTime()), aim.x, aim.y, aimYaw,
			pEdict->v.v_angle.y, blocked ? 1 : 0,
			(bot.ladderReachTime > 0.f) ? gpGlobals->time - bot.ladderReachTime : 0.f,
			reachUp ? 1 : 0);

		RunLadderDrive(bot, kFullDrive, aimYaw, wp.ladderUp ? 0.f : kLadderDownPitch, wp.requiresCrouch,
			reachUp);
		return true;
	}

	float WaypointNavZ(const Nav::Waypoint &wp)
	{
		return wp.point.z - (wp.requiresCrouch ? 0.f : NavPlan::kPlanStandHeight);
	}

	float ElevatorExitYaw(const CBot &bot, size_t arriveWp)
	{
		size_t next = arriveWp + 1;
		if (next < bot.waypoints.size())
		{
			Vector out = bot.waypoints[next].point - bot.waypoints[arriveWp].point;
			if (out.x * out.x + out.y * out.y > 1.f)
				return atan2f(out.y, out.x) * (180.f / kPiF);
		}
		return bot.pEdict->v.v_angle.y;
	}

	void RunElevatorDrive(CBot &bot, const Vector &moveDir, float drive, float aimYaw, bool crouch)
	{
		edict_t *pEdict = bot.pEdict;

		bot.desiredAngles = Vector(0.f, aimYaw, 0.f);
		Vector angles = Bot_UpdateFacing(bot);

		Vector wishDir = moveDir;
		if (drive > 0.f && (pEdict->v.flags & FL_ONGROUND) != 0)
		{
			NavFollow::Drive solved =
				NavFollow::SolveDrive(moveDir * drive, pEdict->v.velocity, Bot_DrivePhysics(pEdict));
			if (solved.wishSpeed > 0.f)
			{
				wishDir = solved.wishDir;
				drive = solved.wishSpeed;
			}
		}

		if ((pEdict->v.flags & FL_DUCKING) != 0)
			drive /= PmSim::kDuckingMultiplier;
		float maxSpeed = Bot_MaxSpeed(pEdict);
		if (drive > maxSpeed)
			drive = maxSpeed;

		float fmove = 0.f, smove = 0.f;
		Bot_DecomposeMove(angles.y + pEdict->v.punchangle.y, wishDir, drive, fmove, smove);
		bot.moveForward = fmove;
		bot.moveSide = smove;

		unsigned short buttons = 0;
		if (fmove > kMoveButtonDeadzone) buttons |= IN_FORWARD;
		else if (fmove < -kMoveButtonDeadzone) buttons |= IN_BACK;
		if (smove > kMoveButtonDeadzone) buttons |= IN_MOVERIGHT;
		else if (smove < -kMoveButtonDeadzone) buttons |= IN_MOVELEFT;
		if (crouch) buttons |= IN_DUCK;

		Bot_RunPlayerMove(bot, angles, fmove, smove, buttons);
	}

	bool FinishElevatorRide(CBot &bot, const Nav::NavMesh &mesh, const char *why)
	{
		edict_t *pEdict = bot.pEdict;
		bot.rideActive = false;

		float navZ = BotNavZ(pEdict);
		float probeZ = (fabsf(bot.rideArriveZ - navZ) <= kElevatorArriveSlack) ? bot.rideArriveZ : navZ;
		Vector probe(pEdict->v.origin.x, pEdict->v.origin.y, probeZ);

		const Nav::Area *target = bot.rideTargetArea >= 0
			? Nav::FindAreaById(mesh, static_cast<uint32_t>(bot.rideTargetArea)) : nullptr;
		const Nav::Area *landedArea = (target && Nav::AreaContainsPoint(*target, probe))
			? target : Nav::FindAreaExact(mesh, probe);
		int landedId = landedArea ? static_cast<int>(landedArea->id) : -1;
		bool decided = landedArea != nullptr;
		bool failed = decided && bot.rideTargetArea >= 0 && landedId != bot.rideTargetArea;

		if (decided && bot.rideConnIndex >= 0)
		{
			uint32_t connIndex = static_cast<uint32_t>(bot.rideConnIndex);
			if (failed)
				Nav::PenalizeConnection(connIndex);
			else
				Nav::RewardConnection(connIndex);
		}

		BOT_VLOG("CrossBot: %s elevator %s after %.2fs at (%.1f,%.1f,%.1f) navz %.1f (stop %.1f) "
			"area %d (wanted %d) conn %d%s\n",
			bot.name, why, gpGlobals->time - bot.rideStartTime, pEdict->v.origin.x, pEdict->v.origin.y,
			pEdict->v.origin.z, navZ, bot.rideArriveZ, landedId, bot.rideTargetArea, bot.rideConnIndex,
			decided ? "" : " undecided");

		if (bot.rideWpIndex < bot.waypoints.size())
			bot.waypoints[bot.rideWpIndex].requiresElevatorRide = false;

		size_t arriveWp = bot.rideWpIndex + 1;
		if (arriveWp < bot.waypoints.size())
		{
			bot.waypointIndex = arriveWp;

			bot.rideFloorWp = arriveWp;
			bot.trackState.progressS = NavPlan::ProfileWaypointS(bot.followProfile, arriveWp);
			bot.lastProgressS = bot.trackState.progressS;
		}

		if (failed || !decided)
			bot.nextRepathTime = gpGlobals->time;

		return false;
	}

			bool RunElevatorRide(CBot &bot, const Nav::NavMesh &mesh)
			{
				edict_t *pEdict = bot.pEdict;

				bot.lastProgressTime = gpGlobals->time;

				float navZ = BotNavZ(pEdict);
				bool arrived = bot.rideUp ? (navZ >= bot.rideArriveZ - kElevatorArriveSlack)
					: (navZ <= bot.rideArriveZ + kElevatorArriveSlack);
		if (arrived)
			return FinishElevatorRide(bot, mesh, "ride reached the stop of");
		if (gpGlobals->time >= bot.rideDeadline)
			return FinishElevatorRide(bot, mesh, "ride timed out on");

		Vector to(bot.ridePoint.x - pEdict->v.origin.x, bot.ridePoint.y - pEdict->v.origin.y, 0.f);
		float drift = sqrtf(to.x * to.x + to.y * to.y);

		Vector moveDir(0.f, 0.f, 0.f);
		float drive = 0.f;
		if (drift > kElevatorHoldRadius)
		{
			moveDir = Norm2D(to);
			drive = Bot_MaxSpeed(pEdict) * PmSim::kDuckingMultiplier;
		}

		RunElevatorDrive(bot, moveDir, drive, bot.rideExitYaw, bot.rideCrouch);
		return true;
	}

	bool RunElevatorApproach(CBot &bot, size_t rideWp)
	{
		edict_t *pEdict = bot.pEdict;
		const Vector &origin = pEdict->v.origin;
		const Nav::Waypoint &wp = bot.waypoints[rideWp];

		size_t arriveWp = rideWp + 1;
		if (arriveWp >= bot.waypoints.size())
			return false;

		Vector to(wp.point.x - origin.x, wp.point.y - origin.y, 0.f);
		float dist = sqrtf(to.x * to.x + to.y * to.y);

		if (dist > kElevatorEngageRadius)
			return false;

		if (dist <= kElevatorBoardRadius && (pEdict->v.flags & FL_ONGROUND) != 0)
		{
			const Nav::Waypoint &arrive = bot.waypoints[arriveWp];

			bot.rideActive = true;
			bot.rideWpIndex = rideWp;
			bot.ridePoint = wp.point;
			bot.rideCrouch = wp.requiresCrouch;
			bot.rideArriveZ = WaypointNavZ(arrive);
			bot.rideUp = bot.rideArriveZ >= BotNavZ(pEdict);
			bot.rideExitYaw = ElevatorExitYaw(bot, arriveWp);
			bot.rideStartTime = gpGlobals->time;
			bot.rideDeadline = gpGlobals->time + wp.elevatorWaitTime +
				wp.elevatorTravelTime * kElevatorTimeMargin + kElevatorTimeGrace;
			bot.rideTargetArea = wp.targetAreaId;
			bot.rideConnIndex = wp.connIndex;

			BOT_VLOG("CrossBot: %s boarded the elevator at (%.1f,%.1f,%.1f) for nav z %.1f (%s), travel %.2fs "
				"wait %.2fs, exit yaw %.0f, targArea %d conn %d\n",
				bot.name, origin.x, origin.y, origin.z, bot.rideArriveZ, bot.rideUp ? "up" : "down",
				wp.elevatorTravelTime, wp.elevatorWaitTime, bot.rideExitYaw, wp.targetAreaId, wp.connIndex);

			RunElevatorRide(bot, *Nav::Get());
			return true;
		}

		Vector moveDir = Norm2D(to);
		float aimYaw = (moveDir.x != 0.f || moveDir.y != 0.f)
			? atan2f(moveDir.y, moveDir.x) * (180.f / kPiF) : pEdict->v.v_angle.y;

		RunElevatorDrive(bot, moveDir, Bot_MaxSpeed(pEdict) * PmSim::kDuckingMultiplier, aimYaw,
			wp.requiresCrouch);
		return true;
	}

	bool RecentlyActivated(const CBot &bot, const Nav::Waypoint &wp)
	{
		const Vector &key = wp.activatorCenter;
		for (const BotActivation &a : bot.activationMemo)
		{
			if ((a.center - key).Length() > kActivationSameRadius)
				continue;
			if (gpGlobals->time - a.time <= kActivationMemory)
				return true;
		}
		return false;
	}

	void NoteActivated(CBot &bot, const Nav::Waypoint &wp)
	{
		const Vector &key = wp.activatorCenter;
		for (BotActivation &a : bot.activationMemo)
		{
			if ((a.center - key).Length() > kActivationSameRadius)
				continue;
			a.time = gpGlobals->time;
			return;
		}

		BotActivation added;
		added.center = key;
		added.time = gpGlobals->time;
		bot.activationMemo.push_back(added);
	}

	size_t NextActivation(const CBot &bot)
	{
		for (size_t i = 0; i < bot.waypoints.size(); ++i)
		{
			const Nav::Waypoint &wp = bot.waypoints[i];
			if (!wp.requiresButtonUse && !wp.requiresTriggerTouch)
				continue;
			if (RecentlyActivated(bot, wp))
				continue;
			return i;
		}
		return bot.waypoints.size();
	}

	bool TriggerTouched(const CBot &bot, const Nav::Waypoint &wp)
	{
		if (!wp.requiresTriggerTouch)
			return false;

		const Vector &origin = bot.pEdict->v.origin;
		float dx = wp.point.x - origin.x, dy = wp.point.y - origin.y;
		if (dx * dx + dy * dy > kTriggerTouchRadius * kTriggerTouchRadius)
			return false;

		return fabsf(wp.point.z - origin.z) <= kTriggerTouchHeight;
	}

	void FinishActivation(CBot &bot, const Nav::Waypoint &wp, const char *why)
	{
		bot.activationActive = false;
		bot.activationPressed = false;
		NoteActivated(bot, wp);

		BOT_VLOG("CrossBot: %s %s the activator at (%.1f,%.1f,%.1f)\n", bot.name, why,
			wp.activatorCenter.x, wp.activatorCenter.y, wp.activatorCenter.z);
	}

	bool RunActivation(CBot &bot, size_t index)
	{
		edict_t *pEdict = bot.pEdict;
		const Nav::Waypoint &wp = bot.waypoints[index];
		if (!wp.requiresButtonUse)
			return false;

		const Vector &origin = pEdict->v.origin;
		Vector to(wp.point.x - origin.x, wp.point.y - origin.y, 0.f);
		float dist = sqrtf(to.x * to.x + to.y * to.y);

		if (!bot.activationActive)
		{
			if (dist > kActivationEngageRadius)
				return false;

			bot.activationActive = true;
			bot.activationPressed = false;
			bot.activationStartTime = gpGlobals->time;
			bot.activationHoldUntil = 0.f;

			BOT_VLOG("CrossBot: %s reached the service point (%.1f,%.1f,%.1f) for the activator at "
				"(%.1f,%.1f,%.1f), wp %zu/%zu, wait %.1fs\n", bot.name, wp.point.x, wp.point.y,
				wp.point.z, wp.activatorCenter.x, wp.activatorCenter.y, wp.activatorCenter.z, index,
				bot.waypoints.size(), wp.buttonWaitTime);
		}

		bot.lastProgressTime = gpGlobals->time;

		Vector eye = origin + pEdict->v.view_ofs;
		Vector los = wp.activatorCenter - eye;
		float flat = sqrtf(los.x * los.x + los.y * los.y);
		if (flat > 0.0001f || fabsf(los.z) > 0.0001f)
			bot.desiredAngles = Vector(-atan2f(los.z, flat) * (180.f / kPiF),
				atan2f(los.y, los.x) * (180.f / kPiF), 0.f);

		Vector angles = Bot_UpdateFacing(bot);

		float useDist = (wp.activatorCenter - origin).Length();
		bool inRange = useDist <= kActivationUseRange;
		bool aimed =
			fabsf(Bot_NormalizeAngle(bot.desiredAngles.y - angles.y)) <= kActivationAimTolerance &&
			fabsf(Bot_NormalizeAngle(bot.desiredAngles.x - angles.x)) <= kActivationAimTolerance;

		unsigned short buttons = 0;
		if (!bot.activationPressed && aimed && inRange)
		{
			buttons |= IN_USE;
			bot.activationPressed = true;
			bot.activationHoldUntil = gpGlobals->time +
				std::max(kActivationMinHold, std::min(wp.buttonWaitTime, kActivationMaxHold));

			BOT_VLOG("CrossBot: %s pressed USE at (%.1f,%.1f,%.1f), %.1f from the activator, holding %.1fs\n",
				bot.name, origin.x, origin.y, origin.z, useDist,
				bot.activationHoldUntil - gpGlobals->time);
		}

		if (bot.activationPressed && gpGlobals->time >= bot.activationHoldUntil)
		{
			FinishActivation(bot, wp, "finished with");
			return false;
		}

		if (gpGlobals->time - bot.activationStartTime > kActivationTimeout)
		{
			FinishActivation(bot, wp, inRange ? "could not aim at" : "could not reach");
			bot.nextRepathTime = gpGlobals->time;
			return false;
		}

		float drive = 0.f;
		Vector moveDir(0.f, 0.f, 0.f);
		if (dist > 1.f && (dist > kActivationStandRadius || !inRange))
		{
			moveDir = Norm2D(to);
			drive = Bot_MaxSpeed(pEdict) * PmSim::kDuckingMultiplier;

			NavFollow::Drive solved = NavFollow::SolveDrive(moveDir * drive, pEdict->v.velocity,
				Bot_DrivePhysics(pEdict));
			if (solved.wishSpeed > 0.f)
			{
				moveDir = solved.wishDir;
				drive = solved.wishSpeed;
			}
		}

		if ((pEdict->v.flags & FL_DUCKING) != 0)
			drive /= PmSim::kDuckingMultiplier;

		float fmove = 0.f, smove = 0.f;
		Bot_DecomposeMove(angles.y + pEdict->v.punchangle.y, moveDir, drive, fmove, smove);
		bot.moveForward = fmove;
		bot.moveSide = smove;

		if (fmove > kMoveButtonDeadzone) buttons |= IN_FORWARD;
		else if (fmove < -kMoveButtonDeadzone) buttons |= IN_BACK;
		if (smove > kMoveButtonDeadzone) buttons |= IN_MOVERIGHT;
		else if (smove < -kMoveButtonDeadzone) buttons |= IN_MOVELEFT;
		if (wp.requiresCrouch) buttons |= IN_DUCK;

		Bot_RunPlayerMove(bot, angles, fmove, smove, buttons);
		return true;
	}

	void FinishCharger(CBot &bot, edict_t *ent, const char *why)
	{
		BOT_VLOG("CrossBot: %s %s at %s after %.1fs, now at %.0f\n", bot.name, why,
			CBotClassOf(ent), gpGlobals->time - bot.chargerStartTime, bot.chargerLastValue);

		bot.chargerActive = false;
		bot.chargerPressed = false;
		BotItems::ForgetGoalItem(bot);
	}

	bool RunChargerUse(CBot &bot)
	{
		edict_t *pEdict = bot.pEdict;
		edict_t *ent = bot.goalItem.Get();
		if (!ent)
			return false;

		ItemKind kind = bot.goalItemKind;
		float value = g_pGameProfile->ChargerValue(kind, pEdict);

		if (!bot.chargerActive)
		{
			bot.chargerActive = true;
			bot.chargerPressed = false;
			bot.chargerStartTime = gpGlobals->time;
			bot.chargerLastGainTime = gpGlobals->time;
			bot.chargerLastValue = value;

			BOT_VLOG("CrossBot: %s reached %s at %.0f, starting to charge\n",
				bot.name, CBotClassOf(ent), value);
		}

		bot.lastProgressTime = gpGlobals->time;

		if (value > bot.chargerLastValue)
			bot.chargerLastGainTime = gpGlobals->time;
		bot.chargerLastValue = value;

		Vector center = BotItems::ItemCenter(ent, kind);
		Vector origin = pEdict->v.origin;
		Vector eye = origin + pEdict->v.view_ofs;
		Vector los = center - eye;
		float flat = sqrtf(los.x * los.x + los.y * los.y);
		if (flat > 0.0001f || fabsf(los.z) > 0.0001f)
			bot.desiredAngles = Vector(-atan2f(los.z, flat) * (180.f / kPiF),
				atan2f(los.y, los.x) * (180.f / kPiF), 0.f);

		Vector angles = Bot_UpdateFacing(bot);

		float useDist = (center - origin).Length();
		bool inRange = useDist <= kChargerUseRange;
		bool aimed =
			fabsf(Bot_NormalizeAngle(bot.desiredAngles.y - angles.y)) <= kActivationAimTolerance &&
			fabsf(Bot_NormalizeAngle(bot.desiredAngles.x - angles.x)) <= kActivationAimTolerance;

		unsigned short buttons = 0;
		if (inRange && aimed)
		{
			buttons |= IN_USE;
			bot.chargerPressed = true;
		}

		if (bot.chargerPressed && gpGlobals->time - bot.chargerLastGainTime > kChargerNoGainGrace)
		{
			if (BotItems::IsChargerEmpty(ent))
				FinishCharger(bot, ent, "drained the charger");
			else
			{
				BotItems::NoteFull(bot, CBotClassOf(ent));
				FinishCharger(bot, ent, "is already full");
			}
			return false;
		}

		if (value >= g_pGameProfile->ChargerFullValue(kind))
		{
			BotItems::NoteFull(bot, CBotClassOf(ent));
			FinishCharger(bot, ent, "topped up");
			return false;
		}

		if (gpGlobals->time - bot.chargerStartTime > kChargerTimeout)
		{
			BotItems::NoteIgnored(bot, ent, g_pGameProfile->ItemUnreachableTimeout());
			FinishCharger(bot, ent, inRange ? "could not aim" : "could not reach");
			return false;
		}

		float drive = 0.f;
		Vector moveDir(0.f, 0.f, 0.f);
		Vector to(center.x - origin.x, center.y - origin.y, 0.f);
		float dist = sqrtf(to.x * to.x + to.y * to.y);
		if (!inRange && dist > 1.f)
		{
			moveDir = Norm2D(to);
			drive = Bot_MaxSpeed(pEdict) * PmSim::kDuckingMultiplier;

			NavFollow::Drive solved = NavFollow::SolveDrive(moveDir * drive, pEdict->v.velocity,
				Bot_DrivePhysics(pEdict));
			if (solved.wishSpeed > 0.f)
			{
				moveDir = solved.wishDir;
				drive = solved.wishSpeed;
			}
		}

		if ((pEdict->v.flags & FL_DUCKING) != 0)
			drive /= PmSim::kDuckingMultiplier;

		float fmove = 0.f, smove = 0.f;
		Bot_DecomposeMove(angles.y + pEdict->v.punchangle.y, moveDir, drive, fmove, smove);
		bot.moveForward = fmove;
		bot.moveSide = smove;

		if (fmove > kMoveButtonDeadzone) buttons |= IN_FORWARD;
		else if (fmove < -kMoveButtonDeadzone) buttons |= IN_BACK;
		if (smove > kMoveButtonDeadzone) buttons |= IN_MOVERIGHT;
		else if (smove < -kMoveButtonDeadzone) buttons |= IN_MOVELEFT;

		Bot_RunPlayerMove(bot, angles, fmove, smove, buttons);
		return true;
	}

	// A healing trigger_hurt has no fixture to face and no button to press: the bot just has to
	// be standing in the brush. It reuses the charger bookkeeping, where chargerPressed means
	// "was inside" rather than "held +use".
	bool RunHealVolume(CBot &bot)
	{
		edict_t *pEdict = bot.pEdict;
		edict_t *ent = bot.goalItem.Get();
		if (!ent)
			return false;

		ItemKind kind = bot.goalItemKind;
		float value = g_pGameProfile->ChargerValue(kind, pEdict);

		if (!bot.chargerActive)
		{
			bot.chargerActive = true;
			bot.chargerPressed = false;
			bot.chargerStartTime = gpGlobals->time;
			bot.chargerLastGainTime = gpGlobals->time;
			bot.chargerLastValue = value;

			BOT_VLOG("CrossBot: %s reached %s at %.0f, wading in\n",
				bot.name, CBotClassOf(ent), value);
		}

		bot.lastProgressTime = gpGlobals->time;

		if (value > bot.chargerLastValue)
			bot.chargerLastGainTime = gpGlobals->time;
		bot.chargerLastValue = value;

		// Exactly the test SV_TouchLinks runs before it calls HurtTouch.
		bool ducking = (pEdict->v.flags & FL_DUCKING) != 0;
		bool inside = BotItems::OriginInsideVolume(ent, pEdict->v.origin,
			ducking ? kHullDuck : kHullHuman);
		if (inside)
			bot.chargerPressed = true;

		// Nothing to aim at, so leave the view to the sense system and keep watching the room.
		BotSense::ApplyLook(bot);
		Vector angles = Bot_UpdateFacing(bot);

		if (bot.chargerPressed && gpGlobals->time - bot.chargerLastGainTime > kHealNoGainGrace)
		{
			if (BotItems::IsVolumeInactive(ent))
				FinishCharger(bot, ent, "saw the pool switch off");
			else if (!inside)
			{
				// Drifted back out rather than discovering the pool is useless; a short
				// blacklist, not the 120s one that says "this classname gives me nothing".
				BotItems::NoteIgnored(bot, ent, g_pGameProfile->ItemUnreachableTimeout());
				FinishCharger(bot, ent, "could not stay in the pool");
			}
			else
			{
				BotItems::NoteFull(bot, CBotClassOf(ent));
				FinishCharger(bot, ent, "is already full");
			}
			return false;
		}

		if (value >= g_pGameProfile->ChargerFullValue(kind))
		{
			BotItems::NoteFull(bot, CBotClassOf(ent));
			FinishCharger(bot, ent, "topped up");
			return false;
		}

		float waited = gpGlobals->time - bot.chargerStartTime;
		if ((!bot.chargerPressed && waited > kHealApproachTimeout) || waited > kHealTimeout)
		{
			BotItems::NoteIgnored(bot, ent, g_pGameProfile->ItemUnreachableTimeout());
			FinishCharger(bot, ent, bot.chargerPressed ? "soaked too long"
				: "could not reach the pool");
			return false;
		}

		Vector origin = pEdict->v.origin;
		float drive = 0.f;
		Vector moveDir(0.f, 0.f, 0.f);
		// Head for the resolved spot inside the volume, not its centre: a large pool can have
		// its middle sitting inside geometry the bot cannot walk through.
		Vector to(bot.goalPoint.x - origin.x, bot.goalPoint.y - origin.y, 0.f);
		float dist = sqrtf(to.x * to.x + to.y * to.y);
		if (!inside && dist > 1.f)
		{
			moveDir = Norm2D(to);
			drive = Bot_MaxSpeed(pEdict) * PmSim::kDuckingMultiplier;

			NavFollow::Drive solved = NavFollow::SolveDrive(moveDir * drive, pEdict->v.velocity,
				Bot_DrivePhysics(pEdict));
			if (solved.wishSpeed > 0.f)
			{
				moveDir = solved.wishDir;
				drive = solved.wishSpeed;
			}
		}

		if ((pEdict->v.flags & FL_DUCKING) != 0)
			drive /= PmSim::kDuckingMultiplier;

		float fmove = 0.f, smove = 0.f;
		Bot_DecomposeMove(angles.y + pEdict->v.punchangle.y, moveDir, drive, fmove, smove);
		bot.moveForward = fmove;
		bot.moveSide = smove;

		unsigned short buttons = 0;
		if (fmove > kMoveButtonDeadzone) buttons |= IN_FORWARD;
		else if (fmove < -kMoveButtonDeadzone) buttons |= IN_BACK;
		if (smove > kMoveButtonDeadzone) buttons |= IN_MOVERIGHT;
		else if (smove < -kMoveButtonDeadzone) buttons |= IN_MOVELEFT;

		Bot_RunPlayerMove(bot, angles, fmove, smove, buttons);
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
		bot.airmoveOpportunistic = false;
		bot.oppoCostsGoalArea = -1;
		bot.airmovePlannedTime = 0.f;
		bot.airmoveMaxTime = kAirmoveMaxAirTime;
		bot.airmoveOverrunLogged = false;
		bot.ladderActive = false;
		bot.ladderTargetArea = -1;
		bot.ladderConnIndex = -1;
		bot.rideActive = false;
		bot.rideTargetArea = -1;
		bot.rideConnIndex = -1;
		bot.rideFloorWp = 0;
		bot.activationActive = false;
		bot.activationPressed = false;
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
		const char *path = (cvar && cvar->string && cvar->string[0]) ? cvar->string : "valve/addons/crossbot/trace/nav_trace.txt";
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
		bot.rideFloorWp = 0;

		NavPlan::PlanTraceContext traces = MakePlanTraces(pEdict);
		bot.pushFields = traces.standPush;
		BotPerf::Enter(BotPerf::ZONE_RP_PROFILE);
		bot.followProfile = NavPlan::BuildPathProfile(bot.waypoints, traces.physics, &traces);
		BotPerf::Leave(BotPerf::ZONE_RP_PROFILE);
		bot.trackState = NavFollow::TrackState();
		bot.lastProgressS = 0.f;
		bot.lastProgressTime = gpGlobals->time;

		bot.avoid.detourEnd = 1 + avoid.detourPoints.size();

		if (extra > 0.f && physics.maxSpeed > 1.f && bot.estimatedArrivalTime > 0.f)
			bot.estimatedArrivalTime += extra / physics.maxSpeed;
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

	const char *ConnTypeName(uint8_t type)
	{
		switch (type)
		{
		case crossbot_nav::CONN_WALK: return "walk";
		case crossbot_nav::CONN_TELEPORT: return "teleport";
		case crossbot_nav::CONN_AIRMOVE: return "airmove";
		case crossbot_nav::CONN_LADDER: return "ladder";
		case crossbot_nav::CONN_ELEVATOR: return "elevator";
		}
		return "?";
	}

	bool Repath(CBot &bot, const Nav::NavMesh &mesh, edict_t *pEdict)
	{
		bool freshAttempt = bot.waypoints.empty();

		const Nav::Area *startArea = Nav::FindAreaNearest(mesh, pEdict->v.origin);
		const Nav::Area *goalArea = Nav::FindAreaNearest(mesh, bot.goalPoint);
		if (!startArea || !goalArea)
			return false;

		Vector clampedGoal = bot.goalPoint;
		clampedGoal.x = std::max(goalArea->mins.x, std::min(goalArea->maxs.x, clampedGoal.x));
		clampedGoal.y = std::max(goalArea->mins.y, std::min(goalArea->maxs.y, clampedGoal.y));
		clampedGoal.z = std::max(goalArea->mins.z, std::min(goalArea->maxs.z, clampedGoal.z));

		BotBreak::PhaseThrough phaseBreakables;

		NavPlan::PlanTraceContext traces = MakePlanTraces(pEdict);
		bot.pushFields = traces.standPush;
		NavPlan::PlanStats planStats;
		float careful = Bot_Carefulness(bot);
		std::chrono::steady_clock::time_point planBegin = std::chrono::steady_clock::now();
		BotPerf::Enter(BotPerf::ZONE_RP_PLAN);
		Nav::PlannedPath path = Nav::PlanPath(mesh, startArea->id, goalArea->id, pEdict->v.origin,
			clampedGoal, &traces, &planStats, careful);
		BotPerf::Leave(BotPerf::ZONE_RP_PLAN);
		float planMs = std::chrono::duration<float, std::milli>(
			std::chrono::steady_clock::now() - planBegin).count();
		if (!path.found)
		{
			BOT_VLOG("CrossBot: %s found no route area %u -> %u from (%.1f,%.1f,%.1f) to (%.1f,%.1f,%.1f) | "
				"search %zu areas/%zu sims, %zu air moves rejected\n",
				bot.name, startArea->id, goalArea->id, pEdict->v.origin.x, pEdict->v.origin.y,
				pEdict->v.origin.z, clampedGoal.x, clampedGoal.y, clampedGoal.z, path.expansions,
				path.airmovesTried, path.airmovesRejected);
			return false;
		}

		bot.waypoints = std::move(path.waypoints);
		bot.waypointIndex = 0;
		bot.airmoveActive = false;
		bot.rideActive = false;
		bot.rideFloorWp = 0;
		bot.activationActive = false;
		bot.activationPressed = false;
		bot.plannedAreaPath = path.areaPath;
		bot.plannedConnPath = path.connPath;
		bot.avoid.detourEnd = 0;
		bot.avoid.detourHit = 0;
		bot.avoid.detourExpire = 0.f;

		BotPerf::Enter(BotPerf::ZONE_RP_PROFILE);
		bot.followProfile = NavPlan::BuildPathProfile(bot.waypoints, traces.physics, &traces);
		BotPerf::Leave(BotPerf::ZONE_RP_PROFILE);
		bot.trackState = NavFollow::TrackState();
		bot.lastProgressS = 0.f;
		BotOppo::OnRepath(bot);

		size_t penalizedOnRoute = 0;
#if BOT_VLOG_ON
		for (uint32_t connIndex : path.connPath)
			if (Nav::ConnectionPenalty(connIndex) > 0.f)
				++penalizedOnRoute;
#endif

		BOTPERF_SCOPE(ZONE_RP_LOG);

		if (freshAttempt)
		{
			bot.pathStartTime = gpGlobals->time;
			bot.estimatedArrivalTime = gpGlobals->time + bot.followProfile.totalTime * kEtaSafetyMargin;
			bot.visitedAreaPath.clear();
			bot.visitedAreaPath.push_back(startArea->id);

			BOT_VLOG("CrossBot: Repath start=(%.1f,%.1f,%.1f) goal=(%.1f,%.1f,%.1f) waypoints=%zu "
				"airmoves=%zu simulated/%zu rescaled/%zu runup-ext/%zu respeeded/%zu kept/%zu retracted/%zu retract-kept/%zu rejected | search %zu areas/%zu sims "
				"in %.1fms | careful=%.2f route=%.1fs score=%.1f | profile segs=%zu len=%.0f "
				"eta=%.1fs arcs=%zu slowed=%zu shrunk=%zu | penalized=%zu\n",
				pEdict->v.origin.x, pEdict->v.origin.y, pEdict->v.origin.z,
				clampedGoal.x, clampedGoal.y, clampedGoal.z, bot.waypoints.size(),
				planStats.airmovesSimulated, planStats.airmovesRescaled,
				planStats.airmovesRunupExtended, planStats.airmovesRespeeded,
				planStats.airmovesRespeedKept, planStats.airmovesRetracted,
				planStats.airmovesRetractKept,
				path.airmovesRejected, path.expansions, path.airmovesTried, planMs,
				careful, path.routeSeconds, path.routeScore,
				bot.followProfile.segs.size(), bot.followProfile.totalLength,
				bot.followProfile.totalTime, bot.followProfile.cornersArced,
				bot.followProfile.cornersSlowed, bot.followProfile.cornersShrunk, penalizedOnRoute);
#if BOT_VLOG_ON
			for (size_t i = 0; i < bot.plannedConnPath.size(); ++i)
			{
				uint32_t ci = bot.plannedConnPath[i];
				if (ci >= mesh.conns.size())
					continue;

				const Nav::Connection &c = mesh.conns[ci];
				BOT_VLOG("CrossBot: conn[%zu]=%u %s %u<->%u cost=%.2f travel=%.2f\n", i, ci,
					ConnTypeName(c.connType), c.areaA, c.areaB, c.costSeconds, c.travelSeconds);
			}
			for (size_t i = 0; i < bot.waypoints.size(); ++i)
			{
				const Nav::Waypoint &wp = bot.waypoints[i];
				if (wp.airmoveLaunch)
				{
					BOT_VLOG("CrossBot: wp[%zu]=(%.1f,%.1f,%.1f) AIRMOVE jump=%d crouch=%d yaw=%.0f speed=%.0f "
						"sim=%d targArea=%d\n",
						i, wp.point.x, wp.point.y, wp.point.z, wp.requiresJump, wp.requiresCrouch,
						wp.launchYaw, wp.launchSpeed, wp.launchSimulated, wp.targetAreaId);
				}
				else if (wp.requiresLadder)
				{
					BOT_VLOG("CrossBot: wp[%zu]=(%.1f,%.1f,%.1f) LADDER up=%d crouch=%d entry yaw=%.0f "
						"speed=%.0f climb yaw=%.0f speed=%.0f off z=%.1f yaw=%.0f jump=%d targArea=%d\n",
						i, wp.point.x, wp.point.y, wp.point.z, wp.ladderUp, wp.requiresCrouch,
						wp.ladderEntryYaw, wp.ladderEntrySpeed, wp.ladderYaw, wp.ladderSpeed,
						wp.ladderDismount.z, wp.ladderDismountYaw, wp.ladderJumpOff, wp.targetAreaId);
				}
				else if (wp.requiresElevatorRide)
				{
					BOT_VLOG("CrossBot: wp[%zu]=(%.1f,%.1f,%.1f) ELEVATOR travel=%.2fs wait=%.2fs crouch=%d "
						"targArea=%d conn=%d\n",
						i, wp.point.x, wp.point.y, wp.point.z, wp.elevatorTravelTime, wp.elevatorWaitTime,
						wp.requiresCrouch, wp.targetAreaId, wp.connIndex);
				}
				else
				{
					BOT_VLOG("CrossBot: wp[%zu]=(%.1f,%.1f,%.1f) jump=%d crouch=%d targArea=%d\n",
						i, wp.point.x, wp.point.y, wp.point.z,
						wp.requiresJump, wp.requiresCrouch, wp.targetAreaId);
				}
			}
#endif
		}
		else
		{
			BOT_VLOG("CrossBot: Replan from=(%.1f,%.1f,%.1f) waypoints=%zu areas=%zu eta=%.1fs | penalized=%zu\n",
				pEdict->v.origin.x, pEdict->v.origin.y, pEdict->v.origin.z, bot.waypoints.size(),
				path.areaPath.size(), bot.followProfile.totalTime, penalizedOnRoute);
		}
		return true;
	}

	bool RepathDue(const CBot &bot)
	{
		return bot.nextRepathTime > 0.f && bot.nextRepathTime <= gpGlobals->time;
	}

	bool TryRepath(CBot &bot, const Nav::NavMesh &mesh, edict_t *pEdict, const char *why)
	{
		BOTPERF_SCOPE(ZONE_REPATH);
		if (!bot.waypoints.empty())
		{
			BOT_VLOG("CrossBot: %s mid-route repath (%s) at (%.1f,%.1f,%.1f) wp %zu/%zu\n",
				bot.name, why, pEdict->v.origin.x, pEdict->v.origin.y, pEdict->v.origin.z,
				bot.waypointIndex, bot.waypoints.size());
		}
		if (!Repath(bot, mesh, pEdict))
		{
			WriteTraceLog(bot);
			BotGoal::OnRepathFailed(bot);
			return false;
		}
		bot.nextRepathTime = 0.f;
		bot.lastProgressTime = gpGlobals->time;
		bot.lastProgressS = 0.f;
		return true;
	}
}

PmSim::PmPhysics Bot_DrivePhysics(edict_t *pEdict)
{
	PmSim::PmPhysics physics = Bot_Physics(pEdict);
	if (AtFrictionEdge(pEdict))
		physics.groundFriction *= physics.edgeFriction;
	return physics;
}

bool Bot_PenalizeCurrentConnection(CBot &bot)
{
	if (bot.currentAreaId < 0)
		return false;

	uint32_t cur = static_cast<uint32_t>(bot.currentAreaId);
	size_t areaSteps = bot.plannedAreaPath.empty() ? 0 : bot.plannedAreaPath.size() - 1;
	size_t count = std::min(bot.plannedConnPath.size(), areaSteps);
	for (size_t i = 0; i < count; ++i)
	{
		if (bot.plannedAreaPath[i] != cur)
			continue;

		Nav::PenalizeConnection(bot.plannedConnPath[i]);

		BOT_VLOG("CrossBot: %s penalizing conn %u (area %u->%u) after a blocked detour, penalty %.1f\n",
			bot.name, bot.plannedConnPath[i], cur, bot.plannedAreaPath[i + 1],
			Nav::ConnectionPenalty(bot.plannedConnPath[i]));
		return true;
	}

	return false;
}

void Bot_ResetAirmoveCache()
{
	g_airmoveCache.Clear();
}

const NavPlan::AirmoveCache &Bot_AirmoveCache()
{
	return g_airmoveCache;
}

bool Bot_ManeuverActive(const CBot &bot)
{
	return bot.airmoveActive || bot.ladderActive || bot.rideActive || bot.activationActive ||
		bot.chargerActive || bot.brk.active || bot.deploy.active || bot.fun.mounted ||
		bot.fun.aboard || bot.fun.arrival;
}

void Bot_ResetNavPath(CBot &bot)
{
	ResetPath(bot);
}

namespace
{
	constexpr float kSwimReach = 40.f;
	constexpr float kSwimUpDrive = 320.f;
	constexpr float kSwimClimbSlack = 8.f;
	constexpr float kSwimAirPanic = 3.f;
	constexpr int kWaterlevelEyes = 3;

	bool RunSwimMove(CBot &bot)
	{
		edict_t *pEdict = bot.pEdict;

		if (pEdict->v.waterlevel < kWaterlevelSwimming)
			return false;

		if ((pEdict->v.flags & FL_WATERJUMP) != 0)
		{
			Vector angles = Bot_UpdateFacing(bot);
			Bot_RunPlayerMove(bot, angles, 0.f, 0.f, 0);
			return true;
		}

		if (bot.waypoints.empty())
			return false;

		size_t wasIndex = bot.waypointIndex;
		while (bot.waypointIndex < bot.waypoints.size() &&
			(bot.waypoints[bot.waypointIndex].point - pEdict->v.origin).Length() < kSwimReach)
			++bot.waypointIndex;

		if (bot.waypointIndex != wasIndex)
			bot.lastProgressTime = gpGlobals->time;

		if (bot.waypointIndex >= bot.waypoints.size())
			return false;

		Vector to = bot.waypoints[bot.waypointIndex].point - pEdict->v.origin;
		float flat = sqrtf(to.x * to.x + to.y * to.y);

		float yaw = (flat > 0.1f) ? atan2f(to.y, to.x) * (180.f / kPiF) : pEdict->v.v_angle.y;
		float pitch = -atan2f(to.z, std::max(flat, 1.f)) * (180.f / kPiF);
		if (pitch > 89.f)
			pitch = 89.f;
		else if (pitch < -89.f)
			pitch = -89.f;

		bot.desiredAngles = Vector(pitch, yaw, 0.f);
		Vector angles = Bot_UpdateFacing(bot);

		float drive = Bot_MaxSpeed(pEdict);
		float upmove = 0.f;

		if (to.z > kSwimClimbSlack)
			upmove = kSwimUpDrive;
		else if (to.z < -kSwimClimbSlack)
			upmove = -kSwimUpDrive;

		bool lowOnAir = pEdict->v.waterlevel >= kWaterlevelEyes &&
			pEdict->v.air_finished > 0.f &&
			(pEdict->v.air_finished - gpGlobals->time) < kSwimAirPanic;
		if (lowOnAir)
			upmove = kSwimUpDrive;

		if (upmove == 0.f && drive == 0.f)
			upmove = 1.f;

		bot.moveForward = drive;
		bot.moveSide = 0.f;

		Bot_RunPlayerMove(bot, angles, drive, 0.f, 0, false, upmove);
		return true;
	}
}

void Bot_MoveNav(CBot &bot)
{
	edict_t *pEdict = bot.pEdict;
		const Nav::NavMesh *mesh = Nav::Get();
		if (!mesh)
			return;

		if (bot.hasLastOrigin && !bot.rideActive && !bot.fun.aboard &&
			(pEdict->v.origin - bot.lastOrigin).Length() > kTeleportJumpThreshold)
			ResetPath(bot);
	bot.lastOrigin = pEdict->v.origin;
	bot.hasLastOrigin = true;

	const Nav::Area *curArea = nullptr;
	{
		BOTPERF_SCOPE(ZONE_FINDAREA);
		curArea = Nav::FindAreaNearest(*mesh, pEdict->v.origin);
	}
	bot.currentAreaId = curArea ? static_cast<int>(curArea->id) : -1;

	{
		BOTPERF_SCOPE(ZONE_SENSE);
		BotSense::Update(bot);
	}
	{
		BOTPERF_SCOPE(ZONE_FIRE);
		BotFire::Think(bot);
	}

	{
		BOTPERF_SCOPE(ZONE_COMBAT);
		BotCombat::Update(bot);
	}

	{
		BOTPERF_SCOPE(ZONE_MEMOS);
		BotItems::RefreshMemos(bot);
	}

	{
		BOTPERF_SCOPE(ZONE_BREAK);
		if (BotBreak::Run(bot))
			return;
	}

	if (!Bot_ManeuverActive(bot))
	{
		BOTPERF_SCOPE(ZONE_DEPLOY_TRY);
		BotDeploy::TryStart(bot, *mesh);
	}

	{
		BOTPERF_SCOPE(ZONE_DEPLOY_RUN);
		if (BotDeploy::Run(bot))
			return;
	}

	if (BotFun::Run(bot))
		return;

	{
		BOTPERF_SCOPE(ZONE_COMBAT);
		if (BotCombat::Run(bot, *mesh))
			return;
	}

	{
		BOTPERF_SCOPE(ZONE_ITEM_GOAL);
		if (BotItems::UpdateGoalItem(bot))
		{
			BotSense::ApplyLook(bot);
			Vector angles = Bot_UpdateFacing(bot);
			RunIdleMove(bot, angles);
			return;
		}
	}

	{
		BOTPERF_SCOPE(ZONE_ITEM_SELECT);
		if (!bot.hasGoal && !BotGoal::Select(bot, *mesh))
		{
			BotSense::ApplyLook(bot);
			Vector angles = Bot_UpdateFacing(bot);
			RunIdleMove(bot, angles);
			return;
		}
	}

	if (bot.trackState.progressS > bot.lastProgressS + kProgressEpsilon)
	{
		bot.lastProgressS = bot.trackState.progressS;
		bot.lastProgressTime = gpGlobals->time;
	}

	bool swimming = pEdict->v.waterlevel >= kWaterlevelSwimming &&
		bot.waypointIndex < bot.waypoints.size();

	bool forcedRepath = RepathDue(bot) && !swimming;
	bool stalled = !swimming && !bot.waypoints.empty() && bot.lastProgressTime > 0.f &&
		gpGlobals->time - bot.lastProgressTime > kRepathInterval;
	if (stalled && !forcedRepath && gpGlobals->time < bot.avoid.blockedUntil)
	{
		BotGoal::OnBlocked(bot, bot.avoid.lastHit);
		BotSense::ApplyLook(bot);
		Vector angles = Bot_UpdateFacing(bot);
		RunIdleMove(bot, angles);
		return;
	}

	if (bot.waypoints.empty() || forcedRepath || stalled)
	{
		if (!TryRepath(bot, *mesh, pEdict, forcedRepath ? "forced" : "stalled"))
			return;
	}

	if (curArea && (bot.visitedAreaPath.empty() || bot.visitedAreaPath.back() != curArea->id))
	{
		RewardEnteredConnection(bot, curArea->id);
		bot.visitedAreaPath.push_back(curArea->id);
	}

	{
		BOTPERF_SCOPE(ZONE_TRACELOG);
		if (bot.traceLog.size() < 100000)
			bot.traceLog.push_back(pEdict->v.origin);
	}

	if (bot.airmoveActive)
	{
		if (RunAirmoveFlight(bot, *mesh))
			return;
		bot.trackState.started = false;
		if (RepathDue(bot) && !TryRepath(bot, *mesh, pEdict, "airmove landed off-plan"))
			return;
	}

	if (bot.ladderActive)
	{
		if (RunLadderMove(bot, *mesh))
			return;
		bot.trackState.started = false;
		if (RepathDue(bot) && !TryRepath(bot, *mesh, pEdict, "ladder finished off-plan"))
			return;
	}

	if (bot.rideActive)
	{
		if (RunElevatorRide(bot, *mesh))
			return;
		if (RepathDue(bot) && !TryRepath(bot, *mesh, pEdict, "elevator ride ended off-plan"))
			return;
	}

	if (RunSwimMove(bot))
	{
		bot.trackState.started = false;
		return;
	}

	BotPerf::Enter(BotPerf::ZONE_OPPO);
	bool oppoJumped = BotOppo::TryJump(bot, *mesh);
	BotPerf::Leave(BotPerf::ZONE_OPPO);

	if (oppoJumped)
	{
		RunAirmoveDrive(bot, Bot_YawDir(bot.airmoveYaw), bot.airmoveSpeed, bot.airmoveYaw,
			bot.airmoveCrouch, true, true);
		return;
	}

	BotPerf::Enter(BotPerf::ZONE_DRIVE);
	PmSim::PmPhysics physics = Bot_DrivePhysics(pEdict);
	BotPerf::Leave(BotPerf::ZONE_DRIVE);

	size_t activationWp = NextActivation(bot);
	while (activationWp < bot.waypoints.size() && TriggerTouched(bot, bot.waypoints[activationWp]))
	{
		FinishActivation(bot, bot.waypoints[activationWp], "touched");
		activationWp = NextActivation(bot);
	}

	if (activationWp >= bot.waypoints.size())
		bot.activationActive = false;

	NavFollow::Config followCfg;
	followCfg.driftVelocity = Bot_DriftVelocity(pEdict, bot.pushFields);
	size_t rideWp = bot.waypoints.size();
	for (size_t i = 0; i < bot.waypoints.size(); ++i)
	{
		const Nav::Waypoint &step = bot.waypoints[i];
		bool gate = step.requiresButtonUse || step.requiresTriggerTouch;
		bool barrier = step.requiresElevatorRide ||
			((step.requiresLadder || step.airmoveLaunch) && i >= bot.waypointIndex);
		if (!gate && !barrier)
			continue;

		if (gate && i < activationWp)
		{
			followCfg.sFloor = NavPlan::ProfileWaypointS(bot.followProfile, i);
			continue;
		}

		followCfg.sLimit = NavPlan::ProfileWaypointS(bot.followProfile, i);
		if (step.requiresElevatorRide)
			rideWp = i;
		break;
	}

	if (bot.rideFloorWp > 0 && bot.rideFloorWp < bot.waypoints.size())
		followCfg.sFloor = std::max(followCfg.sFloor,
			NavPlan::ProfileWaypointS(bot.followProfile, bot.rideFloorWp));

	BotPerf::Enter(BotPerf::ZONE_FOLLOW);
	NavFollow::Decision follow = NavFollow::Track(bot.followProfile, pEdict->v.origin,
		pEdict->v.velocity, physics, bot.trackState, followCfg);
	BotPerf::Leave(BotPerf::ZONE_FOLLOW);

	if (follow.wpIndex < bot.waypoints.size())
		bot.waypointIndex = follow.wpIndex;

	if (follow.done)
	{
		if (BotGoal::StopTier(bot) == BotGoal::GOAL_FUN)
		{
			if (BotFun::RunArrival(bot))
				return;
		}
		else
		{
			if (ItemKindIsCharger(bot.goalItemKind) && RunChargerUse(bot))
				return;

			if (ItemKindIsVolume(bot.goalItemKind) && RunHealVolume(bot))
				return;
		}

		Vector miss = pEdict->v.origin - bot.goalPoint;
		char msg[224];
		snprintf(msg, sizeof(msg),
			"CrossBot: %s reached goal (%.1f,%.1f,%.1f) standing at (%.1f,%.1f,%.1f), %.1f out\n", bot.name,
			bot.goalPoint.x, bot.goalPoint.y, bot.goalPoint.z, pEdict->v.origin.x, pEdict->v.origin.y,
			pEdict->v.origin.z, miss.Length());
		SERVER_PRINT(msg);

		WriteTraceLog(bot);
		BotGoal::OnArrived(bot);
		return;
	}

	if (follow.needRepath && bot.nextRepathTime <= 0.f)
	{
		BOT_VLOG("CrossBot: %s strayed %.1f from the plan at (%.1f,%.1f,%.1f), repathing\n", bot.name,
			follow.lateral, pEdict->v.origin.x, pEdict->v.origin.y, pEdict->v.origin.z);
		bot.nextRepathTime = gpGlobals->time;
	}

	if (bot.waypointIndex >= bot.waypoints.size())
	{
		BotSense::ApplyLook(bot);
		Vector angles = Bot_UpdateFacing(bot);
		RunIdleMove(bot, angles);
		return;
	}

	bool ladderEject = false;
	if (OnLadder(pEdict))
	{
		ladderEject = true;
		size_t limit = std::min(bot.waypointIndex + 3, bot.waypoints.size());
		for (size_t i = bot.waypointIndex; i < limit; ++i)
		{
			if (!bot.waypoints[i].requiresLadder)
				continue;
			bot.waypointIndex = i;
			ladderEject = false;
			break;
		}

		LADDER_DBG(bot, "eject", "wp=%zu/%zu scan=[%d,%d,%d] eject=%d org=(%.1f,%.1f,%.1f)",
			bot.waypointIndex, bot.waypoints.size(),
			(bot.waypointIndex + 0 < bot.waypoints.size())
				? (bot.waypoints[bot.waypointIndex + 0].requiresLadder ? 1 : 0) : -1,
			(bot.waypointIndex + 1 < bot.waypoints.size())
				? (bot.waypoints[bot.waypointIndex + 1].requiresLadder ? 1 : 0) : -1,
			(bot.waypointIndex + 2 < bot.waypoints.size())
				? (bot.waypoints[bot.waypointIndex + 2].requiresLadder ? 1 : 0) : -1,
			ladderEject ? 1 : 0, pEdict->v.origin.x, pEdict->v.origin.y, pEdict->v.origin.z);
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
			BOT_VLOG("CrossBot: %s falling at (%.1f,%.1f,%.1f) heading to wp %zu/%zu (%.1f,%.1f,%.1f)\n", bot.name,
				pEdict->v.origin.x, pEdict->v.origin.y, pEdict->v.origin.z, bot.waypointIndex,
				bot.waypoints.size(), wp->point.x, wp->point.y, wp->point.z);
			bot.fallTelemetryLogged = true;
		}
	}

	if (wp->airmoveLaunch)
	{
		RunAirmoveApproach(bot, *mesh, *wp);
		return;
	}

	if (wp->requiresLadder && RunLadderApproach(bot, *wp))
		return;

	if (rideWp < bot.waypoints.size() && RunElevatorApproach(bot, rideWp))
		return;

	if (activationWp < bot.waypoints.size() && RunActivation(bot, activationWp))
		return;

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

	BotPerf::Enter(BotPerf::ZONE_LOOK);
	BotSense::ApplyLook(bot);

	Vector angles = Bot_UpdateFacing(bot);
	BotPerf::Leave(BotPerf::ZONE_LOOK);

	BotAvoid::Decision avoid;
	if (BotAvoid::Enabled())
	{
		BOTPERF_SCOPE(ZONE_AVOID);
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

	if (avoid.mineTarget && BotBreak::EngageStandoff(bot, avoid.mineTarget, kMineSafeRadius))
	{
		RunIdleMove(bot, angles);
		return;
	}

	if (avoid.breakTarget && BotBreak::Engage(bot, avoid.breakTarget))
	{
		RunIdleMove(bot, angles);
		return;
	}

	if (avoid.searchFailed && !avoid.hitRecedes && Bot_PenalizeCurrentConnection(bot))
		bot.nextRepathTime = gpGlobals->time;

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
	if (ladderEject || (avoid.jump && !OnLadder(pEdict)))
		buttons |= IN_JUMP;

	if (wp->requiresLadder || OnLadder(pEdict))
		LADDER_DBG(bot, "walk",
			"wp=%zu ladderWp=%d fly=%d grnd=%d duck=%d fmove=%.0f smove=%.0f btn=%s "
			"ang=(%.1f,%.1f) org=(%.1f,%.1f,%.1f) vel=(%.0f,%.0f,%.0f) eject=%d avoidJump=%d",
			bot.waypointIndex, wp->requiresLadder ? 1 : 0, OnLadder(pEdict) ? 1 : 0,
			(pEdict->v.flags & FL_ONGROUND) != 0 ? 1 : 0,
			(pEdict->v.flags & FL_DUCKING) != 0 ? 1 : 0, fmove, smove, LadderButtonNames(buttons),
			angles.x, angles.y, pEdict->v.origin.x, pEdict->v.origin.y, pEdict->v.origin.z,
			pEdict->v.velocity.x, pEdict->v.velocity.y, pEdict->v.velocity.z, ladderEject ? 1 : 0,
			avoid.jump ? 1 : 0);

	Bot_RunPlayerMove(bot, angles, fmove, smove, buttons);
}
