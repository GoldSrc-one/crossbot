#include <extdll.h>
#include <meta_api.h>
#include <in_buttons.h>

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
		// Loose arrival radius for elevator/button/door approach; plain ground waypoints use shared steering controller (path_follower.h).
		constexpr float kWaypointReachedDistance = 24.f;
		// Jump only when close and facing within 20°; rate-limited turn smoothing (Bot_UpdateFacing) must catch up first.
		constexpr float kJumpTriggerDistance = 100.f;
		constexpr float kJumpFacingToleranceDeg = 20.f;
		// Door waypoint aims at door center; radius bigger than kWaypointReachedDistance so bot's hull touches brush and triggers touch-to-open.
		constexpr float kDoorWaitTriggerDistance = 64.f;

		// GoldSrc hull convention duplicated here per codebase convention (tools/navgen/src/hull_defs.h has its own copy).
		constexpr int kHullHuman = 1;
		constexpr int kHullPoint = 0;
		// Dodge candidate needs ground under endpoint; clear trace over a pit is clear because there's nothing there.
		constexpr float kDodgeFloorProbeDepth = 90.f;
		// Shorter than Bot_MoveWander's 64u since nav mode has a real heading to check.
		constexpr float kObstacleTraceDistance = 48.f;
		// If dodge hasn't cleared obstacle by this delay, force early repath (may be permanent blockage).
		constexpr float kObstacleStuckRepathDelay = 1.5f;

	float NormalizeAngle(float a)
	{
		while (a > 180.f) a -= 360.f;
		while (a < -180.f) a += 360.f;
		return a;
	}

	// Tries fan of headings around `dir2D`; returns clearest hull trace, closest-to-original first. Replaces single-perpendicular dodge which could point into a second wall (confirmed on crossfire.bsp: bot wedged in right-angle corner computed dodge off first wall into second, freezing).
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

			// A clear (or clearest) forward trace is NOT enough: over a pit edge the trace is clear because there's nothing there. Require ground under the candidate's reach point, or the dodge itself becomes the fall.
				Vector reach = pEdict->v.origin + candidate * (traceDistance * tr.flFraction);
			TraceResult down;
			g_engfuncs.pfnTraceHull(reach, reach - Vector(0.f, 0.f, kDodgeFloorProbeDepth), ignore_monsters,
				kHullPoint, pEdict, &down);
			if (down.flFraction >= 1.0f)
				continue; // leads over open air -- never dodge that way

			if (tr.flFraction >= 1.0f)
				return candidate; // fully clear -- good enough, stop looking

			if (tr.flFraction > bestFraction)
			{
				bestFraction = tr.flFraction;
				best = candidate;
			}
		}

		return best; // nothing fully clear (or nothing floored) -- least-bad candidate
	}
	// Threshold > max single-tick travel; discontinuous jump means respawn/teleport, invalidate path.
		constexpr float kTeleportJumpThreshold = 150.f;
		// Periodic safety repath; longer interval gives tie-flopping corridors more time to settle.
		constexpr float kRepathInterval = 10.f;
		// Straight-line distance/speed is a floor; margin corrects for turns, jumps, avoidance not in estimate.
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
		bot.nextRepathTime = 0.f; // 0 = no forced repath scheduled (see Bot_MoveNav's repath condition)
	}

	// Dumps the per-tick origin recording (see CBot::traceLog) for offline
	// comparison: navgen --dump-svg --trace <file> overlays it on the debug
	// SVG next to the planned line and the momentum simulation. Written
	// whenever a goal ends, reached or dropped, then cleared.
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
		// Captured before bot.waypoints gets overwritten below: true only for
		// the first Repath since the goal was (re)assigned -- a later
		// mid-route safety repath (obstacle-triggered or the periodic
		// kRepathInterval one) finds bot.waypoints still non-empty, so it
		// refreshes the corridor/ETA-inputs without pushing the original
		// arrival deadline back out. See bot.h's field comments.
		bool freshAttempt = bot.waypoints.empty();

		const Nav::Area *startArea = Nav::FindAreaForPoint(mesh, pEdict->v.origin);
		const Nav::Area *goalArea = Nav::FindAreaForPoint(mesh, bot.goalPoint);
		if (!startArea || !goalArea)
			return false;

		Nav::PathResult path = Nav::FindPath(mesh, startArea->id, goalArea->id);
		if (!path.found)
			return false;

		bot.waypoints = Nav::BuildWaypointPath(mesh, path, pEdict->v.origin, bot.goalPoint);
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
		return; // Bot_Move only calls us when Nav::IsLoaded(), but stay safe

	if (bot.hasLastOrigin && (pEdict->v.origin - bot.lastOrigin).Length() > kTeleportJumpThreshold)
		ResetPath(bot); // discontinuous jump -- the old path is meaningless now
	bot.lastOrigin = pEdict->v.origin;
	bot.hasLastOrigin = true;

	// Diagnostics: refreshed every tick regardless of goal state, so
	// crossbot_nav_status always reflects the bot's real current area even
	// between goals.
	const Nav::Area *curArea = Nav::FindAreaForPoint(*mesh, pEdict->v.origin);
	bot.currentAreaId = curArea ? static_cast<int>(curArea->id) : -1;

	if (!bot.hasGoal)
		{
			// No goal -> stand idle. Random wandering caused bots to stroll off ledges before scripted test goals arrived, burying real arrivals in log spam.
			Vector angles = Bot_UpdateFacing(bot);
		g_engfuncs.pfnRunPlayerMove(pEdict, angles, 0.f, 0.f, 0.f, 0, 0, Bot_FrameMsec());
		return;
	}

	// Suppressed while waiting for/riding elevator, holding at button/trigger/door (bot.doorWaitUntil), or walking toward unconsumed activation waypoint. BuildWaypointPath has no "already fired" memory, so repath before crossing re-inserts same trigger waypoint and bot walks back -- confirmed on test20_doortrigger.bsp: trigger far from spawn/door, mid-walk periodic repath reinserted it forever.
		bool hasPendingActivation = false;
	for (size_t wi = bot.waypointIndex; wi < bot.waypoints.size(); ++wi)
	{
		if (bot.waypoints[wi].requiresButtonUse || bot.waypoints[wi].requiresTriggerTouch)
		{
			hasPendingActivation = true;
			break;
		}
	}
	// Waypoint-completion progress tracking (stall detection for the safety
	// repath below -- see CBot::lastProgressTime).
	if (static_cast<int>(bot.waypointIndex) != bot.lastSeenWaypointIndex)
	{
		bot.lastSeenWaypointIndex = static_cast<int>(bot.waypointIndex);
		bot.lastProgressTime = gpGlobals->time;
	}

	// Mid-route repath only when something is actually wrong: obstacle-dodge forced it, or no waypoint completed for kRepathInterval (stall). Old blind periodic timer repathed even while hitting every waypoint -- rebuilding mid-segment moved pursuit line out from under bot threading precise geometry (see CBot::lastProgressTime for confirmed test24_precise2 case).
		bool forcedRepath = bot.nextRepathTime > 0.f && bot.nextRepathTime <= gpGlobals->time;
	bool stalled = !bot.waypoints.empty() && bot.lastProgressTime > 0.f &&
		gpGlobals->time - bot.lastProgressTime > kRepathInterval;
	if (bot.elevatorReadyTime <= 0.f && bot.doorWaitUntil <= 0.f && !hasPendingActivation &&
		(bot.waypoints.empty() || forcedRepath || stalled))
	{
		if (!bot.waypoints.empty())
		{
			// Mid-route repath replaces corridor from current position on; meaningful event for post-hoc trajectory correlation.
			char msg[160];
			snprintf(msg, sizeof(msg), "CrossBot: %s mid-route repath (%s) at (%.1f,%.1f,%.1f) wp %zu/%zu\n",
				bot.name, forcedRepath ? "obstacle-forced" : "stalled", pEdict->v.origin.x, pEdict->v.origin.y,
				pEdict->v.origin.z, bot.waypointIndex, bot.waypoints.size());
			SERVER_PRINT(msg);
		}
		if (!Repath(bot, *mesh, pEdict))
		{
			// Goal unreachable, or a transient point-in-area miss -- drop
			// it and let the next frame pick a fresh one rather than
			// getting stuck retrying the same failing repath forever.
			WriteTraceLog(bot);
			bot.hasGoal = false;
			return;
		}
		bot.nextRepathTime = 0.f; // no scheduled repath -- only forced (obstacle) or stall triggers another
		bot.lastProgressTime = gpGlobals->time;
		bot.lastSeenWaypointIndex = static_cast<int>(bot.waypointIndex);
	}

	// Append to the actual-path trace whenever the bot's real area changes
	// (not every tick -- that would just repeat the same id while standing
	// still or mid-jump over one area). Repath() above already seeds this
	// with the start area on a fresh attempt, so this only ever appends
	// *new* entries after that.
	if (curArea && (bot.visitedAreaPath.empty() || bot.visitedAreaPath.back() != curArea->id))
		bot.visitedAreaPath.push_back(curArea->id);

	// Per-tick origin recording while the goal is active (see
	// CBot::traceLog) -- flushed to disk by WriteTraceLog when the goal
	// ends. Capped defensively; at ~100 ticks/sec this covers well over ten
	// minutes of continuous driving, far past any test's timeout.
	if (bot.traceLog.size() < 100000)
		bot.traceLog.push_back(pEdict->v.origin);

	if (bot.waypointIndex >= bot.waypoints.size())
	{
		// Permanent (not debug-only) telemetry: an external test driver
		// (scripts/nav_target_test.ps1) greps qconsole.log for this exact
		// line rather than racing live rcon polls against
		// crossbot_nav_status -- a short goal can complete well within one
		// poll interval, so the "goal=" field a poll observes may already
		// be gone by the time the poll lands.
		char msg[128];
		snprintf(msg, sizeof(msg), "CrossBot: %s reached goal (%.1f,%.1f,%.1f)\n", bot.name, bot.goalPoint.x,
			bot.goalPoint.y, bot.goalPoint.z);
		SERVER_PRINT(msg);

		WriteTraceLog(bot);
		bot.hasGoal = false; // reached the goal
		ResetPath(bot);
		return;
	}

	const Nav::Waypoint &waypoint = bot.waypoints[bot.waypointIndex];
	Vector toTarget = waypoint.point - pEdict->v.origin;
	float horizDist = sqrtf(toTarget.x * toTarget.x + toTarget.y * toTarget.y);

	// Fall telemetry: log origin first frame of unplanned airborne drop (not a jump waypoint's hop). RCON polling too slow -- by next poll bot's at bottom with momentum gone.
	{
		bool onGroundNow = (pEdict->v.flags & FL_ONGROUND) != 0;
		if (onGroundNow)
		{
			bot.fallTelemetryLogged = false;
		}
		else if (!bot.fallTelemetryLogged && pEdict->v.velocity.z < -120.f && !waypoint.requiresJump)
		{
			char fallMsg[192];
			snprintf(fallMsg, sizeof(fallMsg),
				"CrossBot: %s falling at (%.1f,%.1f,%.1f) heading to wp %zu/%zu (%.1f,%.1f,%.1f)\n", bot.name,
				pEdict->v.origin.x, pEdict->v.origin.y, pEdict->v.origin.z, bot.waypointIndex,
				bot.waypoints.size(), waypoint.point.x, waypoint.point.y, waypoint.point.z);
			SERVER_PRINT(fallMsg);
			bot.fallTelemetryLogged = true;
		}
	}

	if (waypoint.requiresElevatorRide)
		{
			// Not at boarding point yet -- walk there; no avoidance/jump logic (committing to lift shouldn't get nudged sideways).
			if (horizDist >= kWaypointReachedDistance)
			{
				bot.desiredAngles = Vector(0.f, atan2f(toTarget.y, toTarget.x) * (180.f / kPi), 0.f);
				Vector angles = Bot_UpdateFacing(bot);
				g_engfuncs.pfnRunPlayerMove(pEdict, angles, kMoveSpeed, 0.f, 0.f, IN_FORWARD, 0, Bot_FrameMsec());
				return;
			}

			// At boarding point -- hold still; engine's ground-entity attachment carries bot up/down like a player on moving platform.
			if (bot.elevatorReadyTime <= 0.f)
				bot.elevatorReadyTime = gpGlobals->time + waypoint.elevatorTravelTime;

			Vector angles = Bot_UpdateFacing(bot);
			g_engfuncs.pfnRunPlayerMove(pEdict, angles, 0.f, 0.f, 0.f, 0, 0, Bot_FrameMsec());

			if (gpGlobals->time >= bot.elevatorReadyTime)
			{
				bot.elevatorReadyTime = 0.f;
				++bot.waypointIndex;
			}
			return;
		}

		if (waypoint.requiresButtonUse)
			{
				if (horizDist >= kWaypointReachedDistance)
				{
					// Walk to button normally; same approach-then-commit logic as other waypoints.
					bot.desiredAngles = Vector(0.f, atan2f(toTarget.y, toTarget.x) * (180.f / kPi), 0.f);
					Vector angles = Bot_UpdateFacing(bot);
					g_engfuncs.pfnRunPlayerMove(pEdict, angles, kMoveSpeed, 0.f, 0.f, IN_FORWARD, 0, Bot_FrameMsec());
					return;
				}

				// At button -- face it and hold +USE. CBaseButton without SF_BUTTON_TOUCH_ONLY needs PlayerUse's forward-facing search; hold for buttonWaitTime so relay chain fires door before bot walks toward it.
				bot.desiredAngles = Vector(0.f, atan2f(toTarget.y, toTarget.x) * (180.f / kPi), 0.f);
				Vector angles = Bot_UpdateFacing(bot);
				g_engfuncs.pfnRunPlayerMove(pEdict, angles, 0.f, 0.f, 0.f, IN_USE, 0, Bot_FrameMsec());

				if (bot.doorWaitUntil <= 0.f)
			bot.doorWaitUntil = gpGlobals->time + waypoint.buttonWaitTime;

		if (gpGlobals->time >= bot.doorWaitUntil)
		{
			bot.doorWaitUntil = 0.f;
			++bot.waypointIndex;
		}
		return;
	}

	if (waypoint.requiresDoor)
	{
		if (horizDist >= kDoorWaitTriggerDistance)
		{
			// Not close enough to have triggered door -- walk normally, no avoidance (closed door reads as solid obstacle; dodging it defeats the waypoint's purpose).
					bot.desiredAngles = Vector(0.f, atan2f(toTarget.y, toTarget.x) * (180.f / kPi), 0.f);
					Vector angles = Bot_UpdateFacing(bot);
					g_engfuncs.pfnRunPlayerMove(pEdict, angles, kMoveSpeed, 0.f, 0.f, IN_FORWARD, 0, Bot_FrameMsec());
					return;
				}

				// Close enough to have triggered touch-to-open -- hold still instead of contesting still-moving brush collision (test9_door: walking straight at it fought door's collision, deflected bot back out of doorway in infinite loop).
				if (bot.doorWaitUntil <= 0.f)
					bot.doorWaitUntil = gpGlobals->time + waypoint.doorWaitTime;

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
	bool needsJump = !onLadder && waypoint.requiresJump;

	if (needsJump)
		{
			// Jump waypoint landing point often within generic reached-dist of takeoff -> would complete before bot leaves ground. Require actual jump pressed (bot.jumpAttempted) and bot back on ground in target area (via FindAreaForPoint, not just distance to centroid -- centroid can be deep in room behind obstructions).
			bool onGround = (pEdict->v.flags & FL_ONGROUND) != 0;
			bool inTargetArea = waypoint.targetAreaId < 0;
			if (!inTargetArea)
			{
				const Nav::Area *actualArea = Nav::FindAreaForPoint(*mesh, pEdict->v.origin);
				inTargetArea = actualArea && actualArea->id == static_cast<uint32_t>(waypoint.targetAreaId);
			}
			if (bot.jumpAttempted && onGround && inTargetArea)
			{
				bot.jumpAttempted = false;
				++bot.waypointIndex;
				return;
			}
		}

	// Shared steering (waypoint switching, pursuit aim, plan-encoded speed cap) -- see path_follower.h. Only for plain ground; jump/ladder branches have own aiming.
		NavFollow::Decision steer;
	if (!needsJump)
	{
		steer = NavFollow::Steer(bot.waypoints, bot.waypointIndex, pEdict->v.origin);
		if (steer.advance)
		{
			++bot.waypointIndex; // pick up the next waypoint next frame with a fresh direction
			return;
		}
	}

	// A jump waypoint's point is often a real walk away from the takeoff
	// edge (see the reached-check's comment above), not just the final
	// short hop -- disabling avoidance/obstacle-dodging for the *entire*
	// approach, not just the actual commit, left a bot with no way to
	// route around anything in that straight line. Confirmed live on
	// crossfire.bsp: a bot 139 units from a jump waypoint (well outside
	// kJumpTriggerDistance) walked straight into a wall and never moved
	// again -- onground=1, vel exactly (0,0,0), forever, because the old
	// `needsJump` branch skipped NavAvoid::Deflect and the obstacle-trace
	// dodge unconditionally for the whole approach. Only actually committing
	// to the jump (within kJumpTriggerDistance) still wants the straight,
	// undeflected beeline -- that part of the original reasoning holds.
	bool committingToJump = needsJump && horizDist < kJumpTriggerDistance;

	if (onLadder)
	{
		// GoldSrc ladder-touch movement reads climb direction from view
		// pitch combined with forward-move input -- the same technique
		// reference bots use (e.g. jk_botti's BotOnLadder): look up to
		// climb up, down to climb down, hold forward either way.
		float climbPitch = (toTarget.z > 0.f) ? -60.f : 60.f;
		bot.desiredAngles = Vector(climbPitch, atan2f(toTarget.y, toTarget.x) * (180.f / kPi), 0.f);
	}
	else if (committingToJump)
	{
		// No avoidance deflection while actually lined up to commit -- this
		// last short stretch shouldn't get nudged sideways by a nearby bot.
		bot.desiredAngles = Vector(0.f, atan2f(toTarget.y, toTarget.x) * (180.f / kPi), 0.f);
	}
	else
	{
		// The shared controller already picked the pure-pursuit aim point on
		// the planned segment (and the plan-encoded speed cap) -- see
		// path_follower.h. This branch layers the engine-side reactions on
		// top: avoidance deflection and the obstacle feeler.
		Vector aimTarget = steer.aimPoint - pEdict->v.origin;

		// Avoidance for ground steering only -- deflecting mid-climb would fight ladder alignment.
				Vector steerDir = NavAvoid::Deflect(bot, aimTarget);

				// Trace ahead for dynamic obstacles nav mesh doesn't know about; dodge if hit.
				float dirLen2D = sqrtf(steerDir.x * steerDir.x + steerDir.y * steerDir.y);
				if (dirLen2D > 1.f)
				{
					Vector dir2D(steerDir.x / dirLen2D, steerDir.y / dirLen2D, 0.f);
					Vector traceEnd = pEdict->v.origin + dir2D * kObstacleTraceDistance;

					TraceResult tr;
					g_engfuncs.pfnTraceHull(pEdict->v.origin, traceEnd, dont_ignore_monsters, kHullHuman, pEdict, &tr);

					// Feelers catch dynamic entities (players, monsters, pushables), NOT worldspawn. Static brushes are already routed around; worldspawn hit must NOT dodge (tight neck: feeler brushes wall every tick, old dodge deflected toward "clear" side -- which was open air over pit).
					bool hitDynamicObstacle = tr.flFraction < 1.0f && tr.pHit &&
						g_engfuncs.pfnIndexOfEdict(tr.pHit) != 0;

			if (hitDynamicObstacle)
			{
				if (bot.obstacleBlockedSince <= 0.f)
					bot.obstacleBlockedSince = gpGlobals->time;

				steerDir = FindClearDirection(pEdict, dir2D, kObstacleTraceDistance);

				if (gpGlobals->time - bot.obstacleBlockedSince > kObstacleStuckRepathDelay)
					bot.nextRepathTime = gpGlobals->time; // this route may not be viable anymore -- get a fresh one now
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

	// Hold duck through a NAV_AREA_CROUCH area -- hull-3-only clearance
	// somewhere in it (edge_tagger.cpp's ProbeCrouchLinks/main.cpp's hull-3
	// leaf extraction), too tight for the standing hull to fit.
	// waypoint.requiresCrouch (path_funnel.cpp) already covers the area on
	// either side of this specific crossing; also checking curArea directly
	// covers staying ducked for the crouch area's *own* full footprint, not
	// just right at its border waypoint.
	if (waypoint.requiresCrouch || (curArea && (curArea->areaFlags & crossbot_nav::NAV_AREA_CROUCH)))
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

		// Stock HL duck-jump: same impulse, only shrinks hull -- clearance for tight gaps, not height boost. Long-jump module (GameProfile/slj) is separate.
			if (pEdict->v.velocity.z > 0.f)
				buttons |= IN_DUCK;
	}

	// Speed: shared controller's plan-encoded cap for plain ground; heading-error scaling (turn/creep/run -- see path_follower.h Config). Ladder exempt: facing pitches steeply, MOVETYPE_FLY slow already. Jump commit exempt: IN_JUMP only fires within kJumpFacingToleranceDeg, at/below slowdown threshold, so run-up keeps full speed.
		float moveSpeed = (!needsJump && !onLadder) ? steer.baseSpeed : kMoveSpeed;
	if (!onLadder)
	{
		float headingError = fabsf(NormalizeAngle(angles.y - bot.desiredAngles.y));
		moveSpeed = NavFollow::ScaleSpeedByHeadingError(moveSpeed, headingError);
	}

	g_engfuncs.pfnRunPlayerMove(pEdict, angles, moveSpeed, 0.f, 0.f, buttons, 0, Bot_FrameMsec());
}
