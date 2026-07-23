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
	// Loose arrival radius for the elevator/button/door approach branches
	// below. Plain ground waypoints don't use this anymore -- their
	// switching (tight corner radius + passed-the-perpendicular test),
	// pure-pursuit aiming, and speed caps all live in the shared steering
	// controller (src/nav/path_follower.h, NavFollow::Steer), the same code
	// tools/navgen's offline movement simulation runs. The hard-won
	// reasoning behind each rule (thin-beam corner cuts, drift bows,
	// V-turn half-space traps -- all confirmed live on test23_precise/
	// test24_precise2) is documented there.
	constexpr float kWaypointReachedDistance = 24.f;
	// Distance/facing gates before committing to a jump, matching FoxBot's
	// precedent (bot_navigate.cpp): jump only once reasonably close and
	// roughly lined up, rather than jk_botti's simpler "press jump the
	// instant you're within a small radius" -- CrossBot already does
	// rate-limited turn smoothing (Bot_UpdateFacing), so jumping before
	// that smoothing has caught up would send the bot off at the wrong
	// angle.
	constexpr float kJumpTriggerDistance = 100.f;
	constexpr float kJumpFacingToleranceDeg = 20.f;
	// How close counts as "made contact" with a func_door/func_door_rotating
	// waypoint -- bigger than kWaypointReachedDistance since a door waypoint
	// aims at the door's own center, and the goal here is "close enough for
	// the bot's hull to be touching the brush and have triggered its
	// touch-to-open," not "arrived at a point." A door's own footprint runs
	// tens of units across, so a tight radius could read "not there yet"
	// while the bot is already pressed up against it.
	constexpr float kDoorWaitTriggerDistance = 64.f;

	// GoldSrc hull convention (dlls/extdef.h): point_hull=0, human_hull=1,
	// large_hull=2, head_hull=3. Duplicated here rather than sharing a
	// constant across files, matching this codebase's existing convention
	// (see tools/navgen/src/hull_defs.h's own independent copy).
	constexpr int kHullHuman = 1;
	constexpr int kHullPoint = 0;
	// How far below a dodge candidate's endpoint ground must exist for the
	// dodge to be allowed (FindClearDirection) -- a clear forward trace over
	// a pit is clear precisely BECAUSE there's nothing there.
	constexpr float kDodgeFloorProbeDepth = 90.f;
	// Baked nav data only reflects static geometry from whenever navgen ran
	// -- a player-built structure, a monster, or any other solid entity
	// placed at runtime isn't in it at all, so the waypoint direction can't
	// just be trusted blindly. Shorter lookahead than Bot_MoveWander's 64u
	// since nav mode already has a real heading to check, not explore
	// blindly.
	constexpr float kObstacleTraceDistance = 48.f;
	// If a dodge hasn't cleared the obstacle by this long, stop waiting on
	// the periodic repath interval and get a fresh route now -- a dodge
	// that isn't working means this might be a permanent blockage
	// (something built there, not just passing through).
	constexpr float kObstacleStuckRepathDelay = 1.5f;

	float NormalizeAngle(float a)
	{
		while (a > 180.f) a -= 360.f;
		while (a < -180.f) a += 360.f;
		return a;
	}

	// Tries a fan of candidate headings around `dir2D` (unit-length,
	// horizontal) and returns whichever gives the clearest kObstacleTrace-
	// Distance hull trace from the bot's current origin, closest-to-original
	// first. Replaces an earlier single-perpendicular-to-the-hit-normal
	// dodge guess, which could itself point straight into a *second* nearby
	// wall -- confirmed live on crossfire.bsp: a bot wedged in a right-angle
	// corner (two walls, each exactly one hull-radius away, on adjacent
	// sides) computed a dodge off the first wall that pointed directly into
	// the second, leaving it completely frozen (zero velocity, zero
	// displacement) instead of escaping.
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

			// A clear (or clearest) forward trace is NOT enough: over a pit
			// edge the trace is clear precisely because there's nothing
			// there. Require actual ground under the point this candidate
			// would reach, or the dodge itself becomes the fall -- confirmed
			// live on test24_precise2: the feeler deflected the bot
			// "clear"-ward off a thin beam's edge, straight into the pit the
			// planned route was carefully threading around.
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
	// More than any single tick's max travel at kMoveSpeed, even at
	// Bot_FrameMsec()'s 100ms clamp ceiling (250 u/s -> 25 u/tick) -- a jump
	// this big means something moved the bot behind our back (respawn, or a
	// trigger_teleport once Milestone 2h lands), not normal movement. See
	// the plan's "Detecting respawn and teleport" note.
	constexpr float kTeleportJumpThreshold = 150.f;
	// Periodic safety re-path -- cheap at the area counts seen so far, but
	// confirmed live to have a real downside too: recomputing the whole
	// corridor (A* + funnel simplification) from scratch every interval can
	// flip-flop between two similar-cost routes near a tie, especially
	// funnel simplification's hull-trace string-pull through a narrow
	// doorway-width area, which can go either way depending on the bot's
	// exact position that tick. A bot stuck cycling through the same 4-5
	// small areas near a doorway on crossfire.bsp, actively moving the
	// whole time (not frozen) yet never making net progress, is consistent
	// with this. Longer interval doesn't fix the underlying tie, but gives
	// each attempt more time to either finish or reveal it's genuinely
	// stuck (kObstacleStuckRepathDelay's early-repath-on-blockage path
	// still fires independently of this for real obstacles).
	constexpr float kRepathInterval = 10.f;
	// Straight-line distance/kMoveSpeed is a floor, not a real estimate --
	// it can't see turning, jump run-ups/landings, or avoidance detours
	// around other bots. This margin is a rough correction factor, not a
	// measured constant; tighten it once enough live runs establish a real
	// ratio between this estimate and actual arrival times.
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
		FILE *f = fopen("valve/crossbot_data/nav_trace.txt", "w");
		if (f)
		{
			for (const Vector &p : bot.traceLog)
				fprintf(f, "%.1f %.1f %.1f\n", p.x, p.y, p.z);
			fclose(f);
			char msg[128];
			snprintf(msg, sizeof(msg), "CrossBot: wrote nav_trace.txt (%zu points)\n", bot.traceLog.size());
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
		// No goal -> stand idle. This used to pick a random area centroid to
		// wander toward, which caused constant trouble in practice: bots
		// strolled off ledges into inescapable pits before a scripted test's
		// teleport+goal even arrived, and the resulting "reached goal" spam
		// buried real arrivals in the log. A future combat/objective layer
		// is what should be assigning goals, not a coin flip here.
		Vector angles = Bot_UpdateFacing(bot);
		g_engfuncs.pfnRunPlayerMove(pEdict, angles, 0.f, 0.f, 0.f, 0, 0, Bot_FrameMsec());
		return;
	}

	// Suppressed while actively waiting for/riding an elevator, holding at a
	// button/trigger/door (bot.doorWaitUntil, shared by all three -- see
	// bot.h), or still walking toward an unconsumed button/trigger waypoint
	// later in the plan: BuildWaypointPath has no memory of "already fired,"
	// so any repath before that crossing is actually behind the bot just
	// re-inserts a fresh activation waypoint for the same portal and sends
	// it walking all the way back -- confirmed live on test20_doortrigger.bsp,
	// twice: first with only the doorWaitUntil guard (fixed the in-place-hold
	// case), but the trigger volume sits far from both spawn and the door
	// (~384 and ~163 units respectively), so the walk *to* it and the walk
	// *from* it to the door each routinely outlasted kRepathInterval (10s) on
	// their own -- a periodic repath mid-walk (not mid-hold) reinserted the
	// same trigger waypoint over and over, forever, since the crossing was
	// still ahead of the bot's waypointIndex every time. Once the bot's
	// actually past that waypoint (index advanced beyond it), a repath is
	// safe again -- by then the activation has already happened.
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

	// A mid-route repath happens only when something is actually wrong:
	// either the obstacle-dodge branch forced one (bot.nextRepathTime set to
	// "now"), or no waypoint has been completed for kRepathInterval (a
	// stall). The old blind periodic timer repathed even while the bot was
	// hitting every waypoint on schedule -- rebuilding the waypoint list
	// mid-segment moved the pursuit line out from under the bot right as it
	// threaded precise geometry (see CBot::lastProgressTime's comment for
	// the confirmed test24_precise2 case).
	bool forcedRepath = bot.nextRepathTime > 0.f && bot.nextRepathTime <= gpGlobals->time;
	bool stalled = !bot.waypoints.empty() && bot.lastProgressTime > 0.f &&
		gpGlobals->time - bot.lastProgressTime > kRepathInterval;
	if (bot.elevatorReadyTime <= 0.f && bot.doorWaitUntil <= 0.f && !hasPendingActivation &&
		(bot.waypoints.empty() || forcedRepath || stalled))
	{
		if (!bot.waypoints.empty())
		{
			// Permanent telemetry, same spirit as the fall/arrival lines: a
			// mid-route repath replaces the corridor from the current
			// position on, which is a meaningful (and historically
			// bug-prone) event worth being able to correlate with recorded
			// trajectories after the fact.
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

	// Fall telemetry: log the exact origin the first frame of an unplanned
	// airborne drop (not a jump waypoint's own deliberate hop). rcon polling
	// is far too slow to catch where a bot actually stepped off a thin
	// walkway -- by the next poll it's already at the bottom of the pit with
	// its momentum gone.
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
		if (horizDist >= kWaypointReachedDistance)
		{
			// Not at the boarding point yet -- walk there normally. No
			// avoidance/jump logic needed, same reasoning as lining up for
			// a jump: committing to the lift shouldn't get nudged sideways.
			bot.desiredAngles = Vector(0.f, atan2f(toTarget.y, toTarget.x) * (180.f / kPi), 0.f);
			Vector angles = Bot_UpdateFacing(bot);
			g_engfuncs.pfnRunPlayerMove(pEdict, angles, kMoveSpeed, 0.f, 0.f, IN_FORWARD, 0, Bot_FrameMsec());
			return;
		}

		// At the boarding point -- hold still and let the engine's own
		// ground-entity attachment carry the bot up/down, the same as a
		// real player just standing on a moving platform. No forward
		// input at all while waiting.
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
			// Not at the button yet -- walk there normally, same reasoning
			// as every other approach-then-commit waypoint.
			bot.desiredAngles = Vector(0.f, atan2f(toTarget.y, toTarget.x) * (180.f / kPi), 0.f);
			Vector angles = Bot_UpdateFacing(bot);
			g_engfuncs.pfnRunPlayerMove(pEdict, angles, kMoveSpeed, 0.f, 0.f, IN_FORWARD, 0, Bot_FrameMsec());
			return;
		}

		// At the button -- face it and hold +USE. CBaseButton only wires up
		// a Touch function when SF_BUTTON_TOUCH_ONLY is set on the entity;
		// none of the buttons door_extractor.cpp's ResolveButtonForDoor
		// resolves have that flag (the ones confirmed live on
		// test10_doorbutton/test11_doorbutton2 don't), so simply touching
		// it does nothing -- CBasePlayer::PlayerUse's own forward-facing
		// search is what actually finds and activates it, the same as a
		// real player pressing the use key while looking at it. Held for
		// buttonWaitTime (baked as the door's own open-time estimate) so
		// the relay/multi_manager chain fires and the door starts moving
		// before the bot walks toward it.
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
			// Not close enough yet to have made contact -- walk there
			// normally, no avoidance: a closed door reads as an ordinary
			// solid obstacle to the avoidance trace, and deflecting away
			// from the one obstacle the bot actually needs to touch to
			// open it would defeat the whole point of this waypoint.
			bot.desiredAngles = Vector(0.f, atan2f(toTarget.y, toTarget.x) * (180.f / kPi), 0.f);
			Vector angles = Bot_UpdateFacing(bot);
			g_engfuncs.pfnRunPlayerMove(pEdict, angles, kMoveSpeed, 0.f, 0.f, IN_FORWARD, 0, Bot_FrameMsec());
			return;
		}

		// Close enough to have made contact and triggered the door's
		// touch-to-open -- hold still rather than keep contesting the
		// still-moving brush's own collision. Confirmed live on
		// test9_door: continuing to walk straight at it the whole time
		// fought the swinging/sliding door's own collision and deflected
		// the bot back out of the doorway in a loop that never actually
		// finished going through.
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
		// A jump waypoint's landing point is often already within the
		// generic reached-distance of the takeoff point -- that's what
		// makes it a short hop rather than a long walk. Checking distance
		// alone (like every other waypoint) would mark it reached before
		// the bot ever left the ground. Confirmed live: a bot approaching
		// a lift1-side gap oscillated in place forever, vel.z always
		// exactly 0. So completion instead requires an actual jump to have
		// been pressed for this waypoint (bot.jumpAttempted, set below,
		// checked here a frame later so this can't fire the same tick it's
		// set) and the bot to be back on solid ground near the target.
		// horizDist/height-to-point isn't enough either: a jump waypoint's
		// point is just the destination area's raw centroid (see
		// path_funnel.cpp), which for a large or oddly-shaped area can be
		// a real walk away from the gap's edge, behind obstructions the
		// straight-line jump approach never accounted for. Confirmed live
		// both ways: pure horizDist<32 completed the waypoint at the lip
		// of the chasm before ever jumping; requiring the point's exact
		// height too just moved the failure to bots walking dead into a
		// wall trying to reach a centroid buried deep in the destination
		// room. The actual question is the one that matters for
		// correctness: did the bot really arrive in the destination area,
		// not "is it close to this one representative point" -- so ask
		// Nav::FindAreaForPoint about the bot's real origin instead.
		bool onGround = (pEdict->v.flags & FL_ONGROUND) != 0;
		bool inTargetArea = waypoint.targetAreaId < 0; // no area recorded -- fall back to trusting the gate below
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

	// Shared steering decision (waypoint switching, pursuit aim point, and
	// the plan-encoded speed cap) -- see path_follower.h. Only consulted for
	// plain ground movement; the jump branch above and the ladder/commit
	// branches below keep their own aiming.
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

		// Avoidance only applies to normal ground steering -- deflecting a
		// bot mid-climb would fight the ladder's precise alignment needs.
		Vector steerDir = NavAvoid::Deflect(bot, aimTarget);

		// Don't blindly trust the baked path -- trace ahead the same way
		// Bot_MoveWander already does, and dodge if something's actually in
		// the way that the nav mesh doesn't know about.
		float dirLen2D = sqrtf(steerDir.x * steerDir.x + steerDir.y * steerDir.y);
		if (dirLen2D > 1.f)
		{
			Vector dir2D(steerDir.x / dirLen2D, steerDir.y / dirLen2D, 0.f);
			Vector traceEnd = pEdict->v.origin + dir2D * kObstacleTraceDistance;

			TraceResult tr;
			g_engfuncs.pfnTraceHull(pEdict->v.origin, traceEnd, dont_ignore_monsters, kHullHuman, pEdict, &tr);

			// The feeler exists for obstacles the baked nav data can't know
			// about -- and those are entities (players, monsters, pushables,
			// func_walls...), never worldspawn itself. Static world brushes
			// are exactly what the plan already routes around, so a
			// worldspawn hit must NOT trigger a dodge: near a tight neck
			// (test24_precise2's razor beam junction, walls one hull-radius
			// away) the forward trace brushes the wall every tick, and the
			// old unconditional dodge deflected the bot toward the "clear"
			// side -- which was clear because it was the open air above the
			// pit. Recorded-trajectory overlay confirmed the eastward drift
			// beginning exactly one feeler-length before the junction.
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

		// Stock HL duck-jumping doesn't add jump velocity -- PM_Jump's
		// ducking branch produces the identical sqrt(2*800*45) impulse as
		// the non-ducking case, it only shrinks the hull (VEC_DUCK_HULL_MIN/
		// MAX). So this is purely for clearance once airborne through a
		// tight gap, the same technique FoxBot and yapb both use, not an
		// attempt to jump higher or further. (A future GameProfile-driven
		// long-jump-module upgrade, per HLSDK's item_longjump/PM_Jump "slj"
		// check, is a real distance boost -- tracked separately, not this.)
		if (pEdict->v.velocity.z > 0.f)
			buttons |= IN_DUCK;
	}

	// Speed: the shared controller's plan-encoded cap for plain ground
	// movement (jump approaches/commits keep full run-up speed), then
	// heading-error scaling (turn in place / creep / run -- see
	// path_follower.h's Config). Ladder climbing is exempt: facing
	// deliberately pitches steeply there and MOVETYPE_FLY motion is slow
	// already; a jump commit is also effectively exempt (IN_JUMP only fires
	// within kJumpFacingToleranceDeg, at or under the slowdown threshold,
	// so the run-up keeps full speed).
	float moveSpeed = (!needsJump && !onLadder) ? steer.baseSpeed : kMoveSpeed;
	if (!onLadder)
	{
		float headingError = fabsf(NormalizeAngle(angles.y - bot.desiredAngles.y));
		moveSpeed = NavFollow::ScaleSpeedByHeadingError(moveSpeed, headingError);
	}

	g_engfuncs.pfnRunPlayerMove(pEdict, angles, moveSpeed, 0.f, 0.f, buttons, 0, Bot_FrameMsec());
}
