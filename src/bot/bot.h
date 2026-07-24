#pragma once

#include <extdll.h>

#include <vector>

#include "../nav/path_funnel.h"

struct CBot
{
	bool inUse = false;
	edict_t *pEdict = nullptr;
	char name[32] = {};

	// Wander fallback (used when Nav::IsLoaded() is false, i.e. no baked .nav file).
		Vector moveDir = Vector(1.f, 0.f, 0.f);
		float nextDirChangeTime = 0.f;

		// Nav path-following state (additive to wander fields above).
		bool hasGoal = false;
		Vector goalPoint;
		std::vector<Nav::Waypoint> waypoints;
		size_t waypointIndex = 0;
		float nextRepathTime = 0.f;
		// Set when bot reaches elevator boarding waypoint; bot holds still until this time, letting engine's ground-entity attachment carry it up/down. 0 = not waiting.
		float elevatorReadyTime = 0.f;
		// Same for door waypoint (requiresDoor): func_door stays SOLID_BSP until touched, then swings/slides briefly. Hold still after contact to let door finish opening. 0 = not waiting on door.
		float doorWaitUntil = 0.f;
		// Set the frame a jump waypoint actually presses IN_JUMP; jump waypoint only completes once this is true AND bot is grounded again (landing point often within generic reach distance of takeoff point).
		bool jumpAttempted = false;
		// One-shot latch for fall telemetry (log line on first frame of unplanned airborne drop) -- reset on landing so each distinct fall logs once.
		bool fallTelemetryLogged = false;
		// Per-tick origin recording while goal active, dumped to valve/crossbot_data/nav_trace.txt when goal ends (reached or dropped). Cleared on each new goal (BotManager::SetGoalForAllBots).
		std::vector<Vector> traceLog;
		// Set first frame dynamic-obstacle trace finds something baked nav mesh doesn't know about (player-built structure, monster, runtime entity) -- 0 = not blocked. Forces early repath if dodge doesn't clear it.
		float obstacleBlockedSince = 0.f;
		// Stall detection for safety repath: last time bot completed a waypoint or got fresh path. Old blind periodic timer repathed even on schedule, rebuilding mid-segment and moving pursuit line under bot threading precise geometry (confirmed test24_precise2: 10s timer fired at razor beam junction; offline momentum sim with no repath drove identical plan cleanly). Now repath only after this stales for interval.
		float lastProgressTime = 0.f;
		int lastSeenWaypointIndex = 0;
		// Last origin seen by Bot_MoveNav, detects discontinuous jump (respawn, teleporter) and invalidates stale path.
		Vector lastOrigin;
		bool hasLastOrigin = false;

		// Diagnostics (crossbot_nav_status / NavDebug):
		// Which nav area bot's origin resolves to now, refreshed every tick. -1 when off mesh (FindAreaForPoint returns nullptr only when mesh has zero areas).
		int currentAreaId = -1;
		// A* corridor's area sequence (Nav::PathResult::areaPath, refreshed on every Repath including safety repaths) vs. areas bot actually passed through since goal assigned (appended in Bot_MoveNav when currentAreaId changes). Comparing after stall shows exactly where bot left corridor.
		std::vector<uint32_t> plannedAreaPath;
		std::vector<uint32_t> visitedAreaPath;
		// Set once on first fresh goal's path (not touched by later safety repaths), so deadline reflects original assignment, not pushed-back target.
		float pathStartTime = 0.f;
		// Straight-line distance/speed estimate at pathStartTime, with safety margin for turning/jumping/avoidance + elevator ride time (motionless, not covered by speed). Once gpGlobals->time exceeds this while short of goal, bot is OVERDUE.
		float estimatedArrivalTime = 0.f;

		// Desired facing, written by active movement mode (wander or nav); Bot_UpdateFacing smooths entity angles toward this once per frame, shared by all modes.
		Vector desiredAngles;
};
