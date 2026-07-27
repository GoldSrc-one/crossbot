#pragma once

#include <extdll.h>

#include <vector>

#include "../nav/path_funnel.h"

struct CBot
{
	bool inUse = false;
	edict_t *pEdict = nullptr;
	char name[32] = {};

		Vector moveDir = Vector(1.f, 0.f, 0.f);
		float nextDirChangeTime = 0.f;

		bool hasGoal = false;
		Vector goalPoint;
		std::vector<Nav::Waypoint> waypoints;
		size_t waypointIndex = 0;
		float nextRepathTime = 0.f;

		float elevatorReadyTime = 0.f;

		float doorWaitUntil = 0.f;

		bool jumpAttempted = false;

		bool fallTelemetryLogged = false;

		std::vector<Vector> traceLog;

		float obstacleBlockedSince = 0.f;

		float lastProgressTime = 0.f;
		int lastSeenWaypointIndex = 0;

		Vector lastOrigin;
		bool hasLastOrigin = false;

		int currentAreaId = -1;

		std::vector<uint32_t> plannedAreaPath;
		std::vector<uint32_t> visitedAreaPath;

		float pathStartTime = 0.f;

		float estimatedArrivalTime = 0.f;

		Vector desiredAngles;
};

