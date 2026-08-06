#pragma once

#include <extdll.h>

#include <vector>

#include "../nav/nav_path.h"
#include "../nav/path_follower.h"
#include "bot_avoid.h"

struct CBot
{
	bool inUse = false;
	edict_t *pEdict = nullptr;
	char name[32] = {};

	bool hasGoal = false;
	Vector goalPoint;
	std::vector<Nav::Waypoint> waypoints;
	size_t waypointIndex = 0;
	float nextRepathTime = 0.f;

	NavPlan::PlanProfile followProfile;
	NavFollow::TrackState trackState;

	bool airmoveActive = false;
	bool airmoveCrouch = false;
	float airmoveYaw = 0.f;
	float airmoveSpeed = 0.f;
	float airmoveStartTime = 0.f;
	int airmoveTargetArea = -1;
	int airmoveConnIndex = -1;

	BotAvoid::State avoid;

	bool fallTelemetryLogged = false;

	std::vector<Vector> traceLog;

	float lastProgressTime = 0.f;
	float lastProgressS = 0.f;

	Vector lastOrigin;
	bool hasLastOrigin = false;

	int currentAreaId = -1;

	std::vector<uint32_t> plannedAreaPath;
	std::vector<uint32_t> visitedAreaPath;

	float pathStartTime = 0.f;

	float estimatedArrivalTime = 0.f;

	Vector desiredAngles;

	float moveForward = 0.f;
	float moveSide = 0.f;
};

