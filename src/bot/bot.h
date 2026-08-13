#pragma once

#include <extdll.h>

#include <vector>

#include "../nav/nav_path.h"
#include "../nav/path_follower.h"
#include "bot_avoid.h"

struct BotActivation
{
	Vector center;
	float time = 0.f;
};

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

	bool ladderActive = false;
	bool ladderUp = false;
	bool ladderJumpOff = false;
	bool ladderGrabbed = false;
	bool ladderAtTop = false;
	bool ladderCrouch = false;
	float ladderYaw = 0.f;
	float ladderEntryYaw = 0.f;
	float ladderEntrySpeed = 0.f;
	float ladderSpeed = 0.f;
	float ladderDismountYaw = 0.f;
	float ladderDismountZ = 0.f;
	Vector ladderMountPoint;
	Vector ladderExitPoint;
	Vector ladderStallOrigin;
	float ladderReachTime = 0.f;
	float ladderStartTime = 0.f;
	float ladderLeftTime = 0.f;
	float ladderMoveTime = 0.f;
	int ladderStalls = 0;
	int ladderTargetArea = -1;
	int ladderConnIndex = -1;

	bool rideActive = false;
	bool rideCrouch = false;
	bool rideUp = false;
	float rideStartTime = 0.f;
	float rideDeadline = 0.f;
	float rideArriveZ = 0.f;
	float rideExitYaw = 0.f;
	Vector ridePoint;
	int rideTargetArea = -1;
	int rideConnIndex = -1;
	size_t rideWpIndex = 0;
	size_t rideFloorWp = 0;

	bool activationActive = false;
	bool activationPressed = false;
	float activationStartTime = 0.f;
	float activationHoldUntil = 0.f;
	std::vector<BotActivation> activationMemo;

	BotAvoid::State avoid;

	bool fallTelemetryLogged = false;

	std::vector<Vector> traceLog;

	float lastProgressTime = 0.f;
	float lastProgressS = 0.f;

	Vector lastOrigin;
	bool hasLastOrigin = false;

	int currentAreaId = -1;

	std::vector<uint32_t> plannedAreaPath;
	std::vector<uint32_t> plannedConnPath;
	std::vector<uint32_t> visitedAreaPath;

	float pathStartTime = 0.f;

	float estimatedArrivalTime = 0.f;

	Vector desiredAngles;

	float moveForward = 0.f;
	float moveSide = 0.f;
};

