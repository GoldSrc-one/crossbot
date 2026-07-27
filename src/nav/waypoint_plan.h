#pragma once

#include <cstdint>
#include <vector>

#include "mathlib.h"
#include "vector.h"

#include "nav_format.h"

namespace NavPlan
{

struct PlanArea
{
	Vector centroid;
	bool crouch = false;
	bool door = false;
	uint8_t doorActivation = crossbot_nav::DOOR_ACTIVATION_TOUCH;
	Vector doorActivatorPos;
	float doorWaitTime = 0.f;

	bool hasDropPortal = false;
};

struct PlanConnection
{
	uint32_t areaA = 0, areaB = 0;
	Vector pointA, pointB;
	uint8_t connType = crossbot_nav::CONN_WALK;
};

struct PlanWaypoint
{
	Vector point;

	float maxSpeed = 250.f;
	bool requiresElevatorRide = false;
	float elevatorTravelTime = 0.f;
	bool requiresCrouch = false;
	bool requiresDoor = false;
	float doorWaitTime = 0.f;
	bool requiresButtonUse = false;
	float buttonWaitTime = 0.f;
	bool requiresTriggerTouch = false;
	int targetAreaId = -1;

	bool requiresTeleport = false;
	Vector teleportDest;
};

constexpr float kPlanSpeedRun = 250.f;
constexpr float kPlanSpeedCareful = 130.f;
constexpr float kPlanSpeedCrawl = 100.f;
constexpr float kPlanNarrowGateLength = 56.f;

std::vector<PlanWaypoint> BuildWaypointPlan(const std::vector<PlanArea> &areas,
	const std::vector<PlanConnection> &conns, const std::vector<uint32_t> &areaPath,
	const std::vector<uint32_t> &connPath, const Vector &startPoint, const Vector &goalPoint);

}

