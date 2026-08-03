#pragma once

#include <cstdint>
#include <functional>
#include <vector>

#include "mathlib.h"
#include "vector.h"

#include "nav_format.h"
#include "pm_sim.h"

namespace NavPlan
{

struct PlanPlane
{
	Vector normal;
	float dist = 0.f;
};

struct PlanGate
{
	Vector left, right;
	bool valid = false;
	bool clamped = false;
};

struct PlanArea
{
	Vector centroid;
	bool crouch = false;
	bool door = false;
	bool breakable = false;
	uint8_t doorActivation = crossbot_nav::DOOR_ACTIVATION_TOUCH;
	Vector doorActivatorPos;
	float doorWaitTime = 0.f;

	std::vector<PlanPlane> boundPlanes;

	std::vector<PlanGate> edgeGates;
};

struct PlanConnection
{
	uint32_t areaA = 0, areaB = 0;

	uint8_t planeA = crossbot_nav::kNoAreaPlane;
	uint8_t planeB = crossbot_nav::kNoAreaPlane;
	uint8_t connType = crossbot_nav::CONN_WALK;
	uint8_t connFlags = crossbot_nav::CONN_FLAG_NONE;

	float exitSpeed = 0.f;
	float exitAngle = 0.f;
	float jumpBackDist = 0.f;
	float flightTime = 0.f;
	Vector launchPos, landingPos;

	PlanGate gateA, gateB;

	float gateAYaw[2] = {0.f, 0.f};
	float gateASpeed[2] = {0.f, 0.f};
};

struct PlanWaypoint
{
	Vector point;

	bool requiresElevatorRide = false;
	float elevatorTravelTime = 0.f;
	bool requiresCrouch = false;
	bool requiresDoor = false;
	bool requiresBreak = false;
	float doorWaitTime = 0.f;
	bool requiresButtonUse = false;
	float buttonWaitTime = 0.f;
	bool requiresTriggerTouch = false;
	int targetAreaId = -1;
	float clearance = 16.f;

	bool requiresTeleport = false;
	Vector teleportDest;

	bool airmoveLaunch = false;
	bool requiresJump = false;
	float launchSpeed = 0.f;
	float launchYaw = 0.f;
	float jumpBackDist = 0.f;

	bool launchSimulated = false;

	bool launchSimFailed = false;
	Vector launchSimFrom;
	Vector launchSimTo;

	Vector launchSimAim;
};

constexpr float kPlanClearance = 16.f;

constexpr float kPlanClampedClearance = 6.63f;

constexpr float kPlanStandHeight = 18.f;

struct PlanTraceContext
{
	PmSim::PmTraceFn duckTrace;
	PmSim::PmTraceFn standTrace;
	PmSim::PmPhysics physics;
	bool dumpAirmoves = false;
	bool collectRunupTraces = false;
};

struct PlanStats
{
	size_t airmovesSimulated = 0;
	size_t airmovesFromStored = 0;
	size_t airmovesUnplannable = 0;
	size_t airmovesRespeeded = 0;
	size_t corners = 0;
	std::vector<std::vector<Vector>> runupTraces;
	size_t gatesClamped = 0;
};

void BuildAreaGates(PlanArea &area);

// Samples the walkable floor of one area at a 2D point. Returns false where the area has no floor
// there. PlanArea only models a single floor plane, so callers that know the real geometry (navgen)
// supply this to keep gates off the parts of a shared wall where the two floors are a cliff apart.
using PlanFloorFn = std::function<bool(uint32_t areaId, float x, float y, float &zOut)>;

void BuildConnectionGates(const std::vector<PlanArea> &areas, std::vector<PlanConnection> &conns,
	const PlanFloorFn &floorAt = PlanFloorFn(), float maxStep = 0.f);

Vector ConnectionCrossing(const std::vector<PlanArea> &areas, const PlanConnection &conn);

std::vector<PlanWaypoint> BuildWaypointPlan(const std::vector<PlanArea> &areas,
	const std::vector<PlanConnection> &conns, const std::vector<uint32_t> &areaPath,
	const std::vector<uint32_t> &connPath, const Vector &startPoint, const Vector &goalPoint,
	const PlanTraceContext *traces = nullptr, PlanStats *stats = nullptr);

}

