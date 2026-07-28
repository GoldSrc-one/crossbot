#pragma once

#include <cstdint>
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

struct PlanArea
{
	Vector centroid;
	bool crouch = false;
	bool door = false;
	uint8_t doorActivation = crossbot_nav::DOOR_ACTIVATION_TOUCH;
	Vector doorActivatorPos;
	float doorWaitTime = 0.f;

	bool hasDropPortal = false;

	// Inward-facing bounds. Used to stop a corner being nudged into a wall, to find where a path
	// leaves the area, and to test whether a simulated jump came down inside it.
	std::vector<PlanPlane> boundPlanes;
};

struct PlanConnection
{
	uint32_t areaA = 0, areaB = 0;
	Vector pointA, pointB;
	uint8_t connType = crossbot_nav::CONN_WALK;
	uint8_t connFlags = crossbot_nav::CONN_FLAG_NONE;

	// The air move as the sampler proved it, used when a re-simulated launch does not land.
	float exitSpeed = 0.f;
	float exitAngle = 0.f;
	float jumpBackDist = 0.f;
	float flightTime = 0.f;
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

	// This waypoint launches an air move. requiresJump says whether that launch needs the jump
	// button: a drop leaves the ground by walking off, and pressing jump would overshoot it.
	bool airmoveLaunch = false;
	bool requiresJump = false;
	float launchSpeed = 0.f;
	float launchYaw = 0.f;
	float jumpBackDist = 0.f;
	// The launch was re-simulated from where the funnel wanted to leave, rather than taken from
	// the baked air move.
	bool launchSimulated = false;

	// A re-simulation was attempted and missed, so this waypoint fell back to the baked launch.
	// launchSimFrom/To are where that attempt started and where it actually came down, which is
	// what you want drawn when working out why it missed.
	bool launchSimFailed = false;
	Vector launchSimFrom;
	Vector launchSimTo;
};

constexpr float kPlanSpeedRun = 250.f;
constexpr float kPlanSpeedCareful = 130.f;
constexpr float kPlanSpeedCrawl = 100.f;
constexpr float kPlanNarrowGateLength = 56.f;

// The player's hull radius. Gates are inset by this at each end and pushed this far off the portal
// plane into the area on either side, so the funnel can only route through space a hull fits in.
constexpr float kPlanClearance = 16.f;

// Areas are baked in the duck hull, so a floor point is already the ducked origin and a standing
// origin sits this much above it.
constexpr float kPlanStandHeight = 18.f;

// The two hulls a bot moves in. Supplying these lets the planner re-simulate an air move from the
// launch point the funnel actually wants, instead of detouring to the sampled one; leave them
// empty and every air move uses its stored parameters.
struct PlanTraceContext
{
	PmSim::PmTraceFn duckTrace;
	PmSim::PmTraceFn standTrace;
	PmSim::PmPhysics physics;
};

struct PlanStats
{
	size_t airmovesSimulated = 0;
	size_t airmovesFromStored = 0;
	// Air moves that got a run-up node; the rest had no room for one inside the source area.
	size_t airmoveRunups = 0;
	size_t corners = 0;
	// Gates whose offset or end inset had to be cut short because the area was too thin or the
	// portal too narrow to give the full clearance.
	size_t gatesClamped = 0;
};

std::vector<PlanWaypoint> BuildWaypointPlan(const std::vector<PlanArea> &areas,
	const std::vector<PlanConnection> &conns, const std::vector<uint32_t> &areaPath,
	const std::vector<uint32_t> &connPath, const Vector &startPoint, const Vector &goalPoint,
	const PlanTraceContext *traces = nullptr, PlanStats *stats = nullptr);

}

