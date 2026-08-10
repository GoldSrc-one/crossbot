#pragma once

#include <cstdint>
#include <functional>
#include <vector>

#include "mathlib.h"
#include "vector.h"

#include "funnel_graph.h"
#include "nav_format.h"
#include "pm_sim.h"

namespace NavPlan
{

constexpr float kAirmoveLaunchSpeedTolerance = 0.95f;

inline float AirmoveLaunchLead(float speed, float frameTime)
{
	float dt = (frameTime > 0.f) ? frameTime : (1.f / 60.f);
	return (speed > 0.f) ? speed * dt : 0.f;
}

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
	int connIndex = -1;
	float clearance = 16.f;

	bool requiresTeleport = false;
	Vector teleportDest;

	bool requiresLadder = false;
	bool ladderUp = false;
	bool ladderJumpOff = false;
	float ladderYaw = 0.f;
	float ladderEntryYaw = 0.f;
	float ladderEntrySpeed = 0.f;
	float ladderSpeed = 0.f;
	Vector ladderDismount;
	float ladderDismountYaw = 0.f;
	Vector ladderExit;

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
	size_t airmovesUnplannable = 0;
	size_t airmovesRespeeded = 0;
	size_t airmovesRespeedKept = 0;
	size_t corners = 0;
		std::vector<std::vector<Vector>> runupTraces;
		size_t gatesClamped = 0;
	};

	void BuildAreaGates(PlanArea &area);

	using PlanFloorFn = std::function<bool(uint32_t areaId, float x, float y, float &zOut)>;

void BuildConnectionGates(const std::vector<PlanArea> &areas, std::vector<PlanConnection> &conns,
	const PlanFloorFn &floorAt = PlanFloorFn(), float maxStep = 0.f);

Vector ConnectionCrossing(const std::vector<PlanArea> &areas, const PlanConnection &conn);

struct PendingGate
{
	NavFunnel::Gate gate;

	uint32_t insideArea = 0;
	uint32_t toArea = 0;
	bool crouch = false;
	float clearance = kPlanClearance;
};

struct PendingLaunch
{
	size_t index = 0;
	uint32_t connIndex = 0;
	uint32_t fromId = 0, toId = 0;
	bool jump = false;
	bool needsCrouch = false;
	Vector launchFloor;
	float yaw = 0.f;
	float designSpeed = 0.f;
};

class PlanWalker
{
public:
	PlanWalker() = default;
	PlanWalker(const std::vector<PlanArea> &areas, const PlanTraceContext *traces);

	void Reset(const Vector &startPoint);
	bool Cross(const PlanConnection &conn, uint32_t connIndex, uint32_t fromId, uint32_t toId);
	void Finish(const Vector &goalPoint, int lastAreaId);
	void Respeed();

	void AddCost(float seconds) { m_extraTime += seconds; }

	float Time() const;
	const Vector &Frontier() const { return m_frontier; }
	const PlanStats &Stats() const { return m_stats; }
	const std::vector<PlanWaypoint> &Waypoints() const { return m_result; }
	std::vector<PlanWaypoint> TakeWaypoints() { return std::move(m_result); }

private:
	Vector AtPlayerHeight(const Vector &floorPoint, uint32_t areaId) const;
	const PlanGate *GateFor(const PlanConnection &conn, uint32_t insideId) const;
	void PushGateInside(const PlanConnection &conn, uint32_t insideId, uint32_t fromId, uint32_t toId);
	void PushGate(const PlanConnection &conn, uint32_t fromId, uint32_t toId);
	std::vector<NavFunnel::PulledPoint> PeekFunnel(const Vector &target) const;
	bool FlushFunnelTo(const Vector &target);
	void PushWaypoint(const PlanWaypoint &wp, bool countLength = true);
	void UpdateFrontier();

	bool JumpLandsIn(const Vector &launchFloor, float yaw, float speed, bool jump, uint32_t fromAreaId,
		uint32_t destAreaId, bool needsCrouch, Vector &endOut, const char **whyOut) const;

	const std::vector<PlanArea> *m_areas = nullptr;
	const PlanTraceContext *m_traces = nullptr;
	PlanStats m_stats;

	std::vector<PlanWaypoint> m_result;
	std::vector<PendingGate> m_pendingGates;
	std::vector<PendingLaunch> m_launches;

	Vector m_anchor;
	Vector m_frontier;
	float m_committedLength = 0.f;
	float m_pendingLength = 0.f;
	float m_extraTime = 0.f;
};

}

