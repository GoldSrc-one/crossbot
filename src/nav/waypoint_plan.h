#pragma once

#include <cstdint>
#include <functional>
#include <vector>

#include "mathlib.h"
#include "vector.h"

#include "airmove_cache.h"
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

inline bool SubmergedAt(const std::vector<float> *surfaces, uint8_t volumeIndex, float submergeZ)
{
	if (volumeIndex == crossbot_nav::kNoWaterVolume || !surfaces ||
		volumeIndex >= surfaces->size())
		return false;

	return (*surfaces)[volumeIndex] >= submergeZ;
}

struct PlanGate
{
	Vector left, right;
	bool valid = false;
	bool clamped = false;
};

struct PlanDetourPoint
{
	Vector point;
	int areaId = -1;
	float clearance = 16.f;
	bool crouch = false;
};

struct PlanDetour
{
	uint32_t fromAreaId = 0;
	std::vector<PlanDetourPoint> points;
	size_t actIndex = 0;
	uint8_t actKind = crossbot_nav::ACT_NONE;
	Vector actCenter;
	float waitSeconds = 0.f;
	float detourSeconds = 0.f;
	bool blocked = false;
	uint32_t teamMask = 0;
};

struct PlanArea
{
	Vector centroid;
	bool crouch = false;
	bool door = false;
	bool breakable = false;
	bool elevator = false;
	uint8_t activation = crossbot_nav::ACT_NONE;
	float doorWaitTime = 0.f;
	std::vector<PlanDetour> detours;

	uint8_t waterVolume = crossbot_nav::kNoWaterVolume;
	float submergeZ = crossbot_nav::kNavSubmergeNever;

	Vector drift = Vector(0.f, 0.f, 0.f);

	std::vector<PlanPlane> boundPlanes;

	std::vector<PlanGate> edgeGates;
};

void SetDetourTeamMask(uint32_t mask);

uint32_t DetourTeamMask();

const PlanDetour *FindDetour(const PlanArea &area, uint32_t fromId);

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

	float rideSeconds = 0.f;
	float rideSecondsMax = 0.f;
	float rideWaitSeconds = 0.f;

	float gravity = 1.f;

	uint8_t waterVolume = crossbot_nav::kNoWaterVolume;
	float submergeZ = crossbot_nav::kNavSubmergeNever;

	PlanGate gateA, gateB;

	float gateAYaw[2] = {0.f, 0.f};
	float gateASpeed[2] = {0.f, 0.f};
};

struct PlanWaypoint
{
	Vector point;

	bool requiresElevatorRide = false;
	float elevatorTravelTime = 0.f;
	float elevatorWaitTime = 0.f;
	bool requiresCrouch = false;
	bool requiresDoor = false;
	bool requiresBreak = false;
	float doorWaitTime = 0.f;
	bool requiresButtonUse = false;
	float buttonWaitTime = 0.f;
	bool requiresTriggerTouch = false;
	Vector activatorCenter;
	int targetAreaId = -1;
	Vector drift = Vector(0.f, 0.f, 0.f);
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
	PmSim::PmPushFn duckPush;
	PmSim::PmPushFn standPush;
	PmSim::PmArcFn duckArc;
	PmSim::PmArcFn standArc;
	PmSim::PmPhysics physics;
	float gravityMultiplier = 1.f;
	const std::vector<float> *waterSurfaces = nullptr;
	bool dumpAirmoves = false;
	bool collectRunupTraces = false;
	AirmoveCache *airmoveCache = nullptr;
};

struct PlanStats
{
	size_t airmovesSimulated = 0;
	size_t airmovesUnplannable = 0;
	size_t airmovesRescaled = 0;
	size_t airmovesRunupExtended = 0;
	size_t airmovesRespeeded = 0;
	size_t airmovesRespeedKept = 0;
	size_t airmovesRetracted = 0;
	size_t airmovesRetractKept = 0;
	size_t corners = 0;
	size_t gatedRejected = 0;
	size_t waterRejected = 0;
		std::vector<std::vector<Vector>> runupTraces;
		size_t gatesClamped = 0;
	};

	void BuildAreaGates(PlanArea &area);

	using PlanFloorFn = std::function<bool(uint32_t areaId, float x, float y, float &zOut)>;

void BuildConnectionGates(const std::vector<PlanArea> &areas, std::vector<PlanConnection> &conns,
	const PlanFloorFn &floorAt = PlanFloorFn(), float maxStep = 0.f);

Vector ConnectionCrossing(const std::vector<PlanArea> &areas, const PlanConnection &conn);

Vector GateMidpoint(const PlanGate &gate);

struct PendingGate
{
	NavFunnel::Gate gate;

	uint32_t insideArea = 0;
	uint32_t fromArea = 0;
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
	float needRunup = 0.f;
	float haveRunup = 0.f;
	bool atEdge = false;
};

class PlanWalker
{
public:
	PlanWalker() = default;
	PlanWalker(const std::vector<PlanArea> &areas, const PlanTraceContext *traces,
		const std::vector<std::vector<uint32_t>> *walkPredecessors = nullptr,
		const std::vector<std::vector<uint32_t>> *walkSuccessors = nullptr);

	void Reset(const Vector &startPoint);
	bool Cross(const PlanConnection &conn, uint32_t connIndex, uint32_t fromId, uint32_t toId);
	void Finish(const Vector &goalPoint, int lastAreaId);
	void Retract();
	void Respeed();
	void Relax(uint32_t startArea);

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
	bool PushActivationDetour(const PlanArea &fromArea, const PlanArea &toArea, uint32_t fromId);
	void PushWaypoint(const PlanWaypoint &wp, bool countLength = true);
	void UpdateFrontier();

	bool ApplyAirmoveResult(const AirmoveResult &result, uint32_t destAreaId, Vector &endOut,
		const char **whyOut) const;

	bool JumpLandsIn(const Vector &launchFloor, float yaw, float speed, bool jump, uint32_t fromAreaId,
		uint32_t destAreaId, bool needsCrouch, Vector &endOut, const char **whyOut) const;

	bool LaunchAtEdge(const Vector &launchFloor, const Vector &launchDir, uint32_t fromId) const;

	float RunupDistance(const Vector &launchPoint, const Vector &back, float want, uint32_t fromId,
		uint32_t &runupAreaOut, size_t &gatesKeptOut) const;

	float ReachBack(const Vector &from, const Vector &back, float want, uint32_t fromId,
		const std::vector<std::vector<uint32_t>> *adjacency = nullptr,
		std::vector<uint32_t> *crossed = nullptr) const;

	bool LegIsWalkable(const Vector &from, const Vector &to, uint32_t fromId) const;

	int AreaOfWaypoint(size_t index, uint32_t startArea) const;

	bool LegIsClear(size_t from, size_t to, uint32_t fromArea) const;

	bool AreaUnderWater(int areaId) const;

	int AreaContaining(const Vector &point) const;

	const std::vector<PlanArea> *m_areas = nullptr;
	const std::vector<std::vector<uint32_t>> *m_walkPreds = nullptr;
	const std::vector<std::vector<uint32_t>> *m_walkSuccs = nullptr;
	const PlanTraceContext *m_traces = nullptr;
	PlanStats m_stats;

	std::vector<PlanWaypoint> m_result;
	std::vector<PendingGate> m_pendingGates;
	std::vector<PendingLaunch> m_launches;

	Vector m_anchor;
	bool m_anchorFromAirmove = false;
	Vector m_frontier;
	float m_committedLength = 0.f;
	float m_committedSwimLength = 0.f;
	float m_pendingLength = 0.f;
	float m_extraTime = 0.f;
};

}

