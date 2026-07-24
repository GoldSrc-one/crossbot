#pragma once

#include <cstdint>
#include <vector>

#include "mathlib.h" // vec_t -- vector.h's Vector needs this typedef and doesn't include it itself
#include "vector.h"

#include "nav_format.h"

// Engine-independent A*-corridor -> waypoint-polyline planner, shared verbatim between runtime plugin (path_funnel.cpp adapter) and tools/navgen (--dump-svg overlay + movement sim). Third shared module alongside astar_graph.h and funnel_graph.h; ensures both sides produce identical waypoint lists (funnel corners, special-crossing expansions, per-waypoint speed limits).
namespace NavPlan
{

// One area of the corridor, indexed by the same ids areaPath uses.
struct PlanArea
{
	Vector centroid;
	bool crouch = false;                                              // NAV_AREA_CROUCH
	bool door = false;                                                // NAV_AREA_DOOR
	uint8_t doorActivation = crossbot_nav::DOOR_ACTIVATION_TOUCH;     // only meaningful when door
	Vector doorActivatorPos;                                          // only meaningful for button/trigger activation
	float doorWaitTime = 0.f;                                         // only meaningful when door
	// True when any EDGE_DROP portal touches this area: a falling edge is physically nearby anywhere inside, driving planner's careful speed limits.
	bool hasDropPortal = false;
};

struct PlanPortal
{
	uint32_t areaA = 0, areaB = 0;
	Vector pointA, pointB;
	uint8_t edgeType = crossbot_nav::EDGE_WALK;
	Vector buttonPos;      // EDGE_ELEVATOR only: where to stand to call the lift
	float travelTime = 0.f; // EDGE_ELEVATOR only
};

// The planned waypoint -- also runtime's own waypoint type (path_funnel.h aliases Nav::Waypoint to this); fields and exact semantics from path_funnel.h carry over.
struct PlanWaypoint
{
	Vector point;
	// Speed limit for segment ENDING at this waypoint, encoded at plan time from corridor geometry: full run speed normally; walk speed through drop-off-adjacent areas; crawl speed when gates are hull-width-narrow (thin beam/razor junction). Follower caps forward speed at this -- planning-level speed encoding, not runtime guess, so offline sim and live bot slow down in same places.
	float maxSpeed = 250.f;
	bool requiresJump = false;
	bool requiresElevatorRide = false;
	float elevatorTravelTime = 0.f;
	bool requiresCrouch = false;
	bool requiresDoor = false;
	float doorWaitTime = 0.f;
	bool requiresLadder = false;
	bool requiresButtonUse = false;
	float buttonWaitTime = 0.f;
	bool requiresTriggerTouch = false;
	int targetAreaId = -1;
};

// Speed tiers for maxSpeed. Tuned against offline movement sim on test23_precise/test24_precise2 (thin beams over pit).
constexpr float kPlanSpeedRun = 250.f;
constexpr float kPlanSpeedCareful = 130.f; // drop-off-adjacent stretch (~GoldSrc walk speed)
constexpr float kPlanSpeedCrawl = 100.f;   // drop-off-adjacent AND hull-narrow gates
constexpr float kPlanNarrowGateLength = 56.f;

// Turns A* corridor (areaPath/portalPath from NavAStar::FindPath) into waypoint polyline via funnel_graph.h string-pull between hard anchors. Special crossings (jump, ladder, elevator, crouch, door) are fixed points per path_funnel.h; runs of plain WALK/DROP crossings funnel into taut corners, each carrying plan-time maxSpeed of its stretch.
std::vector<PlanWaypoint> BuildWaypointPlan(const std::vector<PlanArea> &areas,
	const std::vector<PlanPortal> &portals, const std::vector<uint32_t> &areaPath,
	const std::vector<uint32_t> &portalPath, const Vector &startPoint, const Vector &goalPoint);

} // namespace NavPlan
