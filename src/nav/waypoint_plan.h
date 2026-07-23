#pragma once

#include <cstdint>
#include <vector>

#include "mathlib.h" // vec_t -- vector.h's Vector needs this typedef and doesn't include it itself
#include "vector.h"

#include "nav_format.h"

// Engine-independent A*-corridor -> waypoint-polyline planner, shared
// verbatim between the runtime plugin (path_funnel.cpp, a thin NavMesh
// adapter around this) and tools/navgen (the --dump-svg path overlay and
// movement simulation) -- the third shared planning module alongside
// astar_graph.h and funnel_graph.h, and the one that finally makes the two
// sides produce literally identical waypoint lists: same funnel corners,
// same special-crossing expansions, same per-waypoint speed limits.
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
	// True when any EDGE_DROP portal touches this area: a falling edge is
	// physically nearby anywhere inside it, which is what drives the
	// planner's careful speed limits below.
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

// The planned waypoint -- also the runtime's own waypoint type
// (path_funnel.h aliases Nav::Waypoint to this), so the fields and their
// exact semantics documented in path_funnel.h's original comments carry
// over unchanged.
struct PlanWaypoint
{
	Vector point;
	// Speed limit for the segment ENDING at this waypoint, encoded at plan
	// time from the corridor's own geometry: full run speed normally,
	// walking speed when the stretch runs through drop-off-adjacent areas,
	// slower still when its gates are hull-width-narrow (a thin beam or
	// razor junction). The follower (path_follower.h) caps its forward
	// speed at this -- planning-level speed encoding, rather than a
	// runtime guess, so the offline movement simulation and the live bot
	// slow down in exactly the same places.
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

// Speed tiers for maxSpeed above. Tuned against the offline movement
// simulation on test23_precise/test24_precise2 (thin beams over a pit).
constexpr float kPlanSpeedRun = 250.f;
constexpr float kPlanSpeedCareful = 130.f; // drop-off-adjacent stretch (roughly GoldSrc walk speed)
constexpr float kPlanSpeedCrawl = 100.f;   // drop-off-adjacent AND hull-narrow gates
constexpr float kPlanNarrowGateLength = 56.f;

// Turns an A* corridor (areaPath/portalPath as produced by
// NavAStar::FindPath) into the waypoint polyline, via funnel_graph.h's
// string-pull between hard anchors. Special crossings (jump, ladder,
// elevator, crouch, door) are fixed points exactly as documented in
// path_funnel.h; runs of plain WALK/DROP crossings funnel into taut
// corners, each carrying the plan-time maxSpeed of its stretch.
std::vector<PlanWaypoint> BuildWaypointPlan(const std::vector<PlanArea> &areas,
	const std::vector<PlanPortal> &portals, const std::vector<uint32_t> &areaPath,
	const std::vector<uint32_t> &portalPath, const Vector &startPoint, const Vector &goalPoint);

} // namespace NavPlan
