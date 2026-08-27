#pragma once

#include <cstdint>
#include <vector>

#include "astar_graph.h"
#include "waypoint_plan.h"

namespace NavPlan
{

struct PlannedRoute
{
	bool found = false;
	bool complete = false;
	size_t airmovesRejected = 0;
	size_t airmovesTried = 0;
	size_t expansions = 0;
	float routeSeconds = 0.f;
	float routeScore = 0.f;
	std::vector<uint32_t> areaPath;
	std::vector<uint32_t> connPath;
	std::vector<PlanWaypoint> waypoints;
};

// careful blends time against exposure-weighted time: an edge costs
// time * (1 - careful) + time * exposure * careful. 0 is the plain fastest route, 1 ignores time
// and minimizes exposure. minExposure is the lowest exposure any edge carries; the heuristic is
// scaled by the same factor's lower bound so it stays admissible.
PlannedRoute PlanRoute(const std::vector<std::vector<uint32_t>> &areaConnIndices,
	const std::vector<NavAStar::Connection> &graphConns, const std::vector<PlanArea> &planAreas,
	const std::vector<PlanConnection> &planConns, uint32_t startArea, uint32_t goalArea,
	const Vector &startPoint, const Vector &goalPoint, const PlanTraceContext *traces = nullptr,
	PlanStats *stats = nullptr, float careful = 0.f, float minExposure = 1.f);

}
