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
	std::vector<uint32_t> areaPath;
	std::vector<uint32_t> connPath;
	std::vector<PlanWaypoint> waypoints;
};

PlannedRoute PlanRoute(const std::vector<std::vector<uint32_t>> &areaConnIndices,
	const std::vector<NavAStar::Connection> &graphConns, const std::vector<PlanArea> &planAreas,
	const std::vector<PlanConnection> &planConns, uint32_t startArea, uint32_t goalArea,
	const Vector &startPoint, const Vector &goalPoint, const PlanTraceContext *traces = nullptr,
	PlanStats *stats = nullptr);

}
