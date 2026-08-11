#pragma once

#include <cstdint>
#include <vector>

#include "area_merger.h"
#include "astar_graph.h"
#include "waypoint_plan.h"

namespace navgen
{

struct PlanModel
{
	std::vector<NavPlan::PlanArea> areas;
	std::vector<NavPlan::PlanConnection> conns;
	std::vector<NavAStar::Connection> graphConns;
	std::vector<std::vector<uint32_t>> areaConnIndices;
};

PlanModel BuildPlanModel(const MergeResult &merged, bool walkOnly = false);

}
