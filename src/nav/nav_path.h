#pragma once

#include <extdll.h>

#include <cstdint>
#include <vector>

#include "nav_mesh.h"
#include "plan_search.h"
#include "waypoint_plan.h"

namespace Nav
{

using Waypoint = NavPlan::PlanWaypoint;

using PlannedPath = NavPlan::PlannedRoute;

PlannedPath PlanPath(const NavMesh &mesh, uint32_t startArea, uint32_t goalArea,
	const Vector &startPoint, const Vector &goalPoint,
	const NavPlan::PlanTraceContext *traces = nullptr, NavPlan::PlanStats *stats = nullptr);

constexpr float kUnreachableSeconds = 1.0e9f;

void AreaCosts(const NavMesh &mesh, uint32_t startArea, std::vector<float> &out);

}

