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

// careful in [0, 1] weights each edge's time by its vantage exposure: 0 is the plain fastest
// route, 1 ignores time and minimizes exposure. See NavPlan::PlanRoute.
PlannedPath PlanPath(const NavMesh &mesh, uint32_t startArea, uint32_t goalArea,
	const Vector &startPoint, const Vector &goalPoint,
	const NavPlan::PlanTraceContext *traces = nullptr, NavPlan::PlanStats *stats = nullptr,
	float careful = 0.f);

constexpr float kUnreachableSeconds = 1.0e9f;

// With careful > 0 the values are no longer seconds but exposure-weighted scores. Callers that
// subtract two of them against a real duration must leave careful at 0.
void AreaCosts(const NavMesh &mesh, uint32_t startArea, std::vector<float> &out,
	float gravityMultiplier = 1.f, float careful = 0.f);

void AreaCostsToGoal(const NavMesh &mesh, uint32_t goalArea, std::vector<float> &out,
	float gravityMultiplier = 1.f, float careful = 0.f);

}

