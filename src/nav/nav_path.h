#pragma once

#include <extdll.h>

#include <cstdint>
#include <vector>

#include "nav_mesh.h"
#include "plan_search.h"
#include "pm_sim.h"
#include "waypoint_plan.h"

namespace Nav
{

using Waypoint = NavPlan::PlanWaypoint;

using PlannedPath = NavPlan::PlannedRoute;

void SetPlanTeam(uint32_t teamBit, bool carrier);

PlannedPath PlanPath(const NavMesh &mesh, uint32_t startArea, uint32_t goalArea,
	const Vector &startPoint, const Vector &goalPoint,
	const NavPlan::PlanTraceContext *traces = nullptr, NavPlan::PlanStats *stats = nullptr,
	float careful = 0.f, const NavPlan::RangeBias &range = NavPlan::RangeBias(),
	const PmSim::FallRule &fall = PmSim::kHlFall);

constexpr float kUnreachableSeconds = 1.0e9f;

void AreaCosts(const NavMesh &mesh, uint32_t startArea, std::vector<float> &out,
	float gravityMultiplier = 1.f, float careful = 0.f, float maxSpeed = 0.f,
	const NavPlan::RangeBias &range = NavPlan::RangeBias(), const PmSim::FallRule &fall = PmSim::kHlFall);

void AreaCostsToGoal(const NavMesh &mesh, uint32_t goalArea, std::vector<float> &out,
	float gravityMultiplier = 1.f, float careful = 0.f, float maxSpeed = 0.f,
	const NavPlan::RangeBias &range = NavPlan::RangeBias(), const PmSim::FallRule &fall = PmSim::kHlFall);

}

