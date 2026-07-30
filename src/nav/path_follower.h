#pragma once

#include <cstddef>
#include <vector>

#include "waypoint_plan.h"

namespace NavFollow
{

struct Config
{
	float maxSpeed = 250.f;
	float cornerRadius = 12.f;
	float finalRadius = 24.f;
	float lookahead = 48.f;
	float brakeRadius = NavPlan::kPlanClearance;
	float pathFreeplay = 0.f;
	float pathCorridor = NavPlan::kPlanClearance;
	float cornerCut = NavPlan::kPlanClearance;
};

struct Decision
{
	bool advance = false;
	Vector aimPoint;
	Vector moveDir;
};

Decision Steer(const std::vector<NavPlan::PlanWaypoint> &waypoints, size_t wpIndex, const Vector &pos,
	const Vector &vel, const Config &cfg = {});

}

