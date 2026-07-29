#pragma once

#include <cstddef>
#include <vector>

#include "waypoint_plan.h"

namespace NavFollow
{

struct Config
{

	float runSpeed = 250.f;

	float cornerRadius = 12.f;
	float finalRadius = 24.f;

	float lookahead = 48.f;
	float lookaheadCareful = 24.f;

	// How far a corner may be cut. BuildWaypointPlan pushes every corner waypoint kPlanClearance
	// off the wall it was string-pulled onto, so that is exactly the room reserved for the turn and
	// steering into it is safe by construction.
	float cornerCut = NavPlan::kPlanClearance;
};

struct Decision
{

	bool advance = false;

	Vector aimPoint;

	float baseSpeed = 0.f;
};

// speed is the mover's current horizontal speed. It only sets how early the turn starts: a bot that
// is barely moving can pivot on the spot and gets no pre-steering, one at full tilt gets all of it.
Decision Steer(const std::vector<NavPlan::PlanWaypoint> &waypoints, size_t wpIndex, const Vector &pos,
	float speed, const Config &cfg = {});

}

