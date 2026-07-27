#pragma once

#include <cstddef>
#include <vector>

#include "waypoint_plan.h"

namespace NavFollow
{

struct Config
{

	float turnRateDegPerSec = 300.f;
	float runSpeed = 250.f;

	float cornerRadius = 12.f;
	float finalRadius = 24.f;

	float lookahead = 48.f;
	float lookaheadCareful = 24.f;

	float turnInPlaceErrorDeg = 60.f;
	float turnSlowdownErrorDeg = 20.f;
	float turnSlowdownFactor = 0.3f;
};

struct Decision
{

	bool advance = false;

	Vector aimPoint;

	float baseSpeed = 0.f;
};

Decision Steer(const std::vector<NavPlan::PlanWaypoint> &waypoints, size_t wpIndex, const Vector &pos,
	const Config &cfg = {});

float ScaleSpeedByHeadingError(float baseSpeed, float headingErrorDeg, const Config &cfg = {});

struct SimResult
{
	std::vector<Vector> points;
	bool finished = false;
};
SimResult Simulate(const std::vector<NavPlan::PlanWaypoint> &waypoints, const Vector &start,
	float dt = 1.f / 60.f, float maxTime = 120.f, const Config &cfg = {});

}

