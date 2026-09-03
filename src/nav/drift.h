#pragma once

#include <cmath>

#include "astar_graph.h"
#include "nav_format.h"

namespace NavDrift
{

struct DriftEval
{
	float seconds = 0.f;
	bool passable = true;
};

inline DriftEval EvalDrift(const crossbot_nav::DriftTerm &term, float maxSpeed, bool forward)
{
	DriftEval out;
	if (term.length <= 0.f || maxSpeed <= 0.f)
		return out;

	float along = forward ? term.along : -term.along;
	float worst = forward ? term.alongMin : -term.alongMax;

	out.passable = (maxSpeed + worst) >= crossbot_nav::kMinDriftSpeed;

	float budget = maxSpeed * maxSpeed - term.perp * term.perp;
	float speed = along + (budget > 0.f ? std::sqrt(budget) : 0.f);
	if (speed < crossbot_nav::kMinDriftSpeed)
		speed = crossbot_nav::kMinDriftSpeed;

	out.seconds = term.length / NavAStar::kNavRunSpeed * (maxSpeed / speed - 1.f);
	return out;
}

inline float DriftSpeedAlong(const Vector &drift, const Vector &dir, float maxSpeed)
{
	float along = drift.x * dir.x + drift.y * dir.y;
	float perpSq = drift.x * drift.x + drift.y * drift.y - along * along;
	if (perpSq < 0.f)
		perpSq = 0.f;

	float budget = maxSpeed * maxSpeed - perpSq;
	float speed = along + (budget > 0.f ? std::sqrt(budget) : 0.f);
	return speed < crossbot_nav::kMinDriftSpeed ? crossbot_nav::kMinDriftSpeed : speed;
}

}
