#pragma once

#include <cstddef>

#include "path_profile.h"

namespace NavFollow
{

struct Config
{
	float rejoinDist = 32.f;
	float projectBack = 32.f;
	float projectAhead = 64.f;
	float rejoinAbort = 96.f;
	float derateFloor = 0.25f;
	float goalRadius = 12.f;
	float sLimit = -1.f;
	float sFloor = -1.f;
};

struct TrackState
{
	float progressS = 0.f;
	float lateral = 0.f;
	bool started = false;
};

struct Decision
{
	Vector wishDir;
	Vector aimPoint;
	float wishSpeed = 0.f;
	float targetSpeed = 0.f;
	float lateral = 0.f;
	size_t wpIndex = 0;
	bool done = false;
	bool needRepath = false;
	bool onGap = false;
};

struct Drive
{
	Vector wishDir;
	float wishSpeed = 0.f;
};

Drive SolveDrive(const Vector &wanted, const Vector &vel, const PmSim::PmPhysics &physics);

Decision Track(const NavPlan::PlanProfile &profile, const Vector &pos, const Vector &vel,
	const PmSim::PmPhysics &physics, TrackState &state, const Config &cfg = {});

}
