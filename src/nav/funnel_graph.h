#pragma once

#include <vector>

#include "mathlib.h"
#include "vector.h"

namespace NavFunnel
{

struct Gate
{
	Vector left, right;
};

// A pulled corner sits exactly on the gate endpoint that broke the funnel, so the caller needs to
// know which gate that was to push the corner off the wall it is touching.
struct PulledPoint
{
	Vector point;
	// Index into the gate list, or kNoGate for the start and goal points.
	size_t gateIndex = static_cast<size_t>(-1);
};

constexpr size_t kNoGate = static_cast<size_t>(-1);

// Gates are the connection's true extent. Clearance is applied to the pulled corners instead of by
// shrinking gates, because shrinking assumes the obstacle lies along the gate span, which only
// holds for some connection orientations.
Gate MakeGate(const Vector &pointA, const Vector &pointB, const Vector &travelDir2D);

std::vector<PulledPoint> StringPull(const Vector &start, const Vector &goal,
	const std::vector<Gate> &gates);

}

