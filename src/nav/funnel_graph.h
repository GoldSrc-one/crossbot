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

struct PulledPoint
{
	Vector point;
	size_t gateIndex = static_cast<size_t>(-1);
};

constexpr size_t kNoGate = static_cast<size_t>(-1);

Gate MakeGate(const Vector &pointA, const Vector &pointB, const Vector &travelDir2D);

std::vector<PulledPoint> StringPull(const Vector &start, const Vector &goal,
	const std::vector<Gate> &gates);

}

