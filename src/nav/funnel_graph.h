#pragma once

#include <vector>

#include "mathlib.h"
#include "vector.h"

namespace NavFunnel
{

struct PortalGate
{
	Vector left, right;
};

constexpr float kGateShrinkDistance = 16.f;

PortalGate MakeGate(const Vector &pointA, const Vector &pointB, const Vector &travelDir2D);

std::vector<Vector> StringPull(const Vector &start, const Vector &goal, const std::vector<PortalGate> &gates);

}

