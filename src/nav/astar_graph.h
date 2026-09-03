#pragma once

#include <cstdint>
#include <vector>

#include "mathlib.h"
#include "nav_format.h"
#include "vector.h"

namespace NavAStar
{

constexpr float kNavRunSpeed = 250.f;

constexpr float kNavSwimSpeed = kNavRunSpeed * 0.8f;

struct Connection
{
	uint32_t areaA = 0, areaB = 0;

	bool oneWay = false;
	crossbot_nav::DriftTerm drift;
	float costSeconds = 0.f;
	float exposure = 1.f;
	Vector crossing = Vector(0.f, 0.f, 0.f);
};

}
