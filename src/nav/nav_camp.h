#pragma once

#include <cstdint>

#include "nav_format.h"

namespace Nav
{

struct CampSpot
{
	Vector pos;
	float yaw = 0.f;
	float pitch = 0.f;
	float score = 0.f;
	float stand[crossbot_nav::kNavCampArcs] = {0.f, 0.f, 0.f, 0.f};
	float crouch[crossbot_nav::kNavCampArcs] = {0.f, 0.f, 0.f, 0.f};
	int areaId = -1;
	uint32_t flags = 0;
};

}
