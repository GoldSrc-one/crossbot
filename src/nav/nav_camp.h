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
	float standScore = 0.f;
	float standSpread = 0.f;
	float standDist = 0.f;
	float crouchScore = 0.f;
	float crouchSpread = 0.f;
	float crouchDist = 0.f;
	int areaId = -1;
	uint32_t flags = 0;
	float arc = 0.f;
	int entIndex = 0;
};

}
