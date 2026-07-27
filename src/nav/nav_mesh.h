#pragma once

#include <extdll.h>

#include <string>
#include <vector>

#include "nav_format.h"

namespace Nav
{

struct AreaPlane
{
	Vector normal;
	float dist = 0.f;
};

struct Area
{
	uint32_t id = 0;
	Vector mins, maxs, centroid;
	uint32_t areaFlags = 0;
	std::vector<uint32_t> connIndices;

	std::vector<AreaPlane> planes;

	uint8_t doorActivation = crossbot_nav::DOOR_ACTIVATION_TOUCH;
	Vector doorActivatorPos;
	float doorWaitTime = 0.f;
};

struct Connection
{
	uint32_t areaA = 0, areaB = 0;
	Vector pointA, pointB;
	uint8_t connType = crossbot_nav::CONN_WALK;
	uint8_t connFlags = crossbot_nav::CONN_FLAG_NONE;
	float traverseCost = 1.f;
};

struct NavMesh
{
	std::string mapName;
	std::vector<Area> areas;
	std::vector<Connection> conns;
};

}

