#pragma once

#include <extdll.h>

#include <string>
#include <vector>

#include "nav_format.h"
#include "waypoint_plan.h"

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

};

struct Connection
{
	uint32_t areaA = 0, areaB = 0;
	uint8_t connType = crossbot_nav::CONN_WALK;
	uint8_t connFlags = crossbot_nav::CONN_FLAG_NONE;
	float costSeconds = 0.f;
	float travelSeconds = 0.f;
	float runtimePenalty = 0.f;

	float exitSpeed = 0.f;
	float exitAngle = 0.f;
	float jumpBackDist = 0.f;
	float flightTime = 0.f;
	Vector launchPos, landingPos;

	float rideSeconds = 0.f;
	float rideSecondsMax = 0.f;
	float rideWaitSeconds = 0.f;

	NavPlan::PlanGate gateA, gateB;
	float gateAYaw[2] = {0.f, 0.f};
	float gateASpeed[2] = {0.f, 0.f};
};

struct NavMesh
{
	std::string mapName;
	std::vector<Area> areas;
	std::vector<Connection> conns;

	std::vector<NavPlan::PlanArea> planAreas;
	std::vector<NavPlan::PlanConnection> planConns;
};

}

