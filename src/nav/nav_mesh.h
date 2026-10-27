#pragma once

#include <extdll.h>

#include <memory>
#include <string>
#include <vector>

#include "bsp_world.h"
#include "nav_camp.h"
#include "nav_format.h"
#include "nav_vantage.h"
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

	uint8_t waterVolume = crossbot_nav::kNoWaterVolume;
	float submergeZ = crossbot_nav::kNavSubmergeNever;

	Vector driftVelocity = Vector(0.f, 0.f, 0.f);

	uint16_t barrier = crossbot_nav::kNoBarrier;
	uint16_t barrierFloor = crossbot_nav::kNoBarrier;
};

struct Barrier
{
	uint32_t modelIndex = 0;
	Vector mins, maxs;

	mutable int cachedEntIndex = -1;
	mutable bool closed = false;
};

struct WaterVolume
{
	uint32_t modelIndex = 0;
	Vector mins, maxs;
	float surfaceZ = 0.f;
	Vector travel;
	bool moving = false;

	mutable int cachedEntIndex = -1;
	mutable float spawnOriginZ = 0.f;
	mutable bool resolved = false;
};

struct Connection
{
	uint32_t areaA = 0, areaB = 0;
	uint8_t connType = crossbot_nav::CONN_WALK;
	uint8_t connFlags = crossbot_nav::CONN_FLAG_NONE;
	float costSeconds = 0.f;
	float travelSeconds = 0.f;
	float runtimePenalty = 0.f;
	uint32_t teamMask = 0;
	float fallHeight = 0.f;

	crossbot_nav::DriftTerm drift;

	float vantageExposure = 1.f;

	float openDistAB = 0.f;
	float openDistBA = 0.f;

	float exitSpeed = 0.f;
	float exitAngle = 0.f;
	float jumpBackDist = 0.f;
	float flightTime = 0.f;
	Vector launchPos, landingPos;

	float rideSeconds = 0.f;
	float rideSecondsMax = 0.f;
	float rideWaitSeconds = 0.f;

	float gravity = 1.f;

	uint8_t waterVolume = crossbot_nav::kNoWaterVolume;
	float submergeZ = crossbot_nav::kNavSubmergeNever;

	NavPlan::PlanGate gateA, gateB;
	float gateAYaw[2] = {0.f, 0.f};
	float gateASpeed[2] = {0.f, 0.f};
};

struct NavMesh
{
	std::string mapName;
	std::shared_ptr<const BspWorld> world;
	float designMaxSpeed = crossbot_nav::kNavDefaultDesignSpeed;
	std::vector<Area> areas;
	std::vector<Connection> conns;
	float minConnExposure = 1.f;

	std::vector<NavPlan::PlanArea> planAreas;
	std::vector<NavPlan::PlanConnection> planConns;

	std::vector<WaterVolume> waterVolumes;
	std::vector<Barrier> barriers;

	VantageField vantageField;
	std::vector<CampSpot> camps;
};

}

