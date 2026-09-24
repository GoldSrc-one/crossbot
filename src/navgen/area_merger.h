#pragma once

#include <vector>

#include "blocking_entities.h"
#include "leaf_extractor.h"
#include "nav_format.h"
#include "polytope.h"

namespace navgen
{

struct DetourPoint
{
	Vec3 pos;
	int areaId = -1;
	double clearance = 16.0;
	bool crouch = false;
};

struct ActivationDetour
{
	int fromAreaId = -1;
	std::vector<DetourPoint> points;
	size_t actIndex = 0;
	uint8_t actKind = crossbot_nav::ACT_BUTTON;
	Vec3 actCenter;
	double waitSeconds = 0.0;
	double detourSeconds = 0.0;
	bool blocked = false;
};

struct Area
{
	int id = 0;
	std::vector<int> memberLeafIds;
	Polytope polytope;
	std::vector<Plane> boundPlanes;
	uint32_t areaFlags = crossbot_nav::NAV_AREA_NONE;
	bool prism = false;

	uint8_t activation = crossbot_nav::ACT_NONE;
	double openSeconds = 0.0;
	int passableIndex = -1;
	Vec3 driftVelocity;
	int elevatorStopId = -1;
	int activationEntity = -1;
	std::vector<ActivatorVolume> activators;
	std::vector<ActivationDetour> detours;

	uint8_t waterVolume = crossbot_nav::kNoWaterVolume;
	double submergeZ = crossbot_nav::kNavSubmergeNever;
	bool hasFloor = true;
	bool alwaysSubmerged = false;
	std::vector<Face> waterCeiling;
};

struct MergedConnection
{
	int areaA = -1;
	int areaB = -1;
	Vec3 pointA;
	Vec3 pointB;

	int planeA = -1;
	int planeB = -1;

	Vec3 gateALeft, gateARight;
	Vec3 gateBLeft, gateBRight;
	bool gateAValid = false;
	bool gateBValid = false;
	bool gateAClamped = false;
	bool gateBClamped = false;
	double gateAYaw[2] = {0.0, 0.0};
	double gateASpeed[2] = {0.0, 0.0};

	uint8_t connType = crossbot_nav::CONN_WALK;
	uint8_t connFlags = crossbot_nav::CONN_FLAG_NONE;

	double exitSpeed = 0.0;
	double exitAngle = 0.0;

	double jumpBackDist = 0.0;
	double fallDamage = 0.0;
	double fallHeight = 0.0;
	double flightTime = 0.0;

	crossbot_nav::DriftTerm drift;

	double rideSeconds = 0.0;
	double rideSecondsMax = 0.0;
	double rideWaitSeconds = 0.0;

	double gravity = 1.0;

	uint8_t waterVolume = crossbot_nav::kNoWaterVolume;
	double submergeZ = crossbot_nav::kNavSubmergeNever;
};

struct MergeResult
{
	std::vector<Area> areas;
	std::vector<MergedConnection> conns;
};

float ConnectionCostSeconds(const MergedConnection &conn, bool withFall = true);

float ConnectionTravelSeconds(const std::vector<Area> &areas, const MergedConnection &conn);

crossbot_nav::DriftTerm ConnectionDriftTerm(const std::vector<Area> &areas, const MergedConnection &conn);

size_t ResolveConnectionPlanes(const std::vector<Area> &areas, std::vector<MergedConnection> &conns);

bool WallPlaneAsLine(const Plane &plane, Vec3 &normalOut, double &distOut);

int WallFacingDirection(const Area &area, const Vec3 &dir2D);

size_t BakeConnectionGates(const std::vector<Area> &areas, std::vector<MergedConnection> &conns);

size_t FilterWalkConnectionsByStep(const std::vector<Area> &areas, std::vector<MergedConnection> &conns,
	double maxStep, size_t *splitOut = nullptr, bool dump = false);

size_t SnapConnectionsToFloor(const std::vector<Area> &areas, std::vector<MergedConnection> &conns);

constexpr size_t kDefaultMaxMergeGroup = 64;

MergeResult MergeAreas(const std::vector<ExtractedLeaf> &leaves, const std::vector<bool> &isCrouchLeaf,
	const std::vector<PassableVolume> &passables, size_t maxMergeGroup = kDefaultMaxMergeGroup,
	size_t *seamsOut = nullptr);

size_t BuildWalkConnections(const std::vector<Area> &areas, std::vector<MergedConnection> &conns);

size_t ValidateAreaFootprints(const std::vector<Area> &areas);

}

