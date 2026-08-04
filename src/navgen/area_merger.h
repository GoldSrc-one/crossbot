#pragma once

#include <vector>

#include "blocking_entities.h"
#include "leaf_extractor.h"
#include "nav_format.h"
#include "polytope.h"
#include "portal_builder.h"

namespace navgen
{

struct Area
{
	int id = 0;
	std::vector<int> memberLeafIds;
	Polytope polytope;
	std::vector<Plane> boundPlanes;
	uint32_t areaFlags = crossbot_nav::NAV_AREA_NONE;
	bool prism = false;

	uint8_t doorActivation = crossbot_nav::DOOR_ACTIVATION_TOUCH;
	Vec3 doorActivatorPos;
	double openSeconds = 0.0;
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
	double flightTime = 0.0;
};

struct MergeResult
{
	std::vector<Area> areas;
	std::vector<MergedConnection> conns;
};

float ConnectionCostSeconds(const MergedConnection &conn, const std::vector<Area> &areas);

size_t ResolveConnectionPlanes(const std::vector<Area> &areas, std::vector<MergedConnection> &conns);

bool WallPlaneAsLine(const Plane &plane, Vec3 &normalOut, double &distOut);

int WallFacingDirection(const Area &area, const Vec3 &dir2D);

size_t BakeConnectionGates(const std::vector<Area> &areas, std::vector<MergedConnection> &conns);

size_t FilterWalkConnectionsByStep(const std::vector<Area> &areas, std::vector<MergedConnection> &conns,
	double maxStep, bool dump = false);

size_t SnapConnectionsToFloor(const std::vector<Area> &areas, std::vector<MergedConnection> &conns);

MergeResult NoMerge(const std::vector<ExtractedLeaf> &leaves,
	const std::vector<bool> &isCrouchLeaf, const std::vector<PassableVolume> &passables);

MergeResult MergeAreas(const std::vector<ExtractedLeaf> &leaves, const std::vector<Portal> &portals,
	const std::vector<bool> &isCrouchLeaf, const std::vector<PassableVolume> &passables);

size_t BuildWalkConnections(const std::vector<Area> &areas, std::vector<MergedConnection> &conns);

size_t ValidateAreaFootprints(const std::vector<Area> &areas);

}

