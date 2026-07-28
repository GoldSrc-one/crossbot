#pragma once

#include <vector>

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
};

struct MergedConnection
{
	int areaA = -1;
	int areaB = -1;
	Vec3 pointA;
	Vec3 pointB;
	double area = 0.0;

	uint8_t connType = crossbot_nav::CONN_WALK;
	uint8_t connFlags = crossbot_nav::CONN_FLAG_NONE;

	double exitSpeed = 0.0;
	double exitAngle = 0.0;
	// How far behind the launch edge the jump was initiated. Zero for drops, and for reverse
	// jumps inferred from a drop rather than simulated.
	double jumpBackDist = 0.0;
	double fallDamage = 0.0;
	double flightTime = 0.0;
};

struct MergeResult
{
	std::vector<Area> areas;
	std::vector<MergedConnection> conns;
};

float TraverseCostFor(const MergedConnection &conn);

size_t FilterWalkConnectionsByStep(const std::vector<Area> &areas, std::vector<MergedConnection> &conns,
	double maxStep);

// Portal endpoints come off the shared face as a diagonal -- one corner on the floor, the other up
// at the ceiling -- so anything reading their z gets a sawtooth. Drops both endpoints of every
// connection onto the floor beneath them, which is also the pair of heights a waypoint on the
// connection interpolates between. Returns how many connections were resolved.
size_t SnapConnectionsToFloor(const std::vector<Area> &areas, std::vector<MergedConnection> &conns);

MergeResult NoMerge(const std::vector<ExtractedLeaf> &leaves, const std::vector<Portal> &portals,
	const std::vector<bool> &isCrouchLeaf);

MergeResult MergeAreas(const std::vector<ExtractedLeaf> &leaves, const std::vector<Portal> &portals,
	const std::vector<bool> &isCrouchLeaf);

}

