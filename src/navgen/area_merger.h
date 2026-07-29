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
	// Index into each area's boundPlanes of the face this connection crosses, -1 when the side has
	// none. Resolved by ResolveConnectionPlanes once the areas are final.
	int planeA = -1;
	int planeB = -1;
	double area = 0.0;

	// Funnel gates with the player radius already applied, one per side. Baked here so the runtime
	// does no geometry at load.
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

// Fills planeA/planeB. A walk crosses one shared face, so both sides resolve by looking for the
// wall the connection lies on. An air move leaves over a real edge but lands wherever it lands, so
// the entry side is taken as the wall the trajectory flies most directly into.
size_t ResolveConnectionPlanes(const std::vector<Area> &areas, std::vector<MergedConnection> &conns);

// A bound plane as a 2D line: unit inward normal and distance. False for floors and ceilings.
bool WallPlaneAsLine(const Plane &plane, Vec3 &normalOut, double &distOut);

// The wall a mover heading along dir2D crosses on the way in: normals point inward, so that is the
// one dir2D agrees with most. -1 when the area has no wall facing that way.
int WallFacingDirection(const Area &area, const Vec3 &dir2D);

// Resolves the planes, builds the gates through the shared planner code, then lifts each gate
// endpoint onto the true floor under it -- the exported bound planes only carry a z-slab for a
// prism, so reading height back from them would flatten every ramp and staircase.
size_t BakeConnectionGates(const std::vector<Area> &areas, std::vector<MergedConnection> &conns);

size_t FilterWalkConnectionsByStep(const std::vector<Area> &areas, std::vector<MergedConnection> &conns,
	double maxStep, bool dump = false);

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

