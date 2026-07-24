#pragma once

#include <extdll.h>

#include <string>
#include <vector>

#include "nav_format.h"

// Runtime-side navigation graph, populated from baked .nav file by nav_loader.cpp. Plain data (no methods beyond small lookups) -- pathfinding/steering logic in path_astar.cpp, path_funnel.cpp, src/bot/bot_move_nav.cpp.
namespace Nav
{

// Half-space plane bounding area's convex polytope: interior satisfies dot(normal, point) - dist >= 0. See nav_format.h::NavAreaPlane.
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
	float gravityScale = 1.f;
	std::vector<uint32_t> portalIndices; // indices into NavMesh::portals
	// This area's real convex-polytope boundary (navgen::Area.polytope, carried through baked format since nav_format.h v4) -- lets FindAreaForPoint test true containment instead of AABB-overlap + nearest-centroid guessing. Empty for pre-v4 .nav can't happen: loader rejects anything but exact version match, so always populated when Area exists.
	std::vector<AreaPlane> planes;
	// Meaningful only when areaFlags & NAV_AREA_DOOR (v5 -- door's own solid-until-opened volume kept as own area, see tools/navgen/src/door_extractor.h). doorActivation is crossbot_nav::DoorActivation; doorActivatorPos is where to walk to press/touch first (DOOR_ACTIVATION_BUTTON/_TRIGGER only); doorWaitTime is how long to hold once triggered/touched before door assumed open.
	uint8_t doorActivation = crossbot_nav::DOOR_ACTIVATION_TOUCH;
	Vector doorActivatorPos;
	float doorWaitTime = 0.f;
};

struct Portal
{
	uint32_t areaA = 0, areaB = 0;
	Vector pointA, pointB; // shared-boundary segment endpoints, input to funnel algorithm
	uint8_t edgeType = crossbot_nav::EDGE_WALK;
	uint8_t edgeFlags = crossbot_nav::EDGE_FLAG_NONE;
	float traverseCost = 1.f;
	// Only meaningful when edgeType == EDGE_ELEVATOR -- see nav_format.h.
	Vector buttonPos;
	float travelTime = 0.f;
};

struct SpecialEdge
{
	uint32_t areaFrom = 0, areaTo = 0;
	uint8_t type = crossbot_nav::EDGE_LADDER;
	Vector anchorFrom, anchorTo;
};

struct NavMesh
{
	std::string mapName;
	std::vector<Area> areas;
	std::vector<Portal> portals;
	std::vector<SpecialEdge> specialEdges;
};

} // namespace Nav
