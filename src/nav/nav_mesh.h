#pragma once

#include <extdll.h>

#include <string>
#include <vector>

#include "nav_format.h"

// Runtime-side navigation graph, populated from a baked .nav file by
// nav_loader.cpp. Deliberately plain data (no methods beyond a couple of
// small lookups) -- pathfinding/steering logic lives in path_astar.cpp,
// path_funnel.cpp, and src/bot/bot_move_nav.cpp, not here.
namespace Nav
{

struct Area
{
	uint32_t id = 0;
	Vector mins, maxs, centroid;
	uint32_t areaFlags = 0;
	float gravityScale = 1.f;
	std::vector<uint32_t> portalIndices; // indices into NavMesh::portals
};

struct Portal
{
	uint32_t areaA = 0, areaB = 0;
	Vector pointA, pointB; // shared-boundary segment endpoints, input to the funnel algorithm
	uint8_t edgeType = crossbot_nav::EDGE_WALK;
	uint8_t edgeFlags = crossbot_nav::EDGE_FLAG_NONE;
	float traverseCost = 1.f;
};

struct SpecialEdge
{
	uint32_t areaFrom = 0, areaTo = 0;
	uint8_t type = crossbot_nav::EDGE_LADDER;
	Vector anchorFrom, anchorTo;
	std::string targetname; // Door edges only; empty if unused
};

struct NavMesh
{
	std::string mapName;
	std::vector<Area> areas;
	std::vector<Portal> portals;
	std::vector<SpecialEdge> specialEdges;
};

} // namespace Nav
