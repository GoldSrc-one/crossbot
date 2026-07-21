#pragma once

#include <cstdint>
#include <vector>

#include "nav_mesh.h"

namespace Nav
{

struct PathResult
{
	bool found = false;
	std::vector<uint32_t> areaPath;   // area ids visited, start..goal inclusive
	std::vector<uint32_t> portalPath; // portal indices crossed, one shorter than areaPath
};

// A* over the area/portal adjacency graph. Edge cost = area-centroid-to-
// area-centroid distance * the portal's traverseCost multiplier (always
// >= 1.0 by construction in navgen's nav_file writer, so the straight-line
// centroid distance used as the heuristic stays admissible). Respects
// EDGE_FLAG_ONE_WAY (areaA -> areaB only, per edge_tagger's TagDropOffs
// convention). Returns found=false if startArea/goalArea are out of range
// or no path exists.
PathResult FindPath(const NavMesh &mesh, uint32_t startArea, uint32_t goalArea);

} // namespace Nav
