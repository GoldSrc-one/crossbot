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

// Thin wrapper around engine-independent NavAStar::FindPath (astar_graph.h/.cpp), shared verbatim with tools/navgen so offline debug SVG path overlay matches live bot route exactly. Just adapts NavMesh's Area/Portal shape into plain graph NavAStar expects. See astar_graph.h for algorithm and behavior (one-way edges, heuristic admissibility, etc.).
PathResult FindPath(const NavMesh &mesh, uint32_t startArea, uint32_t goalArea);

} // namespace Nav
