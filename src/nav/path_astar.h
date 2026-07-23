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

// Thin wrapper around the engine-independent NavAStar::FindPath (src/nav/
// astar_graph.h/.cpp), which is shared verbatim with tools/navgen so the
// offline debug SVG's path overlay matches a live bot's route exactly. This
// wrapper just adapts NavMesh's Area/Portal shape into the plain graph
// NavAStar expects. See astar_graph.h for the actual algorithm and its
// documented behavior (one-way edges, heuristic admissibility, etc).
PathResult FindPath(const NavMesh &mesh, uint32_t startArea, uint32_t goalArea);

} // namespace Nav
