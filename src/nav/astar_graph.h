#pragma once

#include <cstdint>
#include <vector>

#include "mathlib.h" // vec_t -- vector.h's Vector needs this typedef and doesn't include it itself
#include "vector.h"

// Engine-independent A* over a plain area/portal adjacency graph -- shared
// verbatim between the runtime plugin (path_astar.cpp, a thin wrapper around
// this) and tools/navgen (main.cpp's --dump-svg path overlay, so the offline
// debug tool computes literally the same route a live bot's FindPath would,
// not a hand-reimplemented approximation that could quietly drift out of
// sync with the real thing). Uses the real HLSDK Vector type (self-contained,
// no engine linkage) rather than a hand-rolled lookalike, so both the
// engine-linked crossbot_mm build and the standalone navgen build compile
// this one file completely unmodified.
namespace NavAStar
{

struct AreaNode
{
	Vector centroid = Vector(0.f, 0.f, 0.f);
};

struct PortalEdge
{
	uint32_t areaA = 0, areaB = 0;
	// True if this edge is only traversable areaA -> areaB (e.g. a one-way
	// drop-off) -- matches crossbot_nav::EDGE_FLAG_ONE_WAY's convention
	// exactly, just pre-extracted to a plain bool so this file never needs
	// to include nav_format.h's enums either (trivial for a caller to check
	// itself when building this vector).
	bool oneWay = false;
	float traverseCost = 1.f; // A* edge-cost multiplier, matching crossbot_nav::NavPortalRecord::traverseCost
};

struct GraphResult
{
	bool found = false;
	std::vector<uint32_t> areaPath;   // area ids visited, start..goal inclusive
	std::vector<uint32_t> portalPath; // portal indices crossed, one shorter than areaPath
};

// areaPortalIndices[i] lists the indices into `portals` touching area i --
// every caller already has this lying around in some form (Nav::Area::
// portalIndices at runtime, navgen's own per-area portal adjacency at
// offline-dump time), so it's taken as an input rather than rebuilt here.
// Edge cost = node-centroid-to-node-centroid distance * the portal's
// traverseCost multiplier (always >= 1.0 by construction in navgen's nav
// file writer, so the straight-line centroid distance used as the
// heuristic stays admissible). Returns found=false if startArea/goalArea
// are out of range or no path exists.
GraphResult FindPath(const std::vector<AreaNode> &areas, const std::vector<std::vector<uint32_t>> &areaPortalIndices,
	const std::vector<PortalEdge> &portals, uint32_t startArea, uint32_t goalArea);

} // namespace NavAStar
