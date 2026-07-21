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
	std::vector<int> memberLeafIds; // original leaf ids folded into this area
	Polytope polytope;              // union boundary (concatenated member faces, internal ones removed)
};

struct MergedPortal
{
	int areaA = -1;
	int areaB = -1;
	Vec3 pointA;
	Vec3 pointB;
	double area = 0.0;
	// Defaults to a plain walkable edge; edge_tagger.cpp's ProbeJumpGaps/
	// TagDropOffs reclassify these in place once areas exist to tag.
	uint8_t edgeType = crossbot_nav::EDGE_WALK;
	uint8_t edgeFlags = crossbot_nav::EDGE_FLAG_NONE;
};

struct MergeResult
{
	std::vector<Area> areas;
	std::vector<MergedPortal> portals;
};

// Greedily merges portal-adjacent leaves into larger convex areas, with
// merge candidates drawn only from the existing portal-adjacency graph
// (bounded local search) -- deliberately avoids the O(n^2) trap FoxBot's
// abandoned auto-connect step hit.
//
// Only merges leaf pairs whose connecting portal *fully* covers both
// leaves' shared face (matched within a small area tolerance); this keeps
// each merge a simple "remove one matched pair of faces, concatenate the
// rest" operation with an exact convexity test. Leaves that only partially
// share a wall (T-junctions, several small leaves against one larger
// neighbor) are left unmerged rather than attempt a general convex-polytope
// Boolean union -- a conservative simplification, not a correctness risk.
MergeResult MergeAreas(const std::vector<ExtractedLeaf> &leaves, const std::vector<Portal> &portals);

} // namespace navgen
