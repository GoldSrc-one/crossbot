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
	// Only meaningful for edgeType == EDGE_ELEVATOR (elevator_tagger.cpp):
	// where to walk to call the lift, and how long the ride itself takes.
	// Zeroed/unused for every other edge type.
	Vec3 buttonPos;
	double travelTime = 0.0;
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

// Trivial identity conversion: every leaf becomes its own standalone Area
// (no combining at all), and every leaf-to-leaf Portal becomes a
// MergedPortal 1:1. Temporary fallback for while MergeAreas' convexity/
// coverage check has a live, confirmed bug: watching the debug area-floor
// draw in-game (a real human player, not just live-testing coordinates)
// showed a room's floor rendering right up near its ceiling -- meaning
// MergeAreas folded that leaf together with one from the room *above* it
// into a single "area," corrupting the merged polytope's own floor plane.
// Every downstream consumer (NearestFloorPoint, ProbeJumpGaps, the funnel
// waypoint builder, ...) trusts an Area's polytope as ground truth, so a
// bad merge like this doesn't just look wrong in the debug view -- it's
// been the root cause suspected behind several of this milestone's stuck-
// bot bugs. Use this to get a correct (just far more granular, and un-
// optimized) baseline while MergeAreas' actual bug gets tracked down.
MergeResult NoMerge(const std::vector<ExtractedLeaf> &leaves, const std::vector<Portal> &portals);

// Folds any area smaller than `minVolume` into whichever of its portal-
// adjacent neighbors is largest, repeating until every survivor clears the
// threshold (or has no neighbor left to absorb into). Unlike MergeAreas,
// this doesn't attempt a geometric union of the absorbed area's polytope
// into the survivor's -- an area this small contributes nothing meaningful
// to the survivor's shape, so its portal connectivity is what gets kept,
// not its geometry.
//
// Exists because MergeAreas' FullyCovers+convexity requirement leaves
// genuine BSP-compiler leaf slivers (a ramp sliced into many thin pieces
// by the clipnode tree, each far smaller than a standing player's own
// hull) as their own standalone areas -- confirmed live on crossfire.bsp:
// two single-leaf areas next to a ramp with volumes of 1792 and 818 cubic
// units (a standing hull alone is ~32*32*72 = 73728) became real waypoint
// targets, and a bot aiming at a shoebox-sized centroid wedged against
// several walls just got stuck bouncing off them.
MergeResult AbsorbTinyAreas(const std::vector<Area> &areas, const std::vector<MergedPortal> &portals,
	double minVolume);

} // namespace navgen
