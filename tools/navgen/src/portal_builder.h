#pragma once

#include <vector>

#include "leaf_extractor.h"
#include "polytope.h"

namespace navgen
{

struct Portal
{
	int leafA = -1;
	int leafB = -1;
	int faceIndexA = -1; // index into leaves[leafA].polytope.faces on the shared plane
	int faceIndexB = -1; // index into leaves[leafB].polytope.faces on the shared plane
	Vec3 pointA;  // segment endpoints of the shared boundary -- input to the
	Vec3 pointB;  // funnel/string-pulling algorithm at runtime (Milestone 2c)
	double area = 0.0; // shared-face overlap area, for diagnostics/weighting
};

// Detects shared-face adjacency between leaf polytopes by bucketing every
// face by its (canonicalized, quantized) plane equation -- two leaves can
// only be adjacent across a face if their faces lie on the same physical
// plane, so this only ever compares faces *within* a plane's bucket, never
// all leaf pairs in the map. Within a bucket, a real portal exists between a
// leaf whose face matches the bucket's canonical orientation and one whose
// face is the exact flip (a wall's two sides), *and* whose windings overlap
// with non-negligible area on that plane.
std::vector<Portal> BuildPortals(const std::vector<ExtractedLeaf> &leaves);

} // namespace navgen
