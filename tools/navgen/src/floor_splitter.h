#pragma once

#include <utility>
#include <vector>

#include "leaf_extractor.h"

namespace navgen
{

struct FloorSplitResult
{
	// The rewritten leaf set, renumbered so leaves[i].id == i again (same
	// contract NoMerge relies on). Un-carved leaves pass through unchanged.
	std::vector<ExtractedLeaf> leaves;
	// (columnLeafId, supportLeafId) pairs, in new leaf ids: each column is
	// an unsupported air volume that belongs with the leaf underneath it --
	// main.cpp merges each pair's areas together right after NoMerge (see
	// area_merger.h's MergeAreaPairs).
	std::vector<std::pair<int, int>> columnPairs;
};

// Splits every leaf whose bottom face partially opens into another leaf
// below (a hole in the floor) into floor-supported pieces and unsupported
// "air column" pieces, carving along vertical planes through the opening's
// edges.
//
// Why: a hull-1 CONTENTS_EMPTY leaf only promises "the standing hull fits
// here," never "there is floor under you" -- support is not a clipnode
// property. So one convex leaf can legally span a thin walkway rim AND the
// open air above a deep pit next to it (confirmed on test23_precise: a
// single leaf covered the entire pit plus its surrounding rims). Every
// downstream consumer (A*, the funnel string-pull, TagDropOffs) treats any
// straight line inside an area as walkable, which made bots beeline over
// the pit and fall in. After this pass, walkable strips are their own
// leaves, air columns merge into the area below them (where you'd actually
// end up), and the walk portals between strip and pit get their honest
// floor-height delta -- TagDropOffs turns them into one-way drops and A*
// routes around.
//
// Where the leaf's bottom face rests against clipnode-solid there is real
// support by construction: hull-1's outward Minkowski expansion means
// "solid" starts exactly hull-height above the true brush top, i.e. a
// standing origin resting on it. Only bottom-face regions coincident with
// another empty leaf's top face are genuine openings.
FloorSplitResult SplitUnsupportedColumns(const std::vector<ExtractedLeaf> &leaves, double mapRadius);

} // namespace navgen
