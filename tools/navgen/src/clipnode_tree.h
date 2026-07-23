#pragma once

#include <functional>
#include <vector>

#include "bsp_file.h"
#include "polytope.h"

namespace navgen
{

struct PlaneChainEntry
{
	int planeIndex;
	bool front; // true = kept the front (child[0]) half-space at this split
};

// Invoked once per leaf reached while walking a hull's clipnode tree, with
// the accumulated chain of (plane, side) splits from the hull's headnode
// down to that leaf. leafContents is the raw CONTENTS_* value.
using LeafCallback = std::function<void(int leafContents, const std::vector<PlaneChainEntry> &chain)>;

// Recursively walks the clipnode tree for one hull starting at headnode.
// Only invokes `callback` for CONTENTS_EMPTY leaves, plus CONTENTS_WATER
// leaves if includeWater is set (groundwork for Milestone 2g; leaf_extractor
// passes false for the ground-only Milestone 2a pass). CONTENTS_SOLID and
// every other content type is pruned without a callback -- there is nothing
// navigable there.
void WalkClipnodeTree(const BspFile &bsp, int headnode, bool includeWater, const LeafCallback &callback);

// Walks a hull's clipnode tree invoking `callback` for every leaf whose
// contents exactly equals `targetContents` -- any CONTENTS_* value, not
// just EMPTY/WATER. Used to extract a blocking brush entity's own
// CONTENTS_SOLID volume (its model's hull-1 tree) the same way
// WalkClipnodeTree extracts worldspawn's CONTENTS_EMPTY leaves.
void WalkClipnodeTreeForContents(const BspFile &bsp, int headnode, int targetContents, const LeafCallback &callback);

// Classifies a single point against one hull's clipnode tree, descending
// directly to the leaf it falls in (no chain accumulation needed, unlike
// WalkClipnodeTree -- a point has no "other side" to also visit). Returns
// the raw CONTENTS_* value of the leaf reached (CONTENTS_SOLID,
// CONTENTS_EMPTY, ...). Used wherever a single yes/no "is this point
// blocked in hull X" question is needed rather than a full leaf walk --
// e.g. telling a hull-3-only (duck) leaf apart from space hull-1 already
// covers, by checking whether that same point reads as solid under hull 1.
int ClassifyPoint(const BspFile &bsp, int headnode, const Vec3 &point);

// Carves `poly` down one hull's clipnode tree: recursively splits it by
// each node plane it spans, and returns the disjoint convex pieces landing
// in leaves whose contents differ from `removeContents` (pieces in
// `removeContents` leaves are discarded; pieces below `minPieceVolume` are
// dropped as numeric slivers). For blocking-entity subtraction this is the
// structurally-bounded alternative to subtracting the entity's extracted
// solid volumes one by one: those volumes are hull-Minkowski-EXPANDED
// convex leaves that overlap each other heavily (dozens of overlapping
// slabs for a curved wall), so sequential per-volume subtraction
// re-fragments every piece the previous volume produced -- confirmed
// exploding to std::bad_alloc on test35_smoothwall's many-segment curved
// func_wall. Carving by the tree fragments a piece at most once per node
// plane on its path, exactly like the BSP compiler itself would have split
// the space if the brush had been world geometry.
std::vector<Polytope> CarvePolytopeAgainstTree(const BspFile &bsp, int headnode, const Polytope &poly,
	int removeContents, double mapRadius, double minPieceVolume = 1.0, double epsilon = 0.01);

} // namespace navgen
