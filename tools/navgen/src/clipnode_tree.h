#pragma once

#include <functional>
#include <vector>

#include "bsp_file.h"

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

} // namespace navgen
