#pragma once

#include <vector>

#include "bsp_file.h"
#include "clipnode_tree.h"
#include "polytope.h"

namespace navgen
{

struct ExtractedLeaf
{
	int id = 0;
	Polytope polytope;
};

// Reconstructs the convex polytope bounded by a chain of half-space
// constraints (bspc/AAS-style "brush from planes"). Shared by normal leaf
// extraction (below) and blocking_entities.cpp's solid-volume extraction --
// both are "walk a hull's clipnode tree, turn a matching leaf's plane chain
// into a polytope," just for different content types and different models.
Polytope BuildPolytopeFromChain(const BspFile &bsp, const std::vector<PlaneChainEntry> &chain, double mapRadius);

// Walks hull 1 (human-size standing hull) of the given model's clipnode
// tree and produces one convex polytope per CONTENTS_EMPTY leaf reached, via
// bspc/AAS-style plane-chain intersection: each leaf's accumulated (plane,
// side) chain is clipped against a huge bounding cube to recover its
// bounded convex shape. mapRadius must exceed the map's actual extent
// (GoldSrc maps are bounded to roughly +/-4096 units; pass headroom, e.g.
// 8192).
std::vector<ExtractedLeaf> ExtractHull1Leaves(const BspFile &bsp, const dmodel_t &model, double mapRadius);

} // namespace navgen
