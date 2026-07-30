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

	// Index into the passable-volume list when this leaf is the interior of a door or
	// breakable, -1 for ordinary world space. Every split pass must carry it forward.
	int passableIndex = -1;
};

Polytope BuildPolytopeFromChain(const BspFile &bsp, const std::vector<PlaneChainEntry> &chain, double mapRadius);

std::vector<ExtractedLeaf> ExtractHullLeaves(const BspFile &bsp, const dmodel_t &model, double mapRadius,
	int hullIndex);

}

