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

	int passableIndex = -1;
	int elevatorStopId = -1;
};

Polytope BuildPolytopeFromChain(const BspFile &bsp, const std::vector<PlaneChainEntry> &chain, double mapRadius);

std::vector<ExtractedLeaf> ExtractHullLeaves(const BspFile &bsp, const dmodel_t &model, double mapRadius,
	int hullIndex);

}

