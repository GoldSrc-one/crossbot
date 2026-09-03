#pragma once

#include <vector>

#include "bsp_file.h"
#include "clipnode_tree.h"
#include "nav_format.h"
#include "polytope.h"

namespace navgen
{

struct ExtractedLeaf
{
	int id = 0;
	Polytope polytope;

	int passableIndex = -1;
	int elevatorStopId = -1;

	Vec3 driftVelocity;

	uint8_t waterVolume = crossbot_nav::kNoWaterVolume;
	double submergeZ = crossbot_nav::kNavSubmergeNever;
	bool waterColumn = false;
	bool alwaysSubmerged = false;
	std::vector<Face> waterCeiling;
};

Polytope BuildPolytopeFromChain(const BspFile &bsp, const std::vector<PlaneChainEntry> &chain, double mapRadius);

std::vector<ExtractedLeaf> ExtractHullLeaves(const BspFile &bsp, const dmodel_t &model, double mapRadius,
	int hullIndex);

}

