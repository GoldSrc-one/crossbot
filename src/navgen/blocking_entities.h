#pragma once

#include <vector>

#include "bsp_file.h"
#include "entity_parser.h"
#include "leaf_extractor.h"

namespace navgen
{

struct BlockingVolume
{
	int headnode = -1;
	Vec3 origin;
	Vec3 mins;
	Vec3 maxs;
};

std::vector<BlockingVolume> ExtractBlockingVolumes(const BspFile &bsp, const std::vector<Entity> &entities,
	int hullIndex);

bool OverlapsAnyBlockingVolume(const ExtractedLeaf &leaf, const std::vector<BlockingVolume> &blockers);

std::vector<Polytope> SubtractBlockingVolumes(const BspFile &bsp, const Polytope &leafPolytope,
	const std::vector<BlockingVolume> &blockers, double mapRadius, double epsilon = 0.01);

}

