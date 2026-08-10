#pragma once

#include <vector>

#include "bsp_file.h"
#include "entity_parser.h"
#include "leaf_extractor.h"
#include "nav_format.h"

namespace navgen
{

struct BlockingVolume
{
	int headnode = -1;
	Vec3 origin;
	Vec3 mins;
	Vec3 maxs;
};

struct PassableVolume
{
	BlockingVolume volume;
	uint32_t areaFlag = crossbot_nav::NAV_AREA_NONE;
	uint8_t activation = crossbot_nav::DOOR_ACTIVATION_TOUCH;
	Vec3 activatorPos;
	double openSeconds = 0.0;
};

struct PassableSet
{
	std::vector<PassableVolume> passable;
	std::vector<BlockingVolume> impassable;
};

std::vector<BlockingVolume> ExtractBlockingVolumes(const BspFile &bsp, const std::vector<Entity> &entities,
	int hullIndex);

std::vector<BlockingVolume> ExtractLadderVolumes(const BspFile &bsp, const std::vector<Entity> &entities,
	int hullIndex);

PassableSet ExtractPassableVolumes(const BspFile &bsp, const std::vector<Entity> &entities, int hullIndex);

bool OverlapsAnyBlockingVolume(const ExtractedLeaf &leaf, const std::vector<BlockingVolume> &blockers);

bool OverlapsBlockingVolume(const Polytope &poly, const BlockingVolume &blocker);

std::vector<Polytope> SubtractBlockingVolumes(const BspFile &bsp, const Polytope &leafPolytope,
	const std::vector<BlockingVolume> &blockers, double mapRadius, double epsilon = 0.01);

std::vector<Polytope> SubtractLadderVolumes(const Polytope &leafPolytope,
	const std::vector<BlockingVolume> &ladders, double mapRadius, double epsilon = 0.01);

void SplitByPassableVolume(const BspFile &bsp, const Polytope &poly, const BlockingVolume &blocker,
	double mapRadius, std::vector<Polytope> &inside, std::vector<Polytope> &outside, double epsilon = 0.01);

}
