#pragma once

#include <vector>

#include "leaf_extractor.h"
#include "water_volumes.h"

namespace navgen
{

struct Area;

constexpr double kNavFloorToStandOrigin = 18.0;

int GoverningWaterVolume(const std::vector<LiquidVolume> &waterVolumes, const Vec3 &floorPoint);

size_t TagConnectionsByWater(const std::vector<LiquidVolume> &waterVolumes,
	std::vector<MergedConnection> &conns);

size_t BuildLeafWaterCeilings(const BspFile &bsp, int headnode,
	const std::vector<BlockingVolume> &blockers, const std::vector<LiquidVolume> &waterVolumes,
	std::vector<ExtractedLeaf> &leaves);

std::vector<ExtractedLeaf> SplitLeavesByWaterLevels(const std::vector<LiquidVolume> &waterVolumes,
	const std::vector<ExtractedLeaf> &leaves, std::vector<bool> &isCrouchLeaf, double mapRadius,
	size_t *splitOut = nullptr, size_t *taggedOut = nullptr, bool dump = false);

}
