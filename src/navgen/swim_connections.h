#pragma once

#include <vector>

#include "area_merger.h"
#include "blocking_entities.h"
#include "bsp_file.h"
#include "water_volumes.h"

namespace navgen
{

constexpr double kWaterExitStep = 18.0;

constexpr double kWaterJumpLedge = 34.0;

constexpr double kWaterJumpLip = 8.0;

struct SwimStats
{
	size_t swim = 0;
	size_t plunge = 0;
	size_t beach = 0;
	size_t waterjump = 0;
	size_t blockedByGeometry = 0;
};

size_t BuildSwimConnections(const BspFile &bsp, int duckHeadnode,
	const std::vector<BlockingVolume> &blockers, const std::vector<LiquidVolume> &waterVolumes,
	const std::vector<Area> &areas, std::vector<MergedConnection> &conns, SwimStats *statsOut = nullptr,
	bool dump = false);

}
