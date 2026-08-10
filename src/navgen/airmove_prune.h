#pragma once

#include <cstddef>
#include <vector>

#include "area_merger.h"

namespace navgen
{

struct AirmovePruneStats
{
	size_t examined = 0;
	size_t dropped = 0;
	size_t noWalkRoute = 0;
};

constexpr double kDefaultAirmoveWalkRatio = 1.2;
constexpr double kDefaultLadderRatio = 2.0;

AirmovePruneStats PruneWalkableAirmoves(const std::vector<Area> &areas,
	std::vector<MergedConnection> &conns, double maxSpeed, double ratio, bool dump);

AirmovePruneStats PruneRedundantLadders(const std::vector<Area> &areas,
	std::vector<MergedConnection> &conns, double maxSpeed, double ratio, bool dump);

}
