#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "area_merger.h"
#include "blocking_entities.h"
#include "camp_sampler.h"
#include "vantage_sampler.h"
#include "water_volumes.h"

namespace navgen
{

void WriteNavFile(const std::string &path, const std::string &mapName, uint32_t bspChecksum,
	const std::vector<Area> &areas, const std::vector<MergedConnection> &conns, float designMaxSpeed,
	const std::vector<Vantage> &vantages,
	const std::vector<CampSpot> &camps, float vantageSpacing, float maxExposure,
	const std::vector<LiquidVolume> &waterVolumes, const std::vector<PassableVolume> &passables);

}

