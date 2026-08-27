#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "area_merger.h"
#include "vantage_sampler.h"

namespace navgen
{

void WriteNavFile(const std::string &path, const std::string &mapName, uint32_t bspChecksum,
	const std::vector<Area> &areas, const std::vector<MergedConnection> &conns, float designMaxSpeed,
	const std::vector<Vantage> &vantages, float vantageSpacing, float maxExposure);

}

