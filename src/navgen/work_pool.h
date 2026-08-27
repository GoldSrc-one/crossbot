#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <vector>

#include "polytope.h"

namespace navgen
{

size_t DefaultThreadCount();

uint64_t MortonKey(const Vec3 &point, double cell);

std::vector<std::vector<size_t>> BuildWorkBins(const std::vector<double> &cost,
	const std::vector<size_t> &order, size_t tileSize, size_t binCount);

void RunBins(size_t binCount, const std::function<void(size_t)> &body);

}
