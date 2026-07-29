#pragma once

#include <cstdint>
#include <vector>

#include "mathlib.h"
#include "vector.h"

namespace NavAStar
{

struct AreaNode
{
	Vector centroid = Vector(0.f, 0.f, 0.f);
};

struct Connection
{
	uint32_t areaA = 0, areaB = 0;

	bool oneWay = false;
	float traverseCost = 1.f;
	// Where the path crosses from one area to the other, used to shape the traverse cost.
	Vector crossing = Vector(0.f, 0.f, 0.f);
};

struct GraphResult
{
	bool found = false;
	std::vector<uint32_t> areaPath;
	std::vector<uint32_t> connPath;
};

GraphResult FindPath(const std::vector<AreaNode> &areas, const std::vector<std::vector<uint32_t>> &areaConnIndices,
	const std::vector<Connection> &conns, uint32_t startArea, uint32_t goalArea);

}

