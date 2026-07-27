#pragma once

#include <cstdint>
#include <vector>

#include "nav_mesh.h"

namespace Nav
{

struct PathResult
{
	bool found = false;
	std::vector<uint32_t> areaPath;
	std::vector<uint32_t> portalPath;
};

PathResult FindPath(const NavMesh &mesh, uint32_t startArea, uint32_t goalArea);

}

