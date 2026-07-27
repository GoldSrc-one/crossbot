#pragma once

#include <string>
#include <vector>

#include "area_merger.h"
#include "clipnode_tree.h"
#include "polytope.h"

namespace navgen
{

void WriteDebugSvg(const std::string &path, const std::string &mapName, const std::vector<Area> &areas,
	const std::vector<MergedConnection> &conns, const std::vector<Vec3> &pathPoints = {},
	const std::vector<Vec3> &targetPoints = {});

}

