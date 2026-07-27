#pragma once

#include <string>
#include <vector>

#include "area_merger.h"
#include "bsp_file.h"
#include "door_extractor.h"
#include "entity_parser.h"
#include "nav_format.h"

namespace navgen
{

std::vector<bool> ComputeCrouchAreaFlags(const std::vector<Area> &areas, const std::vector<bool> &isCrouchLeaf);

void ProbeCrouchLinks(const BspFile &bsp, int duckHeadnode, const std::vector<Area> &areas,
	const std::vector<bool> &isCrouchLeaf, std::vector<MergedConnection> &conns);

void ConnectRampAreasToFloors(const std::vector<Area> &areas, std::vector<MergedConnection> &conns);

}

