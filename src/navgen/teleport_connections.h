#pragma once

#include <cstddef>
#include <vector>

#include "area_merger.h"
#include "blocking_entities.h"
#include "bsp_file.h"
#include "entity_parser.h"

namespace navgen
{

struct TeleportStats
{
	size_t entities = 0;
	size_t mastered = 0;
	size_t noClients = 0;
	size_t unresolvedDest = 0;
	size_t conns = 0;
};

TeleportStats BuildTeleportConnections(const BspFile &bsp, int standHeadnode,
	const std::vector<BlockingVolume> &standBlockers, const std::vector<Entity> &entities,
	const std::vector<Area> &areas, std::vector<MergedConnection> &conns, bool dump);

}
