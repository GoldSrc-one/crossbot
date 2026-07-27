#pragma once

#include <vector>
#include <unordered_set>

#include "area_merger.h"
#include "entity_parser.h"

namespace navgen
{

std::vector<int> FindSpawnAreas(const std::vector<Entity> &entities, const std::vector<Area> &areas);

std::vector<int> FindTargetAreas(const std::vector<Entity> &entities, const std::vector<Area> &areas);

int FindAreaContainingPoint(const Vec3 &point, const std::vector<Area> &areas);

MergeResult PruneUnreachableAreas(const std::vector<Area> &areas, const std::vector<MergedConnection> &conns,
	const std::vector<int> &seedAreas);

}

