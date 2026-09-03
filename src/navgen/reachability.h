#pragma once

#include <vector>
#include <unordered_set>

#include "area_merger.h"
#include "blocking_entities.h"
#include "entity_parser.h"

namespace navgen
{

std::vector<int> FindSpawnAreas(const std::vector<Entity> &entities, const std::vector<Area> &areas,
	std::vector<Vec3> *originsOut = nullptr);

std::vector<int> FindTargetAreas(const std::vector<Entity> &entities, const std::vector<Area> &areas,
	std::vector<Vec3> *originsOut = nullptr);

int FindAreaContainingPoint(const Vec3 &point, const std::vector<Area> &areas);

size_t PruneLethalConnections(const BspFile &bsp, const std::vector<BlockingVolume> &hurtVolumes,
	std::vector<MergedConnection> &conns);

MergeResult PruneUnreachableAreas(const std::vector<Area> &areas, const std::vector<MergedConnection> &conns,
	const std::vector<int> &seedAreas, float maxSpeed);

}

