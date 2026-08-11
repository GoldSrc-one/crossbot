#pragma once

#include <cstdint>
#include <vector>

#include "area_merger.h"
#include "blocking_entities.h"
#include "bsp_file.h"
#include "entity_parser.h"

namespace navgen
{

constexpr double kUseRange = 56.0;
constexpr double kServiceGridStep = 12.0;

struct ServicePoint
{
	int areaId = -1;
	Vec3 point;
	uint8_t kind = crossbot_nav::ACT_BUTTON;
	Vec3 actCenter;
	int activatorIndex = -1;
	double margin = 0.0;
};

struct DetourStats
{
	size_t gatedAreas = 0;
	size_t activators = 0;
	size_t servicePoints = 0;
	size_t areasWithoutService = 0;
	size_t entries = 0;
	size_t solved = 0;
	size_t degenerate = 0;
	size_t blockedNoGate = 0;
	size_t blockedNoRoute = 0;
	size_t nonWalkEntries = 0;
};

void AreaBounds(const Area &area, Vec3 &minsOut, Vec3 &maxsOut);

std::vector<ServicePoint> FindServicePoints(const BspFile &bsp, int standHeadnode,
	const std::vector<BlockingVolume> &standBlockers, const std::vector<Area> &areas,
	const ActivatorVolume &act, int activatorIndex, bool dump);

DetourStats BuildActivationDetours(const BspFile &bsp, int standHeadnode,
	const std::vector<BlockingVolume> &standBlockers, const std::vector<Entity> &entities,
	const std::vector<PassableVolume> &passables, std::vector<Area> &areas,
	const std::vector<MergedConnection> &conns, bool dump);

}
