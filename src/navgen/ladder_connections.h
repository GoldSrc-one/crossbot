#pragma once

#include <cstddef>
#include <vector>

#include "airmove_sampler.h"
#include "area_merger.h"
#include "bsp_file.h"
#include "entity_parser.h"

namespace navgen
{

struct LadderStats
{
	size_t entities = 0;
	size_t climbFaces = 0;
	size_t mounts = 0;
	size_t conns = 0;
};

constexpr double kLadderClimbSpeed = 200.0;
constexpr double kLadderJumpOffSpeed = 270.0;
constexpr double kLadderDismountStep = 16.0;

LadderStats BuildLadderConnections(const BspFile &bsp, const std::vector<Entity> &entities,
	const HullContext &duckHull, const HullContext &standHull, const std::vector<Area> &areas,
	std::vector<MergedConnection> &conns, const PhysicsParams &physics, bool dump);

}
