#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "airmove_sampler.h"
#include "area_merger.h"
#include "blocking_entities.h"
#include "bsp_file.h"
#include "entity_parser.h"
#include "leaf_extractor.h"
#include "nav_format.h"
#include "polytope.h"

namespace navgen
{

constexpr double kElevatorMinFootprint = 48.0;
constexpr double kElevatorAxisEpsilon = 0.01;

enum ElevatorReject : uint8_t
{
	ELEV_ACCEPTED       = 0,
	ELEV_NOT_A_MOVER    = 1,
	ELEV_NO_MODEL       = 2,
	ELEV_NOT_VERTICAL   = 3,
	ELEV_FOOTPRINT      = 4,
	ELEV_NO_TRAVEL      = 5,
	ELEV_SHAFT_OVERLAP  = 6,
};

const char *ElevatorRejectReason(uint8_t reject);

struct ElevatorStopRef
{
	int moverIndex = -1;
	int stopIndex = -1;
};

struct ElevatorStop
{
	int stopId = 0;
	int globalId = -1;
	double travel = 0.0;
	double topZ = 0.0;
	double slabTopZ = 0.0;
	double rideSeconds = 0.0;
	double waitSeconds = 0.0;
	bool isDefault = false;
	BlockingVolume duckVolume;
	BlockingVolume standVolume;
};

struct ElevatorMover
{
	int entityIndex = -1;
	std::string classname;
	std::string targetname;
	Vec3 movedir;
	Vec3 origin;
	double speed = 0.0;

	Vec3 columnMins, columnMaxs;

	std::vector<ElevatorStop> stops;

	uint8_t activation = crossbot_nav::ACT_NONE;
	std::vector<ActivatorVolume> activators;
};

struct ElevatorSet
{
	std::vector<ElevatorMover> movers;
	std::vector<ElevatorStopRef> stopIndex;
	std::vector<int> claimedEntities;

	std::vector<BlockingVolume> defaultDuck;
	std::vector<BlockingVolume> defaultStand;
	std::vector<BlockingVolume> supportDuck;

	const ElevatorStop *Stop(int globalId) const;
	const ElevatorMover *MoverOf(int globalId) const;
};

ElevatorSet ExtractElevators(const BspFile &bsp, const std::vector<Entity> &entities, double mapRadius,
	bool dump);

std::vector<ExtractedLeaf> SplitLeavesByElevatorStops(const BspFile &bsp, int headnode,
	const std::vector<BlockingVolume> &worldSupport, const ElevatorSet &elevators,
	const std::vector<ExtractedLeaf> &leaves, double mapRadius, size_t *stopLeavesOut, bool dump);

void TagElevatorAreas(const ElevatorSet &elevators, std::vector<Area> &areas);

AirmoveWorlds BuildAirmoveWorlds(const ElevatorSet &elevators, std::vector<BlockingVolume> baseDuck,
	std::vector<BlockingVolume> baseStand);

size_t BuildElevatorConnections(const ElevatorSet &elevators, const std::vector<Area> &areas,
	std::vector<MergedConnection> &conns, bool dump);

}
