#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "area_merger.h"
#include "blocking_entities.h"
#include "bsp_file.h"
#include "bsp_vis.h"
#include "entity_parser.h"
#include "nav_format.h"
#include "polytope.h"
#include "vantage_sampler.h"

namespace navgen
{

constexpr double kCampFovHalfDeg = 45.0;

constexpr double kDefaultCampSpacing = 256.0;
constexpr size_t kDefaultCampMinVisible = 32;
constexpr size_t kDefaultCampClimbSteps = 32;

struct CampSpot
{
	Vec3 pos;
	double yaw = 0.0;
	double pitch = 0.0;
	double score = 0.0;
	double standScore = 0.0;
	double standSpread = 0.0;
	double standDist = 0.0;
	double crouchScore = 0.0;
	double crouchSpread = 0.0;
	double crouchDist = 0.0;
	int areaId = -1;
	uint32_t flags = 0;
	uint64_t objectives = 0;
	size_t climbSteps = 0;
	Vec3 climbStart;
};

constexpr size_t kMaxCampObjectives = 64;

struct CampObjective
{
	std::string name;
	size_t entities = 0;
	std::vector<Vec3> points;
	std::vector<Vec3> mins;
	std::vector<Vec3> maxs;
	std::vector<Vec3> nearby;
};

struct CampParams
{
	double spacing = kDefaultCampSpacing;
	size_t minVisible = kDefaultCampMinVisible;
	size_t maxClimbSteps = kDefaultCampClimbSteps;
	size_t countOverride = 0;
	double gridSize = kDefaultVantageGrid;
	int yawBins = 64;
	int pitchBins = 8;
	double pitchRangeDeg = 45.0;
	size_t threadCount = 0;
};

struct CampStats
{
	size_t gated = 0;
	size_t queued = 0;
	size_t budget = 0;
	size_t kept = 0;
	size_t evaluations = 0;
	size_t climbs = 0;
	size_t cancelled = 0;
	size_t rejected = 0;
	size_t climbStepsTotal = 0;
	size_t climbStepsMax = 0;
	size_t targets = 0;
	size_t pvsPairs = 0;
	size_t visiblePairs = 0;
	size_t worldTraces = 0;
	size_t blockerTraces = 0;
	double buildSeconds = 0.0;
	size_t batches = 0;
	size_t batchItems = 0;
	size_t threads = 0;
	size_t objectives = 0;
	size_t overlookers = 0;
	size_t overlookKept = 0;
	std::vector<size_t> lanePicks;
	double objectiveSeconds = 0.0;
};

std::vector<CampObjective> BuildCampObjectives(const BspFile &bsp, const std::vector<Entity> &entities,
	const MergeResult &merged);

std::vector<CampSpot> BuildCampSpots(const BspFile &bsp, const VisIndex &vis,
	const std::vector<BlockingVolume> &visBlockers, const std::vector<VantageCandidate> &candidates,
	const std::vector<CampObjective> &objectives, const CampParams &params, CampStats *stats);

}
