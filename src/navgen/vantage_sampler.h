#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "area_merger.h"
#include "blocking_entities.h"
#include "bsp_file.h"
#include "bsp_vis.h"
#include "nav_format.h"
#include "polytope.h"

namespace navgen
{

constexpr double kDuckEyeLift = 12.0;
constexpr double kStandEyeLift = 46.0;

// The lattice spacing floor. The actual spacing is derived from the walkable footprint so a map gets about
// kDefaultVantagePoints samples: pair cost is quadratic in the sample count.
constexpr double kDefaultVantageGrid = 15.0;
constexpr size_t kDefaultVantagePoints = 40000;
constexpr double kDefaultVantageSpacing = 128.0;
constexpr double kDefaultVantageCone = 45.0;
constexpr size_t kDefaultVantageNeighbors = 4;

constexpr size_t kMaxWatchDirs = crossbot_nav::kNavWatchDirs;

struct WatchDir
{
	double yaw = 0.0;
	double pitch = 0.0;
	double spread = 0.0;
	double amount = 0.0;
	double dist = 0.0;
};

struct Vantage
{
	Vec3 pos;
	int areaId = -1;
	int leaf = 0;
	double floorZ = 0.0;
	uint32_t areaFlags = 0;
	double exposure = 0.0;
	std::vector<WatchDir> watch;
};

struct VantageCandidate
{
	Vec3 eye;
	Vec3 floor;
	int areaId = -1;
	int leaf = 0;
	uint32_t areaFlags = 0;
	double exposure = 0.0;
	double mainAmount = 0.0;
};

struct VantageParams
{
	double gridSize = 0.0; // 0 = max(kDefaultVantageGrid, sqrt(footprint / maxPoints))
	size_t maxPoints = kDefaultVantagePoints;
	double targetSpacing = kDefaultVantageSpacing;
	double clusterConeDeg = kDefaultVantageCone;
	size_t neighborCount = kDefaultVantageNeighbors;
	size_t maxWatchDirs = kMaxWatchDirs;
	double minClusterFraction = 0.05;
	int yawBins = 64;
	int pitchBins = 8;
	double pitchRangeDeg = 45.0;
	bool verifyNeighbors = false;
	size_t threadCount = 0;
	bool groupCull = false;
};

struct VantageStats
{
	size_t latticePoints = 0;
	size_t solidRejected = 0;
	size_t sampled = 0;
	size_t totalPairs = 0;
	size_t pvsPairs = 0;
	size_t visiblePairs = 0;
	size_t budget = 0;
	size_t kept = 0;
	double footprintArea = 0.0;
	double gridSize = 0.0;
	double maxExposure = 0.0;
	double meanReductionError = 0.0;
	double maxReductionError = 0.0;
	double meanWatchDist = 0.0;
	double maxWatchDist = 0.0;
	size_t neighborChecks = 0;
	size_t neighborMismatches = 0;
	size_t worldTraces = 0;
	size_t blockerTraces = 0;
	size_t groupPairs = 0;
	size_t groupOpen = 0;
	size_t groupBlocked = 0;
	size_t groupMixed = 0;
	size_t cornerTraces = 0;
	size_t verifyChecks = 0;
	size_t verifyFalseVisible = 0;
	size_t verifyFalseBlocked = 0;
	double candidateSeconds = 0.0;
	double leafSeconds = 0.0;
	double pairSeconds = 0.0;
	double traceSeconds = 0.0;
	double deriveSeconds = 0.0;
	double reduceSeconds = 0.0;
	double verifySeconds = 0.0;
	size_t threads = 0;
	double binWallMin = 0.0;
	double binWallMax = 0.0;
};

std::vector<Vantage> BuildVantages(const BspFile &bsp, const VisIndex &vis,
	const std::vector<BlockingVolume> &visBlockers, const std::vector<Area> &areas,
	const VantageParams &params, VantageStats *stats,
	std::vector<VantageCandidate> *candidatesOut = nullptr);

}
