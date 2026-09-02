#pragma once

#include <string>
#include <vector>

#include "area_merger.h"
#include "clipnode_tree.h"
#include "polytope.h"
#include "vantage_sampler.h"
#include "water_volumes.h"

namespace navgen
{

enum PathSegmentKind : uint8_t
{
	PATH_SEG_WALK = 0,
	PATH_SEG_AIRMOVE_SIMULATED,
	PATH_SEG_AIRMOVE_STORED,
	PATH_SEG_LADDER,
};

struct FailedAirmove
{
	Vec3 launch;
	Vec3 landing;
	Vec3 aim;
};

void WriteDebugSvg(const std::string &path, const std::string &mapName, const std::vector<Area> &areas,
	const std::vector<MergedConnection> &conns, const std::vector<Vec3> &pathPoints = {},
	const std::vector<Vec3> &targetPoints = {},
	const std::vector<uint8_t> &pathSegmentKinds = {},
	const std::vector<FailedAirmove> &failedAirmoves = {},
	const std::vector<Vec3> &tracePoints = {},
	const std::vector<std::vector<Vec3>> &runupTraces = {},
	const std::vector<float> &pathSpeeds = {},
	const std::vector<LiquidVolume> &waterVolumes = {});

void WriteVantageSvg(const std::string &path, const std::string &mapName, const std::vector<Area> &areas,
	const std::vector<Vantage> &vantages, const VantageStats &stats);

}

