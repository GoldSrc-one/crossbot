#pragma once

#include <string>
#include <vector>

#include "area_merger.h"
#include "clipnode_tree.h"
#include "polytope.h"

namespace navgen
{

// What the route does over the segment leaving each path point. Parallel to pathPoints; the entry
// for the last point is unused.
enum PathSegmentKind : uint8_t
{
	PATH_SEG_WALK = 0,
	PATH_SEG_AIRMOVE_SIMULATED,
	PATH_SEG_AIRMOVE_STORED,
};

// A re-simulated air move that missed: where the replay launched from, and where it actually came
// down instead of in the destination area.
struct FailedAirmove
{
	Vec3 launch;
	Vec3 landing;
};

void WriteDebugSvg(const std::string &path, const std::string &mapName, const std::vector<Area> &areas,
	const std::vector<MergedConnection> &conns, const std::vector<Vec3> &pathPoints = {},
	const std::vector<Vec3> &targetPoints = {},
	const std::vector<uint8_t> &pathSegmentKinds = {},
	const std::vector<FailedAirmove> &failedAirmoves = {});

}

