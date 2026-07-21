#include "edge_tagger.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <set>
#include <utility>

namespace navgen
{

namespace
{

using namespace crossbot_nav;

// Approximate, tunable constants -- not reverse-engineered engine values.
// Reasonable stand-ins for "roughly what a running jump/safe fall can
// cover," meant to be revisited once real bots (Milestone 2c) can exercise
// them, not treated as ground truth.
constexpr double kMaxJumpHorizontalDistance = 200.0;
constexpr double kMaxJumpHeightDelta = 64.0;
constexpr double kFallDamageThreshold = 200.0;
constexpr double kAreaAabbMargin = 48.0; // ~1.5x a standing hull's radius; door/ladder proximity search slack

int ParseBrushModelIndex(const std::string &model)
{
	if (model.size() < 2 || model[0] != '*')
		return -1;
	char *end = nullptr;
	long idx = std::strtol(model.c_str() + 1, &end, 10);
	if (end == model.c_str() + 1 || *end != '\0' || idx <= 0)
		return -1;
	return static_cast<int>(idx);
}

void AreaBounds(const Area &area, Vec3 &mins, Vec3 &maxs, Vec3 &centroid)
{
	mins = Vec3(1e30, 1e30, 1e30);
	maxs = Vec3(-1e30, -1e30, -1e30);
	Vec3 sum(0, 0, 0);
	int count = 0;

	for (const Face &f : area.polytope.faces)
	{
		for (const Vec3 &v : f.winding)
		{
			mins.x = std::min(mins.x, v.x);
			mins.y = std::min(mins.y, v.y);
			mins.z = std::min(mins.z, v.z);
			maxs.x = std::max(maxs.x, v.x);
			maxs.y = std::max(maxs.y, v.y);
			maxs.z = std::max(maxs.z, v.z);
			sum = sum + v;
			++count;
		}
	}
	centroid = (count > 0) ? sum * (1.0 / count) : Vec3();
}

struct AreaBoundsCache
{
	std::vector<Vec3> mins, maxs, centroid;

	explicit AreaBoundsCache(const std::vector<Area> &areas)
	{
		mins.resize(areas.size());
		maxs.resize(areas.size());
		centroid.resize(areas.size());
		for (size_t i = 0; i < areas.size(); ++i)
			AreaBounds(areas[i], mins[i], maxs[i], centroid[i]);
	}
};

bool AabbOverlapsBox(const Vec3 &boxMins, const Vec3 &boxMaxs, const Vec3 &areaMins, const Vec3 &areaMaxs)
{
	return areaMins.x <= boxMaxs.x && areaMaxs.x >= boxMins.x &&
		areaMins.y <= boxMaxs.y && areaMaxs.y >= boxMins.y &&
		areaMins.z <= boxMaxs.z && areaMaxs.z >= boxMins.z;
}

std::vector<int> FindOverlappingAreas(const Vec3 &boxMins, const Vec3 &boxMaxs, const AreaBoundsCache &cache)
{
	std::vector<int> found;
	for (size_t i = 0; i < cache.mins.size(); ++i)
		if (AabbOverlapsBox(boxMins, boxMaxs, cache.mins[i], cache.maxs[i]))
			found.push_back(static_cast<int>(i));
	return found;
}

void TagLadder(const dmodel_t &model, const AreaBoundsCache &cache, std::vector<TaggedSpecialEdge> &out)
{
	// Margin-expanded, same as TagDoor: a ladder's own climb-trigger brush
	// is often only a few units thick (its footprint sits flush against a
	// wall), well inside the 16-unit-per-side hull-1 Minkowski shrink --
	// meaning no navigable area's AABB would ever directly overlap the raw
	// brush bounds at all. Search nearby instead of requiring literal
	// overlap (found via a real miss on crossfire.bsp: a ladder only 4
	// units wide in Y produced zero overlapping areas without this margin).
	Vec3 boxMins(model.mins[0] - kAreaAabbMargin, model.mins[1] - kAreaAabbMargin, model.mins[2] - kAreaAabbMargin);
	Vec3 boxMaxs(model.maxs[0] + kAreaAabbMargin, model.maxs[1] + kAreaAabbMargin, model.maxs[2] + kAreaAabbMargin);

	std::vector<int> overlapping = FindOverlappingAreas(boxMins, boxMaxs, cache);
	if (overlapping.size() < 2)
		return; // ladder too short/isolated to connect two distinct areas; nothing to tag

	int bottom = overlapping[0], top = overlapping[0];
	for (int a : overlapping)
	{
		if (cache.centroid[static_cast<size_t>(a)].z < cache.centroid[static_cast<size_t>(bottom)].z)
			bottom = a;
		if (cache.centroid[static_cast<size_t>(a)].z > cache.centroid[static_cast<size_t>(top)].z)
			top = a;
	}
	if (bottom == top)
		return;

	Vec3 center((boxMins.x + boxMaxs.x) * 0.5, (boxMins.y + boxMaxs.y) * 0.5, 0.0);

	TaggedSpecialEdge edge;
	edge.areaFrom = bottom;
	edge.areaTo = top;
	edge.type = EDGE_LADDER;
	edge.anchorFrom = Vec3(center.x, center.y, boxMins.z);
	edge.anchorTo = Vec3(center.x, center.y, boxMaxs.z);
	out.push_back(edge);
}

void TagDoor(const dmodel_t &model, const std::string &targetname, const AreaBoundsCache &cache,
	std::vector<TaggedSpecialEdge> &out)
{
	Vec3 boxMins(model.mins[0] - kAreaAabbMargin, model.mins[1] - kAreaAabbMargin, model.mins[2] - kAreaAabbMargin);
	Vec3 boxMaxs(model.maxs[0] + kAreaAabbMargin, model.maxs[1] + kAreaAabbMargin, model.maxs[2] + kAreaAabbMargin);

	std::vector<int> overlapping = FindOverlappingAreas(boxMins, boxMaxs, cache);
	if (overlapping.size() < 2)
		return; // door's excluded footprint doesn't border enough navigable area to bridge

	Vec3 doorCenter((model.mins[0] + model.maxs[0]) * 0.5, (model.mins[1] + model.maxs[1]) * 0.5,
		(model.mins[2] + model.maxs[2]) * 0.5);

	// Connect only the most-separated pair among the candidates (the
	// polytope-diameter trick portal_builder already uses for portal
	// endpoints), not every pair -- connecting all pairs blew up badly in
	// practice: a door surrounded by several small unmerged areas (common,
	// since area_merger only merges full-coverage matches) produced dozens
	// of edges per door on crossfire.bsp (584 edges for 11 doors) instead
	// of the one meaningful "this side to that side" connection. The two
	// most-separated candidates reliably are the true opposite sides even
	// in a cluttered local cluster of small nearby areas.
	int bestI = 0, bestJ = 1;
	double bestDist = -1.0;
	for (size_t i = 0; i < overlapping.size(); ++i)
	{
		for (size_t j = i + 1; j < overlapping.size(); ++j)
		{
			const Vec3 &ci = cache.centroid[static_cast<size_t>(overlapping[i])];
			const Vec3 &cj = cache.centroid[static_cast<size_t>(overlapping[j])];
			double d = (ci - cj).Length();
			if (d > bestDist)
			{
				bestDist = d;
				bestI = static_cast<int>(i);
				bestJ = static_cast<int>(j);
			}
		}
	}

	TaggedSpecialEdge edge;
	edge.areaFrom = overlapping[static_cast<size_t>(bestI)];
	edge.areaTo = overlapping[static_cast<size_t>(bestJ)];
	edge.type = EDGE_DOOR;
	edge.anchorFrom = doorCenter;
	edge.anchorTo = doorCenter;
	edge.targetname = targetname;
	out.push_back(edge);
}

} // namespace

std::vector<TaggedSpecialEdge> TagSpecialEdges(const BspFile &bsp, const std::vector<Entity> &entities,
	const std::vector<Area> &areas)
{
	std::vector<TaggedSpecialEdge> result;
	AreaBoundsCache cache(areas);

	for (const Entity &ent : entities)
	{
		std::string classname = ent.Get("classname");
		bool isLadder = classname == "func_ladder";
		bool isDoor = classname == "func_door" || classname == "func_door_rotating";
		if (!isLadder && !isDoor)
			continue;

		int modelIndex = ParseBrushModelIndex(ent.Get("model"));
		if (modelIndex < 0 || static_cast<size_t>(modelIndex) >= bsp.Models().size())
			continue;
		const dmodel_t &model = bsp.Models()[static_cast<size_t>(modelIndex)];

		if (isLadder)
			TagLadder(model, cache, result);
		else
			TagDoor(model, ent.Get("targetname"), cache, result);
	}

	return result;
}

void ProbeJumpGaps(const std::vector<Area> &areas, std::vector<MergedPortal> &portals)
{
	AreaBoundsCache cache(areas);

	std::set<std::pair<int, int>> connected;
	for (const MergedPortal &p : portals)
		connected.insert({std::min(p.areaA, p.areaB), std::max(p.areaA, p.areaB)});

	size_t n = areas.size();
	for (size_t i = 0; i < n; ++i)
	{
		for (size_t j = i + 1; j < n; ++j)
		{
			std::pair<int, int> key{static_cast<int>(i), static_cast<int>(j)};
			if (connected.count(key))
				continue;

			const Vec3 &ci = cache.centroid[i];
			const Vec3 &cj = cache.centroid[j];
			double horizDist = std::sqrt((ci.x - cj.x) * (ci.x - cj.x) + (ci.y - cj.y) * (ci.y - cj.y));
			double vertDelta = std::fabs(ci.z - cj.z);

			if (horizDist > kMaxJumpHorizontalDistance || vertDelta > kMaxJumpHeightDelta)
				continue;

			MergedPortal jump;
			jump.areaA = static_cast<int>(i);
			jump.areaB = static_cast<int>(j);
			jump.pointA = ci;
			jump.pointB = cj;
			jump.area = 0.0; // not a real shared-face overlap -- a proximity-derived edge
			jump.edgeType = EDGE_JUMP;
			portals.push_back(jump);
			connected.insert(key);
		}
	}
}

void TagDropOffs(const std::vector<Area> &areas, std::vector<MergedPortal> &portals)
{
	AreaBoundsCache cache(areas);

	for (MergedPortal &p : portals)
	{
		if (p.edgeType != EDGE_WALK)
			continue; // don't reclassify jump/ladder/door edges as drops

		double floorA = cache.mins[static_cast<size_t>(p.areaA)].z;
		double floorB = cache.mins[static_cast<size_t>(p.areaB)].z;
		double delta = std::fabs(floorA - floorB);

		if (delta <= kFallDamageThreshold)
			continue; // ordinary step or a fall within safe range -- leave as EDGE_WALK

		p.edgeType = EDGE_DROP;
		p.edgeFlags |= EDGE_FLAG_ONE_WAY;
	}
}

} // namespace navgen
