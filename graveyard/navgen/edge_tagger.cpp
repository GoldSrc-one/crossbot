#include "edge_tagger.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <set>
#include <utility>

#include "clipnode_tree.h"
#include "hull_defs.h"

namespace navgen
{

namespace
{

using namespace crossbot_nav;

constexpr double kWalkableStepHeight = 18.0;
constexpr double kAreaAabbMargin = 48.0;

constexpr double kRampFloorNormalZMin = 0.7;
constexpr double kRampFloorNormalZMax = 0.95;

bool AreaHasRampFloor(const Area &area)
{
	for (const Face &f : area.polytope.faces)
	{
		if (f.plane.normal.z >= kRampFloorNormalZMin && f.plane.normal.z < kRampFloorNormalZMax)
			return true;
	}
	return false;
}

bool PointInSolid(const BspFile &bsp, int headnode, const Vec3 &point)
{
	return ClassifyPoint(bsp, headnode, point) == CONTENTS_SOLID;
}

constexpr double kOcclusionSampleSpacing = 8.0;

bool SegmentClear(const BspFile &bsp, int headnode, const Vec3 &a, const Vec3 &b)
{
	double dx = b.x - a.x, dy = b.y - a.y, dz = b.z - a.z;
	double len = std::sqrt(dx * dx + dy * dy + dz * dz);
	if (len < 1e-6)
		return !PointInSolid(bsp, headnode, a);

	int steps = std::max(1, static_cast<int>(len / kOcclusionSampleSpacing));
	for (int i = 0; i <= steps; ++i)
	{
		double t = static_cast<double>(i) / steps;
		Vec3 p(a.x + dx * t, a.y + dy * t, a.z + dz * t);
		if (PointInSolid(bsp, headnode, p))
			return false;
	}
	return true;
}

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

constexpr double kFloorNormalThreshold = 0.7;

constexpr double kCornerInsetMargin = 18.0;

bool NearestFloorPoint(const Area &area, const Vec3 &target, Vec3 &outPoint)
{
	bool found = false;
	double bestDist2 = 0.0;
	Vec3 best;
	for (const Face &f : area.polytope.faces)
	{
		if (f.plane.normal.z < kFloorNormalThreshold)
			continue;

		for (const Vec3 &v : f.winding)
		{
			double dx = v.x - target.x;
			double dy = v.y - target.y;
			double dz = v.z - target.z;
			double dist2 = dx * dx + dy * dy + dz * dz;
			if (!found || dist2 < bestDist2)
			{
				found = true;
				bestDist2 = dist2;
				best = v;
			}
		}
	}

	if (!found)
		return false;

	Vec3 centroid = area.polytope.Centroid();
	double cdx = centroid.x - best.x;
	double cdy = centroid.y - best.y;
	double centroidDist = std::sqrt(cdx * cdx + cdy * cdy);
	if (centroidDist > 1e-6)
	{
		double inset = std::min(kCornerInsetMargin, centroidDist * 0.5);
		best.x += cdx / centroidDist * inset;
		best.y += cdy / centroidDist * inset;
	}

	outPoint = best;
	return true;
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
	};
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

Vec3 ParseEntityOrigin(const std::string &origin)
{
	double x = 0, y = 0, z = 0;
	std::sscanf(origin.c_str(), "%lf %lf %lf", &x, &y, &z);
	return Vec3(x, y, z);
}

}

std::vector<bool> ComputeCrouchAreaFlags(const std::vector<Area> &areas, const std::vector<bool> &isCrouchLeaf)
{
	std::vector<bool> result(areas.size(), false);
	for (size_t i = 0; i < areas.size(); ++i)
	{
		for (int leafId : areas[i].memberLeafIds)
		{
			if (static_cast<size_t>(leafId) < isCrouchLeaf.size() && isCrouchLeaf[static_cast<size_t>(leafId)])
			{
				result[i] = true;
				break;
			}
		}
	}
	return result;
}

namespace
{

constexpr double kCrouchLinkMaxHorizDistance = 48.0;
constexpr double kCrouchProbeLift = 6.0;

}

void ProbeCrouchLinks(const BspFile &bsp, int duckHeadnode, const std::vector<Area> &areas,
	const std::vector<bool> &isCrouchLeaf, std::vector<MergedConnection> &conns)
{
	std::vector<bool> isCrouchArea = ComputeCrouchAreaFlags(areas, isCrouchLeaf);

	AreaBoundsCache cache(areas);

	struct DisjointSet
	{
		std::vector<int> parent;
		explicit DisjointSet(size_t n) : parent(n)
		{
			for (size_t i = 0; i < n; ++i)
				parent[i] = static_cast<int>(i);
		}
		int Find(int x)
		{
			while (parent[static_cast<size_t>(x)] != x)
			{
				parent[static_cast<size_t>(x)] = parent[static_cast<size_t>(parent[static_cast<size_t>(x)])];
				x = parent[static_cast<size_t>(x)];
			}
			return x;
		}
		void Union(int a, int b)
		{
			a = Find(a);
			b = Find(b);
			if (a != b)
				parent[static_cast<size_t>(a)] = b;
		}
	};

	DisjointSet components(areas.size());
	for (const MergedConnection &c : conns)
		components.Union(c.areaA, c.areaB);

	bool debugDump = std::getenv("NAVGEN_DUMP_CROUCH") != nullptr;

	size_t n = areas.size();
	for (size_t i = 0; i < n; ++i)
	{
		if (!isCrouchArea[i])
			continue;

		for (size_t j = 0; j < n; ++j)
		{
			if (i == j)
				continue;
			if (components.Find(static_cast<int>(i)) == components.Find(static_cast<int>(j)))
				continue;

			const Vec3 &ci = cache.centroid[i];
			const Vec3 &cj = cache.centroid[j];
			double coarseHorizDist = std::sqrt((ci.x - cj.x) * (ci.x - cj.x) + (ci.y - cj.y) * (ci.y - cj.y));
			if (coarseHorizDist > kCrouchLinkMaxHorizDistance + kAreaAabbMargin)
				continue;

			Vec3 nearA, nearB;
			if (!NearestFloorPoint(areas[i], cj, nearA) || !NearestFloorPoint(areas[j], ci, nearB))
			{
				if (debugDump)
					std::printf("  crouch-probe area%zu<->area%zu: no floor point found\n", i, j);
				continue;
			}

			double horizDist = std::sqrt((nearA.x - nearB.x) * (nearA.x - nearB.x) +
				(nearA.y - nearB.y) * (nearA.y - nearB.y));

			double floorA = nearA.z + (isCrouchArea[i] ? kDuckHullMinZ : kHumanHullMinZ);
			double floorB = nearB.z + (isCrouchArea[j] ? kDuckHullMinZ : kHumanHullMinZ);
			double vertDelta = std::fabs(floorA - floorB);

			if (debugDump)
				std::printf("  crouch-probe area%zu<->area%zu: nearA=(%.1f,%.1f,%.1f) nearB=(%.1f,%.1f,%.1f) "
					"floorA=%.1f floorB=%.1f horizDist=%.1f vertDelta=%.1f\n",
					i, j, nearA.x, nearA.y, nearA.z, nearB.x, nearB.y, nearB.z, floorA, floorB, horizDist, vertDelta);

			if (horizDist > kCrouchLinkMaxHorizDistance || vertDelta > kWalkableStepHeight)
				continue;

			double duckOriginA = floorA - kDuckHullMinZ + kCrouchProbeLift;
			double duckOriginB = floorB - kDuckHullMinZ + kCrouchProbeLift;
			Vec3 clearanceA(nearA.x, nearA.y, duckOriginA);
			Vec3 clearanceB(nearB.x, nearB.y, duckOriginB);
			if (!SegmentClear(bsp, duckHeadnode, clearanceA, clearanceB))
			{
				if (debugDump)
					std::printf("  crouch-probe area%zu<->area%zu: segment blocked\n", i, j);
				continue;
			}

			MergedConnection link;
			link.areaA = static_cast<int>(i);
			link.areaB = static_cast<int>(j);
			link.pointA = nearA;
			link.pointB = nearB;
			link.area = 0.0;
			link.connType = crossbot_nav::CONN_WALK;
			link.connFlags = crossbot_nav::CONN_FLAG_NONE;
			conns.push_back(link);
			components.Union(static_cast<int>(i), static_cast<int>(j));
		}
	}
}

void ConnectRampAreasToFloors(const std::vector<Area> &areas, std::vector<MergedConnection> &conns)
{
	constexpr double kMinOverlapArea = 100.0;
	constexpr double kMaxHeightDiff = 16.0;

	struct AreaInfo
	{
		Vec3 mins, maxs;
		bool hasRampFloor = false;
		double floorZ = 1e9;
	};
	std::vector<AreaInfo> info(areas.size());
	for (size_t i = 0; i < areas.size(); ++i)
	{
		const Area &a = areas[i];
		info[i].mins = Vec3(1e9, 1e9, 1e9);
		info[i].maxs = Vec3(-1e9, -1e9, -1e9);
		for (const Face &f : a.polytope.faces)
		{
			for (const Vec3 &v : f.winding)
			{
				info[i].mins.x = std::min(info[i].mins.x, v.x);
				info[i].mins.y = std::min(info[i].mins.y, v.y);
				info[i].mins.z = std::min(info[i].mins.z, v.z);
				info[i].maxs.x = std::max(info[i].maxs.x, v.x);
				info[i].maxs.y = std::max(info[i].maxs.y, v.y);
				info[i].maxs.z = std::max(info[i].maxs.z, v.z);
			}
		}
		info[i].hasRampFloor = AreaHasRampFloor(a);

		for (const Face &f : a.polytope.faces)
		{
			if (f.plane.normal.z > 0.7)
			{
				for (const Vec3 &v : f.winding)
					info[i].floorZ = std::min(info[i].floorZ, v.z);
			}
		}

		if (info[i].floorZ > 1e8)
			info[i].floorZ = info[i].mins.z;
	}

	for (size_t i = 0; i < areas.size(); ++i)
	{
		if (!info[i].hasRampFloor)
			continue;

		for (size_t j = 0; j < areas.size(); ++j)
		{
			if (i == j || info[j].hasRampFloor)
				continue;

			double overlapX = std::max(0.0, std::min(info[i].maxs.x, info[j].maxs.x) - std::max(info[i].mins.x, info[j].mins.x));
			double overlapY = std::max(0.0, std::min(info[i].maxs.y, info[j].maxs.y) - std::max(info[i].mins.y, info[j].mins.y));
			double overlapArea = overlapX * overlapY;
			if (overlapArea < kMinOverlapArea)
				continue;

			double heightDiff = info[i].floorZ - info[j].floorZ;
			if (heightDiff < -kMaxHeightDiff || heightDiff > kMaxHeightDiff)
				continue;

			bool alreadyConnected = false;
			for (const MergedConnection &c : conns)
			{
				if ((c.areaA == static_cast<int>(i) && c.areaB == static_cast<int>(j)) ||
					(c.areaA == static_cast<int>(j) && c.areaB == static_cast<int>(i)))
				{
					alreadyConnected = true;
					break;
				}
			}
			if (alreadyConnected)
				continue;

			double cx = (std::max(info[i].mins.x, info[j].mins.x) + std::min(info[i].maxs.x, info[j].maxs.x)) * 0.5;
			double cy = (std::max(info[i].mins.y, info[j].mins.y) + std::min(info[i].maxs.y, info[j].maxs.y)) * 0.5;

			MergedConnection conn;
			conn.areaA = static_cast<int>(i);
			conn.areaB = static_cast<int>(j);
			conn.pointA = Vec3(cx, cy, info[i].floorZ);
			conn.pointB = Vec3(cx, cy, info[j].floorZ);
			conn.area = overlapArea;
			conn.connType = crossbot_nav::CONN_WALK;
			conn.connFlags = crossbot_nav::CONN_FLAG_NONE;
			conn.connNormal = Vec3(0, 0, 1);
			conns.push_back(conn);
		}
	}

	constexpr double kMaxRampVerticalGap = 64.0;
	for (size_t i = 0; i < areas.size(); ++i)
	{
		if (!info[i].hasRampFloor)
			continue;

		for (size_t j = 0; j < areas.size(); ++j)
		{
			if (i == j || !info[j].hasRampFloor)
				continue;

			double overlapX = std::max(0.0, std::min(info[i].maxs.x, info[j].maxs.x) - std::max(info[i].mins.x, info[j].mins.x));
			double overlapY = std::max(0.0, std::min(info[i].maxs.y, info[j].maxs.y) - std::max(info[i].mins.y, info[j].mins.y));
			double overlapArea = overlapX * overlapY;
			if (overlapArea < kMinOverlapArea)
				continue;

			double gap1 = info[j].mins.z - info[i].maxs.z;
			double gap2 = info[i].mins.z - info[j].maxs.z;
			double verticalGap = std::min(std::abs(gap1), std::abs(gap2));
			if (verticalGap > kMaxRampVerticalGap)
				continue;

			bool alreadyConnected = false;
			for (const MergedConnection &c : conns)
			{
				if ((c.areaA == static_cast<int>(i) && c.areaB == static_cast<int>(j)) ||
					(c.areaA == static_cast<int>(j) && c.areaB == static_cast<int>(i)))
				{
					alreadyConnected = true;
					break;
				}
			}
			if (alreadyConnected)
				continue;

			double cx = (std::max(info[i].mins.x, info[j].mins.x) + std::min(info[i].maxs.x, info[j].maxs.x)) * 0.5;
			double cy = (std::max(info[i].mins.y, info[j].mins.y) + std::min(info[i].maxs.y, info[j].maxs.y)) * 0.5;
			double portalZ = (info[i].maxs.z + info[j].mins.z) * 0.5;

			MergedConnection conn;
			conn.areaA = static_cast<int>(i);
			conn.areaB = static_cast<int>(j);
			conn.pointA = Vec3(cx, cy, portalZ);
			conn.pointB = Vec3(cx, cy, portalZ);
			conn.area = overlapArea;
			conn.connType = crossbot_nav::CONN_WALK;
			conn.connFlags = crossbot_nav::CONN_FLAG_NONE;
			conn.connNormal = Vec3(0, 0, 1);
			conns.push_back(conn);
		}
	}
}

}
