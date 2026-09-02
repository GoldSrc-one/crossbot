#include "swim_connections.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "clipnode_tree.h"
#include "floor_support.h"
#include "hull_defs.h"
#include "water_classifier.h"

namespace navgen
{

namespace
{

constexpr double kSeamSlop = 2.0;
constexpr double kSwimDepth = 24.0;
constexpr double kMinSeamOverlap = 8.0;
constexpr double kLedgeInset = 17.0;

struct AreaInfo
{
	Vec3 mins;
	Vec3 maxs;
	Vec3 centroid;
	double floorZ = 0.0;
	bool hasFloor = false;
};

AreaInfo DescribeArea(const Area &area)
{
	AreaInfo info;
	PolytopeAABB(area.polytope, info.mins, info.maxs);
	info.centroid = area.polytope.Centroid();

	if (const Face *floor = LowestFloorFace(area.polytope))
	{
		info.floorZ = WindingCentroid(floor->winding).z;
		info.hasFloor = true;
	}
	else
	{
		info.floorZ = info.mins.z;
	}

	return info;
}

bool TouchesXY(const AreaInfo &a, const AreaInfo &b)
{
	return !(a.maxs.x + kSeamSlop < b.mins.x || b.maxs.x + kSeamSlop < a.mins.x ||
		a.maxs.y + kSeamSlop < b.mins.y || b.maxs.y + kSeamSlop < a.mins.y);
}

bool OverlapsZ(const AreaInfo &a, const AreaInfo &b)
{
	return !(a.maxs.z + kSeamSlop < b.mins.z || b.maxs.z + kSeamSlop < a.mins.z);
}

bool SharesFace(const AreaInfo &a, const AreaInfo &b)
{
	const double aMin[3] = {a.mins.x, a.mins.y, a.mins.z};
	const double aMax[3] = {a.maxs.x, a.maxs.y, a.maxs.z};
	const double bMin[3] = {b.mins.x, b.mins.y, b.mins.z};
	const double bMax[3] = {b.maxs.x, b.maxs.y, b.maxs.z};

	for (int axis = 0; axis < 3; ++axis)
	{
		bool abut = std::fabs(aMax[axis] - bMin[axis]) <= kSeamSlop ||
			std::fabs(bMax[axis] - aMin[axis]) <= kSeamSlop;
		if (!abut)
			continue;

		bool spanOk = true;
		for (int other = 0; other < 3 && spanOk; ++other)
		{
			if (other == axis)
				continue;
			double overlap = std::min(aMax[other], bMax[other]) - std::max(aMin[other], bMin[other]);
			spanOk = overlap >= kMinSeamOverlap;
		}

		if (spanOk)
			return true;
	}

	return false;
}

Vec3 ContactPoint(const AreaInfo &a, const AreaInfo &b)
{
	double x = (std::max(a.mins.x, b.mins.x) + std::min(a.maxs.x, b.maxs.x)) * 0.5;
	double y = (std::max(a.mins.y, b.mins.y) + std::min(a.maxs.y, b.maxs.y)) * 0.5;
	return Vec3(x, y, 0.0);
}

struct EdgeCandidate
{
	Vec3 dryXY;
	Vec3 wetXY;
};

void CollectEdgeCandidates(const AreaInfo &dry, const AreaInfo &wet, std::vector<EdgeCandidate> &out)
{
	double loX = std::max(dry.mins.x, wet.mins.x);
	double hiX = std::min(dry.maxs.x, wet.maxs.x);
	double loY = std::max(dry.mins.y, wet.mins.y);
	double hiY = std::min(dry.maxs.y, wet.maxs.y);

	double midX = (loX + hiX) * 0.5;
	double midY = (loY + hiY) * 0.5;

	out.push_back({Vec3(midX, midY, 0.0), Vec3(midX, midY, 0.0)});

	const double sides[4][4] = {
		{dry.mins.x, midY, -1.0, 0.0},
		{dry.maxs.x, midY, 1.0, 0.0},
		{midX, dry.mins.y, 0.0, -1.0},
		{midX, dry.maxs.y, 0.0, 1.0},
	};

	for (int s = 0; s < 4; ++s)
	{
		Vec3 edge(sides[s][0], sides[s][1], 0.0);
		Vec3 outward(sides[s][2], sides[s][3], 0.0);

		EdgeCandidate cand;
		cand.dryXY = edge - outward * kLedgeInset;
		cand.wetXY = edge + outward * kLedgeInset;

		if (cand.wetXY.x < wet.mins.x || cand.wetXY.x > wet.maxs.x ||
			cand.wetXY.y < wet.mins.y || cand.wetXY.y > wet.maxs.y)
			continue;
		if (cand.dryXY.x < dry.mins.x || cand.dryXY.x > dry.maxs.x ||
			cand.dryXY.y < dry.mins.y || cand.dryXY.y > dry.maxs.y)
			continue;

		out.push_back(cand);
	}
}

double SwimHeight(const AreaInfo &info, double surfaceZ)
{
	double want = std::min(info.maxs.z, surfaceZ - kSwimDepth);
	return std::max(info.mins.z, std::min(want, info.maxs.z));
}

Vec3 ClampIntoArea(const AreaInfo &info, const Vec3 &point)
{
	return Vec3(std::max(info.mins.x, std::min(point.x, info.maxs.x)),
		std::max(info.mins.y, std::min(point.y, info.maxs.y)),
		std::max(info.mins.z, std::min(point.z, info.maxs.z)));
}

double DryFloorNear(const Area &area, const AreaInfo &info, const Vec3 &contact)
{
	if (const Face *floor = FloorFaceUnder(area.polytope, contact.x, contact.y))
		return FloorHeightAt(*floor, contact.x, contact.y);
	return info.floorZ;
}

MergedConnection MakeSwim(int areaA, int areaB, const Vec3 &pointA, const Vec3 &pointB,
	uint8_t waterVolume, double submergeZ, uint8_t extraFlags)
{
	MergedConnection conn;
	conn.areaA = areaA;
	conn.areaB = areaB;
	conn.pointA = pointA;
	conn.pointB = pointB;
	conn.connType = crossbot_nav::CONN_SWIM;
	conn.connFlags = extraFlags;
	conn.waterVolume = waterVolume;
	conn.submergeZ = submergeZ;
	return conn;
}

}

size_t BuildSwimConnections(const BspFile &bsp, int duckHeadnode,
	const std::vector<BlockingVolume> &blockers, const std::vector<LiquidVolume> &waterVolumes,
	const std::vector<Area> &areas, std::vector<MergedConnection> &conns, SwimStats *statsOut, bool dump)
{
	SwimStats stats;

	if (waterVolumes.empty() || areas.empty())
	{
		if (statsOut)
			*statsOut = stats;
		return 0;
	}

	std::vector<AreaInfo> info(areas.size());
	for (size_t i = 0; i < areas.size(); ++i)
		info[i] = DescribeArea(areas[i]);

	std::vector<size_t> wetAreas;
	for (size_t i = 0; i < areas.size(); ++i)
	{
		if (areas[i].waterVolume != crossbot_nav::kNoWaterVolume)
			wetAreas.push_back(i);
	}

	size_t before = conns.size();

	auto reachable = [&](const Vec3 &from, const Vec3 &to)
	{
		TraceResult tr = TraceHullWithEntities(bsp, duckHeadnode, blockers, from, to);
		return !tr.startsolid && tr.fraction >= 0.999f;
	};

	auto reachableOverLedge = [&](const Vec3 &dryPoint, const Vec3 &wetPoint)
	{
		Vec3 overWater(wetPoint.x, wetPoint.y, dryPoint.z);
		return reachable(dryPoint, overWater) && reachable(overWater, wetPoint);
	};

	for (size_t wi = 0; wi < wetAreas.size(); ++wi)
	{
		size_t i = wetAreas[wi];
		const Area &wet = areas[i];
		uint8_t volIndex = wet.waterVolume;
		if (volIndex >= waterVolumes.size())
			continue;

		const LiquidVolume &vol = waterVolumes[volIndex];
		double surfaceZ = vol.HighSurfaceZ();

		for (size_t j = 0; j < areas.size(); ++j)
		{
			if (j == i)
				continue;

			const Area &other = areas[j];
			bool otherWet = (other.waterVolume == volIndex);

			if (otherWet && j < i)
				continue;
			if (other.waterVolume != crossbot_nav::kNoWaterVolume && !otherWet)
				continue;

			if (!TouchesXY(info[i], info[j]))
				continue;

			if (otherWet && !SharesFace(info[i], info[j]))
				continue;

			double overlapX = std::min(info[i].maxs.x, info[j].maxs.x) -
				std::max(info[i].mins.x, info[j].mins.x);
			double overlapY = std::min(info[i].maxs.y, info[j].maxs.y) -
				std::max(info[i].mins.y, info[j].mins.y);
			if (overlapX < -kSeamSlop || overlapY < -kSeamSlop)
				continue;
			if (std::max(overlapX, overlapY) < kMinSeamOverlap)
				continue;

			Vec3 contact = ContactPoint(info[i], info[j]);

			Vec3 pointA = ClampIntoArea(info[i], Vec3(contact.x, contact.y, SwimHeight(info[i], surfaceZ)));

			if (otherWet)
			{
				Vec3 pointB =
					ClampIntoArea(info[j], Vec3(contact.x, contact.y, SwimHeight(info[j], surfaceZ)));

				if (!reachable(pointA, pointB))
				{
					++stats.blockedByGeometry;
					continue;
				}

				double submergeZ = std::max(wet.submergeZ, other.submergeZ);
				conns.push_back(MakeSwim(static_cast<int>(i), static_cast<int>(j), pointA, pointB,
					volIndex, submergeZ, crossbot_nav::CONN_FLAG_NONE));
				++stats.swim;

				if (dump)
					std::printf("  swim  %4zu <-> %4zu at (%.0f, %.0f, %.0f), submerge %.1f\n", i, j,
						pointA.x, pointA.y, pointA.z, submergeZ);
				continue;
			}

			std::vector<EdgeCandidate> candidates;
			CollectEdgeCandidates(info[j], info[i], candidates);

			bool found = false;
			double dryFloorWorld = 0.0;
			double lift = 0.0;
			Vec3 pointB;

			for (const EdgeCandidate &cand : candidates)
			{
				double dryFloor = DryFloorNear(other, info[j], cand.dryXY);
				double candWorld = dryFloor - kNavFloorToStandOrigin;
				double candLift = candWorld - surfaceZ;

				if (candLift < -kWaterExitStep || candLift > kWaterJumpLedge)
					continue;

				Vec3 tryB = ClampIntoArea(info[j], Vec3(cand.dryXY.x, cand.dryXY.y, dryFloor));
				Vec3 tryA = ClampIntoArea(info[i],
					Vec3(cand.wetXY.x, cand.wetXY.y, SwimHeight(info[i], surfaceZ)));

				if (!reachableOverLedge(tryB, tryA))
					continue;

				pointA = tryA;
				pointB = tryB;
				dryFloorWorld = candWorld;
				lift = candLift;
				found = true;
				break;
			}

			if (!found)
			{
				++stats.blockedByGeometry;
				if (dump)
					std::printf("  walled %4zu -- %4zu: no clear ledge among %zu candidates near "
						"(%.0f, %.0f)\n", j, i, candidates.size(), contact.x, contact.y);
				continue;
			}

			if (lift <= kWaterExitStep)
			{
				conns.push_back(MakeSwim(static_cast<int>(i), static_cast<int>(j), pointA, pointB,
					volIndex, wet.submergeZ, crossbot_nav::CONN_FLAG_NONE));
				++stats.beach;

				if (dump)
					std::printf("  beach %4zu <-> %4zu at (%.0f, %.0f), dry floor %.1f is %.1f above "
						"surface %.1f\n", i, j, contact.x, contact.y, dryFloorWorld, lift, surfaceZ);
				continue;
			}

			if (lift <= kWaterJumpLedge)
			{
				conns.push_back(MakeSwim(static_cast<int>(i), static_cast<int>(j), pointA, pointB,
					volIndex, wet.submergeZ,
					crossbot_nav::CONN_FLAG_ONE_WAY | crossbot_nav::CONN_FLAG_WATERJUMP));
				++stats.waterjump;

				if (dump)
					std::printf("  wjump %4zu  -> %4zu at (%.0f, %.0f), dry floor %.1f is %.1f above "
						"surface %.1f\n", i, j, contact.x, contact.y, dryFloorWorld, lift, surfaceZ);
			}

			conns.push_back(MakeSwim(static_cast<int>(j), static_cast<int>(i), pointB, pointA, volIndex,
				wet.submergeZ, crossbot_nav::CONN_FLAG_ONE_WAY));
			++stats.plunge;

			if (dump)
				std::printf("  dive  %4zu  -> %4zu at (%.0f, %.0f), dropping %.1f into surface %.1f\n",
					j, i, contact.x, contact.y, lift, surfaceZ);
		}
	}

	if (statsOut)
		*statsOut = stats;

	return conns.size() - before;
}

}
