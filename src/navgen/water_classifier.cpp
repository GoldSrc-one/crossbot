#include "water_classifier.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "area_merger.h"
#include "floor_support.h"

namespace navgen
{

namespace
{

constexpr double kMinFragmentVolume = 1.0;
constexpr double kFootprintSlop = 0.5;

void LeafBoundsXY(const Polytope &poly, Vec3 &mins, Vec3 &maxs)
{
	PolytopeAABB(poly, mins, maxs);
}

bool StraddlesPlane(const Polytope &poly, const Plane &plane)
{
	bool front = false;
	bool back = false;

	for (const Face &f : poly.faces)
	{
		for (const Vec3 &v : f.winding)
		{
			double d = plane.normal.Dot(v) - plane.dist;
			if (d > kFootprintSlop)
				front = true;
			else if (d < -kFootprintSlop)
				back = true;

			if (front && back)
				return true;
		}
	}

	return false;
}

void AppendFootprintCuts(const LiquidVolume &vol, std::vector<Plane> &cuts)
{
	const double bounds[4][4] = {
		{1.0, 0.0, 0.0, vol.mins.x},
		{-1.0, 0.0, 0.0, -vol.maxs.x},
		{0.0, 1.0, 0.0, vol.mins.y},
		{0.0, -1.0, 0.0, -vol.maxs.y},
	};

	for (int i = 0; i < 4; ++i)
	{
		Plane p;
		p.normal = Vec3(bounds[i][0], bounds[i][1], bounds[i][2]);
		p.dist = bounds[i][3];
		if (!HasPlane(cuts, p))
			cuts.push_back(p);
	}
}

}

int GoverningWaterVolume(const std::vector<LiquidVolume> &waterVolumes, const Vec3 &floorPoint)
{
	int bestMoving = -1;
	int bestStatic = -1;
	double bestStaticSurface = 0.0;

	Vec3 probe = floorPoint;
	probe.z += kNavFloorToStandOrigin;

	for (size_t i = 0; i < waterVolumes.size(); ++i)
	{
		const LiquidVolume &vol = waterVolumes[i];

		if (floorPoint.x < vol.mins.x - kFootprintSlop || floorPoint.x > vol.maxs.x + kFootprintSlop ||
			floorPoint.y < vol.mins.y - kFootprintSlop || floorPoint.y > vol.maxs.y + kFootprintSlop)
			continue;

		if (vol.HighSurfaceZ() < probe.z)
			continue;

		Vec3 inside = floorPoint;
		inside.z = std::min(vol.maxs.z - kFootprintSlop, std::max(vol.mins.z + kFootprintSlop, probe.z));
		if (!vol.ContainsPoint(inside, kFootprintSlop))
			continue;

		if (vol.moving)
		{
			if (bestMoving < 0)
				bestMoving = static_cast<int>(i);
			continue;
		}

		if (bestStatic < 0 || vol.surfaceZ < bestStaticSurface)
		{
			bestStatic = static_cast<int>(i);
			bestStaticSurface = vol.surfaceZ;
		}
	}

	return (bestMoving >= 0) ? bestMoving : bestStatic;
}

size_t TagConnectionsByWater(const std::vector<LiquidVolume> &waterVolumes,
	std::vector<MergedConnection> &conns)
{
	if (waterVolumes.empty())
		return 0;

	size_t tagged = 0;

	for (MergedConnection &c : conns)
	{
		if (c.connType == crossbot_nav::CONN_LADDER || c.connType == crossbot_nav::CONN_ELEVATOR ||
			c.connType == crossbot_nav::CONN_TELEPORT)
			continue;

		Vec3 lower = (c.pointA.z <= c.pointB.z) ? c.pointA : c.pointB;

		int volIndex = GoverningWaterVolume(waterVolumes, lower);
		if (volIndex < 0)
			continue;

		c.waterVolume = static_cast<uint8_t>(volIndex);
		c.submergeZ = lower.z + kNavFloorToStandOrigin;
		++tagged;
	}

	return tagged;
}

std::vector<ExtractedLeaf> SplitLeavesByWaterLevels(const std::vector<LiquidVolume> &waterVolumes,
	const std::vector<ExtractedLeaf> &leaves, std::vector<bool> &isCrouchLeaf, double mapRadius,
	size_t *splitOut, size_t *taggedOut, bool dump)
{
	if (waterVolumes.empty())
		return leaves;

	std::vector<ExtractedLeaf> out;
	std::vector<bool> crouchOut;
	size_t split = 0;
	size_t tagged = 0;

	auto classify = [&](ExtractedLeaf &leaf)
	{
		const Face *floorFace = LowestFloorFace(leaf.polytope);

		if (leaf.waterColumn)
		{
			Vec3 mins, maxs;
			PolytopeAABB(leaf.polytope, mins, maxs);
			Vec3 probe((mins.x + maxs.x) * 0.5, (mins.y + maxs.y) * 0.5, mins.z);

			int columnVol = -1;
			for (size_t vi = 0; vi < waterVolumes.size(); ++vi)
			{
				if (waterVolumes[vi].HighSurfaceZ() <= mins.z)
					continue;
				Vec3 inside = probe;
				inside.z = std::min(mins.z + 1.0, waterVolumes[vi].HighSurfaceZ() - 1.0);
				if (!waterVolumes[vi].ContainsPoint(inside, 1.0))
					continue;
				columnVol = static_cast<int>(vi);
				break;
			}

			if (columnVol < 0)
				return;

			leaf.waterVolume = static_cast<uint8_t>(columnVol);
			leaf.submergeZ = mins.z;
			++tagged;

			if (dump)
				std::printf("  leaf %4d water column (%.0f, %.0f, %.0f..%.0f) -> water %d, "
					"submerge %.1f\n", leaf.id, probe.x, probe.y, mins.z, maxs.z, columnVol,
					leaf.submergeZ);
			return;
		}

		if (!floorFace)
			return;

		Vec3 floorPoint = WindingCentroid(floorFace->winding);
		int volIndex = GoverningWaterVolume(waterVolumes, floorPoint);
		if (volIndex < 0)
			return;

		leaf.waterVolume = static_cast<uint8_t>(volIndex);
		leaf.submergeZ = floorPoint.z + kNavFloorToStandOrigin;
		++tagged;

		if (dump)
		{
			const LiquidVolume &vol = waterVolumes[static_cast<size_t>(volIndex)];
			std::printf("  leaf %4d floor (%.0f, %.0f, %.0f) -> water %d, submerge %.1f, surface %.1f%s\n",
				leaf.id, floorPoint.x, floorPoint.y, floorPoint.z, volIndex, leaf.submergeZ, vol.surfaceZ,
				(vol.surfaceZ >= leaf.submergeZ) ? " (submerged at spawn)" : "");
		}
	};

	for (size_t li = 0; li < leaves.size(); ++li)
	{
		const ExtractedLeaf &leaf = leaves[li];
		bool crouch = (li < isCrouchLeaf.size()) ? isCrouchLeaf[li] : false;

		Vec3 leafMins, leafMaxs;
		LeafBoundsXY(leaf.polytope, leafMins, leafMaxs);

		std::vector<Plane> cuts;
		for (const LiquidVolume &vol : waterVolumes)
		{
			if (!vol.OverlapsColumn(leafMins, leafMaxs))
				continue;
			if (vol.HighSurfaceZ() < leafMins.z + kNavFloorToStandOrigin)
				continue;

			AppendFootprintCuts(vol, cuts);
		}

		std::vector<Polytope> fragments;
		fragments.push_back(leaf.polytope);

		for (const Plane &cut : cuts)
		{
			std::vector<Polytope> next;
			for (const Polytope &frag : fragments)
			{
				if (!StraddlesPlane(frag, cut))
				{
					next.push_back(frag);
					continue;
				}

				Polytope front = ClipPolytope(frag, cut, true, mapRadius);
				Polytope back = ClipPolytope(frag, cut, false, mapRadius);

				if (!front.Empty() && front.Volume() > kMinFragmentVolume)
					next.push_back(std::move(front));
				if (!back.Empty() && back.Volume() > kMinFragmentVolume)
					next.push_back(std::move(back));
			}
			fragments.swap(next);
		}

		if (fragments.size() > 1)
			++split;

		for (Polytope &frag : fragments)
		{
			ExtractedLeaf piece;
			piece.id = static_cast<int>(out.size());
			piece.polytope = std::move(frag);
			piece.passableIndex = leaf.passableIndex;
			piece.elevatorStopId = leaf.elevatorStopId;
			piece.waterColumn = leaf.waterColumn;
			classify(piece);
			out.push_back(std::move(piece));
			crouchOut.push_back(crouch);
		}
	}

	isCrouchLeaf.swap(crouchOut);

	if (splitOut)
		*splitOut = split;
	if (taggedOut)
		*taggedOut = tagged;

	return out;
}

namespace
{

std::vector<Face> ClipCeilingToSurface(const BspFile &bsp, int headnode,
	const std::vector<BlockingVolume> &blockers, const Polytope &poly, double surfaceZ)
{
	Plane below;
	below.normal = Vec3(0.0, 0.0, -1.0);
	below.dist = -surfaceZ;

	std::vector<Face> out;

	for (const Face &f : poly.faces)
	{
		if (!IsCeilingFace(f))
			continue;

		Winding under = ClipWinding(f.winding, below, true);
		if (under.size() >= 3)
		{
			Face probe;
			probe.plane = f.plane;
			probe.winding = under;
			probe.walkable = false;

			for (Winding &part : ComputeSupportedCeiling(bsp, headnode, blockers, probe))
			{
				if (part.size() >= 3)
					out.push_back({f.plane, std::move(part), false});
			}
		}

		Winding over = ClipWinding(f.winding, below, false);
		if (over.size() < 3)
			continue;

		Face cap;
		cap.plane = below;
		cap.walkable = false;
		cap.winding.reserve(over.size());
		for (const Vec3 &v : over)
			cap.winding.push_back(Vec3(v.x, v.y, surfaceZ));
		out.push_back(std::move(cap));
	}

	return out;
}

}

size_t BuildLeafWaterCeilings(const BspFile &bsp, int headnode,
	const std::vector<BlockingVolume> &blockers, const std::vector<LiquidVolume> &waterVolumes,
	std::vector<ExtractedLeaf> &leaves)
{
	size_t marked = 0;

	for (ExtractedLeaf &leaf : leaves)
	{
		leaf.alwaysSubmerged = false;
		leaf.waterCeiling.clear();

		if (leaf.waterVolume == crossbot_nav::kNoWaterVolume ||
			static_cast<size_t>(leaf.waterVolume) >= waterVolumes.size())
			continue;

		const LiquidVolume &vol = waterVolumes[leaf.waterVolume];
		if (vol.LowSurfaceZ() < leaf.submergeZ)
			continue;

		leaf.alwaysSubmerged = true;
		leaf.waterCeiling = ClipCeilingToSurface(bsp, headnode, blockers, leaf.polytope,
			vol.LowSurfaceZ());
		++marked;
	}

	return marked;
}

}
