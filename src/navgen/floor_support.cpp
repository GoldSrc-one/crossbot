#include "floor_support.h"

#include <cmath>

#include "clipnode_tree.h"

namespace navgen
{

namespace
{

constexpr double kFloorNormalZ = 0.7;
constexpr double kSupportProbeDepth = 1.0;
constexpr double kSeamProbeDist = 2.0;
constexpr double kOnBoundaryEpsilon = 0.1;
constexpr double kMinFragmentVolume = 1.0;
constexpr double kMinSupportedArea = 1.0;

double DistanceToSegment(const Vec3 &p, const Vec3 &a, const Vec3 &b)
{
	Vec3 ab = b - a;
	double lenSq = ab.Dot(ab);
	if (lenSq < 1e-9)
		return (p - a).Length();

	double t = (p - a).Dot(ab) / lenSq;
	if (t < 0.0)
		t = 0.0;
	else if (t > 1.0)
		t = 1.0;

	return (p - (a + ab * t)).Length();
}

}

double Distance2DToWinding(const Winding &w, double x, double y)
{
	bool inside = false;
	double best = 1e30;
	Vec3 p(x, y, 0.0);

	for (size_t i = 0, j = w.size() - 1; i < w.size(); j = i++)
	{
		const Vec3 &a = w[i];
		const Vec3 &b = w[j];

		if ((a.y > y) != (b.y > y) && x < a.x + (y - a.y) / (b.y - a.y) * (b.x - a.x))
			inside = !inside;

		best = std::min(best, DistanceToSegment(p, Vec3(a.x, a.y, 0.0), Vec3(b.x, b.y, 0.0)));
	}

	return inside ? 0.0 : best;
}

void CarveWindingAgainstTree(const BspFile &bsp, int nodeIndex, Winding w, std::vector<Winding> &solid,
	std::vector<Winding> &empty)
{
	if (w.size() < 3)
		return;

	if (nodeIndex < 0)
	{
		if (nodeIndex == CONTENTS_SOLID)
			solid.push_back(std::move(w));
		else
			empty.push_back(std::move(w));
		return;
	}

	const dclipnode_t &node = bsp.Clipnodes()[static_cast<size_t>(nodeIndex)];
	const dplane_t &dp = bsp.Planes()[static_cast<size_t>(node.planenum)];
	Plane plane;
	plane.normal = Vec3(dp.normal[0], dp.normal[1], dp.normal[2]);
	plane.dist = dp.dist;

	CarveWindingAgainstTree(bsp, node.children[0], ClipWinding(w, plane, true), solid, empty);
	CarveWindingAgainstTree(bsp, node.children[1], ClipWinding(w, plane, false), solid, empty);
}

Winding TranslateWinding(const Winding &w, const Vec3 &offset)
{
	Winding out;
	out.reserve(w.size());
	for (const Vec3 &v : w)
		out.push_back(v + offset);
	return out;
}

bool PointInSolid(const BspFile &bsp, int headnode, const std::vector<BlockingVolume> &blockers, const Vec3 &p)
{
	if (ClassifyPoint(bsp, headnode, p) == CONTENTS_SOLID)
		return true;

	for (const BlockingVolume &blocker : blockers)
	{
		if (ClassifyPoint(bsp, blocker.headnode, p - blocker.origin) == CONTENTS_SOLID)
			return true;
	}

	return false;
}

bool OnWindingBoundary(const Vec3 &p, const Winding &w)
{
	for (size_t i = 0; i < w.size(); ++i)
	{
		if (DistanceToSegment(p, w[i], w[(i + 1) % w.size()]) < kOnBoundaryEpsilon)
			return true;
	}
	return false;
}

Vec3 WindingCentroid(const Winding &w)
{
	Vec3 sum(0, 0, 0);
	for (const Vec3 &v : w)
		sum = sum + v;
	return w.empty() ? sum : sum * (1.0 / static_cast<double>(w.size()));
}

bool VerticalPlaneThroughEdge(const Vec3 &a, const Vec3 &b, Plane &out)
{
	Vec3 along = b - a;
	along.z = 0.0;
	double len = along.Length();
	if (len < 1e-6)
		return false;

	along = along * (1.0 / len);
	out.normal = Vec3(-along.y, along.x, 0.0);
	out.dist = out.normal.Dot(a);
	return true;
}

namespace
{

double SupportedArea(const BspFile &bsp, int headnode, const std::vector<BlockingVolume> &blockers,
	const Face &floorFace)
{
	std::vector<Winding> supported = ComputeSupportedFloor(bsp, headnode, blockers, floorFace);

	double area = 0.0;
	for (const Winding &w : supported)
		area += WindingArea(w, floorFace.plane.normal);

	return area;
}

}

const Face *LowestFloorFace(const Polytope &poly)
{
	const Face *best = nullptr;
	for (const Face &f : poly.faces)
	{
		if (f.plane.normal.z <= kFloorNormalZ)
			continue;
		if (f.winding.size() < 3)
			continue;
		if (!best || f.winding[0].z < best->winding[0].z)
			best = &f;
	}
	return best;
}

const Face *FloorFaceUnder(const Polytope &poly, double x, double y)
{
	const Face *best = nullptr;
	double bestDist = 0.0;

	for (const Face &f : poly.faces)
	{
		if (f.plane.normal.z <= kFloorNormalZ)
			continue;
		if (f.winding.size() < 3)
			continue;

		double dist = Distance2DToWinding(f.winding, x, y);
		if (!best || dist < bestDist)
		{
			best = &f;
			bestDist = dist;
		}
	}

	return best;
}

double FloorHeightAt(const Face &floorFace, double x, double y)
{
	const Vec3 &n = floorFace.plane.normal;
	if (std::fabs(n.z) < 1e-6)
		return floorFace.winding.empty() ? 0.0 : floorFace.winding[0].z;

	return (floorFace.plane.dist - n.x * x - n.y * y) / n.z;
}

std::vector<Winding> ComputeSupportedFloor(const BspFile &bsp, int headnode,
	const std::vector<BlockingVolume> &blockers, const Face &floorFace)
{
	Vec3 down = floorFace.plane.normal * kSupportProbeDepth;

	Winding probe;
	probe.reserve(floorFace.winding.size());
	for (const Vec3 &v : floorFace.winding)
		probe.push_back(v - down);

	std::vector<Winding> supported;
	std::vector<Winding> unsupported;
	CarveWindingAgainstTree(bsp, headnode, probe, supported, unsupported);

	for (const BlockingVolume &blocker : blockers)
	{
		if (unsupported.empty())
			break;

		std::vector<Winding> stillUnsupported;
		for (const Winding &w : unsupported)
		{
			std::vector<Winding> blockerSolid;
			std::vector<Winding> blockerEmpty;
			CarveWindingAgainstTree(bsp, blocker.headnode, TranslateWinding(w, blocker.origin * -1.0),
				blockerSolid, blockerEmpty);

			for (const Winding &piece : blockerSolid)
				supported.push_back(TranslateWinding(piece, blocker.origin));
			for (const Winding &piece : blockerEmpty)
				stillUnsupported.push_back(TranslateWinding(piece, blocker.origin));
		}
		unsupported.swap(stillUnsupported);
	}

	for (const Winding &w : unsupported)
	{
		Vec3 centroid = WindingCentroid(w);
		bool stepBelow = false;
		for (double depth = kSupportProbeDepth; depth <= kStepHeight && !stepBelow; depth += 4.0)
		{
			Vec3 probePoint = centroid - floorFace.plane.normal * (depth - kSupportProbeDepth);
			if (PointInSolid(bsp, headnode, blockers, probePoint))
				stepBelow = true;
		}

		if (stepBelow)
			supported.push_back(w);
	}

	for (Winding &w : supported)
		for (Vec3 &v : w)
			v = v + down;

	return supported;
}

std::vector<ExtractedLeaf> SplitLeavesByFloorSupport(const BspFile &bsp, int headnode,
	const std::vector<BlockingVolume> &blockers, const std::vector<ExtractedLeaf> &leaves, double mapRadius,
	size_t *droppedOut, size_t *splitOut, size_t *noFloorOut)
{
	std::vector<ExtractedLeaf> out;
	size_t dropped = 0;
	size_t split = 0;
	size_t noFloor = 0;

	for (const ExtractedLeaf &leaf : leaves)
	{
		const Face *floor = LowestFloorFace(leaf.polytope);
		if (!floor)
		{
			++noFloor;
			continue;
		}

		std::vector<Winding> supported = ComputeSupportedFloor(bsp, headnode, blockers, *floor);
		Vec3 down = floor->plane.normal * kSupportProbeDepth;

		std::vector<Plane> cuts;
		for (const Winding &w : supported)
		{
			Vec3 centroid = WindingCentroid(w);

			for (size_t i = 0; i < w.size(); ++i)
			{
				const Vec3 &a = w[i];
				const Vec3 &b = w[(i + 1) % w.size()];
				Vec3 mid = (a + b) * 0.5;

				if (OnWindingBoundary(mid, floor->winding))
					continue;

				Vec3 outward = mid - centroid;
				double len = outward.Length();
				if (len < 1e-6)
					continue;

				Vec3 beyond = mid + outward * (kSeamProbeDist / len) - down;
				if (PointInSolid(bsp, headnode, blockers, beyond))
					continue;

				Plane plane;
				if (VerticalPlaneThroughEdge(a, b, plane))
					cuts.push_back(plane);
			}
		}

		std::vector<Polytope> fragments;
		fragments.push_back(leaf.polytope);

		for (const Plane &cut : cuts)
		{
			std::vector<Polytope> next;
			for (const Polytope &frag : fragments)
			{
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
			const Face *fragFloor = LowestFloorFace(frag);
			if (!fragFloor)
			{
				++noFloor;
				continue;
			}

			if (SupportedArea(bsp, headnode, blockers, *fragFloor) <= kMinSupportedArea)
			{
				++dropped;
				continue;
			}

			ExtractedLeaf piece;
			piece.id = static_cast<int>(out.size());
			piece.polytope = std::move(frag);
			out.push_back(std::move(piece));
		}
	}

	if (droppedOut)
		*droppedOut = dropped;
	if (splitOut)
		*splitOut = split;
	if (noFloorOut)
		*noFloorOut = noFloor;

	return out;
}

}

