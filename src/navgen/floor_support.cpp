#include "floor_support.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <utility>

#include "clipnode_tree.h"

namespace navgen
{

namespace
{

constexpr double kFloorNormalZ = 0.7;
constexpr double kSupportProbeDepth = 1.0;
constexpr double kCollinearEpsilon = 0.1;
constexpr double kMinRimLength = 0.1;
constexpr double kPlaneNormalEpsilon = 1e-4;
constexpr double kPlaneDistEpsilon = 0.01;
constexpr double kOnBoundaryEpsilon = 0.1;
constexpr double kMinFragmentVolume = 1.0;
constexpr double kMinSupportedArea = 1.0;
constexpr double kMinStepRescueArea = 1024.0;

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

using Span = std::pair<double, double>;

// Records the stretch of the edge a + ab * t, t in [0, 1], that the segment c -> d lies along. Only
// collinear segments cover anything; ones that merely cross the edge's line contribute nothing.
void AddCoverage(const Vec3 &a, const Vec3 &ab, double lenSq, const Vec3 &c, const Vec3 &d,
	std::vector<Span> &spans)
{
	double tc = (c - a).Dot(ab) / lenSq;
	double td = (d - a).Dot(ab) / lenSq;

	if ((c - (a + ab * tc)).Length() > kCollinearEpsilon)
		return;
	if ((d - (a + ab * td)).Length() > kCollinearEpsilon)
		return;

	double lo = std::max(std::min(tc, td), 0.0);
	double hi = std::min(std::max(tc, td), 1.0);
	if (hi > lo)
		spans.emplace_back(lo, hi);
}

void AddWindingCoverage(const Winding &w, const Vec3 &a, const Vec3 &ab, double lenSq,
	std::vector<Span> &spans)
{
	for (size_t i = 0; i < w.size(); ++i)
		AddCoverage(a, ab, lenSq, w[i], w[(i + 1) % w.size()], spans);
}

// True when the whole edge either abuts one of `peers` or runs along the leaf's own floor boundary,
// i.e. no part of it is an edge the leaf has to be cut along. The supported windings tile the floor
// exactly, so whatever lies past an uncovered stretch belongs to some other class of floor by
// construction - no probing needed. Coverage is per-stretch rather than per-edge because one edge is
// routinely both: the strip of upper floor beside a ramp opening meets its neighbour along part of the
// edge and overhangs the ramp along the rest, and only the overhanging part must be cut away.
bool EdgeIsCovered(const std::vector<Winding> *peers, size_t self, const Winding &floorWinding,
	const Vec3 &a, const Vec3 &b)
{
	Vec3 ab = b - a;
	double lenSq = ab.Dot(ab);
	if (lenSq < 1e-9)
		return true;

	std::vector<Span> spans;
	AddWindingCoverage(floorWinding, a, ab, lenSq, spans);
	for (size_t i = 0; peers && i < peers->size(); ++i)
	{
		if (i != self)
			AddWindingCoverage((*peers)[i], a, ab, lenSq, spans);
	}

	std::sort(spans.begin(), spans.end());

	double slack = kMinRimLength / std::sqrt(lenSq);
	double reached = 0.0;
	for (const Span &span : spans)
	{
		if (span.first > reached + slack)
			return false;
		reached = std::max(reached, span.second);
	}

	return reached >= 1.0 - slack;
}

// Two support patches that meet along an edge each want to cut there, once from either side. Clipping
// a polytope twice by the same plane leaves it with two coincident faces, which double-counts in
// Polytope::Volume and gives the portal builder a phantom second face to match against, so a plane goes
// in the cut list only once regardless of which side asked for it.
bool HasPlane(const std::vector<Plane> &planes, const Plane &plane)
{
	for (const Plane &p : planes)
	{
		bool flipped = p.normal.Dot(plane.normal) < 0.0;
		Vec3 n = flipped ? p.normal * -1.0 : p.normal;
		double d = flipped ? -p.dist : p.dist;

		if ((n - plane.normal).Length() < kPlaneNormalEpsilon &&
			std::fabs(d - plane.dist) < kPlaneDistEpsilon)
			return true;
	}
	return false;
}

double SupportedArea(const BspFile &bsp, int headnode, const std::vector<BlockingVolume> &blockers,
	const Face &floorFace)
{
	std::vector<Winding> supported = ComputeSupportedFloor(bsp, headnode, blockers, floorFace);

	double area = 0.0;
	for (const Winding &w : supported)
		area += WindingArea(w, floorFace.plane.normal);

	return area;
}

void CarveAgainstSolids(const BspFile &bsp, int headnode, const std::vector<BlockingVolume> &blockers,
	const Winding &w, std::vector<Winding> &solid, std::vector<Winding> &empty)
{
	std::vector<Winding> remaining;
	CarveWindingAgainstTree(bsp, headnode, w, solid, remaining);

	for (const BlockingVolume &blocker : blockers)
	{
		if (remaining.empty())
			break;

		std::vector<Winding> stillRemaining;
		for (const Winding &piece : remaining)
		{
			std::vector<Winding> blockerSolid;
			std::vector<Winding> blockerEmpty;
			CarveWindingAgainstTree(bsp, blocker.headnode, TranslateWinding(piece, blocker.origin * -1.0),
				blockerSolid, blockerEmpty);

			for (const Winding &part : blockerSolid)
				solid.push_back(TranslateWinding(part, blocker.origin));
			for (const Winding &part : blockerEmpty)
				stillRemaining.push_back(TranslateWinding(part, blocker.origin));
		}
		remaining.swap(stillRemaining);
	}

	for (Winding &piece : remaining)
		empty.push_back(std::move(piece));
}

double TotalSupportedArea(const BspFile &bsp, int headnode, const std::vector<BlockingVolume> &blockers,
	const Polytope &poly)
{
	double total = 0.0;
	for (const Face &f : poly.faces)
	{
		if (f.plane.normal.z <= kFloorNormalZ)
			continue;
		if (f.winding.size() < 3)
			continue;
		total += SupportedArea(bsp, headnode, blockers, f);
	}
	return total;
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

SupportedFloor ComputeSupportedFloorParts(const BspFile &bsp, int headnode,
	const std::vector<BlockingVolume> &blockers, const Face &floorFace)
{
	Vec3 down = floorFace.plane.normal * kSupportProbeDepth;

	Winding probe;
	probe.reserve(floorFace.winding.size());
	for (const Vec3 &v : floorFace.winding)
		probe.push_back(v - down);

	SupportedFloor out;
	std::vector<Winding> unsupported;
	CarveAgainstSolids(bsp, headnode, blockers, probe, out.direct, unsupported);

	Vec3 stepDrop = floorFace.plane.normal * (kStepHeight - kSupportProbeDepth);
	for (const Winding &w : unsupported)
	{
		std::vector<Winding> stepSolid;
		std::vector<Winding> stepEmpty;
		CarveAgainstSolids(bsp, headnode, blockers, TranslateWinding(w, stepDrop * -1.0), stepSolid,
			stepEmpty);

		for (const Winding &piece : stepSolid)
		{
			if (WindingArea(piece, floorFace.plane.normal) < kMinStepRescueArea)
				continue;
			out.stepped.push_back(TranslateWinding(piece, stepDrop));
		}
	}

	for (std::vector<Winding> *set : {&out.direct, &out.stepped})
		for (Winding &w : *set)
			for (Vec3 &v : w)
				v = v + down;

	return out;
}

std::vector<Winding> ComputeSupportedFloor(const BspFile &bsp, int headnode,
	const std::vector<BlockingVolume> &blockers, const Face &floorFace)
{
	SupportedFloor parts = ComputeSupportedFloorParts(bsp, headnode, blockers, floorFace);

	std::vector<Winding> supported = std::move(parts.direct);
	for (Winding &w : parts.stepped)
		supported.push_back(std::move(w));

	return supported;
}

std::vector<ExtractedLeaf> SplitLeavesByFloorSupport(const BspFile &bsp, int headnode,
	const std::vector<BlockingVolume> &blockers, const std::vector<ExtractedLeaf> &leaves, double mapRadius,
	size_t *droppedOut, size_t *splitOut, size_t *noFloorOut, bool dump)
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

		SupportedFloor supported = ComputeSupportedFloorParts(bsp, headnode, blockers, *floor);

		std::vector<Plane> cuts;
		auto outline = [&](const Winding &w, const std::vector<Winding> *peers, size_t self)
		{
			for (size_t i = 0; i < w.size(); ++i)
			{
				const Vec3 &a = w[i];
				const Vec3 &b = w[(i + 1) % w.size()];

				if (EdgeIsCovered(peers, self, floor->winding, a, b))
					continue;

				Plane plane;
				if (VerticalPlaneThroughEdge(a, b, plane) && !HasPlane(cuts, plane))
					cuts.push_back(plane);
			}
		};

		// A directly supported patch only has to be cut out where it stops being directly supported, so
		// an edge it shares with another direct patch is a seam and stays uncut. A stepped patch gets
		// cut out along its whole outline instead: the leaf's floor is not the real floor anywhere in
		// it, and neighbouring stepped patches sit at their own heights - two treads of a staircase
		// share an edge that is a step, not a seam.
		for (size_t wi = 0; wi < supported.direct.size(); ++wi)
			outline(supported.direct[wi], &supported.direct, wi);

		for (const Winding &w : supported.stepped)
			outline(w, nullptr, 0);

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

			double totalSupported = TotalSupportedArea(bsp, headnode, blockers, frag);
			if (totalSupported <= kMinSupportedArea)
			{
				if (dump)
				{
					Vec3 c = WindingCentroid(fragFloor->winding);
					std::printf("  unsupported floor at (%.1f, %.1f, %.1f) normal (%.2f, %.2f, %.2f) "
						"area %.1f volume %.0f\n", c.x, c.y, c.z, fragFloor->plane.normal.x,
						fragFloor->plane.normal.y, fragFloor->plane.normal.z,
						totalSupported, frag.Volume());
				}
				++dropped;
				continue;
			}

			ExtractedLeaf piece;
			piece.id = static_cast<int>(out.size());
			piece.polytope = std::move(frag);
			piece.passableIndex = leaf.passableIndex;
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

