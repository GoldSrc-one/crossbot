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

constexpr double kCollinearEpsilon = 0.1;
constexpr double kMinRimLength = 0.1;
constexpr double kPlaneNormalEpsilon = 1e-4;
constexpr double kPlaneDistEpsilon = 0.01;
constexpr double kLethalProbeLift = 1.0;
constexpr double kOnBoundaryEpsilon = 0.1;
constexpr double kMinNotchArea = 1.0;
constexpr double kMinFragmentVolume = 1.0;
constexpr double kMinSupportedArea = 1.0;
constexpr double kUnionVolumeTolerance = 1e-3;
constexpr size_t kMaxAbsorbPasses = 8;

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

namespace
{

void CarveWindingAgainstTreeSided(const BspFile &bsp, int nodeIndex, Winding w, const Vec3 &bias,
	std::vector<Winding> &solid, std::vector<Winding> &empty)
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

	double biasSide = plane.normal.Dot(bias);
	if (biasSide != 0.0)
	{
		bool coplanar = true;
		for (const Vec3 &v : w)
		{
			if (std::fabs(v.Dot(plane.normal) - plane.dist) > kPlaneDistEpsilon)
			{
				coplanar = false;
				break;
			}
		}

		if (coplanar)
		{
			CarveWindingAgainstTreeSided(bsp, node.children[biasSide > 0.0 ? 0 : 1], std::move(w), bias,
				solid, empty);
			return;
		}
	}

	CarveWindingAgainstTreeSided(bsp, node.children[0], ClipWinding(w, plane, true), bias, solid, empty);
	CarveWindingAgainstTreeSided(bsp, node.children[1], ClipWinding(w, plane, false), bias, solid, empty);
}

}

void CarveWindingAgainstTree(const BspFile &bsp, int nodeIndex, Winding w, std::vector<Winding> &solid,
	std::vector<Winding> &empty)
{
	CarveWindingAgainstTreeSided(bsp, nodeIndex, std::move(w), Vec3(0.0, 0.0, 0.0), solid, empty);
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

namespace
{

using Span = std::pair<double, double>;

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

bool SpansCover(std::vector<Span> &spans, double lenSq)
{
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

std::vector<Span> MergeSpans(std::vector<Span> spans)
{
	std::sort(spans.begin(), spans.end());

	std::vector<Span> out;
	for (const Span &span : spans)
	{
		if (!out.empty() && span.first <= out.back().second)
			out.back().second = std::max(out.back().second, span.second);
		else
			out.push_back(span);
	}
	return out;
}

bool SpansExceed(const std::vector<Span> &wanted, const std::vector<Span> &have, double lenSq)
{
	double slack = kMinRimLength / std::sqrt(lenSq);
	std::vector<Span> merged = MergeSpans(have);

	for (const Span &span : MergeSpans(wanted))
	{
		double cursor = span.first;
		for (const Span &h : merged)
		{
			if (h.second <= cursor)
				continue;
			if (h.first >= span.second)
				break;
			if (h.first - cursor > slack)
				return true;
			cursor = std::max(cursor, h.second);
		}

		if (span.second - cursor > slack)
			return true;
	}

	return false;
}

bool EdgeNeedsCut(const std::vector<const Face *> &floors, size_t floorIndex,
	const std::vector<Winding> &supported, size_t self, const Vec3 &a, const Vec3 &b)
{
	Vec3 ab = b - a;
	double lenSq = ab.Dot(ab);
	if (lenSq < 1e-9)
		return false;

	std::vector<Span> support;
	for (size_t i = 0; i < supported.size(); ++i)
	{
		if (i != self)
			AddWindingCoverage(supported[i], a, ab, lenSq, support);
	}

	std::vector<Span> seam;
	for (size_t j = 0; j < floors.size(); ++j)
	{
		if (j != floorIndex)
			AddWindingCoverage(floors[j]->winding, a, ab, lenSq, seam);
	}

	if (SpansExceed(seam, support, lenSq))
		return true;

	std::vector<Span> covered = support;
	AddWindingCoverage(floors[floorIndex]->winding, a, ab, lenSq, covered);
	return !SpansCover(covered, lenSq);
}

bool FaceNotchesFloor(const Face &f)
{
	return f.plane.normal.z > 0.0 && f.plane.normal.z <= kFloorNormalZ && f.winding.size() >= 3 &&
		WindingAreaXY(f.winding) > kMinNotchArea;
}

bool EdgeLiesOnFloorFace(const std::vector<const Face *> &floors, const Vec3 &a, const Vec3 &b)
{
	for (const Face *f : floors)
	{
		if (std::fabs(a.Dot(f->plane.normal) - f->plane.dist) < kOnBoundaryEpsilon &&
			std::fabs(b.Dot(f->plane.normal) - f->plane.dist) < kOnBoundaryEpsilon)
			return true;
	}
	return false;
}

void CarveAgainstSolids(const BspFile &bsp, int headnode, const std::vector<BlockingVolume> &blockers,
	const Winding &w, const Vec3 &bias, std::vector<Winding> &solid, std::vector<Winding> &empty)
{
	std::vector<Winding> remaining;
	CarveWindingAgainstTreeSided(bsp, headnode, w, bias, solid, remaining);

	for (const BlockingVolume &blocker : blockers)
	{
		if (remaining.empty())
			break;

		std::vector<Winding> stillRemaining;
		for (const Winding &piece : remaining)
		{
			std::vector<Winding> blockerSolid;
			std::vector<Winding> blockerEmpty;
			CarveWindingAgainstTreeSided(bsp, blocker.headnode,
				TranslateWinding(piece, blocker.origin * -1.0), bias, blockerSolid, blockerEmpty);

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

double SupportedArea(const BspFile &bsp, int headnode, const std::vector<BlockingVolume> &blockers,
	const Face &floorFace)
{
	std::vector<Winding> supported = ComputeSupportedFloor(bsp, headnode, blockers, floorFace);

	double area = 0.0;
	for (const Winding &w : supported)
		area += WindingArea(w, floorFace.plane.normal);

	return area;
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

struct Bounds
{
	Vec3 mins = Vec3(1e30, 1e30, 1e30);
	Vec3 maxs = Vec3(-1e30, -1e30, -1e30);
};

Bounds PolytopeBounds(const Polytope &poly)
{
	Bounds b;
	for (const Face &f : poly.faces)
		for (const Vec3 &v : f.winding)
		{
			b.mins.x = std::min(b.mins.x, v.x);
			b.mins.y = std::min(b.mins.y, v.y);
			b.mins.z = std::min(b.mins.z, v.z);
			b.maxs.x = std::max(b.maxs.x, v.x);
			b.maxs.y = std::max(b.maxs.y, v.y);
			b.maxs.z = std::max(b.maxs.z, v.z);
		}
	return b;
}

bool BoundsTouch(const Bounds &a, const Bounds &b)
{
	return a.mins.x <= b.maxs.x + kOnBoundaryEpsilon && b.mins.x <= a.maxs.x + kOnBoundaryEpsilon &&
		a.mins.y <= b.maxs.y + kOnBoundaryEpsilon && b.mins.y <= a.maxs.y + kOnBoundaryEpsilon &&
		a.mins.z <= b.maxs.z + kOnBoundaryEpsilon && b.mins.z <= a.maxs.z + kOnBoundaryEpsilon;
}

bool PlanesOpposed(const Plane &a, const Plane &b)
{
	return (a.normal + b.normal).Length() < kPlaneNormalEpsilon &&
		std::fabs(a.dist + b.dist) < kPlaneDistEpsilon;
}

bool SameDirectedPlane(const Plane &a, const Plane &b)
{
	return (a.normal - b.normal).Length() < kPlaneNormalEpsilon &&
		std::fabs(a.dist - b.dist) < kPlaneDistEpsilon;
}

Polytope PolytopeFromPlanes(const std::vector<Plane> &planes, double mapRadius)
{
	Polytope poly;

	for (size_t i = 0; i < planes.size(); ++i)
	{
		Winding w = BaseWindingForPlane(planes[i], mapRadius);
		for (size_t j = 0; j < planes.size() && !w.empty(); ++j)
		{
			if (j == i)
				continue;
			w = ClipWinding(w, planes[j], true);
		}

		if (w.size() >= 3)
			poly.faces.push_back({planes[i], std::move(w)});
	}

	return poly;
}

bool CollectUnionPlanes(const Polytope &self, const Polytope &other, std::vector<Plane> &planes)
{
	for (const Face &fs : self.faces)
	{
		bool shared = false;
		for (const Face &fo : other.faces)
		{
			if (PlanesOpposed(fs.plane, fo.plane))
			{
				shared = true;
				break;
			}
		}

		if (shared)
			continue;

		if (!PolytopeInsidePlane(other, fs.plane))
			return false;

		bool known = false;
		for (const Plane &p : planes)
		{
			if (SameDirectedPlane(p, fs.plane))
			{
				known = true;
				break;
			}
		}

		if (!known)
			planes.push_back(fs.plane);
	}

	return true;
}

bool StacksOnTopOf(const Polytope &host, const Polytope &frag)
{
	for (const Face &fh : host.faces)
	{
		if (fh.plane.normal.z > -kFloorNormalZ)
			continue;

		for (const Face &ff : frag.faces)
		{
			if (PlanesOpposed(fh.plane, ff.plane))
				return true;
		}
	}
	return false;
}

bool ConvexUnion(const Polytope &host, const Polytope &frag, double mapRadius, Polytope &out)
{
	if (!StacksOnTopOf(host, frag))
		return false;

	std::vector<Plane> planes;
	if (!CollectUnionPlanes(host, frag, planes) || !CollectUnionPlanes(frag, host, planes))
		return false;

	Polytope merged = PolytopeFromPlanes(planes, mapRadius);
	if (merged.Empty())
		return false;

	double want = host.Volume() + frag.Volume();
	if (std::fabs(merged.Volume() - want) > kUnionVolumeTolerance * std::max(1.0, want))
		return false;

	for (Face &f : merged.faces)
	{
		f.walkable = false;
		for (const Face &src : host.faces)
		{
			if (SameDirectedPlane(src.plane, f.plane))
			{
				f.walkable = f.walkable || src.walkable;
				break;
			}
		}
		for (const Face &src : frag.faces)
		{
			if (SameDirectedPlane(src.plane, f.plane))
			{
				f.walkable = f.walkable || src.walkable;
				break;
			}
		}
	}

	out = std::move(merged);
	return true;
}

void SplitAgainstHostColumn(const Polytope &frag, const Polytope &host, double mapRadius, Polytope &column,
	std::vector<Polytope> &outside)
{
	column = frag;

	for (const Face &f : host.faces)
	{
		if (std::fabs(f.plane.normal.z) >= kFloorNormalZ)
			continue;
		if (PolytopeInsidePlane(column, f.plane))
			continue;

		Polytope back = ClipPolytope(column, f.plane, false, mapRadius);
		if (!back.Empty() && back.Volume() > kMinFragmentVolume)
			outside.push_back(std::move(back));

		column = ClipPolytope(column, f.plane, true, mapRadius);
		if (column.Empty())
			break;
	}
}

struct FloorlessPiece
{
	ExtractedLeaf leaf;
	Bounds bounds;
	bool hadFloor = false;
};

bool ClaimSubmergedColumn(ExtractedLeaf &leaf, const std::vector<LiquidVolume> *waterVolumes,
	double mapRadius, std::vector<ExtractedLeaf> &out, bool dump)
{
	if (!waterVolumes || waterVolumes->empty() || leaf.polytope.Empty())
		return false;

	Bounds bounds = PolytopeBounds(leaf.polytope);

	for (const LiquidVolume &vol : *waterVolumes)
	{
		double surfaceZ = vol.HighSurfaceZ();
		if (bounds.mins.z >= surfaceZ)
			continue;
		if (!vol.OverlapsColumn(bounds.mins, bounds.maxs))
			continue;

		Vec3 probe((bounds.mins.x + bounds.maxs.x) * 0.5, (bounds.mins.y + bounds.maxs.y) * 0.5,
			std::min(bounds.mins.z + 1.0, surfaceZ - 1.0));
		if (!vol.ContainsPoint(probe, 1.0))
			continue;

		Polytope kept = leaf.polytope;
		if (bounds.maxs.z > surfaceZ)
		{
			Plane cap;
			cap.normal = Vec3(0.0, 0.0, -1.0);
			cap.dist = -surfaceZ;
			kept = ClipPolytope(kept, cap, true, mapRadius);
		}

		if (kept.Empty() || kept.Volume() <= kMinFragmentVolume)
			continue;

		ExtractedLeaf piece;
		piece.id = static_cast<int>(out.size());
		piece.polytope = std::move(kept);
		piece.passableIndex = leaf.passableIndex;
		piece.elevatorStopId = leaf.elevatorStopId;
		piece.waterColumn = true;

		if (dump)
		{
			Vec3 c = piece.polytope.Centroid();
			std::printf("  water column kept at (%.1f, %.1f, %.1f) volume %.0f, capped at surface %.1f\n",
				c.x, c.y, c.z, piece.polytope.Volume(), surfaceZ);
		}

		out.push_back(std::move(piece));
		return true;
	}

	return false;
}

size_t AbsorbFloorlessPieces(std::vector<ExtractedLeaf> &pieces, std::vector<FloorlessPiece> &floorless,
	double mapRadius)
{
	std::vector<Bounds> hostBounds;
	hostBounds.reserve(pieces.size());
	for (const ExtractedLeaf &piece : pieces)
		hostBounds.push_back(PolytopeBounds(piece.polytope));

	size_t absorbed = 0;
	bool progress = true;

	for (size_t pass = 0; progress && pass < kMaxAbsorbPasses; ++pass)
	{
		progress = false;
		std::vector<FloorlessPiece> spawned;

		for (size_t fi = 0; fi < floorless.size(); ++fi)
		{
			if (floorless[fi].leaf.polytope.Empty())
				continue;

			for (size_t hi = 0; hi < pieces.size(); ++hi)
			{
				if (pieces[hi].passableIndex != floorless[fi].leaf.passableIndex)
					continue;
				if (pieces[hi].elevatorStopId != floorless[fi].leaf.elevatorStopId)
					continue;
				if (!BoundsTouch(hostBounds[hi], floorless[fi].bounds))
					continue;

				Polytope merged;
				if (ConvexUnion(pieces[hi].polytope, floorless[fi].leaf.polytope, mapRadius, merged))
				{
					pieces[hi].polytope = std::move(merged);
					hostBounds[hi] = PolytopeBounds(pieces[hi].polytope);
					floorless[fi].leaf.polytope = Polytope();
					++absorbed;
					progress = true;
					break;
				}

				Polytope column;
				std::vector<Polytope> outside;
				SplitAgainstHostColumn(floorless[fi].leaf.polytope, pieces[hi].polytope, mapRadius, column,
					outside);

				if (outside.empty() || column.Empty() || column.Volume() <= kMinFragmentVolume)
					continue;
				if (!ConvexUnion(pieces[hi].polytope, column, mapRadius, merged))
					continue;

				pieces[hi].polytope = std::move(merged);
				hostBounds[hi] = PolytopeBounds(pieces[hi].polytope);
				floorless[fi].leaf.polytope = Polytope();

				for (Polytope &rest : outside)
				{
					FloorlessPiece piece;
					piece.bounds = PolytopeBounds(rest);
					piece.hadFloor = floorless[fi].hadFloor;
					piece.leaf.passableIndex = floorless[fi].leaf.passableIndex;
					piece.leaf.elevatorStopId = floorless[fi].leaf.elevatorStopId;
					piece.leaf.polytope = std::move(rest);
					spawned.push_back(std::move(piece));
				}

				++absorbed;
				progress = true;
				break;
			}
		}

		for (FloorlessPiece &piece : spawned)
			floorless.push_back(std::move(piece));
	}

	return absorbed;
}

}

const Face *LowestFloorFace(const Polytope &poly)
{
	const Face *best = nullptr;
	for (const Face &f : poly.faces)
	{
		if (!f.walkable || f.plane.normal.z <= kFloorNormalZ)
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
		if (!f.walkable || f.plane.normal.z <= kFloorNormalZ)
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
	std::vector<Winding> supported;
	std::vector<Winding> unsupported;
	CarveAgainstSolids(bsp, headnode, blockers, floorFace.winding, floorFace.plane.normal * -1.0, supported,
		unsupported);

	return supported;
}

std::vector<Winding> ComputeSupportedCeiling(const BspFile &bsp, int headnode,
	const std::vector<BlockingVolume> &blockers, const Face &ceilingFace)
{
	std::vector<Winding> covered;
	std::vector<Winding> open;
	CarveAgainstSolids(bsp, headnode, blockers, ceilingFace.winding, ceilingFace.plane.normal * -1.0,
		covered, open);

	return covered;
}

std::vector<ExtractedLeaf> SplitLeavesByFloorSupport(const BspFile &bsp, int headnode,
	const std::vector<BlockingVolume> &blockers, const std::vector<ExtractedLeaf> &leaves, double mapRadius,
	FloorSupportStats *statsOut, bool dump, const std::vector<LiquidVolume> *waterVolumes)
{
	std::vector<ExtractedLeaf> out;
	std::vector<FloorlessPiece> floorless;
	FloorSupportStats stats;

	for (const ExtractedLeaf &leaf : leaves)
	{
		std::vector<const Face *> floors;
		for (const Face &f : leaf.polytope.faces)
		{
			if (f.plane.normal.z <= kFloorNormalZ)
				continue;
			if (f.winding.size() < 3)
				continue;
			floors.push_back(&f);
		}

		if (floors.empty())
		{
			++stats.noFloor;
			continue;
		}

		std::vector<Winding> supported;
		std::vector<size_t> supportedFace;
		for (size_t fi = 0; fi < floors.size(); ++fi)
		{
			for (Winding &w : ComputeSupportedFloor(bsp, headnode, blockers, *floors[fi]))
			{
				supported.push_back(std::move(w));
				supportedFace.push_back(fi);
			}
		}

		std::vector<Plane> cuts;

		for (size_t si = 0; si < supported.size(); ++si)
		{
			const Winding &w = supported[si];
			for (size_t i = 0; i < w.size(); ++i)
			{
				const Vec3 &a = w[i];
				const Vec3 &b = w[(i + 1) % w.size()];

				if (!EdgeNeedsCut(floors, supportedFace[si], supported, si, a, b))
					continue;

				Plane plane;
				if (VerticalPlaneThroughEdge(a, b, plane) && !HasPlane(cuts, plane))
					cuts.push_back(plane);
			}
		}

		for (const Face &f : leaf.polytope.faces)
		{
			if (!FaceNotchesFloor(f))
				continue;

			for (size_t i = 0; i < f.winding.size(); ++i)
			{
				const Vec3 &a = f.winding[i];
				const Vec3 &b = f.winding[(i + 1) % f.winding.size()];

				if (!EdgeLiesOnFloorFace(floors, a, b))
					continue;

				Plane plane;
				if (VerticalPlaneThroughEdge(a, b, plane) && !HasPlane(cuts, plane))
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
			++stats.split;

		for (Polytope &frag : fragments)
		{
			double totalSupported = 0.0;
			const Face *fragFloor = nullptr;

			for (Face &f : frag.faces)
			{
				if (f.plane.normal.z <= kFloorNormalZ || f.winding.size() < 3)
					continue;

				double area = SupportedArea(bsp, headnode, blockers, f);
				f.walkable = area > kMinSupportedArea;
				totalSupported += area;

				if (!fragFloor || f.winding[0].z < fragFloor->winding[0].z)
					fragFloor = &f;
			}

			ExtractedLeaf piece;
			piece.polytope = std::move(frag);
			piece.passableIndex = leaf.passableIndex;
			piece.elevatorStopId = leaf.elevatorStopId;

			if (fragFloor && totalSupported > kMinSupportedArea)
			{
				piece.id = static_cast<int>(out.size());
				out.push_back(std::move(piece));
				continue;
			}

			FloorlessPiece pending;
			pending.bounds = PolytopeBounds(piece.polytope);
			pending.hadFloor = fragFloor != nullptr;
			pending.leaf = std::move(piece);
			floorless.push_back(std::move(pending));
		}
	}

	stats.absorbed = AbsorbFloorlessPieces(out, floorless, mapRadius);


	for (FloorlessPiece &frag : floorless)
	{
		if (frag.leaf.polytope.Empty())
			continue;

		if (ClaimSubmergedColumn(frag.leaf, waterVolumes, mapRadius, out, dump))
		{
			++stats.waterColumns;
			continue;
		}

		if (!frag.hadFloor)
		{
			++stats.noFloor;
			continue;
		}

		if (dump)
		{
			const Face *fragFloor = nullptr;
			for (const Face &f : frag.leaf.polytope.faces)
			{
				if (f.plane.normal.z <= kFloorNormalZ || f.winding.size() < 3)
					continue;
				if (!fragFloor || f.winding[0].z < fragFloor->winding[0].z)
					fragFloor = &f;
			}

			Vec3 c = WindingCentroid(fragFloor->winding);
			std::printf("  unsupported floor at (%.1f, %.1f, %.1f) normal (%.2f, %.2f, %.2f) "
				"volume %.0f\n", c.x, c.y, c.z, fragFloor->plane.normal.x, fragFloor->plane.normal.y,
				fragFloor->plane.normal.z, frag.leaf.polytope.Volume());
		}

		++stats.dropped;
	}

	for (size_t i = 0; i < out.size(); ++i)
		out[i].id = static_cast<int>(i);

	if (statsOut)
		*statsOut = stats;

	return out;
}


size_t DropLethalFloorLeaves(const BspFile &bsp, int headnode, const std::vector<BlockingVolume> &blockers,
	const std::vector<BlockingVolume> &hurtVolumes, std::vector<ExtractedLeaf> &leaves)
{
	if (hurtVolumes.empty() || leaves.empty())
		return 0;

	std::vector<ExtractedLeaf> kept;
	kept.reserve(leaves.size());

	for (const ExtractedLeaf &leaf : leaves)
	{
		const Face *floorFace = LowestFloorFace(leaf.polytope);
		bool lethal = false;

		if (floorFace)
		{
			Vec3 stand = WindingCentroid(floorFace->winding);
			stand.z += kLethalProbeLift;

			for (const BlockingVolume &vol : hurtVolumes)
			{
				if (stand.x < vol.mins.x || stand.x > vol.maxs.x ||
					stand.y < vol.mins.y || stand.y > vol.maxs.y || stand.z >= vol.maxs.z)
					continue;

				Vec3 entry = stand;
				entry.z = vol.mins.z + kLethalProbeLift;
				if (entry.z <= stand.z)
				{
					lethal = true;
					break;
				}

				TraceResult tr = TraceHullWithEntities(bsp, headnode, blockers, stand, entry);
				if (!tr.startsolid && tr.fraction >= 1.0f)
				{
					lethal = true;
					break;
				}
			}
		}

		if (!lethal)
			kept.push_back(leaf);
	}

	size_t dropped = leaves.size() - kept.size();
	leaves.swap(kept);
	return dropped;
}

}

