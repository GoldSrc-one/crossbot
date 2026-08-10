#include "area_merger.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <map>
#include <numeric>
#include <set>
#include <stdexcept>
#include <unordered_map>

#include "floor_support.h"
#include "waypoint_plan.h"

namespace navgen
{

float ConnectionCostSeconds(const MergedConnection &conn, const std::vector<Area> &areas)
{
	float seconds = 0.0f;

	for (int areaId : {conn.areaA, conn.areaB})
	{
		if (areaId < 0 || static_cast<size_t>(areaId) >= areas.size())
			continue;

		const Area &area = areas[static_cast<size_t>(areaId)];
		if (area.areaFlags & (crossbot_nav::NAV_AREA_DOOR | crossbot_nav::NAV_AREA_BREAKABLE))
			seconds = std::max(seconds, static_cast<float>(area.openSeconds));
	}

	if (conn.connType == crossbot_nav::CONN_AIRMOVE || conn.connType == crossbot_nav::CONN_LADDER)
	{
		float damage = static_cast<float>(conn.fallDamage);
		seconds += static_cast<float>(conn.flightTime);
		seconds += (damage <= 0.0f) ? 0.0f : (damage >= 100.0f ? 1.0e4f : damage * 0.5f);
	}

	if (conn.connType == crossbot_nav::CONN_LADDER)
		seconds += static_cast<float>((conn.pointB - conn.pointA).Length() /
			std::max(conn.exitSpeed, 1.0));

	return seconds;
}

namespace
{

constexpr double kPlaneWallNormalZ = 0.3;
constexpr double kOnPlaneTolerance = 1.0;
constexpr double kLaunchEdgeTolerance = 8.0;
constexpr double kFloorFootprintTolerance = 1.0;
constexpr double kFloorAdjacencyEpsilon = 0.5;
constexpr double kMinStepSpan = 1e-9;
constexpr double kMinSeamLength = 1.0;

bool FloorFaceSpan(const Face &f, const Vec3 &a, const Vec3 &ab, double epsilon, double &lo, double &hi)
{
	double area2 = 0.0;
	for (size_t i = 0; i < f.winding.size(); ++i)
	{
		const Vec3 &p = f.winding[i];
		const Vec3 &q = f.winding[(i + 1) % f.winding.size()];
		area2 += p.x * q.y - q.x * p.y;
	}

	double side = (area2 < 0.0) ? -1.0 : 1.0;

	lo = 0.0;
	hi = 1.0;

	for (size_t i = 0; i < f.winding.size(); ++i)
	{
		const Vec3 &p = f.winding[i];
		const Vec3 &q = f.winding[(i + 1) % f.winding.size()];

		Vec3 n((q.y - p.y) * side, (p.x - q.x) * side, 0.0);
		double len = n.Length();
		if (len < 1e-9)
			continue;
		n = n * (1.0 / len);

		double num = epsilon - ((a.x - p.x) * n.x + (a.y - p.y) * n.y);
				double den = ab.x * n.x + ab.y * n.y;

				if (std::fabs(den) < 1e-12)
				{
					if (num < 0.0)
						return false;
					continue;
				}

				if (den > 0.0)
					hi = std::min(hi, num / den);
				else
					lo = std::max(lo, num / den);

				if (lo > hi)
					return false;
			}

			return true;
}

struct FloorSpan
{
	const Face *face = nullptr;
	double lo = 0.0;
	double hi = 1.0;
};

void CollectFloorSpans(const Polytope &poly, const Vec3 &a, const Vec3 &ab, std::vector<FloorSpan> &out,
	std::vector<double> &breaks)
{
	for (const Face &f : poly.faces)
	{
		if (!IsFloorFace(f))
			continue;

		FloorSpan span;
		if (!FloorFaceSpan(f, a, ab, kFloorAdjacencyEpsilon, span.lo, span.hi))
			continue;

		span.face = &f;
		out.push_back(span);
		breaks.push_back(span.lo);
		breaks.push_back(span.hi);
	}
}

using Span = std::pair<double, double>;

struct SeamSteps
{
	bool valid = false;
	std::vector<Span> walkable;
	double minStep = 0.0;
	Vec3 at;
	const Face *floorA = nullptr;
	const Face *floorB = nullptr;

	bool CoversWholeSeam() const
	{
		return walkable.size() == 1 && walkable[0].first <= kMinStepSpan &&
			walkable[0].second >= 1.0 - kMinStepSpan;
	}
};

bool LinearWithin(double d0, double d1, double limit, double &loOut, double &hiOut)
{
	double slope = d1 - d0;

	if (std::fabs(slope) < 1e-12)
	{
		if (std::fabs(d0) > limit)
			return false;
		loOut = 0.0;
		hiOut = 1.0;
		return true;
	}

	double sa = (limit - d0) / slope;
	double sb = (-limit - d0) / slope;

	double lo = std::max(0.0, std::min(sa, sb));
	double hi = std::min(1.0, std::max(sa, sb));
	if (hi <= lo)
		return false;

	loOut = lo;
	hiOut = hi;
	return true;
}

SeamSteps WalkableSpansAcross(const Polytope &a, const Polytope &b, const Vec3 &pointA, const Vec3 &pointB,
	double maxStep)
{
	Vec3 ab = pointB - pointA;

	std::vector<FloorSpan> spansA, spansB;
	std::vector<double> breaks{0.0, 1.0};
	CollectFloorSpans(a, pointA, ab, spansA, breaks);
	CollectFloorSpans(b, pointA, ab, spansB, breaks);

	std::sort(breaks.begin(), breaks.end());

	SeamSteps out;
	std::vector<const Face *> candA, candB;
	std::vector<Span> runs;

	for (size_t i = 0; i + 1 < breaks.size(); ++i)
	{
		double t0 = breaks[i];
		double t1 = breaks[i + 1];
		if (t1 - t0 < kMinStepSpan)
			continue;

		double mid = (t0 + t1) * 0.5;

		candA.clear();
		candB.clear();
		for (const FloorSpan &s : spansA)
			if (mid >= s.lo && mid <= s.hi)
				candA.push_back(s.face);
		for (const FloorSpan &s : spansB)
			if (mid >= s.lo && mid <= s.hi)
				candB.push_back(s.face);

		if (candA.empty() || candB.empty())
			continue;

		Vec3 p0 = pointA + ab * t0;
		Vec3 p1 = pointA + ab * t1;

		for (const Face *fa : candA)
		{
			for (const Face *fb : candB)
			{
				double d0 = FloorHeightAt(*fa, p0.x, p0.y) - FloorHeightAt(*fb, p0.x, p0.y);
				double d1 = FloorHeightAt(*fa, p1.x, p1.y) - FloorHeightAt(*fb, p1.x, p1.y);

				double lo = 0.0;
				double hi = 0.0;
				if (LinearWithin(d0, d1, maxStep, lo, hi))
					runs.emplace_back(t0 + (t1 - t0) * lo, t0 + (t1 - t0) * hi);

				double step = 0.0;
				double t = t0;

				if ((d0 <= 0.0) == (d1 <= 0.0) && d0 != 0.0 && d1 != 0.0)
				{
					bool atStart = std::fabs(d0) <= std::fabs(d1);
					step = atStart ? std::fabs(d0) : std::fabs(d1);
					t = atStart ? t0 : t1;
				}
				else
				{
					double denom = d0 - d1;
					if (std::fabs(denom) > 1e-12)
						t = t0 + (t1 - t0) * (d0 / denom);
				}

				if (!out.valid || step < out.minStep)
				{
					out.valid = true;
					out.minStep = step;
					out.at = pointA + ab * t;
					out.floorA = fa;
					out.floorB = fb;
				}
			}
		}
	}

	std::sort(runs.begin(), runs.end());
	for (const Span &run : runs)
	{
		if (!out.walkable.empty() && run.first <= out.walkable.back().second + kMinStepSpan)
			out.walkable.back().second = std::max(out.walkable.back().second, run.second);
		else
			out.walkable.push_back(run);
	}

	return out;
}

double DistanceToWall(const Plane &plane, const Vec3 &p, bool &ok)
{
	Vec3 n;
	double d;
	ok = WallPlaneAsLine(plane, n, d);
	return ok ? std::fabs(p.x * n.x + p.y * n.y - d) : 0.0;
}

int WallHoldingSegment(const Area &area, const Vec3 &a, const Vec3 &b)
{
	int best = -1;
	double bestDist = kOnPlaneTolerance;

	for (size_t i = 0; i < area.boundPlanes.size(); ++i)
	{
		bool ok = false;
		double da = DistanceToWall(area.boundPlanes[i], a, ok);
		if (!ok)
			continue;
		double db = DistanceToWall(area.boundPlanes[i], b, ok);

		double worst = std::max(da, db);
		if (worst < bestDist)
		{
			bestDist = worst;
			best = static_cast<int>(i);
		}
	}

	return best;
}

int WallNearestPoint(const Area &area, const Vec3 &p, double tolerance)
{
	int best = -1;
	double bestDist = tolerance;

	for (size_t i = 0; i < area.boundPlanes.size(); ++i)
	{
		bool ok = false;
		double d = DistanceToWall(area.boundPlanes[i], p, ok);
		if (ok && d < bestDist)
		{
			bestDist = d;
			best = static_cast<int>(i);
		}
	}

	return best;
}

}

bool WallPlaneAsLine(const Plane &plane, Vec3 &normalOut, double &distOut)
{
	if (std::fabs(plane.normal.z) > kPlaneWallNormalZ)
		return false;

	double len = std::sqrt(plane.normal.x * plane.normal.x + plane.normal.y * plane.normal.y);
	if (len < 1e-6)
		return false;

	normalOut = Vec3(plane.normal.x / len, plane.normal.y / len, 0.0);
	distOut = plane.dist / len;
	return true;
}

int WallFacingDirection(const Area &area, const Vec3 &dir2D)
{
	int best = -1;
	double bestDot = 0.0;

	for (size_t i = 0; i < area.boundPlanes.size(); ++i)
	{
		Vec3 n;
		double d;
		if (!WallPlaneAsLine(area.boundPlanes[i], n, d))
			continue;

		double dot = dir2D.x * n.x + dir2D.y * n.y;
		if (best < 0 || dot > bestDot)
		{
			bestDot = dot;
			best = static_cast<int>(i);
		}
	}

	return (bestDot > 0.0) ? best : -1;
}

size_t ResolveConnectionPlanes(const std::vector<Area> &areas, std::vector<MergedConnection> &conns)
{
	size_t unresolved = 0;

	for (MergedConnection &c : conns)
	{
		c.planeA = -1;
		c.planeB = -1;

		if (static_cast<size_t>(c.areaA) >= areas.size() || static_cast<size_t>(c.areaB) >= areas.size())
			continue;

		const Area &areaA = areas[static_cast<size_t>(c.areaA)];
		const Area &areaB = areas[static_cast<size_t>(c.areaB)];

		if (c.connType != crossbot_nav::CONN_WALK)
			continue;

		c.planeA = WallHoldingSegment(areaA, c.pointA, c.pointB);
		c.planeB = WallHoldingSegment(areaB, c.pointA, c.pointB);

		if (c.planeA < 0 || c.planeB < 0)
			++unresolved;
	}

	return unresolved;
}

static Vector ToPlanVector(const Vec3 &v)
{
	return Vector(static_cast<float>(v.x), static_cast<float>(v.y), static_cast<float>(v.z));
}

size_t BakeConnectionGates(const std::vector<Area> &areas, std::vector<MergedConnection> &conns)
{
	ResolveConnectionPlanes(areas, conns);

	std::vector<NavPlan::PlanArea> planAreas(areas.size());
	for (size_t i = 0; i < areas.size(); ++i)
	{
		Vec3 c = areas[i].polytope.Centroid();
		planAreas[i].centroid = Vector(static_cast<float>(c.x), static_cast<float>(c.y),
			static_cast<float>(c.z));

		planAreas[i].boundPlanes.reserve(areas[i].boundPlanes.size());
		for (const Plane &p : areas[i].boundPlanes)
		{
			NavPlan::PlanPlane pp;
			pp.normal = Vector(static_cast<float>(p.normal.x), static_cast<float>(p.normal.y),
				static_cast<float>(p.normal.z));
			pp.dist = static_cast<float>(p.dist);
			planAreas[i].boundPlanes.push_back(pp);
		}

		NavPlan::BuildAreaGates(planAreas[i]);
	}

	auto planeIndex = [](int idx)
	{
		return (idx >= 0 && idx < static_cast<int>(crossbot_nav::kNoAreaPlane))
			? static_cast<uint8_t>(idx) : crossbot_nav::kNoAreaPlane;
	};

	std::vector<NavPlan::PlanConnection> planConns(conns.size());
	for (size_t i = 0; i < conns.size(); ++i)
	{
		planConns[i].areaA = static_cast<uint32_t>(conns[i].areaA);
		planConns[i].areaB = static_cast<uint32_t>(conns[i].areaB);
				planConns[i].planeA = planeIndex(conns[i].planeA);
				planConns[i].planeB = planeIndex(conns[i].planeB);
				planConns[i].connType = conns[i].connType;
			}

			auto floorAt = [&](uint32_t areaId, float x, float y, float &zOut)
			{
				if (static_cast<size_t>(areaId) >= areas.size())
					return false;
				const Face *floor = FloorFaceUnder(areas[areaId].polytope, x, y);
				if (!floor || Distance2DToWinding(floor->winding, x, y) > kFloorFootprintTolerance)
					return false;
				zOut = static_cast<float>(FloorHeightAt(*floor, x, y));
				return true;
			};

	NavPlan::BuildConnectionGates(planAreas, planConns, floorAt, static_cast<float>(kStepHeight));

	for (size_t i = 0; i < conns.size(); ++i)
	{
		if (conns[i].gateAValid)
		{
			planConns[i].gateA.valid = true;
			planConns[i].gateA.clamped = false;
			planConns[i].gateA.left = ToPlanVector(conns[i].gateALeft);
			planConns[i].gateA.right = ToPlanVector(conns[i].gateARight);
		}
		if (conns[i].gateBValid)
		{
			planConns[i].gateB.valid = true;
			planConns[i].gateB.clamped = false;
			planConns[i].gateB.left = ToPlanVector(conns[i].gateBLeft);
			planConns[i].gateB.right = ToPlanVector(conns[i].gateBRight);
		}
	}

	auto onFloor = [&](const Vector &p, int areaId)
	{
		Vec3 out(p.x, p.y, p.z);
		if (static_cast<size_t>(areaId) >= areas.size())
			return out;
		const Face *floor = FloorFaceUnder(areas[static_cast<size_t>(areaId)].polytope, out.x, out.y);
		if (floor)
			out.z = FloorHeightAt(*floor, out.x, out.y);
		return out;
	};

	size_t missing = 0;
	for (size_t i = 0; i < conns.size(); ++i)
	{
		MergedConnection &c = conns[i];
		const NavPlan::PlanConnection &pc = planConns[i];

		c.gateAValid = pc.gateA.valid;
		c.gateBValid = pc.gateB.valid;
		c.gateAClamped = pc.gateA.clamped;
		c.gateBClamped = pc.gateB.clamped;

		if (c.connType == crossbot_nav::CONN_LADDER)
			continue;

		if (c.gateAValid)
		{
			c.gateALeft = onFloor(pc.gateA.left, c.areaA);
			c.gateARight = onFloor(pc.gateA.right, c.areaA);
		}
		if (c.gateBValid)
		{
			c.gateBLeft = onFloor(pc.gateB.left, c.areaB);
			c.gateBRight = onFloor(pc.gateB.right, c.areaB);
		}

		if (c.connType != crossbot_nav::CONN_TELEPORT && c.connType != crossbot_nav::CONN_LADDER &&
			(!c.gateAValid || !c.gateBValid))
			++missing;
	}

	return missing;
}

size_t FilterWalkConnectionsByStep(const std::vector<Area> &areas, std::vector<MergedConnection> &conns,
	double maxStep, size_t *splitOut, bool dump)
{
	size_t removed = 0;
	size_t split = 0;

	std::vector<MergedConnection> kept;
	kept.reserve(conns.size());

	for (const MergedConnection &c : conns)
	{
		if (c.connType != crossbot_nav::CONN_WALK ||
			static_cast<size_t>(c.areaA) >= areas.size() || static_cast<size_t>(c.areaB) >= areas.size())
		{
			kept.push_back(c);
			continue;
		}

		SeamSteps step = WalkableSpansAcross(areas[static_cast<size_t>(c.areaA)].polytope,
			areas[static_cast<size_t>(c.areaB)].polytope, c.pointA, c.pointB, maxStep);

		if (!step.valid || step.CoversWholeSeam())
		{
			kept.push_back(c);
			continue;
		}

		if (dump)
		{
			double zA = FloorHeightAt(*step.floorA, step.at.x, step.at.y);
			double zB = FloorHeightAt(*step.floorB, step.at.x, step.at.y);
			double distA = Distance2DToWinding(step.floorA->winding, step.at.x, step.at.y);
			double distB = Distance2DToWinding(step.floorB->winding, step.at.x, step.at.y);
			std::printf("  step %s %3d <-> %3d at (%.1f, %.1f): zA %.1f zB %.1f min step %.1f, %zu "
				"walkable run(s) [faceA nz %.2f off %.1f, faceB nz %.2f off %.1f]\n",
				step.walkable.empty() ? "drop" : "cut ", c.areaA, c.areaB, step.at.x, step.at.y, zA, zB,
				step.minStep, step.walkable.size(), step.floorA->plane.normal.z, distA,
				step.floorB->plane.normal.z, distB);
		}

		Vec3 ab = c.pointB - c.pointA;
		double seamLength = ab.Length();
		size_t emitted = 0;

		for (const Span &run : step.walkable)
		{
			if ((run.second - run.first) * seamLength < kMinSeamLength)
				continue;

			MergedConnection nc = c;
			nc.pointA = c.pointA + ab * run.first;
			nc.pointB = c.pointA + ab * run.second;
			kept.push_back(nc);
			++emitted;
		}

		if (emitted > 0)
			++split;
		else
			++removed;
	}

	conns.swap(kept);

	if (splitOut)
		*splitOut = split;

	return removed;
}

size_t SnapConnectionsToFloor(const std::vector<Area> &areas, std::vector<MergedConnection> &conns)
{
	size_t snapped = 0;

	auto snap = [&](Vec3 &p, int areaId)
	{
		if (static_cast<size_t>(areaId) >= areas.size())
			return false;
		const Face *floor = FloorFaceUnder(areas[static_cast<size_t>(areaId)].polytope, p.x, p.y);
		if (!floor)
			return false;
		p.z = FloorHeightAt(*floor, p.x, p.y);
		return true;
	};

	for (MergedConnection &c : conns)
	{
		bool ok = false;

		if (c.connType == crossbot_nav::CONN_WALK)
		{

			ok = snap(c.pointA, c.areaA);
			ok = snap(c.pointB, c.areaA) || ok;
		}
		else
		{

			ok = snap(c.pointA, c.areaA);
		}

		if (ok)
			++snapped;
	}

	return snapped;
}

namespace
{

constexpr double kPlaneMatchEpsilon = 1e-4;
constexpr double kDistMatchEpsilon = 0.01;

void AddPlaneUnique(std::vector<Plane> &out, const Plane &p)
{
	for (const Plane &q : out)
	{
		if (std::fabs(q.normal.x - p.normal.x) < kPlaneMatchEpsilon &&
			std::fabs(q.normal.y - p.normal.y) < kPlaneMatchEpsilon &&
			std::fabs(q.normal.z - p.normal.z) < kPlaneMatchEpsilon &&
			std::fabs(q.dist - p.dist) < kDistMatchEpsilon)
			return;
	}
	out.push_back(p);
}

void PolytopeBounds(const Polytope &poly, Vec3 &mins, Vec3 &maxs)
{
	mins = Vec3(1e30, 1e30, 1e30);
	maxs = Vec3(-1e30, -1e30, -1e30);
	for (const Face &f : poly.faces)
	{
		for (const Vec3 &v : f.winding)
		{
			mins.x = std::min(mins.x, v.x);
			mins.y = std::min(mins.y, v.y);
			mins.z = std::min(mins.z, v.z);
			maxs.x = std::max(maxs.x, v.x);
			maxs.y = std::max(maxs.y, v.y);
			maxs.z = std::max(maxs.z, v.z);
		}
	}
}

std::vector<Plane> BuildBoundPlanes(const std::vector<const Face *> &visible, const Polytope &poly,
	const Winding &footprint, bool prism)
{
	std::vector<Plane> out;

	if (!prism)
	{
		for (const Face *f : visible)
			AddPlaneUnique(out, f->plane);
		return out;
	}

	Vec3 centroid = WindingCentroid(footprint);
	for (size_t i = 0; i < footprint.size(); ++i)
	{
		Plane plane;
		if (!VerticalPlaneThroughEdge(footprint[i], footprint[(i + 1) % footprint.size()], plane))
			continue;

		if (plane.normal.Dot(centroid) - plane.dist < 0.0)
		{
			plane.normal = plane.normal * -1.0;
			plane.dist = -plane.dist;
		}
		AddPlaneUnique(out, plane);
	}

	Vec3 mins, maxs;
	PolytopeBounds(poly, mins, maxs);

	Plane bottom;
	bottom.normal = Vec3(0, 0, 1);
	bottom.dist = mins.z;
	out.push_back(bottom);

	Plane top;
	top.normal = Vec3(0, 0, -1);
	top.dist = -maxs.z;
	out.push_back(top);

	return out;
}

bool LeafIsCrouch(const std::vector<bool> &isCrouchLeaf, int leafId)
{
	return static_cast<size_t>(leafId) < isCrouchLeaf.size() && isCrouchLeaf[static_cast<size_t>(leafId)];
}

void TagCrouchConnections(const std::vector<Area> &areas, std::vector<MergedConnection> &conns)
{
	for (MergedConnection &c : conns)
	{
		bool crouchA = (areas[static_cast<size_t>(c.areaA)].areaFlags & crossbot_nav::NAV_AREA_CROUCH) != 0;
		bool crouchB = (areas[static_cast<size_t>(c.areaB)].areaFlags & crossbot_nav::NAV_AREA_CROUCH) != 0;
		if (crouchA || crouchB)
			c.connFlags |= crossbot_nav::CONN_FLAG_NEEDS_CROUCH;
	}
}

int LeafPassable(const std::vector<ExtractedLeaf> &leaves, int leafId)
{
	return (static_cast<size_t>(leafId) < leaves.size()) ? leaves[static_cast<size_t>(leafId)].passableIndex : -1;
}

void ApplyPassable(Area &area, const std::vector<PassableVolume> &passables, int passableIndex)
{
	if (passableIndex < 0 || static_cast<size_t>(passableIndex) >= passables.size())
		return;

	const PassableVolume &p = passables[static_cast<size_t>(passableIndex)];
	area.areaFlags |= p.areaFlag;
	area.doorActivation = p.activation;
	area.doorActivatorPos = p.activatorPos;
	area.openSeconds = p.openSeconds;
}

}

namespace
{

constexpr double kCoverageTolerance = 0.02;
constexpr double kConvexEpsilon = 0.5;
constexpr double kZTouchEpsilon = 0.5;
constexpr double kMinFootprintArea = 1.0;
constexpr double kMinSharedFaceArea = 1.0;

bool PortalSharesWholeFace(double portalArea, double faceAreaA, double faceAreaB)
{
	if (faceAreaA < 1e-6 || faceAreaB < 1e-6)
		return false;
	return std::fabs(portalArea - faceAreaA) <= kCoverageTolerance * faceAreaA &&
		std::fabs(portalArea - faceAreaB) <= kCoverageTolerance * faceAreaB;
}

bool SeamIsWalkable(const Polytope &a, const Polytope &b, const Vec3 &pointA, const Vec3 &pointB,
	double maxStep)
{
	SeamSteps step = WalkableSpansAcross(a, b, pointA, pointB, maxStep);
	return step.valid && step.CoversWholeSeam();
}

bool MergeStaysConvex(const std::vector<const Face *> &facesA, const std::vector<const Face *> &facesB,
	double epsilon)
{
	for (const Face *fa : facesA)
		for (const Face *fb : facesB)
			for (const Vec3 &v : fb->winding)
				if (v.Dot(fa->plane.normal) - fa->plane.dist < -epsilon)
					return false;

	for (const Face *fb : facesB)
		for (const Face *fa : facesA)
			for (const Vec3 &v : fa->winding)
				if (v.Dot(fb->plane.normal) - fb->plane.dist < -epsilon)
					return false;

	return true;
}

const Vec3 kUp(0, 0, 1);

Winding MergeFootprints(const Winding &a, const Winding &b)
{
	std::vector<Vec3> pts;
	pts.reserve(a.size() + b.size());
	pts.insert(pts.end(), a.begin(), a.end());
	pts.insert(pts.end(), b.begin(), b.end());
	return ConvexHullXY(pts);
}

double FootprintOverlapArea(const Winding &a, const Winding &b)
{
	Winding inter = IntersectCoplanarPolygons(a, b, kUp);
	return (inter.size() < 3) ? 0.0 : WindingArea(inter, kUp);
}

bool FloorsAreWalkable(const Face &f, const Face &g, double maxStep)
{
	for (const Vec3 &v : g.winding)
	{
		if (Distance2DToWinding(f.winding, v.x, v.y) > kFloorAdjacencyEpsilon)
			continue;
		if (std::fabs(FloorHeightAt(f, v.x, v.y) - v.z) > maxStep)
			return false;
	}
	return true;
}

bool MergedFloorStaysWalkable(const std::vector<const Face *> &facesA, const std::vector<const Face *> &facesB,
	double maxStep)
{
	for (const Face *f : facesA)
	{
		if (!IsFloorFace(*f))
			continue;
		for (const Face *g : facesB)
		{
			if (!IsFloorFace(*g))
				continue;
			if (!FloorsAreWalkable(*f, *g, maxStep) || !FloorsAreWalkable(*g, *f, maxStep))
				return false;
		}
	}
	return true;
}

struct Bounds
{
	Vec3 mins = Vec3(1e30, 1e30, 1e30);
	Vec3 maxs = Vec3(-1e30, -1e30, -1e30);

	void Add(const Vec3 &v)
	{
		mins.x = std::min(mins.x, v.x);
		mins.y = std::min(mins.y, v.y);
		mins.z = std::min(mins.z, v.z);
		maxs.x = std::max(maxs.x, v.x);
		maxs.y = std::max(maxs.y, v.y);
		maxs.z = std::max(maxs.z, v.z);
	}

	void Add(const Bounds &o)
	{
		Add(o.mins);
		Add(o.maxs);
	}

	bool OverlapsXY(const Bounds &o, double epsilon) const
	{
		return maxs.x - o.mins.x > epsilon && o.maxs.x - mins.x > epsilon &&
			maxs.y - o.mins.y > epsilon && o.maxs.y - mins.y > epsilon;
	}

	bool OverlapsZ(const Bounds &o, double epsilon) const
	{
		return maxs.z - o.mins.z > epsilon && o.maxs.z - mins.z > epsilon;
	}
};

struct FloorEdge
{
	int owner;
	double lo, hi;
	Vec3 at, along;
};

struct LineKey
{
	long long dx, dy, offset;

	bool operator==(const LineKey &o) const { return dx == o.dx && dy == o.dy && offset == o.offset; }
};

struct LineKeyHash
{
	size_t operator()(const LineKey &k) const
	{
		auto mix = [](size_t h, long long v)
		{
			size_t hv = std::hash<long long>()(v);
			return h ^ (hv + static_cast<size_t>(0x9e3779b9UL) + (h << 6) + (h >> 2));
		};
		return mix(mix(mix(0, k.dx), k.dy), k.offset);
	}
};

bool EdgeAsLine(const Vec3 &a, const Vec3 &b, LineKey &keyOut, FloorEdge &edgeOut)
{
	double dx = b.x - a.x;
	double dy = b.y - a.y;
	double length = std::sqrt(dx * dx + dy * dy);
	if (length < kMinSeamLength)
		return false;

	dx /= length;
	dy /= length;

	if (dx < -1e-6 || (std::fabs(dx) <= 1e-6 && dy < 0.0))
	{
		dx = -dx;
		dy = -dy;
	}

	auto quantize = [](double v) { return static_cast<long long>(std::llround(v * 100.0)); };
	keyOut = {quantize(dx), quantize(dy), quantize(dx * a.y - dy * a.x)};

	edgeOut.at = a;
	edgeOut.along = Vec3(dx, dy, 0.0);
	edgeOut.lo = a.x * dx + a.y * dy;
	edgeOut.hi = b.x * dx + b.y * dy;
	if (edgeOut.hi < edgeOut.lo)
		std::swap(edgeOut.lo, edgeOut.hi);

	return true;
}

Vec3 PointAlong(const FloorEdge &e, double t)
{
	double base = e.at.x * e.along.x + e.at.y * e.along.y;
	return Vec3(e.at.x + e.along.x * (t - base), e.at.y + e.along.y * (t - base), e.at.z);
}

struct SeamSegment
{
	int a = -1;
	int b = -1;
	Vec3 pointA;
	Vec3 pointB;
	double length = 0.0;
};

std::vector<SeamSegment> CollectFloorSeams(const std::vector<const Polytope *> &polys)
{
	std::unordered_map<LineKey, std::vector<FloorEdge>, LineKeyHash> lines;

	for (size_t pi = 0; pi < polys.size(); ++pi)
	{
		for (const Face &f : polys[pi]->faces)
		{
			if (!IsFloorFace(f))
				continue;

			for (size_t i = 0; i < f.winding.size(); ++i)
			{
				LineKey key;
				FloorEdge edge;
				if (!EdgeAsLine(f.winding[i], f.winding[(i + 1) % f.winding.size()], key, edge))
					continue;

				edge.owner = static_cast<int>(pi);
				lines[key].push_back(edge);
			}
		}
	}

	std::vector<SeamSegment> out;

	for (const auto &entry : lines)
	{
		const std::vector<FloorEdge> &edges = entry.second;

		std::map<std::pair<int, int>, std::vector<std::pair<double, double>>> spans;

		for (size_t i = 0; i + 1 < edges.size(); ++i)
		{
			for (size_t j = i + 1; j < edges.size(); ++j)
			{
				if (edges[i].owner == edges[j].owner)
					continue;

				double lo = std::max(edges[i].lo, edges[j].lo);
				double hi = std::min(edges[i].hi, edges[j].hi);
				if (hi - lo <= kMinSeamLength)
					continue;

				spans[{std::min(edges[i].owner, edges[j].owner),
					std::max(edges[i].owner, edges[j].owner)}].push_back({lo, hi});
			}
		}

		for (auto &pair : spans)
		{
			std::vector<std::pair<double, double>> &intervals = pair.second;
			std::sort(intervals.begin(), intervals.end());

			for (size_t i = 0; i < intervals.size(); ++i)
			{
				double lo = intervals[i].first;
				double hi = intervals[i].second;
				while (i + 1 < intervals.size() && intervals[i + 1].first <= hi)
				{
					hi = std::max(hi, intervals[i + 1].second);
					++i;
				}

				const FloorEdge &ref = edges[0];

				SeamSegment seam;
				seam.a = pair.first.first;
				seam.b = pair.first.second;
				seam.pointA = PointAlong(ref, lo);
				seam.pointB = PointAlong(ref, hi);
				seam.length = hi - lo;
				out.push_back(seam);
			}
		}
	}

	return out;
}

struct MergeCandidate
{
	int leafA = -1;
	int leafB = -1;
	int faceIndexA = -1;
	int faceIndexB = -1;
	Vec3 pointA;
	Vec3 pointB;
	double length = 0.0;
	bool wholeFace = false;
};

void FindSharedFace(const Polytope &a, const Polytope &b, MergeCandidate &out)
{
	for (size_t i = 0; i < a.faces.size(); ++i)
	{
		const Face &fa = a.faces[i];
		if (fa.winding.size() < 3)
			continue;

		for (size_t j = 0; j < b.faces.size(); ++j)
		{
			const Face &fb = b.faces[j];
			if (fb.winding.size() < 3)
				continue;

			if ((fa.plane.normal + fb.plane.normal).Length() > kPlaneMatchEpsilon ||
				std::fabs(fa.plane.dist + fb.plane.dist) > kDistMatchEpsilon)
				continue;

			Winding overlap = IntersectCoplanarPolygons(fa.winding, fb.winding, fa.plane.normal);
			if (overlap.size() < 3)
				continue;

			double area = WindingArea(overlap, fa.plane.normal);
			if (area < kMinSharedFaceArea)
				continue;

			out.faceIndexA = static_cast<int>(i);
			out.faceIndexB = static_cast<int>(j);
			out.wholeFace = PortalSharesWholeFace(area, WindingArea(fa.winding, fa.plane.normal),
				WindingArea(fb.winding, fb.plane.normal));
			return;
		}
	}
}

}

MergeResult MergeAreas(const std::vector<ExtractedLeaf> &leaves, const std::vector<bool> &isCrouchLeaf,
	const std::vector<PassableVolume> &passables, size_t maxMergeGroup, size_t *seamsOut)
{
	int n = static_cast<int>(leaves.size());

	std::vector<int> parent(static_cast<size_t>(n));
	std::iota(parent.begin(), parent.end(), 0);
	auto find = [&](int x)
	{
		while (parent[static_cast<size_t>(x)] != x)
		{
			parent[static_cast<size_t>(x)] = parent[static_cast<size_t>(parent[static_cast<size_t>(x)])];
			x = parent[static_cast<size_t>(x)];
		}
		return x;
	};

	std::vector<std::set<int>> removedFaces(static_cast<size_t>(n));
	std::vector<std::set<int>> hiddenFaces(static_cast<size_t>(n));

	auto facesExcluding = [&](int leafId, const std::vector<std::set<int>> &excluded,
		std::vector<const Face *> &out)
	{
		const Polytope &poly = leaves[static_cast<size_t>(leafId)].polytope;
		const std::set<int> &skip = excluded[static_cast<size_t>(leafId)];
		for (size_t fi = 0; fi < poly.faces.size(); ++fi)
			if (skip.find(static_cast<int>(fi)) == skip.end())
				out.push_back(&poly.faces[fi]);
	};

	std::vector<std::vector<int>> members(static_cast<size_t>(n));
	for (int i = 0; i < n; ++i)
		members[static_cast<size_t>(i)].push_back(i);

	auto groupFaces = [&](int root, const std::vector<std::set<int>> &excluded, std::vector<const Face *> &out)
	{
		for (int leafId : members[static_cast<size_t>(root)])
			facesExcluding(leafId, excluded, out);
	};

	std::vector<Bounds> leafBounds(static_cast<size_t>(n));
	for (int i = 0; i < n; ++i)
		for (const Face &f : leaves[static_cast<size_t>(i)].polytope.faces)
			for (const Vec3 &v : f.winding)
				leafBounds[static_cast<size_t>(i)].Add(v);

	std::vector<Bounds> bounds = leafBounds;
	std::vector<bool> isPrism(static_cast<size_t>(n), false);

	std::vector<Winding> leafFootprint(static_cast<size_t>(n));
	for (int i = 0; i < n; ++i)
		leafFootprint[static_cast<size_t>(i)] = WalkableFootprint(leaves[static_cast<size_t>(i)].polytope);

	std::vector<Winding> footprint = leafFootprint;

	auto FirstSlabConflict = [&](const std::vector<int> &pending, const Bounds &merged,
		const Winding &mergedFootprint)
	{
		for (int i = 0; i < n; ++i)
		{
			int r = find(i);
			if (std::find(pending.begin(), pending.end(), r) != pending.end())
				continue;

			const Bounds &lb = leafBounds[static_cast<size_t>(i)];
			if (!merged.OverlapsZ(lb, kZTouchEpsilon) || !merged.OverlapsXY(lb, kZTouchEpsilon))
				continue;

			if (FootprintOverlapArea(mergedFootprint, leafFootprint[static_cast<size_t>(i)]) > kMinFootprintArea)
				return r;
		}

		return -1;
	};

	auto memberFootprints = [&](const std::vector<int> &pending, std::vector<const Winding *> &out)
	{
		for (int root : pending)
			for (int leafId : members[static_cast<size_t>(root)])
				out.push_back(&leafFootprint[static_cast<size_t>(leafId)]);
	};

	auto CoverageOk = [&](const std::vector<int> &pending, const Winding &mergedFootprint)
	{
		std::vector<const Winding *> fps;
		memberFootprints(pending, fps);

		double covered = 0.0;
		for (size_t i = 0; i < fps.size(); ++i)
		{
			covered += WindingArea(*fps[i], kUp);
			for (size_t j = 0; j < i; ++j)
				covered -= FootprintOverlapArea(*fps[i], *fps[j]);
		}

		return WindingArea(mergedFootprint, kUp) - covered <= kMinFootprintArea;
	};

	auto FirstFootprintGap = [&](const std::vector<int> &pending, const Winding &mergedFootprint)
	{
		std::vector<const Winding *> fps;
		memberFootprints(pending, fps);

		for (int i = 0; i < n; ++i)
		{
			int r = find(i);
			if (std::find(pending.begin(), pending.end(), r) != pending.end())
				continue;

			const Winding &fp = leafFootprint[static_cast<size_t>(i)];
			double claim = FootprintOverlapArea(mergedFootprint, fp);
			if (claim <= kMinFootprintArea)
				continue;

			for (const Winding *member : fps)
				claim -= FootprintOverlapArea(*member, fp);

			if (claim > kMinFootprintArea)
				return r;
		}

		return -1;
	};

	std::vector<const Polytope *> polys;
	polys.reserve(leaves.size());
	for (const ExtractedLeaf &leaf : leaves)
		polys.push_back(&leaf.polytope);

	std::vector<SeamSegment> seams = CollectFloorSeams(polys);
	if (seamsOut)
		*seamsOut = seams.size();

	std::vector<MergeCandidate> candidates;
	for (const SeamSegment &seam : seams)
	{
		if (LeafIsCrouch(isCrouchLeaf, seam.a) != LeafIsCrouch(isCrouchLeaf, seam.b))
			continue;

		if (LeafPassable(leaves, seam.a) != LeafPassable(leaves, seam.b))
			continue;

		if (!SeamIsWalkable(leaves[static_cast<size_t>(seam.a)].polytope,
				leaves[static_cast<size_t>(seam.b)].polytope, seam.pointA, seam.pointB, kStepHeight))
			continue;

		MergeCandidate cand;
		cand.leafA = seam.a;
		cand.leafB = seam.b;
		cand.pointA = seam.pointA;
		cand.pointB = seam.pointB;
		cand.length = seam.length;
		FindSharedFace(leaves[static_cast<size_t>(seam.a)].polytope,
			leaves[static_cast<size_t>(seam.b)].polytope, cand);
		candidates.push_back(cand);
	}
	std::sort(candidates.begin(), candidates.end(),
		[](const MergeCandidate &a, const MergeCandidate &b) { return a.length > b.length; });

	std::vector<std::vector<const MergeCandidate *>> incident(static_cast<size_t>(n));
	for (const MergeCandidate &c : candidates)
	{
		incident[static_cast<size_t>(c.leafA)].push_back(&c);
		incident[static_cast<size_t>(c.leafB)].push_back(&c);
	}

	auto sharedSeams = [&](const std::vector<int> &pending, int other, std::vector<const MergeCandidate *> &out)
	{
		for (int root : pending)
			for (int leafId : members[static_cast<size_t>(root)])
				for (const MergeCandidate *q : incident[static_cast<size_t>(leafId)])
					if (find(q->leafA) == other || find(q->leafB) == other)
						out.push_back(q);
	};

	bool progress = true;
	while (progress)
	{
		progress = false;
		for (const MergeCandidate &cand : candidates)
		{
			int rootA = find(cand.leafA);
			int rootB = find(cand.leafB);
			if (rootA == rootB)
				continue;

			std::vector<int> pending{rootA};
			std::vector<const MergeCandidate *> internal;
			std::vector<const Face *> facesU, floorU;
			groupFaces(rootA, hiddenFaces, facesU);
			groupFaces(rootA, removedFaces, floorU);
			Winding footprintU = footprint[static_cast<size_t>(rootA)];
			Bounds boundsU = bounds[static_cast<size_t>(rootA)];
			bool isPrismU = isPrism[static_cast<size_t>(rootA)];

			int next = rootB;
			bool accepted = false;

			while (true)
			{
				if (pending.size() + 1 > maxMergeGroup)
					break;

				std::vector<const MergeCandidate *> seam;
				sharedSeams(pending, next, seam);

				std::vector<const Face *> facesNext, floorNext;
				groupFaces(next, hiddenFaces, facesNext);
				groupFaces(next, removedFaces, floorNext);

				for (const MergeCandidate *q : seam)
				{
					if (q->faceIndexA < 0)
						continue;

					const Face *f1 = &leaves[static_cast<size_t>(q->leafA)].polytope.faces[static_cast<size_t>(q->faceIndexA)];
					const Face *f2 = &leaves[static_cast<size_t>(q->leafB)].polytope.faces[static_cast<size_t>(q->faceIndexB)];

					std::vector<std::vector<const Face *> *> sides{&facesU, &facesNext};
					if (q->wholeFace)
					{
						sides.push_back(&floorU);
						sides.push_back(&floorNext);
					}

					for (std::vector<const Face *> *side : sides)
					{
						side->erase(std::remove(side->begin(), side->end(), f1), side->end());
						side->erase(std::remove(side->begin(), side->end(), f2), side->end());
					}
				}

				const Winding &fpNext = footprint[static_cast<size_t>(next)];
				Winding footprintNew = MergeFootprints(footprintU, fpNext);

				bool stays3DConvex = MergeStaysConvex(facesU, facesNext, kConvexEpsilon);
				bool resultIsPrism = !stays3DConvex || isPrismU || isPrism[static_cast<size_t>(next)];

				if (!MergedFloorStaysWalkable(floorU, floorNext, kStepHeight))
					break;

				facesU.insert(facesU.end(), facesNext.begin(), facesNext.end());
				floorU.insert(floorU.end(), floorNext.begin(), floorNext.end());
				internal.insert(internal.end(), seam.begin(), seam.end());
				footprintU = std::move(footprintNew);
				boundsU.Add(bounds[static_cast<size_t>(next)]);
				isPrismU = resultIsPrism;
				pending.push_back(next);

				if (!isPrismU)
				{
					accepted = true;
					break;
				}

				int conflict = FirstSlabConflict(pending, boundsU, footprintU);
				if (conflict < 0 && CoverageOk(pending, footprintU))
				{
					accepted = true;
					break;
				}

				if (conflict < 0)
					conflict = FirstFootprintGap(pending, footprintU);

				if (conflict < 0)
					break;

				std::vector<const MergeCandidate *> reachable;
				sharedSeams(pending, conflict, reachable);
				if (reachable.empty())
					break;

				next = conflict;
			}

			if (!accepted)
				continue;

			for (const MergeCandidate *q : internal)
			{
				if (q->faceIndexA < 0)
					continue;

				hiddenFaces[static_cast<size_t>(q->leafA)].insert(q->faceIndexA);
				hiddenFaces[static_cast<size_t>(q->leafB)].insert(q->faceIndexB);
				if (q->wholeFace)
				{
					removedFaces[static_cast<size_t>(q->leafA)].insert(q->faceIndexA);
					removedFaces[static_cast<size_t>(q->leafB)].insert(q->faceIndexB);
				}
			}

			std::vector<int> &into = members[static_cast<size_t>(rootA)];
			for (size_t pi = 1; pi < pending.size(); ++pi)
			{
				int absorbed = pending[pi];
				parent[static_cast<size_t>(absorbed)] = rootA;
				std::vector<int> &from = members[static_cast<size_t>(absorbed)];
				into.insert(into.end(), from.begin(), from.end());
				from.clear();
			}

			bounds[static_cast<size_t>(rootA)] = boundsU;
			footprint[static_cast<size_t>(rootA)] = std::move(footprintU);
			isPrism[static_cast<size_t>(rootA)] = isPrismU;

			progress = true;
		}
	}

	MergeResult result;
	std::vector<int> rootToAreaId(static_cast<size_t>(n), -1);

	for (int i = 0; i < n; ++i)
	{
		int root = find(i);
		if (rootToAreaId[static_cast<size_t>(root)] != -1)
			continue;

		Area area;
		area.id = static_cast<int>(result.areas.size());
		area.prism = isPrism[static_cast<size_t>(root)];
		for (int leafId : members[static_cast<size_t>(root)])
		{
			area.memberLeafIds.push_back(leafId);
			if (LeafIsCrouch(isCrouchLeaf, leafId))
				area.areaFlags |= crossbot_nav::NAV_AREA_CROUCH;
			ApplyPassable(area, passables, LeafPassable(leaves, leafId));
			std::vector<const Face *> faces;
			facesExcluding(leafId, removedFaces, faces);
			for (const Face *f : faces)
				area.polytope.faces.push_back(*f);
		}

		std::vector<const Face *> visible;
		groupFaces(root, hiddenFaces, visible);
		area.boundPlanes = BuildBoundPlanes(visible, area.polytope, footprint[static_cast<size_t>(root)],
			area.prism);

		rootToAreaId[static_cast<size_t>(root)] = area.id;
		result.areas.push_back(std::move(area));
	}

	return result;
}

size_t BuildWalkConnections(const std::vector<Area> &areas, std::vector<MergedConnection> &conns)
{
	std::vector<const Polytope *> polys;
	polys.reserve(areas.size());
	for (const Area &area : areas)
		polys.push_back(&area.polytope);

	size_t first = conns.size();

	for (const SeamSegment &seam : CollectFloorSeams(polys))
	{
		MergedConnection mc;
		mc.areaA = areas[static_cast<size_t>(seam.a)].id;
		mc.areaB = areas[static_cast<size_t>(seam.b)].id;
		mc.pointA = seam.pointA;
		mc.pointB = seam.pointB;
		conns.push_back(mc);
	}

	TagCrouchConnections(areas, conns);

	return conns.size() - first;
}

size_t ValidateAreaFootprints(const std::vector<Area> &areas)
{
	size_t bad = 0;

	for (const Area &area : areas)
	{
		Winding footprint = WalkableFootprint(area.polytope);
		double hull = WindingAreaXY(footprint);
		Vec3 c = area.polytope.Centroid();

		if (footprint.size() < 3 || hull <= kMinFootprintArea)
		{
			std::fprintf(stderr, "warning: area %d near (%.0f %.0f %.0f) has no walkable floor "
				"(%zu polytope faces)\n", area.id, c.x, c.y, c.z, area.polytope.faces.size());
			++bad;
			continue;
		}

		double gap = WalkableFootprintGapArea(area.polytope);
		if (gap > kMinFootprintArea)
		{
			std::fprintf(stderr, "warning: area %d near (%.0f %.0f %.0f) has a concave walkable "
				"footprint: %.1f of its %.1f convex hull has no floor under it\n",
				area.id, c.x, c.y, c.z, gap, hull);
			++bad;
		}
	}

	return bad;
}

}

