#include "area_merger.h"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <set>

#include "floor_support.h"

namespace navgen
{

float TraverseCostFor(const MergedConnection &conn)
{
	if (conn.connType != crossbot_nav::CONN_AIRMOVE)
		return 1.0f;

	float damage = static_cast<float>(conn.fallDamage);
	float damagePenalty = (damage <= 0.0f) ? 0.0f : (damage >= 100.0f ? 1.0e4f : damage * 0.5f);
	return 1.0f + static_cast<float>(conn.flightTime) * 0.5f + damagePenalty;
}

size_t FilterWalkConnectionsByStep(const std::vector<Area> &areas, std::vector<MergedConnection> &conns,
	double maxStep)
{
	size_t removed = 0;

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

		// The floor under the connection, not the area's lowest floor: an area holding
		// both a ramp and the flat ground at its base would otherwise report the step
		// as the whole ramp's rise.
		Vec3 mid = (c.pointA + c.pointB) * 0.5;

		const Face *floorA = FloorFaceUnder(areas[static_cast<size_t>(c.areaA)].polytope, mid.x, mid.y);
		const Face *floorB = FloorFaceUnder(areas[static_cast<size_t>(c.areaB)].polytope, mid.x, mid.y);
		if (!floorA || !floorB)
		{
			kept.push_back(c);
			continue;
		}

		double zA = FloorHeightAt(*floorA, mid.x, mid.y);
		double zB = FloorHeightAt(*floorB, mid.x, mid.y);

		if (std::fabs(zA - zB) > maxStep)
		{
			++removed;
			continue;
		}

		kept.push_back(c);
	}

	conns.swap(kept);
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
			// Both endpoints are corners of the one shared face, so they take their height from
			// the same area -- resolving each against its own side would tilt the gate by the
			// step between the two floors for no geometric reason.
			ok = snap(c.pointA, c.areaA);
			ok = snap(c.pointB, c.areaA) || ok;
		}
		else
		{
			// Only the launch: the landing is where the simulation actually put the player down,
			// and re-deriving it from the destination area would move it whenever the landing
			// grazes an area boundary.
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

void TagCrouchConnections(MergeResult &result)
{
	for (MergedConnection &c : result.conns)
	{
		bool crouchA = (result.areas[static_cast<size_t>(c.areaA)].areaFlags & crossbot_nav::NAV_AREA_CROUCH) != 0;
		bool crouchB = (result.areas[static_cast<size_t>(c.areaB)].areaFlags & crossbot_nav::NAV_AREA_CROUCH) != 0;
		if (crouchA || crouchB)
			c.connFlags |= crossbot_nav::CONN_FLAG_NEEDS_CROUCH;
	}
}

}

MergeResult NoMerge(const std::vector<ExtractedLeaf> &leaves, const std::vector<Portal> &portals,
	const std::vector<bool> &isCrouchLeaf)
{
	MergeResult result;
	result.areas.reserve(leaves.size());
	for (const ExtractedLeaf &leaf : leaves)
	{
		Area area;
		area.id = static_cast<int>(result.areas.size());
		area.memberLeafIds.push_back(leaf.id);
		area.polytope = leaf.polytope;
		for (const Face &f : area.polytope.faces)
			AddPlaneUnique(area.boundPlanes, f.plane);
		if (LeafIsCrouch(isCrouchLeaf, leaf.id))
			area.areaFlags |= crossbot_nav::NAV_AREA_CROUCH;
		result.areas.push_back(std::move(area));
	}

	result.conns.reserve(portals.size());
	for (const Portal &p : portals)
	{
		MergedConnection mp;
		mp.areaA = p.leafA;
		mp.areaB = p.leafB;
		mp.pointA = p.pointA;
		mp.pointB = p.pointB;
		mp.area = p.area;
		result.conns.push_back(mp);
	}

	TagCrouchConnections(result);

	return result;
}

namespace
{

constexpr double kCoverageTolerance = 0.02;
constexpr double kConvexEpsilon = 0.5;
constexpr double kZTouchEpsilon = 0.5;
constexpr double kMinFootprintArea = 1.0;
constexpr double kFootprintAreaTolerance = 0.001;
constexpr double kFloorNormalZ = 0.7;
constexpr double kFloorAdjacencyEpsilon = 0.5;
constexpr size_t kMaxMergeGroups = 64;

bool PortalSharesWholeFace(double portalArea, double faceAreaA, double faceAreaB)
{
	if (faceAreaA < 1e-6 || faceAreaB < 1e-6)
		return false;
	return std::fabs(portalArea - faceAreaA) <= kCoverageTolerance * faceAreaA &&
		std::fabs(portalArea - faceAreaB) <= kCoverageTolerance * faceAreaB;
}

bool SeamIsWalkable(const Polytope &a, const Polytope &b, const Vec3 &mid, double maxStep)
{
	const Face *floorA = FloorFaceUnder(a, mid.x, mid.y);
	const Face *floorB = FloorFaceUnder(b, mid.x, mid.y);
	if (!floorA || !floorB)
		return false;

	double zA = FloorHeightAt(*floorA, mid.x, mid.y);
	double zB = FloorHeightAt(*floorB, mid.x, mid.y);
	return std::fabs(zA - zB) <= maxStep;
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

Winding FootprintOf(const Polytope &poly)
{
	std::vector<Vec3> pts;
	for (const Face &f : poly.faces)
		for (const Vec3 &v : f.winding)
			pts.push_back(v);
	return ConvexHullXY(pts);
}

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

bool IsFloorFace(const Face &f)
{
	return f.plane.normal.z > kFloorNormalZ && f.winding.size() >= 3;
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

bool UnionFootprintIsConvex(const Winding &a, const Winding &b, const Winding &merged)
{
	double areaA = WindingArea(a, kUp);
	double areaB = WindingArea(b, kUp);
	double areaMerged = WindingArea(merged, kUp);
	double covered = areaA + areaB - FootprintOverlapArea(a, b);

	double tolerance = std::max(kMinFootprintArea, kFootprintAreaTolerance * areaMerged);
	return areaMerged - covered <= tolerance;
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

struct MergeCandidate
{
	const Portal *portal = nullptr;
	bool wholeFace = false;
};

}

MergeResult MergeAreas(const std::vector<ExtractedLeaf> &leaves, const std::vector<Portal> &portals,
	const std::vector<bool> &isCrouchLeaf)
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
		leafFootprint[static_cast<size_t>(i)] = FootprintOf(leaves[static_cast<size_t>(i)].polytope);

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

	std::vector<MergeCandidate> candidates;
	for (const Portal &p : portals)
	{
		if (LeafIsCrouch(isCrouchLeaf, p.leafA) != LeafIsCrouch(isCrouchLeaf, p.leafB))
			continue;

		const Face &fa = leaves[static_cast<size_t>(p.leafA)].polytope.faces[static_cast<size_t>(p.faceIndexA)];
		const Face &fb = leaves[static_cast<size_t>(p.leafB)].polytope.faces[static_cast<size_t>(p.faceIndexB)];

		double faceAreaA = WindingArea(fa.winding, fa.plane.normal);
		double faceAreaB = WindingArea(fb.winding, fb.plane.normal);
		bool wholeFace = PortalSharesWholeFace(p.area, faceAreaA, faceAreaB);

		if (!wholeFace)
		{
			Vec3 mid = (p.pointA + p.pointB) * 0.5;
			if (!SeamIsWalkable(leaves[static_cast<size_t>(p.leafA)].polytope,
					leaves[static_cast<size_t>(p.leafB)].polytope, mid, kStepHeight))
				continue;
		}

		candidates.push_back({&p, wholeFace});
	}
	std::sort(candidates.begin(), candidates.end(),
		[](const MergeCandidate &a, const MergeCandidate &b) { return a.portal->area > b.portal->area; });

	std::vector<std::vector<const MergeCandidate *>> incident(static_cast<size_t>(n));
	for (const MergeCandidate &c : candidates)
	{
		incident[static_cast<size_t>(c.portal->leafA)].push_back(&c);
		incident[static_cast<size_t>(c.portal->leafB)].push_back(&c);
	}

	auto sharedPortals = [&](const std::vector<int> &pending, int other, std::vector<const MergeCandidate *> &out)
	{
		for (int root : pending)
			for (int leafId : members[static_cast<size_t>(root)])
				for (const MergeCandidate *q : incident[static_cast<size_t>(leafId)])
					if (find(q->portal->leafA) == other || find(q->portal->leafB) == other)
						out.push_back(q);
	};

	bool progress = true;
	while (progress)
	{
		progress = false;
		for (const MergeCandidate &cand : candidates)
		{
			const Portal *p = cand.portal;
			int rootA = find(p->leafA);
			int rootB = find(p->leafB);
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
				std::vector<const MergeCandidate *> seam;
				sharedPortals(pending, next, seam);

				std::vector<const Face *> facesNext, floorNext;
				groupFaces(next, hiddenFaces, facesNext);
				groupFaces(next, removedFaces, floorNext);

				for (const MergeCandidate *q : seam)
				{
					const Portal *qp = q->portal;
					const Face *f1 = &leaves[static_cast<size_t>(qp->leafA)].polytope.faces[static_cast<size_t>(qp->faceIndexA)];
					const Face *f2 = &leaves[static_cast<size_t>(qp->leafB)].polytope.faces[static_cast<size_t>(qp->faceIndexB)];

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

				if (resultIsPrism && !UnionFootprintIsConvex(footprintU, fpNext, footprintNew))
					break;

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
				if (conflict < 0)
				{
					accepted = true;
					break;
				}

				if (pending.size() >= kMaxMergeGroups)
					break;

				std::vector<const MergeCandidate *> reachable;
				sharedPortals(pending, conflict, reachable);
				if (reachable.empty())
					break;

				next = conflict;
			}

			if (!accepted)
				continue;

			for (const MergeCandidate *q : internal)
			{
				const Portal *qp = q->portal;
				hiddenFaces[static_cast<size_t>(qp->leafA)].insert(qp->faceIndexA);
				hiddenFaces[static_cast<size_t>(qp->leafB)].insert(qp->faceIndexB);
				if (q->wholeFace)
				{
					removedFaces[static_cast<size_t>(qp->leafA)].insert(qp->faceIndexA);
					removedFaces[static_cast<size_t>(qp->leafB)].insert(qp->faceIndexB);
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

	for (const Portal &p : portals)
	{
		int areaA = rootToAreaId[static_cast<size_t>(find(p.leafA))];
		int areaB = rootToAreaId[static_cast<size_t>(find(p.leafB))];
		if (areaA == areaB)
			continue;

		MergedConnection mp;
		mp.areaA = areaA;
		mp.areaB = areaB;
		mp.pointA = p.pointA;
		mp.pointB = p.pointB;
		mp.area = p.area;
		result.conns.push_back(mp);
	}

	TagCrouchConnections(result);

	return result;
}

}

