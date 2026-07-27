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

namespace
{

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

bool PortalSharesWholeFace(double portalArea, double faceAreaA, double faceAreaB)
{
	if (faceAreaA < 1e-6 || faceAreaB < 1e-6)
		return false;
	return std::fabs(portalArea - faceAreaA) <= kCoverageTolerance * faceAreaA &&
		std::fabs(portalArea - faceAreaB) <= kCoverageTolerance * faceAreaB;
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

	auto currentFaces = [&](int leafId, std::vector<const Face *> &out)
	{
		const Polytope &poly = leaves[static_cast<size_t>(leafId)].polytope;
		const std::set<int> &removed = removedFaces[static_cast<size_t>(leafId)];
		for (size_t fi = 0; fi < poly.faces.size(); ++fi)
			if (removed.find(static_cast<int>(fi)) == removed.end())
				out.push_back(&poly.faces[fi]);
	};

	std::vector<std::vector<int>> members(static_cast<size_t>(n));
	for (int i = 0; i < n; ++i)
		members[static_cast<size_t>(i)].push_back(i);

	auto groupFaces = [&](int root, std::vector<const Face *> &out)
	{
		for (int leafId : members[static_cast<size_t>(root)])
			currentFaces(leafId, out);
	};

	std::vector<const Portal *> candidates;
	for (const Portal &p : portals)
	{
		if (LeafIsCrouch(isCrouchLeaf, p.leafA) != LeafIsCrouch(isCrouchLeaf, p.leafB))
			continue;

		const Face &fa = leaves[static_cast<size_t>(p.leafA)].polytope.faces[static_cast<size_t>(p.faceIndexA)];
		const Face &fb = leaves[static_cast<size_t>(p.leafB)].polytope.faces[static_cast<size_t>(p.faceIndexB)];
		double faceAreaA = WindingArea(fa.winding, fa.plane.normal);
		double faceAreaB = WindingArea(fb.winding, fb.plane.normal);
		if (PortalSharesWholeFace(p.area, faceAreaA, faceAreaB))
			candidates.push_back(&p);
	}
	std::sort(candidates.begin(), candidates.end(),
		[](const Portal *a, const Portal *b) { return a->area > b->area; });

	std::vector<std::vector<const Portal *>> incident(static_cast<size_t>(n));
	for (const Portal *p : candidates)
	{
		incident[static_cast<size_t>(p->leafA)].push_back(p);
		incident[static_cast<size_t>(p->leafB)].push_back(p);
	}

	auto sharedPortals = [&](int rootA, int rootB, std::vector<const Portal *> &out)
	{
		for (int leafId : members[static_cast<size_t>(rootA)])
			for (const Portal *q : incident[static_cast<size_t>(leafId)])
				if (find(q->leafA) == rootB || find(q->leafB) == rootB)
					out.push_back(q);
	};

	bool progress = true;
	while (progress)
	{
		progress = false;
		for (const Portal *p : candidates)
		{
			int rootA = find(p->leafA);
			int rootB = find(p->leafB);
			if (rootA == rootB)
				continue;

			std::vector<const Portal *> internal;
			sharedPortals(rootA, rootB, internal);

			std::vector<const Face *> facesA, facesB;
			groupFaces(rootA, facesA);
			groupFaces(rootB, facesB);

			for (const Portal *q : internal)
			{
				const Face *f1 = &leaves[static_cast<size_t>(q->leafA)].polytope.faces[static_cast<size_t>(q->faceIndexA)];
				const Face *f2 = &leaves[static_cast<size_t>(q->leafB)].polytope.faces[static_cast<size_t>(q->faceIndexB)];
				for (std::vector<const Face *> *side : {&facesA, &facesB})
				{
					side->erase(std::remove(side->begin(), side->end(), f1), side->end());
					side->erase(std::remove(side->begin(), side->end(), f2), side->end());
				}
			}

			if (!MergeStaysConvex(facesA, facesB, kConvexEpsilon))
				continue;

			for (const Portal *q : internal)
			{
				removedFaces[static_cast<size_t>(q->leafA)].insert(q->faceIndexA);
				removedFaces[static_cast<size_t>(q->leafB)].insert(q->faceIndexB);
			}

			parent[static_cast<size_t>(rootB)] = rootA;
			std::vector<int> &into = members[static_cast<size_t>(rootA)];
			std::vector<int> &from = members[static_cast<size_t>(rootB)];
			into.insert(into.end(), from.begin(), from.end());
			from.clear();

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
		for (int leafId : members[static_cast<size_t>(root)])
		{
			area.memberLeafIds.push_back(leafId);
			if (LeafIsCrouch(isCrouchLeaf, leafId))
				area.areaFlags |= crossbot_nav::NAV_AREA_CROUCH;
			std::vector<const Face *> faces;
			currentFaces(leafId, faces);
			for (const Face *f : faces)
				area.polytope.faces.push_back(*f);
		}
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

