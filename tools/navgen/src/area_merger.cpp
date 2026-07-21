#include "area_merger.h"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <set>

namespace navgen
{

namespace
{

constexpr double kCoverageTolerance = 0.02; // relative area tolerance for "this portal fully covers the face"
constexpr double kConvexEpsilon = 0.5;      // slack (map units) for the half-space containment test

bool FullyCovers(double portalArea, double faceArea)
{
	if (faceArea < 1e-6)
		return false;
	return std::fabs(portalArea - faceArea) <= kCoverageTolerance * faceArea;
}

// True if joining facesA and facesB (each already excluding the matched
// pair of shared/internal faces) keeps the union convex: every vertex of
// one side must lie within every remaining face-plane's half-space of the
// other side. If either direction fails, the merge would create a reflex
// (concave) edge somewhere along the former internal wall's rim.
//
// Sign note: leaf_extractor's ClipWinding(..., keepFront=true) convention
// keeps the region where dot(normal,x)-dist >= 0 for every plane in a
// leaf's own defining chain -- so a leaf's OWN vertices satisfy that
// "front" (>= -epsilon) condition relative to each of its own face planes,
// including a face plane it shares with a neighbor. The violation to check
// for is therefore the *negative* side (a vertex from the other leaf
// falling behind one of this leaf's own bounding planes), not the positive
// side -- verified against real merge candidates on crossfire.bsp after an
// initial ">" version rejected literally every full-coverage candidate.
bool MergeStaysConvex(const std::vector<const Face *> &facesA, const std::vector<const Face *> &facesB)
{
	for (const Face *fa : facesA)
		for (const Face *fb : facesB)
			for (const Vec3 &v : fb->winding)
				if (v.Dot(fa->plane.normal) - fa->plane.dist < -kConvexEpsilon)
					return false;

	for (const Face *fb : facesB)
		for (const Face *fa : facesA)
			for (const Vec3 &v : fa->winding)
				if (v.Dot(fb->plane.normal) - fb->plane.dist < -kConvexEpsilon)
					return false;

	return true;
}

} // namespace

MergeResult MergeAreas(const std::vector<ExtractedLeaf> &leaves, const std::vector<Portal> &portals)
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

	// removedFaces[leafId] holds the face indices (within that original
	// leaf) that have become internal (merged away) and should be excluded
	// when assembling a group's current boundary.
	std::vector<std::set<int>> removedFaces(static_cast<size_t>(n));

	auto currentFaces = [&](int leafId, std::vector<const Face *> &out)
	{
		const Polytope &poly = leaves[static_cast<size_t>(leafId)].polytope;
		const std::set<int> &removed = removedFaces[static_cast<size_t>(leafId)];
		for (size_t fi = 0; fi < poly.faces.size(); ++fi)
			if (removed.find(static_cast<int>(fi)) == removed.end())
				out.push_back(&poly.faces[fi]);
	};

	// members[root] holds every leaf id currently folded into that DSU
	// root, maintained alongside the DSU to avoid an O(n) scan per merge.
	std::vector<std::vector<int>> members(static_cast<size_t>(n));
	for (int i = 0; i < n; ++i)
		members[static_cast<size_t>(i)].push_back(i);

	auto groupFaces = [&](int root, std::vector<const Face *> &out)
	{
		for (int leafId : members[static_cast<size_t>(root)])
			if (find(leafId) == root)
				currentFaces(leafId, out);
	};

	// Only fully-covering portals are merge candidates (see header comment
	// on partial overlaps). Largest-area first so big, unambiguous matches
	// merge before smaller/marginal ones.
	std::vector<const Portal *> candidates;
	for (const Portal &p : portals)
	{
		const Face &fa = leaves[static_cast<size_t>(p.leafA)].polytope.faces[static_cast<size_t>(p.faceIndexA)];
		const Face &fb = leaves[static_cast<size_t>(p.leafB)].polytope.faces[static_cast<size_t>(p.faceIndexB)];
		double faceAreaA = WindingArea(fa.winding, fa.plane.normal);
		double faceAreaB = WindingArea(fb.winding, fb.plane.normal);
		if (FullyCovers(p.area, faceAreaA) && FullyCovers(p.area, faceAreaB))
			candidates.push_back(&p);
	}
	std::sort(candidates.begin(), candidates.end(),
		[](const Portal *a, const Portal *b) { return a->area > b->area; });

	bool progress = true;
	while (progress)
	{
		progress = false;
		for (const Portal *p : candidates)
		{
			int rootA = find(p->leafA);
			int rootB = find(p->leafB);
			if (rootA == rootB)
				continue; // already merged (this portal or a different path)

			std::vector<const Face *> facesA, facesB;
			groupFaces(rootA, facesA);
			groupFaces(rootB, facesB);

			const Face *internalA = &leaves[static_cast<size_t>(p->leafA)].polytope.faces[static_cast<size_t>(p->faceIndexA)];
			const Face *internalB = &leaves[static_cast<size_t>(p->leafB)].polytope.faces[static_cast<size_t>(p->faceIndexB)];
			facesA.erase(std::remove(facesA.begin(), facesA.end(), internalA), facesA.end());
			facesB.erase(std::remove(facesB.begin(), facesB.end(), internalB), facesB.end());

			if (!MergeStaysConvex(facesA, facesB))
				continue;

			removedFaces[static_cast<size_t>(p->leafA)].insert(p->faceIndexA);
			removedFaces[static_cast<size_t>(p->leafB)].insert(p->faceIndexB);

			parent[static_cast<size_t>(rootB)] = rootA;
			std::vector<int> &into = members[static_cast<size_t>(rootA)];
			std::vector<int> &from = members[static_cast<size_t>(rootB)];
			into.insert(into.end(), from.begin(), from.end());
			from.clear();

			progress = true;
		}
	}

	// Assemble final areas from surviving DSU roots.
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
			if (find(leafId) != root)
				continue;
			area.memberLeafIds.push_back(leafId);
			std::vector<const Face *> faces;
			currentFaces(leafId, faces);
			for (const Face *f : faces)
				area.polytope.faces.push_back(*f);
		}
		rootToAreaId[static_cast<size_t>(root)] = area.id;
		result.areas.push_back(std::move(area));
	}

	// Re-key every original portal (not just merge candidates -- portals
	// that failed the coverage/convexity test remain real area-to-area
	// connections, just not mergeable ones) onto the final area ids, and
	// drop the ones that ended up internal to a merged area.
	for (const Portal &p : portals)
	{
		int areaA = rootToAreaId[static_cast<size_t>(find(p.leafA))];
		int areaB = rootToAreaId[static_cast<size_t>(find(p.leafB))];
		if (areaA == areaB)
			continue;

		MergedPortal mp;
		mp.areaA = areaA;
		mp.areaB = areaB;
		mp.pointA = p.pointA;
		mp.pointB = p.pointB;
		mp.area = p.area;
		result.portals.push_back(mp);
	}

	return result;
}

} // namespace navgen
