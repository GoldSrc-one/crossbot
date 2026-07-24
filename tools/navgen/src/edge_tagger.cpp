#include "edge_tagger.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <set>
#include <utility>

#include "clipnode_tree.h"
#include "hull_defs.h"

namespace navgen
{

namespace
{

using namespace crossbot_nav;

// Approximate, tunable constants -- not reverse-engineered engine values.
// Reasonable stand-ins for "roughly what a running jump/safe fall can
// cover," meant to be revisited once real bots (Milestone 2c) can exercise
// them, not treated as ground truth.
constexpr double kMaxJumpHorizontalDistance = 200.0;
// GoldSrc's jump impulse IS reverse-engineered ground truth, unlike the
// horizontal distance above: PM_Jump (pm_shared.c) always sets
// velocity.z = sqrt(2 * 800 * 45.0) -- a fixed impulse, not scaled by
// running speed -- which caps a plain standing jump's height gain at
// v^2/(2*800) = 45 units. But a duck-jump reaches further still: ducking
// mid-air shrinks the hull from the standing height down to the duck
// hull's, and Bot_MoveNav already presses IN_DUCK once airborne
// (bot_move_nav.cpp's needsJump branch) specifically for this. The real
// ceiling confirmed by the user (a live player test, not a formula) is 63
// units, not 45 -- matching 45 (the jump apex) plus roughly the
// standing/duck hull height difference (a shorter hull's feet can be
// that much higher while its head stays at the same absolute max the
// jump reaches). 60 leaves a small margin below that confirmed ceiling.
constexpr double kMaxJumpHeightDelta = 60.0;
// GoldSrc's real auto-step height (HOSTAGE_STEPSIZE_DEFAULT in the HLSDK
// is the one place it's spelled out as a named constant, 18 units, and
// matches the well-known engine convention) -- anything taller than this
// needs an actual jump input to climb, it can't just be walked up. Found
// via a real live-testing miss: TagDropOffs used to only reclassify a
// portal once its height delta exceeded 200 units (a "will this actually
// hurt" fall-damage threshold, not a "can a bot walk this without
// jumping" one), so a 176-unit ledge -- far beyond walking range but
// under that 200 cutoff -- was left as an ordinary bidirectional
// EDGE_WALK, and a bot routed across it just walked into it and stopped
// forever, live-verified on crossfire.bsp.
constexpr double kWalkableStepHeight = 18.0;
constexpr double kAreaAabbMargin = 48.0; // ~1.5x a standing hull's radius; door/ladder proximity search slack

// Classifies `point` against a hull's clipnode tree (same descent
// WalkClipnodeTree's recursion does, just following one side instead of
// both since a point has no "split" case): true if it lands in a
// CONTENTS_SOLID leaf. Every other content type (empty, water, ...) counts
// as passable for this coarse check -- it isn't trying to be a real
// physics trace, just to answer "is there a wall in the way at all."
bool PointInSolid(const BspFile &bsp, int headnode, const Vec3 &point)
{
	return ClassifyPoint(bsp, headnode, point) == CONTENTS_SOLID;
}

// Coarse straight-line occlusion check for a proximity-derived jump/drop
// connection, sampled every kSampleSpacing units along the segment (hull-1
// point classification, not a real swept-hull trace -- good enough to catch
// "this connection runs straight through a wall," not meant to replace real
// collision). ProbeJumpGaps/TagDropOffs otherwise only compare distance and
// height between two areas' floor points, with no visibility check at all
// (a known, documented limitation -- see ProbeJumpGaps' own header comment)
// -- confirmed live on crossfire.bsp: without this, a jump edge got created
// straight through a real solid wall between two rooms that don't actually
// open into each other there, and a bot sent across it just walked into the
// wall and bounced off it forever, never able to complete the "jump" at all
// since there was never a real gap to jump across in the first place.
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

// GoldSrc's own real max walkable slope (matches the well-known
// STEPSIZE/slope convention, dot(normal, up) >= ~0.7 -- steeper than that
// counts as a wall, not a floor, for standing/walking purposes).
constexpr double kFloorNormalThreshold = 0.7;

// Hull-1 half-width (16 units, GoldSrc's standard human hull) plus a
// couple of units of slop. See NearestFloorPoint's corner-inset comment.
constexpr double kCornerInsetMargin = 18.0;

// The nearest floor-face vertex within `area` to `target`, by real 3D
// distance, nudged inward off the exact corner it was found at. Unlike
// AreaBoundsCache's mins/maxs/centroid, which are reduced from every vertex
// of a merged area's ENTIRE boundary (walls and ceiling included, and
// averaged across however much unrelated geometry got folded into that one
// area by MergeAreas -- which has no height/slope constraint at all, only
// "shared face fully covers + stays convex"), this looks only at genuinely
// floor-like faces and picks whichever specific point is actually closest
// to the gap in question, so a room merged with some unrelated dip or
// alcove elsewhere in its footprint can't drag the answer away from the
// real local floor height. 3D distance (not just x/y) additionally keeps a
// large merged area's own disconnected ledge/mezzanine at a very different
// height from winning just because its x/y happens to line up.
// Confirmed live: naively switching the jump-height check from centroid.z
// to whole-area mins.z (still a single area-wide scalar) collapsed
// crossfire.bsp's reachable areas from 780 to 615 and lost both ladder
// edges -- a worse regression than the bug it was meant to fix, because
// mins.z inherited the same "one scalar for a spatially-varying floor"
// flaw centroid.z had, just in the other direction.
//
// The corner inset exists because the raw vertex is a literal polytope
// corner -- exactly where several boundary planes (floor + one or more
// walls) meet with zero slack, the worst-case spot to aim a real hull at.
// Confirmed live on crossfire.bsp: a jump waypoint's target vertex was
// simultaneously the area's floor AND its wall boundary (origin.x sitting
// exactly hull-radius-distance from a real wall, origin.z exactly on the
// floor) -- a bot placed there (even freshly spawned and teleported
// straight to it, nothing else going on) never registered FL_ONGROUND and
// never moved at all: v_angle stopped changing, velocity.z pinned near
// zero tick after tick, zero displacement. Pulling the point toward the
// area's own centroid (guaranteed interior, since every area is provably
// convex) by a hull-radius-sized margin gives a real hull standing there
// actual clearance from whichever wall(s) met at that corner.
bool NearestFloorPoint(const Area &area, const Vec3 &target, Vec3 &outPoint)
{
	bool found = false;
	double bestDist2 = 0.0;
	Vec3 best;
	for (const Face &f : area.polytope.faces)
	{
		if (f.plane.normal.z < kFloorNormalThreshold)
			continue; // wall or ceiling face, not a standable floor

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
		// Capped at half the distance to the centroid so a small area
		// can't send the point past its own middle and out the other side.
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
	}
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

// A brush entity built around an "origin" brush (the hlcsg convention for
// giving a rotating/moving brush model a pivot point -- func_door_rotating,
// func_rotating, func_train, ...) gets its solid geometry re-expressed by
// the compiler in *local* space relative to that origin brush's centroid,
// with the centroid itself written back into the compiled entity lump as an
// "origin" keyvalue (confirmed live: test9_door's func_door_rotating has no
// "origin" in its *source* .map at all -- the mapper only placed the origin
// brush -- yet the compiled .bsp's entity text has "origin" "384 144 64",
// hlcsg's own addition). dmodel_t::mins/maxs is that same local geometry, so
// using it directly as a world-space box (as if it were an ordinary,
// non-origin-brush entity like a plain func_door) undercounts by however far
// the origin sits from world (0,0,0) -- confirmed live on test9_door: a
// door's local box (-8,-64,-64)-(8,0,32) sitting near true world position
// (384,144,64) found zero overlapping areas, when the real footprint (both
// mins/maxs shifted by that origin) properly bridges both sides. Returns
// (0,0,0) if the entity has no parseable "origin" key (the common case for
// a brush entity with no origin brush at all, e.g. a plain func_door) --
// adding a zero offset is a no-op, so this is safe to always apply.
Vec3 ParseEntityOrigin(const std::string &origin)
{
	double x = 0, y = 0, z = 0;
	std::sscanf(origin.c_str(), "%lf %lf %lf", &x, &y, &z);
	return Vec3(x, y, z);
}

void TagLadder(const dmodel_t &model, const Vec3 &origin, const AreaBoundsCache &cache,
	std::vector<TaggedSpecialEdge> &out)
{
	// Margin-expanded: a ladder's own climb-trigger brush is often only a
	// few units thick (its footprint sits flush against a wall), well
	// inside the 16-unit-per-side hull-1 Minkowski shrink --
	// meaning no navigable area's AABB would ever directly overlap the raw
	// brush bounds at all. Search nearby instead of requiring literal
	// overlap (found via a real miss on crossfire.bsp: a ladder only 4
	// units wide in Y produced zero overlapping areas without this margin).
	Vec3 boxMins(origin.x + model.mins[0] - kAreaAabbMargin, origin.y + model.mins[1] - kAreaAabbMargin,
		origin.z + model.mins[2] - kAreaAabbMargin);
	Vec3 boxMaxs(origin.x + model.maxs[0] + kAreaAabbMargin, origin.y + model.maxs[1] + kAreaAabbMargin,
		origin.z + model.maxs[2] + kAreaAabbMargin);

	std::vector<int> overlapping = FindOverlappingAreas(boxMins, boxMaxs, cache);
	if (overlapping.size() < 2)
		return; // ladder too short/isolated to connect two distinct areas; nothing to tag

	int bottom = overlapping[0], top = overlapping[0];
	for (int a : overlapping)
	{
		if (cache.centroid[static_cast<size_t>(a)].z < cache.centroid[static_cast<size_t>(bottom)].z)
			bottom = a;
		if (cache.centroid[static_cast<size_t>(a)].z > cache.centroid[static_cast<size_t>(top)].z)
			top = a;
	}
	if (bottom == top)
		return;

	Vec3 center((boxMins.x + boxMaxs.x) * 0.5, (boxMins.y + boxMaxs.y) * 0.5, 0.0);

	TaggedSpecialEdge edge;
	edge.areaFrom = bottom;
	edge.areaTo = top;
	edge.type = EDGE_LADDER;
	edge.anchorFrom = Vec3(center.x, center.y, boxMins.z);
	edge.anchorTo = Vec3(center.x, center.y, boxMaxs.z);
	out.push_back(edge);
}

} // namespace

std::vector<TaggedSpecialEdge> TagSpecialEdges(const BspFile &bsp, const std::vector<Entity> &entities,
	const std::vector<Area> &areas)
{
	std::vector<TaggedSpecialEdge> result;
	AreaBoundsCache cache(areas);

	for (const Entity &ent : entities)
	{
		if (ent.Get("classname") != "func_ladder")
			continue; // doors are handled by door_extractor.cpp's ExtractDoorLeaves now, see this file's header comment

		int modelIndex = ParseBrushModelIndex(ent.Get("model"));
		if (modelIndex < 0 || static_cast<size_t>(modelIndex) >= bsp.Models().size())
			continue;
		const dmodel_t &model = bsp.Models()[static_cast<size_t>(modelIndex)];
		Vec3 origin = ParseEntityOrigin(ent.Get("origin"));

		TagLadder(model, origin, cache, result);
	}

	return result;
}

namespace
{
// Plain union-find over area indices, used by ProbeJumpGaps to tell "these
// two areas are already reachable from each other via some chain of real
// walk portals" apart from "these two specific areas just don't happen to
// share a direct portal" -- the two are very different questions. Checking
// only direct-portal adjacency (a std::set of edge pairs) confirmed live to
// under-count connectivity: on a staircase whose treads are each their own
// area, area0 and area4 have no *direct* portal between them but are
// already fully walkable via the intervening tread areas. Direct-adjacency
// checking alone still added a proximity-derived "jump" edge straight
// between them (nothing geometrically wrong with it -- open air, no wall in
// the way -- just entirely redundant), and because A*'s edge cost is
// centroid-to-centroid distance, that shortcut was often *cheaper* than the
// real multi-hop walk chain, so the router picked it and sent the bot
// jumping over a staircase it could have just walked up normally.
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
} // namespace

void ProbeJumpGaps(const BspFile &bsp, int headnode, const std::vector<Area> &areas,
	std::vector<MergedPortal> &portals)
{
	AreaBoundsCache cache(areas);

	DisjointSet components(areas.size());
	for (const MergedPortal &p : portals)
	{
		// Only union through bidirectional edges. Drop edges (EDGE_DROP) are
		// one-way (EDGE_FLAG_ONE_WAY) and must not be treated as connectivity
		// for jump-gap detection -- otherwise areas separated by a drop will
		// appear "already connected" and no jump edge will be added.
		if ((p.edgeType == EDGE_WALK || p.edgeType == EDGE_JUMP ||
			 p.edgeType == EDGE_LADDER || p.edgeType == EDGE_ELEVATOR) &&
			(p.edgeFlags & EDGE_FLAG_ONE_WAY) == 0)
		{
			components.Union(p.areaA, p.areaB);
		}
	}

	size_t n = areas.size();
	for (size_t i = 0; i < n; ++i)
	{
		for (size_t j = i + 1; j < n; ++j)
		{
			if (components.Find(static_cast<int>(i)) == components.Find(static_cast<int>(j)))
				continue; // already reachable via a real walk-portal chain -- no jump needed

			const Vec3 &ci = cache.centroid[i];
			const Vec3 &cj = cache.centroid[j];
			// Cheap coarse reject first (whole-area centroids, generous
			// slack) before paying for the real per-gap floor-vertex
			// search below -- this pair is nowhere near jump range at all,
			// no need to scan either area's polytope.
			double coarseHorizDist = std::sqrt((ci.x - cj.x) * (ci.x - cj.x) + (ci.y - cj.y) * (ci.y - cj.y));
			if (coarseHorizDist > kMaxJumpHorizontalDistance + kAreaAabbMargin)
				continue;

			// Refine using each area's actual nearest floor point to the
			// other side, not the whole area's centroid/mins -- see
			// NearestFloorPoint's comment for why a single area-wide
			// scalar (either one) is unreliable once MergeAreas has folded
			// unrelated geometry (a dip, an alcove, a ramp) into the same
			// area as the part that's actually near this gap.
			Vec3 nearA, nearB;
			if (!NearestFloorPoint(areas[i], cj, nearA) || !NearestFloorPoint(areas[j], ci, nearB))
				continue; // no floor-like face at all (shouldn't happen for a real walkable area)

			double horizDist = std::sqrt((nearA.x - nearB.x) * (nearA.x - nearB.x) +
				(nearA.y - nearB.y) * (nearA.y - nearB.y));
			double vertDelta = std::fabs(nearA.z - nearB.z);

			if (horizDist > kMaxJumpHorizontalDistance || vertDelta > kMaxJumpHeightDelta)
				continue;

			// Distance and height alone can't tell a real gap from a wall two
			// rooms just happen to sit close across -- see SegmentClear's
			// comment for the live crossfire.bsp case this caught. Sample at
			// standing-origin height above each floor point (kHumanHullMaxZ,
			// the same convention hull-1 itself uses -- see hull_defs.h),
			// not the raw floor points themselves: a hull-1 origin classified
			// right at its own floor sits exactly on the leaf boundary and
			// can falsely read as solid, the same zero-clearance problem
			// NearestFloorPoint's own corner-inset exists to avoid.
			Vec3 clearanceA(nearA.x, nearA.y, nearA.z + kHumanHullMaxZ);
			Vec3 clearanceB(nearB.x, nearB.y, nearB.z + kHumanHullMaxZ);
			if (!SegmentClear(bsp, headnode, clearanceA, clearanceB))
				continue;

			MergedPortal jump;
			jump.areaA = static_cast<int>(i);
			jump.areaB = static_cast<int>(j);
			jump.pointA = nearA;
			jump.pointB = nearB;
			jump.area = 0.0; // not a real shared-face overlap -- a proximity-derived edge
			jump.edgeType = EDGE_JUMP;
			portals.push_back(jump);
			components.Union(static_cast<int>(i), static_cast<int>(j));
		}
	}
}

void TagDropOffs(const std::vector<Area> &areas, std::vector<MergedPortal> &portals,
	const std::vector<DoorAreaInfo> &doorAreaInfo)
{
	bool haveDoor = doorAreaInfo.size() == areas.size();

	for (MergedPortal &p : portals)
	{
		if (p.edgeType != EDGE_WALK)
			continue; // don't reclassify jump/ladder/door edges as drops

		// A door leaf's own floor face lands at a wildly different height
		// than its real-world neighbors for reasons that have nothing to do
		// with an actual step/cliff -- see this function's own header
		// comment for the full Minkowski-expansion-direction explanation.
		if (haveDoor && (doorAreaInfo[static_cast<size_t>(p.areaA)].isDoor || doorAreaInfo[static_cast<size_t>(p.areaB)].isDoor))
			continue;

		// Query near the shared boundary itself (real local geometry for a
		// genuine walk portal), not each area's whole-area mins -- same
		// reasoning as ProbeJumpGaps' NearestFloorPoint refinement above:
		// an area's global lowest point can belong to unrelated geometry
		// (a dip, alcove, ramp) folded into the same merged area by
		// MergeAreas, far from this specific boundary.
		Vec3 midpoint = (p.pointA + p.pointB) * 0.5;
		Vec3 floorA, floorB;
		if (!NearestFloorPoint(areas[static_cast<size_t>(p.areaA)], midpoint, floorA) ||
			!NearestFloorPoint(areas[static_cast<size_t>(p.areaB)], midpoint, floorB))
			continue; // no floor face at all -- leave classification alone

		double delta = std::fabs(floorA.z - floorB.z);

		if (delta <= kWalkableStepHeight)
			continue; // genuinely walkable in either direction -- leave as EDGE_WALK

		// From here on this edge needs a real floor point to aim at, not
		// the shared-face's most-separated corners (portal_builder.cpp's
		// PolygonDiameter picks those for area coverage purposes -- they
		// can sit anywhere from floor to ceiling height on a tall shared
		// wall, not necessarily either side's actual floor).
		p.pointA = floorA;
		p.pointB = floorB;

		if (delta <= kMaxJumpHeightDelta)
		{
			// Too tall to walk up (past kWalkableStepHeight) but well
			// within normal jump range -- bidirectional, unlike the
			// EDGE_DROP case below: jumping while stepping *down* across
			// it is harmless (lands fine either way), it's only climbing
			// up that actually needs the jump input, so there's no
			// one-way restriction to apply here.
			p.edgeType = EDGE_JUMP;
			continue;
		}

		p.edgeType = EDGE_DROP;
		p.edgeFlags |= EDGE_FLAG_ONE_WAY;

		// EDGE_FLAG_ONE_WAY's documented convention (relied on by the
		// runtime A*, src/nav/path_astar.cpp) is "traversable areaA ->
		// areaB only." area_merger assigns areaA/areaB in arbitrary
		// portal-discovery order, so without this swap a one-way drop could
		// just as easily end up pointing the wrong way (blocking the real
		// fall and permitting a nonsensical "climb" instead). Ensure areaA
		// is always the higher (from) side here, once, rather than push
		// that assumption onto every consumer.
		if (floorA.z < floorB.z)
		{
			std::swap(p.areaA, p.areaB);
			std::swap(p.pointA, p.pointB);
		}
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
// Small horizontal search radius, unlike ProbeJumpGaps' 200-unit jump range
// -- a crouch link is meant to stand in for a portal_builder shared face
// that just happened not to match, i.e. two genuinely adjacent leaves, not
// a real gap to cross. ~1.5x a standing hull's radius, same reasoning as
// kAreaAabbMargin.
constexpr double kCrouchLinkMaxHorizDistance = 48.0;

// A small inward lift off the exact floor-face vertex NearestFloorPoint
// returns, NOT a hull-height offset the way ProbeJumpGaps' clearance sample
// uses (floor + kHumanHullMaxZ) -- that convention assumes the leaf's own
// valid origin range comfortably spans a full hull height above its floor,
// true for a hull1 leaf (guaranteed >=72 units tall by construction) but
// false here: a hull-3-only leaf can be as little as a duck hull's own
// ~12-unit clearance margin tall, so adding kDuckHullMaxZ (32) to its own
// floor point overshoots clean past that leaf's own ceiling and always
// misreads as solid. This just needs to clear the "exactly on the boundary
// plane" epsilon problem NearestFloorPoint's own corner-inset comment
// describes, not reconstruct a full hull height.
constexpr double kCrouchProbeLift = 6.0;
}

void ProbeCrouchLinks(const BspFile &bsp, int duckHeadnode, const std::vector<Area> &areas,
	const std::vector<bool> &isCrouchLeaf, std::vector<MergedPortal> &portals)
{
	std::vector<bool> isCrouchArea = ComputeCrouchAreaFlags(areas, isCrouchLeaf);

	AreaBoundsCache cache(areas);

	DisjointSet components(areas.size());
	for (const MergedPortal &p : portals)
		components.Union(p.areaA, p.areaB);

	bool debugDump = std::getenv("NAVGEN_DUMP_CROUCH") != nullptr;

	size_t n = areas.size();
	for (size_t i = 0; i < n; ++i)
	{
		if (!isCrouchArea[i])
			continue; // only bridging a crouch area to its neighbors -- plain hull1<->hull1 gaps are portal_builder's job

		for (size_t j = 0; j < n; ++j)
		{
			if (i == j)
				continue;
			if (components.Find(static_cast<int>(i)) == components.Find(static_cast<int>(j)))
				continue; // already reachable via some other chain

			const Vec3 &ci = cache.centroid[i];
			const Vec3 &cj = cache.centroid[j];
			double coarseHorizDist = std::sqrt((ci.x - cj.x) * (ci.x - cj.x) + (ci.y - cj.y) * (ci.y - cj.y));
			if (coarseHorizDist > kCrouchLinkMaxHorizDistance + kAreaAabbMargin)
				continue; // cheap reject before the real per-gap floor-vertex search below

			Vec3 nearA, nearB;
			if (!NearestFloorPoint(areas[i], cj, nearA) || !NearestFloorPoint(areas[j], ci, nearB))
			{
				if (debugDump)
					std::printf("  crouch-probe area%zu<->area%zu: no floor point found\n", i, j);
				continue;
			}

			double horizDist = std::sqrt((nearA.x - nearB.x) * (nearA.x - nearB.x) +
				(nearA.y - nearB.y) * (nearA.y - nearB.y));

			// nearA/nearB are hull *origin* heights, not literal floor
			// heights, and the two areas can come from different hulls
			// (duck origin sits floor+18, standing origin sits floor+36) --
			// subtracting the raw z values directly would compare apples to
			// oranges, off by exactly the 18-unit gap between the two
			// offsets whenever the real floors actually match. Converting
			// each back to its own area's real floor height first (origin +
			// that hull's own, negative, min-Z) before comparing is what
			// vertDelta actually needs to mean here: "how different are the
			// two real floor heights," not "how different are two
			// differently-encoded coordinates that happen to share a unit."
			double floorA = nearA.z + (isCrouchArea[i] ? kDuckHullMinZ : kHumanHullMinZ);
			double floorB = nearB.z + (isCrouchArea[j] ? kDuckHullMinZ : kHumanHullMinZ);
			double vertDelta = std::fabs(floorA - floorB);

			if (debugDump)
				std::printf("  crouch-probe area%zu<->area%zu: nearA=(%.1f,%.1f,%.1f) nearB=(%.1f,%.1f,%.1f) "
					"floorA=%.1f floorB=%.1f horizDist=%.1f vertDelta=%.1f\n",
					i, j, nearA.x, nearA.y, nearA.z, nearB.x, nearB.y, nearB.z, floorA, floorB, horizDist, vertDelta);

			// Ducking through is still just walking, no jump input -- keep
			// this to plain step-height range, not jump range.
			if (horizDist > kCrouchLinkMaxHorizDistance || vertDelta > kWalkableStepHeight)
				continue;

			// Sampled against hull 3's own clipnode tree -- this is the whole
			// reason this connection needs a separate probe instead of a
			// normal portal in the first place. Both endpoints are placed at
			// the SAME real-world height: each area's own true floor (just
			// computed above) plus duck's own minimum valid origin offset
			// (-kDuckHullMinZ) plus a small inward lift -- not nearA.z/
			// nearB.z directly, which are raw hull-origin coordinates on two
			// different scales (see floorA/floorB's own comment) and would
			// put the two endpoints at two different absolute heights,
			// turning a level duck-height clearance check into a spurious
			// diagonal one that climbs straight into the low ceiling.
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

			MergedPortal link;
			link.areaA = static_cast<int>(i);
			link.areaB = static_cast<int>(j);
			link.pointA = nearA;
			link.pointB = nearB;
			link.area = 0.0; // proximity-derived, not a real shared-face overlap
			link.edgeType = EDGE_WALK;
			portals.push_back(link);
			components.Union(static_cast<int>(i), static_cast<int>(j));
		}
	}
}

} // namespace navgen
