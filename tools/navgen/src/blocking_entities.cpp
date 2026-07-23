#include "blocking_entities.h"

#include <cstdio>
#include <cstdlib>

#include "clipnode_tree.h"
#include "hull_defs.h"

namespace navgen
{

namespace
{

constexpr double kOverlapEpsilon = 0.5; // map units of slack, matching area_merger's convexity epsilon

// func_illusionary is deliberately excluded -- confirmed via crossfire.bsp
// (8 instances) that GoldSrc's convention for "looks solid, has no
// collision" is exactly this classname. Extend this list as more
// *permanently* solid brush-entity classnames are encountered; don't guess
// ahead of evidence.
//
// func_detail (a ZHLT compiler extension, not a native GoldSrc entity --
// confirmed via test27_detail.bsp's compiled entity lump, which still
// carries zhlt_detaillevel/zhlt_clipnodedetaillevel keyvalues and its own
// separate "model" "*N") only affects the VIS/RAD compile stages' handling
// of the brush (excluded from BSP visibility splits, to keep detail
// geometry from needlessly fragmenting the world's PVS) -- it does NOT
// change collision at all, and ZHLT still compiles it as its own ordinary
// solid brush model, exactly like func_wall, not folded into worldspawn's
// own model 0. Confirmed live: without this, test27_detail's two func_detail
// walls (meant to reproduce test1_simple's room layout) were invisible to
// leaf extraction entirely, collapsing the whole map to one open box with
// zero internal portals instead of the intended multi-room layout.
//
// func_door/func_door_rotating ARE included here too, same as a
// permanently-solid blocker -- worldspawn's own hull-1 clip tree never
// splits around a *different* model's brushwork (a door is a separate
// entity, exactly like func_wall), so without subtracting the door's
// footprint away, the leaf on either side of a doorway with no other gap
// stays one single continuous leaf that never knew the door was there at
// all. door_extractor.cpp then refills that exact carved-out gap with the
// door's own leaf (flagged NAV_AREA_DOOR), so the split isn't a permanent
// hole -- portal_builder's ordinary coplanar-face matching finds the real
// shared-face portals between the door leaf and both neighboring rooms
// (both extracted from the same hull-1 clip data, so their shared
// boundaries land on identical planes), and the "wait for it to actually
// open" behavior lives on that leaf/area instead of a synthetic edge.
// Confirmed live on test9_door: extracting the door leaf WITHOUT also
// subtracting it first left the door leaf sitting entirely inside
// worldspawn's still-unsplit, already-continuous leaf, with no real
// boundary anywhere for portal_builder to find -- 0 of 2 door leaves ended
// up connected to anything, both pruned as void.
bool IsBlockingClassname(const std::string &classname)
{
	return classname == "func_wall" || classname == "func_breakable" || classname == "func_detail" ||
		classname == "func_door" || classname == "func_door_rotating";
}

// "model" keyvalues on brush entities are "*N" (N = index into
// bsp.Models()). Returns -1 if `model` isn't that form.
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

// See leaf_extractor.h's ExtractedLeaf-adjacent comment / door_extractor.cpp
// for the full hlcsg origin-brush convention -- duplicated here (rather than
// shared) matching this module's existing style of small per-file utility
// copies (ParseBrushModelIndex above is already one). Returns (0,0,0) --a
// safe no-op offset -- for an entity with no origin brush at all.
Vec3 ParseEntityOrigin(const std::string &origin)
{
	double x = 0, y = 0, z = 0;
	std::sscanf(origin.c_str(), "%lf %lf %lf", &x, &y, &z);
	return Vec3(x, y, z);
}

struct Aabb
{
	Vec3 mins{1e30, 1e30, 1e30};
	Vec3 maxs{-1e30, -1e30, -1e30};
};

Aabb ComputeAabb(const Polytope &poly)
{
	Aabb box;
	for (const Face &f : poly.faces)
	{
		for (const Vec3 &v : f.winding)
		{
			box.mins.x = std::min(box.mins.x, v.x);
			box.mins.y = std::min(box.mins.y, v.y);
			box.mins.z = std::min(box.mins.z, v.z);
			box.maxs.x = std::max(box.maxs.x, v.x);
			box.maxs.y = std::max(box.maxs.y, v.y);
			box.maxs.z = std::max(box.maxs.z, v.z);
		}
	}
	return box;
}

bool AabbOverlap(const Aabb &a, const Aabb &b, double epsilon)
{
	return a.mins.x <= b.maxs.x + epsilon && a.maxs.x >= b.mins.x - epsilon &&
		a.mins.y <= b.maxs.y + epsilon && a.maxs.y >= b.mins.y - epsilon &&
		a.mins.z <= b.maxs.z + epsilon && a.maxs.z >= b.mins.z - epsilon;
}

// True if some face of `p` has every vertex of `other` strictly outside
// p's half-space for that face -- a clean separating plane.
bool HasSeparatingFace(const Polytope &p, const Polytope &other, double epsilon)
{
	for (const Face &f : p.faces)
	{
		bool allOutside = true;
		for (const Face &of : other.faces)
		{
			for (const Vec3 &v : of.winding)
			{
				if (v.Dot(f.plane.normal) - f.plane.dist >= -epsilon)
				{
					allOutside = false;
					break;
				}
			}
			if (!allOutside)
				break;
		}
		if (allOutside)
			return true;
	}
	return false;
}

bool ConvexOverlap(const Polytope &a, const Polytope &b, double epsilon)
{
	if (HasSeparatingFace(a, b, epsilon))
		return false;
	if (HasSeparatingFace(b, a, epsilon))
		return false;
	return true;
}

} // namespace

std::vector<BlockingVolume> ExtractBlockingVolumes(const BspFile &bsp, const std::vector<Entity> &entities, double mapRadius)
{
	std::vector<BlockingVolume> result;

	for (const Entity &ent : entities)
	{
		std::string classname = ent.Get("classname");
		if (!IsBlockingClassname(classname))
			continue;

		int modelIndex = ParseBrushModelIndex(ent.Get("model"));
		if (modelIndex < 0 || static_cast<size_t>(modelIndex) >= bsp.Models().size())
			continue; // not a brush entity, or a malformed/out-of-range reference

		const dmodel_t &model = bsp.Models()[static_cast<size_t>(modelIndex)];
		Vec3 origin = ParseEntityOrigin(ent.Get("origin"));

		WalkClipnodeTreeForContents(bsp, model.headnode[kHullHuman], CONTENTS_SOLID,
			[&](int /*contents*/, const std::vector<PlaneChainEntry> &chain)
			{
				Polytope poly = BuildPolytopeFromChain(bsp, chain, mapRadius);
				if (poly.Empty())
					return;

				// Origin-brush correction (a no-op for the common case of no
				// origin brush at all -- see ParseEntityOrigin): the
				// clipnode tree's own planes are in local space relative to
				// the origin brush's centroid, not world space.
				BlockingVolume vol;
				vol.classname = classname;
				vol.polytope = TranslatePolytope(poly, origin);
				vol.headnode = model.headnode[kHullHuman];
				vol.origin = origin;
				result.push_back(std::move(vol));
			});
	}

	return result;
}

bool OverlapsAnyBlockingVolume(const ExtractedLeaf &leaf, const std::vector<BlockingVolume> &blockers)
{
	if (blockers.empty())
		return false;

	Aabb leafBox = ComputeAabb(leaf.polytope);
	for (const BlockingVolume &blocker : blockers)
	{
		Aabb blockerBox = ComputeAabb(blocker.polytope);
		if (!AabbOverlap(leafBox, blockerBox, kOverlapEpsilon))
			continue;
		if (ConvexOverlap(leaf.polytope, blocker.polytope, kOverlapEpsilon))
			return true;
	}
	return false;
}

std::vector<Polytope> SubtractBlockingVolumes(const BspFile &bsp, const Polytope &leafPolytope,
	const std::vector<BlockingVolume> &blockers, double mapRadius, double epsilon)
{
	std::vector<Polytope> pieces{leafPolytope};
	Aabb leafBox = ComputeAabb(leafPolytope);

	// Carve against each distinct blocking ENTITY's clipnode tree, at most
	// once per entity: after one carve every surviving piece is already
	// wholly outside that entity's solid, so its remaining volumes have
	// nothing left to remove. The previous design subtracted the entity's
	// extracted convex solid volumes one at a time instead -- those are
	// hull-Minkowski-expanded leaves that overlap each other heavily, so
	// each subtraction re-fragmented everything the previous one produced;
	// on test35_smoothwall's many-segment curved func_wall that fragmented
	// combinatorially all the way to std::bad_alloc. See
	// CarvePolytopeAgainstTree's own comment.
	std::vector<std::pair<int, Vec3>> carvedEntities;
	for (const BlockingVolume &blocker : blockers)
	{
		if (blocker.headnode < 0)
			continue;
		Aabb blockerBox = ComputeAabb(blocker.polytope);
		if (!AabbOverlap(leafBox, blockerBox, kOverlapEpsilon))
			continue;

		bool alreadyCarved = false;
		for (const auto &[headnode, origin] : carvedEntities)
		{
			if (headnode == blocker.headnode && std::fabs(origin.x - blocker.origin.x) < 0.01 &&
				std::fabs(origin.y - blocker.origin.y) < 0.01 && std::fabs(origin.z - blocker.origin.z) < 0.01)
			{
				alreadyCarved = true;
				break;
			}
		}
		if (alreadyCarved)
			continue;

		std::vector<Polytope> next;
		bool carvedAny = false;
		for (Polytope &piece : pieces)
		{
			// Pieces nowhere near this entity pass through untouched --
			// carving them anyway would still be geometrically correct
			// (every fragment lands in an EMPTY leaf and survives) but
			// would fragment them along the tree's planes for no reason.
			bool overlapsEntity = false;
			for (const BlockingVolume &other : blockers)
			{
				if (other.headnode != blocker.headnode)
					continue;
				if (ConvexOverlap(piece, other.polytope, kOverlapEpsilon))
				{
					overlapsEntity = true;
					break;
				}
			}
			if (!overlapsEntity)
			{
				next.push_back(std::move(piece));
				continue;
			}

			carvedAny = true;
			// The entity's clipnode planes are in its own local space when
			// an origin brush is involved (see ParseEntityOrigin) -- carve
			// in local space, then translate the survivors back.
			Vec3 negOrigin(-blocker.origin.x, -blocker.origin.y, -blocker.origin.z);
			Polytope local = TranslatePolytope(piece, negOrigin);
			for (Polytope &carved : CarvePolytopeAgainstTree(bsp, blocker.headnode, local, CONTENTS_SOLID, mapRadius))
				next.push_back(TranslatePolytope(carved, blocker.origin));
		}
		pieces = std::move(next);
		if (carvedAny)
			carvedEntities.push_back({blocker.headnode, blocker.origin});
	}

	return pieces;
}

} // namespace navgen
