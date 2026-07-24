#include "blocking_entities.h"

#include <algorithm>
#include <cstdio>
#include <cstdlib>

#include "hull_defs.h"
#include "polytope.h"

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

namespace
{
	// Subtract the UNION of multiple convex polytopes from a subject polytope.
	// We split the subject by ALL face planes of ALL subtrahends at once,
	// then keep only fragments that are outside EVERY subtrahend.
	// This gives order-independent results (unlike sequential subtraction).
	std::vector<Polytope> SubtractUnion(const Polytope &subject,
		const std::vector<const Polytope*> &subtrahends,
		double mapRadius, double epsilon)
	{
		if (subtrahends.empty())
			return {subject};

		// Collect all unique face planes from all subtrahends
		struct PlaneWithNormal { Plane plane; Vec3 normal; };
		std::vector<PlaneWithNormal> allPlanes;
		for (const Polytope *sub : subtrahends)
		{
			for (const Face &f : sub->faces)
			{
				bool duplicate = false;
				for (const auto &pwn : allPlanes)
				{
					if (std::fabs(f.plane.normal.x - pwn.plane.normal.x) < epsilon &&
						std::fabs(f.plane.normal.y - pwn.plane.normal.y) < epsilon &&
						std::fabs(f.plane.normal.z - pwn.plane.normal.z) < epsilon &&
						std::fabs(f.plane.dist - pwn.plane.dist) < epsilon)
					{
						duplicate = true;
						break;
					}
				}
				if (!duplicate)
					allPlanes.push_back({f.plane, f.plane.normal});
			}
		}

		if (allPlanes.empty())
			return {subject};

		// Recursively split subject by all planes
		std::function<void(const Polytope&, size_t, std::vector<Polytope>&)> carveRecursive;
		carveRecursive = [&](const Polytope &poly, size_t planeIdx, std::vector<Polytope> &out)
		{
			if (poly.Empty() || planeIdx >= allPlanes.size())
			{
				// Leaf fragment: keep if outside ALL subtrahends
				for (const Polytope *sub : subtrahends)
				{
					if (ConvexOverlap(poly, *sub, epsilon))
						return; // inside at least one subtrahend, discard
				}
				out.push_back(poly);
				return;
			}

			const Plane &plane = allPlanes[planeIdx].plane;
			const Vec3 &normal = allPlanes[planeIdx].normal;

			// Classify poly vertices against this plane
			bool hasFront = false, hasBack = false;
			for (const Face &f : poly.faces)
			{
				for (const Vec3 &v : f.winding)
				{
					double d = v.Dot(normal) - plane.dist;
					if (d > epsilon) hasFront = true;
					else if (d < -epsilon) hasBack = true;
					if (hasFront && hasBack) break;
				}
				if (hasFront && hasBack) break;
			}

			if (hasFront && hasBack)
			{
				// Straddles - split
				Polytope frontPoly, backPoly;
				for (const Face &f : poly.faces)
				{
					const auto &w = f.winding;
					std::vector<double> faceDists;
					faceDists.reserve(w.size());
					for (const Vec3 &v : w)
						faceDists.push_back(v.Dot(normal) - plane.dist);

					std::vector<Vec3> frontW, backW;
					for (size_t i = 0; i < w.size(); ++i)
					{
						size_t j = (i + 1) % w.size();
						double di = faceDists[i], dj = faceDists[j];
						const Vec3 &vi = w[i], &vj = w[j];

						if (di >= -epsilon) frontW.push_back(vi);
						if (di <= epsilon) backW.push_back(vi);

						if ((di > epsilon && dj < -epsilon) || (di < -epsilon && dj > epsilon))
						{
							double t = di / (di - dj);
							Vec3 inter = vi + (vj - vi) * t;
							frontW.push_back(inter);
							backW.push_back(inter);
						}
					}

					if (frontW.size() >= 3)
					{
						Face nf = f;
						nf.winding = std::move(frontW);
						frontPoly.faces.push_back(std::move(nf));
					}
					if (backW.size() >= 3)
					{
						Face nf = f;
						nf.winding = std::move(backW);
						backPoly.faces.push_back(std::move(nf));
					}
				}

				if (!frontPoly.Empty())
					carveRecursive(frontPoly, planeIdx + 1, out);
				if (!backPoly.Empty())
					carveRecursive(backPoly, planeIdx + 1, out);
			}
			else if (hasBack)
			{
				// Entirely on back side (outside this plane's subtrahend half-space)
				carveRecursive(poly, planeIdx + 1, out);
			}
			else
			{
				// Entirely on front side (inside this plane's subtrahend half-space)
				// Continue to test against other planes
				carveRecursive(poly, planeIdx + 1, out);
			}
		};

		std::vector<Polytope> result;
		carveRecursive(subject, 0, result);
		return result;
	}
}

std::vector<Polytope> SubtractBlockingVolumes(const BspFile &bsp, const Polytope &leafPolytope,
	const std::vector<BlockingVolume> &blockers, double mapRadius, double epsilon)
{
	std::vector<Polytope> pieces{leafPolytope};
	Aabb leafBox = ComputeAabb(leafPolytope);

	// Collect all blockers that overlap this leaf
	std::vector<const BlockingVolume*> overlappingBlockers;
	for (const BlockingVolume &blocker : blockers)
	{
		if (blocker.headnode < 0)
			continue;
		Aabb blockerBox = ComputeAabb(blocker.polytope);
		if (!AabbOverlap(leafBox, blockerBox, kOverlapEpsilon))
			continue;
		overlappingBlockers.push_back(&blocker);
	}

	if (overlappingBlockers.empty())
		return pieces;

	// Sort for deterministic behavior regardless of entity grouping
	struct BlockingVolumeWithKey { const BlockingVolume *vol; Vec3 center; };
	std::vector<BlockingVolumeWithKey> sorted;
	sorted.reserve(overlappingBlockers.size());
	for (const BlockingVolume *bv : overlappingBlockers)
	{
		Aabb box = ComputeAabb(bv->polytope);
		Vec3 center = (box.mins + box.maxs) * 0.5;
		sorted.push_back({bv, center});
	}
	std::sort(sorted.begin(), sorted.end(),
		[](const BlockingVolumeWithKey &a, const BlockingVolumeWithKey &b)
		{
			if (std::fabs(a.center.x - b.center.x) > 0.001)
				return a.center.x < b.center.x;
			if (std::fabs(a.center.y - b.center.y) > 0.001)
				return a.center.y < b.center.y;
			return a.center.z < b.center.z;
		});

	// Subtract all overlapping blockers sequentially in sorted order
	for (const auto &sv : sorted)
	{
		const BlockingVolume &blocker = *sv.vol;
		std::vector<Polytope> next;
		for (Polytope &piece : pieces)
		{
			if (!ConvexOverlap(piece, blocker.polytope, kOverlapEpsilon))
			{
				next.push_back(std::move(piece));
				continue;
			}

			std::vector<Polytope> carved = SubtractConvexPolytopes(piece, blocker.polytope, mapRadius, epsilon);
			for (Polytope &c : carved)
				next.push_back(std::move(c));
		}
		pieces = std::move(next);
	}

	return pieces;
}

} // namespace navgen
