#include "blocking_entities.h"

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
// always-solid brush-entity classnames are encountered; don't guess ahead
// of evidence.
bool IsBlockingClassname(const std::string &classname)
{
	return classname == "func_wall" || classname == "func_breakable" ||
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

		WalkClipnodeTreeForContents(bsp, model.headnode[kHullHuman], CONTENTS_SOLID,
			[&](int /*contents*/, const std::vector<PlaneChainEntry> &chain)
			{
				Polytope poly = BuildPolytopeFromChain(bsp, chain, mapRadius);
				if (poly.Empty())
					return;

				BlockingVolume vol;
				vol.classname = classname;
				vol.polytope = std::move(poly);
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

} // namespace navgen
