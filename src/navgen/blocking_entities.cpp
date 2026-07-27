#include "blocking_entities.h"

#include <algorithm>
#include <cstdio>
#include <cstdlib>

#include "clipnode_tree.h"
#include "hull_defs.h"
#include "polytope.h"

namespace navgen
{

namespace
{

constexpr double kOverlapEpsilon = 0.5;
constexpr double kMinPieceVolume = 1.0;

bool IsBlockingClassname(const std::string &classname)
{
	return classname == "func_wall" || classname == "func_breakable" || classname == "func_detail" ||
		classname == "func_door" || classname == "func_door_rotating";
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

Vec3 ParseEntityOrigin(const std::string &origin)
{
	double x = 0, y = 0, z = 0;
	std::sscanf(origin.c_str(), "%lf %lf %lf", &x, &y, &z);
	return Vec3(x, y, z);
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

bool BoundsOverlap(const Vec3 &aMins, const Vec3 &aMaxs, const Vec3 &bMins, const Vec3 &bMaxs, double epsilon)
{
	return aMins.x <= bMaxs.x + epsilon && aMaxs.x >= bMins.x - epsilon &&
		aMins.y <= bMaxs.y + epsilon && aMaxs.y >= bMins.y - epsilon &&
		aMins.z <= bMaxs.z + epsilon && aMaxs.z >= bMins.z - epsilon;
}

}

std::vector<BlockingVolume> ExtractBlockingVolumes(const BspFile &bsp, const std::vector<Entity> &entities,
	int hullIndex)
{
	Vec3 hullMins(kHumanHullMinX, kHumanHullMinY, kHumanHullMinZ);
	Vec3 hullMaxs(kHumanHullMaxX, kHumanHullMaxY, kHumanHullMaxZ);
	if (hullIndex == kHullDuck)
	{
		hullMins = Vec3(kDuckClipHullMinX, kDuckClipHullMinY, kDuckClipHullMinZ);
		hullMaxs = Vec3(kDuckClipHullMaxX, kDuckClipHullMaxY, kDuckClipHullMaxZ);
	}

	std::vector<BlockingVolume> result;

	for (const Entity &ent : entities)
	{
		if (!IsBlockingClassname(ent.Get("classname")))
			continue;

		int modelIndex = ParseBrushModelIndex(ent.Get("model"));
		if (modelIndex < 0 || static_cast<size_t>(modelIndex) >= bsp.Models().size())
			continue;

		const dmodel_t &model = bsp.Models()[static_cast<size_t>(modelIndex)];
		if (model.headnode[hullIndex] < 0)
			continue;

		BlockingVolume vol;
		vol.headnode = model.headnode[hullIndex];
		vol.origin = ParseEntityOrigin(ent.Get("origin"));
		vol.mins = Vec3(model.mins[0] + hullMins.x, model.mins[1] + hullMins.y,
			model.mins[2] + hullMins.z) + vol.origin;
		vol.maxs = Vec3(model.maxs[0] + hullMaxs.x, model.maxs[1] + hullMaxs.y,
			model.maxs[2] + hullMaxs.z) + vol.origin;
		result.push_back(vol);
	}

	return result;
}

bool OverlapsAnyBlockingVolume(const ExtractedLeaf &leaf, const std::vector<BlockingVolume> &blockers)
{
	Vec3 mins, maxs;
	PolytopeBounds(leaf.polytope, mins, maxs);

	for (const BlockingVolume &blocker : blockers)
		if (BoundsOverlap(mins, maxs, blocker.mins, blocker.maxs, kOverlapEpsilon))
			return true;

	return false;
}

std::vector<Polytope> SubtractBlockingVolumes(const BspFile &bsp, const Polytope &leafPolytope,
	const std::vector<BlockingVolume> &blockers, double mapRadius, double epsilon)
{
	std::vector<Polytope> pieces{leafPolytope};

	for (const BlockingVolume &blocker : blockers)
	{
		std::vector<Polytope> next;
		for (Polytope &piece : pieces)
		{
			Vec3 mins, maxs;
			PolytopeBounds(piece, mins, maxs);
			if (!BoundsOverlap(mins, maxs, blocker.mins, blocker.maxs, kOverlapEpsilon))
			{
				next.push_back(std::move(piece));
				continue;
			}

			Polytope local = TranslatePolytope(piece, blocker.origin * -1.0);
			std::vector<Polytope> carved = CarvePolytopeAgainstTree(bsp, blocker.headnode, local,
				CONTENTS_SOLID, mapRadius, kMinPieceVolume, epsilon);
			for (const Polytope &c : carved)
				next.push_back(TranslatePolytope(c, blocker.origin));
		}
		pieces = std::move(next);
	}

	return pieces;
}

}

