#include "reachability.h"

#include <cmath>
#include <cstdio>
#include <limits>

namespace navgen
{

namespace
{

constexpr double kPointInAreaEpsilon = 1.0;

bool IsSpawnClassname(const std::string &classname)
{
	return classname == "info_player_start" || classname == "info_player_deathmatch" ||
		classname == "info_player_coop" || classname == "info_player_teamspawn";
}

bool ParseOrigin(const std::string &origin, Vec3 &out)
{
	double x = 0, y = 0, z = 0;
	if (std::sscanf(origin.c_str(), "%lf %lf %lf", &x, &y, &z) != 3)
		return false;
	out = Vec3(x, y, z);
	return true;
}

bool PointInArea(const Vec3 &point, const Area &area, double epsilon)
{
	for (const Face &f : area.polytope.faces)
		if (point.Dot(f.plane.normal) - f.plane.dist < -epsilon)
			return false;
	return true;
}

Vec3 AreaCentroid(const Area &area)
{
	Vec3 sum(0, 0, 0);
	int count = 0;
	for (const Face &f : area.polytope.faces)
		for (const Vec3 &v : f.winding)
		{
			sum = sum + v;
			++count;
		}
	return (count > 0) ? sum * (1.0 / count) : Vec3();
}

} // namespace

int FindAreaContainingPoint(const Vec3 &point, const std::vector<Area> &areas)
{
	int found = -1;
	for (const Area &area : areas)
	{
		if (PointInArea(point, area, kPointInAreaEpsilon))
		{
			found = area.id;
			break;
		}
	}

	if (found < 0 && !areas.empty())
	{
		// Fallback: nearest area by centroid, so a point sitting exactly on
		// a boundary (or just outside due to compile rounding) still
		// resolves to something instead of being silently dropped.
		double best = std::numeric_limits<double>::max();
		for (const Area &area : areas)
		{
			double d = (AreaCentroid(area) - point).Length();
			if (d < best)
			{
				best = d;
				found = area.id;
			}
		}
	}

	return found;
}

std::vector<int> FindSpawnAreas(const std::vector<Entity> &entities, const std::vector<Area> &areas)
{
	std::vector<int> result;

	for (const Entity &ent : entities)
	{
		if (!IsSpawnClassname(ent.Get("classname")))
			continue;

		Vec3 origin;
		if (!ParseOrigin(ent.Get("origin"), origin))
			continue;

		int found = FindAreaContainingPoint(origin, areas);
		if (found >= 0)
			result.push_back(found);
	}

	return result;
}

MergeResult PruneUnreachableAreas(const std::vector<Area> &areas, const std::vector<MergedPortal> &portals,
	const std::vector<int> &seedAreas)
{
	if (seedAreas.empty())
	{
		MergeResult unchanged;
		unchanged.areas = areas;
		unchanged.portals = portals;
		return unchanged;
	}

	std::vector<std::vector<int>> adjacency(areas.size()); // area id -> portal indices touching it
	for (size_t pi = 0; pi < portals.size(); ++pi)
	{
		adjacency[static_cast<size_t>(portals[pi].areaA)].push_back(static_cast<int>(pi));
		adjacency[static_cast<size_t>(portals[pi].areaB)].push_back(static_cast<int>(pi));
	}

	std::vector<bool> reachable(areas.size(), false);
	std::vector<int> queue;
	for (int s : seedAreas)
	{
		if (s >= 0 && static_cast<size_t>(s) < reachable.size() && !reachable[static_cast<size_t>(s)])
		{
			reachable[static_cast<size_t>(s)] = true;
			queue.push_back(s);
		}
	}

	for (size_t qi = 0; qi < queue.size(); ++qi)
	{
		int a = queue[qi];
		for (int pi : adjacency[static_cast<size_t>(a)])
		{
			const MergedPortal &portal = portals[static_cast<size_t>(pi)];
			bool isAreaA = (portal.areaA == a);

			// EDGE_FLAG_ONE_WAY means areaA -> areaB only (areaA is always
			// the higher/"from" side, per TagDropOffs's convention) --
			// respect that here too, the same way the runtime A*
			// (path_astar.cpp) does. Without this, a real bug this pruning
			// pass had: an area only connected to the rest of the graph via
			// a one-way drop (e.g. jump *off* a tower, no way back up)
			// still got flood-filled as "reachable" in this undirected
			// walk, when in practice no bot standing anywhere else could
			// ever legally path there -- confirmed live via crossbot_nav_goal
			// targeting a real crossfire.bsp rooftop area that survived
			// pruning yet A* always reported unreachable from the ground.
			if ((portal.edgeFlags & crossbot_nav::EDGE_FLAG_ONE_WAY) && !isAreaA)
				continue; // trying to go areaB -> areaA against a one-way edge

			int other = isAreaA ? portal.areaB : portal.areaA;
			if (!reachable[static_cast<size_t>(other)])
			{
				reachable[static_cast<size_t>(other)] = true;
				queue.push_back(other);
			}
		}
	}

	MergeResult result;
	std::vector<int> oldToNew(areas.size(), -1);
	for (size_t i = 0; i < areas.size(); ++i)
	{
		if (!reachable[i])
			continue;
		oldToNew[i] = static_cast<int>(result.areas.size());
		Area a = areas[i];
		a.id = oldToNew[i];
		result.areas.push_back(std::move(a));
	}

	for (const MergedPortal &p : portals)
	{
		if (!reachable[static_cast<size_t>(p.areaA)] || !reachable[static_cast<size_t>(p.areaB)])
			continue;
		MergedPortal np = p;
		np.areaA = oldToNew[static_cast<size_t>(p.areaA)];
		np.areaB = oldToNew[static_cast<size_t>(p.areaB)];
		result.portals.push_back(np);
	}

	return result;
}

} // namespace navgen
