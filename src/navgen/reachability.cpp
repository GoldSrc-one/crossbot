#include "reachability.h"

#include <cmath>
#include <cstdio>
#include <limits>
#include <unordered_set>

#include "nav_math.h"

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

bool IsTargetClassname(const std::string &classname)
{
	return classname == "info_target";
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
	return NavMath::PointInsidePlanes(point, area.boundPlanes, epsilon);
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

}

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

std::vector<int> FindSpawnAreas(const std::vector<Entity> &entities, const std::vector<Area> &areas,
	std::vector<Vec3> *originsOut)
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
		{
			result.push_back(found);
			if (originsOut)
				originsOut->push_back(origin);
		}
	}

	return result;
}

std::vector<int> FindTargetAreas(const std::vector<Entity> &entities, const std::vector<Area> &areas,
	std::vector<Vec3> *originsOut)
{
	std::vector<int> result;

	for (const Entity &ent : entities)
	{
		if (!IsTargetClassname(ent.Get("classname")))
			continue;

		Vec3 origin;
		if (!ParseOrigin(ent.Get("origin"), origin))
			continue;

		int found = FindAreaContainingPoint(origin, areas);
		if (found >= 0)
		{
			result.push_back(found);
			if (originsOut)
				originsOut->push_back(origin);
		}
	}

	return result;
}

MergeResult PruneUnreachableAreas(const std::vector<Area> &areas, const std::vector<MergedConnection> &conns,
	const std::vector<int> &seedAreas)
{
	if (seedAreas.empty())
	{
		MergeResult unchanged;
		unchanged.areas = areas;
		unchanged.conns = conns;
		return unchanged;
	}

	std::vector<std::vector<int>> adjacency(areas.size());
	for (size_t pi = 0; pi < conns.size(); ++pi)
	{
		adjacency[static_cast<size_t>(conns[pi].areaA)].push_back(static_cast<int>(pi));
		adjacency[static_cast<size_t>(conns[pi].areaB)].push_back(static_cast<int>(pi));
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
			const MergedConnection &conn = conns[static_cast<size_t>(pi)];
			bool isAreaA = (conn.areaA == a);

			if ((conn.connFlags & crossbot_nav::CONN_FLAG_ONE_WAY) && !isAreaA)
				continue;

			int other = isAreaA ? conn.areaB : conn.areaA;
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

	for (const MergedConnection &c : conns)
	{
		if (!reachable[static_cast<size_t>(c.areaA)] || !reachable[static_cast<size_t>(c.areaB)])
			continue;
		MergedConnection nc = c;
		nc.areaA = oldToNew[static_cast<size_t>(c.areaA)];
		nc.areaB = oldToNew[static_cast<size_t>(c.areaB)];
		result.conns.push_back(nc);
	}

	return result;
}

}

