#include "airmove_prune.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <functional>
#include <limits>
#include <queue>
#include <unordered_map>

namespace navgen
{

namespace
{

struct WalkEdge
{
	size_t to = 0;
	double seconds = 0.0;
};

double TravelSeconds(const std::vector<Area> &areas, const MergedConnection &conn, double maxSpeed)
{
	if (conn.areaA < 0 || conn.areaB < 0)
		return 0.0;

	Vec3 a = areas[static_cast<size_t>(conn.areaA)].polytope.Centroid();
	Vec3 b = areas[static_cast<size_t>(conn.areaB)].polytope.Centroid();
	double travel = (b - a).Length() / std::max(maxSpeed, 1.0);

	return travel + static_cast<double>(ConnectionCostSeconds(conn));
}

std::vector<double> WalkDijkstra(const std::vector<std::vector<WalkEdge>> &graph, size_t from)
{
	std::vector<double> dist(graph.size(), std::numeric_limits<double>::max());

	using Entry = std::pair<double, size_t>;
	std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open;

	dist[from] = 0.0;
	open.push({0.0, from});

	while (!open.empty())
	{
		Entry top = open.top();
		open.pop();

		if (top.first > dist[top.second])
			continue;

		for (const WalkEdge &e : graph[top.second])
		{
			double next = top.first + e.seconds;
			if (next + 1e-6 >= dist[e.to])
				continue;

			dist[e.to] = next;
			open.push({next, e.to});
		}
	}

	return dist;
}

AirmovePruneStats PruneRedundant(const std::vector<Area> &areas, std::vector<MergedConnection> &conns,
	double maxSpeed, double ratio, bool dump, uint8_t judged, const std::function<bool(uint8_t)> &inRoute,
	const char *judgedName, const char *routeName)
{
	AirmovePruneStats stats;

	std::vector<std::vector<WalkEdge>> graph(areas.size());
	for (const MergedConnection &c : conns)
	{
		if (!inRoute(c.connType))
			continue;
		if (c.connType == crossbot_nav::CONN_AIRMOVE && std::fabs(c.gravity - 1.0) > 0.005)
			continue;
		if (c.areaA < 0 || c.areaB < 0)
			continue;

		double seconds = TravelSeconds(areas, c, maxSpeed);
		graph[static_cast<size_t>(c.areaA)].push_back({static_cast<size_t>(c.areaB), seconds});

		if ((c.connFlags & crossbot_nav::CONN_FLAG_ONE_WAY) == 0)
			graph[static_cast<size_t>(c.areaB)].push_back({static_cast<size_t>(c.areaA), seconds});
	}

	std::unordered_map<size_t, std::vector<double>> cache;
	std::vector<bool> drop(conns.size(), false);

	for (size_t i = 0; i < conns.size(); ++i)
	{
		const MergedConnection &c = conns[i];
		if (c.connType != judged)
			continue;
		if (c.areaA < 0 || c.areaB < 0)
			continue;

		++stats.examined;

		size_t from = static_cast<size_t>(c.areaA);
		size_t to = static_cast<size_t>(c.areaB);

		auto distFrom = [&](size_t src) -> const std::vector<double> &
		{
			auto it = cache.find(src);
			if (it == cache.end())
				it = cache.emplace(src, WalkDijkstra(graph, src)).first;
			return it->second;
		};

		double altSeconds = distFrom(from)[to];

		if ((c.connFlags & crossbot_nav::CONN_FLAG_ONE_WAY) == 0)
			altSeconds = std::max(altSeconds, distFrom(to)[from]);

		double connSeconds = TravelSeconds(areas, c, maxSpeed);

		if (altSeconds >= std::numeric_limits<double>::max())
		{
			++stats.noWalkRoute;
			if (dump)
				std::printf("    %s %d->%d kept: no %s route (%s %.2fs)\n", judgedName, c.areaA, c.areaB,
					routeName, judgedName, connSeconds);
			continue;
		}

		bool redundant = altSeconds < ratio * connSeconds;
		if (redundant)
		{
			drop[i] = true;
			++stats.dropped;
		}

		if (dump)
			std::printf("    %s %d->%d %s: %s %.2fs vs %s %.2fs (ratio %.2f)\n", judgedName, c.areaA,
				c.areaB, redundant ? "dropped" : "kept", routeName, altSeconds, judgedName, connSeconds,
				(connSeconds > 0.0) ? altSeconds / connSeconds : 0.0);
	}

	if (stats.dropped > 0)
	{
		std::vector<MergedConnection> kept;
		kept.reserve(conns.size() - stats.dropped);
		for (size_t i = 0; i < conns.size(); ++i)
			if (!drop[i])
				kept.push_back(conns[i]);
		conns = std::move(kept);
	}

	return stats;
}

}

AirmovePruneStats PruneWalkableAirmoves(const std::vector<Area> &areas,
	std::vector<MergedConnection> &conns, double maxSpeed, double ratio, bool dump)
{
	return PruneRedundant(areas, conns, maxSpeed, ratio, dump, crossbot_nav::CONN_AIRMOVE,
		[](uint8_t t) { return t == crossbot_nav::CONN_WALK; }, "airmove", "walk");
}

AirmovePruneStats PruneRedundantLadders(const std::vector<Area> &areas,
	std::vector<MergedConnection> &conns, double maxSpeed, double ratio, bool dump)
{
	return PruneRedundant(areas, conns, maxSpeed, ratio, dump, crossbot_nav::CONN_LADDER,
		[](uint8_t t) { return t == crossbot_nav::CONN_WALK || t == crossbot_nav::CONN_AIRMOVE; },
		"ladder", "walk+air");
}

}
