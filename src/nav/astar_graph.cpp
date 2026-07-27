#include "astar_graph.h"

#include <algorithm>
#include <limits>
#include <queue>

namespace NavAStar
{

namespace
{

struct OpenEntry
{
	float fScore;
	uint32_t areaId;
};

struct OpenEntryCompare
{
	bool operator()(const OpenEntry &a, const OpenEntry &b) const { return a.fScore > b.fScore; }
};

float Heuristic(const std::vector<AreaNode> &areas, uint32_t a, uint32_t b)
{
	return (areas[a].centroid - areas[b].centroid).Length();
}

static Vector ConnMid(const Conn &c)
{
	return (c.pointA + c.pointB) * 0.5f;
}

}

GraphResult FindPath(const std::vector<AreaNode> &areas, const std::vector<std::vector<uint32_t>> &areaConnIndices,
	const std::vector<Conn> &conns, uint32_t startArea, uint32_t goalArea)
{
	GraphResult result;

	if (startArea >= areas.size() || goalArea >= areas.size())
		return result;

	if (startArea == goalArea)
	{
		result.found = true;
		result.areaPath.push_back(startArea);
		return result;
	}

	size_t n = areas.size();
	std::vector<float> gScore(n, std::numeric_limits<float>::max());
	std::vector<int64_t> cameFromArea(n, -1);
	std::vector<int64_t> cameFromConn(n, -1);
	std::vector<bool> closed(n, false);

	std::priority_queue<OpenEntry, std::vector<OpenEntry>, OpenEntryCompare> open;

	gScore[startArea] = 0.f;
	open.push({Heuristic(areas, startArea, goalArea), startArea});

	while (!open.empty())
	{
		uint32_t current = open.top().areaId;
		open.pop();

		if (closed[current])
			continue;
		closed[current] = true;

		if (current == goalArea)
			break;

		for (uint32_t connIdx : areaConnIndices[current])
		{
			const Conn &conn = conns[connIdx];
			uint32_t neighbor = (conn.areaA == current) ? conn.areaB : conn.areaA;

			if (conn.oneWay && conn.areaA != current)
				continue;

			if (closed[neighbor])
				continue;

			Vector connMid = (conn.pointA + conn.pointB) * 0.5f;
			float approachCost = (areas[current].centroid - connMid).Length();
			float crossCost = (connMid - areas[neighbor].centroid).Length() * conn.traverseCost;
			float edgeCost = approachCost + crossCost;
			float tentativeG = gScore[current] + edgeCost;

			if (tentativeG < gScore[neighbor])
			{
				gScore[neighbor] = tentativeG;
				cameFromArea[neighbor] = current;
				cameFromConn[neighbor] = connIdx;
				open.push({tentativeG + Heuristic(areas, neighbor, goalArea), neighbor});
			}
		}
	}

	if (!closed[goalArea] || gScore[goalArea] >= std::numeric_limits<float>::max())
		return result;

	std::vector<uint32_t> areaPath, connPath;
	uint32_t cur = goalArea;
	while (cur != startArea)
	{
		areaPath.push_back(cur);
		connPath.push_back(static_cast<uint32_t>(cameFromConn[cur]));
		cur = static_cast<uint32_t>(cameFromArea[cur]);
	}
	areaPath.push_back(startArea);

	std::reverse(areaPath.begin(), areaPath.end());
	std::reverse(connPath.begin(), connPath.end());

	result.found = true;
	result.areaPath = std::move(areaPath);
	result.connPath = std::move(connPath);
	return result;
}

}

