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

} // namespace

GraphResult FindPath(const std::vector<AreaNode> &areas, const std::vector<std::vector<uint32_t>> &areaPortalIndices,
	const std::vector<PortalEdge> &portals, uint32_t startArea, uint32_t goalArea)
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
	std::vector<int64_t> cameFromPortal(n, -1);
	std::vector<bool> closed(n, false);

	std::priority_queue<OpenEntry, std::vector<OpenEntry>, OpenEntryCompare> open;

	gScore[startArea] = 0.f;
	open.push({Heuristic(areas, startArea, goalArea), startArea});

	while (!open.empty())
	{
		uint32_t current = open.top().areaId;
		open.pop();

		if (closed[current])
			continue; // stale duplicate entry (lazy-deletion, standard for this priority-queue A* shape)
		closed[current] = true;

		if (current == goalArea)
			break;

		for (uint32_t portalIdx : areaPortalIndices[current])
		{
			const PortalEdge &portal = portals[portalIdx];
			uint32_t neighbor = (portal.areaA == current) ? portal.areaB : portal.areaA;

			if (portal.oneWay && portal.areaA != current)
				continue; // one-way edge (e.g. a drop-off) crossed against its only valid direction

			if (closed[neighbor])
				continue;

			float edgeCost = (areas[current].centroid - areas[neighbor].centroid).Length() * portal.traverseCost;
			float tentativeG = gScore[current] + edgeCost;

			if (tentativeG < gScore[neighbor])
			{
				gScore[neighbor] = tentativeG;
				cameFromArea[neighbor] = current;
				cameFromPortal[neighbor] = portalIdx;
				open.push({tentativeG + Heuristic(areas, neighbor, goalArea), neighbor});
			}
		}
	}

	if (!closed[goalArea] || gScore[goalArea] >= std::numeric_limits<float>::max())
		return result; // unreachable

	std::vector<uint32_t> areaPath, portalPath;
	uint32_t cur = goalArea;
	while (cur != startArea)
	{
		areaPath.push_back(cur);
		portalPath.push_back(static_cast<uint32_t>(cameFromPortal[cur]));
		cur = static_cast<uint32_t>(cameFromArea[cur]);
	}
	areaPath.push_back(startArea);

	std::reverse(areaPath.begin(), areaPath.end());
	std::reverse(portalPath.begin(), portalPath.end());

	result.found = true;
	result.areaPath = std::move(areaPath);
	result.portalPath = std::move(portalPath);
	return result;
}

} // namespace NavAStar
