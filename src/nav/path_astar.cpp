#include "path_astar.h"

#include <algorithm>
#include <cstdint>
#include <limits>
#include <queue>

namespace Nav
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

float Heuristic(const NavMesh &mesh, uint32_t a, uint32_t b)
{
	return (mesh.areas[a].centroid - mesh.areas[b].centroid).Length();
}

} // namespace

PathResult FindPath(const NavMesh &mesh, uint32_t startArea, uint32_t goalArea)
{
	PathResult result;

	if (startArea >= mesh.areas.size() || goalArea >= mesh.areas.size())
		return result;

	if (startArea == goalArea)
	{
		result.found = true;
		result.areaPath.push_back(startArea);
		return result;
	}

	size_t n = mesh.areas.size();
	std::vector<float> gScore(n, std::numeric_limits<float>::max());
	std::vector<int64_t> cameFromArea(n, -1);
	std::vector<int64_t> cameFromPortal(n, -1);
	std::vector<bool> closed(n, false);

	std::priority_queue<OpenEntry, std::vector<OpenEntry>, OpenEntryCompare> open;

	gScore[startArea] = 0.f;
	open.push({Heuristic(mesh, startArea, goalArea), startArea});

	while (!open.empty())
	{
		uint32_t current = open.top().areaId;
		open.pop();

		if (closed[current])
			continue; // stale duplicate entry (lazy-deletion, standard for this priority-queue A* shape)
		closed[current] = true;

		if (current == goalArea)
			break;

		for (uint32_t portalIdx : mesh.areas[current].portalIndices)
		{
			const Portal &portal = mesh.portals[portalIdx];
			uint32_t neighbor = (portal.areaA == current) ? portal.areaB : portal.areaA;

			if ((portal.edgeFlags & crossbot_nav::EDGE_FLAG_ONE_WAY) && portal.areaA != current)
				continue; // one-way edge (e.g. a drop-off) crossed against its only valid direction

			if (closed[neighbor])
				continue;

			float edgeCost =
				(mesh.areas[current].centroid - mesh.areas[neighbor].centroid).Length() * portal.traverseCost;
			float tentativeG = gScore[current] + edgeCost;

			if (tentativeG < gScore[neighbor])
			{
				gScore[neighbor] = tentativeG;
				cameFromArea[neighbor] = current;
				cameFromPortal[neighbor] = portalIdx;
				open.push({tentativeG + Heuristic(mesh, neighbor, goalArea), neighbor});
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

} // namespace Nav
