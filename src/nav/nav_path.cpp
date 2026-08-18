#include "nav_path.h"

#include <queue>

#include "astar_graph.h"

namespace Nav
{

namespace
{

void BuildGraph(const NavMesh &mesh, std::vector<std::vector<uint32_t>> &areaConnIndices,
	std::vector<NavAStar::Connection> &conns)
{
	areaConnIndices.assign(mesh.areas.size(), std::vector<uint32_t>());
	for (size_t i = 0; i < mesh.areas.size(); ++i)
		areaConnIndices[i] = mesh.areas[i].connIndices;

	conns.assign(mesh.conns.size(), NavAStar::Connection());
	for (size_t i = 0; i < mesh.conns.size(); ++i)
	{
		const Connection &c = mesh.conns[i];
		NavAStar::Connection &gc = conns[i];
		gc.areaA = c.areaA;
		gc.areaB = c.areaB;
		gc.crossing = NavPlan::ConnectionCrossing(mesh.planAreas, mesh.planConns[i]);
		gc.oneWay = (c.connFlags & crossbot_nav::CONN_FLAG_ONE_WAY) != 0;
		gc.costSeconds = c.costSeconds + c.runtimePenalty;
	}
}

bool GateAllowsCrossing(const NavMesh &mesh, uint32_t connIdx, uint32_t fromId, uint32_t toId)
{
	if (toId >= mesh.planAreas.size() || connIdx >= mesh.planConns.size())
		return true;

	const NavPlan::PlanArea &toArea = mesh.planAreas[toId];
	if (toArea.activation == crossbot_nav::ACT_NONE)
		return true;

	uint8_t connType = mesh.planConns[connIdx].connType;
	if (connType != crossbot_nav::CONN_WALK && connType != crossbot_nav::CONN_ELEVATOR)
		return false;

	const NavPlan::PlanDetour *detour = NavPlan::FindDetour(toArea, fromId);
	return detour != nullptr && !detour->blocked && !detour->points.empty();
}

}

PlannedPath PlanPath(const NavMesh &mesh, uint32_t startArea, uint32_t goalArea,
	const Vector &startPoint, const Vector &goalPoint, const NavPlan::PlanTraceContext *traces,
	NavPlan::PlanStats *stats)
{
	std::vector<std::vector<uint32_t>> areaConnIndices;
	std::vector<NavAStar::Connection> graphConns;
	BuildGraph(mesh, areaConnIndices, graphConns);

	return NavPlan::PlanRoute(areaConnIndices, graphConns, mesh.planAreas, mesh.planConns, startArea,
		goalArea, startPoint, goalPoint, traces, stats);
}

namespace
{

void AreaCostsImpl(const NavMesh &mesh, uint32_t rootArea, bool reverse, std::vector<float> &out)
{
	size_t n = mesh.areas.size();
	out.assign(n, kUnreachableSeconds);
	if (rootArea >= n)
		return;

	std::vector<bool> settled(n, false);
	out[rootArea] = 0.f;

	typedef std::pair<float, uint32_t> Entry;
	std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open;
	open.push(Entry(0.f, rootArea));

	while (!open.empty())
	{
		uint32_t current = open.top().second;
		float currentDist = open.top().first;
		open.pop();

		if (settled[current])
			continue;
		settled[current] = true;

		for (uint32_t connIdx : mesh.areas[current].connIndices)
		{
			if (connIdx >= mesh.conns.size())
				continue;

			const Connection &c = mesh.conns[connIdx];
			uint32_t neighbor = (c.areaA == current) ? c.areaB : c.areaA;
			uint32_t from = reverse ? neighbor : current;
			uint32_t to = reverse ? current : neighbor;

			if ((c.connFlags & crossbot_nav::CONN_FLAG_ONE_WAY) != 0 && c.areaA != from)
				continue;
			if (neighbor >= n || settled[neighbor])
				continue;
			if (!GateAllowsCrossing(mesh, connIdx, from, to))
				continue;

			float step = currentDist + c.travelSeconds + c.costSeconds + c.runtimePenalty;
			if (step >= out[neighbor])
				continue;

			out[neighbor] = step;
			open.push(Entry(step, neighbor));
		}
	}
}

}

void AreaCosts(const NavMesh &mesh, uint32_t startArea, std::vector<float> &out)
{
	AreaCostsImpl(mesh, startArea, false, out);
}

void AreaCostsToGoal(const NavMesh &mesh, uint32_t goalArea, std::vector<float> &out)
{
	AreaCostsImpl(mesh, goalArea, true, out);
}

}

