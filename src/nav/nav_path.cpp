#include "nav_path.h"

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
	{
		for (uint32_t ci : mesh.areas[i].connIndices)
		{
			if (ci < mesh.conns.size() && mesh.conns[ci].connType == crossbot_nav::CONN_LADDER)
				continue;
			areaConnIndices[i].push_back(ci);
		}
	}

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

}

