#include "nav_path.h"

#include "astar_graph.h"

namespace Nav
{

PathResult FindPath(const NavMesh &mesh, uint32_t startArea, uint32_t goalArea)
{
	std::vector<NavAStar::AreaNode> areas(mesh.areas.size());
	std::vector<std::vector<uint32_t>> areaConnIndices(mesh.areas.size());
	for (size_t i = 0; i < mesh.areas.size(); ++i)
	{
		areas[i].centroid = mesh.areas[i].centroid;
		areaConnIndices[i] = mesh.areas[i].connIndices;
	}

	std::vector<NavAStar::Connection> conns(mesh.conns.size());
	for (size_t i = 0; i < mesh.conns.size(); ++i)
	{
		const Connection &c = mesh.conns[i];
		NavAStar::Connection &gc = conns[i];
		gc.areaA = c.areaA;
		gc.areaB = c.areaB;
		gc.crossing = NavPlan::ConnectionCrossing(mesh.planAreas, mesh.planConns[i]);
		gc.oneWay = (c.connFlags & crossbot_nav::CONN_FLAG_ONE_WAY) != 0;
		gc.costSeconds = c.costSeconds;
	}

	NavAStar::GraphResult graphResult = NavAStar::FindPath(areas, areaConnIndices, conns, startArea, goalArea);

	PathResult result;
	result.found = graphResult.found;
	result.areaPath = std::move(graphResult.areaPath);
	result.connPath = std::move(graphResult.connPath);
	return result;
}

std::vector<Waypoint> BuildWaypointPath(const NavMesh &mesh, const PathResult &path, const Vector &startPoint,
	const Vector &goalPoint, const NavPlan::PlanTraceContext *traces, NavPlan::PlanStats *stats)
{
	return NavPlan::BuildWaypointPlan(mesh.planAreas, mesh.planConns, path.areaPath, path.connPath,
		startPoint, goalPoint, traces, stats);
}

}

