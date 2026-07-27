#include "path_astar.h"

#include "astar_graph.h"

namespace Nav
{

PathResult FindPath(const NavMesh &mesh, uint32_t startArea, uint32_t goalArea)
{
	std::vector<NavAStar::AreaNode> areas(mesh.areas.size());
	for (size_t i = 0; i < mesh.areas.size(); ++i)
		areas[i].centroid = mesh.areas[i].centroid;

	std::vector<std::vector<uint32_t>> areaConnIndices(mesh.areas.size());
	for (size_t i = 0; i < mesh.areas.size(); ++i)
		areaConnIndices[i] = mesh.areas[i].connIndices;

	std::vector<NavAStar::Conn> conns(mesh.portals.size());
	for (size_t i = 0; i < mesh.portals.size(); ++i)
	{
		conns[i].areaA = mesh.portals[i].areaA;
		conns[i].areaB = mesh.portals[i].areaB;
		conns[i].pointA = mesh.portals[i].pointA;
		conns[i].pointB = mesh.portals[i].pointB;
		conns[i].oneWay = (mesh.portals[i].connFlags & crossbot_nav::CONN_FLAG_ONE_WAY) != 0;
		conns[i].traverseCost = mesh.portals[i].traverseCost;
	}

	NavAStar::GraphResult graphResult = NavAStar::FindPath(areas, areaConnIndices, conns, startArea, goalArea);

	std::printf("  [FindPath] startArea=%u goalArea=%u found=%d\n", startArea, goalArea, graphResult.found);
	if (graphResult.found)
	{
		std::printf("  [FindPath] areaPath: ");
		for (size_t i = 0; i < graphResult.areaPath.size(); ++i)
		{
			std::printf("%u", graphResult.areaPath[i]);
			if (i + 1 < graphResult.areaPath.size()) std::printf("->");
		}
		std::printf("\n");
		std::printf("  [FindPath] portalPath: ");
		for (size_t i = 0; i < graphResult.portalPath.size(); ++i)
		{
			std::printf("%u", graphResult.portalPath[i]);
			if (i + 1 < graphResult.portalPath.size()) std::printf("->");
		}
		std::printf("\n");
	}

	PathResult result;
	result.found = graphResult.found;
	result.areaPath = std::move(graphResult.areaPath);
	result.portalPath = std::move(graphResult.portalPath);
	return result;
}

}

