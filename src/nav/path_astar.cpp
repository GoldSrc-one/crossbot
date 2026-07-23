#include "path_astar.h"

#include "astar_graph.h"

namespace Nav
{

PathResult FindPath(const NavMesh &mesh, uint32_t startArea, uint32_t goalArea)
{
	std::vector<NavAStar::AreaNode> areas(mesh.areas.size());
	for (size_t i = 0; i < mesh.areas.size(); ++i)
		areas[i].centroid = mesh.areas[i].centroid;

	std::vector<std::vector<uint32_t>> areaPortalIndices(mesh.areas.size());
	for (size_t i = 0; i < mesh.areas.size(); ++i)
		areaPortalIndices[i] = mesh.areas[i].portalIndices;

	std::vector<NavAStar::PortalEdge> portals(mesh.portals.size());
	for (size_t i = 0; i < mesh.portals.size(); ++i)
	{
		portals[i].areaA = mesh.portals[i].areaA;
		portals[i].areaB = mesh.portals[i].areaB;
		portals[i].oneWay = (mesh.portals[i].edgeFlags & crossbot_nav::EDGE_FLAG_ONE_WAY) != 0;
		portals[i].traverseCost = mesh.portals[i].traverseCost;
	}

	NavAStar::GraphResult graphResult = NavAStar::FindPath(areas, areaPortalIndices, portals, startArea, goalArea);

	PathResult result;
	result.found = graphResult.found;
	result.areaPath = std::move(graphResult.areaPath);
	result.portalPath = std::move(graphResult.portalPath);
	return result;
}

} // namespace Nav
