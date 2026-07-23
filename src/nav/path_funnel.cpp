#include "path_funnel.h"

namespace Nav
{

std::vector<Waypoint> BuildWaypointPath(const NavMesh &mesh, const PathResult &path, const Vector &startPoint,
	const Vector &goalPoint)
{
	std::vector<NavPlan::PlanArea> planAreas(mesh.areas.size());
	for (size_t i = 0; i < mesh.areas.size(); ++i)
	{
		const Area &a = mesh.areas[i];
		NavPlan::PlanArea &pa = planAreas[i];
		pa.centroid = a.centroid;
		pa.crouch = (a.areaFlags & crossbot_nav::NAV_AREA_CROUCH) != 0;
		pa.door = (a.areaFlags & crossbot_nav::NAV_AREA_DOOR) != 0;
		pa.doorActivation = a.doorActivation;
		pa.doorActivatorPos = a.doorActivatorPos;
		pa.doorWaitTime = a.doorWaitTime;
		for (uint32_t pi : a.portalIndices)
		{
			if (mesh.portals[pi].edgeType == crossbot_nav::EDGE_DROP)
			{
				pa.hasDropPortal = true;
				break;
			}
		}
	}

	std::vector<NavPlan::PlanPortal> planPortals(mesh.portals.size());
	for (size_t i = 0; i < mesh.portals.size(); ++i)
	{
		const Portal &p = mesh.portals[i];
		NavPlan::PlanPortal &pp = planPortals[i];
		pp.areaA = p.areaA;
		pp.areaB = p.areaB;
		pp.pointA = p.pointA;
		pp.pointB = p.pointB;
		pp.edgeType = p.edgeType;
		pp.buttonPos = p.buttonPos;
		pp.travelTime = p.travelTime;
	}

	return NavPlan::BuildWaypointPlan(planAreas, planPortals, path.areaPath, path.portalPath, startPoint, goalPoint);
}

} // namespace Nav
