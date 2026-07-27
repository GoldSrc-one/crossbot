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
	}

	std::vector<NavPlan::PlanConnection> planPortals(mesh.connections.size());
	for (size_t i = 0; i < mesh.connections.size(); ++i)
	{
		const Connection &c = mesh.connections[i];
		NavPlan::PlanConnection &pp = planPortals[i];
		pp.areaA = c.areaA;
		pp.areaB = c.areaB;
		pp.pointA = c.pointA;
		pp.pointB = c.pointB;
		pp.connType = c.connType;
	}

	return NavPlan::BuildWaypointPlan(planAreas, planPortals, path.areaPath, path.connPath, startPoint, goalPoint);
}

}

