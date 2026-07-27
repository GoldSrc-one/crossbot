#pragma once

#include <extdll.h>

#include <vector>

#include "nav_mesh.h"
#include "path_astar.h"
#include "waypoint_plan.h"

namespace Nav
{

using Waypoint = NavPlan::PlanWaypoint;

std::vector<Waypoint> BuildWaypointPath(const NavMesh &mesh, const PathResult &path, const Vector &startPoint,
	const Vector &goalPoint);

}

