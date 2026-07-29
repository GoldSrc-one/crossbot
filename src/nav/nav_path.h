#pragma once

#include <extdll.h>

#include <cstdint>
#include <vector>

#include "nav_mesh.h"
#include "waypoint_plan.h"

namespace Nav
{

using Waypoint = NavPlan::PlanWaypoint;

struct PathResult
{
	bool found = false;
	std::vector<uint32_t> areaPath;
	std::vector<uint32_t> connPath;
};

PathResult FindPath(const NavMesh &mesh, uint32_t startArea, uint32_t goalArea);

std::vector<Waypoint> BuildWaypointPath(const NavMesh &mesh, const PathResult &path, const Vector &startPoint,
	const Vector &goalPoint);

}
