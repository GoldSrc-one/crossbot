#pragma once

#include <extdll.h>

#include <vector>

#include "nav_mesh.h"
#include "path_astar.h"
#include "waypoint_plan.h"

namespace Nav
{

// The runtime waypoint IS the shared planner's type (NavPlan::PlanWaypoint) -- see waypoint_plan.h for field semantics (jump/ladder/elevator/door/crouch flags, plan-time maxSpeed, targetAreaId). Sharing the actual type (not a mirrored copy) lets bot_move_nav.cpp hand its waypoint list straight to shared steering controller (path_follower.h) that tools/navgen's offline simulation also runs.
using Waypoint = NavPlan::PlanWaypoint;

// Turns A* area/portal corridor into smoothed waypoint polyline via shared NavPlan::BuildWaypointPlan -- just the NavMesh -> PlanArea/PlanPortal adapter. Whole plan step is pure geometry, no engine traces: previous design string-pulled with live pfnTraceHull sight-line checks, which was root cause of real failures (hull trace only tests solids along line, never floor *under* it, so "clear" trace across open air above pit collapsed careful walkway route into straight line over pit -- confirmed live on test23_precise/test24_precise2). Unexpected runtime obstacles handled in bot_move_nav.cpp (feeler trace + FindClearDirection dodge), not the path.
std::vector<Waypoint> BuildWaypointPath(const NavMesh &mesh, const PathResult &path, const Vector &startPoint,
	const Vector &goalPoint);

} // namespace Nav
