#pragma once

#include <extdll.h>

#include <vector>

#include "nav_mesh.h"
#include "path_astar.h"
#include "waypoint_plan.h"

namespace Nav
{

// The runtime waypoint IS the shared planner's waypoint type -- see
// waypoint_plan.h for the field-by-field semantics (jump/ladder/elevator/
// door/crouch flags, plan-time maxSpeed, targetAreaId). Sharing the actual
// type (not a mirrored copy) is what lets bot_move_nav.cpp hand its
// waypoint list straight to the shared steering controller
// (path_follower.h) that tools/navgen's offline movement simulation also
// runs.
using Waypoint = NavPlan::PlanWaypoint;

// Turns an A* area/portal corridor into a smoothed waypoint polyline via
// the shared NavPlan::BuildWaypointPlan (waypoint_plan.cpp) -- this is
// just the NavMesh -> PlanArea/PlanPortal adapter. The whole plan step is
// pure geometry with no engine traces at all: the previous design
// string-pulled with live pfnTraceHull sight-line checks, and that was the
// root cause of a real class of failures (a hull trace only tests for
// solids along the line, never for floor *under* it, so a "clear" trace
// across the open air above a pit collapsed a careful walkway route into
// a straight line over the pit -- confirmed live on test23_precise/
// test24_precise2). Unexpected runtime obstacles are bot_move_nav.cpp's
// job (its feeler trace + FindClearDirection dodge), not the path's.
std::vector<Waypoint> BuildWaypointPath(const NavMesh &mesh, const PathResult &path, const Vector &startPoint,
	const Vector &goalPoint);

} // namespace Nav
