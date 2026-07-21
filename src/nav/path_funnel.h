#pragma once

#include <extdll.h>

#include <vector>

#include "nav_mesh.h"
#include "path_astar.h"

namespace Nav
{

// Turns an A* area/portal corridor into a smoothed waypoint polyline a bot
// can walk toward one point at a time.
//
// Deviates from the plan's original "Simple Stupid Funnel Algorithm"
// design: SSFA needs each portal's two points consistently oriented as
// (left, right) relative to the direction of travel, which navgen's
// PortalRecord doesn't guarantee -- pointA/pointB are just "the two most
// separated points of the shared-face overlap" (see portal_builder.cpp's
// PolygonDiameter). Getting that orientation right blind, with no visual
// nav-mesh debug tool yet to verify it against, is a real correctness
// risk for a subtle geometry algorithm. Instead: build a waypoint list
// from each crossed portal's midpoint (always a safe point inside the
// doorway/gap) plus the final goal, then simplify with a live,
// hull-1-trace-validated string-pull -- skip ahead to the furthest
// waypoint with a clear standing-hull path, using the runtime's real
// collision geometry (which the offline portal data can't see anyway, and
// is a strictly more reliable correctness check than trusting the baked
// points alone). Revisit with true SSFA once there's a way to visually
// verify it.
std::vector<Vector> BuildWaypointPath(const NavMesh &mesh, const PathResult &path, const Vector &startPoint,
	const Vector &goalPoint, edict_t *pIgnore);

} // namespace Nav
