#pragma once

#include <extdll.h>

#include <vector>

#include "nav_mesh.h"
#include "path_astar.h"

namespace Nav
{

struct Waypoint
{
	Vector point;
	// True when reaching this waypoint from the previous one requires
	// crossing an EDGE_JUMP portal (a proximity-heuristic gap connection,
	// see edge_tagger.cpp's ProbeJumpGaps -- there's no walkable floor the
	// whole way). Bot_MoveNav (src/bot/bot_move_nav.cpp) presses IN_JUMP
	// when it sees this flag instead of just walking at the point.
	bool requiresJump = false;
};

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
//
// EDGE_JUMP portals are never simplified away, even when a straight hull
// trace across the gap happens to report "clear" -- pfnTraceHull only
// tests for solid collision along the line, not for a floor underneath it,
// so a clear trace over a gap doesn't mean the gap is walkable. Every jump
// portal always survives simplification as its own waypoint, tagged
// requiresJump = true.
std::vector<Waypoint> BuildWaypointPath(const NavMesh &mesh, const PathResult &path, const Vector &startPoint,
	const Vector &goalPoint, edict_t *pIgnore);

} // namespace Nav
