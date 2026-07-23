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
	// True when this waypoint is the *boarding* point of an EDGE_ELEVATOR
	// crossing (elevator_tagger.h) -- Bot_MoveNav stops and holds position
	// here for elevatorTravelTime once it arrives, rather than treating
	// the point as reached immediately, letting the engine's own
	// ground-entity attachment carry it up/down like a real player
	// standing on the lift. The waypoint immediately after this one is the
	// far landing point, walked to normally once the ride finishes.
	bool requiresElevatorRide = false;
	float elevatorTravelTime = 0.f; // only meaningful when requiresElevatorRide
	// True when either area this waypoint borders has NAV_AREA_CROUCH set
	// (edge_tagger.cpp's ProbeCrouchLinks) -- a hull-3-only passage. Treated
	// as a hard anchor the same way requiresJump is: a standing-hull
	// (kHullHuman) clearance trace between two ordinary rooms on either side
	// of a crouch-only gap can easily read "clear" by sight-lining over or
	// around the low spot entirely, since it's just a line-of-sight sweep,
	// not a check that the *specific* portal-graph route being simplified is
	// what the trace actually followed -- simplification would silently
	// throw away the one waypoint marking where to duck. Bot_MoveNav holds
	// IN_DUCK while this is set, same spirit as requiresJump/IN_JUMP.
	bool requiresCrouch = false;
	// True for a crossing tagged EDGE_DOOR/EDGE_LADDER (edge_tagger.cpp's
	// TagSpecialEdges, converted to a real MergedPortal in main.cpp
	// alongside TagElevators). Both are hard anchors for the same reason
	// requiresJump/requiresCrouch are (see IsHardAnchor) -- simplification's
	// standing-hull sight-line trace can read "clear" without actually
	// having gone through the door or up the ladder. requiresLadder doesn't
	// drive any special movement logic on its own (Bot_MoveNav's ladder
	// climb keys off the engine's own MOVETYPE_FLY, set automatically on
	// touching the ladder trigger); requiresDoor doesn't either (walking
	// onto a touch-to-open door's footprint triggers it the same way a real
	// player would) -- both fields exist purely to pin the waypoint.
	bool requiresDoor = false;
	bool requiresLadder = false;
	// True when this waypoint is the button/panel to press for a
	// button-gated door (edge_tagger.cpp's ResolveButtonForDoor, portal
	// edgeFlags & EDGE_FLAG_NEEDS_BUTTON) -- inserted immediately before the
	// door's own requiresDoor waypoint, the same "boarding point first"
	// expansion EDGE_ELEVATOR already does for buttonPos. Bot_MoveNav holds
	// IN_USE here (not a touch -- CBaseButton only sets up a Touch function
	// when SF_BUTTON_TOUCH_ONLY is set, which none of the tagged buttons
	// have) for buttonWaitTime before continuing to the door.
	bool requiresButtonUse = false;
	float buttonWaitTime = 0.f; // only meaningful when requiresButtonUse
	// Which area this waypoint's point falls in (from the A* path's own
	// areaPath, so it's always right regardless of how a portal's
	// areaA/areaB happen to be labeled). -1 for waypoints that aren't a
	// portal crossing (the initial start point, the final goal point).
	// A jump portal's point is just one area's raw centroid (see
	// requiresJump's comment) -- for a large or oddly-shaped area, a
	// straight line to that centroid can run into a wall the bot has no
	// way around (avoidance is deliberately off while lined up on a jump).
	// So a jump waypoint's real "reached" test is arrival in this area at
	// all (Nav::FindAreaForPoint on the bot's actual origin, matching this
	// id), not distance to the aim point itself -- confirmed live: aiming
	// at the far centroid directly made bots walk dead into walls and
	// freeze, bouncing on jump input with zero horizontal velocity.
	int targetAreaId = -1;
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
// requiresJump = true. EDGE_ELEVATOR portals are handled the same way for
// the same reason (there's no floor across the shaft either): they expand
// into a boarding waypoint (requiresElevatorRide = true, at whichever end
// is nearer the approach direction) immediately followed by the far
// landing point, and both survive simplification untouched.
std::vector<Waypoint> BuildWaypointPath(const NavMesh &mesh, const PathResult &path, const Vector &startPoint,
	const Vector &goalPoint, edict_t *pIgnore);

} // namespace Nav
