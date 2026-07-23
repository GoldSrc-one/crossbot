#pragma once

#include <extdll.h>

#include <vector>

#include "../nav/path_funnel.h"

// A single bot's state.
struct CBot
{
	bool inUse = false;
	edict_t *pEdict = nullptr;
	char name[32] = {};

	// Wander fallback (Milestone 1) -- used whenever Nav::IsLoaded() is
	// false, i.e. the current map has no baked .nav file.
	Vector moveDir = Vector(1.f, 0.f, 0.f);
	float nextDirChangeTime = 0.f;

	// Nav path-following state (Milestone 2c), additive to the wander
	// fields above, not a replacement -- see src/bot/bot_move_nav.cpp.
	bool hasGoal = false;
	Vector goalPoint;
	std::vector<Nav::Waypoint> waypoints;
	size_t waypointIndex = 0;
	float nextRepathTime = 0.f;
	// Set once a bot arrives at an elevator's boarding waypoint (see
	// Nav::Waypoint::requiresElevatorRide); while gpGlobals->time is before
	// this, Bot_MoveNav holds the bot still instead of advancing, letting
	// the engine's own ground-entity attachment carry it up/down. 0 means
	// "not currently waiting for a ride."
	float elevatorReadyTime = 0.f;
	// Same idea as elevatorReadyTime, for a waypoint tagged requiresDoor
	// (Nav::Waypoint, path_funnel.cpp): a func_door/func_door_rotating is
	// still SOLID_BSP right up until touched, and stays solid (swinging or
	// sliding) for a moment after that -- confirmed live on test9_door: a
	// bot that just kept walking straight at the door the instant it got
	// close ended up contesting the moving brush's own collision over and
	// over, deflecting it back out of the doorway in a loop that never
	// actually got through. Once close enough to have made contact (which
	// is what triggers CBaseDoor's touch-to-open in the first place), the
	// bot instead holds still for a fixed clearance window before finishing
	// the approach, giving the door time to finish opening out of the way.
	// 0 means "not currently waiting on a door."
	float doorWaitUntil = 0.f;
	// Set the frame a jump waypoint actually presses IN_JUMP; a jump
	// waypoint can only be marked reached once this is true and the bot is
	// grounded again, since the landing point is often already within the
	// generic reached-distance of the takeoff point (that's what makes it a
	// short hop) -- without this gate the waypoint would complete before
	// the bot ever left the ground. See Bot_MoveNav.
	bool jumpAttempted = false;
	// One-shot latch for Bot_MoveNav's fall telemetry (a qconsole log line
	// the first frame of an unplanned airborne drop, with the exact origin
	// and the waypoint being walked toward) -- reset on landing so each
	// distinct fall logs exactly once, not every airborne frame.
	bool fallTelemetryLogged = false;
	// Per-tick origin recording while a goal is active, dumped to
	// valve/crossbot_data/nav_trace.txt when the goal ends (reached OR
	// dropped) -- navgen's --dump-svg --trace overlays it on the debug SVG
	// next to the planned line and the momentum simulation, so plan vs.
	// model vs. reality can be compared visually point by point. Cleared on
	// each new goal assignment (BotManager::SetGoalForAllBots).
	std::vector<Vector> traceLog;
	// Set the first frame Bot_MoveNav's dynamic-obstacle trace finds
	// something in the way that the baked nav mesh doesn't know about (a
	// player-built structure, a monster, any solid entity placed at
	// runtime after navgen ran) -- 0 means "not currently blocked". Used
	// to force an early repath if a dodge doesn't clear it soon enough.
	float obstacleBlockedSince = 0.f;
	// Stall detection for the safety repath (Bot_MoveNav): the last time the
	// bot completed a waypoint (or got a fresh path). The old design
	// repathed on a blind periodic timer even while the bot was hitting
	// every waypoint on schedule, which rebuilt the waypoint list
	// mid-segment -- moving the pursuit line out from under the bot right
	// as it threaded precise geometry (confirmed live on test24_precise2:
	// the 10 s timer fired exactly as the bot reached a razor-thin beam
	// junction, and the offline momentum simulation -- which has no repath
	// -- drove the identical plan cleanly). Now a repath outside the forced
	// obstacle case only happens after this has gone stale for the interval.
	float lastProgressTime = 0.f;
	int lastSeenWaypointIndex = 0;
	// Last origin seen by Bot_MoveNav, used to detect a discontinuous jump
	// (respawn, or -- once Milestone 2h's trigger_teleport lands -- a
	// teleporter) and invalidate a now-meaningless stale path instead of
	// trying to walk toward waypoints computed for a position the bot is
	// no longer anywhere near.
	Vector lastOrigin;
	bool hasLastOrigin = false;

	// Diagnostics (crossbot_nav_status / NavDebug, see src/nav/nav_debug.h):
	// which nav area the bot's real origin resolves to right now, refreshed
	// every Bot_MoveNav tick regardless of whether it currently has a goal.
	// -1 when off the mesh entirely (FindAreaForPoint only returns nullptr
	// when the mesh has zero areas, which can't happen once Nav::IsLoaded()
	// is true, but stays a defensive sentinel rather than an assumption).
	int currentAreaId = -1;
	// The A* corridor's own area sequence for the path currently being
	// followed (Nav::PathResult::areaPath, refreshed on every Repath --
	// including mid-route safety repaths, since those replace the corridor
	// from the bot's current position on) versus the areas the bot has
	// actually passed through since the goal was assigned (appended in
	// Bot_MoveNav whenever currentAreaId changes). Comparing the two after a
	// stall shows exactly where the bot left the intended corridor instead
	// of guessing from raw coordinates.
	std::vector<uint32_t> plannedAreaPath;
	std::vector<uint32_t> visitedAreaPath;
	// Set once, the first time a fresh goal gets a path (not touched by
	// later mid-route safety repaths, so the deadline below reflects the
	// original goal assignment, not a repeatedly-pushed-back moving target).
	float pathStartTime = 0.f;
	// A straight-line-distance/speed estimate for the path found at
	// pathStartTime, with a safety margin for turning/jumping/avoidance
	// overhead a straight-line estimate can't see, plus any elevator ride
	// time (spent motionless, not covered by the speed term at all). Once
	// gpGlobals->time passes this while still short of the goal, the bot is
	// running later than a normal traversal should -- see PrintNavStatus's
	// OVERDUE flag.
	float estimatedArrivalTime = 0.f;

	// Desired facing, written by whichever movement mode is active this
	// frame (wander or nav); Bot_UpdateFacing (bot_move.cpp) smooths the
	// entity's actual angles toward this once per frame, shared by every
	// mode so none of them snap viewangles directly. See the "Smooth
	// steering" note in the Milestone 2 plan.
	Vector desiredAngles;
};
