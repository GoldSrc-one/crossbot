#pragma once

#include <cstddef>
#include <vector>

#include "waypoint_plan.h"

// Engine-independent waypoint-following steering controller, shared between
// the runtime bot (bot_move_nav.cpp's plain ground steering) and
// tools/navgen's offline movement simulation -- the same Steer() decides
// waypoint switching, the pursuit aim point, and the speed cap on both
// sides, so the simulated trajectory drawn in the debug SVG is the same
// control law the live bot runs, just integrated against a model of
// GoldSrc's ground physics instead of the real engine. What this module
// deliberately does NOT cover: engine-side reactions (obstacle feelers,
// avoidance deflection, jump/door/elevator special handling) -- those stay
// in bot_move_nav.cpp, layered on top of the Steer decision.
namespace NavFollow
{

struct Config
{
	// Matches bot_move.cpp's kMaxTurnRatePerSec exactly -- the simulation's
	// fidelity depends on it.
	float turnRateDegPerSec = 300.f;
	float runSpeed = 250.f;
	// Intermediate funnel corners are walked THROUGH (tight radius, or
	// passing the corner's perpendicular along the current segment); only
	// the final waypoint keeps a loose arrival radius. See bot_move_nav.cpp's
	// kWaypointCornerRadius comment for the confirmed-live reasoning.
	float cornerRadius = 12.f;
	float finalRadius = 24.f;
	// Pure-pursuit line following: aim this far ahead of the bot's own
	// projection onto the current planned segment (tighter when the
	// waypoint carries a careful maxSpeed), so drift and corner cuts get
	// pulled back onto the line instead of compounding.
	float lookahead = 48.f;
	float lookaheadCareful = 24.f;
	// Turn-then-run speed scaling thresholds (see bot_move_nav.cpp's
	// kTurnInPlaceErrorDeg comment).
	float turnInPlaceErrorDeg = 60.f;
	float turnSlowdownErrorDeg = 20.f;
	float turnSlowdownFactor = 0.3f;
};

struct Decision
{
	// True: the current waypoint counts as reached -- advance the index
	// (and ignore the rest of this decision).
	bool advance = false;
	// Point to steer toward (pure-pursuit aim on the planned segment).
	Vector aimPoint;
	// Forward speed BEFORE heading-error scaling (i.e. min of runSpeed and
	// the waypoint's planned maxSpeed) -- the caller applies
	// ScaleSpeedByHeadingError once it knows this tick's smoothed heading.
	float baseSpeed = 0.f;
};

// One steering decision for the waypoint at wpIndex, from position `pos`.
// Only meaningful for plain ground-walk waypoints -- special waypoints
// (jump commit, elevator/button/door holds, ladder climbs) are the
// caller's own branches.
Decision Steer(const std::vector<NavPlan::PlanWaypoint> &waypoints, size_t wpIndex, const Vector &pos,
	const Config &cfg = {});

// Heading-error speed scaling: full speed only when actually pointed the
// right way, creep while lining up, turn in place when badly misaligned.
// headingErrorDeg is |normalized(currentYaw - desiredYaw)|.
float ScaleSpeedByHeadingError(float baseSpeed, float headingErrorDeg, const Config &cfg = {});

// Offline movement simulation (navgen's --dump-svg): integrates Steer()
// against a model of GoldSrc's ground physics (PM_Friction/PM_Accelerate
// with the stock sv_friction 4 / sv_accelerate 10 / sv_stopspeed 100, and
// the turn-rate-limited facing bot_move.cpp applies), producing the
// trajectory a live bot would actually drive -- momentum, corner
// overshoot, convergence lag and all. Special waypoints are followed as
// plain points (the simulation has no doors/lifts to wait on); z is
// carried along the planned segments purely so a fall-check against
// floor-supported footprints can compare heights.
struct SimResult
{
	std::vector<Vector> points; // one per tick, starting at `start`
	bool finished = false;      // reached the final waypoint within maxTime
};
SimResult Simulate(const std::vector<NavPlan::PlanWaypoint> &waypoints, const Vector &start,
	float dt = 1.f / 60.f, float maxTime = 120.f, const Config &cfg = {});

} // namespace NavFollow
