#pragma once

#include <cstddef>
#include <vector>

#include "waypoint_plan.h"

// Engine-independent waypoint-following steering controller, shared between runtime bot (bot_move_nav.cpp plain ground steering) and tools/navgen offline movement simulation. Same Steer() decides waypoint switching, pursuit aim, and speed cap on both sides -- simulated trajectory in debug SVG matches live bot control law, just integrated against GoldSrc ground physics model instead of real engine. Does NOT cover engine-side reactions (obstacle feelers, avoidance deflection, jump/door/elevator specials) -- those stay in bot_move_nav.cpp layered on top.
namespace NavFollow
{

struct Config
{
	// Matches bot_move.cpp's kMaxTurnRatePerSec exactly -- simulation fidelity depends on it.
	float turnRateDegPerSec = 300.f;
	float runSpeed = 250.f;
	// Intermediate funnel corners walked THROUGH (tight radius or passing corner's perpendicular along current segment); only final waypoint keeps loose arrival radius. See bot_move_nav.cpp kWaypointCornerRadius for live reasoning.
	float cornerRadius = 12.f;
	float finalRadius = 24.f;
	// Pure-pursuit line following: aim this far ahead of bot's projection on current segment (tighter when waypoint carries careful maxSpeed), so drift and corner cuts get pulled back onto line instead of compounding.
	float lookahead = 48.f;
	float lookaheadCareful = 24.f;
	// Turn-then-run speed scaling thresholds (see bot_move_nav.cpp kTurnInPlaceErrorDeg comment).
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

// Offline movement simulation (navgen --dump-svg): integrates Steer() against GoldSrc ground physics model (PM_Friction/PM_Accelerate with stock sv_friction 4 / sv_accelerate 10 / sv_stopspeed 100, and turn-rate-limited facing from bot_move.cpp), producing trajectory live bot would drive -- momentum, corner overshoot, convergence lag. Special waypoints followed as plain points (simulation has no doors/lifts to wait on); z carried along planned segments for fall-check against floor-supported footprints.
struct SimResult
{
	std::vector<Vector> points; // one per tick, starting at `start`
	bool finished = false;      // reached the final waypoint within maxTime
};
SimResult Simulate(const std::vector<NavPlan::PlanWaypoint> &waypoints, const Vector &start,
	float dt = 1.f / 60.f, float maxTime = 120.f, const Config &cfg = {});

} // namespace NavFollow
