#pragma once

#include <cstddef>
#include <vector>

#include "waypoint_plan.h"

namespace NavFollow
{

struct Config
{

	// The speed the mover tops out at. Not a cap the follower imposes -- it is the velocity the
	// corner brake steers toward, and the reference the pre-steer ramp scales against.
	float maxSpeed = 250.f;

	float cornerRadius = 12.f;
	float finalRadius = 24.f;

	float lookahead = 48.f;

	// How close to a node the corner brake takes over. Zero disables it.
	//
	// Measured on and off: on test23_precise and test2_doorway it is worth nothing either way, a
	// couple of percent that sits inside run-to-run noise. On test24_precise2 it is the whole
	// difference between a clean 340-frame run and dropping into the void for 1128 frames. The
	// pre-steer and the cross-track pull both work on where the mover is aimed; the brake is the
	// only part that acts on where it is actually travelling, which is what a tight corner needs.
	float brakeRadius = NavPlan::kPlanClearance;

	// The band over which the mover is pulled back onto the planned line. Inside pathFreeplay it is
	// left alone to take whatever racing line it likes; from there the correction ramps in, and at
	// pathCorridor it is steering for the line and nothing else.
	//
	// The corridor is kPlanClearance because that is what the planner insets its gates by, so that
	// much room around the line is the one thing it guarantees -- past it the mover is somewhere
	// nobody checked.
	//
	// The freeplay is zero, and measurement says it has to be. A dead band buys a straighter, faster
	// line through open corners -- test2_doorway runs 12% quicker at 8 units of freeplay -- but a
	// ledge only as wide as the corridor has no slack to lend: whatever the band allows is spent
	// before any correction starts, and the mover is already over the edge. test24_precise2 falls at
	// 4 units of freeplay and at 8, and completes only at 0.
	float pathFreeplay = 0.f;
	float pathCorridor = NavPlan::kPlanClearance;

	// How far the aim may be displaced along the outgoing segment to start a turn early.
	//
	// Only safe because the cross-track pull below bounds the result. On its own the cut aims the
	// mover off the planned line with nothing to reel it back, and on a ledge no wider than the
	// corridor that is a fall -- test23_precise and test24_precise2 both went into the void that
	// way. Paired with the pull the two do opposite halves of the job: the cut gets the turn started
	// before the corner, the pull caps how far wide of the line it may end up.
	//
	// Swept at 0, 8, 16 and 24 over both precision maps and test2_doorway. Every map improves
	// monotonically and none falls; 16 takes most of it (doorway 544 -> 499 frames) and 24 adds
	// nothing outside run-to-run noise, so this sits at the planner's own kPlanClearance rather than
	// reaching past it for a gain that cannot be measured.
	float cornerCut = NavPlan::kPlanClearance;
};

struct Decision
{

	bool advance = false;

	// The pre-steered target. Kept because it is what you want to see when working out why a route
	// looks the way it does; the mover should drive on moveDir.
	Vector aimPoint;

	// Unit direction to command, in world space. Equal to the direction to aimPoint outside a
	// corner, and the brake correction inside one.
	Vector moveDir;
};

// vel is the mover's current velocity. Its magnitude sets how early the turn starts -- a mover that
// is barely moving can pivot on the spot and gets no pre-steering, one at full tilt gets all of it
// -- and its direction is what the corner brake corrects.
Decision Steer(const std::vector<NavPlan::PlanWaypoint> &waypoints, size_t wpIndex, const Vector &pos,
	const Vector &vel, const Config &cfg = {});

}

