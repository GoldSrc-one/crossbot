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
	// Last origin seen by Bot_MoveNav, used to detect a discontinuous jump
	// (respawn, or -- once Milestone 2h's trigger_teleport lands -- a
	// teleporter) and invalidate a now-meaningless stale path instead of
	// trying to walk toward waypoints computed for a position the bot is
	// no longer anywhere near.
	Vector lastOrigin;
	bool hasLastOrigin = false;

	// Desired facing, written by whichever movement mode is active this
	// frame (wander or nav); Bot_UpdateFacing (bot_move.cpp) smooths the
	// entity's actual angles toward this once per frame, shared by every
	// mode so none of them snap viewangles directly. See the "Smooth
	// steering" note in the Milestone 2 plan.
	Vector desiredAngles;
};
