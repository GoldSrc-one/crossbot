#pragma once

#include "bot.h"

// Drives one bot's movement for the current frame: nav-driven pathing when
// a .nav file is loaded for the current map (see bot_move_nav.h), a
// wall-avoiding random walk otherwise. Both modes route their turning
// through Bot_UpdateFacing so neither snaps viewangles directly (the
// "Smooth steering" note in the Milestone 2 plan).
void Bot_Move(CBot &bot);

// Smooths pEdict->v.angles/v_angle toward bot.desiredAngles at a limited
// turn rate and returns the smoothed angles -- the basis pfnRunPlayerMove
// should use for its forward/side move this frame. Shared by every
// movement mode (wander, nav, future avoidance/combat) so turning
// behavior lives in exactly one place.
Vector Bot_UpdateFacing(CBot &bot);
