#pragma once

#include "bot.h"

// Drives one bot's movement for current frame: nav-driven pathing when .nav file loaded (see bot_move_nav.h), wall-avoiding random walk otherwise. Both modes route turning through Bot_UpdateFacing so neither snaps viewangles directly ("Smooth steering" note in Milestone 2 plan).
void Bot_Move(CBot &bot);

// Smooths pEdict->v.angles/v_angle toward bot.desiredAngles at limited turn rate and returns smoothed angles -- basis for pfnRunPlayerMove's forward/side move this frame. Shared by all movement modes (wander, nav, future avoidance/combat) so turning behavior lives in exactly one place.
Vector Bot_UpdateFacing(CBot &bot);

// msec for every pfnRunPlayerMove call: real elapsed time since engine's last server frame in ms, clamped to sane range. Hardcoded msec (what this replaced) silently freezes movement -- GoldSrc's timing rejects fake client claiming more elapsed time per move than server frame actually took; every reference bot surveyed (FoxBot, jk_botti, Parabot) derives it from real elapsed time, not hardcoded.
byte Bot_FrameMsec();
