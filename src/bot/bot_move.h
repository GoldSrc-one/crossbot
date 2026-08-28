#pragma once

#include "../nav/pm_sim.h"
#include "bot.h"

float Bot_NormalizeAngle(float a);

void Bot_DecomposeMove(float viewYawDeg, const Vector &worldDir, float driveLength, float &forwardMove,
	float &sideMove);

// Latch the engine's frame time for this server frame. Must be called before
// any bot moves: pfnRunPlayerMove leaves gpGlobals->frametime holding the msec
// it was handed, so after the first bot has moved the global no longer holds
// the frame time -- it holds the previous bot's command length.
void Bot_BeginFrame();

// The frame time latched by Bot_BeginFrame. Bot code must use this instead of
// gpGlobals->frametime for anything integrated per frame.
float Bot_FrameTime();

byte Bot_FrameMsec();

bool Bot_HasLongJumpModule(edict_t *pEdict);

Vector Bot_YawDir(float yawDeg);

PmSim::PmTraceFn Bot_EngineTraceFn(edict_t *ignore, int hull);

PmSim::PmPushFn Bot_PushFields(int hull);

float Bot_GravityMultiplier(edict_t *pEdict);

// How cautious the bot's routing should be, in [0, 1]: 0 plans the fastest route, 1 plans the
// least exposed one. Driven by (1 - combat readiness), scaled by crossbot_careful.
float Bot_Carefulness(const CBot &bot);

float Bot_MaxSpeed(edict_t *pEdict);

PmSim::PmPhysics Bot_Physics(edict_t *pEdict);

void Bot_RunPlayerMove(CBot &bot, const Vector &angles, float fmove, float smove,
	unsigned short buttons, bool allowLongJump = false);

