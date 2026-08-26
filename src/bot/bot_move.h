#pragma once

#include "../nav/pm_sim.h"
#include "bot.h"

float Bot_NormalizeAngle(float a);

void Bot_DecomposeMove(float viewYawDeg, const Vector &worldDir, float driveLength, float &forwardMove,
	float &sideMove);

byte Bot_FrameMsec();

bool Bot_HasLongJumpModule(edict_t *pEdict);

Vector Bot_YawDir(float yawDeg);

PmSim::PmTraceFn Bot_EngineTraceFn(edict_t *ignore, int hull);

PmSim::PmPushFn Bot_PushFields(int hull);

float Bot_GravityMultiplier(edict_t *pEdict);

float Bot_MaxSpeed(edict_t *pEdict);

PmSim::PmPhysics Bot_Physics(edict_t *pEdict);

void Bot_RunPlayerMove(CBot &bot, const Vector &angles, float fmove, float smove,
	unsigned short buttons, bool allowLongJump = false);

