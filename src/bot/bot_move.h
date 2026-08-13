#pragma once

#include "bot.h"

float Bot_NormalizeAngle(float a);

Vector Bot_UpdateFacing(CBot &bot);

void Bot_DecomposeMove(float viewYawDeg, const Vector &worldDir, float driveLength, float &forwardMove,
	float &sideMove);

byte Bot_FrameMsec();

void Bot_RunPlayerMove(edict_t *pEdict, const Vector &angles, float fmove, float smove,
	unsigned short buttons);

