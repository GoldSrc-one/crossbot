#pragma once

#include "bot.h"

// Wraps an angle into (-180, 180].
float Bot_NormalizeAngle(float a);

Vector Bot_UpdateFacing(CBot &bot);

void Bot_DecomposeMove(float viewYawDeg, const Vector &worldDir, float driveLength, float &forwardMove,
	float &sideMove);

byte Bot_FrameMsec();

