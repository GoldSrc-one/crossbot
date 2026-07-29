#pragma once

#include "bot.h"

void Bot_Move(CBot &bot);

Vector Bot_UpdateFacing(CBot &bot);

// Splits a world-space direction into the forward/side speeds pfnRunPlayerMove takes, which the
// engine reads along the view axes -- so this is what lets the bot move somewhere other than where
// it is looking. viewYawDeg must be the yaw passed to pfnRunPlayerMove in that same call plus
// punchangle, because PM_CheckParamters builds the movement basis from exactly that.
//
// PM_CheckParamters clamps the pair down to maxspeed before anything reads it, so a driveLength
// well over maxspeed just means "full speed exactly this way", and one below it passes through
// untouched and becomes the actual speed.
void Bot_DecomposeMove(float viewYawDeg, const Vector &worldDir, float driveLength, float &forwardMove,
	float &sideMove);

byte Bot_FrameMsec();

