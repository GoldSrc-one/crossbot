#pragma once

#include <extdll.h>

struct CBot;

namespace BotAim
{

struct State
{
	float yawVel = 0.f;
	float pitchVel = 0.f;

	bool testActive = false;
	bool testSettled = false;
	float testStart = 0.f;
	float testUntil = 0.f;
	float testPeakSpeed = 0.f;
	Vector testAngles;
};

float ViewSpeed(const CBot &bot);

void BeginTest(CBot &bot, float yawDelta, float pitchDelta);

}

Vector Bot_UpdateFacing(CBot &bot);
