#pragma once

#include <extdll.h>

#include <cstddef>

#include "../game/ent_handle.h"

struct CBot;

namespace BotAim
{

constexpr size_t kDelaySlots = 256;

constexpr float kMaxDelay = 0.5f;

struct DelaySlot
{
	float time = 0.f;
	float yaw = 0.f;
	float pitch = 0.f;
};

struct State
{
	float yawVel = 0.f;
	float pitchVel = 0.f;

	float brakeYaw = 1.f;
	float brakePitch = 1.f;
	float flickMag = 0.f;

	bool rngSeeded = false;
	unsigned int rngState = 0;
	float driftPhaseYaw = 0.f;
	float driftPhasePitch = 0.f;

	DelaySlot delaySlots[kDelaySlots];
	size_t delayHead = 0;
	size_t delayCount = 0;

	CBotEntHandle gazeTarget;
	float gazeNextCheck = 0.f;
	bool gazeVisible = false;
	float servedDelay = 0.f;

	bool testActive = false;
	bool testSettled = false;
	float testStart = 0.f;
	float testUntil = 0.f;
	float testPeakSpeed = 0.f;
	Vector testAngles;
};

float ViewSpeed(const CBot &bot);

void BeginTest(CBot &bot, float yawDelta, float pitchDelta);

float Delay();

bool GazeEnabled();

float ErrorScale();

float DriftAmplitude();

void ClearDelayQueue(CBot &bot);

void UpdateGaze(CBot &bot);

bool ApplyGaze(CBot &bot);

bool AnglesToTarget(const CBot &bot, edict_t *target, float &yaw, float &pitch);

float QueueAge(const CBot &bot);

}

Vector Bot_UpdateFacing(CBot &bot);
