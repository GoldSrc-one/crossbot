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
	Vector point;
	Vector vel;
};

struct State
{
	float yawVel = 0.f;
	float pitchVel = 0.f;

	float yawPending = 0.f;
	float pitchPending = 0.f;

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

	float servedDelay = 0.f;

	unsigned int predicts = 0;
	bool predictedNow = false;

	float engagementDelay = 0.f;
	bool delayRolled = false;

	bool testActive = false;
	bool testSettled = false;
	bool testRested = false;
	float testStart = 0.f;
	float testUntil = 0.f;
	float testPeakSpeed = 0.f;
	Vector testAngles;
};

float ViewSpeed(const CBot &bot);

void BeginTest(CBot &bot, float yawDelta, float pitchDelta);

float Delay(const CBot &bot);

void RollDelay(CBot &bot);

float ErrorScale();

float DriftAmplitude();

float Quantum();

float Impulse();

void ClearDelayQueue(CBot &bot);

void PushDelaySlot(CBot &bot, const Vector &point, const Vector &vel);

bool ReadDelayed(const CBot &bot, float delay, Vector &point, Vector *vel = nullptr);

bool ReadPredicted(CBot &bot, float delay, Vector &point, Vector *vel = nullptr);

float RandomRoll(CBot &bot);

bool LosAngles(const Vector &from, const Vector &to, float &yaw, float &pitch);

bool AnglesToTarget(const CBot &bot, edict_t *target, float &yaw, float &pitch);

float QueueAge(const CBot &bot);

float PitchLimit();

}

Vector Bot_UpdateFacing(CBot &bot);
