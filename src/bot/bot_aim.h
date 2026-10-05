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

struct ViewSlot
{
	float time = 0.f;
	Vector off;
	Vector punch;
};

struct State
{
	float yawVel = 0.f;
	float pitchVel = 0.f;

	float flickMag = 0.f;

	float gainYaw = 1.f;
	float gainPitch = 1.f;

	bool rngSeeded = false;
	unsigned int rngState = 0;

	DelaySlot delaySlots[kDelaySlots];
	size_t delayHead = 0;
	size_t delayCount = 0;

	ViewSlot viewSlots[kDelaySlots];
	size_t viewHead = 0;
	size_t viewCount = 0;
	Vector own;
	Vector perceived;
	Vector perceivedPunch;

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

float GainSpread();

void ClearDelayQueue(CBot &bot);

void ClearViewQueue(CBot &bot);

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
