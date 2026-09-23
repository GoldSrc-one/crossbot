#pragma once

#include <extdll.h>

#include <vector>

#include "../game/ent_handle.h"

struct CBot;

namespace BotAbility
{

enum Task : unsigned char
{
	TASK_NONE = 0,
	TASK_HEAL,
	TASK_REPAIR,
	TASK_BUILD,
	TASK_STAB,
	TASK_PIPES,
	TASK_PAD,
	TASK_TOUCH,
	TASK_DISARM,
	TASK_DETPACK,
	TASK_DISCARD,
};

constexpr int kTouchSlots = 33;

struct State
{
	Task task = TASK_NONE;
	CBotEntHandle target;
	Vector point;
	float yaw = 0.f;
	int param = 0;
	int phase = 0;
	int turns = 0;
	float start = 0.f;
	float phaseTime = 0.f;
	float nextTry = 0.f;
	float nextCommand = 0.f;
	float nextSelect = 0.f;
	float nextDisguise = 0.f;
	float nextDetonate = 0.f;
	float nextDiscard = 0.f;
	float lastHealth = 0.f;
	float hurtTime = 0.f;
	float fuseEnd = 0.f;
	float stillSince = 0.f;
	float sentryYaw = 0.f;
	float buildLogHold = 0.f;
	bool engaged = false;
	std::vector<Vector> route;
	size_t routeIndex = 0;
	float touchNext[kTouchSlots] = {};
	const char *why = "-";

	unsigned int heals = 0;
	unsigned int repairs = 0;
	unsigned int builds = 0;
	unsigned int buildFails = 0;
	unsigned int disguises = 0;
	unsigned int feigns = 0;
	unsigned int stabs = 0;
	unsigned int pipeBursts = 0;
	unsigned int detonations = 0;
	unsigned int pads = 0;
	unsigned int padFails = 0;
	unsigned int touches = 0;
	unsigned int uncovers = 0;
	unsigned int disarms = 0;
	unsigned int detpacks = 0;
	unsigned int calls = 0;
	unsigned int discards = 0;
};

inline bool Active(const State &st)
{
	return st.task != TASK_NONE;
}

inline bool FiresDuring(const State &st)
{
	return st.task == TASK_DISARM || (st.task == TASK_DETPACK && st.phase >= 3);
}

const char *TaskName(Task task);

void Reset(CBot &bot);

void PrintStatus(const CBot &bot);

}
