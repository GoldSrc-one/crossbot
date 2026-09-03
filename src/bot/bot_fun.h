#pragma once

#include <extdll.h>

#include <vector>

#include "../game/ent_handle.h"
#include "../nav/nav_mesh.h"
#include "bot_goal.h"

struct CBot;

namespace BotFun
{

enum Kind
{
	FUN_SWITCH = 0,
	FUN_TANK,
	FUN_TRAIN,
};

enum TrainPhase
{
	TRAIN_APPROACH = 0,
	TRAIN_WAIT,
	TRAIN_MOUNT,
	TRAIN_DRIVE,
	TRAIN_RELEASE,
	TRAIN_RIDE,
};

enum Action
{
	FUN_ACT_USE = 0,
	FUN_ACT_TOUCH,
	FUN_ACT_SHOOT,
};

struct Memo
{
	CBotEntHandle ent;
	float readyTime = 0.f;
	float abandonReadiness = -1.f;
};

struct State
{
	CBotEntHandle goal;
	CBotEntHandle tank;
	Kind kind = FUN_SWITCH;
	Action action = FUN_ACT_USE;

	bool arrival = false;
	bool pressed = false;
	float startTime = 0.f;
	float holdUntil = 0.f;
	int startToggle = -1;

	bool mounted = false;
	float mountTime = 0.f;
	float lastSeenEnemy = 0.f;
	int shots = 0;

	CBotEntHandle train;
	bool aboard = false;
	bool dismounting = false;
	TrainPhase trainPhase = TRAIN_APPROACH;
	float phaseStart = 0.f;
	float boardArc = 0.f;
	float boardTime = 0.f;
	Vector boardDeck;
	float lastEta = 0.f;
	float dismountStart = 0.f;
	float offTrainSince = 0.f;

	std::vector<Memo> memo;

	unsigned int rngState = 0;
	bool rngSeeded = false;
};

inline int PackPayload(Kind kind, Action action)
{
	return static_cast<int>(kind) * 16 + static_cast<int>(action);
}

inline Kind PayloadKind(int payload)
{
	return static_cast<Kind>(payload / 16);
}

inline Action PayloadAction(int payload)
{
	return static_cast<Action>(payload % 16);
}

bool Enabled();

bool DebugEnabled();

void Collect(const CBot &bot, const Nav::NavMesh &mesh, std::vector<BotGoal::Candidate> &out);

void Prioritize(std::vector<BotGoal::Candidate> &cands);

void NoteDeath(const CBot &bot);

void Accept(CBot &bot, const BotGoal::Candidate &cand);

bool ValidFor(const CBot &bot, edict_t *ent, int payload);

bool Valid(const CBot &bot);

void Forget(CBot &bot);

bool RunArrival(CBot &bot);

void OnArrived(CBot &bot);

void OnRepathFailed(CBot &bot);

void OnBlocked(CBot &bot, const char *what);

void OnRespawn(CBot &bot);

bool Run(CBot &bot);

bool Mounted(const CBot &bot);

bool Aboard(const CBot &bot);

bool Pressing(const CBot &bot);

void ResetMap();

void PrintCandidates(const Nav::NavMesh &mesh);

const char *KindName(Kind kind);

const char *ActionName(Action action);

}
