#pragma once

#include <extdll.h>

#include <cstdint>
#include <vector>

#include "../game/ent_handle.h"
#include "../nav/nav_mesh.h"

struct CBot;

namespace BotGoal
{

enum Tier
{
	GOAL_NONE = 0,
	GOAL_GATHER,
	GOAL_FUN,
	GOAL_STRATEGIC,
};

struct Candidate
{
	Tier tier = GOAL_NONE;
	edict_t *ent = nullptr;
	uint32_t areaId = 0;
	Vector point;
	int payload = 0;
	float worth = 0.f;
};

struct Stop
{
	Tier tier = GOAL_NONE;
	CBotEntHandle ent;
	int areaId = -1;
	Vector point;
	int payload = 0;

	bool camp = false;
	int campIndex = -1;
};

struct State
{
	Stop target;
	Stop stop;
	bool active = false;
	bool onDetour = false;

	Vector targetOrigin;
	float nextRefreshTime = 0.f;

	std::vector<float> toTargetCosts;
	int toTargetArea = -1;

	std::vector<CBotEntHandle> skipped;

	unsigned int rngState = 0;
	bool rngSeeded = false;
};

float Bias();

float DetourBudget();

bool Select(CBot &bot, const Nav::NavMesh &mesh);

void Commit(CBot &bot, const Candidate &cand);

void RefreshStrategic(CBot &bot, const Nav::NavMesh &mesh);

bool OnArrived(CBot &bot);

bool Advance(CBot &bot);

void CommitCampStop(CBot &bot, const Vector &point, int areaId, int campIndex);

void Clear(CBot &bot);

void OnRepathFailed(CBot &bot);

void OnBlocked(CBot &bot, const char *what);

void OnRespawn(CBot &bot);

Tier StopTier(const CBot &bot);

const char *TierName(Tier tier);

}
