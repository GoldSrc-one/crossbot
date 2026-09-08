#pragma once

#include <extdll.h>

#include <cstdint>
#include <vector>

#include "../game/ent_handle.h"
#include "../nav/nav_mesh.h"
#include "bot_goal.h"

struct CBot;

namespace BotStrategic
{

constexpr float kGoalMoveRadius = 32.f;

struct Memo
{
	CBotEntHandle ent;
	float readyTime = 0.f;
};

struct State
{
	CBotEntHandle goal;

	bool arrival = false;
	bool pressed = false;
	float startTime = 0.f;

	std::vector<Memo> memo;
};

bool Enabled();

bool DebugEnabled();

bool IsGoal(edict_t *ent);

bool ValidFor(const CBot &bot, edict_t *ent);

bool ResolvePoint(const Nav::NavMesh &mesh, const Vector &origin, uint32_t &areaId, Vector &point);

void Collect(const CBot &bot, const Nav::NavMesh &mesh, std::vector<BotGoal::Candidate> &out);

void Accept(CBot &bot, const BotGoal::Candidate &cand);

void Forget(CBot &bot);

bool RunArrival(CBot &bot);

void OnRepathFailed(CBot &bot);

void OnBlocked(CBot &bot, const char *what);

void OnRespawn(CBot &bot);

void PrintCandidates(const Nav::NavMesh &mesh);

}
