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

enum Kind
{
	KIND_MAP = 0,
	KIND_ATTACK,
	KIND_DELIVER,
	KIND_FETCH,
	KIND_SUPPORT,
	KIND_DEFEND,
	KIND_ROAM,
};

struct Intent
{
	edict_t *ent = nullptr;
	edict_t *aim = nullptr;
	Vector point;
	bool volume = false;
	bool pulse = false;
	unsigned short buttons = 0;
	int kind = KIND_MAP;
	int rank = 0;
};

struct Memo
{
	CBotEntHandle ent;
	float readyTime = 0.f;
};

struct State
{
	CBotEntHandle goal;
	int payload = 0;
	CBotEntHandle aim;
	unsigned short buttons = 0;
	bool pulse = false;

	bool arrival = false;
	bool pressed = false;
	float startTime = 0.f;

	std::vector<Memo> memo;

	float spawnTime = 0.f;
	bool defender = false;
	Vector lastOrigin;
};

bool Enabled();

bool DebugEnabled();

bool IsGoal(edict_t *ent);

bool ValidFor(const CBot &bot, edict_t *ent, int payload);

bool ResolvePoint(const Nav::NavMesh &mesh, const Vector &origin, uint32_t &areaId, Vector &point);

void Intents(const CBot &bot, std::vector<Intent> &out);

void Collect(const CBot &bot, const Nav::NavMesh &mesh, std::vector<BotGoal::Candidate> &out);

void Prioritize(std::vector<BotGoal::Candidate> &cands);

const char *KindName(int payload);

float MoveRadius(int payload);

void Accept(CBot &bot, const BotGoal::Candidate &cand);

void Forget(CBot &bot);

bool RunArrival(CBot &bot);

void OnRepathFailed(CBot &bot);

void OnBlocked(CBot &bot, const char *what);

void OnRespawn(CBot &bot);

void NoteLife(CBot &bot);

int KindOf(int payload);

edict_t *Leader(const CBot &bot, edict_t *site);

void PrintCandidates(const Nav::NavMesh &mesh);

void PrintIntents(const CBot &bot);

}
