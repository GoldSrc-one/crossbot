#pragma once

#include <extdll.h>

#include <cstddef>
#include <vector>

#include "../nav/pm_sim.h"

struct CBot;

namespace Nav
{
	struct NavMesh;
}

namespace BotAvoid
{

enum Kind
{
	AVOID_NONE = 0,
	AVOID_DUCK,
	AVOID_JUMP,
	AVOID_SIDE,
	AVOID_FOLLOW,
	AVOID_BLOCKED,
	AVOID_BREAK
};

constexpr float kDetourClearance = 4.f;

struct State
{
	float duckUntil = 0.f;
	float jumpCooldownUntil = 0.f;
	bool jumpArmed = false;

	size_t detourEnd = 0;
	int detourHit = 0;
	float detourExpire = 0.f;
	float nextSearchTime = 0.f;

	float blockedUntil = 0.f;

	Kind lastKind = AVOID_NONE;
	char lastHit[32] = {};
	float lastGap = 0.f;
	float lastTop = 0.f;
};

struct Decision
{
	bool duck = false;
	bool jump = false;
	bool suppressPlanCrouch = false;
	bool slowApproach = false;

	std::vector<Vector> detourPoints;
	size_t detourReplaceUntil = 0;
	bool searchFailed = false;
	bool hitRecedes = false;

	edict_t *breakTarget = nullptr;
};

Decision Update(CBot &bot, const Nav::NavMesh &mesh, const Vector &pathDir,
	const PmSim::PmPhysics &physics);

const char *KindName(Kind kind);

bool Enabled();

}
