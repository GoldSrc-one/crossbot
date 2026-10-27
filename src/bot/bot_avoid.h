#pragma once

#include <extdll.h>

#include <cstddef>
#include <vector>

#include "../game/ent_handle.h"
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
	AVOID_BREAK,
	AVOID_SHOOT
};

constexpr float kDetourClearance = 4.f;
constexpr int kBlockedMines = 4;

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
	float walkOverUntil = 0.f;

	edict_t *guardBeam = nullptr;
	float guardTime = 0.f;
	float guardLogTime = 0.f;

	CBotEntHandle blockedMines[kBlockedMines];
	int blockedMineNext = 0;

	CBotEntHandle jumpBeam;
	float jumpBeamUntil = 0.f;

	CBotEntHandle jumpMine;
	Vector jumpBeamA;
	Vector jumpBeamB;
	float jumpStart = 0.f;
	float jumpSide = 0.f;
	float jumpSpeed = 0.f;
	float jumpGap = 0.f;
	float jumpDist = 0.f;

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
	edict_t *mineTarget = nullptr;
};

Decision Update(CBot &bot, const Nav::NavMesh &mesh, const Vector &pathDir,
	const PmSim::PmPhysics &physics);

const char *KindName(Kind kind);

bool Enabled();

bool BeamJumpActive(const CBot &bot, const edict_t *beamEnt);

bool JumpPendingOn(const CBot &bot, const edict_t *mine);

void ClearJump(CBot &bot);

}
