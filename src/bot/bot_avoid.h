#pragma once

#include <extdll.h>

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
	AVOID_JUMP
};

struct State
{
	float duckUntil = 0.f;
	float jumpCooldownUntil = 0.f;
	bool jumpArmed = false;

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
};

Decision Update(CBot &bot, const Nav::NavMesh &mesh, const Vector &pathDir,
	const PmSim::PmPhysics &physics);

const char *KindName(Kind kind);

bool Enabled();

}
