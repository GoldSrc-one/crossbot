#pragma once

#include <extdll.h>

#include <cstdint>
#include <vector>

#include "../game/game_profile.h"
#include "../nav/nav_mesh.h"

struct CBot;

namespace BotDeploy
{

enum SpotKind : unsigned char
{
	SPOT_NONE = 0,
	SPOT_WALL,
	SPOT_CEILING,
	SPOT_GROUND,
};

enum SpotReject : unsigned char
{
	REJECT_NONE = 0,
	REJECT_TYPE,
	REJECT_GATE,
	REJECT_OPEN,
	REJECT_WIDE,
	REJECT_SURFACE,
	REJECT_COVERAGE,
	REJECT_MINED,
	REJECT_MEMO,
};

struct Spot
{
	uint32_t connIndex = 0;
	uint32_t otherAreaId = 0;

	Vector stand;
	Vector mount;
	Vector aimAngles;

	bool duck = false;
	SpotKind kind = SPOT_NONE;
	SpotReject reject = REJECT_NONE;

	float gateWidth = 0.f;
	float opening = 0.f;
};

enum Phase : unsigned char
{
	PHASE_NONE = 0,
	PHASE_APPROACH,
	PHASE_SELECT,
	PHASE_AIM,
	PHASE_FIRE,
	PHASE_VERIFY,
	PHASE_DETONATE,
};

struct Memo
{
	uint32_t connIndex = 0;
	float expireTime = 0.f;
};

struct State
{
	bool active = false;
	Phase phase = PHASE_NONE;

	Spot spot;
	DeployKind kind = DEPLOY_TRIPMINE;

	int prevWeaponId = 0;
	float startTime = 0.f;
	float nextTryTime = 0.f;
	float nextSelectTime = 0.f;
	float firedTime = 0.f;

	int placements = 0;
	int detonations = 0;
	int aborts = 0;

	const char *lastReason = "idle";

	unsigned int rngState = 0;
	bool rngSeeded = false;

	std::vector<Memo> memo;
};

struct Beam
{
	edict_t *ent = NULL;
	Vector start;
	Vector end;
};

bool Enabled();

SpotKind SurfacePin();

void RefreshBeams();

const std::vector<Beam> &Beams();

const Beam *BeamFor(const edict_t *ent);

bool Active(const CBot &bot);

void TryStart(CBot &bot, const Nav::NavMesh &mesh);

bool Run(CBot &bot);

void OnRespawn(CBot &bot);


const char *PhaseName(Phase phase);

bool EvaluateGate(const Nav::NavMesh &mesh, uint32_t areaId, uint32_t connIndex, DeployKind kind,
	bool duck, Spot &out);

bool FindSpot(const CBot &bot, const Nav::NavMesh &mesh, int areaId, bool duck, Spot &out);

bool FindDropSpot(const CBot &bot, const Nav::NavMesh &mesh, int areaId, Spot &out);

const char *SpotKindName(SpotKind kind);

const char *SpotRejectName(SpotReject reject);

void PrintGates(const Nav::NavMesh &mesh);

}
