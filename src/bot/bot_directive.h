#pragma once

#include <extdll.h>

#include "../game/ent_handle.h"

struct CBot;

namespace Nav
{
struct NavMesh;
}

namespace BotDirective
{

enum StepKind : unsigned char
{
	STEP_GO = 0,
	STEP_FOLLOW,
	STEP_HOLD,
	STEP_LOOK,
	STEP_SELECT,
	STEP_DROP,
	STEP_ABILITY,
	STEP_WAIT,
	STEP_FIRE,
	STEP_PLACE,
	STEP_COUNT,
};

enum Stance : unsigned char
{
	STANCE_ANY = 0,
	STANCE_STAND,
	STANCE_CROUCH,
	STANCE_PRONE,
};

struct Target
{
	CBotEntHandle ent;
	Vector point;
	bool valid = false;
};

constexpr int kMaxSteps = 8;
constexpr int kArgLength = 48;
constexpr int kMaxPending = 4;
constexpr int kPendingLength = 128;
constexpr int kNoteLength = 96;

struct Step
{
	StepKind kind = STEP_WAIT;
	Target target;
	Target watch;
	float within = 0.f;
	float seconds = 0.f;
	int count = 0;
	Stance stance = STANCE_ANY;
	char arg[kArgLength] = {};
	char value[kArgLength] = {};
};

struct State
{
	Step steps[kMaxSteps];
	int count = 0;
	int index = 0;
	float expire = 0.f;
	float stepStart = 0.f;
	bool stepBegun = false;
	int phase = 0;
	int fails = 0;
	int mark = 0;
	int weaponId = 0;
	float phaseTime = 0.f;
	float nextCommit = 0.f;
	Vector committed;
	Vector resolved;
	bool hasResolved = false;
	int resolvedArea = -1;
	Vector spot;

	bool meleeOnly = false;
	bool holdFire = false;
	bool noAutoObjective = false;
	float rulesUntil = 0.f;

	Vector alertPoint;
	float alertUntil = 0.f;

	char pending[kMaxPending][kPendingLength] = {};
	int pendingCount = 0;

	char note[kNoteLength] = {};
	float noteTime = 0.f;
	unsigned int done = 0;
	unsigned int failed = 0;
};

const char *StepName(StepKind kind);

bool ParseStep(const char *name, StepKind &out);

bool ParseStance(const char *name, Stance &out);

bool Active(const CBot &bot);

bool Busy(const CBot &bot);

void Set(CBot &bot, const Step *steps, int count, float seconds);

void Release(CBot &bot, const char *why);

void SetRules(CBot &bot, bool meleeOnly, bool holdFire, bool noAutoObjective, float seconds);

void Alert(CBot &bot, const Vector &point, float seconds);

bool Queue(CBot &bot, const char *cmd);

void Think(CBot &bot);

bool Run(CBot &bot, const Nav::NavMesh &mesh);

void OnGoalFailed(CBot &bot, const char *why);

void OnRespawn(CBot &bot);

void OnNewRound(CBot &bot);

bool MeleeOnly(const CBot &bot);

bool HoldFire(const CBot &bot);

bool NoAutoObjective(const CBot &bot);

bool AlertLook(CBot &bot);

void Describe(const CBot &bot, char *out, size_t size);

void PrintStatus(const CBot &bot);

}
