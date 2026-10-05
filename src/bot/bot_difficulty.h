#pragma once

namespace BotDiff
{

enum Tier
{
	DIFF_EASY = 0,
	DIFF_NORMAL,
	DIFF_HARD,
	DIFF_EXPERT,
	DIFF_COUNT,
};

struct Row
{
	const char *name;
	float aimDelay;
	float turnScale;
	float aimGain;
	float oppoInterval;
	float backPeriod;
	int headPct;
	int predictPct;
	bool combatInterrupt;
	bool camp;
	bool ambushOverride;
	bool pickupPriority;
	bool gaussPolicy;
	bool bunnyHop;
	bool grenadeCook;
	bool backstab;
	bool braceMoving;
	bool wallbang;
	float spreadFit;
	float spreadEarly;
	float spreadLate;
};

constexpr float kDelaySpread = 0.2f;
constexpr float kBackSpread = 1.f / 3.f;

Tier Current();

const Row &Table(Tier tier);

const char *Name(Tier tier);

float Pin(const char *name);

void AimDelay(float &loOut, float &hiOut);

float TurnScale();

float AimGain();

float OppoInterval();

bool BackCheck(float &loOut, float &hiOut);

int HeadPct();

int PredictPct();

bool CombatInterrupt();

bool Camp();

bool AmbushOverride();

bool PickupPriority();

bool GaussPolicy();

bool BunnyHop();

bool GrenadeCook();

bool Backstab();

bool BraceMoving();

bool Wallbang();

float SpreadFit();

void SpreadWait(float &loOut, float &hiOut);

void PrintStatus();

}
