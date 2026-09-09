#include <extdll.h>
#include <meta_api.h>

#include <cstdio>

#include "bot_difficulty.h"

namespace BotDiff
{

namespace
{
	const Row kTable[DIFF_COUNT] = {
		{ "easy",   0.40f, 2.00f, 0.85f, 0.50f, 0.000f,  0,  0, true,  true,  true,  false, false, false },
		{ "normal", 0.30f, 1.00f, 1.00f, 0.25f, 0.000f,  0,  0, true,  true,  true,  false, false, false },
		{ "hard",   0.25f, 0.60f, 1.10f, 0.25f, 1.500f, 60, 70, true,  true,  true,  false, false, false },
		{ "expert", 0.20f, 0.35f, 1.20f, 0.25f, 1.125f, 90, 90, false, false, false, true,  true,  true  },
	};

	bool Boolean(const char *name, bool tiered)
	{
		float pin = Pin(name);
		return pin >= 0.f ? pin != 0.f : tiered;
	}

	float Scalar(const char *name, float tiered)
	{
		float pin = Pin(name);
		return pin >= 0.f ? pin : tiered;
	}
}

Tier Current()
{
	cvar_t *cvar = g_engfuncs.pfnCVarGetPointer("crossbot_difficulty");
	if (!cvar)
		return DIFF_NORMAL;

	int level = static_cast<int>(cvar->value);
	if (level < DIFF_EASY)
		return DIFF_EASY;
	if (level > DIFF_EXPERT)
		return DIFF_EXPERT;

	return static_cast<Tier>(level);
}

const Row &Table(Tier tier)
{
	if (tier < DIFF_EASY || tier > DIFF_EXPERT)
		tier = DIFF_NORMAL;

	return kTable[tier];
}

const char *Name(Tier tier)
{
	return Table(tier).name;
}

float Pin(const char *name)
{
	cvar_t *cvar = g_engfuncs.pfnCVarGetPointer(name);
	return (cvar && cvar->value >= 0.f) ? cvar->value : -1.f;
}

void AimDelay(float &loOut, float &hiOut)
{
	float pin = Pin("crossbot_aim_delay");
	if (pin >= 0.f)
	{
		loOut = pin;
		hiOut = pin;
		return;
	}

	float mid = Table(Current()).aimDelay;
	loOut = mid * (1.f - kDelaySpread);
	hiOut = mid * (1.f + kDelaySpread);
}

float AimError()
{
	return Scalar("crossbot_aim_error", Table(Current()).aimError);
}

float TurnScale()
{
	return Scalar("crossbot_turn_scale", Table(Current()).turnScale);
}

float OppoInterval()
{
	return Scalar("crossbot_oppo_interval", Table(Current()).oppoInterval);
}

bool BackCheck(float &loOut, float &hiOut)
{
	float mid = Scalar("crossbot_back_check", Table(Current()).backPeriod);
	if (mid <= 0.f)
		return false;

	loOut = mid * (1.f - kBackSpread);
	hiOut = mid * (1.f + kBackSpread);
	return true;
}

int HeadPct()
{
	return static_cast<int>(Scalar("crossbot_head_pct",
		static_cast<float>(Table(Current()).headPct)));
}

int PredictPct()
{
	return static_cast<int>(Scalar("crossbot_predict_pct",
		static_cast<float>(Table(Current()).predictPct)));
}

bool CombatInterrupt()
{
	return Boolean("crossbot_combat_interrupt", Table(Current()).combatInterrupt);
}

bool Camp()
{
	return Boolean("crossbot_camp", Table(Current()).camp);
}

bool AmbushOverride()
{
	return Boolean("crossbot_ambush", Table(Current()).ambushOverride);
}

bool PickupPriority()
{
	return Boolean("crossbot_pickup_priority", Table(Current()).pickupPriority);
}

bool GaussPolicy()
{
	return Boolean("crossbot_gauss", Table(Current()).gaussPolicy);
}

bool BunnyHop()
{
	return Boolean("crossbot_bunnyhop", Table(Current()).bunnyHop);
}

void PrintStatus()
{
	Tier tier = Current();
	char msg[256];
	float lo = 0.f, hi = 0.f;

	snprintf(msg, sizeof(msg), "CrossBot: difficulty %d (%s)\n", static_cast<int>(tier), Name(tier));
	SERVER_PRINT(msg);

	AimDelay(lo, hi);
	snprintf(msg, sizeof(msg), "  %-18s %.3f-%.3f  %s\n", "aim_delay", lo, hi,
		Pin("crossbot_aim_delay") >= 0.f ? "pinned" : "tier");
	SERVER_PRINT(msg);

	snprintf(msg, sizeof(msg), "  %-18s %-12.2f %s\n", "aim_error", AimError(),
		Pin("crossbot_aim_error") >= 0.f ? "pinned" : "tier");
	SERVER_PRINT(msg);

	snprintf(msg, sizeof(msg), "  %-18s %-12.2f %s\n", "turn_scale", TurnScale(),
		Pin("crossbot_turn_scale") >= 0.f ? "pinned" : "tier");
	SERVER_PRINT(msg);

	snprintf(msg, sizeof(msg), "  %-18s %-12.2f %s\n", "oppo_interval", OppoInterval(),
		Pin("crossbot_oppo_interval") >= 0.f ? "pinned" : "tier");
	SERVER_PRINT(msg);

	if (BackCheck(lo, hi))
		snprintf(msg, sizeof(msg), "  %-18s %.3f-%.3f  %s\n", "back_check", lo, hi,
			Pin("crossbot_back_check") >= 0.f ? "pinned" : "tier");
	else
		snprintf(msg, sizeof(msg), "  %-18s %-12s %s\n", "back_check", "off",
			Pin("crossbot_back_check") >= 0.f ? "pinned" : "tier");
	SERVER_PRINT(msg);

	snprintf(msg, sizeof(msg), "  %-18s %-12d %s\n", "head_pct", HeadPct(),
		Pin("crossbot_head_pct") >= 0.f ? "pinned" : "tier");
	SERVER_PRINT(msg);

	snprintf(msg, sizeof(msg), "  %-18s %-12d %s\n", "predict_pct", PredictPct(),
		Pin("crossbot_predict_pct") >= 0.f ? "pinned" : "tier");
	SERVER_PRINT(msg);

	static const char *kFlagNames[] = {
		"combat_interrupt", "camp", "ambush_override", "pickup_priority", "gauss", "bunnyhop" };
	static const char *kFlagCvars[] = {
		"crossbot_combat_interrupt", "crossbot_camp", "crossbot_ambush",
		"crossbot_pickup_priority", "crossbot_gauss", "crossbot_bunnyhop" };
	const bool flags[] = { CombatInterrupt(), Camp(), AmbushOverride(),
		PickupPriority(), GaussPolicy(), BunnyHop() };

	for (int i = 0; i < 6; ++i)
	{
		snprintf(msg, sizeof(msg), "  %-18s %-12s %s\n", kFlagNames[i], flags[i] ? "on" : "off",
			Pin(kFlagCvars[i]) >= 0.f ? "pinned" : "tier");
		SERVER_PRINT(msg);
	}
}

}
