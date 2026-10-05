#pragma once

#include <extdll.h>

#include <utility>
#include <vector>

#include "../game/ent_handle.h"

struct CBot;

namespace BotBreak
{

struct State
{
	CBotEntHandle target;
	CBotEntHandle refused;

	bool active = false;

	int prevWeaponId = 0;
	int swings = 0;

	float startTime = 0.f;
	float lastDamageTime = 0.f;
	float lastHealth = 0.f;
	float nextSelectTime = 0.f;
	float nextSwingTime = 0.f;
	float refusedUntil = 0.f;

	float standoff = 0.f;

	char targetName[32] = {};
};

class PhaseThrough
{
public:
	PhaseThrough();
	~PhaseThrough();

	PhaseThrough(const PhaseThrough &) = delete;
	PhaseThrough &operator=(const PhaseThrough &) = delete;

private:
	std::vector<std::pair<edict_t *, int>> m_saved;
};

bool IsTarget(edict_t *ent);

bool AimSwing(CBot &bot, edict_t *ent, Vector &aimAngles, bool &swing);

bool Engage(CBot &bot, edict_t *ent);

bool EngageStandoff(CBot &bot, edict_t *ent, float minRange);

bool Run(CBot &bot);

bool Active(const CBot &bot);

}
