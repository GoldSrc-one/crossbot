#include <extdll.h>
#include <meta_api.h>
#include <in_buttons.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

#include "bot.h"
#include "bot_aim.h"
#include "bot_goal.h"
#include "bot_items.h"
#include "bot_log.h"
#include "bot_move_nav.h"
#include "bot_sense.h"
#include "bot_strategic.h"

namespace
{
	const char *const kGoalClass = "bot_goal";

	constexpr float kGoalDuckRange = 128.f;
	constexpr float kGoalUseRange = 64.f;
	constexpr float kGoalStandRadius = 8.f;
	constexpr float kGoalFailCooldown = 10.f;

	bool OwnedBy(const edict_t *ent, const edict_t *pEdict)
	{
		return !ent->v.owner || ent->v.owner == pEdict;
	}

	edict_t *AimEntity(edict_t *goal)
	{
		edict_t *aim = goal->v.aiment;
		if (!aim || aim->free || g_engfuncs.pfnIndexOfEdict(aim) == 0)
			return nullptr;
		return aim;
	}

	Vector AimCenter(const edict_t *ent)
	{
		return ent->v.origin + (ent->v.mins + ent->v.maxs) * 0.5f;
	}

	float ReachDist(const Vector &dir)
	{
		float flat = sqrtf(dir.x * dir.x + dir.y * dir.y);
		float rise = fabsf(dir.z);
		return flat > rise ? flat : rise;
	}

	float MemoReady(const CBot &bot, edict_t *ent)
	{
		for (const BotStrategic::Memo &m : bot.strat.memo)
			if (m.ent.Is(ent))
				return m.readyTime;
		return 0.f;
	}

	void NoteCooldown(CBot &bot, edict_t *ent, float seconds)
	{
		if (!ent)
			return;

		for (BotStrategic::Memo &m : bot.strat.memo)
		{
			if (!m.ent.Is(ent))
				continue;
			m.readyTime = gpGlobals->time + seconds;
			return;
		}

		BotStrategic::Memo fresh;
		fresh.ent.Set(ent);
		fresh.readyTime = gpGlobals->time + seconds;
		bot.strat.memo.push_back(fresh);
	}

	void PruneMemos(CBot &bot)
	{
		std::vector<BotStrategic::Memo> &memo = bot.strat.memo;
		memo.erase(std::remove_if(memo.begin(), memo.end(),
			[](const BotStrategic::Memo &m) {
				return !m.ent.Get() || gpGlobals->time >= m.readyTime;
			}), memo.end());
	}
}

namespace BotStrategic
{

bool Enabled()
{
	cvar_t *cvar = g_engfuncs.pfnCVarGetPointer("crossbot_strategic");
	return !cvar || cvar->value != 0.f;
}

bool DebugEnabled()
{
#if BOT_VLOG_ON
	cvar_t *cvar = g_engfuncs.pfnCVarGetPointer("crossbot_strategic_debug");
	return cvar && cvar->value != 0.f;
#else
	return false;
#endif
}

bool IsGoal(edict_t *ent)
{
	return ent && !ent->free && strcmp(CBotClassOf(ent), kGoalClass) == 0;
}

bool ValidFor(const CBot &bot, edict_t *ent)
{
	return IsGoal(ent) && OwnedBy(ent, bot.pEdict);
}

bool ResolvePoint(const Nav::NavMesh &mesh, const Vector &origin, uint32_t &areaId, Vector &point)
{
	const Nav::Area *area = BotItems::StandArea(mesh, origin);
	if (!area)
		return false;

	areaId = area->id;
	point = BotItems::ClampToArea(*area, origin);
	return true;
}

void Collect(const CBot &bot, const Nav::NavMesh &mesh, std::vector<BotGoal::Candidate> &out)
{
	if (bot.currentAreaId < 0 || !Enabled())
		return;

	PruneMemos(const_cast<CBot &>(bot));

	edict_t *ent = nullptr;
	while (true)
	{
		ent = FIND_ENTITY_BY_STRING(ent, "classname", kGoalClass);
		if (!ent || g_engfuncs.pfnIndexOfEdict(ent) == 0)
			break;

		if (!OwnedBy(ent, bot.pEdict))
			continue;

		float ready = MemoReady(bot, ent);
		if (ready > gpGlobals->time)
		{
			if (DebugEnabled())
				BOT_VLOG("CrossBot: %s leaves the goal at (%.1f,%.1f,%.1f) alone for %.1fs more\n",
					bot.name, ent->v.origin.x, ent->v.origin.y, ent->v.origin.z,
					ready - gpGlobals->time);
			continue;
		}

		uint32_t areaId = 0;
		Vector point;
		if (!ResolvePoint(mesh, ent->v.origin, areaId, point))
		{
			if (DebugEnabled())
				BOT_VLOG("CrossBot: %s cannot place the goal at (%.1f,%.1f,%.1f) in any nav area\n",
					bot.name, ent->v.origin.x, ent->v.origin.y, ent->v.origin.z);
			continue;
		}

		if (DebugEnabled())
			BOT_VLOG("CrossBot: %s sees a goal at (%.1f,%.1f,%.1f) in area %u, aiment %s\n", bot.name,
				ent->v.origin.x, ent->v.origin.y, ent->v.origin.z, areaId,
				AimEntity(ent) ? CBotTargetNameOf(AimEntity(ent)) : "none");

		BotGoal::Candidate cand;
		cand.tier = BotGoal::GOAL_STRATEGIC;
		cand.ent = ent;
		cand.areaId = areaId;
		cand.point = point;
		out.push_back(cand);
	}
}

void Accept(CBot &bot, const BotGoal::Candidate &cand)
{
	State &st = bot.strat;
	st.goal.Set(cand.ent);
	st.arrival = false;
	st.pressed = false;
	st.startTime = 0.f;
}

void Forget(CBot &bot)
{
	State &st = bot.strat;
	st.goal.Clear();
	st.arrival = false;
	st.pressed = false;
	st.startTime = 0.f;
}

bool RunArrival(CBot &bot)
{
	State &st = bot.strat;
	edict_t *pEdict = bot.pEdict;
	edict_t *ent = st.goal.Get();

	if (!ValidFor(bot, ent))
		return false;

	if ((ent->v.origin - bot.goal.targetOrigin).Length() > kGoalMoveRadius)
	{
		BOT_VLOG("CrossBot: %s is done here, the goal moved to (%.1f,%.1f,%.1f)\n", bot.name,
			ent->v.origin.x, ent->v.origin.y, ent->v.origin.z);
		return false;
	}

	if (!st.arrival)
	{
		st.arrival = true;
		st.pressed = false;
		st.startTime = gpGlobals->time;

		BOT_VLOG("CrossBot: %s is holding the strategic goal at (%.1f,%.1f,%.1f)\n", bot.name,
			bot.goalPoint.x, bot.goalPoint.y, bot.goalPoint.z);
	}

	bot.lastProgressTime = gpGlobals->time;

	unsigned short buttons = 0;
	edict_t *aim = AimEntity(ent);
	bool facing = false;

	if (aim)
	{
		Vector eye = pEdict->v.origin + pEdict->v.view_ofs;
		Vector center = AimCenter(aim);
		Vector dir = center - eye;
		float dist = ReachDist(dir);

		TraceResult tr = {};
		g_engfuncs.pfnTraceLine(center, eye, ignore_monsters, aim, &tr);
		bool clear = tr.flFraction == 1.f || tr.pHit == pEdict;

		if (dist <= kGoalDuckRange || clear)
		{
			Bot_FaceDir(bot, dir);
			facing = true;

			if (dist <= kGoalDuckRange && (ent->v.button & IN_DUCK) != 0)
				buttons |= IN_DUCK;
			if (dist <= kGoalUseRange && (ent->v.button & IN_USE) != 0)
				buttons |= IN_USE;

			if (buttons != 0 && !st.pressed)
			{
				st.pressed = true;

				BOT_VLOG("CrossBot: %s works %s at the strategic goal, %.1f out\n", bot.name,
					CBotClassOf(aim), dist);
			}
		}
	}

	if (!facing)
		BotSense::ApplyLook(bot);

	Vector angles = Bot_UpdateFacing(bot);
	Bot_DriveTo(bot, angles, bot.goalPoint, false, buttons, 0.f, kGoalStandRadius);
	return true;
}

void OnRepathFailed(CBot &bot)
{
	edict_t *ent = bot.strat.goal.Get();
	if (ent)
	{
		BOT_VLOG("CrossBot: %s could not plan a route to the strategic goal at (%.1f,%.1f,%.1f), "
			"leaving it for %.0fs\n", bot.name, ent->v.origin.x, ent->v.origin.y, ent->v.origin.z,
			kGoalFailCooldown);

		NoteCooldown(bot, ent, kGoalFailCooldown);
	}

	Forget(bot);
}

void OnBlocked(CBot &bot, const char *what)
{
	edict_t *ent = bot.strat.goal.Get();
	if (ent)
	{
		BOT_VLOG("CrossBot: %s could not get past %s reaching the strategic goal at "
			"(%.1f,%.1f,%.1f), leaving it for %.0fs\n", bot.name, what, ent->v.origin.x,
			ent->v.origin.y, ent->v.origin.z, kGoalFailCooldown);

		NoteCooldown(bot, ent, kGoalFailCooldown);
	}

	Forget(bot);
}

void OnRespawn(CBot &bot)
{
	bot.strat.memo.clear();
	Forget(bot);
}

void PrintCandidates(const Nav::NavMesh &mesh)
{
	int count = 0;

	edict_t *ent = nullptr;
	while (true)
	{
		ent = FIND_ENTITY_BY_STRING(ent, "classname", kGoalClass);
		if (!ent || g_engfuncs.pfnIndexOfEdict(ent) == 0)
			break;

		++count;

		uint32_t areaId = 0;
		Vector point;
		char areaBuf[16] = "none";
		if (ResolvePoint(mesh, ent->v.origin, areaId, point))
			snprintf(areaBuf, sizeof(areaBuf), "%u", areaId);

		edict_t *aim = AimEntity(ent);

		char msg[320];
		snprintf(msg, sizeof(msg),
			"CrossBot: goal at (%.1f,%.1f,%.1f) area %s for %s, aiment %s, buttons %s%s\n",
			ent->v.origin.x, ent->v.origin.y, ent->v.origin.z, areaBuf,
			ent->v.owner ? CBotPlayerNameOf(ent->v.owner) : "everyone",
			aim ? CBotTargetNameOf(aim) : "none",
			(ent->v.button & IN_USE) != 0 ? "USE " : "",
			(ent->v.button & IN_DUCK) != 0 ? "DUCK" : "");
		SERVER_PRINT(msg);
	}

	char summary[96];
	snprintf(summary, sizeof(summary), "CrossBot: %d bot_goal entities\n", count);
	SERVER_PRINT(summary);
}

}
