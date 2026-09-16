#include <extdll.h>
#include <meta_api.h>

#include <algorithm>
#include <cmath>
#include <unordered_map>
#include <vector>

#include "../game/objectives.h"
#include "../game/teams.h"
#include "../nav/astar_graph.h"
#include "../nav/nav_camp.h"
#include "../nav/nav_mesh.h"
#include "../nav/nav_path.h"
#include "../nav/plan_search.h"
#include "bot.h"
#include "bot_camp.h"
#include "bot_goal.h"
#include "bot_items.h"
#include "bot_log.h"
#include "bot_move.h"
#include "bot_strategic.h"
#include "bot_tactics.h"

namespace
{
	constexpr float kDefaultBudget = 3.f;
	constexpr float kDefaultMargin = 1.f;
	constexpr float kBiasFloor = 2.f;
	constexpr float kFriendSpotRadius = 96.f;
	constexpr float kNearTarget = 4.f;
	constexpr float kSupportHoldMax = 10.f;
	constexpr float kDefendHoldMin = 5.f;
	constexpr float kDefendHoldMax = 15.f;
	constexpr float kFitFloor = 0.1f;

	std::unordered_map<uint32_t, std::vector<float>> g_spawnFields;

	struct Choice
	{
		int campIndex = -1;
		float out = 0.f;
		float runner = 0.f;
		float fit = 1.f;
	};

	float CvarValue(const char *name, float fallback)
	{
		cvar_t *cvar = g_engfuncs.pfnCVarGetPointer(name);
		float v = cvar ? cvar->value : fallback;
		return v < 0.f ? 0.f : v;
	}

	float Weight(float seconds)
	{
		return 1.f / powf(seconds + kBiasFloor, BotGoal::Bias());
	}

	float At(const std::vector<float> &field, int area)
	{
		return (area >= 0 && static_cast<size_t>(area) < field.size()) ? field[area] : Nav::kUnreachableSeconds;
	}

	bool AreaOf(const Nav::NavMesh &mesh, const Vector &origin, uint32_t &area)
	{
		const Nav::Area *found = BotItems::StandArea(mesh, origin);
		if (!found)
			return false;
		area = found->id;
		return true;
	}

	const std::vector<float> &SpawnField(const Nav::NavMesh &mesh, uint32_t area)
	{
		auto it = g_spawnFields.find(area);
		if (it != g_spawnFields.end())
			return it->second;

		std::vector<float> &field = g_spawnFields[area];
		Nav::AreaCosts(mesh, area, field);
		return field;
	}

	bool LiveFriend(const CBot &bot, edict_t *ent)
	{
		if (!ent || ent == bot.pEdict || !CBotIsLivePlayer(ent))
			return false;
		return Teams::Of(bot.pEdict, ent) == Teams::REL_FRIEND;
	}

	bool FriendNear(const CBot &bot, const Vector &pos)
	{
		for (int i = 1; i <= gpGlobals->maxClients; ++i)
		{
			edict_t *ent = g_engfuncs.pfnPEntityOfEntIndex(i);
			if (LiveFriend(bot, ent) && (ent->v.origin - pos).Length() <= kFriendSpotRadius)
				return true;
		}
		return false;
	}

	edict_t *PickFriend(const CBot &bot)
	{
		std::vector<edict_t *> friends;
		std::vector<float> weights;
		float total = 0.f;
		for (int i = 1; i <= gpGlobals->maxClients; ++i)
		{
			edict_t *ent = g_engfuncs.pfnPEntityOfEntIndex(i);
			if (!LiveFriend(bot, ent))
				continue;

			float w = Weight((ent->v.origin - bot.pEdict->v.origin).Length() / NavAStar::kNavRunSpeed);
			friends.push_back(ent);
			weights.push_back(w);
			total += w;
		}

		if (friends.empty())
			return nullptr;

		float roll = g_engfuncs.pfnRandomFloat(0.f, total);
		for (size_t i = 0; i < friends.size(); ++i)
		{
			roll -= weights[i];
			if (roll <= 0.f)
				return friends[i];
		}
		return friends.back();
	}

	int Roll(const std::vector<Choice> &choices)
	{
		if (choices.empty())
			return -1;

		float total = 0.f;
		for (const Choice &c : choices)
			total += Weight(c.out) * c.fit;

		float roll = g_engfuncs.pfnRandomFloat(0.f, total);
		for (size_t i = 0; i < choices.size(); ++i)
		{
			roll -= Weight(choices[i].out) * choices[i].fit;
			if (roll <= 0.f)
				return static_cast<int>(i);
		}
		return static_cast<int>(choices.size()) - 1;
	}

	float Fit(const NavPlan::RangeBias &range, const Nav::CampSpot &camp)
	{
		return std::max(kFitFloor, 1.f - range.Weight() * range.Mismatch(BotCamp::SpotDist(camp)));
	}

	bool Usable(const CBot &bot, const Nav::CampSpot &camp, size_t index)
	{
		return BotCamp::SpotReady(bot, static_cast<int>(index)) && !FriendNear(bot, camp.pos);
	}
}

namespace BotTactics
{

bool Enabled()
{
	cvar_t *cvar = g_engfuncs.pfnCVarGetPointer("crossbot_tactics");
	return !cvar || cvar->value != 0.f;
}

bool DebugEnabled()
{
#if BOT_VLOG_ON
	cvar_t *cvar = g_engfuncs.pfnCVarGetPointer("crossbot_tactics_debug");
	return cvar && cvar->value != 0.f;
#else
	return false;
#endif
}

bool Pick(CBot &bot, const Nav::NavMesh &mesh, const std::vector<float> &fromBot)
{
	if (!Enabled() || mesh.camps.empty())
		return false;

	const BotGoal::Stop &target = bot.goal.target;
	int kind = BotStrategic::KindOf(target.payload);
	if (kind != BotStrategic::KIND_SUPPORT && kind != BotStrategic::KIND_DEFEND)
		return false;

	edict_t *targetEnt = target.ent.Get();
	const std::vector<float> &toTarget = bot.goal.toTargetCosts;
	if (!targetEnt || toTarget.empty())
		return false;

	float budget = CvarValue("crossbot_tactics_budget", kDefaultBudget);
	float margin = CvarValue("crossbot_tactics_margin", kDefaultMargin);
	bool defend = kind == BotStrategic::KIND_DEFEND;
	NavPlan::RangeBias range = Bot_RangeBias(bot, false);

	edict_t *runner = nullptr;
	int spawnIndex = -1;
	uint32_t runnerArea = 0;
	float clock = 0.f;
	std::vector<float> runnerField;
	const std::vector<float> *from = nullptr;

	if (!bot.camp.scared && defend)
	{
		int team = Teams::TeamOf(bot.pEdict);
		const Objectives::Tables &tables = Objectives::Get();
		std::vector<int> enemy;
		for (size_t i = 0; i < tables.spawns.size(); ++i)
		{
			if (tables.spawns[i].team != team && tables.spawns[i].ent.Get())
				enemy.push_back(static_cast<int>(i));
		}

		if (!enemy.empty())
		{
			spawnIndex = enemy[g_engfuncs.pfnRandomLong(0, static_cast<int>(enemy.size()) - 1)];
			edict_t *spawn = tables.spawns[spawnIndex].ent.Get();
			if (AreaOf(mesh, spawn->v.origin, runnerArea))
			{
				from = &SpawnField(mesh, runnerArea);
				clock = gpGlobals->time - bot.strat.spawnTime;
			}
		}
	}
	else if (!bot.camp.scared)
	{
		runner = BotStrategic::Leader(bot, targetEnt);
		if (!runner)
			runner = PickFriend(bot);
		if (runner && AreaOf(mesh, runner->v.origin, runnerArea))
		{
			Nav::AreaCosts(mesh, runnerArea, runnerField);
			from = &runnerField;
		}
	}

	std::vector<Choice> choices;
	float runnerToTarget = from ? At(toTarget, static_cast<int>(runnerArea)) : Nav::kUnreachableSeconds;
	if (runnerToTarget < Nav::kUnreachableSeconds)
	{
		for (size_t i = 0; i < mesh.camps.size(); ++i)
		{
			const Nav::CampSpot &camp = mesh.camps[i];
			float in = At(toTarget, camp.areaId);
			float out = At(fromBot, camp.areaId);
			float r = At(*from, camp.areaId);
			if (in >= Nav::kUnreachableSeconds || out >= Nav::kUnreachableSeconds || r >= Nav::kUnreachableSeconds)
				continue;
			if (in >= runnerToTarget || r + in - runnerToTarget > budget)
				continue;
			if (clock + out + margin >= r)
				continue;
			if (!Usable(bot, camp, i))
				continue;

			Choice c;
			c.campIndex = static_cast<int>(i);
			c.out = out;
			c.runner = r;
			c.fit = Fit(range, camp);
			choices.push_back(c);
		}
	}

	int pick = Roll(choices);
	if (pick >= 0)
	{
		const Choice &c = choices[pick];
		float hold = defend ? g_engfuncs.pfnRandomFloat(kDefendHoldMin, kDefendHoldMax)
			: g_engfuncs.pfnRandomFloat(0.f, kSupportHoldMax);

		if (defend)
			BOT_VLOG("CrossBot: %s guards %s against spawn %d at spot %d (%.1fs, enemy %.1fs, %.0fs into its life, "
				"fit %.2f), holding %.1fs\n", bot.name, CBotTargetNameOf(targetEnt), spawnIndex, c.campIndex, c.out,
				c.runner, clock, c.fit, hold);
		else
			BOT_VLOG("CrossBot: %s supports %s toward %s, camps at spot %d (%.1fs, runner %.1fs, fit %.2f), "
				"holding %.1fs\n", bot.name, CBotPlayerNameOf(runner), CBotTargetNameOf(targetEnt), c.campIndex, c.out,
				c.runner, c.fit, hold);

		BotCamp::CommitTimed(bot, mesh, c.campIndex, hold, c.runner, c.out, clock + margin);
		return true;
	}

	if (!defend)
	{
		if (DebugEnabled())
			BOT_VLOG("CrossBot: %s finds no spot ahead of %s, heading for %s\n", bot.name,
				runner ? CBotPlayerNameOf(runner) : "anyone", CBotTargetNameOf(targetEnt));
		return false;
	}

	choices.clear();
	for (size_t i = 0; i < mesh.camps.size(); ++i)
	{
		const Nav::CampSpot &camp = mesh.camps[i];
		float in = At(toTarget, camp.areaId);
		float out = At(fromBot, camp.areaId);
		if (in > kNearTarget || out >= Nav::kUnreachableSeconds || !Usable(bot, camp, i))
			continue;

		Choice c;
		c.campIndex = static_cast<int>(i);
		c.out = out;
		c.fit = Fit(range, camp);
		choices.push_back(c);
	}

	pick = Roll(choices);
	if (pick < 0)
		return false;

	const Choice &c = choices[pick];
	float hold = g_engfuncs.pfnRandomFloat(kDefendHoldMin, kDefendHoldMax);
	BOT_VLOG("CrossBot: %s falls back to spot %d near %s (%.1fs from it, %.0fs into its life, fit %.2f), "
		"holding %.1fs\n", bot.name, c.campIndex, CBotTargetNameOf(targetEnt), c.out, clock, c.fit, hold);

	BotCamp::CommitTimed(bot, mesh, c.campIndex, hold, 0.f, 0.f, 0.f);
	return true;
}

void ForgetMap()
{
	g_spawnFields.clear();
}

}
