#include <extdll.h>
#include <meta_api.h>

#include <algorithm>
#include <cstdint>

#include "../game/game_profile.h"
#include "../nav/astar_graph.h"
#include "../nav/nav.h"
#include "../nav/nav_path.h"
#include "bot.h"
#include "bot_camp.h"
#include "bot_fun.h"
#include "bot_goal.h"
#include "bot_difficulty.h"
#include "bot_items.h"
#include "bot_log.h"
#include "bot_move.h"
#include "bot_move_nav.h"
#include "bot_strategic.h"
#include "bot_tactics.h"

extern IGameProfile *g_pGameProfile;

namespace
{
	constexpr float kSelectRetryInterval = 0.5f;
	constexpr float kStrategicRefreshInterval = 0.25f;
	constexpr float kGoalBiasFloor = 2.f;
	constexpr float kDefaultBias = 2.f;
	constexpr float kDefaultDetourBudget = 6.f;

	unsigned int NextRand(CBot &bot)
	{
		BotGoal::State &st = bot.goal;
		if (!st.rngSeeded)
		{
			st.rngSeeded = true;
			st.rngState = static_cast<unsigned int>(gpGlobals->time * 1000.f) ^
				static_cast<unsigned int>(reinterpret_cast<uintptr_t>(bot.pEdict));
		}
		st.rngState = st.rngState * 1664525u + 1013904223u;
		return st.rngState;
	}

	float RandUnit(CBot &bot)
	{
		return static_cast<float>(NextRand(bot) >> 8) / 16777216.f;
	}

	float CandCost(const CBot &bot, const Nav::NavMesh &mesh, const std::vector<float> &costs,
		const BotGoal::Candidate &cand)
	{
		if (cand.areaId >= costs.size() || costs[cand.areaId] >= Nav::kUnreachableSeconds)
			return Nav::kUnreachableSeconds;

		Vector from = (static_cast<int>(cand.areaId) == bot.currentAreaId)
			? bot.pEdict->v.origin : mesh.areas[cand.areaId].centroid;

		return costs[cand.areaId] + (cand.point - from).Length() / NavAStar::kNavRunSpeed;
	}

	void CollectTier(const CBot &bot, const Nav::NavMesh &mesh, const std::vector<float> &costs,
		BotGoal::Tier tier, std::vector<BotGoal::Candidate> &out)
	{
		if (g_pGameProfile && !g_pGameProfile->AllowsGoalTier(tier))
			return;

		switch (tier)
		{
		case BotGoal::GOAL_STRATEGIC:
			BotStrategic::Collect(bot, mesh, out);
			break;
		case BotGoal::GOAL_FUN:
			BotFun::Collect(bot, mesh, out);
			break;
		case BotGoal::GOAL_GATHER:
			BotItems::Collect(bot, mesh, costs, out);
			break;
		default:
			break;
		}
	}

	void PruneUnreachable(const CBot &bot, const Nav::NavMesh &mesh,
		const std::vector<float> &costs, std::vector<BotGoal::Candidate> &cands)
	{
		cands.erase(std::remove_if(cands.begin(), cands.end(),
			[&](const BotGoal::Candidate &c) {
				return CandCost(bot, mesh, costs, c) >= Nav::kUnreachableSeconds;
			}), cands.end());
	}

	void KeepCheapestPerEntity(const CBot &bot, const Nav::NavMesh &mesh, const std::vector<float> &costs,
		std::vector<BotGoal::Candidate> &cands)
	{
		std::vector<BotGoal::Candidate> kept;
		std::vector<float> keptCost;
		for (const BotGoal::Candidate &c : cands)
		{
			float cost = CandCost(bot, mesh, costs, c);
			size_t i = 0;
			while (i < kept.size() && (kept[i].ent != c.ent || kept[i].payload != c.payload))
				++i;

			if (i == kept.size())
			{
				kept.push_back(c);
				keptCost.push_back(cost);
			}
			else if (cost < keptCost[i])
			{
				kept[i] = c;
				keptCost[i] = cost;
			}
		}
		cands.swap(kept);
	}

	const BotGoal::Candidate *PickCheapest(const CBot &bot, const Nav::NavMesh &mesh,
		const std::vector<float> &costs, const std::vector<BotGoal::Candidate> &cands, float &costOut)
	{
		const BotGoal::Candidate *best = nullptr;
		float bestCost = 0.f;
		for (const BotGoal::Candidate &c : cands)
		{
			float cost = CandCost(bot, mesh, costs, c);
			if (!best || cost < bestCost)
			{
				best = &c;
				bestCost = cost;
			}
		}
		costOut = bestCost;
		return best;
	}

	const BotGoal::Candidate *PickWeighted(CBot &bot, const Nav::NavMesh &mesh,
		const std::vector<float> &costs, const std::vector<BotGoal::Candidate> &cands, float &costOut)
	{
		float bias = BotGoal::Bias();
		float total = 0.f;
		std::vector<float> weights;
		weights.reserve(cands.size());

		for (const BotGoal::Candidate &c : cands)
		{
			float cost = CandCost(bot, mesh, costs, c);
			float w = 1.f / powf(cost + kGoalBiasFloor, bias);
			weights.push_back(w);
			total += w;
		}

		if (total <= 0.f)
			return PickCheapest(bot, mesh, costs, cands, costOut);

		float roll = RandUnit(bot) * total;
		for (size_t i = 0; i < cands.size(); ++i)
		{
			roll -= weights[i];
			if (roll > 0.f && i + 1 < cands.size())
				continue;
			costOut = CandCost(bot, mesh, costs, cands[i]);
			return &cands[i];
		}

		return PickCheapest(bot, mesh, costs, cands, costOut);
	}

	bool Skipped(const CBot &bot, edict_t *ent)
	{
		for (const CBotEntHandle &h : bot.goal.skipped)
			if (h.Is(ent))
				return true;
		return false;
	}

	bool TargetValid(const CBot &bot)
	{
		const BotGoal::Stop &target = bot.goal.target;
		edict_t *ent = target.ent.Get();
		if (!ent)
			return false;

		if (target.tier == BotGoal::GOAL_FUN)
			return BotFun::ValidFor(bot, ent, target.payload);

		if (target.tier == BotGoal::GOAL_GATHER)
			return !BotItems::IsUnavailable(ent, static_cast<ItemKind>(target.payload));

		if (target.tier == BotGoal::GOAL_STRATEGIC)
			return BotStrategic::ValidFor(bot, ent, target.payload);

		return true;
	}

	void CommitStop(CBot &bot, const BotGoal::Stop &stop, bool detour)
	{
		BotGoal::State &st = bot.goal;

		BotItems::ForgetGoalItem(bot);
		BotFun::Forget(bot);
		BotStrategic::Forget(bot);

		st.stop = stop;
		st.onDetour = detour;
		st.active = true;

		BotGoal::Candidate cand;
		cand.tier = stop.tier;
		cand.ent = stop.ent.Get();
		cand.areaId = static_cast<uint32_t>(stop.areaId < 0 ? 0 : stop.areaId);
		cand.point = stop.point;
		cand.payload = stop.payload;

		if (stop.tier == BotGoal::GOAL_FUN)
			BotFun::Accept(bot, cand);
		else if (stop.tier == BotGoal::GOAL_GATHER)
			BotItems::Accept(bot, cand);
		else if (stop.tier == BotGoal::GOAL_STRATEGIC)
			BotStrategic::Accept(bot, cand);

		bot.goalPoint = stop.point;
		bot.hasGoal = true;
	}

	BotGoal::Stop MakeStop(const BotGoal::Candidate &cand)
	{
		BotGoal::Stop stop;
		stop.tier = cand.tier;
		stop.ent.Set(cand.ent);
		stop.areaId = static_cast<int>(cand.areaId);
		stop.point = cand.point;
		stop.payload = cand.payload;
		return stop;
	}

	void PickStop(CBot &bot, const Nav::NavMesh &mesh, const std::vector<float> &fromBot)
	{
		BotGoal::State &st = bot.goal;

		if (st.toTargetArea < 0)
		{
			CommitStop(bot, st.target, false);
			return;
		}

		const std::vector<float> &toGoal = st.toTargetCosts;
		if (bot.currentAreaId < 0 || static_cast<size_t>(bot.currentAreaId) >= toGoal.size())
		{
			CommitStop(bot, st.target, false);
			return;
		}

		float here = toGoal[bot.currentAreaId];
		if (here >= Nav::kUnreachableSeconds)
		{
			CommitStop(bot, st.target, false);
			return;
		}

		if (st.target.tier == BotGoal::GOAL_STRATEGIC && BotTactics::Pick(bot, mesh, fromBot))
			return;

		float budget = BotGoal::DetourBudget();

		std::vector<BotGoal::Candidate> gather;
		BotItems::Collect(bot, mesh, fromBot, gather);

		const BotGoal::Candidate *best = nullptr;
		float bestFrom = 0.f;

		for (const BotGoal::Candidate &c : gather)
		{
			if (c.areaId >= toGoal.size() || c.areaId >= fromBot.size())
				continue;
			if (st.target.ent.Is(c.ent) || Skipped(bot, c.ent))
				continue;

			float in = toGoal[c.areaId];
			float out = fromBot[c.areaId];
			if (in >= Nav::kUnreachableSeconds || out >= Nav::kUnreachableSeconds)
				continue;
			if (in >= here)
				continue;
			if (out + in - here > budget)
				continue;

			if (!best || out < bestFrom)
			{
				best = &c;
				bestFrom = out;
			}
		}

		if (!best)
		{
			CommitStop(bot, st.target, false);
			return;
		}

		BOT_VLOG("CrossBot: %s detours to %s at (%.1f,%.1f,%.1f) area %u on the way, %.1fs out, "
			"%.1fs left after (was %.1fs)\n", bot.name, CBotClassOf(best->ent), best->point.x,
			best->point.y, best->point.z, best->areaId, bestFrom, toGoal[best->areaId], here);

		CommitStop(bot, MakeStop(*best), true);
	}
}

namespace BotGoal
{

float Bias()
{
	cvar_t *cvar = g_engfuncs.pfnCVarGetPointer("crossbot_goal_bias");
	float v = cvar ? cvar->value : kDefaultBias;
	return v < 0.f ? 0.f : v;
}

float DetourBudget()
{
	cvar_t *cvar = g_engfuncs.pfnCVarGetPointer("crossbot_detour_budget");
	float v = cvar ? cvar->value : kDefaultDetourBudget;
	return v < 0.f ? 0.f : v;
}

const char *TierName(Tier tier)
{
	switch (tier)
	{
	case GOAL_GATHER: return "gather";
	case GOAL_FUN: return "fun";
	case GOAL_STRATEGIC: return "strategic";
	default: break;
	}
	return "none";
}

Tier StopTier(const CBot &bot)
{
	return bot.goal.active ? bot.goal.stop.tier : GOAL_NONE;
}

bool Select(CBot &bot, const Nav::NavMesh &mesh)
{
	if (!g_pGameProfile || bot.currentAreaId < 0)
		return false;

	if (gpGlobals->time < bot.nextSelectTime)
		return false;

	std::vector<float> costs;
	Nav::AreaCosts(mesh, static_cast<uint32_t>(bot.currentAreaId), costs,
		Bot_GravityMultiplier(bot.pEdict), Bot_Carefulness(bot),
		Bot_MaxSpeed(bot.pEdict), Bot_RangeBias(bot), Bot_FallDamage(bot));

	std::vector<Candidate> promoted;
	if (BotDiff::PickupPriority())
	{
		CollectTier(bot, mesh, costs, GOAL_GATHER, promoted);
		PruneUnreachable(bot, mesh, costs, promoted);
		BotItems::KeepPreferred(promoted);
	}

	std::vector<Candidate> cands;
	Tier tier = GOAL_NONE;

	for (int t = GOAL_STRATEGIC; t >= GOAL_GATHER; --t)
	{
		cands.clear();
		CollectTier(bot, mesh, costs, static_cast<Tier>(t), cands);
		PruneUnreachable(bot, mesh, costs, cands);
		if (t == GOAL_STRATEGIC)
		{
			BotStrategic::Prioritize(cands);
			KeepCheapestPerEntity(bot, mesh, costs, cands);
		}
		if (t == GOAL_FUN)
		{
			BotFun::Prioritize(cands);
			cands.insert(cands.end(), promoted.begin(), promoted.end());
		}
		if (!cands.empty())
		{
			tier = static_cast<Tier>(t);
			break;
		}
	}

	if (tier == GOAL_NONE)
	{
		bot.nextSelectTime = gpGlobals->time + kSelectRetryInterval;
		return false;
	}

	float cost = 0.f;
	const Candidate *best = (tier == GOAL_GATHER)
		? PickCheapest(bot, mesh, costs, cands, cost)
		: PickWeighted(bot, mesh, costs, cands, cost);

	if (!best)
	{
		bot.nextSelectTime = gpGlobals->time + kSelectRetryInterval;
		return false;
	}

	State &st = bot.goal;
	st.target = MakeStop(*best);
	st.targetOrigin = best->ent ? best->ent->v.origin : best->point;
	st.skipped.clear();
	st.active = true;

	bool strategic = best->tier == GOAL_STRATEGIC;
	BOT_VLOG("CrossBot: %s wants %s%s%s%s %s at (%.1f,%.1f,%.1f) in area %u, %.1fs away, %zu candidates\n",
		bot.name, TierName(best->tier), (best->tier != tier) ? " (promoted)" : "",
		strategic ? " to " : "", strategic ? BotStrategic::KindName(best->payload) : "",
		CBotClassOf(best->ent), best->point.x, best->point.y,
		best->point.z, best->areaId, cost, cands.size());

	if (tier > GOAL_GATHER)
	{
		Nav::AreaCostsToGoal(mesh, static_cast<uint32_t>(st.target.areaId), st.toTargetCosts,
			Bot_GravityMultiplier(bot.pEdict), Bot_Carefulness(bot),
			Bot_MaxSpeed(bot.pEdict), Bot_RangeBias(bot), Bot_FallDamage(bot));
		st.toTargetArea = st.target.areaId;
		BotCamp::BuildRoute(bot, mesh, costs);
	}
	else
	{
		st.toTargetCosts.clear();
		st.toTargetArea = -1;
		BotCamp::Reset(bot);
	}

	PickStop(bot, mesh, costs);
	return true;
}

void RefreshStrategic(CBot &bot, const Nav::NavMesh &mesh)
{
	State &st = bot.goal;

	if (!st.active || st.target.tier != GOAL_STRATEGIC || bot.strat.arrival)
		return;

	if (gpGlobals->time < st.nextRefreshTime)
		return;

	st.nextRefreshTime = gpGlobals->time + kStrategicRefreshInterval;

	edict_t *ent = st.target.ent.Get();
	if (!BotStrategic::ValidFor(bot, ent, st.target.payload))
	{
		Clear(bot);
		return;
	}

	if ((ent->v.origin - st.targetOrigin).Length() <= BotStrategic::MoveRadius(st.target.payload))
		return;

	if (bot.currentAreaId < 0)
		return;

	uint32_t areaId = 0;
	Vector point;
	if (!BotStrategic::ResolvePoint(mesh, ent->v.origin, areaId, point))
	{
		Clear(bot);
		return;
	}

	BOT_VLOG("CrossBot: %s follows the strategic goal to (%.1f,%.1f,%.1f) in area %u\n",
		bot.name, point.x, point.y, point.z, areaId);

	st.target.areaId = static_cast<int>(areaId);
	st.target.point = point;
	st.targetOrigin = ent->v.origin;
	st.skipped.clear();

	Nav::AreaCostsToGoal(mesh, areaId, st.toTargetCosts, Bot_GravityMultiplier(bot.pEdict),
		Bot_Carefulness(bot), Bot_MaxSpeed(bot.pEdict), Bot_RangeBias(bot), Bot_FallDamage(bot));
	st.toTargetArea = st.target.areaId;

	std::vector<float> costs;
	Nav::AreaCosts(mesh, static_cast<uint32_t>(bot.currentAreaId), costs,
		Bot_GravityMultiplier(bot.pEdict), Bot_Carefulness(bot),
		Bot_MaxSpeed(bot.pEdict), Bot_RangeBias(bot), Bot_FallDamage(bot));

	BotCamp::BuildRoute(bot, mesh, costs);

	PickStop(bot, mesh, costs);
	Bot_ResetNavPath(bot);
}

bool Advance(CBot &bot)
{
	State &st = bot.goal;

	if (!st.active || !st.onDetour)
	{
		Clear(bot);
		return false;
	}

	const Nav::NavMesh *mesh = Nav::Get();
	if (!mesh || bot.currentAreaId < 0 || !TargetValid(bot))
	{
		Clear(bot);
		return false;
	}

	std::vector<float> costs;
	Nav::AreaCosts(*mesh, static_cast<uint32_t>(bot.currentAreaId), costs,
		Bot_GravityMultiplier(bot.pEdict), Bot_Carefulness(bot),
		Bot_MaxSpeed(bot.pEdict), Bot_RangeBias(bot), Bot_FallDamage(bot));

	PickStop(bot, *mesh, costs);
	Bot_ResetNavPath(bot);
	return true;
}

void CommitCampStop(CBot &bot, const Vector &point, int areaId, int campIndex)
{
	Stop stop;
	stop.tier = GOAL_NONE;
	stop.areaId = areaId;
	stop.point = point;
	stop.camp = true;
	stop.campIndex = campIndex;

	CommitStop(bot, stop, true);
}

bool OnArrived(CBot &bot)
{
	if (bot.goal.stop.camp)
		return BotCamp::OnArrived(bot);

	if (bot.goal.stop.tier == GOAL_FUN)
		BotFun::OnArrived(bot);
	else if (bot.goal.stop.tier != GOAL_STRATEGIC)
		BotItems::OnArrived(bot);

	return Advance(bot);
}

void Clear(CBot &bot)
{
	State &st = bot.goal;

	BotItems::ForgetGoalItem(bot);
	BotFun::Forget(bot);
	BotStrategic::Forget(bot);

	st.target = Stop();
	st.stop = Stop();
	st.active = false;
	st.onDetour = false;
	st.targetOrigin = Vector(0.f, 0.f, 0.f);
	st.nextRefreshTime = 0.f;
	st.toTargetCosts.clear();
	st.toTargetArea = -1;
	st.skipped.clear();
	BotCamp::Reset(bot);

	bot.hasGoal = false;
	Bot_ResetNavPath(bot);
}

void OnRepathFailed(CBot &bot)
{
	State &st = bot.goal;

	if (st.active && st.stop.camp)
	{
		BotCamp::Abandon(bot, "no route to it");
		Advance(bot);
		return;
	}

	if (st.active && st.onDetour)
	{
		edict_t *ent = st.stop.ent.Get();
		if (ent)
		{
			BOT_VLOG("CrossBot: %s could not plan a route to the detour %s, going straight on\n",
				bot.name, CBotClassOf(ent));
			BotItems::NoteIgnored(bot, ent, g_pGameProfile->ItemUnreachableTimeout());
		}
		st.skipped.push_back(st.stop.ent);
		BotItems::ForgetGoalItem(bot);
		if (Advance(bot))
			return;
		return;
	}

	if (st.stop.tier == GOAL_FUN)
		BotFun::OnRepathFailed(bot);
	else if (st.stop.tier == GOAL_STRATEGIC)
		BotStrategic::OnRepathFailed(bot);
	else
		BotItems::OnRepathFailed(bot);

	Clear(bot);
}

void OnBlocked(CBot &bot, const char *what)
{
	State &st = bot.goal;

	if (st.active && st.stop.camp)
	{
		BotCamp::Abandon(bot, "blocked on the way to it");
		Advance(bot);
		return;
	}

	if (st.active && st.onDetour)
	{
		edict_t *ent = st.stop.ent.Get();
		if (ent)
		{
			BOT_VLOG("CrossBot: %s could not get past %s reaching the detour %s, going straight on\n",
				bot.name, what, CBotClassOf(ent));
			BotItems::NoteIgnored(bot, ent, g_pGameProfile->ItemUnreachableTimeout());
		}
		st.skipped.push_back(st.stop.ent);
		BotItems::ForgetGoalItem(bot);
		if (Advance(bot))
			return;
		return;
	}

	if (st.stop.tier == GOAL_FUN)
		BotFun::OnBlocked(bot, what);
	else if (st.stop.tier == GOAL_STRATEGIC)
		BotStrategic::OnBlocked(bot, what);
	else
		BotItems::OnBlocked(bot, what);

	Clear(bot);
}

void OnRespawn(CBot &bot)
{
	BotItems::OnRespawn(bot);
	BotFun::OnRespawn(bot);
	BotStrategic::OnRespawn(bot);
	BotCamp::OnRespawn(bot);
	Clear(bot);
}

}
