#include <extdll.h>
#include <meta_api.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iterator>

#include "../game/cvars.h"
#include "../game/game_profile.h"
#include "../game/objectives.h"
#include "../nav/astar_graph.h"
#include "../nav/nav.h"
#include "../nav/nav_path.h"
#include "../nav/nav_query.h"
#include "bot.h"
#include "bot_camp.h"
#include "bot_combat.h"
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
	constexpr float kBias = 2.f;
	constexpr float kDetourBudget = 6.f;
	constexpr float kRevengeHoldMin = 4.f;
	constexpr float kRevengeHoldMax = 10.f;

	bool RevengeOn()
	{
		cvar_t *cvar = CBotCvar("crossbot_revenge");
		return cvar && cvar->value != 0.f;
	}

	float RevengeBudget()
	{
		cvar_t *cvar = CBotCvar("crossbot_revenge_budget");
		return cvar ? cvar->value : 10.f;
	}

	bool RevengeDebug()
	{
		cvar_t *cvar = CBotCvar("crossbot_revenge_debug");
		return cvar && cvar->value != 0.f;
	}

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

	float RevengeWeight(const std::vector<float> &fromBot, int area)
	{
		return 1.f / powf(fromBot[area] + kGoalBiasFloor, BotGoal::Bias());
	}

	bool RevengeReachable(const CBot &bot, const std::vector<float> &fromBot, int area)
	{
		return area >= 0 && static_cast<size_t>(area) < fromBot.size() && area != bot.currentAreaId &&
			fromBot[area] < Nav::kUnreachableSeconds;
	}

	void ChooseRevenge(CBot &bot, const Nav::NavMesh &mesh, const std::vector<float> &fromBot)
	{
		BotRevenge &rv = bot.revenge;
		std::vector<int> camps;
		std::vector<float> weights;
		float total = 0.f;

		const Nav::Area *spot = Nav::FindAreaNearest(mesh, rv.point);
		int spotArea = spot ? static_cast<int>(spot->id) : -1;
		float spotWeight = 0.f;
		if (RevengeReachable(bot, fromBot, spotArea))
		{
			spotWeight = RevengeWeight(fromBot, spotArea);
			total += spotWeight;
		}

		float radius = BotGoal::RevengeLosRadius();
		float cosFov = BotGoal::RevengeFovCos();
		int traced = 0;
		for (size_t i = 0; i < mesh.camps.size(); ++i)
		{
			const Nav::CampSpot &camp = mesh.camps[i];
			if (!RevengeReachable(bot, fromBot, camp.areaId) || !BotCamp::SpotAllowed(bot, camp))
				continue;

			Vector eye = BotCamp::SpotEye(camp);
			Vector delta = eye - rv.eye;
			float dist = delta.Length();
			if (dist > radius || dist < 1.f)
				continue;
			if (DotProduct(delta * (1.f / dist), rv.aimForward) >= cosFov)
				continue;

			++traced;
			TraceResult tr;
			TRACE_LINE(eye, rv.point, ignore_monsters, bot.pEdict, &tr);
			if (tr.flFraction < 1.f || tr.fStartSolid)
				continue;

			float w = RevengeWeight(fromBot, camp.areaId);
			camps.push_back(static_cast<int>(i));
			weights.push_back(w);
			total += w;
		}

		rv.campIndex = -1;
		rv.areaId = -1;
		if (total <= 0.f)
		{
			rv.active = false;
			if (RevengeDebug())
				BOT_VLOG("CrossBot: %s has nowhere reachable to take revenge from (%d camps traced)\n",
					bot.name, traced);
			return;
		}

		float roll = RandUnit(bot) * total - spotWeight;
		for (size_t i = 0; roll > 0.f && i < camps.size(); ++i)
		{
			rv.campIndex = camps[i];
			roll -= weights[i];
		}

		if (rv.campIndex >= 0)
		{
			const Nav::CampSpot &camp = mesh.camps[rv.campIndex];
			rv.areaId = camp.areaId;
			rv.dest = camp.pos;
		}
		else
		{
			rv.areaId = spotArea;
			rv.dest = rv.point;
		}

		if (RevengeDebug())
			BOT_VLOG("CrossBot: %s picks revenge %s %d at (%.1f,%.1f,%.1f), %.1fs out, of %zu camps "
				"(%d traced)%s\n", bot.name, rv.campIndex >= 0 ? "camp" : "spot area",
				rv.campIndex >= 0 ? rv.campIndex : rv.areaId, rv.dest.x, rv.dest.y, rv.dest.z,
				fromBot[rv.areaId], camps.size(), traced, spotWeight > 0.f ? " plus the spot" : "");
	}

	bool RevengeReady(CBot &bot, const Nav::NavMesh &mesh, const std::vector<float> &fromBot,
		BotGoal::Stop &out)
	{
		BotRevenge &rv = bot.revenge;
		if (!rv.active || !RevengeOn())
			return false;

		if (rv.areaId < 0)
			ChooseRevenge(bot, mesh, fromBot);

		if (!rv.active || bot.currentAreaId == rv.areaId)
		{
			rv.active = false;
			return false;
		}

		out = BotGoal::Stop();
		out.tier = BotGoal::GOAL_NONE;
		out.areaId = rv.areaId;
		out.point = rv.dest;
		out.revenge = true;
		out.campIndex = rv.campIndex;
		return true;
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
		switch (tier)
		{
		case BotGoal::GOAL_STRATEGIC:
			BotStrategic::Collect(bot, mesh, costs, out);
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
			float cost = c.rollCost >= 0.f ? c.rollCost : CandCost(bot, mesh, costs, c);
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

	bool Carrying(const CBot &bot)
	{
		for (const Objectives::Carrier &carrier : Objectives::Get().carriers)
		{
			if (carrier.player.Is(bot.pEdict))
				return true;
		}
		return false;
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

		const Nav::NavMesh *mesh = Nav::Get();
		if (stop.revenge && stop.campIndex >= 0 && mesh)
		{
			BotCamp::CommitSnipe(bot, *mesh, stop.campIndex,
				g_engfuncs.pfnRandomFloat(kRevengeHoldMin, kRevengeHoldMax), 0, bot.revenge.point);
			st.stop.revenge = true;
			return;
		}

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

		if (!st.target.revenge && !BotStrategic::Urgent(bot))
		{
			BotGoal::Stop rv;
			if (RevengeReady(bot, mesh, fromBot, rv))
			{
				size_t ra = static_cast<size_t>(rv.areaId);
				if (ra < toGoal.size() && ra < fromBot.size())
				{
					float in = toGoal[ra];
					float out = fromBot[ra];
					if (in < Nav::kUnreachableSeconds && out < Nav::kUnreachableSeconds &&
						in < here && out + in - here <= RevengeBudget())
					{
						if (RevengeDebug())
							BOT_VLOG("CrossBot: %s detours for revenge to (%.1f,%.1f,%.1f) area %d, "
								"%.1fs out\n", bot.name, rv.point.x, rv.point.y, rv.point.z, rv.areaId, out);
						CommitStop(bot, rv, true);
						return;
					}
				}
			}
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
			if (out + in - here > budget + c.worth)
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
	return kBias;
}

float DetourBudget()
{
	return kDetourBudget;
}

const char *TierName(Tier tier)
{
	switch (tier)
	{
	case GOAL_GATHER: return "gather";
	case GOAL_FUN: return "fun";
	case GOAL_STRATEGIC: return "strategic";
	case GOAL_DIRECTIVE: return "directive";
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
		Bot_MaxSpeed(bot.pEdict), Bot_RangeBias(bot), Bot_FallDamage(bot), &bot.hurt);

	std::vector<Candidate> gather;
	CollectTier(bot, mesh, costs, GOAL_GATHER, gather);
	PruneUnreachable(bot, mesh, costs, gather);

	std::vector<Candidate> promoted;
	if (BotDiff::PickupPriority())
	{
		promoted = gather;
		BotItems::KeepPreferred(promoted);
	}

	std::vector<Candidate> cands;
	std::vector<Candidate> roam;
	Tier tier = GOAL_NONE;

	if (!Carrying(bot))
	{
		std::copy_if(gather.begin(), gather.end(), std::back_inserter(cands), [&](const Candidate &c) {
			return c.worth > CandCost(bot, mesh, costs, c);
		});
		if (!cands.empty())
			tier = GOAL_GATHER;
	}
	bool needed = tier != GOAL_NONE;

	for (int t = GOAL_STRATEGIC; tier == GOAL_NONE && t >= GOAL_GATHER; --t)
	{
		cands.clear();
		if (t == GOAL_GATHER)
			cands = gather;
		else
			CollectTier(bot, mesh, costs, static_cast<Tier>(t), cands);
		PruneUnreachable(bot, mesh, costs, cands);
		if (t == GOAL_STRATEGIC)
		{
			BotStrategic::Prioritize(cands);
			KeepCheapestPerEntity(bot, mesh, costs, cands);
			if (std::all_of(cands.begin(), cands.end(), [](const Candidate &c) {
					return BotStrategic::KindOf(c.payload) == BotStrategic::KIND_ROAM; }))
				roam.swap(cands);
		}
		if (t == GOAL_FUN)
		{
			BotFun::Prioritize(cands);
			cands.insert(cands.end(), promoted.begin(), promoted.end());
			if (cands.empty() && !roam.empty())
			{
				cands.swap(roam);
				t = GOAL_STRATEGIC;
			}
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
		bot.name, TierName(best->tier), (best->tier != tier) ? " (promoted)" : (needed ? " (needed)" : ""),
		strategic ? " to " : "", strategic ? BotStrategic::KindName(best->payload) : "",
		CBotClassOf(best->ent), best->point.x, best->point.y,
		best->point.z, best->areaId, cost, cands.size());

	bool dmClass = !needed && (best->tier == GOAL_GATHER ||
		(best->tier == GOAL_STRATEGIC && BotStrategic::KindOf(best->payload) == BotStrategic::KIND_ROAM));
	if (dmClass)
	{
		Stop rv;
		if (RevengeReady(bot, mesh, costs, rv))
		{
			if (RevengeDebug())
				BOT_VLOG("CrossBot: %s seeks revenge at (%.1f,%.1f,%.1f) area %d\n",
					bot.name, rv.point.x, rv.point.y, rv.point.z, rv.areaId);
			st.target = rv;
			st.targetOrigin = rv.point;
			tier = GOAL_FUN;
		}
	}

	if (tier > GOAL_GATHER)
	{
		Nav::AreaCostsToGoal(mesh, static_cast<uint32_t>(st.target.areaId), st.toTargetCosts,
			Bot_GravityMultiplier(bot.pEdict), Bot_Carefulness(bot),
			Bot_MaxSpeed(bot.pEdict), Bot_RangeBias(bot), Bot_FallDamage(bot), &bot.hurt);
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

void Commit(CBot &bot, const Candidate &cand)
{
	State &st = bot.goal;
	st.target = MakeStop(cand);
	st.targetOrigin = cand.ent ? cand.ent->v.origin : cand.point;
	st.skipped.clear();
	st.toTargetCosts.clear();
	st.toTargetArea = -1;
	BotCamp::Reset(bot);

	CommitStop(bot, st.target, false);
	Bot_ResetNavPath(bot);
}

void RefreshStrategic(CBot &bot, const Nav::NavMesh &mesh)
{
	State &st = bot.goal;

	if (!st.active || st.target.tier != GOAL_STRATEGIC)
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

	if (BotStrategic::Outranked(bot, st.target.payload))
	{
		BOT_VLOG("CrossBot: %s drops the %s goal for a more urgent one\n", bot.name,
			BotStrategic::KindName(st.target.payload));
		bot.strat.overrideDrop = gpGlobals->time;
		Clear(bot);
		return;
	}

	if (bot.strat.arrival || bot.camp.holding)
		return;

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
		Bot_Carefulness(bot), Bot_MaxSpeed(bot.pEdict), Bot_RangeBias(bot), Bot_FallDamage(bot), &bot.hurt);
	st.toTargetArea = st.target.areaId;

	std::vector<float> costs;
	Nav::AreaCosts(mesh, static_cast<uint32_t>(bot.currentAreaId), costs,
		Bot_GravityMultiplier(bot.pEdict), Bot_Carefulness(bot),
		Bot_MaxSpeed(bot.pEdict), Bot_RangeBias(bot), Bot_FallDamage(bot), &bot.hurt);

	BotCamp::BuildRoute(bot, mesh, costs);

	PickStop(bot, mesh, costs);
	Bot_ResetNavPath(bot);
}

bool Advance(CBot &bot)
{
	State &st = bot.goal;

	if (st.stop.revenge)
	{
		bot.revenge.active = false;
		if (st.target.revenge)
		{
			Clear(bot);
			return false;
		}
	}

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
		Bot_MaxSpeed(bot.pEdict), Bot_RangeBias(bot), Bot_FallDamage(bot), &bot.hurt);

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
	if (bot.goal.stop.revenge)
	{
		if (RevengeDebug())
			BOT_VLOG("CrossBot: %s reached its revenge %s\n", bot.name, bot.goal.stop.camp ? "camp" : "spot");
		bot.revenge.active = false;
		return bot.goal.stop.camp ? BotCamp::OnArrived(bot) : Advance(bot);
	}

	if (bot.goal.stop.camp)
		return BotCamp::OnArrived(bot);

	if (bot.goal.stop.tier == GOAL_FUN)
		BotFun::OnArrived(bot);
	else if (bot.goal.stop.tier != GOAL_STRATEGIC && bot.goal.stop.tier != GOAL_DIRECTIVE)
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

	if (st.active && st.stop.revenge)
	{
		if (RevengeDebug())
			BOT_VLOG("CrossBot: %s has no unseen route for revenge, giving up on it\n", bot.name);
		bot.revenge.active = false;
		if (st.onDetour)
		{
			Advance(bot);
			return;
		}
		Clear(bot);
		return;
	}

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
	else if (st.stop.tier == GOAL_DIRECTIVE)
		BotDirective::OnGoalFailed(bot, "no route");
	else
		BotItems::OnRepathFailed(bot);

	Clear(bot);
}

void OnBlocked(CBot &bot, const char *what)
{
	State &st = bot.goal;

	if (st.active && st.stop.revenge)
	{
		bot.revenge.active = false;
		if (st.onDetour)
		{
			Advance(bot);
			return;
		}
		Clear(bot);
		return;
	}

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
	else if (st.stop.tier == GOAL_DIRECTIVE)
		BotDirective::OnGoalFailed(bot, what);
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

	bot.revenge.active = bot.revenge.pending && RevengeOn();
	bot.revenge.pending = false;
}

void NoteDeath(CBot &bot)
{
	bot.revenge.pending = false;
	bot.revenge.areaId = -1;
	if (!RevengeOn() || !bot.pEdict)
		return;

	edict_t *killer = BotCombat::Attacker(bot.pEdict);
	if (!killer)
		return;

	edict_t *tracked = BotCombat::Enemy(bot);
	if (!tracked)
		tracked = bot.sense.enemy.Get();
	if (killer != tracked)
		return;

	Vector forward, right, up;
	g_engfuncs.pfnAngleVectors(killer->v.v_angle, forward, right, up);

	bot.revenge.pending = true;
	bot.revenge.point = bot.sense.enemyPos;
	bot.revenge.eye = killer->v.origin + killer->v.view_ofs;
	bot.revenge.aimForward = forward;

	if (RevengeDebug())
		BOT_VLOG("CrossBot: %s was killed by %s, will seek revenge at (%.1f,%.1f,%.1f)\n",
			bot.name, STRING(killer->v.netname), bot.revenge.point.x, bot.revenge.point.y,
			bot.revenge.point.z);
}

bool RevengeEnabled()
{
	return RevengeOn();
}

float RevengeFovCos()
{
	cvar_t *cvar = CBotCvar("crossbot_revenge_fov");
	float deg = cvar ? cvar->value : 120.f;
	return cosf(deg * 0.5f * 0.01745329252f);
}

float RevengeLosRadius()
{
	cvar_t *cvar = CBotCvar("crossbot_revenge_los_radius");
	return (cvar && cvar->value > 0.f) ? cvar->value : 2048.f;
}

}
