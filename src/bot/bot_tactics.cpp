#include <extdll.h>
#include <meta_api.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <unordered_map>
#include <vector>

#include "../game/game_profile.h"
#include "../game/cvars.h"
#include "../game/objectives.h"
#include "../game/teams.h"
#include "../nav/astar_graph.h"
#include "../nav/nav.h"
#include "../nav/nav_barrier.h"
#include "../nav/nav_camp.h"
#include "../nav/nav_math.h"
#include "../nav/nav_mesh.h"
#include "../nav/nav_path.h"
#include "../nav/nav_query.h"
#include "../nav/plan_search.h"
#include "bot.h"
#include "bot_aim.h"
#include "bot_break.h"
#include "bot_camp.h"
#include "bot_fire.h"
#include "bot_goal.h"
#include "bot_items.h"
#include "bot_log.h"
#include "bot_move.h"
#include "bot_strategic.h"
#include "bot_tactics.h"

namespace
{
	constexpr float kBudget = 3.f;
	constexpr float kMargin = 1.f;
	constexpr float kBiasFloor = 2.f;
	constexpr float kWalkFloor = 2.f;
	constexpr float kCrowdRadius = 1024.f;
	constexpr float kDeployBonus = 10.f;
	constexpr float kSupportHoldMax = 10.f;
	constexpr float kDefendHoldMin = 5.f;
	constexpr float kDefendHoldMax = 15.f;
	constexpr float kFitFloor = 0.1f;
	constexpr int kThreatSteps = 16;
	constexpr float kThreatReach = 512.f;
	constexpr float kSnipeHoldMin = 5.f;
	constexpr float kSnipeHoldMax = 15.f;
	constexpr float kSnipeCone = 45.f;
	constexpr float kSnipeDist = 512.f;
	constexpr float kSnipeDistRatio = 2.f;
	constexpr float kSnipeTargetLift = 12.f;

	using crossbot_nav::kOverlookMargin;
	using crossbot_nav::kOverlookReach;
	using crossbot_nav::kOverlookSamples;
	using crossbot_nav::kOverlookChest;
	constexpr int kOverlookBreakHops = 3;

	std::unordered_map<int, std::vector<float>> g_teamFields;
	uint32_t g_teamFieldsGeneration = 0;
	struct OverlookSet
	{
		Vector point;
		std::vector<BotTactics::OverlookSpot> spots;
	};
	std::unordered_map<int, OverlookSet> g_overlooks;
	struct ObjectSight
	{
		Vector point;
		std::vector<uint8_t> sees;
	};
	std::unordered_map<int, ObjectSight> g_objectSight;

	struct Choice
	{
		int campIndex = -1;
		int slot = -1;
		float out = 0.f;
		float runner = 0.f;
		float fit = 1.f;
		float weight = 0.f;
	};

	float CvarValue(const char *name, float fallback)
	{
		cvar_t *cvar = CBotCvar(name);
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

	const std::vector<float> &TeamField(const Nav::NavMesh &mesh, int team)
	{
		uint32_t generation = Nav::BarrierGeneration(mesh);
		if (generation != g_teamFieldsGeneration)
		{
			g_teamFields.clear();
			g_teamFieldsGeneration = generation;
		}

		auto it = g_teamFields.find(team);
		if (it != g_teamFields.end())
			return it->second;

		std::vector<float> *field = &g_teamFields[team];
		std::vector<uint32_t> roots;
		for (const Objectives::Spawn &spawn : Objectives::Get().spawns)
		{
			edict_t *ent = spawn.ent.Get();
			uint32_t area = 0;
			if (spawn.team == team && ent && AreaOf(mesh, ent->v.origin, area))
				roots.push_back(area);
		}

		Nav::ScopedPlanTeam as(static_cast<uint32_t>(Objectives::TeamBit(team)));
		Nav::AreaCosts(mesh, roots, *field);
		return *field;
	}

	bool LiveFriend(const CBot &bot, edict_t *ent)
	{
		if (!ent || ent == bot.pEdict || !CBotIsLivePlayer(ent) || (g_pGameProfile && g_pGameProfile->IsDead(ent)))
			return false;
		return Teams::Of(bot.pEdict, ent) == Teams::REL_FRIEND;
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

	float Worth(const CBot &bot, const Nav::CampSpot &camp, float fit, float walk)
	{
		float crowd = kCrowdRadius;
		for (int i = 1; i <= gpGlobals->maxClients; ++i)
		{
			edict_t *ent = g_engfuncs.pfnPEntityOfEntIndex(i);
			if (LiveFriend(bot, ent))
				crowd = std::min(crowd, (ent->v.origin - camp.pos).Length());
		}
		float score = std::max(0.f, camp.score);
		if ((camp.flags & crossbot_nav::CAMP_DEPLOY) != 0)
			score *= (g_pGameProfile && g_pGameProfile->CanDeploy(bot.pEdict)) ? kDeployBonus : 1.f / kDeployBonus;
		return score * fit * crowd / kCrowdRadius / std::max(walk, kWalkFloor);
	}

	int Roll(const std::vector<Choice> &choices)
	{
		if (choices.empty())
			return -1;

		float total = 0.f;
		for (const Choice &c : choices)
			total += c.weight;

		float roll = g_engfuncs.pfnRandomFloat(0.f, total);
		for (size_t i = 0; i < choices.size(); ++i)
		{
			roll -= choices[i].weight;
			if (roll <= 0.f)
				return static_cast<int>(i);
		}
		return static_cast<int>(choices.size()) - 1;
	}

	float Fit(const NavPlan::RangeBias &range, const Nav::CampSpot &camp)
	{
		return std::max(kFitFloor, 1.f - range.Weight() * range.Mismatch(BotCamp::SpotDist(camp)));
	}

	Vector Downhill(const Nav::NavMesh &mesh, const std::vector<float> &field, int areaId, const Vector &from)
	{
		int current = areaId;
		for (int step = 0; step < kThreatSteps && current >= 0; ++step)
		{
			const Nav::Area &area = mesh.areas[current];
			Vector d = area.centroid - from;
			if (step > 0 && d.x * d.x + d.y * d.y >= kThreatReach * kThreatReach)
				break;

			int best = -1;
			float bestCost = At(field, current);
			for (uint32_t ci : area.connIndices)
			{
				const Nav::Connection &c = mesh.conns[ci];
				int other = static_cast<int>(c.areaA == area.id ? c.areaB : c.areaA);
				float cost = At(field, other);
				if (cost < bestCost)
				{
					best = other;
					bestCost = cost;
				}
			}
			if (best < 0)
				break;
			current = best;
		}
		return current >= 0 ? mesh.areas[current].centroid : from;
	}

	bool Usable(const CBot &bot, const Nav::NavMesh &mesh, const Nav::CampSpot &camp, size_t index,
		const std::vector<float> *threat, bool anyone = false)
	{
		if (!BotCamp::SpotReady(bot, static_cast<int>(index)) || BotCamp::SpotOccupied(bot, camp.pos))
			return false;
		if ((camp.flags & crossbot_nav::CAMP_DEPLOY) == 0)
			return true;
		if (anyone)
		{
			const Objectives::Sandbag *bag = Objectives::FindSandbag(camp.entIndex);
			return bag && bag->open;
		}
		if (!BotCamp::SpotAllowed(bot, camp))
			return false;
		return threat && BotCamp::FacesThreat(camp, Downhill(mesh, *threat, camp.areaId, camp.pos));
	}

	float OffAxis(const Nav::CampSpot &camp, Vector los)
	{
		float yaw = camp.yaw * NavMath::kPiF / 180.f;
		float pitch = camp.pitch * NavMath::kPiF / 180.f;
		Vector facing(cosf(pitch) * cosf(yaw), cosf(pitch) * sinf(yaw), -sinf(pitch));
		float dot = DotProduct(facing, los.Normalize());
		return acosf(std::max(-1.f, std::min(1.f, dot))) * 180.f / NavMath::kPiF;
	}

	float BoxDist(const Vector &p, const Vector &mins, const Vector &maxs)
	{
		Vector d(std::max(0.f, std::max(mins.x - p.x, p.x - maxs.x)),
			std::max(0.f, std::max(mins.y - p.y, p.y - maxs.y)),
			std::max(0.f, std::max(mins.z - p.z, p.z - maxs.z)));
		return d.Length();
	}

	bool SeesThrough(const Vector &from, const Vector &to, const Vector &mins, const Vector &maxs, bool box,
		int &clearEnt)
	{
		Vector start = from;
		edict_t *skip = nullptr;
		for (int hop = 0; hop < kOverlookBreakHops; ++hop)
		{
			TraceResult tr;
			TRACE_LINE(start, to, ignore_monsters, skip, &tr);
			if (tr.fStartSolid || tr.fAllSolid)
				return false;
			if (tr.flFraction >= 1.f || (box && BoxDist(tr.vecEndPos, mins, maxs) <= kOverlookMargin))
				return true;
			if (!tr.pHit || tr.pHit == skip || !BotBreak::IsTarget(tr.pHit))
				return false;
			if (!clearEnt)
				clearEnt = ENTINDEX(tr.pHit);
			skip = tr.pHit;
			start = tr.vecEndPos;
		}
		return false;
	}

	const std::vector<uint8_t> &SightOf(const Nav::NavMesh &mesh, edict_t *ent, const Vector &point)
	{
		int index = ENTINDEX(ent);
		auto it = g_objectSight.find(index);
		if (it != g_objectSight.end() && it->second.sees.size() == mesh.camps.size() &&
			(it->second.point - point).Length() <= kOverlookMargin)
			return it->second.sees;

		auto start = std::chrono::steady_clock::now();
		ObjectSight &sight = g_objectSight[index];
		sight.point = point;
		sight.sees.assign(mesh.camps.size(), 0);
		int seen = 0;
		for (size_t i = 0; i < mesh.camps.size(); ++i)
		{
			int clearEnt = 0;
			sight.sees[i] = SeesThrough(BotCamp::SpotEye(mesh.camps[i]), point, point, point, true, clearEnt) ? 1 : 0;
			seen += sight.sees[i];
		}
		BOT_VLOG("CrossBot: %s #%d is seen by %d of %d camp spots (%.1f ms)\n", CBotTargetNameOf(ent), index, seen,
			static_cast<int>(mesh.camps.size()),
			std::chrono::duration<float, std::milli>(std::chrono::steady_clock::now() - start).count());
		return sight.sees;
	}

	void GroupShare(const Nav::NavMesh &mesh, edict_t *goal, std::vector<float> &share)
	{
		share.clear();
		const std::vector<Objectives::Payload> &payloads = Objectives::Get().payloads;
		int group = 0;
		int id = 0;
		for (const Objectives::Payload &p : payloads)
		{
			if (p.ent.Get() == goal)
			{
				group = p.group;
				id = p.id;
			}
		}
		if (group <= 0)
			return;

		int members = 0;
		share.assign(mesh.camps.size(), 0.f);
		for (const Objectives::Payload &p : payloads)
		{
			edict_t *ent = p.ent.Get();
			if (p.group != group || p.id != id || !ent)
				continue;
			const std::vector<uint8_t> &sees = SightOf(mesh, ent, p.point);
			for (size_t i = 0; i < share.size(); ++i)
				share[i] += sees[i];
			++members;
		}
		if (members < 2)
		{
			share.clear();
			return;
		}
		for (float &s : share)
			s = std::max(1.f, s) / members;
	}

	bool Overlook(CBot &bot, const Nav::NavMesh &mesh, const std::vector<float> &fromBot, edict_t *goal,
		const Vector &watch, const std::vector<float> *toTarget, bool camper)
	{
		const std::vector<BotTactics::OverlookSpot> &spots = BotTactics::Overlooks(mesh, goal, watch);
		if (spots.empty())
			return false;

		std::vector<float> threat;
		BotTactics::EnemyField(bot, mesh, threat);
		NavPlan::RangeBias range = Bot_RangeBias(bot, false);
		std::vector<float> share;
		GroupShare(mesh, goal, share);

		bool attack = toTarget != nullptr;
		float here = attack ? At(*toTarget, bot.currentAreaId) : Nav::kUnreachableSeconds;

		std::vector<Choice> choices;
		for (size_t s = 0; s < spots.size(); ++s)
		{
			int i = spots[s].campIndex;
			const Nav::CampSpot &camp = mesh.camps[i];
			float out = At(fromBot, camp.areaId);
			bool usable = out < Nav::kUnreachableSeconds &&
				Usable(bot, mesh, camp, static_cast<size_t>(i), &threat, attack);
			bool ahead = !attack || At(*toTarget, camp.areaId) < here;
			if (BotTactics::DebugEnabled())
				BOT_VLOG("CrossBot: %s overlook spot %d at (%.0f,%.0f,%.0f) for %s: score %.1f, share %.2f, walk %.1fs, "
					"to goal %.1fs (here %.1fs), usable %d, ahead %d, breaks %d\n", bot.name, i, camp.pos.x, camp.pos.y,
					camp.pos.z, CBotTargetNameOf(goal), camp.score, share.empty() ? 1.f : share[i], out,
					attack ? At(*toTarget, camp.areaId) : 0.f, here, usable, ahead, spots[s].clearEnt);
			if (!usable || !ahead)
				continue;

			Choice c;
			c.campIndex = i;
			c.slot = static_cast<int>(s);
			c.out = out;
			c.fit = std::max(kFitFloor,
				1.f - range.Weight() * range.Mismatch((spots[s].seen - BotCamp::SpotEye(camp)).Length()));
			c.weight = Worth(bot, camp, c.fit, out) * (share.empty() ? 1.f : share[i]);
			choices.push_back(c);
		}

		int pick = Roll(choices);
		if (pick < 0)
		{
			if (BotTactics::DebugEnabled())
				BOT_VLOG("CrossBot: %s finds none of the %d spots overlooking %s free\n", bot.name,
					static_cast<int>(spots.size()), CBotTargetNameOf(goal));
			return false;
		}

		const Choice &c = choices[pick];
		const BotTactics::OverlookSpot &spot = spots[c.slot];
		const Nav::CampSpot &camp = mesh.camps[c.campIndex];
		float seenDist = (spot.seen - BotCamp::SpotEye(camp)).Length();
		float hold = !attack ? g_engfuncs.pfnRandomFloat(kDefendHoldMin, kDefendHoldMax)
			: camper ? g_engfuncs.pfnRandomFloat(0.f, kSupportHoldMax) : 0.f;
		BotCamp::CommitSnipe(bot, mesh, c.campIndex, hold, 0, spot.seen);
		bot.camp.spot.clearEnt = spot.clearEnt;

		if (attack)
			BOT_VLOG("CrossBot: %s approaches %s from overlook spot %d (%.0f away, %.1fs walk, %.1fs to the target "
				"from there, %.1fs from here, score %.1f, fit %.2f, %d spots%s), %s %.1fs\n", bot.name,
				CBotTargetNameOf(goal), c.campIndex, seenDist, c.out, At(*toTarget, camp.areaId), here, camp.score,
				c.fit, static_cast<int>(choices.size()), spot.clearEnt ? ", breaking through" : "",
				camper ? "holding" : "passing", bot.camp.spot.holdFor);
		else
			BOT_VLOG("CrossBot: %s overlooks %s from spot %d (%.0f away, %.1fs walk, score %.1f, fit %.2f, %d spots%s), "
				"holding %.1fs\n", bot.name, CBotTargetNameOf(goal), c.campIndex, seenDist, c.out, camp.score, c.fit,
				static_cast<int>(choices.size()), spot.clearEnt ? ", breaking through" : "", bot.camp.spot.holdFor);
		return true;
	}

	bool Snipe(CBot &bot, const Nav::NavMesh &mesh, const std::vector<float> &fromBot, const Vector &goalPoint)
	{
		if (CvarValue("crossbot_snipe", 1.f) == 0.f)
			return false;

		if (BotStrategic::Urgent(bot))
			return false;

		const BotGoal::Stop &target = bot.goal.target;
		edict_t *targetEnt = target.ent.Get();
		const std::vector<float> &toTarget = bot.goal.toTargetCosts;
		float here = At(toTarget, bot.currentAreaId);
		if (!targetEnt || here >= Nav::kUnreachableSeconds)
			return false;

		int weaponId = BotFire::SnipeWeapon(bot);
		if (weaponId <= 0 || BotTactics::Closest(bot))
			return false;

		Vector watch = goalPoint + Vector(0.f, 0.f, kSnipeTargetLift);

		std::vector<Choice> choices;
		for (size_t i = 0; i < mesh.camps.size(); ++i)
		{
			const Nav::CampSpot &camp = mesh.camps[i];
			float out = At(fromBot, camp.areaId);
			if (out >= Nav::kUnreachableSeconds || out > here)
				continue;

			float spotDist = BotCamp::SpotDist(camp);
			if (spotDist < kSnipeDist)
				continue;

			Vector los = watch - BotCamp::SpotEye(camp);
			float d = los.Length();
			if (d < kSnipeDist || d * kSnipeDistRatio < spotDist || d > spotDist * kSnipeDistRatio)
				continue;
			if (OffAxis(camp, los) > kSnipeCone)
				continue;
			if (!Usable(bot, mesh, camp, i, &toTarget))
				continue;

			Choice c;
			c.campIndex = static_cast<int>(i);
			c.out = out;
			c.weight = Worth(bot, camp, c.fit, out);
			choices.push_back(c);
		}

		int pick = Roll(choices);
		if (pick < 0)
		{
			if (BotTactics::DebugEnabled())
				BOT_VLOG("CrossBot: %s finds no sniping post over %s\n", bot.name, CBotTargetNameOf(targetEnt));
			return false;
		}

		const Choice &c = choices[pick];
		const Nav::CampSpot &camp = mesh.camps[c.campIndex];
		Vector los = watch - BotCamp::SpotEye(camp);
		const WeaponSpec *spec = g_pGameProfile->GetWeaponSpec(weaponId);
		BotCamp::CommitSnipe(bot, mesh, c.campIndex, g_engfuncs.pfnRandomFloat(kSnipeHoldMin, kSnipeHoldMax),
			weaponId, watch);

		BOT_VLOG("CrossBot: %s snipes %s from spot %d (%.0f away, %.0f deg off its facing, %.1fs walk, %.1fs "
			"from the target, score %.1f), holding %.1fs with %s\n", bot.name, CBotTargetNameOf(targetEnt),
			c.campIndex, los.Length(), OffAxis(camp, los), c.out, here, camp.score, bot.camp.spot.holdFor,
			spec && spec->classname ? spec->classname : "?");
		return true;
	}
}

namespace BotTactics
{

bool Enabled()
{
	cvar_t *cvar = CBotCvar("crossbot_tactics");
	return !cvar || cvar->value != 0.f;
}

bool DebugEnabled()
{
#if BOT_VLOG_ON
	cvar_t *cvar = CBotCvar("crossbot_tactics_debug");
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
	edict_t *targetEnt = target.ent.Get();
	bool defend = kind == BotStrategic::KIND_DEFEND;
	bool camper = BotFire::CampWeapon(bot);
	if (!defend && camper && Snipe(bot, mesh, fromBot, target.point))
		return true;

	bool support = kind == BotStrategic::KIND_SUPPORT;
	bool attack = !defend && kind != BotStrategic::KIND_ROAM && kind != BotStrategic::KIND_HEAL &&
		!BotStrategic::Urgent(bot);
	if ((!defend && !attack) || !targetEnt || target.areaId < 0 || bot.currentAreaId < 0)
		return false;

	std::vector<float> toTarget, fromHere;
	Nav::AreaCostsToGoal(mesh, static_cast<uint32_t>(target.areaId), toTarget);
	Nav::AreaCosts(mesh, static_cast<uint32_t>(bot.currentAreaId), fromHere);

	NavPlan::RangeBias range = Bot_RangeBias(bot, false);

	edict_t *runner = nullptr;
	float clock = 0.f;
	float runnerToTarget = Nav::kUnreachableSeconds;
	std::vector<float> runnerField;
	const std::vector<float> *from = nullptr;

	if (!bot.camp.scared && defend)
	{
		EnemyField(bot, mesh, runnerField);
		from = &runnerField;
		runnerToTarget = At(runnerField, target.areaId);
		clock = gpGlobals->time - bot.strat.spawnTime;
	}
	else if (!bot.camp.scared && support && camper)
	{
		uint32_t runnerArea = 0;
		runner = BotStrategic::Leader(bot, targetEnt);
		if (!runner)
			runner = PickFriend(bot);
		if (runner && AreaOf(mesh, runner->v.origin, runnerArea))
		{
			Nav::AreaCosts(mesh, runnerArea, runnerField);
			from = &runnerField;
			runnerToTarget = At(toTarget, static_cast<int>(runnerArea));
		}
	}

	std::vector<Choice> choices;
	float ahead = defend ? Nav::kUnreachableSeconds : At(toTarget, bot.currentAreaId);
	if (runnerToTarget < Nav::kUnreachableSeconds)
	{
		for (size_t i = 0; i < mesh.camps.size(); ++i)
		{
			const Nav::CampSpot &camp = mesh.camps[i];
			float in = At(toTarget, camp.areaId);
			float out = At(fromHere, camp.areaId);
			float r = At(*from, camp.areaId);
			if (in >= Nav::kUnreachableSeconds || out >= Nav::kUnreachableSeconds || r >= Nav::kUnreachableSeconds)
				continue;
			if (in >= runnerToTarget || in >= ahead || r + in - runnerToTarget > kBudget)
				continue;
			if (clock + out + kMargin >= r)
				continue;
			if (!Usable(bot, mesh, camp, i, defend ? from : &toTarget))
				continue;

			Choice c;
			c.campIndex = static_cast<int>(i);
			c.out = out;
			c.runner = r;
			c.fit = Fit(range, camp);
			c.weight = Worth(bot, camp, c.fit, out);
			choices.push_back(c);
		}
	}

	int pick = Roll(choices);
	if (pick < 0 && (defend || !Closest(bot)))
	{
		Vector goalPoint = target.point;
		BotStrategic::IntentPoint(bot, targetEnt, kind, goalPoint);
		if (defend)
			return Overlook(bot, mesh, fromBot, targetEnt, goalPoint, nullptr, camper);
		if (Overlook(bot, mesh, fromBot, targetEnt, goalPoint, &toTarget, camper))
			return true;
	}
	if (pick < 0)
	{
		if (DebugEnabled())
			BOT_VLOG("CrossBot: %s finds no spot ahead of %s, heading for %s\n", bot.name,
				defend ? "the enemy" : runner ? CBotPlayerNameOf(runner) : "anyone", CBotTargetNameOf(targetEnt));
		return false;
	}

	const Choice &c = choices[pick];
	const Nav::CampSpot &camp = mesh.camps[c.campIndex];
	float hold = defend ? g_engfuncs.pfnRandomFloat(kDefendHoldMin, kDefendHoldMax)
		: g_engfuncs.pfnRandomFloat(0.f, kSupportHoldMax);
	BotCamp::CommitTimed(bot, mesh, c.campIndex, hold, c.runner, c.out, clock + kMargin);

	if (defend)
		BOT_VLOG("CrossBot: %s guards %s at spot %d (%.1fs, enemy %.1fs, %.0fs into its life, score %.1f, fit %.2f), "
			"holding %.1fs\n", bot.name, CBotTargetNameOf(targetEnt), c.campIndex, c.out, c.runner, clock, camp.score,
			c.fit, bot.camp.spot.holdFor);
	else
		BOT_VLOG("CrossBot: %s supports %s toward %s, camps at spot %d (%.1fs, runner %.1fs, score %.1f, fit %.2f), "
			"holding %.1fs\n", bot.name, CBotPlayerNameOf(runner), CBotTargetNameOf(targetEnt), c.campIndex, c.out,
			c.runner, camp.score, c.fit, bot.camp.spot.holdFor);
	return true;
}

bool Closest(const CBot &bot)
{
	const Nav::NavMesh *mesh = Nav::Get();
	float here = At(bot.goal.toTargetCosts, bot.currentAreaId);
	if (!mesh || here >= Nav::kUnreachableSeconds)
		return true;

	for (int i = 1; i <= gpGlobals->maxClients; ++i)
	{
		edict_t *ent = g_engfuncs.pfnPEntityOfEntIndex(i);
		if (!LiveFriend(bot, ent))
			continue;

		uint32_t area = 0;
		if (AreaOf(*mesh, ent->v.origin, area) && At(bot.goal.toTargetCosts, static_cast<int>(area)) < here)
			return false;
	}
	return true;
}

void EnemyField(const CBot &bot, const Nav::NavMesh &mesh, std::vector<float> &out)
{
	out.assign(mesh.areas.size(), Nav::kUnreachableSeconds);

	int team = Teams::TeamOf(bot.pEdict);
	std::vector<int> teams;
	for (const Objectives::Spawn &spawn : Objectives::Get().spawns)
	{
		if (spawn.team != team && std::find(teams.begin(), teams.end(), spawn.team) == teams.end())
			teams.push_back(spawn.team);
	}

	for (int enemy : teams)
	{
		const std::vector<float> &field = TeamField(mesh, enemy);
		for (size_t i = 0; i < out.size() && i < field.size(); ++i)
			out[i] = std::min(out[i], field[i]);
	}
}

const std::vector<OverlookSpot> &Overlooks(const Nav::NavMesh &mesh, edict_t *goal, const Vector &point)
{
	int index = ENTINDEX(goal);
	auto it = g_overlooks.find(index);
	if (it != g_overlooks.end() && (it->second.point - point).Length() <= kOverlookMargin)
		return it->second.spots;

	OverlookSet &set = g_overlooks[index];
	set.point = point;
	std::vector<OverlookSpot> &out = set.spots;
	out.clear();
	Vector mins = point;
	Vector maxs = point;
	const char *model = goal->v.model ? STRING(goal->v.model) : nullptr;
	if (model && model[0] == '*')
	{
		mins = goal->v.absmin;
		maxs = goal->v.absmax;
	}

	std::vector<Vector> samples(1, point);
	const Nav::Area *goalArea = Nav::FindAreaNearest(mesh, point);
	if (goalArea)
	{
		std::vector<float> field;
		Nav::AreaCostsToGoal(mesh, goalArea->id, field);
		std::vector<std::pair<float, uint32_t>> closeAreas;
		for (size_t i = 0; i < field.size(); ++i)
		{
			if (i != goalArea->id && field[i] <= kOverlookReach)
				closeAreas.push_back(std::make_pair(field[i], static_cast<uint32_t>(i)));
		}
		std::sort(closeAreas.begin(), closeAreas.end());
		if (closeAreas.size() > kOverlookSamples)
			closeAreas.resize(kOverlookSamples);
		for (const std::pair<float, uint32_t> &n : closeAreas)
		{
			const Nav::Area &area = mesh.areas[n.second];
			samples.push_back(Vector(area.centroid.x, area.centroid.y,
				Nav::AreaFloorZ(area, area.centroid.x, area.centroid.y) + kOverlookChest));
		}
	}

	int atPoint = 0, nearby = 0, broken = 0;
	for (size_t i = 0; i < mesh.camps.size(); ++i)
	{
		Vector eye = BotCamp::SpotEye(mesh.camps[i]);
		for (size_t s = 0; s < samples.size(); ++s)
		{
			int clearEnt = 0;
			if (!SeesThrough(eye, samples[s], mins, maxs, s == 0, clearEnt))
				continue;

			OverlookSpot spot;
			spot.campIndex = static_cast<int>(i);
			spot.seen = samples[s];
			spot.clearEnt = clearEnt;
			out.push_back(spot);
			++(s == 0 ? atPoint : nearby);
			if (clearEnt)
				++broken;
			break;
		}
	}

	BOT_VLOG("CrossBot: %s #%d is overlooked by %d of %d camp spots (%d see it, %d see within %.0fs of it, "
		"%d through a breakable)\n", CBotTargetNameOf(goal), index, static_cast<int>(out.size()),
		static_cast<int>(mesh.camps.size()), atPoint, nearby, kOverlookReach, broken);
	return out;
}

void ForgetMap()
{
	g_teamFields.clear();
	g_overlooks.clear();
	g_objectSight.clear();
}

}
