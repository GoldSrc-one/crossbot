#include <extdll.h>
#include <meta_api.h>
#include <in_buttons.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

#include "../game/objectives.h"
#include "../game/teams.h"
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
	constexpr float kSpawnJump = 128.f;
	constexpr float kSpawnNear = 64.f;
	constexpr float kUseStandoff = 40.f;
	constexpr float kGuardStandoff = 96.f;
	constexpr float kGoalFailCooldown = 10.f;
	constexpr float kHoldLimit = 30.f;
	constexpr float kHoldCooldown = 20.f;
	constexpr float kRoamCooldown = 60.f;
	constexpr float kUsePulse = 0.2f;

	constexpr int kRankShift = 8;
	constexpr int kKindMask = 0xFF;
	constexpr int kRankDeliver = 3;
	constexpr int kRankTake = 2;
	constexpr int kRankSupport = 1;
	constexpr int kRankIdle = 0;

	using BotStrategic::Intent;

	int RankOf(int payload)
	{
		return payload >> kRankShift;
	}

	bool OwnedBy(const edict_t *ent, const edict_t *pEdict)
	{
		return !ent->v.owner || ent->v.owner == pEdict;
	}

	bool MapGoalFor(const CBot &bot, edict_t *ent)
	{
		if (!BotStrategic::IsGoal(ent) || !OwnedBy(ent, bot.pEdict))
			return false;
		return !ent->v.owner || !Objectives::Active();
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

	struct Held
	{
		int id = 0;
		int group = 0;
	};

	bool Matches(int id, int group, int needId, int needGroup)
	{
		if (needGroup == Objectives::kAnyGroup)
			return true;
		if (needId != 0 && id == needId)
			return true;
		return needGroup > 0 && group == needGroup;
	}

	Intent &Push(std::vector<Intent> &out, edict_t *ent, const Vector &point, bool volume, int kind, int rank)
	{
		Intent intent;
		intent.ent = ent;
		intent.point = point;
		intent.volume = volume;
		intent.kind = kind;
		intent.rank = rank;
		out.push_back(intent);
		return out.back();
	}

	int TeamAlive(int team)
	{
		int count = 0;
		for (int i = 1; i <= gpGlobals->maxClients; ++i)
		{
			edict_t *ent = g_engfuncs.pfnPEntityOfEntIndex(i);
			if (ent && CBotIsLivePlayer(ent) && Teams::TeamOf(ent) == team)
				++count;
		}
		return count;
	}

	bool FindIntent(const CBot &bot, edict_t *ent, int kind, Intent &found)
	{
		std::vector<Intent> intents;
		BotStrategic::Intents(bot, intents);
		for (const Intent &intent : intents)
		{
			if (intent.ent == ent && intent.kind == kind)
			{
				found = intent;
				return true;
			}
		}
		return false;
	}

	float Standoff(const Intent &intent)
	{
		if (intent.volume)
			return 0.f;
		if (intent.kind == BotStrategic::KIND_FETCH && intent.buttons != 0)
			return kUseStandoff;
		if (intent.kind == BotStrategic::KIND_DEFEND)
			return kGuardStandoff;
		return 0.f;
	}

	bool PlaceShort(const CBot &bot, const Nav::NavMesh &mesh, const Intent &intent, BotItems::Placement &place)
	{
		float standoff = Standoff(intent);
		if (standoff > 0.f)
		{
			Vector away = bot.pEdict->v.origin - intent.point;
			away.z = 0.f;
			float len = away.Length();
			if (len > standoff &&
				BotStrategic::ResolvePoint(mesh, intent.point + away * (standoff / len), place.areaId, place.point))
				return true;
		}
		return BotStrategic::ResolvePoint(mesh, intent.point, place.areaId, place.point);
	}

	bool HoldLimited(int kind)
	{
		return kind == BotStrategic::KIND_ATTACK || kind == BotStrategic::KIND_DELIVER ||
			kind == BotStrategic::KIND_FETCH || kind == BotStrategic::KIND_DEFEND || kind == BotStrategic::KIND_SUPPORT;
	}

	void StartLife(CBot &bot)
	{
		BotStrategic::State &st = bot.strat;
		st.spawnTime = gpGlobals->time;
		st.defender = g_engfuncs.pfnRandomLong(0, 1) != 0;
		st.lastOrigin = bot.pEdict->v.origin;

		if (Objectives::Active())
			BOT_VLOG("CrossBot: %s starts a life as %s\n", bot.name, st.defender ? "a defender" : "an attacker");
	}

	bool NearOwnSpawn(const CBot &bot)
	{
		if (!Objectives::Active())
			return false;

		int team = Teams::TeamOf(bot.pEdict);
		for (const Objectives::Spawn &spawn : Objectives::Get().spawns)
		{
			edict_t *ent = spawn.ent.Get();
			if (ent && spawn.team == team && (ent->v.origin - bot.pEdict->v.origin).Length() <= kSpawnNear)
				return true;
		}
		return false;
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

bool ValidFor(const CBot &bot, edict_t *ent, int payload)
{
	if (!ent || ent->free)
		return false;

	if (KindOf(payload) == KIND_MAP)
		return MapGoalFor(bot, ent);

	Intent found;
	return FindIntent(bot, ent, KindOf(payload), found);
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

void Intents(const CBot &bot, std::vector<Intent> &out)
{
	out.clear();
	if (!Enabled() || !Objectives::Active())
		return;

	int team = Teams::TeamOf(bot.pEdict);
	int bit = Objectives::TeamBit(team);
	if (!bit)
		return;

	const Objectives::Tables &tables = Objectives::Get();

	std::vector<Held> held;
	int sharedMine = 0;
	int sharedTotal = 0;
	for (const Objectives::Carrier &carrier : tables.carriers)
	{
		if (carrier.shared)
			++sharedTotal;
		if (!carrier.player.Is(bot.pEdict))
			continue;

		Held h;
		h.id = carrier.id;
		h.group = carrier.group;
		held.push_back(h);
		if (carrier.shared)
			++sharedMine;
	}

	for (const Objectives::Payload &payload : tables.payloads)
	{
		if (payload.shared && payload.ent.Get())
			++sharedTotal;
	}

	auto meets = [&](const Objectives::Site &site) {
		for (const Held &h : held)
		{
			if (Matches(h.id, h.group, site.needId, site.needGroup))
				return true;
		}
		return false;
	};

	auto loose = [&](const Objectives::Site &site) {
		for (const Objectives::Payload &payload : tables.payloads)
		{
			if ((payload.carryTeams & bit) != 0 && payload.ent.Get() &&
				Matches(payload.id, payload.group, site.needId, site.needGroup))
				return true;
		}
		return false;
	};

	for (const Objectives::Site &site : tables.sites)
	{
		edict_t *ent = site.ent.Get();
		if (!ent)
			continue;

		if ((site.attackTeams & bit) != 0)
		{
			bool needs = site.needId != 0 || site.needGroup != 0;
			Intent *intent = nullptr;
			if (!needs && site.urgent)
				intent = &Push(out, ent, site.point, site.volume, KIND_ATTACK, kRankDeliver);
			else if (needs && meets(site))
				intent = &Push(out, ent, site.point, site.volume, KIND_DELIVER, kRankDeliver);
			else if (!needs || !loose(site))
				intent = &Push(out, ent, site.point, site.volume, KIND_SUPPORT, kRankSupport);

			if (intent)
			{
				intent->aim = site.aim.Get();
				intent->buttons = site.buttons;
			}
		}
		else if ((site.defendTeams & bit) != 0)
		{
			Push(out, ent, site.point, site.volume, KIND_DEFEND, site.urgent ? kRankTake : kRankIdle);
		}
	}

	int cap = std::max(1, sharedTotal / std::max(1, TeamAlive(team)));
	for (const Objectives::Payload &payload : tables.payloads)
	{
		edict_t *ent = payload.ent.Get();
		if (!ent)
			continue;

		if ((payload.carryTeams & bit) != 0)
		{
			if (payload.shared ? sharedMine >= cap : !held.empty())
				continue;

			Intent &intent = Push(out, ent, ent->v.origin, false, KIND_FETCH, kRankTake);
			intent.aim = payload.buttons ? ent : nullptr;
			intent.buttons = payload.buttons;
			intent.pulse = payload.shared;
		}
		else if ((payload.defendTeams & bit) != 0)
		{
			Push(out, ent, ent->v.origin, false, KIND_DEFEND, kRankSupport);
		}
	}

	bool support = false;
	bool defend = false;
	for (const Intent &intent : out)
	{
		support = support || (intent.kind == KIND_SUPPORT && intent.rank <= kRankSupport);
		defend = defend || (intent.kind == KIND_DEFEND && intent.rank <= kRankSupport);
	}
	if (support && defend)
	{
		int drop = bot.strat.defender ? KIND_SUPPORT : KIND_DEFEND;
		out.erase(std::remove_if(out.begin(), out.end(), [drop](const Intent &intent) {
			return intent.kind == drop && intent.rank <= kRankSupport;
		}), out.end());
	}

	if (!out.empty())
		return;

	for (const Objectives::Spawn &spawn : tables.spawns)
	{
		if (edict_t *ent = spawn.ent.Get())
			Push(out, ent, ent->v.origin, false, KIND_ROAM, kRankIdle);
	}
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

		if (!MapGoalFor(bot, ent))
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
		cand.payload = KIND_MAP | (kRankTake << kRankShift);
		out.push_back(cand);
	}

	std::vector<Intent> intents;
	Intents(bot, intents);

	std::vector<BotItems::Placement> places;
	for (const Intent &intent : intents)
	{
		if (MemoReady(bot, intent.ent) > gpGlobals->time)
			continue;

		places.clear();
		if (intent.volume)
			BotItems::VolumeAreas(mesh, intent.ent, bot.pEdict->v.origin, places);

		if (places.empty())
		{
			BotItems::Placement place;
			if (!PlaceShort(bot, mesh, intent, place))
			{
				if (DebugEnabled())
					BOT_VLOG("CrossBot: %s cannot place the %s objective %s in any nav area\n", bot.name,
						KindName(intent.kind), CBotTargetNameOf(intent.ent));
				continue;
			}
			places.push_back(place);
		}

		for (const BotItems::Placement &place : places)
		{
			BotGoal::Candidate cand;
			cand.tier = BotGoal::GOAL_STRATEGIC;
			cand.ent = intent.ent;
			cand.areaId = place.areaId;
			cand.point = place.point;
			cand.payload = intent.kind | (intent.rank << kRankShift);
			out.push_back(cand);
		}
	}
}

void Prioritize(std::vector<BotGoal::Candidate> &cands)
{
	int best = -1;
	for (const BotGoal::Candidate &cand : cands)
		best = std::max(best, RankOf(cand.payload));

	cands.erase(std::remove_if(cands.begin(), cands.end(),
		[best](const BotGoal::Candidate &cand) { return RankOf(cand.payload) != best; }), cands.end());
}

float MoveRadius(int)
{
	return kGoalMoveRadius;
}

const char *KindName(int payload)
{
	switch (KindOf(payload))
	{
	case KIND_MAP: return "bot_goal";
	case KIND_ATTACK: return "attack";
	case KIND_DELIVER: return "deliver";
	case KIND_FETCH: return "fetch";
	case KIND_SUPPORT: return "support";
	case KIND_DEFEND: return "defend";
	case KIND_ROAM: return "roam";
	default: break;
	}
	return "?";
}

void Accept(CBot &bot, const BotGoal::Candidate &cand)
{
	State &st = bot.strat;
	st.goal.Set(cand.ent);
	st.payload = cand.payload;
	st.aim.Clear();
	st.buttons = 0;
	st.pulse = false;
	st.arrival = false;
	st.pressed = false;
	st.startTime = 0.f;

	Intent found;
	if (KindOf(cand.payload) != KIND_MAP && FindIntent(bot, cand.ent, KindOf(cand.payload), found))
	{
		st.aim.Set(found.aim);
		st.buttons = found.buttons;
		st.pulse = found.pulse;
	}
}

void Forget(CBot &bot)
{
	State &st = bot.strat;
	st.goal.Clear();
	st.payload = 0;
	st.aim.Clear();
	st.buttons = 0;
	st.pulse = false;
	st.arrival = false;
	st.pressed = false;
	st.startTime = 0.f;
}

bool RunArrival(CBot &bot)
{
	State &st = bot.strat;
	edict_t *pEdict = bot.pEdict;
	edict_t *ent = st.goal.Get();
	int kind = KindOf(st.payload);

	if (!ValidFor(bot, ent, st.payload))
		return false;

	if ((ent->v.origin - bot.goal.targetOrigin).Length() > MoveRadius(st.payload))
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

		BOT_VLOG("CrossBot: %s is holding the %s goal %s at (%.1f,%.1f,%.1f)\n", bot.name, KindName(kind),
			CBotTargetNameOf(ent), bot.goalPoint.x, bot.goalPoint.y, bot.goalPoint.z);
	}

	if (kind == KIND_ROAM)
	{
		NoteCooldown(bot, ent, kRoamCooldown);
		return false;
	}

	if (HoldLimited(kind) && gpGlobals->time - st.startTime > kHoldLimit)
	{
		BOT_VLOG("CrossBot: %s has held the %s goal %s for %.0fs, moving on\n", bot.name, KindName(kind),
			CBotTargetNameOf(ent), kHoldLimit);
		NoteCooldown(bot, ent, kHoldCooldown);
		return false;
	}

	bot.lastProgressTime = gpGlobals->time;

	edict_t *aim = (kind == KIND_MAP) ? AimEntity(ent) : st.aim.Get();
	int want = (kind == KIND_MAP) ? ent->v.button : st.buttons;
	if (st.pulse && fmodf(gpGlobals->time - st.startTime, 2.f * kUsePulse) >= kUsePulse)
		want &= ~IN_USE;

	unsigned short buttons = 0;
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

			if (dist <= kGoalDuckRange && (want & IN_DUCK) != 0)
				buttons |= IN_DUCK;
			if (dist <= kGoalUseRange && (want & IN_USE) != 0)
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
		BOT_VLOG("CrossBot: %s could not plan a route to the strategic goal %s, leaving it for %.0fs\n",
			bot.name, CBotTargetNameOf(ent), kGoalFailCooldown);

		NoteCooldown(bot, ent, kGoalFailCooldown);
	}

	Forget(bot);
}

void OnBlocked(CBot &bot, const char *what)
{
	edict_t *ent = bot.strat.goal.Get();
	if (ent)
	{
		BOT_VLOG("CrossBot: %s could not get past %s reaching the strategic goal %s, leaving it for %.0fs\n",
			bot.name, what, CBotTargetNameOf(ent), kGoalFailCooldown);

		NoteCooldown(bot, ent, kGoalFailCooldown);
	}

	Forget(bot);
}

void OnRespawn(CBot &bot)
{
	bot.strat.memo.clear();
	StartLife(bot);
	Forget(bot);
}

void NoteLife(CBot &bot)
{
	State &st = bot.strat;
	Vector origin = bot.pEdict->v.origin;
	bool jumped = (origin - st.lastOrigin).Length() > kSpawnJump;
	st.lastOrigin = origin;
	if (st.spawnTime > 0.f && !(jumped && NearOwnSpawn(bot)))
		return;

	StartLife(bot);
}

int KindOf(int payload)
{
	return payload & kKindMask;
}

edict_t *Leader(const CBot &bot, edict_t *siteEnt)
{
	if (!Objectives::Active())
		return nullptr;

	const Objectives::Tables &tables = Objectives::Get();
	const Objectives::Site *site = nullptr;
	for (const Objectives::Site &s : tables.sites)
	{
		if (s.ent.Is(siteEnt))
		{
			site = &s;
			break;
		}
	}
	if (!site || (site->needId == 0 && site->needGroup == 0))
		return nullptr;

	int team = Teams::TeamOf(bot.pEdict);
	edict_t *best = nullptr;
	float bestDist = 0.f;
	for (const Objectives::Carrier &carrier : tables.carriers)
	{
		edict_t *player = carrier.player.Get();
		if (!player || player == bot.pEdict || carrier.team != team || !CBotIsLivePlayer(player))
			continue;
		if (!Matches(carrier.id, carrier.group, site->needId, site->needGroup))
			continue;

		float d = (player->v.origin - bot.pEdict->v.origin).Length();
		if (!best || d < bestDist)
		{
			best = player;
			bestDist = d;
		}
	}
	return best;
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

void PrintIntents(const CBot &bot)
{
	std::vector<Intent> intents;
	Intents(bot, intents);

	char msg[320];
	snprintf(msg, sizeof(msg), "CrossBot: %s on team %d has %d objective intents\n", bot.name,
		Teams::TeamOf(bot.pEdict), static_cast<int>(intents.size()));
	SERVER_PRINT(msg);

	for (const Intent &intent : intents)
	{
		snprintf(msg, sizeof(msg), "CrossBot:   %s rank %d %s #%d at (%.0f,%.0f,%.0f)%s%s\n", KindName(intent.kind),
			intent.rank, CBotTargetNameOf(intent.ent), g_engfuncs.pfnIndexOfEdict(intent.ent), intent.point.x,
			intent.point.y, intent.point.z, intent.volume ? " volume" : "", (intent.buttons & IN_USE) ? " USE" : "");
		SERVER_PRINT(msg);
	}
}

}
