#include <extdll.h>
#include <meta_api.h>

#include <algorithm>
#include <string.h>

#include "../game/game_profile.h"
#include "../nav/astar_graph.h"
#include "../nav/nav.h"
#include "../nav/nav_path.h"
#include "../nav/nav_query.h"
#include "bot.h"
#include "bot_items.h"
#include "bot_move.h"
#include "bot_move_nav.h"
#include "bot_sense.h"

extern IGameProfile *g_pGameProfile;

namespace
{
	constexpr float kItemAreaLift = 18.f;
	constexpr float kChargerUseRadius = 48.f;
	constexpr float kChargerSpotSearch = 96.f;
	constexpr float kGoalItemSeenInterval = 0.25f;
	constexpr float kSelectRetryInterval = 0.5f;
	constexpr float kMinReadiness = 0.01f;

	float AxisDistance(float value, float lo, float hi)
	{
		if (value < lo)
			return lo - value;
		if (value > hi)
			return value - hi;
		return 0.f;
	}

	float BoxDistance(const Vector &point, const Vector &mins, const Vector &maxs)
	{
		float dx = AxisDistance(point.x, mins.x, maxs.x);
		float dy = AxisDistance(point.y, mins.y, maxs.y);
		float dz = AxisDistance(point.z, mins.z, maxs.z);
		return sqrtf(dx * dx + dy * dy + dz * dz);
	}

	Vector ClampIntoArea(const Nav::Area &area, const Vector &point)
	{
		Vector out = point;
		out.x = std::max(area.mins.x, std::min(area.maxs.x, out.x));
		out.y = std::max(area.mins.y, std::min(area.maxs.y, out.y));
		out.z = std::max(area.mins.z, std::min(area.maxs.z, out.z));
		return out;
	}

	const Nav::Area *ResolvePickupArea(const Nav::NavMesh &mesh, const Vector &origin)
	{
		if (const Nav::Area *lifted =
			Nav::FindAreaExact(mesh, origin + Vector(0.f, 0.f, kItemAreaLift)))
			return lifted;
		return Nav::FindAreaExact(mesh, origin);
	}

	const Nav::Area *ResolveChargerArea(const Nav::NavMesh &mesh, const Vector &center)
	{
		const Nav::Area *best = nullptr;
		float bestDist = 0.f;

		for (const Nav::Area &area : mesh.areas)
		{
			float dist = BoxDistance(center, area.mins, area.maxs);
			if (dist > kChargerSpotSearch)
				continue;

			Vector spot = ClampIntoArea(area, center);
			float useDist = (spot - center).Length();
			if (useDist > kChargerUseRadius)
				continue;

			if (!best || useDist < bestDist)
			{
				best = &area;
				bestDist = useDist;
			}
		}

		return best;
	}

	const Nav::Area *ResolveItemArea(const Nav::NavMesh &mesh, edict_t *ent, ItemKind kind,
		Vector &point)
	{
		if (!ItemKindIsCharger(kind))
		{
			point = ent->v.origin;
			return ResolvePickupArea(mesh, ent->v.origin);
		}

		Vector center = BotItems::ItemCenter(ent, kind);
		const Nav::Area *area = ResolveChargerArea(mesh, center);
		point = area ? ClampIntoArea(*area, center) : center;
		return area;
	}

	enum ScanReason
	{
		SCAN_OK = 0,
		SCAN_GONE,
		SCAN_RESPAWNING,
		SCAN_IGNORED,
		SCAN_FULL,
		SCAN_NO_AREA,
	};

	struct Candidate
	{
		edict_t *ent = nullptr;
		int index = 0;
		ItemKind kind = ITEM_NONE;
		uint32_t areaId = 0;
		bool hasArea = false;
		bool isEnemy = false;
		Vector point;
		ScanReason reason = SCAN_OK;
		float seconds = 0.f;
	};

	const BotItemMemo *MemoFor(const CBot &bot, edict_t *ent)
	{
		for (const BotItemMemo &memo : bot.itemMemo)
			if (memo.ent.Is(ent) && memo.readyTime > gpGlobals->time)
				return &memo;
		return nullptr;
	}

	const BotFullMemo *FullFor(const CBot &bot, const char *classname)
	{
		for (const BotFullMemo &memo : bot.fullMemo)
			if (memo.expireTime > gpGlobals->time && strcmp(memo.classname, classname) == 0)
				return &memo;
		return nullptr;
	}

	void PruneMemos(CBot &bot)
	{
		bot.itemMemo.erase(std::remove_if(bot.itemMemo.begin(), bot.itemMemo.end(),
			[](const BotItemMemo &m) {
				return m.readyTime <= gpGlobals->time || m.ent.Get() == nullptr;
			}), bot.itemMemo.end());

		bot.fullMemo.erase(std::remove_if(bot.fullMemo.begin(), bot.fullMemo.end(),
			[](const BotFullMemo &m) { return m.expireTime <= gpGlobals->time; }),
			bot.fullMemo.end());
	}

	void ScanItems(const CBot &bot, const Nav::NavMesh &mesh, std::vector<Candidate> &out)
	{
		for (int i = gpGlobals->maxClients + 1; i < gpGlobals->maxEntities; ++i)
		{
			edict_t *ent = g_engfuncs.pfnPEntityOfEntIndex(i);
			if (!ent || ent->free)
				continue;

			const char *classname = CBotClassOf(ent);
			ItemKind kind = g_pGameProfile->ClassifyItem(classname);
			if (kind == ITEM_NONE)
				continue;

			Candidate cand;
			cand.ent = ent;
			cand.index = i;
			cand.kind = kind;

			const Nav::Area *area = ResolveItemArea(mesh, ent, kind, cand.point);
			cand.hasArea = area != nullptr;
			if (area)
				cand.areaId = area->id;

			if (BotItems::IsUnavailable(ent, kind))
				cand.reason = SCAN_GONE;
			else if (const BotItemMemo *memo = MemoFor(bot, ent))
			{
				cand.reason = memo->respawning ? SCAN_RESPAWNING : SCAN_IGNORED;
				cand.seconds = memo->readyTime - gpGlobals->time;
			}
			else if (const BotFullMemo *full = FullFor(bot, classname))
			{
				cand.reason = SCAN_FULL;
				cand.seconds = full->expireTime - gpGlobals->time;
			}
			else if (!area)
				cand.reason = SCAN_NO_AREA;

			out.push_back(cand);
		}
	}

	float Readiness(const CBot &bot)
	{
		if (!g_pGameProfile)
			return 0.f;
		return g_pGameProfile->EvaluateCombat(bot.pEdict, bot.inventory);
	}

	void AppendEnemy(const CBot &bot, const Nav::NavMesh &mesh, std::vector<Candidate> &out)
	{
		if (!BotItems::ChaseEnabled())
			return;

		edict_t *enemy = bot.sense.enemy.Get();
		if (!enemy)
			return;

		Candidate cand;
		cand.ent = enemy;
		cand.index = g_engfuncs.pfnIndexOfEdict(enemy);
		cand.isEnemy = true;
		cand.point = bot.sense.enemyPos;

		const Nav::Area *area = ResolvePickupArea(mesh, bot.sense.enemyPos);
		cand.hasArea = area != nullptr;
		if (area)
			cand.areaId = area->id;
		else
			cand.reason = SCAN_NO_AREA;

		out.push_back(cand);
	}

	float ItemCost(const CBot &bot, const Nav::NavMesh &mesh, const std::vector<float> &costs,
		const Candidate &cand)
	{
		if (cand.areaId >= costs.size() || costs[cand.areaId] >= Nav::kUnreachableSeconds)
			return Nav::kUnreachableSeconds;

		Vector from = (static_cast<int>(cand.areaId) == bot.currentAreaId)
			? bot.pEdict->v.origin : mesh.areas[cand.areaId].centroid;

		float cost = costs[cand.areaId] + (cand.point - from).Length() / NavAStar::kNavRunSpeed;

		if (!cand.isEnemy)
			return cost;

		float readiness = Readiness(bot);
		if (readiness < kMinReadiness)
			return Nav::kUnreachableSeconds;

		return cost / readiness;
	}

	bool DebugEnabled()
	{
		cvar_t *cvar = g_engfuncs.pfnCVarGetPointer("crossbot_gather_debug");
		return cvar && cvar->value != 0.f;
	}

	void DumpScan(const CBot &bot, const Nav::NavMesh &mesh, const std::vector<float> &costs,
		const std::vector<Candidate> &items, const Candidate *picked)
	{
		size_t open = 0;
		for (const Candidate &cand : items)
			if (cand.reason == SCAN_OK)
				++open;

		char msg[256];
		snprintf(msg, sizeof(msg),
			"CrossBot: %s scanning from area %d: %zu items, %zu selectable, readiness %.2f\n",
			bot.name, bot.currentAreaId, items.size(), open, Readiness(bot));
		SERVER_PRINT(msg);

		for (const Candidate &cand : items)
		{
			char where[48];
			char why[64];

			if (cand.hasArea)
				snprintf(where, sizeof(where), "area %u", cand.areaId);
			else
				snprintf(where, sizeof(where), "area none");

			switch (cand.reason)
			{
			case SCAN_GONE:
				snprintf(why, sizeof(why), "%s", ItemKindIsCharger(cand.kind) ? "empty" : "taken");
				break;
			case SCAN_RESPAWNING:
				snprintf(why, sizeof(why), "respawning for %.0fs", cand.seconds);
				break;
			case SCAN_IGNORED:
				snprintf(why, sizeof(why), "ignored for %.0fs", cand.seconds);
				break;
			case SCAN_FULL:
				snprintf(why, sizeof(why), "full for %.0fs", cand.seconds);
				break;
			case SCAN_NO_AREA:
				snprintf(why, sizeof(why), "no nav area");
				break;
			default:
			{
				float cost = ItemCost(bot, mesh, costs, cand);
				if (cost >= Nav::kUnreachableSeconds)
					snprintf(why, sizeof(why), "no route");
				else
					snprintf(why, sizeof(why), "%.1fs away%s", cost,
						picked && picked->ent == cand.ent ? " PICKED" : "");
				break;
			}
			}

			snprintf(msg, sizeof(msg), "CrossBot:   #%d %s%s at (%.0f,%.0f,%.0f) %s, %s\n", cand.index,
				cand.isEnemy ? "ENEMY " : "", CBotClassOf(cand.ent), cand.point.x, cand.point.y,
				cand.point.z, where, why);
			SERVER_PRINT(msg);
		}
	}
}

namespace BotItems
{

bool IsPickupGone(edict_t *ent)
{
	if (!ent || ent->free)
		return true;
	return ent->v.solid == SOLID_NOT || (ent->v.effects & EF_NODRAW) != 0 ||
		(ent->v.flags & FL_KILLME) != 0;
}

bool IsChargerEmpty(edict_t *ent)
{
	return !ent || ent->free || ent->v.frame != 0.f;
}

bool IsUnavailable(edict_t *ent, ItemKind kind)
{
	return ItemKindIsCharger(kind) ? IsChargerEmpty(ent) : IsPickupGone(ent);
}

Vector ItemCenter(edict_t *ent, ItemKind kind)
{
	if (!ent)
		return Vector(0.f, 0.f, 0.f);
	if (!ItemKindIsCharger(kind))
		return ent->v.origin;
	return (ent->v.absmin + ent->v.absmax) * 0.5f;
}

void NoteMemo(CBot &bot, edict_t *ent, float seconds, bool respawning)
{
	if (!ent || ent->free)
		return;

	float ready = gpGlobals->time + seconds;
	for (BotItemMemo &memo : bot.itemMemo)
	{
		if (memo.ent.Is(ent))
		{
			memo.readyTime = std::max(memo.readyTime, ready);
			memo.respawning = respawning;
			return;
		}
	}

	BotItemMemo memo;
	memo.ent.Set(ent);
	memo.kind = bot.goalItem.Is(ent) ? bot.goalItemKind : g_pGameProfile->ClassifyItem(CBotClassOf(ent));
	memo.areaId = bot.goalItem.Is(ent) ? bot.goalItemArea : bot.currentAreaId;
	memo.readyTime = ready;
	memo.respawning = respawning;
	bot.itemMemo.push_back(memo);
}

void NoteIgnored(CBot &bot, edict_t *ent, float seconds)
{
	NoteMemo(bot, ent, seconds, false);
}

void NoteRespawning(CBot &bot, edict_t *ent, ItemKind kind)
{
	NoteMemo(bot, ent, g_pGameProfile->ItemRespawnDelay(kind), true);
}

void RefreshMemos(CBot &bot)
{
	if (bot.currentAreaId < 0)
		return;

	for (size_t i = 0; i < bot.itemMemo.size();)
	{
		const BotItemMemo &memo = bot.itemMemo[i];
		bool here = memo.respawning && memo.areaId == bot.currentAreaId;
		edict_t *ent = here ? memo.ent.Get() : nullptr;
		if (!ent || IsUnavailable(ent, memo.kind))
		{
			++i;
			continue;
		}

		char msg[192];
		snprintf(msg, sizeof(msg), "CrossBot: %s finds %s back already, forgetting it was taken\n",
			bot.name, CBotClassOf(ent));
		SERVER_PRINT(msg);

		bot.itemMemo.erase(bot.itemMemo.begin() + i);
	}
}

void NoteFull(CBot &bot, const char *classname)
{
	if (!classname || !classname[0])
		return;

	float expire = gpGlobals->time + g_pGameProfile->ItemFullTimeout();
	for (BotFullMemo &memo : bot.fullMemo)
	{
		if (strcmp(memo.classname, classname) == 0)
		{
			memo.expireTime = std::max(memo.expireTime, expire);
			return;
		}
	}

	BotFullMemo memo;
	strncpy(memo.classname, classname, sizeof(memo.classname) - 1);
	memo.classname[sizeof(memo.classname) - 1] = '\0';
	memo.expireTime = expire;
	bot.fullMemo.push_back(memo);
}

void ForgetGoalItem(CBot &bot)
{
	bot.goalIsEnemy = false;
	bot.goalItem.Clear();
	bot.goalItemKind = ITEM_NONE;
	bot.goalItemArea = -1;
	bot.goalItemTouched = false;
	bot.goalItemDone = false;
	bot.goalItemSeenCheck = 0.f;
	bot.chargerActive = false;
	bot.chargerPressed = false;
}

void ClearGoalItem(CBot &bot)
{
	ForgetGoalItem(bot);
	bot.hasGoal = false;
	Bot_ResetNavPath(bot);
}

bool Enabled()
{
	cvar_t *cvar = g_engfuncs.pfnCVarGetPointer("crossbot_gather");
	return !cvar || cvar->value != 0.f;
}

bool ChaseEnabled()
{
	cvar_t *cvar = g_engfuncs.pfnCVarGetPointer("crossbot_chase");
	return !cvar || cvar->value != 0.f;
}

bool SelectGoal(CBot &bot, const Nav::NavMesh &mesh)
{
	if (!g_pGameProfile || bot.currentAreaId < 0 || !Enabled())
		return false;

	if (gpGlobals->time < bot.nextSelectTime)
		return false;

	PruneMemos(bot);

	std::vector<Candidate> items;
	ScanItems(bot, mesh, items);
	AppendEnemy(bot, mesh, items);

	std::vector<float> costs;
	Nav::AreaCosts(mesh, static_cast<uint32_t>(bot.currentAreaId), costs,
		Bot_GravityMultiplier(bot.pEdict), Bot_Carefulness(bot));

	const Candidate *best = nullptr;
	float bestCost = 0.f;
	for (const Candidate &cand : items)
	{
		if (cand.reason != SCAN_OK)
			continue;

		float cost = ItemCost(bot, mesh, costs, cand);
		if (cost >= Nav::kUnreachableSeconds)
			continue;

		if (!best || cost < bestCost)
		{
			best = &cand;
			bestCost = cost;
		}
	}

	if (DebugEnabled())
		DumpScan(bot, mesh, costs, items, best);

	if (!best)
	{
		bot.nextSelectTime = gpGlobals->time + kSelectRetryInterval;
		return false;
	}

	ForgetGoalItem(bot);

	if (!best->isEnemy)
	{
		bot.goalItem.Set(best->ent);
		bot.goalItemKind = best->kind;
		bot.goalItemArea = static_cast<int>(best->areaId);
		bot.goalItemSelectTime = gpGlobals->time;
	}

	bot.goalIsEnemy = best->isEnemy;
	bot.goalPoint = best->point;
	bot.hasGoal = true;

	size_t open = 0;
	for (const Candidate &cand : items)
		if (cand.reason == SCAN_OK)
			++open;

	char msg[224];
	snprintf(msg, sizeof(msg),
		"CrossBot: %s wants %s%s at (%.1f,%.1f,%.1f) in area %u, %.1fs away, %zu candidates\n",
		bot.name, best->isEnemy ? "enemy " : "", CBotClassOf(best->ent), best->point.x,
		best->point.y, best->point.z, best->areaId, bestCost, open);
	SERVER_PRINT(msg);

	return true;
}

bool UpdateGoalItem(CBot &bot)
{
	if (bot.goalItemDone)
	{
		ClearGoalItem(bot);
		return true;
	}

	edict_t *ent = bot.goalItem.Get();
	if (bot.goalItemKind == ITEM_NONE)
		return false;

	if (!ent)
	{
		ClearGoalItem(bot);
		return true;
	}

	if (!IsUnavailable(ent, bot.goalItemKind))
		return false;

	char msg[192];

	if (ItemKindIsCharger(bot.goalItemKind))
	{
		snprintf(msg, sizeof(msg), "CrossBot: %s drained %s, now at %.0f\n", bot.name,
			CBotClassOf(ent), g_pGameProfile->ChargerValue(bot.goalItemKind, bot.pEdict));
		SERVER_PRINT(msg);
		ClearGoalItem(bot);
		return true;
	}

	if (bot.currentAreaId == bot.goalItemArea)
	{
		snprintf(msg, sizeof(msg), "CrossBot: %s arrived to find %s already taken\n", bot.name,
			CBotClassOf(ent));
		SERVER_PRINT(msg);
		NoteRespawning(bot, ent, bot.goalItemKind);
		ClearGoalItem(bot);
		return true;
	}

	if (gpGlobals->time < bot.goalItemSeenCheck)
		return false;
	bot.goalItemSeenCheck = gpGlobals->time + kGoalItemSeenInterval;

	Vector eye = bot.pEdict->v.origin + bot.pEdict->v.view_ofs;
	TraceResult tr;
	TRACE_LINE(ent->v.origin, eye, dont_ignore_monsters, ent, &tr);
	if (tr.pHit != bot.pEdict)
		return false;

	snprintf(msg, sizeof(msg), "CrossBot: %s saw %s was already taken, giving up on it\n", bot.name,
		CBotClassOf(ent));
	SERVER_PRINT(msg);
	NoteRespawning(bot, ent, bot.goalItemKind);
	ClearGoalItem(bot);
	return true;
}

void OnTouch(CBot &bot, edict_t *ent)
{
	if (bot.goalItemDone || !bot.goalItem.Is(ent) || ItemKindIsCharger(bot.goalItemKind))
		return;

	bot.goalItemTouched = true;

	char msg[192];
	if (IsPickupGone(ent))
	{
		snprintf(msg, sizeof(msg), "CrossBot: %s picked up %s, back in %.0fs\n", bot.name,
			CBotClassOf(ent), g_pGameProfile->ItemRespawnDelay(bot.goalItemKind));
		SERVER_PRINT(msg);
		NoteRespawning(bot, ent, bot.goalItemKind);
	}
	else
	{
		const char *classname = CBotClassOf(ent);
		snprintf(msg, sizeof(msg), "CrossBot: %s is full of %s, ignoring it for %.0fs\n", bot.name,
			classname, g_pGameProfile->ItemFullTimeout());
		SERVER_PRINT(msg);
		NoteFull(bot, classname);
	}

	bot.goalItemDone = true;
}

void OnArrived(CBot &bot)
{
	if (bot.goalIsEnemy)
	{
		char msg[192];
		snprintf(msg, sizeof(msg), "CrossBot: %s reached where it last saw the enemy, resuming\n",
			bot.name);
		SERVER_PRINT(msg);
		BotSense::ForgetEnemy(bot);
		return;
	}

	edict_t *ent = bot.goalItem.Get();
	if (!ent || ItemKindIsCharger(bot.goalItemKind))
	{
		ForgetGoalItem(bot);
		return;
	}

	if (IsPickupGone(ent))
	{
		char msg[192];
		snprintf(msg, sizeof(msg), "CrossBot: %s arrived to find %s taken, back in %.0fs\n", bot.name,
			CBotClassOf(ent), g_pGameProfile->ItemRespawnDelay(bot.goalItemKind));
		SERVER_PRINT(msg);
		NoteRespawning(bot, ent, bot.goalItemKind);
		ForgetGoalItem(bot);
		return;
	}

	if (!bot.goalItemTouched)
	{
		char msg[208];
		snprintf(msg, sizeof(msg),
			"CrossBot: %s reached %s without touching it, ignoring it for %.0fs\n", bot.name,
			CBotClassOf(ent), g_pGameProfile->ItemUnreachableTimeout());
		SERVER_PRINT(msg);
		NoteIgnored(bot, ent, g_pGameProfile->ItemUnreachableTimeout());
	}

	ForgetGoalItem(bot);
}

void OnRepathFailed(CBot &bot)
{
	if (bot.goalIsEnemy)
	{
		char msg[192];
		snprintf(msg, sizeof(msg), "CrossBot: %s could not plan a route to the enemy, forgetting it\n",
			bot.name);
		SERVER_PRINT(msg);
		BotSense::ForgetEnemy(bot);
		return;
	}

	edict_t *ent = bot.goalItem.Get();
	if (ent)
	{
		char msg[208];
		snprintf(msg, sizeof(msg), "CrossBot: %s could not plan a route to %s, ignoring it for %.0fs\n",
			bot.name, CBotClassOf(ent), g_pGameProfile->ItemUnreachableTimeout());
		SERVER_PRINT(msg);
		NoteIgnored(bot, ent, g_pGameProfile->ItemUnreachableTimeout());
	}

	ForgetGoalItem(bot);
}

void OnBlocked(CBot &bot, const char *what)
{
	if (bot.goalIsEnemy)
	{
		char msg[208];
		snprintf(msg, sizeof(msg),
			"CrossBot: %s could not get past %s reaching the enemy, forgetting it\n", bot.name, what);
		SERVER_PRINT(msg);
		BotSense::ForgetEnemy(bot);
		return;
	}

	edict_t *ent = bot.goalItem.Get();
	if (ent)
	{
		char msg[224];
		snprintf(msg, sizeof(msg),
			"CrossBot: %s could not get past %s reaching %s, ignoring it for %.0fs\n",
			bot.name, what, CBotClassOf(ent), g_pGameProfile->ItemUnreachableTimeout());
		SERVER_PRINT(msg);
		NoteIgnored(bot, ent, g_pGameProfile->ItemUnreachableTimeout());
	}

	ClearGoalItem(bot);
}

void OnRespawn(CBot &bot)
{
	bot.fullMemo.clear();

	if (bot.goalItemKind != ITEM_NONE)
		ClearGoalItem(bot);
}

void PrintCandidates(const Nav::NavMesh &mesh)
{
	size_t shown = 0;

	for (int i = gpGlobals->maxClients + 1; i < gpGlobals->maxEntities; ++i)
	{
		edict_t *ent = g_engfuncs.pfnPEntityOfEntIndex(i);
		if (!ent || ent->free)
			continue;

		const char *classname = CBotClassOf(ent);
		ItemKind kind = g_pGameProfile->ClassifyItem(classname);
		if (kind == ITEM_NONE)
			continue;

		Vector center;
		const Nav::Area *area = ResolveItemArea(mesh, ent, kind, center);

		const char *state = "live";
		if (ItemKindIsCharger(kind))
			state = IsChargerEmpty(ent) ? "charger empty" : "live";
		else if (IsPickupGone(ent))
			state = "gone";

		char areaBuf[16] = "none";
		if (area)
			snprintf(areaBuf, sizeof(areaBuf), "%u", area->id);

		char msg[256];
		snprintf(msg, sizeof(msg),
			"CrossBot: item %s kind %d at (%.1f,%.1f,%.1f) area %s frame %.0f solid %d %s\n",
			classname, static_cast<int>(kind), center.x, center.y, center.z, areaBuf, ent->v.frame,
			ent->v.solid, state);
		SERVER_PRINT(msg);
		++shown;
	}

	char summary[96];
	snprintf(summary, sizeof(summary), "CrossBot: %zu item entities\n", shown);
	SERVER_PRINT(summary);
}

}
