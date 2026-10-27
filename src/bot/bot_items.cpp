#include <extdll.h>
#include <meta_api.h>

#include <algorithm>
#include <string.h>

#include "../game/crossplay.h"
#include "../game/cvars.h"
#include "../game/game_profile.h"
#include "../nav/astar_graph.h"
#include "../nav/nav.h"
#include "../nav/nav_math.h"
#include "../nav/nav_path.h"
#include "../nav/nav_query.h"
#include "bot.h"
#include "bot_fire.h"
#include "bot_goal.h"
#include "bot_items.h"
#include "bot_log.h"
#include "bot_move.h"
#include "bot_move_nav.h"
#include "bot_sight.h"

extern IGameProfile *g_pGameProfile;

namespace
{
	using NavMath::BoxesOverlap;

	constexpr float kItemAreaLift = 18.f;
	constexpr float kItemReachHeight = 72.f;
	constexpr float kChargerUseRadius = 48.f;
	constexpr float kChargerSpotSearch = 96.f;
	constexpr float kGoalItemSeenInterval = 0.25f;
	constexpr float kRecollectCooldown = 15.f;
	constexpr float kHealReachMargin = 48.f;
	constexpr float kStandOriginLift = 18.f;
	constexpr int kHullHuman = 1;
	constexpr int kHullDuck = 3;
	constexpr size_t kHealAreaCandidates = 8;

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

	bool WithinReach(const Nav::Area *area, const Vector &origin)
	{
		return area && fabsf(ClampIntoArea(*area, origin).z - origin.z) <= kItemReachHeight;
	}

	const Nav::Area *ResolvePickupArea(const Nav::NavMesh &mesh, const Vector &origin)
	{
		const Nav::Area *lifted = Nav::FindAreaExact(mesh, origin + Vector(0.f, 0.f, kItemAreaLift));
		if (WithinReach(lifted, origin))
			return lifted;

		const Nav::Area *level = Nav::FindAreaExact(mesh, origin);
		return WithinReach(level, origin) ? level : nullptr;
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

	using BotItems::Placement;

	void ResolveVolumeAreas(const Nav::NavMesh &mesh, edict_t *ent, const Vector &from,
		std::vector<Placement> &out, bool all = false)
	{
		Vector volMins = ent->v.absmin;
		Vector volMaxs = ent->v.absmax;

		Vector reachMins(volMins.x - kHealReachMargin, volMins.y - kHealReachMargin,
			volMins.z - kHealReachMargin);
		Vector reachMaxs(volMaxs.x + kHealReachMargin, volMaxs.y + kHealReachMargin,
			volMaxs.z + kHealReachMargin);

		for (const Nav::Area &area : mesh.areas)
		{
			if (!BoxesOverlap(area.mins, area.maxs, reachMins, reachMaxs))
				continue;

			Vector spot = area.centroid;
			spot.x = std::max(volMins.x, std::min(volMaxs.x, spot.x));
			spot.y = std::max(volMins.y, std::min(volMaxs.y, spot.y));
			spot = ClampIntoArea(area, spot);

			if (!Nav::AreaContainsPoint(area, spot))
				spot = area.centroid;
			if (!Nav::AreaContainsPoint(area, spot))
				continue;
			spot.z = Nav::AreaFloorZ(area, spot.x, spot.y);

			TraceResult tr;
			Vector top(spot.x, spot.y, std::min(area.maxs.z, volMaxs.z));
			g_engfuncs.pfnTraceHull(top, Vector(spot.x, spot.y, area.mins.z), ignore_monsters, kHullDuck, nullptr, &tr);
			if (!tr.fStartSolid && !tr.fAllSolid && tr.flFraction < 1.f && tr.vecEndPos.z > spot.z)
				spot.z = tr.vecEndPos.z;

			bool crouch = (area.areaFlags & crossbot_nav::NAV_AREA_CROUCH) != 0;
			Vector probe = crouch ? spot : spot + Vector(0.f, 0.f, kStandOriginLift);
			if (!BotItems::OriginInsideVolume(ent, probe, crouch ? kHullDuck : kHullHuman))
				continue;

			Placement place;
			place.areaId = area.id;
			place.point = spot;
			out.push_back(place);
		}

		if (all || out.size() <= kHealAreaCandidates)
			return;

		std::partial_sort(out.begin(), out.begin() + kHealAreaCandidates, out.end(),
			[&from](const Placement &a, const Placement &b) {
				return (a.point - from).LengthSquared() < (b.point - from).LengthSquared();
			});
		out.resize(kHealAreaCandidates);
	}

	Vector ResolveItemPlacements(const Nav::NavMesh &mesh, edict_t *ent, ItemKind kind,
		const Vector &from, std::vector<Placement> &out)
	{
		if (ItemKindIsVolume(kind))
		{
			ResolveVolumeAreas(mesh, ent, from, out);
			return BotItems::ItemCenter(ent, kind);
		}

		Vector point;
		const Nav::Area *area = nullptr;

		if (ItemKindIsCharger(kind))
		{
			Vector center = BotItems::ItemCenter(ent, kind);
			area = ResolveChargerArea(mesh, center);
			point = area ? ClampIntoArea(*area, center) : center;
		}
		else
		{
			area = ResolvePickupArea(mesh, ent->v.origin);
			point = area ? ClampIntoArea(*area, ent->v.origin) : ent->v.origin;
		}

		if (area)
		{
			Placement place;
			place.areaId = area->id;
			place.point = point;
			out.push_back(place);
		}

		return point;
	}

	enum ScanReason
	{
		SCAN_OK = 0,
		SCAN_GONE,
		SCAN_RESPAWNING,
		SCAN_IGNORED,
		SCAN_FULL,
		SCAN_SATED,
		SCAN_NO_AREA,
	};

	struct Candidate
	{
		edict_t *ent = nullptr;
		int index = 0;
		ItemKind kind = ITEM_NONE;
		uint32_t areaId = 0;
		bool hasArea = false;
		Vector point;
		ScanReason reason = SCAN_OK;
		float seconds = 0.f;
		float worth = 0.f;
	};

	int JudgeGame(const edict_t *ent)
	{
		int owner = XPlay::OwnerOf(ent);
		return XPlay::Supported(owner) ? owner : XPlay::Current();
	}

	float AmmoNeed(const CBot &bot)
	{
		int guns = 0;
		int dry = 0;
		for (int i = 0; i < bot.inventory.count; ++i)
		{
			const BotWeapon &w = bot.inventory.weapons[i];
			if (w.id == 0 || g_pGameProfile->IsMeleeWeapon(w.id))
				continue;
			++guns;
			if (!BotFire::WeaponArmed(bot.inventory, w))
				++dry;
		}
		return guns ? static_cast<float>(dry) / static_cast<float>(guns) : 0.f;
	}

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

	bool Sated(const CBot &bot, ItemKind kind)
	{
		return g_pGameProfile->ChargerValue(kind, bot.pEdict) >=
			g_pGameProfile->ChargerFullValue(kind);
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
		float ammoNeed = AmmoNeed(bot);

		for (int i = gpGlobals->maxClients + 1; i < gpGlobals->maxEntities; ++i)
		{
			edict_t *ent = g_engfuncs.pfnPEntityOfEntIndex(i);
			if (!ent || ent->free)
				continue;

			const char *classname = CBotClassOf(ent);
			ItemKind kind = ITEM_NONE;
			bool sated = false;
			float worth = 0.f;
			{
				XPlay::ScopedGame scope(JudgeGame(ent));
				if (!g_pGameProfile)
					continue;
				kind = g_pGameProfile->ClassifyItem(ent);
				if (kind == ITEM_NONE)
					continue;
				sated = (ItemKindUsesValue(kind) && Sated(bot, kind)) ||
					!g_pGameProfile->ItemUsable(bot.pEdict, ent);
				if (!sated)
					worth = g_pGameProfile->ItemWorth(bot.pEdict, ent, ammoNeed);
			}

			Candidate cand;
			cand.ent = ent;
			cand.index = i;
			cand.kind = kind;
			cand.worth = worth;

			std::vector<Placement> places;
			cand.point = ResolveItemPlacements(mesh, ent, kind, bot.pEdict->v.origin, places);

			if (BotItems::IsUnavailable(ent, kind))
				cand.reason = SCAN_GONE;
			else if (sated)
				cand.reason = SCAN_SATED;
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
			else if (places.empty())
				cand.reason = SCAN_NO_AREA;

			if (places.empty())
			{
				out.push_back(cand);
				continue;
			}

			size_t count = (cand.reason == SCAN_OK) ? places.size() : 1;
			for (size_t p = 0; p < count; ++p)
			{
				Candidate one = cand;
				one.areaId = places[p].areaId;
				one.hasArea = true;
				one.point = places[p].point;
				out.push_back(one);
			}
		}
	}

	float Readiness(const CBot &bot)
	{
		if (!g_pGameProfile)
			return 0.f;
		return g_pGameProfile->EvaluateCombat(bot.pEdict, bot.inventory);
	}

	float ItemCost(const CBot &bot, const Nav::NavMesh &mesh, const std::vector<float> &costs,
		const Candidate &cand)
	{
		if (cand.areaId >= costs.size() || costs[cand.areaId] >= Nav::kUnreachableSeconds)
			return Nav::kUnreachableSeconds;

		Vector from = (static_cast<int>(cand.areaId) == bot.currentAreaId)
			? bot.pEdict->v.origin : mesh.areas[cand.areaId].centroid;

		return costs[cand.areaId] + (cand.point - from).Length() / NavAStar::kNavRunSpeed;
	}

	bool DebugEnabled()
	{
#if BOT_VLOG_ON
		cvar_t *cvar = CBotCvar("crossbot_gather_debug");
		return cvar && cvar->value != 0.f;
#else
		return false;
#endif
	}

	void DumpScan(const CBot &bot, const Nav::NavMesh &mesh, const std::vector<float> &costs,
		const std::vector<Candidate> &items, const Candidate *picked)
	{
		size_t open = 0;
		for (const Candidate &cand : items)
			if (cand.reason == SCAN_OK)
				++open;

		BOT_VLOG("CrossBot: %s scanning from area %d: %zu items, %zu selectable, readiness %.2f\n",
			bot.name, bot.currentAreaId, items.size(), open, Readiness(bot));

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
				if (ItemKindIsVolume(cand.kind))
					snprintf(why, sizeof(why), "switched off");
				else
					snprintf(why, sizeof(why), "%s",
						ItemKindIsCharger(cand.kind) ? "empty" : "taken");
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
			case SCAN_SATED:
				snprintf(why, sizeof(why), "no need");
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
					snprintf(why, sizeof(why), "%.1fs away, worth %.1fs%s", cost, cand.worth,
						picked && picked->ent == cand.ent ? " PICKED" : "");
				break;
			}
			}

			BOT_VLOG("CrossBot:   #%d %s at (%.0f,%.0f,%.0f) %s, %s\n", cand.index,
				CBotClassOf(cand.ent), cand.point.x, cand.point.y, cand.point.z, where, why);
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

bool OriginInsideVolume(edict_t *ent, const Vector &origin, int hull)
{
	if (!ent || ent->free)
		return false;

	TraceResult tr;
	g_engfuncs.pfnTraceModel(origin, origin, hull, ent, &tr);
	return tr.fStartSolid != 0 || tr.fAllSolid != 0;
}

bool IsVolumeInactive(edict_t *ent)
{
	if (!ent || ent->free)
		return true;
	return ent->v.solid == SOLID_NOT || (ent->v.flags & FL_KILLME) != 0;
}

bool IsUnavailable(edict_t *ent, ItemKind kind)
{
	if (ItemKindIsVolume(kind))
		return IsVolumeInactive(ent);
	return ItemKindIsCharger(kind) ? IsChargerEmpty(ent) : IsPickupGone(ent);
}

Vector ItemCenter(edict_t *ent, ItemKind kind)
{
	if (!ent)
		return Vector(0.f, 0.f, 0.f);
	if (!ItemKindIsBrush(kind))
		return ent->v.origin;
	return (ent->v.absmin + ent->v.absmax) * 0.5f;
}

ItemKind Classify(edict_t *ent)
{
	XPlay::ScopedGame scope(JudgeGame(ent));
	return g_pGameProfile ? g_pGameProfile->ClassifyItem(ent) : ITEM_NONE;
}

float ChargerValue(edict_t *ent, ItemKind kind, edict_t *player)
{
	XPlay::ScopedGame scope(JudgeGame(ent));
	return g_pGameProfile ? g_pGameProfile->ChargerValue(kind, player) : 0.f;
}

float ChargerFull(edict_t *ent, ItemKind kind)
{
	XPlay::ScopedGame scope(JudgeGame(ent));
	return g_pGameProfile ? g_pGameProfile->ChargerFullValue(kind) : 0.f;
}

float RespawnIn(edict_t *ent, ItemKind kind)
{
	XPlay::ScopedGame scope(JudgeGame(ent));
	return g_pGameProfile ? g_pGameProfile->ItemRespawnIn(ent, kind) : 0.f;
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
	memo.kind = bot.goalItem.Is(ent) ? bot.goalItemKind : Classify(ent);
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
	NoteMemo(bot, ent, RespawnIn(ent, kind), true);
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

		BOT_VLOG("CrossBot: %s finds %s back already, forgetting it was taken\n",
			bot.name, CBotClassOf(ent));

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
	BotGoal::Clear(bot);
}

bool Enabled()
{
	cvar_t *cvar = CBotCvar("crossbot_gather");
	return !cvar || cvar->value != 0.f;
}

Vector ClampToArea(const Nav::Area &area, const Vector &point)
{
	return ClampIntoArea(area, point);
}

const Nav::Area *StandArea(const Nav::NavMesh &mesh, const Vector &center)
{
	return ResolveChargerArea(mesh, center);
}

void VolumeAreas(const Nav::NavMesh &mesh, edict_t *ent, const Vector &from,
	std::vector<Placement> &out, bool all)
{
	ResolveVolumeAreas(mesh, ent, from, out, all);
}

void Collect(const CBot &bot, const Nav::NavMesh &mesh, const std::vector<float> &costs,
	std::vector<BotGoal::Candidate> &out)
{
	if (!g_pGameProfile || bot.currentAreaId < 0 || !Enabled())
		return;

	PruneMemos(const_cast<CBot &>(bot));

	std::vector<Candidate> items;
	ScanItems(bot, mesh, items);

	if (DebugEnabled())
		DumpScan(bot, mesh, costs, items, nullptr);

	for (const Candidate &cand : items)
	{
		if (cand.reason != SCAN_OK || !cand.hasArea)
			continue;

		BotGoal::Candidate one;
		one.tier = BotGoal::GOAL_GATHER;
		one.ent = cand.ent;
		one.areaId = cand.areaId;
		one.point = cand.point;
		one.payload = static_cast<int>(cand.kind);
		one.worth = cand.worth;
		out.push_back(one);
	}
}

void KeepPreferred(std::vector<BotGoal::Candidate> &cands)
{
	if (!g_pGameProfile)
	{
		cands.clear();
		return;
	}

	cands.erase(std::remove_if(cands.begin(), cands.end(),
		[](const BotGoal::Candidate &cand) {
			XPlay::ScopedGame scope(JudgeGame(cand.ent));
			return !g_pGameProfile->ItemPreferred(cand.ent, static_cast<ItemKind>(cand.payload));
		}), cands.end());
}

void Accept(CBot &bot, const BotGoal::Candidate &cand)
{
	ForgetGoalItem(bot);

	bot.goalItem.Set(cand.ent);
	bot.goalItemKind = static_cast<ItemKind>(cand.payload);
	bot.goalItemArea = static_cast<int>(cand.areaId);
	bot.goalItemSelectTime = gpGlobals->time;
}

void DropGoalItem(CBot &bot)
{
	ForgetGoalItem(bot);
	BotGoal::Advance(bot);
}

bool UpdateGoalItem(CBot &bot)
{
	if (bot.goalItemDone)
	{
		DropGoalItem(bot);
		return true;
	}

	edict_t *ent = bot.goalItem.Get();
	if (bot.goalItemKind == ITEM_NONE)
		return false;

	if (!ent)
	{
		DropGoalItem(bot);
		return true;
	}

	if (!IsUnavailable(ent, bot.goalItemKind))
		return false;


	if (ItemKindUsesValue(bot.goalItemKind))
	{
		BOT_VLOG("CrossBot: %s %s %s, now at %.0f\n", bot.name,
			ItemKindIsVolume(bot.goalItemKind) ? "found switched off" : "drained",
			CBotClassOf(ent), ChargerValue(ent, bot.goalItemKind, bot.pEdict));
		DropGoalItem(bot);
		return true;
	}

	if (bot.currentAreaId == bot.goalItemArea)
	{
		BOT_VLOG("CrossBot: %s arrived to find %s already taken\n", bot.name,
			CBotClassOf(ent));
		NoteRespawning(bot, ent, bot.goalItemKind);
		DropGoalItem(bot);
		return true;
	}

	if (gpGlobals->time < bot.goalItemSeenCheck)
		return false;
	bot.goalItemSeenCheck = gpGlobals->time + kGoalItemSeenInterval;

	Vector eye = bot.pEdict->v.origin + bot.pEdict->v.view_ofs;
	if (BotSight::Clarity(ent->v.origin, eye, dont_ignore_monsters, ent, bot.pEdict) <= 0.f)
		return false;

	BOT_VLOG("CrossBot: %s saw %s was already taken, giving up on it\n", bot.name,
		CBotClassOf(ent));
	NoteRespawning(bot, ent, bot.goalItemKind);
	DropGoalItem(bot);
	return true;
}

void OnTouch(CBot &bot, edict_t *ent)
{
	if (bot.goalItemDone || !bot.goalItem.Is(ent) || ItemKindUsesValue(bot.goalItemKind))
		return;

	bot.goalItemTouched = true;

	if (IsPickupGone(ent))
	{
		float secs = RespawnIn(ent, bot.goalItemKind);
		BOT_VLOG("CrossBot: %s picked up %s, back in %.0fs\n", bot.name, CBotClassOf(ent), secs);
		if (secs < kRecollectCooldown)
			NoteIgnored(bot, ent, kRecollectCooldown);
		else
			NoteRespawning(bot, ent, bot.goalItemKind);
	}
	else
	{
		const char *classname = CBotClassOf(ent);
		BOT_VLOG("CrossBot: %s is full of %s, ignoring it for %.0fs\n", bot.name,
			classname, g_pGameProfile->ItemFullTimeout());
		NoteFull(bot, classname);
	}

	bot.goalItemDone = true;
}

void OnArrived(CBot &bot)
{
	edict_t *ent = bot.goalItem.Get();
	if (!ent || ItemKindUsesValue(bot.goalItemKind))
	{
		ForgetGoalItem(bot);
		return;
	}

	if (IsPickupGone(ent))
	{
		BOT_VLOG("CrossBot: %s arrived to find %s taken, back in %.0fs\n", bot.name,
			CBotClassOf(ent), RespawnIn(ent, bot.goalItemKind));
		NoteRespawning(bot, ent, bot.goalItemKind);
		ForgetGoalItem(bot);
		return;
	}

	if (!bot.goalItemTouched)
	{
		BOT_VLOG("CrossBot: %s reached %s without touching it (item z %.0f..%.0f, feet %.0f), "
			"ignoring it for %.0fs\n", bot.name, CBotClassOf(ent), ent->v.absmin.z, ent->v.absmax.z,
			bot.pEdict->v.absmin.z, g_pGameProfile->ItemUnreachableTimeout());
		NoteIgnored(bot, ent, g_pGameProfile->ItemUnreachableTimeout());
	}

	ForgetGoalItem(bot);
}

void OnRepathFailed(CBot &bot)
{
	edict_t *ent = bot.goalItem.Get();
	if (ent)
	{
		BOT_VLOG("CrossBot: %s could not plan a route to %s, ignoring it for %.0fs\n",
			bot.name, CBotClassOf(ent), g_pGameProfile->ItemUnreachableTimeout());
		NoteIgnored(bot, ent, g_pGameProfile->ItemUnreachableTimeout());
	}

	ForgetGoalItem(bot);
}

void OnBlocked(CBot &bot, const char *what)
{
	edict_t *ent = bot.goalItem.Get();
	if (ent)
	{
		BOT_VLOG("CrossBot: %s could not get past %s reaching %s, ignoring it for %.0fs\n",
			bot.name, what, CBotClassOf(ent), g_pGameProfile->ItemUnreachableTimeout());
		NoteIgnored(bot, ent, g_pGameProfile->ItemUnreachableTimeout());
	}

	ForgetGoalItem(bot);
}

void OnRespawn(CBot &bot)
{
	bot.fullMemo.clear();

	if (bot.goalItemKind != ITEM_NONE)
		ForgetGoalItem(bot);
}

void PrintCandidates(const Nav::NavMesh &mesh)
{
	if (!g_pGameProfile)
	{
		SERVER_PRINT("CrossBot: no game profile active\n");
		return;
	}

	size_t shown = 0;

	for (int i = gpGlobals->maxClients + 1; i < gpGlobals->maxEntities; ++i)
	{
		edict_t *ent = g_engfuncs.pfnPEntityOfEntIndex(i);
		if (!ent || ent->free)
			continue;

		const char *classname = CBotClassOf(ent);
		ItemKind kind = Classify(ent);
		if (kind == ITEM_NONE)
			continue;

		std::vector<Placement> places;
		Vector center = ResolveItemPlacements(mesh, ent, kind, ItemCenter(ent, kind), places);

		const char *state = "live";
		if (ItemKindIsVolume(kind))
			state = IsVolumeInactive(ent) ? "volume off" : "live";
		else if (ItemKindIsCharger(kind))
			state = IsChargerEmpty(ent) ? "charger empty" : "live";
		else if (IsPickupGone(ent))
			state = "gone";

		char areaBuf[64] = "none";
		int written = 0;
		for (size_t p = 0; p < places.size() && written >= 0 && written + 8 < (int)sizeof(areaBuf); ++p)
			written += snprintf(areaBuf + written, sizeof(areaBuf) - written, "%s%u",
				p == 0 ? "" : ",", places[p].areaId);

		char msg[320];
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
