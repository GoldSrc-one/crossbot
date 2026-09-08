#include <extdll.h>
#include <meta_api.h>
#include <in_buttons.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>

#include "../game/game_profile.h"
#include "../game/gamedata.h"
#include "../nav/nav.h"
#include "../nav/nav_math.h"
#include "../nav/nav_query.h"
#include "../nav/path_follower.h"
#include "../nav/pm_sim.h"
#include "bot.h"
#include "bot_aim.h"
#include "bot_break.h"
#include "bot_combat.h"
#include "bot_fun.h"
#include "bot_goal.h"
#include "bot_items.h"
#include "bot_log.h"
#include "bot_move.h"
#include "bot_move_nav.h"
#include "bot_sense.h"

extern IGameProfile *g_pGameProfile;

namespace
{
	using NavMath::kPiF;
	using NavMath::Norm2D;

	constexpr int kTsAtTop = 0;
	constexpr int kTsAtBottom = 1;
	constexpr int kTsGoingUp = 2;
	constexpr int kTsGoingDown = 3;

	constexpr int kSfButtonToggle = 32;
	constexpr int kSfButtonTouchOnly = 256;
	constexpr int kSfTankCanControl = 0x20;
	constexpr int kSfMultiInit = 1;

	constexpr int kChainDepth = 6;

	constexpr float kFunUseRange = 56.f;
	constexpr float kUseDotFloor = 0.75f;
	constexpr float kFunAimTolerance = 8.f;
	constexpr float kFunStandRadius = 8.f;
	constexpr float kFunArrivalTimeout = 6.f;
	constexpr float kFunMinHold = 0.5f;
	constexpr float kFunPulse = 0.2f;

	constexpr float kFunSwitchCooldown = 30.f;
	constexpr float kFunFailCooldown = 30.f;
	constexpr float kFunTankShotCooldown = 20.f;
	constexpr float kFunTankFailCooldown = 15.f;

	constexpr float kFunBoredom = 10.f;
	constexpr float kFunMountGrace = 1.5f;

	constexpr float kTankAimTolerance = 4.f;
	constexpr float kTankReachRange = 56.f;

	constexpr int kSfTrainNoControl = 2;

	constexpr float kTrainStopEpsilon = 1.f;
	constexpr float kTrainBoardRange = 112.f;
	constexpr float kTrainStepUp = 32.f;
	constexpr float kTrainBoardRise = 40.f;
	constexpr float kTrainCentreRadius = 24.f;
	constexpr float kTrainBoardArcSlack = 192.f;
	constexpr float kTrainCatchWindow = 3.f;
	constexpr float kTrainSideClear = 40.f;
	constexpr float kTrainBrink = 24.f;
	constexpr float kStandHalfHeight = 36.f;
	constexpr float kTrainGroundProbe = 96.f;
	constexpr float kDuckHalfHeight = 18.f;
	constexpr float kTrainHopRange = 112.f;
	constexpr float kTrainSafeDrop = 96.f;
	constexpr float kTrainRideMax = 60.f;
	constexpr float kTrainJumpGap = 40.f;
	constexpr float kTrainDriveTimeout = 8.f;
	constexpr float kTrainOffGrace = 0.4f;
	constexpr float kTrainUnreachable = 1.0e9f;

	constexpr int kTrainMaxNodes = 512;

	constexpr int kHullDuck = 3;

	struct Track
	{
		std::vector<Vector> nodes;
		std::vector<float> arcs;
		float length = 0.f;
		bool closed = false;
	};

	struct BoardPoint
	{
		unsigned int areaId = 0;
		Vector point;
		Vector deck;
		float arc = 0.f;
		float standZ = 0.f;
	};

	struct Entry
	{
		int index = 0;
		BotFun::Kind kind = BotFun::FUN_SWITCH;
		BotFun::Action action = BotFun::FUN_ACT_USE;
		char terminal[32] = {};

		Track track;
		std::vector<BoardPoint> board;
		bool boardBuilt = false;
	};

	std::vector<Entry> g_table;
	char g_tableMap[64] = {};
	bool g_tableBuilt = false;

	void *Priv(edict_t *ent)
	{
		return CBotPrivateData(ent);
	}

	bool PeekInt(edict_t *ent, int offset, int &out)
	{
		void *base = Priv(ent);
		if (!base || offset < 0)
			return false;
		out = GameData::Peek<int>(base, offset);
		return true;
	}

	bool PeekFloat(edict_t *ent, int offset, float &out)
	{
		void *base = Priv(ent);
		if (!base || offset < 0)
			return false;
		out = GameData::Peek<float>(base, offset);
		return true;
	}

	edict_t *PeekEdict(edict_t *ent, int offset)
	{
		void *base = Priv(ent);
		if (!base || offset < 0)
			return nullptr;

		void *obj = GameData::Peek<void *>(base, offset);
		if (!obj)
			return nullptr;

		entvars_t *pev = GameData::Peek<entvars_t *>(obj, g_offsets.pev);
		if (!pev || !pev->pContainingEntity || pev->pContainingEntity->free)
			return nullptr;

		return pev->pContainingEntity;
	}

	const char *NameOf(edict_t *ent)
	{
		if (!ent || !gpGlobals->pStringBase)
			return "";
		const char *s = gpGlobals->pStringBase + static_cast<unsigned int>(ent->v.targetname);
		return s ? s : "";
	}

	const char *TargetOf(edict_t *ent)
	{
		if (!ent || !gpGlobals->pStringBase)
			return "";
		const char *s = gpGlobals->pStringBase + static_cast<unsigned int>(ent->v.target);
		return s ? s : "";
	}

	Vector BrushCenter(edict_t *ent)
	{
		return (ent->v.absmin + ent->v.absmax) * 0.5f;
	}

	bool MasterSatisfied(const char *name)
	{
		if (!name || !name[0])
			return true;

		for (int i = 1; i < gpGlobals->maxEntities; ++i)
		{
			edict_t *ent = g_engfuncs.pfnPEntityOfEntIndex(i);
			if (!ent || ent->free || strcmp(NameOf(ent), name) != 0)
				continue;
			if (strcmp(CBotClassOf(ent), "multisource") != 0)
				continue;
			if ((static_cast<int>(ent->v.spawnflags) & kSfMultiInit) != 0)
				return false;

			int total = 0;
			if (!PeekInt(ent, g_offsets.ms_iTotal, total))
				return false;

			void *base = Priv(ent);
			for (int t = 0; t < total && t < 32; ++t)
			{
				if (GameData::Peek<int>(base, g_offsets.ms_rgTriggered + t * 4) == 0)
					return false;
			}
			return true;
		}

		return true;
	}

	void PushTargets(edict_t *ent, std::vector<std::string> &out)
	{
		const char *cls = CBotClassOf(ent);

		if (strcmp(cls, "multi_manager") == 0)
		{
			int count = 0;
			if (!PeekInt(ent, g_offsets.mm_cTargets, count))
				return;

			void *base = Priv(ent);
			for (int i = 0; i < count && i < 16; ++i)
			{
				int idx = GameData::Peek<int>(base, g_offsets.mm_iTargetName + i * 4);
				const char *name = gpGlobals->pStringBase
					? gpGlobals->pStringBase + static_cast<unsigned int>(idx) : nullptr;
				if (name && name[0])
					out.push_back(name);
			}
			return;
		}

		const char *target = TargetOf(ent);
		if (target && target[0])
			out.push_back(target);
	}

	bool PassesThrough(const char *cls)
	{
		return strcmp(cls, "trigger_relay") == 0 || strcmp(cls, "multisource") == 0 ||
			strcmp(cls, "multi_manager") == 0;
	}

	bool ChainTerminal(edict_t *src, char *out, size_t outSize)
	{
		std::vector<std::string> frontier;
		std::vector<std::string> seenNames;
		std::vector<int> seenEnts;

		PushTargets(src, frontier);

		for (int depth = 0; depth < kChainDepth && !frontier.empty(); ++depth)
		{
			std::vector<std::string> next;

			for (const std::string &name : frontier)
			{
				if (std::find(seenNames.begin(), seenNames.end(), name) != seenNames.end())
					continue;
				seenNames.push_back(name);

				for (int i = 1; i < gpGlobals->maxEntities; ++i)
				{
					edict_t *ent = g_engfuncs.pfnPEntityOfEntIndex(i);
					if (!ent || ent->free || ent == src)
						continue;
					if (strcmp(NameOf(ent), name.c_str()) != 0)
						continue;
					if (std::find(seenEnts.begin(), seenEnts.end(), i) != seenEnts.end())
						continue;
					seenEnts.push_back(i);

					const char *cls = CBotClassOf(ent);
					if (PassesThrough(cls))
					{
						PushTargets(ent, next);
						continue;
					}

					if (g_pGameProfile && g_pGameProfile->IsFunTerminal(ent))
					{
						snprintf(out, outSize, "%s", cls);
						return true;
					}
				}
			}

			frontier.swap(next);
		}

		return false;
	}

	edict_t *TankOfControls(edict_t *controls)
	{
		if (edict_t *cached = PeekEdict(controls, g_offsets.tankctl_pTank))
			return cached;

		const char *target = TargetOf(controls);
		if (!target || !target[0])
			return nullptr;

		for (int i = 1; i < gpGlobals->maxEntities; ++i)
		{
			edict_t *ent = g_engfuncs.pfnPEntityOfEntIndex(i);
			if (!ent || ent->free || strcmp(NameOf(ent), target) != 0)
				continue;
			if (strncmp(CBotClassOf(ent), "func_tank", 9) == 0)
				return ent;
		}

		return nullptr;
	}

	edict_t *FindPathTrack(const char *name)
	{
		if (!name || !name[0])
			return nullptr;

		for (int i = 1; i < gpGlobals->maxEntities; ++i)
		{
			edict_t *ent = g_engfuncs.pfnPEntityOfEntIndex(i);
			if (!ent || ent->free || strcmp(NameOf(ent), name) != 0)
				continue;
			if (strcmp(CBotClassOf(ent), "path_track") == 0)
				return ent;
		}

		return nullptr;
	}

	void BuildTrack(edict_t *train, Track &track)
	{
		std::vector<int> seen;
		const char *name = TargetOf(train);

		while (static_cast<int>(track.nodes.size()) < kTrainMaxNodes)
		{
			edict_t *node = FindPathTrack(name);
			if (!node)
				break;

			int index = g_engfuncs.pfnIndexOfEdict(node);
			if (std::find(seen.begin(), seen.end(), index) != seen.end())
			{
				track.closed = !seen.empty() && index == seen.front();
				break;
			}

			seen.push_back(index);
			track.nodes.push_back(node->v.origin);
			name = TargetOf(node);
		}

		size_t count = track.nodes.size();
		track.arcs.assign(count, 0.f);
		if (count < 2)
		{
			track.closed = false;
			return;
		}

		float acc = 0.f;
		for (size_t i = 0; i + 1 < count; ++i)
		{
			track.arcs[i] = acc;
			acc += (track.nodes[i + 1] - track.nodes[i]).Length();
		}
		track.arcs[count - 1] = acc;

		if (track.closed)
			acc += (track.nodes[0] - track.nodes[count - 1]).Length();
		track.length = acc;
	}

	size_t TrackSegments(const Track &track)
	{
		size_t count = track.nodes.size();
		if (count < 2)
			return 0;
		return track.closed ? count : count - 1;
	}

	float ProjectOnTrack(const Track &track, const Vector &point)
	{
		size_t count = track.nodes.size();
		size_t segs = TrackSegments(track);
		if (segs == 0)
			return 0.f;

		float best = 1.0e30f;
		float bestArc = 0.f;

		for (size_t i = 0; i < segs; ++i)
		{
			const Vector &a = track.nodes[i];
			const Vector &b = track.nodes[(i + 1) % count];
			Vector d = b - a;

			float len2 = DotProduct(d, d);
			float u = len2 > 0.0001f ? DotProduct(point - a, d) / len2 : 0.f;
			u = u < 0.f ? 0.f : (u > 1.f ? 1.f : u);

			Vector q = a + d * u;
			float dist = (point - q).Length();
			if (dist < best)
			{
				best = dist;
				bestArc = track.arcs[i] + d.Length() * u;
			}
		}

		return bestArc;
	}

	float ArcForward(const Track &track, float from, float to, float sign)
	{
		if (track.length <= 0.f)
			return kTrainUnreachable;

		float d = sign >= 0.f ? (to - from) : (from - to);

		if (!track.closed)
			return d >= 0.f ? d : kTrainUnreachable;

		d = fmodf(d, track.length);
		if (d < 0.f)
			d += track.length;
		return d;
	}

	Vector TrainRight(float yawDegrees)
	{
		float yaw = yawDegrees * (kPiF / 180.f);
		return Vector(sinf(yaw), -cosf(yaw), 0.f);
	}

	Vector TrainForward(float yawDegrees)
	{
		float yaw = yawDegrees * (kPiF / 180.f);
		return Vector(cosf(yaw), sinf(yaw), 0.f);
	}

	Vector DeckAt(edict_t *train, const Vector &origin, const Vector &forward, const Vector &right)
	{
		Vector mid = (train->v.mins + train->v.maxs) * 0.5f;
		Vector deck = origin + forward * mid.x - right * mid.y;
		deck.z = origin.z + train->v.maxs.z;
		return deck;
	}

	Vector TrainDeck(edict_t *train)
	{
		return DeckAt(train, train->v.origin, TrainForward(train->v.angles.y),
			TrainRight(train->v.angles.y));
	}

	float TrainEta(const Track &track, edict_t *train, float boardArc)
	{
		float speed = train->v.speed;
		if (fabsf(speed) < kTrainStopEpsilon)
			return kTrainUnreachable;

		float arc = ProjectOnTrack(track, train->v.origin);
		return ArcForward(track, arc, boardArc, speed) / fabsf(speed);
	}

	bool IsSwitchClass(const char *cls)
	{
		return strcmp(cls, "func_button") == 0 || strcmp(cls, "func_rot_button") == 0 ||
			strcmp(cls, "trigger_multiple") == 0 || strcmp(cls, "trigger_once") == 0;
	}

	bool IsTriggerClass(const char *cls)
	{
		return strcmp(cls, "trigger_multiple") == 0 || strcmp(cls, "trigger_once") == 0;
	}

	BotFun::Action ActionFor(edict_t *ent)
	{
		const char *cls = CBotClassOf(ent);
		if (IsTriggerClass(cls))
			return BotFun::FUN_ACT_TOUCH;
		if (ent->v.health > 0.f && ent->v.takedamage != DAMAGE_NO)
			return BotFun::FUN_ACT_SHOOT;
		if ((static_cast<int>(ent->v.spawnflags) & kSfButtonTouchOnly) != 0)
			return BotFun::FUN_ACT_TOUCH;
		return BotFun::FUN_ACT_USE;
	}

	void BuildTable()
	{
		const char *map = STRING(gpGlobals->mapname);
		if (g_tableBuilt && strncmp(g_tableMap, map, sizeof(g_tableMap) - 1) == 0)
			return;

		g_table.clear();
		snprintf(g_tableMap, sizeof(g_tableMap), "%s", map);
		g_tableBuilt = true;

		for (int i = 1; i < gpGlobals->maxEntities; ++i)
		{
			edict_t *ent = g_engfuncs.pfnPEntityOfEntIndex(i);
			if (!ent || ent->free)
				continue;

			const char *cls = CBotClassOf(ent);

			if (strcmp(cls, "func_tankcontrols") == 0)
			{
				edict_t *tank = TankOfControls(ent);
				if (!tank)
					continue;
				if ((static_cast<int>(tank->v.spawnflags) & kSfTankCanControl) == 0)
					continue;

				Entry entry;
				entry.index = i;
				entry.kind = BotFun::FUN_TANK;
				entry.action = BotFun::FUN_ACT_USE;
				snprintf(entry.terminal, sizeof(entry.terminal), "%s", CBotClassOf(tank));
				g_table.push_back(entry);
				continue;
			}

			if (strcmp(cls, "func_tracktrain") == 0)
			{
				if ((static_cast<int>(ent->v.spawnflags) & kSfTrainNoControl) != 0)
					continue;
				if (ent->v.impulse == 0)
					continue;

				Entry entry;
				entry.index = i;
				entry.kind = BotFun::FUN_TRAIN;
				entry.action = BotFun::FUN_ACT_USE;
				BuildTrack(ent, entry.track);
				if (TrackSegments(entry.track) == 0)
					continue;

				snprintf(entry.terminal, sizeof(entry.terminal), "path_track");
				g_table.push_back(entry);
				continue;
			}

			if (!IsSwitchClass(cls))
				continue;

			Entry entry;
			entry.index = i;
			entry.kind = BotFun::FUN_SWITCH;
			entry.action = ActionFor(ent);
			if (!ChainTerminal(ent, entry.terminal, sizeof(entry.terminal)))
				continue;

			g_table.push_back(entry);
		}

		BOT_VLOG("CrossBot: fun table for %s has %zu entries\n", g_tableMap, g_table.size());
	}

	edict_t *EntryEdict(const Entry &entry)
	{
		edict_t *ent = g_engfuncs.pfnPEntityOfEntIndex(entry.index);
		if (!ent || ent->free)
			return nullptr;
		return ent;
	}

	const Entry *EntryFor(edict_t *ent)
	{
		if (!ent || ent->free)
			return nullptr;

		int index = g_engfuncs.pfnIndexOfEdict(ent);
		for (const Entry &entry : g_table)
			if (entry.index == index)
				return &entry;
		return nullptr;
	}

	bool SwitchReady(edict_t *ent, BotFun::Action action)
	{
		if (action == BotFun::FUN_ACT_TOUCH && IsTriggerClass(CBotClassOf(ent)))
			return ent->v.solid != SOLID_NOT && ent->v.nextthink <= gpGlobals->time;

		int state = 0;
		if (!PeekInt(ent, g_offsets.tog_toggleState, state))
			return false;

		if (state == kTsGoingUp || state == kTsGoingDown)
			return false;
		if (state == kTsAtBottom)
			return true;

		if (state != kTsAtTop)
			return false;

		int stay = 1;
		if (!PeekInt(ent, g_offsets.btn_fStayPushed, stay))
			return false;

		return stay == 0 && (static_cast<int>(ent->v.spawnflags) & kSfButtonToggle) != 0;
	}

	bool TankFree(edict_t *tank, edict_t *forBot)
	{
		if (!tank || tank->free)
			return false;
		if ((static_cast<int>(tank->v.spawnflags) & kSfTankCanControl) == 0)
			return false;

		edict_t *controller = PeekEdict(tank, g_offsets.tank_pController);
		if (controller && controller != forBot)
			return false;

		int master = 0;
		if (PeekInt(tank, g_offsets.tank_iszMaster, master) && master != 0 && gpGlobals->pStringBase)
		{
			const char *name = gpGlobals->pStringBase + static_cast<unsigned int>(master);
			if (!MasterSatisfied(name))
				return false;
		}

		return true;
	}

	edict_t *MountedTank(edict_t *pEdict)
	{
		void *base = Priv(pEdict);
		if (!base || g_offsets.plr_pTank < 0)
			return nullptr;

		edict_t *ent = GameData::Peek<edict_t *>(base, g_offsets.plr_pTank);
		if (!ent || ent->free)
			return nullptr;

		int serial = GameData::Peek<int>(base, g_offsets.plr_pTank + 4);
		return ent->serialnumber == serial ? ent : nullptr;
	}

	bool IsController(edict_t *pEdict, edict_t *tank)
	{
		if (!tank || tank->free)
			return false;
		return MountedTank(pEdict) == tank ||
			PeekEdict(tank, g_offsets.tank_pController) == pEdict;
	}

	const BotFun::Memo *MemoFor(const CBot &bot, edict_t *ent)
	{
		for (const BotFun::Memo &m : bot.fun.memo)
			if (m.ent.Is(ent))
				return &m;
		return nullptr;
	}

	BotFun::Memo *MemoSlot(CBot &bot, edict_t *ent)
	{
		if (!ent || ent->free)
			return nullptr;

		for (BotFun::Memo &m : bot.fun.memo)
			if (m.ent.Is(ent))
				return &m;

		BotFun::Memo added;
		added.ent.Set(ent);
		bot.fun.memo.push_back(added);
		return &bot.fun.memo.back();
	}

	void NoteCooldown(CBot &bot, edict_t *ent, float seconds)
	{
		if (BotFun::Memo *memo = MemoSlot(bot, ent))
			memo->readyTime = std::max(memo->readyTime, gpGlobals->time + seconds);
	}

	float Readiness(const CBot &bot)
	{
		if (!g_pGameProfile)
			return 0.f;
		return g_pGameProfile->EvaluateCombat(bot.pEdict, bot.inventory);
	}

	bool Blocked(const CBot &bot, edict_t *ent)
	{
		const BotFun::Memo *memo = MemoFor(bot, ent);
		if (!memo)
			return false;
		if (memo->readyTime > gpGlobals->time)
			return true;
		if (memo->abandonReadiness >= 0.f && Readiness(bot) <= memo->abandonReadiness)
			return true;
		return false;
	}

	bool HullClear(edict_t *ent, const Vector &from, const Vector &to)
	{
		TraceResult tr;
		g_engfuncs.pfnTraceHull(from, to, ignore_monsters, kHullDuck, ent, &tr);
		return tr.fStartSolid == 0 && tr.fAllSolid == 0 && tr.flFraction >= 0.99f;
	}

	bool GroundBelow(edict_t *ent, const Vector &at, float &floorZ)
	{
		Vector to(at.x, at.y, at.z - kTrainGroundProbe);

		TraceResult tr;
		g_engfuncs.pfnTraceHull(at, to, ignore_monsters, kHullDuck, ent, &tr);

		if (tr.fStartSolid != 0 || tr.fAllSolid != 0 || tr.flFraction >= 1.f)
			return false;

		floorZ = tr.vecEndPos.z - kDuckHalfHeight;
		return true;
	}

	void CollectVolume(const Nav::NavMesh &mesh, edict_t *ent, const Vector &from,
		std::vector<BotItems::Placement> &out)
	{
		std::vector<BotItems::Placement> raw;
		BotItems::VolumeAreas(mesh, ent, from, raw);

		for (const BotItems::Placement &p : raw)
		{
			if (p.areaId >= mesh.areas.size())
				continue;
			const Nav::Area &area = mesh.areas[p.areaId];
			if (!HullClear(ent, area.centroid, p.point))
				continue;
			out.push_back(p);
		}
	}

	void CollectUsePoints(const Nav::NavMesh &mesh, edict_t *ent, const Vector &from,
		std::vector<BotItems::Placement> &out)
	{
		Vector center = BrushCenter(ent);

		if (const Nav::Area *area = BotItems::StandArea(mesh, center))
		{
			BotItems::Placement p;
			p.areaId = area->id;
			p.point = BotItems::ClampToArea(*area, center);
			out.push_back(p);
		}

		std::vector<BotItems::Placement> volume;
		CollectVolume(mesh, ent, from, volume);

		for (const BotItems::Placement &p : volume)
		{
			if ((p.point - center).Length() > kTankReachRange)
				continue;
			if (!out.empty() && out[0].areaId == p.areaId)
				continue;
			out.push_back(p);
		}
	}

	void BuildBoardPoints(const Nav::NavMesh &mesh, edict_t *train, Entry &entry)
	{
		if (entry.boardBuilt)
			return;
		entry.boardBuilt = true;

		const Track &track = entry.track;
		size_t count = track.nodes.size();
		size_t segs = TrackSegments(track);
		if (segs == 0)
			return;

		float halfWidth = (train->v.maxs.y - train->v.mins.y) * 0.5f;
		float clear = halfWidth + kTrainSideClear;
		const float offsets[3] = {clear, clear + 40.f, clear + 80.f};

		for (size_t i = 0; i < count; ++i)
		{
			const Vector &node = track.nodes[i];

			Vector d = track.nodes[(i + 1) % count] - node;
			float flat = sqrtf(d.x * d.x + d.y * d.y);
			Vector travel = flat > 0.0001f ? Vector(d.x / flat, d.y / flat, 0.f)
				: Vector(1.f, 0.f, 0.f);

			Vector forward = travel * -1.f;
			Vector right(forward.y, -forward.x, 0.f);

			Vector deck = DeckAt(train, node, forward, right);

			for (int s = -1; s <= 1; s += 2)
			{
				for (int o = 0; o < 3; ++o)
				{
					Vector probe = deck + right * (static_cast<float>(s) * offsets[o]);

					const Nav::Area *area = Nav::FindAreaNearest(mesh, probe);
					if (!area)
						continue;

					Vector point = BotItems::ClampToArea(*area,
						Vector(probe.x, probe.y, area->centroid.z));
					float dx = point.x - deck.x;
					float dy = point.y - deck.y;
					float reach = sqrtf(dx * dx + dy * dy);

					if (reach > clear + kTrainBoardRange)
						continue;
					if (fabsf(dx * right.x + dy * right.y) < clear)
						continue;

					float floorZ = 0.f;
					if (!GroundBelow(train, point, floorZ))
						continue;

					if (deck.z - floorZ > kTrainStepUp)
						continue;
					if (floorZ - deck.z > kTrainBoardRise)
						continue;

					float rideZ = deck.z + kStandHalfHeight;
					float standZ = floorZ + kStandHalfHeight;

					Vector airFrom(point.x, point.y, std::max(standZ, rideZ));
					Vector airTo(deck.x, deck.y, rideZ);
					if (!HullClear(train, airFrom, airTo))
						continue;

					size_t slot = entry.board.size();
					for (size_t b = 0; b < entry.board.size(); ++b)
					{
						if (entry.board[b].areaId == area->id)
						{
							slot = b;
							break;
						}
					}

					if (slot < entry.board.size())
					{
						const BoardPoint &have = entry.board[slot];
						float bx = have.point.x - have.deck.x;
						float by = have.point.y - have.deck.y;
						if (sqrtf(bx * bx + by * by) <= reach)
							continue;
					}
					else
					{
						entry.board.push_back(BoardPoint());
					}

					entry.board[slot].areaId = area->id;
					entry.board[slot].point = point;
					entry.board[slot].deck = deck;
					entry.board[slot].arc = track.arcs[i];
					entry.board[slot].standZ = standZ;
				}
			}
		}
	}

	float AngleDistance(float next, float cur)
	{
		return Bot_NormalizeAngle(next - cur);
	}

	struct TankCone
	{
		float yawCenter = 0.f;
		float yawRange = 0.f;
		float pitchCenter = 0.f;
		float pitchRange = 0.f;
		bool valid = false;
	};

	TankCone ReadCone(edict_t *tank)
	{
		TankCone cone;
		cone.valid = PeekFloat(tank, g_offsets.tank_yawCenter, cone.yawCenter) &&
			PeekFloat(tank, g_offsets.tank_yawRange, cone.yawRange) &&
			PeekFloat(tank, g_offsets.tank_pitchCenter, cone.pitchCenter) &&
			PeekFloat(tank, g_offsets.tank_pitchRange, cone.pitchRange);
		return cone;
	}

	bool AnglesToward(edict_t *tank, const Vector &point, float &yaw, float &pitch)
	{
		Vector dir = point - tank->v.origin;
		float flat = sqrtf(dir.x * dir.x + dir.y * dir.y);
		if (flat < 0.0001f && fabsf(dir.z) < 0.0001f)
			return false;

		yaw = atan2f(dir.y, dir.x) * (180.f / kPiF);
		pitch = -atan2f(dir.z, flat) * (180.f / kPiF);
		return true;
	}

	bool InCone(const TankCone &cone, float yaw, float pitch)
	{
		if (!cone.valid)
			return false;
		if (fabsf(AngleDistance(yaw, cone.yawCenter)) > cone.yawRange)
			return false;
		if (fabsf(AngleDistance(pitch, cone.pitchCenter)) > cone.pitchRange)
			return false;
		return true;
	}

	void DriveTo(CBot &bot, const Vector &angles, const Vector &to, bool stop,
		unsigned short extra, float speed = 0.f)
	{
		Bot_DriveTo(bot, angles, to, stop, extra, speed, kFunStandRadius);
	}

	void FaceDir(CBot &bot, const Vector &dir)
	{
		Bot_FaceDir(bot, dir);
	}

	Vector ForwardFromAngles(const Vector &angles)
	{
		float pitch = angles.x * (kPiF / 180.f);
		float yaw = angles.y * (kPiF / 180.f);
		return Vector(cosf(pitch) * cosf(yaw), cosf(pitch) * sinf(yaw), -sinf(pitch));
	}

	Vector UseDir(edict_t *ent, const Vector &eye)
	{
		Vector los = BrushCenter(ent) - eye;
		Vector half = ent->v.size * 0.5f;
		Vector out = los;

		for (int i = 0; i < 3; ++i)
		{
			if (out[i] > half[i]) out[i] -= half[i];
			else if (out[i] < -half[i]) out[i] += half[i];
			else out[i] = 0.f;
		}

		float len = out.Length();
		if (len < 0.0001f)
		{
			len = los.Length();
			return len < 0.0001f ? Vector(1.f, 0.f, 0.f) : los * (1.f / len);
		}
		return out * (1.f / len);
	}

	float UseBoxDist(edict_t *ent, const Vector &org)
	{
		float sum = 0.f;
		for (int i = 0; i < 3; ++i)
		{
			float d = 0.f;
			if (org[i] < ent->v.absmin[i]) d = org[i] - ent->v.absmin[i];
			else if (org[i] > ent->v.absmax[i]) d = org[i] - ent->v.absmax[i];
			sum += d * d;
		}
		return sqrtf(sum);
	}

	bool Aimed(const CBot &bot, const Vector &angles)
	{
		return fabsf(Bot_NormalizeAngle(bot.desiredAngles.y - angles.y)) <= kFunAimTolerance &&
			fabsf(Bot_NormalizeAngle(bot.desiredAngles.x - angles.x)) <= kFunAimTolerance;
	}

	void FinishSwitch(CBot &bot, edict_t *ent, float cooldown, const char *why)
	{
		NoteCooldown(bot, ent, cooldown);

		BOT_VLOG("CrossBot: %s %s %s at (%.1f,%.1f,%.1f), leaving it for %.0fs\n", bot.name, why,
			CBotClassOf(ent), bot.goalPoint.x, bot.goalPoint.y, bot.goalPoint.z, cooldown);

		bot.fun.arrival = false;
		bot.fun.pressed = false;
	}

	bool Grounded(edict_t *pEdict)
	{
		return (static_cast<int>(pEdict->v.flags) & FL_ONGROUND) != 0;
	}

	const char *TrainPhaseName(BotFun::TrainPhase phase)
	{
		switch (phase)
		{
		case BotFun::TRAIN_APPROACH: return "approach";
		case BotFun::TRAIN_WAIT: return "wait";
		case BotFun::TRAIN_MOUNT: return "mount";
		case BotFun::TRAIN_DRIVE: return "drive";
		case BotFun::TRAIN_RELEASE: return "release";
		case BotFun::TRAIN_RIDE: return "ride";
		}
		return "unknown";
	}

	bool RunTrainRide(CBot &bot);

	bool RunTrainArrival(CBot &bot, edict_t *train)
	{
		BotFun::State &st = bot.fun;

		if (!EntryFor(train))
		{
			FinishSwitch(bot, train, kFunFailCooldown, "lost the track of");
			return false;
		}

		st.aboard = true;
		st.dismounting = false;
		st.arrival = false;
		st.pressed = false;
		st.offTrainSince = 0.f;
		st.boardTime = gpGlobals->time;
		st.phaseStart = gpGlobals->time;
		st.trainPhase = fabsf(train->v.speed) < kTrainStopEpsilon ? BotFun::TRAIN_APPROACH
			: BotFun::TRAIN_WAIT;

		return RunTrainRide(bot);
	}

	void LeaveTrain(CBot &bot)
	{
		bot.fun.aboard = false;
		bot.fun.dismounting = false;
		bot.fun.train.Clear();
	}

	bool TryTrainHop(CBot &bot, edict_t *train, float eta, const Vector &angles)
	{
		BotFun::State &st = bot.fun;
		edict_t *pEdict = bot.pEdict;

		float speed = fabsf(train->v.speed);
		if (speed < kTrainStopEpsilon)
			return false;

		float gravity = Bot_Physics(pEdict).gravity;
		if (gravity <= 0.f)
			return false;

		float driveSpeed = Bot_MaxSpeed(pEdict);
		if (driveSpeed <= 0.f)
			return false;

		float halfWidth = (train->v.maxs.y - train->v.mins.y) * 0.5f;

		float dx = st.boardDeck.x - pEdict->v.origin.x;
		float dy = st.boardDeck.y - pEdict->v.origin.y;
		float lateral = sqrtf(dx * dx + dy * dy);

		float rise = (st.boardDeck.z + kStandHalfHeight) - pEdict->v.origin.z;
		bool needJump = rise > 0.f;

		float clearFrom = 0.f;
		float clearTo = 0.f;

		if (needJump)
		{
			float disc = PmSim::kJumpImpulse * PmSim::kJumpImpulse - 2.f * gravity * rise;
			if (rise > PmSim::PlainJumpHeight(gravity) || disc < 0.f)
				return false;

			clearFrom = (PmSim::kJumpImpulse - sqrtf(disc)) / gravity;
			clearTo = (PmSim::kJumpImpulse + sqrtf(disc)) / gravity;
		}
		else
		{
			clearTo = sqrtf(2.f * -rise / gravity);
		}

		float brink = halfWidth + kTrainBrink;
		bool aloft = eta >= clearFrom && eta <= clearTo;

		if (!Grounded(pEdict))
		{
			DriveTo(bot, angles, st.boardDeck, false, 0, driveSpeed);
			return true;
		}

		if (lateral > brink)
		{
			if (eta > clearTo + (lateral - brink) / driveSpeed)
				return false;

			DriveTo(bot, angles, st.boardDeck, false, 0, driveSpeed);
			return true;
		}

		if (!aloft)
		{
			bot.moveForward = 0.f;
			bot.moveSide = 0.f;
			Bot_RunPlayerMove(bot, angles, 0.f, 0.f, 0);
			return true;
		}

		BOT_VLOG("CrossBot: %s hops for the tram, %s %.0f, aloft %.2f-%.2fs, lateral %.0f, "
			"deck due in %.2fs\n", bot.name, needJump ? "rise" : "drop", fabsf(rise),
			clearFrom, clearTo, lateral, eta);

		DriveTo(bot, angles, st.boardDeck, false, needJump ? IN_JUMP : 0, driveSpeed);
		return true;
	}

	bool RunTrainBoarding(CBot &bot, edict_t *train, const Entry &entry)
	{
		BotFun::State &st = bot.fun;
		edict_t *pEdict = bot.pEdict;

		bool stopped = fabsf(train->v.speed) < kTrainStopEpsilon;
		Vector deck = TrainDeck(train);
		Vector angles = Bot_UpdateFacing(bot);

		float budget = stopped ? kFunArrivalTimeout : kTrainCatchWindow + kFunArrivalTimeout;

		if (gpGlobals->time - st.boardTime > budget)
		{
			NoteCooldown(bot, train, kFunFailCooldown);

			BOT_VLOG("CrossBot: %s %s at (%.1f,%.1f,%.1f), leaving it for %.0fs\n", bot.name,
				stopped ? "could not get on the tram" : "missed the tram",
				pEdict->v.origin.x, pEdict->v.origin.y, pEdict->v.origin.z, kFunFailCooldown);

			LeaveTrain(bot);
			BotGoal::Clear(bot);
			return false;
		}

		if (stopped)
		{
			unsigned short extra = 0;
			if (Grounded(pEdict) && pEdict->v.origin.z < deck.z &&
				UseBoxDist(train, pEdict->v.origin) <= kTrainJumpGap)
				extra |= IN_JUMP;

			DriveTo(bot, angles, deck, false, extra);
			return true;
		}

		float eta = TrainEta(entry.track, train, st.boardArc);

		if (st.lastEta >= kTrainUnreachable)
		{
			float readiness = Readiness(bot);

			if (eta > kTrainCatchWindow)
			{
				if (BotFun::Memo *memo = MemoSlot(bot, train))
					memo->abandonReadiness = readiness;

				BOT_VLOG("CrossBot: %s reached the track, tram not due for %.1fs, "
					"leaving it until readiness beats %.2f\n", bot.name, eta, readiness);

				LeaveTrain(bot);
				BotGoal::Clear(bot);
				return false;
			}

			BOT_VLOG("CrossBot: %s reached the track, tram due in %.1fs, waiting\n", bot.name, eta);
		}
		st.lastEta = eta;

		if (TryTrainHop(bot, train, eta, angles))
			return true;

		bot.moveForward = 0.f;
		bot.moveSide = 0.f;
		Bot_RunPlayerMove(bot, angles, 0.f, 0.f, 0);
		return true;
	}

	bool RunTrainRide(CBot &bot)
	{
		BotFun::State &st = bot.fun;
		edict_t *pEdict = bot.pEdict;
		edict_t *train = st.train.Get();
		const Entry *entry = EntryFor(train);

		if (!train || !entry || !g_pGameProfile || g_pGameProfile->IsDead(pEdict))
		{
			BOT_VLOG("CrossBot: %s is off the tram\n", bot.name);
			LeaveTrain(bot);
			BotGoal::Clear(bot);
			return false;
		}

		bool onTrain = pEdict->v.groundentity == train;
		bool boarded = st.trainPhase >= BotFun::TRAIN_MOUNT;

		bot.lastProgressTime = gpGlobals->time;

		if (bot.sense.enemy.Get())
			st.lastSeenEnemy = gpGlobals->time;

		BotSense::ApplyLook(bot);

		if (!boarded)
		{
			if (!onTrain)
				return RunTrainBoarding(bot, train, *entry);

			st.offTrainSince = 0.f;
			st.boardTime = gpGlobals->time;
			st.phaseStart = gpGlobals->time;
			st.lastSeenEnemy = gpGlobals->time;
			st.trainPhase = BotFun::TRAIN_MOUNT;

			BOT_VLOG("CrossBot: %s boarded the tram\n", bot.name);
		}

		if (!onTrain)
		{
			if (st.offTrainSince == 0.f)
			{
				st.offTrainSince = gpGlobals->time;
			}
			else if (gpGlobals->time - st.offTrainSince > kTrainOffGrace)
			{
				BOT_VLOG("CrossBot: %s is off the tram\n", bot.name);
				LeaveTrain(bot);
				BotGoal::Clear(bot);
				return false;
			}
		}
		else
		{
			st.offTrainSince = 0.f;
		}

		if (st.trainPhase != BotFun::TRAIN_RIDE && onTrain)
		{
			float speed = train->v.speed;
			float top = fabsf(static_cast<float>(train->v.impulse));
			bool holding = (static_cast<int>(pEdict->v.flags) & FL_ONTRAIN) != 0;
			bool stopped = fabsf(speed) < kTrainStopEpsilon;

			BotFun::TrainPhase want;
			if (holding)
				want = fabsf(speed) + kTrainStopEpsilon < top ? BotFun::TRAIN_DRIVE
					: BotFun::TRAIN_RELEASE;
			else
				want = stopped ? BotFun::TRAIN_MOUNT : BotFun::TRAIN_RIDE;

			if (gpGlobals->time - st.boardTime > kTrainDriveTimeout)
				want = BotFun::TRAIN_RIDE;

			if (want != st.trainPhase)
			{
				if (want == BotFun::TRAIN_RELEASE)
					BOT_VLOG("CrossBot: %s is driving the tram, speed %.0f\n", bot.name,
						fabsf(speed));
				else if (want == BotFun::TRAIN_RIDE)
					BOT_VLOG("CrossBot: %s is riding the tram at %.0f\n", bot.name, fabsf(speed));

				st.trainPhase = want;
				st.phaseStart = gpGlobals->time;
			}

			if (want != BotFun::TRAIN_RIDE)
			{
				bool pulse = static_cast<int>((gpGlobals->time - st.phaseStart) / kFunPulse) % 2 == 0;
				Vector driveAngles = Bot_UpdateFacing(bot);

				if (want == BotFun::TRAIN_MOUNT)
				{
					Vector deck = TrainDeck(train);
					float ox = deck.x - pEdict->v.origin.x;
					float oy = deck.y - pEdict->v.origin.y;
					bool centred = sqrtf(ox * ox + oy * oy) <= kTrainCentreRadius;

					unsigned short buttons = 0;
					if (centred && pulse)
						buttons |= IN_USE;

					DriveTo(bot, driveAngles, deck, centred, buttons);
					return true;
				}

				unsigned short buttons = 0;
				if (pulse)
					buttons |= want == BotFun::TRAIN_DRIVE ? IN_FORWARD : IN_USE;

				bot.moveForward = 0.f;
				bot.moveSide = 0.f;
				Bot_RunPlayerMove(bot, driveAngles, 0.f, 0.f, buttons);
				return true;
			}
		}

		if (!st.dismounting && gpGlobals->time - st.lastSeenEnemy > kFunBoredom)
		{
			float readiness = Readiness(bot);
			if (BotFun::Memo *memo = MemoSlot(bot, train))
			{
				memo->abandonReadiness = readiness;
				memo->readyTime = gpGlobals->time + kFunTankFailCooldown;
			}

			BOT_VLOG("CrossBot: %s is bored of the tram, nobody around for %.1fs, readiness %.2f\n",
				bot.name, gpGlobals->time - st.lastSeenEnemy, readiness);

			st.dismounting = true;
			st.dismountStart = gpGlobals->time;
		}

		Vector angles = Bot_UpdateFacing(bot);

		if (st.dismounting)
		{
			const Nav::NavMesh *mesh = Nav::Get();
			Vector deck = TrainDeck(train);
			Vector right = TrainRight(train->v.angles.y);

			float halfWidth = (train->v.maxs.y - train->v.mins.y) * 0.5f;

			for (int s = -1; mesh && s <= 1; s += 2)
			{
				Vector land = deck + right * (static_cast<float>(s) * (halfWidth + kTrainHopRange));

				const Nav::Area *area = Nav::FindAreaNearest(*mesh, land);
				if (!area)
					continue;

				Vector point = BotItems::ClampToArea(*area,
					Vector(land.x, land.y, area->centroid.z));
				float dx = point.x - deck.x;
				float dy = point.y - deck.y;
				if (sqrtf(dx * dx + dy * dy) > halfWidth + kTrainHopRange)
					continue;
				if (deck.z - point.z > kTrainSafeDrop)
					continue;
				if (!HullClear(train, deck + Vector(0.f, 0.f, 1.f),
					Vector(point.x, point.y, deck.z + 1.f)))
					continue;

				BOT_VLOG("CrossBot: %s jumped off the tram onto area %u\n", bot.name, area->id);

				LeaveTrain(bot);
				DriveTo(bot, angles, point, false, IN_JUMP);
				BotGoal::Clear(bot);
				return true;
			}

			if (gpGlobals->time - st.dismountStart > kTrainRideMax)
			{
				BOT_VLOG("CrossBot: %s gave up dismounting after %.0fs with nowhere to land\n",
					bot.name, gpGlobals->time - st.dismountStart);

				LeaveTrain(bot);
				BotGoal::Clear(bot);
				return false;
			}
		}

		Vector deck = TrainDeck(train);
		float ox = deck.x - pEdict->v.origin.x;
		float oy = deck.y - pEdict->v.origin.y;

		if (sqrtf(ox * ox + oy * oy) > kTrainCentreRadius)
		{
			DriveTo(bot, angles, deck, false, 0);
			return true;
		}

		bot.moveForward = 0.f;
		bot.moveSide = 0.f;
		Bot_RunPlayerMove(bot, angles, 0.f, 0.f, 0);
		return true;
	}
}

namespace BotFun
{

bool Enabled()
{
	cvar_t *cvar = g_engfuncs.pfnCVarGetPointer("crossbot_fun");
	return !cvar || cvar->value != 0.f;
}

bool DebugEnabled()
{
#if BOT_VLOG_ON
	cvar_t *cvar = g_engfuncs.pfnCVarGetPointer("crossbot_fun_debug");
	return cvar && cvar->value != 0.f;
#else
	return false;
#endif
}

const char *KindName(Kind kind)
{
	switch (kind)
	{
	case FUN_TANK: return "tank";
	case FUN_TRAIN: return "tram";
	default: break;
	}
	return "switch";
}

const char *ActionName(Action action)
{
	switch (action)
	{
	case FUN_ACT_TOUCH: return "touch";
	case FUN_ACT_SHOOT: return "shoot";
	default: break;
	}
	return "use";
}

void ResetMap()
{
	g_table.clear();
	g_tableMap[0] = '\0';
	g_tableBuilt = false;
}

void Collect(const CBot &bot, const Nav::NavMesh &mesh, std::vector<BotGoal::Candidate> &out)
{
	if (!Enabled() || !g_pGameProfile || bot.currentAreaId < 0)
		return;

	BuildTable();

	const Vector &from = bot.pEdict->v.origin;

	for (Entry &entry : g_table)
	{
		edict_t *ent = EntryEdict(entry);
		if (!ent || Blocked(bot, ent))
			continue;

		if (entry.kind == FUN_TRAIN)
		{
			BuildBoardPoints(mesh, ent, entry);

			bool stopped = fabsf(ent->v.speed) < kTrainStopEpsilon;
			float here = stopped ? ProjectOnTrack(entry.track, ent->v.origin) : 0.f;

			for (const BoardPoint &b : entry.board)
			{
				if (stopped)
				{
					float ahead = ArcForward(entry.track, here, b.arc, 1.f);
					float behind = ArcForward(entry.track, b.arc, here, 1.f);
					if (std::min(ahead, behind) > kTrainBoardArcSlack)
						continue;
				}

				BotGoal::Candidate cand;
				cand.tier = BotGoal::GOAL_FUN;
				cand.ent = ent;
				cand.areaId = b.areaId;
				cand.point = b.point;
				cand.payload = PackPayload(FUN_TRAIN, FUN_ACT_USE);
				out.push_back(cand);
			}
			continue;
		}

		if (entry.kind == FUN_TANK)
		{
			edict_t *tank = TankOfControls(ent);
			if (!TankFree(tank, bot.pEdict))
				continue;

			std::vector<BotItems::Placement> places;
			CollectUsePoints(mesh, ent, from, places);

			for (const BotItems::Placement &p : places)
			{
				BotGoal::Candidate cand;
				cand.tier = BotGoal::GOAL_FUN;
				cand.ent = ent;
				cand.areaId = p.areaId;
				cand.point = p.point;
				cand.payload = PackPayload(FUN_TANK, FUN_ACT_USE);
				out.push_back(cand);
			}
			continue;
		}

		if (!SwitchReady(ent, entry.action))
			continue;

		if (entry.action == FUN_ACT_TOUCH)
		{
			std::vector<BotItems::Placement> places;
			CollectVolume(mesh, ent, from, places);

			for (const BotItems::Placement &p : places)
			{
				BotGoal::Candidate cand;
				cand.tier = BotGoal::GOAL_FUN;
				cand.ent = ent;
				cand.areaId = p.areaId;
				cand.point = p.point;
				cand.payload = PackPayload(FUN_SWITCH, FUN_ACT_TOUCH);
				out.push_back(cand);
			}
			continue;
		}

		Vector center = BrushCenter(ent);
		const Nav::Area *area = BotItems::StandArea(mesh, center);
		if (!area)
			continue;

		BotGoal::Candidate cand;
		cand.tier = BotGoal::GOAL_FUN;
		cand.ent = ent;
		cand.areaId = area->id;
		cand.point = BotItems::ClampToArea(*area, center);
		cand.payload = PackPayload(FUN_SWITCH, entry.action);
		out.push_back(cand);
	}
}

void NoteDeath(const CBot &bot)
{
	if (!DebugEnabled())
		return;

	edict_t *train = nullptr;
	for (const Entry &entry : g_table)
	{
		if (entry.kind == FUN_TRAIN)
		{
			train = EntryEdict(entry);
			break;
		}
	}

	if (!train)
		return;

	const Vector &at = bot.pEdict->v.origin;

	BOT_VLOG("CrossBot: %s died at (%.0f,%.0f,%.0f), %.0f from the tram doing %.0f, "
		"onground %d, tram phase %s\n", bot.name, at.x, at.y, at.z,
		UseBoxDist(train, at), train->v.speed, Grounded(bot.pEdict) ? 1 : 0,
		bot.fun.aboard ? TrainPhaseName(bot.fun.trainPhase) : "not engaged");
}

void Prioritize(std::vector<BotGoal::Candidate> &cands)
{
	bool hasSwitch = false;
	for (const BotGoal::Candidate &cand : cands)
	{
		if (PayloadKind(cand.payload) == FUN_SWITCH)
		{
			hasSwitch = true;
			break;
		}
	}

	if (!hasSwitch)
		return;

	size_t before = cands.size();

	cands.erase(std::remove_if(cands.begin(), cands.end(),
		[](const BotGoal::Candidate &cand) {
			return PayloadKind(cand.payload) != FUN_SWITCH;
		}), cands.end());

	if (DebugEnabled() && cands.size() != before)
		BOT_VLOG("CrossBot: fun priority kept %d switch candidates, dropped %d others\n",
			static_cast<int>(cands.size()), static_cast<int>(before - cands.size()));
}

void Accept(CBot &bot, const BotGoal::Candidate &cand)
{
	State &st = bot.fun;

	st.goal.Set(cand.ent);
	st.kind = PayloadKind(cand.payload);
	st.action = PayloadAction(cand.payload);
	st.arrival = false;
	st.pressed = false;
	st.startTime = 0.f;
	st.holdUntil = 0.f;
	st.startToggle = -1;
	st.shots = 0;

	st.tank.Clear();
	if (st.kind == FUN_TANK)
	{
		if (edict_t *tank = TankOfControls(cand.ent))
			st.tank.Set(tank);
	}

	if (st.kind == FUN_TRAIN)
	{
		st.train.Set(cand.ent);
		st.trainPhase = TRAIN_APPROACH;
		st.phaseStart = gpGlobals->time;
		st.boardArc = 0.f;
		st.boardDeck = cand.point;
		st.lastEta = kTrainUnreachable;
		st.offTrainSince = 0.f;

		if (const Entry *entry = EntryFor(cand.ent))
		{
			for (const BoardPoint &b : entry->board)
			{
				if (b.areaId == cand.areaId)
				{
					st.boardArc = b.arc;
					st.boardDeck = b.deck;
					break;
				}
			}
		}
	}
}

bool ValidFor(const CBot &bot, edict_t *ent, int payload)
{
	if (!ent || !Enabled())
		return false;

	if (Blocked(bot, ent))
		return false;

	if (PayloadKind(payload) == FUN_TRAIN)
		return true;

	if (PayloadKind(payload) == FUN_TANK)
	{
		if (bot.fun.mounted && bot.fun.goal.Is(ent))
			return true;
		return TankFree(TankOfControls(ent), bot.pEdict);
	}

	return SwitchReady(ent, PayloadAction(payload));
}

bool Valid(const CBot &bot)
{
	const State &st = bot.fun;
	return ValidFor(bot, st.goal.Get(), PackPayload(st.kind, st.action));
}

void Forget(CBot &bot)
{
	State &st = bot.fun;
	st.goal.Clear();
	st.tank.Clear();
	st.arrival = false;
	st.pressed = false;
	st.startTime = 0.f;
	st.holdUntil = 0.f;
	st.startToggle = -1;
}

bool Mounted(const CBot &bot)
{
	return bot.fun.mounted;
}

bool Aboard(const CBot &bot)
{
	return bot.fun.aboard;
}

bool Pressing(const CBot &bot)
{
	const State &st = bot.fun;

	if (st.kind == FUN_TRAIN)
		return st.aboard && (st.trainPhase == TRAIN_MOUNT || st.trainPhase == TRAIN_RELEASE);

	return st.arrival && st.action != FUN_ACT_SHOOT;
}

bool RunArrival(CBot &bot)
{
	State &st = bot.fun;
	edict_t *pEdict = bot.pEdict;
	edict_t *ent = st.goal.Get();
	if (!ent)
		return false;

	if (!st.arrival)
	{
		st.arrival = true;
		st.pressed = false;
		st.startTime = gpGlobals->time;
		st.holdUntil = 0.f;
		st.startToggle = -1;
		PeekInt(ent, g_offsets.tog_toggleState, st.startToggle);

		BOT_VLOG("CrossBot: %s reached the %s %s at (%.1f,%.1f,%.1f), going to %s it\n", bot.name,
			KindName(st.kind), CBotClassOf(ent), bot.goalPoint.x, bot.goalPoint.y, bot.goalPoint.z,
			ActionName(st.action));
	}

	bot.lastProgressTime = gpGlobals->time;

	if (st.kind == FUN_SWITCH && st.action == FUN_ACT_SHOOT)
	{
		if (BotBreak::Active(bot))
			return true;

		int state = -1;
		PeekInt(ent, g_offsets.tog_toggleState, state);
		if (state != st.startToggle || ent->v.health <= 0.f)
		{
			FinishSwitch(bot, ent, kFunSwitchCooldown, "shot open");
			return false;
		}

		if (gpGlobals->time - st.startTime > kFunArrivalTimeout || !BotBreak::Engage(bot, ent))
		{
			FinishSwitch(bot, ent, kFunFailCooldown, "could not shoot");
			return false;
		}

		return true;
	}

	if (st.kind == FUN_TRAIN)
		return RunTrainArrival(bot, ent);

	Vector center = BrushCenter(ent);

	if (st.action == FUN_ACT_TOUCH)
	{
		bool ducking = (pEdict->v.flags & FL_DUCKING) != 0;
		bool inside = BotItems::OriginInsideVolume(ent, pEdict->v.origin, ducking ? 3 : 1);

		BotSense::ApplyLook(bot);
		Vector angles = Bot_UpdateFacing(bot);

		if (inside)
		{
			FinishSwitch(bot, ent, std::max(ent->v.nextthink - gpGlobals->time, kFunSwitchCooldown),
				"walked into");
			return false;
		}

		if (gpGlobals->time - st.startTime > kFunArrivalTimeout)
		{
			FinishSwitch(bot, ent, kFunFailCooldown, "could not reach");
			return false;
		}

		DriveTo(bot, angles, bot.goalPoint, false, 0);
		return true;
	}

	Vector eye = pEdict->v.origin + pEdict->v.view_ofs;
	Vector useDir = UseDir(ent, eye);
	FaceDir(bot, useDir);
	Vector angles = Bot_UpdateFacing(bot);

	float useDist = UseBoxDist(ent, pEdict->v.origin);
	bool inRange = useDist <= kFunUseRange;
	bool aimed = Aimed(bot, angles) && DotProduct(useDir, ForwardFromAngles(angles)) > kUseDotFloor;

	unsigned short buttons = 0;
	if (inRange && aimed)
	{
		float since = gpGlobals->time - st.startTime;
		if (static_cast<int>(since / kFunPulse) % 2 == 0)
			buttons |= IN_USE;

		if (!st.pressed)
		{
			st.pressed = true;
			st.holdUntil = gpGlobals->time + kFunMinHold;

			BOT_VLOG("CrossBot: %s pressed USE at the %s %s, %.1f out\n", bot.name,
				KindName(st.kind), CBotClassOf(ent), useDist);
		}
	}

	if (st.kind == FUN_TANK)
	{
		edict_t *tank = st.tank.Get();
		if (IsController(pEdict, tank))
		{
			st.mounted = true;
			st.mountTime = gpGlobals->time;
			st.lastSeenEnemy = gpGlobals->time;
			st.arrival = false;
			st.pressed = false;
			st.shots = 0;

			BOT_VLOG("CrossBot: %s mounted the tank %s at (%.1f,%.1f,%.1f)\n", bot.name,
				CBotClassOf(tank), pEdict->v.origin.x, pEdict->v.origin.y, pEdict->v.origin.z);
			return true;
		}
	}
	else if (st.pressed)
	{
		int state = -1;
		PeekInt(ent, g_offsets.tog_toggleState, state);
		if (state != st.startToggle)
		{
			float wait = 0.f;
			PeekFloat(ent, g_offsets.tog_flWait, wait);
			FinishSwitch(bot, ent, std::max(wait, kFunSwitchCooldown), "pressed");
			return false;
		}
	}

	if (gpGlobals->time - st.startTime > kFunArrivalTimeout)
	{
		const char *why = "could not reach";
		if (st.pressed)
			why = "got nothing out of";
		else if (inRange)
			why = "could not aim at";

		FinishSwitch(bot, ent, kFunFailCooldown, why);
		return false;
	}

	DriveTo(bot, angles, center, inRange, buttons);
	return true;
}

bool Run(CBot &bot)
{
	State &st = bot.fun;

	if (st.aboard)
		return RunTrainRide(bot);

	if (!st.mounted)
		return false;

	edict_t *pEdict = bot.pEdict;
	edict_t *tank = st.tank.Get();

	if (!tank || !IsController(pEdict, tank) || !g_pGameProfile ||
		g_pGameProfile->IsDead(pEdict))
	{
		BOT_VLOG("CrossBot: %s is off the tank\n", bot.name);
		st.mounted = false;
		BotGoal::Clear(bot);
		return false;
	}

	bot.lastProgressTime = gpGlobals->time;

	TankCone cone = ReadCone(tank);

	edict_t *hurt = BotCombat::DamageSource(pEdict);
	if (hurt && gpGlobals->time - st.mountTime > kFunMountGrace)
	{
		float yaw = 0.f, pitch = 0.f;
		bool covered = AnglesToward(tank, hurt->v.origin + hurt->v.view_ofs, yaw, pitch) &&
			InCone(cone, yaw, pitch);

		if (!covered)
		{
			NoteCooldown(bot, st.goal.Get(), kFunTankShotCooldown);

			BOT_VLOG("CrossBot: %s is shot from behind the tank by %s, bailing out\n", bot.name,
				CBotPlayerNameOf(hurt));

			st.mounted = false;
			BotGoal::Clear(bot);
			return false;
		}
	}

	edict_t *enemy = bot.sense.enemy.Get();
	Vector target = bot.sense.enemyPos;
	if (enemy)
	{
		target = enemy->v.origin + enemy->v.view_ofs;
		st.lastSeenEnemy = gpGlobals->time;
	}

	if (gpGlobals->time - st.lastSeenEnemy > kFunBoredom)
	{
		float readiness = Readiness(bot);
		if (BotFun::Memo *memo = MemoSlot(bot, st.goal.Get()))
		{
			memo->abandonReadiness = readiness;
			memo->readyTime = gpGlobals->time + kFunTankFailCooldown;
		}

		BOT_VLOG("CrossBot: %s is bored of the tank, nobody around for %.1fs, readiness %.2f\n",
			bot.name, gpGlobals->time - st.lastSeenEnemy, readiness);

		st.mounted = false;
		BotGoal::Clear(bot);
		return false;
	}

	float yaw = 0.f, pitch = 0.f;
	bool haveTarget = enemy && AnglesToward(tank, target, yaw, pitch);
	bool onTarget = false;

	if (haveTarget && InCone(cone, yaw, pitch))
	{
		bot.desiredAngles = Vector(pitch, yaw, 0.f);
		onTarget = fabsf(AngleDistance(yaw, tank->v.angles.y)) <= kTankAimTolerance &&
			fabsf(AngleDistance(pitch, tank->v.angles.x)) <= kTankAimTolerance;
	}
	else if (cone.valid)
	{
		bot.desiredAngles = Vector(cone.pitchCenter, cone.yawCenter, 0.f);
	}

	Vector angles = Bot_UpdateFacing(bot);

	unsigned short buttons = 0;
	if (onTarget)
	{
		buttons |= IN_ATTACK;
		++st.shots;
	}

	bot.moveForward = 0.f;
	bot.moveSide = 0.f;
	Bot_RunPlayerMove(bot, angles, 0.f, 0.f, buttons);
	return true;
}

void OnArrived(CBot &bot)
{
	edict_t *ent = bot.fun.goal.Get();
	if (ent && !bot.fun.mounted)
		NoteCooldown(bot, ent, kFunFailCooldown);

	Forget(bot);
}

void OnRepathFailed(CBot &bot)
{
	edict_t *ent = bot.fun.goal.Get();
	if (ent)
	{
		BOT_VLOG("CrossBot: %s could not plan a route to the %s %s, leaving it for %.0fs\n",
			bot.name, KindName(bot.fun.kind), CBotClassOf(ent), kFunFailCooldown);
		NoteCooldown(bot, ent, kFunFailCooldown);
	}

	Forget(bot);
}

void OnBlocked(CBot &bot, const char *what)
{
	edict_t *ent = bot.fun.goal.Get();
	if (ent)
	{
		BOT_VLOG("CrossBot: %s could not get past %s reaching the %s %s, leaving it for %.0fs\n",
			bot.name, what, KindName(bot.fun.kind), CBotClassOf(ent), kFunFailCooldown);
		NoteCooldown(bot, ent, kFunFailCooldown);
	}

	Forget(bot);
}

void OnRespawn(CBot &bot)
{
	bot.fun.mounted = false;
	bot.fun.aboard = false;
	bot.fun.dismounting = false;
	bot.fun.train.Clear();
	bot.fun.memo.clear();
	Forget(bot);
}

void PrintCandidates(const Nav::NavMesh &mesh)
{
	BuildTable();

	for (Entry &entry : g_table)
	{
		edict_t *ent = EntryEdict(entry);
		if (!ent)
			continue;

		Vector center = BrushCenter(ent);

		char areaBuf[80] = "none";
		const char *why = "ready";

		if (entry.kind == FUN_TRAIN)
		{
			BuildBoardPoints(mesh, ent, entry);

			int written = 0;
			for (size_t p = 0; p < entry.board.size() && written >= 0 &&
				written + 8 < static_cast<int>(sizeof(areaBuf)); ++p)
				written += snprintf(areaBuf + written, sizeof(areaBuf) - written, "%s%u",
					p == 0 ? "" : ",", entry.board[p].areaId);

			if (entry.board.empty())
				why = "no nav area";

			Vector deck = TrainDeck(ent);

			char line[400];
			snprintf(line, sizeof(line),
				"CrossBot: fun tram use %s at (%.1f,%.1f,%.1f) speed %.0f deck (%.1f,%.1f,%.1f) "
				"area %s -> %s, %s\n",
				CBotClassOf(ent), center.x, center.y, center.z, ent->v.speed, deck.x, deck.y, deck.z,
				entry.board.empty() ? "none" : areaBuf, entry.terminal, why);
			SERVER_PRINT(line);

			if (DebugEnabled())
			{
				for (const BoardPoint &b : entry.board)
				{
					char spot[200];
					snprintf(spot, sizeof(spot),
						"CrossBot:   tram board area %u at (%.1f,%.1f,%.1f), deck (%.1f,%.1f,%.1f),"
						" arc %.0f\n", b.areaId, b.point.x, b.point.y, b.point.z, b.deck.x, b.deck.y,
						b.deck.z, b.arc);
					SERVER_PRINT(spot);
				}
			}
			continue;
		}

		if (entry.kind == FUN_TANK)
		{
			edict_t *tank = TankOfControls(ent);
			if (!TankFree(tank, nullptr))
				why = "taken or locked";
		}
		else if (!SwitchReady(ent, entry.action))
		{
			why = "waiting";
		}

		std::vector<BotItems::Placement> places;
		if (entry.kind == FUN_TANK)
		{
			CollectUsePoints(mesh, ent, center, places);
		}
		else if (entry.action == FUN_ACT_TOUCH)
		{
			CollectVolume(mesh, ent, center, places);
		}
		else if (const Nav::Area *area = BotItems::StandArea(mesh, center))
		{
			BotItems::Placement p;
			p.areaId = area->id;
			p.point = BotItems::ClampToArea(*area, center);
			places.push_back(p);
		}

		if (places.empty())
			why = "no nav area";

		int written = 0;
		for (size_t p = 0; p < places.size() && written >= 0 &&
			written + 8 < static_cast<int>(sizeof(areaBuf)); ++p)
			written += snprintf(areaBuf + written, sizeof(areaBuf) - written, "%s%u",
				p == 0 ? "" : ",", places[p].areaId);

		char msg[400];
		snprintf(msg, sizeof(msg),
			"CrossBot: fun %s %s %s at (%.1f,%.1f,%.1f) area %s -> %s, %s\n",
			KindName(entry.kind), ActionName(entry.action), CBotClassOf(ent), center.x, center.y,
			center.z, places.empty() ? "none" : areaBuf, entry.terminal, why);
		SERVER_PRINT(msg);
	}

	char summary[96];
	snprintf(summary, sizeof(summary), "CrossBot: %zu fun entities\n", g_table.size());
	SERVER_PRINT(summary);
}

}
