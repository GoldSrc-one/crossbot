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
#include "../nav/nav_math.h"
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

	constexpr float kMoveDeadzone = 0.01f;

	constexpr int kHullDuck = 3;

	struct Entry
	{
		int index = 0;
		BotFun::Kind kind = BotFun::FUN_SWITCH;
		BotFun::Action action = BotFun::FUN_ACT_USE;
		char terminal[32] = {};
	};

	std::vector<Entry> g_table;
	char g_tableMap[64] = {};
	bool g_tableBuilt = false;

	void *Priv(edict_t *ent)
	{
		return ent ? ent->pvPrivateData : nullptr;
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
		unsigned short extra)
	{
		edict_t *pEdict = bot.pEdict;

		float drive = 0.f;
		Vector moveDir(0.f, 0.f, 0.f);
		Vector flat(to.x - pEdict->v.origin.x, to.y - pEdict->v.origin.y, 0.f);
		float dist = sqrtf(flat.x * flat.x + flat.y * flat.y);

		if (!stop && dist > kFunStandRadius)
		{
			moveDir = Norm2D(flat);
			drive = Bot_MaxSpeed(pEdict) * PmSim::kDuckingMultiplier;

			NavFollow::Drive solved = NavFollow::SolveDrive(moveDir * drive, pEdict->v.velocity,
				Bot_DrivePhysics(pEdict));
			if (solved.wishSpeed > 0.f)
			{
				moveDir = solved.wishDir;
				drive = solved.wishSpeed;
			}
		}

		if ((pEdict->v.flags & FL_DUCKING) != 0 && drive > 0.f)
			drive /= PmSim::kDuckingMultiplier;

		float fmove = 0.f, smove = 0.f;
		Bot_DecomposeMove(angles.y + pEdict->v.punchangle.y, moveDir, drive, fmove, smove);
		bot.moveForward = fmove;
		bot.moveSide = smove;

		unsigned short buttons = extra;
		if (fmove > kMoveDeadzone) buttons |= IN_FORWARD;
		else if (fmove < -kMoveDeadzone) buttons |= IN_BACK;
		if (smove > kMoveDeadzone) buttons |= IN_MOVERIGHT;
		else if (smove < -kMoveDeadzone) buttons |= IN_MOVELEFT;

		Bot_RunPlayerMove(bot, angles, fmove, smove, buttons);
	}

	void FaceDir(CBot &bot, const Vector &dir)
	{
		float flat = sqrtf(dir.x * dir.x + dir.y * dir.y);
		if (flat > 0.0001f || fabsf(dir.z) > 0.0001f)
			bot.desiredAngles = Vector(-atan2f(dir.z, flat) * (180.f / kPiF),
				atan2f(dir.y, dir.x) * (180.f / kPiF), 0.f);
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
	return kind == FUN_TANK ? "tank" : "switch";
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

	for (const Entry &entry : g_table)
	{
		edict_t *ent = EntryEdict(entry);
		if (!ent || Blocked(bot, ent))
			continue;

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
}

bool ValidFor(const CBot &bot, edict_t *ent, int payload)
{
	if (!ent || !Enabled())
		return false;

	if (Blocked(bot, ent))
		return false;

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

bool Pressing(const CBot &bot)
{
	return bot.fun.arrival && bot.fun.action != FUN_ACT_SHOOT;
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
	bot.fun.memo.clear();
	Forget(bot);
}

void PrintCandidates(const Nav::NavMesh &mesh)
{
	BuildTable();

	for (const Entry &entry : g_table)
	{
		edict_t *ent = EntryEdict(entry);
		if (!ent)
			continue;

		Vector center = BrushCenter(ent);

		char areaBuf[80] = "none";
		const char *why = "ready";

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
