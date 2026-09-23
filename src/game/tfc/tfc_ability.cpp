#include <extdll.h>
#include <meta_api.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>

#include "../../bot/bot.h"
#include "../../bot/bot_aim.h"
#include "../../bot/bot_difficulty.h"
#include "../../bot/bot_fire.h"
#include "../../bot/bot_log.h"
#include "../../bot/bot_move.h"
#include "../../bot/bot_move_nav.h"
#include "../../bot/bot_sense.h"
#include "../../bot/bot_strategic.h"
#include "../../nav/nav.h"
#include "../../nav/nav_path.h"
#include "../../nav/nav_query.h"
#include "../../nav/path_follower.h"
#include "../crossplay.h"
#include "../ent_handle.h"
#include "../fake_cmd.h"
#include "../gamedata.h"
#include "../objectives.h"
#include "../teams.h"
#include "tfc_ability.h"
#include "tfc_weapons.h"

namespace
{
	using BotAbility::State;

	constexpr int kClassScout = 1;
	constexpr int kClassDemoman = 4;
	constexpr int kClassMedic = 5;
	constexpr int kClassSpy = 8;
	constexpr int kClassEngineer = 9;

	constexpr int kDeadFeign = 5;
	constexpr int kMaxPlayers = 32;
	constexpr int kHullHuman = 1;

	constexpr float kDecideInterval = 0.5f;
	constexpr float kEngagedMemory = 2.f;
	constexpr float kSelectInterval = 0.5f;
	constexpr float kMoveDeadzone = 0.01f;
	constexpr float kBrakeGain = 4.f;
	constexpr float kLookReach = 100.f;
	constexpr float kAimTolerance = 4.f;

	constexpr float kHealRatio = 0.95f;
	constexpr float kHealStop = 40.f;
	constexpr float kCallRatio = 0.5f;
	constexpr float kCallMemory = 30.f;
	constexpr float kMeleeReach = 64.f;
	constexpr float kHealTimeout = 12.f;

	constexpr int kBuildDispenser = 1;
	constexpr int kBuildSentry = 2;
	constexpr int kBuildEntrance = 4;
	constexpr int kBuildExit = 5;
	constexpr int kCellsDispenser = 100;
	constexpr int kCellsSentry = 130;
	constexpr int kCellsTeleporter = 125;
	constexpr int kCellsUpgrade = 130;
	constexpr float kBuildRoom = 64.f;
	constexpr float kSpawnRadius = 500.f;
	constexpr float kBuildSettle = 0.3f;
	constexpr float kBuildConfirm = 1.f;
	constexpr float kBuildTimeout = 12.f;
	constexpr float kBuildRetry = 10.f;
	constexpr float kFaceTolerance = 5.f;
	constexpr float kSettleSpeed = 20.f;

	constexpr float kRepairRange = 700.f;
	constexpr float kRepairStop = 44.f;
	constexpr float kRepairTimeout = 15.f;

	constexpr float kStabRange = 300.f;
	constexpr float kStabLost = 400.f;
	constexpr float kStabBehind = -0.3f;
	constexpr float kStabSpotted = 0.3f;
	constexpr float kStabStop = 24.f;
	constexpr float kStabReach = 56.f;
	constexpr float kStabTimeout = 4.f;
	constexpr float kDisguiseRetry = 5.f;

	constexpr float kFeignMin = 3.f;
	constexpr float kFeignMax = 12.f;
	constexpr float kFeignWatchRange = 500.f;
	constexpr float kFeignWatchDot = 0.8f;
	constexpr float kFeignCommandGap = 1.f;

	constexpr int kMaxPipes = 6;
	constexpr float kPipeShare = 0.7f;
	constexpr float kPipeReachMax = 500.f;
	constexpr float kPipeTimeout = 6.f;
	constexpr float kPipeCalm = 1.f;
	constexpr float kPipeTrigger = 120.f;
	constexpr float kPipeSelfClear = 150.f;
	constexpr float kPipeFriendClear = 400.f;
	constexpr float kDetonateInterval = 0.25f;
	constexpr float kDetonateGap = 1.f;

	constexpr float kPadEngage = 64.f;
	constexpr float kPadStop = 4.f;
	constexpr float kPadTimeout = 10.f;
	constexpr float kPadApproachGain = 4.f;
	constexpr float kPadBusyWait = 3.f;

	constexpr float kPadStillSpeed = 1.f;
	constexpr float kPadStillGrace = 0.3f;

	constexpr int kTeleEntrance = 4;
	constexpr int kTeleExit = 5;
	constexpr int kTeleReady = 2;
	constexpr int kTeleSending = 3;

	constexpr float kSentryTurnDelay = 0.3f;
	constexpr float kSentryTurnGap = 0.25f;
	constexpr int kSentryTurnStep = 45;

	constexpr float kHurtHold = 2.f;

	constexpr float kTouchRange = 800.f;
	constexpr float kTouchFov = 0.5f;
	constexpr float kTouchGap = 2.f;
	constexpr float kTouchTimeout = 4.f;
	constexpr float kTouchCooldown = 30.f;
	constexpr float kTouchFriendFar = 4096.f;

	constexpr float kDisarmRange = 1000.f;
	constexpr float kDisarmTimeout = 8.f;
	constexpr float kDisarmNear = 64.f;

	constexpr float kDetpackReach = 1400.f;
	constexpr float kDetpackArrive = 96.f;
	constexpr float kDetpackStack = 200.f;
	constexpr float kDetpackFriendClear = 65.f;
	constexpr float kDetpackScaredClear = 700.f;
	constexpr float kDetpackArm = 3.f;
	constexpr float kDetpackConfirm = 1.f;
	constexpr float kDetpackArmSlack = 1.f;
	constexpr int kDetpackFuseMin = 5;
	constexpr int kDetpackFuseMax = 50;
	constexpr int kDetpackFuseObjective = 10;
	constexpr float kDetpackSafeDist = 800.f;
	constexpr float kDetpackRetry = 10.f;
	constexpr float kDetpackAfterBlast = 5.f;
	constexpr float kRetreatStop = 32.f;

	constexpr float kDiscardRetry = 60.f;
	constexpr float kDiscardTimeout = 1.5f;

	constexpr int kGoalActivateTouch = 1;
	constexpr int kGoalStateActive = 1;
	constexpr int kGoalStateRemoved = 3;
	constexpr int kTfcTeams = 4;
	constexpr int kGrenadeCap = 4;
	constexpr float kStationFull = 100.f;

	const int kDisguises[] = { 1, 2, 3, 4, 5, 6, 7, 9 };

	float g_feignStart[kMaxPlayers + 1];
	float g_feignCommand[kMaxPlayers + 1];
	float g_saveMe[kMaxPlayers + 1];

	int FieldInt(const edict_t *ent, int offset)
	{
		void *priv = CBotPrivateData(ent);
		return (priv && offset >= 0) ? GameData::Peek<int>(priv, offset) : 0;
	}

	edict_t *RealOwner(const edict_t *ent)
	{
		void *priv = CBotPrivateData(ent);
		if (!priv || g_offsets.ent_realOwner < 0)
			return nullptr;
		edict_t *owner = GameData::Peek<edict_t *>(priv, g_offsets.ent_realOwner);
		if (!owner || owner->free)
			return nullptr;
		return owner->serialnumber == GameData::Peek<int>(priv, g_offsets.ent_realOwner + 4) ? owner : nullptr;
	}

	float FieldFloat(const edict_t *ent, int offset)
	{
		void *priv = CBotPrivateData(ent);
		return (priv && offset >= 0) ? GameData::Peek<float>(priv, offset) : 0.f;
	}

	float TimeLeft(float until, float now, float cap)
	{
		float left = until - now;
		return (left > 0.f && left <= cap) ? left : 0.f;
	}

	bool CarryingGoal(const edict_t *player)
	{
		for (int i = gpGlobals->maxClients + 1; i < gpGlobals->maxEntities; ++i)
		{
			edict_t *ent = g_engfuncs.pfnPEntityOfEntIndex(i);
			if (!ent || ent->free || (ent->v.owner != player && ent->v.aiment != player))
				continue;
			if (strcmp(CBotClassOf(ent), "item_tfgoal") == 0)
				return true;
		}
		return false;
	}

	bool IsDisguised(const edict_t *ent)
	{
		if (FieldInt(ent, g_offsets.ent_isUndercover) != 1)
			return false;
		int team = FieldInt(ent, g_offsets.ent_undercoverTeam);
		return team != 0 && team != ent->v.team && !CarryingGoal(ent);
	}

	float BoxGap2D(const edict_t *a, const edict_t *b)
	{
		float gapX = std::max(a->v.absmin.x - b->v.absmax.x, b->v.absmin.x - a->v.absmax.x);
		float gapY = std::max(a->v.absmin.y - b->v.absmax.y, b->v.absmin.y - a->v.absmax.y);
		return std::max(gapX, gapY);
	}

	bool DetpackReady(const edict_t *ent)
	{
		return ent->v.playerclass == kClassDemoman && FieldInt(ent, g_offsets.ent_ammoDetpack) >= 1 &&
			FieldInt(ent, g_offsets.ent_isDetpacking) == 0;
	}

	bool IsPackClass(const char *classname)
	{
		return strcmp(classname, "info_tfgoal") == 0 || strcmp(classname, "i_t_g") == 0;
	}

	bool IsPack(const edict_t *ent)
	{
		if ((FieldInt(ent, g_offsets.ent_goalActivation) & kGoalActivateTouch) == 0)
			return false;
		if (ent->v.health < 0.f || ent->v.frags > 0.f)
			return false;
		for (int i = 0; i < kTfcTeams; ++i)
		{
			if (g_offsets.ent_increaseTeam >= 0 && FieldInt(ent, g_offsets.ent_increaseTeam + i * 4) > 0)
				return false;
		}
		return ent->v.health > 0.f || ent->v.armorvalue > 0.f ||
			FieldInt(ent, g_offsets.ent_ammoShells) > 0 || FieldInt(ent, g_offsets.ent_ammoNails) > 0 ||
			FieldInt(ent, g_offsets.ent_ammoCells) > 0 || FieldInt(ent, g_offsets.ent_ammoRockets) > 0 ||
			FieldInt(ent, g_offsets.ent_noGrenades1) > 0 || FieldInt(ent, g_offsets.ent_noGrenades2) > 0 ||
			FieldInt(ent, g_offsets.ent_ammoDetpack) > 0;
	}

	bool NeedsAmmo(const edict_t *player, const edict_t *pack, int ammo, int maxAmmo)
	{
		int max = FieldInt(player, maxAmmo);
		return FieldInt(pack, ammo) > 0 && max > 0 && FieldInt(player, ammo) < max;
	}

	bool NeedsGrenade(const edict_t *player, const edict_t *pack, int count, int type)
	{
		return FieldInt(pack, count) > 0 && FieldInt(player, type) != 0 && FieldInt(player, count) < kGrenadeCap;
	}

	bool PackUseful(const edict_t *player, const edict_t *pack)
	{
		int state = FieldInt(pack, g_offsets.ent_goalState);
		if (state == kGoalStateActive || state == kGoalStateRemoved)
			return false;
		int team = FieldInt(pack, g_offsets.ent_teamNo);
		if (team != 0 && team != player->v.team)
			return false;
		if (pack->v.playerclass != 0 && pack->v.playerclass != player->v.playerclass)
			return false;
		if (FieldInt(pack, g_offsets.ent_itemsAllowed) != 0)
			return false;

		const GameOffsets &o = g_offsets;
		return (pack->v.health > 0.f && player->v.health < player->v.max_health) ||
			(pack->v.armorvalue > 0.f && player->v.armorvalue < FieldFloat(player, o.ent_maxarmor)) ||
			NeedsAmmo(player, pack, o.ent_ammoShells, o.ent_maxammoShells) ||
			NeedsAmmo(player, pack, o.ent_ammoNails, o.ent_maxammoNails) ||
			NeedsAmmo(player, pack, o.ent_ammoCells, o.ent_maxammoCells) ||
			NeedsAmmo(player, pack, o.ent_ammoRockets, o.ent_maxammoRockets) ||
			NeedsGrenade(player, pack, o.ent_noGrenades1, o.ent_tpGrenades1) ||
			NeedsGrenade(player, pack, o.ent_noGrenades2, o.ent_tpGrenades2) ||
			(FieldInt(pack, o.ent_ammoDetpack) > 0 && player->v.playerclass == kClassDemoman &&
				FieldInt(player, o.ent_ammoDetpack) < 1);
	}

	bool AmmoboxUseful(const edict_t *player, const edict_t *box)
	{
		if (box->v.enemy == player)
			return false;
		const GameOffsets &o = g_offsets;
		return NeedsAmmo(player, box, o.ent_ammoShells, o.ent_maxammoShells) ||
			NeedsAmmo(player, box, o.ent_ammoNails, o.ent_maxammoNails) ||
			NeedsAmmo(player, box, o.ent_ammoCells, o.ent_maxammoCells) ||
			NeedsAmmo(player, box, o.ent_ammoRockets, o.ent_maxammoRockets);
	}

	Vector Eye(const edict_t *ent)
	{
		return ent->v.origin + ent->v.view_ofs;
	}

	Vector Centre(const edict_t *ent)
	{
		return (ent->v.absmin + ent->v.absmax) * 0.5f;
	}

	float MaxHealth(const edict_t *ent)
	{
		return ent->v.max_health > 0.f ? ent->v.max_health : 100.f;
	}

	bool Related(edict_t *self, edict_t *ent, Teams::Relation rel)
	{
		return ent && ent != self && CBotIsLivePlayer(ent) && !g_pGameProfile->IsDead(ent) &&
			Teams::Of(self, ent) == rel;
	}

	bool Engaged(const CBot &bot)
	{
		return bot.sense.target.Get() ||
			(bot.sense.enemy.Get() && gpGlobals->time - bot.sense.enemyTime < kEngagedMemory);
	}

	bool Clear(edict_t *self, const Vector &from, const Vector &to, const edict_t *target)
	{
		TraceResult tr;
		TRACE_LINE(from, to, ignore_monsters, self, &tr);
		return tr.flFraction >= 1.f || (target && tr.pHit == target);
	}

	bool FriendNear(edict_t *self, const Vector &point, float radius)
	{
		for (int i = 1; i <= gpGlobals->maxClients; ++i)
		{
			edict_t *ent = g_engfuncs.pfnPEntityOfEntIndex(i);
			if (Related(self, ent, Teams::REL_FRIEND) && (ent->v.origin - point).Length() < radius)
				return true;
		}
		return false;
	}

	const BotWeapon *Carried(const CBot &bot, int id)
	{
		const BotInventory &inv = bot.inventory;
		for (int i = 0; i < inv.count; ++i)
		{
			if (inv.weapons[i].id == id)
				return &inv.weapons[i];
		}
		return nullptr;
	}

	bool Holding(const CBot &bot, int id)
	{
		const BotInventory &inv = bot.inventory;
		return inv.activeIndex >= 0 && inv.activeIndex < inv.count && inv.weapons[inv.activeIndex].id == id;
	}

	bool Select(CBot &bot, int id)
	{
		if (Holding(bot, id))
			return true;

		State &st = bot.ability;
		const WeaponSpec *spec = TFC_WeaponSpec(id);
		if (spec && gpGlobals->time >= st.nextSelect)
		{
			g_pGameProfile->SelectWeapon(bot.pEdict, *spec);
			st.nextSelect = gpGlobals->time + kSelectInterval;
		}
		return false;
	}

	void Command(CBot &bot, const char *cmd)
	{
		FakeCmd::Run(bot.pEdict, cmd);
	}

	bool End(State &st, const char *why)
	{
		st.task = BotAbility::TASK_NONE;
		st.phase = 0;
		st.target.Clear();
		st.why = why;
		return false;
	}

	void Face(CBot &bot, const Vector &point)
	{
		float yaw = 0.f, pitch = 0.f;
		if (BotAim::LosAngles(Eye(bot.pEdict), point, yaw, pitch))
			bot.desiredAngles = Vector(pitch, yaw, 0.f);
	}

	float Drive(CBot &bot, const Vector &point, float stop, unsigned short buttons, float gain = 0.f)
	{
		edict_t *pEdict = bot.pEdict;
		Vector angles = Bot_UpdateFacing(bot);

		Vector to(point.x - pEdict->v.origin.x, point.y - pEdict->v.origin.y, 0.f);
		float dist = to.Length2D();

		Vector moveDir(0.f, 0.f, 0.f);
		float drive = 0.f;
		if (dist > stop)
		{
			moveDir = to * (1.f / dist);
			drive = Bot_MaxSpeed(pEdict);
			if (gain > 0.f)
				drive = std::min(drive, dist * gain);

			NavFollow::Drive solved = NavFollow::SolveDrive(moveDir * drive, pEdict->v.velocity,
				Bot_DrivePhysics(pEdict));
			if (solved.wishSpeed > 0.f)
			{
				moveDir = solved.wishDir;
				drive = solved.wishSpeed;
			}
		}
		else if (gain > 0.f && pEdict->v.velocity.Length2D() > 1.f)
		{
			Vector vel(pEdict->v.velocity.x, pEdict->v.velocity.y, 0.f);
			drive = std::min(Bot_MaxSpeed(pEdict), vel.Length2D() * kBrakeGain);
			moveDir = vel * (-1.f / vel.Length2D());
		}

		if ((pEdict->v.flags & FL_DUCKING) != 0)
			drive /= PmSim::kDuckingMultiplier;

		float fmove = 0.f, smove = 0.f;
		Bot_DecomposeMove(angles.y + pEdict->v.punchangle.y, moveDir, drive, fmove, smove);
		bot.moveForward = fmove;
		bot.moveSide = smove;

		if (fmove > kMoveDeadzone) buttons |= IN_FORWARD;
		else if (fmove < -kMoveDeadzone) buttons |= IN_BACK;
		if (smove > kMoveDeadzone) buttons |= IN_MOVERIGHT;
		else if (smove < -kMoveDeadzone) buttons |= IN_MOVELEFT;

		bot.lastProgressTime = gpGlobals->time;
		Bot_RunPlayerMove(bot, angles, fmove, smove, buttons);
		return dist;
	}

	float Stand(CBot &bot, unsigned short buttons)
	{
		return Drive(bot, bot.pEdict->v.origin, 1e9f, buttons);
	}

	bool TurnedTo(const edict_t *pEdict, float yaw)
	{
		float diff = pEdict->v.v_angle.y - yaw;
		while (diff > 180.f) diff -= 360.f;
		while (diff < -180.f) diff += 360.f;
		return fabsf(diff) < kFaceTolerance;
	}

	bool AimedAt(const CBot &bot)
	{
		const Vector &want = bot.desiredAngles;
		const Vector &have = bot.pEdict->v.v_angle;
		float dy = have.y - want.y;
		while (dy > 180.f) dy -= 360.f;
		while (dy < -180.f) dy += 360.f;
		return fabsf(dy) < kAimTolerance && fabsf(have.x - want.x) < kAimTolerance;
	}

	template <typename Fn>
	void ForEachClass(const char *classname, Fn fn)
	{
		for (int i = gpGlobals->maxClients + 1; i < gpGlobals->maxEntities; ++i)
		{
			edict_t *ent = g_engfuncs.pfnPEntityOfEntIndex(i);
			if (!ent || ent->free || ent->v.classname == 0)
				continue;
			if (strcmp(CBotClassOf(ent), classname) == 0)
				fn(ent);
		}
	}

	bool NearOwnSpawn(const CBot &bot, float radius)
	{
		for (const Objectives::Spawn &spawn : Objectives::Get().spawns)
		{
			edict_t *ent = spawn.ent.Get();
			if (ent && spawn.team == bot.pEdict->v.team && (ent->v.origin - bot.pEdict->v.origin).Length() < radius)
				return true;
		}
		return false;
	}

	bool RoomYaw(const CBot &bot, float want, float &out)
	{
		const float offsets[] = { 0.f, 45.f, -45.f, 90.f, -90.f, 135.f, -135.f, 180.f };
		edict_t *pEdict = bot.pEdict;
		for (float offset : offsets)
		{
			Vector dir = Bot_YawDir(want + offset);
			TraceResult tr;
			g_engfuncs.pfnTraceHull(pEdict->v.origin, pEdict->v.origin + dir * kBuildRoom, ignore_monsters,
				kHullHuman, pEdict, &tr);
			if (tr.flFraction >= 1.f && !tr.fStartSolid)
			{
				out = want + offset;
				return true;
			}
		}
		return false;
	}

	edict_t *PadEntrance(int conn);

	bool StartPad(CBot &bot)
	{
		State &st = bot.ability;
		size_t from = bot.waypointIndex > 0 ? bot.waypointIndex - 1 : 0;
		for (size_t k = from; k <= bot.waypointIndex && k < bot.waypoints.size(); ++k)
		{
			const Nav::Waypoint &wp = bot.waypoints[k];
			if (!wp.requiresTeleport || !Nav::IsAddedTeleport(wp.connIndex))
				continue;
			if ((wp.point - bot.pEdict->v.origin).Length2D() > kPadEngage)
				continue;

			st.task = BotAbility::TASK_PAD;
			st.point = wp.point;
			st.param = wp.connIndex;
			st.start = gpGlobals->time;
			st.stillSince = 0.f;
			st.why = "on a pad";
			++st.pads;
			BOT_VLOG("CrossBot: %s steps onto teleporter pad (conn %d)\n", bot.name, wp.connIndex);
			return true;
		}
		return false;
	}

	bool RunPad(CBot &bot)
	{
		State &st = bot.ability;
		bool onRoute = false;
		for (const Nav::Waypoint &wp : bot.waypoints)
			onRoute = onRoute || (wp.requiresTeleport && wp.connIndex == st.param);
		if (!onRoute)
		{
			float away = (bot.pEdict->v.origin - st.point).Length();
			BOT_VLOG("CrossBot: %s leaves pad conn %d, now %.0f from it\n", bot.name, st.param, away);
			return End(st, away > kPadEngage ? "teleported" : "route changed");
		}

		if (!Nav::IsAddedTeleport(st.param))
		{
			Bot_ResetNavPath(bot);
			return End(st, "pad unlinked");
		}

		float now = gpGlobals->time;
		edict_t *entrance = PadEntrance(st.param);
		edict_t *self = bot.pEdict;
		bool still = entrance && self->v.groundentity == entrance && self->v.velocity.Length() < kPadStillSpeed;
		if (!still)
			st.stillSince = 0.f;
		else if (st.stillSince <= 0.f || st.stillSince > now)
			st.stillSince = now;

		bool refused = still && now - st.stillSince > kPadStillGrace &&
			FieldInt(entrance, g_offsets.tele_iState) != kTeleSending;
		if (refused || now - st.start > kPadTimeout)
		{
			BOT_VLOG("CrossBot: %s gives up on pad conn %d: %s\n", bot.name, st.param,
				refused ? "the entrance did not send" : "never got onto it");
			Nav::PenalizeConnection(static_cast<uint32_t>(st.param));
			Bot_ResetNavPath(bot);
			++st.padFails;
			return End(st, refused ? "not teleported" : "pad timed out");
		}

		bot.desiredAngles = bot.pEdict->v.v_angle;
		float dist = Drive(bot, st.point, kPadStop, 0, kPadApproachGain);
		if (gpGlobals->time >= st.phaseTime)
		{
			st.phaseTime = gpGlobals->time + 1.f;
			edict_t *ground = bot.pEdict->v.groundentity;
			BOT_VLOG("CrossBot: %s waits on pad conn %d: %.1f off, speed %.2f, on %s, z %.1f vs %.1f\n", bot.name,
				st.param, dist, bot.pEdict->v.velocity.Length(), ground ? CBotClassOf(ground) : "-",
				bot.pEdict->v.origin.z, st.point.z);
		}
		return true;
	}

	bool SameArea(const CBot &bot, const Nav::NavMesh &mesh, const edict_t *ent)
	{
		const Nav::Area *area = Nav::FindAreaNearest(mesh, ent->v.origin);
		return area && bot.currentAreaId >= 0 && static_cast<int>(area->id) == bot.currentAreaId;
	}

	bool StartHeal(CBot &bot, const Nav::NavMesh &mesh)
	{
		State &st = bot.ability;
		edict_t *self = bot.pEdict;
		edict_t *best = nullptr;
		float bestRatio = kHealRatio;

		for (int i = 1; i <= gpGlobals->maxClients; ++i)
		{
			edict_t *ent = g_engfuncs.pfnPEntityOfEntIndex(i);
			if (!Related(self, ent, Teams::REL_FRIEND))
				continue;
			float ratio = ent->v.health / MaxHealth(ent);
			if (ratio >= bestRatio || !SameArea(bot, mesh, ent))
				continue;
			best = ent;
			bestRatio = ratio;
		}

		if (!best || !Carried(bot, TFC_MEDIKIT))
			return false;

		st.task = BotAbility::TASK_HEAL;
		st.target.Set(best);
		st.start = gpGlobals->time;
		st.why = "healing";
		BOT_VLOG("CrossBot: %s goes to heal %s (%.0f/%.0f)\n", bot.name, CBotPlayerNameOf(best),
			best->v.health, MaxHealth(best));
		return true;
	}

	bool RunHeal(CBot &bot, const Nav::NavMesh &mesh)
	{
		State &st = bot.ability;
		edict_t *self = bot.pEdict;
		edict_t *t = st.target.Get();

		if (!Related(self, t, Teams::REL_FRIEND))
			return End(st, "patient gone");
		if (t->v.health >= MaxHealth(t))
		{
			++st.heals;
			return End(st, "healed");
		}
		if (Engaged(bot))
			return End(st, "enemy");
		if (gpGlobals->time - st.start > kHealTimeout)
			return End(st, "heal timed out");

		float dist = (t->v.origin - self->v.origin).Length();
		if (dist >= kMeleeReach && !SameArea(bot, mesh, t))
			return End(st, "patient left the area");

		bool ready = Select(bot, TFC_MEDIKIT);
		Face(bot, Centre(t));
		Drive(bot, t->v.origin, kHealStop, (ready && dist < kMeleeReach) ? IN_ATTACK : 0);
		return true;
	}

	bool StartDiscard(CBot &bot, const Nav::NavMesh &mesh)
	{
		State &st = bot.ability;
		edict_t *self = bot.pEdict;
		float now = gpGlobals->time;
		if (now < st.nextDiscard || self->v.deadflag == kDeadFeign || Engaged(bot))
			return false;

		int game = XPlay::OwnerOf(self);
		edict_t *best = nullptr;
		float bestDist = 0.f;
		for (int i = 1; i <= gpGlobals->maxClients; ++i)
		{
			edict_t *ent = g_engfuncs.pfnPEntityOfEntIndex(i);
			if (!Related(self, ent, Teams::REL_FRIEND) || XPlay::OwnerOf(ent) != game)
				continue;
			if (ent->v.playerclass == 0 || ent->v.playerclass == self->v.playerclass || !SameArea(bot, mesh, ent))
				continue;
			float dist = (ent->v.origin - self->v.origin).Length();
			if (best && dist >= bestDist)
				continue;
			best = ent;
			bestDist = dist;
		}

		if (!best || CarryingGoal(self))
			return false;

		st.task = BotAbility::TASK_DISCARD;
		st.target.Set(best);
		st.start = now;
		st.nextDiscard = now + kDiscardRetry;
		st.why = "sharing ammo";
		BOT_VLOG("CrossBot: %s turns to share ammo with %s\n", bot.name, CBotPlayerNameOf(best));
		return true;
	}

	bool RunDiscard(CBot &bot, const Nav::NavMesh &mesh)
	{
		State &st = bot.ability;
		edict_t *t = st.target.Get();

		if (!Related(bot.pEdict, t, Teams::REL_FRIEND) || !SameArea(bot, mesh, t))
			return End(st, "teammate left");
		if (gpGlobals->time - st.start > kDiscardTimeout)
			return End(st, "discard timed out");

		Face(bot, Centre(t));
		if (AimedAt(bot))
		{
			Command(bot, "discard");
			++st.discards;
			BOT_VLOG("CrossBot: %s discards ammo at %s\n", bot.name, CBotPlayerNameOf(t));
			return End(st, "shared ammo");
		}
		Stand(bot, 0);
		return true;
	}

	bool NeedsWork(const edict_t *building, int cells)
	{
		if (cells <= 0)
			return false;
		if (building->v.max_health > 0.f && building->v.health < building->v.max_health)
			return true;
		const char *model = STRING(building->v.model);
		return strcmp(CBotClassOf(building), "building_sentrygun") == 0 && cells >= kCellsUpgrade &&
			model && !strstr(model, "sentry3");
	}

	bool StartRepair(CBot &bot)
	{
		State &st = bot.ability;
		edict_t *self = bot.pEdict;
		int cells = FieldInt(self, g_offsets.ent_ammoCells);
		edict_t *pick = nullptr;

		bool teams = Teams::Active(self);
		auto consider = [&](edict_t *ent) {
			if (pick || FieldInt(ent, g_offsets.ent_teamNo) != self->v.team || !NeedsWork(ent, cells))
				return;
			if (!teams && RealOwner(ent) != self)
				return;
			if ((ent->v.origin - self->v.origin).Length() > kRepairRange)
				return;
			if (Clear(self, Eye(self), Centre(ent), ent))
				pick = ent;
		};
		ForEachClass("building_sentrygun", consider);
		ForEachClass("building_dispenser", consider);
		ForEachClass("building_teleporter", consider);

		if (!pick || !Carried(bot, TFC_SPANNER))
			return false;

		st.task = BotAbility::TASK_REPAIR;
		st.target.Set(pick);
		st.start = gpGlobals->time;
		st.why = "repairing";
		edict_t *owner = RealOwner(pick);
		BOT_VLOG("CrossBot: %s goes to work on %s's %s (%.0f/%.0f, %d cells)\n", bot.name,
			owner ? CBotPlayerNameOf(owner) : "nobody", CBotClassOf(pick), pick->v.health, pick->v.max_health, cells);
		return true;
	}

	bool RunRepair(CBot &bot)
	{
		State &st = bot.ability;
		edict_t *self = bot.pEdict;
		edict_t *t = st.target.Get();

		if (!t)
			return End(st, "building gone");
		if (!NeedsWork(t, FieldInt(self, g_offsets.ent_ammoCells)))
		{
			++st.repairs;
			return End(st, "building done");
		}
		if (Engaged(bot))
			return End(st, "enemy");
		if (gpGlobals->time - st.start > kRepairTimeout)
			return End(st, "repair timed out");

		bool ready = Select(bot, TFC_SPANNER);
		Face(bot, Centre(t));
		float dist = (t->v.origin - self->v.origin).Length2D();
		Drive(bot, t->v.origin, kRepairStop, (ready && dist < kMeleeReach) ? IN_ATTACK : 0);
		return true;
	}

	bool BuildFail(CBot &bot, const char *why)
	{
		State &st = bot.ability;
		++st.buildFails;
		st.nextTry = gpGlobals->time + kBuildRetry;
		BOT_VLOG("CrossBot: %s fails to build %d: %s, phase %d, %s, speed %.0f\n", bot.name, st.param, why,
			st.phase, (bot.pEdict->v.flags & FL_ONGROUND) ? "grounded" : "airborne", bot.pEdict->v.velocity.Length2D());
		return End(st, why);
	}

	bool StartBuild(CBot &bot)
	{
		State &st = bot.ability;
		edict_t *self = bot.pEdict;
		if (FieldInt(self, g_offsets.ent_isBuilding) != 0 || (self->v.flags & FL_ONGROUND) == 0)
			return false;

		int cells = FieldInt(self, g_offsets.ent_ammoCells);
		bool sentry = FieldInt(self, g_offsets.ent_hasSentry) != 0;
		bool dispenser = FieldInt(self, g_offsets.ent_hasDispenser) != 0;
		bool entrance = FieldInt(self, g_offsets.ent_hasEntryTele) != 0;
		bool exit = FieldInt(self, g_offsets.ent_hasExitTele) != 0;

		int kind = 0;
		float yaw = self->v.v_angle.y;
		if (bot.camp.holding)
		{
			yaw = bot.camp.spot.yaw;
			bool sentryHere = !BotStrategic::HasFortifyTarget(bot) ||
				BotStrategic::TargetKind(bot) == BotStrategic::KIND_DEFEND;
			if (!sentry)
			{
				if (sentryHere && cells >= kCellsSentry)
					kind = kBuildSentry;
			}
			else if (!dispenser)
			{
				if (cells >= kCellsDispenser)
				{
					kind = kBuildDispenser;
					yaw += 90.f;
				}
			}
			else if (entrance && !exit && cells >= kCellsTeleporter)
			{
				kind = kBuildExit;
				yaw += 180.f;
			}
		}
		else if (sentry && dispenser && !entrance && cells >= kCellsTeleporter && NearOwnSpawn(bot, kSpawnRadius))
		{
			kind = kBuildEntrance;
		}

		if (!kind)
		{
			if (bot.camp.holding && st.buildLogHold != bot.camp.holdStart)
			{
				st.buildLogHold = bot.camp.holdStart;
				BOT_VLOG("CrossBot: %s builds nothing at its camp: %d cells, sentry %d dispenser %d, kind %d\n",
					bot.name, cells, sentry, dispenser, BotStrategic::TargetKind(bot));
			}
			return false;
		}

		if (kind == kBuildSentry)
			st.sentryYaw = yaw;
		st.param = kind;
		if (!RoomYaw(bot, yaw, st.yaw))
		{
			BuildFail(bot, "no room to build");
			return false;
		}

		st.task = BotAbility::TASK_BUILD;
		st.param = kind;
		st.phase = 0;
		st.start = gpGlobals->time;
		st.why = "building";
		BOT_VLOG("CrossBot: %s builds %d facing %.0f (%d cells)\n", bot.name, kind, st.yaw, cells);
		return true;
	}

	bool RunBuild(CBot &bot)
	{
		State &st = bot.ability;
		edict_t *self = bot.pEdict;
		float now = gpGlobals->time;
		bool building = FieldInt(self, g_offsets.ent_isBuilding) != 0;

		if (st.phase == 0 && Engaged(bot))
			return End(st, "enemy");
		if (now - st.start > kBuildTimeout)
		{
			return BuildFail(bot, "build timed out");
		}

		bot.desiredAngles = Vector(0.f, st.yaw, 0.f);

		if (st.phase == 0)
		{
			bool settled = (self->v.flags & FL_ONGROUND) != 0 && TurnedTo(self, st.yaw) &&
				self->v.velocity.Length2D() < kSettleSpeed && now - st.start > kBuildSettle;
			if (settled)
			{
				char cmd[16];
				snprintf(cmd, sizeof(cmd), "build %d", st.param);
				Command(bot, cmd);
				st.phase = 1;
				st.phaseTime = now;
			}
		}
		else if (st.phase == 1)
		{
			if (building)
				st.phase = 2;
			else if (now - st.phaseTime > kBuildConfirm)
			{
				return BuildFail(bot, "build refused");
			}
		}
		else if (st.phase == 2)
		{
			if (!building)
			{
				++st.builds;
				BOT_VLOG("CrossBot: %s finished building %d\n", bot.name, st.param);
				if (st.param != kBuildSentry)
					return End(st, "built");

				int offset = static_cast<int>(lroundf(st.yaw - st.sentryYaw));
				int turn = ((180 - offset) % 360 + 360) % 360;
				st.turns = (turn + kSentryTurnStep / 2) / kSentryTurnStep;
				st.phase = 3;
				st.phaseTime = now + kSentryTurnDelay;
			}
		}
		else
		{
			if (st.turns <= 0 || FieldInt(self, g_offsets.ent_hasSentry) == 0)
				return End(st, "built");
			if (now >= st.phaseTime)
			{
				bool half = st.turns >= 4;
				BOT_VLOG("CrossBot: %s turns its sentry %d degrees, %d steps left\n", bot.name, half ? 180 : 45,
					st.turns - (half ? 4 : 1));
				Command(bot, half ? "rotatesentry180" : "rotatesentry");
				st.turns -= half ? 4 : 1;
				st.phaseTime = now + kSentryTurnGap;
			}
		}

		Stand(bot, 0);
		return true;
	}

	bool StartStab(CBot &bot)
	{
		State &st = bot.ability;
		edict_t *self = bot.pEdict;
		edict_t *t = bot.sense.target.Get();
		if (!BotDiff::Backstab() || !Related(self, t, Teams::REL_ENEMY) || !Carried(bot, TFC_KNIFE))
			return false;

		Vector away = self->v.origin - t->v.origin;
		float dist = away.Length();
		if (dist > kStabRange || dist < 1.f)
			return false;

		MAKE_VECTORS(Vector(0.f, t->v.v_angle.y, 0.f));
		if (DotProduct2D(gpGlobals->v_forward, away * (1.f / dist)) > kStabBehind)
			return false;

		st.task = BotAbility::TASK_STAB;
		st.target.Set(t);
		st.start = gpGlobals->time;
		st.why = "stabbing";
		return true;
	}

	bool RunStab(CBot &bot)
	{
		State &st = bot.ability;
		edict_t *self = bot.pEdict;
		edict_t *t = st.target.Get();

		if (!t || g_pGameProfile->IsDead(t))
		{
			if (t)
				++st.stabs;
			return End(st, "victim down");
		}
		if (gpGlobals->time - st.start > kStabTimeout)
			return End(st, "stab timed out");

		Vector away = self->v.origin - t->v.origin;
		float dist = away.Length();
		if (dist > kStabLost)
			return End(st, "victim lost");

		MAKE_VECTORS(Vector(0.f, t->v.v_angle.y, 0.f));
		if (dist > 1.f && DotProduct2D(gpGlobals->v_forward, away * (1.f / dist)) > kStabSpotted)
			return End(st, "spotted");

		bool ready = Select(bot, TFC_KNIFE);
		Face(bot, Centre(t));
		Drive(bot, t->v.origin, kStabStop, (ready && dist < kStabReach) ? IN_ATTACK : 0);
		return true;
	}

	void SpyPassive(CBot &bot)
	{
		State &st = bot.ability;
		edict_t *self = bot.pEdict;
		float now = gpGlobals->time;

		if (Teams::Active(self) && FieldInt(self, g_offsets.ent_isUndercover) == 0 && now >= st.nextDisguise &&
			st.task != BotAbility::TASK_STAB && !Engaged(bot) && !CarryingGoal(self))
		{
			char cmd[32];
			snprintf(cmd, sizeof(cmd), "disguise_enemy %d",
				kDisguises[RANDOM_LONG(0, static_cast<int>(ARRAYSIZE(kDisguises)) - 1)]);
			Command(bot, cmd);
			st.nextDisguise = now + kDisguiseRetry;
			++st.disguises;
		}

		bool hurt = st.lastHealth > 0.f && self->v.health < st.lastHealth && self->v.health > 0.f;
		if (hurt)
			st.hurtTime = now;
		if (hurt && (self->v.flags & FL_ONGROUND) != 0 && st.task != BotAbility::TASK_PAD && bot.grenade.slot < 0)
		{
			float chance = 1.f - self->v.health / MaxHealth(self);
			if (RANDOM_FLOAT(0.f, 1.f) < chance)
			{
				Command(bot, "feign");
				++st.feigns;
				End(st, "feigning");
				BOT_VLOG("CrossBot: %s feigns death at %.0f health\n", bot.name, self->v.health);
			}
		}
	}

	template <typename Fn>
	void ForEachPipe(const edict_t *self, Fn fn)
	{
		ForEachClass("tf_gl_pipebomb", [&](edict_t *ent) {
			if (ent->v.owner == self)
				fn(ent);
		});
	}

	int CountPipes(const edict_t *self)
	{
		int count = 0;
		ForEachPipe(self, [&](edict_t *) { ++count; });
		return count;
	}

	void DemoDetonate(CBot &bot)
	{
		State &st = bot.ability;
		edict_t *self = bot.pEdict;
		float now = gpGlobals->time;
		if (now < st.nextDetonate)
			return;
		st.nextDetonate = now + kDetonateInterval;

		bool fire = false;
		ForEachPipe(self, [&](edict_t *pipe) {
			if (fire || (pipe->v.origin - self->v.origin).Length() < kPipeSelfClear)
				return;
			fire = BotSense::NearestLive(bot, pipe->v.origin, kPipeTrigger, true) != nullptr &&
				!FriendNear(self, pipe->v.origin, kPipeTrigger);
		});

		if (!fire)
			return;
		Command(bot, "detpipe");
		++st.detonations;
		st.nextDetonate = now + kDetonateGap;
		BOT_VLOG("CrossBot: %s detonates its pipebombs\n", bot.name);
	}

	bool StartPipes(CBot &bot)
	{
		State &st = bot.ability;
		edict_t *self = bot.pEdict;
		if (!bot.camp.scared || !Carried(bot, TFC_PL) || CountPipes(self) >= kMaxPipes)
			return false;

		const BotSense::Suspicion *contact = BotSense::NearestLive(bot, self->v.origin,
			bot.camp.scareDist + kPipeTrigger, true);
		if (!contact)
			return false;

		Vector to = contact->pos - self->v.origin;
		float len = to.Length();
		if (len < 1.f)
			return false;
		Vector want = self->v.origin + to * (std::min(len * kPipeShare, kPipeReachMax) / len);

		TraceResult tr;
		TRACE_LINE(Eye(self), want, ignore_monsters, self, &tr);
		if (FriendNear(self, tr.vecEndPos, kPipeFriendClear))
			return false;

		st.task = BotAbility::TASK_PIPES;
		st.point = tr.vecEndPos;
		st.start = gpGlobals->time;
		st.why = "laying pipes";
		++st.pipeBursts;
		BOT_VLOG("CrossBot: %s lays pipes toward (%.0f,%.0f,%.0f)\n", bot.name, st.point.x, st.point.y,
			st.point.z);
		return true;
	}

	bool RunPipes(CBot &bot)
	{
		State &st = bot.ability;
		edict_t *self = bot.pEdict;
		float now = gpGlobals->time;

		if (!bot.camp.scared && now - st.start > kPipeCalm)
			return End(st, "calm");
		if (now - st.start > kPipeTimeout)
			return End(st, "pipes timed out");
		if (CountPipes(self) >= kMaxPipes)
			return End(st, "pipes laid");

		const WeaponSpec *spec = TFC_WeaponSpec(TFC_PL);
		const BotWeapon *pl = Carried(bot, TFC_PL);
		if (!spec || !pl)
			return End(st, "no launcher");
		if (pl->clip == 0 && bot.inventory.ammo[std::max(0, pl->primaryAmmoType)] <= 0)
			return End(st, "no pipes");

		Vector look;
		float flight = 0.f;
		if (!BotFire::SolveOffhand(spec->primary, Eye(self), st.point, Vector(0.f, 0.f, 0.f), look, flight))
			return End(st, "no arc");
		Face(bot, Eye(self) + look * kLookReach);

		bool ready = Select(bot, TFC_PL);
		unsigned short buttons = 0;
		if (ready && pl->clip == 0)
			buttons = IN_RELOAD;
		else if (ready && AimedAt(bot) && pl->nextPrimaryAttack <= 0.f && !pl->reloading)
			buttons = IN_ATTACK;
		Stand(bot, buttons);
		return true;
	}

	bool LooksFriendly(edict_t *self, edict_t *ent)
	{
		if (!ent || ent == self || !CBotIsLivePlayer(ent) || g_pGameProfile->IsDead(ent))
			return false;
		Teams::Relation rel = Teams::Of(self, ent);
		return rel == Teams::REL_FRIEND || (rel == Teams::REL_ENEMY && IsDisguised(ent));
	}

	bool StartTouch(CBot &bot)
	{
		State &st = bot.ability;
		edict_t *self = bot.pEdict;
		float now = gpGlobals->time;

		MAKE_VECTORS(self->v.v_angle);
		Vector forward = gpGlobals->v_forward;

		edict_t *best = nullptr;
		float bestDist = kTouchRange;
		for (int i = 1; i <= gpGlobals->maxClients && i < BotAbility::kTouchSlots; ++i)
		{
			edict_t *ent = g_engfuncs.pfnPEntityOfEntIndex(i);
			if (!LooksFriendly(self, ent) || TimeLeft(st.touchNext[i], now, kTouchCooldown) > 0.f)
				continue;

			Vector to = ent->v.origin - self->v.origin;
			float dist = to.Length();
			if (dist >= bestDist || dist < 1.f || DotProduct(forward, to * (1.f / dist)) < kTouchFov)
				continue;

			float friendDist = kTouchFriendFar;
			for (int j = 1; j <= gpGlobals->maxClients; ++j)
			{
				edict_t *other = g_engfuncs.pfnPEntityOfEntIndex(j);
				if (other != ent && LooksFriendly(self, other))
					friendDist = std::min(friendDist, (other->v.origin - ent->v.origin).Length());
			}
			if (!BotSense::NearestLive(bot, ent->v.origin, friendDist, true))
				continue;

			best = ent;
			bestDist = dist;
		}

		if (!best)
			return false;

		st.touchNext[g_engfuncs.pfnIndexOfEdict(best)] = now + kTouchCooldown;
		st.task = BotAbility::TASK_TOUCH;
		st.target.Set(best);
		st.start = now;
		st.why = "touching a suspect";
		BOT_VLOG("CrossBot: %s suspects %s and runs to touch it\n", bot.name, CBotPlayerNameOf(best));
		return true;
	}

	bool RunTouch(CBot &bot)
	{
		State &st = bot.ability;
		edict_t *self = bot.pEdict;
		edict_t *t = st.target.Get();

		if (!t || !CBotIsLivePlayer(t) || g_pGameProfile->IsDead(t))
			return End(st, "suspect gone");

		bool enemy = Teams::Of(self, t) == Teams::REL_ENEMY;
		if (enemy && !IsDisguised(t))
		{
			++st.uncovers;
			BOT_VLOG("CrossBot: %s uncovered %s\n", bot.name, CBotPlayerNameOf(t));
			return End(st, "uncovered");
		}
		if (gpGlobals->time - st.start > kTouchTimeout)
			return End(st, "touch timed out");
		if (!enemy && BoxGap2D(self, t) <= kTouchGap)
		{
			++st.touches;
			return End(st, "touched a friend");
		}

		Face(bot, Centre(t));
		Drive(bot, t->v.origin, 0.f, 0);
		return true;
	}

	bool StartDisarm(CBot &bot)
	{
		State &st = bot.ability;
		edict_t *self = bot.pEdict;
		edict_t *best = nullptr;
		float bestDist = kDisarmRange;

		ForEachClass("detpack", [&](edict_t *pack) {
			edict_t *owner = pack->v.owner;
			if (!owner || Teams::Of(self, owner) != Teams::REL_ENEMY)
				return;
			float dist = (pack->v.origin - self->v.origin).Length();
			if (dist < bestDist && Clear(self, Eye(self), Centre(pack), pack))
			{
				best = pack;
				bestDist = dist;
			}
		});

		if (!best)
			return false;

		st.task = BotAbility::TASK_DISARM;
		st.target.Set(best);
		st.point = best->v.origin;
		st.start = gpGlobals->time;
		st.why = "disarming";
		BOT_VLOG("CrossBot: %s runs to disarm a detpack %.0f away\n", bot.name, bestDist);
		return true;
	}

	bool RunDisarm(CBot &bot)
	{
		State &st = bot.ability;
		edict_t *self = bot.pEdict;
		edict_t *pack = st.target.Get();

		if (!pack)
		{
			bool reached = (self->v.origin - st.point).Length() <= kDisarmNear;
			if (reached)
				++st.disarms;
			BOT_VLOG("CrossBot: %s sees the detpack gone, %s\n", bot.name, reached ? "disarmed" : "from afar");
			return End(st, reached ? "disarmed" : "detpack gone");
		}
		if (gpGlobals->time - st.start > kDisarmTimeout)
			return End(st, "disarm timed out");

		st.point = pack->v.origin;
		if (!BotSense::ApplyLook(bot) || (bot.sense.lookSource != BotSense::LOOK_AIM &&
			bot.sense.lookSource != BotSense::LOOK_PREDICT))
			Face(bot, Centre(pack));
		Drive(bot, pack->v.origin, 0.f, 0);
		return true;
	}

	int DetpackFuse(const CBot &bot, bool objective)
	{
		bool hard = BotDiff::Current() >= BotDiff::DIFF_HARD;
		if (!hard)
			return kDetpackFuseMax;
		if (objective)
			return bot.camp.scared ? kDetpackFuseMin : kDetpackFuseObjective;

		float speed = std::max(1.f, Bot_MaxSpeed(bot.pEdict));
		float eta = bot.camp.scareDist / speed;
		if (eta < kDetpackArm)
			return 0;
		return std::max(kDetpackFuseMin, std::min(kDetpackFuseMax, static_cast<int>(lroundf(eta - kDetpackArm))));
	}

	bool StartDetpack(CBot &bot)
	{
		State &st = bot.ability;
		edict_t *self = bot.pEdict;
		if (!DetpackReady(self) || (self->v.flags & FL_ONGROUND) == 0)
			return false;

		const Objectives::Site *site = BotStrategic::DetpackGoal(bot);
		edict_t *goal = site ? site->ent.Get() : nullptr;
		if (goal)
		{
			float dist = (site->point - self->v.origin).Length();
			bool spot = (bot.goalPoint - self->v.origin).Length() < kDetpackArrive;
			if (dist > kDetpackReach || !(spot || Clear(self, Eye(self), Centre(goal), goal)))
				return false;
		}
		else if (Objectives::Get().detpackMap || !bot.camp.scared ||
			FriendNear(self, self->v.origin, kDetpackScaredClear))
			return false;

		int fuse = DetpackFuse(bot, goal != nullptr);
		if (fuse <= 0 || FriendNear(self, self->v.origin, kDetpackFriendClear))
			return false;

		bool stacked = false;
		ForEachClass("detpack", [&](edict_t *pack) {
			stacked = stacked || (pack->v.origin - self->v.origin).Length() < kDetpackStack;
		});
		if (stacked)
			return false;

		st.task = BotAbility::TASK_DETPACK;
		st.target.Set(goal);
		st.param = fuse;
		st.phase = 0;
		st.start = gpGlobals->time;
		st.point = self->v.origin;
		st.route.clear();
		st.routeIndex = 0;
		st.why = goal ? "detpacking the objective" : "detpacking the approach";
		BOT_VLOG("CrossBot: %s sets a %d s detpack %s\n", bot.name, fuse,
			goal ? "at the objective" : "for the enemy coming in");
		return true;
	}

	void PlanRetreat(CBot &bot, const Nav::NavMesh &mesh)
	{
		State &st = bot.ability;
		edict_t *self = bot.pEdict;
		st.route.clear();
		st.routeIndex = 0;
		if (bot.currentAreaId < 0)
			return;

		std::vector<float> costs;
		Nav::AreaCosts(mesh, static_cast<uint32_t>(bot.currentAreaId), costs, Bot_GravityMultiplier(self),
			Bot_Carefulness(bot), Bot_MaxSpeed(self), Bot_RangeBias(bot), Bot_FallDamage(bot));

		const Nav::Area *best = nullptr;
		for (const Nav::Area &area : mesh.areas)
		{
			if (area.id >= costs.size() || costs[area.id] >= Nav::kUnreachableSeconds)
				continue;
			if ((area.centroid - st.point).Length() < kDetpackSafeDist)
				continue;
			if (!best || costs[area.id] < costs[best->id])
				best = &area;
		}
		if (!best)
			return;

		Nav::PlannedPath path = Nav::PlanPath(mesh, static_cast<uint32_t>(bot.currentAreaId), best->id,
			self->v.origin, best->centroid);
		for (const Nav::Waypoint &wp : path.waypoints)
			st.route.push_back(wp.point);
		BOT_VLOG("CrossBot: %s retreats %.1fs to area %u, %zu waypoints\n", bot.name, costs[best->id], best->id,
			st.route.size());
	}

	bool RunDetpack(CBot &bot, const Nav::NavMesh &mesh)
	{
		State &st = bot.ability;
		edict_t *self = bot.pEdict;
		float now = gpGlobals->time;
		bool detpacking = FieldInt(self, g_offsets.ent_isDetpacking) != 0;

		if (st.phase == 0)
		{
			if (self->v.velocity.Length2D() < kSettleSpeed && (self->v.flags & FL_ONGROUND) != 0)
			{
				char cmd[32];
				snprintf(cmd, sizeof(cmd), "detstart %d", st.param);
				Command(bot, cmd);
				st.phase = 1;
				st.phaseTime = now;
			}
		}
		else if (st.phase == 1)
		{
			if (detpacking)
			{
				st.phase = 2;
				st.phaseTime = now;
			}
			else if (now - st.phaseTime > kDetpackConfirm)
			{
				st.nextTry = now + kDetpackRetry;
				return End(st, "detpack refused");
			}
		}
		else if (st.phase == 2)
		{
			if (!detpacking || now - st.phaseTime > kDetpackArm + kDetpackArmSlack)
			{
				++st.detpacks;
				st.fuseEnd = now + static_cast<float>(st.param);
				st.phase = 3;
				if (st.target.Get())
					BotStrategic::LeaveGoal(bot, static_cast<float>(st.param) + kDetpackAfterBlast);
				Bot_ResetNavPath(bot);
				PlanRetreat(bot, mesh);
			}
		}
		else
		{
			if (now > st.fuseEnd + kDetpackArmSlack || st.fuseEnd - now > kDetpackFuseMax + kDetpackArmSlack)
			{
				Bot_ResetNavPath(bot);
				return End(st, "detpack went off");
			}

			while (st.routeIndex < st.route.size() &&
				(st.route[st.routeIndex] - self->v.origin).Length2D() < kRetreatStop)
				++st.routeIndex;

			if (st.routeIndex < st.route.size())
			{
				const Vector &next = st.route[st.routeIndex];
				if (!BotSense::ApplyLook(bot))
					Face(bot, next + self->v.view_ofs);
				Drive(bot, next, 0.f, 0);
				return true;
			}
			BotSense::ApplyLook(bot);
		}

		Stand(bot, 0);
		return true;
	}

	bool Continue(CBot &bot, const Nav::NavMesh &mesh)
	{
		switch (bot.ability.task)
		{
		case BotAbility::TASK_PAD:     return RunPad(bot);
		case BotAbility::TASK_HEAL:    return RunHeal(bot, mesh);
		case BotAbility::TASK_REPAIR:  return RunRepair(bot);
		case BotAbility::TASK_BUILD:   return RunBuild(bot);
		case BotAbility::TASK_STAB:    return RunStab(bot);
		case BotAbility::TASK_PIPES:   return RunPipes(bot);
		case BotAbility::TASK_TOUCH:   return RunTouch(bot);
		case BotAbility::TASK_DISARM:  return RunDisarm(bot);
		case BotAbility::TASK_DETPACK: return RunDetpack(bot, mesh);
		case BotAbility::TASK_DISCARD: return RunDiscard(bot, mesh);
		default:                       return false;
		}
	}

	struct Link
	{
		CBotEntHandle entrance;
		CBotEntHandle exit;
		int conn = -1;
	};

	std::vector<Link> g_links;
	float g_lastRefresh = 0.f;

	edict_t *PadEntrance(int conn)
	{
		for (const Link &link : g_links)
		{
			if (link.conn == conn)
				return link.entrance.Get();
		}
		return nullptr;
	}

	bool EnemyWatching(edict_t *self)
	{
		for (int i = 1; i <= gpGlobals->maxClients; ++i)
		{
			edict_t *ent = g_engfuncs.pfnPEntityOfEntIndex(i);
			if (!Related(self, ent, Teams::REL_ENEMY))
				continue;
			Vector to = self->v.origin - ent->v.origin;
			float dist = to.Length();
			if (dist > kFeignWatchRange || dist < 1.f)
				continue;
			MAKE_VECTORS(ent->v.v_angle);
			if (DotProduct(gpGlobals->v_forward, to * (1.f / dist)) < kFeignWatchDot)
				continue;
			if (Clear(self, Eye(ent), Eye(self), self))
				return true;
		}
		return false;
	}
}

namespace TFCAbility
{

bool Run(CBot &bot, const Nav::NavMesh &mesh)
{
	State &st = bot.ability;
	edict_t *self = bot.pEdict;
	int cls = self->v.playerclass;
	float now = gpGlobals->time;

	if (cls == kClassSpy)
		SpyPassive(bot);
	if (cls == kClassDemoman)
		DemoDetonate(bot);
	st.lastHealth = self->v.health;

	bool engaged = Engaged(bot);
	if (st.engaged && !engaged && cls != kClassMedic && Teams::Active(self) && self->v.deadflag != kDeadFeign &&
		self->v.health < MaxHealth(self) * kCallRatio)
	{
		Command(bot, "saveme");
		++st.calls;
		BOT_VLOG("CrossBot: %s calls for a medic at %.0f health\n", bot.name, self->v.health);
	}
	st.engaged = engaged;

	if (st.task == BotAbility::TASK_NONE)
		StartPad(bot);

	if (TimeLeft(st.nextTry, now, kDetpackRetry) <= 0.f)
		st.nextTry = now;

	if (st.task == BotAbility::TASK_NONE && now >= st.nextTry)
	{
		st.nextTry = now + kDecideInterval;
		switch (cls)
		{
		case kClassMedic:
			if (!engaged)
				StartHeal(bot, mesh);
			break;
		case kClassEngineer:
			if (!engaged && !StartBuild(bot))
				StartRepair(bot);
			break;
		case kClassSpy:
			StartStab(bot);
			break;
		case kClassDemoman:
			if (!StartDetpack(bot))
				StartPipes(bot);
			break;
		case kClassScout:
			if (!StartDisarm(bot) && !engaged)
				StartTouch(bot);
			break;
		default:
			break;
		}
		if (st.task == BotAbility::TASK_NONE)
			StartDiscard(bot, mesh);
	}

	return Continue(bot, mesh);
}

bool Disguised(edict_t *ent)
{
	return IsDisguised(ent);
}

bool HoldFire(const CBot &bot)
{
	const edict_t *self = bot.pEdict;
	return self->v.playerclass == kClassSpy && FieldInt(self, g_offsets.ent_isUndercover) == 1 &&
		TimeLeft(bot.ability.hurtTime + kHurtHold, gpGlobals->time, kHurtHold) <= 0.f;
}

int ClassifyItem(edict_t *ent)
{
	const char *classname = CBotClassOf(ent);
	if (strcmp(classname, "building_dispenser") == 0)
		return ITEM_STATION;
	if (strcmp(classname, "item_tfgoal") == 0)
		return ITEM_NONE;
	if (IsPackClass(classname))
		return IsPack(ent) ? ITEM_PICKUP : ITEM_NONE;
	if (strcmp(classname, "tf_ammobox") == 0)
		return ((ent->v.flags & FL_ONGROUND) != 0 &&
			gpGlobals->time >= FieldFloat(ent, g_offsets.box_flTouchableAt)) ? ITEM_PICKUP : ITEM_NONE;
	return -1;
}

bool ItemUsable(edict_t *player, edict_t *item)
{
	const char *classname = CBotClassOf(item);
	if (strcmp(classname, "building_dispenser") == 0)
		return FieldInt(item, g_offsets.ent_teamNo) == player->v.team;
	if (IsPackClass(classname))
		return PackUseful(player, item);
	if (strcmp(classname, "tf_ammobox") == 0)
		return AmmoboxUseful(player, item);
	return true;
}

float StationValue(edict_t *player)
{
	const GameOffsets &o = g_offsets;
	const int pairs[4][2] = {
		{ o.ent_ammoShells, o.ent_maxammoShells },
		{ o.ent_ammoNails, o.ent_maxammoNails },
		{ o.ent_ammoCells, o.ent_maxammoCells },
		{ o.ent_ammoRockets, o.ent_maxammoRockets },
	};

	float have = 0.f;
	float total = 0.f;
	for (const auto &pair : pairs)
	{
		float max = static_cast<float>(FieldInt(player, pair[1]));
		if (max <= 0.f)
			continue;
		have += std::min(max, static_cast<float>(FieldInt(player, pair[0])));
		total += max;
	}

	float maxArmor = FieldFloat(player, o.ent_maxarmor);
	if (maxArmor > 0.f)
	{
		have += std::min(maxArmor, player->v.armorvalue);
		total += maxArmor;
	}
	return total > 0.f ? kStationFull * have / total : kStationFull;
}

bool Fortifies(edict_t *ent)
{
	return ent->v.playerclass == kClassEngineer && FieldInt(ent, g_offsets.ent_hasSentry) == 0;
}

bool CanDetpack(edict_t *ent)
{
	return DetpackReady(ent);
}

void NoteSound(edict_t *src, const char *sample)
{
	int slot = g_engfuncs.pfnIndexOfEdict(src);
	if (slot >= 1 && slot <= kMaxPlayers && sample && strncmp(sample, "speech/saveme", 13) == 0)
		g_saveMe[slot] = gpGlobals->time;
}

void Patients(edict_t *medic, std::vector<edict_t *> &out)
{
	if (medic->v.playerclass != kClassMedic)
		return;

	float now = gpGlobals->time;
	for (int i = 1; i <= gpGlobals->maxClients && i <= kMaxPlayers; ++i)
	{
		edict_t *ent = g_engfuncs.pfnPEntityOfEntIndex(i);
		if (TimeLeft(g_saveMe[i] + kCallMemory, now, kCallMemory) > 0.f && Related(medic, ent, Teams::REL_FRIEND) &&
			ent->v.health < MaxHealth(ent))
			out.push_back(ent);
	}
}

void RefreshTeleporters()
{
	float now = gpGlobals->time;
	if (now < g_lastRefresh)
		g_links.clear();
	g_lastRefresh = now;

	const Nav::NavMesh *mesh = Nav::Get();
	if (!mesh || g_offsets.tele_iType < 0 || g_offsets.tele_iState < 0)
		return;

	struct Pad
	{
		edict_t *ent;
		edict_t *owner;
		int type;
		int state;
	};
	std::vector<Pad> pads;
	ForEachClass("building_teleporter", [&](edict_t *ent) {
		edict_t *owner = RealOwner(ent);
		if (owner)
			pads.push_back({ ent, owner, FieldInt(ent, g_offsets.tele_iType), FieldInt(ent, g_offsets.tele_iState) });
	});

	std::vector<Link> kept;
	for (const Pad &in : pads)
	{
		if (in.type != kTeleEntrance || in.state <= 0)
			continue;
		const Pad *out = nullptr;
		for (const Pad &p : pads)
		{
			if (p.type == kTeleExit && p.owner == in.owner && p.state > 0)
				out = &p;
		}
		if (!out)
			continue;

		float wait = in.state == kTeleReady ? 0.f : kPadBusyWait;
		Link *found = nullptr;
		for (Link &link : g_links)
		{
			if (link.entrance.Is(in.ent) && link.exit.Is(out->ent))
				found = &link;
		}

		if (found)
		{
			Nav::SetTeleportWait(found->conn, wait);
			kept.push_back(*found);
			found->conn = -1;
			continue;
		}

		const Nav::Area *from = Nav::FindAreaNearest(*mesh, in.ent->v.origin);
		const Nav::Area *to = Nav::FindAreaNearest(*mesh, out->ent->v.origin);
		if (!from || !to)
			continue;

		Link link;
		link.entrance.Set(in.ent);
		link.exit.Set(out->ent);
		link.conn = Nav::AddTeleport(in.ent->v.origin, from->id, out->ent->v.origin, to->id, wait,
			static_cast<uint32_t>(Objectives::TeamBit(in.owner->v.team)));
		if (link.conn < 0)
			continue;
		kept.push_back(link);
		BOT_VLOG("CrossBot: teleporter pair of %s linked as conn %d, area %u -> %u\n",
			CBotPlayerNameOf(in.owner), link.conn, from->id, to->id);
	}

	for (const Link &link : g_links)
	{
		if (link.conn >= 0)
			Nav::RemoveTeleport(link.conn);
	}
	g_links.swap(kept);
}

bool FeignHold(edict_t *pEdict)
{
	int slot = g_engfuncs.pfnIndexOfEdict(pEdict);
	if (slot < 1 || slot > kMaxPlayers)
		return false;

	float now = gpGlobals->time;
	if (pEdict->v.deadflag != kDeadFeign || pEdict->v.health <= 0.f)
	{
		g_feignStart[slot] = 0.f;
		return false;
	}

	if (g_feignStart[slot] <= 0.f || g_feignStart[slot] > now)
		g_feignStart[slot] = now;

	float held = now - g_feignStart[slot];
	bool rise = held >= kFeignMax || (held >= kFeignMin && !EnemyWatching(pEdict));
	if (rise && (now >= g_feignCommand[slot] || g_feignCommand[slot] - now > kFeignCommandGap))
	{
		FakeCmd::Run(pEdict, "feign");
		g_feignCommand[slot] = now + kFeignCommandGap;
	}
	return true;
}

}
