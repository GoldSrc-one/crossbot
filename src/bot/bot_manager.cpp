#include <extdll.h>
#include <meta_api.h>

#include "bot_manager.h"
#include "bot.h"
#include "bot_combat.h"
#include "bot_deploy.h"
#include "bot_fire.h"
#include "bot_fun.h"
#include "bot_goal.h"
#include "bot_items.h"
#include "bot_log.h"
#include "bot_move.h"
#include "bot_move_nav.h"
#include "bot_opportunist.h"
#include "bot_perf.h"
#include "bot_sense.h"
#include "bot_timing.h"
#include "bot_weapons.h"
#include "../game/game_profile.h"
#include "../nav/nav.h"

#include <cmath>
#include <string>

IGameProfile *g_pGameProfile = nullptr;

namespace
{

bool g_driftSpeedChecked = false;
	constexpr int kMaxBots = 32;

	constexpr float kBeamRefresh = 0.25f;

	float g_nextBeamRefresh = 0.f;

	CBot g_bots[kMaxBots];
	int g_botSerial = 0;

	CBot *FindFreeSlot()
	{
		for (auto &bot : g_bots)
		{
			if (!bot.inUse)
				return &bot;
		}
		return nullptr;
	}

	constexpr float kGravityEpsilon = 0.01f;

	bool TouchDebugEnabled()
	{
#if BOT_VLOG_ON
		cvar_t *cvar = g_engfuncs.pfnCVarGetPointer("crossbot_gather_debug");
		return cvar && cvar->value != 0.f;
#else
		return false;
#endif
	}

	void DumpTouch(const CBot &bot, edict_t *ent)
	{
		edict_t *goal = bot.goalItem.Get();
		BOT_VLOG("CrossBot: TOUCH %s at (%.1f,%.1f,%.1f) <- #%d %s at (%.1f,%.1f,%.1f) "
			"solid=%d effects=0x%x flags=0x%x free=%d | goal #%d %s done=%d\n",
			bot.name, bot.pEdict->v.origin.x, bot.pEdict->v.origin.y, bot.pEdict->v.origin.z,
			g_engfuncs.pfnIndexOfEdict(ent), CBotClassOf(ent), ent->v.origin.x, ent->v.origin.y,
			ent->v.origin.z, ent->v.solid, ent->v.effects, ent->v.flags, ent->free ? 1 : 0,
			goal ? g_engfuncs.pfnIndexOfEdict(goal) : -1, goal ? CBotClassOf(goal) : "-",
			bot.goalItemDone ? 1 : 0);
	}
}

namespace BotManager
{

CBot *FindBot(edict_t *pEdict)
{
	for (auto &bot : g_bots)
	{
		if (bot.inUse && bot.pEdict == pEdict)
			return &bot;
	}
	return nullptr;
}

void ServerActivate(edict_t *pEdictList, int edictCount, int clientMax)
{
	(void)pEdictList;
	(void)edictCount;
	(void)clientMax;

	for (auto &bot : g_bots)
		bot = CBot();

	BotSense::ResetBoneCache();
	Bot_ResetAirmoveCache();
	BotFun::ResetMap();

	g_pGameProfile = GameProfile_Detect();

	char msg[256];
	if (g_pGameProfile)
		snprintf(msg, sizeof(msg), "CrossBot: detected game profile '%s'.\n", g_pGameProfile->GetName());
	else
		snprintf(msg, sizeof(msg), "CrossBot: no known game profile matched this mod; bots disabled.\n");
	SERVER_PRINT(msg);

	const char *mapName = g_engfuncs.pfnSzFromIndex(gpGlobals->mapname);
	Nav::LoadForMap(mapName);

	g_driftSpeedChecked = false;
}

void ServerDeactivate()
{
	for (auto &bot : g_bots)
		bot = CBot();

	BotSense::ResetBoneCache();
	Bot_ResetAirmoveCache();
	BotFun::ResetMap();

	g_pGameProfile = nullptr;
	Nav::Unload();
}

void ClientPutInServer(edict_t *pEntity)
{

	(void)pEntity;
}

void ClientDisconnect(edict_t *pEntity)
{
	CBot *pBot = FindBot(pEntity);
	if (pBot)
	{
		BotTiming::Reset(pEntity);
		*pBot = CBot();
	}
}

void StartFrame()
{
	if (!g_pGameProfile)
		return;

	BotPerf::FrameBegin();

	Bot_BeginFrame();
	BotTiming::BeginFrame();

	if (gpGlobals->time >= g_nextBeamRefresh || gpGlobals->time < g_nextBeamRefresh - kBeamRefresh)
	{
		BOTPERF_SCOPE(ZONE_BEAMS);
		g_nextBeamRefresh = gpGlobals->time + kBeamRefresh;
		BotDeploy::RefreshBeams();
	}

	bool anyBot = false;
	for (const auto &bot : g_bots)
		anyBot = anyBot || bot.inUse;

	if (anyBot && !g_driftSpeedChecked)
	{
		g_driftSpeedChecked = true;

		const Nav::NavMesh *mesh = Nav::Get();
		float serverSpeed = g_engfuncs.pfnCVarGetFloat("sv_maxspeed");
		if (mesh && serverSpeed > 1.f && fabsf(serverSpeed - mesh->designMaxSpeed) > 1.f)
		{
			bool drifts = false;
			for (const Nav::Area &area : mesh->areas)
			{
				if (area.driftVelocity.x != 0.f || area.driftVelocity.y != 0.f)
				{
					drifts = true;
					break;
				}
			}

			if (drifts)
			{
				char warn[256];
				snprintf(warn, sizeof(warn),
					"CrossBot: nav baked at %.0f u/s but sv_maxspeed is %.0f; conveyor and push-field "
					"costs adapt per bot, but navgen pruned at the baked speed.\n",
					mesh->designMaxSpeed, serverSpeed);
				SERVER_PRINT(warn);
			}
		}
	}

	for (auto &bot : g_bots)
	{
		if (!bot.inUse)
			continue;

		BotPerf::NoteBot(bot.name);

		bool dead = g_pGameProfile->IsDead(bot.pEdict);
		if (dead)
		{
			BOTPERF_SCOPE(ZONE_RESPAWN);
			if (!bot.wasDead)
				BotFun::NoteDeath(bot);
			g_pGameProfile->Respawn(bot.pEdict);
			bot.wasDead = true;
			BotTiming::EndBot(bot);
			continue;
		}

		if (bot.wasDead)
		{
			BOTPERF_SCOPE(ZONE_RESPAWN);
			bot.wasDead = false;
			bot.gravity = 1.f;
			BotGoal::OnRespawn(bot);
			BotSense::OnRespawn(bot);
			BotCombat::OnRespawn(bot);
			BotFire::Reset(bot);
			BotDeploy::OnRespawn(bot);
			bot.brk = BotBreak::State();
		}

		{
			BOTPERF_SCOPE(ZONE_WEAPONS);
			BotWeapons::Refresh(bot);
			BotFire::AccrueCarry(bot);
		}

		{
			BOTPERF_SCOPE(ZONE_MOVENAV);
			Bot_MoveNav(bot);
		}

		BotTiming::EndBot(bot);
	}

	BotTiming::EndFrame();

	BotPerf::FrameEnd();
}

namespace
{
	void NoteGravity(CBot &bot)
	{
		float gravity = Bot_GravityMultiplier(bot.pEdict);
		if (std::fabs(gravity - bot.gravity) <= kGravityEpsilon)
			return;

		if (TouchDebugEnabled())
		{
			BOT_VLOG("CrossBot: %s gravity x%.2f -> x%.2f, repathing\n",
				bot.name, bot.gravity, gravity);
		}

		bot.gravity = gravity;
		bot.nextRepathTime = gpGlobals->time;
	}
}

void Touch(edict_t *pentTouched, edict_t *pentOther)
{
	if (!g_pGameProfile || !pentTouched || !pentOther)
		return;

	bool debug = TouchDebugEnabled();

	if (CBot *bot = FindBot(pentOther))
	{
		if (debug)
			DumpTouch(*bot, pentTouched);
		BotItems::OnTouch(*bot, pentTouched);
		NoteGravity(*bot);
	}
	else if (CBot *other = FindBot(pentTouched))
	{
		if (debug)
			DumpTouch(*other, pentOther);
		BotItems::OnTouch(*other, pentOther);
		NoteGravity(*other);
	}
}

bool AddBot(const char *name)
{
	if (!g_pGameProfile)
	{
		SERVER_PRINT("CrossBot: cannot add bot, no game profile detected.\n");
		return false;
	}

	CBot *pSlot = FindFreeSlot();
	if (!pSlot)
	{
		char msg[64];
		snprintf(msg, sizeof(msg), "CrossBot: cannot add bot, all %d slots in use.\n", kMaxBots);
		SERVER_PRINT(msg);
		return false;
	}

	char botName[32];
	if (name && name[0])
		snprintf(botName, sizeof(botName), "%s", name);
	else
		snprintf(botName, sizeof(botName), "CrossBot%d", ++g_botSerial);

	edict_t *pEdict = CREATE_FAKE_CLIENT(botName);
	if (!pEdict)
	{
		SERVER_PRINT("CrossBot: engine refused to create a fake client.\n");
		return false;
	}

		char rejectReason[128] = {};
	MDLL_ClientConnect(pEdict, botName, "127.0.0.1", rejectReason);

	if (rejectReason[0])
	{
		char msg[192];
		snprintf(msg, sizeof(msg), "CrossBot: game rejected bot connection: %s\n", rejectReason);
		SERVER_PRINT(msg);
		return false;
	}

	MDLL_ClientPutInServer(pEdict);
	pEdict->v.flags |= FL_CLIENT | FL_FAKECLIENT;

	pSlot->inUse = true;
	pSlot->pEdict = pEdict;
	snprintf(pSlot->name, sizeof(pSlot->name), "%s", botName);

	return true;
}

bool SetGoalForAllBots(const Vector &point)
{
	bool any = false;
	for (auto &bot : g_bots)
	{
		if (!bot.inUse)
			continue;
		bot.goalPoint = point;
		bot.hasGoal = true;
		Bot_ResetNavPath(bot);
		bot.traceLog.clear();
		any = true;
	}
	return any;
}

bool KillAllBots()
{
	bool any = false;
	for (auto &bot : g_bots)
	{
		if (!bot.inUse)
			continue;

		MDLL_ClientKill(bot.pEdict);
		any = true;
	}
	return any;
}

bool TeleportAllBots(const Vector &point)
{
	bool any = false;
	for (auto &bot : g_bots)
	{
		if (!bot.inUse)
			continue;

		SET_ORIGIN(bot.pEdict, point);
		bot.pEdict->v.velocity = Vector(0.f, 0.f, 0.f);
		bot.pEdict->v.v_angle = Vector(0.f, 0.f, 0.f);
		bot.pEdict->v.angles = Vector(0.f, 0.f, 0.f);
		bot.aim.yawVel = 0.f;
		bot.aim.pitchVel = 0.f;
		bot.aim.yawPending = 0.f;
		bot.aim.pitchPending = 0.f;
		BotAim::ClearDelayQueue(bot);
		Bot_ResetNavPath(bot);
		any = true;
	}
	return any;
}

void AppendAreaList(std::string &msg, const char *label, const std::vector<uint32_t> &ids, bool fromTail)
{
	constexpr size_t kMaxShown = 24;
	msg += " ";
	msg += label;
	msg += "=[";

	size_t begin = 0;
	size_t end = ids.size();
	bool truncated = ids.size() > kMaxShown;
	if (truncated)
	{
		if (fromTail)
			begin = ids.size() - kMaxShown;
		else
			end = kMaxShown;
	}

	if (truncated && fromTail)
		msg += "...,";
	char idBuf[16];
	for (size_t i = begin; i < end; ++i)
	{
		snprintf(idBuf, sizeof(idBuf), "%u", ids[i]);
		msg += idBuf;
		if (i + 1 < end)
			msg += ",";
	}
	if (truncated && !fromTail)
		msg += ",...";
	msg += "]";
}

void PrintAimStatus()
{
	int shown = 0;
	for (auto &bot : g_bots)
	{
		if (!bot.inUse)
			continue;

		++shown;

		edict_t *target = bot.sense.target.Get();
		const Vector &view = bot.pEdict->v.v_angle;

		char errBuf[64] = "-";
		float truthYaw = 0.f, truthPitch = 0.f;
		if (BotAim::AnglesToTarget(bot, target, truthYaw, truthPitch))
		{
			float dYaw = Bot_NormalizeAngle(truthYaw - view.y);
			float dPitch = Bot_NormalizeAngle(truthPitch - view.x);
			snprintf(errBuf, sizeof(errBuf), "%.2f (yaw %+.2f pitch %+.2f)",
				sqrtf(dYaw * dYaw + dPitch * dPitch), dYaw, dPitch);
		}

		float fov = bot.pEdict->v.fov;
		if (fov <= 0.f)
			fov = 90.f;

		char line[512];
		snprintf(line, sizeof(line),
			"CrossBot: %s target=%s bone=%d leadblk=%d queue=%zu/%zu hist=%.2fs served=%.3fs "
			"hand=%.0f deg/s brake=(%.3f,%.3f) fov=%.0f quant=%.4f view=(%.1f,%.1f) err=%s\n",
			bot.name, target ? CBotPlayerNameOf(target) : "-", bot.sense.targetBone,
			bot.sense.leadVetoes, bot.aim.delayCount, BotAim::kDelaySlots, BotAim::QueueAge(bot),
			bot.aim.servedDelay, BotAim::ViewSpeed(bot), bot.aim.brakeYaw, bot.aim.brakePitch,
			fov, BotAim::Quantum() * fov / 90.f, view.x, view.y, errBuf);
		SERVER_PRINT(line);
	}

	char summary[256];
	snprintf(summary, sizeof(summary),
		"CrossBot: %d bot(s), sense=%d aim_delay=%.3f aim_error=%.2f aim_drift=%.2f "
		"aim_quantum=%.4f aim_impulse=%.1f\n", shown,
		BotSense::Enabled() ? 1 : 0, BotAim::Delay(), BotAim::ErrorScale(),
		BotAim::DriftAmplitude(), BotAim::Quantum(), BotAim::Impulse());
	SERVER_PRINT(summary);
}

void PrintSenseStatus()
{
	int shown = 0;
	for (auto &bot : g_bots)
	{
		if (!bot.inUse)
			continue;

		++shown;

		edict_t *target = bot.sense.target.Get();
		edict_t *enemy = bot.sense.enemy.Get();
		const BotSense::Suspicion *fresh = BotSense::Freshest(bot);

		char freshBuf[96] = "-";
		if (fresh)
			snprintf(freshBuf, sizeof(freshBuf), "%s age=%.2fs ent=%d (%.0f,%.0f,%.0f)",
				BotSense::SourceName(fresh->source), gpGlobals->time - fresh->time,
				g_engfuncs.pfnIndexOfEdict(fresh->ent.Get()), fresh->pos.x, fresh->pos.y,
				fresh->pos.z);

		char lookBuf[96] = "-";
		if (bot.sense.lookValid)
			snprintf(lookBuf, sizeof(lookBuf), "(%.2f,%.2f,%.2f) auth=%.2f kept=%d",
				bot.sense.lookDir.x, bot.sense.lookDir.y, bot.sense.lookDir.z,
				bot.sense.lookAuthority, bot.sense.lookEntries);

		int kinds[BotSense::KIND_COUNT];
		int live = BotSense::LiveCounts(bot, kinds);

		const unsigned int *pushes = bot.sense.pushes;

		char line[640];
		snprintf(line, sizeof(line),
			"CrossBot: %s target=%s enemy=%s enemyAge=%.1fs bone=%d leadblk=%d "
			"cands=%d keep=%s keeps=inc:%u,new:%u susp=%d/%zu "
			"kinds=p:%d,m:%d,o:%d "
			"tier=%s fresh=%s look=%s pushed=sgt:%u,snd:%u,evt:%u,dmg:%u\n",
			bot.name, target ? CBotPlayerNameOf(target) : "-",
			enemy ? CBotPlayerNameOf(enemy) : "-",
			enemy ? gpGlobals->time - bot.sense.enemyTime : 0.f, bot.sense.targetBone,
			bot.sense.leadVetoes,
			bot.sense.visionCands, BotSense::KeepName(bot.sense.visionKeep),
			bot.sense.keeps[BotSense::KEEP_INCUMBENT], bot.sense.keeps[BotSense::KEEP_NEW],
			live, BotSense::kSuspicionSlots,
			kinds[BotSense::KIND_PLAYER], kinds[BotSense::KIND_MONSTER],
			kinds[BotSense::KIND_OTHER],
			bot.sense.lookValid ? BotSense::KindName(bot.sense.lookKind) : "-", freshBuf, lookBuf,
			pushes[BotSense::SUSP_SIGHT], pushes[BotSense::SUSP_SOUND],
			pushes[BotSense::SUSP_EVENT], pushes[BotSense::SUSP_DAMAGE]);
		SERVER_PRINT(line);
	}

	char summary[160];
	snprintf(summary, sizeof(summary),
		"CrossBot: %d bot(s), sense=%d sense_decay=%.2f\n", shown,
		BotSense::Enabled() ? 1 : 0, BotSense::Decay());
	SERVER_PRINT(summary);
}

void PrintCombatStatus()
{
	char head[128];
	snprintf(head, sizeof(head), "CrossBot: crossbot_combat=%d crossbot_combat_debug=%d\n",
		BotCombat::Enabled() ? 1 : 0, BotCombat::DebugEnabled() ? 1 : 0);
	SERVER_PRINT(head);

	for (auto &bot : g_bots)
	{
		if (!bot.inUse)
			continue;

		const BotCombat::State &st = bot.combat;
		edict_t *enemy = st.enemy.Get();

		char line[320];
		if (!st.active)
		{
			snprintf(line, sizeof(line), "CrossBot: %s not in combat (%s)\n", bot.name,
				st.lastReason);
			SERVER_PRINT(line);
			continue;
		}

		snprintf(line, sizeof(line),
			"CrossBot: %s fighting %s for %.1fs, unseen %.1fs, driving=%d area %d\n", bot.name,
			enemy ? CBotPlayerNameOf(enemy) : "-", gpGlobals->time - st.startTime,
			gpGlobals->time - st.lastSightTime, st.driving ? 1 : 0, bot.currentAreaId);
		SERVER_PRINT(line);

		if (st.haveProbe)
			snprintf(line, sizeof(line),
				"CrossBot:   probe (%.0f,%.0f,%.0f) area %d exposure %.4f, here %.4f, %s\n",
				st.probePoint.x, st.probePoint.y, st.probePoint.z, st.probeArea,
				st.probeExposure, st.hereExposure, st.lastReason);
		else
			snprintf(line, sizeof(line), "CrossBot:   no probe, here %.4f, %s\n", st.hereExposure,
				st.lastReason);
		SERVER_PRINT(line);
	}
}

void PrintVantageStatus(const Vector *at)
{
	const Nav::NavMesh *mesh = Nav::Get();
	if (!mesh)
	{
		SERVER_PRINT("CrossBot: no nav mesh loaded\n");
		return;
	}

	const Nav::VantageField &field = mesh->vantageField;
	char summary[192];
	snprintf(summary, sizeof(summary),
		"CrossBot: vantage field %zu points, spacing %.0f, cell %.0f, grid %dx%dx%d, "
		"maxExposure %.4f, crossbot_vantage=%d\n",
		field.vantages.size(), field.spacing, field.cell, field.dims[0], field.dims[1],
		field.dims[2], field.maxExposure, BotSense::VantageEnabled() ? 1 : 0);
	SERVER_PRINT(summary);

	if (field.vantages.empty())
		return;

	auto report = [&](const char *who, const Vector &eye)
	{
		Nav::VantageBlend blend;
		if (!Nav::BlendVantage(field, eye, blend))
		{
			char none[160];
			snprintf(none, sizeof(none), "CrossBot: %s at (%.0f,%.0f,%.0f): no blend\n", who,
				eye.x, eye.y, eye.z);
			SERVER_PRINT(none);
			return;
		}

		char head[256];
		snprintf(head, sizeof(head),
			"CrossBot: %s at (%.0f,%.0f,%.0f): %d neighbors, nearest %.1f, exposure %.4f "
			"(%.2f of max), focus %.2f/%.2f/%.2f/%.2f\n",
			who, eye.x, eye.y, eye.z, blend.neighbors, blend.nearestDist, blend.exposure,
			field.maxExposure > 0.f ? blend.exposure / field.maxExposure : 0.f,
			blend.focus[0], blend.focus[1], blend.focus[2], blend.focus[3]);
		SERVER_PRINT(head);

		for (int i = 0; i < blend.watchCount; ++i)
		{
			const Nav::VantageWatch &w = blend.watch[i];
			char line[160];
			snprintf(line, sizeof(line),
				"CrossBot:   watch %d: yaw %.1f pitch %.1f spread %.1f amount %.3f\n", i, w.yaw,
				w.pitch, w.spread, w.amount);
			SERVER_PRINT(line);
		}
	};

	if (at)
	{
		report("point", *at);
		return;
	}

	int shown = 0;
	for (auto &bot : g_bots)
	{
		if (!bot.inUse)
			continue;

		++shown;
		report(bot.name, bot.pEdict->v.origin + bot.pEdict->v.view_ofs);

		char drive[224];
		snprintf(drive, sizeof(drive),
			"CrossBot:   view=(%.1f,%.1f) desired=(%.1f,%.1f) look=%s wp=%zu/%zu\n",
			bot.pEdict->v.v_angle.x, bot.pEdict->v.v_angle.y, bot.desiredAngles.x,
			bot.desiredAngles.y, BotSense::LookSourceName(bot.sense.lookSource),
			bot.waypointIndex, bot.waypoints.size());
		SERVER_PRINT(drive);
	}

	if (shown == 0)
		SERVER_PRINT("CrossBot: no bots; pass \"x y z\" to sample a point\n");
}

void PrintFireStatus()
{
	int shown = 0;
	for (auto &bot : g_bots)
	{
		if (!bot.inUse)
			continue;

		++shown;

		const BotFire::State &fire = bot.fire;
		const BotInventory &inv = bot.inventory;

		char heldBuf[64] = "-";
		char ammoBuf[64] = "-";
		if (inv.activeIndex >= 0 && inv.activeIndex < inv.count)
		{
			const BotWeapon &w = inv.weapons[inv.activeIndex];
			const WeaponSpec *spec = g_pGameProfile ? g_pGameProfile->GetWeaponSpec(w.id) : nullptr;
			snprintf(heldBuf, sizeof(heldBuf), "%s", spec && spec->classname ? spec->classname : "?");

			int prim = (w.primaryAmmoType >= 0 && w.primaryAmmoType < MAX_AMMO_SLOTS)
				? inv.ammo[w.primaryAmmoType] : -1;
			int sec = (w.secondaryAmmoType >= 0 && w.secondaryAmmoType < MAX_AMMO_SLOTS)
				? inv.ammo[w.secondaryAmmoType] : -1;
			snprintf(ammoBuf, sizeof(ammoBuf), "clip=%d prim=%d sec=%d rel=%d", w.clip, prim, sec,
				w.reloading ? 1 : 0);
		}

		const char *aimKind = "target";
		if (fire.aimArc)
			aimKind = "arc";
		else if (fire.aimHoming)
			aimKind = "homing";
		else if (fire.aimLead)
			aimKind = "lead";

		char aimBuf[96] = "-";
		if (fire.lastMiss >= 0.f)
			snprintf(aimBuf, sizeof(aimBuf), "aim=%s flight=%.0f miss=%.2f tol=%.2f",
				aimKind, fire.flightTime * 1000.f, fire.lastMiss, fire.lastTol);

		float held = (fire.commit != BotFire::COMMIT_NONE)
			? (gpGlobals->time - fire.commitStart) : -1.f;

		edict_t *target = bot.sense.target.Get();

		char line[768];
		snprintf(line, sizeof(line),
			"CrossBot: %s held=%s zoom=%d mode=%s why=%s %s %s commit=%s for=%.2f shots=%d "
			"leadshots=%d chargeshots=%d arcshots=%d throwshots=%d guides=%d capped=%d uphill=%d "
			"selfveto=%d aimveto=%d targetaims=%d switches=%d reloads=%d emptyswitch=%d idleswitch=%d "
			"idlereload=%d target=%s\n",
			bot.name, heldBuf, fire.zoomed ? 1 : 0, BotFire::ModeName(fire.mode), fire.lastReason,
			ammoBuf, aimBuf, BotFire::CommitName(fire.commit), held, fire.shots,
			fire.leadShots, fire.chargeShots, fire.arcShots, fire.throwShots, fire.guides,
			fire.commitCaps,
			fire.uphillVetoes, fire.selfVetoes, fire.aimVetoes, fire.targetAims,
			fire.switches, fire.reloads,
			fire.emptySwitches, fire.idleSwitches, fire.idleReloads,
			target ? CBotPlayerNameOf(target) : "-");
		SERVER_PRINT(line);
	}

	char summary[160];
	snprintf(summary, sizeof(summary), "CrossBot: %d bot(s), fire=%d\n", shown,
		BotFire::Enabled() ? 1 : 0);
	SERVER_PRINT(summary);
}

void PrintDeployStatus()
{
	int shown = 0;
	for (auto &bot : g_bots)
	{
		if (!bot.inUse)
			continue;

		++shown;

		const BotDeploy::State &st = bot.deploy;

		char line[320];
		snprintf(line, sizeof(line),
			"CrossBot: %s deploy=%s conn=%u open=%.0f surface=%s stance=%s placed=%d dets=%d "
			"aborts=%d memo=%zu why=%s\n",
			bot.name, BotDeploy::PhaseName(st.phase), st.spot.connIndex, st.spot.opening,
			BotDeploy::SpotKindName(st.spot.kind), st.spot.duck ? "duck" : "stand", st.placements,
			st.detonations, st.aborts, st.memo.size(), st.lastReason);
		SERVER_PRINT(line);
	}

	char summary[160];
	snprintf(summary, sizeof(summary), "CrossBot: %d bot(s), deploy=%d\n", shown,
		BotDeploy::Enabled() ? 1 : 0);
	SERVER_PRINT(summary);
}

void PrintWeaponCarry()
{
	int shown = 0;
	for (auto &bot : g_bots)
	{
		if (!bot.inUse)
			continue;

		++shown;

		const BotInventory &inv = bot.inventory;

		char heldBuf[64] = "-";
		if (inv.activeIndex >= 0 && inv.activeIndex < inv.count)
		{
			const WeaponSpec *spec = g_pGameProfile
				? g_pGameProfile->GetWeaponSpec(inv.weapons[inv.activeIndex].id) : nullptr;
			snprintf(heldBuf, sizeof(heldBuf), "%s", spec && spec->classname ? spec->classname : "?");
		}

		char line[1024];
		const int cap = static_cast<int>(sizeof(line)) - 1;
		int used = snprintf(line, sizeof(line), "CrossBot: %s held=%s", bot.name, heldBuf);
		if (used > cap)
			used = cap;
		int carried = 0;

		for (int id = 1; id < BotFire::kMaxWeaponIds; ++id)
		{
			float carry = BotFire::CarryOf(bot, id);
			if (carry <= 0.f)
				continue;

			const WeaponSpec *spec = g_pGameProfile ? g_pGameProfile->GetWeaponSpec(id) : nullptr;
			const char *name = (spec && spec->classname) ? spec->classname : "?";

			++carried;

			if (used < cap)
			{
				used += snprintf(line + used, sizeof(line) - used, " %s=%.1f/%.1f", name, carry,
					spec ? spec->value : 0.f);
				if (used > cap)
					used = cap;
			}
		}

		if (used < cap)
			snprintf(line + used, sizeof(line) - used, " carried=%d\n", carried);

		SERVER_PRINT(line);
	}

	char summary[160];
	snprintf(summary, sizeof(summary), "CrossBot: %d bot(s)\n", shown);
	SERVER_PRINT(summary);
}

void OnSound(edict_t *src, int channel, const char *sample, float volume, float attenuation)
{
	if (!BotSense::Enabled() || !src || src->free)
		return;
	if (g_engfuncs.pfnIndexOfEdict(src) == 0)
		return;
	if (volume <= 0.f)
		return;
	if (sample && strcmp(sample, "common/null.wav") == 0)
		return;

	Vector pos = BotSense::EntitySoundOrigin(src);

	for (auto &bot : g_bots)
	{
		if (!bot.inUse || bot.pEdict == src)
			continue;
		BotSense::Hear(bot, src, pos, volume, attenuation);
	}
}

void OnEvent(int flags, const edict_t *pInvoker)
{
	if (!BotSense::Enabled() || !pInvoker || pInvoker->free)
		return;
	if ((flags & FEV_HOSTONLY) != 0)
		return;
	if (!CBotIsLivePlayer(pInvoker))
		return;

	for (auto &bot : g_bots)
	{
		if (!bot.inUse || bot.pEdict == pInvoker)
			continue;
		BotSense::Notice(bot, pInvoker);
	}
}

void PrintNavStatus()
{
	for (auto &bot : g_bots)
	{
		if (!bot.inUse)
			continue;
		Vector origin = bot.pEdict->v.origin;
		Vector vel = bot.pEdict->v.velocity;
		Vector angles = bot.pEdict->v.v_angle;
		char targetBuf[72] = "(none)";
		if (bot.waypointIndex < bot.waypoints.size())
		{
			const Nav::Waypoint &wp = bot.waypoints[bot.waypointIndex];
			snprintf(targetBuf, sizeof(targetBuf), "(%.0f,%.0f,%.0f)%s", wp.point.x, wp.point.y, wp.point.z,
				wp.requiresJump ? "[JUMP]" : "");
		}

		bool onGround = (bot.pEdict->v.flags & FL_ONGROUND) != 0;

		float duckLeft = bot.avoid.duckUntil - gpGlobals->time;
		if (duckLeft < 0.f)
			duckLeft = 0.f;

		size_t detourLeft = (bot.waypointIndex < bot.avoid.detourEnd)
			? bot.avoid.detourEnd - bot.waypointIndex : 0;

		char line[600];
		snprintf(line, sizeof(line),
			"CrossBot: %s origin=(%.0f,%.0f,%.0f) vel=(%.0f,%.0f,%.0f) angles=(%.0f,%.0f) move=(%.0f,%.0f) "
			"hasGoal=%d goal=(%.0f,%.0f,%.0f) target=%s waypoint=%zu/%zu movetype=%d area=%d onground=%d "
			"avoid=%s duckLeft=%.2f hit=%s gap=%.0f top=%.0f detour=%zu",
			bot.name, origin.x, origin.y, origin.z, vel.x, vel.y, vel.z, angles.x, angles.y,
			bot.moveForward, bot.moveSide,
			bot.hasGoal ? 1 : 0, bot.goalPoint.x, bot.goalPoint.y, bot.goalPoint.z, targetBuf,
			bot.waypointIndex, bot.waypoints.size(), bot.pEdict->v.movetype, bot.currentAreaId, onGround ? 1 : 0,
			BotAvoid::KindName(bot.avoid.lastKind), duckLeft,
			bot.avoid.lastHit[0] ? bot.avoid.lastHit : "-", bot.avoid.lastGap, bot.avoid.lastTop,
			detourLeft);
		std::string msg = line;

		edict_t *goalItem = bot.goalItem.Get();
		snprintf(line, sizeof(line), " item=%s itemKind=%d itemArea=%d memo=%zu full=%zu",
			goalItem ? CBotClassOf(goalItem) : "-", static_cast<int>(bot.goalItemKind),
			bot.goalItemArea, bot.itemMemo.size(), bot.fullMemo.size());
		msg += line;

		edict_t *targetEnt = bot.goal.target.ent.Get();
		snprintf(line, sizeof(line), " tier=%s stopTier=%s detour=%d target=%s targetArea=%d",
			BotGoal::TierName(bot.goal.target.tier), BotGoal::TierName(bot.goal.stop.tier),
			bot.goal.onDetour ? 1 : 0, targetEnt ? CBotClassOf(targetEnt) : "-",
			bot.goal.target.areaId);
		msg += line;

		if (bot.fun.mounted || bot.fun.goal.Get())
		{
			edict_t *funEnt = bot.fun.goal.Get();
			snprintf(line, sizeof(line), " fun=%s funKind=%s funAct=%s mounted=%d shots=%d",
				funEnt ? CBotClassOf(funEnt) : "-", BotFun::KindName(bot.fun.kind),
				BotFun::ActionName(bot.fun.action), bot.fun.mounted ? 1 : 0, bot.fun.shots);
			msg += line;
		}

		edict_t *enemy = bot.sense.enemy.Get();
		snprintf(line, sizeof(line), " combat=%d enemy=%s enemyAge=%.1fs ready=%.2f",
			bot.combat.active ? 1 : 0, enemy ? CBotPlayerNameOf(enemy) : "-",
			enemy ? gpGlobals->time - bot.sense.enemyTime : 0.f,
			g_pGameProfile ? g_pGameProfile->EvaluateCombat(bot.pEdict, bot.inventory) : 0.f);
		msg += line;

		if (bot.brk.active)
		{
			snprintf(line, sizeof(line), " breaking=%s health=%.0f swings=%d for=%.1fs",
				bot.brk.targetName, bot.brk.lastHealth, bot.brk.swings,
				gpGlobals->time - bot.brk.startTime);
			msg += line;
		}

		if (bot.chargerActive)
		{
			snprintf(line, sizeof(line), " charging=%.1fs value=%.0f",
				gpGlobals->time - bot.chargerStartTime, bot.chargerLastValue);
			msg += line;
		}

		if (bot.rideActive)
		{
			snprintf(line, sizeof(line), " riding=%.1fs", gpGlobals->time - bot.rideStartTime);
			msg += line;
		}

		if (bot.ladderActive)
		{
			snprintf(line, sizeof(line), " ladder=%.1fs up=%d grabbed=%d top=%d stalls=%d offz=%.0f",
				gpGlobals->time - bot.ladderStartTime, bot.ladderUp ? 1 : 0, bot.ladderGrabbed ? 1 : 0,
				bot.ladderAtTop ? 1 : 0, bot.ladderStalls, bot.ladderDismountZ);
			msg += line;
		}

		if (bot.hasGoal && bot.estimatedArrivalTime > 0.f)
		{
			float elapsed = gpGlobals->time - bot.pathStartTime;
			float budget = bot.estimatedArrivalTime - bot.pathStartTime;
			bool overdue = gpGlobals->time > bot.estimatedArrivalTime;
			snprintf(line, sizeof(line), " elapsed=%.1fs/eta=%.1fs%s", elapsed, budget, overdue ? " OVERDUE" : "");
			msg += line;
		}

		AppendAreaList(msg, "planned", bot.plannedAreaPath, false);
		AppendAreaList(msg, "actual", bot.visitedAreaPath, true);

		if (bot.hasGoal && !bot.waypoints.empty())
		{
			msg += " waypoints=[";
			for (size_t i = 0; i < bot.waypoints.size(); ++i)
			{
				const Nav::Waypoint &wp = bot.waypoints[i];
				char wpBuf[128];
							snprintf(wpBuf, sizeof(wpBuf), "(wp%zu:(%.0f,%.0f,%.0f)%s%s",
					i, wp.point.x, wp.point.y, wp.point.z,
					wp.requiresJump ? " J" : "",
					wp.requiresCrouch ? " C" : "");
				msg += wpBuf;
				if (i + 1 < bot.waypoints.size())
					msg += ",";
			}
			msg += "]";
		}

		msg += "\n";
		SERVER_PRINT(msg.c_str());
	}
}

void PrintWaypoints()
{
		for (auto &bot : g_bots)
		{
			if (!bot.inUse)
				continue;
			char line[512];
			snprintf(line, sizeof(line), "CrossBot: %s waypoints (%zu):\n", bot.name, bot.waypoints.size());
			SERVER_PRINT(line);
			for (size_t i = 0; i < bot.waypoints.size(); ++i)
			{
				const Nav::Waypoint &wp = bot.waypoints[i];
				snprintf(line, sizeof(line),
					"  wp[%zu]=(%.1f,%.1f,%.1f) jump=%d crouch=%d targArea=%d\n",
					i, wp.point.x, wp.point.y, wp.point.z,
					wp.requiresJump, wp.requiresCrouch, wp.targetAreaId);
				SERVER_PRINT(line);
			}
		}
	}

}

