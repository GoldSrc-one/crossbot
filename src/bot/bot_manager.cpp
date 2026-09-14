#include <extdll.h>
#include <meta_api.h>

#include "bot_manager.h"
#include "bot.h"
#include "bot_camp.h"
#include "bot_combat.h"
#include "bot_deploy.h"
#include "bot_difficulty.h"
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
#include "../game/crossplay.h"
#include "../game/gamedata.h"
#include "../game/game_profile.h"
#include "../game/player_models.h"
#include "../game/teams.h"
#include "../nav/bsp_trace.h"
#include "../nav/nav.h"
#include "../nav/nav_math.h"

#include <cmath>
#include <string>

IGameProfile *g_pGameProfile = nullptr;

namespace
{

	constexpr int kMaxBots = 32;

	constexpr int kMaxDriftSpeeds = 8;

	float g_driftSpeeds[kMaxDriftSpeeds] = {};
	int g_driftSpeedCount = 0;
	bool g_driftScanned = false;
	bool g_meshDrifts = false;

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

	using XPlay::ScopedGame;

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

	XPlay::DetectProfiles();
	Teams::DetectMap();

	if (XPlay::PickRandomSupported() < 0)
		SERVER_PRINT("CrossBot: no loaded game is playable; bots disabled.\n");

	const char *mapName = g_engfuncs.pfnSzFromIndex(gpGlobals->mapname);
	Nav::LoadForMap(mapName);
	BspTrace::LoadForMap(mapName);

	g_driftSpeedCount = 0;
	g_driftScanned = false;
	g_meshDrifts = false;
}

void ServerDeactivate()
{
	for (auto &bot : g_bots)
		bot = CBot();

	BotSense::ResetBoneCache();
	Bot_ResetAirmoveCache();
	BotFun::ResetMap();

	XPlay::ForgetProfiles();
	Teams::ForgetMap();
	PlayerModels::Forget();
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

namespace
{
	void ThinkOneBot(CBot &bot)
	{
		BotPerf::NoteBot(bot.name);

		bool dead = g_pGameProfile->IsDead(bot.pEdict);
		if (dead)
		{
			BOTPERF_SCOPE(ZONE_RESPAWN);
			if (!bot.wasDead)
			{
				BotFun::NoteDeath(bot);
				BotDeploy::NoteDeath(bot);
			}
			g_pGameProfile->Respawn(bot.pEdict);
			bot.wasDead = true;
			BotTiming::EndBot(bot);
			return;
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
			bot.daze = BotDaze::State();
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

	bool NoteDriftSpeed(float speed)
	{
		for (int i = 0; i < g_driftSpeedCount; ++i)
		{
			if (fabsf(g_driftSpeeds[i] - speed) <= 1.f)
				return false;
		}

		if (g_driftSpeedCount >= kMaxDriftSpeeds)
			return false;

		g_driftSpeeds[g_driftSpeedCount++] = speed;
		return true;
	}

	void CheckDriftSpeeds()
	{
		const Nav::NavMesh *mesh = Nav::Get();
		if (!mesh)
			return;

		if (!g_driftScanned)
		{
			g_driftScanned = true;
			for (const Nav::Area &area : mesh->areas)
			{
				if (area.driftVelocity.x != 0.f || area.driftVelocity.y != 0.f)
				{
					g_meshDrifts = true;
					break;
				}
			}
		}

		if (!g_meshDrifts || g_driftSpeedCount >= kMaxDriftSpeeds)
			return;

		for (const auto &bot : g_bots)
		{
			if (!bot.inUse || !bot.pEdict || !Bot_MoveMaxSpeedKnown(bot.pEdict))
				continue;

			float speed = Bot_MaxSpeed(bot.pEdict);
			if (!NoteDriftSpeed(speed) || fabsf(speed - mesh->designMaxSpeed) <= 1.f)
				continue;

			char warn[256];
			snprintf(warn, sizeof(warn),
				"CrossBot: nav baked at %.0f u/s but %s runs at %.0f; conveyor and push-field "
				"costs adapt per bot, but navgen pruned at the baked speed.\n",
				mesh->designMaxSpeed, bot.name, speed);
			SERVER_PRINT(warn);
		}
	}
}

void StartFrame()
{
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

	if (anyBot && XPlay::Supported(XPlay::MapOwner()))
	{
		int mapOwner = XPlay::MapOwner();
		XPlay::EnterGame(mapOwner);
		XPlay::InGame(mapOwner, [] { BotSense::RefreshMonsters(); });
	}

	if (anyBot)
		CheckDriftSpeeds();

	for (auto &bot : g_bots)
	{
		if (!bot.inUse)
			continue;

		XPlay::EnterGame(bot.gameDll);
		if (!g_pGameProfile)
			continue;

		XPlay::InGame(bot.gameDll, [&bot] { ThinkOneBot(bot); });
	}

	XPlay::LeaveGame();

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
	if (!pentTouched || !pentOther)
		return;

	CBot *bot = FindBot(pentOther);
	edict_t *hit = pentTouched;
	if (!bot)
	{
		bot = FindBot(pentTouched);
		hit = pentOther;
	}

	if (!bot)
		return;

	ScopedGame ctx(bot->gameDll);
	if (!g_pGameProfile)
		return;

	bool debug = TouchDebugEnabled();

	XPlay::InGame(bot->gameDll, [&] {
		if (debug)
			DumpTouch(*bot, hit);
		BotItems::OnTouch(*bot, hit);
		NoteGravity(*bot);
	});
}

bool AddBot(int iGame, const char *name, const char *arg)
{
	char why[256];
	if (!XPlay::Supported(iGame))
	{
		snprintf(why, sizeof(why), "CrossBot: cannot add bot, game '%s' is not playable.\n",
			XPlay::GameDir(iGame));
		SERVER_PRINT(why);
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

	edict_t *pEdict = nullptr;
	XPlay::InGame(iGame, [&] { pEdict = CREATE_FAKE_CLIENT(botName); });

	if (!pEdict)
	{
		SERVER_PRINT("CrossBot: engine refused to create a fake client.\n");
		return false;
	}

	char chose[32] = {};
	XPlay::ProfileFor(iGame)->ApplyAddArg(iGame, pEdict, arg, chose, sizeof(chose));

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
	pSlot->gameDll = iGame;
	pSlot->pEdict = pEdict;
	snprintf(pSlot->name, sizeof(pSlot->name), "%s", botName);

	char msg[192];
	snprintf(msg, sizeof(msg), "CrossBot: added %s to '%s' as '%s'.\n", botName,
		XPlay::GameDir(iGame), chose[0] ? chose : "default");
	SERVER_PRINT(msg);

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

		XPlay::InGame(bot.gameDll, [&bot] { MDLL_ClientKill(bot.pEdict); });
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

void PrintLong(const std::string &msg)
{
	constexpr size_t kConsoleLine = 1000;

	size_t at = 0;
	while (at < msg.size())
	{
		if (msg.size() - at <= kConsoleLine)
		{
			SERVER_PRINT(msg.c_str() + at);
			return;
		}

		size_t cut = kConsoleLine;
		while (cut > kConsoleLine / 2 && msg[at + cut] != ' ' && msg[at + cut] != ',')
			--cut;

		SERVER_PRINT(msg.substr(at, cut).c_str());
		at += cut;
	}
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

		ScopedGame ctx(bot.gameDll);

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
			"CrossBot: %s target=%s bone=%d leadblk=%d queue=%zu/%zu hist=%.2fs delay=%.3fs "
			"pred=%u served=%.3fs hand=%.0f deg/s brake=(%.3f,%.3f) fov=%.0f quant=%.4f "
			"view=(%.1f,%.1f) err=%s\n",
			bot.name, target ? CBotTargetNameOf(target) : "-", bot.sense.targetBone,
			bot.sense.leadVetoes, bot.aim.delayCount, BotAim::kDelaySlots, BotAim::QueueAge(bot),
			BotAim::Delay(bot), bot.aim.predicts, bot.aim.servedDelay, BotAim::ViewSpeed(bot), bot.aim.brakeYaw,
			bot.aim.brakePitch,
			fov, BotAim::Quantum() * fov / 90.f, view.x, view.y, errBuf);
		SERVER_PRINT(line);
	}

	float delayLo = 0.f, delayHi = 0.f;
	BotDiff::AimDelay(delayLo, delayHi);

	char summary[256];
	snprintf(summary, sizeof(summary),
		"CrossBot: %d bot(s), sense=%d difficulty=%s aim_delay=%.3f-%.3f aim_error=%.2f "
		"aim_drift=%.2f aim_quantum=%.4f aim_impulse=%.1f\n", shown,
		BotSense::Enabled() ? 1 : 0, BotDiff::Name(BotDiff::Current()), delayLo, delayHi,
		BotAim::ErrorScale(), BotAim::DriftAmplitude(), BotAim::Quantum(), BotAim::Impulse());
	SERVER_PRINT(summary);
}

void PrintSenseStatus()
{
	int shown = 0;
	for (auto &bot : g_bots)
	{
		if (!bot.inUse)
			continue;

		ScopedGame ctx(bot.gameDll);

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
		int hunting = BotSense::LiveHunting(bot);

		const unsigned int *pushes = bot.sense.pushes;

		char line[768];
		snprintf(line, sizeof(line),
			"CrossBot: %s team=%d target=%s enemy=%s enemyAge=%.1fs bone=%d leadblk=%d "
			"cands=%d keep=%s keeps=inc:%u,new:%u susp=%d/%zu "
			"kinds=p:%d,m:%d,o:%d hunt=%d "
			"mon=list:%d,seen:%u,mine:%u,trc:%u,hit:%u "
			"tier=%s fresh=%s look=%s pushed=sgt:%u,snd:%u,evt:%u,dmg:%u\n",
			bot.name, Teams::TeamOf(bot.pEdict), target ? CBotTargetNameOf(target) : "-",
			enemy ? CBotTargetNameOf(enemy) : "-",
			enemy ? gpGlobals->time - bot.sense.enemyTime : 0.f, bot.sense.targetBone,
			bot.sense.leadVetoes,
			bot.sense.visionCands, BotSense::KeepName(bot.sense.visionKeep),
			bot.sense.keeps[BotSense::KEEP_INCUMBENT], bot.sense.keeps[BotSense::KEEP_NEW],
			live, BotSense::kSuspicionSlots,
			kinds[BotSense::KIND_PLAYER], kinds[BotSense::KIND_MONSTER],
			kinds[BotSense::KIND_OTHER], hunting,
			BotSense::MonsterCount(), bot.sense.monSeen, bot.sense.monMine,
			bot.sense.monTraces, bot.sense.monHits,
			bot.sense.lookValid ? BotSense::KindName(bot.sense.lookKind) : "-", freshBuf, lookBuf,
			pushes[BotSense::SUSP_SIGHT], pushes[BotSense::SUSP_SOUND],
			pushes[BotSense::SUSP_EVENT], pushes[BotSense::SUSP_DAMAGE]);
		SERVER_PRINT(line);
	}

	char summary[160];
	snprintf(summary, sizeof(summary),
		"CrossBot: %d bot(s), sense=%d sense_decay=%.2f teams=%d\n", shown,
		BotSense::Enabled() ? 1 : 0, BotSense::Decay(), Teams::Enabled() ? 1 : 0);
	SERVER_PRINT(summary);
}

void PrintMonsters()
{
	int mapOwner = XPlay::MapOwner();
	ScopedGame ctx(mapOwner);
	XPlay::InGame(mapOwner, [] { BotSense::RefreshMonsters(); });

	int count = 0;
	for (int i = 1; i < gpGlobals->maxEntities; ++i)
	{
		edict_t *ent = g_engfuncs.pfnPEntityOfEntIndex(i);
		if (!ent || ent->free)
			continue;
		if ((ent->v.flags & FL_MONSTER) == 0)
			continue;

		++count;

		edict_t *foe = g_pGameProfile ? g_pGameProfile->MonsterEnemy(ent) : nullptr;
		Vector centre = BotSense::EntitySoundOrigin(ent);

		char line[400];
		snprintf(line, sizeof(line),
			"CrossBot: monster #%d %s flags=0x%x health=%.0f takedamage=%d solid=%d "
			"box=(%.0f,%.0f,%.0f) enemy=%d:%s\n",
			i, CBotClassOf(ent), ent->v.flags, ent->v.health,
			static_cast<int>(ent->v.takedamage), ent->v.solid,
			centre.x, centre.y, centre.z,
			foe ? g_engfuncs.pfnIndexOfEdict(foe) : -1,
			foe ? CBotTargetNameOf(foe) : "-");
		SERVER_PRINT(line);

		for (auto &bot : g_bots)
		{
			if (!bot.inUse)
				continue;

			Vector eye = bot.pEdict->v.origin + bot.pEdict->v.view_ofs;
			Vector los = centre - eye;
			float dist = los.Length();

			bool mine = foe == bot.pEdict;

			bool cone = false;
			if (dist > 0.001f)
			{
				MAKE_VECTORS(bot.pEdict->v.v_angle);
				float fov = bot.pEdict->v.fov;
				if (fov <= 0.f)
					fov = 90.f;
				cone = DotProduct(gpGlobals->v_forward, los * (1.f / dist)) >=
					cosf(fov * 0.5f * NavMath::kPiF / 180.f);
			}

			TraceResult tr;
			TRACE_LINE(eye, centre, dont_ignore_monsters, bot.pEdict, &tr);
			bool los_ok = tr.flFraction >= 1.f || tr.pHit == ent;

			char sub[256];
			snprintf(sub, sizeof(sub),
				"CrossBot:   %s dist=%.0f mine=%d cone=%d los=%d hit=%s\n",
				bot.name, dist, mine ? 1 : 0, cone ? 1 : 0, los_ok ? 1 : 0,
				tr.pHit ? CBotClassOf(tr.pHit) : "-");
			SERVER_PRINT(sub);
		}
	}

	char summary[160];
	snprintf(summary, sizeof(summary),
		"CrossBot: %d monster edict(s), %d in the sense list, mon_hEnemy offset %d\n",
		count, BotSense::MonsterCount(), g_offsets.mon_hEnemy);
	SERVER_PRINT(summary);
}

void PrintTeamsStatus()
{
	char head[192];
	snprintf(head, sizeof(head),
		"CrossBot: map owner '%s' teamgame=%d crossbot_teams=%d -> teams %s\n",
		Teams::MapGameDir(), Teams::MapTeamplay() ? 1 : 0, Teams::Enabled() ? 1 : 0,
		Teams::Enabled() ? "on" : "off");
	SERVER_PRINT(head);

	for (int i = 1; i <= gpGlobals->maxClients; ++i)
	{
		edict_t *ent = g_engfuncs.pfnPEntityOfEntIndex(i);
		if (!CBotIsLivePlayer(ent))
			continue;

		char line[320];
		int written = snprintf(line, sizeof(line),
			"CrossBot: player #%d %s team=%d bot=%d", i, CBotPlayerNameOf(ent),
			Teams::TeamOf(ent), FindBot(ent) ? 1 : 0);

		for (auto &bot : g_bots)
		{
			if (!bot.inUse || bot.pEdict == ent)
				continue;
			if (written < 0 || written >= static_cast<int>(sizeof(line)))
				break;

			written += snprintf(line + written, sizeof(line) - written, " | %s:%s shoot=%d",
				bot.name, Teams::RelationName(Teams::Of(bot.pEdict, ent)),
				Teams::Attackable(bot.pEdict, ent) ? 1 : 0);
		}

		SERVER_PRINT(line);
		SERVER_PRINT("\n");
	}
}

void PrintCombatStatus()
{
	char head[128];
	snprintf(head, sizeof(head),
		"CrossBot: crossbot_combat=%d crossbot_combat_debug=%d crossbot_ambush=%d "
		"crossbot_duel=%d\n",
		BotCombat::Enabled() ? 1 : 0, BotCombat::DebugEnabled() ? 1 : 0,
		BotCombat::AmbushEnabled() ? 1 : 0, BotCombat::DuelEnabled() ? 1 : 0);
	SERVER_PRINT(head);

	for (auto &bot : g_bots)
	{
		if (!bot.inUse)
			continue;

		ScopedGame ctx(bot.gameDll);

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
			"CrossBot: %s fighting %s for %.1fs, unseen %.1fs, aware=%d ambush=%d, "
			"melee=%d inarea=%d duel=%d, driving=%d area %d\n",
			bot.name, enemy ? CBotTargetNameOf(enemy) : "-",
			gpGlobals->time - st.startTime, gpGlobals->time - st.lastSightTime,
			st.enemyAware ? 1 : 0, BotCombat::Ambushing(bot) ? 1 : 0,
			st.enemyMelee ? 1 : 0, st.enemyInArea ? 1 : 0, BotCombat::Dueling(bot) ? 1 : 0,
			st.driving ? 1 : 0, bot.currentAreaId);
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

void PrintCampStatus(const Vector *at)
{
	const Nav::NavMesh *mesh = Nav::Get();
	if (!mesh || mesh->camps.empty())
	{
		SERVER_PRINT("CrossBot: no camp spots loaded\n");
		return;
	}

	char head[192];
	snprintf(head, sizeof(head), "CrossBot: %d camp spots, crossbot_camp=%d\n",
		static_cast<int>(mesh->camps.size()), BotCamp::Enabled() ? 1 : 0);
	SERVER_PRINT(head);

	auto report = [&](const char *who, const Vector &origin)
	{
		const Nav::CampSpot *best = nullptr;
		float bestDist = 0.f;
		for (const Nav::CampSpot &spot : mesh->camps)
		{
			float d = (spot.pos - origin).Length();
			if (!best || d < bestDist)
			{
				best = &spot;
				bestDist = d;
			}
		}

		if (!best)
			return;

		char line[320];
		snprintf(line, sizeof(line),
			"CrossBot: %s at (%.0f,%.0f,%.0f): nearest camp %.1f away at (%.0f,%.0f,%.0f) "
			"area %d, yaw %.1f pitch %.1f, score %.2f%s\n",
			who, origin.x, origin.y, origin.z, bestDist, best->pos.x, best->pos.y, best->pos.z,
			best->areaId, best->yaw, best->pitch, best->score,
			(best->flags & crossbot_nav::CAMP_NO_STAND) ? " (crouch only)" : "");
		SERVER_PRINT(line);

		snprintf(line, sizeof(line),
			"CrossBot:   stand  score %.2f spread %.1f deg dist %.0f\n",
			best->standScore, best->standSpread, best->standDist);
		SERVER_PRINT(line);

		snprintf(line, sizeof(line),
			"CrossBot:   crouch score %.2f spread %.1f deg dist %.0f\n",
			best->crouchScore, best->crouchSpread, best->crouchDist);
		SERVER_PRINT(line);
	};

	if (at)
	{
		report("point", *at);
		return;
	}

	for (auto &bot : g_bots)
	{
		if (!bot.inUse)
			continue;

		ScopedGame ctx(bot.gameDll);
		report(bot.name, bot.pEdict->v.origin);

		const BotCamp::State &camp = bot.camp;
		char line[320];
		if (camp.holding)
			snprintf(line, sizeof(line),
				"CrossBot:   camping %.1fs at (%.0f,%.0f,%.0f) area %d %s%s, %zu on route, %u so far\n",
				gpGlobals->time - camp.holdStart, camp.spot.pos.x, camp.spot.pos.y, camp.spot.pos.z,
				camp.spot.areaId, camp.spot.duck ? "crouched" : "standing",
				camp.spot.zoom ? " zoomed" : "", camp.route.size(), camp.camps);
		else
			snprintf(line, sizeof(line), "CrossBot:   not camping (%s), %zu on route, %u so far\n",
				gpGlobals->time < camp.nextCampTime ? "cooling down" : "ready", camp.route.size(),
				camp.camps);
		SERVER_PRINT(line);

		if (camp.scared)
			snprintf(line, sizeof(line),
				"CrossBot:     scared: nearest contact %.0f out, closed in %.1fs ago\n",
				camp.scareDist, gpGlobals->time - camp.scareTime);
		else if (camp.friendInView)
			snprintf(line, sizeof(line), "CrossBot:     calm: a friend is in view\n");
		else
			snprintf(line, sizeof(line), "CrossBot:     nothing has come closer\n");
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

	auto report = [&](const char *who, const Vector &eye, float yaw)
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
			"(%.2f of max)\n",
			who, eye.x, eye.y, eye.z, blend.neighbors, blend.nearestDist, blend.exposure,
			field.maxExposure > 0.f ? blend.exposure / field.maxExposure : 0.f);
		SERVER_PRINT(head);

		for (int i = 0; i < blend.watchCount; ++i)
		{
			const Nav::VantageWatch &w = blend.watch[i];
			char line[160];
			snprintf(line, sizeof(line),
				"CrossBot:   watch %d: yaw %.1f pitch %.1f spread %.1f amount %.3f dist %.0f\n", i,
				w.yaw, w.pitch, w.spread, w.amount, w.dist);
			SERVER_PRINT(line);
		}

		char open[224];
		snprintf(open, sizeof(open),
			"CrossBot:   open dist: facing(%.0f) %.0f | +x %.0f +y %.0f -x %.0f -y %.0f\n", yaw,
			Nav::VantageOpenDistance(blend, Bot_YawDir(yaw)),
			Nav::VantageOpenDistance(blend, Vector(1.f, 0.f, 0.f)),
			Nav::VantageOpenDistance(blend, Vector(0.f, 1.f, 0.f)),
			Nav::VantageOpenDistance(blend, Vector(-1.f, 0.f, 0.f)),
			Nav::VantageOpenDistance(blend, Vector(0.f, -1.f, 0.f)));
		SERVER_PRINT(open);
	};

	if (at)
	{
		report("point", *at, 0.f);
		return;
	}

	int shown = 0;
	for (auto &bot : g_bots)
	{
		if (!bot.inUse)
			continue;

		ScopedGame ctx(bot.gameDll);

		++shown;
		report(bot.name, bot.pEdict->v.origin + bot.pEdict->v.view_ofs, bot.pEdict->v.v_angle.y);

		char drive[288];
		snprintf(drive, sizeof(drive),
			"CrossBot:   view=(%.1f,%.1f) desired=(%.1f,%.1f) look=%s glances=%u bhops=%u "
			"refuse=%u/%u/%u/%u/%u wp=%zu/%zu\n",
			bot.pEdict->v.v_angle.x, bot.pEdict->v.v_angle.y, bot.desiredAngles.x,
			bot.desiredAngles.y, BotSense::LookSourceName(bot.sense.lookSource),
			bot.sense.backGlances, bot.bunnyHops,
			bot.bunnyRefuse[0], bot.bunnyRefuse[1], bot.bunnyRefuse[2], bot.bunnyRefuse[3],
			bot.bunnyRefuse[4], bot.waypointIndex, bot.waypoints.size());
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

		ScopedGame ctx(bot.gameDll);

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
			"leadshots=%d chargeshots=%d arcshots=%d throwshots=%d guides=%d capped=%d "
			"stickyshots=%d uphill=%d selfveto=%d aimveto=%d targetaims=%d "
			"switches=%d reloads=%d emptyswitch=%d idleswitch=%d "
			"idlereload=%d ambush=%d duel=%d target=%s\n",
			bot.name, heldBuf, fire.zoomed ? 1 : 0, BotFire::ModeName(fire.mode), fire.lastReason,
			ammoBuf, aimBuf, BotFire::CommitName(fire.commit), held, fire.shots,
			fire.leadShots, fire.chargeShots, fire.arcShots, fire.throwShots, fire.guides,
			fire.commitCaps, fire.stickyShots,
			fire.uphillVetoes, fire.selfVetoes, fire.aimVetoes, fire.targetAims,
			fire.switches, fire.reloads,
			fire.emptySwitches, fire.idleSwitches, fire.idleReloads, fire.ambushes, fire.duels,
			target ? CBotTargetNameOf(target) : "-");
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

		ScopedGame ctx(bot.gameDll);

		++shown;

		const BotDeploy::State &st = bot.deploy;

		char line[320];
		snprintf(line, sizeof(line),
			"CrossBot: %s deploy=%s conn=%u open=%.0f surface=%s stance=%s placed=%d dets=%d "
			"aborts=%d selfhits=%d escapes=%d memo=%zu why=%s\n",
			bot.name, BotDeploy::PhaseName(st.phase), st.spot.connIndex, st.spot.opening,
			BotDeploy::SpotKindName(st.spot.kind), st.spot.duck ? "duck" : "stand", st.placements,
			st.detonations, st.aborts, st.selfHits, st.escapes, st.memo.size(), st.lastReason);
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

		ScopedGame ctx(bot.gameDll);

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

		ScopedGame ctx(bot.gameDll);
		if (!g_pGameProfile)
			continue;

		XPlay::InGame(bot.gameDll, [&] { BotSense::Hear(bot, src, pos, volume, attenuation); });
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

		ScopedGame ctx(bot.gameDll);
		if (!g_pGameProfile)
			continue;

		XPlay::InGame(bot.gameDll, [&] { BotSense::Notice(bot, pInvoker); });
	}
}

void PrintNavStatus()
{
	for (auto &bot : g_bots)
	{
		if (!bot.inUse)
			continue;

		ScopedGame ctx(bot.gameDll);
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
		snprintf(line, sizeof(line),
			" combat=%d aware=%d duel=%d enemy=%s enemyAge=%.1fs ready=%.2f",
			bot.combat.active ? 1 : 0, bot.combat.enemyAware ? 1 : 0,
			BotCombat::Dueling(bot) ? 1 : 0,
			enemy ? CBotTargetNameOf(enemy) : "-",
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
		PrintLong(msg);
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

