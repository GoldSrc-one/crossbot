#include <extdll.h>
#include <meta_api.h>

#include "bot_manager.h"
#include "bot.h"
#include "bot_items.h"
#include "bot_move_nav.h"
#include "bot_weapons.h"
#include "../game/game_profile.h"
#include "../nav/nav.h"

#include <string>

IGameProfile *g_pGameProfile = nullptr;

namespace
{
	constexpr int kMaxBots = 16;

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

	bool TouchDebugEnabled()
	{
		cvar_t *cvar = g_engfuncs.pfnCVarGetPointer("crossbot_gather_debug");
		return cvar && cvar->value != 0.f;
	}

	void DumpTouch(const CBot &bot, edict_t *ent)
	{
		edict_t *goal = bot.goalItem.Get();
		char msg[288];
		snprintf(msg, sizeof(msg),
			"CrossBot: TOUCH %s at (%.1f,%.1f,%.1f) <- #%d %s at (%.1f,%.1f,%.1f) "
			"solid=%d effects=0x%x flags=0x%x free=%d | goal #%d %s done=%d\n",
			bot.name, bot.pEdict->v.origin.x, bot.pEdict->v.origin.y, bot.pEdict->v.origin.z,
			g_engfuncs.pfnIndexOfEdict(ent), CBotClassOf(ent), ent->v.origin.x, ent->v.origin.y,
			ent->v.origin.z, ent->v.solid, ent->v.effects, ent->v.flags, ent->free ? 1 : 0,
			goal ? g_engfuncs.pfnIndexOfEdict(goal) : -1, goal ? CBotClassOf(goal) : "-",
			bot.goalItemDone ? 1 : 0);
		SERVER_PRINT(msg);
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

	g_pGameProfile = GameProfile_Detect();

	char msg[256];
	if (g_pGameProfile)
		snprintf(msg, sizeof(msg), "CrossBot: detected game profile '%s'.\n", g_pGameProfile->GetName());
	else
		snprintf(msg, sizeof(msg), "CrossBot: no known game profile matched this mod; bots disabled.\n");
	SERVER_PRINT(msg);

	const char *mapName = g_engfuncs.pfnSzFromIndex(gpGlobals->mapname);
	Nav::LoadForMap(mapName);
}

void ServerDeactivate()
{
	for (auto &bot : g_bots)
		bot = CBot();

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
		*pBot = CBot();
}

void StartFrame()
{
	if (!g_pGameProfile)
		return;

	for (auto &bot : g_bots)
	{
		if (!bot.inUse)
			continue;

		bool dead = g_pGameProfile->IsDead(bot.pEdict);
		if (dead)
		{
			g_pGameProfile->Respawn(bot.pEdict);
			bot.wasDead = true;
			continue;
		}

		if (bot.wasDead)
		{
			bot.wasDead = false;
			BotItems::OnRespawn(bot);
		}

		BotWeapons::Refresh(bot);

		Bot_MoveNav(bot);
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
	}
	else if (CBot *other = FindBot(pentTouched))
	{
		if (debug)
			DumpTouch(*other, pentOther);
		BotItems::OnTouch(*other, pentOther);
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

