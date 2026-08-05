#include <extdll.h>
#include <meta_api.h>

#include "commands.h"
#include "../bot/bot_manager.h"
#include "../nav/nav.h"

namespace
{
	void Cmd_CrossBotAdd()
	{
		const char *name = CMD_ARGC() > 1 ? CMD_ARGV(1) : nullptr;
		BotManager::AddBot(name);
	}

	void Cmd_CrossBotNavGoal()
	{
		if (CMD_ARGC() < 4)
		{
			SERVER_PRINT("usage: crossbot_nav_goal <x> <y> <z>\n");
			return;
		}

		Vector point(static_cast<float>(atof(CMD_ARGV(1))), static_cast<float>(atof(CMD_ARGV(2))),
			static_cast<float>(atof(CMD_ARGV(3))));

		if (!BotManager::SetGoalForAllBots(point))
			SERVER_PRINT("CrossBot: no bots connected.\n");
	}

	void Cmd_CrossBotTeleport()
	{
		if (CMD_ARGC() < 4)
		{
			SERVER_PRINT("usage: crossbot_teleport <x> <y> <z>\n");
			return;
		}

		Vector point(static_cast<float>(atof(CMD_ARGV(1))), static_cast<float>(atof(CMD_ARGV(2))),
			static_cast<float>(atof(CMD_ARGV(3))));

		if (!BotManager::TeleportAllBots(point))
			SERVER_PRINT("CrossBot: no bots connected.\n");
	}

	void Cmd_CrossBotNavStatus()
	{
		BotManager::PrintNavStatus();
	}

	void Cmd_CrossBotNavWaypoints()
	{
		BotManager::PrintWaypoints();
	}

	void Cmd_CrossBotListSpawns()
	{
		static const char *kSpawnClassnames[] = {"info_player_start", "info_player_deathmatch"};
		for (const char *classname : kSpawnClassnames)
		{
			edict_t *ent = nullptr;
			while (true)
			{
				ent = FIND_ENTITY_BY_STRING(ent, "classname", classname);
				if (!ent || g_engfuncs.pfnIndexOfEdict(ent) == 0)
					break;

				char msg[128];
				snprintf(msg, sizeof(msg), "CrossBot: spawn %s (%.1f,%.1f,%.1f)\n", classname,
					ent->v.origin.x, ent->v.origin.y, ent->v.origin.z);
				SERVER_PRINT(msg);
			}
		}
	}

	void Cmd_CrossBotListTargets()
	{
		edict_t *ent = nullptr;
		while (true)
		{
			ent = FIND_ENTITY_BY_STRING(ent, "classname", "info_target");
			if (!ent || g_engfuncs.pfnIndexOfEdict(ent) == 0)
				break;

			char msg[128];
			snprintf(msg, sizeof(msg), "CrossBot: target info_target (%.1f,%.1f,%.1f)\n",
				ent->v.origin.x, ent->v.origin.y, ent->v.origin.z);
			SERVER_PRINT(msg);
		}
	}

	void Cmd_CrossBotNavPenalties()
	{
		const Nav::NavMesh *mesh = Nav::Get();
		if (!mesh)
		{
			SERVER_PRINT("CrossBot: no nav mesh loaded\n");
			return;
		}

		size_t shown = 0;
		for (size_t i = 0; i < mesh->conns.size(); ++i)
		{
			const Nav::Connection &c = mesh->conns[i];
			if (c.runtimePenalty <= 0.f)
				continue;

			char msg[160];
			snprintf(msg, sizeof(msg),
				"CrossBot: conn %zu %u->%u type %u cost %.2fs penalty %.2fs\n", i, c.areaA, c.areaB,
				static_cast<unsigned>(c.connType), c.costSeconds, c.runtimePenalty);
			SERVER_PRINT(msg);
			++shown;
		}

		char summary[96];
		snprintf(summary, sizeof(summary), "CrossBot: %zu/%zu connections penalized\n", shown,
			mesh->conns.size());
		SERVER_PRINT(summary);
	}
}

namespace CrossBotCommands
{

void Register()
{
	ADD_SERVER_COMMAND("crossbot_add", Cmd_CrossBotAdd);
	ADD_SERVER_COMMAND("crossbot_nav_goal", Cmd_CrossBotNavGoal);
	ADD_SERVER_COMMAND("crossbot_teleport", Cmd_CrossBotTeleport);
	ADD_SERVER_COMMAND("crossbot_nav_status", Cmd_CrossBotNavStatus);
	ADD_SERVER_COMMAND("crossbot_list_spawns", Cmd_CrossBotListSpawns);
	ADD_SERVER_COMMAND("crossbot_list_targets", Cmd_CrossBotListTargets);
	ADD_SERVER_COMMAND("crossbot_waypoints", Cmd_CrossBotNavWaypoints);
	ADD_SERVER_COMMAND("crossbot_nav_penalties", Cmd_CrossBotNavPenalties);
}

}
