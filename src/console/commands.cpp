#include <extdll.h>
#include <meta_api.h>

#include "commands.h"
#include "../bot/bot_manager.h"

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
}

}
