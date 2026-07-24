#include <extdll.h>
#include <meta_api.h>

#include "commands.h"
#include "../bot/bot_manager.h"
#include "../nav/nav_avoid.h"
#include "../nav/nav_debug.h"

namespace
{
	void Cmd_CrossBotAdd()
	{
		const char *name = CMD_ARGC() > 1 ? CMD_ARGV(1) : nullptr;
		BotManager::AddBot(name);
	}

	void Cmd_CrossBotKick()
	{
		if (!BotManager::RemoveBot())
			SERVER_PRINT("CrossBot: no bots connected.\n");
	}

	void Cmd_CrossBotFill()
	{
		int target = CMD_ARGC() > 1 ? atoi(CMD_ARGV(1)) : 1;

		while (BotManager::GetBotCount() < target)
		{
			if (!BotManager::AddBot(nullptr))
				break;
		}
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

	void Cmd_CrossBotNavAvoid()
	{
		if (CMD_ARGC() < 2)
		{
			char msg[64];
			snprintf(msg, sizeof(msg), "crossbot_nav_avoid is %d\n", NavAvoid::IsEnabled() ? 1 : 0);
			SERVER_PRINT(msg);
			return;
		}
		NavAvoid::SetEnabled(atoi(CMD_ARGV(1)) != 0);
	}

	void Cmd_CrossBotNavDebugPath()
	{
		if (CMD_ARGC() < 2)
		{
			char msg[64];
			snprintf(msg, sizeof(msg), "crossbot_nav_debug_path is %d\n", NavDebug::PathBeamsEnabled() ? 1 : 0);
			SERVER_PRINT(msg);
			return;
		}
		NavDebug::SetPathBeamsEnabled(atoi(CMD_ARGV(1)) != 0);
	}

	void Cmd_CrossBotNavDebugArea()
	{
		if (CMD_ARGC() < 2)
		{
			char msg[64];
			snprintf(msg, sizeof(msg), "crossbot_nav_debug_area is %d\n", NavDebug::AreaBeamsEnabled() ? 1 : 0);
			SERVER_PRINT(msg);
			return;
		}
		NavDebug::SetAreaBeamsEnabled(atoi(CMD_ARGV(1)) != 0);
	}

	// Dev/testing hook: prints every info_player_start/info_player_deathmatch entity's coordinates so external test driver can pick spawn pairs without separate BSP entity parser -- this is the exact same live world bot spawns/navigates in.
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
					break; // wraps back to world entity (index 0) when exhausted

				char msg[128];
				snprintf(msg, sizeof(msg), "CrossBot: spawn %s (%.1f,%.1f,%.1f)\n", classname,
					ent->v.origin.x, ent->v.origin.y, ent->v.origin.z);
				SERVER_PRINT(msg);
			}
		}
	}

	// Same idea as Cmd_CrossBotListSpawns, but for info_target -- the goal marker used by simple single-target test maps (e.g. test1_simple), so test driver can discover coordinate for crossbot_nav_goal without its own BSP entity parser.
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
	ADD_SERVER_COMMAND("crossbot_kick", Cmd_CrossBotKick);
	ADD_SERVER_COMMAND("crossbot_fill", Cmd_CrossBotFill);
	ADD_SERVER_COMMAND("crossbot_nav_goal", Cmd_CrossBotNavGoal);
	ADD_SERVER_COMMAND("crossbot_teleport", Cmd_CrossBotTeleport);
	ADD_SERVER_COMMAND("crossbot_nav_status", Cmd_CrossBotNavStatus);
	ADD_SERVER_COMMAND("crossbot_nav_avoid", Cmd_CrossBotNavAvoid);
	ADD_SERVER_COMMAND("crossbot_nav_debug_path", Cmd_CrossBotNavDebugPath);
	ADD_SERVER_COMMAND("crossbot_nav_debug_area", Cmd_CrossBotNavDebugArea);
	ADD_SERVER_COMMAND("crossbot_list_spawns", Cmd_CrossBotListSpawns);
	ADD_SERVER_COMMAND("crossbot_list_targets", Cmd_CrossBotListTargets);
}

}
