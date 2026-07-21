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

	void Cmd_CrossBotNavStatus()
	{
		BotManager::PrintNavStatus();
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
	ADD_SERVER_COMMAND("crossbot_nav_status", Cmd_CrossBotNavStatus);
}

}
