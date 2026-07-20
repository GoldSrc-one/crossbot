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
}

namespace CrossBotCommands
{

void Register()
{
	ADD_SERVER_COMMAND("crossbot_add", Cmd_CrossBotAdd);
	ADD_SERVER_COMMAND("crossbot_kick", Cmd_CrossBotKick);
	ADD_SERVER_COMMAND("crossbot_fill", Cmd_CrossBotFill);
}

}
