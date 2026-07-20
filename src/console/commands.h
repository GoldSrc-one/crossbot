#pragma once

// Registers crossbot_* server console commands (pfnAddServerCommand).
// Called once from the pfnGameInit post-hook in src/meta/dll_hooks.cpp.
namespace CrossBotCommands
{
	void Register();
}
