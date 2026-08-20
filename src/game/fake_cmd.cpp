#include <extdll.h>
#include <meta_api.h>

#include "fake_cmd.h"

namespace
{
	bool g_active = false;

	char g_line[FakeCmd::kMaxLength] = {};
	char g_tokens[FakeCmd::kMaxLength] = {};
	const char *g_argv[FakeCmd::kMaxArgs] = {};
	int g_argc = 0;
	const char *g_args = nullptr;

	const char *kEmpty = "";

	void Tokenize(const char *cmd)
	{
		g_argc = 0;
		g_args = nullptr;

		snprintf(g_line, sizeof(g_line), "%s", cmd);
		memcpy(g_tokens, g_line, sizeof(g_tokens));

		char *p = g_tokens;
		while (*p && g_argc < FakeCmd::kMaxArgs)
		{
			while (*p == ' ' || *p == '\t')
				++p;
			if (!*p)
				break;

			g_argv[g_argc++] = p;

			while (*p && *p != ' ' && *p != '\t')
				++p;
			if (*p)
				*p++ = '\0';

			if (g_argc == 1)
				g_args = g_line + (p - g_tokens);
		}
	}
}

namespace FakeCmd
{

bool Run(edict_t *player, const char *cmd)
{
	if (!player || player->free || !cmd || !cmd[0])
		return false;

	if (strlen(cmd) >= kMaxLength)
		return false;

	if (g_active)
		return false;

	Tokenize(cmd);
	if (g_argc == 0)
		return false;

	g_active = true;
	MDLL_ClientCommand(player);
	g_active = false;

	return true;
}

const char *Args()
{
	if (!g_active)
		RETURN_META_VALUE(MRES_IGNORED, kEmpty);

	RETURN_META_VALUE(MRES_SUPERCEDE, g_args ? g_args : kEmpty);
}

const char *Argv(int argc)
{
	if (!g_active)
		RETURN_META_VALUE(MRES_IGNORED, kEmpty);

	if (argc < 0 || argc >= g_argc)
		RETURN_META_VALUE(MRES_SUPERCEDE, kEmpty);

	RETURN_META_VALUE(MRES_SUPERCEDE, g_argv[argc]);
}

int Argc()
{
	if (!g_active)
		RETURN_META_VALUE(MRES_IGNORED, 0);

	RETURN_META_VALUE(MRES_SUPERCEDE, g_argc);
}

}
