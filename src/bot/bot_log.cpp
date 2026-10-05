#include "bot_log.h"

#ifdef CROSSBOT_VERBOSE

#include <extdll.h>
#include <meta_api.h>

#include <stdarg.h>
#include <stdio.h>

namespace BotLog
{

bool Enabled()
{
	static cvar_t *cvar = nullptr;
	if (!cvar)
		cvar = g_engfuncs.pfnCVarGetPointer("crossbot_verbose");
	return cvar && cvar->value != 0.f;
}

void Verbosef(const char *fmt, ...)
{
	char msg[1024];

	va_list args;
	va_start(args, fmt);
	vsnprintf(msg, sizeof(msg), fmt, args);
	va_end(args);

	SERVER_PRINT(msg);
}

}

#endif
