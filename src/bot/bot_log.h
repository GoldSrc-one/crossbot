#pragma once

namespace BotLog
{
	bool Enabled();
	void Verbosef(const char *fmt, ...);
}

#ifdef CROSSBOT_VERBOSE
	#define BOT_VLOG(...) do { if (BotLog::Enabled()) BotLog::Verbosef(__VA_ARGS__); } while (0)
	#define BOT_VLOG_ON 1
#else
	#define BOT_VLOG(...) ((void)0)
	#define BOT_VLOG_ON 0
#endif
