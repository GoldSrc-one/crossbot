#pragma once

namespace BotStall
{
#if defined(CROSSBOT_PERF) && defined(_WIN32)

	void Register();
	void Beat(bool inCrossbot);
	void Flush();
	void Shutdown();

#else

	inline void Register() {}
	inline void Beat(bool) {}
	inline void Flush() {}
	inline void Shutdown() {}

#endif
}
