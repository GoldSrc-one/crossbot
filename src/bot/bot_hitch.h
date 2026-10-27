#pragma once

namespace BotHitch
{
	enum
	{
		SECTION_PRE = 0,
		SECTION_BOTS,
		SECTION_BRAIN,
		SECTION_COUNT
	};

#ifdef CROSSBOT_PERF

	void Register();
	void Reset();

	void FrameBegin();
	void Section(int section);
	void BotDone(const char *name);
	void FrameEnd();

#else

	inline void Register() {}
	inline void Reset() {}

	inline void FrameBegin() {}
	inline void Section(int) {}
	inline void BotDone(const char *) {}
	inline void FrameEnd() {}

#endif
}
