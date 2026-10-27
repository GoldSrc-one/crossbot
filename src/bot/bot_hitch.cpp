#ifdef CROSSBOT_PERF

#include <extdll.h>
#include <meta_api.h>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <vector>

#include "bot_hitch.h"
#include "../game/cvars.h"
#include "bot_stall.h"

namespace
{
	using Clock = std::chrono::steady_clock;

	constexpr int kMaxLinesPerSecond = 4;
	constexpr double kStaleGapMs = 5000.0;

	cvar_t g_hitchCvar = { "crossbot_hitch", "50", FCVAR_SERVER };
	cvar_t g_hitchReportCvar = { "crossbot_hitch_report", "0", FCVAR_SERVER };
	cvar_t *g_hitch = nullptr;
	cvar_t *g_hitchReport = nullptr;

	const char *const kSectionNames[BotHitch::SECTION_COUNT] = { "pre", "bots", "brain" };

	Clock::time_point g_begin;
	Clock::time_point g_mark;
	Clock::time_point g_prevEnd;
	bool g_havePrev = false;

	int g_section = BotHitch::SECTION_PRE;
	double g_sectionMs[BotHitch::SECTION_COUNT] = {};
	Clock::time_point g_botMark;
	double g_worstBotMs = 0.0;
	char g_worstBot[32] = {};
	int g_botCount = 0;

	Clock::time_point g_lineWindow;
	int g_linesThisWindow = 0;

	Clock::time_point g_reportStart;
	std::vector<float> g_intervals;
	std::vector<float> g_selfs;
	int g_hitches = 0;

	double Ms(Clock::duration d)
	{
		return std::chrono::duration<double, std::milli>(d).count();
	}

	float Percentile(std::vector<float> &v, double p)
	{
		if (v.empty())
			return 0.f;
		size_t idx = static_cast<size_t>(p * (v.size() - 1) + 0.5);
		std::nth_element(v.begin(), v.begin() + idx, v.end());
		return v[idx];
	}

	void Close(Clock::time_point now)
	{
		g_sectionMs[g_section] += Ms(now - g_mark);
		g_mark = now;
	}

	void Report(Clock::time_point now)
	{
		double wall = Ms(now - g_reportStart) / 1000.0;
		float maxInterval = g_intervals.empty() ? 0.f : *std::max_element(g_intervals.begin(), g_intervals.end());
		float maxSelf = g_selfs.empty() ? 0.f : *std::max_element(g_selfs.begin(), g_selfs.end());
		int over17 = static_cast<int>(std::count_if(g_intervals.begin(), g_intervals.end(), [](float f) { return f > 16.7f; }));
		char line[384];
		snprintf(line, sizeof(line),
			"CrossBot HITCH report %.1fs: %d frames (%.1f fps) | interval p50 %.2f p99 %.2f max %.2f ms, %d over 16.7ms | "
			"crossbot p50 %.2f p99 %.2f max %.2f ms | hitches %d\n",
			wall, static_cast<int>(g_intervals.size()), wall > 0.0 ? g_intervals.size() / wall : 0.0,
			Percentile(g_intervals, 0.5), Percentile(g_intervals, 0.99), maxInterval, over17,
			Percentile(g_selfs, 0.5), Percentile(g_selfs, 0.99), maxSelf, g_hitches);
		SERVER_PRINT(line);
		g_intervals.clear();
		g_selfs.clear();
		g_hitches = 0;
		g_reportStart = now;
	}
}

namespace BotHitch
{

void Register()
{
	g_engfuncs.pfnCvar_RegisterVariable(&g_hitchCvar);
	g_engfuncs.pfnCvar_RegisterVariable(&g_hitchReportCvar);
	g_hitch = CBotCvar("crossbot_hitch");
	g_hitchReport = CBotCvar("crossbot_hitch_report");
	BotStall::Register();
}

void Reset()
{
	g_havePrev = false;
	g_intervals.clear();
	g_selfs.clear();
	g_hitches = 0;
	g_reportStart = Clock::now();
}

void FrameBegin()
{
	BotStall::Beat(true);
	g_begin = Clock::now();
	g_mark = g_begin;
	g_section = SECTION_PRE;
	for (double &ms : g_sectionMs)
		ms = 0.0;
	g_worstBotMs = 0.0;
	g_worstBot[0] = '\0';
	g_botCount = 0;
}

void Section(int section)
{
	Clock::time_point now = Clock::now();
	Close(now);
	g_section = section;
	g_botMark = now;
}

void BotDone(const char *name)
{
	Clock::time_point now = Clock::now();
	double ms = Ms(now - g_botMark);
	g_botMark = now;
	++g_botCount;
	if (ms > g_worstBotMs)
	{
		g_worstBotMs = ms;
		snprintf(g_worstBot, sizeof(g_worstBot), "%s", name ? name : "?");
	}
}

void FrameEnd()
{
	Clock::time_point end = Clock::now();
	Close(end);

	double self = Ms(end - g_begin);
	double outside = g_havePrev ? Ms(g_begin - g_prevEnd) : 0.0;
	double interval = g_havePrev ? Ms(end - g_prevEnd) : 0.0;
	bool stale = interval > kStaleGapMs;

	float threshold = g_hitch ? g_hitch->value : 0.f;
	float every = g_hitchReport ? g_hitchReport->value : 0.f;

	if (g_havePrev && !stale && every > 0.f)
	{
		g_intervals.push_back(static_cast<float>(interval));
		g_selfs.push_back(static_cast<float>(self));
	}

	if (threshold > 0.f && g_havePrev && !stale && (self >= threshold || outside >= threshold))
	{
		++g_hitches;
		if (Ms(end - g_lineWindow) >= 1000.0)
		{
			g_lineWindow = end;
			g_linesThisWindow = 0;
		}
		if (g_linesThisWindow++ < kMaxLinesPerSecond)
		{
			char line[384];
			snprintf(line, sizeof(line),
				"CrossBot HITCH %.1fms at t=%.2f: outside crossbot %.1fms, crossbot %.1fms (%s %.1f, %s %.1f, %s %.1f) | "
				"%d bots, worst %s %.1fms | host frametime %.1fms\n",
				interval, gpGlobals->time, outside, self,
				kSectionNames[SECTION_PRE], g_sectionMs[SECTION_PRE],
				kSectionNames[SECTION_BOTS], g_sectionMs[SECTION_BOTS],
				kSectionNames[SECTION_BRAIN], g_sectionMs[SECTION_BRAIN],
				g_botCount, g_worstBot[0] ? g_worstBot : "-", g_worstBotMs, gpGlobals->frametime * 1000.f);
			SERVER_PRINT(line);
		}
	}

	if (every > 0.f && Ms(end - g_reportStart) >= every * 1000.0)
		Report(end);
	else if (every <= 0.f)
		g_reportStart = end;

	BotStall::Flush();

	g_prevEnd = Clock::now();
	g_havePrev = true;
	BotStall::Beat(false);
}

}

#endif
