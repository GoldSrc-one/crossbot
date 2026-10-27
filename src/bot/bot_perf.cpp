#include "bot_perf.h"
#include "../game/cvars.h"

#ifdef CROSSBOT_PERF

#include <extdll.h>
#include <meta_api.h>

#include <windows.h>

#include <algorithm>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <vector>

namespace
{
	const char *kZoneNames[BotPerf::ZONE_COUNT] = {
		"outside",
		"FRAME",
		"beams",
		"respawn",
		"weapons",
		"movenav",
		"findarea",
		"sense",
		"sn_vantage",
		"sn_damage",
		"sn_vision",
		"sn_bone",
		"sn_los",
		"sn_cov",
		"sn_enemy",
		"sn_look",
		"fire",
		"combat",
		"memos",
		"break",
		"deploy_try",
		"deploy_run",
		"item_goal",
		"item_select",
		"repath",
		"rp_plan",
		"rp_profile",
		"rp_log",
		"plantraces",
		"pushfields",
		"follow",
		"avoid",
		"oppo",
		"look",
		"drive",
		"runmove",
		"tracelog",
	};

	struct ZoneAcc
	{
		long long incl = 0;
		long long self = 0;
		long long trace = 0;
		int calls = 0;
		int traces = 0;
		int byType[BotPerf::TRACE_TYPE_COUNT] = {};
		int monsterTraces = 0;
		double traceLen = 0.0;
	};

	const char *kTraceTypeNames[BotPerf::TRACE_TYPE_COUNT] = { "line", "hull", "model", "mhull" };

	struct StackEntry
	{
		int zone = 0;
		long long start = 0;
		long long child = 0;
	};

	struct Row
	{
		int zone;
		double self;
	};

	bool RowGreater(const Row &a, const Row &b)
	{
		return a.self > b.self;
	}

	double g_msPerTick = 0.0;

	ZoneAcc g_frame[BotPerf::ZONE_COUNT];
	ZoneAcc g_window[BotPerf::ZONE_COUNT];
	double g_zoneFrameMax[BotPerf::ZONE_COUNT] = {};

	ZoneAcc g_worst[BotPerf::ZONE_COUNT];
	double g_worstMs = 0.0;
	char g_worstBot[32] = {};
	double g_worstBotMs = 0.0;

	StackEntry g_stack[24];
	int g_depth = 0;

	long long g_traceStart = 0;
	int g_traceZone = 0;
	int g_traceDepth = 0;
	int g_traceType = 0;
	double g_traceLen = 0.0;
	bool g_traceMonsters = false;

	const char *g_curBot = "";
	long long g_botStart = 0;
	char g_frameWorstBot[32] = {};
	double g_frameWorstBotMs = 0.0;

	std::vector<double> g_frameMs;

	double g_windowStart = 0.0;
	int g_spikesLogged = 0;
	int g_spikeCount = 0;

	bool g_active = false;
	float g_reportInterval = 5.f;
	float g_spikeMs = 5.f;

	long long Now()
	{
		LARGE_INTEGER li;
		QueryPerformanceCounter(&li);
		return li.QuadPart;
	}

	double Ms(long long ticks)
	{
		return static_cast<double>(ticks) * g_msPerTick;
	}

	double RealSeconds()
	{
		return g_engfuncs.pfnTime();
	}

	void ResetWindow()
	{
		for (int i = 0; i < BotPerf::ZONE_COUNT; ++i)
		{
			g_window[i] = ZoneAcc();
			g_zoneFrameMax[i] = 0.0;
			g_worst[i] = ZoneAcc();
		}
		g_frameMs.clear();
		g_worstMs = 0.0;
		g_worstBot[0] = '\0';
		g_worstBotMs = 0.0;
		g_spikesLogged = 0;
		g_spikeCount = 0;
		g_windowStart = RealSeconds();
	}

	double Percentile(const std::vector<double> &sorted, double p)
	{
		if (sorted.empty())
			return 0.0;
		size_t idx = static_cast<size_t>(p * (sorted.size() - 1) + 0.5);
		if (idx >= sorted.size())
			idx = sorted.size() - 1;
		return sorted[idx];
	}

	void CollectRows(const ZoneAcc *src, std::vector<Row> &rows, bool skipFrame)
	{
		rows.clear();
		for (int i = 0; i < BotPerf::ZONE_COUNT; ++i)
		{
			if (src[i].calls == 0)
				continue;
			if (skipFrame && i == BotPerf::ZONE_FRAME)
				continue;
			Row r;
			r.zone = i;
			r.self = Ms(src[i].self);
			rows.push_back(r);
		}
		std::sort(rows.begin(), rows.end(), RowGreater);
	}

	void Report()
	{
		double wall = RealSeconds() - g_windowStart;
		std::vector<double> sorted = g_frameMs;
		std::sort(sorted.begin(), sorted.end());

		double total = 0.0;
		for (size_t i = 0; i < g_frameMs.size(); ++i)
			total += g_frameMs[i];

		char msg[512];
		snprintf(msg, sizeof(msg),
			"CrossBot PERF window %.1fs frames %d | crossbot %.0fms total (%.1f%% of wall) | "
			"frame avg %.3f p50 %.3f p90 %.3f p99 %.3f max %.3f ms | spikes>=%.1fms %d\n",
			wall, static_cast<int>(g_frameMs.size()), total,
			wall > 0.0 ? total / (wall * 1000.0) * 100.0 : 0.0,
			g_frameMs.empty() ? 0.0 : total / g_frameMs.size(),
			Percentile(sorted, 0.50), Percentile(sorted, 0.90), Percentile(sorted, 0.99),
			sorted.empty() ? 0.0 : sorted.back(), g_spikeMs, g_spikeCount);
		SERVER_PRINT(msg);

		double frames = g_frameMs.empty() ? 1.0 : static_cast<double>(g_frameMs.size());

		std::vector<Row> rows;
		CollectRows(g_window, rows, false);

		for (size_t i = 0; i < rows.size(); ++i)
		{
			const ZoneAcc &z = g_window[rows[i].zone];
			snprintf(msg, sizeof(msg),
				"CrossBot PERF   %-11s self %8.1fms %5.1f%% | incl %8.1fms | %6.2f calls/f | "
				"self %.4fms/f | worst frame %.3fms | traces %.2f/f %.1fms\n",
				kZoneNames[rows[i].zone], rows[i].self,
				total > 0.0 ? rows[i].self / total * 100.0 : 0.0, Ms(z.incl),
				z.calls / frames, rows[i].self / frames,
				g_zoneFrameMax[rows[i].zone],
				z.traces / frames, Ms(z.trace));
			SERVER_PRINT(msg);
		}

		long long allTraceTicks = 0;
		int allTraces = 0;
		int allByType[BotPerf::TRACE_TYPE_COUNT] = {};
		for (int i = 0; i < BotPerf::ZONE_COUNT; ++i)
		{
			allTraceTicks += g_window[i].trace;
			allTraces += g_window[i].traces;
			for (int t = 0; t < BotPerf::TRACE_TYPE_COUNT; ++t)
				allByType[t] += g_window[i].byType[t];
		}

		snprintf(msg, sizeof(msg),
			"CrossBot PERF traces %d total %.0fms (%.1f%% of crossbot) | %.1f/frame %.4fms each | "
			"line %d hull %d model %d mhull %d\n",
			allTraces, Ms(allTraceTicks),
			total > 0.0 ? Ms(allTraceTicks) / total * 100.0 : 0.0,
			allTraces / frames, allTraces ? Ms(allTraceTicks) / allTraces : 0.0,
			allByType[0], allByType[1], allByType[2], allByType[3]);
		SERVER_PRINT(msg);

		std::vector<Row> trows;
		trows.clear();
		for (int i = 0; i < BotPerf::ZONE_COUNT; ++i)
		{
			if (g_window[i].traces == 0)
				continue;
			Row r;
			r.zone = i;
			r.self = Ms(g_window[i].trace);
			trows.push_back(r);
		}
		std::sort(trows.begin(), trows.end(), RowGreater);

		for (size_t i = 0; i < trows.size(); ++i)
		{
			const ZoneAcc &z = g_window[trows[i].zone];
			snprintf(msg, sizeof(msg),
				"CrossBot PERF  T %-11s %8.1fms %5.1f%% of trace | %9.2f/f | %.4fms each | "
				"mean %.0fu | vs ents %.0f%% | line %d hull %d model %d mhull %d\n",
				kZoneNames[trows[i].zone], trows[i].self,
				allTraceTicks > 0 ? trows[i].self / Ms(allTraceTicks) * 100.0 : 0.0,
				z.traces / frames, z.traces ? trows[i].self / z.traces : 0.0,
				z.traces ? z.traceLen / z.traces : 0.0,
				z.traces ? 100.0 * z.monsterTraces / z.traces : 0.0,
				z.byType[0], z.byType[1], z.byType[2], z.byType[3]);
			SERVER_PRINT(msg);
		}

		if (g_worstMs > 0.0)
		{
			snprintf(msg, sizeof(msg),
				"CrossBot PERF worst frame %.3fms (worst single bot %s %.3fms):\n",
				g_worstMs, g_worstBot[0] ? g_worstBot : "-", g_worstBotMs);
			SERVER_PRINT(msg);

			std::vector<Row> wrows;
			CollectRows(g_worst, wrows, true);

			for (size_t i = 0; i < wrows.size() && i < 10; ++i)
			{
				const ZoneAcc &z = g_worst[wrows[i].zone];
				snprintf(msg, sizeof(msg),
					"CrossBot PERF     %-11s self %7.3fms incl %7.3fms calls %d traces %d %.3fms\n",
					kZoneNames[wrows[i].zone], wrows[i].self, Ms(z.incl), z.calls, z.traces,
					Ms(z.trace));
				SERVER_PRINT(msg);
			}
		}
	}
}

namespace
{
	void (*g_realTraceLine)(const float *, const float *, int, edict_t *, TraceResult *) = nullptr;
	void (*g_realTraceHull)(const float *, const float *, int, int, edict_t *, TraceResult *) = nullptr;
	void (*g_realTraceModel)(const float *, const float *, int, edict_t *, TraceResult *) = nullptr;
	int (*g_realTraceMonsterHull)(edict_t *, const float *, const float *, int, edict_t *,
		TraceResult *) = nullptr;

	void WrapTraceLine(const float *v1, const float *v2, int fNoMonsters, edict_t *pentToSkip,
		TraceResult *ptr)
	{
		BotPerf::TraceBegin(BotPerf::TRACE_LINE_T, v1, v2, fNoMonsters);
		g_realTraceLine(v1, v2, fNoMonsters, pentToSkip, ptr);
		BotPerf::TraceEnd();
	}

	void WrapTraceHull(const float *v1, const float *v2, int fNoMonsters, int hullNumber,
		edict_t *pentToSkip, TraceResult *ptr)
	{
		BotPerf::TraceBegin(BotPerf::TRACE_HULL_T, v1, v2, fNoMonsters);
		g_realTraceHull(v1, v2, fNoMonsters, hullNumber, pentToSkip, ptr);
		BotPerf::TraceEnd();
	}

	void WrapTraceModel(const float *v1, const float *v2, int hullNumber, edict_t *pent,
		TraceResult *ptr)
	{
		BotPerf::TraceBegin(BotPerf::TRACE_MODEL_T, v1, v2, 1);
		g_realTraceModel(v1, v2, hullNumber, pent, ptr);
		BotPerf::TraceEnd();
	}

	int WrapTraceMonsterHull(edict_t *pEdict, const float *v1, const float *v2, int fNoMonsters,
		edict_t *pentToSkip, TraceResult *ptr)
	{
		BotPerf::TraceBegin(BotPerf::TRACE_MONSTER_T, v1, v2, fNoMonsters);
		int r = g_realTraceMonsterHull(pEdict, v1, v2, fNoMonsters, pentToSkip, ptr);
		BotPerf::TraceEnd();
		return r;
	}
}

namespace BotPerf
{

void InstallTraceWrappers()
{
	if (g_realTraceLine)
		return;

	g_realTraceLine = g_engfuncs.pfnTraceLine;
	g_realTraceHull = g_engfuncs.pfnTraceHull;
	g_realTraceModel = g_engfuncs.pfnTraceModel;
	g_realTraceMonsterHull = g_engfuncs.pfnTraceMonsterHull;

	g_engfuncs.pfnTraceLine = WrapTraceLine;
	g_engfuncs.pfnTraceHull = WrapTraceHull;
	g_engfuncs.pfnTraceModel = WrapTraceModel;
	g_engfuncs.pfnTraceMonsterHull = WrapTraceMonsterHull;
}

bool Enabled()
{
	return g_active;
}

void Reset()
{
	g_depth = 0;
	g_traceDepth = 0;
	for (int i = 0; i < ZONE_COUNT; ++i)
		g_frame[i] = ZoneAcc();
	ResetWindow();
}

void Enter(Zone z)
{
	if (!g_active || g_depth >= 24)
		return;
	StackEntry &e = g_stack[g_depth++];
	e.zone = z;
	e.start = Now();
	e.child = 0;
}

void Leave(Zone z)
{
	if (!g_active || g_depth <= 0)
		return;
	StackEntry &e = g_stack[--g_depth];
	if (e.zone != static_cast<int>(z))
	{
		g_depth = 0;
		return;
	}

	long long elapsed = Now() - e.start;
	ZoneAcc &acc = g_frame[z];
	acc.incl += elapsed;
	acc.self += elapsed - e.child;
	++acc.calls;

	if (g_depth > 0)
		g_stack[g_depth - 1].child += elapsed;
}

void NoteBot(const char *name)
{
	if (!g_active)
		return;

	long long now = Now();
	if (g_curBot && g_curBot[0])
	{
		double ms = Ms(now - g_botStart);
		if (ms > g_frameWorstBotMs)
		{
			g_frameWorstBotMs = ms;
			snprintf(g_frameWorstBot, sizeof(g_frameWorstBot), "%s", g_curBot);
		}
	}
	g_curBot = name ? name : "";
	g_botStart = now;
}

void TraceBegin(TraceType type, const float *v1, const float *v2, int fNoMonsters)
{
	if (!g_active)
		return;
	if (g_traceDepth++ != 0)
		return;
	g_traceZone = g_depth > 0 ? g_stack[g_depth - 1].zone : ZONE_OUTSIDE;
	g_traceType = type;
	g_traceMonsters = fNoMonsters == 0;

	g_traceLen = 0.0;
	if (v1 && v2)
	{
		double dx = v2[0] - v1[0];
		double dy = v2[1] - v1[1];
		double dz = v2[2] - v1[2];
		g_traceLen = sqrt(dx * dx + dy * dy + dz * dz);
	}

	g_traceStart = Now();
}

void TraceEnd()
{
	if (!g_active)
		return;
	if (--g_traceDepth != 0)
	{
		if (g_traceDepth < 0)
			g_traceDepth = 0;
		return;
	}
	ZoneAcc &acc = g_frame[g_traceZone];
	acc.trace += Now() - g_traceStart;
	++acc.traces;
	++acc.byType[g_traceType];
	acc.traceLen += g_traceLen;
	if (g_traceMonsters)
		++acc.monsterTraces;
}

void FrameBegin()
{
	cvar_t *cvar = CBotCvar("crossbot_perf");
	float want = cvar ? cvar->value : 0.f;

	if (want <= 0.f)
	{
		if (g_active)
		{
			g_active = false;
			g_depth = 0;
			g_traceDepth = 0;
		}
		return;
	}

	if (g_msPerTick == 0.0)
	{
		LARGE_INTEGER f;
		QueryPerformanceFrequency(&f);
		g_msPerTick = 1000.0 / static_cast<double>(f.QuadPart);
	}

	if (!g_active)
	{
		g_active = true;
		Reset();
	}

	g_reportInterval = want;

	cvar_t *spike = CBotCvar("crossbot_perf_spike");
	g_spikeMs = spike ? spike->value : 5.f;

	for (int i = 0; i < ZONE_COUNT; ++i)
		g_frame[i] = ZoneAcc();
	g_depth = 0;
	g_traceDepth = 0;
	g_frameWorstBot[0] = '\0';
	g_frameWorstBotMs = 0.0;
	g_curBot = "";
	g_botStart = Now();

	Enter(ZONE_FRAME);
}

void FrameEnd()
{
	if (!g_active)
		return;

	NoteBot("");
	Leave(ZONE_FRAME);

	double frameMs = Ms(g_frame[ZONE_FRAME].incl);
	g_frameMs.push_back(frameMs);

	int frameTraces = 0;
	long long frameTraceTicks = 0;

	for (int i = 0; i < ZONE_COUNT; ++i)
	{
		g_window[i].incl += g_frame[i].incl;
		g_window[i].self += g_frame[i].self;
		g_window[i].trace += g_frame[i].trace;
		g_window[i].calls += g_frame[i].calls;
		g_window[i].traces += g_frame[i].traces;
		g_window[i].monsterTraces += g_frame[i].monsterTraces;
		g_window[i].traceLen += g_frame[i].traceLen;
		for (int t = 0; t < TRACE_TYPE_COUNT; ++t)
			g_window[i].byType[t] += g_frame[i].byType[t];

		frameTraces += g_frame[i].traces;
		frameTraceTicks += g_frame[i].trace;

		double selfMs = Ms(g_frame[i].self);
		if (selfMs > g_zoneFrameMax[i])
			g_zoneFrameMax[i] = selfMs;
	}

	if (frameMs > g_worstMs)
	{
		g_worstMs = frameMs;
		memcpy(g_worst, g_frame, sizeof(g_worst));
		g_worstBotMs = g_frameWorstBotMs;
		snprintf(g_worstBot, sizeof(g_worstBot), "%s", g_frameWorstBot);
	}

	if (g_spikeMs > 0.f && frameMs >= g_spikeMs)
	{
		++g_spikeCount;
		if (g_spikesLogged < 8)
		{
			++g_spikesLogged;

			std::vector<Row> rows;
			CollectRows(g_frame, rows, true);

			char detail[320] = {};
			for (size_t i = 0; i < rows.size() && i < 4; ++i)
			{
				if (rows[i].self <= 0.0)
					break;
				size_t len = strlen(detail);
				snprintf(detail + len, sizeof(detail) - len, " %s=%.2f/%d",
					kZoneNames[rows[i].zone], rows[i].self, g_frame[rows[i].zone].calls);
			}

			char msg[448];
			snprintf(msg, sizeof(msg),
				"CrossBot PERF SPIKE %.2fms (host frame %.2fms) traces %d/%.2fms | "
				"worst bot %s %.2fms |%s\n",
				frameMs, gpGlobals->frametime * 1000.f, frameTraces, Ms(frameTraceTicks),
				g_frameWorstBot[0] ? g_frameWorstBot : "-", g_frameWorstBotMs, detail);
			SERVER_PRINT(msg);
		}
	}

	if (RealSeconds() - g_windowStart >= g_reportInterval)
	{
		Report();
		ResetWindow();
	}
}

void Status()
{
	if (!g_active)
	{
		SERVER_PRINT("CrossBot PERF: disabled (set crossbot_perf <report seconds>)\n");
		return;
	}
	Report();
}

}

#endif
