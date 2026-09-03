#include "bsp_trace.h"

#ifdef CROSSBOT_BSPTRACE

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <vector>

#include <extdll.h>
#include <meta_api.h>

#include "bsp_file.h"
#include "clipnode_tree.h"
#include "blocking_entities.h"
#include "hull_defs.h"

namespace BspTrace
{

namespace
{

std::unique_ptr<navgen::BspFile> g_bsp;
int g_headnode[4] = {-1, -1, -1, -1};
std::vector<navgen::BlockingVolume> g_noBlockers;
std::vector<navgen::BlockingVolume> g_blockers[4];
float g_blockersBuilt = -1.f;

int BrushModelIndex(const char *model)
{
	if (!model || model[0] != '*')
		return -1;
	int n = atoi(model + 1);
	return n > 0 ? n : -1;
}

void RebuildBlockers()
{
	if (!g_bsp)
		return;
	if (g_blockersBuilt == gpGlobals->time)
		return;
	g_blockersBuilt = gpGlobals->time;

	for (int h = 0; h < 4; ++h)
		g_blockers[h].clear();

	const std::vector<dmodel_t> &models = g_bsp->Models();

	for (int i = gpGlobals->maxClients + 1; i < gpGlobals->maxEntities; ++i)
	{
		edict_t *ent = g_engfuncs.pfnPEntityOfEntIndex(i);
		if (!ent || ent->free)
			continue;
		if (ent->v.solid != SOLID_BSP)
			continue;
		if ((ent->v.flags & FL_WORLDBRUSH) != 0)
			continue;

		int mi = BrushModelIndex(STRING(ent->v.model));
		if (mi < 0 || static_cast<size_t>(mi) >= models.size())
			continue;

		const dmodel_t &model = models[static_cast<size_t>(mi)];

		for (int h = 0; h < 4; ++h)
		{
			if (model.headnode[h] < 0)
				continue;
			navgen::BlockingVolume vol;
			vol.headnode = model.headnode[h];
			vol.origin = navgen::Vec3(ent->v.origin.x, ent->v.origin.y, ent->v.origin.z);
			vol.mins = navgen::Vec3(model.mins[0], model.mins[1], model.mins[2]) + vol.origin;
			vol.maxs = navgen::Vec3(model.maxs[0], model.maxs[1], model.maxs[2]) + vol.origin;
			vol.entityIndex = i;
			g_blockers[h].push_back(vol);
		}
	}
}

bool SegmentHitsBox(const Vector &start, const Vector &end, const navgen::Vec3 &mins,
	const navgen::Vec3 &maxs, float pad)
{
	float lo = 0.f;
	float hi = 1.f;
	for (int axis = 0; axis < 3; ++axis)
	{
		float s = axis == 0 ? start.x : (axis == 1 ? start.y : start.z);
		float e = axis == 0 ? end.x : (axis == 1 ? end.y : end.z);
		float bmin = static_cast<float>(axis == 0 ? mins.x : (axis == 1 ? mins.y : mins.z)) - pad;
		float bmax = static_cast<float>(axis == 0 ? maxs.x : (axis == 1 ? maxs.y : maxs.z)) + pad;
		float d = e - s;
		if (std::fabs(d) < 1e-6f)
		{
			if (s < bmin || s > bmax)
				return false;
			continue;
		}
		float t0 = (bmin - s) / d;
		float t1 = (bmax - s) / d;
		if (t0 > t1)
		{
			float tmp = t0;
			t0 = t1;
			t1 = tmp;
		}
		if (t0 > lo)
			lo = t0;
		if (t1 < hi)
			hi = t1;
		if (lo > hi)
			return false;
	}
	return true;
}

struct Side
{
	double ticks = 0.0;
	unsigned long long count = 0;
};

struct Stats
{
	Side engine;
	Side bsp;
	unsigned long long compared = 0;
	unsigned long long fracMismatch = 0;
	unsigned long long endMismatch = 0;
	unsigned long long solidMismatch = 0;
	double worstFracDelta = 0.0;
	double worstEndDelta = 0.0;
	double timerTicks = 0.0;
	unsigned long long timerCount = 0;
};

Stats g_stats;

double g_ticksPerMs = 0.0;

double Now()
{
	LARGE_INTEGER li;
	QueryPerformanceCounter(&li);
	return static_cast<double>(li.QuadPart);
}

void InitFrequency()
{
	if (g_ticksPerMs > 0.0)
		return;
	LARGE_INTEGER f;
	QueryPerformanceFrequency(&f);
	g_ticksPerMs = static_cast<double>(f.QuadPart) / 1000.0;
}

double Us(double ticks, unsigned long long count)
{
	if (count == 0 || g_ticksPerMs <= 0.0)
		return 0.0;
	return (ticks / count) / g_ticksPerMs * 1000.0;
}

PmSim::PmTrace RunBsp(int headnode, int hull, bool withBlockers, const Vector &start,
	const Vector &end)
{
	navgen::Vec3 s(start.x, start.y, start.z);
	navgen::Vec3 e(end.x, end.y, end.z);

	navgen::TraceResult tr = navgen::TraceHull(*g_bsp, headnode, s, e);

	if (withBlockers && tr.fraction > 0.f)
	{
		const std::vector<navgen::BlockingVolume> &list = g_blockers[hull];
		for (size_t i = 0; i < list.size(); ++i)
		{
			const navgen::BlockingVolume &b = list[i];
			if (!SegmentHitsBox(start, end, b.mins, b.maxs, 64.f))
				continue;
			navgen::TraceResult bt = navgen::TraceHull(*g_bsp, b.headnode, s - b.origin,
				e - b.origin);
			if (bt.fraction < tr.fraction)
			{
				tr = bt;
				tr.ent = 1;
				if (bt.fraction < 1.f)
					tr.endpos = bt.endpos + b.origin;
			}
		}
	}

	PmSim::PmTrace out;
	out.endpos = Vector(static_cast<float>(tr.endpos.x), static_cast<float>(tr.endpos.y),
		static_cast<float>(tr.endpos.z));
	out.planeNormal = Vector(static_cast<float>(tr.planeNormal.x),
		static_cast<float>(tr.planeNormal.y), static_cast<float>(tr.planeNormal.z));
	out.fraction = tr.fraction;
	out.allsolid = tr.allsolid;
	out.startsolid = tr.startsolid;
	out.hitEntity = false;
	return out;
}

}

void Clear()
{
	g_bsp.reset();
	for (int i = 0; i < 4; ++i)
		g_headnode[i] = -1;
	ResetStats();
}

void LoadForMap(const char *mapName)
{
	Clear();
	InitFrequency();

	char gamedir[256] = {0};
	g_engfuncs.pfnGetGameDir(gamedir);

	char bspPath[512];
	snprintf(bspPath, sizeof(bspPath), "%s/maps/%s.bsp", gamedir, mapName);

	try
	{
		g_bsp.reset(new navgen::BspFile(bspPath));
	}
	catch (...)
	{
		g_bsp.reset();
		char msg[600];
		snprintf(msg, sizeof(msg), "CrossBot: bsp trace disabled, could not read %s\n", bspPath);
		SERVER_PRINT(msg);
		return;
	}

	if (g_bsp->Models().empty())
	{
		g_bsp.reset();
		return;
	}

	const dmodel_t &world = g_bsp->Models()[0];
	for (int i = 0; i < 4; ++i)
		g_headnode[i] = world.headnode[i];
}

bool Ready()
{
	return g_bsp != nullptr;
}

Mode CurrentMode()
{
	if (!Ready())
		return MODE_OFF;
	float v = g_engfuncs.pfnCVarGetFloat("crossbot_bsptrace");
	if (v >= 2.f)
		return MODE_BSP;
	if (v >= 1.f)
		return MODE_COMPARE;
	return MODE_OFF;
}

PmSim::PmTraceFn Wrap(PmSim::PmTraceFn engine, int hull)
{
	Mode mode = CurrentMode();
	if (mode == MODE_OFF || hull < 0 || hull > 3 || g_headnode[hull] < 0)
		return engine;

	int headnode = g_headnode[hull];

	bool withBlockers = g_engfuncs.pfnCVarGetFloat("crossbot_bsptrace_blockers") > 0.f;
	if (withBlockers)
		RebuildBlockers();

	if (mode == MODE_BSP)
	{
		return [headnode, hull, withBlockers](const Vector &start, const Vector &end)
		{
			return RunBsp(headnode, hull, withBlockers, start, end);
		};
	}

	return [engine, headnode, hull, withBlockers](const Vector &start, const Vector &end)
	{
		double t0 = Now();
		PmSim::PmTrace a = engine(start, end);
		double t1 = Now();
		PmSim::PmTrace b = RunBsp(headnode, hull, withBlockers, start, end);
		double t2 = Now();

		double t3 = Now();

		g_stats.engine.ticks += t1 - t0;
		++g_stats.engine.count;
		g_stats.bsp.ticks += t2 - t1;
		++g_stats.bsp.count;
		g_stats.timerTicks += t3 - t2;
		++g_stats.timerCount;
		++g_stats.compared;

		double fd = std::fabs(static_cast<double>(a.fraction) - static_cast<double>(b.fraction));
		if (fd > 0.001)
		{
			++g_stats.fracMismatch;
			if (fd > g_stats.worstFracDelta)
				g_stats.worstFracDelta = fd;
		}

		double ed = (a.endpos - b.endpos).Length();
		if (ed > 0.5)
		{
			++g_stats.endMismatch;
			if (ed > g_stats.worstEndDelta)
				g_stats.worstEndDelta = ed;
		}

		if (a.startsolid != b.startsolid || a.allsolid != b.allsolid)
			++g_stats.solidMismatch;

		return a;
	};
}

void ResetStats()
{
	g_stats = Stats();
}

void PrintStats()
{
	char msg[640];

	if (!Ready())
	{
		SERVER_PRINT("CrossBot: bsp trace not loaded for this map\n");
		return;
	}

	if (g_stats.compared == 0)
	{
		snprintf(msg, sizeof(msg),
			"CrossBot: bsp trace ready, no comparisons yet (set crossbot_bsptrace 1)\n");
		SERVER_PRINT(msg);
		return;
	}

	double engineUs = Us(g_stats.engine.ticks, g_stats.engine.count);
	double bspUs = Us(g_stats.bsp.ticks, g_stats.bsp.count);
	double timerUs = Us(g_stats.timerTicks, g_stats.timerCount);

	snprintf(msg, sizeof(msg),
		"CrossBot BSPTRACE %llu traces | engine %.3fus | bsp %.3fus | timer overhead %.3fus | "
		"blockers %s (%zu stand / %zu duck)\n",
		g_stats.compared, engineUs, bspUs, timerUs,
		g_engfuncs.pfnCVarGetFloat("crossbot_bsptrace_blockers") > 0.f ? "on" : "off",
		g_blockers[1].size(), g_blockers[3].size());
	SERVER_PRINT(msg);

	double engineNet = engineUs - timerUs;
	double bspNet = bspUs - timerUs;
	snprintf(msg, sizeof(msg),
		"CrossBot BSPTRACE net of timer: engine %.3fus | bsp %.3fus | speedup %.2fx\n",
		engineNet, bspNet, bspNet > 0.0 ? engineNet / bspNet : 0.0);
	SERVER_PRINT(msg);

	snprintf(msg, sizeof(msg),
		"CrossBot BSPTRACE disagree: fraction %llu (%.3f%%, worst %.4f) | endpos %llu (%.3f%%, worst %.2fu) | solid %llu\n",
		g_stats.fracMismatch, 100.0 * g_stats.fracMismatch / g_stats.compared, g_stats.worstFracDelta,
		g_stats.endMismatch, 100.0 * g_stats.endMismatch / g_stats.compared, g_stats.worstEndDelta,
		g_stats.solidMismatch);
	SERVER_PRINT(msg);
}

}

#endif
