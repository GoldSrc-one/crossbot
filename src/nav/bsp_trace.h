#pragma once

#include "pm_sim.h"

namespace BspTrace
{

#ifdef CROSSBOT_BSPTRACE

enum Mode
{
	MODE_OFF = 0,
	MODE_COMPARE = 1,
	MODE_BSP = 2
};

void LoadForMap(const char *mapName);
void Clear();
bool Ready();

Mode CurrentMode();

PmSim::PmTraceFn Wrap(PmSim::PmTraceFn engine, int hull);

void ResetStats();
void PrintStats();

#else

inline void LoadForMap(const char *) {}
inline void Clear() {}

#endif

}

#ifdef CROSSBOT_BSPTRACE
	#define BSPTRACE_WRAP(fn, hull) BspTrace::Wrap((fn), (hull))
#else
	#define BSPTRACE_WRAP(fn, hull) (fn)
#endif
