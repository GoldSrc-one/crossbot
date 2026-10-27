#include "nav_barrier.h"

#include <cstdio>
#include <cstring>

#include <extdll.h>
#include <meta_api.h>

namespace Nav
{

namespace
{

constexpr float kRescanInterval = 1.f;

const NavMesh *g_cachedMesh = nullptr;
float g_cachedTime = -1.f;
float g_nextScan = 0.f;
uint32_t g_generation = 0;

bool Matches(const edict_t *ent, const char *wanted, const Barrier &b)
{
	if (!ent || ent->free)
		return false;

	const char *model = STRING(ent->v.model);
	if (!model || std::strcmp(model, wanted) != 0)
		return false;

	return ent->v.absmin.x <= b.maxs.x && ent->v.absmax.x >= b.mins.x &&
		ent->v.absmin.y <= b.maxs.y && ent->v.absmax.y >= b.mins.y &&
		ent->v.absmin.z <= b.maxs.z && ent->v.absmax.z >= b.mins.z;
}

edict_t *Find(const char *wanted, const Barrier &b)
{
	for (int i = 1; i < gpGlobals->maxEntities; ++i)
	{
		edict_t *ent = g_engfuncs.pfnPEntityOfEntIndex(i);
		if (Matches(ent, wanted, b))
			return ent;
	}
	return nullptr;
}

void Refresh(const NavMesh &mesh)
{
	if (g_cachedMesh == &mesh && g_cachedTime == gpGlobals->time)
		return;

	bool changed = g_cachedMesh != &mesh;
	bool scan = changed || gpGlobals->time >= g_nextScan;
	bool scanned = false;

	for (const Barrier &b : mesh.barriers)
	{
		char wanted[16];
		snprintf(wanted, sizeof(wanted), "*%u", b.modelIndex);

		edict_t *ent = b.cachedEntIndex > 0 ? g_engfuncs.pfnPEntityOfEntIndex(b.cachedEntIndex) : nullptr;
		if (!Matches(ent, wanted, b))
		{
			ent = nullptr;
			if (scan)
			{
				ent = Find(wanted, b);
				b.cachedEntIndex = ent ? ENTINDEX(ent) : -1;
				scanned = true;
			}
		}

		bool closed = ent && ent->v.solid != SOLID_NOT;
		if (closed != b.closed)
		{
			b.closed = closed;
			changed = true;
		}
	}

	if (scanned)
		g_nextScan = gpGlobals->time + kRescanInterval;
	if (changed)
		++g_generation;

	g_cachedMesh = &mesh;
	g_cachedTime = gpGlobals->time;
}

}

bool BarrierClosed(const NavMesh &mesh, uint16_t barrier)
{
	if (barrier == crossbot_nav::kNoBarrier || barrier >= mesh.barriers.size())
		return false;

	Refresh(mesh);
	return mesh.barriers[barrier].closed;
}

uint32_t BarrierGeneration(const NavMesh &mesh)
{
	Refresh(mesh);
	return g_generation;
}

void ResetBarrierCache()
{
	g_cachedMesh = nullptr;
	g_cachedTime = -1.f;
	g_nextScan = 0.f;
}

}
