#include "nav_water.h"

#include <cstdio>
#include <cstring>

#include <extdll.h>
#include <meta_api.h>

namespace Nav
{

namespace
{

std::vector<float> g_surfaces;
const NavMesh *g_cachedMesh = nullptr;
float g_cachedTime = -1.f;

edict_t *FindBrushModelEntity(uint32_t modelIndex)
{
	char wanted[16];
	snprintf(wanted, sizeof(wanted), "*%u", modelIndex);

	for (int i = 1; i < gpGlobals->maxEntities; ++i)
	{
		edict_t *ent = g_engfuncs.pfnPEntityOfEntIndex(i);
		if (!ent || ent->free)
			continue;

		const char *model = STRING(ent->v.model);
		if (model && std::strcmp(model, wanted) == 0)
			return ent;
	}

	return nullptr;
}

void Refresh(const NavMesh &mesh)
{
	if (g_cachedMesh == &mesh && g_cachedTime == gpGlobals->time)
		return;

	g_surfaces.assign(mesh.waterVolumes.size(), 0.f);

	for (size_t i = 0; i < mesh.waterVolumes.size(); ++i)
	{
		const WaterVolume &vol = mesh.waterVolumes[i];
		g_surfaces[i] = vol.surfaceZ;

		if (!vol.moving || vol.modelIndex == 0)
			continue;

		if (!vol.resolved)
		{
			edict_t *ent = FindBrushModelEntity(vol.modelIndex);
			vol.resolved = true;
			vol.cachedEntIndex = ent ? ENTINDEX(ent) : -1;
			vol.spawnOriginZ = ent ? ent->v.origin.z : 0.f;
		}

		if (vol.cachedEntIndex <= 0)
			continue;

		edict_t *ent = g_engfuncs.pfnPEntityOfEntIndex(vol.cachedEntIndex);
		if (!ent || ent->free)
			continue;

		g_surfaces[i] = vol.surfaceZ + (ent->v.origin.z - vol.spawnOriginZ);
	}

	g_cachedMesh = &mesh;
	g_cachedTime = gpGlobals->time;
}

}

const std::vector<float> &WaterSurfaces(const NavMesh &mesh)
{
	Refresh(mesh);
	return g_surfaces;
}

float WaterSurfaceZ(const NavMesh &mesh, uint8_t volumeIndex)
{
	if (volumeIndex == crossbot_nav::kNoWaterVolume)
		return -crossbot_nav::kNavSubmergeNever;

	Refresh(mesh);
	if (volumeIndex >= g_surfaces.size())
		return -crossbot_nav::kNavSubmergeNever;

	return g_surfaces[volumeIndex];
}

bool Submerged(const NavMesh &mesh, uint8_t volumeIndex, float submergeZ)
{
	if (volumeIndex == crossbot_nav::kNoWaterVolume)
		return false;

	return WaterSurfaceZ(mesh, volumeIndex) >= submergeZ;
}

void ResetWaterCache()
{
	g_surfaces.clear();
	g_cachedMesh = nullptr;
	g_cachedTime = -1.f;
}

}
