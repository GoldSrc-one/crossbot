#include "nav.h"

#include "nav_loader.h"

namespace Nav
{

namespace
{
std::unique_ptr<NavMesh> g_mesh;
}

void LoadForMap(const char *mapName)
{
	g_mesh = LoadNavFile(mapName);
}

void Unload()
{
	g_mesh.reset();
}

bool IsLoaded()
{
	return g_mesh != nullptr;
}

const NavMesh *Get()
{
	return g_mesh.get();
}

namespace
{
constexpr float kConnFailPenalty = 5.f;
constexpr float kConnPenaltyEpsilon = 0.1f;

Connection *ConnAt(uint32_t connIndex)
{
	if (!g_mesh || connIndex >= g_mesh->conns.size())
		return nullptr;
	return &g_mesh->conns[connIndex];
}
}

void PenalizeConnection(uint32_t connIndex)
{
	if (Connection *c = ConnAt(connIndex))
		c->runtimePenalty += kConnFailPenalty;
}

void RewardConnection(uint32_t connIndex)
{
	if (Connection *c = ConnAt(connIndex))
	{
		c->runtimePenalty *= 0.5f;
		if (c->runtimePenalty < kConnPenaltyEpsilon)
			c->runtimePenalty = 0.f;
	}
}

float ConnectionPenalty(uint32_t connIndex)
{
	const Connection *c = ConnAt(connIndex);
	return c ? c->runtimePenalty : 0.f;
}

}

