#include "nav.h"

#include <algorithm>
#include <cmath>

#include "nav_loader.h"
#include "nav_water.h"

namespace Nav
{

namespace
{
std::unique_ptr<NavMesh> g_mesh;
std::vector<int> g_addedFree;
std::vector<int> g_addedUsed;
}

void LoadForMap(const char *mapName)
{
	ResetWaterCache();
	g_addedFree.clear();
	g_addedUsed.clear();
	g_mesh = LoadNavFile(mapName);
}

void Unload()
{
	ResetWaterCache();
	g_addedFree.clear();
	g_addedUsed.clear();
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

namespace
{
constexpr float kPadStepSeconds = 0.5f;
constexpr float kActCenterMatch = 2.f;


void Unlist(std::vector<uint32_t> &list, uint32_t idx)
{
	list.erase(std::remove(list.begin(), list.end(), idx), list.end());
}
}

int AddTeleport(const Vector &launch, uint32_t fromArea, const Vector &landing, uint32_t toArea,
	float waitSeconds, uint32_t teamMask)
{
	if (!g_mesh || fromArea >= g_mesh->areas.size() || toArea >= g_mesh->areas.size() || fromArea == toArea)
		return -1;

	int idx;
	if (!g_addedFree.empty())
	{
		idx = g_addedFree.back();
		g_addedFree.pop_back();
	}
	else
	{
		idx = static_cast<int>(g_mesh->conns.size());
		g_mesh->conns.emplace_back();
		g_mesh->planConns.emplace_back();
	}
	g_addedUsed.push_back(idx);

	Connection &c = g_mesh->conns[idx];
	c = Connection();
	c.areaA = fromArea;
	c.areaB = toArea;
	c.connType = crossbot_nav::CONN_TELEPORT;
	c.connFlags = crossbot_nav::CONN_FLAG_ONE_WAY;
	c.travelSeconds = kPadStepSeconds;
	c.costSeconds = waitSeconds;
	c.teamMask = teamMask;
	c.launchPos = launch;
	c.landingPos = landing;

	NavPlan::PlanConnection &pc = g_mesh->planConns[idx];
	pc = NavPlan::PlanConnection();
	pc.areaA = fromArea;
	pc.areaB = toArea;
	pc.connType = c.connType;
	pc.connFlags = c.connFlags;
	pc.launchPos = launch;
	pc.landingPos = landing;

	g_mesh->areas[fromArea].connIndices.push_back(static_cast<uint32_t>(idx));
	g_mesh->areas[toArea].connIndices.push_back(static_cast<uint32_t>(idx));
	return idx;
}

void RemoveTeleport(int connIndex)
{
	auto used = std::find(g_addedUsed.begin(), g_addedUsed.end(), connIndex);
	if (!g_mesh || used == g_addedUsed.end())
		return;
	g_addedUsed.erase(used);
	g_addedFree.push_back(connIndex);

	Connection &c = g_mesh->conns[connIndex];
	Unlist(g_mesh->areas[c.areaA].connIndices, static_cast<uint32_t>(connIndex));
	Unlist(g_mesh->areas[c.areaB].connIndices, static_cast<uint32_t>(connIndex));
	c.teamMask = ~0u;
}

void SetTeleportWait(int connIndex, float waitSeconds)
{
	if (g_mesh && IsAddedTeleport(connIndex))
		g_mesh->conns[connIndex].costSeconds = waitSeconds;
}

bool IsAddedTeleport(int connIndex)
{
	return std::find(g_addedUsed.begin(), g_addedUsed.end(), connIndex) != g_addedUsed.end();
}

void SetDetourTeams(const Vector &actCenter, uint32_t teamMask)
{
	if (!g_mesh)
		return;

	for (NavPlan::PlanArea &area : g_mesh->planAreas)
		for (NavPlan::PlanDetour &d : area.detours)
			if (fabsf(d.actCenter.x - actCenter.x) <= kActCenterMatch &&
				fabsf(d.actCenter.y - actCenter.y) <= kActCenterMatch &&
				fabsf(d.actCenter.z - actCenter.z) <= kActCenterMatch)
				d.teamMask = teamMask;
}

void AddCamp(const CampSpot &spot)
{
	if (g_mesh)
		g_mesh->camps.push_back(spot);
}

}

