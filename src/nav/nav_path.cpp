#include "nav_path.h"

#include "drift.h"

#include <algorithm>
#include <cmath>
#include <queue>

#include "astar_graph.h"
#include "nav_barrier.h"
#include "nav_water.h"
#include "pm_sim.h"

namespace Nav
{

namespace
{

uint32_t g_planTeamMask = 0;

bool TeamBlocks(const Connection &c)
{
	return c.teamMask != 0 && (c.teamMask & g_planTeamMask) == 0;
}

bool LiveBlocks(const NavMesh &mesh, const Connection &c)
{
	return crossbot_nav::ConnBlockedByWater(c.connType, ConnSubmerged(mesh, c)) || ConnBarred(mesh, c);
}

float FallCost(const Connection &c, const PmSim::FallRule &fall)
{
	if (c.fallHeight <= 0.f)
		return 0.f;
	if (c.connType != crossbot_nav::CONN_AIRMOVE && c.connType != crossbot_nav::CONN_LADDER &&
		c.connType != crossbot_nav::CONN_TELEPORT)
		return 0.f;
	float speed = PmSim::FallSpeed(c.fallHeight, PmSim::kGravity * c.gravity);
	return PmSim::FallPenaltySeconds(PmSim::FallDamageFor(speed, fall), fall.health);
}

constexpr float kHurtSecondsPerHp = 0.05f;
constexpr float kHurtForgetHpPerSecond = 100.f / 300.f;

void HurtSeconds(const NavMesh &mesh, const HurtLog &hurt, std::vector<float> &perConn)
{
	perConn.assign(mesh.conns.size(), 0.f);
	for (int i = 0; i < hurt.count; ++i)
	{
		const HurtEntry &e = hurt.entries[i];
		float seconds = e.hp * kHurtSecondsPerHp;
		if (!e.area)
		{
			if (e.id < perConn.size())
				perConn[e.id] += seconds;
			continue;
		}
		if (e.id >= mesh.areas.size())
			continue;
		for (uint32_t ci : mesh.areas[e.id].connIndices)
		{
			if (ci < perConn.size())
				perConn[ci] += seconds * 0.5f;
		}
	}
}

void BuildGraph(const NavMesh &mesh, std::vector<std::vector<uint32_t>> &areaConnIndices,
	std::vector<NavAStar::Connection> &conns, const PmSim::FallRule &fall)
{
	areaConnIndices.assign(mesh.areas.size(), std::vector<uint32_t>());
	for (size_t i = 0; i < mesh.areas.size(); ++i)
	{
		const std::vector<uint32_t> &src = mesh.areas[i].connIndices;
		std::vector<uint32_t> &dst = areaConnIndices[i];
		dst.reserve(src.size());
		for (uint32_t idx : src)
		{
			if (idx < mesh.conns.size() && (LiveBlocks(mesh, mesh.conns[idx]) || TeamBlocks(mesh.conns[idx])))
				continue;
			dst.push_back(idx);
		}
	}

	conns.assign(mesh.conns.size(), NavAStar::Connection());
	for (size_t i = 0; i < mesh.conns.size(); ++i)
	{
		const Connection &c = mesh.conns[i];
		NavAStar::Connection &gc = conns[i];
		gc.areaA = c.areaA;
		gc.areaB = c.areaB;
		gc.crossing = NavPlan::ConnectionCrossing(mesh.planAreas, mesh.planConns[i]);
		gc.oneWay = (c.connFlags & crossbot_nav::CONN_FLAG_ONE_WAY) != 0;
		gc.drift = c.drift;
		gc.costSeconds = c.costSeconds + c.runtimePenalty + FallCost(c, fall);
		gc.exposure = c.vantageExposure;
		gc.openDistAB = c.openDistAB;
		gc.openDistBA = c.openDistBA;
	}
}

bool GateAllowsCrossing(const NavMesh &mesh, uint32_t connIdx, uint32_t fromId, uint32_t toId)
{
	if (toId >= mesh.planAreas.size() || connIdx >= mesh.planConns.size())
		return true;

	const NavPlan::PlanArea &toArea = mesh.planAreas[toId];
	if (toArea.activation == crossbot_nav::ACT_NONE)
		return true;

	uint8_t connType = mesh.planConns[connIdx].connType;
	if (connType != crossbot_nav::CONN_WALK && connType != crossbot_nav::CONN_ELEVATOR)
		return false;

	const NavPlan::PlanDetour *detour = NavPlan::FindDetour(toArea, fromId);
	return detour != nullptr && !detour->blocked && !detour->points.empty();
}

}

void SetPlanTeam(uint32_t teamBit, bool carrier)
{
	g_planTeamMask = carrier ? 0u : teamBit;
	NavPlan::SetDetourTeamMask(teamBit);
}

ScopedPlanTeam::ScopedPlanTeam(uint32_t teamBit)
	: planMask(g_planTeamMask), detourMask(NavPlan::DetourTeamMask())
{
	SetPlanTeam(teamBit, false);
}

ScopedPlanTeam::~ScopedPlanTeam()
{
	g_planTeamMask = planMask;
	NavPlan::SetDetourTeamMask(detourMask);
}

void HurtLog::Decay(float now)
{
	float forget = (now - decayedAt) * kHurtForgetHpPerSecond;
	decayedAt = now;
	if (forget <= 0.f)
		return;

	for (int i = 0; i < count;)
	{
		entries[i].hp -= forget;
		if (entries[i].hp > 0.f)
		{
			++i;
			continue;
		}
		entries[i] = entries[--count];
	}
}

void HurtLog::Add(uint32_t id, bool area, float hp, float now)
{
	Decay(now);

	for (int i = 0; i < count; ++i)
	{
		if (entries[i].id != id || entries[i].area != area)
			continue;
		entries[i].hp += hp;
		entries[i].time = now;
		return;
	}

	int slot = count;
	if (count < kHurtSlots)
		++count;
	else
	{
		slot = 0;
		for (int i = 1; i < count; ++i)
		{
			if (entries[i].time < entries[slot].time)
				slot = i;
		}
	}

	entries[slot].id = id;
	entries[slot].area = area;
	entries[slot].hp = hp;
	entries[slot].time = now;
}

float HurtLog::Total(uint32_t id, bool area) const
{
	for (int i = 0; i < count; ++i)
	{
		if (entries[i].id == id && entries[i].area == area)
			return entries[i].hp;
	}
	return 0.f;
}

PathSearch::PathSearch(const NavMesh &mesh, uint32_t startArea, uint32_t goalArea,
	const Vector &startPoint, const Vector &goalPoint, const NavPlan::PlanTraceContext *traces,
	float careful, const NavPlan::RangeBias &range, const PmSim::FallRule &fall, const HurtLog *hurt,
	const NavPlan::RevengeLos *los)
	: m_mesh(mesh), m_detourMask(NavPlan::DetourTeamMask()), m_hasTraces(traces != nullptr)
{
	if (los)
	{
		m_los = *los;
		m_hasLos = true;
	}
	if (traces)
		m_traces = *traces;
	if (m_traces.waterSurfaces)
	{
		m_water = *m_traces.waterSurfaces;
		m_traces.waterSurfaces = &m_water;
	}

	BuildGraph(mesh, m_areaConnIndices, m_graphConns, fall);

	if (hurt && hurt->count > 0)
	{
		std::vector<float> hurtSeconds;
		HurtSeconds(mesh, *hurt, hurtSeconds);
		for (size_t i = 0; i < m_graphConns.size(); ++i)
			m_graphConns[i].costSeconds += hurtSeconds[i];
	}

	float maxSpeed = (traces && traces->physics.maxSpeed > 1.f) ? traces->physics.maxSpeed
		: mesh.designMaxSpeed;

	m_search.reset(new NavPlan::RouteSearch(m_areaConnIndices, m_graphConns, mesh.planAreas, mesh.planConns,
		startArea, goalArea, startPoint, goalPoint, m_hasTraces ? &m_traces : nullptr, careful,
		mesh.minConnExposure, maxSpeed, range, m_hasLos ? &m_los : nullptr));
}

PathSearch::~PathSearch() = default;

bool PathSearch::Step(std::chrono::steady_clock::time_point deadline)
{
	NavPlan::SetDetourTeamMask(m_detourMask);
	return m_search->Step(deadline);
}

PlannedPath PathSearch::Finish(NavPlan::PlanStats *stats)
{
	NavPlan::SetDetourTeamMask(m_detourMask);
	PlannedPath path = m_search->Finish(stats);
	for (NavPlan::PlanWaypoint &wp : path.waypoints)
	{
		if (wp.targetAreaId >= 0 && static_cast<size_t>(wp.targetAreaId) < m_mesh.planAreas.size())
			wp.drift = m_mesh.planAreas[static_cast<size_t>(wp.targetAreaId)].drift;
	}
	return path;
}

PlannedPath PlanPath(const NavMesh &mesh, uint32_t startArea, uint32_t goalArea,
	const Vector &startPoint, const Vector &goalPoint, const NavPlan::PlanTraceContext *traces,
	NavPlan::PlanStats *stats, float careful, const NavPlan::RangeBias &range, const PmSim::FallRule &fall,
	const HurtLog *hurt, const NavPlan::RevengeLos *los)
{
	PathSearch search(mesh, startArea, goalArea, startPoint, goalPoint, traces, careful, range, fall, hurt,
		los);
	search.Step(std::chrono::steady_clock::time_point::max());
	return search.Finish(stats);
}

namespace
{

constexpr float kGravityEpsilon = 0.01f;

void AreaCostsImpl(const NavMesh &mesh, const uint32_t *roots, size_t rootCount, bool reverse,
	std::vector<float> &out, float gravityMultiplier, float careful, float maxSpeed,
	const NavPlan::RangeBias &range, const PmSim::FallRule &fall, const HurtLog *hurt)
{
	size_t n = mesh.areas.size();
	out.assign(n, kUnreachableSeconds);

	careful = std::max(0.f, std::min(1.f, careful));

	std::vector<float> hurtSeconds;
	if (hurt && hurt->count > 0)
		HurtSeconds(mesh, *hurt, hurtSeconds);

	std::vector<bool> settled(n, false);

	typedef std::pair<float, uint32_t> Entry;
	std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open;
	for (size_t i = 0; i < rootCount; ++i)
	{
		if (roots[i] >= n || out[roots[i]] == 0.f)
			continue;
		out[roots[i]] = 0.f;
		open.push(Entry(0.f, roots[i]));
	}

	while (!open.empty())
	{
		uint32_t current = open.top().second;
		float currentDist = open.top().first;
		open.pop();

		if (settled[current])
			continue;
		settled[current] = true;

		for (uint32_t connIdx : mesh.areas[current].connIndices)
		{
			if (connIdx >= mesh.conns.size())
				continue;

			const Connection &c = mesh.conns[connIdx];
			uint32_t neighbor = (c.areaA == current) ? c.areaB : c.areaA;
			uint32_t from = reverse ? neighbor : current;
			uint32_t to = reverse ? current : neighbor;

			if (!crossbot_nav::ConnAllowsDirection(c.connFlags, c.areaA, from))
				continue;
			if (neighbor >= n || settled[neighbor])
				continue;
			if (!GateAllowsCrossing(mesh, connIdx, from, to))
				continue;
			if (c.connType == crossbot_nav::CONN_AIRMOVE &&
				std::fabs(c.gravity - gravityMultiplier) > kGravityEpsilon)
				continue;
			if (LiveBlocks(mesh, c) || TeamBlocks(c))
				continue;

			NavDrift::DriftEval drift = NavDrift::EvalDrift(c.drift, maxSpeed, c.areaA == from);
			if (!drift.passable)
				continue;

			float base = c.travelSeconds + c.costSeconds + c.runtimePenalty + drift.seconds +
				FallCost(c, fall);
			if (!hurtSeconds.empty())
				base += hurtSeconds[connIdx];
			if (base < 0.f)
				base = 0.f;
			float openDist = (c.areaA == from) ? c.openDistAB : c.openDistBA;
			float step = currentDist + base * (1.f - careful + careful * c.vantageExposure) *
				range.Scale(openDist);
			if (step >= out[neighbor])
				continue;

			out[neighbor] = step;
			open.push(Entry(step, neighbor));
		}
	}
}

}

void AreaCosts(const NavMesh &mesh, uint32_t startArea, std::vector<float> &out,
	float gravityMultiplier, float careful, float maxSpeed, const NavPlan::RangeBias &range,
	const PmSim::FallRule &fall, const HurtLog *hurt)
{
	AreaCostsImpl(mesh, &startArea, 1, false, out, gravityMultiplier, careful,
		maxSpeed > 1.f ? maxSpeed : mesh.designMaxSpeed, range, fall, hurt);
}

void AreaCosts(const NavMesh &mesh, const std::vector<uint32_t> &startAreas, std::vector<float> &out)
{
	AreaCostsImpl(mesh, startAreas.data(), startAreas.size(), false, out, 1.f, 0.f, mesh.designMaxSpeed,
		NavPlan::RangeBias(), PmSim::kHlFall, nullptr);
}

void AreaCostsToGoal(const NavMesh &mesh, uint32_t goalArea, std::vector<float> &out,
	float gravityMultiplier, float careful, float maxSpeed, const NavPlan::RangeBias &range,
	const PmSim::FallRule &fall, const HurtLog *hurt)
{
	AreaCostsImpl(mesh, &goalArea, 1, true, out, gravityMultiplier, careful,
		maxSpeed > 1.f ? maxSpeed : mesh.designMaxSpeed, range, fall, hurt);
}

}

