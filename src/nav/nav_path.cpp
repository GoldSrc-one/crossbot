#include "nav_path.h"

#include "drift.h"

#include <algorithm>
#include <cmath>
#include <queue>

#include "astar_graph.h"
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

bool WaterBlocks(const NavMesh &mesh, const Connection &c)
{
	return crossbot_nav::ConnBlockedByWater(c.connType, ConnSubmerged(mesh, c));
}

float FallCost(const Connection &c, const PmSim::FallRule &fall)
{
	if (c.fallHeight <= 0.f)
		return 0.f;
	if (c.connType != crossbot_nav::CONN_AIRMOVE && c.connType != crossbot_nav::CONN_LADDER &&
		c.connType != crossbot_nav::CONN_TELEPORT)
		return 0.f;
	float speed = PmSim::FallSpeed(c.fallHeight, PmSim::kGravity * c.gravity);
	return PmSim::FallPenaltySeconds(PmSim::FallDamageFor(speed, fall));
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
			if (idx < mesh.conns.size() && (WaterBlocks(mesh, mesh.conns[idx]) || TeamBlocks(mesh.conns[idx])))
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

void SetPlanTeamMask(uint32_t mask)
{
	g_planTeamMask = mask;
}

PlannedPath PlanPath(const NavMesh &mesh, uint32_t startArea, uint32_t goalArea,
	const Vector &startPoint, const Vector &goalPoint, const NavPlan::PlanTraceContext *traces,
	NavPlan::PlanStats *stats, float careful, const NavPlan::RangeBias &range, const PmSim::FallRule &fall)
{
	std::vector<std::vector<uint32_t>> areaConnIndices;
	std::vector<NavAStar::Connection> graphConns;
	BuildGraph(mesh, areaConnIndices, graphConns, fall);

	float maxSpeed = (traces && traces->physics.maxSpeed > 1.f) ? traces->physics.maxSpeed
		: mesh.designMaxSpeed;

	PlannedPath path = NavPlan::PlanRoute(areaConnIndices, graphConns, mesh.planAreas, mesh.planConns,
		startArea, goalArea, startPoint, goalPoint, traces, stats, careful, mesh.minConnExposure,
		maxSpeed, range);

	for (NavPlan::PlanWaypoint &wp : path.waypoints)
	{
		if (wp.targetAreaId >= 0 && static_cast<size_t>(wp.targetAreaId) < mesh.planAreas.size())
			wp.drift = mesh.planAreas[static_cast<size_t>(wp.targetAreaId)].drift;
	}

	return path;
}

namespace
{

constexpr float kGravityEpsilon = 0.01f;

void AreaCostsImpl(const NavMesh &mesh, uint32_t rootArea, bool reverse, std::vector<float> &out,
	float gravityMultiplier, float careful, float maxSpeed, const NavPlan::RangeBias &range,
	const PmSim::FallRule &fall)
{
	size_t n = mesh.areas.size();
	out.assign(n, kUnreachableSeconds);
	if (rootArea >= n)
		return;

	careful = std::max(0.f, std::min(1.f, careful));

	std::vector<bool> settled(n, false);
	out[rootArea] = 0.f;

	typedef std::pair<float, uint32_t> Entry;
	std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open;
	open.push(Entry(0.f, rootArea));

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
			if (WaterBlocks(mesh, c) || TeamBlocks(c))
				continue;

			NavDrift::DriftEval drift = NavDrift::EvalDrift(c.drift, maxSpeed, c.areaA == from);
			if (!drift.passable)
				continue;

			float base = c.travelSeconds + c.costSeconds + c.runtimePenalty + drift.seconds +
				FallCost(c, fall);
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
	const PmSim::FallRule &fall)
{
	AreaCostsImpl(mesh, startArea, false, out, gravityMultiplier, careful,
		maxSpeed > 1.f ? maxSpeed : mesh.designMaxSpeed, range, fall);
}

void AreaCostsToGoal(const NavMesh &mesh, uint32_t goalArea, std::vector<float> &out,
	float gravityMultiplier, float careful, float maxSpeed, const NavPlan::RangeBias &range,
	const PmSim::FallRule &fall)
{
	AreaCostsImpl(mesh, goalArea, true, out, gravityMultiplier, careful,
		maxSpeed > 1.f ? maxSpeed : mesh.designMaxSpeed, range, fall);
}

}

