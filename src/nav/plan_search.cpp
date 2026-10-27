#include "plan_search.h"

#include <algorithm>
#include <limits>

#include "drift.h"

namespace NavPlan
{

namespace
{

constexpr float kPlanCostEpsilon = 1e-4f;

}

struct SearchNode
{
	PlanWalker walker;
	float g = std::numeric_limits<float>::max();
	float score = std::numeric_limits<float>::max();
	int64_t parentArea = -1;
	int64_t parentConn = -1;
};

RouteSearch::RouteSearch(const std::vector<std::vector<uint32_t>> &areaConnIndices,
	const std::vector<NavAStar::Connection> &graphConns, const std::vector<PlanArea> &planAreas,
	const std::vector<PlanConnection> &planConns, uint32_t startArea, uint32_t goalArea,
	const Vector &startPoint, const Vector &goalPoint, const PlanTraceContext *traces,
	float careful, float minExposure, float maxSpeed, const RangeBias &range,
	const RevengeLos *los)
	: m_areaConnIndices(areaConnIndices), m_graphConns(graphConns), m_planConns(planConns),
	m_traces(traces), m_startArea(startArea), m_goalArea(goalArea), m_goalPoint(goalPoint),
	m_maxSpeed(maxSpeed), m_range(range)
{
	size_t n = planAreas.size();
	if (startArea >= n || goalArea >= n || areaConnIndices.size() < n)
	{
		m_done = true;
		return;
	}
	m_valid = true;

	if (los)
	{
		m_hasLos = true;
		m_los = *los;
		m_blockedOwned.assign(planConns.size(), false);
		m_blocked = &m_blockedOwned;
	}

	m_careful = std::max(0.f, std::min(1.f, careful));
	minExposure = std::max(0.f, std::min(1.f, minExposure));
	m_hScale = (1.f - m_careful + m_careful * minExposure) * range.BestScale();

	m_walkPreds.resize(n);
	m_walkSuccs.resize(n);
	size_t connCount = std::min(graphConns.size(), planConns.size());
	for (size_t i = 0; i < connCount; ++i)
	{
		if (planConns[i].connType != crossbot_nav::CONN_WALK)
			continue;
		if (m_blocked && i < m_blocked->size() && (*m_blocked)[i])
			continue;

		const NavAStar::Connection &gc = graphConns[i];
		if (gc.areaA >= n || gc.areaB >= n)
			continue;

		m_walkPreds[gc.areaB].push_back(gc.areaA);
		m_walkSuccs[gc.areaA].push_back(gc.areaB);
		if (!gc.oneWay)
		{
			m_walkPreds[gc.areaA].push_back(gc.areaB);
			m_walkSuccs[gc.areaB].push_back(gc.areaA);
		}
	}

	m_state.resize(n);
	m_closed.assign(n, false);

	m_state[startArea].walker = PlanWalker(planAreas, traces, &m_walkPreds, &m_walkSuccs);
	m_state[startArea].walker.Reset(startPoint);
	m_state[startArea].g = 0.f;
	m_state[startArea].score = 0.f;

	m_open.push({Heuristic(startPoint), startArea});
}

RouteSearch::~RouteSearch() = default;

float RouteSearch::Heuristic(const Vector &from) const
{
	return (m_goalPoint - from).Length() / NavAStar::kNavRunSpeed * m_hScale;
}

void RouteSearch::Relax(uint32_t current, uint32_t connIdx)
{
	if (connIdx >= m_graphConns.size() || connIdx >= m_planConns.size())
		return;

	const NavAStar::Connection &gc = m_graphConns[connIdx];
	uint32_t neighbor = (gc.areaA == current) ? gc.areaB : gc.areaA;

	if (gc.oneWay && gc.areaA != current)
		return;

	if (m_blocked && connIdx < m_blocked->size() && (*m_blocked)[connIdx])
		return;

	NavDrift::DriftEval drift = NavDrift::EvalDrift(gc.drift, m_maxSpeed, gc.areaA == current);
	if (!drift.passable)
		return;
	if (neighbor >= m_state.size())
		return;

	if (m_state[current].score + kPlanCostEpsilon >= m_state[neighbor].score)
		return;

	if (m_planConns[connIdx].connType == crossbot_nav::CONN_AIRMOVE)
		++m_result.airmovesTried;

	PlanWalker next = m_state[current].walker;
	if (!next.Cross(m_planConns[connIdx], connIdx, current, neighbor))
	{
		++m_result.airmovesRejected;
		return;
	}

	next.AddCost(gc.costSeconds + drift.seconds);

	float openDist = (gc.areaA == current) ? gc.openDistAB : gc.openDistBA;

	float g = std::max(next.Time(), m_state[current].g);
	float score = m_state[current].score +
		(g - m_state[current].g) * (1.f - m_careful + m_careful * gc.exposure) *
		m_range.Scale(openDist);
	if (score + kPlanCostEpsilon >= m_state[neighbor].score)
		return;

	m_state[neighbor].g = g;
	m_state[neighbor].score = score;
	m_state[neighbor].walker = std::move(next);
	m_state[neighbor].parentArea = static_cast<int64_t>(current);
	m_state[neighbor].parentConn = static_cast<int64_t>(connIdx);

	m_closed[neighbor] = false;
	m_open.push({score + Heuristic(m_state[neighbor].walker.Frontier()), neighbor});
}

bool RouteSearch::BuildLosSlice(std::chrono::steady_clock::time_point deadline)
{
	if (!m_traces || !m_traces->standTrace)
	{
		m_losDone = true;
		return true;
	}

	size_t n = std::min(m_graphConns.size(), m_planConns.size());
	int since = 0;

	while (m_losCursor < n)
	{
		if (++since >= 64)
		{
			since = 0;
			if (std::chrono::steady_clock::now() >= deadline)
				return false;
		}

		size_t i = m_losCursor++;
		if (m_planConns[i].connType != crossbot_nav::CONN_WALK)
			continue;

		Vector c = m_graphConns[i].crossing;
		c.z += 29.f;

		Vector delta = c - m_los.eye;
		float dist = delta.Length();
		if (dist > m_los.radius || dist < 1.f)
			continue;

		Vector dir = delta * (1.f / dist);
		float facing = dir.x * m_los.forward.x + dir.y * m_los.forward.y + dir.z * m_los.forward.z;
		if (facing < m_los.cosFov)
			continue;

		++m_result.losTraced;
		PmSim::PmTrace tr = m_traces->standTrace(m_los.eye, c);
		if (tr.fraction >= 1.f && !tr.startsolid)
		{
			m_blockedOwned[i] = true;
			++m_result.losBlocked;
		}
	}

	m_losDone = true;
	return true;
}

bool RouteSearch::Step(std::chrono::steady_clock::time_point deadline)
{
	if (m_hasLos && !m_losDone)
	{
		if (!BuildLosSlice(deadline))
			return false;
	}

	bool first = true;
	while (!m_done)
	{
		if (m_current < 0)
		{
			if (m_open.empty())
				break;

			uint32_t current = m_open.top().areaId;
			m_open.pop();

			if (m_closed[current])
				continue;
			m_closed[current] = true;
			++m_result.expansions;

			if (current == m_goalArea)
				break;

			m_current = static_cast<int64_t>(current);
			m_edge = 0;
		}

		uint32_t current = static_cast<uint32_t>(m_current);
		const std::vector<uint32_t> &edges = m_areaConnIndices[current];
		while (m_edge < edges.size())
		{
			if (!first && std::chrono::steady_clock::now() >= deadline)
				return false;
			first = false;
			Relax(current, edges[m_edge++]);
		}
		m_current = -1;
	}
	m_done = true;
	return true;
}

PlannedRoute RouteSearch::Finish(PlanStats *stats)
{
	PlannedRoute result = m_result;
	if (!m_valid || m_state[m_goalArea].g >= std::numeric_limits<float>::max())
		return result;

	size_t n = m_state.size();
	std::vector<uint32_t> areaPath, connPath;
	uint32_t cur = m_goalArea;
	size_t guard = 0;

	while (cur != m_startArea)
	{
		if (m_state[cur].parentArea < 0 || ++guard > n)
			return result;

		areaPath.push_back(cur);
		connPath.push_back(static_cast<uint32_t>(m_state[cur].parentConn));
		cur = static_cast<uint32_t>(m_state[cur].parentArea);
	}

	areaPath.push_back(m_startArea);
	std::reverse(areaPath.begin(), areaPath.end());
	std::reverse(connPath.begin(), connPath.end());

	PlanWalker walker = std::move(m_state[m_goalArea].walker);
	walker.Finish(m_goalPoint, static_cast<int>(m_goalArea));
	walker.Retract();
	walker.Respeed();
	walker.Relax(m_startArea);

	result.found = true;
	result.complete = true;
	result.routeSeconds = m_state[m_goalArea].g;
	result.routeScore = m_state[m_goalArea].score;
	result.areaPath = std::move(areaPath);
	result.connPath = std::move(connPath);

	if (stats)
		*stats = walker.Stats();

	result.waypoints = walker.TakeWaypoints();
	return result;
}

PlannedRoute PlanRoute(const std::vector<std::vector<uint32_t>> &areaConnIndices,
	const std::vector<NavAStar::Connection> &graphConns, const std::vector<PlanArea> &planAreas,
	const std::vector<PlanConnection> &planConns, uint32_t startArea, uint32_t goalArea,
	const Vector &startPoint, const Vector &goalPoint, const PlanTraceContext *traces,
	PlanStats *stats, float careful, float minExposure, float maxSpeed, const RangeBias &range,
	const RevengeLos *los)
{
	RouteSearch search(areaConnIndices, graphConns, planAreas, planConns, startArea, goalArea, startPoint,
		goalPoint, traces, careful, minExposure, maxSpeed, range, los);
	search.Step(std::chrono::steady_clock::time_point::max());
	return search.Finish(stats);
}

}
