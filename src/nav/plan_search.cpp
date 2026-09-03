#include "plan_search.h"

#include <algorithm>
#include <limits>
#include <queue>

#include "drift.h"

namespace NavPlan
{

namespace
{

constexpr float kPlanCostEpsilon = 1e-4f;

struct OpenEntry
{
	float fScore;
	uint32_t areaId;
};

struct OpenEntryCompare
{
	bool operator()(const OpenEntry &a, const OpenEntry &b) const { return a.fScore > b.fScore; }
};

struct SearchNode
{
	PlanWalker walker;
	float g = std::numeric_limits<float>::max();
	float score = std::numeric_limits<float>::max();
	int64_t parentArea = -1;
	int64_t parentConn = -1;
};

}

PlannedRoute PlanRoute(const std::vector<std::vector<uint32_t>> &areaConnIndices,
	const std::vector<NavAStar::Connection> &graphConns, const std::vector<PlanArea> &planAreas,
	const std::vector<PlanConnection> &planConns, uint32_t startArea, uint32_t goalArea,
	const Vector &startPoint, const Vector &goalPoint, const PlanTraceContext *traces,
	PlanStats *stats, float careful, float minExposure, float maxSpeed)
{
	PlannedRoute result;

	size_t n = planAreas.size();
	if (startArea >= n || goalArea >= n || areaConnIndices.size() < n)
		return result;

	careful = std::max(0.f, std::min(1.f, careful));
	minExposure = std::max(0.f, std::min(1.f, minExposure));

	const float hScale = 1.f - careful + careful * minExposure;

	auto Heuristic = [&](const Vector &from)
	{
		return (goalPoint - from).Length() / NavAStar::kNavRunSpeed * hScale;
	};

	std::vector<std::vector<uint32_t>> walkPreds(n);
	size_t connCount = std::min(graphConns.size(), planConns.size());
	for (size_t i = 0; i < connCount; ++i)
	{
		if (planConns[i].connType != crossbot_nav::CONN_WALK)
			continue;

		const NavAStar::Connection &gc = graphConns[i];
		if (gc.areaA >= n || gc.areaB >= n)
			continue;

		walkPreds[gc.areaB].push_back(gc.areaA);
		if (!gc.oneWay)
			walkPreds[gc.areaA].push_back(gc.areaB);
	}

	std::vector<SearchNode> state(n);
	std::vector<bool> closed(n, false);

	state[startArea].walker = PlanWalker(planAreas, traces, &walkPreds);
	state[startArea].walker.Reset(startPoint);
	state[startArea].g = 0.f;
	state[startArea].score = 0.f;

	std::priority_queue<OpenEntry, std::vector<OpenEntry>, OpenEntryCompare> open;
	open.push({Heuristic(startPoint), startArea});

	while (!open.empty())
	{
		uint32_t current = open.top().areaId;
		open.pop();

		if (closed[current])
			continue;
		closed[current] = true;
		++result.expansions;

		if (current == goalArea)
			break;

		for (uint32_t connIdx : areaConnIndices[current])
		{
			if (connIdx >= graphConns.size() || connIdx >= planConns.size())
				continue;

			const NavAStar::Connection &gc = graphConns[connIdx];
			uint32_t neighbor = (gc.areaA == current) ? gc.areaB : gc.areaA;

			if (gc.oneWay && gc.areaA != current)
				continue;

			NavDrift::DriftEval drift = NavDrift::EvalDrift(gc.drift, maxSpeed, gc.areaA == current);
			if (!drift.passable)
				continue;
			if (neighbor >= n)
				continue;

			if (state[current].score + kPlanCostEpsilon >= state[neighbor].score)
				continue;

			if (planConns[connIdx].connType == crossbot_nav::CONN_AIRMOVE)
				++result.airmovesTried;

			PlanWalker next = state[current].walker;
			if (!next.Cross(planConns[connIdx], connIdx, current, neighbor))
			{
				++result.airmovesRejected;
				continue;
			}

			next.AddCost(gc.costSeconds + drift.seconds);

			float g = std::max(next.Time(), state[current].g);
			float score = state[current].score +
				(g - state[current].g) * (1.f - careful + careful * gc.exposure);
			if (score + kPlanCostEpsilon >= state[neighbor].score)
				continue;

			state[neighbor].g = g;
			state[neighbor].score = score;
			state[neighbor].walker = std::move(next);
			state[neighbor].parentArea = static_cast<int64_t>(current);
			state[neighbor].parentConn = static_cast<int64_t>(connIdx);

			closed[neighbor] = false;
			open.push({score + Heuristic(state[neighbor].walker.Frontier()), neighbor});
		}
	}

	if (state[goalArea].g >= std::numeric_limits<float>::max())
		return result;

	std::vector<uint32_t> areaPath, connPath;
	uint32_t cur = goalArea;
	size_t guard = 0;

	while (cur != startArea)
	{
		if (state[cur].parentArea < 0 || ++guard > n)
			return result;

		areaPath.push_back(cur);
		connPath.push_back(static_cast<uint32_t>(state[cur].parentConn));
		cur = static_cast<uint32_t>(state[cur].parentArea);
	}

	areaPath.push_back(startArea);
	std::reverse(areaPath.begin(), areaPath.end());
	std::reverse(connPath.begin(), connPath.end());

	PlanWalker walker = std::move(state[goalArea].walker);
	walker.Finish(goalPoint, static_cast<int>(goalArea));
	walker.Retract();
	walker.Respeed();

	result.found = true;
	result.complete = true;
	result.routeSeconds = state[goalArea].g;
	result.routeScore = state[goalArea].score;
	result.areaPath = std::move(areaPath);
	result.connPath = std::move(connPath);

	if (stats)
		*stats = walker.Stats();

	result.waypoints = walker.TakeWaypoints();
	return result;
}

}
