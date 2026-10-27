#pragma once

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <queue>
#include <vector>

#include "astar_graph.h"
#include "waypoint_plan.h"

namespace NavPlan
{

struct RangeBias
{
	float weight = 0.f;
	float shortDist = 512.f;
	float longDist = 1024.f;
	bool preferLong = false;

	float Mismatch(float openDist) const
	{
		float span = std::max(1.f, longDist - shortDist);
		float t = std::max(0.f, std::min(1.f, (openDist - shortDist) / span));
		return preferLong ? (1.f - t) : t;
	}

	float Weight() const { return std::max(0.f, std::min(1.f, weight)); }

	float Scale(float openDist) const
	{
		float w = Weight();
		return 1.f - w + w * Mismatch(openDist);
	}

	float BestScale() const { return 1.f - Weight(); }
};

struct RevengeLos
{
	Vector eye;
	Vector forward;
	float cosFov = 1.f;
	float radius = 0.f;
};

struct PlannedRoute
{
	bool found = false;
	bool complete = false;
	size_t airmovesRejected = 0;
	size_t airmovesTried = 0;
	size_t expansions = 0;
	size_t losTraced = 0;
	size_t losBlocked = 0;
	float routeSeconds = 0.f;
	float routeScore = 0.f;
	std::vector<uint32_t> areaPath;
	std::vector<uint32_t> connPath;
	std::vector<PlanWaypoint> waypoints;
};

struct SearchNode;

class RouteSearch
{
public:
	RouteSearch(const std::vector<std::vector<uint32_t>> &areaConnIndices,
		const std::vector<NavAStar::Connection> &graphConns, const std::vector<PlanArea> &planAreas,
		const std::vector<PlanConnection> &planConns, uint32_t startArea, uint32_t goalArea,
		const Vector &startPoint, const Vector &goalPoint, const PlanTraceContext *traces,
		float careful, float minExposure, float maxSpeed, const RangeBias &range,
		const RevengeLos *los = nullptr);
	~RouteSearch();
	RouteSearch(const RouteSearch &) = delete;
	RouteSearch &operator=(const RouteSearch &) = delete;

	bool Step(std::chrono::steady_clock::time_point deadline);
	PlannedRoute Finish(PlanStats *stats);

private:
	struct OpenEntry
	{
		float fScore;
		uint32_t areaId;
		bool operator<(const OpenEntry &o) const { return fScore > o.fScore; }
	};

	float Heuristic(const Vector &from) const;
	void Relax(uint32_t current, uint32_t connIdx);
	bool BuildLosSlice(std::chrono::steady_clock::time_point deadline);

	const std::vector<std::vector<uint32_t>> &m_areaConnIndices;
	const std::vector<NavAStar::Connection> &m_graphConns;
	const std::vector<PlanConnection> &m_planConns;
	const PlanTraceContext *m_traces = nullptr;
	uint32_t m_startArea;
	uint32_t m_goalArea;
	Vector m_goalPoint;
	float m_careful;
	float m_maxSpeed;
	float m_hScale;
	RangeBias m_range;
	RevengeLos m_los;
	bool m_hasLos = false;
	bool m_losDone = false;
	size_t m_losCursor = 0;
	std::vector<bool> m_blockedOwned;
	const std::vector<bool> *m_blocked = nullptr;
	bool m_valid = false;
	bool m_done = false;
	int64_t m_current = -1;
	size_t m_edge = 0;
	PlannedRoute m_result;
	std::vector<std::vector<uint32_t>> m_walkPreds;
	std::vector<std::vector<uint32_t>> m_walkSuccs;
	std::vector<SearchNode> m_state;
	std::vector<bool> m_closed;
	std::priority_queue<OpenEntry> m_open;
};

PlannedRoute PlanRoute(const std::vector<std::vector<uint32_t>> &areaConnIndices,
	const std::vector<NavAStar::Connection> &graphConns, const std::vector<PlanArea> &planAreas,
	const std::vector<PlanConnection> &planConns, uint32_t startArea, uint32_t goalArea,
	const Vector &startPoint, const Vector &goalPoint, const PlanTraceContext *traces = nullptr,
	PlanStats *stats = nullptr, float careful = 0.f, float minExposure = 1.f,
	float maxSpeed = crossbot_nav::kNavDefaultDesignSpeed, const RangeBias &range = RangeBias(),
	const RevengeLos *los = nullptr);

}
