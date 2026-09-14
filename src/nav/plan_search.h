#pragma once

#include <algorithm>
#include <cstdint>
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

struct PlannedRoute
{
	bool found = false;
	bool complete = false;
	size_t airmovesRejected = 0;
	size_t airmovesTried = 0;
	size_t expansions = 0;
	float routeSeconds = 0.f;
	float routeScore = 0.f;
	std::vector<uint32_t> areaPath;
	std::vector<uint32_t> connPath;
	std::vector<PlanWaypoint> waypoints;
};

PlannedRoute PlanRoute(const std::vector<std::vector<uint32_t>> &areaConnIndices,
	const std::vector<NavAStar::Connection> &graphConns, const std::vector<PlanArea> &planAreas,
	const std::vector<PlanConnection> &planConns, uint32_t startArea, uint32_t goalArea,
	const Vector &startPoint, const Vector &goalPoint, const PlanTraceContext *traces = nullptr,
	PlanStats *stats = nullptr, float careful = 0.f, float minExposure = 1.f,
	float maxSpeed = crossbot_nav::kNavDefaultDesignSpeed, const RangeBias &range = RangeBias());

}
