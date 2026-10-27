#pragma once

#include <extdll.h>

#include <chrono>
#include <cstdint>
#include <memory>
#include <vector>

#include "nav_mesh.h"
#include "plan_search.h"
#include "pm_sim.h"
#include "waypoint_plan.h"

namespace Nav
{

using Waypoint = NavPlan::PlanWaypoint;

using PlannedPath = NavPlan::PlannedRoute;

void SetPlanTeam(uint32_t teamBit, bool carrier);

struct ScopedPlanTeam
{
	explicit ScopedPlanTeam(uint32_t teamBit);
	~ScopedPlanTeam();
	ScopedPlanTeam(const ScopedPlanTeam &) = delete;
	ScopedPlanTeam &operator=(const ScopedPlanTeam &) = delete;

	uint32_t planMask;
	uint32_t detourMask;
};

constexpr int kHurtSlots = 32;

struct HurtEntry
{
	uint32_t id;
	bool area;
	float hp;
	float time;
};

struct HurtLog
{
	HurtEntry entries[kHurtSlots];
	int count = 0;
	float decayedAt = 0.f;

	void Decay(float now);
	void Add(uint32_t id, bool area, float hp, float now);
	float Total(uint32_t id, bool area) const;
};

class PathSearch
{
public:
	PathSearch(const NavMesh &mesh, uint32_t startArea, uint32_t goalArea, const Vector &startPoint,
		const Vector &goalPoint, const NavPlan::PlanTraceContext *traces, float careful,
		const NavPlan::RangeBias &range, const PmSim::FallRule &fall, const HurtLog *hurt,
		const NavPlan::RevengeLos *los = nullptr);
	~PathSearch();
	PathSearch(const PathSearch &) = delete;
	PathSearch &operator=(const PathSearch &) = delete;

	bool Step(std::chrono::steady_clock::time_point deadline);
	PlannedPath Finish(NavPlan::PlanStats *stats);
	const NavMesh &Mesh() const { return m_mesh; }
	const NavPlan::PlanTraceContext &Traces() const { return m_traces; }

private:
	const NavMesh &m_mesh;
	NavPlan::PlanTraceContext m_traces;
	std::vector<float> m_water;
	uint32_t m_detourMask;
	bool m_hasTraces;
	std::vector<std::vector<uint32_t>> m_areaConnIndices;
	std::vector<NavAStar::Connection> m_graphConns;
	NavPlan::RevengeLos m_los;
	bool m_hasLos = false;
	std::unique_ptr<NavPlan::RouteSearch> m_search;
};

PlannedPath PlanPath(const NavMesh &mesh, uint32_t startArea, uint32_t goalArea,
	const Vector &startPoint, const Vector &goalPoint,
	const NavPlan::PlanTraceContext *traces = nullptr, NavPlan::PlanStats *stats = nullptr,
	float careful = 0.f, const NavPlan::RangeBias &range = NavPlan::RangeBias(),
	const PmSim::FallRule &fall = PmSim::kHlFall, const HurtLog *hurt = nullptr,
	const NavPlan::RevengeLos *los = nullptr);

constexpr float kUnreachableSeconds = 1.0e9f;

void AreaCosts(const NavMesh &mesh, uint32_t startArea, std::vector<float> &out,
	float gravityMultiplier = 1.f, float careful = 0.f, float maxSpeed = 0.f,
	const NavPlan::RangeBias &range = NavPlan::RangeBias(), const PmSim::FallRule &fall = PmSim::kHlFall,
	const HurtLog *hurt = nullptr);

void AreaCosts(const NavMesh &mesh, const std::vector<uint32_t> &startAreas, std::vector<float> &out);

void AreaCostsToGoal(const NavMesh &mesh, uint32_t goalArea, std::vector<float> &out,
	float gravityMultiplier = 1.f, float careful = 0.f, float maxSpeed = 0.f,
	const NavPlan::RangeBias &range = NavPlan::RangeBias(), const PmSim::FallRule &fall = PmSim::kHlFall,
	const HurtLog *hurt = nullptr);

}

