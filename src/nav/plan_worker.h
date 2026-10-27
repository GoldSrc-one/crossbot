#pragma once

#include <atomic>
#include <memory>

#include "nav_path.h"
#include "path_profile.h"

namespace Nav
{

struct PlanJob
{
	std::unique_ptr<PathSearch> search;
	std::atomic<bool> done{ false };
	PlannedPath path;
	NavPlan::PlanStats stats;
	NavPlan::PlanProfile profile;
	float workMs = 0.f;
};

void SubmitPlan(const std::shared_ptr<PlanJob> &job);

void FlushPlans();

void StopPlanWorker();

}
