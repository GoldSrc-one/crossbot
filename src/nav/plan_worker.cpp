#include "plan_worker.h"

#include <chrono>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <thread>

namespace Nav
{

namespace
{

constexpr std::chrono::milliseconds kPlanSlice(4);

std::mutex g_mutex;
std::condition_variable g_wake;
std::condition_variable g_idle;
std::deque<std::weak_ptr<PlanJob>> g_queue;
std::thread g_thread;
bool g_busy = false;
bool g_quit = false;

void RunSlice(PlanJob &job)
{
	std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
	if (job.search->Step(begin + kPlanSlice))
	{
		job.path = job.search->Finish(&job.stats);
		if (job.path.found)
		{
			const NavPlan::PlanTraceContext &traces = job.search->Traces();
			job.profile = NavPlan::BuildPathProfile(job.path.waypoints, traces.physics, &traces);
		}
		job.done.store(true, std::memory_order_release);
	}
	job.workMs += std::chrono::duration<float, std::milli>(std::chrono::steady_clock::now() - begin).count();
}

void Work()
{
	std::unique_lock<std::mutex> lock(g_mutex);
	while (true)
	{
		g_wake.wait(lock, [] { return g_quit || !g_queue.empty(); });
		if (g_quit)
			return;

		std::shared_ptr<PlanJob> job = g_queue.front().lock();
		g_queue.pop_front();
		if (!job)
			continue;

		g_busy = true;
		lock.unlock();
		RunSlice(*job);
		lock.lock();
		g_busy = false;

		if (!job->done.load(std::memory_order_relaxed))
			g_queue.push_back(job);
		g_idle.notify_all();
	}
}

}

void SubmitPlan(const std::shared_ptr<PlanJob> &job)
{
	std::lock_guard<std::mutex> lock(g_mutex);
	if (!g_thread.joinable())
	{
		g_quit = false;
		g_thread = std::thread(Work);
	}
	g_queue.push_back(job);
	g_wake.notify_one();
}

void FlushPlans()
{
	std::unique_lock<std::mutex> lock(g_mutex);
	g_queue.clear();
	g_idle.wait(lock, [] { return !g_busy; });
}

void StopPlanWorker()
{
	{
		std::lock_guard<std::mutex> lock(g_mutex);
		g_quit = true;
		g_queue.clear();
	}
	g_wake.notify_all();
	if (g_thread.joinable())
		g_thread.join();
}

}
