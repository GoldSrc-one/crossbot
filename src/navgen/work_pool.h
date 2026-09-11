#pragma once

#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <functional>
#include <mutex>
#include <thread>
#include <vector>

#include "polytope.h"

namespace navgen
{

size_t DefaultThreadCount();

uint64_t MortonKey(const Vec3 &point, double cell);

std::vector<std::vector<size_t>> BuildWorkBins(const std::vector<double> &cost,
	const std::vector<size_t> &order, size_t tileSize, size_t binCount);

void RunBins(size_t binCount, const std::function<void(size_t)> &body);

class ThreadPool
{
public:

	explicit ThreadPool(size_t threads);
	~ThreadPool();

	ThreadPool(const ThreadPool &) = delete;
	ThreadPool &operator=(const ThreadPool &) = delete;

	size_t ThreadCount() const { return m_workers.size() + 1; }


	void Run(size_t count, const std::function<void(size_t, size_t)> &body);

private:

	void Drain(size_t slot);
	void WorkerLoop(size_t slot);

	std::vector<std::thread> m_workers;
	std::mutex m_mutex;
	std::condition_variable m_startCv;
	std::condition_variable m_doneCv;
	std::vector<std::exception_ptr> m_errors;
	const std::function<void(size_t, size_t)> *m_body = nullptr;
	std::atomic<size_t> m_next;
	size_t m_count = 0;
	size_t m_active = 0;
	uint64_t m_generation = 0;
	bool m_stop = false;
};

}
