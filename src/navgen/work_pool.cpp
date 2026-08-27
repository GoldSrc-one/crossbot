#include "work_pool.h"

#include <algorithm>
#include <cmath>
#include <exception>
#include <thread>

namespace navgen
{

namespace
{

uint64_t SpreadBits(uint64_t v)
{
	v &= 0x1FFFFFull;
	v = (v | (v << 32)) & 0x1F00000000FFFFull;
	v = (v | (v << 16)) & 0x1F0000FF0000FFull;
	v = (v | (v << 8)) & 0x100F00F00F00F00Full;
	v = (v | (v << 4)) & 0x10C30C30C30C30C3ull;
	v = (v | (v << 2)) & 0x1249249249249249ull;
	return v;
}

uint64_t Quantize(double v, double cell)
{
	if (!(cell > 0.0))
		cell = 1.0;

	double scaled = std::floor(v / cell) + 1048576.0;
	if (!(scaled > 0.0))
		return 0;
	if (scaled > 2097151.0)
		return 2097151ull;
	return static_cast<uint64_t>(scaled);
}

}

size_t DefaultThreadCount()
{
	unsigned int detected = std::thread::hardware_concurrency();
	if (detected == 0)
		return 4;

	return static_cast<size_t>(detected) + 1;
}

uint64_t MortonKey(const Vec3 &point, double cell)
{
	return SpreadBits(Quantize(point.x, cell)) |
		(SpreadBits(Quantize(point.y, cell)) << 1) |
		(SpreadBits(Quantize(point.z, cell)) << 2);
}

std::vector<std::vector<size_t>> BuildWorkBins(const std::vector<double> &cost,
	const std::vector<size_t> &order, size_t tileSize, size_t binCount)
{
	std::vector<std::vector<size_t>> bins(binCount);
	if (binCount == 0 || order.empty())
		return bins;

	if (tileSize == 0)
		tileSize = 1;

	struct Tile
	{
		size_t begin = 0;
		size_t end = 0;
		double cost = 0.0;
	};

	std::vector<Tile> tiles;
	tiles.reserve(order.size() / tileSize + 1);
	for (size_t begin = 0; begin < order.size(); begin += tileSize)
	{
		Tile tile;
		tile.begin = begin;
		tile.end = std::min(begin + tileSize, order.size());
		for (size_t k = tile.begin; k < tile.end; ++k)
			tile.cost += cost[order[k]];
		tiles.push_back(tile);
	}

	std::vector<size_t> byCost(tiles.size());
	for (size_t i = 0; i < tiles.size(); ++i)
		byCost[i] = i;

	std::sort(byCost.begin(), byCost.end(), [&](size_t a, size_t b)
	{
		if (tiles[a].cost != tiles[b].cost)
			return tiles[a].cost > tiles[b].cost;
		return a < b;
	});

	std::vector<double> load(binCount, 0.0);
	std::vector<std::vector<size_t>> binTiles(binCount);
	for (size_t t : byCost)
	{
		size_t best = 0;
		for (size_t b = 1; b < binCount; ++b)
		{
			if (load[b] < load[best])
				best = b;
		}

		load[best] += tiles[t].cost;
		binTiles[best].push_back(t);
	}

	for (size_t b = 0; b < binCount; ++b)
	{
		std::sort(binTiles[b].begin(), binTiles[b].end());

		size_t total = 0;
		for (size_t t : binTiles[b])
			total += tiles[t].end - tiles[t].begin;

		bins[b].reserve(total);
		for (size_t t : binTiles[b])
		{
			for (size_t k = tiles[t].begin; k < tiles[t].end; ++k)
				bins[b].push_back(order[k]);
		}
	}

	return bins;
}

void RunBins(size_t binCount, const std::function<void(size_t)> &body)
{
	if (binCount == 0)
		return;

	if (binCount == 1)
	{
		body(0);
		return;
	}

	std::vector<std::exception_ptr> errors(binCount);
	std::vector<std::thread> workers;
	workers.reserve(binCount - 1);

	for (size_t b = 1; b < binCount; ++b)
	{
		workers.emplace_back([&errors, &body, b]()
		{
			try
			{
				body(b);
			}
			catch (...)
			{
				errors[b] = std::current_exception();
			}
		});
	}

	try
	{
		body(0);
	}
	catch (...)
	{
		errors[0] = std::current_exception();
	}

	for (std::thread &worker : workers)
		worker.join();

	for (const std::exception_ptr &error : errors)
	{
		if (error)
			std::rethrow_exception(error);
	}
}

}
