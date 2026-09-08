#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <utility>
#include <vector>

#include "polytope.h"

namespace navgen
{

struct PointCellKey
{
	int x = 0, y = 0, z = 0;

	bool operator==(const PointCellKey &o) const { return x == o.x && y == o.y && z == o.z; }
};

struct PointCellKeyHash
{
	size_t operator()(const PointCellKey &k) const
	{
		uint64_t h = static_cast<uint64_t>(static_cast<uint32_t>(k.x)) * 73856093ull;
		h ^= static_cast<uint64_t>(static_cast<uint32_t>(k.y)) * 19349663ull;
		h ^= static_cast<uint64_t>(static_cast<uint32_t>(k.z)) * 83492791ull;
		return static_cast<size_t>(h ^ (h >> 32));
	}
};

class PointGrid
{
public:

	PointGrid(const std::vector<Vec3> &points, double cell)
		: m_points(points), m_cell(cell)
	{
		for (size_t i = 0; i < points.size(); ++i)
			m_cells[KeyFor(points[i])].push_back(i);
	}

	void Remove(size_t index)
	{
		std::vector<size_t> &bucket = m_cells[KeyFor(m_points[index])];
		bucket.erase(std::remove(bucket.begin(), bucket.end(), index), bucket.end());
	}

	void FindNearest(const Vec3 &p, size_t skip, size_t k, size_t aliveCount,
		std::vector<size_t> &out) const
	{
		out.clear();
		if (k == 0)
			return;

		std::vector<std::pair<double, size_t>> best;
		PointCellKey center = KeyFor(p);

		size_t reachable = (skip < m_points.size()) ? aliveCount - 1 : aliveCount;
		size_t want = std::min(k, reachable);
		if (want == 0)
			return;

		for (int r = 0;; ++r)
		{
			GatherRing(center, r, p, skip, best);

			std::sort(best.begin(), best.end());
			if (best.size() > k)
				best.resize(k);

			if (best.size() >= want)
			{
				double guaranteed = static_cast<double>(r) * m_cell;
				if (best[want - 1].first <= guaranteed * guaranteed)
					break;
			}

			if (r > m_maxRing)
				break;
		}

		for (const std::pair<double, size_t> &e : best)
			out.push_back(e.second);
	}

	void FindNearestBrute(const Vec3 &p, size_t skip, size_t k, const std::vector<bool> &alive,
		std::vector<size_t> &out) const
	{
		out.clear();
		std::vector<std::pair<double, size_t>> best;
		for (size_t i = 0; i < m_points.size(); ++i)
		{
			if (i == skip || !alive[i])
				continue;
			best.push_back({DistSq(p, m_points[i]), i});
		}
		std::sort(best.begin(), best.end());
		if (best.size() > k)
			best.resize(k);
		for (const std::pair<double, size_t> &e : best)
			out.push_back(e.second);
	}

	void FindWithin(const Vec3 &p, double radius, std::vector<size_t> &out) const
	{
		out.clear();
		if (!(radius > 0.0))
			return;

		double radiusSq = radius * radius;
		int span = static_cast<int>(std::floor(radius / m_cell)) + 1;
		PointCellKey center = KeyFor(p);

		for (int dz = -span; dz <= span; ++dz)
		{
			for (int dy = -span; dy <= span; ++dy)
			{
				for (int dx = -span; dx <= span; ++dx)
				{
					PointCellKey key;
					key.x = center.x + dx;
					key.y = center.y + dy;
					key.z = center.z + dz;

					std::unordered_map<PointCellKey, std::vector<size_t>, PointCellKeyHash>::const_iterator it =
						m_cells.find(key);
					if (it == m_cells.end())
						continue;

					for (size_t idx : it->second)
					{
						if (DistSq(p, m_points[idx]) <= radiusSq)
							out.push_back(idx);
					}
				}
			}
		}

		std::sort(out.begin(), out.end());
	}

	void SetExtent(const Vec3 &mins, const Vec3 &maxs)
	{
		double span = std::max(maxs.x - mins.x, std::max(maxs.y - mins.y, maxs.z - mins.z));
		m_maxRing = static_cast<int>(std::ceil(span / m_cell)) + 2;
	}

private:

	static double DistSq(const Vec3 &a, const Vec3 &b)
	{
		double dx = a.x - b.x, dy = a.y - b.y, dz = a.z - b.z;
		return dx * dx + dy * dy + dz * dz;
	}

	PointCellKey KeyFor(const Vec3 &p) const
	{
		PointCellKey k;
		k.x = static_cast<int>(std::floor(p.x / m_cell));
		k.y = static_cast<int>(std::floor(p.y / m_cell));
		k.z = static_cast<int>(std::floor(p.z / m_cell));
		return k;
	}

	void GatherCell(const PointCellKey &key, const Vec3 &p, size_t skip,
		std::vector<std::pair<double, size_t>> &best) const
	{
		std::unordered_map<PointCellKey, std::vector<size_t>, PointCellKeyHash>::const_iterator it =
			m_cells.find(key);
		if (it == m_cells.end())
			return;

		for (size_t idx : it->second)
		{
			if (idx == skip)
				continue;
			best.push_back({DistSq(p, m_points[idx]), idx});
		}
	}

	void GatherRing(const PointCellKey &center, int r, const Vec3 &p, size_t skip,
		std::vector<std::pair<double, size_t>> &best) const
	{
		if (r == 0)
		{
			GatherCell(center, p, skip, best);
			return;
		}

		for (int dz = -r; dz <= r; ++dz)
		{
			for (int dy = -r; dy <= r; ++dy)
			{
				for (int dx = -r; dx <= r; ++dx)
				{
					if (std::abs(dx) != r && std::abs(dy) != r && std::abs(dz) != r)
						continue;
					PointCellKey key;
					key.x = center.x + dx;
					key.y = center.y + dy;
					key.z = center.z + dz;
					GatherCell(key, p, skip, best);
				}
			}
		}
	}

	const std::vector<Vec3> &m_points;
	double m_cell;
	int m_maxRing = 64;
	std::unordered_map<PointCellKey, std::vector<size_t>, PointCellKeyHash> m_cells;
};

}
