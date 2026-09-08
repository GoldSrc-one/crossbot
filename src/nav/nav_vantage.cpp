#include "nav_vantage.h"

#include <algorithm>
#include <cmath>

#include "nav_math.h"

namespace Nav
{

namespace
{

using NavMath::kPiF;

constexpr float kDeg = kPiF / 180.f;
constexpr float kMinCellSize = 32.f;
constexpr int kMaxGridDim = 128;

struct Candidate
{
	uint32_t index;
	float distSq;
};

struct MergeEntry
{
	Vector dir;
	float weight;
	float spread;
	bool used;
};

int CellAxis(const VantageField &field, float value, int axis)
{
	float base = (axis == 0) ? field.gridMins.x : (axis == 1) ? field.gridMins.y : field.gridMins.z;
	int c = static_cast<int>(std::floor((value - base) / field.cell));
	if (c < 0)
		c = 0;
	else if (c >= field.dims[axis])
		c = field.dims[axis] - 1;
	return c;
}

size_t CellIndex(const VantageField &field, int x, int y, int z)
{
	return (static_cast<size_t>(z) * static_cast<size_t>(field.dims[1]) +
		static_cast<size_t>(y)) * static_cast<size_t>(field.dims[0]) + static_cast<size_t>(x);
}

void ConsiderCell(const VantageField &field, int x, int y, int z, const Vector &eye,
	Candidate *best, int &found)
{
	size_t cell = CellIndex(field, x, y, z);
	uint32_t begin = field.cellStart[cell];
	uint32_t end = field.cellStart[cell + 1];

	for (uint32_t i = begin; i < end; ++i)
	{
		uint32_t index = field.cellItems[i];
		Vector d = field.vantages[index].pos - eye;
		float distSq = d.x * d.x + d.y * d.y + d.z * d.z;

		if (found < static_cast<int>(kVantageBlendNeighbors))
		{
			best[found].index = index;
			best[found].distSq = distSq;
			++found;
			for (int k = found - 1; k > 0 && best[k].distSq < best[k - 1].distSq; --k)
				std::swap(best[k], best[k - 1]);
			continue;
		}

		if (distSq >= best[kVantageBlendNeighbors - 1].distSq)
			continue;

		best[kVantageBlendNeighbors - 1].index = index;
		best[kVantageBlendNeighbors - 1].distSq = distSq;
		for (int k = static_cast<int>(kVantageBlendNeighbors) - 1;
			k > 0 && best[k].distSq < best[k - 1].distSq; --k)
			std::swap(best[k], best[k - 1]);
	}
}

int FindNearest(const VantageField &field, const Vector &eye, Candidate *best)
{
	int cx = CellAxis(field, eye.x, 0);
	int cy = CellAxis(field, eye.y, 1);
	int cz = CellAxis(field, eye.z, 2);

	int maxRing = std::max(std::max(field.dims[0], field.dims[1]), field.dims[2]);
	int found = 0;

	for (int ring = 0; ring <= maxRing; ++ring)
	{
		for (int z = cz - ring; z <= cz + ring; ++z)
		{
			if (z < 0 || z >= field.dims[2])
				continue;
			for (int y = cy - ring; y <= cy + ring; ++y)
			{
				if (y < 0 || y >= field.dims[1])
					continue;
				for (int x = cx - ring; x <= cx + ring; ++x)
				{
					if (x < 0 || x >= field.dims[0])
						continue;
					if (ring > 0 && std::abs(x - cx) != ring && std::abs(y - cy) != ring &&
						std::abs(z - cz) != ring)
						continue;

					ConsiderCell(field, x, y, z, eye, best, found);
				}
			}
		}

		if (found < static_cast<int>(kVantageBlendNeighbors))
			continue;

		float safe = static_cast<float>(ring) * field.cell;
		if (best[kVantageBlendNeighbors - 1].distSq <= safe * safe)
			break;
	}

	return found;
}

void MergeWatchDirs(MergeEntry *entries, int count, VantageBlend &out)
{
	out.watchCount = 0;

	float total = 0.f;
	for (int i = 0; i < count; ++i)
		total += entries[i].weight;
	if (total <= 0.f)
		return;

	float cosCone = std::cos(kVantageMergeCone * kDeg);

	while (out.watchCount < static_cast<int>(crossbot_nav::kNavWatchDirs))
	{
		int seed = -1;
		for (int i = 0; i < count; ++i)
		{
			if (entries[i].used)
				continue;
			if (seed < 0 || entries[i].weight > entries[seed].weight)
				seed = i;
		}
		if (seed < 0 || entries[seed].weight <= 0.f)
			break;

		Vector dir = entries[seed].dir;
		Vector acc(0.f, 0.f, 0.f);
		for (int i = 0; i < count; ++i)
		{
			if (entries[i].used || DotProduct(entries[i].dir, dir) < cosCone)
				continue;
			acc = acc + entries[i].dir * entries[i].weight;
		}
		float accLen = acc.Length();
		if (accLen > 0.0001f)
			dir = acc * (1.f / accLen);

		float weight = 0.f;
		float angleSq = 0.f;
		for (int i = 0; i < count; ++i)
		{
			if (entries[i].used)
				continue;
			float d = DotProduct(entries[i].dir, dir);
			if (d < cosCone)
				continue;

			float angle = std::acos(std::max(-1.f, std::min(1.f, d))) / kDeg;
			weight += entries[i].weight;
			angleSq += entries[i].weight * (angle * angle + entries[i].spread * entries[i].spread);
			entries[i].used = true;
		}
		if (weight <= 0.f)
			break;

		VantageWatch &wd = out.watch[out.watchCount++];
		float flat = std::sqrt(dir.x * dir.x + dir.y * dir.y);
		wd.yaw = atan2f(dir.y, dir.x) / kDeg;
		wd.pitch = -atan2f(dir.z, flat) / kDeg;
		wd.spread = std::sqrt(angleSq / weight);
		wd.amount = weight;
	}

	std::sort(out.watch, out.watch + out.watchCount,
		[](const VantageWatch &a, const VantageWatch &b) { return a.amount > b.amount; });

	float kept = 0.f;
	for (int i = 0; i < out.watchCount; ++i)
		kept += out.watch[i].amount;
	if (kept <= 0.f)
	{
		out.watchCount = 0;
		return;
	}

	while (out.watchCount > 1 && out.watch[out.watchCount - 1].amount < kVantageMinAmount * kept)
		--out.watchCount;

	kept = 0.f;
	for (int i = 0; i < out.watchCount; ++i)
		kept += out.watch[i].amount;
	for (int i = 0; i < out.watchCount; ++i)
		out.watch[i].amount /= kept;
}

}

Vector VantageDir(float yaw, float pitch)
{
	float cp = cosf(pitch * kDeg);
	return Vector(cosf(yaw * kDeg) * cp, sinf(yaw * kDeg) * cp, -sinf(pitch * kDeg));
}

void BuildVantageGrid(VantageField &field)
{
	field.cellStart.clear();
	field.cellItems.clear();
	field.dims[0] = field.dims[1] = field.dims[2] = 0;

	if (field.vantages.empty())
		return;

	Vector mins = field.vantages[0].pos;
	Vector maxs = mins;
	for (const Vantage &v : field.vantages)
	{
		mins.x = std::min(mins.x, v.pos.x);
		mins.y = std::min(mins.y, v.pos.y);
		mins.z = std::min(mins.z, v.pos.z);
		maxs.x = std::max(maxs.x, v.pos.x);
		maxs.y = std::max(maxs.y, v.pos.y);
		maxs.z = std::max(maxs.z, v.pos.z);
	}

	field.gridMins = mins;
	field.cell = std::max(field.spacing, kMinCellSize);

	for (int pass = 0; pass < 16; ++pass)
	{
		for (int axis = 0; axis < 3; ++axis)
		{
			float span = (axis == 0) ? maxs.x - mins.x
				: (axis == 1) ? maxs.y - mins.y : maxs.z - mins.z;
			int n = static_cast<int>(std::floor(span / field.cell)) + 1;
			field.dims[axis] = std::max(1, std::min(n, kMaxGridDim));
		}

		double cells = static_cast<double>(field.dims[0]) * field.dims[1] * field.dims[2];
		if (cells <= static_cast<double>(field.vantages.size()) * 64.0 + 4096.0)
			break;

		field.cell *= 2.f;
	}

	size_t cells = static_cast<size_t>(field.dims[0]) * static_cast<size_t>(field.dims[1]) *
		static_cast<size_t>(field.dims[2]);
	field.cellStart.assign(cells + 1, 0);

	std::vector<size_t> where(field.vantages.size());
	for (size_t i = 0; i < field.vantages.size(); ++i)
	{
		const Vector &p = field.vantages[i].pos;
		where[i] = CellIndex(field, CellAxis(field, p.x, 0), CellAxis(field, p.y, 1),
			CellAxis(field, p.z, 2));
		++field.cellStart[where[i] + 1];
	}

	for (size_t i = 0; i < cells; ++i)
		field.cellStart[i + 1] += field.cellStart[i];

	std::vector<uint32_t> cursor(field.cellStart.begin(), field.cellStart.end() - 1);
	field.cellItems.resize(field.vantages.size());
	for (size_t i = 0; i < field.vantages.size(); ++i)
		field.cellItems[cursor[where[i]]++] = static_cast<uint32_t>(i);
}

bool BlendVantage(const VantageField &field, const Vector &eye, VantageBlend &out)
{
	out = VantageBlend();

	if (field.vantages.empty() || field.cellStart.empty())
		return false;

	Candidate best[kVantageBlendNeighbors];
	int found = FindNearest(field, eye, best);
	if (found <= 0)
		return false;

	MergeEntry entries[kVantageBlendNeighbors * crossbot_nav::kNavWatchDirs];
	int entryCount = 0;
	float wsum = 0.f;

	for (int n = 0; n < found; ++n)
	{
		const Vantage &v = field.vantages[best[n].index];
		float w = 1.f / (best[n].distSq + kVantageIdwEpsilon);
		wsum += w;

		out.exposure += w * v.exposure;

		for (int k = 0; k < v.watchCount; ++k)
		{
			if (v.watch[k].amount <= 0.f)
				continue;
			MergeEntry &e = entries[entryCount++];
			e.dir = VantageDir(v.watch[k].yaw, v.watch[k].pitch);
			e.weight = w * v.watch[k].amount;
			e.spread = v.watch[k].spread;
			e.used = false;
		}
	}

	if (wsum <= 0.f)
		return false;

	out.exposure /= wsum;

	MergeWatchDirs(entries, entryCount, out);

	out.neighbors = found;
	out.nearestDist = std::sqrt(best[0].distSq);
	out.valid = out.watchCount > 0;
	return out.valid;
}

}
