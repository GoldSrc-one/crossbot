#include "vantage_sampler.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <queue>
#include <unordered_map>

#include "clipnode_tree.h"
#include "floor_support.h"
#include "hull_defs.h"
#include "point_grid.h"
#include "vis_sampling.h"
#include "work_pool.h"

namespace navgen
{

namespace
{

constexpr double kColumnMergeZ = 32.0;
constexpr double kIdwEpsilon = 1.0;
constexpr double kExposureErrorWeight = 2.0;
constexpr double kVantagePvsCost = 1.0;
constexpr double kVantageTraceCost = 18.0;
constexpr size_t kVantageTileSlots = 256;

struct ColumnKey
{
	int x = 0, y = 0;

	bool operator==(const ColumnKey &o) const { return x == o.x && y == o.y; }
};

struct ColumnKeyHash
{
	size_t operator()(const ColumnKey &k) const
	{
		uint64_t h = static_cast<uint64_t>(static_cast<uint32_t>(k.x)) * 73856093ull;
		h ^= static_cast<uint64_t>(static_cast<uint32_t>(k.y)) * 19349663ull;
		return static_cast<size_t>(h ^ (h >> 32));
	}
};

void CollectCandidates(const std::vector<Area> &areas, const VantageParams &params,
	std::vector<Vantage> &out, VantageStats &stats)
{
	std::unordered_map<ColumnKey, std::vector<size_t>, ColumnKeyHash> columns;

	for (const Area &area : areas)
	{
		bool crouch = (area.areaFlags & crossbot_nav::NAV_AREA_CROUCH) != 0;
		double lift = crouch ? kDuckEyeLift : (kDuckEyeLift + kStandEyeLift) * 0.5;

		double minX = 1e30, maxX = -1e30, minY = 1e30, maxY = -1e30;
		for (const Face &f : area.polytope.faces)
		{
			if (!IsFloorFace(f) || f.winding.size() < 3)
				continue;
			for (const Vec3 &v : f.winding)
			{
				minX = std::min(minX, v.x);
				maxX = std::max(maxX, v.x);
				minY = std::min(minY, v.y);
				maxY = std::max(maxY, v.y);
			}
		}

		if (minX > maxX)
			continue;

		int gx0 = static_cast<int>(std::ceil(minX / params.gridSize));
		int gx1 = static_cast<int>(std::floor(maxX / params.gridSize));
		int gy0 = static_cast<int>(std::ceil(minY / params.gridSize));
		int gy1 = static_cast<int>(std::floor(maxY / params.gridSize));

		for (int gy = gy0; gy <= gy1; ++gy)
		{
			for (int gx = gx0; gx <= gx1; ++gx)
			{
				double x = gx * params.gridSize;
				double y = gy * params.gridSize;
				++stats.latticePoints;

				for (const Face &f : area.polytope.faces)
				{
					if (!IsFloorFace(f) || f.winding.size() < 3)
						continue;
					if (Distance2DToWinding(f.winding, x, y) > 0.0)
						continue;

					double z = FloorHeightAt(f, x, y) + lift;

					ColumnKey key;
					key.x = gx;
					key.y = gy;
					std::vector<size_t> &column = columns[key];

					bool duplicate = false;
					for (size_t existing : column)
					{
						if (std::fabs(out[existing].pos.z - z) < kColumnMergeZ)
						{
							duplicate = true;
							break;
						}
					}
					if (duplicate)
						continue;

					Vantage v;
					v.pos = Vec3(x, y, z);
					v.areaId = area.id;
					v.floorZ = z - lift;
					v.areaFlags = area.areaFlags;
					column.push_back(out.size());
					out.push_back(v);
				}
			}
		}
	}
}

void ExtractWatchDirs(const std::vector<double> &hist, const std::vector<double> &distHist,
	const BinLayout &bins, const VantageParams &params, Vantage &vantage)
{
	double total = 0.0;
	for (double w : hist)
		total += w;
	if (total <= 0.0)
		return;

	std::vector<double> work = hist;
	double cosCone = std::cos(params.clusterConeDeg * kVisDeg);
	double remaining = total;

	for (size_t k = 0; k < params.maxWatchDirs; ++k)
	{
		if (remaining <= params.minClusterFraction * total)
			break;

		size_t peak = 0;
		double peakWeight = 0.0;
		for (size_t b = 0; b < work.size(); ++b)
		{
			if (work[b] > peakWeight)
			{
				peakWeight = work[b];
				peak = b;
			}
		}
		if (peakWeight <= 0.0)
			break;

		Vec3 dir = bins.Dir(peak);
		for (int pass = 0; pass < 3; ++pass)
		{
			Vec3 acc;
			for (size_t b = 0; b < work.size(); ++b)
			{
				if (work[b] <= 0.0)
					continue;
				if (bins.Dir(b).Dot(dir) < cosCone)
					continue;
				acc = acc + bins.Dir(b) * work[b];
			}
			if (acc.Length() < 1e-9)
				break;
			dir = acc.Normalized();
		}

		double weight = 0.0;
		double angleSq = 0.0;
		double distAcc = 0.0;
		for (size_t b = 0; b < work.size(); ++b)
		{
			if (work[b] <= 0.0)
				continue;
			double d = bins.Dir(b).Dot(dir);
			if (d < cosCone)
				continue;
			double angle = std::acos(std::max(-1.0, std::min(1.0, d))) / kVisDeg;
			weight += work[b];
			angleSq += work[b] * angle * angle;
			distAcc += distHist[b];
			work[b] = 0.0;
		}

		if (weight <= 0.0)
			break;

		WatchDir wd;
		AnglesFromDir(dir, wd.yaw, wd.pitch);
		wd.spread = std::sqrt(angleSq / weight);
		wd.amount = weight / total;
		wd.dist = distAcc / weight;
		vantage.watch.push_back(wd);

		remaining -= weight;
	}

	std::sort(vantage.watch.begin(), vantage.watch.end(),
		[](const WatchDir &a, const WatchDir &b) { return a.amount > b.amount; });
}

double InterpolationError(const std::vector<std::vector<double>> &shape,
	const std::vector<double> &exposure, size_t index, const std::vector<size_t> &neighbors,
	const std::vector<Vec3> &points)
{
	if (neighbors.empty())
		return 1e30;

	std::vector<double> est(shape[index].size(), 0.0);
	double estExposure = 0.0;
	double wsum = 0.0;
	for (size_t n : neighbors)
	{
		double dx = points[index].x - points[n].x;
		double dy = points[index].y - points[n].y;
		double dz = points[index].z - points[n].z;
		double w = 1.0 / (dx * dx + dy * dy + dz * dz + kIdwEpsilon);
		wsum += w;
		estExposure += w * exposure[n];
		for (size_t b = 0; b < est.size(); ++b)
			est[b] += w * shape[n][b];
	}

	if (wsum <= 0.0)
		return 1e30;

	double err = 0.0;
	for (size_t b = 0; b < est.size(); ++b)
		err += std::fabs(shape[index][b] - est[b] / wsum);

	err += kExposureErrorWeight * std::fabs(exposure[index] - estExposure / wsum);

	return err;
}

std::vector<size_t> HullCornersXY(const std::vector<Vec3> &pts)
{
	size_t n = pts.size();
	std::vector<size_t> idx(n);
	for (size_t i = 0; i < n; ++i)
		idx[i] = i;
	std::sort(idx.begin(), idx.end(), [&](size_t a, size_t b)
	{
		if (pts[a].x != pts[b].x)
			return pts[a].x < pts[b].x;
		return pts[a].y < pts[b].y;
	});
	std::vector<size_t> hull;
	hull.reserve(n);
	for (int pass = 0; pass < 2; ++pass)
	{
		size_t base = hull.size();
		for (size_t k = 0; k < n; ++k)
		{
			size_t i = (pass == 0) ? idx[k] : idx[n - 1 - k];
			while (hull.size() >= base + 2)
			{
				size_t b = hull[hull.size() - 2];
				size_t a = hull[hull.size() - 1];
				double cross = (pts[a].x - pts[b].x) * (pts[i].y - pts[b].y) -
					(pts[a].y - pts[b].y) * (pts[i].x - pts[b].x);
				if (cross > 0.0)
					break;
				hull.pop_back();
			}
			hull.push_back(i);
		}
		if (!hull.empty())
			hull.pop_back();
	}
	if (hull.empty() && n > 0)
		hull.push_back(idx[0]);
	return hull;
}

}

std::vector<Vantage> BuildVantages(const BspFile &bsp, const VisIndex &vis,
	const std::vector<BlockingVolume> &visBlockers, const std::vector<Area> &areas,
	const VantageParams &params, VantageStats *statsOut,
	std::vector<VantageCandidate> *candidatesOut)
{
	VantageStats stats;
	std::chrono::steady_clock::time_point sampleStart = std::chrono::steady_clock::now();

	std::vector<Vantage> vantages;
	CollectCandidates(areas, params, vantages, stats);

	for (const Area &area : areas)
		stats.footprintArea += WindingAreaXY(WalkableFootprint(area.polytope));

	HullTree hull = vis.Hull0();
	int worldHeadnode = bsp.Models()[0].headnode[kHullPoint];

	stats.candidateSeconds = std::chrono::duration<double>(
		std::chrono::steady_clock::now() - sampleStart).count();

	std::chrono::steady_clock::time_point leafStart = std::chrono::steady_clock::now();

	std::vector<Vantage> solidFiltered;
	solidFiltered.reserve(vantages.size());
	for (Vantage &v : vantages)
	{
		if (ClassifyPoint(hull, worldHeadnode, v.pos) == CONTENTS_SOLID)
		{
			++stats.solidRejected;
			continue;
		}
		v.leaf = vis.LeafForPoint(v.pos);
		solidFiltered.push_back(v);
	}
	vantages.swap(solidFiltered);
	stats.sampled = vantages.size();

	stats.leafSeconds = std::chrono::duration<double>(
		std::chrono::steady_clock::now() - leafStart).count();

	std::vector<VisBlockerBox> blockers = BuildVisBlockerBoxes(visBlockers);

	size_t count = vantages.size();
	BinLayout bins(params.yawBins, params.pitchBins, params.pitchRangeDeg);
	size_t binCount = bins.Count();

	std::vector<std::atomic<uint32_t>> tally(count * binCount);
	for (std::atomic<uint32_t> &slot : tally)
		slot.store(0, std::memory_order_relaxed);

	// Sightline lengths, summed per bin alongside the counts. Freed with the tally,
	// so the extra footprint never overlaps the shape vectors built further down.
	std::vector<std::atomic<uint32_t>> distTally(count * binCount);
	for (std::atomic<uint32_t> &slot : distTally)
		slot.store(0, std::memory_order_relaxed);

	auto recordVisible = [&](size_t i, size_t j)
	{
		Vec3 d = vantages[j].pos - vantages[i].pos;
		double yaw = 0.0, pitch = 0.0;
		AnglesFromDir(d, yaw, pitch);
		uint32_t len = static_cast<uint32_t>(d.Length());
		size_t forward = i * binCount + bins.BinFor(yaw, pitch);
		size_t backward = j * binCount + bins.BinFor(yaw + 180.0, -pitch);
		tally[forward].fetch_add(1, std::memory_order_relaxed);
		tally[backward].fetch_add(1, std::memory_order_relaxed);
		distTally[forward].fetch_add(len, std::memory_order_relaxed);
		distTally[backward].fetch_add(len, std::memory_order_relaxed);
	};

	std::vector<size_t> order(count);
	for (size_t i = 0; i < count; ++i)
		order[i] = i;
	std::sort(order.begin(), order.end(),
		[&](size_t a, size_t b) { return vantages[a].leaf < vantages[b].leaf; });

	stats.totalPairs = count * (count > 0 ? count - 1 : 0) / 2;

	size_t threads = (params.threadCount > 0) ? params.threadCount : DefaultThreadCount();
	threads = std::min(threads, std::max<size_t>(1, count));
	stats.threads = threads;

	std::vector<size_t> slots(count);
	std::vector<double> slotCost(count, 0.0);
	std::vector<int> leafIds;
	std::vector<size_t> leafBegin;
	std::vector<size_t> leafEnd;
	{
		for (size_t a = 0; a < count; ++a)
		{
			slots[a] = a;

			int leaf = vantages[order[a]].leaf;
			if (leafIds.empty() || leafIds.back() != leaf)
			{
				leafIds.push_back(leaf);
				leafBegin.push_back(a);
				leafEnd.push_back(a);
			}
			leafEnd.back() = a + 1;
		}

		VisRowReader reader(vis);
		for (size_t l = 0; l < leafIds.size(); ++l)
		{
			const uint8_t *row = reader.VisRow(leafIds[l]);

			double after = 0.0;
			for (size_t m = l + 1; m < leafIds.size(); ++m)
			{
				if (vis.RowVisible(row, leafIds[m]))
					after += static_cast<double>(leafEnd[m] - leafBegin[m]);
			}

			bool selfVisible = vis.RowVisible(row, leafIds[l]);
			for (size_t a = leafBegin[l]; a < leafEnd[l]; ++a)
			{
				double traced = after;
				if (selfVisible)
					traced += static_cast<double>(leafEnd[l] - a - 1);

				slotCost[a] = static_cast<double>(count - 1 - a) * kVantagePvsCost +
					traced * kVantageTraceCost;
			}
		}
	}

	std::vector<std::vector<size_t>> workBins =
		BuildWorkBins(slotCost, slots, kVantageTileSlots, threads);

	std::chrono::steady_clock::time_point pairStart = std::chrono::steady_clock::now();

	if (params.groupCull)
	{
	std::chrono::steady_clock::time_point groupStart = std::chrono::steady_clock::now();
	stats.threads = 1;
	VisRowReader groupReader(vis);
	TraceCounters groupCounters;
	HullOriginChain groupChain;
	std::vector<Vec3> groupPts;
	std::vector<size_t> hullL;
	std::vector<size_t> hullM;
	for (size_t l = 0; l < leafIds.size(); ++l)
	{
		size_t beginL = leafBegin[l];
		size_t endL = leafEnd[l];
		const uint8_t *rowL = groupReader.VisRow(leafIds[l]);
		if (vis.RowVisible(rowL, leafIds[l]))
		{
			for (size_t a = beginL; a < endL; ++a)
			{
				size_t i = order[a];
				for (size_t b = a + 1; b < endL; ++b)
				{
					size_t j = order[b];
					++stats.pvsPairs;
					if (Occluded(hull, worldHeadnode, blockers, vantages[i].pos,
						vantages[j].pos, groupCounters, groupChain))
						continue;
					++stats.visiblePairs;
					recordVisible(i, j);
				}
			}
		}
		for (size_t m = l + 1; m < leafIds.size(); ++m)
		{
			++stats.groupPairs;
			if (rowL && !vis.RowVisible(rowL, leafIds[m]))
				continue;
			size_t beginM = leafBegin[m];
			size_t endM = leafEnd[m];
			size_t nL = endL - beginL;
			size_t nM = endM - beginM;
			size_t pairs = nL * nM;
			stats.pvsPairs += pairs;
			groupPts.clear();
			for (size_t a = beginL; a < endL; ++a)
				groupPts.push_back(vantages[order[a]].pos);
			hullL = HullCornersXY(groupPts);
			groupPts.clear();
			for (size_t b = beginM; b < endM; ++b)
				groupPts.push_back(vantages[order[b]].pos);
			hullM = HullCornersXY(groupPts);
			bool tiny = hullL.size() * hullM.size() * 2 >= pairs;
			size_t openCorners = 0;
			size_t cornerTests = 0;
			if (!tiny)
			{
				for (size_t cl : hullL)
				{
					for (size_t cm : hullM)
					{
						size_t i = order[beginL + cl];
						size_t j = order[beginM + cm];
						++cornerTests;
						if (!Occluded(hull, worldHeadnode, blockers, vantages[i].pos,
							vantages[j].pos, groupCounters, groupChain))
							++openCorners;
					}
				}
				stats.cornerTraces += cornerTests;
			}
			bool expectOpen = false;
			if (!tiny && openCorners == cornerTests)
			{
				++stats.groupOpen;
				expectOpen = true;
				for (size_t a = beginL; a < endL; ++a)
				{
					for (size_t b = beginM; b < endM; ++b)
					{
						++stats.visiblePairs;
						recordVisible(order[a], order[b]);
					}
				}
			}
			else if (!tiny && openCorners == 0)
			{
				++stats.groupBlocked;
			}
			else
			{
				++stats.groupMixed;
				for (size_t a = beginL; a < endL; ++a)
				{
					size_t i = order[a];
					for (size_t b = beginM; b < endM; ++b)
					{
						size_t j = order[b];
						if (Occluded(hull, worldHeadnode, blockers, vantages[i].pos,
							vantages[j].pos, groupCounters, groupChain))
							continue;
						++stats.visiblePairs;
						recordVisible(i, j);
					}
				}
				continue;
			}
			uint32_t rng = static_cast<uint32_t>(l * 2654435761ull + m * 40503ull + 1ull);
			size_t checks = std::min<size_t>(32, pairs);
			for (size_t k = 0; k < checks; ++k)
			{
				rng = rng * 1664525u + 1013904223u;
				size_t a = beginL + static_cast<size_t>(rng >> 8) % nL;
				rng = rng * 1664525u + 1013904223u;
				size_t b = beginM + static_cast<size_t>(rng >> 8) % nM;
				++stats.verifyChecks;
				bool blocked = Occluded(hull, worldHeadnode, blockers,
					vantages[order[a]].pos, vantages[order[b]].pos, groupCounters, groupChain);
				if (expectOpen && blocked)
					++stats.verifyFalseVisible;
				if (!expectOpen && !blocked)
					++stats.verifyFalseBlocked;
			}
		}
	}
	stats.worldTraces = groupCounters.worldTraces;
	stats.blockerTraces = groupCounters.blockerTraces;
	double groupWall = std::chrono::duration<double>(
		std::chrono::steady_clock::now() - groupStart).count();
	stats.traceSeconds = groupWall;
	stats.binWallMin = groupWall;
	stats.binWallMax = groupWall;
	}
	else
	{
	std::vector<VantageStats> binStats(threads);
	std::vector<double> binWall(threads, 0.0);

	RunBins(threads, [&](size_t binIndex)
	{
		std::chrono::steady_clock::time_point binStart = std::chrono::steady_clock::now();
		VantageStats &local = binStats[binIndex];
		VisRowReader reader(vis);
		TraceCounters counters;
		HullOriginChain chain;
		double traceTicks = 0.0;

		for (size_t a : workBins[binIndex])
		{
			size_t i = order[a];
			const uint8_t *row = reader.VisRow(vantages[i].leaf);
			for (size_t b = a + 1; b < count; ++b)
			{
				size_t j = order[b];
				if (row && !vis.RowVisible(row, vantages[j].leaf))
					continue;

				++local.pvsPairs;

				std::chrono::steady_clock::time_point traceStart = std::chrono::steady_clock::now();
				bool blocked = Occluded(hull, worldHeadnode, blockers, vantages[i].pos, vantages[j].pos,
					counters, chain);
				traceTicks += std::chrono::duration<double>(
					std::chrono::steady_clock::now() - traceStart).count();

				if (blocked)
					continue;

				++local.visiblePairs;
				recordVisible(i, j);
			}
		}

		local.worldTraces = counters.worldTraces;
		local.blockerTraces = counters.blockerTraces;
		local.traceSeconds = traceTicks;
		binWall[binIndex] = std::chrono::duration<double>(
			std::chrono::steady_clock::now() - binStart).count();
	});

	for (size_t b = 0; b < threads; ++b)
	{
		stats.pvsPairs += binStats[b].pvsPairs;
		stats.visiblePairs += binStats[b].visiblePairs;
		stats.worldTraces += binStats[b].worldTraces;
		stats.blockerTraces += binStats[b].blockerTraces;
		stats.traceSeconds += binStats[b].traceSeconds;
	}

	stats.binWallMin = binWall.empty() ? 0.0 : *std::min_element(binWall.begin(), binWall.end());
	stats.binWallMax = binWall.empty() ? 0.0 : *std::max_element(binWall.begin(), binWall.end());
	}

	stats.pairSeconds = std::chrono::duration<double>(
		std::chrono::steady_clock::now() - pairStart).count();

	std::vector<std::vector<double>> hist(count);
	std::vector<std::vector<double>> distHist(count);
	for (size_t i = 0; i < count; ++i)
	{
		hist[i].resize(binCount, 0.0);
		distHist[i].resize(binCount, 0.0);
		for (size_t b = 0; b < binCount; ++b)
		{
			hist[i][b] = static_cast<double>(tally[i * binCount + b].load(std::memory_order_relaxed));
			distHist[i][b] =
				static_cast<double>(distTally[i * binCount + b].load(std::memory_order_relaxed));
		}
	}

	tally = std::vector<std::atomic<uint32_t>>();
	distTally = std::vector<std::atomic<uint32_t>>();

	std::chrono::steady_clock::time_point deriveStart = std::chrono::steady_clock::now();

	double denom = (count > 1) ? static_cast<double>(count - 1) : 1.0;
	for (size_t i = 0; i < count; ++i)
	{
		double sum = 0.0;
		for (double w : hist[i])
			sum += w;
		vantages[i].exposure = sum / denom;
		stats.maxExposure = std::max(stats.maxExposure, vantages[i].exposure);
		ExtractWatchDirs(hist[i], distHist[i], bins, params, vantages[i]);
	}

	distHist.clear();
	distHist.shrink_to_fit();

	stats.deriveSeconds = std::chrono::duration<double>(
		std::chrono::steady_clock::now() - deriveStart).count();

	if (candidatesOut)
	{
		candidatesOut->resize(count);
		for (size_t i = 0; i < count; ++i)
		{
			const Vantage &v = vantages[i];
			VantageCandidate &c = (*candidatesOut)[i];
			c.eye = v.pos;
			c.floor = Vec3(v.pos.x, v.pos.y, v.floorZ);
			c.areaId = v.areaId;
			c.leaf = v.leaf;
			c.areaFlags = v.areaFlags;
			c.exposure = v.exposure;
			c.mainAmount = v.watch.empty() ? 0.0 : v.watch[0].amount;
		}
	}

	std::chrono::steady_clock::time_point reduceStart = std::chrono::steady_clock::now();

	std::vector<std::vector<double>> shape(count);
	std::vector<double> exposureNorm(count, 0.0);
	double exposureScale = 1.0 / std::max(1e-9, stats.maxExposure);
	for (size_t i = 0; i < count; ++i)
	{
		shape[i].resize(bins.Count(), 0.0);
		double sum = 0.0;
		for (double w : hist[i])
			sum += w;
		if (sum > 0.0)
		{
			for (size_t b = 0; b < bins.Count(); ++b)
				shape[i][b] = hist[i][b] / sum;
		}
		exposureNorm[i] = vantages[i].exposure * exposureScale;
	}

	hist.clear();
	hist.shrink_to_fit();

	std::vector<Vec3> points(count);
	Vec3 extentMins(1e30, 1e30, 1e30), extentMaxs(-1e30, -1e30, -1e30);
	for (size_t i = 0; i < count; ++i)
	{
		points[i] = vantages[i].pos;
		extentMins.x = std::min(extentMins.x, points[i].x);
		extentMins.y = std::min(extentMins.y, points[i].y);
		extentMins.z = std::min(extentMins.z, points[i].z);
		extentMaxs.x = std::max(extentMaxs.x, points[i].x);
		extentMaxs.y = std::max(extentMaxs.y, points[i].y);
		extentMaxs.z = std::max(extentMaxs.z, points[i].z);
	}

	double cell = std::max(32.0, params.gridSize * 2.0);
	PointGrid grid(points, cell);
	if (count > 0)
		grid.SetExtent(extentMins, extentMaxs);

	size_t budget = static_cast<size_t>(stats.footprintArea /
		(params.targetSpacing * params.targetSpacing));
	budget = std::max(budget, params.neighborCount + 1);
	stats.budget = budget;

	std::vector<bool> alive(count, true);
	size_t aliveCount = count;

	std::vector<uint32_t> version(count, 0);
	std::vector<size_t> neighbors;
	std::vector<size_t> brute;

	struct QueueEntry
	{
		double err;
		size_t index;
		uint32_t version;

		bool operator>(const QueueEntry &o) const { return err > o.err; }
	};

	std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<QueueEntry>> queue;

	auto Evaluate = [&](size_t i)
	{
		grid.FindNearest(points[i], i, params.neighborCount, aliveCount, neighbors);

		if (params.verifyNeighbors)
		{
			std::chrono::steady_clock::time_point verifyStart = std::chrono::steady_clock::now();
			grid.FindNearestBrute(points[i], i, params.neighborCount, alive, brute);
			std::vector<size_t> sortedA = neighbors;
			std::vector<size_t> sortedB = brute;
			std::sort(sortedA.begin(), sortedA.end());
			std::sort(sortedB.begin(), sortedB.end());
			++stats.neighborChecks;
			if (sortedA != sortedB)
				++stats.neighborMismatches;
			stats.verifySeconds += std::chrono::duration<double>(
				std::chrono::steady_clock::now() - verifyStart).count();
		}

		double err = InterpolationError(shape, exposureNorm, i, neighbors, points);
		queue.push({err, i, version[i]});
	};

	if (aliveCount > budget)
	{
		for (size_t i = 0; i < count; ++i)
			Evaluate(i);
	}

	double errSum = 0.0;
	size_t removed = 0;

	while (aliveCount > budget && !queue.empty())
	{
		QueueEntry top = queue.top();
		queue.pop();

		if (!alive[top.index] || top.version != version[top.index])
			continue;

		grid.FindNearest(points[top.index], top.index, params.neighborCount, aliveCount, neighbors);
		double fresh = InterpolationError(shape, exposureNorm, top.index, neighbors, points);
		if (fresh > top.err + 1e-12)
		{
			++version[top.index];
			queue.push({fresh, top.index, version[top.index]});
			continue;
		}

		alive[top.index] = false;
		grid.Remove(top.index);
		--aliveCount;
		++removed;
		errSum += fresh;
		stats.maxReductionError = std::max(stats.maxReductionError, fresh);

		std::vector<size_t> stale = neighbors;
		for (size_t n : stale)
		{
			if (!alive[n])
				continue;
			++version[n];
			Evaluate(n);
		}
	}

	if (removed > 0)
		stats.meanReductionError = errSum / static_cast<double>(removed);

	std::vector<Vantage> kept;
	kept.reserve(aliveCount);
	for (size_t i = 0; i < count; ++i)
	{
		if (alive[i])
			kept.push_back(vantages[i]);
	}

	stats.kept = kept.size();

	double distSum = 0.0;
	size_t distCount = 0;
	for (const Vantage &v : kept)
	{
		for (const WatchDir &wd : v.watch)
		{
			distSum += wd.dist;
			++distCount;
			stats.maxWatchDist = std::max(stats.maxWatchDist, wd.dist);
		}
	}
	stats.meanWatchDist = (distCount > 0) ? distSum / static_cast<double>(distCount) : 0.0;
	stats.reduceSeconds = std::chrono::duration<double>(
		std::chrono::steady_clock::now() - reduceStart).count() - stats.verifySeconds;

	if (statsOut)
		*statsOut = stats;

	return kept;
}

}
