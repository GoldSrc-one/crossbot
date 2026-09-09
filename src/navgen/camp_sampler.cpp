#include "camp_sampler.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <queue>

#include "clipnode_tree.h"
#include "hull_defs.h"
#include "point_grid.h"
#include "vis_sampling.h"
#include "work_pool.h"

namespace navgen
{

namespace
{

constexpr size_t kCampSeeds = 8;
constexpr int kCampRefinePasses = 3;
constexpr double kCampNeighborScale = 1.5;
constexpr double kCampNeighborMaxZ = 32.0;
constexpr double kCampGridCell = 32.0;
constexpr double kCampMinTargetDistSq = 1.0e-6;
constexpr double kCampMinGain = 1.02;

struct CampTarget
{
	Vec3 dir;
	double dist = 0.0;
};

struct FovStats
{
	double score = 0.0;
	double spread = 0.0;
	double dist = 0.0;
};

struct Evaluation
{
	double score = 0.0;
	double yaw = 0.0;
	double pitch = 0.0;
	FovStats stand;
	FovStats crouch;
	uint32_t flags = 0;
	bool valid = false;
};

struct SlotState
{
	std::unique_ptr<VisRowReader> standReader;
	std::unique_ptr<VisRowReader> crouchReader;
	HullOriginChain standChain;
	HullOriginChain crouchChain;
	TraceCounters counters;
	std::vector<CampTarget> standTargets;
	std::vector<CampTarget> crouchTargets;
	std::vector<double> hist;
	size_t evaluations = 0;
	size_t pvsPairs = 0;
	size_t visiblePairs = 0;
};

// Targets inside the cone against targets outside it, plus how wide an arc those
// inside actually span and how far away they sit.
FovStats EvaluateFov(const std::vector<CampTarget> &targets, const Vec3 &axis, double cosHalf)
{
	FovStats out;

	size_t inside = 0;
	double angleSq = 0.0;
	double distSum = 0.0;

	for (const CampTarget &t : targets)
	{
		double dot = t.dir.Dot(axis);
		if (dot < cosHalf)
			continue;

		double angle = std::acos(std::max(-1.0, std::min(1.0, dot))) / kVisDeg;
		++inside;
		angleSq += angle * angle;
		distSum += t.dist;
	}

	size_t outside = targets.size() - inside;
	double denom = (outside > 0) ? static_cast<double>(outside) : 1.0;
	out.score = static_cast<double>(inside) / denom;

	if (inside > 0)
	{
		out.spread = std::sqrt(angleSq / static_cast<double>(inside));
		out.dist = distSum / static_cast<double>(inside);
	}

	return out;
}

Vec3 RefineAxis(const std::vector<CampTarget> &standTargets,
	const std::vector<CampTarget> &crouchTargets, Vec3 axis, double cosWidest)
{
	for (int pass = 0; pass < kCampRefinePasses; ++pass)
	{
		Vec3 acc;
		for (const CampTarget &t : standTargets)
		{
			if (t.dir.Dot(axis) >= cosWidest)
				acc = acc + t.dir;
		}
		for (const CampTarget &t : crouchTargets)
		{
			if (t.dir.Dot(axis) >= cosWidest)
				acc = acc + t.dir;
		}

		if (acc.Length() < 1e-9)
			break;

		axis = acc.Normalized();
	}

	return axis;
}

struct CampContext
{
	const VisIndex *vis = nullptr;
	const std::vector<VantageCandidate> *candidates = nullptr;
	const std::vector<VisBlockerBox> *blockers = nullptr;
	HullTree hull;
	int worldHeadnode = 0;
	const BinLayout *bins = nullptr;
	size_t minVisible = 0;
};

void EvaluateCandidate(const CampContext &ctx, size_t index, SlotState &slot, Evaluation &out)
{
	out = Evaluation();
	++slot.evaluations;

	const std::vector<VantageCandidate> &candidates = *ctx.candidates;
	const VantageCandidate &self = candidates[index];

	Vec3 standEye(self.floor.x, self.floor.y, self.floor.z + kStandEyeLift);
	Vec3 crouchEye(self.floor.x, self.floor.y, self.floor.z + kDuckEyeLift);

	bool canStand = (self.areaFlags & crossbot_nav::NAV_AREA_CROUCH) == 0;
	if (canStand && ClassifyPoint(ctx.hull, ctx.worldHeadnode, standEye) == CONTENTS_SOLID)
		canStand = false;

	if (ClassifyPoint(ctx.hull, ctx.worldHeadnode, crouchEye) == CONTENTS_SOLID)
		return;

	int standLeaf = canStand ? ctx.vis->LeafForPoint(standEye) : -1;
	int crouchLeaf = ctx.vis->LeafForPoint(crouchEye);

	const uint8_t *standRow = canStand ? slot.standReader->VisRow(standLeaf) : nullptr;
	const uint8_t *crouchRow = slot.crouchReader->VisRow(crouchLeaf);

	slot.standTargets.clear();
	slot.crouchTargets.clear();

	for (size_t j = 0; j < candidates.size(); ++j)
	{
		if (j == index)
			continue;

		const VantageCandidate &target = candidates[j];

		bool standPvs = canStand && (!standRow || ctx.vis->RowVisible(standRow, target.leaf));
		bool crouchPvs = !crouchRow || ctx.vis->RowVisible(crouchRow, target.leaf);
		if (!standPvs && !crouchPvs)
			continue;

		++slot.pvsPairs;

		if (standPvs)
		{
			Vec3 delta = target.eye - standEye;
			if (delta.Dot(delta) > kCampMinTargetDistSq &&
				!Occluded(ctx.hull, ctx.worldHeadnode, *ctx.blockers, standEye, target.eye, slot.counters,
					slot.standChain))
			{
				CampTarget t;
				t.dist = delta.Length();
				t.dir = delta * (1.0 / t.dist);
				slot.standTargets.push_back(t);
				++slot.visiblePairs;
			}
		}

		if (crouchPvs)
		{
			Vec3 delta = target.eye - crouchEye;
			if (delta.Dot(delta) > kCampMinTargetDistSq &&
				!Occluded(ctx.hull, ctx.worldHeadnode, *ctx.blockers, crouchEye, target.eye, slot.counters,
					slot.crouchChain))
			{
				CampTarget t;
				t.dist = delta.Length();
				t.dir = delta * (1.0 / t.dist);
				slot.crouchTargets.push_back(t);
				++slot.visiblePairs;
			}
		}
	}

	size_t visible = std::max(slot.standTargets.size(), slot.crouchTargets.size());
	if (visible < ctx.minVisible)
		return;

	double cosHalf = std::cos(kCampFovHalfDeg * kVisDeg);

	const BinLayout &bins = *ctx.bins;
	slot.hist.assign(bins.Count(), 0.0);

	Vec3 centroid;
	for (const CampTarget &t : slot.standTargets)
	{
		double yaw = 0.0, pitch = 0.0;
		AnglesFromDir(t.dir, yaw, pitch);
		slot.hist[bins.BinFor(yaw, pitch)] += 1.0;
		centroid = centroid + t.dir;
	}
	for (const CampTarget &t : slot.crouchTargets)
	{
		double yaw = 0.0, pitch = 0.0;
		AnglesFromDir(t.dir, yaw, pitch);
		slot.hist[bins.BinFor(yaw, pitch)] += 1.0;
		centroid = centroid + t.dir;
	}

	std::vector<Vec3> seeds;
	seeds.reserve(kCampSeeds + 1);

	if (centroid.Length() > 1e-9)
		seeds.push_back(centroid.Normalized());

	for (size_t s = 0; s < kCampSeeds; ++s)
	{
		size_t peak = bins.Count();
		double peakWeight = 0.0;
		for (size_t b = 0; b < slot.hist.size(); ++b)
		{
			if (slot.hist[b] > peakWeight)
			{
				peakWeight = slot.hist[b];
				peak = b;
			}
		}

		if (peak >= bins.Count())
			break;

		seeds.push_back(bins.Dir(peak));
		slot.hist[peak] = 0.0;
	}

	bool have = false;
	for (const Vec3 &seed : seeds)
	{
		Vec3 axis = RefineAxis(slot.standTargets, slot.crouchTargets, seed, cosHalf);

		FovStats stand;
		if (canStand)
			stand = EvaluateFov(slot.standTargets, axis, cosHalf);
		FovStats crouch = EvaluateFov(slot.crouchTargets, axis, cosHalf);

		double score = std::max(stand.score, crouch.score);
		if (have && score <= out.score)
			continue;

		have = true;
		out.score = score;
		AnglesFromDir(axis, out.yaw, out.pitch);
		out.stand = stand;
		out.crouch = crouch;
	}

	if (!have)
		return;

	out.flags = canStand ? crossbot_nav::CAMP_NONE : crossbot_nav::CAMP_NO_STAND;
	out.valid = true;
}

}

std::vector<CampSpot> BuildCampSpots(const BspFile &bsp, const VisIndex &vis,
	const std::vector<BlockingVolume> &visBlockers, const std::vector<VantageCandidate> &candidates,
	const CampParams &params, CampStats *statsOut)
{
	CampStats stats;
	std::chrono::steady_clock::time_point buildStart = std::chrono::steady_clock::now();

	std::vector<CampSpot> kept;
	size_t count = candidates.size();
	if (count == 0)
	{
		if (statsOut)
			*statsOut = stats;
		return kept;
	}

	std::vector<VisBlockerBox> blockers = BuildVisBlockerBoxes(visBlockers);
	BinLayout bins(params.yawBins, params.pitchBins, params.pitchRangeDeg);

	CampContext ctx;
	ctx.vis = &vis;
	ctx.candidates = &candidates;
	ctx.blockers = &blockers;
	ctx.hull = vis.Hull0();
	ctx.worldHeadnode = bsp.Models()[0].headnode[kHullPoint];
	ctx.bins = &bins;
	ctx.minVisible = params.minVisible;

	stats.targets = count;

	double area = static_cast<double>(count) * params.gridSize * params.gridSize;
	size_t budget = (params.countOverride > 0)
		? params.countOverride
		: static_cast<size_t>(area / (params.spacing * params.spacing));
	budget = std::max<size_t>(budget, 1);
	stats.budget = budget;

	double denom = (count > 1) ? static_cast<double>(count - 1) : 1.0;
	double minSeen = static_cast<double>(params.minVisible);

	struct Entry
	{
		double score;
		size_t index;
	};

	struct Less
	{
		bool operator()(const Entry &a, const Entry &b) const
		{
			if (a.score != b.score)
				return a.score < b.score;
			return a.index > b.index;
		}
	};

	std::vector<double> score(count, 0.0);
	std::priority_queue<Entry, std::vector<Entry>, Less> queue;

	const uint32_t rejectFlags = crossbot_nav::NAV_AREA_WATER | crossbot_nav::NAV_AREA_ELEVATOR;

	for (size_t i = 0; i < count; ++i)
	{
		const VantageCandidate &c = candidates[i];
		if ((c.areaFlags & rejectFlags) != 0 || c.mainAmount <= 0.0 || c.exposure <= 0.0 ||
			c.exposure * denom < minSeen)
		{
			++stats.gated;
			continue;
		}

		score[i] = c.mainAmount / c.exposure;
		Entry e;
		e.score = score[i];
		e.index = i;
		queue.push(e);
		++stats.queued;
	}

	std::vector<Vec3> points(count);
	Vec3 extentMins(1e30, 1e30, 1e30), extentMaxs(-1e30, -1e30, -1e30);
	for (size_t i = 0; i < count; ++i)
	{
		points[i] = candidates[i].floor;
		extentMins.x = std::min(extentMins.x, points[i].x);
		extentMins.y = std::min(extentMins.y, points[i].y);
		extentMins.z = std::min(extentMins.z, points[i].z);
		extentMaxs.x = std::max(extentMaxs.x, points[i].x);
		extentMaxs.y = std::max(extentMaxs.y, points[i].y);
		extentMaxs.z = std::max(extentMaxs.z, points[i].z);
	}

	PointGrid grid(points, kCampGridCell);
	grid.SetExtent(extentMins, extentMaxs);

	size_t threads = (params.threadCount > 0) ? params.threadCount : DefaultThreadCount();
	threads = std::max<size_t>(1, threads);
	stats.threads = threads;

	ThreadPool pool(threads);

	std::vector<std::unique_ptr<SlotState>> slots;
	slots.reserve(pool.ThreadCount());
	for (size_t s = 0; s < pool.ThreadCount(); ++s)
	{
		std::unique_ptr<SlotState> slot(new SlotState());
		slot->standReader.reset(new VisRowReader(vis));
		slot->crouchReader.reset(new VisRowReader(vis));
		slots.push_back(std::move(slot));
	}

	std::vector<Evaluation> cache(count);
	std::vector<uint8_t> cached(count, 0);
	std::vector<uint8_t> consumed(count, 0);
	std::vector<uint32_t> consultMark(count, 0);
	std::vector<double> minDist(count, 0.0);

	std::vector<size_t> batch;
	std::vector<size_t> neighbors;
	std::vector<size_t> found;
	std::vector<size_t> consulted;
	std::vector<size_t> touched;
	uint32_t climbId = 0;

	std::function<void(size_t, size_t)> body = [&](size_t item, size_t slot)
	{
		size_t idx = batch[item];
		EvaluateCandidate(ctx, idx, *slots[slot], cache[idx]);
		cached[idx] = 1;
	};

	double neighborRadius = params.gridSize * kCampNeighborScale;
	double neighborQueryRadius = std::sqrt(neighborRadius * neighborRadius +
		kCampNeighborMaxZ * kCampNeighborMaxZ);

	while (kept.size() < budget && !queue.empty())
	{
		Entry top = queue.top();
		queue.pop();

		if (consumed[top.index] != 0)
			continue;

		if (top.score != score[top.index])
		{
			top.score = score[top.index];
			queue.push(top);
			continue;
		}

		++climbId;
		consulted.clear();

		size_t current = top.index;
		size_t climbSteps = 0;
		bool cancelled = false;
		bool rejected = false;

		for (size_t step = 0;; ++step)
		{
			grid.FindWithin(points[current], neighborQueryRadius, found);

			neighbors.clear();
			for (size_t n : found)
			{
				if (n == current)
					continue;
				if (std::fabs(points[n].z - points[current].z) > kCampNeighborMaxZ)
					continue;
				double nx = points[n].x - points[current].x;
				double ny = points[n].y - points[current].y;
				if (nx * nx + ny * ny > neighborRadius * neighborRadius)
					continue;
				neighbors.push_back(n);
			}

			batch.clear();
			if (cached[current] == 0)
				batch.push_back(current);
			for (size_t n : neighbors)
			{
				if (cached[n] == 0)
					batch.push_back(n);
			}

			if (!batch.empty())
			{
				++stats.batches;
				stats.batchItems += batch.size();
			}

			pool.Run(batch.size(), body);

			if (consultMark[current] != climbId)
			{
				consultMark[current] = climbId;
				consulted.push_back(current);
			}
			for (size_t n : neighbors)
			{
				if (consultMark[n] != climbId)
				{
					consultMark[n] = climbId;
					consulted.push_back(n);
				}
			}

			if (!cache[current].valid)
			{
				rejected = true;
				break;
			}

			size_t best = count;
			double bestScore = cache[current].score * kCampMinGain;
			for (size_t n : neighbors)
			{
				if (!cache[n].valid)
					continue;
				if (cache[n].score > bestScore)
				{
					bestScore = cache[n].score;
					best = n;
				}
			}

			if (best >= count)
				break;

			if (consumed[best] != 0)
			{
				cancelled = true;
				break;
			}

			if (step + 1 >= params.maxClimbSteps)
				break;

			current = best;
			++climbSteps;
		}

		for (size_t c : consulted)
			consumed[c] = 1;

		++stats.climbs;
		stats.climbStepsTotal += climbSteps;
		stats.climbStepsMax = std::max(stats.climbStepsMax, climbSteps);

		if (rejected)
		{
			++stats.rejected;
			continue;
		}

		touched.clear();
		for (size_t v : consulted)
		{
			grid.FindWithin(points[v], params.spacing, found);
			for (size_t f : found)
			{
				double dx = points[f].x - points[v].x;
				double dy = points[f].y - points[v].y;
				double dz = points[f].z - points[v].z;
				double d = std::sqrt(dx * dx + dy * dy + dz * dz);

				if (consultMark[f] == climbId)
					continue;

				if (minDist[f] == 0.0)
					touched.push_back(f);
				if (minDist[f] == 0.0 || d < minDist[f])
					minDist[f] = std::max(d, 1e-9);
			}
		}

		for (size_t f : touched)
		{
			if (consumed[f] == 0)
			{
				double factor = std::min(1.0, minDist[f] / params.spacing);
				score[f] *= factor;
			}
			minDist[f] = 0.0;
		}

		if (cancelled)
		{
			++stats.cancelled;
			continue;
		}

		const Evaluation &ev = cache[current];
		const VantageCandidate &winner = candidates[current];

		CampSpot spot;
		spot.pos = winner.floor;
		spot.yaw = ev.yaw;
		spot.pitch = ev.pitch;
		spot.score = ev.score;
		spot.standScore = ev.stand.score;
		spot.standSpread = ev.stand.spread;
		spot.standDist = ev.stand.dist;
		spot.crouchScore = ev.crouch.score;
		spot.crouchSpread = ev.crouch.spread;
		spot.crouchDist = ev.crouch.dist;
		spot.areaId = winner.areaId;
		spot.flags = ev.flags;
		spot.climbStart = points[top.index];
		spot.climbSteps = climbSteps;
		kept.push_back(spot);

	}

	for (const std::unique_ptr<SlotState> &slot : slots)
	{
		stats.evaluations += slot->evaluations;
		stats.pvsPairs += slot->pvsPairs;
		stats.visiblePairs += slot->visiblePairs;
		stats.worldTraces += slot->counters.worldTraces;
		stats.blockerTraces += slot->counters.blockerTraces;
	}

	stats.kept = kept.size();
	stats.idleSeconds = pool.IdleSeconds();
	stats.buildSeconds = std::chrono::duration<double>(
		std::chrono::steady_clock::now() - buildStart).count();

	if (statsOut)
		*statsOut = stats;

	return kept;
}

}
