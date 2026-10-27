#include "camp_sampler.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <queue>
#include <unordered_map>

#include "clipnode_tree.h"
#include "hull_defs.h"
#include "point_grid.h"
#include "reachability.h"
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
constexpr size_t kCampBatchChunk = 256;
constexpr double kCampBudgetScale = 2.0;

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
	BatchTraceScratch scratch;
	std::vector<uint8_t> blocked;
	std::vector<uint8_t> standBlocked;
	std::vector<uint8_t> crouchBlocked;
	TraceCounters counters;
	std::vector<CampTarget> standTargets;
	std::vector<CampTarget> crouchTargets;
	std::vector<double> hist;
	size_t evaluations = 0;
	size_t pvsPairs = 0;
	size_t visiblePairs = 0;
};

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
	std::vector<size_t> targetOrder;
	std::vector<float> targetEyes;
	std::vector<size_t> groupBegin;
	std::vector<size_t> groupEnd;
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

	slot.standBlocked.resize(candidates.size());
	slot.crouchBlocked.resize(candidates.size());
	slot.blocked.resize(2 * kCampBatchChunk);
	const float eyes[6] = {
		static_cast<float>(standEye.x), static_cast<float>(standEye.y), static_cast<float>(standEye.z),
		static_cast<float>(crouchEye.x), static_cast<float>(crouchEye.y), static_cast<float>(crouchEye.z)};
	for (size_t g = 0; g < ctx.groupBegin.size(); ++g)
	{
		size_t begin = ctx.groupBegin[g];
		size_t m = ctx.groupEnd[g] - begin;
		int leaf = candidates[ctx.targetOrder[begin]].leaf;
		bool standPvs = canStand && (!standRow || ctx.vis->RowVisible(standRow, leaf));
		bool crouchPvs = !crouchRow || ctx.vis->RowVisible(crouchRow, leaf);
		if (!standPvs && !crouchPvs)
			continue;

		size_t n = (standPvs && crouchPvs) ? 2 : 1;
		SegmentsBlockedBatch(ctx.hull, ctx.worldHeadnode, standPvs ? eyes : eyes + 3, n,
			ctx.targetEyes.data() + 3 * begin, m, slot.blocked.data(), slot.scratch);
		slot.counters.worldTraces += n * m;
		for (size_t y = 0; y < m; ++y)
		{
			size_t j = ctx.targetOrder[begin + y];
			if (standPvs)
				slot.standBlocked[j] = slot.blocked[y];
			if (crouchPvs)
				slot.crouchBlocked[j] = slot.blocked[(n - 1) * m + y];
		}
	}

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
			if (delta.Dot(delta) > kCampMinTargetDistSq && slot.standBlocked[j] == 0 &&
				!BlockersOccluded(ctx.hull, *ctx.blockers, standEye, target.eye, slot.counters))
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
			if (delta.Dot(delta) > kCampMinTargetDistSq && slot.crouchBlocked[j] == 0 &&
				!BlockersOccluded(ctx.hull, *ctx.blockers, crouchEye, target.eye, slot.counters))
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

double BoxDist(const Vec3 &p, const Vec3 &mins, const Vec3 &maxs)
{
	Vec3 d(std::max(0.0, std::max(mins.x - p.x, p.x - maxs.x)),
		std::max(0.0, std::max(mins.y - p.y, p.y - maxs.y)),
		std::max(0.0, std::max(mins.z - p.z, p.z - maxs.z)));
	return d.Length();
}

bool SeesObjective(const CampContext &ctx, const CampObjective &obj, const std::vector<int> &nearbyLeaves,
	const Vec3 &eye, const uint8_t *row, SlotState &slot)
{
	for (size_t p = 0; p < obj.points.size(); ++p)
	{
		TraceResult tr = TraceHull(ctx.hull, ctx.worldHeadnode, eye, obj.points[p]);
		++slot.counters.worldTraces;
		if (tr.startsolid || tr.allsolid)
			continue;
		if (tr.fraction < 1.f && BoxDist(tr.endpos, obj.mins[p], obj.maxs[p]) > crossbot_nav::kOverlookMargin)
			continue;
		if (!BlockersOccluded(ctx.hull, *ctx.blockers, eye, tr.endpos, slot.counters))
			return true;
	}

	for (size_t s = 0; s < obj.nearby.size(); ++s)
	{
		if (row && !ctx.vis->RowVisible(row, nearbyLeaves[s]))
			continue;
		if (!Occluded(ctx.hull, ctx.worldHeadnode, *ctx.blockers, eye, obj.nearby[s], slot.counters))
			return true;
	}

	return false;
}

uint64_t SeenObjectives(const CampContext &ctx, const std::vector<CampObjective> &objectives,
	const std::vector<std::vector<int>> &nearbyLeaves, size_t index, SlotState &slot)
{
	const VantageCandidate &self = (*ctx.candidates)[index];
	Vec3 eye(self.floor.x, self.floor.y, self.floor.z + kStandEyeLift);
	if ((self.areaFlags & crossbot_nav::NAV_AREA_CROUCH) != 0 ||
		ClassifyPoint(ctx.hull, ctx.worldHeadnode, eye) == CONTENTS_SOLID)
		eye.z = self.floor.z + kDuckEyeLift;
	if (ClassifyPoint(ctx.hull, ctx.worldHeadnode, eye) == CONTENTS_SOLID)
		return 0;

	const uint8_t *row = slot.standReader->VisRow(ctx.vis->LeafForPoint(eye));
	uint64_t seen = 0;
	for (size_t k = 0; k < objectives.size(); ++k)
	{
		if (SeesObjective(ctx, objectives[k], nearbyLeaves[k], eye, row, slot))
			seen |= uint64_t(1) << k;
	}
	return seen;
}

constexpr double kAreaFloorNormalZ = 0.3;
constexpr int kTfcActivateTouch = 1;
constexpr int kTfcActivateDetpack = 2;
constexpr int kDodSfTnt = 64;

void AddNearby(const MergeResult &merged, const std::vector<std::vector<size_t>> &areaConns, const Vec3 &point,
	std::vector<Vec3> &out)
{
	int goal = FindAreaContainingPoint(point, merged.areas);
	if (goal < 0)
		return;

	const double reach = crossbot_nav::kOverlookReach;
	std::vector<double> field(merged.areas.size(), reach + 1.0);
	typedef std::pair<double, int> Entry;
	std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open;
	field[goal] = 0.0;
	open.push(Entry(0.0, goal));
	while (!open.empty())
	{
		Entry top = open.top();
		open.pop();
		int current = top.second;
		if (top.first > field[current])
			continue;

		for (size_t ci : areaConns[current])
		{
			const MergedConnection &c = merged.conns[ci];
			int from = (c.areaA == current) ? c.areaB : c.areaA;
			if (!crossbot_nav::ConnAllowsDirection(c.connFlags, static_cast<uint32_t>(c.areaA),
				static_cast<uint32_t>(from)))
				continue;
			if (c.connType == crossbot_nav::CONN_AIRMOVE && std::fabs(c.gravity - 1.0) > 1e-3)
				continue;

			double base = static_cast<double>(ConnectionTravelSeconds(merged.areas, c)) +
				static_cast<double>(ConnectionCostSeconds(c, true));
			double step = top.first + std::max(0.0, base);
			if (step > reach || step >= field[from])
				continue;
			field[from] = step;
			open.push(Entry(step, from));
		}
	}

	std::vector<Entry> close;
	for (size_t i = 0; i < field.size(); ++i)
	{
		if (static_cast<int>(i) != goal && field[i] <= reach)
			close.push_back(Entry(field[i], static_cast<int>(i)));
	}
	std::sort(close.begin(), close.end());
	if (close.size() > crossbot_nav::kOverlookSamples)
		close.resize(crossbot_nav::kOverlookSamples);

	for (const Entry &e : close)
	{
		const Area &area = merged.areas[e.second];
		Vec3 sum;
		double minZ = 1e30;
		size_t n = 0;
		for (const Face &f : area.polytope.faces)
		{
			for (const Vec3 &v : f.winding)
			{
				sum = sum + v;
				minZ = std::min(minZ, v.z);
				++n;
			}
		}
		if (n == 0)
			continue;

		Vec3 c = sum * (1.0 / static_cast<double>(n));
		double floorZ = minZ;
		bool any = false;
		for (const Plane &pl : area.boundPlanes)
		{
			if (pl.normal.z <= kAreaFloorNormalZ)
				continue;
			double z = (pl.dist - pl.normal.x * c.x - pl.normal.y * c.y) / pl.normal.z;
			if (!any || z > floorZ)
			{
				floorZ = z;
				any = true;
			}
		}
		out.push_back(Vec3(c.x, c.y, floorZ + crossbot_nav::kOverlookChest));
	}
}

bool TfcObjective(const Entity &ent)
{
	const std::string *ga = ent.Find("goal_activation");
	if (!ga)
		ga = ent.Find("g_a");
	int activation = ga ? std::atoi(ga->c_str()) : 0;
	if ((activation & kTfcActivateDetpack) != 0)
		return true;
	if ((activation & kTfcActivateTouch) == 0)
		return false;
	if (std::atof(ent.Get("frags").c_str()) > 0.0)
		return true;
	for (int team = 1; team <= 4; ++team)
	{
		if (std::atof(ent.Get("increase_team" + std::to_string(team)).c_str()) > 0.0)
			return true;
	}
	return false;
}

std::string ObjectiveName(const Entity &ent)
{
	std::string name = ent.Get("classname");
	const char *keys[] = { "point_name", "netname", "targetname", "model" };
	for (const char *key : keys)
	{
		std::string value = ent.Get(key);
		if (!value.empty())
			return name + " " + value;
	}
	return name;
}

}

std::vector<CampObjective> BuildCampObjectives(const BspFile &bsp, const std::vector<Entity> &entities,
	const MergeResult &merged)
{
	std::vector<CampObjective> out;

	bool dod = false;
	bool rescue = false;
	bool hostages = false;
	for (const Entity &ent : entities)
	{
		std::string cls = ent.Get("classname");
		dod = dod || cls.compare(0, 4, "dod_") == 0;
		rescue = rescue || cls == "func_hostage_rescue" || cls == "info_hostage_rescue";
		hostages = hostages || cls == "hostage_entity";
	}

	std::vector<std::vector<size_t>> areaConns(merged.areas.size());
	for (size_t i = 0; i < merged.conns.size(); ++i)
	{
		const MergedConnection &c = merged.conns[i];
		if (c.areaA < 0 || c.areaB < 0 || c.areaA == c.areaB)
			continue;
		areaConns[c.areaA].push_back(i);
		areaConns[c.areaB].push_back(i);
	}

	auto open = [&](const Entity &ent)
	{
		CampObjective obj;
		obj.name = ObjectiveName(ent);
		out.push_back(obj);
		return out.size() - 1;
	};

	auto join = [&](const Entity &ent, size_t index)
	{
		Vec3 origin = ParseEntityOrigin(ent.Get("origin"));
		Vec3 point = origin;
		Vec3 mins = origin;
		Vec3 maxs = origin;
		int model = ParseBrushModelIndex(ent.Get("model"));
		if (model > 0 && model < static_cast<int>(bsp.Models().size()))
		{
			const dmodel_t &m = bsp.Models()[model];
			mins = Vec3(origin.x + m.mins[0], origin.y + m.mins[1], origin.z + m.mins[2]);
			maxs = Vec3(origin.x + m.maxs[0], origin.y + m.maxs[1], origin.z + m.maxs[2]);
			point = ModelWorldCenter(m, origin);
		}

		CampObjective &obj = out[index];
		obj.points.push_back(point);
		obj.mins.push_back(mins);
		obj.maxs.push_back(maxs);
		++obj.entities;
		AddNearby(merged, areaConns, point, obj.nearby);
	};

	std::unordered_map<std::string, size_t> controlPoints;
	std::unordered_map<std::string, size_t> objectGroups;
	size_t hostageObjective = SIZE_MAX;
	size_t rescueObjective = SIZE_MAX;
	for (const Entity &ent : entities)
	{
		std::string cls = ent.Get("classname");
		if (cls == "func_bomb_target" || cls == "info_bomb_target" || cls == "func_hostage_rescue" ||
			cls == "info_hostage_rescue" || cls == "func_vip_safetyzone" || cls == "func_escapezone" ||
			(cls == "dod_object" && ent.Get("object_group").empty()) || cls == "item_tfgoal" ||
			(dod && cls == "func_breakable" && (std::atoi(ent.Get("spawnflags").c_str()) & kDodSfTnt) != 0) ||
			((cls == "info_tfgoal" || cls == "i_t_g") && TfcObjective(ent)))
		{
			join(ent, open(ent));
		}
		else if (cls == "dod_control_point")
		{
			size_t index = open(ent);
			join(ent, index);
			std::string name = ent.Get("targetname");
			if (!name.empty())
				controlPoints[name] = index;
		}
		else if (cls == "dod_object")
		{
			std::string group = ent.Get("object_group");
			auto it = objectGroups.find(group);
			if (it == objectGroups.end())
			{
				it = objectGroups.emplace(group, open(ent)).first;
				out[it->second].name = cls + " " + group;
			}
			join(ent, it->second);
		}
		else if (cls == "hostage_entity")
		{
			if (hostageObjective == SIZE_MAX)
				hostageObjective = open(ent);
			join(ent, hostageObjective);
		}
		else if (cls == "info_player_start" && hostages && !rescue)
		{
			if (rescueObjective == SIZE_MAX)
				rescueObjective = open(ent);
			join(ent, rescueObjective);
		}
	}

	for (const Entity &ent : entities)
	{
		if (ent.Get("classname") != "dod_capture_area")
			continue;
		auto it = controlPoints.find(ent.Get("target"));
		join(ent, it != controlPoints.end() ? it->second : open(ent));
	}

	if (out.size() > kMaxCampObjectives)
	{
		std::printf("camp objectives: %zu found, only the first %zu rank camp spots\n", out.size(),
			kMaxCampObjectives);
		out.resize(kMaxCampObjectives);
	}

	return out;
}

std::vector<CampSpot> BuildCampSpots(const BspFile &bsp, const VisIndex &vis,
	const std::vector<BlockingVolume> &visBlockers, const std::vector<VantageCandidate> &candidates,
	const std::vector<CampObjective> &objectives, const CampParams &params, CampStats *statsOut)
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
	double gridScale = kDefaultVantageGrid / params.gridSize;
	size_t minVisible = std::max<size_t>(1, static_cast<size_t>(
		std::lround(static_cast<double>(params.minVisible) * gridScale * gridScale)));
	ctx.minVisible = minVisible;

	ctx.targetOrder.resize(count);
	for (size_t i = 0; i < count; ++i)
		ctx.targetOrder[i] = i;
	std::stable_sort(ctx.targetOrder.begin(), ctx.targetOrder.end(),
		[&](size_t a, size_t b) { return candidates[a].leaf < candidates[b].leaf; });
	ctx.targetEyes.resize(3 * count);
	for (size_t k = 0; k < count; ++k)
	{
		const Vec3 &eye = candidates[ctx.targetOrder[k]].eye;
		ctx.targetEyes[3 * k] = static_cast<float>(eye.x);
		ctx.targetEyes[3 * k + 1] = static_cast<float>(eye.y);
		ctx.targetEyes[3 * k + 2] = static_cast<float>(eye.z);
		int leaf = candidates[ctx.targetOrder[k]].leaf;
		if (k == 0 || candidates[ctx.targetOrder[k - 1]].leaf != leaf ||
			k - ctx.groupBegin.back() >= kCampBatchChunk)
		{
			ctx.groupBegin.push_back(k);
			ctx.groupEnd.push_back(k);
		}
		ctx.groupEnd.back() = k + 1;
	}

	stats.targets = count;

	double area = static_cast<double>(count) * params.gridSize * params.gridSize;
	size_t budget = (params.countOverride > 0)
		? params.countOverride
		: static_cast<size_t>(kCampBudgetScale * area / (params.spacing * params.spacing));
	budget = std::max<size_t>(budget, 1);
	stats.budget = budget;

	double denom = (count > 1) ? static_cast<double>(count - 1) : 1.0;
	double minSeen = static_cast<double>(minVisible);

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

	std::vector<uint64_t> sees(count, 0);
	std::vector<int> tier(count, 0);
	stats.objectives = objectives.size();
	if (!objectives.empty())
	{
		std::chrono::steady_clock::time_point objectiveStart = std::chrono::steady_clock::now();
		std::vector<std::vector<int>> nearbyLeaves(objectives.size());
		for (size_t k = 0; k < objectives.size(); ++k)
		{
			for (const Vec3 &p : objectives[k].nearby)
				nearbyLeaves[k].push_back(vis.LeafForPoint(p));
		}

		pool.Run(count, [&](size_t item, size_t slot)
		{
			sees[item] = SeenObjectives(ctx, objectives, nearbyLeaves, item, *slots[slot]);
		});

		for (size_t i = 0; i < count; ++i)
		{
			tier[i] = sees[i] != 0 ? 1 : 0;
			if (tier[i] > 0)
				++stats.overlookers;		}
		stats.objectiveSeconds = std::chrono::duration<double>(
			std::chrono::steady_clock::now() - objectiveStart).count();
	}

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
	std::vector<std::priority_queue<Entry, std::vector<Entry>, Less>> queues(objectives.size() + 1);
	const size_t none = objectives.size();
	stats.lanePicks.assign(queues.size(), 0);

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
		if (sees[i] == 0)
			queues[none].push(e);
		for (size_t k = 0; k < objectives.size(); ++k)
		{
			if ((sees[i] >> k) & 1)
				queues[k].push(e);
		}
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

	while (kept.size() < budget)
	{
		size_t lane = queues.size();
		for (size_t q = 0; q < queues.size(); ++q)
		{
			if (!queues[q].empty() && (lane == queues.size() || stats.lanePicks[q] < stats.lanePicks[lane]))
				lane = q;
		}
		if (lane == queues.size())
			break;

		std::priority_queue<Entry, std::vector<Entry>, Less> &queue = queues[lane];
		uint64_t laneBit = (lane == none) ? 0 : (uint64_t(1) << lane);
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
				if ((sees[n] & laneBit) != laneBit)
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
			int bestTier = tier[current];
			double bestScore = cache[current].score * kCampMinGain;
			for (size_t n : neighbors)
			{
				if (!cache[n].valid)
					continue;
				if (tier[n] > bestTier || (tier[n] == bestTier && cache[n].score > bestScore))
				{
					bestTier = tier[n];
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
			if (consumed[f] == 0 && tier[f] <= tier[current])
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
		spot.objectives = sees[current];
		if (sees[current] != 0)
			++stats.overlookKept;
		else
			++stats.lanePicks[none];
		for (size_t k = 0; k < objectives.size(); ++k)
		{
			if ((sees[current] >> k) & 1)
				++stats.lanePicks[k];
		}
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
	stats.buildSeconds = std::chrono::duration<double>(
		std::chrono::steady_clock::now() - buildStart).count();

	if (statsOut)
		*statsOut = stats;

	return kept;
}

}
