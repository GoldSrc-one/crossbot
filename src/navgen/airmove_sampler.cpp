#include "airmove_sampler.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <chrono>
#include <cstdint>
#include <map>
#include <memory>
#include <random>
#include <unordered_set>
#include <utility>

#include "hull_defs.h"
#include "nav_format.h"
#include "nav_math.h"
#include "work_pool.h"

namespace navgen
{

namespace
{

using NavMath::kPi;

using PmSim::kGroundNormalZ;
using PmSim::kGroundTraceDist;
using PmSim::kJumpImpulse;

constexpr double kWorldBoundsSlop = 64.0;

struct AirmoveSample
{
	Vec3 launchPos;
	Vec3 landingPos;
	float yaw = 0.f;
	float speed = 0.f;
	AirmoveTrajectory traj;
	uint64_t itemId = 0;
	uint32_t seq = 0;
};

bool SampleOrderLess(const AirmoveSample &a, const AirmoveSample &b)
{
	if (a.itemId != b.itemId)
		return a.itemId < b.itemId;
	return a.seq < b.seq;
}

uint64_t Mix64(uint64_t x)
{
	x += 0x9E3779B97F4A7C15ull;
	x = (x ^ (x >> 30)) * 0xBF58476D1CE4E5B9ull;
	x = (x ^ (x >> 27)) * 0x94D049BB133111EBull;
	return x ^ (x >> 31);
}

uint64_t EdgeSeed(unsigned int base, int phase, int areaId, int modeIndex, size_t edgeIndex)
{
	uint64_t h = Mix64(static_cast<uint64_t>(base));
	h = Mix64(h ^ static_cast<uint64_t>(static_cast<unsigned int>(phase)));
	h = Mix64(h ^ static_cast<uint64_t>(static_cast<unsigned int>(areaId)));
	h = Mix64(h ^ static_cast<uint64_t>(static_cast<unsigned int>(modeIndex)));
	h = Mix64(h ^ static_cast<uint64_t>(edgeIndex));
	return h;
}

Vec3 AreaCenter(const Area &area)
{
	Vec3 sum(0.0, 0.0, 0.0);
	size_t count = 0;

	for (const Face &face : area.polytope.faces)
	{
		for (const Vec3 &v : face.winding)
		{
			sum = sum + v;
			++count;
		}
	}

	if (count == 0)
		return sum;

	return sum * (1.0 / static_cast<double>(count));
}

struct GatePoint
{
	Vec3 pos;
	float yaw = 0.f;
	float speed = 0.f;
};

struct SampleGate
{
	Vec3 left, right;
	float yaw[2] = {0.f, 0.f};
	float speed[2] = {0.f, 0.f};
	bool valid = false;
};

struct BinEntry
{
	AirmoveTrajectory best;
	std::vector<AirmoveSample> samples;
};

enum Mode
{
	kModeStand = 0,
	kModeCrouchJump,
	kModeDuck,
	kModeCount,
};

constexpr size_t kAirmoveTileItems = 32;
constexpr double kAirmoveMortonCell = 64.0;

constexpr int kFitIterations = 4;
constexpr double kTukeyTuning = 4.685;
constexpr double kMadScale = 1.4826;
constexpr double kMinFitScale = 1.0;
constexpr double kGateSigmas = 1.0;
constexpr double kMinGateSigma = 1.0;

SampleGate FitGate(const std::vector<GatePoint> &points)
{
	SampleGate gate;
	std::vector<double> weight(points.size(), 1.0);
	std::vector<double> resid(points.size(), 0.0);
	Vec3 mean(0.0, 0.0, 0.0);
	Vec3 along(1.0, 0.0, 0.0);
	double weightSum = 0.0;

	for (int iter = 0; iter < kFitIterations; ++iter)
	{
		weightSum = 0.0;
		Vec3 m(0.0, 0.0, 0.0);
		for (size_t i = 0; i < points.size(); ++i)
		{
			m = m + points[i].pos * weight[i];
			weightSum += weight[i];
		}
		if (weightSum < 1e-9)
			return gate;

		mean = m * (1.0 / weightSum);

		double cxx = 0.0, cxy = 0.0, cyy = 0.0;
		for (size_t i = 0; i < points.size(); ++i)
		{
			double dx = points[i].pos.x - mean.x;
			double dy = points[i].pos.y - mean.y;
			cxx += weight[i] * dx * dx;
			cxy += weight[i] * dx * dy;
			cyy += weight[i] * dy * dy;
		}
		cxx /= weightSum;
		cxy /= weightSum;
		cyy /= weightSum;

		if (std::fabs(cxy) > 1e-9)
		{
			double diff = cxx - cyy;
			double lambda = 0.5 * ((cxx + cyy) + std::sqrt(diff * diff + 4.0 * cxy * cxy));
			Vec3 v(cxy, lambda - cxx, 0.0);
			double len = std::sqrt(v.x * v.x + v.y * v.y);
			if (len > 1e-9)
				along = v * (1.0 / len);
		}
		else
		{
			along = (cxx >= cyy) ? Vec3(1.0, 0.0, 0.0) : Vec3(0.0, 1.0, 0.0);
		}

		Vec3 across(-along.y, along.x, 0.0);
		for (size_t i = 0; i < points.size(); ++i)
		{
			double dx = points[i].pos.x - mean.x;
			double dy = points[i].pos.y - mean.y;
			resid[i] = std::fabs(dx * across.x + dy * across.y);
		}

		if (iter + 1 == kFitIterations)
			break;

		std::vector<double> sorted = resid;
		std::nth_element(sorted.begin(), sorted.begin() + sorted.size() / 2, sorted.end());
		double scale = std::max(kMadScale * sorted[sorted.size() / 2], kMinFitScale);

		for (size_t i = 0; i < points.size(); ++i)
		{
			double u = resid[i] / (kTukeyTuning * scale);
			weight[i] = (u < 1.0) ? (1.0 - u * u) * (1.0 - u * u) : 0.0;
		}
	}

	std::vector<double> t(points.size(), 0.0);
	double tMean = 0.0, tMin = 1e30, tMax = -1e30;
	for (size_t i = 0; i < points.size(); ++i)
	{
		t[i] = (points[i].pos.x - mean.x) * along.x + (points[i].pos.y - mean.y) * along.y;
		tMean += weight[i] * t[i];
		if (weight[i] > 0.0)
		{
			tMin = std::min(tMin, t[i]);
			tMax = std::max(tMax, t[i]);
		}
	}
	if (weightSum < 1e-9 || tMin > tMax)
		return gate;
	tMean /= weightSum;

	double variance = 0.0;
	for (size_t i = 0; i < points.size(); ++i)
	{
		double dt = t[i] - tMean;
		variance += weight[i] * dt * dt;
	}
	variance /= weightSum;

	double sigma = std::sqrt(variance);
	if (sigma < kMinGateSigma)
		return gate;

	double tLo = std::max(tMean - kGateSigmas * sigma, tMin);
	double tHi = std::min(tMean + kGateSigmas * sigma, tMax);
	if (tHi - tLo < kMinGateSigma)
		return gate;

	double bandwidth = std::max((tHi - tLo) * 0.5, kMinGateSigma);
	auto localAt = [&](double tAt, double (*get)(const GatePoint &))
	{
		double num = 0.0, den = 0.0;
		for (size_t i = 0; i < points.size(); ++i)
		{
			double u = std::fabs((t[i] - tAt) / bandwidth);
			if (u >= 1.0)
				continue;
			double tricube = (1.0 - u * u * u);
			double w = weight[i] * tricube * tricube * tricube;
			num += w * get(points[i]);
			den += w;
		}

		if (den < 1e-9)
		{
			for (size_t i = 0; i < points.size(); ++i)
			{
				num += weight[i] * get(points[i]);
				den += weight[i];
			}
		}

		return (den > 1e-9) ? (num / den) : 0.0;
	};

	auto getZ = [](const GatePoint &p) { return p.pos.z; };
	auto getSpeed = [](const GatePoint &p) { return static_cast<double>(p.speed); };

	auto getCos = [](const GatePoint &p) { return std::cos(p.yaw * kPi / 180.0); };
	auto getSin = [](const GatePoint &p) { return std::sin(p.yaw * kPi / 180.0); };

	double zLo = localAt(tLo, getZ), zHi = localAt(tHi, getZ);
	double speedLo = localAt(tLo, getSpeed), speedHi = localAt(tHi, getSpeed);
	double cosLo = localAt(tLo, getCos), cosHi = localAt(tHi, getCos);
	double sinLo = localAt(tLo, getSin), sinHi = localAt(tHi, getSin);

	gate.left = Vec3(mean.x + along.x * tLo, mean.y + along.y * tLo, zLo);
	gate.right = Vec3(mean.x + along.x * tHi, mean.y + along.y * tHi, zHi);
	gate.yaw[0] = static_cast<float>(std::atan2(sinLo, cosLo) * 180.0 / kPi);
	gate.yaw[1] = static_cast<float>(std::atan2(sinHi, cosHi) * 180.0 / kPi);
	gate.speed[0] = static_cast<float>(std::max(0.0, speedLo));
	gate.speed[1] = static_cast<float>(std::max(0.0, speedHi));
	gate.valid = true;
	return gate;
}

bool SeedEdgeFromLandings(const BinEntry &entry, LaunchEdge &out)
{
	Vec3 dirSum(0.0, 0.0, 0.0);
	std::vector<Vec3> landings;

	for (const AirmoveSample &s : entry.samples)
	{
		Vec3 climb = s.launchPos - s.landingPos;
		climb.z = 0.0;
		double len = std::sqrt(climb.x * climb.x + climb.y * climb.y);
		if (len < 1e-6)
			continue;

		dirSum = dirSum + climb * (1.0 / len);
		landings.push_back(s.landingPos);
	}

	double dirLen = std::sqrt(dirSum.x * dirSum.x + dirSum.y * dirSum.y);
	if (landings.size() < 2 || dirLen < 1e-6)
		return false;

	out.outward = dirSum * (1.0 / dirLen);
	Vec3 along(-out.outward.y, out.outward.x, 0.0);

	double lo = 1e30, hi = -1e30;
	size_t loIdx = 0, hiIdx = 0;
	for (size_t i = 0; i < landings.size(); ++i)
	{
		double s = landings[i].x * along.x + landings[i].y * along.y;
		if (s < lo)
		{
			lo = s;
			loIdx = i;
		}
		if (s > hi)
		{
			hi = s;
			hiIdx = i;
		}
	}

	out.a = landings[loIdx];
	out.b = landings[hiIdx];

	Vec3 span = out.b - out.a;
	out.length = std::sqrt(span.x * span.x + span.y * span.y);
	return out.length > 1.0;
}

bool BuildSampledGates(const BinEntry &entry, MergedConnection &conn)
{
	std::vector<GatePoint> launches, landings;

	for (const AirmoveSample &s : entry.samples)
	{
		launches.push_back({s.launchPos, s.yaw, s.speed});
		landings.push_back({s.landingPos, s.yaw, s.speed});
	}

	SampleGate exitGate = FitGate(launches);
	SampleGate entryGate = FitGate(landings);
	if (!exitGate.valid || !entryGate.valid)
		return false;

	conn.gateAValid = true;
	conn.gateALeft = exitGate.left;
	conn.gateARight = exitGate.right;
	conn.gateAYaw[0] = exitGate.yaw[0];
	conn.gateAYaw[1] = exitGate.yaw[1];
	conn.gateASpeed[0] = exitGate.speed[0];
	conn.gateASpeed[1] = exitGate.speed[1];

	conn.gateBValid = true;
	conn.gateBLeft = entryGate.left;
	conn.gateBRight = entryGate.right;
	return true;
}

std::vector<LaunchEdge> CollectPerimeterEdges(const Area &area)
{
	std::vector<LaunchEdge> edges;

	for (const Face &face : area.polytope.faces)
	{
		if (!face.walkable || face.plane.normal.z <= kFloorNormalZ)
			continue;
		if (face.winding.size() < 3)
			continue;

		Vec3 center(0, 0, 0);
		for (const Vec3 &v : face.winding)
			center = center + v;
		center = center * (1.0 / static_cast<double>(face.winding.size()));

		for (size_t i = 0; i < face.winding.size(); ++i)
		{
			const Vec3 &a = face.winding[i];
			const Vec3 &b = face.winding[(i + 1) % face.winding.size()];

			double len = (b - a).Length();
			if (len < 1.0)
				continue;

			Vec3 outward = (b - a).Cross(face.plane.normal);
			outward.z = 0.0;
			double outLen = outward.Length();
			if (outLen < 1e-6)
				continue;
			outward = outward * (1.0 / outLen);

			Vec3 mid = (a + b) * 0.5;
			if (outward.Dot(center - mid) > 0.0)
				outward = outward * -1.0;

			LaunchEdge edge;
			edge.a = a;
			edge.b = b;
			edge.outward = outward;
			edge.length = len;
			edges.push_back(edge);
		}
	}

	return edges;
}

long long PairKey(int from, int to)
{
	return static_cast<long long>(from) * 1000000LL + static_cast<long long>(to);
}

bool BetterTrajectory(const AirmoveTrajectory &candidate, const AirmoveTrajectory &current)
{
	if (candidate.fallDamage < current.fallDamage - 0.01f)
		return true;
	if (candidate.fallDamage > current.fallDamage + 0.01f)
		return false;
	return candidate.flightTime < current.flightTime;
}

enum Bin
{
	kBinStandDrop = 0,
	kBinStandJump,
	kBinDuckDrop,
	kBinDuckJump,
	kBinCount,
};

int BinFor(bool duckHull, bool jumped)
{
	return (duckHull ? kBinDuckDrop : kBinStandDrop) + (jumped ? 1 : 0);
}

struct SweepMode
{
	const PM_MoveSimulator *sim = nullptr;
	const char *name = "";
	double probeZ = 0.0;
	double launchZ = 0.0;
	double navZAdjust = 0.0;
	float maxSpeed = 0.0f;
	bool duckHull = false;
	bool drop = true;
	bool jump = true;
};

size_t CountSupportedEdges(const PM_MoveSimulator &sim, const std::vector<LaunchEdge> &edges,
	double probeZ, const AirmoveSampleParams &params)
{
	size_t supported = 0;

	for (const LaunchEdge &edge : edges)
	{
		Vec3 launchPos = (edge.a + edge.b) * 0.5 - edge.outward * params.inset;
		launchPos.z += params.lift;

		if (sim.IsSupported(Vec3(launchPos.x, launchPos.y, launchPos.z + probeZ)))
			++supported;
	}

	return supported;
}

}

std::vector<LaunchEdge> FilterLaunchEdges(const PM_MoveSimulator &sim, const Area &area, double probeZ,
	const AirmoveSampleParams &params)
{
	std::vector<LaunchEdge> edges;

	for (const LaunchEdge &edge : CollectPerimeterEdges(area))
	{
		Vec3 mid = (edge.a + edge.b) * 0.5;
		mid.z += params.lift + probeZ;

		if (!sim.VoidAhead(mid, edge.outward))
			continue;

		edges.push_back(edge);
	}

	return edges;
}

std::vector<LaunchEdge> PushLaunchEdges(const Area &area, const std::vector<PushVolume> &pushes)
{
	std::vector<LaunchEdge> edges;

	for (const PushVolume &vol : pushes)
	{
		const Vec3 &mn = vol.volume.mins;
		const Vec3 &mx = vol.volume.maxs;
		struct Side { Vec3 a, b, inward; };
		const Side sides[4] = {
			{Vec3(mn.x, mn.y, 0.0), Vec3(mn.x, mx.y, 0.0), Vec3(1.0, 0.0, 0.0)},
			{Vec3(mx.x, mn.y, 0.0), Vec3(mx.x, mx.y, 0.0), Vec3(-1.0, 0.0, 0.0)},
			{Vec3(mn.x, mn.y, 0.0), Vec3(mx.x, mn.y, 0.0), Vec3(0.0, 1.0, 0.0)},
			{Vec3(mn.x, mx.y, 0.0), Vec3(mx.x, mx.y, 0.0), Vec3(0.0, -1.0, 0.0)},
		};

		for (const Face &face : area.polytope.faces)
		{
			if (!face.walkable || face.plane.normal.z <= kFloorNormalZ)
				continue;
			if (face.winding.size() < 3)
				continue;

			Vec3 center(0, 0, 0);
			for (const Vec3 &v : face.winding)
				center = center + v;
			center = center * (1.0 / static_cast<double>(face.winding.size()));

			if (center.z + 1.0 < mn.z || center.z + 1.0 > mx.z)
				continue;

			for (const Side &side : sides)
			{
				double tEnter = 0.0, tExit = 1.0;
				bool alive = true;

				for (size_t i = 0; i < face.winding.size() && alive; ++i)
				{
					const Vec3 &p0 = face.winding[i];
					const Vec3 &p1 = face.winding[(i + 1) % face.winding.size()];

					Vec3 inward(-(p1.y - p0.y), p1.x - p0.x, 0.0);
					double len = std::sqrt(inward.x * inward.x + inward.y * inward.y);
					if (len < 1e-9)
						continue;
					inward = inward * (1.0 / len);
					if (inward.x * (center.x - p0.x) + inward.y * (center.y - p0.y) < 0.0)
						inward = inward * -1.0;

					double f0 = inward.x * (side.a.x - p0.x) + inward.y * (side.a.y - p0.y);
					double f1 = inward.x * (side.b.x - p0.x) + inward.y * (side.b.y - p0.y);

					if (f0 < 0.0 && f1 < 0.0)
					{
						alive = false;
						break;
					}
					if (f0 < 0.0)
						tEnter = std::max(tEnter, f0 / (f0 - f1));
					else if (f1 < 0.0)
						tExit = std::min(tExit, f0 / (f0 - f1));
				}

				if (!alive || tExit - tEnter < 1e-6)
					continue;

				Vec3 span = side.b - side.a;
				Vec3 a = side.a + span * tEnter;
				Vec3 b = side.a + span * tExit;

				const Plane &pl = face.plane;
				if (std::fabs(pl.normal.z) < 1e-9)
					continue;
				a.z = (pl.dist - pl.normal.x * a.x - pl.normal.y * a.y) / pl.normal.z;
				b.z = (pl.dist - pl.normal.x * b.x - pl.normal.y * b.y) / pl.normal.z;

				LaunchEdge edge;
				edge.a = a;
				edge.b = b;
				edge.outward = side.inward;
				edge.noJumpBack = true;
				edge.length = (b - a).Length();
				if (edge.length < 1.0)
					continue;

				edges.push_back(edge);
			}
		}
	}

	return edges;
}

int FindAreaStrict(const Vec3 &point, const std::vector<Area> &areas)
{
	for (const Area &area : areas)
	{
		bool inside = true;
		for (const Plane &p : area.boundPlanes)
		{
			if (point.Dot(p.normal) - p.dist < -1.0)
			{
				inside = false;
				break;
			}
		}
		if (inside)
			return area.id;
	}
	return -1;
}

PM_MoveSimulator::PM_MoveSimulator(const BspFile &bsp, int headnode, const std::vector<BlockingVolume> &blockers,
	const AirmoveSampleParams &params, const PhysicsParams &physics,
	const std::vector<BlockingVolume> *ladders, const std::vector<PushVolume> *pushes)
	: m_bsp(bsp)
	, m_headnode(headnode)
	, m_blockers(blockers)
	, m_ladders(ladders)
	, m_pushes(pushes)
	, m_params(params)
{
	m_pmPhysics.maxSpeed = physics.maxSpeed;
	m_pmPhysics.gravity = physics.gravity;
	m_pmPhysics.airAccelerate = physics.airAccelerate;
	m_pmPhysics.friction = physics.friction;
	m_pmPhysics.frameTime = params.frameTime;

	const dmodel_t &world = bsp.Models()[0];
	m_worldMins = Vec3(world.mins[0] - kWorldBoundsSlop, world.mins[1] - kWorldBoundsSlop,
		world.mins[2] - kWorldBoundsSlop);
	m_worldMaxs = Vec3(world.maxs[0] + kWorldBoundsSlop, world.maxs[1] + kWorldBoundsSlop,
		world.maxs[2] + kWorldBoundsSlop);
}

TraceResult PM_MoveSimulator::Trace(const Vec3 &start, const Vec3 &end) const
{
	return TraceHullWithEntities(m_bsp, m_headnode, m_blockers, start, end);
}

PmSim::PmTraceFn PM_MoveSimulator::TraceFn() const
{
	const BspFile *bsp = &m_bsp;
	int headnode = m_headnode;
	const std::vector<BlockingVolume> *blockers = &m_blockers;

	return [bsp, headnode, blockers](const Vector &start, const Vector &end)
	{
		TraceResult tr = TraceHullWithEntities(*bsp, headnode, *blockers,
			Vec3(start.x, start.y, start.z), Vec3(end.x, end.y, end.z));

		PmSim::PmTrace out;
		out.endpos = Vector(static_cast<float>(tr.endpos.x), static_cast<float>(tr.endpos.y),
			static_cast<float>(tr.endpos.z));
		out.planeNormal = Vector(static_cast<float>(tr.planeNormal.x), static_cast<float>(tr.planeNormal.y),
			static_cast<float>(tr.planeNormal.z));
		out.fraction = tr.fraction;
		out.allsolid = tr.allsolid;
		out.startsolid = tr.startsolid;
		return out;
	};
}

PmSim::PmPushFn PM_MoveSimulator::PushFn() const
{
	if (!m_pushes || m_pushes->empty())
		return nullptr;

	const PM_MoveSimulator *self = this;
	return [self](const Vector &origin)
	{
		Vec3 push = self->PushAt(Vec3(origin.x, origin.y, origin.z));
		return Vector(static_cast<float>(push.x), static_cast<float>(push.y),
			static_cast<float>(push.z));
	};
}

Vec3 PM_MoveSimulator::PushAt(const Vec3 &origin) const
{
	Vec3 total(0.0, 0.0, 0.0);
	if (!m_pushes)
		return total;

	for (const PushVolume &vol : *m_pushes)
	{
		if (origin.x < vol.volume.mins.x || origin.x > vol.volume.maxs.x ||
			origin.y < vol.volume.mins.y || origin.y > vol.volume.maxs.y ||
			origin.z < vol.volume.mins.z || origin.z > vol.volume.maxs.z)
			continue;
		if (ClassifyPoint(m_bsp, vol.volume.headnode, origin - vol.volume.origin) != CONTENTS_SOLID)
			continue;

		total = total + vol.push;
	}

	return total;
}

bool PM_MoveSimulator::OnLadder(const Vec3 &origin) const
{
	if (!m_ladders)
		return false;

	for (const BlockingVolume &vol : *m_ladders)
	{
		if (origin.x < vol.mins.x || origin.x > vol.maxs.x ||
			origin.y < vol.mins.y || origin.y > vol.maxs.y ||
			origin.z < vol.mins.z || origin.z > vol.maxs.z)
			continue;
		if (ClassifyPoint(m_bsp, vol.headnode, origin - vol.origin) == CONTENTS_SOLID)
			return true;
	}

	return false;
}

bool PM_MoveSimulator::IsSupported(const Vec3 &origin) const
{
	TraceResult trace = Trace(origin, Vec3(origin.x, origin.y, origin.z - kGroundTraceDist));
	return trace.fraction < 1.0f && trace.planeNormal.z >= kGroundNormalZ;
}

bool PM_MoveSimulator::VoidAhead(const Vec3 &origin, const Vec3 &dir) const
{
	return !GroundAhead(origin, dir) && !BlockedAhead(origin, dir);
}

bool PM_MoveSimulator::BlockedAhead(const Vec3 &origin, const Vec3 &dir) const
{
	Vec3 ahead(origin.x + dir.x * m_params.edgeProbe, origin.y + dir.y * m_params.edgeProbe, origin.z);
	TraceResult trace = Trace(origin, ahead);
	return trace.allsolid || trace.startsolid || trace.fraction < 1.0f;
}

bool PM_MoveSimulator::GroundAhead(const Vec3 &origin, const Vec3 &dir) const
{

	TraceResult up = Trace(origin, Vec3(origin.x, origin.y, origin.z + m_params.stepHeight));
	Vec3 raised = up.startsolid ? origin : up.endpos;

	Vec3 ahead(origin.x + dir.x * m_params.edgeProbe, origin.y + dir.y * m_params.edgeProbe, raised.z);
	TraceResult forward = Trace(raised, ahead);
	if (!forward.startsolid && forward.fraction > 0.0f)
		ahead = forward.endpos;

	TraceResult trace = Trace(ahead,
		Vec3(ahead.x, ahead.y, origin.z - m_params.stepHeight - kGroundTraceDist));

	if (trace.allsolid || trace.startsolid)
		return false;

	return trace.fraction < 1.0f && trace.planeNormal.z >= kGroundNormalZ;
}

AirmoveTrajectory PM_MoveSimulator::Simulate(const Vec3 &launchPos, float yawDegrees, float commandedSpeed,
	bool jump, const std::vector<Area> &areas, int sourceArea, int &landingArea, double navZAdjust) const
{
	AirmoveTrajectory result;
	result.launchPos = Vec3(launchPos.x, launchPos.y, launchPos.z + navZAdjust);
	result.exitSpeed = commandedSpeed;
	result.exitYaw = yawDegrees;
	result.jumped = jump;
	landingArea = -1;

	PmSim::PmSimulator sim(TraceFn(), m_pmPhysics, PushFn());
	sim.Begin(Vector(static_cast<float>(launchPos.x), static_cast<float>(launchPos.y),
		static_cast<float>(launchPos.z)), yawDegrees, commandedSpeed, jump);

	return Fly(sim, result, false, areas, sourceArea, landingArea, navZAdjust);
}

AirmoveTrajectory PM_MoveSimulator::SimulateAirborne(const Vec3 &launchPos, const Vec3 &velocity,
	float yawDegrees, float commandedSpeed, const std::vector<Area> &areas, int sourceArea,
	int &landingArea, double navZAdjust) const
{
	AirmoveTrajectory result;
	result.launchPos = Vec3(launchPos.x, launchPos.y, launchPos.z + navZAdjust);
	result.exitSpeed = commandedSpeed;
	result.exitYaw = yawDegrees;
	landingArea = -1;

	PmSim::PmSimulator sim(TraceFn(), m_pmPhysics, PushFn());
	sim.BeginAirborne(Vector(static_cast<float>(launchPos.x), static_cast<float>(launchPos.y),
		static_cast<float>(launchPos.z)),
		Vector(static_cast<float>(velocity.x), static_cast<float>(velocity.y),
			static_cast<float>(velocity.z)), yawDegrees, commandedSpeed);

	return Fly(sim, result, true, areas, sourceArea, landingArea, navZAdjust);
}

AirmoveTrajectory PM_MoveSimulator::Fly(PmSim::PmSimulator &sim, AirmoveTrajectory &result, bool airborne,
	const std::vector<Area> &areas, int sourceArea, int &landingArea, double navZAdjust) const
{
	while (sim.Time() < m_params.maxSimulationTime)
	{
		if (!sim.Step())
			return result;

		const Vector &pos = sim.Origin();

		if (!sim.OnGround())
		{
			airborne = true;
			if (OnLadder(Vec3(pos.x, pos.y, pos.z)))
			{
				result.grabbedLadder = true;
				result.ladderPos = Vec3(pos.x, pos.y, pos.z + navZAdjust);
				result.flightTime = sim.Time();
				return result;
			}
		}
		else if (airborne)
		{
			Vec3 landing(pos.x, pos.y, pos.z + navZAdjust);
			int area = FindAreaStrict(landing, areas);

			if (area != sourceArea)
			{
				if (area < 0)
					return result;

				result.landingPos = landing;
				result.flightTime = sim.Time();
				result.landed = true;
				result.fallDamage = PmSim::FallDamageFor(sim.FallSpeed());
				landingArea = area;
				return result;
			}

			if (sim.Time() > m_params.groundGraceTime)
				return result;

			sim.ResetFallSpeed();
			airborne = false;
		}
		else if (sim.Time() > m_params.groundGraceTime)
		{
			return result;
		}

		if (pos.x < m_worldMins.x || pos.y < m_worldMins.y || pos.z < m_worldMins.z ||
			pos.x > m_worldMaxs.x || pos.y > m_worldMaxs.y || pos.z > m_worldMaxs.z)
			return result;
	}

	return result;
}

namespace
{

struct WorldConfig
{
	std::vector<BlockingVolume> duck;
	std::vector<BlockingVolume> stand;
	std::unique_ptr<PM_MoveSimulator> standSim;
	std::unique_ptr<PM_MoveSimulator> crouchJumpSim;
	std::unique_ptr<PM_MoveSimulator> duckSim;
};

struct WorkItem
{
	LaunchEdge edge;
	Vec3 mid;
	size_t areaIndex = 0;
	int worldIndex = 0;
	int modeIndex = 0;
	bool seeded = false;
	int edgeSamples = 0;
	uint64_t seed = 0;
	uint64_t id = 0;
};

struct EnumRequest
{
	size_t areaIndex = 0;
	int phase = 0;
	std::vector<LaunchEdge> seed;
};

struct SampleOutput
{
	std::map<long long, BinEntry> bins[kBinCount];
	size_t simulated = 0;
	std::unordered_set<int> sourceAreas;
	std::vector<size_t> recorded;
};

int ModesFor(const Area &area, int *out)
{
	if (area.areaFlags & crossbot_nav::NAV_AREA_CROUCH)
	{
		out[0] = kModeDuck;
		return 1;
	}

	out[0] = kModeStand;
	out[1] = kModeCrouchJump;
	return 2;
}

}

void ProbeAirMoves(const BspFile &bsp, const HullContext &duckHull, const HullContext &standHull,
	const AirmoveWorlds &worlds, const std::vector<Area> &areas, std::vector<MergedConnection> &conns,
	const PhysicsParams &physics, const AirmoveSampleParams &params, bool dumpElevators)
{
	std::unordered_set<long long> existingPairs;
	for (const MergedConnection &c : conns)
	{
		if (c.connType == crossbot_nav::CONN_AIRMOVE)
			continue;

		existingPairs.insert(PairKey(c.areaA, c.areaB));
		existingPairs.insert(PairKey(c.areaB, c.areaA));
	}

	PhysicsParams duckPhysics = physics;
	duckPhysics.maxSpeed = physics.maxSpeed * physics.duckSpeedMultiplier;

	std::vector<std::unique_ptr<WorldConfig>> configs;
	std::vector<int> worldOfStop(worlds.stops.size(), 0);

	auto composeWorld = [&](int placedStop)
	{
		std::unique_ptr<WorldConfig> config(new WorldConfig());

		std::vector<int> selected = worlds.defaultStopOf;
		if (placedStop >= 0)
		{
			const AirmoveWorldStop &stop = worlds.stops[static_cast<size_t>(placedStop)];
			if (stop.moverId >= 0 && static_cast<size_t>(stop.moverId) < selected.size())
				selected[static_cast<size_t>(stop.moverId)] = placedStop;
		}

		config->duck = worlds.baseDuck;
		config->stand = worlds.baseStand;
		for (int stopIndex : selected)
		{
			if (stopIndex < 0 || static_cast<size_t>(stopIndex) >= worlds.stops.size())
				continue;

			const AirmoveWorldStop &stop = worlds.stops[static_cast<size_t>(stopIndex)];
			config->duck.push_back(stop.duckVolume);
			config->stand.push_back(stop.standVolume);
		}

		config->standSim.reset(new PM_MoveSimulator(bsp, standHull.headnode, config->stand, params,
			physics, standHull.ladders, standHull.pushes));
		config->crouchJumpSim.reset(new PM_MoveSimulator(bsp, duckHull.headnode, config->duck, params,
			physics, duckHull.ladders, duckHull.pushes));
		config->duckSim.reset(new PM_MoveSimulator(bsp, duckHull.headnode, config->duck, params,
			duckPhysics, duckHull.ladders, duckHull.pushes));

		configs.push_back(std::move(config));
		return static_cast<int>(configs.size()) - 1;
	};

	composeWorld(-1);
	for (size_t s = 0; s < worlds.stops.size(); ++s)
	{
		const AirmoveWorldStop &stop = worlds.stops[s];
		if (stop.isDefault || stop.moverId < 0 ||
			static_cast<size_t>(stop.moverId) >= worlds.defaultStopOf.size())
			continue;

		worldOfStop[s] = composeWorld(static_cast<int>(s));
	}

	SweepMode modes[kModeCount];

	modes[kModeStand].name = "stand";
	modes[kModeStand].probeZ = kStandProbeLift;
	modes[kModeStand].launchZ = kStandProbeLift;
	modes[kModeStand].navZAdjust = -kStandProbeLift;
	modes[kModeStand].maxSpeed = physics.maxSpeed;

	modes[kModeCrouchJump].name = "crouchjump";
	modes[kModeCrouchJump].launchZ = kStandProbeLift;
	modes[kModeCrouchJump].maxSpeed = physics.maxSpeed;
	modes[kModeCrouchJump].duckHull = true;
	modes[kModeCrouchJump].drop = false;

	modes[kModeDuck].name = "duck";
	modes[kModeDuck].maxSpeed = duckPhysics.maxSpeed;
	modes[kModeDuck].duckHull = true;

	std::vector<int> worldOfArea(areas.size(), 0);
	for (size_t a = 0; a < areas.size(); ++a)
	{
		int stopIndex = areas[a].elevatorStopId;
		if (stopIndex >= 0 && static_cast<size_t>(stopIndex) < worldOfStop.size())
			worldOfArea[a] = worldOfStop[static_cast<size_t>(stopIndex)];
	}

	auto simFor = [&](int worldIndex, int modeIndex) -> const PM_MoveSimulator &
	{
		const WorldConfig &config = *configs[static_cast<size_t>(worldIndex)];
		if (modeIndex == kModeCrouchJump)
			return *config.crouchJumpSim;
		if (modeIndex == kModeDuck)
			return *config.duckSim;
		return *config.standSim;
	};

	size_t threads = (params.threadCount > 0) ? params.threadCount : DefaultThreadCount();

	std::vector<size_t> areaLaunchEdges(areas.size(), 0);
	uint64_t nextItemId = 0;
	double enumSeconds = 0.0;
	double sampleSeconds = 0.0;
	std::vector<double> binTotals(threads, 0.0);

	auto enumerate = [&](const std::vector<EnumRequest> &requests)
	{
		std::vector<WorkItem> items;
		if (requests.empty())
			return items;

		std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();

		std::vector<double> cost(requests.size(), 0.0);
		std::vector<std::pair<uint64_t, size_t>> keys(requests.size());
		for (size_t r = 0; r < requests.size(); ++r)
		{
			const Area &area = areas[requests[r].areaIndex];
			double weight = 0.0;
			if (requests[r].seed.empty())
			{
				for (const Face &face : area.polytope.faces)
				{
					if (face.walkable && face.plane.normal.z > kFloorNormalZ)
						weight += static_cast<double>(face.winding.size());
				}
			}
			else
			{
				weight = static_cast<double>(requests[r].seed.size());
			}

			cost[r] = std::max(1.0, weight);
			keys[r] = std::make_pair(MortonKey(AreaCenter(area), kAirmoveMortonCell), r);
		}

		std::sort(keys.begin(), keys.end());
		std::vector<size_t> order(requests.size());
		for (size_t k = 0; k < keys.size(); ++k)
			order[k] = keys[k].second;

		size_t binCount = std::min(threads, requests.size());
		std::vector<std::vector<size_t>> bins = BuildWorkBins(cost, order, kAirmoveTileItems, binCount);

		std::vector<std::vector<WorkItem>> perRequest(requests.size());
		std::vector<size_t> perRequestEdges(requests.size(), 0);

		RunBins(binCount, [&](size_t binIndex)
		{
			for (size_t r : bins[binIndex])
			{
				const EnumRequest &request = requests[r];
				const Area &area = areas[request.areaIndex];
				int worldIndex = worldOfArea[request.areaIndex];

				int modeList[kModeCount];
				int modeCount = ModesFor(area, modeList);

				for (int m = 0; m < modeCount; ++m)
				{
					int modeIndex = modeList[m];
					const std::vector<LaunchEdge> *edges = &request.seed;

					std::vector<LaunchEdge> perimeter;
					if (request.seed.empty())
					{
						perimeter = FilterLaunchEdges(simFor(worldIndex, modeIndex), area,
							modes[modeIndex].probeZ, params);
						edges = &perimeter;
					}

					perRequestEdges[r] += edges->size();

					for (size_t e = 0; e < edges->size(); ++e)
					{
						const LaunchEdge &edge = (*edges)[e];
						int edgeSamples = static_cast<int>(std::lround(
							edge.length * static_cast<double>(params.samplesPerUnit)));
						if (edgeSamples <= 0)
							continue;

						WorkItem item;
						item.edge = edge;
						item.mid = (edge.a + edge.b) * 0.5;
						item.areaIndex = request.areaIndex;
						item.worldIndex = worldIndex;
						item.modeIndex = modeIndex;
						item.seeded = !request.seed.empty();
						item.edgeSamples = edgeSamples;
						item.seed = EdgeSeed(params.seed, request.phase, area.id, modeIndex, e);
						perRequest[r].push_back(item);
					}
				}
			}
		});

		for (size_t r = 0; r < requests.size(); ++r)
		{
			areaLaunchEdges[requests[r].areaIndex] += perRequestEdges[r];
			for (WorkItem &item : perRequest[r])
			{
				item.id = nextItemId++;
				items.push_back(item);
			}
		}

		enumSeconds += std::chrono::duration<double>(
			std::chrono::steady_clock::now() - start).count();

		return items;
	};

	auto sample = [&](const std::vector<WorkItem> &items, SampleOutput &out)
	{
		if (items.empty())
			return;

		std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();

		std::vector<double> cost(items.size(), 0.0);
		std::vector<std::pair<uint64_t, size_t>> keys(items.size());
		for (size_t i = 0; i < items.size(); ++i)
		{
			const SweepMode &mode = modes[items[i].modeIndex];
			int variants = (mode.jump ? 1 : 0) + ((mode.drop && !items[i].seeded) ? 1 : 0);
			cost[i] = static_cast<double>(items[i].edgeSamples) *
				static_cast<double>(std::max(1, variants));
			keys[i] = std::make_pair(MortonKey(items[i].mid, kAirmoveMortonCell), i);
		}

		std::sort(keys.begin(), keys.end());
		std::vector<size_t> order(items.size());
		for (size_t k = 0; k < keys.size(); ++k)
			order[k] = keys[k].second;

		size_t binCount = std::min(threads, items.size());
		std::vector<std::vector<size_t>> bins = BuildWorkBins(cost, order, kAirmoveTileItems, binCount);

		std::vector<SampleOutput> parts(binCount);
		std::vector<double> binWall(binCount, 0.0);

		RunBins(binCount, [&](size_t binIndex)
		{
			std::chrono::steady_clock::time_point binStart = std::chrono::steady_clock::now();

			SampleOutput &local = parts[binIndex];
			local.recorded.assign(areas.size(), 0);

			for (size_t index : bins[binIndex])
			{
				const WorkItem &item = items[index];
				const Area &area = areas[item.areaIndex];

				SweepMode mode = modes[item.modeIndex];
				if (item.seeded)
					mode.drop = false;
				mode.sim = &simFor(item.worldIndex, item.modeIndex);

				std::mt19937 rng(static_cast<uint32_t>(item.seed));
				std::uniform_real_distribution<float> unit(0.0f, 1.0f);

				float baseYaw = static_cast<float>(
					std::atan2(item.edge.outward.y, item.edge.outward.x) * 180.0 / kPi);

				for (int s = 0; s < item.edgeSamples; ++s)
				{
					double t = unit(rng);
					float yaw = baseYaw + (unit(rng) * 2.0f - 1.0f) * params.yawSpread;
					float speed = unit(rng) * mode.maxSpeed;

					float jumpBack = item.edge.noJumpBack ? 0.0f : unit(rng) * params.jumpBackMax;

					Vec3 edgePos = item.edge.a + (item.edge.b - item.edge.a) * t;

					for (int variant = 0; variant < 2; ++variant)
					{
						bool jump = (variant == 1);
						if (jump ? !mode.jump : !mode.drop)
							continue;

						double setBack = params.inset + (jump ? static_cast<double>(jumpBack) : 0.0);
						Vec3 launchPos = edgePos - item.edge.outward * setBack;
						launchPos.z += params.lift;

						if (!mode.sim->IsSupported(Vec3(launchPos.x, launchPos.y,
							launchPos.z + mode.probeZ)))
							continue;

						if (item.seeded && FindAreaStrict(launchPos, areas) != area.id)
							continue;

						launchPos.z += mode.launchZ;

						int landingArea = -1;
						AirmoveTrajectory traj = mode.sim->Simulate(launchPos, yaw, speed, jump, areas,
							area.id, landingArea, mode.navZAdjust);
						traj.jumpBackDist = jump ? jumpBack : 0.0f;
						traj.duckHull = mode.duckHull;

						traj.launchPos.z -= mode.launchZ - mode.probeZ;
						++local.simulated;

						if (!traj.landed || landingArea < 0)
							continue;

						long long key = PairKey(area.id, landingArea);
						if (existingPairs.count(key))
							continue;

						AirmoveSample record;
						record.launchPos = traj.launchPos;
						record.landingPos = traj.landingPos;
						record.yaw = traj.exitYaw;
						record.speed = traj.exitSpeed;
						record.traj = traj;
						record.itemId = item.id;
						record.seq = static_cast<uint32_t>(s) * 2u + static_cast<uint32_t>(variant);

						local.bins[BinFor(mode.duckHull, jump)][key].samples.push_back(record);
						local.sourceAreas.insert(area.id);
						++local.recorded[item.areaIndex];
					}
				}
			}

			binWall[binIndex] = std::chrono::duration<double>(
				std::chrono::steady_clock::now() - binStart).count();
		});

		for (size_t b = 0; b < binCount; ++b)
		{
			out.simulated += parts[b].simulated;
			for (int id : parts[b].sourceAreas)
				out.sourceAreas.insert(id);

			for (size_t a = 0; a < areas.size(); ++a)
				out.recorded[a] += parts[b].recorded[a];

			for (int k = 0; k < kBinCount; ++k)
			{
				for (const std::pair<const long long, BinEntry> &entry : parts[b].bins[k])
				{
					std::vector<AirmoveSample> &dst = out.bins[k][entry.first].samples;
					dst.insert(dst.end(), entry.second.samples.begin(), entry.second.samples.end());
				}
			}
		}

		sampleSeconds += std::chrono::duration<double>(
			std::chrono::steady_clock::now() - start).count();
		for (size_t b = 0; b < binCount; ++b)
			binTotals[b] += binWall[b];
	};

	auto finalize = [&](SampleOutput &target)
	{
		for (int k = 0; k < kBinCount; ++k)
		{
			for (std::pair<const long long, BinEntry> &entry : target.bins[k])
			{
				std::vector<AirmoveSample> &samples = entry.second.samples;
				std::sort(samples.begin(), samples.end(), SampleOrderLess);

				for (size_t i = 0; i < samples.size(); ++i)
				{
					if (i == 0 || BetterTrajectory(samples[i].traj, entry.second.best))
						entry.second.best = samples[i].traj;
				}
			}
		}
	};

	SampleOutput out;
	out.recorded.assign(areas.size(), 0);

	std::vector<EnumRequest> requests;
	for (size_t a = 0; a < areas.size(); ++a)
	{
		EnumRequest request;
		request.areaIndex = a;
		request.phase = 0;
		requests.push_back(request);
	}

	size_t pushSeedAreas = 0;
	size_t pushSeedEdges = 0;
	for (size_t a = 0; a < areas.size(); ++a)
	{
		bool crouch = (areas[a].areaFlags & crossbot_nav::NAV_AREA_CROUCH) != 0;
		const std::vector<PushVolume> *volumes = crouch ? duckHull.pushes : standHull.pushes;
		if (!volumes || volumes->empty())
			continue;

		std::vector<LaunchEdge> seed = PushLaunchEdges(areas[a], *volumes);
		if (seed.empty())
			continue;

		pushSeedEdges += seed.size();
		++pushSeedAreas;

		EnumRequest request;
		request.areaIndex = a;
		request.phase = 1;
		request.seed = seed;
		requests.push_back(request);
	}

	sample(enumerate(requests), out);
	finalize(out);

	double maxJumpHeight = static_cast<double>(params.maxJumpHeight) -
		PmSim::PlainJumpHeight(PmSim::kGravity) + PmSim::PlainJumpHeight(physics.gravity);

	std::vector<EnumRequest> reverse;
	for (int bin : {kBinStandDrop, kBinDuckDrop})
	{
		for (const std::pair<const long long, BinEntry> &entry : out.bins[bin])
		{
			int from = static_cast<int>(entry.first / 1000000LL);
			int to = static_cast<int>(entry.first % 1000000LL);
			const AirmoveTrajectory &drop = entry.second.best;

			double rise = drop.launchPos.z - drop.landingPos.z;
			if (rise <= 0.0 || rise > maxJumpHeight)
				continue;
			if (existingPairs.count(PairKey(to, from)) || static_cast<size_t>(to) >= areas.size())
				continue;

			LaunchEdge seed;
			if (!SeedEdgeFromLandings(entry.second, seed))
				continue;

			EnumRequest request;
			request.areaIndex = static_cast<size_t>(to);
			request.phase = 2;
			request.seed.push_back(seed);
			reverse.push_back(request);
		}
	}

	size_t reverseSeeds = reverse.size();
	if (!reverse.empty())
	{
		sample(enumerate(reverse), out);
		finalize(out);
	}

	if (dumpElevators)
	{
		for (size_t a = 0; a < areas.size(); ++a)
		{
			int stopIndex = areas[a].elevatorStopId;
			if (stopIndex < 0 || static_cast<size_t>(stopIndex) >= worlds.stops.size())
				continue;

			const AirmoveWorldStop &stop = worlds.stops[static_cast<size_t>(stopIndex)];
			if (stop.isDefault || stop.moverId < 0)
				continue;

			bool crouch = (areas[a].areaFlags & crossbot_nav::NAV_AREA_CROUCH) != 0;
			int modeIndex = crouch ? kModeDuck : kModeStand;
			double probeZ = crouch ? 0.0 : kStandProbeLift;

			std::vector<LaunchEdge> perimeter = CollectPerimeterEdges(areas[a]);
			size_t parked = CountSupportedEdges(simFor(0, modeIndex), perimeter, probeZ, params);
			size_t placed = CountSupportedEdges(simFor(worldOfArea[a], modeIndex), perimeter, probeZ,
				params);

			std::printf("  airmove area %d (mover %d stop %d placed): %zu perimeter edges, "
				"support %zu parked -> %zu placed, %zu launch edges, %zu samples\n",
				areas[a].id, stop.moverId, stopIndex, perimeter.size(), parked, placed,
				areaLaunchEdges[a], out.recorded[a]);
		}
	}

	std::map<long long, bool> emitted;
	size_t unfitted = 0;
	for (int bin = 0; bin < kBinCount; ++bin)
	{
		for (const std::pair<const long long, BinEntry> &entry : out.bins[bin])
		{
			if (emitted.count(entry.first))
				continue;

			const AirmoveTrajectory &traj = entry.second.best;

			MergedConnection conn;
			conn.areaA = static_cast<int>(entry.first / 1000000LL);
			conn.areaB = static_cast<int>(entry.first % 1000000LL);
			conn.pointA = traj.launchPos;
			conn.pointB = traj.landingPos;
			if (!BuildSampledGates(entry.second, conn))
			{
				++unfitted;
				continue;
			}

			emitted.emplace(entry.first, traj.duckHull);
			conn.connType = crossbot_nav::CONN_AIRMOVE;
			conn.connFlags = crossbot_nav::CONN_FLAG_ONE_WAY;
			if (traj.duckHull)
				conn.connFlags |= crossbot_nav::CONN_FLAG_NEEDS_CROUCH;
			if (traj.jumped)
				conn.connFlags |= crossbot_nav::CONN_FLAG_JUMP;
			conn.exitSpeed = traj.exitSpeed;
			conn.exitAngle = traj.exitYaw;
			conn.jumpBackDist = traj.jumpBackDist;
			conn.fallDamage = traj.fallDamage;
			conn.flightTime = traj.flightTime;
			conn.gravity = physics.gravityMultiplier;
			conns.push_back(conn);
		}
	}

	size_t crouchOnly = 0;
	for (const std::pair<const long long, bool> &entry : emitted)
	{
		if (entry.second)
			++crouchOnly;
	}

	std::printf("air move sampling at gravity x%.2f: %zu trajectories at %.2f samples/unit from %zu areas "
		"(%zu launched), "
		"%zu pairs [stand drop %zu, stand jump %zu, duck drop %zu, duck jump %zu] -> %zu connections "
		"(%zu crouch-only, %zu drops seeded a climb back under %.0f units, "
		"%zu dropped for too few samples to fit a gate)\n",
		physics.gravityMultiplier,
		out.simulated, params.samplesPerUnit, areas.size(), out.sourceAreas.size(),
		out.bins[kBinStandDrop].size() + out.bins[kBinStandJump].size() +
			out.bins[kBinDuckDrop].size() + out.bins[kBinDuckJump].size(),
		out.bins[kBinStandDrop].size(), out.bins[kBinStandJump].size(),
		out.bins[kBinDuckDrop].size(), out.bins[kBinDuckJump].size(), emitted.size(), crouchOnly,
		reverseSeeds, maxJumpHeight, unfitted);

	std::printf("  air move sampling on %zu threads: %.1fs launch-edge scan + %.1fs simulation "
		"(slowest bin %.1fs, fastest %.1fs)\n",
		threads, enumSeconds, sampleSeconds,
		*std::max_element(binTotals.begin(), binTotals.end()),
		*std::min_element(binTotals.begin(), binTotals.end()));

	if (pushSeedEdges > 0)
		std::printf("  %zu push-field launch edges seeded across %zu areas\n", pushSeedEdges,
			pushSeedAreas);
}

}
