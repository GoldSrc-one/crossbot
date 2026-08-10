#include "airmove_sampler.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <map>
#include <random>
#include <unordered_set>
#include <utility>

#include "hull_defs.h"
#include "nav_format.h"
#include "nav_math.h"

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
};

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
	const std::vector<BlockingVolume> *ladders)
	: m_bsp(bsp)
	, m_headnode(headnode)
	, m_blockers(blockers)
	, m_ladders(ladders)
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

	PmSim::PmSimulator sim(TraceFn(), m_pmPhysics);
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

	PmSim::PmSimulator sim(TraceFn(), m_pmPhysics);
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

void ProbeAirMoves(const BspFile &bsp, const HullContext &duckHull, const HullContext &standHull,
	const std::vector<Area> &areas, std::vector<MergedConnection> &conns,
	const PhysicsParams &physics, const AirmoveSampleParams &params)
{
	std::unordered_set<long long> existingPairs;
	for (const MergedConnection &c : conns)
	{
		existingPairs.insert(PairKey(c.areaA, c.areaB));
		existingPairs.insert(PairKey(c.areaB, c.areaA));
	}

	PhysicsParams duckPhysics = physics;
	duckPhysics.maxSpeed = physics.maxSpeed * physics.duckSpeedMultiplier;

	PM_MoveSimulator standSim(bsp, standHull.headnode, *standHull.blockers, params, physics,
		standHull.ladders);
	PM_MoveSimulator crouchJumpSim(bsp, duckHull.headnode, *duckHull.blockers, params, physics,
		duckHull.ladders);
	PM_MoveSimulator duckSim(bsp, duckHull.headnode, *duckHull.blockers, params, duckPhysics,
		duckHull.ladders);

	SweepMode standSweep;
	standSweep.sim = &standSim;
	standSweep.name = "stand";
	standSweep.probeZ = kStandProbeLift;
	standSweep.launchZ = kStandProbeLift;
	standSweep.navZAdjust = -kStandProbeLift;
	standSweep.maxSpeed = physics.maxSpeed;

	SweepMode crouchJumpSweep;
	crouchJumpSweep.sim = &crouchJumpSim;
	crouchJumpSweep.name = "crouchjump";
	crouchJumpSweep.launchZ = kStandProbeLift;
	crouchJumpSweep.maxSpeed = physics.maxSpeed;
	crouchJumpSweep.duckHull = true;
	crouchJumpSweep.drop = false;

	SweepMode duckSweep;
	duckSweep.sim = &duckSim;
	duckSweep.name = "duck";
	duckSweep.maxSpeed = duckPhysics.maxSpeed;
	duckSweep.duckHull = true;

	std::mt19937 rng(params.seed);
	std::uniform_real_distribution<float> unit(0.0f, 1.0f);

	std::map<long long, BinEntry> bins[kBinCount];
	size_t simulated = 0;
	std::unordered_set<int> sourceAreas;
	size_t unfitted = 0;

	auto runSweep = [&](const SweepMode &mode, const Area &area, const std::vector<LaunchEdge> &edges,
		bool requireLaunchInside)
	{
		for (const LaunchEdge &edge : edges)
		{
			int edgeSamples = static_cast<int>(std::lround(edge.length * static_cast<double>(params.samplesPerUnit)));
			if (edgeSamples <= 0)
				continue;

			float baseYaw = static_cast<float>(std::atan2(edge.outward.y, edge.outward.x) * 180.0 / kPi);

			for (int s = 0; s < edgeSamples; ++s)
			{
				double t = unit(rng);
				float yaw = baseYaw + (unit(rng) * 2.0f - 1.0f) * params.yawSpread;
				float speed = unit(rng) * mode.maxSpeed;

				float jumpBack = unit(rng) * params.jumpBackMax;

				Vec3 edgePos = edge.a + (edge.b - edge.a) * t;

				for (int variant = 0; variant < 2; ++variant)
				{
					bool jump = (variant == 1);
					if (jump ? !mode.jump : !mode.drop)
						continue;

					double setBack = params.inset + (jump ? static_cast<double>(jumpBack) : 0.0);
					Vec3 launchPos = edgePos - edge.outward * setBack;
					launchPos.z += params.lift;

					if (!mode.sim->IsSupported(Vec3(launchPos.x, launchPos.y, launchPos.z + mode.probeZ)))
						continue;

					if (requireLaunchInside && FindAreaStrict(launchPos, areas) != area.id)
						continue;

					launchPos.z += mode.launchZ;

					int landingArea = -1;
					AirmoveTrajectory traj = mode.sim->Simulate(launchPos, yaw, speed, jump, areas,
						area.id, landingArea, mode.navZAdjust);
					traj.jumpBackDist = jump ? jumpBack : 0.0f;
					traj.duckHull = mode.duckHull;

					traj.launchPos.z -= mode.launchZ - mode.probeZ;
					++simulated;

					if (!traj.landed || landingArea < 0)
						continue;

					long long key = PairKey(area.id, landingArea);
					if (existingPairs.count(key))
						continue;

					std::map<long long, BinEntry> &bin = bins[BinFor(mode.duckHull, jump)];
					BinEntry &entry = bin[key];
					if (entry.samples.empty() || BetterTrajectory(traj, entry.best))
						entry.best = traj;

					AirmoveSample sample;
					sample.launchPos = traj.launchPos;
					sample.landingPos = traj.landingPos;
					sample.yaw = traj.exitYaw;
					sample.speed = traj.exitSpeed;
					entry.samples.push_back(sample);

					sourceAreas.insert(area.id);
				}
			}
		}
	};

	auto sweepArea = [&](const Area &area, const std::vector<LaunchEdge> *seed)
	{
		auto run = [&](const SweepMode &base, const PM_MoveSimulator &sim)
		{
			SweepMode mode = base;
			if (seed)
				mode.drop = false;

			std::vector<LaunchEdge> perimeter;
			if (!seed)
				perimeter = FilterLaunchEdges(sim, area, mode.probeZ, params);

			runSweep(mode, area, seed ? *seed : perimeter, seed != nullptr);
		};

		if (area.areaFlags & crossbot_nav::NAV_AREA_CROUCH)
		{

			run(duckSweep, duckSim);
			return;
		}

		run(standSweep, standSim);
		run(crouchJumpSweep, crouchJumpSim);
	};

	for (const Area &area : areas)
		sweepArea(area, nullptr);

	size_t reverseSeeds = 0;
	for (int bin : {kBinStandDrop, kBinDuckDrop})
	{
		for (const auto &entry : bins[bin])
		{
			int from = static_cast<int>(entry.first / 1000000LL);
			int to = static_cast<int>(entry.first % 1000000LL);
			const AirmoveTrajectory &drop = entry.second.best;

			double rise = drop.launchPos.z - drop.landingPos.z;
			if (rise <= 0.0 || rise > params.maxJumpHeight)
				continue;
			if (existingPairs.count(PairKey(to, from)) || static_cast<size_t>(to) >= areas.size())
				continue;

			LaunchEdge seed;
			if (!SeedEdgeFromLandings(entry.second, seed))
				continue;

			std::vector<LaunchEdge> seedEdges{seed};
			sweepArea(areas[static_cast<size_t>(to)], &seedEdges);
			++reverseSeeds;
		}
	}

	std::map<long long, bool> emitted;
	for (int bin = 0; bin < kBinCount; ++bin)
	{
		for (const auto &entry : bins[bin])
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
			conns.push_back(conn);
		}
	}

	size_t crouchOnly = 0;
	for (const auto &entry : emitted)
		if (entry.second)
			++crouchOnly;

	std::printf("air move sampling: %zu trajectories at %.2f samples/unit from %zu areas (%zu launched), "
		"%zu pairs [stand drop %zu, stand jump %zu, duck drop %zu, duck jump %zu] -> %zu connections "
		"(%zu crouch-only, %zu drops seeded a climb back under %.0f units, "
		"%zu dropped for too few samples to fit a gate)\n",
		simulated, params.samplesPerUnit, areas.size(), sourceAreas.size(),
		bins[kBinStandDrop].size() + bins[kBinStandJump].size() + bins[kBinDuckDrop].size() +
			bins[kBinDuckJump].size(),
		bins[kBinStandDrop].size(), bins[kBinStandJump].size(), bins[kBinDuckDrop].size(),
		bins[kBinDuckJump].size(), emitted.size(), crouchOnly, reverseSeeds, params.maxJumpHeight,
		unfitted);
}

}

