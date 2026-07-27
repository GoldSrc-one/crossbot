#include "airmove_sampler.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <map>
#include <random>
#include <unordered_set>
#include <utility>

#include "nav_format.h"

namespace navgen
{

namespace
{

constexpr double kPi = 3.14159265358979323846;

constexpr float kJumpImpulse = 268.328156f;
constexpr float kBunnyJumpMaxSpeedFactor = 1.7f;
constexpr float kAirAccelWishSpeedCap = 30.0f;
constexpr float kMaxOnGroundVelocityZ = 180.0f;
constexpr float kGroundNormalZ = 0.7f;
constexpr float kGroundTraceDist = 2.0f;

constexpr float kPlayerFatalFallSpeed = 1024.0f;
constexpr float kPlayerMaxSafeFallSpeed = 580.0f;
constexpr float kDamageForFallSpeed = 100.0f / (kPlayerFatalFallSpeed - kPlayerMaxSafeFallSpeed);

constexpr int kMaxClipPlanes = 5;
constexpr double kFloorNormalZ = 0.7;
constexpr double kWorldBoundsSlop = 64.0;

inline void VectorCopy(const float *in, float *out)
{
	out[0] = in[0];
	out[1] = in[1];
	out[2] = in[2];
}

inline float DotProduct(const float *a, const float *b)
{
	return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

inline void CrossProduct(const float *a, const float *b, float *out)
{
	out[0] = a[1] * b[2] - a[2] * b[1];
	out[1] = a[2] * b[0] - a[0] * b[2];
	out[2] = a[0] * b[1] - a[1] * b[0];
}

inline float VectorNormalize(float *v)
{
	float len = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
	if (len > 0.0f)
	{
		float ilen = 1.0f / len;
		v[0] *= ilen;
		v[1] *= ilen;
		v[2] *= ilen;
	}
	return len;
}

struct LaunchEdge
{
	Vec3 a;
	Vec3 b;
	Vec3 outward;
	double length = 0.0;
};

std::vector<LaunchEdge> CollectPerimeterEdges(const Area &area)
{
	std::vector<LaunchEdge> edges;

	for (const Face &face : area.polytope.faces)
	{
		if (face.plane.normal.z <= kFloorNormalZ)
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

int FindAreaStrict(const Vec3 &point, const std::vector<Area> &areas)
{
	for (const Area &area : areas)
	{
		bool inside = true;
		for (const Face &f : area.polytope.faces)
		{
			if (point.Dot(f.plane.normal) - f.plane.dist < -1.0)
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

}

PM_MoveSimulator::PM_MoveSimulator(const BspFile &bsp, int headnode, const std::vector<BlockingVolume> &blockers,
	const AirmoveSampleParams &params, const PhysicsParams &physics)
	: m_bsp(bsp)
	, m_headnode(headnode)
	, m_blockers(blockers)
	, m_params(params)
	, m_physics(physics)
{
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

void PM_MoveSimulator::AddCorrectGravity(float *velocity) const
{
	velocity[2] -= m_physics.gravity * 0.5f * m_params.frameTime;
}

void PM_MoveSimulator::FixupGravityVelocity(float *velocity) const
{
	velocity[2] -= m_physics.gravity * 0.5f * m_params.frameTime;
}

void PM_MoveSimulator::PreventMegaBunnyJumping(float *velocity) const
{
	float maxScaledSpeed = kBunnyJumpMaxSpeedFactor * m_physics.maxSpeed;
	if (maxScaledSpeed <= 0.0f)
		return;

	float spd = std::sqrt(DotProduct(velocity, velocity));
	if (spd <= maxScaledSpeed)
		return;

	float fraction = (maxScaledSpeed / spd) * 0.65f;
	velocity[0] *= fraction;
	velocity[1] *= fraction;
	velocity[2] *= fraction;
}

void PM_MoveSimulator::AirAccelerate(const float *wishdir, float wishspeed, float *velocity) const
{
	float wishspd = wishspeed;
	if (wishspd > kAirAccelWishSpeedCap)
		wishspd = kAirAccelWishSpeedCap;

	float currentspeed = DotProduct(velocity, wishdir);
	float addspeed = wishspd - currentspeed;
	if (addspeed <= 0.0f)
		return;

	float accelspeed = m_physics.airAccelerate * wishspeed * m_params.frameTime * m_physics.friction;
	if (accelspeed > addspeed)
		accelspeed = addspeed;

	velocity[0] += accelspeed * wishdir[0];
	velocity[1] += accelspeed * wishdir[1];
	velocity[2] += accelspeed * wishdir[2];
}

int PM_MoveSimulator::FlyMove(float *origin, float *velocity, float dt) const
{
	int numplanes = 0;
	float planes[kMaxClipPlanes][3] = {{0}};
	float original_velocity[3], new_velocity[3];
	float end[3];
	float time_left = dt;
	int blocked = 0;

	VectorCopy(velocity, original_velocity);

	for (int bumpcount = 0; bumpcount < 4; ++bumpcount)
	{
		if (velocity[0] == 0.0f && velocity[1] == 0.0f && velocity[2] == 0.0f)
			break;

		for (int i = 0; i < 3; ++i)
			end[i] = origin[i] + time_left * velocity[i];

		TraceResult trace = Trace(Vec3(origin[0], origin[1], origin[2]), Vec3(end[0], end[1], end[2]));

		if (trace.allsolid)
		{
			velocity[0] = velocity[1] = velocity[2] = 0.0f;
			return 4;
		}

		if (trace.fraction > 0.0f)
		{
			origin[0] = static_cast<float>(trace.endpos.x);
			origin[1] = static_cast<float>(trace.endpos.y);
			origin[2] = static_cast<float>(trace.endpos.z);
			VectorCopy(velocity, original_velocity);
			numplanes = 0;
		}

		if (trace.fraction == 1.0f)
			break;

		if (trace.planeNormal.z > kGroundNormalZ)
			blocked |= 1;
		if (trace.planeNormal.z == 0.0)
			blocked |= 2;

		time_left -= time_left * trace.fraction;

		if (numplanes >= kMaxClipPlanes)
		{
			velocity[0] = velocity[1] = velocity[2] = 0.0f;
			break;
		}

		planes[numplanes][0] = static_cast<float>(trace.planeNormal.x);
		planes[numplanes][1] = static_cast<float>(trace.planeNormal.y);
		planes[numplanes][2] = static_cast<float>(trace.planeNormal.z);
		numplanes++;

		int i;
		for (i = 0; i < numplanes; ++i)
		{
			float backoff = DotProduct(original_velocity, planes[i]);
			if (backoff < 0.0f)
			{
				for (int j = 0; j < 3; ++j)
					new_velocity[j] = original_velocity[j] - backoff * planes[i][j];
			}
			else
			{
				for (int j = 0; j < 3; ++j)
					new_velocity[j] = original_velocity[j];
			}

			int k;
			for (k = 0; k < numplanes; ++k)
			{
				if (k != i && DotProduct(new_velocity, planes[k]) < 0.0f)
					break;
			}

			if (k == numplanes)
			{
				VectorCopy(new_velocity, velocity);
				VectorCopy(new_velocity, original_velocity);
				break;
			}
		}

		if (i != numplanes)
		{
			if (numplanes == 2)
			{
				float dir[3];
				CrossProduct(planes[0], planes[1], dir);
				float d = DotProduct(dir, velocity);
				for (int k = 0; k < 3; ++k)
					velocity[k] = dir[k] * d;
			}
			else
			{
				velocity[0] = velocity[1] = velocity[2] = 0.0f;
				break;
			}
		}
	}

	return blocked;
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

bool PM_MoveSimulator::CatagorizePosition(const float *origin, const float *velocity) const
{
	if (velocity[2] > kMaxOnGroundVelocityZ)
		return false;

	Vec3 start(origin[0], origin[1], origin[2]);
	Vec3 end(origin[0], origin[1], origin[2] - kGroundTraceDist);
	TraceResult trace = Trace(start, end);

	if (trace.fraction >= 1.0f)
		return false;

	return trace.planeNormal.z >= kGroundNormalZ;
}

AirmoveTrajectory PM_MoveSimulator::Simulate(const Vec3 &launchPos, float yawDegrees, float commandedSpeed,
	bool jump, const std::vector<Area> &areas, int sourceArea, int &landingArea) const
{
	AirmoveTrajectory result;
	result.launchPos = launchPos;
	result.exitSpeed = commandedSpeed;
	result.exitYaw = yawDegrees;
	result.jumped = jump;
	landingArea = -1;

	float yawRad = static_cast<float>(yawDegrees * kPi / 180.0);
	float forward[3] = { std::cos(yawRad), std::sin(yawRad), 0.0f };

	float pos[3] = { static_cast<float>(launchPos.x), static_cast<float>(launchPos.y),
		static_cast<float>(launchPos.z) };
	float vel[3] = { forward[0] * commandedSpeed, forward[1] * commandedSpeed, 0.0f };

	bool onGround = true;
	bool jumpPending = jump;
	float fallVelocity = 0.0f;
	float totalTime = 0.0f;

	while (totalTime < m_params.maxSimulationTime)
	{
		if (!onGround)
			fallVelocity = -vel[2];

		AddCorrectGravity(vel);

		if (jumpPending && onGround)
		{
			PreventMegaBunnyJumping(vel);
			vel[2] = kJumpImpulse;
			FixupGravityVelocity(vel);
			jumpPending = false;
			onGround = false;
		}

		float wishvel[3] = { forward[0] * commandedSpeed, forward[1] * commandedSpeed, 0.0f };
		float wishdir[3] = { wishvel[0], wishvel[1], wishvel[2] };
		float wishspeed = VectorNormalize(wishdir);
		if (wishspeed > m_physics.maxSpeed)
			wishspeed = m_physics.maxSpeed;

		AirAccelerate(wishdir, wishspeed, vel);

		if (FlyMove(pos, vel, m_params.frameTime) == 4)
			return result;

		onGround = CatagorizePosition(pos, vel);

		FixupGravityVelocity(vel);

		totalTime += m_params.frameTime;

		if (onGround)
		{
			vel[2] = 0.0f;

			Vec3 landing(pos[0], pos[1], pos[2]);
			int area = FindAreaStrict(landing, areas);

			if (area != sourceArea)
			{
				if (area < 0)
					return result;

				result.landingPos = landing;
				result.flightTime = totalTime;
				result.landed = true;
				if (fallVelocity > kPlayerMaxSafeFallSpeed)
					result.fallDamage = (fallVelocity - kPlayerMaxSafeFallSpeed) * kDamageForFallSpeed;
				landingArea = area;
				return result;
			}

			if (totalTime > m_params.groundGraceTime)
				return result;

			fallVelocity = 0.0f;
		}

		if (pos[0] < m_worldMins.x || pos[1] < m_worldMins.y || pos[2] < m_worldMins.z ||
			pos[0] > m_worldMaxs.x || pos[1] > m_worldMaxs.y || pos[2] > m_worldMaxs.z)
			return result;
	}

	return result;
}

void ProbeAirMoves(const BspFile &bsp, int headnode, const std::vector<BlockingVolume> &blockers,
	const std::vector<Area> &areas, std::vector<MergedConnection> &conns,
	const PhysicsParams &physics, const AirmoveSampleParams &params)
{
	std::unordered_set<long long> existingPairs;
	for (const MergedConnection &c : conns)
	{
		existingPairs.insert(PairKey(c.areaA, c.areaB));
		existingPairs.insert(PairKey(c.areaB, c.areaA));
	}

	PM_MoveSimulator sim(bsp, headnode, blockers, params, physics);
	std::mt19937 rng(params.seed);
	std::uniform_real_distribution<float> unit(0.0f, 1.0f);

	std::map<long long, AirmoveTrajectory> best;
	std::map<long long, AirmoveTrajectory> bestDrop;
	size_t simulated = 0;
	size_t areasWithEdges = 0;

	for (const Area &area : areas)
	{
		std::vector<LaunchEdge> edges;
		for (const LaunchEdge &edge : CollectPerimeterEdges(area))
		{
			Vec3 mid = (edge.a + edge.b) * 0.5;
			mid.z += params.lift;

			if (!sim.VoidAhead(mid, edge.outward))
				continue;

			edges.push_back(edge);
		}

		if (edges.empty())
			continue;

		bool produced = false;

		for (const LaunchEdge &edge : edges)
		{
			int edgeSamples = static_cast<int>(std::lround(edge.length * static_cast<double>(params.samplesPerUnit)));
			if (edgeSamples <= 0)
				continue;

			float baseYaw = static_cast<float>(std::atan2(edge.outward.y, edge.outward.x) * 180.0 / kPi);

			for (int s = 0; s < edgeSamples; ++s)
			{
				bool jump = unit(rng) < 0.5f;

				// Pure falls step off at the edge; jumps may be initiated further back,
				// clearing the remaining ground on the way out.
				float jumpBack = jump ? unit(rng) * params.jumpBackMax : 0.0f;

				double t = unit(rng);
				Vec3 launchPos = edge.a + (edge.b - edge.a) * t -
					edge.outward * (params.inset + static_cast<double>(jumpBack));
				launchPos.z += params.lift;

				if (!sim.IsSupported(launchPos))
					continue;

				float yaw = baseYaw + (unit(rng) * 2.0f - 1.0f) * params.yawSpread;
				float speed = unit(rng) * physics.maxSpeed;

				int landingArea = -1;
				AirmoveTrajectory traj = sim.Simulate(launchPos, yaw, speed, jump, areas, area.id, landingArea);
				traj.jumpBackDist = jumpBack;
				++simulated;

				if (!traj.landed || landingArea < 0)
					continue;

				long long key = PairKey(area.id, landingArea);
				if (existingPairs.count(key))
					continue;

				auto it = best.find(key);
				if (it == best.end())
				{
					best.emplace(key, traj);
					produced = true;
				}
				else if (BetterTrajectory(traj, it->second))
				{
					it->second = traj;
				}

				if (!traj.jumped)
				{
					auto dropIt = bestDrop.find(key);
					if (dropIt == bestDrop.end())
						bestDrop.emplace(key, traj);
					else if (BetterTrajectory(traj, dropIt->second))
						dropIt->second = traj;
				}
			}
		}

		if (produced)
			++areasWithEdges;
	}

	for (const auto &entry : best)
	{
		int from = static_cast<int>(entry.first / 1000000LL);
		int to = static_cast<int>(entry.first % 1000000LL);
		const AirmoveTrajectory &traj = entry.second;

		MergedConnection conn;
		conn.areaA = from;
		conn.areaB = to;
		conn.pointA = traj.launchPos;
		conn.pointB = traj.landingPos;
		conn.area = 0.0;
		conn.connType = crossbot_nav::CONN_AIRMOVE;
		conn.connFlags = crossbot_nav::CONN_FLAG_ONE_WAY;
		conn.exitSpeed = traj.exitSpeed;
		conn.exitAngle = traj.exitYaw;
		conn.jumpBackDist = traj.jumpBackDist;
		conn.fallDamage = traj.fallDamage;
		conn.flightTime = traj.flightTime;
		conns.push_back(conn);
	}

	size_t derived = 0;
	for (const auto &entry : bestDrop)
	{
		int from = static_cast<int>(entry.first / 1000000LL);
		int to = static_cast<int>(entry.first % 1000000LL);
		const AirmoveTrajectory &drop = entry.second;

		double rise = drop.launchPos.z - drop.landingPos.z;
		if (rise <= 0.0 || rise > params.maxJumpHeight)
			continue;

		long long reverseKey = PairKey(to, from);
		if (existingPairs.count(reverseKey) || best.count(reverseKey))
			continue;

		MergedConnection conn;
		conn.areaA = to;
		conn.areaB = from;
		conn.pointA = drop.landingPos;
		conn.pointB = drop.launchPos;
		conn.area = 0.0;
		conn.connType = crossbot_nav::CONN_AIRMOVE;
		conn.connFlags = crossbot_nav::CONN_FLAG_ONE_WAY;
		conn.exitSpeed = drop.exitSpeed;
		conn.exitAngle = (drop.exitYaw >= 180.0f) ? drop.exitYaw - 180.0f : drop.exitYaw + 180.0f;
		conn.fallDamage = 0.0;
		conn.flightTime = drop.flightTime;
		conns.push_back(conn);
		++derived;
	}

	std::printf("air move sampling: %zu trajectories at %.2f samples/unit from %zu areas -> %zu edges "
		"(%zu source areas), %zu reverse jumps derived from drops under %.0f units\n",
		simulated, params.samplesPerUnit, areas.size(), best.size(), areasWithEdges, derived,
		params.maxJumpHeight);
}

}

