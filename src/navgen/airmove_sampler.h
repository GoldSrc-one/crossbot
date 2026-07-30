#pragma once

#include <vector>

#include "area_merger.h"
#include "blocking_entities.h"
#include "bsp_file.h"
#include "clipnode_tree.h"
#include "pm_sim.h"

namespace navgen
{

struct PhysicsParams
{
	float maxSpeed = 320.0f;
	float gravity = 800.0f;
	float airAccelerate = 10.0f;
	float friction = 1.0f;

	float duckSpeedMultiplier = 0.333f;
};

struct AirmoveSampleParams
{
	float samplesPerUnit = 1.0f;
	float inset = 2.0f;
	float lift = 1.0f;
	float edgeProbe = 8.0f;
	float stepHeight = 18.0f;
	float maxJumpHeight = 64.0f;
	float yawSpread = 90.0f;

	float jumpBackMax = 32.0f;
	float groundGraceTime = 0.5f;
	float frameTime = 1.0f / 60.0f;
	float maxSimulationTime = 6.0f;
	unsigned int seed = 0x5EEDBEEFu;
};

struct AirmoveTrajectory
{
	Vec3 launchPos;
	Vec3 landingPos;
	float exitSpeed = 0.0f;
	float exitYaw = 0.0f;
	float jumpBackDist = 0.0f;
	float fallDamage = 0.0f;
	float flightTime = 0.0f;
	bool jumped = false;
	bool landed = false;

	bool duckHull = false;
};

struct HullContext
{
	int headnode = -1;
	const std::vector<BlockingVolume> *blockers = nullptr;
};

class PM_MoveSimulator
{
public:
	PM_MoveSimulator(const BspFile &bsp, int headnode, const std::vector<BlockingVolume> &blockers,
		const AirmoveSampleParams &params, const PhysicsParams &physics);

	AirmoveTrajectory Simulate(const Vec3 &launchPos, float yawDegrees, float commandedSpeed, bool jump,
		const std::vector<Area> &areas, int sourceArea, int &landingArea, double navZAdjust = 0.0) const;

	bool IsSupported(const Vec3 &origin) const;
	bool VoidAhead(const Vec3 &origin, const Vec3 &dir) const;
	bool BlockedAhead(const Vec3 &origin, const Vec3 &dir) const;
	bool GroundAhead(const Vec3 &origin, const Vec3 &dir) const;
	TraceResult Trace(const Vec3 &start, const Vec3 &end) const;

	PmSim::PmTraceFn TraceFn() const;
	const PmSim::PmPhysics &Physics() const { return m_pmPhysics; }

private:

	const BspFile &m_bsp;
	int m_headnode;
	const std::vector<BlockingVolume> &m_blockers;
	AirmoveSampleParams m_params;
	PmSim::PmPhysics m_pmPhysics;
	Vec3 m_worldMins;
	Vec3 m_worldMaxs;
};

void ProbeAirMoves(const BspFile &bsp, const HullContext &duckHull, const HullContext &standHull,
	const std::vector<Area> &areas, std::vector<MergedConnection> &conns,
	const PhysicsParams &physics, const AirmoveSampleParams &params = AirmoveSampleParams());

}

