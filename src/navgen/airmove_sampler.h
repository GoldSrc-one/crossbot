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

	bool grabbedLadder = false;
	Vec3 ladderPos;

	bool duckHull = false;
};

struct LaunchEdge
{
	Vec3 a;
	Vec3 b;
	Vec3 outward;
	double length = 0.0;

	bool yawRange = false;
	float yawLo = 0.0f;
	float yawHi = 0.0f;
};

struct HullContext
{
	int headnode = -1;
	const std::vector<BlockingVolume> *blockers = nullptr;
	const std::vector<BlockingVolume> *ladders = nullptr;
};

struct AirmoveWorldStop
{
	int moverId = -1;
	bool isDefault = false;
	BlockingVolume duckVolume;
	BlockingVolume standVolume;
};

struct AirmoveWorlds
{
	std::vector<BlockingVolume> baseDuck;
	std::vector<BlockingVolume> baseStand;
	std::vector<AirmoveWorldStop> stops;
	std::vector<int> defaultStopOf;
};

class PM_MoveSimulator
{
public:
	PM_MoveSimulator(const BspFile &bsp, int headnode, const std::vector<BlockingVolume> &blockers,
		const AirmoveSampleParams &params, const PhysicsParams &physics,
		const std::vector<BlockingVolume> *ladders = nullptr);

	AirmoveTrajectory Simulate(const Vec3 &launchPos, float yawDegrees, float commandedSpeed, bool jump,
		const std::vector<Area> &areas, int sourceArea, int &landingArea, double navZAdjust = 0.0) const;

	AirmoveTrajectory SimulateAirborne(const Vec3 &launchPos, const Vec3 &velocity, float yawDegrees,
		float commandedSpeed, const std::vector<Area> &areas, int sourceArea, int &landingArea,
		double navZAdjust = 0.0) const;

	bool IsSupported(const Vec3 &origin) const;
	bool VoidAhead(const Vec3 &origin, const Vec3 &dir) const;
	bool BlockedAhead(const Vec3 &origin, const Vec3 &dir) const;
	bool GroundAhead(const Vec3 &origin, const Vec3 &dir) const;
	bool OnLadder(const Vec3 &origin) const;
	TraceResult Trace(const Vec3 &start, const Vec3 &end) const;

	PmSim::PmTraceFn TraceFn() const;
	const PmSim::PmPhysics &Physics() const { return m_pmPhysics; }

private:
	AirmoveTrajectory Fly(PmSim::PmSimulator &sim, AirmoveTrajectory &result, bool airborne,
		const std::vector<Area> &areas, int sourceArea, int &landingArea, double navZAdjust) const;

	const BspFile &m_bsp;
	int m_headnode;
	const std::vector<BlockingVolume> &m_blockers;
	const std::vector<BlockingVolume> *m_ladders;
	AirmoveSampleParams m_params;
	PmSim::PmPhysics m_pmPhysics;
	Vec3 m_worldMins;
	Vec3 m_worldMaxs;
};

int FindAreaStrict(const Vec3 &point, const std::vector<Area> &areas);

std::vector<LaunchEdge> FilterLaunchEdges(const PM_MoveSimulator &sim, const Area &area, double probeZ,
	const AirmoveSampleParams &params);

void ProbeAirMoves(const BspFile &bsp, const HullContext &duckHull, const HullContext &standHull,
	const AirmoveWorlds &worlds, const std::vector<Area> &areas, std::vector<MergedConnection> &conns,
	const PhysicsParams &physics, const AirmoveSampleParams &params = AirmoveSampleParams(),
	bool dumpElevators = false);

}

