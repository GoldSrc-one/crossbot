#pragma once

#include <vector>

#include "area_merger.h"
#include "blocking_entities.h"
#include "bsp_file.h"
#include "clipnode_tree.h"

namespace navgen
{

struct PhysicsParams
{
	float maxSpeed = 320.0f;
	float gravity = 800.0f;
	float airAccelerate = 10.0f;
	float friction = 1.0f;
	// HL's PLAYER_DUCKING_MULTIPLIER. Scales maxSpeed for a ducked player, which feeds the
	// wish-speed clamp and PreventMegaBunnyJumping just as pmove->maxspeed does in game.
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
	// How far behind the launch edge a jump may be initiated. Sampled per jump
	// sample; pure falls always launch at the edge inset.
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
	// Traced against the duck clipnode hull, so the move needs a crouch to fit.
	bool duckHull = false;
};

// One clipnode hull plus the blocking volumes expanded for it. The two hulls disagree about
// what fits, so a trace is only meaningful against a matched pair.
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

	// launchPos is in this simulator's hull space. navZAdjust is added to simulated positions to
	// express them in nav space (the hull-3 space the areas live in) -- a hull-1 origin sits
	// kStandProbeLift above a hull-3 one over the same floor, so area lookups and the returned
	// launch/landing points would be that far off without it.
	AirmoveTrajectory Simulate(const Vec3 &launchPos, float yawDegrees, float commandedSpeed, bool jump,
		const std::vector<Area> &areas, int sourceArea, int &landingArea, double navZAdjust = 0.0) const;

private:
	void AddCorrectGravity(float *velocity) const;
	void FixupGravityVelocity(float *velocity) const;
	void PreventMegaBunnyJumping(float *velocity) const;
	void AirAccelerate(const float *wishdir, float wishspeed, float *velocity) const;
	int FlyMove(float *origin, float *velocity, float dt) const;
	bool CatagorizePosition(const float *origin, const float *velocity) const;

public:
	bool IsSupported(const Vec3 &origin) const;
	bool VoidAhead(const Vec3 &origin, const Vec3 &dir) const;
	bool BlockedAhead(const Vec3 &origin, const Vec3 &dir) const;
	bool GroundAhead(const Vec3 &origin, const Vec3 &dir) const;
	TraceResult Trace(const Vec3 &start, const Vec3 &end) const;

private:

	const BspFile &m_bsp;
	int m_headnode;
	const std::vector<BlockingVolume> &m_blockers;
	AirmoveSampleParams m_params;
	PhysicsParams m_physics;
	Vec3 m_worldMins;
	Vec3 m_worldMaxs;
};

void ProbeAirMoves(const BspFile &bsp, const HullContext &duckHull, const HullContext &standHull,
	const std::vector<Area> &areas, std::vector<MergedConnection> &conns,
	const PhysicsParams &physics, const AirmoveSampleParams &params = AirmoveSampleParams());

}

