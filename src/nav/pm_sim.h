#pragma once

#include <functional>

#include "mathlib.h"
#include "vector.h"

namespace PmSim
{

struct PmTrace
{
	Vector endpos;
	Vector planeNormal;
	float fraction = 1.f;
	bool allsolid = false;
	bool startsolid = false;
};

using PmTraceFn = std::function<PmTrace(const Vector &start, const Vector &end)>;

struct PmPhysics
{
	float maxSpeed = 320.f;
	float gravity = 800.f;
	float airAccelerate = 10.f;
	float accelerate = 10.f;
	float groundFriction = 4.f;
	float stopSpeed = 100.f;
	float edgeFriction = 2.f;
	float friction = 1.f;
	float frameTime = 1.f / 60.f;
};

constexpr float kGravity = 800.f;

constexpr float kDuckingMultiplier = 0.333f;

constexpr float kJumpImpulse = 268.328156f;
constexpr float kLongJumpSpeed = 350.f * 1.6f;
constexpr float kLongJumpImpulse = 299.332591f;
constexpr float kBunnyJumpMaxSpeedFactor = 1.7f;
constexpr float kAirAccelWishSpeedCap = 30.f;
constexpr float kMaxOnGroundVelocityZ = 180.f;
constexpr float kGroundNormalZ = 0.7f;
constexpr float kGroundTraceDist = 2.f;
constexpr float kEdgeProbeAhead = 16.f;
constexpr float kEdgeProbeDrop = 34.f;

constexpr float kPlayerFatalFallSpeed = 1024.f;
constexpr float kPlayerMaxSafeFallSpeed = 580.f;
constexpr float kDamageForFallSpeed = 100.f / (kPlayerFatalFallSpeed - kPlayerMaxSafeFallSpeed);

float FallDamageFor(float fallSpeed);
float PlainJumpHeight(float gravity);

class PmSimulator
{
public:
	PmSimulator(PmTraceFn trace, const PmPhysics &physics);

	void Begin(const Vector &origin, float yawDegrees, float commandedSpeed, bool jump);
	void BeginAirborne(const Vector &origin, const Vector &velocity, float yawDegrees, float commandedSpeed);
	bool Step();

	void RequestJump() { m_jumpPending = true; }

	const Vector &Origin() const { return m_origin; }
	bool OnGround() const { return m_onGround; }
	float Time() const { return m_time; }
	float FallSpeed() const { return m_fallSpeed; }
	void ResetFallSpeed() { m_fallSpeed = 0.f; }

private:
	void Reset(const Vector &origin, const Vector &velocity, float yawDegrees, float commandedSpeed,
		bool onGround, bool jump);
	void AddCorrectGravity(float *velocity) const;
	void FixupGravityVelocity(float *velocity) const;
	void PreventMegaBunnyJumping(float *velocity) const;
	void AirAccelerate(const float *wishdir, float wishspeed, float *velocity) const;
	int FlyMove(float *origin, float *velocity, float dt) const;
	bool CatagorizePosition(const float *origin, const float *velocity) const;

	PmTraceFn m_trace;
	PmPhysics m_physics;

	float m_pos[3] = {0.f, 0.f, 0.f};
	float m_vel[3] = {0.f, 0.f, 0.f};
	float m_forward[3] = {0.f, 0.f, 0.f};
	float m_commandedSpeed = 0.f;

	Vector m_origin;
	bool m_onGround = true;
	bool m_jumpPending = false;
	float m_fallSpeed = 0.f;
	float m_time = 0.f;
};

}
