#pragma once

#include <functional>

#include "mathlib.h"
#include "vector.h"

// A transcription of the Half-Life player movement code, driven a frame at a time. Both the nav
// bake and the bot run this same core so a jump the sampler proved is a jump the bot can make;
// the only thing either side supplies is how to trace a hull through the world.
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
	float friction = 1.f;
	float frameTime = 1.f / 60.f;
};

constexpr float kJumpImpulse = 268.328156f;
constexpr float kBunnyJumpMaxSpeedFactor = 1.7f;
constexpr float kAirAccelWishSpeedCap = 30.f;
constexpr float kMaxOnGroundVelocityZ = 180.f;
constexpr float kGroundNormalZ = 0.7f;
constexpr float kGroundTraceDist = 2.f;

constexpr float kPlayerFatalFallSpeed = 1024.f;
constexpr float kPlayerMaxSafeFallSpeed = 580.f;
constexpr float kDamageForFallSpeed = 100.f / (kPlayerFatalFallSpeed - kPlayerMaxSafeFallSpeed);

// Damage a landing at this downward speed costs. Zero below the safe threshold.
float FallDamageFor(float fallSpeed);

// The apex of a standing jump, from the impulse and gravity. Anything taller needs the extra
// reach of ducking at the top.
float PlainJumpHeight(float gravity);

class PmSimulator
{
public:
	PmSimulator(PmTraceFn trace, const PmPhysics &physics);

	// Places the mover on the ground at origin, running flat out along yaw. A pending jump fires
	// on the first Step.
	void Begin(const Vector &origin, float yawDegrees, float commandedSpeed, bool jump);

	// Advances one frame. False once the mover is wedged and can no longer be moved, which the
	// caller should treat as the run ending without a landing.
	bool Step();

	const Vector &Origin() const { return m_origin; }
	bool OnGround() const { return m_onGround; }
	float Time() const { return m_time; }

	// Downward speed entering the current frame, which is what a landing this frame fell at.
	float FallSpeed() const { return m_fallSpeed; }

	// Call after deciding a ground contact does not end the run, so the next fall is measured
	// from here rather than from the original launch.
	void ResetFallSpeed() { m_fallSpeed = 0.f; }

private:
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
