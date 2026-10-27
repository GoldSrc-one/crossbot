#include "pm_sim.h"

#include <algorithm>
#include <cmath>

#include "arc_math.h"
#include "nav_math.h"

namespace PmSim
{

namespace
{

using NavMath::kPiF;

constexpr int kMaxClipPlanes = 5;

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

}

float FallDamageFor(float fallSpeed, const FallRule &rule)
{
	if (rule.perSpeed <= 0.f || fallSpeed <= rule.safeSpeed)
		return 0.f;
	return (fallSpeed - rule.safeSpeed) * rule.perSpeed;
}

float FallHeight(float fallSpeed, float gravity)
{
	return gravity > 0.f ? fallSpeed * fallSpeed / (2.f * gravity) : 0.f;
}

float FallSpeed(float fallHeight, float gravity)
{
	return (fallHeight > 0.f && gravity > 0.f) ? sqrtf(2.f * gravity * fallHeight) : 0.f;
}

float FallPenaltySeconds(float damage, float health)
{
	if (damage <= 0.f)
		return 0.f;
	return damage >= health ? 1.0e4f : damage * 0.5f;
}

float PlainJumpHeight(float gravity)
{
	if (gravity <= 0.f)
		return 0.f;
	return kJumpImpulse * kJumpImpulse / (2.f * gravity);
}

PmSimulator::PmSimulator(PmTraceFn trace, const PmPhysics &physics, PmPushFn push)
	: m_trace(std::move(trace))
	, m_push(std::move(push))
	, m_physics(physics)
{
}

void PmSimulator::Reset(const Vector &origin, const Vector &velocity, float yawDegrees,
	float commandedSpeed, bool onGround, bool jump)
{
	float yawRad = yawDegrees * kPiF / 180.f;
	m_forward[0] = std::cos(yawRad);
	m_forward[1] = std::sin(yawRad);
	m_forward[2] = 0.f;

	m_pos[0] = origin.x;
	m_pos[1] = origin.y;
	m_pos[2] = origin.z;

	m_commandedSpeed = commandedSpeed;
	m_vel[0] = velocity.x;
	m_vel[1] = velocity.y;
	m_vel[2] = velocity.z;

	m_origin = origin;
	m_onGround = onGround;
	m_jumpPending = jump;
	m_fallSpeed = 0.f;
	m_touchedEntity = false;
	m_time = 0.f;
}

void PmSimulator::Begin(const Vector &origin, float yawDegrees, float commandedSpeed, bool jump)
{
	float yawRad = yawDegrees * kPiF / 180.f;
	Vector velocity(std::cos(yawRad) * commandedSpeed, std::sin(yawRad) * commandedSpeed, 0.f);
	Reset(origin, velocity, yawDegrees, commandedSpeed, true, jump);
}

void PmSimulator::BeginAirborne(const Vector &origin, const Vector &velocity, float yawDegrees,
	float commandedSpeed)
{
	Reset(origin, velocity, yawDegrees, commandedSpeed, false, false);
}

void PmSimulator::CheckVelocity(float *velocity) const
{
	float bound = m_physics.maxVelocity;
	for (int i = 0; i < 3; ++i)
	{
		if (std::isnan(velocity[i]))
			velocity[i] = 0.0f;
		else if (velocity[i] > bound)
			velocity[i] = bound;
		else if (velocity[i] < -bound)
			velocity[i] = -bound;
	}
}

void PmSimulator::AddCorrectGravity(float *velocity, float *baseVelocity) const
{
	velocity[2] -= m_physics.gravity * 0.5f * m_physics.frameTime;

	velocity[2] += baseVelocity[2] * m_physics.frameTime;
	baseVelocity[2] = 0.0f;

	CheckVelocity(velocity);
}

void PmSimulator::FixupGravityVelocity(float *velocity) const
{
	velocity[2] -= m_physics.gravity * 0.5f * m_physics.frameTime;
	CheckVelocity(velocity);
}

void PmSimulator::PreventMegaBunnyJumping(float *velocity) const
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

void PmSimulator::AirAccelerate(const float *wishdir, float wishspeed, float *velocity) const
{
	float wishspd = wishspeed;
	if (wishspd > kAirAccelWishSpeedCap)
		wishspd = kAirAccelWishSpeedCap;

	float currentspeed = DotProduct(velocity, wishdir);
	float addspeed = wishspd - currentspeed;
	if (addspeed <= 0.0f)
		return;

	float accelspeed = m_physics.airAccelerate * wishspeed * m_physics.frameTime * m_physics.friction;
	if (accelspeed > addspeed)
		accelspeed = addspeed;

	velocity[0] += accelspeed * wishdir[0];
	velocity[1] += accelspeed * wishdir[1];
	velocity[2] += accelspeed * wishdir[2];
}

int PmSimulator::FlyMove(float *origin, float *velocity, float dt) const
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

		PmTrace trace = m_trace(Vector(origin[0], origin[1], origin[2]), Vector(end[0], end[1], end[2]));
		m_touchedEntity = m_touchedEntity || trace.hitEntity;

		if (trace.allsolid)
		{
			velocity[0] = velocity[1] = velocity[2] = 0.0f;
			return 4;
		}

		if (trace.fraction > 0.0f)
		{
			origin[0] = trace.endpos.x;
			origin[1] = trace.endpos.y;
			origin[2] = trace.endpos.z;
			VectorCopy(velocity, original_velocity);
			numplanes = 0;
		}

		if (trace.fraction == 1.0f)
			break;

		if (trace.planeNormal.z > kGroundNormalZ)
			blocked |= 1;
		if (trace.planeNormal.z == 0.0f)
			blocked |= 2;

		time_left -= time_left * trace.fraction;

		if (numplanes >= kMaxClipPlanes)
		{
			velocity[0] = velocity[1] = velocity[2] = 0.0f;
			break;
		}

		planes[numplanes][0] = trace.planeNormal.x;
		planes[numplanes][1] = trace.planeNormal.y;
		planes[numplanes][2] = trace.planeNormal.z;
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
				break;
			}
		}

		if (i == numplanes)
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

bool PmSimulator::CatagorizePosition(const float *origin, const float *velocity) const
{
	if (velocity[2] > kMaxOnGroundVelocityZ)
		return false;

	PmTrace trace = m_trace(Vector(origin[0], origin[1], origin[2]),
		Vector(origin[0], origin[1], origin[2] - kGroundTraceDist));

	m_touchedEntity = m_touchedEntity || trace.hitEntity;

	if (trace.fraction >= 1.0f)
		return false;

	return trace.planeNormal.z >= kGroundNormalZ;
}

bool PmSimulator::Step()
{
	if (!m_onGround)
		m_fallSpeed = -m_vel[2];

	float base[3] = {0.0f, 0.0f, 0.0f};
	if (m_push)
	{
		Vector push = m_push(m_origin);
		base[0] = push.x;
		base[1] = push.y;
		base[2] = push.z;
		if (push.x != 0.f || push.y != 0.f || push.z != 0.f)
			m_touchedEntity = true;
	}

	AddCorrectGravity(m_vel, base);

	if (m_jumpPending && m_onGround)
	{
		PreventMegaBunnyJumping(m_vel);
		m_vel[2] = kJumpImpulse;
		FixupGravityVelocity(m_vel);
		m_jumpPending = false;
		m_onGround = false;
	}

	float wishvel[3] = { m_forward[0] * m_commandedSpeed, m_forward[1] * m_commandedSpeed, 0.0f };
	float wishdir[3] = { wishvel[0], wishvel[1], wishvel[2] };
	float wishspeed = VectorNormalize(wishdir);
	if (wishspeed > m_physics.maxSpeed)
		wishspeed = m_physics.maxSpeed;

	AirAccelerate(wishdir, wishspeed, m_vel);

	for (int i = 0; i < 3; ++i)
		m_vel[i] += base[i];

	if (FlyMove(m_pos, m_vel, m_physics.frameTime) == 4)
	{
		for (int i = 0; i < 3; ++i)
			m_vel[i] -= base[i];
		m_origin = Vector(m_pos[0], m_pos[1], m_pos[2]);
		return false;
	}

	m_onGround = CatagorizePosition(m_pos, m_vel);

	for (int i = 0; i < 3; ++i)
		m_vel[i] -= base[i];
	CheckVelocity(m_vel);

	FixupGravityVelocity(m_vel);

	m_time += m_physics.frameTime;

	if (m_onGround)
		m_vel[2] = 0.0f;

	m_origin = Vector(m_pos[0], m_pos[1], m_pos[2]);
	return true;
}

int PmSimulator::Coast(const PmArcFn &arc, float maxTime)
{
	float dt = m_physics.frameTime;
	float g = m_physics.gravity;
	if (!arc || m_onGround || m_jumpPending || g <= 0.f || dt <= 0.f)
		return 0;

	float wishdir[3] = { m_forward[0] * m_commandedSpeed, m_forward[1] * m_commandedSpeed, 0.0f };
	float wishspeed = VectorNormalize(wishdir);
	wishspeed = std::min(wishspeed, std::min(m_physics.maxSpeed, kAirAccelWishSpeedCap));
	if (DotProduct(m_vel, wishdir) < wishspeed)
		return 0;

	maxTime = std::min(maxTime, (m_vel[2] + m_physics.maxVelocity) / g);
	if (maxTime < 3.f * dt)
		return 0;

	Vector vel = Velocity();
	float safe = arc(m_origin, vel, g, maxTime);
	int frames = static_cast<int>(std::floor(safe / dt)) - 1;
	if (frames < 2)
		return 0;

	float t = static_cast<float>(frames) * dt;
	m_origin = ArcMath::ArcPos(m_origin, vel, g, t);
	m_pos[0] = m_origin.x;
	m_pos[1] = m_origin.y;
	m_pos[2] = m_origin.z;
	m_vel[2] = ArcMath::ArcVelZ(m_vel[2], g, t);
	m_fallSpeed = -m_vel[2];
	m_time += t;
	return frames;
}

}

