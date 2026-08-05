#include "path_follower.h"

#include <algorithm>
#include <cmath>

#include "nav_math.h"

namespace NavFollow
{

namespace
{

using NavMath::Norm2D;

float HorizDist(const Vector &a, const Vector &b)
{
	float dx = b.x - a.x, dy = b.y - a.y;
	return std::sqrt(dx * dx + dy * dy);
}

}

Drive SolveDrive(const Vector &wanted, const Vector &vel, const PmSim::PmPhysics &physics)
{
	Drive d;

	float dt = physics.frameTime;
	float speed = std::sqrt(vel.x * vel.x + vel.y * vel.y);

	float f = physics.groundFriction * physics.friction;
	Vector afterFriction = vel;
	afterFriction.z = 0.f;
	if (speed >= 0.1f)
	{
		float drop = f * std::max(speed, physics.stopSpeed) * dt;
		float scaled = std::max(speed - drop, 0.f) / speed;
		afterFriction.x = vel.x * scaled;
		afterFriction.y = vel.y * scaled;
	}

	Vector need(wanted.x - afterFriction.x, wanted.y - afterFriction.y, 0.f);
	float needLen = std::sqrt(need.x * need.x + need.y * need.y);
	if (needLen < 0.0001f)
		return d;

	d.wishDir = Vector(need.x / needLen, need.y / needLen, 0.f);

	float rate = physics.accelerate * dt * physics.friction;
	float along = afterFriction.x * d.wishDir.x + afterFriction.y * d.wishDir.y;
	float byRate = (rate > 0.0001f) ? (needLen / rate) : physics.maxSpeed;
	float byCap = along + needLen;

	d.wishSpeed = std::max(byRate, byCap);
	d.wishSpeed = std::max(0.f, std::min(physics.maxSpeed, d.wishSpeed));

	return d;
}

Decision Track(const NavPlan::PlanProfile &profile, const Vector &pos, const Vector &vel,
	const PmSim::PmPhysics &physics, TrackState &state, const Config &cfg)
{
	Decision d;
	if (profile.Empty())
	{
		d.done = true;
		return d;
	}

	float dt = physics.frameTime > 0.f ? physics.frameTime : 1.f / 60.f;
	float speed = std::sqrt(vel.x * vel.x + vel.y * vel.y);

	float lateral = 0.f;
	float s;
	if (!state.started)
	{
		s = NavPlan::ProjectOntoProfile(profile, pos, 0.f, profile.totalLength, &lateral);
		state.started = true;
	}
	else
	{
		float sMin = state.progressS - cfg.projectBack;
		float sMax = state.progressS + speed * dt * 4.f + cfg.projectAhead;
		s = NavPlan::ProjectOntoProfile(profile, pos, sMin, sMax, &lateral);
	}

	state.progressS = s;
	state.lateral = lateral;
	d.lateral = lateral;

	NavPlan::ProfileSample here = NavPlan::SampleProfile(profile, s);
	d.wpIndex = here.wpIndex;
	d.onGap = here.gap;

	const NavPlan::ProfileSeg &lastSeg = profile.segs.back();
	if (HorizDist(pos, lastSeg.b) < cfg.goalRadius)
	{
		d.done = true;
		return d;
	}

	NavPlan::ProfileSample lead = NavPlan::SampleProfile(profile, s + speed * dt);
	d.aimPoint = lead.point;

	Vector aimDir = lead.tangent;
	if (aimDir.x == 0.f && aimDir.y == 0.f)
		aimDir = here.tangent;

	Vector toChain(here.point.x - pos.x, here.point.y - pos.y, 0.f);
	float off = std::sqrt(toChain.x * toChain.x + toChain.y * toChain.y);
	if (off > 0.0001f && cfg.rejoinDist > 0.0001f)
	{
		float gain = std::min(1.f, off / cfg.rejoinDist) / off;
		aimDir = Norm2D(aimDir + Vector(toChain.x * gain, toChain.y * gain, 0.f));
	}

	if (aimDir.x == 0.f && aimDir.y == 0.f)
	{
		d.done = true;
		return d;
	}

	float clearance = std::max(here.clearance, 1.f);
	float derate = 1.f;
	if (lateral > clearance)
		derate = std::max(cfg.derateFloor, 1.f - (lateral - clearance) / clearance);
	if (lateral > cfg.rejoinAbort)
		d.needRepath = true;

	float target = std::min(here.speed, physics.maxSpeed) * derate;
	d.targetSpeed = target;

	Drive solved = SolveDrive(aimDir * target, vel, physics);
	d.wishDir = (solved.wishSpeed > 0.f) ? solved.wishDir : aimDir;
	d.wishSpeed = solved.wishSpeed;

	return d;
}

}
