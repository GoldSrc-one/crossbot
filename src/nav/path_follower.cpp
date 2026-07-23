#include "path_follower.h"

#include <cmath>

namespace NavFollow
{

namespace
{

constexpr float kPi = 3.14159265f;

float NormalizeAngle(float a)
{
	while (a > 180.f)
		a -= 360.f;
	while (a < -180.f)
		a += 360.f;
	return a;
}

float HorizDist(const Vector &a, const Vector &b)
{
	float dx = b.x - a.x, dy = b.y - a.y;
	return std::sqrt(dx * dx + dy * dy);
}

} // namespace

Decision Steer(const std::vector<NavPlan::PlanWaypoint> &waypoints, size_t wpIndex, const Vector &pos,
	const Config &cfg)
{
	Decision d;
	if (wpIndex >= waypoints.size())
	{
		d.advance = false;
		d.aimPoint = pos;
		return d;
	}

	const NavPlan::PlanWaypoint &wp = waypoints[wpIndex];
	float horizDist = HorizDist(pos, wp.point);
	bool isFinal = wpIndex + 1 >= waypoints.size();

	// Reached? Final waypoint: loose radius. Intermediate corner: tight
	// radius, or passing the corner's perpendicular along the CURRENT
	// segment. Deliberately not a "crossed into the next segment's
	// half-space" test -- on a sharp V-turn (more than 90 degrees) the
	// whole incoming segment already lies in that half-space and it would
	// fire instantly. Skip-proof at any speed: whatever the tick step,
	// some frame lands at-or-past the perpendicular.
	bool reached = horizDist < (isFinal ? cfg.finalRadius : cfg.cornerRadius);
	if (!reached && !isFinal && wpIndex > 0)
	{
		const Vector &segStart = waypoints[wpIndex - 1].point;
		Vector seg = wp.point - segStart;
		Vector toBot = pos - segStart;
		float segLen2DSq = seg.x * seg.x + seg.y * seg.y;
		if (segLen2DSq > 1.f && (toBot.x * seg.x + toBot.y * seg.y) >= segLen2DSq)
			reached = true;
	}
	if (reached)
	{
		d.advance = true;
		return d;
	}

	d.baseSpeed = (wp.maxSpeed < cfg.runSpeed) ? wp.maxSpeed : cfg.runSpeed;

	// Pure-pursuit line following: aim at a point on the planned segment
	// ahead of our own projection onto it, so drift/corner cuts get
	// corrected back onto the line instead of compounding (endpoint-homing
	// converges along a bowed arc -- confirmed live on test23_precise's
	// beams, where that bow was the difference between crossing and
	// falling). waypoints[0] is always the path's own start point, so any
	// waypoint being walked toward has a real segment behind it.
	float lookahead = (d.baseSpeed < cfg.runSpeed) ? cfg.lookaheadCareful : cfg.lookahead;
	d.aimPoint = wp.point;
	if (wpIndex > 0)
	{
		const Vector &segStart = waypoints[wpIndex - 1].point;
		Vector seg = wp.point - segStart;
		float segLen2DSq = seg.x * seg.x + seg.y * seg.y;
		if (segLen2DSq > 1.f)
		{
			Vector toBot = pos - segStart;
			float t = (toBot.x * seg.x + toBot.y * seg.y) / segLen2DSq;
			if (t < 0.f)
				t = 0.f;
			float tAhead = t + lookahead / std::sqrt(segLen2DSq);
			if (tAhead > 1.f)
				tAhead = 1.f;
			d.aimPoint = segStart + seg * tAhead;
		}
	}
	return d;
}

float ScaleSpeedByHeadingError(float baseSpeed, float headingErrorDeg, const Config &cfg)
{
	if (headingErrorDeg > cfg.turnInPlaceErrorDeg)
		return 0.f;
	if (headingErrorDeg > cfg.turnSlowdownErrorDeg)
		return baseSpeed * cfg.turnSlowdownFactor;
	return baseSpeed;
}

SimResult Simulate(const std::vector<NavPlan::PlanWaypoint> &waypoints, const Vector &start, float dt,
	float maxTime, const Config &cfg)
{
	// Stock GoldSrc ground-movement constants (PM_Friction/PM_Accelerate,
	// pm_shared/pm_math): the default dedicated-server cvar values every
	// test run uses. Not configurable until a real need shows up.
	constexpr float kFriction = 4.f;
	constexpr float kAccelerate = 10.f;
	constexpr float kStopSpeed = 100.f;

	SimResult result;
	if (waypoints.empty())
		return result;

	Vector pos = start;
	float velX = 0.f, velY = 0.f;
	float yaw = 0.f;
	{
		// Start facing the first real steering target, like a bot that just
		// finished turning toward its route before setting off.
		size_t aimIdx = (waypoints.size() > 1) ? 1 : 0;
		yaw = std::atan2(waypoints[aimIdx].point.y - pos.y, waypoints[aimIdx].point.x - pos.x) * (180.f / kPi);
	}
	size_t wpIndex = 0;

	int maxTicks = static_cast<int>(maxTime / dt);
	result.points.push_back(pos);

	for (int tick = 0; tick < maxTicks; ++tick)
	{
		// Waypoint switching first, exactly like Bot_MoveNav's frame order.
		Decision d = Steer(waypoints, wpIndex, pos, cfg);
		while (d.advance)
		{
			++wpIndex;
			if (wpIndex >= waypoints.size())
			{
				result.finished = true;
				return result;
			}
			d = Steer(waypoints, wpIndex, pos, cfg);
		}

		// Turn-rate-limited facing (bot_move.cpp's Bot_UpdateFacing).
		float desiredYaw = std::atan2(d.aimPoint.y - pos.y, d.aimPoint.x - pos.x) * (180.f / kPi);
		float yawDelta = NormalizeAngle(desiredYaw - yaw);
		float maxStep = cfg.turnRateDegPerSec * dt;
		float yawStep = (yawDelta > maxStep) ? maxStep : (yawDelta < -maxStep ? -maxStep : yawDelta);
		yaw = NormalizeAngle(yaw + yawStep);

		float headingError = std::fabs(NormalizeAngle(desiredYaw - yaw));
		float wishSpeed = ScaleSpeedByHeadingError(d.baseSpeed, headingError, cfg);

		// PM_Friction.
		float speed = std::sqrt(velX * velX + velY * velY);
		if (speed > 0.1f)
		{
			float control = (speed < kStopSpeed) ? kStopSpeed : speed;
			float newSpeed = speed - control * kFriction * dt;
			if (newSpeed < 0.f)
				newSpeed = 0.f;
			float scale = newSpeed / speed;
			velX *= scale;
			velY *= scale;
		}

		// PM_Accelerate toward the facing direction (IN_FORWARD only, like
		// the real bot's pfnRunPlayerMove input).
		if (wishSpeed > 0.f)
		{
			float yawRad = yaw * (kPi / 180.f);
			float wishX = std::cos(yawRad), wishY = std::sin(yawRad);
			float currentSpeed = velX * wishX + velY * wishY;
			float addSpeed = wishSpeed - currentSpeed;
			if (addSpeed > 0.f)
			{
				float accelSpeed = kAccelerate * dt * wishSpeed;
				if (accelSpeed > addSpeed)
					accelSpeed = addSpeed;
				velX += accelSpeed * wishX;
				velY += accelSpeed * wishY;
			}
		}

		pos.x += velX * dt;
		pos.y += velY * dt;

		// Carry z along the current planned segment (closest-point
		// interpolation) -- the simulation has no gravity; z exists purely
		// so a caller's fall-check can compare against floor heights.
		if (wpIndex > 0)
		{
			const Vector &segStart = waypoints[wpIndex - 1].point;
			const Vector &segEnd = waypoints[wpIndex].point;
			Vector seg = segEnd - segStart;
			float segLen2DSq = seg.x * seg.x + seg.y * seg.y;
			if (segLen2DSq > 1.f)
			{
				float t = ((pos.x - segStart.x) * seg.x + (pos.y - segStart.y) * seg.y) / segLen2DSq;
				if (t < 0.f)
					t = 0.f;
				if (t > 1.f)
					t = 1.f;
				pos.z = segStart.z + seg.z * t;
			}
		}

		result.points.push_back(pos);
	}

	return result;
}

} // namespace NavFollow
