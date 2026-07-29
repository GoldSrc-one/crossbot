#include "path_follower.h"

#include <cmath>

namespace NavFollow
{

namespace
{

float HorizDist(const Vector &a, const Vector &b)
{
	float dx = b.x - a.x, dy = b.y - a.y;
	return std::sqrt(dx * dx + dy * dy);
}

// Unit length in the xy plane, zero if the input has no horizontal extent.
Vector Norm2D(const Vector &v)
{
	float len = std::sqrt(v.x * v.x + v.y * v.y);
	if (len < 0.0001f)
		return Vector(0.f, 0.f, 0.f);
	return Vector(v.x / len, v.y / len, 0.f);
}

}

Decision Steer(const std::vector<NavPlan::PlanWaypoint> &waypoints, size_t wpIndex, const Vector &pos,
	float speed, const Config &cfg)
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

	// Pre-steer into the corner. A mover at speed cannot turn on the spot -- HL ground accel needs
	// a couple of tenths to redirect 250 u/s -- so holding the aim on the corner until it is
	// reached carries it wide into whatever is on the far side. Instead start bending toward the
	// outgoing segment while still short of the corner.
	//
	// The aim never leaves a disc of cornerCut around the corner, so the turn bites at most that
	// far into the inside of it -- which is the clearance the planner reserved when it pushed the
	// waypoint off the wall. An air-move launch has to be hit squarely, so it is left alone.
	if (!isFinal && !wp.airmoveLaunch)
	{
		float cut = cfg.cornerCut * (speed / cfg.runSpeed);
		if (cut > cfg.cornerCut)
			cut = cfg.cornerCut;

		if (cut > 0.1f && horizDist < cut + cfg.cornerRadius)
		{
			Vector outDir = Norm2D(waypoints[wpIndex + 1].point - wp.point);
			if (outDir.x != 0.f || outDir.y != 0.f)
			{
				float blend = (cut + cfg.cornerRadius - horizDist) / cut;
				if (blend > 1.f)
					blend = 1.f;
				d.aimPoint = wp.point + outDir * (cut * blend);
			}
		}
	}

	return d;
}

}

