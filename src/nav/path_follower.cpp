#include "path_follower.h"

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

Decision Steer(const std::vector<NavPlan::PlanWaypoint> &waypoints, size_t wpIndex, const Vector &pos,
	const Vector &vel, const Config &cfg)
{
	Decision d;
	if (wpIndex >= waypoints.size())
	{
		d.advance = false;
		d.aimPoint = pos;
		return d;
	}

	float speed = std::sqrt(vel.x * vel.x + vel.y * vel.y);

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

	float lookahead = cfg.lookahead;
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

	if (!isFinal && !wp.airmoveLaunch)
	{
		float cut = cfg.cornerCut * (speed / cfg.maxSpeed);
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

	Vector dir = Norm2D(d.aimPoint - pos);

	if (wpIndex > 0 && cfg.pathCorridor > cfg.pathFreeplay && (dir.x != 0.f || dir.y != 0.f))
	{
		const Vector &segStart = waypoints[wpIndex - 1].point;
		Vector segDir = Norm2D(wp.point - segStart);
		if (segDir.x != 0.f || segDir.y != 0.f)
		{
			Vector toBot(pos.x - segStart.x, pos.y - segStart.y, 0.f);
			float along = toBot.x * segDir.x + toBot.y * segDir.y;
			Vector off(toBot.x - segDir.x * along, toBot.y - segDir.y * along, 0.f);

			float strayed = std::sqrt(off.x * off.x + off.y * off.y);
			if (strayed > cfg.pathFreeplay)
			{
				float w = (strayed - cfg.pathFreeplay) / (cfg.pathCorridor - cfg.pathFreeplay);
				if (w > 1.f)
					w = 1.f;

				Vector pull(-off.x / strayed, -off.y / strayed, 0.f);
				Vector corrected = Norm2D(Vector(dir.x * (1.f - w) + pull.x * w,
					dir.y * (1.f - w) + pull.y * w, 0.f));
				if (corrected.x != 0.f || corrected.y != 0.f)
					dir = corrected;
			}
		}
	}

	d.moveDir = dir;

	if (!wp.airmoveLaunch && horizDist < cfg.brakeRadius && (dir.x != 0.f || dir.y != 0.f))
	{
		float s = (speed > cfg.maxSpeed) ? speed : cfg.maxSpeed;
		Vector error(dir.x * s - vel.x, dir.y * s - vel.y, 0.f);

		float errorLen = std::sqrt(error.x * error.x + error.y * error.y);
		if (errorLen > 0.0001f)
		{
			float w = errorLen / cfg.maxSpeed;
			if (w > 1.f)
				w = 1.f;

			Vector blended = Norm2D(Vector(dir.x * (1.f - w) + (error.x / errorLen) * w,
				dir.y * (1.f - w) + (error.y / errorLen) * w, 0.f));
			if (blended.x != 0.f || blended.y != 0.f)
				d.moveDir = blended;
		}
	}

	return d;
}

}

