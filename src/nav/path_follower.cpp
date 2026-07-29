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

	// Cross-track correction. Aiming at a point ahead says nothing about how far the mover has been
	// thrown sideways getting there, and momentum through a sharp corner throws it a long way: an 80
	// degree turn at top speed swings it over 20 units wide of the line. The funnel's line is the
	// only route the planner actually vouched for, so on a narrow ledge that overshoot is a fall
	// rather than a scenic detour.
	//
	// The pull ramps across the band between freeplay and corridor: inside the freeplay none at all,
	// halfway across it is half aim and half pull, at the corridor edge it is nothing but pull. The
	// dead band matters -- correcting from zero fights every honest racing line through a corner and
	// leaves the mover sawing at the path rather than following it.
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

	// Inside the corner the aim point stops being the useful thing: what matters is not where the
	// mover is pointed but where it is drifting. Steering along the velocity error folds the whole
	// approach into one correction -- the lateral part cancels the drift, the forward part carries
	// it through -- and it pulls hardest exactly when the turn is sharpest.
	//
	// Mirroring the velocity about dir would cancel the same drift, but its forward term is (v.dir),
	// which inverts past 90 degrees and drives the mover backwards out of a hairpin. The velocity
	// error carries (s - v.dir) instead, which cannot go negative while s >= |v|, so taking s as the
	// larger of maxSpeed and the current speed makes it safe even when a launch or a push has put
	// the mover over maxspeed. No clamp needed.
	//
	// An air-move launch is exempt: it has to be hit at the speed and heading the sampler proved.
	//
	// How far to lean on the correction is set by how big it is, measured against top speed. That
	// weighting is doing real work in both directions. Near the top speed a few degrees of drift
	// gives a short error vector pointing almost square across dir, and steering straight down it
	// would answer a slight wobble with a hard sideways shove; the weight keeps that case on dir.
	// A hairpin gives an error as long as top speed and takes the correction whole.
	//
	// The drive stays flat out either way. Commanding a smaller wish speed does not brake in this
	// engine -- PM_Accelerate only ever adds along wishdir, so a small wish speed just lets friction
	// win and the mover coasts. Braking means commanding *against* the velocity at full strength,
	// which is what the correction direction already does.
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

