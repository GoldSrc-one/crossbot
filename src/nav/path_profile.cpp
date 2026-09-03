#include "path_profile.h"

#include "drift.h"

#include <algorithm>
#include <cmath>

#include "nav_math.h"

namespace NavPlan
{

namespace
{

using NavMath::Norm2D;

constexpr float kMinArcAngle = 0.035f;
constexpr float kMaxArcAngle = 3.1241f;
constexpr float kMaxArcRadius = 4096.f;
constexpr float kMinArcRadius = 1.f;
constexpr float kEpsLen = 0.01f;
constexpr int kEtaMaxSteps = 20000;
constexpr int kFitPasses = 3;
constexpr int kTraceShrinkTries = 3;
constexpr float kTraceShrinkFactor = 0.6f;
constexpr int kAccelSolveIters = 40;
constexpr float kEpsSpeed = 0.01f;
constexpr float kUnreachableDistance = 1e9f;

float Dist2D(const Vector &a, const Vector &b)
{
	float dx = b.x - a.x, dy = b.y - a.y;
	return std::sqrt(dx * dx + dy * dy);
}

float Cross2D(const Vector &a, const Vector &b)
{
	return a.x * b.y - a.y * b.x;
}

float LateralAccel(float speed, float accel, float friction, float stopSpeed)
{
	float drag = friction * std::max(speed, stopSpeed);
	float sq = accel * accel - drag * drag;
	return sq > 1.f ? std::sqrt(sq) : 1.f;
}

float BrakeBack(float vEnd, float dist, float friction, float stopSpeed)
{
	if (dist <= 0.f || friction <= 0.f)
		return vEnd;

	if (vEnd >= stopSpeed)
		return vEnd + friction * dist;

	float toStop = (stopSpeed * stopSpeed - vEnd * vEnd) / (2.f * friction * stopSpeed);
	if (dist <= toStop)
		return std::sqrt(vEnd * vEnd + 2.f * friction * stopSpeed * dist);

	return stopSpeed + friction * (dist - toStop);
}

float AccelDistanceAt(float speed, float accel, float friction)
{
	return -speed / friction - (accel / (friction * friction)) * std::log(accel - friction * speed);
}

float SegSpeedAt(const ProfileSeg &seg, float local, float friction, float stopSpeed)
{
	float remaining = std::max(0.f, seg.sLen - local);
	float ceiling = std::max(seg.vEntry, seg.vExit);
	return std::min(ceiling, BrakeBack(seg.vExit, remaining, friction, stopSpeed));
}

struct Corner
{
	bool arc = false;
	float theta = 0.f;
	float radius = 0.f;
	float tangent = 0.f;
	float vCap = 0.f;
	Vector u, w;
	Vector entry, exit, center;
	float startAngle = 0.f;
	float turn = 0.f;
	bool slowed = false;
	bool shrunk = false;
};

}

float AccelForward(float v0, float dist, float accel, float friction, float stopSpeed)
{
	float v = std::max(v0, 0.f);
	if (dist <= kEpsLen || accel <= 0.f || friction <= 0.f)
		return v;

	float remaining = dist;

	if (v < stopSpeed)
	{
		float slowGain = accel - friction * stopSpeed;
		if (slowGain <= 0.f)
			return v;

		float toStop = (stopSpeed * stopSpeed - v * v) / (2.f * slowGain);
		if (remaining <= toStop)
			return std::sqrt(v * v + 2.f * slowGain * remaining);

		remaining -= toStop;
		v = stopSpeed;
	}

	float terminal = accel / friction;
	if (v >= terminal - kEpsSpeed)
		return v;

	float base = AccelDistanceAt(v, accel, friction);
	float lo = v;
	float hi = terminal - kEpsSpeed;
	for (int i = 0; i < kAccelSolveIters; ++i)
	{
		float mid = 0.5f * (lo + hi);
		if (AccelDistanceAt(mid, accel, friction) - base > remaining)
			hi = mid;
		else
			lo = mid;
	}

	return lo;
}

float AccelDistance(float v0, float v1, float accel, float friction, float stopSpeed)
{
	float v = std::max(v0, 0.f);
	if (v1 <= v || accel <= 0.f || friction <= 0.f)
		return 0.f;

	float terminal = accel / friction;
	if (v1 >= terminal - kEpsSpeed)
		return kUnreachableDistance;

	float dist = 0.f;

	if (v < stopSpeed)
	{
		float slowGain = accel - friction * stopSpeed;
		if (slowGain <= 0.f)
			return kUnreachableDistance;

		float top = std::min(v1, stopSpeed);
		dist += (top * top - v * v) / (2.f * slowGain);
		v = top;
	}

	if (v1 > v)
		dist += AccelDistanceAt(v1, accel, friction) - AccelDistanceAt(v, accel, friction);

	return dist;
}

float SpeedForRadius(float radius, float accel, float friction, float stopSpeed)
{
	if (radius <= 0.f || accel <= 0.f)
		return 0.f;

	float lowAccel = LateralAccel(0.f, accel, friction, stopSpeed);
	float vLow = std::sqrt(radius * lowAccel);
	if (vLow <= stopSpeed)
		return vLow;

	float r2 = radius * radius;
	float f2 = friction * friction;
	float b = r2 * f2;
	float disc = b * b + 4.f * r2 * accel * accel;
	float u = 0.5f * (-b + std::sqrt(disc));
	return u > 0.f ? std::sqrt(u) : 0.f;
}

float RadiusForSpeed(float speed, float accel, float friction, float stopSpeed)
{
	if (speed <= 0.f)
		return 0.f;
	return speed * speed / LateralAccel(speed, accel, friction, stopSpeed);
}

float PivotSpeed(const PmSim::PmPhysics &physics, float maxSpeed)
{
	return physics.accelerate * maxSpeed * physics.frameTime;
}

PlanProfile BuildPathProfile(const std::vector<PlanWaypoint> &waypoints,
	const PmSim::PmPhysics &physics, const PlanTraceContext *traces)
{
	PlanProfile profile;
	size_t n = waypoints.size();
	if (n < 2)
		return profile;

	float f = physics.groundFriction * physics.friction;
	if (f <= 0.f)
		f = 4.f;
	float s0 = physics.stopSpeed > 0.f ? physics.stopSpeed : 100.f;

	profile.friction = f;
	profile.stopSpeed = s0;
	profile.accel = physics.accelerate * physics.maxSpeed;

	auto NodeMaxSpeed = [&](size_t i)
	{
		float v = physics.maxSpeed;
		if (waypoints[i].requiresCrouch && !waypoints[i].airmoveLaunch && !waypoints[i].requiresLadder)
			v *= PmSim::kDuckingMultiplier;

		const Vector &drift = waypoints[i].drift;
		if (drift.x != 0.f || drift.y != 0.f)
		{
			size_t from = (i + 1 < n) ? i : i - 1;
			Vector seg = waypoints[from + 1].point - waypoints[from].point;
			seg.z = 0.f;
			float len = seg.Length();
			if (len > 1e-3f)
				v = NavDrift::DriftSpeedAlong(drift, seg * (1.f / len), v);
		}

		return v;
	};

	auto NodeAccel = [&](size_t i)
	{
		return physics.accelerate * NodeMaxSpeed(i);
	};

	auto IsGap = [&](size_t i)
	{
		return waypoints[i].airmoveLaunch || waypoints[i].requiresTeleport ||
			waypoints[i].requiresLadder || waypoints[i].requiresElevatorRide;
	};

	auto IsStop = [&](size_t i)
	{
		return waypoints[i].requiresButtonUse || waypoints[i].requiresTriggerTouch ||
			waypoints[i].requiresDoor || waypoints[i].requiresBreak;
	};

	auto Suppressed = [&](size_t i)
	{
		if (IsGap(i) || IsStop(i))
			return true;
		if (i > 0 && (IsGap(i - 1) || IsStop(i - 1)))
			return true;
		return false;
	};

	auto PrecedesLaunch = [&](size_t i)
	{
		return i + 1 < n && waypoints[i + 1].airmoveLaunch;
	};

	std::vector<Corner> corners(n);
	std::vector<float> segLen(n > 0 ? n - 1 : 0, 0.f);
	for (size_t i = 0; i + 1 < n; ++i)
		segLen[i] = Dist2D(waypoints[i].point, waypoints[i + 1].point);

	for (size_t i = 1; i + 1 < n; ++i)
	{
		Corner &c = corners[i];
		c.vCap = NodeMaxSpeed(i);

		if (Suppressed(i) || segLen[i - 1] < kEpsLen || segLen[i] < kEpsLen)
			continue;

		c.u = Norm2D(waypoints[i].point - waypoints[i - 1].point);
		c.w = Norm2D(waypoints[i + 1].point - waypoints[i].point);
		if ((c.u.x == 0.f && c.u.y == 0.f) || (c.w.x == 0.f && c.w.y == 0.f))
			continue;

		float dot = c.u.x * c.w.x + c.u.y * c.w.y;
		dot = std::max(-1.f, std::min(1.f, dot));
		c.theta = std::acos(dot);
		if (c.theta < kMinArcAngle)
			continue;
		if (c.theta > kMaxArcAngle)
			c.theta = kMaxArcAngle;

		c.arc = true;
	}

	for (int pass = 0; pass < kFitPasses; ++pass)
	{
		for (size_t i = 1; i + 1 < n; ++i)
		{
			Corner &c = corners[i];
			if (!c.arc)
				continue;

			float half = c.theta * 0.5f;
			float cosH = std::cos(half);
			float sinH = std::sin(half);
			float tanH = (cosH > 0.0001f) ? (sinH / cosH) : 1e6f;

			float clearance = std::max(waypoints[i].clearance, 0.f);
			float rGeo = (1.f - cosH) > 0.0001f ? clearance * cosH / (1.f - cosH) : kMaxArcRadius;
			rGeo = std::min(rGeo, kMaxArcRadius);

			float availIn = segLen[i - 1] - (i >= 2 ? corners[i - 1].tangent : 0.f);
			float availOut = segLen[i] - (i + 2 < n ? corners[i + 1].tangent : 0.f);
			if (pass == 0)
			{
				availIn = segLen[i - 1] * 0.5f;
				availOut = segLen[i] * 0.5f;
			}
			if (PrecedesLaunch(i))
				availOut = std::min(availOut, segLen[i] * 0.5f);
			availIn = std::max(availIn, 0.f);
			availOut = std::max(availOut, 0.f);

			float rLen = std::min(availIn, availOut) / tanH;
			float r = std::min(rGeo, rLen);

			float accel = NodeAccel(i);
			float vMax = NodeMaxSpeed(i);
			float v = std::min(SpeedForRadius(r, accel, f, s0), vMax);
			r = std::min(r, RadiusForSpeed(v, accel, f, s0));

			float pivot = std::min(PivotSpeed(physics, vMax), vMax);
			if (v < pivot || r < kMinArcRadius)
			{
				c.arc = false;
				c.radius = 0.f;
				c.tangent = 0.f;
				c.vCap = std::min(pivot, vMax);
				c.slowed = true;
				continue;
			}

			c.radius = r;
			c.tangent = r * tanH;
			c.vCap = v;
			c.slowed = v < vMax - 0.5f;

			Vector bis = Norm2D(c.w - c.u);
			if (bis.x == 0.f && bis.y == 0.f)
			{
				c.arc = false;
				c.vCap = std::min(pivot, vMax);
				continue;
			}

			const Vector &p = waypoints[i].point;
			c.entry = p - c.u * c.tangent;
			c.exit = p + c.w * c.tangent;
			c.entry.z = p.z;
			c.exit.z = p.z;
			c.center = p + bis * (r / cosH);
			c.center.z = p.z;
			c.startAngle = std::atan2(c.entry.y - c.center.y, c.entry.x - c.center.x);
			c.turn = (Cross2D(c.u, c.w) >= 0.f) ? c.theta : -c.theta;
		}
	}

	if (traces)
	{
		for (size_t i = 1; i + 1 < n; ++i)
		{
			Corner &c = corners[i];
			if (!c.arc)
				continue;

			const PmSim::PmTraceFn &trace =
				waypoints[i].requiresCrouch ? traces->duckTrace : traces->standTrace;
			if (!trace)
				continue;

			for (int tryIdx = 0; tryIdx < kTraceShrinkTries; ++tryIdx)
			{
				float midAngle = c.startAngle + c.turn * 0.5f;
				Vector apex(c.center.x + c.radius * std::cos(midAngle),
					c.center.y + c.radius * std::sin(midAngle), c.entry.z);

				PmSim::PmTrace t1 = trace(c.entry, apex);
				PmSim::PmTrace t2 = trace(apex, c.exit);
				if (t1.fraction >= 1.f && t2.fraction >= 1.f && !t1.startsolid && !t2.startsolid)
					break;

				float half = c.theta * 0.5f;
				float cosH = std::cos(half);
				float tanH = (cosH > 0.0001f) ? (std::sin(half) / cosH) : 1e6f;

				c.radius *= kTraceShrinkFactor;
				c.shrunk = true;

				float accel = NodeAccel(i);
				float vMax = NodeMaxSpeed(i);
				float pivot = std::min(PivotSpeed(physics, vMax), vMax);
				float v = std::min(SpeedForRadius(c.radius, accel, f, s0), vMax);

				if (v < pivot || c.radius < kMinArcRadius)
				{
					c.arc = false;
					c.radius = 0.f;
					c.tangent = 0.f;
					c.vCap = std::min(pivot, vMax);
					c.slowed = true;
					break;
				}

				c.vCap = v;
				c.slowed = true;
				c.tangent = c.radius * tanH;

				const Vector &p = waypoints[i].point;
				Vector bis = Norm2D(c.w - c.u);
				c.entry = p - c.u * c.tangent;
				c.exit = p + c.w * c.tangent;
				c.entry.z = p.z;
				c.exit.z = p.z;
				c.center = p + bis * (c.radius / cosH);
				c.center.z = p.z;
				c.startAngle = std::atan2(c.entry.y - c.center.y, c.entry.x - c.center.x);
			}
		}
	}

	auto PushSeg = [&](const ProfileSeg &segIn)
	{
		ProfileSeg seg = segIn;
		seg.sStart = profile.totalLength;
		profile.totalLength += seg.sLen;
		profile.segs.push_back(seg);
	};

	Vector cursor = waypoints[0].point;
	for (size_t i = 1; i < n; ++i)
	{
		const Corner &c = corners[i];
		bool last = (i + 1 == n);
		Vector lineEnd = (!last && c.arc) ? c.entry : waypoints[i].point;

		ProfileSeg line;
		line.isArc = false;
		line.gap = IsGap(i - 1);
		line.a = cursor;
		line.b = lineEnd;
		line.sLen = Dist2D(cursor, lineEnd);
		line.wpIndex = i;
		line.clearance = std::min(waypoints[i - 1].clearance, waypoints[i].clearance);
		line.vCap = std::min(NodeMaxSpeed(i - 1), NodeMaxSpeed(i));
		if (waypoints[i - 1].requiresLadder && waypoints[i - 1].ladderSpeed > 1.f)
			line.vCap = std::min(line.vCap, waypoints[i - 1].ladderSpeed);
		if (line.sLen > kEpsLen)
			PushSeg(line);

		cursor = lineEnd;

		if (!last && c.arc)
		{
			ProfileSeg arc;
			arc.isArc = true;
			arc.a = c.entry;
			arc.b = c.exit;
			arc.center = c.center;
			arc.radius = c.radius;
			arc.startAngle = c.startAngle;
			arc.turn = c.turn;
			arc.sLen = c.radius * std::fabs(c.turn);
			arc.wpIndex = i;
			arc.clearance = waypoints[i].clearance;
			arc.vCap = c.vCap;
			PushSeg(arc);

			cursor = c.exit;
			++profile.cornersArced;
		}

		if (c.slowed)
			++profile.cornersSlowed;
		if (c.shrunk)
			++profile.cornersShrunk;
	}

	if (profile.segs.empty())
		return profile;

	std::vector<float> nodeCap(n, 0.f);
	for (size_t i = 0; i < n; ++i)
		nodeCap[i] = NodeMaxSpeed(i);
	for (size_t i = 1; i + 1 < n; ++i)
		if (!corners[i].arc)
			nodeCap[i] = std::min(nodeCap[i], corners[i].vCap);
	nodeCap[n - 1] = 0.f;

	float vEnd = 0.f;
	for (size_t k = profile.segs.size(); k-- > 0;)
	{
		ProfileSeg &seg = profile.segs[k];

		float cap = seg.vCap;
		if (!seg.isArc)
			cap = std::min(cap, nodeCap[std::min(seg.wpIndex, n - 1)]);

		seg.vExit = std::min(cap, vEnd);
		seg.vEntry = std::min(seg.vCap, BrakeBack(seg.vExit, seg.sLen, f, s0));
		if (!seg.isArc && !seg.gap && seg.wpIndex >= 1)
			seg.vEntry = std::min(seg.vEntry, nodeCap[seg.wpIndex - 1]);

		if (seg.gap && seg.wpIndex >= 1)
		{
			const PlanWaypoint &startWp = waypoints[seg.wpIndex - 1];
			vEnd = (startWp.airmoveLaunch && startWp.launchSpeed > 1.f)
				? startWp.launchSpeed
				: NodeMaxSpeed(seg.wpIndex - 1);
		}
		else
		{
			vEnd = seg.vEntry;
		}
	}

	float fwdAccel = profile.accel;
	float prevExit = 0.f;
	for (ProfileSeg &seg : profile.segs)
	{
		if (seg.gap)
		{
			if (seg.wpIndex >= 1)
			{
				const PlanWaypoint &startWp = waypoints[seg.wpIndex - 1];
				prevExit = (startWp.airmoveLaunch && startWp.launchSpeed > 1.f)
					? std::min(startWp.launchSpeed, prevExit)
					: seg.vExit;
			}
			else
			{
				prevExit = seg.vExit;
			}
			seg.vReachEntry = prevExit;
			seg.vReachExit = prevExit;
			continue;
		}

		seg.vReachEntry = std::min(seg.vEntry, prevExit);
		seg.vReachExit = std::min(seg.vExit,
			AccelForward(seg.vReachEntry, seg.sLen, fwdAccel, f, s0));
		prevExit = seg.vReachExit;
	}

	float v = 0.f;
	float dt = physics.frameTime > 0.f ? physics.frameTime : 1.f / 60.f;
	float accel = physics.accelerate * physics.maxSpeed;
	int steps = 0;
	for (ProfileSeg &seg : profile.segs)
	{
		float local = 0.f;
		while (local < seg.sLen && steps < kEtaMaxSteps)
		{
			float limit = SegSpeedAt(seg, local, f, s0);
			if (v < limit)
				v += (accel - f * std::max(v, s0)) * dt;
			v = std::min(v, limit);
			v = std::max(v, 1.f);
			local += v * dt;
			profile.totalTime += dt;
			++steps;
		}
	}

	for (const PlanWaypoint &wp : waypoints)
		if (wp.requiresElevatorRide)
			profile.totalTime += wp.elevatorWaitTime + wp.elevatorTravelTime;

	return profile;
}

float ProfileLaunchReach(const PlanProfile &profile, size_t wpIndex, float lead,
	float edgeFriction, float edgeZone)
{
	const ProfileSeg *approach = nullptr;
	for (const ProfileSeg &seg : profile.segs)
		if (seg.wpIndex == wpIndex && !seg.gap)
			approach = &seg;

	if (!approach)
		return 0.f;

	float clipped = std::max(lead, 0.f);
	float usable = std::max(0.f, approach->sLen - clipped);
	float edge = std::min(usable, std::max(0.f, edgeZone - clipped));

	float reach = AccelForward(approach->vReachEntry, usable - edge, profile.accel, profile.friction,
		profile.stopSpeed);
	reach = AccelForward(reach, edge, profile.accel, profile.friction * edgeFriction,
		profile.stopSpeed);

	return std::min(reach, approach->vExit);
}

float ProfileWaypointS(const PlanProfile &profile, size_t wpIndex)
{
	for (const ProfileSeg &seg : profile.segs)
	{
		if (seg.wpIndex < wpIndex)
			continue;
		if (seg.wpIndex == wpIndex && !seg.isArc)
			return seg.sStart + seg.sLen;
		return seg.sStart;
	}
	return profile.totalLength;
}

ProfileSample SampleProfile(const PlanProfile &profile, float s)
{
	ProfileSample out;
	if (profile.segs.empty())
		return out;

	if (s < 0.f)
		s = 0.f;
	if (s > profile.totalLength)
		s = profile.totalLength;

	size_t k = 0;
	while (k + 1 < profile.segs.size() && s > profile.segs[k].sStart + profile.segs[k].sLen)
		++k;

	const ProfileSeg &seg = profile.segs[k];
	float local = s - seg.sStart;
	float t = seg.sLen > kEpsLen ? local / seg.sLen : 0.f;
	t = std::max(0.f, std::min(1.f, t));

	out.clearance = seg.clearance;
	out.wpIndex = seg.wpIndex;
	out.gap = seg.gap;
	out.speed = SegSpeedAt(seg, local, profile.friction, profile.stopSpeed);

	if (seg.isArc)
	{
		float ang = seg.startAngle + seg.turn * t;
		out.point = Vector(seg.center.x + seg.radius * std::cos(ang),
			seg.center.y + seg.radius * std::sin(ang), seg.a.z + (seg.b.z - seg.a.z) * t);
		float sign = (seg.turn >= 0.f) ? 1.f : -1.f;
		out.tangent = Vector(-std::sin(ang) * sign, std::cos(ang) * sign, 0.f);
	}
	else
	{
		out.point = seg.a + (seg.b - seg.a) * t;
		out.tangent = Norm2D(seg.b - seg.a);
	}

	return out;
}

float ProjectOntoProfile(const PlanProfile &profile, const Vector &pos, float sMin, float sMax,
	float *lateralOut)
{
	if (lateralOut)
		*lateralOut = 0.f;
	if (profile.segs.empty())
		return 0.f;

	sMin = std::max(0.f, sMin);
	sMax = std::min(profile.totalLength, sMax);
	if (sMax < sMin)
		sMax = sMin;

	float bestS = sMin;
	float bestDist = 1e18f;

	for (const ProfileSeg &seg : profile.segs)
	{
		float segEnd = seg.sStart + seg.sLen;
		if (segEnd < sMin || seg.sStart > sMax || seg.sLen <= kEpsLen)
			continue;

		float candS;
		if (seg.isArc)
		{
			float dx = pos.x - seg.center.x, dy = pos.y - seg.center.y;
			float ang = std::atan2(dy, dx);
			float rel = ang - seg.startAngle;
			while (rel > NavMath::kPiF)
				rel -= 2.f * NavMath::kPiF;
			while (rel < -NavMath::kPiF)
				rel += 2.f * NavMath::kPiF;

			float t = (std::fabs(seg.turn) > 0.0001f) ? (rel / seg.turn) : 0.f;
			t = std::max(0.f, std::min(1.f, t));
			candS = seg.sStart + t * seg.sLen;
		}
		else
		{
			Vector d = seg.b - seg.a;
			float lenSq = d.x * d.x + d.y * d.y;
			float t = 0.f;
			if (lenSq > kEpsLen)
				t = ((pos.x - seg.a.x) * d.x + (pos.y - seg.a.y) * d.y) / lenSq;
			t = std::max(0.f, std::min(1.f, t));
			candS = seg.sStart + t * seg.sLen;
		}

		candS = std::max(sMin, std::min(sMax, candS));

		ProfileSample sample = SampleProfile(profile, candS);
		float dist = Dist2D(sample.point, pos);
		if (dist < bestDist)
		{
			bestDist = dist;
			bestS = candS;
		}
	}

	if (lateralOut)
		*lateralOut = (bestDist < 1e17f) ? bestDist : 0.f;
	return bestS;
}

}
