#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>

#include "mathlib.h"
#include "const.h"
#include "bspfile.h"

#include "arc_math.h"

namespace HullTrace
{

struct HullTree
{
	const dclipnode_t *clipnodes = nullptr;
	size_t clipnodeCount = 0;
	const dplane_t *planes = nullptr;
	size_t planeCount = 0;
};

enum class HullStep
{
	Continue,
	Stop,
	Clear
};

constexpr double kDistEpsilon = 0.03125;

template <typename V>
auto PlaneDiff(const dplane_t &plane, const V &p)
{
	using T = decltype(p.x);
	return static_cast<T>(plane.normal[0]) * p.x + static_cast<T>(plane.normal[1]) * p.y +
		static_cast<T>(plane.normal[2]) * p.z - static_cast<T>(plane.dist);
}

template <typename V>
int ClassifyAt(const HullTree &hull, int nodeIndex, const V &point)
{
	while (nodeIndex >= 0)
	{
		const dclipnode_t &node = hull.clipnodes[static_cast<size_t>(nodeIndex)];
		const dplane_t &plane = hull.planes[static_cast<size_t>(node.planenum)];
		nodeIndex = (PlaneDiff(plane, point) >= 0) ? node.children[0] : node.children[1];
	}
	return nodeIndex;
}

template <bool BoolOnly, typename V, typename T, typename Trace>
HullStep HullCheck(const HullTree &hull, int headnode, int nodeIndex, T p1f, T p2f, const V &p1, const V &p2,
	Trace &trace)
{
	if (nodeIndex < 0)
	{
		if constexpr (BoolOnly)
		{
			return (nodeIndex == CONTENTS_SOLID) ? HullStep::Stop : HullStep::Continue;
		}
		else
		{
			if (nodeIndex != CONTENTS_SOLID)
			{
				trace.allsolid = false;
				trace.contents = nodeIndex;
			}
			else
			{
				trace.startsolid = true;
			}
			return HullStep::Continue;
		}
	}

	const dclipnode_t &node = hull.clipnodes[static_cast<size_t>(nodeIndex)];
	const dplane_t &plane = hull.planes[static_cast<size_t>(node.planenum)];

	T t1 = PlaneDiff(plane, p1);
	T t2 = PlaneDiff(plane, p2);

	if (t1 >= T(0) && t2 >= T(0))
		return HullCheck<BoolOnly>(hull, headnode, node.children[0], p1f, p2f, p1, p2, trace);
	if (t1 < T(0) && t2 < T(0))
		return HullCheck<BoolOnly>(hull, headnode, node.children[1], p1f, p2f, p1, p2, trace);

	T eps = static_cast<T>(kDistEpsilon);
	T frac = (t1 < T(0)) ? (t1 + eps) / (t1 - t2) : (t1 - eps) / (t1 - t2);
	frac = std::clamp(frac, T(0), T(1));

	T midf = p1f + (p2f - p1f) * frac;
	V mid = p1 + (p2 - p1) * frac;

	int side = (t1 < T(0)) ? 1 : 0;

	HullStep nearStep = HullCheck<BoolOnly>(hull, headnode, node.children[side], p1f, midf, p1, mid, trace);
	if (nearStep != HullStep::Continue)
		return nearStep;

	if (ClassifyAt(hull, node.children[side ^ 1], mid) != CONTENTS_SOLID)
		return HullCheck<BoolOnly>(hull, headnode, node.children[side ^ 1], midf, p2f, mid, p2, trace);

	if constexpr (BoolOnly)
	{
		if (static_cast<float>(midf) < 1.0f)
			return HullStep::Stop;
	}
	else
	{
		if (trace.allsolid)
			return HullStep::Stop;

		T sign = (side == 0) ? T(1) : T(-1);
		trace.planeNormal = V(static_cast<T>(plane.normal[0]) * sign, static_cast<T>(plane.normal[1]) * sign,
			static_cast<T>(plane.normal[2]) * sign);
	}

	while (ClassifyAt(hull, headnode, mid) == CONTENTS_SOLID)
	{
		frac -= T(0.1);
		if (frac < T(0))
			break;
		midf = p1f + (p2f - p1f) * frac;
		mid = p1 + (p2 - p1) * frac;
	}

	if constexpr (BoolOnly)
	{
		return (static_cast<float>(midf) < 1.0f) ? HullStep::Stop : HullStep::Clear;
	}
	else
	{
		trace.fraction = static_cast<decltype(trace.fraction)>(midf);
		trace.endpos = mid;
		return HullStep::Stop;
	}
}

template <typename V, typename T>
struct ArcTrace
{
	const HullTree &hull;
	int headnode;
	V start;
	V vel;
	V accel;
	T time;
	V normal;
	bool allsolid = true;
	bool startsolid = false;
	int contents = CONTENTS_EMPTY;
};

template <typename V, typename T>
V ArcAt(const ArcTrace<V, T> &trace, T t)
{
	T h = T(0.5) * t * t;
	return V(trace.start.x + trace.vel.x * t + trace.accel.x * h, trace.start.y + trace.vel.y * t + trace.accel.y * h,
		trace.start.z + trace.vel.z * t + trace.accel.z * h);
}

template <typename V, typename T>
HullStep ArcHit(ArcTrace<V, T> &trace, const dplane_t &plane, int side, T split, T ta, T tb)
{
	if (trace.allsolid)
		return HullStep::Stop;

	T sign = (side == 0) ? T(1) : T(-1);
	trace.normal = V(static_cast<T>(plane.normal[0]) * sign, static_cast<T>(plane.normal[1]) * sign,
		static_cast<T>(plane.normal[2]) * sign);

	T t = split;
	T backoff = T(0.1) * (tb - ta);
	while (backoff > T(0) && ClassifyAt(trace.hull, trace.headnode, ArcAt(trace, t)) == CONTENTS_SOLID)
	{
		if (t - backoff < ta)
			break;
		t -= backoff;
	}

	trace.time = t;
	return HullStep::Stop;
}

template <typename V, typename T>
HullStep ArcCheck(ArcTrace<V, T> &trace, int nodeIndex, T ta, T tb)
{
	if (nodeIndex < 0)
	{
		if (nodeIndex != CONTENTS_SOLID)
		{
			trace.allsolid = false;
			trace.contents = nodeIndex;
		}
		else
		{
			trace.startsolid = true;
		}
		return HullStep::Continue;
	}

	const dclipnode_t &node = trace.hull.clipnodes[static_cast<size_t>(nodeIndex)];
	const dplane_t &plane = trace.hull.planes[static_cast<size_t>(node.planenum)];

	T nx = static_cast<T>(plane.normal[0]);
	T ny = static_cast<T>(plane.normal[1]);
	T nz = static_cast<T>(plane.normal[2]);
	T f0 = PlaneDiff(plane, trace.start);
	T b = nx * trace.vel.x + ny * trace.vel.y + nz * trace.vel.z;
	T c = T(0.5) * (nx * trace.accel.x + ny * trace.accel.y + nz * trace.accel.z);

	T roots[2];
	int count = ArcMath::ArcPlaneRoots(f0, b, c, ta, tb, roots);

	T firstEnd = (count > 0) ? roots[0] : tb;
	T probe = (ta + firstEnd) * T(0.5);
	int side = (f0 + (b + c * probe) * probe >= T(0)) ? 0 : 1;

	T cur = ta;
	for (int i = 0; i < count; ++i)
	{
		T r = roots[i];
		T slope = std::fabs(b + T(2) * c * r);
		T split = std::clamp((slope > T(0)) ? r - static_cast<T>(kDistEpsilon) / slope : cur, cur, r);
		HullStep nearStep = ArcCheck(trace, node.children[side], cur, split);
		if (nearStep != HullStep::Continue)
			return nearStep;

		if (ClassifyAt(trace.hull, node.children[side ^ 1], ArcAt(trace, split)) == CONTENTS_SOLID)
			return ArcHit(trace, plane, side, split, ta, tb);

		cur = split;
		side ^= 1;
	}

	return ArcCheck(trace, node.children[side], cur, tb);
}

template <typename V, typename T>
ArcTrace<V, T> TraceArc(const HullTree &hull, int headnode, const V &start, const V &vel, const V &accel, T maxTime)
{
	ArcTrace<V, T> trace{ hull, headnode, start, vel, accel, maxTime, V() };
	ArcCheck(trace, headnode, T(0), maxTime);
	return trace;
}

}
