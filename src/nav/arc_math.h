#pragma once

#include <algorithm>
#include <cmath>

namespace ArcMath
{

template <typename V, typename T>
V ArcPos(const V &p0, const V &v0, T g, T t)
{
	return V(p0.x + v0.x * t, p0.y + v0.y * t, p0.z + v0.z * t - T(0.5) * g * t * t);
}

template <typename T>
T ArcVelZ(T vz0, T g, T t)
{
	return vz0 - g * t;
}

template <typename T>
T ArcPeakZ(T z0, T vz0, T g, T t)
{
	T apex = (g > T(0)) ? std::clamp(vz0 / g, T(0), t) : (vz0 > T(0) ? t : T(0));
	return z0 + vz0 * apex - T(0.5) * g * apex * apex;
}

template <typename T>
int ArcPlaneRoots(T f0, T b, T c, T ta, T tb, T roots[2])
{
	T r[2];
	int n = 0;
	if (c == T(0))
	{
		if (b == T(0))
			return 0;
		r[n++] = -f0 / b;
	}
	else
	{
		T disc = b * b - T(4) * c * f0;
		if (disc <= T(0))
			return 0;
		T q = T(-0.5) * (b + std::copysign(std::sqrt(disc), b));
		r[0] = q / c;
		r[1] = f0 / q;
		if (r[0] > r[1])
			std::swap(r[0], r[1]);
		n = 2;
	}

	int count = 0;
	for (int i = 0; i < n; ++i)
	{
		if (r[i] > ta && r[i] < tb)
			roots[count++] = r[i];
	}
	return count;
}

template <typename V, typename T>
void ArcBounds(const V &p0, const V &v0, T g, T t, V &mins, V &maxs)
{
	V p1 = ArcPos(p0, v0, g, t);
	mins = V(std::min(p0.x, p1.x), std::min(p0.y, p1.y), std::min(p0.z, p1.z));
	maxs = V(std::max(p0.x, p1.x), std::max(p0.y, p1.y), std::max(p0.z, p1.z));
	maxs.z = std::max(maxs.z, ArcPeakZ(p0.z, v0.z, g, t));
}

template <typename T>
bool ArcLinearSlab(T p, T v, T lo, T hi, T &t0, T &t1)
{
	if (v == T(0))
		return p >= lo && p <= hi;
	T a = (lo - p) / v;
	T b = (hi - p) / v;
	if (a > b)
		std::swap(a, b);
	t0 = std::max(t0, a);
	t1 = std::min(t1, b);
	return t0 <= t1;
}

template <typename V, typename T>
T ArcEnterBox(const V &p0, const V &v0, T g, const V &mins, const V &maxs, T tmax)
{
	T t0 = T(0);
	T t1 = tmax;
	if (!ArcLinearSlab<T>(p0.x, v0.x, mins.x, maxs.x, t0, t1) ||
		!ArcLinearSlab<T>(p0.y, v0.y, mins.y, maxs.y, t0, t1))
		return tmax;

	T candidates[5] = { t0 };
	int n = 1;
	T c = T(-0.5) * g;
	n += ArcPlaneRoots<T>(p0.z - maxs.z, v0.z, c, t0, t1, candidates + n);
	n += ArcPlaneRoots<T>(p0.z - mins.z, v0.z, c, t0, t1, candidates + n);
	std::sort(candidates, candidates + n);

	T slack = (maxs.z - mins.z) * T(1e-6) + T(1e-3);
	for (int i = 0; i < n; ++i)
	{
		T z = p0.z + v0.z * candidates[i] + c * candidates[i] * candidates[i];
		if (z >= mins.z - slack && z <= maxs.z + slack)
			return candidates[i];
	}
	return tmax;
}

}
