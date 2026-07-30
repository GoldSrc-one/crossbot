#pragma once

#include <cmath>

namespace NavMath
{

constexpr double kPi = 3.14159265358979323846;
constexpr float kPiF = static_cast<float>(kPi);

template <typename V>
V Norm2D(const V &v)
{
	auto len = std::sqrt(v.x * v.x + v.y * v.y);
	if (len < 0.0001f)
		return V(0.f, 0.f, 0.f);
	return V(v.x / len, v.y / len, 0.f);
}

template <typename Point, typename PlaneRange, typename Epsilon>
bool PointInsidePlanes(const Point &p, const PlaneRange &planes, Epsilon epsilon)
{
	if (planes.empty())
		return false;
	for (const auto &pl : planes)
		if (p.x * pl.normal.x + p.y * pl.normal.y + p.z * pl.normal.z - pl.dist < -epsilon)
			return false;
	return true;
}

}
