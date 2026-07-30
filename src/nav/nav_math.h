#pragma once

// Math shared by the bot, the runtime nav code and the offline navgen compiler.
//
// Header-only and dependency-free on purpose: navgen names the src/nav translation units it
// links one by one in CMakeLists.txt, so a .cpp here would have to be registered on two
// targets, and pulling in the SDK's vector.h would drag it into navgen's geometry code,
// which is deliberately SDK-free.

#include <cmath>

namespace NavMath
{

constexpr double kPi = 3.14159265358979323846;
constexpr float kPiF = static_cast<float>(kPi);

// Unit vector in the XY plane, zero when the input has no horizontal component. Templated
// on the vector type for the same reason as above: it keeps vector.h out of this header,
// and it works for both the SDK's float Vector and navgen's double Vec3.
template <typename V>
V Norm2D(const V &v)
{
	auto len = std::sqrt(v.x * v.x + v.y * v.y);
	if (len < 0.0001f)
		return V(0.f, 0.f, 0.f);
	return V(v.x / len, v.y / len, 0.f);
}

// An area is the intersection of its half-spaces and its plane normals point inward, so a
// point is inside when it sits in front of every plane. Templated over the plane container
// because the three area representations spell it differently -- Nav::AreaPlane and
// NavPlan::PlanPlane hold a float Vector, navgen::Plane holds a double Vec3 -- while all of
// them expose .normal{.x,.y,.z} and .dist.
//
// navgen uses this to decide which area a spawn or info_target sits in and the runtime uses
// it to decide which area a bot sits in. If the two ever drift, bots get routed out of areas
// they are not standing in, so both sides go through this one predicate.
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
