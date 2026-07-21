#include "polytope.h"

#include <cmath>

namespace navgen
{

double Vec3::Length() const
{
	return std::sqrt(x * x + y * y + z * z);
}

Vec3 Vec3::Normalized() const
{
	double len = Length();
	if (len < 1e-9)
		return Vec3(0, 0, 0);
	return Vec3(x / len, y / len, z / len);
}

Winding BaseWindingForPlane(const Plane &plane, double mapRadius)
{
	const Vec3 &n = plane.normal;

	// Pick whichever cardinal axis is *least* aligned with the normal as the
	// seed for an orthogonal in-plane basis, then Gram-Schmidt it off the
	// normal -- standard base-winding-for-plane construction.
	double ax = std::fabs(n.x);
	double ay = std::fabs(n.y);
	double az = std::fabs(n.z);

	Vec3 seed;
	if (az <= ax && az <= ay)
		seed = Vec3(0, 0, 1);
	else if (ay <= ax)
		seed = Vec3(0, 1, 0);
	else
		seed = Vec3(1, 0, 0);

	Vec3 up = (seed - n * seed.Dot(n)).Normalized();
	Vec3 right = n.Cross(up).Normalized();

	up = up * mapRadius;
	right = right * mapRadius;

	Vec3 org = n * plane.dist;

	Winding w;
	w.push_back(org - right + up);
	w.push_back(org + right + up);
	w.push_back(org + right - up);
	w.push_back(org - right - up);
	return w;
}

Winding ClipWinding(const Winding &winding, const Plane &plane, bool keepFront, double epsilon)
{
	if (winding.empty())
		return {};

	size_t n = winding.size();
	std::vector<double> dists(n);
	std::vector<int> sides(n); // 1 front, -1 back, 0 on-plane

	for (size_t i = 0; i < n; ++i)
	{
		double d = winding[i].Dot(plane.normal) - plane.dist;
		dists[i] = d;
		sides[i] = (d > epsilon) ? 1 : (d < -epsilon ? -1 : 0);
	}

	Winding result;
	for (size_t i = 0; i < n; ++i)
	{
		size_t j = (i + 1) % n;

		bool keepI = keepFront ? (sides[i] >= 0) : (sides[i] <= 0);
		if (keepI)
			result.push_back(winding[i]);

		bool crosses = (sides[i] > 0 && sides[j] < 0) || (sides[i] < 0 && sides[j] > 0);
		if (crosses)
		{
			double t = dists[i] / (dists[i] - dists[j]);
			result.push_back(winding[i] + (winding[j] - winding[i]) * t);
		}
	}
	return result;
}

double WindingArea(const Winding &winding, const Vec3 &normal)
{
	if (winding.size() < 3)
		return 0.0;

	Vec3 sum(0, 0, 0);
	for (size_t i = 1; i + 1 < winding.size(); ++i)
	{
		Vec3 a = winding[i] - winding[0];
		Vec3 b = winding[i + 1] - winding[0];
		sum = sum + a.Cross(b);
	}
	return std::fabs(sum.Dot(normal.Normalized())) * 0.5;
}

double Polytope::Volume() const
{
	if (faces.empty())
		return 0.0;

	// Signed-tetrahedron-volume summation from an arbitrary reference point
	// is exact for any reference point over a closed convex polytope, so an
	// approximate "average of all face vertices" reference is fine here.
	Vec3 ref = Centroid();

	double vol = 0.0;
	for (const Face &f : faces)
	{
		const Winding &w = f.winding;
		if (w.size() < 3)
			continue;
		for (size_t i = 1; i + 1 < w.size(); ++i)
		{
			Vec3 a = w[0] - ref;
			Vec3 b = w[i] - ref;
			Vec3 c = w[i + 1] - ref;
			vol += a.Dot(b.Cross(c)) / 6.0;
		}
	}
	return std::fabs(vol);
}

Vec3 Polytope::Centroid() const
{
	Vec3 sum(0, 0, 0);
	int count = 0;
	for (const Face &f : faces)
	{
		for (const Vec3 &v : f.winding)
		{
			sum = sum + v;
			++count;
		}
	}
	if (count == 0)
		return Vec3();
	return sum * (1.0 / count);
}

} // namespace navgen
