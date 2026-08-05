#include "polytope.h"

#include <algorithm>
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
	std::vector<int> sides(n);

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

double WindingAreaXY(const Winding &winding)
{
	if (winding.size() < 3)
		return 0.0;

	double sum = 0.0;
	for (size_t i = 0; i < winding.size(); ++i)
	{
		const Vec3 &a = winding[i];
		const Vec3 &b = winding[(i + 1) % winding.size()];
		sum += a.x * b.y - b.x * a.y;
	}
	return std::fabs(sum) * 0.5;
}

bool IsFloorFace(const Face &face)
{
	return face.walkable && face.plane.normal.z > kFloorNormalZ && face.winding.size() >= 3;
}

Winding WalkableFootprint(const Polytope &poly)
{
	std::vector<Vec3> pts;
	for (const Face &f : poly.faces)
	{
		if (!IsFloorFace(f))
			continue;
		for (const Vec3 &v : f.winding)
			pts.push_back(v);
	}
	return ConvexHullXY(pts);
}

namespace
{

Winding ProjectXY(const Winding &winding)
{
	Winding out;
	out.reserve(winding.size());
	for (const Vec3 &v : winding)
		out.push_back(Vec3(v.x, v.y, 0.0));

	double sum = 0.0;
	for (size_t i = 0; i < out.size(); ++i)
	{
		const Vec3 &a = out[i];
		const Vec3 &b = out[(i + 1) % out.size()];
		sum += a.x * b.y - b.x * a.y;
	}
	if (sum < 0.0)
		std::reverse(out.begin(), out.end());

	return out;
}

std::vector<Winding> SubtractConvexXY(const Winding &subject, const Winding &clip)
{
	std::vector<Winding> outside;
	Winding inside = subject;

	for (size_t i = 0; i < clip.size() && inside.size() >= 3; ++i)
	{
		const Vec3 &a = clip[i];
		const Vec3 &b = clip[(i + 1) % clip.size()];

		Vec3 edge = b - a;
		Vec3 normal(-edge.y, edge.x, 0.0);
		double len = normal.Length();
		if (len < 1e-9)
			continue;

		Plane plane;
		plane.normal = normal * (1.0 / len);
		plane.dist = plane.normal.Dot(a);

		Winding beyond = ClipWinding(inside, plane, false);
		if (beyond.size() >= 3)
			outside.push_back(std::move(beyond));

		inside = ClipWinding(inside, plane, true);
	}

	return outside;
}

}

double WalkableFootprintGapArea(const Polytope &poly)
{
	Winding hull = WalkableFootprint(poly);
	if (hull.size() < 3)
		return 0.0;

	std::vector<Winding> remainder;
	remainder.push_back(std::move(hull));

	for (const Face &f : poly.faces)
	{
		if (!IsFloorFace(f))
			continue;

		Winding clip = ProjectXY(f.winding);

		std::vector<Winding> next;
		for (const Winding &piece : remainder)
			for (Winding &part : SubtractConvexXY(piece, clip))
				next.push_back(std::move(part));
		remainder.swap(next);
	}

	double gap = 0.0;
	for (const Winding &w : remainder)
		gap += WindingAreaXY(w);
	return gap;
}

Winding ConvexHullXY(const std::vector<Vec3> &points)
{
	Winding pts;
	pts.reserve(points.size());
	for (const Vec3 &p : points)
		pts.push_back(Vec3(p.x, p.y, 0.0));

	std::sort(pts.begin(), pts.end(), [](const Vec3 &a, const Vec3 &b)
		{ return a.x < b.x || (a.x == b.x && a.y < b.y); });
	pts.erase(std::unique(pts.begin(), pts.end(),
			[](const Vec3 &a, const Vec3 &b) { return a.x == b.x && a.y == b.y; }),
		pts.end());

	int n = static_cast<int>(pts.size());
	if (n < 3)
		return pts;

	auto cross = [](const Vec3 &o, const Vec3 &a, const Vec3 &b)
	{
		return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);
	};

	Winding hull(2 * static_cast<size_t>(n));
	int k = 0;
	for (int i = 0; i < n; ++i)
	{
		while (k >= 2 && cross(hull[static_cast<size_t>(k - 2)], hull[static_cast<size_t>(k - 1)],
			pts[static_cast<size_t>(i)]) <= 0.0)
			--k;
		hull[static_cast<size_t>(k++)] = pts[static_cast<size_t>(i)];
	}

	int lower = k + 1;
	for (int i = n - 2; i >= 0; --i)
	{
		while (k >= lower && cross(hull[static_cast<size_t>(k - 2)], hull[static_cast<size_t>(k - 1)],
			pts[static_cast<size_t>(i)]) <= 0.0)
			--k;
		hull[static_cast<size_t>(k++)] = pts[static_cast<size_t>(i)];
	}

	hull.resize(static_cast<size_t>(k - 1));
	return hull;
}

Winding IntersectCoplanarPolygons(const Winding &subject, const Winding &clip, const Vec3 &planeNormal)
{
	if (clip.size() < 3)
		return {};

	Vec3 centroid(0, 0, 0);
	for (const Vec3 &v : clip)
		centroid = centroid + v;
	centroid = centroid * (1.0 / static_cast<double>(clip.size()));

	Winding result = subject;
	for (size_t i = 0; i < clip.size() && !result.empty(); ++i)
	{
		Vec3 e1 = clip[i];
		Vec3 e2 = clip[(i + 1) % clip.size()];
		Vec3 edgeDir = (e2 - e1).Normalized();
		if (edgeDir.Length() < 1e-9)
			continue;

		Plane cutPlane;
		cutPlane.normal = planeNormal.Cross(edgeDir).Normalized();
		cutPlane.dist = cutPlane.normal.Dot(e1);

		if (cutPlane.normal.Dot(centroid) - cutPlane.dist < 0.0)
		{
			cutPlane.normal = cutPlane.normal * -1.0;
			cutPlane.dist = -cutPlane.dist;
		}

		result = ClipWinding(result, cutPlane,  true, 0.01);
	}
	return result;
}

double Polytope::Volume() const
{
	if (faces.empty())
		return 0.0;

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

Polytope ClipPolytope(const Polytope &poly, const Plane &plane, bool keepFront, double mapRadius, double epsilon)
{
	Polytope result;

	for (const Face &f : poly.faces)
	{
		Winding w = ClipWinding(f.winding, plane, keepFront, epsilon);
		if (w.size() >= 3)
			result.faces.push_back({f.plane, std::move(w), f.walkable});
	}

	Plane facePlane = plane;
	if (!keepFront)
	{
		facePlane.normal = plane.normal * -1.0;
		facePlane.dist = -plane.dist;
	}

	Winding cross = BaseWindingForPlane(facePlane, mapRadius);
	for (const Face &f : poly.faces)
	{
		cross = ClipWinding(cross, f.plane,  true, epsilon);
		if (cross.empty())
			break;
	}
	if (cross.size() >= 3)
		result.faces.push_back({facePlane, std::move(cross)});

	return result;
}

Polytope TranslatePolytope(const Polytope &poly, const Vec3 &offset)
{
	Polytope result;
	result.faces.reserve(poly.faces.size());
	for (const Face &f : poly.faces)
	{
		Face nf;
		nf.plane.normal = f.plane.normal;
		nf.plane.dist = f.plane.dist + f.plane.normal.Dot(offset);
		nf.walkable = f.walkable;
		nf.winding.reserve(f.winding.size());
		for (const Vec3 &v : f.winding)
			nf.winding.push_back(v + offset);
		result.faces.push_back(std::move(nf));
	}
	return result;
}

}

