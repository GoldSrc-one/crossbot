#pragma once

#include <vector>

namespace navgen
{

struct Vec3
{
	double x = 0.0, y = 0.0, z = 0.0;

	Vec3() = default;
	Vec3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}

	Vec3 operator+(const Vec3 &o) const { return Vec3(x + o.x, y + o.y, z + o.z); }
	Vec3 operator-(const Vec3 &o) const { return Vec3(x - o.x, y - o.y, z - o.z); }
	Vec3 operator*(double s) const { return Vec3(x * s, y * s, z * s); }

	double Dot(const Vec3 &o) const { return x * o.x + y * o.y + z * o.z; }
	Vec3 Cross(const Vec3 &o) const
	{
		return Vec3(y * o.z - z * o.y, z * o.x - x * o.z, x * o.y - y * o.x);
	}
	double Length() const;
	Vec3 Normalized() const;
};

struct Plane
{
	Vec3 normal;
	double dist = 0.0;
};

using Winding = std::vector<Vec3>;

Winding BaseWindingForPlane(const Plane &plane, double mapRadius);

Winding ClipWinding(const Winding &winding, const Plane &plane, bool keepFront, double epsilon = 0.01);

double WindingArea(const Winding &winding, const Vec3 &normal);

Winding IntersectCoplanarPolygons(const Winding &subject, const Winding &clip, const Vec3 &planeNormal);

Winding ConvexHullXY(const std::vector<Vec3> &points);

struct Face
{
	Plane plane;
	Winding winding;
};

struct Polytope
{
	std::vector<Face> faces;

	bool Empty() const { return faces.empty(); }

	double Volume() const;
	Vec3 Centroid() const;
};

Polytope ClipPolytope(const Polytope &poly, const Plane &plane, bool keepFront, double mapRadius,
	double epsilon = 0.01);

Polytope TranslatePolytope(const Polytope &poly, const Vec3 &offset);

}

