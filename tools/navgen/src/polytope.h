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

// A plane in the form dot(normal, x) = dist. The "front" half-space is
// dot(normal, x) - dist >= 0, matching GoldSrc's dplane_t / dclipnode_t
// child[0]=front, child[1]=back convention.
struct Plane
{
	Vec3 normal;
	double dist = 0.0;
};

// An ordered polygon boundary, used both as an intermediate clipping
// primitive and as a leaf's finished face boundary.
using Winding = std::vector<Vec3>;

// A huge quad lying exactly on `plane`, meant to be clipped down by
// ClipWinding against the rest of a plane chain -- the standard
// "brush reconstruction from planes" starting primitive (as used by id's
// map compilers and bspc). mapRadius must exceed the map's actual extent.
Winding BaseWindingForPlane(const Plane &plane, double mapRadius);

// Clips `winding` to one half-space of `plane` (Sutherland-Hodgman, single
// plane): keepFront=true keeps dot(normal,x)-dist >= -epsilon, keepFront=
// false keeps <= epsilon. Returns an empty winding if nothing survives.
Winding ClipWinding(const Winding &winding, const Plane &plane, bool keepFront, double epsilon = 0.01);

// Area of a planar winding (fan-triangulated from its first vertex; assumes
// all vertices are coplanar and the winding is convex, non-self-intersecting).
double WindingArea(const Winding &winding, const Vec3 &normal);

struct Face
{
	Plane plane;
	Winding winding; // clipped down to the polytope's actual boundary
};

struct Polytope
{
	std::vector<Face> faces;

	bool Empty() const { return faces.empty(); }

	// Both computed from face-vertex averages/tetrahedron decomposition --
	// approximate but exact enough for --dump-leaves sanity stats.
	double Volume() const;
	Vec3 Centroid() const;
};

} // namespace navgen
