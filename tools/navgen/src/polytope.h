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

// Clips `subject` (a winding lying on the same plane as `clip`, normal
// `planeNormal`) down to its intersection with the convex polygon `clip`,
// by successively cutting `subject` against each of clip's edges -- reuses
// ClipWinding by building, per edge, a plane containing that edge and
// perpendicular to planeNormal, oriented to keep clip's own interior.
// Shared by portal_builder.cpp (shared-face portal detection) and
// floor_splitter.cpp (floor-opening detection -- the same coplanar-overlap
// question asked of horizontal faces).
Winding IntersectCoplanarPolygons(const Winding &subject, const Winding &clip, const Vec3 &planeNormal);

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

// Clips a whole polytope to one half-space of `plane` (same keepFront
// convention as ClipWinding), the polytope analogue of ClipWinding: every
// existing face's winding gets clipped down, and -- if `plane` actually cuts
// through the polytope's interior -- a new cross-section face is added for
// `plane` itself, built the same "brush from planes" way BuildPolytopeFromChain
// builds any other face (a huge quad on the plane, clipped down by every one
// of the polytope's own original faces). mapRadius must exceed the map's
// actual extent (see BaseWindingForPlane). Returns an empty polytope if
// nothing survives.
Polytope ClipPolytope(const Polytope &poly, const Plane &plane, bool keepFront, double mapRadius,
	double epsilon = 0.01);

// Translates every face's winding (and correspondingly its plane.dist) by
// `offset` -- for a brush entity compiled around an "origin" brush
// (func_door_rotating, func_rotating, func_train, ...), the clipnode tree's
// planes are expressed in LOCAL space relative to that origin brush's
// centroid, not world space (hlcsg's own convention -- see
// edge_tagger.cpp/door_extractor.cpp's ParseEntityOrigin comment for the
// full explanation). A zero offset is a no-op, so this is safe to always
// apply, even to an entity with no origin brush at all.
Polytope TranslatePolytope(const Polytope &poly, const Vec3 &offset);

} // namespace navgen
