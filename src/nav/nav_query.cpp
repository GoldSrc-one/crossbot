#include "nav_query.h"

namespace Nav
{

namespace
{
constexpr float kAabbEpsilon = 4.0f; // slack so points right at a boundary still resolve
constexpr float kPlaneEpsilon = 2.0f; // slack for the half-space containment test, same spirit as kAabbEpsilon

// True containment in the area's real convex polytope (nav_format.h v4),
// not just its AABB -- MergeAreas guarantees every area is genuinely
// convex, so this is exact modulo float slop, unlike the AABB+centroid
// guess below which can (and, live-verified on crossfire.bsp, did) pick
// the wrong area entirely for two AABB-overlapping-but-not-actually-
// containing neighbors.
bool ContainsPoint(const Area &area, const Vector &point)
{
	if (area.planes.empty())
		return false; // shouldn't happen for a v4+ file, but fail closed rather than false-claim containment
	for (const AreaPlane &plane : area.planes)
	{
		float d = plane.normal.x * point.x + plane.normal.y * point.y + plane.normal.z * point.z - plane.dist;
		if (d < -kPlaneEpsilon)
			return false;
	}
	return true;
}
}

const Area *FindAreaForPoint(const NavMesh &mesh, const Vector &point)
{
	for (const Area &area : mesh.areas)
	{
		if (point.x < area.mins.x - kAabbEpsilon || point.x > area.maxs.x + kAabbEpsilon ||
			point.y < area.mins.y - kAabbEpsilon || point.y > area.maxs.y + kAabbEpsilon ||
			point.z < area.mins.z - kAabbEpsilon || point.z > area.maxs.z + kAabbEpsilon)
			continue; // not even AABB-close -- skip the plane test, it's cheap but not free

		if (ContainsPoint(area, point))
			return &area; // real containment -- trust it immediately, no need to keep searching
	}

	// No area's real polytope actually contains the point (e.g. it's
	// slightly outside every one, like an elevator landing point derived
	// from the door's own compiled bounds rather than the adjacent floor
	// area's) -- fall back to AABB-overlap + nearest-centroid, same
	// two-tier approach tools/navgen's own point-to-area lookups use.
	const Area *best = nullptr;
	float bestDist = 0.f;
	for (const Area &area : mesh.areas)
	{
		if (point.x < area.mins.x - kAabbEpsilon || point.x > area.maxs.x + kAabbEpsilon ||
			point.y < area.mins.y - kAabbEpsilon || point.y > area.maxs.y + kAabbEpsilon ||
			point.z < area.mins.z - kAabbEpsilon || point.z > area.maxs.z + kAabbEpsilon)
			continue;

		float dist = (point - area.centroid).Length();
		if (!best || dist < bestDist)
		{
			best = &area;
			bestDist = dist;
		}
	}

	if (best || mesh.areas.empty())
		return best;

	// Nothing was within any area's (epsilon-padded) AABB at all -- fails
	// safe with a nearest-centroid fallback instead of returning nullptr,
	// matching the same two-tier resolution tools/navgen's own point-to-
	// area lookups already use (reachability.cpp's FindSpawnAreas,
	// elevator_tagger.cpp's FindAreaForWorldPoint). Without this, any
	// point that happens to land just outside every AABB -- like an
	// elevator's landing point, which is derived from the *door's*
	// compiled bounds rather than the adjacent floor area's -- silently
	// fails to resolve at all, confirmed live via crossbot_nav_goal.
	const Area *nearest = nullptr;
	float nearestDist = 0.f;
	for (const Area &area : mesh.areas)
	{
		float dist = (point - area.centroid).Length();
		if (!nearest || dist < nearestDist)
		{
			nearest = &area;
			nearestDist = dist;
		}
	}
	return nearest;
}

const Area *FindAreaById(const NavMesh &mesh, uint32_t id)
{
	if (id < mesh.areas.size() && mesh.areas[id].id == id)
		return &mesh.areas[id];
	for (const Area &area : mesh.areas)
		if (area.id == id)
			return &area;
	return nullptr;
}

} // namespace Nav
