#pragma once

#include <extdll.h>

#include "nav_mesh.h"

namespace Nav
{

// Finds the area containing `point`. Each area's real convex-polytope
// boundary planes are baked into the format (nav_format.h v4), so this
// first does a true half-space containment test (areas are provably
// convex and non-overlapping in 3D, per MergeAreas -- exact modulo float
// slop) rather than trusting AABB overlap alone, which can (and did, live
// on crossfire.bsp -- silently poisoning Repath's start/goal area choice)
// resolve to the wrong area entirely when two areas' AABBs overlap but
// their actual shapes don't at that point. Falls back to AABB-overlap +
// nearest-centroid, then global-nearest-centroid, only when no area's
// real polytope contains the point at all (e.g. a point derived from
// compiled entity bounds rather than the nav mesh itself, like an
// elevator landing point) -- mirrors tools/navgen's own two-tier
// point-to-area lookups. Only returns nullptr if the mesh has no areas at
// all. Linear-scan cost is fine at the area counts seen so far (~700-800
// on crossfire.bsp); revisit only if profiling on a much larger map shows
// it matters.
const Area *FindAreaForPoint(const NavMesh &mesh, const Vector &point);

// Finds the area with the given id. navgen/nav_loader keep area ids
// contiguous and index-matched, so this fast-paths to mesh.areas[id]; the
// linear scan is only a defensive fallback against that invariant ever
// slipping. Returns nullptr if no area has this id.
const Area *FindAreaById(const NavMesh &mesh, uint32_t id);

} // namespace Nav
