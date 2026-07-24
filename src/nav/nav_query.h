#pragma once

#include <extdll.h>

#include "nav_mesh.h"

namespace Nav
{

// Finds the area containing `point`. Uses each area's real convex-polytope boundary planes (baked in nav_format.h v4) for true half-space containment test (areas are provably convex and non-overlapping in 3D per MergeAreas). AABB-only overlap can resolve to wrong area when two areas' AABBs overlap but shapes don't at that point (live bug on crossfire.bsp -- silently poisoned Repath start/goal area choice). Falls back to AABB+nearest-centroid, then global-nearest-centroid, only when no real polytope contains the point (e.g., point from compiled entity bounds like elevator landing). Returns nullptr only if mesh has zero areas. Linear scan is fine at ~700-800 areas on crossfire.bsp.
const Area *FindAreaForPoint(const NavMesh &mesh, const Vector &point);

// Finds the area with the given id. navgen/nav_loader keep area ids
// contiguous and index-matched, so this fast-paths to mesh.areas[id]; the
// linear scan is only a defensive fallback against that invariant ever
// slipping. Returns nullptr if no area has this id.
const Area *FindAreaById(const NavMesh &mesh, uint32_t id);

} // namespace Nav
