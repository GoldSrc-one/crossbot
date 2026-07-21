#pragma once

#include <extdll.h>

#include "nav_mesh.h"

namespace Nav
{

// Finds the area containing `point`, via a linear AABB scan (areas are
// convex and non-overlapping in true 3D space, but the baked format only
// stores each area's bounding box, not its exact polytope, so AABBs can
// overlap near shared boundaries even where the true shapes don't --
// ties are broken by nearest centroid). Returns nullptr if no area's AABB
// contains the point at all. Linear-scan cost is fine at the area counts
// seen so far (~800 on crossfire.bsp); revisit only if profiling on a
// much larger map shows it matters.
const Area *FindAreaForPoint(const NavMesh &mesh, const Vector &point);

} // namespace Nav
