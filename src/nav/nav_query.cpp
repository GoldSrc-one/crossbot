#include "nav_query.h"

namespace Nav
{

namespace
{
constexpr float kAabbEpsilon = 4.0f; // slack so points right at a boundary still resolve
}

const Area *FindAreaForPoint(const NavMesh &mesh, const Vector &point)
{
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

	return best;
}

} // namespace Nav
