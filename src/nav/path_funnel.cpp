#include "path_funnel.h"

#include <meta_api.h>

namespace Nav
{

namespace
{

// GoldSrc hull convention (dlls/extdef.h): point_hull=0, human_hull=1,
// large_hull=2, head_hull=3. Hardcoded here rather than pulling in a dlls
// header for one constant -- mirrors tools/navgen/src/hull_defs.h's own
// independent copy of the same well-known engine convention.
constexpr int kHullHuman = 1;

bool HasHullClearance(const Vector &from, const Vector &to, edict_t *pIgnore)
{
	TraceResult tr;
	g_engfuncs.pfnTraceHull(from, to, ignore_monsters, kHullHuman, pIgnore, &tr);
	return tr.flFraction >= 1.0f;
}

} // namespace

std::vector<Vector> BuildWaypointPath(const NavMesh &mesh, const PathResult &path, const Vector &startPoint,
	const Vector &goalPoint, edict_t *pIgnore)
{
	std::vector<Vector> raw;
	raw.push_back(startPoint);
	for (uint32_t portalIdx : path.portalPath)
	{
		const Portal &portal = mesh.portals[portalIdx];
		raw.push_back((portal.pointA + portal.pointB) * 0.5f);
	}
	raw.push_back(goalPoint);

	if (raw.size() <= 2)
		return raw;

	std::vector<Vector> simplified;
	simplified.push_back(raw.front());

	size_t anchor = 0;
	while (anchor < raw.size() - 1)
	{
		size_t furthest = anchor + 1;
		for (size_t candidate = anchor + 2; candidate < raw.size(); ++candidate)
		{
			if (HasHullClearance(raw[anchor], raw[candidate], pIgnore))
				furthest = candidate;
		}
		simplified.push_back(raw[furthest]);
		anchor = furthest;
	}

	return simplified;
}

} // namespace Nav
