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

std::vector<Waypoint> BuildWaypointPath(const NavMesh &mesh, const PathResult &path, const Vector &startPoint,
	const Vector &goalPoint, edict_t *pIgnore)
{
	std::vector<Waypoint> raw;
	raw.push_back({startPoint, false});
	for (uint32_t portalIdx : path.portalPath)
	{
		const Portal &portal = mesh.portals[portalIdx];
		bool isJump = portal.edgeType == crossbot_nav::EDGE_JUMP;
		raw.push_back({(portal.pointA + portal.pointB) * 0.5f, isJump});
	}
	raw.push_back({goalPoint, false});

	if (raw.size() <= 2)
		return raw;

	std::vector<Waypoint> simplified;
	simplified.push_back(raw.front());

	size_t anchor = 0;
	while (anchor < raw.size() - 1)
	{
		// Jump waypoints are hard stops: never merge past one via a
		// shortcut hull trace (see the header comment for why that trace
		// can't actually validate a gap crossing).
		if (raw[anchor + 1].requiresJump)
		{
			simplified.push_back(raw[anchor + 1]);
			anchor = anchor + 1;
			continue;
		}

		size_t furthest = anchor + 1;
		for (size_t candidate = anchor + 2; candidate < raw.size(); ++candidate)
		{
			if (raw[candidate].requiresJump)
				break; // don't skip over/through a required jump
			if (HasHullClearance(raw[anchor].point, raw[candidate].point, pIgnore))
				furthest = candidate;
		}
		simplified.push_back(raw[furthest]);
		anchor = furthest;
	}

	return simplified;
}

} // namespace Nav
