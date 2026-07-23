#include "path_funnel.h"

#include <meta_api.h>

#include <cstdio>

#include "nav_query.h"

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

// Never simplified away, even when a straight hull trace across the gap
// happens to report "clear" -- pfnTraceHull only tests for solid collision
// along the line, not for a floor underneath it, so a clear trace over a
// jump gap or an elevator shaft doesn't mean it's walkable.
bool IsHardAnchor(const Waypoint &wp)
{
	return wp.requiresJump || wp.requiresElevatorRide || wp.requiresCrouch || wp.requiresDoor || wp.requiresLadder ||
		wp.requiresButtonUse;
}

bool AreaRequiresCrouch(const NavMesh &mesh, uint32_t areaId)
{
	const Area *area = FindAreaById(mesh, areaId);
	return area && (area->areaFlags & crossbot_nav::NAV_AREA_CROUCH) != 0;
}

} // namespace

std::vector<Waypoint> BuildWaypointPath(const NavMesh &mesh, const PathResult &path, const Vector &startPoint,
	const Vector &goalPoint, edict_t *pIgnore)
{
	std::vector<Waypoint> raw;
	raw.push_back({startPoint});
	for (size_t i = 0; i < path.portalPath.size(); ++i)
	{
		uint32_t portalIdx = path.portalPath[i];
		const Portal &portal = mesh.portals[portalIdx];

		if (portal.edgeType == crossbot_nav::EDGE_ELEVATOR)
		{
			// areaA is always the bottom landing (elevator_tagger.cpp's
			// convention, mirroring TagDropOffs' one-way drop-off
			// convention) -- checking which area we're arriving *from*
			// this step is what tells us whether we're calling the lift up
			// or just riding it back down.
			bool goingUp = (path.areaPath[i] == portal.areaA);

			if (goingUp)
			{
				// No special handling needed -- walking onto the button's
				// footprint touch-activates it like any func_button, the
				// same way a real player calls the lift.
				raw.push_back({portal.buttonPos});
			}

			Waypoint ride;
			ride.point = goingUp ? portal.pointA : portal.pointB;
			ride.requiresElevatorRide = true;
			ride.elevatorTravelTime = portal.travelTime;
			raw.push_back(ride);

			// Where riding actually leaves the bot -- a plain waypoint,
			// walked to normally once Bot_MoveNav's hold-and-wait finishes.
			raw.push_back({goingUp ? portal.pointB : portal.pointA});
			continue;
		}

		Waypoint wp;
		if (portal.edgeType == crossbot_nav::EDGE_JUMP)
		{
			// Unlike a real portal (pointA/pointB are two points on the
			// actual shared-face boundary -- see this file's header
			// comment), a jump portal's pointA/pointB are literally the
			// two areas' raw centroids (a proximity guess, ProbeJumpGaps
			// in edge_tagger.cpp) -- their midpoint sits roughly over the
			// gap itself, not on any real standable face. Aim at the far
			// side (the actual destination) instead; targetAreaId below
			// is what makes "reached" work even though that centroid can
			// be a long, wall-obstructed walk from the takeoff point.
			bool travelingFromA = (path.areaPath[i] == portal.areaA);
			wp.point = travelingFromA ? portal.pointB : portal.pointA;
		}
		else if (portal.edgeType == crossbot_nav::EDGE_LADDER)
		{
			// edge_tagger.cpp's TagLadder sets anchorFrom/anchorTo (->
			// pointA/pointB here) to the ladder's bottom/top respectively,
			// not two points on a shared face -- same reasoning as
			// EDGE_JUMP above, aim at the far end (wherever this crossing
			// is actually headed), not their midpoint (which would aim the
			// bot at a point floating in mid-air halfway up the ladder).
			// The actual climb itself needs no special waypoint handling:
			// Bot_MoveNav's onLadder branch keys off the engine's own
			// movetype (MOVETYPE_FLY), set automatically once the bot's
			// hull touches the ladder's trigger volume, not off anything
			// stored on the waypoint.
			bool travelingFromA = (path.areaPath[i] == portal.areaA);
			wp.point = travelingFromA ? portal.pointB : portal.pointA;
		}
		else
		{
			wp.point = (portal.pointA + portal.pointB) * 0.5f;
		}
		wp.requiresJump = (portal.edgeType == crossbot_nav::EDGE_JUMP);
		wp.targetAreaId = static_cast<int>(path.areaPath[i + 1]);
		wp.requiresCrouch =
			AreaRequiresCrouch(mesh, static_cast<uint32_t>(path.areaPath[i])) ||
			AreaRequiresCrouch(mesh, wp.targetAreaId);
		// Doors and ladders are never simplified away (see IsHardAnchor):
		// a standing-hull sight-line trace between the rooms on either side
		// can easily read "clear" without actually having followed this
		// specific crossing (over/around a closed door, past a ladder shaft
		// through open space above it), the same class of false positive
		// EDGE_JUMP/EDGE_ELEVATOR/requiresCrouch already guard against.
		wp.requiresDoor = (portal.edgeType == crossbot_nav::EDGE_DOOR);
		wp.requiresLadder = (portal.edgeType == crossbot_nav::EDGE_LADDER);

		// Button-gated door (edge_tagger.cpp's ResolveButtonForDoor): expand
		// into a boarding-style pair, same as EDGE_ELEVATOR above -- walk to
		// the button first, then the door itself.
		if (portal.edgeType == crossbot_nav::EDGE_DOOR && (portal.edgeFlags & crossbot_nav::EDGE_FLAG_NEEDS_BUTTON))
		{
			Waypoint buttonWp;
			buttonWp.point = portal.buttonPos;
			buttonWp.requiresButtonUse = true;
			buttonWp.buttonWaitTime = portal.travelTime;
			buttonWp.targetAreaId = static_cast<int>(path.areaPath[i]); // still on the approach side
			raw.push_back(buttonWp);
		}

		raw.push_back(wp);
	}
	raw.push_back({goalPoint});

	if (raw.size() <= 2)
		return raw;

	std::vector<Waypoint> simplified;
	simplified.push_back(raw.front());

	size_t anchor = 0;
	while (anchor < raw.size() - 1)
	{
		if (IsHardAnchor(raw[anchor + 1]))
		{
			simplified.push_back(raw[anchor + 1]);
			anchor = anchor + 1;
			continue;
		}

		size_t furthest = anchor + 1;
		for (size_t candidate = anchor + 2; candidate < raw.size(); ++candidate)
		{
			if (IsHardAnchor(raw[candidate]))
				break; // don't skip over/through a required jump or elevator ride

			// Stop at the first candidate that fails, don't keep scanning
			// past it -- a real walkable path is a *contiguous* run of
			// locally-clear segments, so a blocked candidate followed by a
			// coincidentally-clear further one doesn't mean the further one
			// is actually reachable in a straight line; it means the trace
			// happened to sneak past some other opening (a window, a gap
			// between two separate rooms) that has nothing to do with the
			// real A*-computed corridor. Confirmed live on crossfire.bsp:
			// without this break, simplification collapsed a real multi-room
			// route into a single "clear" hull trace toward a point 172
			// units higher and in a different room -- re-tracing that exact
			// line afterward showed it blocked by a wall a quarter of the
			// way along, and the bot spent the rest of the run bouncing
			// around near the doorway that trace happened to skim past.
			if (!HasHullClearance(raw[anchor].point, raw[candidate].point, pIgnore))
				break;
			furthest = candidate;
		}
		simplified.push_back(raw[furthest]);
		anchor = furthest;
	}

	return simplified;
}

} // namespace Nav
