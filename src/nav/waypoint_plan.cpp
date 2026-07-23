#include "waypoint_plan.h"

#include <cmath>

#include "funnel_graph.h"

namespace NavPlan
{

namespace
{

float SegLen2D(const Vector &a, const Vector &b)
{
	float dx = b.x - a.x, dy = b.y - a.y;
	return std::sqrt(dx * dx + dy * dy);
}

} // namespace

std::vector<PlanWaypoint> BuildWaypointPlan(const std::vector<PlanArea> &areas,
	const std::vector<PlanPortal> &portals, const std::vector<uint32_t> &areaPath,
	const std::vector<uint32_t> &portalPath, const Vector &startPoint, const Vector &goalPoint)
{
	std::vector<PlanWaypoint> result;
	result.push_back({startPoint});

	// Runs of consecutive plain crossings (ordinary WALK portals, and DROP
	// portals -- walking off a ledge anywhere along its gate is equally
	// valid) accumulate as funnel gates; any special crossing is a fixed
	// point the string-pull must pass through exactly, so it flushes the
	// pending stretch first. anchorPoint tracks the last fixed point
	// emitted, i.e. where the next funnel stretch starts.
	Vector anchorPoint = startPoint;
	std::vector<NavFunnel::PortalGate> pendingGates;
	// Stretch bookkeeping for maxSpeed (see PlanWaypoint::maxSpeed):
	// narrowest unshrunk gate in the pending stretch, and whether any area
	// the stretch passes through borders a drop-off.
	float stretchMinGate = 1e9f;
	bool stretchNearDrop = false;

	auto StretchSpeed = [&]() -> float
	{
		if (!stretchNearDrop)
			return kPlanSpeedRun;
		return (stretchMinGate < kPlanNarrowGateLength) ? kPlanSpeedCrawl : kPlanSpeedCareful;
	};

	auto FlushFunnelTo = [&](const Vector &target)
	{
		float speed = StretchSpeed();
		std::vector<Vector> pulled = NavFunnel::StringPull(anchorPoint, target, pendingGates);
		// Interior corners only: pulled.front() is anchorPoint (already
		// emitted) and pulled.back() is `target` (the caller emits it with
		// whatever flags it carries).
		for (size_t k = 1; k + 1 < pulled.size(); ++k)
		{
			PlanWaypoint corner;
			corner.point = pulled[k];
			corner.maxSpeed = speed;
			result.push_back(corner);
		}
		pendingGates.clear();
		stretchMinGate = 1e9f;
		stretchNearDrop = false;
		return speed;
	};

	for (size_t i = 0; i < portalPath.size(); ++i)
	{
		const PlanPortal &portal = portals[portalPath[i]];
		const PlanArea &fromArea = areas[areaPath[i]];
		const PlanArea &toArea = areas[areaPath[i + 1]];

		if (portal.edgeType == crossbot_nav::EDGE_ELEVATOR)
		{
			// areaA is always the bottom landing (elevator_tagger.cpp's
			// convention) -- which area we're arriving *from* tells us
			// whether we're calling the lift up or just riding it back down.
			bool goingUp = (areaPath[i] == portal.areaA);

			if (goingUp)
			{
				// Walking onto the button's footprint touch-activates it
				// like any func_button, the same way a real player calls
				// the lift.
				float speed = FlushFunnelTo(portal.buttonPos);
				PlanWaypoint buttonWp;
				buttonWp.point = portal.buttonPos;
				buttonWp.maxSpeed = speed;
				result.push_back(buttonWp);
			}

			PlanWaypoint ride;
			ride.point = goingUp ? portal.pointA : portal.pointB;
			ride.requiresElevatorRide = true;
			ride.elevatorTravelTime = portal.travelTime;
			ride.maxSpeed = goingUp ? kPlanSpeedRun : FlushFunnelTo(ride.point);
			if (goingUp)
			{
				pendingGates.clear();
				stretchMinGate = 1e9f;
				stretchNearDrop = false;
			}
			result.push_back(ride);

			// Where riding actually leaves the bot -- a plain waypoint,
			// walked to normally once the hold-and-wait finishes.
			PlanWaypoint landing;
			landing.point = goingUp ? portal.pointB : portal.pointA;
			result.push_back(landing);
			anchorPoint = landing.point;
			continue;
		}

		PlanWaypoint wp;
		if (portal.edgeType == crossbot_nav::EDGE_JUMP || portal.edgeType == crossbot_nav::EDGE_LADDER)
		{
			// A jump portal's pointA/pointB are the two areas' own nearest
			// floor points (ProbeJumpGaps), a ladder's its bottom/top
			// anchors (TagLadder) -- not a shared-face gate either way. Aim
			// at the far side (wherever this crossing is actually headed),
			// not their midpoint, which would sit over the gap / in mid-air
			// halfway up the ladder. See path_funnel.h for the full
			// reasoning, including why a jump's real "reached" test is
			// area arrival (targetAreaId), not this point.
			bool travelingFromA = (areaPath[i] == portal.areaA);
			wp.point = travelingFromA ? portal.pointB : portal.pointA;
		}
		else
		{
			wp.point = (portal.pointA + portal.pointB) * 0.5f;
		}
		wp.requiresJump = (portal.edgeType == crossbot_nav::EDGE_JUMP);
		wp.targetAreaId = static_cast<int>(areaPath[i + 1]);
		wp.requiresCrouch = fromArea.crouch || toArea.crouch;
		wp.requiresLadder = (portal.edgeType == crossbot_nav::EDGE_LADDER);

		// Entering a door area (NAV_AREA_DOOR, nav_format.h v5): a fixed
		// crossing point -- the funnel must route through the (currently
		// closed, solid) door itself, not pull a taut line only
		// geometry-without-doors would permit. Button/trigger-gated doors
		// expand into a boarding-style pair first: activator, then door.
		if (toArea.door)
		{
			wp.requiresDoor = true;
			wp.doorWaitTime = toArea.doorWaitTime;
		}

		bool isSpecial = wp.requiresJump || wp.requiresLadder || wp.requiresCrouch || wp.requiresDoor;
		if (!isSpecial)
		{
			// Plain crossing -- just another funnel gate, no waypoint of
			// its own unless the string-pull decides a corner belongs here.
			Vector travelDir = toArea.centroid - fromArea.centroid;
			pendingGates.push_back(NavFunnel::MakeGate(portal.pointA, portal.pointB, travelDir));
			float gateLen = SegLen2D(portal.pointA, portal.pointB);
			if (gateLen < stretchMinGate)
				stretchMinGate = gateLen;
			if (fromArea.hasDropPortal || toArea.hasDropPortal)
				stretchNearDrop = true;
			continue;
		}

		if (toArea.door && toArea.doorActivation == crossbot_nav::DOOR_ACTIVATION_BUTTON)
		{
			PlanWaypoint buttonWp;
			buttonWp.point = toArea.doorActivatorPos;
			buttonWp.requiresButtonUse = true;
			buttonWp.buttonWaitTime = toArea.doorWaitTime;
			buttonWp.targetAreaId = static_cast<int>(areaPath[i]); // still on the approach side
			buttonWp.maxSpeed = FlushFunnelTo(buttonWp.point);
			result.push_back(buttonWp);
			anchorPoint = buttonWp.point;
		}
		else if (toArea.door && toArea.doorActivation == crossbot_nav::DOOR_ACTIVATION_TRIGGER)
		{
			PlanWaypoint triggerWp;
			triggerWp.point = toArea.doorActivatorPos;
			triggerWp.requiresTriggerTouch = true;
			triggerWp.targetAreaId = static_cast<int>(areaPath[i]); // still on the approach side
			triggerWp.maxSpeed = FlushFunnelTo(triggerWp.point);
			result.push_back(triggerWp);
			anchorPoint = triggerWp.point;
		}

		wp.maxSpeed = FlushFunnelTo(wp.point);
		result.push_back(wp);
		anchorPoint = wp.point;
	}

	{
		float speed = StretchSpeed();
		std::vector<Vector> pulled = NavFunnel::StringPull(anchorPoint, goalPoint, pendingGates);
		for (size_t k = 1; k + 1 < pulled.size(); ++k)
		{
			PlanWaypoint corner;
			corner.point = pulled[k];
			corner.maxSpeed = speed;
			result.push_back(corner);
		}
		PlanWaypoint goalWp;
		goalWp.point = goalPoint;
		goalWp.maxSpeed = speed;
		result.push_back(goalWp);
	}

	return result;
}

} // namespace NavPlan
