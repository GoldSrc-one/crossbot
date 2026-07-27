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

}

std::vector<PlanWaypoint> BuildWaypointPlan(const std::vector<PlanArea> &areas,
	const std::vector<PlanConnection> &conns, const std::vector<uint32_t> &areaPath,
	const std::vector<uint32_t> &connPath, const Vector &startPoint, const Vector &goalPoint)
{
	std::vector<PlanWaypoint> result;
	result.push_back({startPoint});

	Vector anchorPoint = startPoint;
	std::vector<NavFunnel::PortalGate> pendingGates;

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

	for (size_t i = 0; i < connPath.size(); ++i)
	{
		const PlanConnection &conn = conns[connPath[i]];
		const PlanArea &fromArea = areas[areaPath[i]];
		const PlanArea &toArea = areas[areaPath[i + 1]];

		if (conn.connType == crossbot_nav::CONN_TELEPORT)
		{

			bool enteringTrigger = (areaPath[i] == conn.areaA);
			if (!enteringTrigger)
			{

				enteringTrigger = true;
			}

			float speed = FlushFunnelTo(conn.pointA);
			PlanWaypoint triggerWp;
			triggerWp.point = conn.pointA;
			triggerWp.requiresTeleport = true;
			triggerWp.teleportDest = conn.pointB;
			triggerWp.maxSpeed = speed;
			result.push_back(triggerWp);

			PlanWaypoint destWp;
			destWp.point = conn.pointB;
			result.push_back(destWp);
			anchorPoint = destWp.point;
			continue;
		}

		PlanWaypoint wp;

		bool travelingFromA = (areaPath[i] == conn.areaA);
		const Vector &destPoint = travelingFromA ? conn.pointB : conn.pointA;
		const Vector &srcPoint = travelingFromA ? conn.pointA : conn.pointB;

		bool isSharedFaceWalk = false;
		if (conn.connType == crossbot_nav::CONN_WALK)
		{

			isSharedFaceWalk = (std::fabs(conn.pointA.x - conn.pointB.x) < 0.1f ||
				std::fabs(conn.pointA.y - conn.pointB.y) < 0.1f) &&
				std::fabs(conn.pointA.z - conn.pointB.z) <= 1.0f;
		}

		if (isSharedFaceWalk)
		{

			Vector travelDir = toArea.centroid - fromArea.centroid;
			pendingGates.push_back(NavFunnel::MakeGate(conn.pointA, conn.pointB, travelDir));
			float gateLen = SegLen2D(conn.pointA, conn.pointB);
			if (gateLen < stretchMinGate)
				stretchMinGate = gateLen;
			if (fromArea.hasDropPortal || toArea.hasDropPortal)
				stretchNearDrop = true;
			continue;
		}

		bool isCrouchCorridorWalk = false;
		if (conn.connType == crossbot_nav::CONN_WALK && fromArea.crouch && toArea.crouch)
		{
			bool portalHorizontal = std::fabs(conn.pointA.y - conn.pointB.y) < 5.0f;
			Vector travelDir = toArea.centroid - fromArea.centroid;
			bool travelHorizontal = std::fabs(travelDir.x) > std::fabs(travelDir.y);
			bool similarCentroidY = std::fabs(fromArea.centroid.y - toArea.centroid.y) < 50.0f;
			if (portalHorizontal && travelHorizontal && similarCentroidY)
				isCrouchCorridorWalk = true;
		}

		if (isCrouchCorridorWalk)
		{

			float cy = fromArea.centroid.y - 80.0f;
			wp.point = Vector((conn.pointA.x + conn.pointB.x) * 0.5f, cy, 0);
		}
		else
		{

			Vector travelDir = toArea.centroid - fromArea.centroid;
			bool horizontalCorridorPortal = false;
			float wallY = 0.0f;
			if (conn.connType == crossbot_nav::CONN_WALK)
			{

				bool portalVertical = std::fabs(conn.pointA.x - conn.pointB.x) < 1.0f;
				float portalHeight = std::fabs(conn.pointA.y - conn.pointB.y);
				bool travelHorizontal = std::fabs(travelDir.x) > std::fabs(travelDir.y);
				if (portalVertical && portalHeight > 150.0f && portalHeight < 250.0f && travelHorizontal)
				{

					float corridorCenterY = (fromArea.centroid.y + toArea.centroid.y) * 0.5f;
					float sideScore = 0.0f;
					if (i > 0)
					{
						const NavPlan::PlanArea &prevArea = areas[areaPath[i - 1]];
						sideScore += prevArea.centroid.y - corridorCenterY;
					}
					if (i + 2 < static_cast<int>(areaPath.size()))
					{
						const NavPlan::PlanArea &nextArea = areas[areaPath[i + 2]];
						sideScore += nextArea.centroid.y - corridorCenterY;
					}

					if (fromArea.crouch || toArea.crouch)
						wallY = corridorCenterY + 80.0f;
					else if (sideScore >= 0)
						wallY = corridorCenterY + 80.0f;
					else
						wallY = corridorCenterY - 80.0f;
					horizontalCorridorPortal = true;
				}

				else
				{
							bool portalHorizontal = std::fabs(conn.pointA.y - conn.pointB.y) < 5.0f;
					bool travelHorizontal = std::fabs(travelDir.x) > std::fabs(travelDir.y);

					bool similarCentroidY = std::fabs(fromArea.centroid.y - toArea.centroid.y) < 50.0f;
					if (portalHorizontal && travelHorizontal && similarCentroidY)
					{
						float corridorCenterY = (fromArea.centroid.y + toArea.centroid.y) * 0.5f;

						if (fromArea.crouch || toArea.crouch)
							wallY = corridorCenterY + 80.0f;
						else
						{
							float sideScore = 0.0f;
							if (i > 0)
							{
								const NavPlan::PlanArea &prevArea = areas[areaPath[i - 1]];
								sideScore += prevArea.centroid.y - corridorCenterY;
							}
							if (i + 2 < static_cast<int>(areaPath.size()))
							{
								const NavPlan::PlanArea &nextArea = areas[areaPath[i + 2]];
								sideScore += nextArea.centroid.y - corridorCenterY;
							}
							if (sideScore >= 0)
								wallY = corridorCenterY + 80.0f;
							else
								wallY = corridorCenterY - 80.0f;
						}
						horizontalCorridorPortal = true;
					}
				}
			}

			if (horizontalCorridorPortal)
			{
				wp.point = Vector((conn.pointA.x + conn.pointB.x) * 0.5f, wallY, 0);
			}
			else
			{
				wp.point = (conn.pointA + conn.pointB) * 0.5f;
			}
		}

		if (toArea.door)
		{
			wp.requiresDoor = true;
			wp.doorWaitTime = toArea.doorWaitTime;
		}

		wp.targetAreaId = static_cast<int>(areaPath[i + 1]);
		wp.requiresCrouch = fromArea.crouch || toArea.crouch;

		bool isSpecial = wp.requiresCrouch || wp.requiresDoor;
		if (!isSpecial)
		{

			Vector travelDir = toArea.centroid - fromArea.centroid;

			pendingGates.push_back(NavFunnel::MakeGate(conn.pointA, conn.pointB, travelDir));

			float gateLen = SegLen2D(conn.pointA, conn.pointB);
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
			buttonWp.targetAreaId = static_cast<int>(areaPath[i]);
			buttonWp.maxSpeed = FlushFunnelTo(buttonWp.point);
			result.push_back(buttonWp);
			anchorPoint = buttonWp.point;
		}
		else if (toArea.door && toArea.doorActivation == crossbot_nav::DOOR_ACTIVATION_TRIGGER)
		{
			PlanWaypoint triggerWp;
			triggerWp.point = toArea.doorActivatorPos;
			triggerWp.requiresTriggerTouch = true;
			triggerWp.targetAreaId = static_cast<int>(areaPath[i]);
			triggerWp.maxSpeed = FlushFunnelTo(triggerWp.point);
			result.push_back(triggerWp);
			anchorPoint = triggerWp.point;
		}

		wp.maxSpeed = FlushFunnelTo(wp.point);
		result.push_back(wp);

		anchorPoint = wp.point;
	}

	PlanWaypoint goalWp;
	goalWp.maxSpeed = FlushFunnelTo(goalPoint);
	goalWp.point = goalPoint;
	if (!areaPath.empty())
		goalWp.targetAreaId = static_cast<int>(areaPath.back());
	result.push_back(goalWp);

	return result;
}

}

