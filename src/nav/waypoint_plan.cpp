#include "waypoint_plan.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "funnel_graph.h"

namespace NavPlan
{

namespace
{

constexpr float kPi = 3.14159265f;

// A plane counts as a wall, rather than floor or ceiling, below this much vertical tilt. Only walls
// may limit a horizontal nudge: a corner sits on its own floor plane, so letting the floor take
// part would clamp every offset on a slope to nothing.
constexpr float kWallNormalZ = 0.3f;

// Tolerance for "still inside an area". Slightly negative: a corner slid along its gate stays on
// the face the two areas share, which is exactly where a crossing waypoint belongs, and a strictly
// positive margin would reject it from both sides at once.
constexpr float kInsideEpsilon = -0.01f;

// Matches AirmoveSampleParams::inset, so a replayed launch stands off the lip exactly as the
// sampler's did rather than straddling the boundary plane.
constexpr float kLaunchInset = 2.f;

constexpr float kAirmoveSimMaxTime = 6.f;

// Ignore the first moments of a jump: the mover is still standing on the launch ledge.
constexpr float kAirmoveSimMinTime = 0.1f;

float SegLen2D(const Vector &a, const Vector &b)
{
	float dx = b.x - a.x, dy = b.y - a.y;
	return std::sqrt(dx * dx + dy * dy);
}

// Unit length in the xy plane, zero if the input has no horizontal extent.
Vector Norm2D(const Vector &v)
{
	float len = std::sqrt(v.x * v.x + v.y * v.y);
	if (len < 0.0001f)
		return Vector(0.f, 0.f, 0.f);
	return Vector(v.x / len, v.y / len, 0.f);
}

float Yaw2D(const Vector &dir)
{
	return std::atan2(dir.y, dir.x) * (180.f / kPi);
}

bool PointInArea(const Vector &p, const PlanArea &area, float epsilon)
{
	if (area.boundPlanes.empty())
		return false;
	for (const PlanPlane &pl : area.boundPlanes)
		if (p.x * pl.normal.x + p.y * pl.normal.y + p.z * pl.normal.z - pl.dist < -epsilon)
			return false;
	return true;
}

// Whether p is inside the area's footprint, judged on walls alone. Floors and ceilings are left
// out so a point resting on a sloped floor still counts as inside its own column.
bool InAreaColumn(const Vector &p, const PlanArea &area, float margin)
{
	if (area.boundPlanes.empty())
		return false;
	for (const PlanPlane &pl : area.boundPlanes)
	{
		if (std::fabs(pl.normal.z) > kWallNormalZ)
			continue;
		if (p.x * pl.normal.x + p.y * pl.normal.y - pl.dist < margin)
			return false;
	}
	return true;
}

// How far p can travel along dir before leaving the area through one of its walls.
float MaxTravelInside(const Vector &p, const Vector &dir, const PlanArea &area)
{
	float best = 1e9f;
	for (const PlanPlane &pl : area.boundPlanes)
	{
		if (std::fabs(pl.normal.z) > kWallNormalZ)
			continue;

		float along = dir.x * pl.normal.x + dir.y * pl.normal.y;
		if (along >= -0.0001f)
			continue; // moving along or away from this wall, it cannot stop us

		float ahead = p.x * pl.normal.x + p.y * pl.normal.y + p.z * pl.normal.z - pl.dist;
		float allowed = ahead / -along;
		if (allowed < best)
			best = allowed;
	}
	return best;
}

// Floor height at p, read off the gate it came from: the two gate ends carry the floor height at
// their own corners, so a point along the gate interpolates between them.
float GateFloorZ(const NavFunnel::PortalGate &gate, const Vector &p)
{
	Vector span = gate.right - gate.left;
	float len2Sq = span.x * span.x + span.y * span.y;
	if (len2Sq < 0.0001f)
		return gate.left.z;

	float t = ((p.x - gate.left.x) * span.x + (p.y - gate.left.y) * span.y) / len2Sq;
	if (t < 0.f)
		t = 0.f;
	if (t > 1.f)
		t = 1.f;
	return gate.left.z + span.z * t;
}

// Ground distance to allow behind a launch so the bot arrives at speed. A tenth of the target
// speed: 32 units at a full run. Deliberately generous against the ~27 units the tick-by-tick
// acceleration actually needs, since arriving early at speed costs nothing and arriving slow
// wrecks the jump.
float GroundRunupDistance(float speed)
{
	return (speed > 0.f) ? speed * 0.1f : 0.f;
}

// Where the segment from -> to leaves the area, in 2D. False when it never does.
bool ExitPointFromArea(const Vector &from, const Vector &to, const PlanArea &area, Vector &out,
	int *whyNot = nullptr)
{
	Vector dir = Norm2D(to - from);
	if (dir.x == 0.f && dir.y == 0.f)
	{
		if (whyNot) *whyNot = 1; // degenerate direction
		return false;
	}

	float t = MaxTravelInside(from, dir, area);
	if (t >= 1e9f)
	{
		if (whyNot) *whyNot = 2; // no wall faces the ray
		return false;
	}
	if (t < 0.f)
	{
		if (whyNot) *whyNot = 3; // the approach point is already outside the area
		return false;
	}

	float span = SegLen2D(from, to);
	if (t > span)
	{
		if (whyNot) *whyNot = 4; // the landing is inside the source area's own column
		return false;
	}

	out = Vector(from.x + dir.x * t, from.y + dir.y * t, from.z);
	return true;
}

struct PendingGate
{
	NavFunnel::PortalGate gate;
	// The area this gate was pushed into. Each portal contributes one gate on each side, so a
	// corner's height and crouch state come from the area it actually stands in.
	uint32_t insideArea = 0;
	uint32_t toArea = 0;
	bool crouch = false;
};

}

std::vector<PlanWaypoint> BuildWaypointPlan(const std::vector<PlanArea> &areas,
	const std::vector<PlanConnection> &conns, const std::vector<uint32_t> &areaPath,
	const std::vector<uint32_t> &connPath, const Vector &startPoint, const Vector &goalPoint,
	const PlanTraceContext *traces, PlanStats *stats)
{
	std::vector<PlanWaypoint> result;
	result.push_back({startPoint});

	Vector anchorPoint = startPoint;
	std::vector<PendingGate> pendingGates;

	float stretchMinGate = 1e9f;
	bool stretchNearDrop = false;

	// A point standing on the floor of this area, at the height the bot's origin will be.
	auto AtPlayerHeight = [&](const Vector &floorPoint, uint32_t areaId)
	{
		bool crouch = (areaId < areas.size()) && areas[areaId].crouch;
		return Vector(floorPoint.x, floorPoint.y, floorPoint.z + (crouch ? 0.f : kPlanStandHeight));
	};

	auto StretchSpeed = [&]() -> float
	{
		if (!stretchNearDrop)
			return kPlanSpeedRun;
		return (stretchMinGate < kPlanNarrowGateLength) ? kPlanSpeedCrawl : kPlanSpeedCareful;
	};

	// Each portal becomes two gates: the portal segment pulled in from both ends by the player
	// radius, then pushed that far off the portal plane into the area on either side. Funnelling
	// through those instead of through the raw portal is what keeps the route off the walls -- a
	// corner can then only land inside an area, never on the plane it shares with a wall.
	auto PushGate = [&](const PlanConnection &conn, uint32_t fromId, uint32_t toId)
	{
		const PlanArea &fromArea = areas[fromId];
		const PlanArea &toArea = areas[toId];

		NavFunnel::PortalGate base =
			NavFunnel::MakeGate(conn.pointA, conn.pointB, toArea.centroid - fromArea.centroid);

		float gateLen = SegLen2D(base.left, base.right);
		if (gateLen < stretchMinGate)
			stretchMinGate = gateLen;
		if (fromArea.hasDropPortal || toArea.hasDropPortal)
			stretchNearDrop = true;

		Vector along = Norm2D(base.right - base.left);
		Vector normal(-along.y, along.x, 0.f);
		Vector mid = (base.left + base.right) * 0.5f;
		if ((toArea.centroid.x - mid.x) * normal.x + (toArea.centroid.y - mid.y) * normal.y < 0.f)
			normal = Vector(-normal.x, -normal.y, 0.f);

		auto onGate = [&](const Vector &p)
		{
			return Vector(p.x, p.y, GateFloorZ(base, p));
		};
		auto shift = [&](const Vector &p, const Vector &dir, float d)
		{
			return Vector(p.x + dir.x * d, p.y + dir.y * d, p.z);
		};

		auto build = [&](const Vector &dir, uint32_t insideId)
		{
			const PlanArea &inside = areas[insideId];

			PendingGate pg;
			pg.insideArea = insideId;
			pg.toArea = toId;
			pg.crouch = fromArea.crouch || toArea.crouch;

			Vector l = onGate(shift(base.left, along, kPlanClearance));
			Vector r = onGate(shift(base.right, along, -kPlanClearance));

			bool usable = gateLen > 2.f * kPlanClearance &&
				InAreaColumn(l, inside, kInsideEpsilon) && InAreaColumn(r, inside, kInsideEpsilon);

			if (!usable)
			{
				// Nothing survives the inset, so aim at the middle of the opening. Push it off the
				// plane if that still lands inside, otherwise leave it on the boundary.
				Vector c = onGate(mid);
				Vector pushed = shift(c, dir, kPlanClearance);
				pg.gate.left = pg.gate.right = InAreaColumn(pushed, inside, kInsideEpsilon) ? pushed : c;
				if (stats)
					++stats->gatesClamped;
			}
			else
			{
				Vector pl = shift(l, dir, kPlanClearance);
				Vector pr = shift(r, dir, kPlanClearance);
				bool fits = InAreaColumn(pl, inside, kInsideEpsilon) &&
					InAreaColumn(pr, inside, kInsideEpsilon);
				pg.gate.left = fits ? pl : l;
				pg.gate.right = fits ? pr : r;
				if (!fits && stats)
					++stats->gatesClamped;
			}

			pendingGates.push_back(pg);
		};

		// Leaving side first, then the entered side, so they are crossed in path order.
		build(Vector(-normal.x, -normal.y, 0.f), fromId);
		build(normal, toId);
	};

	// The last corner the most recent flush emitted, in raw pulled coordinates. An air move needs it
	// to know which way the path is heading as it leaves the area.
	Vector lastFlushCorner;
	bool haveFlushCorner = false;

	auto FlushFunnelTo = [&](const Vector &target)
	{
		float speed = StretchSpeed();
		haveFlushCorner = false;

		std::vector<NavFunnel::PortalGate> gates;
		gates.reserve(pendingGates.size());
		for (const PendingGate &pg : pendingGates)
			gates.push_back(pg.gate);

		std::vector<NavFunnel::PulledPoint> pulled = NavFunnel::StringPull(anchorPoint, target, gates);

		for (size_t k = 1; k + 1 < pulled.size(); ++k)
		{
			lastFlushCorner = pulled[k].point;
			haveFlushCorner = true;

			PlanWaypoint corner;
			corner.maxSpeed = speed;
			corner.point = pulled[k].point;

			if (pulled[k].gateIndex < pendingGates.size())
			{
				const PendingGate &pg = pendingGates[pulled[k].gateIndex];
				Vector p = pulled[k].point;
				p.z = GateFloorZ(pg.gate, p);
				corner.point = AtPlayerHeight(p, pg.insideArea);
				corner.requiresCrouch = pg.crouch;
				corner.targetAreaId = static_cast<int>(pg.toArea);
			}

			if (stats)
				++stats->corners;
			result.push_back(corner);
		}

		pendingGates.clear();
		stretchMinGate = 1e9f;
		stretchNearDrop = false;
		return speed;
	};

	// Replays an air move from a candidate launch point and reports whether it still comes down
	// inside the destination area.
	// endOut receives where the replay actually came down, whether or not that was the right area,
	// so a failed attempt can be drawn.
	auto JumpLandsIn = [&](const Vector &launchFloor, float yaw, float speed, bool jump,
		uint32_t destAreaId, bool needsCrouch, Vector &endOut, const char **whyOut = nullptr) -> bool
	{
		endOut = launchFloor;
		if (whyOut)
			*whyOut = "no traces";

		if (!traces)
			return false;

		const PmSim::PmTraceFn &trace = needsCrouch ? traces->duckTrace : traces->standTrace;
		if (!trace)
			return false;

		// The stand hull's origin sits kPlanStandHeight above a duck-hull floor point, and every
		// position it reports is offset by the same amount.
		float zAdjust = needsCrouch ? 0.f : kPlanStandHeight;

		PmSim::PmPhysics physics = traces->physics;
		PmSim::PmSimulator sim(trace, physics);
		sim.Begin(Vector(launchFloor.x, launchFloor.y, launchFloor.z + zAdjust), yaw, speed, jump);

		auto record = [&]()
		{
			const Vector &p = sim.Origin();
			endOut = Vector(p.x, p.y, p.z - zAdjust);
		};

		while (sim.Time() < kAirmoveSimMaxTime)
		{
			if (!sim.Step())
			{
				record();
				if (whyOut) *whyOut = "wedged";
				return false;
			}

			if (!sim.OnGround())
				continue;

			if (sim.Time() < kAirmoveSimMinTime)
			{
				sim.ResetFallSpeed();
				continue;
			}

			record();
			bool in = PointInArea(endOut, areas[destAreaId], 1.f);
			if (whyOut) *whyOut = in ? "landed in area" : "landed elsewhere";
			return in;
		}

		record();
		if (whyOut) *whyOut = "timed out";
		return false;
	};

	for (size_t i = 0; i < connPath.size(); ++i)
	{
		const PlanConnection &conn = conns[connPath[i]];
		uint32_t fromId = areaPath[i];
		uint32_t toId = areaPath[i + 1];
		const PlanArea &fromArea = areas[fromId];
		const PlanArea &toArea = areas[toId];

		if (conn.connType == crossbot_nav::CONN_TELEPORT)
		{
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

		if (conn.connType == crossbot_nav::CONN_AIRMOVE)
		{
			// Air moves are one-way, so the sampled launch is always pointA.
			bool needsCrouch = (conn.connFlags & crossbot_nav::CONN_FLAG_NEEDS_CROUCH) != 0;
			bool jump = (conn.connFlags & crossbot_nav::CONN_FLAG_JUMP) != 0;

			// One pull, aimed at the landing. Its corners are the approach and get emitted here; the
			// launch is then just where the final segment leaves the area, so the heading we jump on
			// is the heading the path already had.
			float speed = FlushFunnelTo(AtPlayerHeight(conn.pointB, toId));
			Vector approachFrom = haveFlushCorner ? lastFlushCorner : anchorPoint;

			Vector candidate = conn.pointA;
			int whyNot = 0;
			if (ExitPointFromArea(approachFrom, conn.pointB, fromArea, candidate, &whyNot))
			{
				// The crossing sits on the boundary plane itself. A drop wants to be right at the lip,
				// but a jump has to start its run-up behind it -- and when the far side is a wall
				// rather than a drop, standing on the plane buries the hull in it. Stand off by the
				// same amount the sampler did: its launches are inset + jumpBack behind the edge.
				float standOff = jump ? (kLaunchInset + conn.jumpBackDist) : 0.f;
				Vector back = Norm2D(conn.pointB - candidate);
				Vector pulled(candidate.x - back.x * standOff, candidate.y - back.y * standOff,
					conn.pointA.z);

				candidate = InAreaColumn(pulled, fromArea, kInsideEpsilon)
					? pulled
					: Vector(candidate.x, candidate.y, conn.pointA.z);
			}
			else if (whyNot == 4)
			{
				// The landing is under the area we are on, so there is no boundary to cross: walk to
				// directly above it and step off.
				Vector overhead(conn.pointB.x, conn.pointB.y, conn.pointA.z);
				candidate = InAreaColumn(overhead, fromArea, kInsideEpsilon) ? overhead : conn.pointA;
			}
			else
			{
				// No approach direction to speak of -- the path already ends on the landing. The baked
				// launch is the only opinion available.
				candidate = conn.pointA;
			}

			if (whyNot != 0 && std::getenv("NAVGEN_DUMP_AIRMOVE"))
				std::printf("    airmove %u->%u: no funnel launch, reason %d (approach %.1f,%.1f  "
					"landing %.1f,%.1f)\n", fromId, toId, whyNot, approachFrom.x, approachFrom.y,
					conn.pointB.x, conn.pointB.y);

			Vector launch = conn.pointA;
			float yaw = conn.exitAngle;
			bool usedSim = false;
			bool simTried = false;
			Vector simFrom = candidate, simTo = candidate;

			if (traces)
			{
				// The path heading first: that is the direction the bot genuinely arrives on, being
				// the segment it just walked. The baked angle is the fallback for when the launch has
				// not moved far enough for the heading to matter.
				Vector pathDir = Norm2D(conn.pointB - candidate);
				float yawPath = (pathDir.x != 0.f || pathDir.y != 0.f) ? Yaw2D(pathDir) : conn.exitAngle;

				// Path heading first, then the baked angle. The baked one was measured from the baked
				// launch so it aims at nothing in particular once the launch has moved -- but the
				// simulation is the arbiter either way, so a hit is a hit and it costs one replay.
				const float candYaws[2] = { yawPath, conn.exitAngle };

				simTried = true;
				for (int yi = 0; yi < 2; ++yi)
				{
					float candYaw = candYaws[yi];
					Vector end;
					const char *why = "";
					bool ok = JumpLandsIn(candidate, candYaw, conn.exitSpeed, jump, toId, needsCrouch,
						end, &why);
					// Only the first attempt is worth drawing: it is the one aimed along the path.
					if (yi == 0)
						simTo = end;
					if (!ok && std::getenv("NAVGEN_DUMP_AIRMOVE"))
						std::printf("    airmove %u->%u sim miss: %s  launch (%.1f,%.1f,%.1f) yaw %.0f "
							"speed %.0f jump %d -> end (%.1f,%.1f,%.1f)\n", fromId, toId, why,
							candidate.x, candidate.y, candidate.z, candYaw, conn.exitSpeed, jump ? 1 : 0,
							end.x, end.y, end.z);
					if (!ok)
						continue;
					launch = candidate;
					yaw = candYaw;
					usedSim = true;
					break;
				}
			}

			if (stats)
			{
				if (usedSim)
					++stats->airmovesSimulated;
				else
					++stats->airmovesFromStored;
			}

			// Only worth a run-up node when the approach is too short to reach launch speed: with the
			// heading taken from the path, a longer approach already arrives pointing the right way.
			float runupDist = GroundRunupDistance(conn.exitSpeed);
			if (SegLen2D(approachFrom, launch) < runupDist)
			{
				float yawRad = yaw * (kPi / 180.f);
				Vector runup(launch.x - std::cos(yawRad) * runupDist,
					launch.y - std::sin(yawRad) * runupDist, launch.z);

				if (InAreaColumn(runup, fromArea, kInsideEpsilon))
				{
					PlanWaypoint runupWp;
					runupWp.point = AtPlayerHeight(runup, fromId);
					runupWp.maxSpeed = speed;
					runupWp.requiresCrouch = needsCrouch;
					runupWp.targetAreaId = static_cast<int>(fromId);
					result.push_back(runupWp);

					if (stats)
						++stats->airmoveRunups;
				}
			}

			PlanWaypoint launchWp;
			launchWp.point = AtPlayerHeight(launch, fromId);
			launchWp.airmoveLaunch = true;
			launchWp.requiresJump = jump;
			launchWp.launchSpeed = conn.exitSpeed;
			launchWp.launchYaw = yaw;
			launchWp.jumpBackDist = conn.jumpBackDist;
			launchWp.launchSimulated = usedSim;
			launchWp.launchSimFailed = simTried && !usedSim;
			launchWp.launchSimFrom = simFrom;
			launchWp.launchSimTo = simTo;
			launchWp.requiresCrouch = needsCrouch;
			launchWp.maxSpeed = speed;
			launchWp.targetAreaId = static_cast<int>(toId);
			result.push_back(launchWp);

			PlanWaypoint landWp;
			landWp.point = AtPlayerHeight(conn.pointB, toId);
			landWp.targetAreaId = static_cast<int>(toId);
			result.push_back(landWp);

			anchorPoint = landWp.point;
			continue;
		}

		// Everything else is a walk through a shared face. It goes in the funnel, and any corner
		// it produces gets its clearance from ResolveCorner rather than from the connection's
		// orientation.
		bool needsStop = toArea.door;
		if (!needsStop)
		{
			PushGate(conn, fromId, toId);
			continue;
		}

		PlanWaypoint wp;
		wp.point = AtPlayerHeight((conn.pointA + conn.pointB) * 0.5f, toId);
		wp.requiresDoor = true;
		wp.doorWaitTime = toArea.doorWaitTime;
		wp.targetAreaId = static_cast<int>(toId);
		wp.requiresCrouch = fromArea.crouch || toArea.crouch;

		if (toArea.doorActivation == crossbot_nav::DOOR_ACTIVATION_BUTTON)
		{
			PlanWaypoint buttonWp;
			buttonWp.point = toArea.doorActivatorPos;
			buttonWp.requiresButtonUse = true;
			buttonWp.buttonWaitTime = toArea.doorWaitTime;
			buttonWp.targetAreaId = static_cast<int>(fromId);
			buttonWp.maxSpeed = FlushFunnelTo(buttonWp.point);
			result.push_back(buttonWp);
			anchorPoint = buttonWp.point;
		}
		else if (toArea.doorActivation == crossbot_nav::DOOR_ACTIVATION_TRIGGER)
		{
			PlanWaypoint triggerWp;
			triggerWp.point = toArea.doorActivatorPos;
			triggerWp.requiresTriggerTouch = true;
			triggerWp.targetAreaId = static_cast<int>(fromId);
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
