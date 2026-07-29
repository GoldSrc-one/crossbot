#include "waypoint_plan.h"

#include <cmath>
#include <cstdio>

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

// Floor height at p, read off the gate it came from: the two gate ends carry the floor height at
// their own corners, so a point along the gate interpolates between them.
float GateFloorZ(const NavFunnel::Gate &gate, const Vector &p)
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

// Where a route crosses a gate. Falls back to the gate's middle when the route misses it
// altogether, which only happens if the funnel had no reason to steer through it.
Vector CrossingOnGate(const std::vector<Vector> &line, const NavFunnel::Gate &gate)
{
	Vector mid = (gate.left + gate.right) * 0.5f;

	Vector span = gate.right - gate.left;
	Vector normal(-span.y, span.x, 0.f);
	float nlen = std::sqrt(normal.x * normal.x + normal.y * normal.y);
	if (nlen < 0.0001f || line.size() < 2)
		return mid;
	normal = Vector(normal.x / nlen, normal.y / nlen, 0.f);

	float gateAt = gate.left.x * normal.x + gate.left.y * normal.y;
	float spanLenSq = span.x * span.x + span.y * span.y;

	for (size_t i = 0; i + 1 < line.size(); ++i)
	{
		float da = line[i].x * normal.x + line[i].y * normal.y - gateAt;
		float db = line[i + 1].x * normal.x + line[i + 1].y * normal.y - gateAt;
		if ((da > 0.f) == (db > 0.f) || std::fabs(da - db) < 0.0001f)
			continue;

		float t = da / (da - db);
		Vector hit = line[i] + (line[i + 1] - line[i]) * t;

		float s = ((hit.x - gate.left.x) * span.x + (hit.y - gate.left.y) * span.y) / spanLenSq;
		s = (s < 0.f) ? 0.f : (s > 1.f ? 1.f : s);
		return gate.left + span * s;
	}

	return mid;
}

// Where p sits along the gate, 0 at left and 1 at right. Measured against the stored gate rather
// than the funnel's reordered copy, since the per-end parameters were recorded in stored order.
float GateFraction(const PlanGate &gate, const Vector &p)
{
	Vector span = gate.right - gate.left;
	float lenSq = span.x * span.x + span.y * span.y;
	if (lenSq < 0.0001f)
		return 0.f;

	float t = ((p.x - gate.left.x) * span.x + (p.y - gate.left.y) * span.y) / lenSq;
	return (t < 0.f) ? 0.f : (t > 1.f ? 1.f : t);
}

// Shortest way round, so a gate whose ends straddle +/-180 does not sweep the long way.
float LerpYaw(float a, float b, float t)
{
	float d = b - a;
	while (d > 180.f)
		d -= 360.f;
	while (d < -180.f)
		d += 360.f;
	return a + d * t;
}

struct PendingGate
{
	NavFunnel::Gate gate;
	// The area this gate was pushed into. Each connection contributes one gate on each side, so a
	// corner's height and crouch state come from the area it actually stands in.
	uint32_t insideArea = 0;
	uint32_t toArea = 0;
	bool crouch = false;
};

const PlanPlane *FloorPlaneOf(const PlanArea &area)
{
	const PlanPlane *best = nullptr;
	for (const PlanPlane &pl : area.boundPlanes)
		if (!best || pl.normal.z > best->normal.z)
			best = &pl;
	return (best && best->normal.z > kWallNormalZ) ? best : nullptr;
}

float AreaFloorZ(const PlanArea &area, float x, float y)
{
	const PlanPlane *floor = FloorPlaneOf(area);
	if (!floor || floor->normal.z < 0.0001f)
		return 0.f;
	return (floor->dist - floor->normal.x * x - floor->normal.y * y) / floor->normal.z;
}

// The line one bound plane carries, trimmed to the span that is actually open. Trimming by the
// owning area alone gives that area's whole edge; trimming by the neighbour as well gives the two
// areas' shared face, which is the only part a path may cross.
struct EdgeLine
{
	Vector inward = Vector(0.f, 0.f, 0.f);
	Vector along = Vector(0.f, 0.f, 0.f);
	Vector base = Vector(0.f, 0.f, 0.f);
	float lo = -1e9f;
	float hi = 1e9f;
	bool valid = false;
};

bool ClipLineByWalls(EdgeLine &line, const PlanArea &area, const PlanPlane *skip)
{
	for (const PlanPlane &other : area.boundPlanes)
	{
		if (&other == skip)
			continue;
		if (std::fabs(other.normal.z) > kWallNormalZ)
			continue;

		float denom = other.normal.x * line.along.x + other.normal.y * line.along.y;
		float at = line.base.x * other.normal.x + line.base.y * other.normal.y - other.dist;

		if (std::fabs(denom) < 0.0001f)
		{
			if (at < -0.05f)
				return false;
			continue;
		}

		float s = -at / denom;
		if (denom > 0.f)
		{
			if (s > line.lo)
				line.lo = s;
		}
		else if (s < line.hi)
		{
			line.hi = s;
		}
	}

	return (line.hi - line.lo) > 0.0001f;
}

EdgeLine EdgeOfPlane(const PlanArea &area, size_t planeIdx)
{
	EdgeLine line;
	if (planeIdx >= area.boundPlanes.size())
		return line;

	const PlanPlane &pl = area.boundPlanes[planeIdx];
	if (std::fabs(pl.normal.z) > kWallNormalZ)
		return line;

	float nlen = std::sqrt(pl.normal.x * pl.normal.x + pl.normal.y * pl.normal.y);
	if (nlen < 0.0001f)
		return line;

	line.inward = Vector(pl.normal.x / nlen, pl.normal.y / nlen, 0.f);
	float dist = pl.dist / nlen;
	line.along = Vector(-line.inward.y, line.inward.x, 0.f);
	line.base = Vector(line.inward.x * dist, line.inward.y * dist, 0.f);
	line.valid = ClipLineByWalls(line, area, &pl) && line.lo > -1e8f && line.hi < 1e8f;

	return line;
}

PlanGate GateFromEdge(const EdgeLine &line, const PlanArea &inside, const Vector &inward)
{
	PlanGate gate;
	if (!line.valid)
		return gate;

	auto shift = [](const Vector &p, const Vector &dir, float d)
	{
		return Vector(p.x + dir.x * d, p.y + dir.y * d, p.z);
	};
	auto onFloor = [&](const Vector &p)
	{
		return Vector(p.x, p.y, AreaFloorZ(inside, p.x, p.y));
	};

	Vector e1(line.base.x + line.along.x * line.lo, line.base.y + line.along.y * line.lo, 0.f);
	Vector e2(line.base.x + line.along.x * line.hi, line.base.y + line.along.y * line.hi, 0.f);

	gate.valid = true;

	Vector l = shift(e1, line.along, kPlanClearance);
	Vector r = shift(e2, line.along, -kPlanClearance);

	bool usable = (line.hi - line.lo) > 2.f * kPlanClearance &&
		InAreaColumn(l, inside, kInsideEpsilon) && InAreaColumn(r, inside, kInsideEpsilon);

	if (!usable)
	{
		Vector c((e1.x + e2.x) * 0.5f, (e1.y + e2.y) * 0.5f, 0.f);
		Vector pushed = shift(c, inward, kPlanClearance);
		gate.left = gate.right = onFloor(InAreaColumn(pushed, inside, kInsideEpsilon) ? pushed : c);
		gate.clamped = true;
		return gate;
	}

	Vector pushedL = shift(l, inward, kPlanClearance);
	Vector pushedR = shift(r, inward, kPlanClearance);
	bool fits = InAreaColumn(pushedL, inside, kInsideEpsilon) &&
		InAreaColumn(pushedR, inside, kInsideEpsilon);

	gate.left = onFloor(fits ? pushedL : l);
	gate.right = onFloor(fits ? pushedR : r);
	gate.clamped = !fits;
	return gate;
}

}

void BuildAreaGates(PlanArea &area)
{
	area.edgeGates.assign(area.boundPlanes.size(), PlanGate());

	for (size_t i = 0; i < area.boundPlanes.size(); ++i)
	{
		EdgeLine line = EdgeOfPlane(area, i);
		if (line.valid)
			area.edgeGates[i] = GateFromEdge(line, area, line.inward);
	}
}

void BuildConnectionGates(const std::vector<PlanArea> &areas, std::vector<PlanConnection> &conns)
{
	for (PlanConnection &conn : conns)
	{
		conn.gateA = PlanGate();
		conn.gateB = PlanGate();

		if (conn.connType == crossbot_nav::CONN_TELEPORT)
			continue;
		if (conn.areaA >= areas.size() || conn.areaB >= areas.size())
			continue;

		const PlanArea &a = areas[conn.areaA];
		const PlanArea &b = areas[conn.areaB];

		// An air move's gates are measured off the samples that proved it, so there is nothing
		// here to derive.
		if (conn.connType == crossbot_nav::CONN_AIRMOVE)
			continue;

		EdgeLine shared = EdgeOfPlane(a, conn.planeA);
		if (shared.valid && ClipLineByWalls(shared, b, nullptr))
		{
			conn.gateA = GateFromEdge(shared, a, shared.inward);
			conn.gateB = GateFromEdge(shared, b,
				Vector(-shared.inward.x, -shared.inward.y, 0.f));
			continue;
		}

		if (conn.planeA < a.edgeGates.size())
			conn.gateA = a.edgeGates[conn.planeA];
		if (conn.planeB < b.edgeGates.size())
			conn.gateB = b.edgeGates[conn.planeB];
	}
}

Vector ConnectionCrossing(const std::vector<PlanArea> &areas, const PlanConnection &conn)
{
	if (conn.connType != crossbot_nav::CONN_WALK)
		return conn.launchPos;

	Vector sum(0.f, 0.f, 0.f);
	int found = 0;

	auto add = [&](const PlanGate &g)
	{
		if (!g.valid)
			return;
		sum = sum + (g.left + g.right) * 0.5f;
		++found;
	};

	add(conn.gateA);
	add(conn.gateB);

	if (found > 0)
		return sum * (1.f / static_cast<float>(found));

	if (conn.areaA < areas.size() && conn.areaB < areas.size())
		return (areas[conn.areaA].centroid + areas[conn.areaB].centroid) * 0.5f;
	return sum;
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

	bool stretchNearDrop = false;

	// A point standing on the floor of this area, at the height the bot's origin will be.
	auto AtPlayerHeight = [&](const Vector &floorPoint, uint32_t areaId)
	{
		bool crouch = (areaId < areas.size()) && areas[areaId].crouch;
		return Vector(floorPoint.x, floorPoint.y, floorPoint.z + (crouch ? 0.f : kPlanStandHeight));
	};

	auto StretchSpeed = [&]() -> float
	{
		return stretchNearDrop ? kPlanSpeedCareful : kPlanSpeedRun;
	};

	// The gate an area carries on the face this connection crosses, already inset and pushed
	// inward when the area was loaded.
	auto GateFor = [&](const PlanConnection &conn, uint32_t insideId) -> const PlanGate *
	{
		const PlanGate &g = (conn.areaA == insideId) ? conn.gateA : conn.gateB;
		return g.valid ? &g : nullptr;
	};

	// One gate on each side of the crossing, leaving side first so they are met in path order.
	// Funnelling through those rather than the shared face itself keeps the route off the walls.
	auto PushGate = [&](const PlanConnection &conn, uint32_t fromId, uint32_t toId)
	{
		const PlanArea &fromArea = areas[fromId];
		const PlanArea &toArea = areas[toId];
		Vector travel = toArea.centroid - fromArea.centroid;

		if (fromArea.hasDropConnection || toArea.hasDropConnection)
			stretchNearDrop = true;

		auto build = [&](uint32_t insideId)
		{
			const PlanGate *g = GateFor(conn, insideId);
			if (!g)
				return;

			if (g->clamped && stats)
				++stats->gatesClamped;

			PendingGate pg;
			pg.insideArea = insideId;
			pg.toArea = toId;
			pg.crouch = fromArea.crouch || toArea.crouch;
			pg.gate = NavFunnel::MakeGate(g->left, g->right, travel);
			pendingGates.push_back(pg);
		};

		build(fromId);
		build(toId);
	};

	// Runs the funnel without consuming it, so an air move can see where the route would cross its
	// gates before committing to a launch.
	auto PeekFunnel = [&](const Vector &target)
	{
		std::vector<NavFunnel::Gate> gates;
		gates.reserve(pendingGates.size());
		for (const PendingGate &pg : pendingGates)
			gates.push_back(pg.gate);

		std::vector<NavFunnel::PulledPoint> pulled = NavFunnel::StringPull(anchorPoint, target, gates);

		std::vector<Vector> line;
		line.reserve(pulled.size());
		for (const NavFunnel::PulledPoint &p : pulled)
			line.push_back(p.point);
		return line;
	};

	auto FlushFunnelTo = [&](const Vector &target)
	{
		float speed = StretchSpeed();

		std::vector<NavFunnel::Gate> gates;
		gates.reserve(pendingGates.size());
		for (const PendingGate &pg : pendingGates)
			gates.push_back(pg.gate);

		std::vector<NavFunnel::PulledPoint> pulled = NavFunnel::StringPull(anchorPoint, target, gates);

		for (size_t k = 1; k + 1 < pulled.size(); ++k)
		{
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
			float speed = FlushFunnelTo(conn.launchPos);

			PlanWaypoint triggerWp;
			triggerWp.point = conn.launchPos;
			triggerWp.requiresTeleport = true;
			triggerWp.teleportDest = conn.landingPos;
			triggerWp.maxSpeed = speed;
			result.push_back(triggerWp);

			PlanWaypoint destWp;
			destWp.point = conn.landingPos;
			result.push_back(destWp);
			anchorPoint = destWp.point;
			continue;
		}

		if (conn.connType == crossbot_nav::CONN_AIRMOVE)
		{
			bool needsCrouch = (conn.connFlags & crossbot_nav::CONN_FLAG_NEEDS_CROUCH) != 0;
			bool jump = (conn.connFlags & crossbot_nav::CONN_FLAG_JUMP) != 0;

			// The air move goes through the funnel like any other crossing: its gates say where the
			// route should leave and where it should come down. Only if replaying that launch
			// misses does the baked one get used instead.
			size_t gatesBefore = pendingGates.size();
			PushGate(conn, fromId, toId);

			Vector launch = conn.launchPos;
			Vector land = conn.landingPos;
			float yaw = conn.exitAngle;
			float launchSpeed = conn.exitSpeed;
			bool usedSim = false;
			Vector simFrom = launch, simTo = land, simAim = land;

			if (traces && pendingGates.size() > gatesBefore)
			{
				std::vector<Vector> line = PeekFunnel(AtPlayerHeight(conn.landingPos, toId));

				Vector exitPoint = CrossingOnGate(line, pendingGates[gatesBefore].gate);
				Vector entryPoint = (pendingGates.size() > gatesBefore + 1)
					? CrossingOnGate(line, pendingGates[gatesBefore + 1].gate)
					: AtPlayerHeight(conn.landingPos, toId);

				// The samples that set each end of the gate flew their own way, so read the launch
				// off the gate at the point the route actually crosses it.
				float t = GateFraction(conn.gateA, exitPoint);
				float candYaw = LerpYaw(conn.gateAYaw[0], conn.gateAYaw[1], t);
				float candSpeed = conn.gateASpeed[0] + (conn.gateASpeed[1] - conn.gateASpeed[0]) * t;
				if (candSpeed <= 0.f)
					candSpeed = conn.exitSpeed;

				Vector end;
				const char *why = "";
				simFrom = exitPoint;
				simAim = entryPoint;
				usedSim = JumpLandsIn(exitPoint, candYaw, candSpeed, jump, toId, needsCrouch,
					end, &why);
				simTo = end;

				if (usedSim)
				{
					launch = exitPoint;
					land = end;
					yaw = candYaw;
					launchSpeed = candSpeed;
				}
				else if (traces->dumpAirmoves)
				{
					std::printf("    airmove %u->%u sim miss: %s  t %.2f launch (%.1f,%.1f,%.1f) "
						"yaw %.0f speed %.0f jump %d -> end (%.1f,%.1f,%.1f)  entry (%.1f,%.1f)\n",
						fromId, toId, why, t, exitPoint.x, exitPoint.y, exitPoint.z, candYaw,
						candSpeed, jump ? 1 : 0, end.x, end.y, end.z, entryPoint.x, entryPoint.y);
				}
			}

			// The gates have said their piece. The flush target is the launch itself, so carrying
			// them any further would only constrain the route past where it leaves the ground.
			pendingGates.resize(gatesBefore);

			if (stats)
			{
				if (usedSim)
					++stats->airmovesSimulated;
				else
					++stats->airmovesFromStored;
			}

			float speed = FlushFunnelTo(AtPlayerHeight(launch, fromId));

			PlanWaypoint launchWp;
			launchWp.point = AtPlayerHeight(launch, fromId);
			launchWp.airmoveLaunch = true;
			launchWp.requiresJump = jump;
			launchWp.launchSpeed = launchSpeed;
			launchWp.launchYaw = yaw;
			launchWp.jumpBackDist = conn.jumpBackDist;
			launchWp.launchSimulated = usedSim;
			launchWp.launchSimFailed = !usedSim;
			launchWp.launchSimFrom = simFrom;
			launchWp.launchSimTo = simTo;
			launchWp.launchSimAim = simAim;
			launchWp.requiresCrouch = needsCrouch;
			launchWp.maxSpeed = speed;
			launchWp.targetAreaId = static_cast<int>(toId);
			result.push_back(launchWp);

			PlanWaypoint landWp;
			landWp.point = AtPlayerHeight(land, toId);
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

		const PlanGate *doorGate = GateFor(conn, toId);
		Vector doorPoint = doorGate
			? Vector((doorGate->left.x + doorGate->right.x) * 0.5f,
				(doorGate->left.y + doorGate->right.y) * 0.5f,
				(doorGate->left.z + doorGate->right.z) * 0.5f)
			: toArea.centroid;

		PlanWaypoint wp;
		wp.point = AtPlayerHeight(doorPoint, toId);
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
