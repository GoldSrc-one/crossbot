#include "waypoint_plan.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "funnel_graph.h"
#include "nav_math.h"
#include "path_profile.h"

namespace NavPlan
{

namespace
{

constexpr float kWallNormalZ = 0.3f;

constexpr float kInsideEpsilon = -0.01f;

constexpr float kLaunchInset = 2.f;

constexpr float kPlanLaunchLift = 1.f;

constexpr float kAirmoveSimMaxTime = 6.f;

constexpr float kAirmoveSimMinTime = 0.1f;

constexpr float kDegToRad = 3.14159265358979323846f / 180.f;

constexpr float kPlanAirmoveRunupTime = 0.1f;
constexpr float kPlanAirmoveRunupMin = 8.f;
constexpr float kPlanAirmoveRespeedEps = 1.f;

struct PendingLaunch
{
	size_t index = 0;
	uint32_t fromId = 0, toId = 0;
	bool jump = false;
	bool needsCrouch = false;
	Vector launchFloor;
	float yaw = 0.f;
	float designSpeed = 0.f;
};

bool PointInArea(const Vector &p, const PlanArea &area, float epsilon)
{
	return NavMath::PointInsidePlanes(p, area.boundPlanes, epsilon);
}

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

Vector NearestPointOnGate(const std::vector<Vector> &line, const Vector &left, const Vector &right)
{
	Vector span = right - left;
	float spanSq = span.x * span.x + span.y * span.y;
	if (spanSq < 0.0001f || line.size() < 2)
		return left;

	float bestDistSq = 1e30f;
	Vector best = left;

	for (size_t i = 0; i + 1 < line.size(); ++i)
	{
		Vector seg = line[i + 1] - line[i];
		float segSq = seg.x * seg.x + seg.y * seg.y;
		if (segSq < 0.0001f)
			continue;

		Vector w = left - line[i];
		float b = span.x * seg.x + span.y * seg.y;
		float d = span.x * w.x + span.y * w.y;
		float e = seg.x * w.x + seg.y * w.y;

		float denom = spanSq * segSq - b * b;
		float s = (std::fabs(denom) > 0.0001f) ? ((b * e - segSq * d) / denom) : 0.f;
		s = (s < 0.f) ? 0.f : (s > 1.f ? 1.f : s);

		float u = (b * s + e) / segSq;
		u = (u < 0.f) ? 0.f : (u > 1.f ? 1.f : u);

		s = (b * u - d) / spanSq;
		s = (s < 0.f) ? 0.f : (s > 1.f ? 1.f : s);

		Vector onGate = left + span * s;
		Vector onLine = line[i] + seg * u;
		float dx = onGate.x - onLine.x, dy = onGate.y - onLine.y;
		float distSq = dx * dx + dy * dy;

		if (distSq < bestDistSq)
		{
			bestDistSq = distSq;
			best = onGate;
		}
	}

	return best;
}

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

float GateFraction(const PlanGate &gate, const Vector &p)
{
	Vector span = gate.right - gate.left;
	float lenSq = span.x * span.x + span.y * span.y;
	if (lenSq < 0.0001f)
		return 0.f;

	float t = ((p.x - gate.left.x) * span.x + (p.y - gate.left.y) * span.y) / lenSq;
	return (t < 0.f) ? 0.f : (t > 1.f ? 1.f : t);
}

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

	uint32_t insideArea = 0;
	uint32_t toArea = 0;
	bool crouch = false;
	float clearance = kPlanClearance;
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

constexpr float kStepClipSpacing = 4.f;
constexpr int kStepClipRefineIters = 8;
constexpr float kStepClipMinSpan = 1.f;

bool ClipLineByStep(EdgeLine &line, uint32_t areaA, uint32_t areaB, const PlanFloorFn &floorAt,
	float maxStep)
{
	auto steppable = [&](float s)
	{
		float x = line.base.x + line.along.x * s;
		float y = line.base.y + line.along.y * s;
		float za = 0.f, zb = 0.f;
		if (!floorAt(areaA, x, y, za) || !floorAt(areaB, x, y, zb))
			return false;
		return std::fabs(za - zb) <= maxStep;
	};

	int samples = static_cast<int>((line.hi - line.lo) / kStepClipSpacing) + 1;
	if (samples < 2)
		samples = 2;
	if (samples > 512)
		samples = 512;

	float step = (line.hi - line.lo) / static_cast<float>(samples - 1);

	int bestStart = -1, bestEnd = -1;
	int runStart = -1;
	for (int i = 0; i < samples; ++i)
	{
		if (steppable(line.lo + step * static_cast<float>(i)))
		{
			if (runStart < 0)
				runStart = i;
			if (bestStart < 0 || i - runStart > bestEnd - bestStart)
			{
				bestStart = runStart;
				bestEnd = i;
			}
		}
		else
		{
			runStart = -1;
		}
	}

	if (bestStart < 0)
		return false;

	auto refine = [&](float good, float bad)
	{
		for (int i = 0; i < kStepClipRefineIters; ++i)
		{
			float mid = (good + bad) * 0.5f;
			(steppable(mid) ? good : bad) = mid;
		}
		return good;
	};

	float lo = line.lo + step * static_cast<float>(bestStart);
	float hi = line.lo + step * static_cast<float>(bestEnd);

	if (bestStart > 0)
		lo = refine(lo, lo - step);
	if (bestEnd < samples - 1)
		hi = refine(hi, hi + step);

	line.lo = lo;
	line.hi = hi;
	return (line.hi - line.lo) > kStepClipMinSpan;
}

float GateClearance(const PlanGate &gate)
{
	if (gate.clamped)
		return kPlanClampedClearance;

	float dx = gate.left.x - gate.right.x;
	float dy = gate.left.y - gate.right.y;
	float half = std::sqrt(dx * dx + dy * dy) * 0.5f;
	return std::min(kPlanClearance + half, kPlanClearance * 2.f);
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

void BuildConnectionGates(const std::vector<PlanArea> &areas, std::vector<PlanConnection> &conns,
	const PlanFloorFn &floorAt, float maxStep)
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

		if (conn.connType == crossbot_nav::CONN_AIRMOVE)
			continue;

		EdgeLine shared = EdgeOfPlane(a, conn.planeA);
				bool fromA = shared.valid && ClipLineByWalls(shared, b, nullptr);

				if (!fromA)
				{
					shared = EdgeOfPlane(b, conn.planeB);
					if (shared.valid && !ClipLineByWalls(shared, a, nullptr))
						shared.valid = false;
				}

				if (shared.valid && (!floorAt || maxStep <= 0.f ||
					ClipLineByStep(shared, conn.areaA, conn.areaB, floorAt, maxStep)))
				{
					Vector inwardA = fromA ? shared.inward
						: Vector(-shared.inward.x, -shared.inward.y, 0.f);

					conn.gateA = GateFromEdge(shared, a, inwardA);
					conn.gateB = GateFromEdge(shared, b, Vector(-inwardA.x, -inwardA.y, 0.f));
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
	std::vector<PendingLaunch> launches;

	auto AtPlayerHeight = [&](const Vector &floorPoint, uint32_t areaId)
	{
		bool crouch = (areaId < areas.size()) && areas[areaId].crouch;
		return Vector(floorPoint.x, floorPoint.y, floorPoint.z + (crouch ? 0.f : kPlanStandHeight));
	};

	auto GateFor = [&](const PlanConnection &conn, uint32_t insideId) -> const PlanGate *
	{
		const PlanGate &g = (conn.areaA == insideId) ? conn.gateA : conn.gateB;
		return g.valid ? &g : nullptr;
	};

	auto PushGateInside = [&](const PlanConnection &conn, uint32_t insideId, uint32_t fromId,
		uint32_t toId)
	{
		const PlanGate *g = GateFor(conn, insideId);
		if (!g)
			return;

		const PlanArea &fromArea = areas[fromId];
		const PlanArea &toArea = areas[toId];

		if (g->clamped && stats)
			++stats->gatesClamped;

		PendingGate pg;
		pg.insideArea = insideId;
		pg.toArea = toId;
		pg.crouch = fromArea.crouch || toArea.crouch;
		pg.clearance = GateClearance(*g);
		pg.gate = NavFunnel::MakeGate(g->left, g->right, toArea.centroid - fromArea.centroid);
		pendingGates.push_back(pg);
	};

	auto PushGate = [&](const PlanConnection &conn, uint32_t fromId, uint32_t toId)
	{
		PushGateInside(conn, fromId, fromId, toId);
		PushGateInside(conn, toId, fromId, toId);
	};

	auto PeekFunnel = [&](const Vector &target)
	{
		std::vector<NavFunnel::Gate> gates;
		gates.reserve(pendingGates.size());
		for (const PendingGate &pg : pendingGates)
			gates.push_back(pg.gate);

		return NavFunnel::StringPull(anchorPoint, target, gates);
	};

	auto FlushFunnelTo = [&](const Vector &target)
	{
		std::vector<NavFunnel::Gate> gates;
		gates.reserve(pendingGates.size());
		for (const PendingGate &pg : pendingGates)
			gates.push_back(pg.gate);

		std::vector<NavFunnel::PulledPoint> pulled = NavFunnel::StringPull(anchorPoint, target, gates);

		for (size_t k = 1; k + 1 < pulled.size(); ++k)
		{
			PlanWaypoint corner;
			corner.point = pulled[k].point;

			if (pulled[k].gateIndex < pendingGates.size())
			{
				const PendingGate &pg = pendingGates[pulled[k].gateIndex];
				Vector p = pulled[k].point;
				p.z = GateFloorZ(pg.gate, p);
				corner.point = AtPlayerHeight(p, pg.insideArea);
				corner.requiresCrouch = pg.crouch;
				corner.targetAreaId = static_cast<int>(pg.toArea);
				corner.clearance = pg.clearance;
			}

			if (stats)
				++stats->corners;
			result.push_back(corner);
		}

		pendingGates.clear();
	};

	auto JumpLandsIn = [&](const Vector &launchFloor, float yaw, float speed, bool jump,
		uint32_t fromAreaId, uint32_t destAreaId, bool needsCrouch, Vector &endOut,
		const char **whyOut = nullptr) -> bool
	{
		endOut = launchFloor;
		if (whyOut)
			*whyOut = "no traces";

		if (!traces)
			return false;

		const PmSim::PmTraceFn &trace = needsCrouch ? traces->duckTrace : traces->standTrace;
		if (!trace)
			return false;

		bool duckLaunch = (fromAreaId < areas.size()) && areas[fromAreaId].crouch;
		float launchLift = duckLaunch ? 0.f : kPlanStandHeight;
		float zAdjust = needsCrouch ? 0.f : kPlanStandHeight;

		PmSim::PmPhysics physics = traces->physics;
		PmSim::PmSimulator sim(trace, physics);
		sim.Begin(Vector(launchFloor.x, launchFloor.y, launchFloor.z + launchLift + kPlanLaunchLift),
			yaw, speed, false);

		auto record = [&]()
		{
			const Vector &p = sim.Origin();
			endOut = Vector(p.x, p.y, p.z - zAdjust);
		};

		if (jump)
			sim.RequestJump();

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
			FlushFunnelTo(conn.launchPos);

			PlanWaypoint triggerWp;
			triggerWp.point = conn.launchPos;
			triggerWp.requiresTeleport = true;
			triggerWp.teleportDest = conn.landingPos;
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

			size_t gatesBefore = pendingGates.size();
			PushGateInside(conn, toId, fromId, toId);

			Vector launch = conn.launchPos;
			Vector land = conn.landingPos;
			float yaw = conn.exitAngle;
			float launchSpeed = conn.exitSpeed;
			bool usedSim = false;
			Vector simFrom = launch, simTo = land, simAim = land;

			if (traces && pendingGates.size() > gatesBefore)
			{
				std::vector<NavFunnel::PulledPoint> pulled =
					PeekFunnel(AtPlayerHeight(conn.landingPos, toId));

				std::vector<Vector> line;
				line.reserve(pulled.size());
				for (const NavFunnel::PulledPoint &p : pulled)
					line.push_back(p.point);

				const PlanGate *exitG = GateFor(conn, fromId);
				Vector exitPoint = exitG
					? NearestPointOnGate(line, exitG->left, exitG->right)
					: conn.launchPos;

				Vector entryPoint = AtPlayerHeight(conn.landingPos, toId);
				if (pendingGates.size() > gatesBefore)
				{
					entryPoint = CrossingOnGate(line, pendingGates[gatesBefore].gate);
					for (const NavFunnel::PulledPoint &p : pulled)
						if (p.gateIndex == gatesBefore)
							entryPoint = p.point;
				}

				float t = GateFraction(conn.gateA, exitPoint);
				float candYaw = LerpYaw(conn.gateAYaw[0], conn.gateAYaw[1], t);
				float candSpeed = conn.gateASpeed[0] + (conn.gateASpeed[1] - conn.gateASpeed[0]) * t;
				if (candSpeed <= 0.f)
					candSpeed = conn.exitSpeed;

				Vector end;
				const char *why = "";
				simFrom = exitPoint;
				simAim = entryPoint;
				usedSim = JumpLandsIn(exitPoint, candYaw, candSpeed, jump, fromId, toId, needsCrouch,
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

			pendingGates.resize(gatesBefore);

			if (stats)
			{
				if (usedSim)
					++stats->airmovesSimulated;
				else
					++stats->airmovesFromStored;
			}
			if (!usedSim)
			{
				if (stats)
					++stats->airmovesUnplannable;
				FlushFunnelTo(AtPlayerHeight(conn.launchPos, fromId));
				return result;
			}

			Vector launchPoint = AtPlayerHeight(launch, fromId);
			Vector launchDir(std::cos(yaw * kDegToRad), std::sin(yaw * kDegToRad), 0.f);
			float runupBack = std::max(launchSpeed * kPlanAirmoveRunupTime, kPlanAirmoveRunupMin);

			PlanWaypoint runupWp;
			runupWp.point = launchPoint - launchDir * runupBack;
			runupWp.targetAreaId = static_cast<int>(fromId);
			FlushFunnelTo(runupWp.point);
			result.push_back(runupWp);

			PlanWaypoint launchWp;
			launchWp.point = launchPoint;
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
			launchWp.targetAreaId = static_cast<int>(toId);

			PendingLaunch pl;
			pl.index = result.size();
			pl.fromId = fromId;
			pl.toId = toId;
			pl.jump = jump;
			pl.needsCrouch = needsCrouch;
			pl.launchFloor = launch;
			pl.yaw = yaw;
			pl.designSpeed = launchSpeed;
			launches.push_back(pl);

			result.push_back(launchWp);

			PlanWaypoint landWp;
			landWp.point = AtPlayerHeight(land, toId);
			landWp.targetAreaId = static_cast<int>(toId);
			result.push_back(landWp);

			anchorPoint = landWp.point;
			continue;
		}

		bool needsStop = toArea.door || toArea.breakable;
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
		wp.requiresDoor = toArea.door;
		wp.requiresBreak = toArea.breakable;
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
			FlushFunnelTo(buttonWp.point);
			result.push_back(buttonWp);
			anchorPoint = buttonWp.point;
		}
		else if (toArea.doorActivation == crossbot_nav::DOOR_ACTIVATION_TRIGGER)
		{
			PlanWaypoint triggerWp;
			triggerWp.point = toArea.doorActivatorPos;
			triggerWp.requiresTriggerTouch = true;
			triggerWp.targetAreaId = static_cast<int>(fromId);
			FlushFunnelTo(triggerWp.point);
			result.push_back(triggerWp);
			anchorPoint = triggerWp.point;
		}

		FlushFunnelTo(wp.point);
		result.push_back(wp);
		anchorPoint = wp.point;
	}

	PlanWaypoint goalWp;
	FlushFunnelTo(goalPoint);
	goalWp.point = goalPoint;
	if (!areaPath.empty())
		goalWp.targetAreaId = static_cast<int>(areaPath.back());
	result.push_back(goalWp);

	if (!traces || launches.empty())
		return result;

	PlanProfile probe = BuildPathProfile(result, traces->physics, traces);

	for (const PendingLaunch &pl : launches)
	{
		float reached = ProfileReachAtWaypoint(probe, pl.index);
		if (reached <= 0.f || reached >= pl.designSpeed - kPlanAirmoveRespeedEps)
			continue;

		Vector end;
		const char *why = "";
		bool ok = JumpLandsIn(pl.launchFloor, pl.yaw, reached, pl.jump, pl.fromId, pl.toId,
			pl.needsCrouch, end, &why);

		if (traces->dumpAirmoves)
		{
			std::printf("    airmove %u->%u respeed %.0f -> %.0f: %s -> end (%.1f,%.1f,%.1f)\n",
				pl.fromId, pl.toId, pl.designSpeed, reached, why, end.x, end.y, end.z);
		}

		if (ok)
		{
			result[pl.index].launchSpeed = reached;
			result[pl.index].launchSimTo = end;
			result[pl.index + 1].point = AtPlayerHeight(end, pl.toId);
			if (stats)
				++stats->airmovesRespeeded;
			continue;
		}

		result.resize(pl.index);
		if (stats)
		{
			if (stats->airmovesSimulated > 0)
				--stats->airmovesSimulated;
			++stats->airmovesUnplannable;
		}
		break;
	}

	return result;
}

}

