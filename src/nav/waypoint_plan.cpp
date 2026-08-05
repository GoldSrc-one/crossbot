#include "waypoint_plan.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "astar_graph.h"
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

bool PointInArea(const Vector &p, const PlanArea &area, float epsilon)
{
	return NavMath::PointInsidePlanes(p, area.boundPlanes, epsilon);
}

// An area's footprint is not fixed by its vertical planes alone. A floor rising one way and a
// ceiling falling the other pinch the column shut where they meet, and that closure is a vertical
// half-plane no bound plane records: a wedge under a ramp is capped by the ramp itself, and the
// sliver between two faces of a pyramid is bounded that way at both ends. Eliminating z from the
// half-space system recovers those bounds exactly - for a floor f and a ceiling c, "the floor stays
// under the ceiling" is one linear constraint in x and y, and it is implied by the two planes, so
// it can only trim overreach, never cut into the area.
template <typename Fn>
void ForEachImpliedWall(const PlanArea &area, Fn &&fn)
{
	for (const PlanPlane &f : area.boundPlanes)
	{
		if (f.normal.z <= kWallNormalZ)
			continue;

		for (const PlanPlane &c : area.boundPlanes)
		{
			if (c.normal.z >= -kWallNormalZ)
				continue;

			float nx = f.normal.z * c.normal.x - c.normal.z * f.normal.x;
			float ny = f.normal.z * c.normal.y - c.normal.z * f.normal.y;
			float dist = f.normal.z * c.dist - c.normal.z * f.dist;

			// Parallel in plan: the two never converge, so they bound nothing here.
			float len = std::sqrt(nx * nx + ny * ny);
			if (len < 0.0001f)
				continue;

			fn(nx / len, ny / len, dist / len);
		}
	}
}

bool InAreaColumn(const Vector &p, const PlanArea &area, float margin)
{
	if (area.boundPlanes.empty())
		return false;

	bool inside = true;
	auto test = [&](float nx, float ny, float dist)
	{
		if (p.x * nx + p.y * ny - dist < margin)
			inside = false;
	};

	for (const PlanPlane &pl : area.boundPlanes)
	{
		if (std::fabs(pl.normal.z) > kWallNormalZ)
			continue;
		test(pl.normal.x, pl.normal.y, pl.dist);
		if (!inside)
			return false;
	}

	ForEachImpliedWall(area, test);
	return inside;
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
	bool bounded = true;

	auto clip = [&](float nx, float ny, float dist)
	{
		float denom = nx * line.along.x + ny * line.along.y;
		float at = line.base.x * nx + line.base.y * ny - dist;

		if (std::fabs(denom) < 0.0001f)
		{
			if (at < -0.05f)
				bounded = false;
			return;
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
	};

	for (const PlanPlane &other : area.boundPlanes)
	{
		if (&other == skip)
			continue;
		if (std::fabs(other.normal.z) > kWallNormalZ)
			continue;

		clip(other.normal.x, other.normal.y, other.dist);
	}

	ForEachImpliedWall(area, clip);

	return bounded && (line.hi - line.lo) > 0.0001f;
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

PlanWalker::PlanWalker(const std::vector<PlanArea> &areas, const PlanTraceContext *traces)
	: m_areas(&areas), m_traces(traces)
{
}

void PlanWalker::Reset(const Vector &startPoint)
{
	m_result.clear();
	m_pendingGates.clear();
	m_launches.clear();

	m_anchor = startPoint;
	m_frontier = startPoint;
	m_committedLength = 0.f;
	m_pendingLength = 0.f;
	m_extraTime = 0.f;
	m_stats = PlanStats();

	m_result.push_back({startPoint});
}

float PlanWalker::Time() const
{
	return (m_committedLength + m_pendingLength) / NavAStar::kNavRunSpeed + m_extraTime;
}

Vector PlanWalker::AtPlayerHeight(const Vector &floorPoint, uint32_t areaId) const
{
	bool crouch = (areaId < m_areas->size()) && (*m_areas)[areaId].crouch;
	return Vector(floorPoint.x, floorPoint.y, floorPoint.z + (crouch ? 0.f : kPlanStandHeight));
}

const PlanGate *PlanWalker::GateFor(const PlanConnection &conn, uint32_t insideId) const
{
	const PlanGate &g = (conn.areaA == insideId) ? conn.gateA : conn.gateB;
	return g.valid ? &g : nullptr;
}

void PlanWalker::PushGateInside(const PlanConnection &conn, uint32_t insideId, uint32_t fromId,
	uint32_t toId)
{
	const PlanGate *g = GateFor(conn, insideId);
	if (!g)
		return;

	const PlanArea &fromArea = (*m_areas)[fromId];
	const PlanArea &toArea = (*m_areas)[toId];

	if (g->clamped)
		++m_stats.gatesClamped;

	PendingGate pg;
	pg.insideArea = insideId;
	pg.toArea = toId;
	pg.crouch = fromArea.crouch || toArea.crouch;
	pg.clearance = GateClearance(*g);
	pg.gate = NavFunnel::MakeGate(g->left, g->right, toArea.centroid - fromArea.centroid);
	m_pendingGates.push_back(pg);
}

void PlanWalker::PushGate(const PlanConnection &conn, uint32_t fromId, uint32_t toId)
{
	PushGateInside(conn, fromId, fromId, toId);
	PushGateInside(conn, toId, fromId, toId);
}

std::vector<NavFunnel::PulledPoint> PlanWalker::PeekFunnel(const Vector &target) const
{
	std::vector<NavFunnel::Gate> gates;
	gates.reserve(m_pendingGates.size());
	for (const PendingGate &pg : m_pendingGates)
		gates.push_back(pg.gate);

	return NavFunnel::StringPull(m_anchor, target, gates);
}

void PlanWalker::PushWaypoint(const PlanWaypoint &wp, bool countLength)
{
	if (countLength && !m_result.empty())
		m_committedLength += (wp.point - m_result.back().point).Length();
	m_result.push_back(wp);
}

bool PlanWalker::FlushFunnelTo(const Vector &target)
{
	std::vector<NavFunnel::PulledPoint> pulled = PeekFunnel(target);

	size_t gateCount = m_pendingGates.size();
	size_t spanStart = 0;
	bool tailCrouch = false;

	auto SpanCrouch = [&](size_t lo, size_t hi)
	{
		for (size_t g = lo; g < hi && g < gateCount; ++g)
			if (m_pendingGates[g].crouch)
				return true;
		return false;
	};

	if (pulled.size() < 2)
		tailCrouch = SpanCrouch(0, gateCount);

	for (size_t k = 1; k < pulled.size(); ++k)
	{
		size_t gate = pulled[k].gateIndex;
		bool crouch = SpanCrouch(spanStart, gate == NavFunnel::kNoGate ? gateCount : gate + 1);
		if (gate != NavFunnel::kNoGate)
			spanStart = gate;

		if (k + 1 == pulled.size())
		{
			tailCrouch = crouch;
			break;
		}

		PlanWaypoint corner;
		corner.point = pulled[k].point;
		corner.requiresCrouch = crouch;

		if (gate < gateCount)
		{
			const PendingGate &pg = m_pendingGates[gate];
			Vector p = pulled[k].point;
			p.z = GateFloorZ(pg.gate, p);
			corner.point = AtPlayerHeight(p, pg.insideArea);
			corner.targetAreaId = static_cast<int>(pg.toArea);
			corner.clearance = pg.clearance;
		}

		++m_stats.corners;
		PushWaypoint(corner);
	}

	m_pendingGates.clear();
	m_pendingLength = 0.f;
	return tailCrouch;
}

void PlanWalker::UpdateFrontier()
{
	if (m_pendingGates.empty())
	{
		m_frontier = m_anchor;
		m_pendingLength = 0.f;
		return;
	}

	const NavFunnel::Gate &last = m_pendingGates.back().gate;
	Vector target = (last.left + last.right) * 0.5f;

	std::vector<NavFunnel::PulledPoint> pulled = PeekFunnel(target);

	float len = 0.f;
	for (size_t k = 1; k < pulled.size(); ++k)
		len += (pulled[k].point - pulled[k - 1].point).Length();

	m_frontier = target;
	m_pendingLength = len;
}

bool PlanWalker::JumpLandsIn(const Vector &launchFloor, float yaw, float speed, bool jump,
	uint32_t fromAreaId, uint32_t destAreaId, bool needsCrouch, Vector &endOut,
	const char **whyOut) const
{
	endOut = launchFloor;
	if (whyOut)
		*whyOut = "no traces";

	if (!m_traces)
		return false;

	const PmSim::PmTraceFn &trace = needsCrouch ? m_traces->duckTrace : m_traces->standTrace;
	if (!trace)
		return false;

	bool duckLaunch = (fromAreaId < m_areas->size()) && (*m_areas)[fromAreaId].crouch;
	float launchLift = duckLaunch ? 0.f : kPlanStandHeight;
	float zAdjust = needsCrouch ? 0.f : kPlanStandHeight;

	PmSim::PmPhysics physics = m_traces->physics;
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
		bool in = PointInArea(endOut, (*m_areas)[destAreaId], 1.f);
		if (whyOut) *whyOut = in ? "landed in area" : "landed elsewhere";
		return in;
	}

	record();
	if (whyOut) *whyOut = "timed out";
	return false;
}

bool PlanWalker::Cross(const PlanConnection &conn, uint32_t connIndex, uint32_t fromId, uint32_t toId)
{
	const PlanArea &fromArea = (*m_areas)[fromId];
	const PlanArea &toArea = (*m_areas)[toId];

	if (conn.connType == crossbot_nav::CONN_TELEPORT)
	{
		bool tailCrouch = FlushFunnelTo(conn.launchPos);

		PlanWaypoint triggerWp;
		triggerWp.point = conn.launchPos;
		triggerWp.requiresTeleport = true;
		triggerWp.teleportDest = conn.landingPos;
		triggerWp.requiresCrouch = tailCrouch || fromArea.crouch;
		PushWaypoint(triggerWp);

		PlanWaypoint destWp;
		destWp.point = conn.landingPos;
		destWp.requiresCrouch = toArea.crouch;
		PushWaypoint(destWp, false);

		m_anchor = destWp.point;
		UpdateFrontier();
		return true;
	}

	if (conn.connType == crossbot_nav::CONN_AIRMOVE)
	{
		bool needsCrouch = (conn.connFlags & crossbot_nav::CONN_FLAG_NEEDS_CROUCH) != 0;
		bool jump = (conn.connFlags & crossbot_nav::CONN_FLAG_JUMP) != 0;

		size_t gatesBefore = m_pendingGates.size();
		PushGateInside(conn, toId, fromId, toId);

		Vector launch = conn.launchPos;
		Vector land = conn.landingPos;
		float yaw = conn.exitAngle;
		float launchSpeed = conn.exitSpeed;
		bool usedSim = false;
		Vector simFrom = launch, simTo = land, simAim = land;

		if (m_traces && m_pendingGates.size() > gatesBefore)
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
			if (m_pendingGates.size() > gatesBefore)
			{
				entryPoint = CrossingOnGate(line, m_pendingGates[gatesBefore].gate);
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
			else if (m_traces->dumpAirmoves)
			{
				std::printf("    airmove %u->%u sim miss: %s  t %.2f launch (%.1f,%.1f,%.1f) "
					"yaw %.0f speed %.0f jump %d -> end (%.1f,%.1f,%.1f)  entry (%.1f,%.1f)\n",
					fromId, toId, why, t, exitPoint.x, exitPoint.y, exitPoint.z, candYaw,
					candSpeed, jump ? 1 : 0, end.x, end.y, end.z, entryPoint.x, entryPoint.y);
			}
		}

		m_pendingGates.resize(gatesBefore);

		if (!usedSim)
		{
			++m_stats.airmovesUnplannable;
			return false;
		}

		++m_stats.airmovesSimulated;

		Vector launchPoint = AtPlayerHeight(launch, fromId);
		Vector launchDir(std::cos(yaw * kDegToRad), std::sin(yaw * kDegToRad), 0.f);
		float runupBack = std::max(launchSpeed * kPlanAirmoveRunupTime, kPlanAirmoveRunupMin);

		PlanWaypoint runupWp;
		runupWp.point = launchPoint - launchDir * runupBack;
		runupWp.targetAreaId = static_cast<int>(fromId);
		runupWp.requiresCrouch = FlushFunnelTo(runupWp.point) || fromArea.crouch;
		PushWaypoint(runupWp);

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
		pl.index = m_result.size();
		pl.connIndex = connIndex;
		pl.fromId = fromId;
		pl.toId = toId;
		pl.jump = jump;
		pl.needsCrouch = needsCrouch;
		pl.launchFloor = launch;
		pl.yaw = yaw;
		pl.designSpeed = launchSpeed;
		m_launches.push_back(pl);

		PushWaypoint(launchWp);

		PlanWaypoint landWp;
		landWp.point = AtPlayerHeight(land, toId);
		landWp.targetAreaId = static_cast<int>(toId);
		landWp.requiresCrouch = toArea.crouch;
		PushWaypoint(landWp, false);

		m_anchor = landWp.point;
		UpdateFrontier();
		return true;
	}

	bool needsStop = toArea.door || toArea.breakable;
	if (!needsStop)
	{
		PushGate(conn, fromId, toId);
		UpdateFrontier();
		return true;
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
		buttonWp.requiresCrouch = FlushFunnelTo(buttonWp.point) || fromArea.crouch;
		PushWaypoint(buttonWp);
		m_anchor = buttonWp.point;
	}
	else if (toArea.doorActivation == crossbot_nav::DOOR_ACTIVATION_TRIGGER)
	{
		PlanWaypoint triggerWp;
		triggerWp.point = toArea.doorActivatorPos;
		triggerWp.requiresTriggerTouch = true;
		triggerWp.targetAreaId = static_cast<int>(fromId);
		triggerWp.requiresCrouch = FlushFunnelTo(triggerWp.point) || fromArea.crouch;
		PushWaypoint(triggerWp);
		m_anchor = triggerWp.point;
	}

	if (FlushFunnelTo(wp.point))
		wp.requiresCrouch = true;
	PushWaypoint(wp);
	m_anchor = wp.point;

	UpdateFrontier();
	return true;
}

void PlanWalker::Finish(const Vector &goalPoint, int lastAreaId)
{
	PlanWaypoint goalWp;
	goalWp.requiresCrouch = FlushFunnelTo(goalPoint);
	goalWp.point = goalPoint;
	if (lastAreaId >= 0)
	{
		goalWp.targetAreaId = lastAreaId;
		size_t id = static_cast<size_t>(lastAreaId);
		if (id < m_areas->size() && (*m_areas)[id].crouch)
			goalWp.requiresCrouch = true;
	}
	PushWaypoint(goalWp);
}

void PlanWalker::Respeed()
{
	if (!m_traces || m_launches.empty())
		return;

	PlanProfile probe = BuildPathProfile(m_result, m_traces->physics, m_traces);

	for (const PendingLaunch &pl : m_launches)
	{
		float reached = ProfileReachAtWaypoint(probe, pl.index);
		if (reached <= 0.f || reached >= pl.designSpeed - kPlanAirmoveRespeedEps)
			continue;

		Vector end;
		const char *why = "";
		bool ok = JumpLandsIn(pl.launchFloor, pl.yaw, reached, pl.jump, pl.fromId, pl.toId,
			pl.needsCrouch, end, &why);

		if (m_traces->dumpAirmoves)
		{
			std::printf("    airmove %u->%u respeed %.0f -> %.0f: %s -> end (%.1f,%.1f,%.1f)\n",
				pl.fromId, pl.toId, pl.designSpeed, reached, why, end.x, end.y, end.z);
		}

		if (ok)
		{
			m_result[pl.index].launchSpeed = reached;
			m_result[pl.index].launchSimTo = end;
			m_result[pl.index + 1].point = AtPlayerHeight(end, pl.toId);
			++m_stats.airmovesRespeeded;
			continue;
		}

		++m_stats.airmovesRespeedKept;
	}
}

}

