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

uint32_t g_detourTeamMask = ~0u;

constexpr float kWallNormalZ = 0.3f;

constexpr float kPlanNoCeiling = 1.0e9f;

constexpr float kPlanSwimMergeDist = 1.f;

constexpr float kPlanSwimCeilingSlack = 2.f;

constexpr float kInsideEpsilon = -0.01f;

constexpr float kLaunchInset = 2.f;

constexpr float kPlanLaunchLift = 1.f;

constexpr float kAirmoveSimMaxTime = 6.f;

constexpr float kAirmoveSimMinTime = 0.1f;

constexpr float kDegToRad = 3.14159265358979323846f / 180.f;

constexpr float kPlanGravityEpsilon = 0.01f;

constexpr float kPlanAirmoveRunupTime = 0.1f;
constexpr float kPlanAirmoveRunupMin = 8.f;
constexpr float kPlanAirmoveRunupEps = 1.f;
constexpr float kPlanRunupSeamStep = 18.f;
constexpr float kPlanRelaxClearance = 12.f;
constexpr float kPlanAirmoveRespeedEps = 1.f;
constexpr float kPlanAirmoveRangeEps = 1.f;
constexpr float kPlanAirmoveFlightEps = 0.01f;
constexpr float kPlanAirmoveRetractAlign = 0.5f;

float ClampTravelInsideArea(const PlanArea &area, const Vector &from, const Vector &dir, float want,
	float inset)
{
	float limit = want;

	for (const PlanPlane &p : area.boundPlanes)
	{
		if (std::fabs(p.normal.z) > kWallNormalZ)
			continue;

		float denom = p.normal.x * dir.x + p.normal.y * dir.y;
		if (denom >= -0.0001f)
			continue;

		float margin = from.x * p.normal.x + from.y * p.normal.y + from.z * p.normal.z - p.dist - inset;
		if (margin <= 0.f)
			return 0.f;

		limit = std::min(limit, margin / -denom);
	}

	return std::max(limit, 0.f);
}

bool PointInArea(const Vector &p, const PlanArea &area, float epsilon)
{
	return NavMath::PointInsidePlanes(p, area.boundPlanes, epsilon);
}

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

float YawDelta(float a, float b)
{
	float d = b - a;
	while (d > 180.f)
		d -= 360.f;
	while (d < -180.f)
		d += 360.f;
	return d;
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
	bool any = false;
	float best = 0.f;

	for (const PlanPlane &pl : area.boundPlanes)
	{
		if (pl.normal.z <= kWallNormalZ)
			continue;

		float z = (pl.dist - pl.normal.x * x - pl.normal.y * y) / pl.normal.z;
		if (!any || z > best)
		{
			best = z;
			any = true;
		}
	}

	return any ? best : 0.f;
}

float AreaCeilingZ(const PlanArea &area, float x, float y)
{
	bool any = false;
	float best = 0.f;

	for (const PlanPlane &pl : area.boundPlanes)
	{
		if (pl.normal.z >= -kWallNormalZ)
			continue;

		float z = (pl.dist - pl.normal.x * x - pl.normal.y * y) / pl.normal.z;
		if (!any || z < best)
		{
			best = z;
			any = true;
		}
	}

	return any ? best : kPlanNoCeiling;
}

float SwimOriginZ(const PlanArea &area, float x, float y, float surfaceZ)
{
	float ceiling = std::min(AreaCeilingZ(area, x, y), surfaceZ);
	float floorZ = AreaFloorZ(area, x, y);
	float lift = area.crouch ? 0.f : kPlanStandHeight;

	float z = ceiling - lift - kPlanSwimCeilingSlack;
	if (z < floorZ)
		z = std::min(ceiling, floorZ + lift);

	return z;
}

bool FloorsMeetAt(const PlanArea &here, const PlanArea &there, const Vector &seam, float tolerance)
{
	const PlanPlane *a = FloorPlaneOf(here);
	const PlanPlane *b = FloorPlaneOf(there);
	if (!a || !b)
		return false;

	float zHere = AreaFloorZ(here, seam.x, seam.y);
	float zThere = AreaFloorZ(there, seam.x, seam.y);
	return std::fabs(zThere - zHere) <= tolerance;
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

	auto pushFrom = [&](const Vector &p)
	{
		EdgeLine depth;
		depth.base = p;
		depth.along = inward;
		depth.lo = 0.f;
		if (!ClipLineByWalls(depth, inside, nullptr) || depth.hi > 1e8f)
			return kPlanClearance;
		return std::min(kPlanClearance, 0.5f * depth.hi);
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
		Vector pushed = shift(c, inward, pushFrom(c));
		gate.left = gate.right = onFloor(InAreaColumn(pushed, inside, kInsideEpsilon) ? pushed : c);
		gate.clamped = true;
		return gate;
	}

	float push = std::min(pushFrom(l), pushFrom(r));
	Vector pushedL = shift(l, inward, push);
	Vector pushedR = shift(r, inward, push);
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

float CornerClearance(const PendingGate &pg, const std::vector<PlanArea> &areas, const Vector &prev,
	const Vector &p, const Vector &next)
{
	Vector bis = NavMath::Norm2D(NavMath::Norm2D(next - p) - NavMath::Norm2D(p - prev));
	if (bis.x == 0.f && bis.y == 0.f)
		return pg.clearance;

	uint32_t otherId = (pg.insideArea == pg.toArea) ? pg.fromArea : pg.toArea;
	const PlanArea *inside = pg.insideArea < areas.size() ? &areas[pg.insideArea] : nullptr;
	const PlanArea *other = otherId < areas.size() ? &areas[otherId] : nullptr;

	const float candidates[] = { pg.clearance, kPlanClearance, kPlanClampedClearance };
	for (float c : candidates)
	{
		if (c > pg.clearance)
			continue;
		Vector apex(p.x + bis.x * c, p.y + bis.y * c, p.z);
		if ((inside && InAreaColumn(apex, *inside, kInsideEpsilon)) ||
			(other && InAreaColumn(apex, *other, kInsideEpsilon)))
			return c;
	}
	return std::min(pg.clearance, kPlanClampedClearance);
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

		if (conn.connType == crossbot_nav::CONN_TELEPORT || conn.connType == crossbot_nav::CONN_ELEVATOR)
			continue;
		if (conn.areaA >= areas.size() || conn.areaB >= areas.size())
			continue;

		const PlanArea &a = areas[conn.areaA];
		const PlanArea &b = areas[conn.areaB];

		if (conn.connType == crossbot_nav::CONN_AIRMOVE || conn.connType == crossbot_nav::CONN_LADDER ||
			conn.connType == crossbot_nav::CONN_SWIM)
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

void SetDetourTeamMask(uint32_t mask)
{
	g_detourTeamMask = mask;
}

const PlanDetour *FindDetour(const PlanArea &area, uint32_t fromId)
{
	for (const PlanDetour &d : area.detours)
		if (d.fromAreaId == fromId && (d.teamMask == 0 || (d.teamMask & g_detourTeamMask) != 0))
			return &d;
	return nullptr;
}

Vector GateMidpoint(const PlanGate &gate)
{
	return Vector((gate.left.x + gate.right.x) * 0.5f, (gate.left.y + gate.right.y) * 0.5f,
		(gate.left.z + gate.right.z) * 0.5f);
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

PlanWalker::PlanWalker(const std::vector<PlanArea> &areas, const PlanTraceContext *traces,
	const std::vector<std::vector<uint32_t>> *walkPredecessors,
	const std::vector<std::vector<uint32_t>> *walkSuccessors)
	: m_areas(&areas), m_walkPreds(walkPredecessors), m_walkSuccs(walkSuccessors), m_traces(traces)
{
}

void PlanWalker::Reset(const Vector &startPoint)
{
	m_result.clear();
	m_pendingGates.clear();
	m_launches.clear();

	m_anchor = startPoint;
	m_anchorFromAirmove = false;
	m_frontier = startPoint;
	m_committedLength = 0.f;
	m_committedSwimLength = 0.f;
	m_pendingLength = 0.f;
	m_extraTime = 0.f;
	m_stats = PlanStats();

	m_result.push_back({startPoint});
}

float PlanWalker::Time() const
{
	float onFoot = m_committedLength - m_committedSwimLength + m_pendingLength;

	return onFoot / NavAStar::kNavRunSpeed + m_committedSwimLength / NavAStar::kNavSwimSpeed +
		m_extraTime;
}

bool PlanWalker::AreaUnderWater(int areaId) const
{
	if (areaId < 0 || static_cast<size_t>(areaId) >= m_areas->size())
		return false;

	const PlanArea &area = (*m_areas)[static_cast<size_t>(areaId)];

	return SubmergedAt(m_traces ? m_traces->waterSurfaces : nullptr, area.waterVolume,
		area.submergeZ);
}

Vector PlanWalker::AtPlayerHeight(const Vector &floorPoint, uint32_t areaId) const
{
	if (areaId >= m_areas->size())
		return Vector(floorPoint.x, floorPoint.y, floorPoint.z + kPlanStandHeight);

	const PlanArea &area = (*m_areas)[areaId];
	const std::vector<float> *surfaces = m_traces ? m_traces->waterSurfaces : nullptr;

	if (SubmergedAt(surfaces, area.waterVolume, area.submergeZ))
	{
		float surfaceZ = (*surfaces)[area.waterVolume];
		return Vector(floorPoint.x, floorPoint.y,
			SwimOriginZ(area, floorPoint.x, floorPoint.y, surfaceZ));
	}

	return Vector(floorPoint.x, floorPoint.y,
		floorPoint.z + (area.crouch ? 0.f : kPlanStandHeight));
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
	pg.fromArea = fromId;
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
	{
		float len = (wp.point - m_result.back().point).Length();
		m_committedLength += len;

		if (AreaUnderWater(m_result.back().targetAreaId) || AreaUnderWater(wp.targetAreaId))
			m_committedSwimLength += len;
	}

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
			corner.clearance = CornerClearance(pg, *m_areas, pulled[k - 1].point, pulled[k].point,
				pulled[k + 1].point);
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

bool PlanWalker::ApplyAirmoveResult(const AirmoveResult &result, uint32_t destAreaId,
	Vector &endOut, const char **whyOut) const
{
	endOut = result.end;

	if (result.outcome == AIRMOVE_WEDGED)
	{
		if (whyOut) *whyOut = "wedged";
		return false;
	}

	if (result.outcome == AIRMOVE_TIMEOUT)
	{
		if (whyOut) *whyOut = "timed out";
		return false;
	}

	bool in = PointInArea(endOut, (*m_areas)[destAreaId], 1.f);
	if (whyOut) *whyOut = in ? "landed in area" : "landed elsewhere";
	return in;
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

	const PmSim::PmPushFn &push = needsCrouch ? m_traces->duckPush : m_traces->standPush;

	bool duckLaunch = (fromAreaId < m_areas->size()) && (*m_areas)[fromAreaId].crouch;
	float launchLift = duckLaunch ? 0.f : kPlanStandHeight;
	float zAdjust = needsCrouch ? 0.f : kPlanStandHeight;

	PmSim::PmPhysics physics = m_traces->physics;
	physics.frameTime = PmSim::kPlanSimFrameTime;

	Vector simStart(launchFloor.x, launchFloor.y, launchFloor.z + launchLift + kPlanLaunchLift);

	AirmoveCache *cache = m_traces->airmoveCache;
	bool caching = cache != nullptr;
	AirmoveKey key;

	if (caching)
	{
		key.x = simStart.x;
		key.y = simStart.y;
		key.z = simStart.z;
		key.yaw = yaw;
		key.speed = speed;
		key.physics = AirmovePhysicsKey(physics.maxSpeed, physics.gravity, physics.airAccelerate,
			physics.maxVelocity);
		key.jump = jump ? 1 : 0;
		key.crouch = needsCrouch ? 1 : 0;
	}

	AirmoveResult cached;
	bool haveCached = caching && cache->Lookup(key, cached);
	if (haveCached && !cache->Verify())
		return ApplyAirmoveResult(cached, destAreaId, endOut, whyOut);

	PmSim::PmSimulator sim(trace, physics, push);
	sim.Begin(simStart, yaw, speed, false);

	if (jump)
		sim.RequestJump();

	AirmoveResult result;
	result.outcome = AIRMOVE_TIMEOUT;

	while (sim.Time() < kAirmoveSimMaxTime)
	{
		if (!sim.Step())
		{
			result.outcome = AIRMOVE_WEDGED;
			break;
		}

		if (!sim.OnGround())
			continue;

		if (sim.Time() < kAirmoveSimMinTime)
		{
			sim.ResetFallSpeed();
			continue;
		}

		result.outcome = AIRMOVE_LANDED;
		break;
	}

	const Vector &landed = sim.Origin();
	result.end = Vector(landed.x, landed.y, landed.z - zAdjust);

	if (haveCached)
		cache->NoteVerified(cached, result, sim.TouchedEntity());
	else if (caching)
	{
		if (sim.TouchedEntity())
			cache->NoteUncacheable();
		else
			cache->Insert(key, result);
	}

	return ApplyAirmoveResult(result, destAreaId, endOut, whyOut);
}

bool PlanWalker::LaunchAtEdge(const Vector &launchFloor, const Vector &launchDir,
	uint32_t fromId) const
{
	if (!m_traces)
		return false;

	bool duckLaunch = (fromId < m_areas->size()) && (*m_areas)[fromId].crouch;
	const PmSim::PmTraceFn &trace = duckLaunch ? m_traces->duckTrace : m_traces->standTrace;
	if (!trace)
		return false;

	Vector start = launchFloor + launchDir * PmSim::kEdgeProbeAhead;
	Vector end(start.x, start.y, start.z - PmSim::kEdgeProbeDrop);

	return trace(start, end).fraction == 1.f;
}

float PlanWalker::RunupDistance(const Vector &launchPoint, const Vector &back, float want,
	uint32_t fromId, uint32_t &runupAreaOut, size_t &gatesKeptOut) const
{
	runupAreaOut = fromId;
	gatesKeptOut = m_pendingGates.size();

	uint32_t area = fromId;
	Vector pos = launchPoint;
	float total = 0.f;
	float remaining = want;
	size_t bound = m_pendingGates.size();

	while (true)
	{
		float step = ClampTravelInsideArea((*m_areas)[area], pos, back, remaining, kLaunchInset);
		if (step >= remaining)
		{
			total += remaining;
			break;
		}

		size_t gate = bound;
		while (gate > 0 && m_pendingGates[gate - 1].insideArea == area)
			--gate;

		if (gate == 0)
		{
			total += step;
			break;
		}

		uint32_t prev = m_pendingGates[gate - 1].insideArea;
		float toSeam = ClampTravelInsideArea((*m_areas)[area], pos, back, remaining, 0.f);
		Vector seamPos = pos + back * toSeam;

		if (prev >= m_areas->size() || (*m_areas)[prev].crouch ||
			!InAreaColumn(seamPos, (*m_areas)[prev], -kLaunchInset) ||
			!FloorsMeetAt((*m_areas)[area], (*m_areas)[prev], seamPos, kPlanRunupSeamStep))
		{
			total += step;
			break;
		}

		total += toSeam;
		remaining -= toSeam;
		pos = seamPos;
		area = prev;
		bound = gate - 1;
		runupAreaOut = prev;
		gatesKeptOut = gate - 1;
	}

	return total;
}

float PlanWalker::ReachBack(const Vector &from, const Vector &back, float want, uint32_t fromId,
	const std::vector<std::vector<uint32_t>> *adjacency, std::vector<uint32_t> *crossed) const
{
	const std::vector<std::vector<uint32_t>> *adj = adjacency ? adjacency : m_walkPreds;
	if (!adj || fromId >= m_areas->size() || want <= 0.f)
		return 0.f;

	std::vector<uint32_t> local;
	std::vector<uint32_t> &seen = crossed ? *crossed : local;
	seen.clear();
	uint32_t area = fromId;
	Vector pos = from;
	float total = 0.f;
	float remaining = want;

	while (true)
	{
		seen.push_back(area);

		float step = ClampTravelInsideArea((*m_areas)[area], pos, back, remaining, kLaunchInset);
		if (step >= remaining)
			return total + remaining;

		float toSeam = ClampTravelInsideArea((*m_areas)[area], pos, back, remaining, 0.f);
		Vector seamPos = pos + back * toSeam;

		uint32_t next = static_cast<uint32_t>(m_areas->size());
		if (area < adj->size())
		{
			for (uint32_t cand : (*adj)[area])
			{
				if (cand >= m_areas->size() || (*m_areas)[cand].crouch)
					continue;
				if (std::find(seen.begin(), seen.end(), cand) != seen.end())
					continue;
				if (!InAreaColumn(seamPos, (*m_areas)[cand], -kLaunchInset))
					continue;
				if (!FloorsMeetAt((*m_areas)[area], (*m_areas)[cand], seamPos, kPlanRunupSeamStep))
					continue;

				next = cand;
				break;
			}
		}

		if (next >= m_areas->size())
			return total + step;

		total += toSeam;
		remaining -= toSeam;
		pos = seamPos;
		area = next;
	}
}

bool PlanWalker::LegIsWalkable(const Vector &from, const Vector &to, uint32_t fromId) const
{
	Vector dir(to.x - from.x, to.y - from.y, 0.f);
	float len = dir.Length();
	if (len <= kPlanAirmoveRangeEps)
		return false;

	dir = dir * (1.f / len);

	return ReachBack(from, dir, len, fromId) >= len - kPlanAirmoveRangeEps;
}

int PlanWalker::AreaOfWaypoint(size_t index, uint32_t startArea) const
{
	if (index == 0)
		return static_cast<int>(startArea);

	const PlanWaypoint &wp = m_result[index];
	if (wp.targetAreaId < 0 || static_cast<size_t>(wp.targetAreaId) >= m_areas->size())
		return -1;

	uint32_t hint = static_cast<uint32_t>(wp.targetAreaId);
	int best = -1;
	float bestDz = kPlanRunupSeamStep;

	auto consider = [&](uint32_t id)
	{
		if (id >= m_areas->size())
			return;
		const PlanArea &area = (*m_areas)[id];
		if (!InAreaColumn(wp.point, area, -kPlanAirmoveRangeEps))
			return;
		Vector floor(wp.point.x, wp.point.y, AreaFloorZ(area, wp.point.x, wp.point.y));
		float dz = std::fabs(AtPlayerHeight(floor, id).z - wp.point.z);
		if (dz <= bestDz)
		{
			bestDz = dz;
			best = static_cast<int>(id);
		}
	};

	consider(hint);
	if (best >= 0)
		return best;

	for (const std::vector<std::vector<uint32_t>> *adj : { m_walkPreds, m_walkSuccs })
		if (adj && hint < adj->size())
			for (uint32_t id : (*adj)[hint])
				consider(id);

	return best;
}

bool PlanWalker::LegIsClear(size_t from, size_t to, uint32_t fromArea) const
{
	const Vector &a = m_result[from].point;
	const Vector &b = m_result[to].point;

	Vector dir(b.x - a.x, b.y - a.y, 0.f);
	float len = dir.Length();
	if (len <= kPlanAirmoveRangeEps)
		return false;

	dir = dir * (1.f / len);
	Vector side(-dir.y, dir.x, 0.f);

	std::vector<uint32_t> crossed;

	auto walk = [&](const Vector &p, const Vector &d, float want, uint32_t area)
	{
		if (ReachBack(p, d, want, area, m_walkSuccs, &crossed) < want - kPlanAirmoveRangeEps)
			return false;

		for (uint32_t id : crossed)
		{
			const PlanArea &ar = (*m_areas)[id];
			if (ar.door || ar.breakable || ar.elevator || ar.crouch ||
				ar.activation != crossbot_nav::ACT_NONE || ar.drift.x != 0.f || ar.drift.y != 0.f ||
				ar.drift.z != 0.f || AreaUnderWater(static_cast<int>(id)))
				return false;
		}
		return !crossed.empty();
	};

	if (!walk(a, dir, len, fromArea))
		return false;

	uint32_t endId = crossed.back();
	const PlanArea &end = (*m_areas)[endId];
	Vector endFloor(b.x, b.y, AreaFloorZ(end, b.x, b.y));
	if (std::fabs(AtPlayerHeight(endFloor, endId).z - b.z) > kPlanRunupSeamStep)
		return false;

	for (float s : { -1.f, 1.f })
	{
		Vector out = side * s;
		if (!walk(a, out, kPlanRelaxClearance, fromArea))
			return false;
		if (!walk(a + out * kPlanRelaxClearance, dir, len, crossed.back()))
			return false;
	}

	return true;
}

void PlanWalker::Relax(uint32_t startArea)
{
	if (!m_walkSuccs)
		return;

	auto plain = [](const PlanWaypoint &wp)
	{
		return wp.connIndex < 0 && !wp.requiresElevatorRide && !wp.requiresDoor && !wp.requiresBreak &&
			!wp.requiresButtonUse && !wp.requiresTriggerTouch && !wp.requiresTeleport &&
			!wp.requiresLadder && !wp.airmoveLaunch && !wp.requiresJump;
	};

	for (size_t i = 0; i + 2 < m_result.size(); ++i)
	{
		if (!plain(m_result[i]))
			continue;

		int area = AreaOfWaypoint(i, startArea);
		if (area < 0)
			continue;

		size_t runEnd = i + 1;
		while (runEnd + 1 < m_result.size() && plain(m_result[runEnd]) && plain(m_result[runEnd + 1]))
			++runEnd;

		size_t k = runEnd;
		while (k > i + 1 && !LegIsClear(i, k, static_cast<uint32_t>(area)))
			--k;

		if (k > i + 1)
			m_result.erase(m_result.begin() + static_cast<std::ptrdiff_t>(i + 1),
				m_result.begin() + static_cast<std::ptrdiff_t>(k));
	}
}

int PlanWalker::AreaContaining(const Vector &point) const
{
	for (size_t i = 0; i < m_areas->size(); ++i)
		if (PointInArea(point, (*m_areas)[i], 1.f))
			return static_cast<int>(i);

	return -1;
}

bool PlanWalker::PushActivationDetour(const PlanArea &fromArea, const PlanArea &toArea, uint32_t fromId)
{
	const PlanDetour *detour = FindDetour(toArea, fromId);
	if (!detour || detour->blocked || detour->points.empty())
		return false;

	bool tailCrouch = FlushFunnelTo(detour->points.front().point);

	for (size_t i = 0; i < detour->points.size(); ++i)
	{
		const PlanDetourPoint &dp = detour->points[i];

		PlanWaypoint stepWp;
		stepWp.point = dp.point;
		stepWp.targetAreaId = dp.areaId;
		stepWp.clearance = dp.clearance;
		stepWp.requiresCrouch = dp.crouch || (i == 0 && tailCrouch) || fromArea.crouch;

		if (i == detour->actIndex)
		{
			stepWp.activatorCenter = detour->actCenter;
			if (detour->actKind == crossbot_nav::ACT_TRIGGER)
			{
				stepWp.requiresTriggerTouch = true;
			}
			else
			{
				stepWp.requiresButtonUse = true;
				stepWp.buttonWaitTime = detour->waitSeconds;
			}
		}

		PushWaypoint(stepWp);
	}

	m_anchor = detour->points.back().point;
	m_anchorFromAirmove = false;
	return true;
}

bool PlanWalker::Cross(const PlanConnection &conn, uint32_t connIndex, uint32_t fromId, uint32_t toId)
{
	const PlanArea &fromArea = (*m_areas)[fromId];
	const PlanArea &toArea = (*m_areas)[toId];

	{
		const std::vector<float> *surfaces = m_traces ? m_traces->waterSurfaces : nullptr;
		if (crossbot_nav::ConnBlockedByWater(conn.connType,
				SubmergedAt(surfaces, conn.waterVolume, conn.submergeZ)))
		{
			++m_stats.waterRejected;
			return false;
		}
	}

	if (toArea.activation != crossbot_nav::ACT_NONE &&
		conn.connType != crossbot_nav::CONN_WALK &&
		conn.connType != crossbot_nav::CONN_ELEVATOR)
	{
		++m_stats.gatedRejected;
		return false;
	}

	if (conn.connType == crossbot_nav::CONN_ELEVATOR)
	{
		Vector board = AtPlayerHeight(conn.launchPos, fromId);
		Vector arrive = AtPlayerHeight(conn.landingPos, toId);

		if (toArea.activation != crossbot_nav::ACT_NONE && !PushActivationDetour(fromArea, toArea, fromId))
			return false;

		m_extraTime += conn.rideWaitSeconds;

		PlanWaypoint rideWp;
		rideWp.point = board;
		rideWp.requiresElevatorRide = true;
		rideWp.elevatorTravelTime = conn.rideSeconds;
		rideWp.elevatorWaitTime = conn.rideWaitSeconds;
		rideWp.targetAreaId = static_cast<int>(toId);
		rideWp.connIndex = static_cast<int>(connIndex);
		rideWp.requiresCrouch = FlushFunnelTo(board) || fromArea.crouch;
		PushWaypoint(rideWp);

		PlanWaypoint arriveWp;
		arriveWp.point = arrive;
		arriveWp.targetAreaId = static_cast<int>(toId);
		arriveWp.requiresCrouch = toArea.crouch;
		PushWaypoint(arriveWp, false);

		m_anchor = arriveWp.point;
		m_anchorFromAirmove = false;
		UpdateFrontier();
		return true;
	}

	if (conn.connType == crossbot_nav::CONN_SWIM)
	{
		bool fromA = (fromId == conn.areaA);

		Vector entry = AtPlayerHeight(fromA ? conn.launchPos : conn.landingPos, fromId);
		Vector exit = AtPlayerHeight(fromA ? conn.landingPos : conn.launchPos, toId);

		bool tailCrouch = FlushFunnelTo(entry);

		auto atTail = [&](const Vector &p)
		{
			return !m_result.empty() && (m_result.back().point - p).Length() <= kPlanSwimMergeDist;
		};

		if (!atTail(entry) && (exit - entry).Length() > kPlanSwimMergeDist)
		{
			PlanWaypoint entryWp;
			entryWp.point = entry;
			entryWp.targetAreaId = static_cast<int>(fromId);
			entryWp.connIndex = static_cast<int>(connIndex);
			entryWp.requiresCrouch = tailCrouch || fromArea.crouch;
			PushWaypoint(entryWp);
			tailCrouch = false;
		}

		if (atTail(exit))
		{
			PlanWaypoint &tail = m_result.back();
			tail.targetAreaId = static_cast<int>(toId);
			tail.connIndex = static_cast<int>(connIndex);
			tail.requiresCrouch = tail.requiresCrouch || tailCrouch || toArea.crouch;
		}
		else
		{
			PlanWaypoint exitWp;
			exitWp.point = exit;
			exitWp.targetAreaId = static_cast<int>(toId);
			exitWp.connIndex = static_cast<int>(connIndex);
			exitWp.requiresCrouch = tailCrouch || toArea.crouch;
			PushWaypoint(exitWp);
		}

		m_anchor = m_result.back().point;
		m_anchorFromAirmove = false;
		UpdateFrontier();
		return true;
	}

	if (conn.connType == crossbot_nav::CONN_TELEPORT)
	{
		Vector trigger = AtPlayerHeight(conn.launchPos, fromId);
		Vector dest = AtPlayerHeight(conn.landingPos, toId);

		bool tailCrouch = FlushFunnelTo(trigger);

		PlanWaypoint triggerWp;
		triggerWp.point = trigger;
		triggerWp.requiresTeleport = true;
		triggerWp.teleportDest = dest;
		triggerWp.targetAreaId = static_cast<int>(toId);
		triggerWp.connIndex = static_cast<int>(connIndex);
		triggerWp.requiresCrouch = tailCrouch || fromArea.crouch;
		PushWaypoint(triggerWp);

		PlanWaypoint destWp;
		destWp.point = dest;
		destWp.targetAreaId = static_cast<int>(toId);
		destWp.requiresCrouch = toArea.crouch;
		PushWaypoint(destWp, false);

		m_anchor = destWp.point;
		m_anchorFromAirmove = false;
		UpdateFrontier();
		return true;
	}

	if (conn.connType == crossbot_nav::CONN_LADDER)
	{
		bool up = (fromId == conn.areaA);
		Vector mount = AtPlayerHeight(up ? conn.launchPos : conn.gateB.left, fromId);
		Vector exit = AtPlayerHeight(up ? conn.landingPos : conn.launchPos, toId);

		bool tailCrouch = FlushFunnelTo(mount);

		PlanWaypoint mountWp;
		mountWp.point = mount;
		mountWp.requiresLadder = true;
		mountWp.ladderUp = up;
		mountWp.ladderYaw = conn.exitAngle;
		mountWp.ladderEntryYaw = up ? conn.exitAngle : conn.gateAYaw[0];
		mountWp.ladderEntrySpeed = up ? 0.f : conn.gateASpeed[0];
		mountWp.ladderSpeed = conn.exitSpeed;
		mountWp.ladderDismount = up ? conn.gateB.right : conn.launchPos;
		mountWp.ladderDismountYaw = up ? conn.gateAYaw[1] : conn.exitAngle + 180.f;
		mountWp.ladderJumpOff = up && std::fabs(YawDelta(conn.exitAngle, conn.gateAYaw[1])) > 135.f;
		mountWp.ladderExit = exit;
		mountWp.requiresCrouch = tailCrouch || fromArea.crouch ||
			(conn.connFlags & crossbot_nav::CONN_FLAG_NEEDS_CROUCH) != 0;
		mountWp.targetAreaId = static_cast<int>(toId);
		mountWp.connIndex = static_cast<int>(connIndex);
		PushWaypoint(mountWp);

		PlanWaypoint exitWp;
		exitWp.point = exit;
		exitWp.targetAreaId = static_cast<int>(toId);
		exitWp.requiresCrouch = toArea.crouch;
		PushWaypoint(exitWp, false);

		m_anchor = exitWp.point;
		m_anchorFromAirmove = false;
		UpdateFrontier();
		return true;
	}

	if (conn.connType == crossbot_nav::CONN_AIRMOVE)
	{
		if (m_traces && std::fabs(conn.gravity - m_traces->gravityMultiplier) > kPlanGravityEpsilon)
			return false;

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
			candSpeed = std::min(candSpeed, m_traces->physics.maxSpeed);

			Vector end;
			const char *why = "";
			simFrom = exitPoint;
			simAim = entryPoint;
			usedSim = JumpLandsIn(exitPoint, candYaw, candSpeed, jump, fromId, toId, needsCrouch,
				end, &why);

			if (!usedSim)
			{
				Vector candDir(std::cos(candYaw * kDegToRad), std::sin(candYaw * kDegToRad), 0.f);
				const Vector &toCentroid = (*m_areas)[toId].centroid;
				float want = (toCentroid.x - exitPoint.x) * candDir.x +
					(toCentroid.y - exitPoint.y) * candDir.y;
				float got = (exitPoint - end).Length2D();
				float rescaled = 0.f;
				if (want > kPlanAirmoveRangeEps)
				{
					if (conn.flightTime > kPlanAirmoveFlightEps)
						rescaled = std::min(want / conn.flightTime, m_traces->physics.maxSpeed);
					else if (got > kPlanAirmoveRangeEps)
						rescaled = std::min(candSpeed * (want / got), m_traces->physics.maxSpeed);
				}

				if (rescaled > kPlanAirmoveRangeEps &&
					std::fabs(rescaled - candSpeed) > kPlanAirmoveRespeedEps)
				{
					Vector rescaledEnd;
					const char *rescaledWhy = "";
					usedSim = JumpLandsIn(exitPoint, candYaw, rescaled, jump, fromId, toId,
						needsCrouch, rescaledEnd, &rescaledWhy);

					if (m_traces->dumpAirmoves)
					{
						std::printf("    airmove %u->%u rescale %.0f -> %.0f (want %.0f got %.0f): "
							"%s -> end (%.1f,%.1f,%.1f)\n",
							fromId, toId, candSpeed, rescaled, want, got, rescaledWhy,
							rescaledEnd.x, rescaledEnd.y, rescaledEnd.z);
					}

					if (usedSim)
					{
						candSpeed = rescaled;
						end = rescaledEnd;
						++m_stats.airmovesRescaled;
					}
				}
			}

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
		float accel = m_traces->physics.accelerate * m_traces->physics.maxSpeed;
		float ground = m_traces->physics.groundFriction * m_traces->physics.friction;
		bool atEdge = LaunchAtEdge(launch, launchDir, fromId);
		if (atEdge)
			ground *= m_traces->physics.edgeFriction;
		float needed = AccelDistance(0.f, launchSpeed, accel, ground, m_traces->physics.stopSpeed) +
			AirmoveLaunchLead(launchSpeed, m_traces->physics.frameTime);

		float runupBack = std::max(launchSpeed * kPlanAirmoveRunupTime, kPlanAirmoveRunupMin);
		runupBack = std::max(runupBack, needed);

		if (m_anchorFromAirmove)
		{
			float anchorBack = (launchPoint.x - m_anchor.x) * launchDir.x +
				(launchPoint.y - m_anchor.y) * launchDir.y;
			runupBack = std::min(runupBack, std::max(0.f, anchorBack));
		}

		uint32_t runupArea = fromId;
		size_t gatesKept = m_pendingGates.size();
		runupBack = RunupDistance(launchPoint, launchDir * -1.f, runupBack, fromId, runupArea,
			gatesKept);

		bool hasRunup = runupBack > kPlanAirmoveRunupEps;
		Vector runupPoint = launchPoint - launchDir * runupBack;

		if (hasRunup && runupArea != fromId)
		{
			const PlanArea &backArea = (*m_areas)[runupArea];
			runupPoint.z = AreaFloorZ(backArea, runupPoint.x, runupPoint.y);
			runupPoint = AtPlayerHeight(runupPoint, runupArea);
			m_pendingGates.resize(gatesKept);
			++m_stats.airmovesRunupExtended;
		}

		bool runupCrouch = FlushFunnelTo(hasRunup ? runupPoint : launchPoint);

		if (hasRunup)
		{
			PlanWaypoint runupWp;
			runupWp.point = runupPoint;
			runupWp.targetAreaId = static_cast<int>(runupArea);
			runupWp.requiresCrouch = runupCrouch || fromArea.crouch;
			PushWaypoint(runupWp);
		}

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
		launchWp.connIndex = static_cast<int>(connIndex);

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
		pl.needRunup = needed;
		pl.haveRunup = runupBack;
		pl.atEdge = atEdge;
		m_launches.push_back(pl);

		PushWaypoint(launchWp);

		PlanWaypoint landWp;
		landWp.point = AtPlayerHeight(land, toId);
		landWp.targetAreaId = static_cast<int>(toId);
		landWp.requiresCrouch = toArea.crouch;
		PushWaypoint(landWp, false);

		m_anchor = landWp.point;
		m_anchorFromAirmove = true;
		UpdateFrontier();
		return true;
	}

	bool needsStop = toArea.door || toArea.breakable ||
		(toArea.elevator && toArea.activation != crossbot_nav::ACT_NONE);
	if (!needsStop)
	{
		PushGate(conn, fromId, toId);
		UpdateFrontier();
		return true;
	}

	m_extraTime += toArea.doorWaitTime;

	const PlanGate *doorGate = GateFor(conn, toId);
	Vector doorPoint = doorGate ? GateMidpoint(*doorGate) : toArea.centroid;

	PlanWaypoint wp;
	wp.point = AtPlayerHeight(doorPoint, toId);
	wp.requiresDoor = toArea.door;
	wp.requiresBreak = toArea.breakable;
	wp.doorWaitTime = toArea.doorWaitTime;
	wp.targetAreaId = static_cast<int>(toId);
	wp.requiresCrouch = fromArea.crouch || toArea.crouch;

	if (toArea.activation != crossbot_nav::ACT_NONE && !PushActivationDetour(fromArea, toArea, fromId))
		return false;

	if (FlushFunnelTo(wp.point))
		wp.requiresCrouch = true;
	PushWaypoint(wp);
	m_anchor = wp.point;
	m_anchorFromAirmove = false;

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

void PlanWalker::Retract()
{
	if (!m_traces || m_launches.size() < 2)
		return;

	for (size_t k = 1; k < m_launches.size(); ++k)
	{
		PendingLaunch &prev = m_launches[k - 1];
		const PendingLaunch &later = m_launches[k];

		bool hasRunupNode = (later.index == prev.index + 3);
		if (!hasRunupNode && later.index != prev.index + 2)
			continue;

		float extra = later.needRunup - later.haveRunup;
		if (extra <= kPlanAirmoveRunupEps)
			continue;

		const Vector &launchPoint = m_result[prev.index].point;
		const Vector &landPoint = m_result[prev.index + 1].point;

		float got = (launchPoint - landPoint).Length2D();
		if (got <= kPlanAirmoveRangeEps)
		{
			++m_stats.airmovesRetractKept;
			continue;
		}

		Vector laterDir(std::cos(later.yaw * kDegToRad), std::sin(later.yaw * kDegToRad), 0.f);
		float align = ((landPoint.x - launchPoint.x) * laterDir.x +
			(landPoint.y - launchPoint.y) * laterDir.y) / got;

		if (align < kPlanAirmoveRetractAlign)
		{
			++m_stats.airmovesRetractKept;
			continue;
		}

		float want = got - extra / align;
		float pulled = prev.designSpeed * (want / got);

		if (want <= kPlanAirmoveRangeEps || prev.designSpeed - pulled < kPlanAirmoveRespeedEps)
		{
			++m_stats.airmovesRetractKept;
			continue;
		}

		Vector end;
		const char *why = "";
		bool ok = JumpLandsIn(prev.launchFloor, prev.yaw, pulled, prev.jump, prev.fromId, prev.toId,
			prev.needsCrouch, end, &why);

		uint32_t landArea = prev.toId;
		if (!ok)
		{
			int alt = AreaContaining(end);
			if (alt >= 0 && static_cast<uint32_t>(alt) != prev.toId &&
				!(*m_areas)[static_cast<size_t>(alt)].crouch &&
				LegIsWalkable(m_result[later.index].point, end, later.fromId))
			{
				landArea = static_cast<uint32_t>(alt);
				ok = true;
				why = "landed in neighbour";
			}
		}

		if (m_traces->dumpAirmoves)
		{
			std::printf("    airmove %u->%u retract %.0f -> %.0f (need %.1f have %.1f "
				"align %.2f): %s -> end (%.1f,%.1f,%.1f) area %u\n",
				prev.fromId, prev.toId, prev.designSpeed, pulled, later.needRunup,
				later.haveRunup, align, why, end.x, end.y, end.z, landArea);
		}

		if (!ok)
		{
			++m_stats.airmovesRetractKept;
			continue;
		}

		m_result[prev.index].launchSpeed = pulled;
		m_result[prev.index].launchSimTo = end;
		m_result[prev.index + 1].point = AtPlayerHeight(end, landArea);
		if (landArea != prev.toId)
		{
			m_result[prev.index + 1].targetAreaId = static_cast<int>(landArea);
			m_result[prev.index + 1].requiresCrouch = (*m_areas)[landArea].crouch;
		}
		prev.designSpeed = pulled;
		++m_stats.airmovesRetracted;

		if (!hasRunupNode)
			continue;

		PlanWaypoint &runupWp = m_result[later.index - 1];
		if (runupWp.targetAreaId < 0 || static_cast<size_t>(runupWp.targetAreaId) >= m_areas->size())
			continue;

		uint32_t runupArea = static_cast<uint32_t>(runupWp.targetAreaId);
		const PlanArea &backArea = (*m_areas)[runupArea];
		Vector moved = m_result[later.index].point - laterDir * later.needRunup;

		if (!InAreaColumn(moved, backArea, -kLaunchInset))
			continue;

		moved.z = AreaFloorZ(backArea, moved.x, moved.y);
		runupWp.point = AtPlayerHeight(moved, runupArea);
	}
}

void PlanWalker::Respeed()
{
	if (!m_traces || m_launches.empty())
		return;

	PlanProfile probe = BuildPathProfile(m_result, m_traces->physics, m_traces);

	for (const PendingLaunch &pl : m_launches)
	{
		float lead = AirmoveLaunchLead(pl.designSpeed, m_traces->physics.frameTime);
		float reached = ProfileLaunchReach(probe, pl.index, lead,
			pl.atEdge ? m_traces->physics.edgeFriction : 1.f, PmSim::kEdgeProbeAhead);
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

