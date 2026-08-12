#include "activation_detours.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "astar_graph.h"
#include "clipnode_tree.h"
#include "floor_support.h"
#include "hull_defs.h"
#include "nav_math.h"
#include "plan_model.h"
#include "plan_search.h"
#include "polytope.h"

namespace navgen
{

namespace
{

constexpr double kFloorFootprintTolerance = 1.0;
constexpr double kMinServiceArea = 1.0;
constexpr double kInsideEpsilon = 1.0;

const char *KindName(uint8_t kind)
{
	switch (kind)
	{
	case crossbot_nav::ACT_BUTTON: return "button";
	case crossbot_nav::ACT_TRIGGER: return "trigger";
	case crossbot_nav::ACT_ATTACK: return "attack";
	default: return "none";
	}
}

bool AreaCrouch(const Area &area)
{
	return (area.areaFlags & crossbot_nav::NAV_AREA_CROUCH) != 0;
}

double PlayerLift(const Area &area)
{
	return AreaCrouch(area) ? 0.0 : kStandProbeLift;
}

void HullFor(const Area &area, Vec3 &hullMins, Vec3 &hullMaxs)
{
	if (AreaCrouch(area))
	{
		hullMins = Vec3(kDuckClipHullMinX, kDuckClipHullMinY, kDuckClipHullMinZ);
		hullMaxs = Vec3(kDuckClipHullMaxX, kDuckClipHullMaxY, kDuckClipHullMaxZ);
		return;
	}
	hullMins = Vec3(kHumanHullMinX, kHumanHullMinY, kHumanHullMinZ);
	hullMaxs = Vec3(kHumanHullMaxX, kHumanHullMaxY, kHumanHullMaxZ);
}

bool BoxesOverlap(const Vec3 &aMins, const Vec3 &aMaxs, const Vec3 &bMins, const Vec3 &bMaxs)
{
	return aMins.x <= bMaxs.x && aMaxs.x >= bMins.x && aMins.y <= bMaxs.y && aMaxs.y >= bMins.y &&
		aMins.z <= bMaxs.z && aMaxs.z >= bMins.z;
}

bool StandablePoint(const BspFile &bsp, int standHeadnode, const std::vector<BlockingVolume> &blockers,
	const Area &area, double x, double y, Vec3 &floorOut, Vec3 &originOut)
{
	const Face *floor = FloorFaceUnder(area.polytope, x, y);
	if (!floor || Distance2DToWinding(floor->winding, x, y) > kFloorFootprintTolerance)
		return false;

	floorOut = Vec3(x, y, FloorHeightAt(*floor, x, y));
	if (!NavMath::PointInsidePlanes(floorOut, area.boundPlanes, kInsideEpsilon))
		return false;

	originOut = Vec3(x, y, floorOut.z + PlayerLift(area));
	if (PointInSolid(bsp, standHeadnode, blockers, originOut))
		return false;

	return true;
}

bool HullClearsBox(const Area &area, const Vec3 &mins, const Vec3 &maxs, const Vec3 &origin)
{
	Vec3 hullMins, hullMaxs;
	HullFor(area, hullMins, hullMaxs);

	Vec3 lo = mins - hullMaxs;
	Vec3 hi = maxs - hullMins;

	return !(origin.x > lo.x && origin.x < hi.x && origin.y > lo.y && origin.y < hi.y &&
		origin.z > lo.z && origin.z < hi.z);
}

bool ServiceableArea(const Area &area)
{
	return area.activation == crossbot_nav::ACT_NONE;
}

struct PendingDetour
{
	int areaId = -1;
	ActivationDetour detour;
};

Vector ToVector(const Vec3 &v)
{
	return Vector(static_cast<float>(v.x), static_cast<float>(v.y), static_cast<float>(v.z));
}

Vec3 ToVec3(const Vector &v)
{
	return Vec3(v.x, v.y, v.z);
}

bool EntryGatePoint(const std::vector<Area> &areas, const MergedConnection &conn, int fromId, Vec3 &out)
{
	bool fromIsA = (conn.areaA == fromId);

	if (fromIsA ? !conn.gateAValid : !conn.gateBValid)
		return false;

	Vec3 left = fromIsA ? conn.gateALeft : conn.gateBLeft;
	Vec3 right = fromIsA ? conn.gateARight : conn.gateBRight;
	Vec3 mid = (left + right) * 0.5;

	const Area &from = areas[static_cast<size_t>(fromId)];
	out = Vec3(mid.x, mid.y, mid.z + PlayerLift(from));
	return true;
}

double PolylineLength(const std::vector<NavPlan::PlanWaypoint> &wps)
{
	double total = 0.0;
	for (size_t i = 1; i < wps.size(); ++i)
		total += (ToVec3(wps[i].point) - ToVec3(wps[i - 1].point)).Length();
	return total;
}

bool UsableLeg(const NavPlan::PlannedRoute &route)
{
	if (!route.found || route.waypoints.empty())
		return false;

	for (const NavPlan::PlanWaypoint &wp : route.waypoints)
		if (wp.airmoveLaunch || wp.requiresLadder || wp.requiresTeleport)
			return false;

	return true;
}

void AppendLeg(const std::vector<NavPlan::PlanWaypoint> &wps, size_t first, int fallbackAreaId,
	ActivationDetour &detour)
{
	for (size_t i = first; i < wps.size(); ++i)
	{
		DetourPoint dp;
		dp.pos = ToVec3(wps[i].point);
		dp.areaId = wps[i].targetAreaId >= 0 ? wps[i].targetAreaId : fallbackAreaId;
		dp.clearance = wps[i].clearance;
		dp.crouch = wps[i].requiresCrouch;
		detour.points.push_back(dp);
	}
}

bool SolveDetour(const PlanModel &model, const std::vector<Area> &areas,
	const std::vector<ServicePoint> &service, uint32_t fromId, const Vec3 &gatePoint,
	ActivationDetour &detour)
{
	Vector gate = ToVector(gatePoint);

	bool haveBest = false;
	double bestLength = 0.0;
	ActivationDetour best;

	for (const ServicePoint &sp : service)
	{
		if (sp.areaId < 0 || static_cast<size_t>(sp.areaId) >= areas.size())
			continue;

		Vector target = ToVector(sp.point);

		NavPlan::PlannedRoute out = NavPlan::PlanRoute(model.areaConnIndices, model.graphConns,
			model.areas, model.conns, fromId, static_cast<uint32_t>(sp.areaId), gate, target);
		if (!UsableLeg(out))
			continue;

		NavPlan::PlannedRoute back = NavPlan::PlanRoute(model.areaConnIndices, model.graphConns,
			model.areas, model.conns, static_cast<uint32_t>(sp.areaId), fromId, target, gate);
		if (!UsableLeg(back))
			continue;

		double length = PolylineLength(out.waypoints) + PolylineLength(back.waypoints);
		if (haveBest && length >= bestLength)
			continue;

		ActivationDetour cand;
		cand.fromAreaId = detour.fromAreaId;
		cand.waitSeconds = detour.waitSeconds;
		cand.actKind = sp.kind;
		cand.actCenter = sp.actCenter;

		AppendLeg(out.waypoints, 0, static_cast<int>(fromId), cand);
		cand.actIndex = cand.points.size() - 1;
		AppendLeg(back.waypoints, 1, sp.areaId, cand);
		cand.points.back().areaId = static_cast<int>(fromId);

		cand.detourSeconds = length / static_cast<double>(NavAStar::kNavRunSpeed);

		best = cand;
		bestLength = length;
		haveBest = true;
	}

	if (!haveBest)
		return false;

	detour = best;
	return true;
}

void CollectTriggerPoints(const BspFile &bsp, int standHeadnode,
	const std::vector<BlockingVolume> &blockers, const std::vector<Area> &areas,
	const ActivatorVolume &act, int activatorIndex, std::vector<ServicePoint> &out, bool dump)
{
	for (size_t i = 0; i < areas.size(); ++i)
	{
		const Area &area = areas[i];
		if (!ServiceableArea(area))
			continue;

		Vec3 hullMins, hullMaxs;
		HullFor(area, hullMins, hullMaxs);

		Vec3 boxMins = act.mins - hullMaxs;
		Vec3 boxMaxs = act.maxs - hullMins;

		Vec3 areaMins, areaMaxs;
		AreaBounds(area, areaMins, areaMaxs);
		if (!BoxesOverlap(areaMins, areaMaxs, boxMins, boxMaxs))
			continue;

		for (const Face &f : area.polytope.faces)
		{
			if (!IsFloorFace(f))
				continue;

			Winding w = f.winding;
			Plane sides[4];
			sides[0].normal = Vec3(1.0, 0.0, 0.0);
			sides[0].dist = boxMins.x;
			sides[1].normal = Vec3(-1.0, 0.0, 0.0);
			sides[1].dist = -boxMaxs.x;
			sides[2].normal = Vec3(0.0, 1.0, 0.0);
			sides[2].dist = boxMins.y;
			sides[3].normal = Vec3(0.0, -1.0, 0.0);
			sides[3].dist = -boxMaxs.y;

			for (int s = 0; s < 4 && !w.empty(); ++s)
				w = ClipWinding(w, sides[s], true);

			if (w.size() < 3 || WindingAreaXY(w) <= kMinServiceArea)
				continue;

			Vec3 mid = WindingCentroid(w);

			Vec3 floorPoint, origin;
			if (!StandablePoint(bsp, standHeadnode, blockers, area, mid.x, mid.y, floorPoint, origin))
				continue;

			if (origin.z < boxMins.z || origin.z > boxMaxs.z)
			{
				if (dump)
					std::printf("      area %d: floor overlap but origin z %.1f outside touch z "
						"[%.1f, %.1f]  reject\n", static_cast<int>(i), origin.z, boxMins.z, boxMaxs.z);
				continue;
			}

			ServicePoint sp;
			sp.areaId = static_cast<int>(i);
			sp.point = origin;
			sp.kind = crossbot_nav::ACT_TRIGGER;
			sp.actCenter = act.center;
			sp.activatorIndex = activatorIndex;
			sp.margin = WindingAreaXY(w);
			out.push_back(sp);
			break;
		}
	}
}

void CollectButtonPoints(const BspFile &bsp, int standHeadnode,
	const std::vector<BlockingVolume> &blockers, const std::vector<Area> &areas,
	const ActivatorVolume &act, int activatorIndex, std::vector<ServicePoint> &out, bool dump)
{
	Vec3 reach(kUseRange, kUseRange, kUseRange);
	Vec3 reachMins = act.center - reach;
	Vec3 reachMaxs = act.center + reach;

	for (size_t i = 0; i < areas.size(); ++i)
	{
		const Area &area = areas[i];
		if (!ServiceableArea(area))
			continue;

		Vec3 areaMins, areaMaxs;
		AreaBounds(area, areaMins, areaMaxs);
		if (!BoxesOverlap(areaMins, areaMaxs, reachMins, reachMaxs))
			continue;

		double loX = std::max(areaMins.x, reachMins.x);
		double hiX = std::min(areaMaxs.x, reachMaxs.x);
		double loY = std::max(areaMins.y, reachMins.y);
		double hiY = std::min(areaMaxs.y, reachMaxs.y);

		std::vector<Vec3> samples;
		samples.push_back(Vec3(std::min(std::max(act.center.x, loX), hiX),
			std::min(std::max(act.center.y, loY), hiY), 0.0));
		for (double x = loX; x <= hiX + 1e-6; x += kServiceGridStep)
			for (double y = loY; y <= hiY + 1e-6; y += kServiceGridStep)
				samples.push_back(Vec3(x, y, 0.0));

		ServicePoint best;
		bool haveBest = false;

		for (const Vec3 &s : samples)
		{
			Vec3 floorPoint, origin;
			if (!StandablePoint(bsp, standHeadnode, blockers, area, s.x, s.y, floorPoint, origin))
				continue;

			double dist = (origin - act.center).Length();
			if (dist > kUseRange)
				continue;

			if (!HullClearsBox(area, act.mins, act.maxs, origin))
				continue;

			ServicePoint sp;
			sp.areaId = static_cast<int>(i);
			sp.point = origin;
			sp.kind = crossbot_nav::ACT_BUTTON;
			sp.actCenter = act.center;
			sp.activatorIndex = activatorIndex;
			sp.margin = kUseRange - dist;

			if (!haveBest || sp.margin > best.margin)
			{
				best = sp;
				haveBest = true;
			}
		}

		if (haveBest)
		{
			out.push_back(best);
		}
		else if (dump)
		{
			std::printf("      area %d: no standable sample within %.1f of centre  reject\n",
				static_cast<int>(i), kUseRange);
		}
	}
}

}

void AreaBounds(const Area &area, Vec3 &minsOut, Vec3 &maxsOut)
{
	minsOut = Vec3(1e30, 1e30, 1e30);
	maxsOut = Vec3(-1e30, -1e30, -1e30);

	for (const Face &f : area.polytope.faces)
	{
		for (const Vec3 &v : f.winding)
		{
			minsOut.x = std::min(minsOut.x, v.x);
			minsOut.y = std::min(minsOut.y, v.y);
			minsOut.z = std::min(minsOut.z, v.z);
			maxsOut.x = std::max(maxsOut.x, v.x);
			maxsOut.y = std::max(maxsOut.y, v.y);
			maxsOut.z = std::max(maxsOut.z, v.z);
		}
	}
}

std::vector<ServicePoint> FindServicePoints(const BspFile &bsp, int standHeadnode,
	const std::vector<BlockingVolume> &standBlockers, const std::vector<Area> &areas,
	const ActivatorVolume &act, int activatorIndex, bool dump)
{
	std::vector<ServicePoint> out;

	if (act.kind == crossbot_nav::ACT_TRIGGER)
		CollectTriggerPoints(bsp, standHeadnode, standBlockers, areas, act, activatorIndex, out, dump);
	else
		CollectButtonPoints(bsp, standHeadnode, standBlockers, areas, act, activatorIndex, out, dump);

	std::sort(out.begin(), out.end(), [](const ServicePoint &a, const ServicePoint &b)
	{
		return a.margin > b.margin;
	});

	return out;
}

DetourStats BuildActivationDetours(const BspFile &bsp, int standHeadnode,
	const std::vector<BlockingVolume> &standBlockers, const std::vector<Entity> &entities,
	const std::vector<PassableVolume> &passables, std::vector<Area> &areas,
	const std::vector<MergedConnection> &conns, bool dump)
{
	DetourStats stats;

	MergeResult snapshot;
	snapshot.areas = areas;
	snapshot.conns = conns;
	PlanModel model = BuildPlanModel(snapshot, true);

	std::vector<PendingDetour> pending;

	for (size_t a = 0; a < areas.size(); ++a)
	{
		Area &area = areas[a];
		if (area.activation == crossbot_nav::ACT_NONE)
			continue;

		++stats.gatedAreas;

		if (dump)
		{
			std::string classname;
			std::string targetname;
			if (area.activationEntity >= 0 && static_cast<size_t>(area.activationEntity) < entities.size())
			{
				classname = entities[static_cast<size_t>(area.activationEntity)].Get("classname");
				targetname = entities[static_cast<size_t>(area.activationEntity)].Get("targetname");
			}
			std::printf("activators: area %d %s \"%s\" (%.2fs open, %zu activators)\n",
				static_cast<int>(a), classname.c_str(), targetname.c_str(), area.openSeconds,
				area.activators.size());
		}

		std::vector<ServicePoint> service;

		for (size_t k = 0; k < area.activators.size(); ++k)
		{
			const ActivatorVolume &act = area.activators[k];
			++stats.activators;

			if (dump)
				std::printf("    %s [%d] centre (%.1f, %.1f, %.1f) box (%.0f,%.0f,%.0f)-(%.0f,%.0f,%.0f)\n",
					KindName(act.kind), act.entityIndex, act.center.x, act.center.y, act.center.z,
					act.mins.x, act.mins.y, act.mins.z, act.maxs.x, act.maxs.y, act.maxs.z);

			std::vector<ServicePoint> found = FindServicePoints(bsp, standHeadnode, standBlockers, areas,
				act, static_cast<int>(k), dump);

			if (dump)
			{
				for (const ServicePoint &sp : found)
					std::printf("      service (%.1f, %.1f, %.1f) area %d margin %.1f  KEEP\n",
						sp.point.x, sp.point.y, sp.point.z, sp.areaId, sp.margin);
			}

			service.insert(service.end(), found.begin(), found.end());
		}

		stats.servicePoints += service.size();
		if (service.empty())
		{
			++stats.areasWithoutService;
			if (dump)
				std::printf("    no service point found for any activator\n");
		}

		for (size_t ci = 0; ci < conns.size(); ++ci)
		{
			const MergedConnection &conn = conns[ci];

			int fromId = -1;
			if (conn.areaA == static_cast<int>(a))
				fromId = conn.areaB;
			else if (conn.areaB == static_cast<int>(a))
				fromId = conn.areaA;
			else
				continue;

			bool ride = conn.connType == crossbot_nav::CONN_ELEVATOR;
			if (conn.connType != crossbot_nav::CONN_WALK && !ride)
			{
				++stats.nonWalkEntries;
				if (dump)
					std::printf("  detour area %d <- %d : rejected, entry connection is not walk\n",
						static_cast<int>(a), fromId);
				continue;
			}

			if ((conn.connFlags & crossbot_nav::CONN_FLAG_ONE_WAY) && conn.areaB != static_cast<int>(a))
				continue;

			++stats.entries;

			ActivationDetour detour;
			detour.fromAreaId = fromId;
			detour.waitSeconds = area.openSeconds;

			Vec3 gatePoint;
			if (ride)
				gatePoint = conn.pointA;
			else if (!EntryGatePoint(areas, conn, fromId, gatePoint))
			{
				detour.blocked = true;
				++stats.blockedNoGate;
				pending.push_back({static_cast<int>(a), detour});
				if (dump)
					std::printf("  detour area %d <- %d : BLOCKED (no gate on the entering side)\n",
						static_cast<int>(a), fromId);
				continue;
			}

			if (!SolveDetour(model, areas, service, static_cast<uint32_t>(fromId), gatePoint, detour))
			{
				detour.blocked = true;
				++stats.blockedNoRoute;
				pending.push_back({static_cast<int>(a), detour});
				if (dump)
					std::printf("  detour area %d <- %d : BLOCKED (no route to any service point)\n",
						static_cast<int>(a), fromId);
				continue;
			}

			++stats.solved;
			if (detour.points.size() <= 2)
				++stats.degenerate;

			pending.push_back({static_cast<int>(a), detour});

			if (dump)
			{
				std::printf("  detour area %d <- %d : %s, %zu pts, %.1fs\n", static_cast<int>(a), fromId,
					KindName(detour.actKind), detour.points.size(), detour.detourSeconds);
				for (size_t p = 0; p < detour.points.size(); ++p)
					std::printf("      [%zu] (%.1f, %.1f, %.1f) area %d clr %.1f%s\n", p,
						detour.points[p].pos.x, detour.points[p].pos.y, detour.points[p].pos.z,
						detour.points[p].areaId, detour.points[p].clearance,
						p == detour.actIndex ? (detour.actKind == crossbot_nav::ACT_TRIGGER
							? "  <-- TRIGGER TOUCH" : "  <-- BUTTON USE") : "");
			}
		}
	}

	for (const PendingDetour &pd : pending)
		areas[static_cast<size_t>(pd.areaId)].detours.push_back(pd.detour);

	return stats;
}

}
