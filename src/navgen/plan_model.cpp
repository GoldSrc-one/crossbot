#include "plan_model.h"

namespace navgen
{

namespace
{

Vector ToVector(const Vec3 &v)
{
	return Vector(static_cast<float>(v.x), static_cast<float>(v.y), static_cast<float>(v.z));
}

}

PlanModel BuildPlanModel(const MergeResult &merged, bool walkOnly)
{
	PlanModel model;

	model.areas.resize(merged.areas.size());
	for (size_t i = 0; i < merged.areas.size(); ++i)
	{
		NavPlan::PlanArea &pa = model.areas[i];
		const Area &area = merged.areas[i];

		pa.centroid = ToVector(area.polytope.Centroid());
		pa.crouch = (area.areaFlags & crossbot_nav::NAV_AREA_CROUCH) != 0;
		pa.door = (area.areaFlags & crossbot_nav::NAV_AREA_DOOR) != 0;
		pa.breakable = (area.areaFlags & crossbot_nav::NAV_AREA_BREAKABLE) != 0;
		pa.activation = area.activation;
		pa.doorWaitTime = static_cast<float>(area.openSeconds);

		pa.detours.reserve(area.detours.size());
		for (const ActivationDetour &d : area.detours)
		{
			NavPlan::PlanDetour pd;
			pd.fromAreaId = static_cast<uint32_t>(d.fromAreaId);
			pd.actIndex = d.actIndex;
			pd.actKind = d.actKind;
			pd.waitSeconds = static_cast<float>(d.waitSeconds);
			pd.detourSeconds = static_cast<float>(d.detourSeconds);
			pd.blocked = d.blocked;

			pd.points.reserve(d.points.size());
			for (const DetourPoint &p : d.points)
			{
				NavPlan::PlanDetourPoint pp;
				pp.point = ToVector(p.pos);
				pp.areaId = p.areaId;
				pp.clearance = static_cast<float>(p.clearance);
				pp.crouch = p.crouch;
				pd.points.push_back(pp);
			}

			pa.detours.push_back(pd);
		}

		pa.boundPlanes.reserve(area.boundPlanes.size());
		for (const Plane &p : area.boundPlanes)
		{
			NavPlan::PlanPlane pp;
			pp.normal = Vector(static_cast<float>(p.normal.x), static_cast<float>(p.normal.y),
				static_cast<float>(p.normal.z));
			pp.dist = static_cast<float>(p.dist);
			pa.boundPlanes.push_back(pp);
		}
	}

	model.graphConns.resize(merged.conns.size());
	model.conns.resize(merged.conns.size());
	model.areaConnIndices.resize(merged.areas.size());
	for (size_t i = 0; i < merged.conns.size(); ++i)
	{
		const MergedConnection &c = merged.conns[i];
		Vector pointA = ToVector(c.pointA);
		Vector pointB = ToVector(c.pointB);

		NavAStar::Connection &gc = model.graphConns[i];
		gc.areaA = static_cast<uint32_t>(c.areaA);
		gc.areaB = static_cast<uint32_t>(c.areaB);
		gc.oneWay = (c.connFlags & crossbot_nav::CONN_FLAG_ONE_WAY) != 0;
		gc.costSeconds = ConnectionCostSeconds(c);

		NavPlan::PlanConnection &pc = model.conns[i];
		pc.areaA = gc.areaA;
		pc.areaB = gc.areaB;
		pc.connType = c.connType;
		pc.gateA.valid = c.gateAValid;
		pc.gateA.clamped = c.gateAClamped;
		pc.gateA.left = ToVector(c.gateALeft);
		pc.gateA.right = ToVector(c.gateARight);
		pc.gateB.valid = c.gateBValid;
		pc.gateB.clamped = c.gateBClamped;
		pc.gateB.left = ToVector(c.gateBLeft);
		pc.gateB.right = ToVector(c.gateBRight);
		for (int k = 0; k < 2; ++k)
		{
			pc.gateAYaw[k] = static_cast<float>(c.gateAYaw[k]);
			pc.gateASpeed[k] = static_cast<float>(c.gateASpeed[k]);
		}
		pc.connFlags = c.connFlags;
		pc.exitSpeed = static_cast<float>(c.exitSpeed);
		pc.exitAngle = static_cast<float>(c.exitAngle);
		pc.jumpBackDist = static_cast<float>(c.jumpBackDist);
		pc.flightTime = static_cast<float>(c.flightTime);
		pc.launchPos = pointA;
		pc.landingPos = pointB;

		if (walkOnly && c.connType != crossbot_nav::CONN_WALK)
			continue;

		model.areaConnIndices[static_cast<size_t>(c.areaA)].push_back(static_cast<uint32_t>(i));
		model.areaConnIndices[static_cast<size_t>(c.areaB)].push_back(static_cast<uint32_t>(i));
	}

	for (size_t i = 0; i < model.conns.size(); ++i)
		model.graphConns[i].crossing = NavPlan::ConnectionCrossing(model.areas, model.conns[i]);

	return model;
}

}
