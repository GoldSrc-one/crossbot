#include "teleport_connections.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <map>
#include <string>

#include "activation_detours.h"
#include "floor_support.h"
#include "hull_defs.h"
#include "nav_format.h"
#include "nav_math.h"
#include "pm_sim.h"

namespace navgen
{

namespace
{

constexpr int kSfTriggerNoClients = 2;

constexpr double kLandingInsideEpsilon = 1.0;
constexpr double kLandingFootprintTolerance = 1.0;
constexpr double kLandingFloorSlack = 1.0;

constexpr double kLandingLift = -kHumanHullMinZ + 1.0 - kStandProbeLift;

long long DirectedPairKey(int from, int to)
{
	return static_cast<long long>(from) * 1000000LL + static_cast<long long>(to);
}

const Entity *FindByTargetname(const std::vector<Entity> &entities, const std::string &name)
{
	if (name.empty())
		return nullptr;

	for (const Entity &ent : entities)
		if (ent.Get("targetname") == name)
			return &ent;

	return nullptr;
}

bool ResolveLanding(const std::vector<Area> &areas, const Vec3 &landing, int &areaOut, Vec3 &pointOut,
	double &dropOut)
{
	auto floorPoint = [&](const Area &area, double &zOut)
	{
		const Face *floor = FloorFaceUnder(area.polytope, landing.x, landing.y);
		if (!floor || Distance2DToWinding(floor->winding, landing.x, landing.y) >
			kLandingFootprintTolerance)
			return false;
		zOut = FloorHeightAt(*floor, landing.x, landing.y);
		return true;
	};

	for (const Area &area : areas)
	{
		if (!NavMath::PointInsidePlanes(landing, area.boundPlanes, kLandingInsideEpsilon))
			continue;

		double z = 0.0;
		if (!floorPoint(area, z))
			continue;

		areaOut = area.id;
		pointOut = Vec3(landing.x, landing.y, z);
		dropOut = std::max(landing.z - z, 0.0);
		return true;
	}

	int best = -1;
	double bestZ = 0.0;
	for (const Area &area : areas)
	{
		double z = 0.0;
		if (!floorPoint(area, z))
			continue;
		if (z > landing.z + kLandingFloorSlack)
			continue;
		if (best < 0 || z > bestZ)
		{
			best = area.id;
			bestZ = z;
		}
	}

	if (best < 0)
		return false;

	areaOut = best;
	pointOut = Vec3(landing.x, landing.y, bestZ);
	dropOut = std::max(landing.z - bestZ, 0.0);
	return true;
}

}

TeleportStats BuildTeleportConnections(const BspFile &bsp, int standHeadnode,
	const std::vector<BlockingVolume> &standBlockers, const std::vector<Entity> &entities,
	const std::vector<Area> &areas, std::vector<MergedConnection> &conns, bool dump)
{
	TeleportStats stats;
	std::map<long long, size_t> emitted;

	for (size_t i = 0; i < entities.size(); ++i)
	{
		const Entity &ent = entities[i];
		if (ent.Get("classname") != "trigger_teleport")
			continue;

		++stats.entities;

		if ((ParseInt(ent.Get("spawnflags"), 0) & kSfTriggerNoClients) != 0)
		{
			++stats.noClients;
			if (dump)
				std::printf("    entity %zu: SF_TRIGGER_NOCLIENTS, players cannot fire it  skip\n", i);
			continue;
		}

		if (!ent.Get("master").empty())
		{
			++stats.mastered;
			if (dump)
				std::printf("    entity %zu: gated by master \"%s\"  skip\n", i,
					ent.Get("master").c_str());
			continue;
		}

		int modelIndex = ParseBrushModelIndex(ent.Get("model"));
		if (modelIndex < 0 || static_cast<size_t>(modelIndex) >= bsp.Models().size())
		{
			++stats.unresolvedDest;
			if (dump)
				std::printf("    entity %zu: no brush model  skip\n", i);
			continue;
		}

		const Entity *dest = FindByTargetname(entities, ent.Get("target"));
		if (dest == nullptr)
		{
			++stats.unresolvedDest;
			if (dump)
				std::printf("    entity %zu: target \"%s\" matches no entity  skip\n", i,
					ent.Get("target").c_str());
			continue;
		}

		if (dest->Find("origin") == nullptr)
		{
			++stats.unresolvedDest;
			if (dump)
				std::printf("    entity %zu: target \"%s\" has no origin  skip\n", i,
					ent.Get("target").c_str());
			continue;
		}

		Vec3 landing = ParseEntityOrigin(dest->Get("origin"));
		landing.z += kLandingLift;

		int destArea = -1;
		Vec3 destPoint;
		double drop = 0.0;
		if (!ResolveLanding(areas, landing, destArea, destPoint, drop))
		{
			++stats.unresolvedDest;
			if (dump)
				std::printf("    entity %zu: landing (%.1f, %.1f, %.1f) is off the mesh  skip\n", i,
					landing.x, landing.y, landing.z);
			continue;
		}

		double fallSpeed = std::sqrt(2.0 * PmSim::kGravity * drop);
		double fallDamage = PmSim::FallDamageFor(static_cast<float>(fallSpeed));
		double flightTime = std::sqrt(2.0 * drop / PmSim::kGravity);

		const dmodel_t &model = bsp.Models()[static_cast<size_t>(modelIndex)];
		Vec3 origin = ParseEntityOrigin(ent.Get("origin"));

		ActivatorVolume act;
		act.kind = crossbot_nav::ACT_TRIGGER;
		act.mins = Vec3(model.mins[0], model.mins[1], model.mins[2]) + origin;
		act.maxs = Vec3(model.maxs[0], model.maxs[1], model.maxs[2]) + origin;
		act.center = ModelWorldCenter(model, origin);
		act.entityIndex = static_cast<int>(i);

		if (dump)
			std::printf("    entity %zu: trigger (%.0f, %.0f, %.0f)-(%.0f, %.0f, %.0f) -> \"%s\" "
				"lands area %d at (%.1f, %.1f, %.1f), drop %.0f, fall damage %.1f\n",
				i, act.mins.x, act.mins.y, act.mins.z, act.maxs.x, act.maxs.y, act.maxs.z,
				ent.Get("target").c_str(), destArea, destPoint.x, destPoint.y, destPoint.z, drop,
				fallDamage);

		std::vector<ServicePoint> sources = FindServicePoints(bsp, standHeadnode, standBlockers, areas,
			act, static_cast<int>(i), dump);

		for (const ServicePoint &sp : sources)
		{
			if (sp.areaId < 0 || sp.areaId == destArea)
				continue;

			MergedConnection mc;
			mc.areaA = sp.areaId;
			mc.areaB = destArea;
			mc.connType = crossbot_nav::CONN_TELEPORT;
			mc.connFlags = crossbot_nav::CONN_FLAG_ONE_WAY;
			mc.pointA = sp.point;
			mc.pointB = destPoint;
			mc.fallDamage = fallDamage;
			mc.fallHeight = drop;
			mc.flightTime = flightTime;

			long long key = DirectedPairKey(mc.areaA, mc.areaB);
			auto it = emitted.find(key);
			if (it != emitted.end())
			{
				MergedConnection &prev = conns[it->second];
				if (mc.fallDamage < prev.fallDamage - 0.01)
					prev = mc;
				continue;
			}

			emitted[key] = conns.size();
			conns.push_back(mc);
			++stats.conns;

			if (dump)
				std::printf("      + teleport %d -> %d from (%.1f, %.1f, %.1f)\n", mc.areaA, mc.areaB,
					mc.pointA.x, mc.pointA.y, mc.pointA.z);
		}

		if (dump && sources.empty())
			std::printf("      no area can walk into this trigger\n");
	}

	return stats;
}

}
