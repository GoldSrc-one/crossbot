#include "nav_loader.h"

#include <cstdio>
#include <cstring>
#include <vector>

#include <extdll.h>
#include <meta_api.h>

#include "../game/crossplay.h"
#include "bsp_checksum.h"
#include "nav_format.h"

namespace Nav
{

namespace
{

using namespace crossbot_nav;

template <typename T>
bool ReadArray(FILE *f, uint32_t count, std::vector<T> &out)
{
	out.resize(count);
	if (count == 0)
		return true;
	return std::fread(out.data(), sizeof(T), count, f) == count;
}

}

std::unique_ptr<NavMesh> LoadNavFile(const char *mapName)
{
	char navPath[512];
	snprintf(navPath, sizeof(navPath), "%s/addons/crossbot/nav/%s.nav", XPlay::HyperDir(), mapName);

	FILE *f = std::fopen(navPath, "rb");
	if (!f)
	{
		char msg[560];
		snprintf(msg, sizeof(msg), "CrossBot: no nav file at %s (bot will wander).\n", navPath);
		SERVER_PRINT(msg);
		return nullptr;
	}

	NavHeader header;
	if (std::fread(&header, sizeof(header), 1, f) != 1 ||
		std::memcmp(header.magic, kNavMagic, sizeof(header.magic)) != 0 ||
		header.version != kNavVersion)
	{
		char msg[560];
		snprintf(msg, sizeof(msg), "CrossBot: %s is not a valid crossbot_nav file, ignoring.\n", navPath);
		SERVER_PRINT(msg);
		std::fclose(f);
		return nullptr;
	}

	std::vector<NavAreaRecord> areaRecords;
	std::vector<NavConnectionRecord> connRecords;
	std::vector<uint32_t> areaConnIndex;
	std::vector<NavAreaPlane> areaPlanes;
	std::vector<NavActivationDetourRecord> detourRecords;
	std::vector<NavDetourPointRecord> detourPoints;
	std::vector<NavVantageRecord> vantageRecords;
	std::vector<NavWaterVolumeRecord> waterRecords;
	std::vector<NavCampRecord> campRecords;

	bool ok = ReadArray(f, header.areaCount, areaRecords) &&
		ReadArray(f, header.connCount, connRecords) &&
		ReadArray(f, header.areaConnIndexCount, areaConnIndex) &&
		ReadArray(f, header.areaPlaneCount, areaPlanes) &&
		ReadArray(f, header.detourCount, detourRecords) &&
		ReadArray(f, header.detourPointCount, detourPoints) &&
		ReadArray(f, header.vantageCount, vantageRecords) &&
		ReadArray(f, header.waterVolumeCount, waterRecords) &&
		ReadArray(f, header.campCount, campRecords);
	std::fclose(f);

	if (!ok)
	{
		char msg[560];
		snprintf(msg, sizeof(msg), "CrossBot: %s is truncated/corrupt, ignoring.\n", navPath);
		SERVER_PRINT(msg);
		return nullptr;
	}

	char bspPath[512];
	XPlay::FindMapBsp(mapName, bspPath, sizeof(bspPath));
	uint32_t liveChecksum = ComputeLiveBspChecksum(bspPath);
	if (liveChecksum == 0)
	{
		char msg[1024];
		snprintf(msg, sizeof(msg), "CrossBot: could not verify %s against live map %s (bot will wander).\n", navPath, bspPath);
		SERVER_PRINT(msg);
		return nullptr;
	}
	if (liveChecksum != header.bspChecksum)
	{
		char msg[1024];
		snprintf(msg, sizeof(msg), "CrossBot: %s is stale (checksum mismatch against %s), ignoring.\n", navPath, bspPath);
		SERVER_PRINT(msg);
		return nullptr;
	}

	auto mesh = std::make_unique<NavMesh>();
	mesh->mapName = mapName;
	mesh->designMaxSpeed = (header.designMaxSpeed > 1.f) ? header.designMaxSpeed : kNavDefaultDesignSpeed;

	mesh->areas.resize(areaRecords.size());
	for (size_t i = 0; i < areaRecords.size(); ++i)
	{
		const NavAreaRecord &rec = areaRecords[i];
		Area &area = mesh->areas[i];
		area.id = rec.id;
		area.mins = Vector(rec.mins[0], rec.mins[1], rec.mins[2]);
		area.maxs = Vector(rec.maxs[0], rec.maxs[1], rec.maxs[2]);
		area.centroid = Vector(rec.centroid[0], rec.centroid[1], rec.centroid[2]);
		area.areaFlags = rec.areaFlags;
		area.waterVolume = rec.waterVolume;
		area.submergeZ = rec.submergeZ;
		area.driftVelocity = Vector(rec.driftVelocity[0], rec.driftVelocity[1], rec.driftVelocity[2]);
		area.connIndices.reserve(rec.connCount);
		for (uint32_t k = 0; k < rec.connCount; ++k)
			area.connIndices.push_back(areaConnIndex[rec.connOffset + k]);

		area.planes.reserve(rec.planeCount);
		for (uint32_t k = 0; k < rec.planeCount; ++k)
		{
			const NavAreaPlane &p = areaPlanes[rec.planeOffset + k];
			AreaPlane plane;
			plane.normal = Vector(p.normal[0], p.normal[1], p.normal[2]);
			plane.dist = p.dist;
			area.planes.push_back(plane);
		}
	}

	mesh->conns.resize(connRecords.size());
	for (size_t i = 0; i < connRecords.size(); ++i)
	{
		const NavConnectionRecord &rec = connRecords[i];
		Connection &conn = mesh->conns[i];
		conn.areaA = rec.areaA;
		conn.areaB = rec.areaB;
		conn.connType = rec.connType;
		conn.connFlags = rec.connFlags;
		conn.costSeconds = rec.costSeconds;
		conn.travelSeconds = rec.travelSeconds;
		conn.drift = rec.drift;
		conn.exitSpeed = rec.exitSpeed;
		conn.exitAngle = rec.exitAngle;
		conn.jumpBackDist = rec.jumpBackDist;
		conn.flightTime = rec.flightTime;
		conn.launchPos = Vector(rec.launchPos[0], rec.launchPos[1], rec.launchPos[2]);
		conn.landingPos = Vector(rec.landingPos[0], rec.landingPos[1], rec.landingPos[2]);
		conn.rideSeconds = rec.rideSeconds;
		conn.rideSecondsMax = rec.rideSecondsMax;
		conn.rideWaitSeconds = rec.rideWaitSeconds;
		conn.gravity = rec.gravity;
		conn.waterVolume = rec.waterVolume;
		conn.submergeZ = rec.submergeZ;

		conn.gateA.clamped = (rec.gateValid & crossbot_nav::GATE_CLAMPED_A) != 0;
		conn.gateB.clamped = (rec.gateValid & crossbot_nav::GATE_CLAMPED_B) != 0;
		conn.gateA.valid = (rec.gateValid & crossbot_nav::GATE_VALID_A) != 0;
		conn.gateA.left = Vector(rec.gateA[0][0], rec.gateA[0][1], rec.gateA[0][2]);
		conn.gateA.right = Vector(rec.gateA[1][0], rec.gateA[1][1], rec.gateA[1][2]);
		conn.gateB.valid = (rec.gateValid & crossbot_nav::GATE_VALID_B) != 0;
		conn.gateB.left = Vector(rec.gateB[0][0], rec.gateB[0][1], rec.gateB[0][2]);
		conn.gateB.right = Vector(rec.gateB[1][0], rec.gateB[1][1], rec.gateB[1][2]);
		for (int k = 0; k < 2; ++k)
		{
			conn.gateAYaw[k] = rec.gateAYaw[k];
			conn.gateASpeed[k] = rec.gateASpeed[k];
		}
	}

	mesh->planAreas.resize(mesh->areas.size());
	for (size_t i = 0; i < mesh->areas.size(); ++i)
	{
		const Area &a = mesh->areas[i];
		const NavAreaRecord &rec = areaRecords[i];
		NavPlan::PlanArea &pa = mesh->planAreas[i];
		pa.centroid = a.centroid;
		pa.crouch = (a.areaFlags & crossbot_nav::NAV_AREA_CROUCH) != 0;
		pa.door = (a.areaFlags & crossbot_nav::NAV_AREA_DOOR) != 0;
		pa.breakable = (a.areaFlags & crossbot_nav::NAV_AREA_BREAKABLE) != 0;
		pa.elevator = (a.areaFlags & crossbot_nav::NAV_AREA_ELEVATOR) != 0;
		pa.activation = rec.activation;
		pa.doorWaitTime = rec.openSeconds;
		pa.waterVolume = rec.waterVolume;
		pa.submergeZ = rec.submergeZ;
		pa.drift = a.driftVelocity;

		pa.detours.reserve(rec.detourCount);
		for (uint32_t k = 0; k < rec.detourCount; ++k)
		{
			const NavActivationDetourRecord &drec = detourRecords[rec.detourOffset + k];
			NavPlan::PlanDetour pd;
			pd.fromAreaId = drec.fromAreaId;
			pd.actIndex = drec.actIndex;
			pd.actKind = drec.actKind;
			pd.actCenter = Vector(drec.actCenter[0], drec.actCenter[1], drec.actCenter[2]);
			pd.waitSeconds = drec.waitSeconds;
			pd.detourSeconds = drec.detourSeconds;
			pd.blocked = (drec.flags & crossbot_nav::DETOUR_FLAG_BLOCKED) != 0;

			pd.points.reserve(drec.pointCount);
			for (uint32_t p = 0; p < drec.pointCount; ++p)
			{
				const NavDetourPointRecord &prec = detourPoints[drec.pointOffset + p];
				NavPlan::PlanDetourPoint pp;
				pp.point = Vector(prec.pos[0], prec.pos[1], prec.pos[2]);
				pp.areaId = static_cast<int>(prec.areaId);
				pp.clearance = prec.clearance;
				pp.crouch = (prec.flags & crossbot_nav::DETOUR_PT_CROUCH) != 0;
				pd.points.push_back(pp);
			}

			pa.detours.push_back(pd);
		}

		pa.boundPlanes.reserve(a.planes.size());
		for (const AreaPlane &src : a.planes)
		{
			NavPlan::PlanPlane plane;
			plane.normal = src.normal;
			plane.dist = src.dist;
			pa.boundPlanes.push_back(plane);
		}
	}

	mesh->planConns.resize(mesh->conns.size());
	for (size_t i = 0; i < mesh->conns.size(); ++i)
	{
		const Connection &c = mesh->conns[i];
		NavPlan::PlanConnection &pc = mesh->planConns[i];
		pc.areaA = c.areaA;
		pc.areaB = c.areaB;
		pc.connType = c.connType;
		pc.connFlags = c.connFlags;
		pc.exitSpeed = c.exitSpeed;
		pc.exitAngle = c.exitAngle;
		pc.jumpBackDist = c.jumpBackDist;
		pc.flightTime = c.flightTime;
		pc.launchPos = c.launchPos;
		pc.landingPos = c.landingPos;
		pc.rideSeconds = c.rideSeconds;
		pc.rideSecondsMax = c.rideSecondsMax;
		pc.rideWaitSeconds = c.rideWaitSeconds;
		pc.gravity = c.gravity;
		pc.waterVolume = c.waterVolume;
		pc.submergeZ = c.submergeZ;
		pc.gateA = c.gateA;
		pc.gateB = c.gateB;
		for (int k = 0; k < 2; ++k)
		{
			pc.gateAYaw[k] = c.gateAYaw[k];
			pc.gateASpeed[k] = c.gateASpeed[k];
		}

	}

	mesh->waterVolumes.resize(waterRecords.size());
	for (size_t i = 0; i < waterRecords.size(); ++i)
	{
		const NavWaterVolumeRecord &rec = waterRecords[i];
		WaterVolume &vol = mesh->waterVolumes[i];
		vol.modelIndex = rec.modelIndex;
		vol.mins = Vector(rec.mins[0], rec.mins[1], rec.mins[2]);
		vol.maxs = Vector(rec.maxs[0], rec.maxs[1], rec.maxs[2]);
		vol.surfaceZ = rec.surfaceZ;
		vol.travel = Vector(rec.travel[0], rec.travel[1], rec.travel[2]);
		vol.moving = (rec.flags & crossbot_nav::WATERVOL_MOVING) != 0;
	}

	mesh->vantageField.spacing = header.vantageSpacing;
	mesh->vantageField.maxExposure = header.maxExposure;
	mesh->vantageField.vantages.resize(vantageRecords.size());
	for (size_t i = 0; i < vantageRecords.size(); ++i)
	{
		const NavVantageRecord &rec = vantageRecords[i];
		Vantage &v = mesh->vantageField.vantages[i];
		v.pos = Vector(rec.pos[0], rec.pos[1], rec.pos[2]);
		v.exposure = rec.exposure;
		v.watchCount = 0;
		for (size_t k = 0; k < kNavWatchDirs; ++k)
		{
			if (rec.watch[k].amount <= 0.f)
				continue;
			VantageWatch &wd = v.watch[v.watchCount++];
			wd.yaw = rec.watch[k].yaw;
			wd.pitch = rec.watch[k].pitch;
			wd.spread = rec.watch[k].spread;
			wd.amount = rec.watch[k].amount;
		}
	}
	BuildVantageGrid(mesh->vantageField);

	mesh->camps.resize(campRecords.size());
	for (size_t i = 0; i < campRecords.size(); ++i)
	{
		const NavCampRecord &rec = campRecords[i];
		CampSpot &spot = mesh->camps[i];
		spot.pos = Vector(rec.pos[0], rec.pos[1], rec.pos[2]);
		spot.yaw = rec.yaw;
		spot.pitch = rec.pitch;
		spot.score = rec.score;
		for (size_t k = 0; k < kNavCampArcs; ++k)
		{
			spot.stand[k] = rec.stand[k];
			spot.crouch[k] = rec.crouch[k];
		}
		spot.areaId = static_cast<int>(rec.areaId);
		spot.flags = rec.flags;
	}

	float exposureSum = 0.f;
	size_t exposureKnown = 0;
	for (size_t i = 0; i < mesh->conns.size(); ++i)
	{
		Connection &conn = mesh->conns[i];
		conn.vantageExposure = 1.f;

		if (i >= mesh->planConns.size() || mesh->vantageField.maxExposure <= 0.f)
			continue;

		Vector point = NavPlan::ConnectionCrossing(mesh->planAreas, mesh->planConns[i]);

		bool crouch = (conn.areaA < mesh->planAreas.size() && mesh->planAreas[conn.areaA].crouch) ||
			(conn.areaB < mesh->planAreas.size() && mesh->planAreas[conn.areaB].crouch);
		point.z += crouch ? kVantageDuckEyeLift : kVantageMeanEyeLift;

		VantageBlend blend;
		BlendVantage(mesh->vantageField, point, blend);
		if (blend.neighbors <= 0)
			continue;

		float exposure = blend.exposure / mesh->vantageField.maxExposure;
		if (exposure < 0.f)
			exposure = 0.f;
		if (exposure > 1.f)
			exposure = 1.f;

		conn.vantageExposure = exposure;
		exposureSum += exposure;
		++exposureKnown;
		if (exposure < mesh->minConnExposure)
			mesh->minConnExposure = exposure;
	}

	char msg[700];
	snprintf(msg, sizeof(msg),
		"CrossBot: loaded nav file %s (%zu areas, %zu connections, %zu vantages, %zu camps); "
		"connection exposure known %zu, mean %.3f, min %.3f.\n",
		navPath, mesh->areas.size(), mesh->conns.size(), mesh->vantageField.vantages.size(), mesh->camps.size(),
		exposureKnown, exposureKnown > 0 ? exposureSum / static_cast<float>(exposureKnown) : 1.f,
		mesh->minConnExposure);
	SERVER_PRINT(msg);

	return mesh;
}

}

