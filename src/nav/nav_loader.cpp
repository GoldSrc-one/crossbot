#include "nav_loader.h"

#include <cstdio>
#include <cstring>
#include <vector>

#include <extdll.h>
#include <meta_api.h>

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
	char gamedir[256] = {0};
	g_engfuncs.pfnGetGameDir(gamedir);

	char navPath[512];
	snprintf(navPath, sizeof(navPath), "%s/crossbot_data/nav/%s.nav", gamedir, mapName);

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

	bool ok = ReadArray(f, header.areaCount, areaRecords) &&
		ReadArray(f, header.connCount, connRecords) &&
		ReadArray(f, header.areaConnIndexCount, areaConnIndex) &&
		ReadArray(f, header.areaPlaneCount, areaPlanes);
	std::fclose(f);

	if (!ok)
	{
		char msg[560];
		snprintf(msg, sizeof(msg), "CrossBot: %s is truncated/corrupt, ignoring.\n", navPath);
		SERVER_PRINT(msg);
		return nullptr;
	}

	char bspPath[512];
	snprintf(bspPath, sizeof(bspPath), "%s/maps/%s.bsp", gamedir, mapName);
	uint32_t liveChecksum = ComputeLiveBspChecksum(bspPath);
	if (liveChecksum == 0)
	{
		char msg[1100];
		snprintf(msg, sizeof(msg), "CrossBot: could not verify %s against live map %s (bot will wander).\n", navPath, bspPath);
		SERVER_PRINT(msg);
		return nullptr;
	}
	if (liveChecksum != header.bspChecksum)
	{
		char msg[1100];
		snprintf(msg, sizeof(msg), "CrossBot: %s is stale (checksum mismatch against %s), ignoring.\n", navPath, bspPath);
		SERVER_PRINT(msg);
		return nullptr;
	}

	auto mesh = std::make_unique<NavMesh>();
	mesh->mapName = mapName;

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
		area.doorActivation = rec.doorActivation;
		area.doorActivatorPos = Vector(rec.doorActivatorPos[0], rec.doorActivatorPos[1], rec.doorActivatorPos[2]);
		area.doorWaitTime = rec.doorWaitTime;
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
		conn.traverseCost = rec.traverseCost;
		conn.exitSpeed = rec.exitSpeed;
		conn.exitAngle = rec.exitAngle;
		conn.jumpBackDist = rec.jumpBackDist;
		conn.flightTime = rec.flightTime;
		conn.launchPos = Vector(rec.launchPos[0], rec.launchPos[1], rec.launchPos[2]);
		conn.landingPos = Vector(rec.landingPos[0], rec.landingPos[1], rec.landingPos[2]);

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
		NavPlan::PlanArea &pa = mesh->planAreas[i];
		pa.centroid = a.centroid;
		pa.crouch = (a.areaFlags & crossbot_nav::NAV_AREA_CROUCH) != 0;
		pa.door = (a.areaFlags & crossbot_nav::NAV_AREA_DOOR) != 0;
		pa.doorActivation = a.doorActivation;
		pa.doorActivatorPos = a.doorActivatorPos;
		pa.doorWaitTime = a.doorWaitTime;

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
		pc.gateA = c.gateA;
		pc.gateB = c.gateB;
		for (int k = 0; k < 2; ++k)
		{
			pc.gateAYaw[k] = c.gateAYaw[k];
			pc.gateASpeed[k] = c.gateASpeed[k];
		}

	}

	char msg[700];
	snprintf(msg, sizeof(msg), "CrossBot: loaded nav file %s (%zu areas, %zu connections).\n",
		navPath, mesh->areas.size(), mesh->conns.size());
	SERVER_PRINT(msg);

	return mesh;
}

}

