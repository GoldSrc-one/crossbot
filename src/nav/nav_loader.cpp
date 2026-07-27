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
		conn.pointA = Vector(rec.pointA[0], rec.pointA[1], rec.pointA[2]);
		conn.pointB = Vector(rec.pointB[0], rec.pointB[1], rec.pointB[2]);
		conn.connType = rec.connType;
		conn.connFlags = rec.connFlags;
		conn.traverseCost = rec.traverseCost;
	}

	char msg[700];
	snprintf(msg, sizeof(msg), "CrossBot: loaded nav file %s (%zu areas, %zu connections).\n",
		navPath, mesh->areas.size(), mesh->conns.size());
	SERVER_PRINT(msg);

	return mesh;
}

}

