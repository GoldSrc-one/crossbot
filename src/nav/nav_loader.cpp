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

} // namespace

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
	std::vector<NavPortalRecord> portalRecords;
	std::vector<uint32_t> areaPortalIndex;
	std::vector<NavSpecialEdgeRecord> specialEdgeRecords;
	std::vector<char> stringTable;
	std::vector<NavAreaPlane> areaPlanes;

	bool ok = ReadArray(f, header.areaCount, areaRecords) &&
		ReadArray(f, header.portalCount, portalRecords) &&
		ReadArray(f, header.areaPortalIndexCount, areaPortalIndex) &&
		ReadArray(f, header.specialEdgeCount, specialEdgeRecords) &&
		ReadArray(f, header.stringTableSize, stringTable) &&
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
		area.gravityScale = rec.gravityScale;
		area.doorActivation = rec.doorActivation;
		area.doorActivatorPos = Vector(rec.doorActivatorPos[0], rec.doorActivatorPos[1], rec.doorActivatorPos[2]);
		area.doorWaitTime = rec.doorWaitTime;
		area.portalIndices.reserve(rec.portalCount);
		for (uint32_t k = 0; k < rec.portalCount; ++k)
			area.portalIndices.push_back(areaPortalIndex[rec.portalOffset + k]);

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

	mesh->portals.resize(portalRecords.size());
	for (size_t i = 0; i < portalRecords.size(); ++i)
	{
		const NavPortalRecord &rec = portalRecords[i];
		Portal &portal = mesh->portals[i];
		portal.areaA = rec.areaA;
		portal.areaB = rec.areaB;
		portal.pointA = Vector(rec.pointA[0], rec.pointA[1], rec.pointA[2]);
		portal.pointB = Vector(rec.pointB[0], rec.pointB[1], rec.pointB[2]);
		portal.edgeType = rec.edgeType;
		portal.edgeFlags = rec.edgeFlags;
		portal.traverseCost = rec.traverseCost;
		portal.buttonPos = Vector(rec.buttonPos[0], rec.buttonPos[1], rec.buttonPos[2]);
		portal.travelTime = rec.travelTime;
	}

	mesh->specialEdges.resize(specialEdgeRecords.size());
	for (size_t i = 0; i < specialEdgeRecords.size(); ++i)
	{
		const NavSpecialEdgeRecord &rec = specialEdgeRecords[i];
		SpecialEdge &edge = mesh->specialEdges[i];
		edge.areaFrom = rec.areaFrom;
		edge.areaTo = rec.areaTo;
		edge.type = rec.type;
		edge.anchorFrom = Vector(rec.anchorFrom[0], rec.anchorFrom[1], rec.anchorFrom[2]);
		edge.anchorTo = Vector(rec.anchorTo[0], rec.anchorTo[1], rec.anchorTo[2]);
	}

	char msg[700];
	snprintf(msg, sizeof(msg), "CrossBot: loaded nav file %s (%zu areas, %zu portals, %zu special edges).\n",
		navPath, mesh->areas.size(), mesh->portals.size(), mesh->specialEdges.size());
	SERVER_PRINT(msg);

	return mesh;
}

} // namespace Nav
