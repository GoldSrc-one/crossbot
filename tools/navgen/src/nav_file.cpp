#include "nav_file.h"

#include <cstdio>
#include <cstring>
#include <limits>
#include <stdexcept>

#include "nav_format.h"

namespace navgen
{

namespace
{

using namespace crossbot_nav;

void ComputeBounds(const Polytope &poly, float mins[3], float maxs[3], float centroid[3])
{
	Vec3 lo(1e30, 1e30, 1e30);
	Vec3 hi(-1e30, -1e30, -1e30);
	Vec3 sum(0, 0, 0);
	int count = 0;

	for (const Face &f : poly.faces)
	{
		for (const Vec3 &v : f.winding)
		{
			lo.x = std::min(lo.x, v.x);
			lo.y = std::min(lo.y, v.y);
			lo.z = std::min(lo.z, v.z);
			hi.x = std::max(hi.x, v.x);
			hi.y = std::max(hi.y, v.y);
			hi.z = std::max(hi.z, v.z);
			sum = sum + v;
			++count;
		}
	}

	mins[0] = static_cast<float>(lo.x);
	mins[1] = static_cast<float>(lo.y);
	mins[2] = static_cast<float>(lo.z);
	maxs[0] = static_cast<float>(hi.x);
	maxs[1] = static_cast<float>(hi.y);
	maxs[2] = static_cast<float>(hi.z);

	Vec3 c = (count > 0) ? sum * (1.0 / count) : Vec3();
	centroid[0] = static_cast<float>(c.x);
	centroid[1] = static_cast<float>(c.y);
	centroid[2] = static_cast<float>(c.z);
}

uint16_t ToU16Count(size_t n, const char *what)
{
	if (n > std::numeric_limits<uint16_t>::max())
		throw std::runtime_error(std::string("nav file overflow: too many ") + what + " for one area");
	return static_cast<uint16_t>(n);
}

void WriteRaw(FILE *f, const void *data, size_t bytes, const char *what)
{
	if (bytes > 0 && std::fwrite(data, 1, bytes, f) != bytes)
		throw std::runtime_error(std::string("failed writing ") + what);
}

} // namespace

void WriteNavFile(const std::string &path, const std::string &mapName, uint32_t bspChecksum,
	const std::vector<Area> &areas, const std::vector<MergedPortal> &portals,
	const std::vector<TaggedSpecialEdge> &specialEdges, const std::vector<TacticalScore> &tacticalScores,
	const std::vector<bool> &crouchAreas, const std::vector<DoorAreaInfo> &doorAreaInfo)
{
	bool haveTactical = tacticalScores.size() == areas.size();
	bool haveCrouch = crouchAreas.size() == areas.size();
	bool haveDoor = doorAreaInfo.size() == areas.size();
	FILE *f = std::fopen(path.c_str(), "wb");
	if (!f)
		throw std::runtime_error("cannot open nav file for writing: " + path);

	// AreaPortalIndex: each portal contributes its index to both its areaA's
	// and areaB's slice, grouped contiguously per area (offset/count).
	std::vector<std::vector<uint32_t>> perArea(areas.size());
	for (size_t pi = 0; pi < portals.size(); ++pi)
	{
		perArea[static_cast<size_t>(portals[pi].areaA)].push_back(static_cast<uint32_t>(pi));
		perArea[static_cast<size_t>(portals[pi].areaB)].push_back(static_cast<uint32_t>(pi));
	}

	std::vector<NavAreaRecord> areaRecords(areas.size());
	std::vector<uint32_t> areaPortalIndex;
	std::vector<NavAreaPlane> areaPlanes;

	for (size_t i = 0; i < areas.size(); ++i)
	{
		NavAreaRecord &rec = areaRecords[i];
		rec.id = static_cast<uint32_t>(areas[i].id);
		ComputeBounds(areas[i].polytope, rec.mins, rec.maxs, rec.centroid);
		rec.areaFlags = NAV_AREA_NONE; // water/gravity flags land in later milestones (2g/2h)
		rec.gravityScale = 1.0f;
		if (haveCrouch && crouchAreas[i])
			rec.areaFlags |= NAV_AREA_CROUCH;
		rec.doorActivation = DOOR_ACTIVATION_TOUCH;
		rec.doorActivatorPos[0] = rec.doorActivatorPos[1] = rec.doorActivatorPos[2] = 0.0f;
		rec.doorWaitTime = 0.0f;
		if (haveDoor && doorAreaInfo[i].isDoor)
		{
			const DoorAreaInfo &d = doorAreaInfo[i];
			rec.areaFlags |= NAV_AREA_DOOR;
			rec.doorActivation = d.needsButton ? DOOR_ACTIVATION_BUTTON
				: d.needsTrigger              ? DOOR_ACTIVATION_TRIGGER
											   : DOOR_ACTIVATION_TOUCH;
			rec.doorActivatorPos[0] = static_cast<float>(d.activatorPos.x);
			rec.doorActivatorPos[1] = static_cast<float>(d.activatorPos.y);
			rec.doorActivatorPos[2] = static_cast<float>(d.activatorPos.z);
			// Not a reverse-engineered per-door value -- covers a typical
			// func_door's open animation (sliding or a func_door_rotating's
			// ~90-degree swing) with margin, matching this project's
			// existing EDGE_ELEVATOR/button-door travelTime convention.
			rec.doorWaitTime = 1.5f;
		}
		if (haveTactical)
		{
			const TacticalScore &t = tacticalScores[i];
			rec.exposureScore = static_cast<float>(t.exposureScore);
			rec.concealmentScore = static_cast<float>(t.concealmentScore);
			rec.dangerScore = static_cast<float>(t.dangerScore);
			if (t.isVantage) rec.areaFlags |= NAV_AREA_VANTAGE;
			if (t.isCamp) rec.areaFlags |= NAV_AREA_CAMP;
			if (t.isChokepointCover) rec.areaFlags |= NAV_AREA_CHOKEPOINT_COVER;
		}
		else
		{
			rec.exposureScore = 0.0f;
			rec.concealmentScore = 0.0f;
			rec.dangerScore = 0.0f;
		}
		rec.portalOffset = ToU16Count(areaPortalIndex.size(), "area-portal-index offset");
		rec.portalCount = ToU16Count(perArea[i].size(), "portals-per-area");

		for (uint32_t idx : perArea[i])
			areaPortalIndex.push_back(idx);

		rec.planeOffset = ToU16Count(areaPlanes.size(), "area-plane offset");
		rec.planeCount = ToU16Count(areas[i].polytope.faces.size(), "planes-per-area");

		for (const Face &face : areas[i].polytope.faces)
		{
			NavAreaPlane plane;
			plane.normal[0] = static_cast<float>(face.plane.normal.x);
			plane.normal[1] = static_cast<float>(face.plane.normal.y);
			plane.normal[2] = static_cast<float>(face.plane.normal.z);
			plane.dist = static_cast<float>(face.plane.dist);
			areaPlanes.push_back(plane);
		}
	}

	std::vector<NavPortalRecord> portalRecords(portals.size());
	for (size_t i = 0; i < portals.size(); ++i)
	{
		const MergedPortal &p = portals[i];
		NavPortalRecord &rec = portalRecords[i];
		rec.areaA = static_cast<uint32_t>(p.areaA);
		rec.areaB = static_cast<uint32_t>(p.areaB);
		rec.pointA[0] = static_cast<float>(p.pointA.x);
		rec.pointA[1] = static_cast<float>(p.pointA.y);
		rec.pointA[2] = static_cast<float>(p.pointA.z);
		rec.pointB[0] = static_cast<float>(p.pointB.x);
		rec.pointB[1] = static_cast<float>(p.pointB.y);
		rec.pointB[2] = static_cast<float>(p.pointB.z);
		rec.edgeType = p.edgeType;
		rec.edgeFlags = p.edgeFlags;
		// Jump edges are a proximity heuristic, not a validated path -- bias
		// A* away from them when an equivalent walk edge exists elsewhere.
		// Elevators carry a real wait-for-it/ride-it time cost on top of the
		// raw centroid distance already counted, so they're weighted higher
		// still -- prefer a walkable route when one exists, but still use
		// the lift when it's the only (or genuinely better) option.
		rec.traverseCost = (p.edgeType == EDGE_JUMP) ? 1.5f : (p.edgeType == EDGE_ELEVATOR) ? 2.0f : 1.0f;
		rec.buttonPos[0] = static_cast<float>(p.buttonPos.x);
		rec.buttonPos[1] = static_cast<float>(p.buttonPos.y);
		rec.buttonPos[2] = static_cast<float>(p.buttonPos.z);
		rec.travelTime = static_cast<float>(p.travelTime);
	}

	// String table is currently always empty (see NavSpecialEdgeRecord's own
	// comment) -- reserved for a future producer, not written to by anything
	// today.
	std::vector<char> stringTable;

	std::vector<NavSpecialEdgeRecord> specialEdgeRecords(specialEdges.size());
	for (size_t i = 0; i < specialEdges.size(); ++i)
	{
		const TaggedSpecialEdge &e = specialEdges[i];
		NavSpecialEdgeRecord &rec = specialEdgeRecords[i];
		rec.areaFrom = static_cast<uint32_t>(e.areaFrom);
		rec.areaTo = static_cast<uint32_t>(e.areaTo);
		rec.type = e.type;
		rec.anchorFrom[0] = static_cast<float>(e.anchorFrom.x);
		rec.anchorFrom[1] = static_cast<float>(e.anchorFrom.y);
		rec.anchorFrom[2] = static_cast<float>(e.anchorFrom.z);
		rec.anchorTo[0] = static_cast<float>(e.anchorTo.x);
		rec.anchorTo[1] = static_cast<float>(e.anchorTo.y);
		rec.anchorTo[2] = static_cast<float>(e.anchorTo.z);
		rec.stringTableOffset = -1;
	}

	NavHeader header{};
	std::memcpy(header.magic, kNavMagic, sizeof(header.magic));
	header.version = kNavVersion;
	std::memset(header.mapName, 0, sizeof(header.mapName));
	std::strncpy(header.mapName, mapName.c_str(), sizeof(header.mapName) - 1);
	header.bspChecksum = bspChecksum;
	header.flags = 0;
	header.areaCount = static_cast<uint32_t>(areaRecords.size());
	header.portalCount = static_cast<uint32_t>(portalRecords.size());
	header.areaPortalIndexCount = static_cast<uint32_t>(areaPortalIndex.size());
	header.specialEdgeCount = static_cast<uint32_t>(specialEdgeRecords.size());
	header.stringTableSize = static_cast<uint32_t>(stringTable.size());
	header.areaPlaneCount = static_cast<uint32_t>(areaPlanes.size());

	WriteRaw(f, &header, sizeof(header), "header");
	WriteRaw(f, areaRecords.data(), areaRecords.size() * sizeof(NavAreaRecord), "area records");
	WriteRaw(f, portalRecords.data(), portalRecords.size() * sizeof(NavPortalRecord), "portal records");
	WriteRaw(f, areaPortalIndex.data(), areaPortalIndex.size() * sizeof(uint32_t), "area-portal index");
	WriteRaw(f, specialEdgeRecords.data(), specialEdgeRecords.size() * sizeof(NavSpecialEdgeRecord), "special edge records");
	WriteRaw(f, stringTable.data(), stringTable.size(), "string table");
	WriteRaw(f, areaPlanes.data(), areaPlanes.size() * sizeof(NavAreaPlane), "area planes");

	std::fclose(f);
}

} // namespace navgen
