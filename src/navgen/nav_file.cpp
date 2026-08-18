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

}

void WriteNavFile(const std::string &path, const std::string &mapName, uint32_t bspChecksum,
	const std::vector<Area> &areas, const std::vector<MergedConnection> &conns, float designMaxSpeed)
{
	FILE *f = std::fopen(path.c_str(), "wb");
	if (!f)
		throw std::runtime_error("cannot open nav file for writing: " + path);

	std::vector<std::vector<uint32_t>> perArea(areas.size());
	for (size_t ci = 0; ci < conns.size(); ++ci)
	{
		perArea[static_cast<size_t>(conns[ci].areaA)].push_back(static_cast<uint32_t>(ci));
		perArea[static_cast<size_t>(conns[ci].areaB)].push_back(static_cast<uint32_t>(ci));
	}

	std::vector<NavAreaRecord> areaRecords(areas.size());
	std::vector<uint32_t> areaConnIndex;
	std::vector<NavAreaPlane> areaPlanes;
	std::vector<NavActivationDetourRecord> detourRecords;
	std::vector<NavDetourPointRecord> detourPoints;

	for (size_t i = 0; i < areas.size(); ++i)
	{
		NavAreaRecord &rec = areaRecords[i];
		rec.id = static_cast<uint32_t>(areas[i].id);
		ComputeBounds(areas[i].polytope, rec.mins, rec.maxs, rec.centroid);
		rec.areaFlags = areas[i].areaFlags;
		rec.activation = areas[i].activation;
		rec.pad0[0] = rec.pad0[1] = rec.pad0[2] = 0;
		rec.openSeconds = static_cast<float>(areas[i].openSeconds);

		rec.detourOffset = static_cast<uint32_t>(detourRecords.size());
		rec.detourCount = static_cast<uint32_t>(areas[i].detours.size());

		for (const ActivationDetour &d : areas[i].detours)
		{
			NavActivationDetourRecord drec;
			drec.fromAreaId = static_cast<uint32_t>(d.fromAreaId);
			drec.pointOffset = static_cast<uint32_t>(detourPoints.size());
			drec.pointCount = static_cast<uint32_t>(d.points.size());
			drec.actIndex = static_cast<uint32_t>(d.actIndex);
			drec.waitSeconds = static_cast<float>(d.waitSeconds);
			drec.detourSeconds = static_cast<float>(d.detourSeconds);
			drec.actCenter[0] = static_cast<float>(d.actCenter.x);
			drec.actCenter[1] = static_cast<float>(d.actCenter.y);
			drec.actCenter[2] = static_cast<float>(d.actCenter.z);
			drec.actKind = d.actKind;
			drec.flags = d.blocked ? crossbot_nav::DETOUR_FLAG_BLOCKED : crossbot_nav::DETOUR_FLAG_NONE;
			drec.pad[0] = drec.pad[1] = 0;
			detourRecords.push_back(drec);

			for (const DetourPoint &p : d.points)
			{
				NavDetourPointRecord prec;
				prec.pos[0] = static_cast<float>(p.pos.x);
				prec.pos[1] = static_cast<float>(p.pos.y);
				prec.pos[2] = static_cast<float>(p.pos.z);
				prec.areaId = static_cast<uint32_t>(p.areaId);
				prec.clearance = static_cast<float>(p.clearance);
				prec.flags = p.crouch ? crossbot_nav::DETOUR_PT_CROUCH : crossbot_nav::DETOUR_PT_NONE;
				prec.pad[0] = prec.pad[1] = prec.pad[2] = 0;
				detourPoints.push_back(prec);
			}
		}

		rec.connOffset = ToU16Count(areaConnIndex.size(), "area-conn-index offset");
		rec.connCount = ToU16Count(perArea[i].size(), "conns-per-area");

		for (uint32_t idx : perArea[i])
			areaConnIndex.push_back(idx);

		rec.planeOffset = ToU16Count(areaPlanes.size(), "area-plane offset");
		rec.planeCount = ToU16Count(areas[i].boundPlanes.size(), "planes-per-area");

		for (const Plane &src : areas[i].boundPlanes)
		{
			NavAreaPlane plane;
			plane.normal[0] = static_cast<float>(src.normal.x);
			plane.normal[1] = static_cast<float>(src.normal.y);
			plane.normal[2] = static_cast<float>(src.normal.z);
			plane.dist = static_cast<float>(src.dist);
			areaPlanes.push_back(plane);
		}
	}

	std::vector<NavConnectionRecord> connRecords(conns.size());
	for (size_t i = 0; i < conns.size(); ++i)
	{
		const MergedConnection &c = conns[i];
		NavConnectionRecord &rec = connRecords[i];
		rec.areaA = static_cast<uint32_t>(c.areaA);
		rec.areaB = static_cast<uint32_t>(c.areaB);
		rec.connType = c.connType;
		rec.connFlags = c.connFlags;
		rec.gateValid = static_cast<uint8_t>((c.gateAValid ? GATE_VALID_A : 0) |
			(c.gateBValid ? GATE_VALID_B : 0) | (c.gateAClamped ? GATE_CLAMPED_A : 0) |
			(c.gateBClamped ? GATE_CLAMPED_B : 0));
		rec.pad = 0;

		auto storeGate = [](float (&dst)[2][3], const Vec3 &left, const Vec3 &right)
		{
			dst[0][0] = static_cast<float>(left.x);
			dst[0][1] = static_cast<float>(left.y);
			dst[0][2] = static_cast<float>(left.z);
			dst[1][0] = static_cast<float>(right.x);
			dst[1][1] = static_cast<float>(right.y);
			dst[1][2] = static_cast<float>(right.z);
		};
		storeGate(rec.gateA, c.gateALeft, c.gateARight);
		storeGate(rec.gateB, c.gateBLeft, c.gateBRight);
		for (int k = 0; k < 2; ++k)
		{
			rec.gateAYaw[k] = static_cast<float>(c.gateAYaw[k]);
			rec.gateASpeed[k] = static_cast<float>(c.gateASpeed[k]);
		}

		rec.costSeconds = ConnectionCostSeconds(c);
		rec.travelSeconds = ConnectionTravelSeconds(areas, c);

		rec.exitSpeed = static_cast<float>(c.exitSpeed);
		rec.exitAngle = static_cast<float>(c.exitAngle);
		rec.jumpBackDist = static_cast<float>(c.jumpBackDist);
		rec.fallDamage = static_cast<float>(c.fallDamage);
		rec.flightTime = static_cast<float>(c.flightTime);
		rec.launchPos[0] = static_cast<float>(c.pointA.x);
		rec.launchPos[1] = static_cast<float>(c.pointA.y);
		rec.launchPos[2] = static_cast<float>(c.pointA.z);
		rec.landingPos[0] = static_cast<float>(c.pointB.x);
		rec.landingPos[1] = static_cast<float>(c.pointB.y);
		rec.landingPos[2] = static_cast<float>(c.pointB.z);
		rec.rideSeconds = static_cast<float>(c.rideSeconds);
		rec.rideSecondsMax = static_cast<float>(c.rideSecondsMax);
		rec.rideWaitSeconds = static_cast<float>(c.rideWaitSeconds);
	}

	NavHeader header{};
	std::memcpy(header.magic, kNavMagic, sizeof(header.magic));
	header.version = kNavVersion;
	std::memset(header.mapName, 0, sizeof(header.mapName));
	std::strncpy(header.mapName, mapName.c_str(), sizeof(header.mapName) - 1);
	header.bspChecksum = bspChecksum;
	header.flags = 0;
	header.areaCount = static_cast<uint32_t>(areaRecords.size());
	header.connCount = static_cast<uint32_t>(connRecords.size());
	header.areaConnIndexCount = static_cast<uint32_t>(areaConnIndex.size());
	header.areaPlaneCount = static_cast<uint32_t>(areaPlanes.size());
	header.detourCount = static_cast<uint32_t>(detourRecords.size());
	header.detourPointCount = static_cast<uint32_t>(detourPoints.size());
	header.designMaxSpeed = designMaxSpeed;

	WriteRaw(f, &header, sizeof(header), "header");
	WriteRaw(f, areaRecords.data(), areaRecords.size() * sizeof(NavAreaRecord), "area records");
	WriteRaw(f, connRecords.data(), connRecords.size() * sizeof(NavConnectionRecord), "connection records");
	WriteRaw(f, areaConnIndex.data(), areaConnIndex.size() * sizeof(uint32_t), "area-conn index");
	WriteRaw(f, areaPlanes.data(), areaPlanes.size() * sizeof(NavAreaPlane), "area planes");
	WriteRaw(f, detourRecords.data(), detourRecords.size() * sizeof(NavActivationDetourRecord),
		"activation detours");
	WriteRaw(f, detourPoints.data(), detourPoints.size() * sizeof(NavDetourPointRecord), "detour points");

	std::fclose(f);
}

}

