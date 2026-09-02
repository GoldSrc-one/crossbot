#pragma once

#include <string>
#include <vector>

#include "blocking_entities.h"
#include "bsp_file.h"
#include "entity_parser.h"
#include "leaf_extractor.h"
#include "polytope.h"

namespace navgen
{

struct MergedConnection;

struct LiquidVolume
{
	int contents = CONTENTS_WATER;
	int modelIndex = 0;
	int entityIndex = -1;

	std::vector<Polytope> pieces;
	Vec3 mins;
	Vec3 maxs;

	double surfaceZ = 0.0;
	Vec3 travel;
	double speed = 0.0;
	bool moving = false;

	double LowSurfaceZ() const { return (travel.z < 0.0) ? surfaceZ + travel.z : surfaceZ; }
	double HighSurfaceZ() const { return (travel.z > 0.0) ? surfaceZ + travel.z : surfaceZ; }

	bool ContainsPoint(const Vec3 &p, double epsilon = 0.01) const;
	bool OverlapsColumn(const Vec3 &boxMins, const Vec3 &boxMaxs) const;
};

struct LiquidSet
{
	std::vector<LiquidVolume> water;
	std::vector<LiquidVolume> hazard;
};

LiquidSet ExtractLiquidVolumes(const BspFile &bsp, const std::vector<Entity> &entities, double mapRadius);

bool PointInPolytope(const Polytope &poly, const Vec3 &p, double epsilon = 0.01);

void PolytopeAABB(const Polytope &poly, Vec3 &mins, Vec3 &maxs);

void DumpLiquidVolumes(const std::string &bspPath, const LiquidSet &liquids);

size_t DropLiquidHazardLeaves(const std::vector<LiquidVolume> &hazards,
	std::vector<ExtractedLeaf> &leaves);

size_t PruneLiquidHazardConnections(const std::vector<LiquidVolume> &hazards,
	std::vector<MergedConnection> &conns);

}
