#pragma once

#include <vector>

#include "area_merger.h"
#include "blocking_entities.h"
#include "bsp_file.h"
#include "leaf_extractor.h"
#include "polytope.h"
#include "water_volumes.h"

namespace navgen
{

constexpr double kStepHeight = 18.0;

void CarveWindingAgainstTree(const BspFile &bsp, int nodeIndex, Winding w, std::vector<Winding> &solid,
	std::vector<Winding> &empty);

Winding TranslateWinding(const Winding &w, const Vec3 &offset);

Vec3 WindingCentroid(const Winding &w);

bool OnWindingBoundary(const Vec3 &p, const Winding &w);

bool VerticalPlaneThroughEdge(const Vec3 &a, const Vec3 &b, Plane &out);

bool HasPlane(const std::vector<Plane> &planes, const Plane &plane);

double Distance2DToWinding(const Winding &w, double x, double y);

bool PointInSolid(const BspFile &bsp, int headnode, const std::vector<BlockingVolume> &blockers, const Vec3 &p);

size_t TagBarrierFloors(const BspFile &bsp, int headnode, const std::vector<PassableVolume> &passables,
	std::vector<Area> &areas);

const Face *LowestFloorFace(const Polytope &poly);

const Face *FloorFaceUnder(const Polytope &poly, double x, double y);

double FloorHeightAt(const Face &floorFace, double x, double y);

std::vector<Winding> ComputeSupportedFloor(const BspFile &bsp, int headnode,
	const std::vector<BlockingVolume> &blockers, const Face &floorFace);

std::vector<Winding> ComputeSupportedCeiling(const BspFile &bsp, int headnode,
	const std::vector<BlockingVolume> &blockers, const Face &ceilingFace);

struct FloorSupportStats
{
	size_t dropped = 0;
	size_t split = 0;
	size_t noFloor = 0;
	size_t absorbed = 0;
	size_t waterColumns = 0;
};

size_t DropSkyFloorLeaves(const BspFile &bsp, std::vector<ExtractedLeaf> &leaves);

size_t TagDriftLeaves(const BspFile &bsp, const std::vector<DriftVolume> &drifts,
	std::vector<ExtractedLeaf> &leaves);

size_t DropLethalFloorLeaves(const BspFile &bsp, int headnode, const std::vector<BlockingVolume> &blockers,
	const std::vector<BlockingVolume> &hurtVolumes, std::vector<ExtractedLeaf> &leaves);

std::vector<ExtractedLeaf> SplitLeavesByFloorSupport(const BspFile &bsp, int headnode,
	const std::vector<BlockingVolume> &blockers, const std::vector<ExtractedLeaf> &leaves, double mapRadius,
	FloorSupportStats *statsOut = nullptr, bool dump = false,
	const std::vector<LiquidVolume> *waterVolumes = nullptr);

}

