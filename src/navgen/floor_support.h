#pragma once

#include <vector>

#include "blocking_entities.h"
#include "bsp_file.h"
#include "leaf_extractor.h"
#include "polytope.h"

namespace navgen
{

constexpr double kStepHeight = 18.0;

void CarveWindingAgainstTree(const BspFile &bsp, int nodeIndex, Winding w, std::vector<Winding> &solid,
	std::vector<Winding> &empty);

Winding TranslateWinding(const Winding &w, const Vec3 &offset);

Vec3 WindingCentroid(const Winding &w);

bool OnWindingBoundary(const Vec3 &p, const Winding &w);

bool VerticalPlaneThroughEdge(const Vec3 &a, const Vec3 &b, Plane &out);

double Distance2DToWinding(const Winding &w, double x, double y);

bool PointInSolid(const BspFile &bsp, int headnode, const std::vector<BlockingVolume> &blockers, const Vec3 &p);

const Face *LowestFloorFace(const Polytope &poly);

// The floor face directly under (x,y). A convex polytope's lower boundary is hit
// once by any vertical line, so this is unambiguous; when (x,y) falls just outside
// every face (a connection midpoint sitting on a shared edge) the nearest one wins.
const Face *FloorFaceUnder(const Polytope &poly, double x, double y);

double FloorHeightAt(const Face &floorFace, double x, double y);

std::vector<Winding> ComputeSupportedFloor(const BspFile &bsp, int headnode,
	const std::vector<BlockingVolume> &blockers, const Face &floorFace);

std::vector<ExtractedLeaf> SplitLeavesByFloorSupport(const BspFile &bsp, int headnode,
	const std::vector<BlockingVolume> &blockers, const std::vector<ExtractedLeaf> &leaves, double mapRadius,
	size_t *droppedOut = nullptr, size_t *splitOut = nullptr, size_t *noFloorOut = nullptr,
	bool dump = false);

}

