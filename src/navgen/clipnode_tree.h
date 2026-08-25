#pragma once

#include <functional>
#include <vector>

#include "bsp_file.h"
#include "polytope.h"

namespace navgen
{

struct BlockingVolume;

struct PlaneChainEntry
{
	int planeIndex;
	bool front;
};

using LeafCallback = std::function<void(int leafContents, const std::vector<PlaneChainEntry> &chain)>;

using ContentsFilter = std::function<bool(int leafContents)>;

void WalkClipnodeTree(const BspFile &bsp, int headnode, bool includeWater, const LeafCallback &callback,
	const ContentsFilter &filter = ContentsFilter());

int ClassifyPoint(const BspFile &bsp, int headnode, const Vec3 &point);

std::vector<Polytope> CarvePolytopeAgainstTree(const BspFile &bsp, int headnode, const Polytope &poly,
	int removeContents, double mapRadius, double minPieceVolume = 1.0, double epsilon = 0.01,
	bool *removedAny = nullptr);

struct TraceResult
{
    Vec3 start;
    Vec3 end;
    Vec3 endpos;
    Vec3 planeNormal;
    float fraction = 1.0f;
    bool allsolid = false;
    bool startsolid = false;
    int ent = -1;
    int contents = CONTENTS_EMPTY;
};

TraceResult TraceHull(const BspFile &bsp, int headnode, const Vec3 &start, const Vec3 &end);

TraceResult TraceHullWithEntities(const BspFile &bsp, int worldHeadnode,
    const std::vector<BlockingVolume> &blockers, const Vec3 &start, const Vec3 &end);

}

