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

struct HullTree
{
    const dclipnode_t *clipnodes = nullptr;
    size_t clipnodeCount = 0;
    const dplane_t *planes = nullptr;
    size_t planeCount = 0;
};

HullTree WorldHull(const BspFile &bsp);

void WalkHullTree(const HullTree &hull, int headnode, const ContentsFilter &filter,
	const LeafCallback &callback);

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

int ClassifyPoint(const HullTree &hull, int headnode, const Vec3 &point);

TraceResult TraceHull(const HullTree &hull, int headnode, const Vec3 &start, const Vec3 &end);

bool SegmentBlocked(const HullTree &hull, int headnode, const Vec3 &start, const Vec3 &end);

struct HullChainStep
{
    double t1;
    float normal[3];
    float dist;
    int farChild;
};

struct HullChainSplit
{
    double frac;
    double midf;
    double p2f;
    Vec3 mid;
    Vec3 p2;
    int farChild;
};

struct HullOriginChain
{
    Vec3 origin;
    const dclipnode_t *clipnodes = nullptr;
    int headnode = 0;
    int leafContents = 0;
    bool valid = false;
    std::vector<HullChainStep> steps;
    std::vector<HullChainSplit> splits;
};

bool SegmentBlocked(const HullTree &hull, int headnode, const Vec3 &start, const Vec3 &end,
    HullOriginChain &chain);

TraceResult TraceHullWithEntities(const BspFile &bsp, int worldHeadnode,
    const std::vector<BlockingVolume> &blockers, const Vec3 &start, const Vec3 &end);

}

