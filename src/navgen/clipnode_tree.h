#pragma once

#include <algorithm>
#include <cmath>
#include <functional>
#include <vector>

#include "bsp_file.h"
#include "hull_trace.h"
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

using HullTree = HullTrace::HullTree;

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

struct BatchTraceScratch
{
    const dclipnode_t *clipnodes = nullptr;
    int headnode = 0;
    int depth = -1;
    std::vector<float> ts;
    std::vector<float> te;
    std::vector<uint32_t> combo;
    std::vector<float> p1f;
    std::vector<float> p2f;
    std::vector<float> soa;
    std::vector<uint8_t> blockedShift;
};

void SegmentsBlockedBatch(const HullTree &hull, int headnode, const float *starts, size_t n,
    const float *ends, size_t m, uint8_t *blocked, BatchTraceScratch &scratch);

inline bool SegmentHitsBox(const Vec3 &a, const Vec3 &b, const Vec3 &mins, const Vec3 &maxs)
{
    double origin[3] = {a.x, a.y, a.z};
    double delta[3] = {b.x - a.x, b.y - a.y, b.z - a.z};
    double lo[3] = {mins.x, mins.y, mins.z};
    double hi[3] = {maxs.x, maxs.y, maxs.z};

    double tmin = 0.0;
    double tmax = 1.0;

    for (int axis = 0; axis < 3; ++axis)
    {
        if (std::fabs(delta[axis]) < 1e-9)
        {
            if (origin[axis] < lo[axis] || origin[axis] > hi[axis])
                return false;
            continue;
        }

        double inv = 1.0 / delta[axis];
        double t1 = (lo[axis] - origin[axis]) * inv;
        double t2 = (hi[axis] - origin[axis]) * inv;
        if (t1 > t2)
            std::swap(t1, t2);

        tmin = std::max(tmin, t1);
        tmax = std::min(tmax, t2);
        if (tmin > tmax)
            return false;
    }

    return true;
}

TraceResult TraceHullWithEntities(const BspFile &bsp, int worldHeadnode,
    const std::vector<BlockingVolume> &blockers, const Vec3 &start, const Vec3 &end);

TraceResult TraceHullArc(const HullTree &hull, int headnode, const Vec3 &start, const Vec3 &vel,
    double gravity, double maxTime);

TraceResult TraceHullArcWithEntities(const BspFile &bsp, int worldHeadnode,
    const std::vector<BlockingVolume> &blockers, const Vec3 &start, const Vec3 &vel, double gravity,
    double maxTime);

}

