#include "clipnode_tree.h"
#include "blocking_entities.h"

namespace navgen
{

namespace
{

void WalkRecursive(const BspFile &bsp, int nodeIndex, bool includeWater,
	std::vector<PlaneChainEntry> &chain, const LeafCallback &callback)
{
	if (nodeIndex < 0)
	{

		if (nodeIndex == CONTENTS_EMPTY || (includeWater && nodeIndex == CONTENTS_WATER))
			callback(nodeIndex, chain);
		return;
	}

	const dclipnode_t &node = bsp.Clipnodes()[static_cast<size_t>(nodeIndex)];

	chain.push_back({node.planenum, true});
	WalkRecursive(bsp, node.children[0], includeWater, chain, callback);
	chain.pop_back();

	chain.push_back({node.planenum, false});
	WalkRecursive(bsp, node.children[1], includeWater, chain, callback);
	chain.pop_back();
}

}

void WalkClipnodeTree(const BspFile &bsp, int headnode, bool includeWater, const LeafCallback &callback)
{
	std::vector<PlaneChainEntry> chain;
	WalkRecursive(bsp, headnode, includeWater, chain, callback);
}

namespace
{

constexpr int8_t kUnclassified = -2;
constexpr int8_t kMixed = -1;
constexpr int8_t kAllKept = 0;
constexpr int8_t kAllRemoved = 1;

int8_t ClassifySubtree(const BspFile &bsp, int nodeIndex, int removeContents, std::vector<int8_t> &memo)
{
	if (nodeIndex < 0)
		return (nodeIndex == removeContents) ? kAllRemoved : kAllKept;

	int8_t &slot = memo[static_cast<size_t>(nodeIndex)];
	if (slot != kUnclassified)
		return slot;

	const dclipnode_t &node = bsp.Clipnodes()[static_cast<size_t>(nodeIndex)];
	int8_t front = ClassifySubtree(bsp, node.children[0], removeContents, memo);
	int8_t back = ClassifySubtree(bsp, node.children[1], removeContents, memo);

	slot = (front == back) ? front : kMixed;
	return slot;
}

void CarveRecursive(const BspFile &bsp, int nodeIndex, Polytope poly, int removeContents, double mapRadius,
	double minPieceVolume, double epsilon, std::vector<int8_t> &memo, std::vector<Polytope> &out)
{
	if (poly.Empty())
		return;

	int8_t classification = ClassifySubtree(bsp, nodeIndex, removeContents, memo);
	if (classification == kAllRemoved)
		return;
	if (classification == kAllKept)
	{
		if (poly.Volume() > minPieceVolume)
			out.push_back(std::move(poly));
		return;
	}

	const dclipnode_t &node = bsp.Clipnodes()[static_cast<size_t>(nodeIndex)];
	const dplane_t &dp = bsp.Planes()[static_cast<size_t>(node.planenum)];
	Plane plane;
	plane.normal = Vec3(dp.normal[0], dp.normal[1], dp.normal[2]);
	plane.dist = dp.dist;

	Polytope front = ClipPolytope(poly, plane, true, mapRadius, epsilon);
	Polytope back = ClipPolytope(poly, plane, false, mapRadius, epsilon);
	CarveRecursive(bsp, node.children[0], std::move(front), removeContents, mapRadius, minPieceVolume, epsilon, memo, out);
	CarveRecursive(bsp, node.children[1], std::move(back), removeContents, mapRadius, minPieceVolume, epsilon, memo, out);
}

}

std::vector<Polytope> CarvePolytopeAgainstTree(const BspFile &bsp, int headnode, const Polytope &poly,
	int removeContents, double mapRadius, double minPieceVolume, double epsilon)
{
	std::vector<int8_t> memo(bsp.Clipnodes().size(), kUnclassified);
	std::vector<Polytope> out;
	CarveRecursive(bsp, headnode, poly, removeContents, mapRadius, minPieceVolume, epsilon, memo, out);
	return out;
}

namespace
{

constexpr double kDistEpsilon = 0.03125;

double PlaneDiff(const dplane_t &plane, const Vec3 &p)
{
	return static_cast<double>(plane.normal[0]) * p.x + static_cast<double>(plane.normal[1]) * p.y +
		static_cast<double>(plane.normal[2]) * p.z - plane.dist;
}

}

int ClassifyPoint(const BspFile &bsp, int headnode, const Vec3 &point)
{
	int nodeIndex = headnode;
	while (nodeIndex >= 0)
	{
		const dclipnode_t &node = bsp.Clipnodes()[static_cast<size_t>(nodeIndex)];
		const dplane_t &plane = bsp.Planes()[static_cast<size_t>(node.planenum)];
		nodeIndex = (PlaneDiff(plane, point) >= 0.0) ? node.children[0] : node.children[1];
	}
	return nodeIndex;
}

namespace
{

bool RecursiveHullCheck(const BspFile &bsp, int headnode, int nodeIndex, double p1f, double p2f,
	const Vec3 &p1, const Vec3 &p2, TraceResult &trace)
{
	if (nodeIndex < 0)
	{
		if (nodeIndex != CONTENTS_SOLID)
		{
			trace.allsolid = false;
			trace.contents = nodeIndex;
		}
		else
		{
			trace.startsolid = true;
		}
		return true;
	}

	const dclipnode_t &node = bsp.Clipnodes()[static_cast<size_t>(nodeIndex)];
	const dplane_t &plane = bsp.Planes()[static_cast<size_t>(node.planenum)];

	double t1 = PlaneDiff(plane, p1);
	double t2 = PlaneDiff(plane, p2);

	if (t1 >= 0.0 && t2 >= 0.0)
		return RecursiveHullCheck(bsp, headnode, node.children[0], p1f, p2f, p1, p2, trace);
	if (t1 < 0.0 && t2 < 0.0)
		return RecursiveHullCheck(bsp, headnode, node.children[1], p1f, p2f, p1, p2, trace);

	double frac = (t1 < 0.0)
		? (t1 + kDistEpsilon) / (t1 - t2)
		: (t1 - kDistEpsilon) / (t1 - t2);
	if (frac < 0.0)
		frac = 0.0;
	else if (frac > 1.0)
		frac = 1.0;

	double midf = p1f + (p2f - p1f) * frac;
	Vec3 mid = p1 + (p2 - p1) * frac;

	int side = (t1 < 0.0) ? 1 : 0;

	if (!RecursiveHullCheck(bsp, headnode, node.children[side], p1f, midf, p1, mid, trace))
		return false;

	if (ClassifyPoint(bsp, node.children[side ^ 1], mid) != CONTENTS_SOLID)
		return RecursiveHullCheck(bsp, headnode, node.children[side ^ 1], midf, p2f, mid, p2, trace);

	if (trace.allsolid)
		return false;

	if (side == 0)
		trace.planeNormal = Vec3(plane.normal[0], plane.normal[1], plane.normal[2]);
	else
		trace.planeNormal = Vec3(-plane.normal[0], -plane.normal[1], -plane.normal[2]);

	while (ClassifyPoint(bsp, headnode, mid) == CONTENTS_SOLID)
	{
		frac -= 0.1;
		if (frac < 0.0)
		{
			trace.fraction = static_cast<float>(midf);
			trace.endpos = mid;
			return false;
		}
		midf = p1f + (p2f - p1f) * frac;
		mid = p1 + (p2 - p1) * frac;
	}

	trace.fraction = static_cast<float>(midf);
	trace.endpos = mid;
	return false;
}

}

TraceResult TraceHull(const BspFile &bsp, int headnode, const Vec3 &start, const Vec3 &end)
{
	TraceResult trace;
	trace.start = start;
	trace.end = end;
	trace.endpos = end;
	trace.fraction = 1.0f;
	trace.allsolid = true;
	trace.startsolid = false;
	trace.contents = CONTENTS_EMPTY;

	RecursiveHullCheck(bsp, headnode, headnode, 0.0, 1.0, start, end, trace);

	return trace;
}

TraceResult TraceHullWithEntities(const BspFile &bsp, int worldHeadnode,
    const std::vector<BlockingVolume> &blockers, const Vec3 &start, const Vec3 &end)
{

    TraceResult worldTr = TraceHull(bsp, worldHeadnode, start, end);

    if (worldTr.fraction <= 0.0f)
        return worldTr;

    TraceResult bestTr = worldTr;

    for (const BlockingVolume &blocker : blockers)
    {

        Vec3 localStart = start - blocker.origin;
        Vec3 localEnd = end - blocker.origin;
        TraceResult tr = TraceHull(bsp, blocker.headnode, localStart, localEnd);
        if (tr.fraction < bestTr.fraction)
        {
            bestTr = tr;
            bestTr.ent = 1;

            if (tr.fraction < 1.0f)
            {
                bestTr.endpos = tr.endpos + blocker.origin;
            }
        }
    }

    return bestTr;
}

}

