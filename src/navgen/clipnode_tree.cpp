#include "clipnode_tree.h"
#include "blocking_entities.h"

namespace navgen
{

namespace
{

void WalkRecursive(const BspFile &bsp, int nodeIndex, bool includeWater,
	std::vector<PlaneChainEntry> &chain, const LeafCallback &callback, const ContentsFilter &filter)
{
	if (nodeIndex < 0)
	{

		bool wanted = filter ? filter(nodeIndex)
			: (nodeIndex == CONTENTS_EMPTY || (includeWater && nodeIndex == CONTENTS_WATER));
		if (wanted)
			callback(nodeIndex, chain);
		return;
	}

	const dclipnode_t &node = bsp.Clipnodes()[static_cast<size_t>(nodeIndex)];

	chain.push_back({node.planenum, true});
	WalkRecursive(bsp, node.children[0], includeWater, chain, callback, filter);
	chain.pop_back();

	chain.push_back({node.planenum, false});
	WalkRecursive(bsp, node.children[1], includeWater, chain, callback, filter);
	chain.pop_back();
}

void WalkHullRecursive(const HullTree &hull, int nodeIndex, std::vector<PlaneChainEntry> &chain,
	const LeafCallback &callback, const ContentsFilter &filter)
{
	if (nodeIndex < 0)
	{
		if (filter(nodeIndex))
			callback(nodeIndex, chain);
		return;
	}

	if (static_cast<size_t>(nodeIndex) >= hull.clipnodeCount)
		return;

	const dclipnode_t &node = hull.clipnodes[static_cast<size_t>(nodeIndex)];

	chain.push_back({node.planenum, true});
	WalkHullRecursive(hull, node.children[0], chain, callback, filter);
	chain.pop_back();

	chain.push_back({node.planenum, false});
	WalkHullRecursive(hull, node.children[1], chain, callback, filter);
	chain.pop_back();
}

}

void WalkClipnodeTree(const BspFile &bsp, int headnode, bool includeWater, const LeafCallback &callback,
	const ContentsFilter &filter)
{
	std::vector<PlaneChainEntry> chain;
	WalkRecursive(bsp, headnode, includeWater, chain, callback, filter);
}

void WalkHullTree(const HullTree &hull, int headnode, const ContentsFilter &filter,
	const LeafCallback &callback)
{
	if (!filter || !callback)
		return;

	std::vector<PlaneChainEntry> chain;
	WalkHullRecursive(hull, headnode, chain, callback, filter);
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
	double minPieceVolume, double epsilon, std::vector<int8_t> &memo, std::vector<Polytope> &out,
	bool *removedAny)
{
	if (poly.Empty())
		return;

	int8_t classification = ClassifySubtree(bsp, nodeIndex, removeContents, memo);
	if (classification == kAllRemoved)
	{
		if (removedAny && poly.Volume() > minPieceVolume)
			*removedAny = true;
		return;
	}
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
	CarveRecursive(bsp, node.children[0], std::move(front), removeContents, mapRadius, minPieceVolume, epsilon, memo, out, removedAny);
	CarveRecursive(bsp, node.children[1], std::move(back), removeContents, mapRadius, minPieceVolume, epsilon, memo, out, removedAny);
}

}

std::vector<Polytope> CarvePolytopeAgainstTree(const BspFile &bsp, int headnode, const Polytope &poly,
	int removeContents, double mapRadius, double minPieceVolume, double epsilon, bool *removedAny)
{
	std::vector<int8_t> memo(bsp.Clipnodes().size(), kUnclassified);
	std::vector<Polytope> out;
	CarveRecursive(bsp, headnode, poly, removeContents, mapRadius, minPieceVolume, epsilon, memo, out,
		removedAny);
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

HullTree WorldHull(const BspFile &bsp)
{
	HullTree hull;
	hull.clipnodes = bsp.Clipnodes().data();
	hull.clipnodeCount = bsp.Clipnodes().size();
	hull.planes = bsp.Planes().data();
	hull.planeCount = bsp.Planes().size();
	return hull;
}

int ClassifyPoint(const HullTree &hull, int headnode, const Vec3 &point)
{
	int nodeIndex = headnode;
	while (nodeIndex >= 0)
	{
		const dclipnode_t &node = hull.clipnodes[static_cast<size_t>(nodeIndex)];
		const dplane_t &plane = hull.planes[static_cast<size_t>(node.planenum)];
		nodeIndex = (PlaneDiff(plane, point) >= 0.0) ? node.children[0] : node.children[1];
	}
	return nodeIndex;
}

int ClassifyPoint(const BspFile &bsp, int headnode, const Vec3 &point)
{
	return ClassifyPoint(WorldHull(bsp), headnode, point);
}

namespace
{

enum class HullStep
{
	Continue,
	Stop,
	Clear
};

struct NullTrace
{
};

template <bool BoolOnly, typename Trace>
HullStep HullCheck(const HullTree &hull, int headnode, int nodeIndex, double p1f, double p2f,
	const Vec3 &p1, const Vec3 &p2, Trace &trace)
{
	if (nodeIndex < 0)
	{
		if constexpr (BoolOnly)
		{
			return (nodeIndex == CONTENTS_SOLID) ? HullStep::Stop : HullStep::Continue;
		}
		else
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
			return HullStep::Continue;
		}
	}

	const dclipnode_t &node = hull.clipnodes[static_cast<size_t>(nodeIndex)];
	const dplane_t &plane = hull.planes[static_cast<size_t>(node.planenum)];

	double t1 = PlaneDiff(plane, p1);
	double t2 = PlaneDiff(plane, p2);

	if (t1 >= 0.0 && t2 >= 0.0)
		return HullCheck<BoolOnly>(hull, headnode, node.children[0], p1f, p2f, p1, p2, trace);
	if (t1 < 0.0 && t2 < 0.0)
		return HullCheck<BoolOnly>(hull, headnode, node.children[1], p1f, p2f, p1, p2, trace);

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

	HullStep nearStep = HullCheck<BoolOnly>(hull, headnode, node.children[side], p1f, midf, p1, mid,
		trace);
	if (nearStep != HullStep::Continue)
		return nearStep;

	if (ClassifyPoint(hull, node.children[side ^ 1], mid) != CONTENTS_SOLID)
		return HullCheck<BoolOnly>(hull, headnode, node.children[side ^ 1], midf, p2f, mid, p2, trace);

	if constexpr (BoolOnly)
	{
		if (static_cast<float>(midf) < 1.0f)
			return HullStep::Stop;
	}
	else
	{
		if (trace.allsolid)
			return HullStep::Stop;

		if (side == 0)
			trace.planeNormal = Vec3(plane.normal[0], plane.normal[1], plane.normal[2]);
		else
			trace.planeNormal = Vec3(-plane.normal[0], -plane.normal[1], -plane.normal[2]);
	}

	while (ClassifyPoint(hull, headnode, mid) == CONTENTS_SOLID)
	{
		frac -= 0.1;
		if (frac < 0.0)
			break;
		midf = p1f + (p2f - p1f) * frac;
		mid = p1 + (p2 - p1) * frac;
	}

	if constexpr (BoolOnly)
	{
		return (static_cast<float>(midf) < 1.0f) ? HullStep::Stop : HullStep::Clear;
	}
	else
	{
		trace.fraction = static_cast<float>(midf);
		trace.endpos = mid;
		return HullStep::Stop;
	}
}

void BuildOriginChain(const HullTree &hull, int headnode, const Vec3 &start, HullOriginChain &chain)
{
	chain.steps.clear();
	chain.origin = start;
	chain.clipnodes = hull.clipnodes;
	chain.headnode = headnode;

	int nodeIndex = headnode;
	while (nodeIndex >= 0)
	{
		const dclipnode_t &node = hull.clipnodes[static_cast<size_t>(nodeIndex)];
		const dplane_t &plane = hull.planes[static_cast<size_t>(node.planenum)];

		HullChainStep step;
		step.t1 = PlaneDiff(plane, start);
		step.normal[0] = plane.normal[0];
		step.normal[1] = plane.normal[1];
		step.normal[2] = plane.normal[2];
		step.dist = plane.dist;

		int side = (step.t1 < 0.0) ? 1 : 0;
		step.farChild = node.children[side ^ 1];
		chain.steps.push_back(step);

		nodeIndex = node.children[side];
	}

	chain.leafContents = nodeIndex;
	chain.valid = true;
}

bool SegmentBlockedCached(const HullTree &hull, int headnode, const Vec3 &start, const Vec3 &end,
	HullOriginChain &chain)
{
	const double p1f = 0.0;
	double p2f = 1.0;
	Vec3 p2 = end;

	chain.splits.clear();

	for (const HullChainStep &step : chain.steps)
	{
		double t1 = step.t1;
		double t2 = static_cast<double>(step.normal[0]) * p2.x +
			static_cast<double>(step.normal[1]) * p2.y +
			static_cast<double>(step.normal[2]) * p2.z - step.dist;

		if (t1 >= 0.0 && t2 >= 0.0)
			continue;
		if (t1 < 0.0 && t2 < 0.0)
			continue;

		double frac = (t1 < 0.0)
			? (t1 + kDistEpsilon) / (t1 - t2)
			: (t1 - kDistEpsilon) / (t1 - t2);
		if (frac < 0.0)
			frac = 0.0;
		else if (frac > 1.0)
			frac = 1.0;

		HullChainSplit split;
		split.frac = frac;
		split.midf = p1f + (p2f - p1f) * frac;
		split.p2f = p2f;
		split.mid = start + (p2 - start) * frac;
		split.p2 = p2;
		split.farChild = step.farChild;
		chain.splits.push_back(split);

		p2f = split.midf;
		p2 = split.mid;
	}

	HullStep result = (chain.leafContents == CONTENTS_SOLID) ? HullStep::Stop : HullStep::Continue;

	for (size_t k = chain.splits.size(); k-- > 0; )
	{
		if (result != HullStep::Continue)
			break;

		const HullChainSplit &e = chain.splits[k];

		if (ClassifyPoint(hull, e.farChild, e.mid) != CONTENTS_SOLID)
		{
			NullTrace nullTrace;
			result = HullCheck<true>(hull, headnode, e.farChild, e.midf, e.p2f, e.mid, e.p2,
				nullTrace);
			continue;
		}

		if (static_cast<float>(e.midf) < 1.0f)
			return true;

		double frac = e.frac;
		double midf = e.midf;
		Vec3 mid = e.mid;
		while (ClassifyPoint(hull, headnode, mid) == CONTENTS_SOLID)
		{
			frac -= 0.1;
			if (frac < 0.0)
				break;
			midf = p1f + (e.p2f - p1f) * frac;
			mid = start + (e.p2 - start) * frac;
		}

		return static_cast<float>(midf) < 1.0f;
	}

	return result == HullStep::Stop;
}

}

TraceResult TraceHull(const BspFile &bsp, int headnode, const Vec3 &start, const Vec3 &end)
{
	return TraceHull(WorldHull(bsp), headnode, start, end);
}

TraceResult TraceHull(const HullTree &hull, int headnode, const Vec3 &start, const Vec3 &end)
{
	TraceResult trace;
	trace.start = start;
	trace.end = end;
	trace.endpos = end;
	trace.fraction = 1.0f;
	trace.allsolid = true;
	trace.startsolid = false;
	trace.contents = CONTENTS_EMPTY;

	HullCheck<false>(hull, headnode, headnode, 0.0, 1.0, start, end, trace);

	return trace;
}

bool SegmentBlocked(const HullTree &hull, int headnode, const Vec3 &start, const Vec3 &end)
{
	NullTrace trace;
	return HullCheck<true>(hull, headnode, headnode, 0.0, 1.0, start, end, trace) ==
		HullStep::Stop;
}

bool SegmentBlocked(const HullTree &hull, int headnode, const Vec3 &start, const Vec3 &end,
	HullOriginChain &chain)
{
	if (!chain.valid || chain.clipnodes != hull.clipnodes || chain.headnode != headnode ||
		chain.origin.x != start.x || chain.origin.y != start.y || chain.origin.z != start.z)
	{
		BuildOriginChain(hull, headnode, start, chain);
	}

	return SegmentBlockedCached(hull, headnode, start, end, chain);
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

