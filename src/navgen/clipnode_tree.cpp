#include "clipnode_tree.h"
#include "blocking_entities.h"

#include <immintrin.h>
#include <intrin.h>

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

namespace
{

int HullDepth(const HullTree &hull, int nodeIndex, std::vector<int> &memo)
{
	if (nodeIndex < 0)
		return 0;

	int &slot = memo[static_cast<size_t>(nodeIndex)];
	if (slot < 0)
	{
		const dclipnode_t &node = hull.clipnodes[static_cast<size_t>(nodeIndex)];
		int front = HullDepth(hull, node.children[0], memo);
		int back = HullDepth(hull, node.children[1], memo);
		slot = 1 + (front > back ? front : back);
	}
	return slot;
}

struct BatchContext
{
	const HullTree &hull;
	const float *starts = nullptr;
	const float *ends = nullptr;
	size_t n = 0;
	size_t m = 0;
	uint8_t *blocked = nullptr;
	float *ts = nullptr;
	float *te = nullptr;
	uint32_t *combo = nullptr;
	float *p1f = nullptr;
	float *p2f = nullptr;
	// combo = (i << shift) | j, which is also the combo's index into blocked.
	unsigned shift = 0;
	uint32_t jMask = 0;
	// Coordinates split per axis, padded to a multiple of 4 by repeating the last point.
	const float *sx = nullptr;
	const float *sy = nullptr;
	const float *sz = nullptr;
	const float *ex = nullptr;
	const float *ey = nullptr;
	const float *ez = nullptr;
	size_t nPad = 0;
	size_t mPad = 0;
};

inline float PlaneDistF(const dplane_t &plane, const float *p)
{
	return plane.normal[0] * p[0] + plane.normal[1] * p[1] + plane.normal[2] * p[2] - plane.dist;
}

// The node distance fills return 1 when every distance they wrote is >= 0 and 2 when every one is < 0:
// each entry's t1 and t2 lerp between its own start and end distance, so the whole list then belongs to
// one child. 0 otherwise.

// All n + m distances, four points per step. Same operation order as PlaneDistF, so the results are
// bit-identical to FillUsedDistances; the repeated padding points leave the min and max unchanged.
int FillAllDistances(const BatchContext &ctx, const dplane_t &plane)
{
	const __m128 nx = _mm_set1_ps(plane.normal[0]);
	const __m128 ny = _mm_set1_ps(plane.normal[1]);
	const __m128 nz = _mm_set1_ps(plane.normal[2]);
	const __m128 dist = _mm_set1_ps(plane.dist);
	__m128 lo = _mm_set1_ps(3.0e38f);
	__m128 hi = _mm_set1_ps(-3.0e38f);
	auto fill = [&](const float *x, const float *y, const float *z, float *out, size_t countPad) {
		for (size_t k = 0; k < countPad; k += 4)
		{
			__m128 t = _mm_add_ps(_mm_mul_ps(nx, _mm_loadu_ps(x + k)), _mm_mul_ps(ny, _mm_loadu_ps(y + k)));
			t = _mm_sub_ps(_mm_add_ps(t, _mm_mul_ps(nz, _mm_loadu_ps(z + k))), dist);
			_mm_storeu_ps(out + k, t);
			lo = _mm_min_ps(lo, t);
			hi = _mm_max_ps(hi, t);
		}
	};
	fill(ctx.sx, ctx.sy, ctx.sz, ctx.ts, ctx.nPad);
	fill(ctx.ex, ctx.ey, ctx.ez, ctx.te, ctx.mPad);

	const __m128 zero = _mm_setzero_ps();
	if (_mm_movemask_ps(_mm_cmplt_ps(lo, zero)) == 0)
		return 1;
	if (_mm_movemask_ps(_mm_cmpge_ps(hi, zero)) == 0)
		return 2;
	return 0;
}

// Only the distances a narrow list touches: fewer than n + m.
int FillUsedDistances(const BatchContext &ctx, const dplane_t &plane, const uint32_t *combos, size_t count)
{
	float lo = 3.0e38f;
	float hi = -3.0e38f;
	for (size_t k = 0; k < count; ++k)
	{
		size_t i = combos[k] >> ctx.shift;
		size_t j = combos[k] & ctx.jMask;
		float a = ctx.ts[i] = PlaneDistF(plane, ctx.starts + 3 * i);
		float b = ctx.te[j] = PlaneDistF(plane, ctx.ends + 3 * j);
		lo = a < lo ? a : lo;
		lo = b < lo ? b : lo;
		hi = a > hi ? a : hi;
		hi = b > hi ? b : hi;
	}
	return lo >= 0.0f ? 1 : (hi < 0.0f ? 2 : 0);
}

// pshufb masks that pack the lanes set in a 4-bit mask to the front of a 4 x 4-byte vector.
struct LaneCompressTable
{
	uint8_t shuffle[16][16];

	LaneCompressTable()
	{
		for (unsigned mask = 0; mask < 16; ++mask)
		{
			unsigned out = 0;
			for (unsigned lane = 0; lane < 4; ++lane)
			{
				if (mask & (1u << lane))
				{
					for (unsigned b = 0; b < 4; ++b)
						shuffle[mask][out * 4 + b] = static_cast<uint8_t>(lane * 4 + b);
					++out;
				}
			}
			for (unsigned b = out * 4; b < 16; ++b)
				shuffle[mask][b] = 0x80;
		}
	}
};

const LaneCompressTable kLaneCompress;

inline void LaneStore(const BatchContext &ctx, size_t at, unsigned mask, __m128i combo, __m128 p1f, __m128 p2f)
{
	__m128i shuffle = _mm_loadu_si128(reinterpret_cast<const __m128i *>(kLaneCompress.shuffle[mask]));
	_mm_storeu_si128(reinterpret_cast<__m128i *>(ctx.combo + at), _mm_shuffle_epi8(combo, shuffle));
	_mm_storeu_ps(ctx.p1f + at, _mm_castsi128_ps(_mm_shuffle_epi8(_mm_castps_si128(p1f), shuffle)));
	_mm_storeu_ps(ctx.p2f + at, _mm_castsi128_ps(_mm_shuffle_epi8(_mm_castps_si128(p2f), shuffle)));
}

inline void MarkLanes(const BatchContext &ctx, size_t at, unsigned mask)
{
	for (; mask != 0; mask &= mask - 1)
	{
		unsigned long lane;
		_BitScanForward(&lane, mask);
		ctx.blocked[ctx.combo[at + lane]] = 1;
	}
}

// The entries at [base, base + count) belong to this node and are never written. Children get two fresh
// lists above top, each at most count long plus a 4-slot gap, so nothing is restored on the way back up.
// Unlike HullCheck<true> there is no hit position: any solid leaf reached means blocked, so the
// ClassifyPoint pre-check and back-off loop go away and the order of the two child calls is free.
// The distance buffers are shared by every level: a node is done with them before it recurses.
//
// Entries go through four SSE lanes per step. The split is computed for every lane and blended in
// (branching around it measured slower), so the only per-entry decisions are two movemasks. Each list is
// written a whole vector at a time and advances by popcount; the gap after it absorbs the unused lanes,
// and reads past count are masked out. Measured on crossfire 16x16: ~58 ns per combo, against ~220 for
// the scalar per-entry version of this descent and ~550 for SegmentBlocked. AVX2 lanes gained up to 8%
// on 16x16 and lost 4-7% at 6x6 and below.
void BatchDescend(const BatchContext &ctx, int nodeIndex, size_t base, size_t count, size_t top)
{
	const dclipnode_t &node = ctx.hull.clipnodes[static_cast<size_t>(nodeIndex)];
	const dplane_t &plane = ctx.hull.planes[static_cast<size_t>(node.planenum)];
	int front = node.children[0];
	int back = node.children[1];

	int side = (2 * count >= ctx.n + ctx.m) ? FillAllDistances(ctx, plane)
		: FillUsedDistances(ctx, plane, ctx.combo + base, count);
	if (side != 0)
	{
		// Hand the whole list to that child as is: same entries, same top, nothing copied.
		int child = node.children[side - 1];
		if (child >= 0)
			BatchDescend(ctx, child, base, count, top);
		else if (child == CONTENTS_SOLID)
			for (size_t k = 0; k < count; ++k)
				ctx.blocked[ctx.combo[base + k]] = 1;
		return;
	}

	const unsigned shift = ctx.shift;
	const uint32_t jMask = ctx.jMask;
	const size_t frontBase = top;
	const size_t backBase = top + count + 4;
	size_t frontCount = 0;
	size_t backCount = 0;
	const __m128 zero = _mm_setzero_ps();
	const __m128 one = _mm_set1_ps(1.0f);
	const __m128 eps = _mm_set1_ps(static_cast<float>(kDistEpsilon));
	const __m128 negEps = _mm_sub_ps(zero, eps);
	for (size_t k = 0; k < count; k += 4)
	{
		size_t at = base + k;
		const uint32_t *c = ctx.combo + at;
		__m128 ts;
		__m128 te;
		unsigned live = 0;
		if (count - k >= 4)
		{
			// Built from register values: four float stores and a 16-byte reload would stall store forwarding.
			ts = _mm_setr_ps(ctx.ts[c[0] >> shift], ctx.ts[c[1] >> shift], ctx.ts[c[2] >> shift],
				ctx.ts[c[3] >> shift]);
			te = _mm_setr_ps(ctx.te[c[0] & jMask], ctx.te[c[1] & jMask], ctx.te[c[2] & jMask],
				ctx.te[c[3] & jMask]);
			for (unsigned lane = 0; lane < 4; ++lane)
				live |= static_cast<unsigned>(ctx.blocked[c[lane]] == 0) << lane;
		}
		else
		{
			alignas(16) float tsLane[4] = {};
			alignas(16) float teLane[4] = {};
			for (size_t lane = 0; lane < count - k; ++lane)
			{
				tsLane[lane] = ctx.ts[c[lane] >> shift];
				teLane[lane] = ctx.te[c[lane] & jMask];
				live |= static_cast<unsigned>(ctx.blocked[c[lane]] == 0) << lane;
			}
			ts = _mm_load_ps(tsLane);
			te = _mm_load_ps(teLane);
		}

		__m128 dt = _mm_sub_ps(te, ts);
		__m128 p1f = _mm_loadu_ps(ctx.p1f + at);
		__m128 p2f = _mm_loadu_ps(ctx.p2f + at);
		__m128 t1 = _mm_add_ps(ts, _mm_mul_ps(dt, p1f));
		__m128 t2 = _mm_add_ps(ts, _mm_mul_ps(dt, p2f));
		__m128 front1 = _mm_cmpge_ps(t1, zero);
		__m128 front2 = _mm_cmpge_ps(t2, zero);
		unsigned m1 = static_cast<unsigned>(_mm_movemask_ps(front1));
		unsigned m2 = static_cast<unsigned>(_mm_movemask_ps(front2));
		unsigned frontMask = (m1 | m2) & live;
		unsigned backMask = ~(m1 & m2) & live;

		// HullCheck's split: eps leans toward p1's side. p1's half goes to p1's child, the rest to the other.
		__m128 frac = _mm_div_ps(_mm_add_ps(t1, _mm_blendv_ps(eps, negEps, front1)), _mm_sub_ps(t1, t2));
		frac = _mm_min_ps(_mm_max_ps(frac, zero), one);
		__m128 midf = _mm_add_ps(p1f, _mm_mul_ps(_mm_sub_ps(p2f, p1f), frac));
		__m128 split = _mm_xor_ps(front1, front2);
		__m128 splitP1Front = _mm_and_ps(split, front1);
		__m128 splitP1Back = _mm_andnot_ps(front1, split);

		__m128i combo = _mm_loadu_si128(reinterpret_cast<const __m128i *>(c));
		if (front >= 0)
		{
			LaneStore(ctx, frontBase + frontCount, frontMask, combo, _mm_blendv_ps(p1f, midf, splitP1Back),
				_mm_blendv_ps(p2f, midf, splitP1Front));
			frontCount += static_cast<size_t>(_mm_popcnt_u32(frontMask));
		}
		else if (front == CONTENTS_SOLID)
		{
			MarkLanes(ctx, at, frontMask);
		}
		if (back >= 0)
		{
			LaneStore(ctx, backBase + backCount, backMask, combo, _mm_blendv_ps(p1f, midf, splitP1Front),
				_mm_blendv_ps(p2f, midf, splitP1Back));
			backCount += static_cast<size_t>(_mm_popcnt_u32(backMask));
		}
		else if (back == CONTENTS_SOLID)
		{
			MarkLanes(ctx, at, backMask);
		}
	}

	size_t childTop = top + 2 * count + 8;
	if (frontCount > 0)
		BatchDescend(ctx, front, frontBase, frontCount, childTop);
	if (backCount > 0)
		BatchDescend(ctx, back, backBase, backCount, childTop);
}

// Above the node where the starts or the ends first disagree, every segment lies wholly on one side, so
// the batch can start there. Starts in one leaf and ends in one leaf make this their common ancestor.
int SkipSharedPrefix(const BatchContext &ctx, int node)
{
	while (node >= 0)
	{
		const dclipnode_t &cn = ctx.hull.clipnodes[static_cast<size_t>(node)];
		int side = FillAllDistances(ctx, ctx.hull.planes[static_cast<size_t>(cn.planenum)]);
		if (side == 0)
			return node;
		node = cn.children[side - 1];
	}
	return node;
}

}

void SegmentsBlockedBatch(const HullTree &hull, int headnode, const float *starts, size_t n,
	const float *ends, size_t m, uint8_t *blocked, BatchTraceScratch &scratch)
{
	size_t combos = n * m;
	if (combos == 0)
		return;

	// The descent needs SSE4.1 (which implies SSSE3) and POPCNT: CPUID leaf 1, ECX bits 19 and 23.
	static const bool hasSimd = [] {
		int info[4] = {};
		__cpuid(info, 1);
		return (info[2] & (1 << 19)) != 0 && (info[2] & (1 << 23)) != 0;
	}();
	if (!hasSimd)
	{
		for (size_t i = 0; i < n; ++i)
		{
			const float *s = starts + 3 * i;
			for (size_t j = 0; j < m; ++j)
			{
				const float *e = ends + 3 * j;
				blocked[i * m + j] =
					SegmentBlocked(hull, headnode, Vec3(s[0], s[1], s[2]), Vec3(e[0], e[1], e[2])) ? 1 : 0;
			}
		}
		return;
	}

	BatchContext ctx = {hull};
	ctx.starts = starts;
	ctx.ends = ends;
	ctx.n = n;
	ctx.m = m;
	ctx.nPad = (n + 3) & ~static_cast<size_t>(3);
	ctx.mPad = (m + 3) & ~static_cast<size_t>(3);
	size_t nPad = ctx.nPad;
	size_t mPad = ctx.mPad;
	if (scratch.ts.size() < nPad)
		scratch.ts.resize(nPad);
	if (scratch.te.size() < mPad)
		scratch.te.resize(mPad);
	if (scratch.soa.size() < 3 * (nPad + mPad))
		scratch.soa.resize(3 * (nPad + mPad));
	ctx.ts = scratch.ts.data();
	ctx.te = scratch.te.data();

	float *soa = scratch.soa.data();
	for (size_t k = 0; k < nPad; ++k)
	{
		const float *p = starts + 3 * (k < n ? k : n - 1);
		soa[k] = p[0];
		soa[nPad + k] = p[1];
		soa[2 * nPad + k] = p[2];
	}
	for (size_t k = 0; k < mPad; ++k)
	{
		const float *p = ends + 3 * (k < m ? k : m - 1);
		soa[3 * nPad + k] = p[0];
		soa[3 * nPad + mPad + k] = p[1];
		soa[3 * nPad + 2 * mPad + k] = p[2];
	}
	ctx.sx = soa;
	ctx.sy = soa + nPad;
	ctx.sz = soa + 2 * nPad;
	ctx.ex = soa + 3 * nPad;
	ctx.ey = soa + 3 * nPad + mPad;
	ctx.ez = soa + 3 * nPad + 2 * mPad;

	int node = SkipSharedPrefix(ctx, headnode);
	uint8_t initial = (node == CONTENTS_SOLID) ? 1 : 0;
	for (size_t c = 0; c < combos; ++c)
		blocked[c] = initial;
	if (node < 0)
		return;

	if (scratch.depth < 0 || scratch.clipnodes != hull.clipnodes || scratch.headnode != headnode)
	{
		std::vector<int> memo(hull.clipnodeCount, -1);
		scratch.depth = HullDepth(hull, headnode, memo);
		scratch.clipnodes = hull.clipnodes;
		scratch.headnode = headnode;
	}

	// Each level adds two lists of at most combos plus their 4-slot gaps; the 8 on top cover reads past
	// the end of the deepest list.
	size_t capacity = combos + static_cast<size_t>(scratch.depth) * (2 * combos + 8) + 8;
	if (scratch.combo.size() < capacity)
	{
		scratch.combo.resize(capacity);
		scratch.p1f.resize(capacity);
		scratch.p2f.resize(capacity);
	}
	ctx.combo = scratch.combo.data();
	ctx.p1f = scratch.p1f.data();
	ctx.p2f = scratch.p2f.data();

	while ((static_cast<size_t>(1) << ctx.shift) < m)
		++ctx.shift;
	ctx.jMask = (static_cast<uint32_t>(1) << ctx.shift) - 1;
	for (size_t i = 0; i < n; ++i)
	{
		for (size_t j = 0; j < m; ++j)
		{
			ctx.combo[i * m + j] = static_cast<uint32_t>((i << ctx.shift) | j);
			ctx.p1f[i * m + j] = 0.0f;
			ctx.p2f[i * m + j] = 1.0f;
		}
	}

	// (i << shift) | j is the caller's i * m + j only for a power-of-two m; any other m works on a copy.
	const bool remap = (static_cast<size_t>(1) << ctx.shift) != m;
	ctx.blocked = blocked;
	if (remap)
	{
		size_t shifted = n << ctx.shift;
		if (scratch.blockedShift.size() < shifted)
			scratch.blockedShift.resize(shifted);
		std::fill(scratch.blockedShift.begin(), scratch.blockedShift.begin() + shifted, uint8_t(0));
		ctx.blocked = scratch.blockedShift.data();
	}

	BatchDescend(ctx, node, 0, combos, combos);

	if (remap)
		for (size_t i = 0; i < n; ++i)
			for (size_t j = 0; j < m; ++j)
				blocked[i * m + j] = ctx.blocked[(i << ctx.shift) | j];
}

TraceResult TraceHullWithEntities(const BspFile &bsp, int worldHeadnode,
    const std::vector<BlockingVolume> &blockers, const Vec3 &start, const Vec3 &end)
{

    TraceResult worldTr = TraceHull(bsp, worldHeadnode, start, end);

    if (worldTr.fraction <= 0.0f)
        return worldTr;

    TraceResult bestTr = worldTr;

    // Same cull as the engine's SV_ClipToLinks: the trace's swept box against the entity's absolute bounds
    // grown by 1 (SV_LinkEdict). Blocker bounds are already hull-expanded, so the sweep is just the segment.
    // A tighter segment-vs-box test changed results: qbsp's hull expansion can reach past those bounds.
    Vec3 sweepMins(std::min(start.x, end.x) - 1.0, std::min(start.y, end.y) - 1.0, std::min(start.z, end.z) - 1.0);
    Vec3 sweepMaxs(std::max(start.x, end.x) + 1.0, std::max(start.y, end.y) + 1.0, std::max(start.z, end.z) + 1.0);
    for (const BlockingVolume &blocker : blockers)
    {
        if (sweepMins.x > blocker.maxs.x || sweepMins.y > blocker.maxs.y || sweepMins.z > blocker.maxs.z ||
            sweepMaxs.x < blocker.mins.x || sweepMaxs.y < blocker.mins.y || sweepMaxs.z < blocker.mins.z)
            continue;

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

