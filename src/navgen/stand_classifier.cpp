#include "stand_classifier.h"

#include <algorithm>
#include <cmath>

#include "clipnode_tree.h"
#include "floor_support.h"
#include "hull_defs.h"

namespace navgen
{

namespace
{

constexpr double kStandProbeEpsilon = 0.1;
constexpr double kMinFragmentVolume = 1.0;
constexpr double kCoverageTolerance = 0.01;
constexpr double kMinStandableArea = 1.0;

Vec3 StandProbeOffset()
{
	return Vec3(0.0, 0.0, kStandProbeLift + kStandProbeEpsilon);
}

bool FloorPointIsCrouchOnly(const BspFile &bsp, int standHeadnode,
	const std::vector<BlockingVolume> &standBlockers, const Vec3 &floorPoint)
{
	return PointInSolid(bsp, standHeadnode, standBlockers, floorPoint + StandProbeOffset());
}

std::vector<const Face *> CollectFloorFaces(const Polytope &poly)
{
	std::vector<const Face *> floors;
	for (const Face &f : poly.faces)
	{
		if (IsFloorFace(f))
			floors.push_back(&f);
	}
	return floors;
}

bool ClassifyPolytope(const BspFile &bsp, int standHeadnode, const std::vector<BlockingVolume> &standBlockers,
	const Polytope &poly)
{
	for (const Face *floor : CollectFloorFaces(poly))
	{
		if (FloorPointIsCrouchOnly(bsp, standHeadnode, standBlockers, WindingCentroid(floor->winding)))
			return true;
	}
	return false;
}

}

std::vector<Winding> ComputeStandableFloor(const BspFile &bsp, int standHeadnode,
	const std::vector<BlockingVolume> &standBlockers, const Face &floorFace)
{
	Vec3 lift = StandProbeOffset();

	std::vector<Winding> solid;
	std::vector<Winding> standable;
	CarveWindingAgainstTree(bsp, standHeadnode, TranslateWinding(floorFace.winding, lift), solid, standable);

	for (const BlockingVolume &blocker : standBlockers)
	{
		if (standable.empty())
			break;

		std::vector<Winding> stillStandable;
		for (const Winding &w : standable)
		{
			std::vector<Winding> blockerSolid;
			std::vector<Winding> blockerEmpty;
			CarveWindingAgainstTree(bsp, blocker.headnode, TranslateWinding(w, blocker.origin * -1.0),
				blockerSolid, blockerEmpty);

			for (const Winding &piece : blockerEmpty)
				stillStandable.push_back(TranslateWinding(piece, blocker.origin));
		}
		standable.swap(stillStandable);
	}

	for (Winding &w : standable)
		w = TranslateWinding(w, lift * -1.0);

	return standable;
}

std::vector<ExtractedLeaf> SplitLeavesByStandClearance(const BspFile &bsp, int standHeadnode,
	const std::vector<BlockingVolume> &standBlockers, const std::vector<ExtractedLeaf> &leaves, double mapRadius,
	std::vector<bool> &isCrouchLeafOut, size_t *splitOut)
{
	std::vector<ExtractedLeaf> out;
	isCrouchLeafOut.clear();
	size_t split = 0;

	auto emit = [&](Polytope poly, bool crouch, const ExtractedLeaf &from)
	{
		ExtractedLeaf piece;
		piece.id = static_cast<int>(out.size());
		piece.polytope = std::move(poly);
		piece.passableIndex = from.passableIndex;
		piece.elevatorStopId = from.elevatorStopId;
		out.push_back(std::move(piece));
		isCrouchLeafOut.push_back(crouch);
	};

	for (const ExtractedLeaf &leaf : leaves)
	{
		std::vector<const Face *> floors = CollectFloorFaces(leaf.polytope);
		if (floors.empty())
		{
			emit(leaf.polytope, false, leaf);
			continue;
		}

		double floorArea = 0.0;
		for (const Face *floor : floors)
			floorArea += WindingArea(floor->winding, floor->plane.normal);

		std::vector<Winding> standable;
		std::vector<size_t> standableFace;
		for (size_t fi = 0; fi < floors.size(); ++fi)
		{
			for (Winding &w : ComputeStandableFloor(bsp, standHeadnode, standBlockers, *floors[fi]))
			{
				standable.push_back(std::move(w));
				standableFace.push_back(fi);
			}
		}

		double standableArea = 0.0;
		for (size_t si = 0; si < standable.size(); ++si)
			standableArea += WindingArea(standable[si], floors[standableFace[si]]->plane.normal);

		if (standableArea <= kMinStandableArea)
		{
			emit(leaf.polytope, true, leaf);
			continue;
		}

		if (standableArea >= floorArea * (1.0 - kCoverageTolerance))
		{
			emit(leaf.polytope, false, leaf);
			continue;
		}

		std::vector<Plane> cuts;
		for (size_t si = 0; si < standable.size(); ++si)
		{
			const Winding &w = standable[si];
			const Winding &owner = floors[standableFace[si]]->winding;

			for (size_t i = 0; i < w.size(); ++i)
			{
				const Vec3 &a = w[i];
				const Vec3 &b = w[(i + 1) % w.size()];

				if (OnWindingBoundary((a + b) * 0.5, owner))
					continue;

				Plane plane;
				if (VerticalPlaneThroughEdge(a, b, plane) && !HasPlane(cuts, plane))
					cuts.push_back(plane);
			}
		}

		std::vector<Polytope> fragments;
		fragments.push_back(leaf.polytope);

		for (const Plane &cut : cuts)
		{
			std::vector<Polytope> next;
			for (const Polytope &frag : fragments)
			{
				Polytope front = ClipPolytope(frag, cut, true, mapRadius);
				Polytope back = ClipPolytope(frag, cut, false, mapRadius);

				if (!front.Empty() && front.Volume() > kMinFragmentVolume)
					next.push_back(std::move(front));
				if (!back.Empty() && back.Volume() > kMinFragmentVolume)
					next.push_back(std::move(back));
			}
			fragments.swap(next);
		}

		if (fragments.size() > 1)
			++split;

		for (Polytope &frag : fragments)
		{
			bool crouch = ClassifyPolytope(bsp, standHeadnode, standBlockers, frag);
			emit(std::move(frag), crouch, leaf);
		}
	}

	if (splitOut)
		*splitOut = split;

	return out;
}

}

