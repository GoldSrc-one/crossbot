#include "floor_splitter.h"

#include <cmath>
#include <cstdlib>

namespace navgen
{

namespace
{

constexpr double kHorizontalNormalZ = 0.7;

constexpr double kPlaneCoincidenceEps = 0.1;

constexpr double kMinHoleArea = 1.0;
constexpr double kMinPieceVolume = 1.0;

struct Hole
{
	size_t upperLeaf;
	size_t lowerLeaf;
	Winding polygon;
	Vec3 planeNormal;
};

bool PlanesCoincideOpposite(const Plane &bottom, const Plane &top)
{
	return std::fabs(bottom.normal.x + top.normal.x) < 0.01 &&
		std::fabs(bottom.normal.y + top.normal.y) < 0.01 &&
		std::fabs(bottom.normal.z + top.normal.z) < 0.01 &&
		std::fabs(bottom.dist + top.dist) < kPlaneCoincidenceEps;
}

Vec3 WindingCentroid(const Winding &w)
{
	Vec3 sum(0, 0, 0);
	for (const Vec3 &v : w)
		sum = sum + v;
	return (w.empty()) ? sum : sum * (1.0 / static_cast<double>(w.size()));
}

}

FloorSplitResult SplitUnsupportedColumns(const std::vector<ExtractedLeaf> &leaves, double mapRadius)
{

	std::vector<Hole> holes;
	for (size_t u = 0; u < leaves.size(); ++u)
	{
		for (const Face &fU : leaves[u].polytope.faces)
		{
			if (fU.plane.normal.z < kHorizontalNormalZ)
				continue;

			for (size_t l = 0; l < leaves.size(); ++l)
			{
				if (l == u)
					continue;
				for (const Face &fL : leaves[l].polytope.faces)
				{
					if (fL.plane.normal.z > -kHorizontalNormalZ)
						continue;
					if (!PlanesCoincideOpposite(fU.plane, fL.plane))
						continue;

					Winding overlap = IntersectCoplanarPolygons(fU.winding, fL.winding, fU.plane.normal);
					if (overlap.size() < 3 || WindingArea(overlap, fU.plane.normal) < kMinHoleArea)
						continue;

					double upperBottomArea = WindingArea(fU.winding, fU.plane.normal);
					double overlapArea = WindingArea(overlap, fU.plane.normal);
					if (overlapArea > upperBottomArea - 1.0)
						continue;

					holes.push_back({u, l, overlap, fU.plane.normal});
				}
			}
		}
	}

	std::vector<std::vector<size_t>> holesByUpper(leaves.size());
	for (size_t hi = 0; hi < holes.size(); ++hi)
		holesByUpper[holes[hi].upperLeaf].push_back(hi);

	struct PieceOut
	{
		Polytope poly;
		bool isColumn = false;
		size_t supportLeaf = 0;
	};

	std::vector<std::vector<PieceOut>> outByOriginal(leaves.size());

	for (size_t u = 0; u < leaves.size(); ++u)
	{
		if (holesByUpper[u].empty())
		{
			outByOriginal[u].push_back({leaves[u].polytope, false, 0});
			continue;
		}

		std::vector<Polytope> pieces = {leaves[u].polytope};
		std::vector<PieceOut> columns;

		for (size_t hi : holesByUpper[u])
		{
			const Hole &hole = holes[hi];
			std::vector<Polytope> nextPieces;

			for (Polytope &piece : pieces)
			{

				Winding clipped = hole.polygon;
				for (const Face &f : piece.faces)
				{
					if (std::fabs(f.plane.normal.z) > 0.99)
						continue;
					clipped = ClipWinding(clipped, f.plane,  true, 0.01);
					if (clipped.empty())
						break;
				}
				if (clipped.size() < 3 || WindingArea(clipped, hole.planeNormal) < kMinHoleArea)
				{
					nextPieces.push_back(std::move(piece));
					continue;
				}

				Vec3 holeCentroid = WindingCentroid(clipped);
				Polytope current = piece;
				for (size_t ei = 0; ei < clipped.size() && !current.Empty(); ++ei)
				{
					Vec3 e1 = clipped[ei];
					Vec3 e2 = clipped[(ei + 1) % clipped.size()];
					Vec3 edgeDir = (e2 - e1).Normalized();
					if (edgeDir.Length() < 1e-9)
						continue;

					Plane cut;
					cut.normal = edgeDir.Cross(Vec3(0, 0, 1)).Normalized();
					if (cut.normal.Length() < 1e-9)
						continue;
					cut.dist = cut.normal.Dot(e1);
					if (cut.normal.Dot(holeCentroid) - cut.dist < 0.0)
					{
						cut.normal = cut.normal * -1.0;
						cut.dist = -cut.dist;
					}

					Polytope outside = ClipPolytope(current, cut,  false, mapRadius);
					current = ClipPolytope(current, cut,  true, mapRadius);
					if (!outside.Empty() && outside.Volume() > kMinPieceVolume)
						nextPieces.push_back(std::move(outside));
				}
				if (!current.Empty() && current.Volume() > kMinPieceVolume)
					columns.push_back({std::move(current), true, hole.lowerLeaf});
			}

			pieces = std::move(nextPieces);
		}

		for (Polytope &p : pieces)
			outByOriginal[u].push_back({std::move(p), false, 0});
		for (PieceOut &c : columns)
			outByOriginal[u].push_back(std::move(c));
	}

	FloorSplitResult result;
	std::vector<std::vector<int>> newIdsOfOriginal(leaves.size());
	std::vector<std::pair<size_t, size_t>> columnRefs;

	for (size_t u = 0; u < leaves.size(); ++u)
	{
		for (size_t k = 0; k < outByOriginal[u].size(); ++k)
		{
			ExtractedLeaf leaf;
			leaf.id = static_cast<int>(result.leaves.size());
			leaf.polytope = std::move(outByOriginal[u][k].poly);
			newIdsOfOriginal[u].push_back(leaf.id);
			if (outByOriginal[u][k].isColumn)
				columnRefs.push_back({u, k});
			result.leaves.push_back(std::move(leaf));
		}
	}

	auto RepresentativeNewId = [&](size_t originalLeaf) -> int
	{
		const std::vector<PieceOut> &outs = outByOriginal[originalLeaf];
		const std::vector<int> &ids = newIdsOfOriginal[originalLeaf];
		int best = -1;
		double bestVolume = -1.0;
		bool bestIsSupported = false;
		for (size_t k = 0; k < outs.size(); ++k)
		{
			double vol = result.leaves[static_cast<size_t>(ids[k])].polytope.Volume();
			bool supported = !outs[k].isColumn;

			if (best < 0 || (supported && !bestIsSupported) ||
				(supported == bestIsSupported && vol > bestVolume))
			{
				best = ids[k];
				bestVolume = vol;
				bestIsSupported = supported;
			}
		}
		return best;
	};

	for (const auto &[u, k] : columnRefs)
	{
		int columnId = newIdsOfOriginal[u][k];
		int supportId = RepresentativeNewId(outByOriginal[u][k].supportLeaf);
		if (supportId >= 0 && supportId != columnId)
			result.columnPairs.push_back({columnId, supportId});
	}

	return result;
}

}

