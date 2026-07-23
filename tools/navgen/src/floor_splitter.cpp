#include "floor_splitter.h"

#include <cmath>
#include <cstdlib>

namespace navgen
{

namespace
{

// A face counts as "horizontal enough" to be a floor/ceiling boundary when
// its (inward-pointing, see polytope.h's Plane comment) normal is at least
// this vertical. 0.7 ~= 45 degrees, matching the walkable-slope convention
// used informally elsewhere in this milestone.
constexpr double kHorizontalNormalZ = 0.7;
// Slack (map units) for "these two faces lie on the same physical plane."
constexpr double kPlaneCoincidenceEps = 0.1;
// Openings/pieces below these thresholds are geometric noise, not real
// holes or rooms -- same spirit as portal_builder's kMinPortalArea.
constexpr double kMinHoleArea = 1.0;
constexpr double kMinPieceVolume = 1.0;

struct Hole
{
	size_t upperLeaf; // original leaf index whose bottom face has the opening
	size_t lowerLeaf; // original leaf index the opening drops into
	Winding polygon;  // the opening itself, on the shared plane
	Vec3 planeNormal; // the upper leaf's bottom-face normal (points up/into the leaf)
};

// A leaf's bottom face has an inward normal pointing up; the leaf below it
// has a top face with inward normal pointing down, on the same physical
// plane (opposite orientation). Same convention portal_builder's plane
// bucketing canonicalizes away -- spelled out directly here since we only
// ever compare bottoms against tops.
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

} // namespace

FloorSplitResult SplitUnsupportedColumns(const std::vector<ExtractedLeaf> &leaves, double mapRadius)
{
	// --- 1. Find every floor opening: a bottom face of one leaf coincident
	// (and overlapping) with the top face of another. O(n^2 * faces^2) pair
	// scan -- acceptable at GoldSrc leaf counts (a few hundred per map);
	// bucket by plane like portal_builder if it ever shows up in profiles. ---
	std::vector<Hole> holes;
	for (size_t u = 0; u < leaves.size(); ++u)
	{
		for (const Face &fU : leaves[u].polytope.faces)
		{
			if (fU.plane.normal.z < kHorizontalNormalZ)
				continue; // not a bottom face

			for (size_t l = 0; l < leaves.size(); ++l)
			{
				if (l == u)
					continue;
				for (const Face &fL : leaves[l].polytope.faces)
				{
					if (fL.plane.normal.z > -kHorizontalNormalZ)
						continue; // not a top face
					if (!PlanesCoincideOpposite(fU.plane, fL.plane))
						continue;

					Winding overlap = IntersectCoplanarPolygons(fU.winding, fL.winding, fU.plane.normal);
					if (overlap.size() < 3 || WindingArea(overlap, fU.plane.normal) < kMinHoleArea)
						continue;

					holes.push_back({u, l, overlap, fU.plane.normal});
				}
			}
		}
	}

	std::vector<std::vector<size_t>> holesByUpper(leaves.size());
	for (size_t hi = 0; hi < holes.size(); ++hi)
		holesByUpper[holes[hi].upperLeaf].push_back(hi);

	// --- 2. Carve each holed leaf into supported pieces + air columns. ---
	struct PieceOut
	{
		Polytope poly;
		bool isColumn = false;
		size_t supportLeaf = 0; // original lower-leaf index; only meaningful when isColumn
	};
	// Per original leaf: its output pieces (a single passthrough entry for
	// un-carved leaves).
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
				// A previous hole's carve planes run across the whole leaf,
				// so this hole may straddle several current pieces -- clip
				// the opening down to this piece's own footprint first, and
				// carve only if anything meaningful is left. Near-horizontal
				// planes are skipped: the opening lies ON the bottom plane
				// (and strictly under the top), so only the vertical-ish
				// planes actually shape the footprint, and clipping a polygon
				// by a plane it lies in is numerically pointless anyway.
				Winding clipped = hole.polygon;
				for (const Face &f : piece.faces)
				{
					if (std::fabs(f.plane.normal.z) > 0.99)
						continue;
					clipped = ClipWinding(clipped, f.plane, /*keepFront=*/true, 0.01);
					if (clipped.empty())
						break;
				}
				if (clipped.size() < 3 || WindingArea(clipped, hole.planeNormal) < kMinHoleArea)
				{
					nextPieces.push_back(std::move(piece));
					continue;
				}

				// Carve: successively split the piece by a vertical plane
				// through each opening edge (inward normal, toward the
				// opening's interior). What falls outside any edge is a
				// floor-supported remainder piece; what survives inside all
				// of them is the unsupported column over the opening.
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
						continue; // vertical edge?! -- shouldn't happen on a horizontal polygon
					cut.dist = cut.normal.Dot(e1);
					if (cut.normal.Dot(holeCentroid) - cut.dist < 0.0)
					{
						cut.normal = cut.normal * -1.0;
						cut.dist = -cut.dist;
					}

					Polytope outside = ClipPolytope(current, cut, /*keepFront=*/false, mapRadius);
					current = ClipPolytope(current, cut, /*keepFront=*/true, mapRadius);
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

	// --- 3. Renumber into a flat leaf list, then resolve each column's
	// support pair into new ids. A column's recorded support is an ORIGINAL
	// leaf index; if that leaf was itself carved, pair with its largest
	// supported piece (or, for a leaf that became pure column(s) -- air over
	// air -- its largest column, which union-find chains onward to whatever
	// finally supports THAT). Ambiguity here only matters for multi-tier
	// stacked openings, where any piece of the right lower leaf still merges
	// into the same final blob. ---
	FloorSplitResult result;
	std::vector<std::vector<int>> newIdsOfOriginal(leaves.size());
	std::vector<std::pair<size_t, size_t>> columnRefs; // (originalLeaf, index within outByOriginal[originalLeaf])

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
			// Prefer any supported piece over any column; volume breaks ties.
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

} // namespace navgen
