#include "leaf_extractor.h"

#include <utility>

#include "clipnode_tree.h"
#include "hull_defs.h"

namespace navgen
{

namespace
{

Plane PlaneFromChainEntry(const BspFile &bsp, const PlaneChainEntry &entry)
{
	const dplane_t &dp = bsp.Planes()[static_cast<size_t>(entry.planeIndex)];

	Plane p;
	p.normal = Vec3(dp.normal[0], dp.normal[1], dp.normal[2]);
	p.dist = dp.dist;

	// A chain entry that kept the *back* (child[1]) half-space at its split
	// needs the flipped plane to describe that half-space as a "front" (i.e.
	// dot(normal,x)-dist >= 0) half-space, matching ClipWinding's convention.
	if (!entry.front)
	{
		p.normal = p.normal * -1.0;
		p.dist = -p.dist;
	}
	return p;
}

} // namespace

// Reconstructs the convex polytope bounded by a chain of half-space
// constraints, via the standard "brush from planes" technique used by id's
// map compilers and bspc: for each plane, start with a huge quad lying on
// it, then clip that quad by every other plane in the chain. What survives
// is that plane's contribution to the polytope's boundary. Exported (not
// leaf_extractor-private) because blocking_entities.cpp reuses it verbatim
// to extract a blocking brush entity's own CONTENTS_SOLID volume.
Polytope BuildPolytopeFromChain(const BspFile &bsp, const std::vector<PlaneChainEntry> &chain, double mapRadius)
{
	Polytope poly;

	for (size_t i = 0; i < chain.size(); ++i)
	{
		Plane facePlane = PlaneFromChainEntry(bsp, chain[i]);
		Winding w = BaseWindingForPlane(facePlane, mapRadius);

		for (size_t j = 0; j < chain.size() && !w.empty(); ++j)
		{
			if (j == i)
				continue;
			Plane clipPlane = PlaneFromChainEntry(bsp, chain[j]);
			w = ClipWinding(w, clipPlane, /*keepFront=*/true, 0.01);
		}

		if (!w.empty())
			poly.faces.push_back({facePlane, std::move(w)});
	}

	return poly;
}

std::vector<ExtractedLeaf> ExtractHullLeaves(const BspFile &bsp, const dmodel_t &model, double mapRadius,
	int hullIndex)
{
	std::vector<ExtractedLeaf> leaves;

	WalkClipnodeTree(bsp, model.headnode[hullIndex], /*includeWater=*/false,
		[&](int /*leafContents*/, const std::vector<PlaneChainEntry> &chain)
		{
			Polytope poly = BuildPolytopeFromChain(bsp, chain, mapRadius);
			if (poly.Empty())
				return; // degenerate (epsilon-thin sliver); skip rather than emit garbage

			ExtractedLeaf leaf;
			leaf.id = static_cast<int>(leaves.size());
			leaf.polytope = std::move(poly);
			leaves.push_back(std::move(leaf));
		});

	return leaves;
}

std::vector<ExtractedLeaf> ExtractHull1Leaves(const BspFile &bsp, const dmodel_t &model, double mapRadius)
{
	return ExtractHullLeaves(bsp, model, mapRadius, kHullHuman);
}

} // namespace navgen
