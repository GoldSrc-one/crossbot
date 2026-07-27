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

	if (!entry.front)
	{
		p.normal = p.normal * -1.0;
		p.dist = -p.dist;
	}
	return p;
}

}

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
			w = ClipWinding(w, clipPlane,  true, 0.01);
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

	WalkClipnodeTree(bsp, model.headnode[hullIndex],  false,
		[&](int  , const std::vector<PlaneChainEntry> &chain)
		{
			Polytope poly = BuildPolytopeFromChain(bsp, chain, mapRadius);
			if (poly.Empty())
				return;

			ExtractedLeaf leaf;
			leaf.id = static_cast<int>(leaves.size());
			leaf.polytope = std::move(poly);
			leaves.push_back(std::move(leaf));
		});

	return leaves;
}

}

