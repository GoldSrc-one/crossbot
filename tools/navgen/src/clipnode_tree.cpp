#include "clipnode_tree.h"

namespace navgen
{

namespace
{

void WalkRecursive(const BspFile &bsp, int nodeIndex, bool includeWater,
	std::vector<PlaneChainEntry> &chain, const LeafCallback &callback)
{
	if (nodeIndex < 0)
	{
		// Leaf: nodeIndex itself is the CONTENTS_* value.
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

} // namespace

void WalkClipnodeTree(const BspFile &bsp, int headnode, bool includeWater, const LeafCallback &callback)
{
	std::vector<PlaneChainEntry> chain;
	WalkRecursive(bsp, headnode, includeWater, chain, callback);
}

namespace
{

void WalkForContentsRecursive(const BspFile &bsp, int nodeIndex, int targetContents,
	std::vector<PlaneChainEntry> &chain, const LeafCallback &callback)
{
	if (nodeIndex < 0)
	{
		if (nodeIndex == targetContents)
			callback(nodeIndex, chain);
		return;
	}

	const dclipnode_t &node = bsp.Clipnodes()[static_cast<size_t>(nodeIndex)];

	chain.push_back({node.planenum, true});
	WalkForContentsRecursive(bsp, node.children[0], targetContents, chain, callback);
	chain.pop_back();

	chain.push_back({node.planenum, false});
	WalkForContentsRecursive(bsp, node.children[1], targetContents, chain, callback);
	chain.pop_back();
}

} // namespace

void WalkClipnodeTreeForContents(const BspFile &bsp, int headnode, int targetContents, const LeafCallback &callback)
{
	std::vector<PlaneChainEntry> chain;
	WalkForContentsRecursive(bsp, headnode, targetContents, chain, callback);
}

namespace
{

void CarveRecursive(const BspFile &bsp, int nodeIndex, Polytope poly, int removeContents, double mapRadius,
	double minPieceVolume, double epsilon, std::vector<Polytope> &out)
{
	if (poly.Empty())
		return;

	if (nodeIndex < 0)
	{
		// Leaf: nodeIndex itself is the CONTENTS_* value.
		if (nodeIndex != removeContents && poly.Volume() > minPieceVolume)
			out.push_back(std::move(poly));
		return;
	}

	const dclipnode_t &node = bsp.Clipnodes()[static_cast<size_t>(nodeIndex)];
	const dplane_t &dp = bsp.Planes()[static_cast<size_t>(node.planenum)];
	Plane plane;
	plane.normal = Vec3(dp.normal[0], dp.normal[1], dp.normal[2]);
	plane.dist = dp.dist;

	// children[0] is the front side (dot >= dist), children[1] the back --
	// same convention ClassifyPoint descends by.
	Polytope front = ClipPolytope(poly, plane, /*keepFront=*/true, mapRadius, epsilon);
	Polytope back = ClipPolytope(poly, plane, /*keepFront=*/false, mapRadius, epsilon);
	CarveRecursive(bsp, node.children[0], std::move(front), removeContents, mapRadius, minPieceVolume, epsilon, out);
	CarveRecursive(bsp, node.children[1], std::move(back), removeContents, mapRadius, minPieceVolume, epsilon, out);
}

} // namespace

std::vector<Polytope> CarvePolytopeAgainstTree(const BspFile &bsp, int headnode, const Polytope &poly,
	int removeContents, double mapRadius, double minPieceVolume, double epsilon)
{
	std::vector<Polytope> out;
	CarveRecursive(bsp, headnode, poly, removeContents, mapRadius, minPieceVolume, epsilon, out);
	return out;
}

int ClassifyPoint(const BspFile &bsp, int headnode, const Vec3 &point)
{
	int nodeIndex = headnode;
	while (nodeIndex >= 0)
	{
		const dclipnode_t &node = bsp.Clipnodes()[static_cast<size_t>(nodeIndex)];
		const dplane_t &plane = bsp.Planes()[static_cast<size_t>(node.planenum)];
		double d = static_cast<double>(plane.normal[0]) * point.x + static_cast<double>(plane.normal[1]) * point.y +
			static_cast<double>(plane.normal[2]) * point.z - plane.dist;
		nodeIndex = (d >= 0.0) ? node.children[0] : node.children[1];
	}
	return nodeIndex;
}

} // namespace navgen
