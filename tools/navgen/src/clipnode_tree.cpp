#include "clipnode_tree.h"

namespace navgen
{

namespace
{

// Classify a clipnode subtree relative to targetContents.
// Returns:
//   1 = ALL_SOLID (entire subtree is targetContents)
//   0 = ALL_EMPTY (no leaf in subtree is targetContents)
//  -1 = MIXED (contains both targetContents and other contents)
int ClassifyNodeContents(const BspFile &bsp, int nodeIndex, int targetContents)
{
	if (nodeIndex < 0)
	{
		return (nodeIndex == targetContents) ? 1 : 0;
	}

	const dclipnode_t &node = bsp.Clipnodes()[static_cast<size_t>(nodeIndex)];
	int front = ClassifyNodeContents(bsp, node.children[0], targetContents);
	int back = ClassifyNodeContents(bsp, node.children[1], targetContents);

	if (front == 1 && back == 1)
		return 1; // ALL_SOLID
	if (front == 0 && back == 0)
		return 0; // ALL_EMPTY
	return -1; // MIXED
}

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

	// Optimization: classify the subtree to avoid unnecessary splits.
	// If the entire subtree is solid, the polytope is inside the blocker -> discard.
	// If the entire subtree is empty, the polytope is outside -> keep as-is.
	// Only split if the subtree is mixed (contains both solid and empty leaves).
	static std::unordered_map<int, int> nodeClassification;
	nodeClassification.clear();

	std::function<int(int)> classify = [&](int idx) -> int {
		if (idx < 0)
			return (idx == removeContents) ? 1 : 0;

		auto it = nodeClassification.find(idx);
		if (it != nodeClassification.end())
			return it->second;

		const dclipnode_t &node = bsp.Clipnodes()[static_cast<size_t>(idx)];
		int front = classify(node.children[0]);
		int back = classify(node.children[1]);

		int result;
		if (front == 1 && back == 1)
			result = 1; // ALL_SOLID
		else if (front == 0 && back == 0)
			result = 0; // ALL_EMPTY
		else
			result = -1; // MIXED

		nodeClassification[idx] = result;
		return result;
	};

	int classification = classify(nodeIndex);
	if (classification == 1)
	{
		// Entire subtree is solid - polytope is inside blocker, discard it
		return;
	}
	if (classification == 0)
	{
		// Entire subtree is empty - polytope is outside blocker, keep as-is
		if (poly.Volume() > minPieceVolume)
			out.push_back(std::move(poly));
		return;
	}

	// Mixed: need to split
	const dclipnode_t &node = bsp.Clipnodes()[static_cast<size_t>(nodeIndex)];
	const dplane_t &dp = bsp.Planes()[static_cast<size_t>(node.planenum)];
	Plane plane;
	plane.normal = Vec3(dp.normal[0], dp.normal[1], dp.normal[2]);
	plane.dist = dp.dist;

	// children[0] is the front side (dot >= dist), children[1] the back
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
