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

} // namespace navgen
