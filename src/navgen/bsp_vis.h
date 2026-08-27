#pragma once

#include <cstdint>
#include <vector>

#include "bsp_file.h"
#include "clipnode_tree.h"
#include "polytope.h"

namespace navgen
{

std::vector<dclipnode_t> BuildHull0Clipnodes(const BspFile &bsp);

class VisIndex
{
public:

	explicit VisIndex(const BspFile &bsp);

	HullTree Hull0() const;

	bool HasVisData() const { return m_hasVis; }

	size_t LeafCount() const { return m_leafCount; }

	size_t RowBytes() const { return m_rowBytes; }

	int LeafForPoint(const Vec3 &point) const;

	bool LeafVisible(int fromLeaf, int toLeaf) const;

	bool RowVisible(const uint8_t *row, int toLeaf) const;

private:

	friend class VisRowReader;

	const BspFile &m_bsp;
	std::vector<dclipnode_t> m_hull0;
	size_t m_leafCount = 0;
	size_t m_rowBytes = 0;
	bool m_hasVis = false;
};

class VisRowReader
{
public:

	explicit VisRowReader(const VisIndex &vis);

	const uint8_t *VisRow(int fromLeaf);

	bool RowVisible(const uint8_t *row, int toLeaf) const { return m_vis.RowVisible(row, toLeaf); }

private:

	void DecompressRow(int leaf);

	const VisIndex &m_vis;
	std::vector<uint8_t> m_row;
	int m_rowLeaf = -1;
};

}
