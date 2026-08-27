#include "bsp_vis.h"

#include <cstring>
#include <stdexcept>

namespace navgen
{

std::vector<dclipnode_t> BuildHull0Clipnodes(const BspFile &bsp)
{
	const std::vector<dnode_t> &nodes = bsp.Nodes();
	const std::vector<dleaf_t> &leafs = bsp.Leafs();

	std::vector<dclipnode_t> out(nodes.size());
	for (size_t i = 0; i < nodes.size(); ++i)
	{
		const dnode_t &in = nodes[i];
		out[i].planenum = in.planenum;
		for (int j = 0; j < 2; ++j)
		{
			short child = in.children[j];
			if (child >= 0)
			{
				out[i].children[j] = child;
			}
			else
			{
				size_t leafIndex = static_cast<size_t>(-1 - child);
				if (leafIndex >= leafs.size())
					throw std::runtime_error("BSP node references a leaf out of range");
				out[i].children[j] = static_cast<short>(leafs[leafIndex].contents);
			}
		}
	}

	return out;
}

VisIndex::VisIndex(const BspFile &bsp)
	: m_bsp(bsp)
{
	m_hull0 = BuildHull0Clipnodes(bsp);
	m_leafCount = bsp.Leafs().size();
	m_rowBytes = (m_leafCount + 7) >> 3;
	m_hasVis = !bsp.VisData().empty() && m_leafCount > 1;
}

HullTree VisIndex::Hull0() const
{
	HullTree hull;
	hull.clipnodes = m_hull0.data();
	hull.clipnodeCount = m_hull0.size();
	hull.planes = m_bsp.Planes().data();
	hull.planeCount = m_bsp.Planes().size();
	return hull;
}

int VisIndex::LeafForPoint(const Vec3 &point) const
{
	const std::vector<dnode_t> &nodes = m_bsp.Nodes();
	const std::vector<dplane_t> &planes = m_bsp.Planes();
	if (nodes.empty())
		return 0;

	int nodeIndex = 0;
	while (nodeIndex >= 0)
	{
		const dnode_t &node = nodes[static_cast<size_t>(nodeIndex)];
		const dplane_t &plane = planes[static_cast<size_t>(node.planenum)];
		double d = static_cast<double>(plane.normal[0]) * point.x +
			static_cast<double>(plane.normal[1]) * point.y +
			static_cast<double>(plane.normal[2]) * point.z - static_cast<double>(plane.dist);
		nodeIndex = (d >= 0.0) ? node.children[0] : node.children[1];
	}

	return -1 - nodeIndex;
}

bool VisIndex::RowVisible(const uint8_t *row, int toLeaf) const
{
	if (!row)
		return true;
	if (toLeaf <= 0 || static_cast<size_t>(toLeaf) >= m_leafCount)
		return true;

	size_t bit = static_cast<size_t>(toLeaf - 1);
	return (row[bit >> 3] & (1u << (bit & 7))) != 0;
}

bool VisIndex::LeafVisible(int fromLeaf, int toLeaf) const
{
	VisRowReader reader(*this);
	return RowVisible(reader.VisRow(fromLeaf), toLeaf);
}

VisRowReader::VisRowReader(const VisIndex &vis)
	: m_vis(vis)
{
	m_row.assign(vis.RowBytes(), 0xFF);
}

void VisRowReader::DecompressRow(int leaf)
{
	if (m_rowLeaf == leaf)
		return;

	m_rowLeaf = leaf;

	size_t rowBytes = m_vis.m_rowBytes;

	if (!m_vis.m_hasVis || leaf <= 0 || static_cast<size_t>(leaf) >= m_vis.m_leafCount)
	{
		std::memset(m_row.data(), 0xFF, rowBytes);
		return;
	}

	int visofs = m_vis.m_bsp.Leafs()[static_cast<size_t>(leaf)].visofs;
	if (visofs < 0)
	{
		std::memset(m_row.data(), 0xFF, rowBytes);
		return;
	}

	const std::vector<uint8_t> &vis = m_vis.m_bsp.VisData();
	size_t in = static_cast<size_t>(visofs);
	size_t out = 0;

	while (out < rowBytes)
	{
		if (in >= vis.size())
		{
			std::memset(m_row.data() + out, 0, rowBytes - out);
			return;
		}

		if (vis[in] != 0)
		{
			m_row[out++] = vis[in++];
			continue;
		}

		if (in + 1 >= vis.size())
		{
			std::memset(m_row.data() + out, 0, rowBytes - out);
			return;
		}

		size_t run = vis[in + 1];
		in += 2;
		while (run > 0 && out < rowBytes)
		{
			m_row[out++] = 0;
			--run;
		}
	}
}

const uint8_t *VisRowReader::VisRow(int fromLeaf)
{
	if (!m_vis.m_hasVis || fromLeaf <= 0 || static_cast<size_t>(fromLeaf) >= m_vis.m_leafCount)
		return nullptr;
	if (m_vis.m_bsp.Leafs()[static_cast<size_t>(fromLeaf)].visofs < 0)
		return nullptr;

	DecompressRow(fromLeaf);
	return m_row.data();
}

}
