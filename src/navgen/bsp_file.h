#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "mathlib.h"
#include "const.h"
#include "bspfile.h"

namespace navgen
{

class BspFile
{
public:

	explicit BspFile(const std::string &path);

	const std::vector<dplane_t> &Planes() const { return m_planes; }
	const std::vector<dclipnode_t> &Clipnodes() const { return m_clipnodes; }
	const std::vector<dmodel_t> &Models() const { return m_models; }
	const std::vector<dnode_t> &Nodes() const { return m_nodes; }
	const std::vector<dleaf_t> &Leafs() const { return m_leafs; }
	const std::vector<uint8_t> &VisData() const { return m_visData; }
	const std::string &EntityText() const { return m_entityText; }

	uint32_t Checksum() const;

private:
	std::vector<dplane_t> m_planes;
	std::vector<dclipnode_t> m_clipnodes;
	std::vector<dmodel_t> m_models;
	std::vector<dnode_t> m_nodes;
	std::vector<dleaf_t> m_leafs;
	std::vector<uint8_t> m_visData;
	std::string m_entityText;
};

}

