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
	const std::string &EntityText() const { return m_entityText; }

	uint32_t Checksum() const;

private:
	std::vector<dplane_t> m_planes;
	std::vector<dclipnode_t> m_clipnodes;
	std::vector<dmodel_t> m_models;
	std::string m_entityText;
};

}

