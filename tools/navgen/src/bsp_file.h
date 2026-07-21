#pragma once

#include <cstdint>
#include <string>
#include <vector>

// Pulls in dheader_t/dplane_t/dclipnode_t/dmodel_t/LUMP_*, CONTENTS_*, and
// the `byte` typedef from the vendored, engine-independent HLSDK headers --
// navgen has no Metamod/engine linkage, it only needs the BSP30 struct
// layout and content-code constants, both pure POD/macro definitions.
// Order matters: const.h uses vec3_t (from mathlib.h) and bspfile.h uses
// `byte` (typedef'd by const.h) -- in the full engine build other headers
// pull these in first; here navgen has to do it explicitly, in this order.
#include "mathlib.h"
#include "const.h"
#include "bspfile.h"

namespace navgen
{

// Read-only view over one loaded .bsp file's relevant lumps (planes,
// clipnodes, models, entity text). Loads the whole file into memory --
// GoldSrc BSPs are a few MB at most, no need for streaming/mmap.
class BspFile
{
public:
	// Throws std::runtime_error on any read/format failure.
	explicit BspFile(const std::string &path);

	const std::vector<dplane_t> &Planes() const { return m_planes; }
	const std::vector<dclipnode_t> &Clipnodes() const { return m_clipnodes; }
	const std::vector<dmodel_t> &Models() const { return m_models; }
	const std::string &EntityText() const { return m_entityText; }

	// CRC32 over the concatenated planes/clipnodes/models lump bytes --
	// embedded in the .nav file header so the runtime loader can detect a
	// stale nav file recompiled against a different version of the map.
	uint32_t Checksum() const;

private:
	std::vector<dplane_t> m_planes;
	std::vector<dclipnode_t> m_clipnodes;
	std::vector<dmodel_t> m_models;
	std::string m_entityText;
};

} // namespace navgen
