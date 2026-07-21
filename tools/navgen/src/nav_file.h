#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "area_merger.h"
#include "edge_tagger.h"

namespace navgen
{

// Serializes `areas`/`portals`/`specialEdges` to `path` in the shared
// crossbot_nav on-disk format (src/nav/nav_format.h, included verbatim by
// the runtime loader too). mapName and bspChecksum are embedded in the
// header for the runtime loader's staleness check. Throws
// std::runtime_error on any write failure or on a count that overflows the
// format's fixed-width fields.
void WriteNavFile(const std::string &path, const std::string &mapName, uint32_t bspChecksum,
	const std::vector<Area> &areas, const std::vector<MergedPortal> &portals,
	const std::vector<TaggedSpecialEdge> &specialEdges = {});

} // namespace navgen
