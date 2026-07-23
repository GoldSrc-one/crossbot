#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "area_merger.h"
#include "edge_tagger.h"
#include "tactical_tagger.h"

namespace navgen
{

// Serializes `areas`/`portals`/`specialEdges` to `path` in the shared
// crossbot_nav on-disk format (src/nav/nav_format.h, included verbatim by
// the runtime loader too). mapName and bspChecksum are embedded in the
// header for the runtime loader's staleness check. `tacticalScores`, if
// non-empty, must be parallel to `areas` (same size, same order) --
// Milestone 2e's per-area vantage/camp/chokepoint-cover data; omitted
// (default empty) means every area writes NAV_AREA_NONE and zeroed scores.
// `crouchAreas`, if non-empty, must also be parallel to `areas`: true marks
// an area that only has hull-3 (duck) clearance somewhere in it, sourced
// from leaf_extractor's hull-3 pass rather than hull-1 -- sets
// NAV_AREA_CROUCH so the runtime knows to hold duck while inside.
// Throws std::runtime_error on any write failure or on a count that
// overflows the format's fixed-width fields.
void WriteNavFile(const std::string &path, const std::string &mapName, uint32_t bspChecksum,
	const std::vector<Area> &areas, const std::vector<MergedPortal> &portals,
	const std::vector<TaggedSpecialEdge> &specialEdges = {},
	const std::vector<TacticalScore> &tacticalScores = {}, const std::vector<bool> &crouchAreas = {});

} // namespace navgen
