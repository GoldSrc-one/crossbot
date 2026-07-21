#pragma once

#include <string>
#include <vector>

#include "area_merger.h"
#include "bsp_file.h"
#include "entity_parser.h"
#include "nav_format.h"

namespace navgen
{

struct TaggedSpecialEdge
{
	int areaFrom = -1;
	int areaTo = -1;
	uint8_t type = crossbot_nav::EDGE_LADDER;
	Vec3 anchorFrom;
	Vec3 anchorTo;
	std::string targetname; // Door edges only: re-resolved to a live edict via FIND_ENTITY_BY_STRING at runtime
};

// func_ladder -> a Ladder SpecialEdge between whichever area overlapping
// the ladder's bounding box has the lowest centroid (bottom anchor) and
// whichever has the highest (top anchor). func_door/func_door_rotating ->
// a Door SpecialEdge for every pair of areas found within a small margin
// of the door's bounding box (its footprint was already excluded from the
// navigable leaf set by blocking_entities, so the "areas adjacent to the
// hole" are exactly what this margin search recovers).
std::vector<TaggedSpecialEdge> TagSpecialEdges(const BspFile &bsp, const std::vector<Entity> &entities,
	const std::vector<Area> &areas);

// Proximity-heuristic jump-gap probing: for every pair of areas that
// aren't already portal-connected, within a horizontal center-distance and
// vertical delta plausible for a running jump, appends a new EDGE_JUMP
// MergedPortal. This is *not* a physics simulation -- no trace validation
// that the gap is actually clear, no accounting for a bot's actual jump
// arc -- just a bounded geometric heuristic, the same kind of tunable
// approximation used elsewhere in this tool (e.g. area_merger's coverage
// tolerance). Revisit with real trace-based validation if false positives
// (jump edges over gaps that aren't really crossable) turn out to matter
// once Milestone 2c's bots actually try to use them.
void ProbeJumpGaps(const std::vector<Area> &areas, std::vector<MergedPortal> &portals);

// Reclassifies existing portals whose two areas' floors (approximated by
// AABB mins.z) differ by more than a safe-fall threshold as one-way
// EDGE_DROP edges. The threshold is an approximate, tunable stand-in for
// GoldSrc's speed-based fall-damage formula, not a reverse-engineered
// constant -- revisit against real engine behavior if it proves off.
void TagDropOffs(const std::vector<Area> &areas, std::vector<MergedPortal> &portals);

} // namespace navgen
