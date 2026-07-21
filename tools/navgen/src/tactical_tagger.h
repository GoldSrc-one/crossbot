#pragma once

#include <vector>

#include "area_merger.h"
#include "bsp_file.h"

namespace navgen
{

struct TacticalScore
{
	double exposureScore = 0.0;    // total unobstructed sightline reach to other areas -- rewards a few long
	                                // sightlines (the real vantage-point trait) over many short ones
	double concealmentScore = 0.0; // inverse of *nearby* visibility specifically (see kNearbyThreatRadius) --
	                                // an area can have a long sightline to somewhere far away (good vantage)
	                                // while still being hidden from anything close enough to be an immediate
	                                // threat, so this is a distinct measurement from exposureScore, not its
	                                // literal inverse
	double dangerScore = 0.0;      // how exposed this area is *to* other high-exposure areas -- each visible
	                                // neighbor's own exposureScore propagated back onto this one
	bool isVantage = false;
	bool isCamp = false;
	bool isChokepointCover = false;
};

// Computes per-area tactical scores from distance-bounded, hull-1-line-of-
// sight-validated area-to-area visibility. Run after MergeAreas and
// PruneUnreachableAreas (only scores areas that survive both) and before
// nav_file serialization -- see the plan's "Tactical tagging" architecture
// note. These are explicitly heuristic, tunable scores, not ground truth,
// the same role a human's hand-placed "sniper spot" flag plays in every
// reference bot surveyed for this project (none of them derive it
// automatically).
std::vector<TacticalScore> TagTactical(const BspFile &bsp, int hull1Headnode, const std::vector<Area> &areas,
	const std::vector<MergedPortal> &portals);

} // namespace navgen
