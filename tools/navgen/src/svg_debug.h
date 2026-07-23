#pragma once

#include <string>
#include <vector>

#include "area_merger.h"
#include "door_extractor.h"
#include "polytope.h"
#include "tactical_tagger.h"

namespace navgen
{

// Writes a top-down (X/Y, Z ignored) SVG rendering of the final area/portal
// graph for offline inspection -- an area's fill encodes its structural
// flag (crouch-only vs. ordinary), small badge letters at its centroid mark
// tactical flags (vantage/camp/chokepoint-cover), and a line per portal/
// special-edge connects each pair of neighboring areas' centroids, styled
// (color/dash/arrowhead) by edge type, so the whole connectivity graph is
// visible at a glance without needing a live server. spawnPoints/
// targetPoints are just markers (info_player_*/info_target entity origins),
// not otherwise used.
//
// leafPolytopesById, if non-empty, is looked up by each area's own
// memberLeafIds (indexed by leaf id, e.g. main.cpp's pre-NoMerge `leaves`
// vector) to draw one footprint per ORIGINAL member leaf rather than just
// area.polytope's own boundary. AbsorbTinyAreas folds a tiny area's portal
// connectivity into a larger neighbor without unioning its geometry in (see
// its own header comment) -- area.polytope alone only ever covers the
// survivor's own original leaf, so any absorbed member's volume would
// otherwise render as an unfilled hole even though it's genuinely part of
// that (reachable) area. Pass an empty vector to fall back to area.polytope
// only (e.g. for the pre-second-prune diagnostic dump, which runs before
// AbsorbTinyAreas' bookkeeping would even be meaningful to look up).
// doorAreaInfo, if non-empty, must be parallel to `areas` (door_extractor.h's
// ComputeDoorAreaInfo output) -- a door area (NAV_AREA_DOOR) renders in a
// distinct color from an ordinary or crouch-only area, with a small badge
// showing how it's activated (T=touch, B=button, G=trigger), since it's the
// one area type where "who's whose neighbor" alone doesn't tell the whole
// story -- a door area's own portals into/out of it look identical to any
// other WALK portal, but crossing into one means waiting for it to open.
// pathPoints, if non-empty, is the planned waypoint polyline computed by
// main.cpp via the shared planning modules (NavAStar::FindPath +
// NavPlan::BuildWaypointPlan -- the exact code the runtime bot runs), drawn
// as a highlighted overlay so the debug view shows exactly the route a live
// bot would take. simPathPoints, if non-empty, is the momentum-simulated
// trajectory (NavFollow::Simulate: same steering controller as the live
// bot, integrated against modeled GoldSrc ground physics) -- where it
// deviates from pathPoints is exactly the corner-cut/drift the real bot
// will exhibit. simFallPoints marks where the simulation left supported
// floor (a predicted fall). tracePoints, if non-empty, is a RECORDED live
// bot trajectory (the runtime's nav_trace.txt, via --trace) -- plan vs.
// simulation vs. reality, all on one picture.
void WriteDebugSvg(const std::string &path, const std::string &mapName, const std::vector<Area> &areas,
	const std::vector<MergedPortal> &portals, const std::vector<bool> &crouchAreas,
	const std::vector<TacticalScore> &tacticalScores, const std::vector<Vec3> &spawnPoints,
	const std::vector<Vec3> &targetPoints, const std::vector<Polytope> &leafPolytopesById = {},
	const std::vector<DoorAreaInfo> &doorAreaInfo = {}, const std::vector<Vec3> &pathPoints = {},
	const std::vector<Vec3> &simPathPoints = {}, const std::vector<Vec3> &simFallPoints = {},
	const std::vector<Vec3> &tracePoints = {});

} // namespace navgen
