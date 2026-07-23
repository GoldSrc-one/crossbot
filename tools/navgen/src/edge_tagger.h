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
	// Door edges only: true if this door has a targetname (so it can't be
	// touch-opened) but TagDoor still resolved a func_button that opens it
	// (directly, or via one level of trigger_relay/multi_manager
	// indirection) -- buttonPos is that button's position, matching
	// EDGE_ELEVATOR's own button-then-ride fields.
	bool needsButton = false;
	Vec3 buttonPos;
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
// MergedPortal. Not a real physics simulation -- no accounting for a bot's
// actual jump arc, just a bounded geometric heuristic, the same kind of
// tunable approximation used elsewhere in this tool (e.g. area_merger's
// coverage tolerance) -- but it does now reject candidates via a coarse
// hull-1 point-classification sample along the straight line between the
// two floor points (SegmentClear in edge_tagger.cpp), added after a real
// live failure on crossfire.bsp: without it, distance/height alone happily
// connected two rooms with a solid wall between them, and a bot sent across
// that "jump" just walked into the wall and bounced off it forever.
void ProbeJumpGaps(const BspFile &bsp, int headnode, const std::vector<Area> &areas,
	std::vector<MergedPortal> &portals);

// Reclassifies existing portals whose two areas' floors (approximated by
// AABB mins.z) differ by more than a safe-fall threshold as one-way
// EDGE_DROP edges. The threshold is an approximate, tunable stand-in for
// GoldSrc's speed-based fall-damage formula, not a reverse-engineered
// constant -- revisit against real engine behavior if it proves off.
void TagDropOffs(const std::vector<Area> &areas, std::vector<MergedPortal> &portals);

// True per area if any of its member (pre-merge) leaf ids is a hull-3-only
// (crouch) leaf, per isCrouchLeaf (indexed by leaf id, built by main.cpp's
// hull-3 extraction pass). Used both by ProbeCrouchLinks internally and by
// main.cpp again at the very end to set NAV_AREA_CROUCH for the baked file
// -- factored out so both call sites agree without duplicating the logic.
std::vector<bool> ComputeCrouchAreaFlags(const std::vector<Area> &areas, const std::vector<bool> &isCrouchLeaf);

// Bridges a crouch-only area (hull-3 leaf with no hull-1 counterpart) to its
// nearest hull-1 neighbor(s) with a plain EDGE_WALK portal. portal_builder's
// coplanar-shared-face matching (the normal way two adjacent leaves connect)
// doesn't work across a hull-1/hull-3 boundary here: hull1 and hull3 share
// the same horizontal (X/Y) footprint (hull_defs.h) so a *side* wall lands
// on the same plane for both, but the geometry that actually gates a crouch
// passage is the *ceiling* dropping low enough to block hull1 -- a Z-normal
// plane, and hull1/hull3's differing vertical half-heights put that ceiling
// constraint at a different clip distance per hull, so it's never coplanar
// between the two leaves' polytopes. This is a proximity heuristic in the
// same spirit as ProbeJumpGaps (nearest floor point, distance/height gate,
// then a straight-line occlusion sample) rather than portal_builder's exact
// shared-face intersection -- the differences are: it only ever considers
// crouch-area candidates, its distance/height gates are the plain walkable-
// step range (not jump range, since ducking through is still just walking,
// no jump input needed), and its clearance sample is at duck height
// (kDuckHullMaxZ) using hull 3's own clipnode tree, not hull 1's.
void ProbeCrouchLinks(const BspFile &bsp, int duckHeadnode, const std::vector<Area> &areas,
	const std::vector<bool> &isCrouchLeaf, std::vector<MergedPortal> &portals);

} // namespace navgen
