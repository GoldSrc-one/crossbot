#pragma once

#include <cstdint>

// On-disk layout for a baked <mapname>.nav file, shared verbatim between
// tools/navgen (the writer) and the runtime plugin's src/nav/ loader
// (Milestone 2c) -- one source of truth for both sides. Plain POD structs,
// fixed-size headers plus flat variable-length arrays with counts; no STL
// in the layout so both an fwrite-based writer and an fread-based loader
// stay trivial. Default struct alignment (no explicit packing) is used
// deliberately -- every field is already 1 or 4 bytes, so there is no
// padding ambiguity between compilers reading/writing this format.

namespace crossbot_nav
{

constexpr char kNavMagic[4] = {'C', 'B', 'N', 'V'};
// v5: doors are no longer a proximity-anchored special edge bridging a gap
// (the old EDGE_DOOR/EDGE_FLAG_NEEDS_BUTTON/EDGE_FLAG_NEEDS_TRIGGER, and
// tools/navgen/src/edge_tagger.cpp's TagDoor) -- a door's own solid volume
// is now kept as its own real navigable leaf/area (tools/navgen/src/
// door_extractor.cpp's ExtractDoorLeaves), flagged NAV_AREA_DOOR below, with
// portal_builder's ordinary coplanar-face matching finding the real portals
// into and out of it on both sides. The activation metadata (button/
// trigger-gated or plain touch, and how long to wait once triggered) moved
// from the portal to the area record accordingly -- see
// NavAreaRecord::doorActivation/doorActivatorPos/doorWaitTime. Prompted by a
// live observation that the old approach silently mishandled func_door_
// rotating's origin-brush offset in one code path (blocking_entities.cpp's
// ExtractBlockingVolumes) but not another (edge_tagger.cpp's TagDoor), since
// the same origin-correction logic had to be duplicated in both places for
// two different representations of the same door.
//
// v4: NavAreaRecord gained planeOffset/planeCount into a new flat
// AreaPlanes[] table -- each area's real convex-polytope boundary planes
// (navgen::Area.polytope.faces, already computed and provably convex by
// MergeAreas, just never serialized before). Lets the runtime test true
// point-in-area containment instead of only the AABB+nearest-centroid
// approximation, which was confirmed live to occasionally resolve a
// bot's real position to the wrong (nearby, overlapping-AABB) area --
// silently poisoning Repath's start/goal area choice and any "did I
// actually arrive in the target area" check with a plausible-looking but
// wrong answer, not a crash, so it went unnoticed until traced through
// several layers of live testing on crossfire.bsp.
constexpr uint32_t kNavVersion = 5;

// AreaRecord.areaFlags bits.
enum AreaFlags : uint32_t
{
	NAV_AREA_NONE             = 0,
	NAV_AREA_CROUCH           = 1u << 0, // hull-3-only clearance somewhere in this area
	NAV_AREA_LADDER_TOP       = 1u << 1,
	NAV_AREA_WATER            = 1u << 2, // Milestone 2g
	NAV_AREA_VANTAGE          = 1u << 3, // Milestone 2e: sniper/vantage candidate
	NAV_AREA_CAMP             = 1u << 4, // Milestone 2e: ambush/camp candidate
	NAV_AREA_CHOKEPOINT_COVER = 1u << 5, // Milestone 2e: sentry/turret placement candidate
	NAV_AREA_GRAVITY_SCALE    = 1u << 6, // Milestone 2h: overlaps a trigger_gravity volume
	// This area IS a func_door/func_door_rotating's own solid-until-opened
	// volume (door_extractor.cpp's ExtractDoorLeaves) -- see
	// NavAreaRecord::doorActivation/doorActivatorPos/doorWaitTime for how to
	// actually get it open.
	NAV_AREA_DOOR             = 1u << 7,
};

// PortalRecord/SpecialEdgeRecord type tag.
enum EdgeType : uint8_t
{
	EDGE_WALK   = 0,
	EDGE_JUMP   = 1,
	EDGE_LADDER = 2,
	// EDGE_DOOR = 3 is retired as of v5 (see kNavVersion's own comment) --
	// left unused rather than renumbering EDGE_DROP/EDGE_LAUNCH/EDGE_ELEVATOR
	// below it, in case any tooling still has the old numbering baked in.
	EDGE_DROP   = 4,
	EDGE_LAUNCH = 5, // Milestone 2h: trigger_push
	EDGE_ELEVATOR = 6, // button-gated vertical func_door lift, see tools/navgen/src/elevator_tagger.h
};

// PortalRecord.edgeFlags bits.
enum EdgeFlags : uint8_t
{
	EDGE_FLAG_NONE          = 0,
	EDGE_FLAG_ONE_WAY       = 1u << 0, // e.g. a drop-off you can't climb back up
	EDGE_FLAG_NEEDS_CROUCH  = 1u << 1,
	// Bits 2/3 (EDGE_FLAG_NEEDS_BUTTON/EDGE_FLAG_NEEDS_TRIGGER) are retired
	// as of v5 -- doors carry their own activation info directly on the area
	// record now (NavAreaRecord::doorActivation), not on the portal that
	// leads into it.
};

// NavAreaRecord.doorActivation values -- meaningful only when areaFlags &
// NAV_AREA_DOOR is set.
enum DoorActivation : uint8_t
{
	DOOR_ACTIVATION_TOUCH   = 0, // plain door, no targetname -- touching it (walking up) opens it
	DOOR_ACTIVATION_BUTTON  = 1, // targetname'd door_extractor.cpp resolved a func_button for
	DOOR_ACTIVATION_TRIGGER = 2, // targetname'd door_extractor.cpp resolved a trigger_multiple for
};

struct NavHeader
{
	char magic[4];
	uint32_t version;
	char mapName[64];
	uint32_t bspChecksum;         // recomputed and checked by the runtime loader; mismatch = stale, don't trust
	uint32_t flags;
	uint32_t areaCount;
	uint32_t portalCount;
	uint32_t areaPortalIndexCount;
	uint32_t specialEdgeCount;
	uint32_t stringTableSize;     // bytes
	uint32_t areaPlaneCount;
};

struct NavAreaRecord
{
	uint32_t id;
	float mins[3];
	float maxs[3];
	float centroid[3];
	uint32_t areaFlags;
	float gravityScale;   // 1.0 = normal gravity; meaningful only if NAV_AREA_GRAVITY_SCALE is set
	// Milestone 2e tactical scores -- see tools/navgen/src/tactical_tagger.h
	// for exactly what each measures. Heuristic, tunable, not ground truth.
	// Zeroed (and the NAV_AREA_VANTAGE/CAMP/CHOKEPOINT_COVER flags left
	// unset) when navgen wasn't given tactical data to write, e.g. a
	// synthetic test fixture with too little geometry for the scores to
	// mean anything.
	float exposureScore;
	float concealmentScore;
	float dangerScore;
	uint16_t portalOffset; // slice into AreaPortalIndex[]
	uint16_t portalCount;
	uint16_t planeOffset; // slice into AreaPlanes[] -- this area's convex-polytope boundary
	uint16_t planeCount;
	// Meaningful only when areaFlags & NAV_AREA_DOOR is set (door_extractor.
	// cpp's ExtractDoorLeaves). doorActivation is a DoorActivation value:
	// DOOR_ACTIVATION_TOUCH needs no activator (doorActivatorPos is zeroed),
	// _BUTTON/_TRIGGER give doorActivatorPos as where to walk to press/touch
	// it first. doorWaitTime is how long to hold once triggered (or, for a
	// plain touch door, once contact is made) before the door is assumed
	// open -- not a per-door reverse-engineered value, just a fixed margin
	// wide enough for a typical door's open animation (see door_extractor.
	// cpp/main.cpp).
	uint8_t doorActivation;
	float doorActivatorPos[3];
	float doorWaitTime;
};

// A half-space plane bounding one area's convex polytope: the area's
// interior satisfies dot(normal, point) - dist >= 0 for every one of its
// planes (matches navgen::Plane's convention exactly, see polytope.h).
struct NavAreaPlane
{
	float normal[3];
	float dist;
};

struct NavPortalRecord
{
	uint32_t areaA;
	uint32_t areaB;
	float pointA[3]; // shared-boundary segment endpoints -- input to the funnel algorithm
	float pointB[3];
	uint8_t edgeType;
	uint8_t edgeFlags;
	float traverseCost; // A* edge-cost multiplier
	// Meaningful for edgeType == EDGE_ELEVATOR only (as of v5 -- doors moved
	// their own equivalent fields to NavAreaRecord, see its own comment):
	// where to walk to call/press the button, and how long the ride takes.
	// Zeroed for every other edge type -- a few extra bytes on every portal
	// record rather than a separate sparse table, matching this format's
	// existing "flat POD, no per-type variants" philosophy.
	float buttonPos[3];
	float travelTime;
};

// Richer typed edges that aren't a natural convex-area adjacency (a ladder
// overlays otherwise-solid geometry; launch pads, Milestone 2h, connect
// areas with no shared face at all). As of v5, ladders are the only
// producer -- doors moved to being their own real leaf/area, see
// NAV_AREA_DOOR's comment.
struct NavSpecialEdgeRecord
{
	uint32_t areaFrom;
	uint32_t areaTo;
	uint8_t type;
	float anchorFrom[3];
	float anchorTo[3];
	int32_t stringTableOffset; // currently always -1 (unused) -- reserved for a future producer that needs it
};

} // namespace crossbot_nav
