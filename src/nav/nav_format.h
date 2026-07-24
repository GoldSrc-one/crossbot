#pragma once

#include <cstdint>

// On-disk layout for baked <mapname>.nav file, shared verbatim between tools/navgen (writer) and src/nav/ loader. Plain POD, fixed headers + flat arrays; no STL. Default alignment used deliberately (fields are 1 or 4 bytes, no padding ambiguity between compilers).

namespace crossbot_nav
{

constexpr char kNavMagic[4] = {'C', 'B', 'N', 'V'};
// v5: doors are now real navigable areas (NAV_AREA_DOOR) with coplanar-face portals, not proximity-anchored special edges (old EDGE_DOOR/EDGE_FLAG_NEEDS_BUTTON/EDGE_FLAG_NEEDS_TRIGGER retired). Activation metadata moved from portal to area record (doorActivation/doorActivatorPos/doorWaitTime). Fixes func_door_rotating origin-brush offset handled inconsistently in two code paths.
// v4: NavAreaRecord gained planeOffset/planeCount into flat AreaPlanes[] table -- each area's real convex-polytope boundary planes (already computed by MergeAreas, never serialized). Enables true point-in-area containment vs AABB+nearest-centroid approximation, which occasionally resolved bot to wrong overlapping-AABB area (silent wrong answer, not crash, went unnoticed until traced on crossfire.bsp).
constexpr uint32_t kNavVersion = 5;

// AreaRecord.areaFlags bits.
enum AreaFlags : uint32_t
{
	NAV_AREA_NONE             = 0,
	NAV_AREA_CROUCH           = 1u << 0, // hull-3-only clearance somewhere in this area
	NAV_AREA_LADDER_TOP       = 1u << 1,
	NAV_AREA_WATER            = 1u << 2,
	NAV_AREA_VANTAGE          = 1u << 3, // sniper/vantage candidate
	NAV_AREA_CAMP             = 1u << 4, // ambush/camp candidate
	NAV_AREA_CHOKEPOINT_COVER = 1u << 5, // sentry/turret placement candidate
	NAV_AREA_GRAVITY_SCALE    = 1u << 6, // overlaps a trigger_gravity volume
	// This area IS a func_door/func_door_rotating's own solid-until-opened volume (door_extractor.cpp's ExtractDoorLeaves) -- see NavAreaRecord::doorActivation/doorActivatorPos/doorWaitTime for how to open it.
	NAV_AREA_DOOR             = 1u << 7,
};

// PortalRecord/SpecialEdgeRecord type tag.
enum EdgeType : uint8_t
{
	EDGE_WALK   = 0,
	EDGE_JUMP   = 1,
	EDGE_LADDER = 2,
	// EDGE_DOOR = 3 retired in v5 -- left unused rather than renumbering EDGE_DROP/EDGE_LAUNCH/EDGE_ELEVATOR below, in case tooling has old numbering baked in.
	EDGE_DROP   = 4,
	EDGE_LAUNCH = 5, // trigger_push
	EDGE_ELEVATOR = 6, // button-gated vertical func_door lift (see tools/navgen/src/elevator_tagger.h)
};

// PortalRecord.edgeFlags bits.
enum EdgeFlags : uint8_t
{
	EDGE_FLAG_NONE          = 0,
	EDGE_FLAG_ONE_WAY       = 1u << 0, // e.g. drop-off you can't climb back up
	EDGE_FLAG_NEEDS_CROUCH  = 1u << 1,
	// Bits 2/3 (EDGE_FLAG_NEEDS_BUTTON/EDGE_FLAG_NEEDS_TRIGGER) retired in v5 -- doors carry activation info on area record (NavAreaRecord::doorActivation), not portal.
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
	// Milestone 2e tactical scores (tools/navgen/src/tactical_tagger.h). Heuristic, tunable, not ground truth. Zeroed (flags unset) when navgen lacks tactical data (e.g., synthetic test fixture).
		float exposureScore;
		float concealmentScore;
		float dangerScore;
	uint16_t portalOffset; // slice into AreaPortalIndex[]
	uint16_t portalCount;
	uint16_t planeOffset; // slice into AreaPlanes[] -- this area's convex-polytope boundary
	uint16_t planeCount;
	// Meaningful only when areaFlags & NAV_AREA_DOOR is set (door_extractor.cpp's ExtractDoorLeaves). doorActivation is a DoorActivation value: DOOR_ACTIVATION_TOUCH needs no activator (doorActivatorPos zeroed); _BUTTON/_TRIGGER give doorActivatorPos as where to walk to press/touch first. doorWaitTime is how long to hold once triggered (or, for touch door, once contact made) before door assumed open -- fixed margin wide enough for typical door open animation (see door_extractor.cpp/main.cpp).
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
	// Meaningful for EDGE_ELEVATOR only (v5: doors moved equivalent fields to NavAreaRecord). Where to walk to call/press button, and ride duration. Zeroed for other edge types -- few extra bytes per portal vs sparse table, matching flat-POD-no-variants philosophy.
		float buttonPos[3];
		float travelTime;
	};

	// Richer typed edges that aren't natural convex-area adjacency (ladder overlays solid geometry; launch pads connect areas with no shared face). v5: only ladders produce these -- doors became own real leaf/area (see NAV_AREA_DOOR).
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
