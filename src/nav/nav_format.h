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
constexpr uint32_t kNavVersion = 1;

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
};

// PortalRecord/SpecialEdgeRecord type tag.
enum EdgeType : uint8_t
{
	EDGE_WALK   = 0,
	EDGE_JUMP   = 1,
	EDGE_LADDER = 2,
	EDGE_DOOR   = 3,
	EDGE_DROP   = 4,
	EDGE_LAUNCH = 5, // Milestone 2h: trigger_push
};

// PortalRecord.edgeFlags bits.
enum EdgeFlags : uint8_t
{
	EDGE_FLAG_NONE          = 0,
	EDGE_FLAG_ONE_WAY       = 1u << 0, // e.g. a drop-off you can't climb back up
	EDGE_FLAG_NEEDS_CROUCH  = 1u << 1,
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
};

struct NavAreaRecord
{
	uint32_t id;
	float mins[3];
	float maxs[3];
	float centroid[3];
	uint32_t areaFlags;
	float gravityScale;   // 1.0 = normal gravity; meaningful only if NAV_AREA_GRAVITY_SCALE is set
	uint16_t portalOffset; // slice into AreaPortalIndex[]
	uint16_t portalCount;
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
};

// Richer typed edges that aren't a natural convex-area adjacency (ladders/
// doors often overlay otherwise-solid geometry; launch pads connect areas
// with no shared face at all).
struct NavSpecialEdgeRecord
{
	uint32_t areaFrom;
	uint32_t areaTo;
	uint8_t type;
	float anchorFrom[3];
	float anchorTo[3];
	int32_t stringTableOffset; // -1 if unused, else a byte offset into StringTable (e.g. a door's targetname)
};

} // namespace crossbot_nav
