#pragma once

#include <cstdint>

namespace crossbot_nav
{

constexpr char kNavMagic[4] = {'C', 'B', 'N', 'V'};

constexpr uint32_t kNavVersion = 17;

constexpr uint8_t kNoAreaPlane = 0xFF;

constexpr float kNavDefaultDesignSpeed = 320.f;

enum AreaFlags : uint32_t
{
	NAV_AREA_NONE             = 0,
	NAV_AREA_CROUCH           = 1u << 0,
	NAV_AREA_LADDER_TOP       = 1u << 1,
	NAV_AREA_WATER            = 1u << 2,

	NAV_AREA_DOOR             = 1u << 7,
	NAV_AREA_BREAKABLE        = 1u << 8,
	NAV_AREA_ELEVATOR         = 1u << 9,
};

enum ConnType : uint8_t
{
	CONN_WALK     = 0,
	CONN_TELEPORT = 1,
	CONN_AIRMOVE  = 2,
	CONN_LADDER   = 3,
	CONN_ELEVATOR = 4,
};

enum ConnFlags : uint8_t
{
	CONN_FLAG_NONE         = 0,
	CONN_FLAG_ONE_WAY      = 1u << 0,
	CONN_FLAG_NEEDS_CROUCH = 1u << 1,
	CONN_FLAG_JUMP         = 1u << 2,
};

enum ActivationKind : uint8_t
{
	ACT_NONE    = 0,
	ACT_BUTTON  = 1,
	ACT_TRIGGER = 2,
	ACT_ATTACK  = 3,
};

enum DetourFlags : uint8_t
{
	DETOUR_FLAG_NONE    = 0,
	DETOUR_FLAG_BLOCKED = 1u << 0,
};

enum DetourPointFlags : uint8_t
{
	DETOUR_PT_NONE   = 0,
	DETOUR_PT_CROUCH = 1u << 0,
};

struct NavHeader
{
	char magic[4];
	uint32_t version;
	char mapName[64];
	uint32_t bspChecksum;
	uint32_t flags;
	uint32_t areaCount;
	uint32_t connCount;
	uint32_t areaConnIndexCount;
	uint32_t areaPlaneCount;
	uint32_t detourCount;
	uint32_t detourPointCount;
	float designMaxSpeed;
};

static_assert(sizeof(NavHeader) == 108, "NavHeader layout changed without a version bump");

struct NavAreaRecord
{
	uint32_t id;
	float mins[3];
	float maxs[3];
	float centroid[3];
	uint32_t areaFlags;
	uint16_t connOffset;
	uint16_t connCount;
	uint16_t planeOffset;
	uint16_t planeCount;

	uint8_t activation;
	uint8_t pad0[3];
	float openSeconds;
	uint32_t detourOffset;
	uint32_t detourCount;
};

struct NavActivationDetourRecord
{
	uint32_t fromAreaId;
	uint32_t pointOffset;
	uint32_t pointCount;
	uint32_t actIndex;
	float waitSeconds;
	float detourSeconds;
	float actCenter[3];
	uint8_t actKind;
	uint8_t flags;
	uint8_t pad[2];
};

struct NavDetourPointRecord
{
	float pos[3];
	uint32_t areaId;
	float clearance;
	uint8_t flags;
	uint8_t pad[3];
};

static_assert(sizeof(NavAreaRecord) == 68, "NavAreaRecord layout changed without a version bump");
static_assert(sizeof(NavActivationDetourRecord) == 40, "NavActivationDetourRecord layout changed");
static_assert(sizeof(NavDetourPointRecord) == 24, "NavDetourPointRecord layout changed");

struct NavAreaPlane
{
	float normal[3];
	float dist;
};

struct NavConnectionRecord
{
	uint32_t areaA;
	uint32_t areaB;
	uint8_t connType;
	uint8_t connFlags;
	uint8_t gateValid;
	uint8_t pad;
	float costSeconds;
	float travelSeconds;

	float gateA[2][3];
	float gateB[2][3];
	float gateAYaw[2];
	float gateASpeed[2];

	float exitSpeed;
	float exitAngle;
	float jumpBackDist;
	float fallDamage;
	float flightTime;
	float launchPos[3];
	float landingPos[3];

	float rideSeconds;
	float rideSecondsMax;
	float rideWaitSeconds;

	// pev->gravity multiplier this air move was sampled at. 1.0 for every other connection type.
	float gravity;
};

static_assert(sizeof(NavConnectionRecord) == 144, "NavConnectionRecord layout changed without a version bump");

enum GateValidFlags : uint8_t
{
	GATE_VALID_NONE = 0,
	GATE_VALID_A    = 1u << 0,
	GATE_VALID_B    = 1u << 1,
	GATE_CLAMPED_A  = 1u << 2,
	GATE_CLAMPED_B  = 1u << 3,
};

}

