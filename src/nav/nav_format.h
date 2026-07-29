#pragma once

#include <cstdint>

namespace crossbot_nav
{

constexpr char kNavMagic[4] = {'C', 'B', 'N', 'V'};

constexpr uint32_t kNavVersion = 9;

constexpr uint8_t kNoAreaPlane = 0xFF;

enum AreaFlags : uint32_t
{
	NAV_AREA_NONE             = 0,
	NAV_AREA_CROUCH           = 1u << 0,
	NAV_AREA_LADDER_TOP       = 1u << 1,
	NAV_AREA_WATER            = 1u << 2,

	NAV_AREA_DOOR             = 1u << 7,
};

enum ConnType : uint8_t
{
	CONN_WALK     = 0,
	CONN_TELEPORT = 1,
	CONN_AIRMOVE  = 2,
};

enum ConnFlags : uint8_t
{
	CONN_FLAG_NONE         = 0,
	CONN_FLAG_ONE_WAY      = 1u << 0,
	CONN_FLAG_NEEDS_CROUCH = 1u << 1,
	// The air move leaves the ground under jump impulse rather than by walking off. Not derivable
	// from jumpBackDist: a reverse jump inferred from a drop carries no run-up, and a sampled jump
	// may have drawn one close to zero.
	CONN_FLAG_JUMP         = 1u << 2,

};

enum DoorActivation : uint8_t
{
	DOOR_ACTIVATION_TOUCH   = 0,
	DOOR_ACTIVATION_BUTTON  = 1,
	DOOR_ACTIVATION_TRIGGER = 2,
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
};

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

	uint8_t doorActivation;
	float doorActivatorPos[3];
	float doorWaitTime;
};

struct NavAreaPlane
{
	float normal[3];
	float dist;
};

// gateA and gateB are the funnel gates on each side of the crossing, baked by navgen with the
// player radius already applied: inset at both ends and pushed that far into the owning area. A
// zero-length gate means the opening was too narrow to inset and the midpoint is the only way
// through. gateValid says which sides carry one; a teleport carries neither.
struct NavConnectionRecord
{
	uint32_t areaA;
	uint32_t areaB;
	uint8_t connType;
	uint8_t connFlags;
	uint8_t gateValid;
	uint8_t pad;
	float traverseCost;

	float gateA[2][3];
	float gateB[2][3];
	// Launch parameters at gateA's two ends, to be interpolated across it: the samples that set
	// each end flew with their own yaw and speed, and one pair does not serve the whole span.
	float gateAYaw[2];
	float gateASpeed[2];

	float exitSpeed;
	float exitAngle;
	float jumpBackDist;
	float fallDamage;
	float flightTime;
	float launchPos[3];
	float landingPos[3];
};

enum GateValidFlags : uint8_t
{
	GATE_VALID_NONE = 0,
	GATE_VALID_A    = 1u << 0,
	GATE_VALID_B    = 1u << 1,
	GATE_CLAMPED_A  = 1u << 2,
	GATE_CLAMPED_B  = 1u << 3,
};

}

