#pragma once

#include <string>
#include <vector>

#include "area_merger.h"
#include "bsp_file.h"
#include "entity_parser.h"
#include "polytope.h"

namespace navgen
{

// One func_door/func_door_rotating entity's own solid volume, kept as its
// own navigable leaf (see main.cpp) instead of being subtracted away like a
// permanently-solid blocker (blocking_entities.cpp) -- a door eventually
// becomes passable, so its footprint deserves a real area (flagged
// NAV_AREA_DOOR at the area level, see nav_format.h) rather than a gap
// bridged by a proximity-anchored synthetic edge.
struct DoorLeaf
{
	Polytope polytope; // world-space, origin-brush corrected
	std::string targetname; // debug/labeling only -- nothing re-resolves it live
	// True if this door has a targetname (so CBaseDoor::SetTouch(NULL) means
	// it can't be touch-opened) and a func_button/trigger_multiple was
	// resolved that fires it. False (for either) means a plain touch door --
	// still its own leaf, still needs the same "wait for it to swing open"
	// handling, just no activator to walk to first.
	bool needsButton = false;
	bool needsTrigger = false;
	Vec3 activatorPos; // meaningful only when needsButton or needsTrigger
};

// func_door/func_door_rotating -> one DoorLeaf per entity with a resolvable
// "model" keyvalue (skips anything else -- a stray point entity misusing
// one of these classnames shouldn't abort the run, matching
// blocking_entities.cpp's own convention). Resolves each door's activation
// the same way this module's ResolveButtonForDoor/ResolveTriggerForDoor
// always have: a func_button that targets it directly or through one level
// of trigger_relay/multi_manager indirection, else a trigger_multiple that
// fires it directly by touch. Origin-brush corrected (func_door_rotating
// almost always has one; a plain func_door's is a no-op zero offset).
std::vector<DoorLeaf> ExtractDoorLeaves(const BspFile &bsp, const std::vector<Entity> &entities, double mapRadius);

// Per-area equivalent of a DoorLeaf's activation info -- isDoor is the
// area-level counterpart of NAV_AREA_DOOR (nav_format.h).
struct DoorAreaInfo
{
	bool isDoor = false;
	bool needsButton = false;
	bool needsTrigger = false;
	Vec3 activatorPos;
};

// True (with activation info attached) for any final area whose
// memberLeafIds includes a door leaf id, per infoByLeafId (indexed by leaf
// id, built by main.cpp alongside isCrouchLeaf/isDoorLeaf from this same
// ExtractDoorLeaves pass). Same reasoning as edge_tagger.h's
// ComputeCrouchAreaFlags -- memberLeafIds survives AbsorbTinyAreas folding a
// tiny area's connectivity into a larger neighbor (area_merger.cpp now
// carries every absorbed leaf's id into the survivor), so this still finds
// the right leaf even after that. If more than one door leaf somehow ends
// up in the same final area (no real test case has), the first one found
// wins -- a documented simplification, not silently wrong.
std::vector<DoorAreaInfo> ComputeDoorAreaInfo(const std::vector<Area> &areas,
	const std::vector<DoorAreaInfo> &infoByLeafId);

} // namespace navgen
