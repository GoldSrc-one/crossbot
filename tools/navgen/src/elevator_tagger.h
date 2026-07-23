#pragma once

#include <vector>

#include "area_merger.h"
#include "bsp_file.h"
#include "entity_parser.h"

namespace navgen
{

// Detects button-gated vertical func_door elevators -- the classic HLDM
// "lift" pattern: a func_button whose target matches exactly one
// func_door's targetname, with that door moving straight up or down
// between two resting positions (angle -1/-2, the Quake/GoldSrc "up"/
// "down" movedir convention) -- and emits a bidirectional EDGE_ELEVATOR
// portal connecting the bottom and top landing areas, carrying the
// button's position and the door's travel time for the runtime's
// board/wait/ride steering (src/bot/bot_move_nav.cpp).
//
// Deliberately scoped to exactly this one pattern rather than every
// possible func_door/func_plat use: crossfire.bsp's lift1-4 are exactly
// this, and a target shared by more than one door (multi-door trigger
// sequences, like the airstrike bunker doors) or a non-vertical mover is
// skipped rather than guessed at -- see the plan's "Moving platforms" note
// for func_plat/func_tracktrain, which this does not attempt.
std::vector<MergedPortal> TagElevators(const BspFile &bsp, const std::vector<Entity> &entities,
	const std::vector<Area> &areas);

} // namespace navgen
