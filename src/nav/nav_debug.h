#pragma once

#include <extdll.h>

// Milestone 2 dev/testing visualization -- draws TE_BEAMPOINTS temp entities
// so a spectating human can watch a bot's planned path and see which nav
// area (and its portal-adjacent neighbors) they're currently aiming at, the
// same live-inspection technique classic HL bots and CS's ZBot/CNavArea
// debug draw both use. Purely visual (broadcast temp entities, no gameplay
// side effect); every call here is a cheap no-op once its toggle is off,
// Nav::IsLoaded() is false, or the beam sprite failed to precache.
namespace NavDebug
{

// Precaches the beam sprite. Must run from BotManager::ServerActivate --
// that's still within the engine's map-load precache window (confirmed by
// this being the same hook Nav::LoadForMap already uses for its own
// map-load-time setup), and it's the standard point every classic HL bot
// precaches its own debug/effect sprites from.
void Precache();

// Called once per server frame from BotManager::StartFrame; throttles its
// own redraw rate internally (see kDrawInterval in nav_debug.cpp) so it's
// cheap to call unconditionally every tick.
void Draw();

// crossbot_nav_debug_path console toggle: draws each bot's current
// waypoint polyline (origin -> next waypoint -> ... -> goal).
void SetPathBeamsEnabled(bool enabled);
bool PathBeamsEnabled();

// crossbot_nav_debug_area console toggle: for every connected human player,
// traces their aim and draws the nav area it lands in plus that area's
// portal-adjacent neighbors.
void SetAreaBeamsEnabled(bool enabled);
bool AreaBeamsEnabled();

} // namespace NavDebug
