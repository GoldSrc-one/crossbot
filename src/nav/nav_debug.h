#pragma once

#include <extdll.h>

// Milestone 2 dev/testing visualization -- draws TE_BEAMPOINTS temp entities so spectator can watch bot's planned path and see which nav area (and portal-adjacent neighbors) they're currently aiming at, same live-inspection technique as classic HL bots and CS's ZBot/CNavArea debug draw. Purely visual (broadcast temp entities, no gameplay side effect); every call is cheap no-op once toggle is off, Nav::IsLoaded() false, or beam sprite failed to precache.
namespace NavDebug
{

// Precaches beam sprite. Must run from BotManager::ServerActivate -- still within engine's map-load precache window (confirmed: same hook Nav::LoadForMap uses for map-load-time setup), standard point every classic HL bot precaches debug/effect sprites from.
void Precache();

// Called once per server frame from BotManager::StartFrame; throttles own redraw rate internally (see kDrawInterval in nav_debug.cpp) so cheap to call unconditionally every tick.
void Draw();

// crossbot_nav_debug_path console toggle: draws each bot's current waypoint polyline (origin -> next waypoint -> ... -> goal).
void SetPathBeamsEnabled(bool enabled);
bool PathBeamsEnabled();

// crossbot_nav_debug_area console toggle: for each connected human, traces their aim and draws the nav area it lands in plus that area's portal-adjacent neighbors.
void SetAreaBeamsEnabled(bool enabled);
bool AreaBeamsEnabled();

} // namespace NavDebug
