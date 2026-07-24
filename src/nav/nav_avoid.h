#pragma once

#include <extdll.h>

struct CBot;

// Per-bot local avoidance (steering-layer add-on). Runs after Bot_MoveNav picks current waypoint (after A*/funnel corridor), only perturbs *how* bot heads toward waypoint, never *which* area-level path. See plan's "Multi-agent local avoidance" architecture note.
namespace NavAvoid
{

// Deflects `desiredDir` (the bot's current horizontal steering direction --
// only its angle matters, magnitude is not meaningful to the caller) away
// from other nearby bots. Returns `desiredDir` unchanged when avoidance is
// disabled or no other bot is close enough to matter.
Vector Deflect(CBot &bot, const Vector &desiredDir);

void SetEnabled(bool enabled);
bool IsEnabled();

} // namespace NavAvoid
