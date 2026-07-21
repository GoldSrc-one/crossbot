#pragma once

#include <extdll.h>

struct CBot;

// Milestone 2d: per-bot local avoidance. Strictly a steering-layer add-on --
// it runs after Bot_MoveNav has already picked the current waypoint (i.e.
// after the A*/funnel corridor), and only ever perturbs *how* a bot heads
// toward that waypoint, never *which* area-level path it's following. See
// the plan's "Multi-agent local avoidance" architecture note.
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
