#pragma once

#include <vector>

#include "mathlib.h" // vec_t -- vector.h's Vector needs this typedef and doesn't include it itself
#include "vector.h"

// Engine-independent Simple Stupid Funnel Algorithm (SSFA) over a corridor
// of portal "gates" -- shared verbatim between the runtime plugin
// (path_funnel.cpp) and tools/navgen (main.cpp's --dump-svg path overlay),
// exactly like astar_graph.h/.cpp: the offline debug view draws literally
// the same string-pulled line a live bot walks, because it runs this same
// code. Pure geometry, no engine traces at all -- runtime obstacle handling
// stays where it belongs, in bot_move_nav.cpp's live feeler traces
// (kObstacleTraceDistance + FindClearDirection), which exist to catch
// what baked nav data can't know about, not to re-derive the path.
namespace NavFunnel
{

// One portal crossing's walkable extent, oriented relative to the direction
// of travel: `left` is the crossing's leftmost passable point as seen by an
// agent walking through it, `right` the rightmost. Build these with
// MakeGate below rather than by hand -- navgen's baked portal points are
// just "the two most separated points of the shared-face overlap"
// (portal_builder.cpp's PolygonDiameter) with no orientation guarantee,
// and MakeGate is what fixes that up (and shrinks the gate by the agent's
// half-width so the pulled string can't hug an edge).
struct PortalGate
{
	Vector left, right;
};

// Half of the player hull's 32-unit width: the funnel pulls the path tight
// against gate endpoints, and an endpoint sitting exactly on a doorframe or
// on a walkway's rim is somewhere the hull *center* can't actually be --
// confirmed live on test23_precise's thin walkways ringing a pit, where a
// path hugging the inner rim leaves zero margin for steering error.
constexpr float kGateShrinkDistance = 16.f;

// Builds an oriented, shrunk gate from a portal's two stored points.
// travelDir2D is the horizontal direction of travel through the crossing
// (typically nextAreaCentroid - currentAreaCentroid; z is ignored) and is
// what disambiguates which stored point is "left". Each endpoint is pulled
// kGateShrinkDistance toward the other, clamped so a gate narrower than
// twice that still keeps a degenerate-but-ordered pair at its midpoint.
PortalGate MakeGate(const Vector &pointA, const Vector &pointB, const Vector &travelDir2D);

// String-pulls start -> goal through `gates` (in crossing order) and
// returns the taut polyline: start, each funnel corner, goal. Corners land
// on (shrunk) gate endpoints, and keep that endpoint's z -- the funnel
// itself is a 2D (x/y) algorithm, same planar simplification the rest of
// the nav pipeline already makes (see svg_debug.h's top-down view).
// With no gates at all this is just {start, goal}.
std::vector<Vector> StringPull(const Vector &start, const Vector &goal, const std::vector<PortalGate> &gates);

} // namespace NavFunnel
