#pragma once

#include <vector>

#include "mathlib.h" // vec_t -- vector.h's Vector needs this typedef and doesn't include it itself
#include "vector.h"

// Engine-independent Simple Stupid Funnel Algorithm (SSFA) over corridor of portal gates -- shared verbatim between runtime plugin (path_funnel.cpp) and tools/navgen (--dump-svg path overlay), like astar_graph.h/.cpp. Offline debug view draws same string-pulled line live bot walks because it runs same code. Pure geometry, no engine traces -- runtime obstacle handling in bot_move_nav.cpp (kObstacleTraceDistance + FindClearDirection) catches what baked nav can't know, doesn't re-derive path.
namespace NavFunnel
{

// One portal crossing's walkable extent, oriented relative to travel direction: `left` = leftmost passable point as seen by agent walking through, `right` = rightmost. Build with MakeGate -- navgen's baked portal points are just "two most separated points of shared-face overlap" (portal_builder.cpp PolygonDiameter) with no orientation guarantee; MakeGate fixes that (and shrinks gate by agent half-width so pulled string can't hug edge).
struct PortalGate
{
	Vector left, right;
};

// Half of player hull's 32u width: funnel pulls path tight against gate endpoints; endpoint on doorframe/walkway rim is where hull *center* can't be -- confirmed live on test23_precise's thin walkways over pit, where path hugging inner rim leaves zero margin for steering error.
constexpr float kGateShrinkDistance = 16.f;

// Builds oriented, shrunk gate from portal's two stored points. travelDir2D = horizontal direction of travel through crossing (typically nextAreaCentroid - currentAreaCentroid; z ignored) disambiguates which stored point is "left". Each endpoint pulled kGateShrinkDistance toward other, clamped so gate narrower than 2*kGateShrinkDistance still keeps degenerate-but-ordered pair at its midpoint.
PortalGate MakeGate(const Vector &pointA, const Vector &pointB, const Vector &travelDir2D);

// String-pulls start -> goal through `gates` (in crossing order) and returns taut polyline: start, each funnel corner, goal. Corners land on (shrunk) gate endpoints, keeping endpoint's z -- funnel is 2D (x/y) algorithm, same planar simplification as rest of nav pipeline (see svg_debug.h top-down view). With no gates: {start, goal}.
std::vector<Vector> StringPull(const Vector &start, const Vector &goal, const std::vector<PortalGate> &gates);

} // namespace NavFunnel
