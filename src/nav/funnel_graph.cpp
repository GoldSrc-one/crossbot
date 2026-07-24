#include "funnel_graph.h"

#include <cmath>

namespace NavFunnel
{

namespace
{

// Twice signed area of triangle (a,b,c) in XY plane -- positive when c lies to left of a->b (counter-clockwise). Standard SSFA predicate (same as Recast's dtTriArea2D).
float TriArea2(const Vector &a, const Vector &b, const Vector &c)
{
	float abx = b.x - a.x, aby = b.y - a.y;
	float acx = c.x - a.x, acy = c.y - a.y;
	return acx * aby - abx * acy;
}

bool SamePoint2D(const Vector &a, const Vector &b)
{
	// Generous epsilon for portal-endpoint coordinates tens/hundreds of units apart -- anything this close is same funnel vertex for taut-path purposes.
	constexpr float kEps = 0.001f;
	return std::fabs(a.x - b.x) < kEps && std::fabs(a.y - b.y) < kEps;
}

} // namespace

PortalGate MakeGate(const Vector &pointA, const Vector &pointB, const Vector &travelDir2D)
{
	// Which stored point is "left" as seen by an agent walking along
	// travelDir2D: A is left of B when it sits on the left of the travel
	// direction relative to B, i.e. cross2(travelDir, A - B) > 0.
	float side = travelDir2D.x * (pointA.y - pointB.y) - travelDir2D.y * (pointA.x - pointB.x);
	PortalGate gate;
	gate.left = (side > 0.f) ? pointA : pointB;
	gate.right = (side > 0.f) ? pointB : pointA;

	// Shrink each endpoint toward the other by the agent's half-width (see
	// kGateShrinkDistance's comment), clamped to the gate's own half-length
	// so a narrow gate degenerates to its midpoint instead of inverting.
	Vector span = gate.right - gate.left;
	float len2D = std::sqrt(span.x * span.x + span.y * span.y);
	if (len2D > 0.001f)
	{
		float shrink = kGateShrinkDistance;
		if (shrink > len2D * 0.5f)
			shrink = len2D * 0.5f;
		Vector step = span * (shrink / len2D);
		gate.left = gate.left + step;
		gate.right = gate.right - step;
	}
	return gate;
}

std::vector<Vector> StringPull(const Vector &start, const Vector &goal, const std::vector<PortalGate> &gates)
{
	std::vector<Vector> path;
	path.push_back(start);

	if (gates.empty())
	{
		path.push_back(goal);
		return path;
	}

	// Standard SSFA (Mononen's "Simple Stupid Funnel Algorithm", the same
	// shape Recast's findStraightPath uses): walk the gates keeping a funnel
	// of (apex, left, right); when a new gate edge would cross over the
	// opposite funnel edge, the opposite edge's endpoint becomes a path
	// corner and the scan restarts from it. The goal acts as one final
	// degenerate gate (left == right == goal).
	size_t n = gates.size();
	Vector apex = start, funnelLeft = gates[0].left, funnelRight = gates[0].right;
	size_t apexIndex = 0, leftIndex = 0, rightIndex = 0;

	for (size_t i = 1; i <= n; ++i)
	{
		const Vector &left = (i == n) ? goal : gates[i].left;
		const Vector &right = (i == n) ? goal : gates[i].right;

		// Tighten the right funnel edge.
		if (TriArea2(apex, funnelRight, right) <= 0.f)
		{
			if (SamePoint2D(apex, funnelRight) || TriArea2(apex, funnelLeft, right) > 0.f)
			{
				funnelRight = right;
				rightIndex = i;
			}
			else
			{
				// Right would cross over left -- the left endpoint is a corner.
				path.push_back(funnelLeft);
				apex = funnelLeft;
				apexIndex = leftIndex;
				funnelLeft = apex;
				funnelRight = apex;
				leftIndex = apexIndex;
				rightIndex = apexIndex;
				i = apexIndex; // restart scan just past the new apex (loop ++ makes it apexIndex+1)
				continue;
			}
		}

		// Tighten the left funnel edge (mirror of the above).
		if (TriArea2(apex, funnelLeft, left) >= 0.f)
		{
			if (SamePoint2D(apex, funnelLeft) || TriArea2(apex, funnelRight, left) < 0.f)
			{
				funnelLeft = left;
				leftIndex = i;
			}
			else
			{
				path.push_back(funnelRight);
				apex = funnelRight;
				apexIndex = rightIndex;
				funnelLeft = apex;
				funnelRight = apex;
				leftIndex = apexIndex;
				rightIndex = apexIndex;
				i = apexIndex;
				continue;
			}
		}
	}

	path.push_back(goal);
	return path;
}

} // namespace NavFunnel
