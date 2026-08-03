#include "funnel_graph.h"

#include <cmath>

namespace NavFunnel
{

namespace
{

float TriArea2(const Vector &a, const Vector &b, const Vector &c)
{
	float abx = b.x - a.x, aby = b.y - a.y;
	float acx = c.x - a.x, acy = c.y - a.y;
	return acx * aby - abx * acy;
}

bool SamePoint2D(const Vector &a, const Vector &b)
{

	constexpr float kEps = 0.001f;
	return std::fabs(a.x - b.x) < kEps && std::fabs(a.y - b.y) < kEps;
}

}

Gate MakeGate(const Vector &pointA, const Vector &pointB, const Vector &travelDir2D)
{

	float side = travelDir2D.x * (pointA.y - pointB.y) - travelDir2D.y * (pointA.x - pointB.x);
	Gate gate;
	gate.left = (side > 0.f) ? pointA : pointB;
	gate.right = (side > 0.f) ? pointB : pointA;
	return gate;
}

std::vector<PulledPoint> StringPull(const Vector &start, const Vector &goal,
	const std::vector<Gate> &gates)
{
	std::vector<PulledPoint> path;
	path.push_back({start, kNoGate});

	auto push = [&](const PulledPoint &p)
	{
		if (!path.empty() && SamePoint2D(path.back().point, p.point))
			path.back() = p;
		else
			path.push_back(p);
	};

	if (gates.empty())
	{
		push({goal, kNoGate});
		return path;
	}

	size_t n = gates.size();
	Vector apex = start, funnelLeft = gates[0].left, funnelRight = gates[0].right;
	size_t apexIndex = 0, leftIndex = 0, rightIndex = 0;

	for (size_t i = 1; i <= n; ++i)
	{
		const Vector &left = (i == n) ? goal : gates[i].left;
		const Vector &right = (i == n) ? goal : gates[i].right;

		if (TriArea2(apex, funnelRight, right) <= 0.f)
		{
			if (SamePoint2D(apex, funnelRight) || TriArea2(apex, funnelLeft, right) > 0.f)
			{
				funnelRight = right;
				rightIndex = i;
			}
			else
			{

				push({funnelLeft, leftIndex});
				apex = funnelLeft;
				apexIndex = leftIndex;
				funnelLeft = apex;
				funnelRight = apex;
				leftIndex = apexIndex;
				rightIndex = apexIndex;
				i = apexIndex;
				continue;
			}
		}

		if (TriArea2(apex, funnelLeft, left) >= 0.f)
		{
			if (SamePoint2D(apex, funnelLeft) || TriArea2(apex, funnelRight, left) < 0.f)
			{
				funnelLeft = left;
				leftIndex = i;
			}
			else
			{
				push({funnelRight, rightIndex});
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

	push({goal, kNoGate});
	return path;
}

}

