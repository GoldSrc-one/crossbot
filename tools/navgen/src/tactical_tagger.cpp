#include "tactical_tagger.h"

#include <algorithm>

namespace navgen
{

namespace
{

// Beyond typical HLDM engagement range -- prefilters most pairs cheaply
// before the (much pricier) hull-trace visibility check.
constexpr double kMaxSightlineDistance = 4000.0;

// "Immediate threat" radius for concealmentScore -- distinct from the full
// sightline cutoff above. A spot can have a long sightline to somewhere far
// away (which is what makes it a good vantage point) while still being
// well-hidden from anything actually close enough to threaten it;
// concealmentScore specifically measures the latter.
constexpr double kNearbyThreatRadius = 800.0;

// A portal narrower than this (its shared-face area, from portal_builder's
// output) reads as a doorway/corridor chokepoint rather than an open room
// boundary -- a standard HL door opening is roughly 64x96 = ~6144 sq
// units, so this comfortably includes doorways while excluding big
// open room-to-room connections.
constexpr double kChokepointPortalArea = 8000.0;
constexpr double kChokepointMinStandoff = 150.0;
constexpr double kChokepointMaxRange = 1200.0; // "covering" a door from sniper range isn't the same trait

bool TraceRecursive(const BspFile &bsp, int node, const Vec3 &p1, const Vec3 &p2)
{
	if (node < 0)
		return node != CONTENTS_SOLID;

	const dclipnode_t &cn = bsp.Clipnodes()[static_cast<size_t>(node)];
	const dplane_t &pl = bsp.Planes()[static_cast<size_t>(cn.planenum)];
	Vec3 normal(pl.normal[0], pl.normal[1], pl.normal[2]);
	double dist = pl.dist;

	double d1 = normal.Dot(p1) - dist;
	double d2 = normal.Dot(p2) - dist;

	if (d1 >= 0.0 && d2 >= 0.0)
		return TraceRecursive(bsp, cn.children[0], p1, p2);
	if (d1 < 0.0 && d2 < 0.0)
		return TraceRecursive(bsp, cn.children[1], p1, p2);

	// Segment straddles the plane -- split at the intersection and check
	// the near side first, same as the engine's own hull-trace algorithm,
	// so a solid hit before the split point is detected before continuing
	// past it.
	double t = d1 / (d1 - d2);
	Vec3 mid = p1 + (p2 - p1) * t;

	int nearSide = (d1 < 0.0) ? 1 : 0;
	int farSide = (d1 < 0.0) ? 0 : 1;

	if (!TraceRecursive(bsp, cn.children[nearSide], p1, mid))
		return false;
	return TraceRecursive(bsp, cn.children[farSide], mid, p2);
}

// True if a straight line between p1/p2 doesn't cross solid geometry.
// Walks hull 1's clipnode tree (the same hull leaf_extractor/
// blocking_entities already trust elsewhere in navgen) rather than hull 0
// -- hull 1's ~16-unit-per-side Minkowski inflation makes this a slightly
// conservative proxy for true eye-level line-of-sight (a sightline
// grazing very close to a corner could read as blocked when a human's
// actual eye-point trace would clear it), acceptable for a heuristic
// tactical score, not attempted to be exact.
bool TraceClear(const BspFile &bsp, int hull1Headnode, const Vec3 &p1, const Vec3 &p2)
{
	return TraceRecursive(bsp, hull1Headnode, p1, p2);
}

double Percentile(std::vector<double> values, double fraction)
{
	if (values.empty())
		return 0.0;
	std::sort(values.begin(), values.end());
	size_t idx = static_cast<size_t>(static_cast<double>(values.size() - 1) * fraction);
	return values[idx];
}

} // namespace

std::vector<TacticalScore> TagTactical(const BspFile &bsp, int hull1Headnode, const std::vector<Area> &areas,
	const std::vector<MergedPortal> &portals)
{
	size_t n = areas.size();
	std::vector<TacticalScore> scores(n);
	std::vector<Vec3> centroid(n);
	for (size_t i = 0; i < n; ++i)
		centroid[i] = areas[i].polytope.Centroid();

	// Visibility pass: distance-prefiltered, hull-trace-validated. O(n^2)
	// candidate pairs, but the cheap centroid-distance check eliminates
	// most of them before the pricier trace -- fine for an offline batch
	// tool with no frame-slicing need (same reasoning the rest of navgen's
	// pipeline already relies on).
	std::vector<int> nearbyVisibleCount(n, 0);
	std::vector<std::vector<size_t>> visibleTo(n);

	for (size_t i = 0; i < n; ++i)
	{
		for (size_t j = i + 1; j < n; ++j)
		{
			double dist = (centroid[i] - centroid[j]).Length();
			if (dist > kMaxSightlineDistance)
				continue;
			if (!TraceClear(bsp, hull1Headnode, centroid[i], centroid[j]))
				continue;

			scores[i].exposureScore += dist;
			scores[j].exposureScore += dist;
			visibleTo[i].push_back(j);
			visibleTo[j].push_back(i);

			if (dist <= kNearbyThreatRadius)
			{
				++nearbyVisibleCount[i];
				++nearbyVisibleCount[j];
			}
		}
	}

	for (size_t i = 0; i < n; ++i)
		scores[i].concealmentScore = 1.0 / (1.0 + static_cast<double>(nearbyVisibleCount[i]));

	// dangerScore: propagate each visible neighbor's own exposureScore back
	// onto this area -- being seen *by* a long-sightline vantage spot is
	// dangerous even if this area's own sightlines are all short.
	for (size_t i = 0; i < n; ++i)
	{
		double danger = 0.0;
		for (size_t j : visibleTo[i])
			danger += scores[j].exposureScore;
		scores[i].dangerScore = danger;
	}

	// Vantage: top-decile exposureScore (and > 0, so an isolated area with
	// no visibility data at all never qualifies by default).
	std::vector<double> exposureValues;
	for (const TacticalScore &s : scores)
		exposureValues.push_back(s.exposureScore);
	double vantageThreshold = Percentile(exposureValues, 0.9);
	for (TacticalScore &s : scores)
		if (s.exposureScore > 0.0 && s.exposureScore >= vantageThreshold)
			s.isVantage = true;

	// Camp: well-concealed from nearby threats, but not isolated -- still
	// portal-connected to somewhere with real traffic potential (portal
	// count as a simple connectivity proxy for "on/near a route", since
	// navgen has no actual traffic data to draw on).
	std::vector<int> portalCount(n, 0);
	for (const MergedPortal &p : portals)
	{
		if (p.areaA >= 0 && static_cast<size_t>(p.areaA) < n)
			++portalCount[static_cast<size_t>(p.areaA)];
		if (p.areaB >= 0 && static_cast<size_t>(p.areaB) < n)
			++portalCount[static_cast<size_t>(p.areaB)];
	}
	std::vector<double> portalCountValues;
	for (int c : portalCount)
		portalCountValues.push_back(static_cast<double>(c));
	double medianPortalCount = Percentile(portalCountValues, 0.5);

	std::vector<double> concealmentValues;
	for (const TacticalScore &s : scores)
		concealmentValues.push_back(s.concealmentScore);
	double concealmentThreshold = Percentile(concealmentValues, 0.9);

	for (size_t i = 0; i < n; ++i)
		if (scores[i].concealmentScore >= concealmentThreshold &&
			static_cast<double>(portalCount[i]) >= medianPortalCount)
			scores[i].isCamp = true;

	// Chokepoint cover: has a sightline onto a narrow portal (doorway/
	// corridor) from real standoff distance, at moderate (not sniper)
	// range -- a sentry/turret placement candidate. Scored rather than a
	// plain pass/fail: a first cut at "sees any narrow portal within
	// range" flagged nearly every area on crossfire.bsp (773 of 798) --
	// narrow connecting portals between small unmerged areas turn out to
	// be common, so a binary test isn't selective at all. Weighting by how
	// narrow the portal is and keeping only the top 5% by that score
	// (tighter than vantage/camp's 90th percentile, since covering a
	// specific chokepoint is meant to be a comparatively rare trait) fixes
	// that regardless of how many raw candidate pairs exist.
	std::vector<double> chokepointScore(n, 0.0);
	for (const MergedPortal &p : portals)
	{
		if (p.area <= 0.0 || p.area >= kChokepointPortalArea)
			continue; // area == 0 is a proximity-derived edge (jump gap), not a real doorway

		double narrowness = 1.0 - (p.area / kChokepointPortalArea); // 0 (at cutoff) .. ~1 (a tight gap)
		Vec3 portalMid = (p.pointA + p.pointB) * 0.5;
		for (size_t i = 0; i < n; ++i)
		{
			double dist = (centroid[i] - portalMid).Length();
			if (dist < kChokepointMinStandoff || dist > kChokepointMaxRange)
				continue;
			if (TraceClear(bsp, hull1Headnode, centroid[i], portalMid))
				chokepointScore[i] += narrowness;
		}
	}

	double chokepointThreshold = Percentile(chokepointScore, 0.95);
	for (size_t i = 0; i < n; ++i)
		if (chokepointScore[i] > 0.0 && chokepointScore[i] >= chokepointThreshold)
			scores[i].isChokepointCover = true;

	return scores;
}

} // namespace navgen
