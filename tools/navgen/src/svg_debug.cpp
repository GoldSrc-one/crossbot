#include "svg_debug.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <stdexcept>

#include "nav_format.h"

namespace navgen
{

namespace
{

struct Pt2
{
	double x = 0.0, y = 0.0;
};

double Cross2(const Pt2 &o, const Pt2 &a, const Pt2 &b)
{
	return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);
}

// Andrew's monotone chain: the XY projection of a convex 3D polytope is
// itself convex, so this always recovers exactly the area's true top-down
// footprint from its (possibly many, possibly duplicated between coplanar
// faces) projected vertices.
std::vector<Pt2> ConvexHull2D(std::vector<Pt2> pts)
{
	std::sort(pts.begin(), pts.end(), [](const Pt2 &a, const Pt2 &b)
		{ return a.x < b.x || (a.x == b.x && a.y < b.y); });
	pts.erase(std::unique(pts.begin(), pts.end(),
				  [](const Pt2 &a, const Pt2 &b) { return a.x == b.x && a.y == b.y; }),
		pts.end());

	int n = static_cast<int>(pts.size());
	if (n < 3)
		return pts;

	std::vector<Pt2> hull(2 * static_cast<size_t>(n));
	int k = 0;
	for (int i = 0; i < n; ++i)
	{
		while (k >= 2 && Cross2(hull[static_cast<size_t>(k - 2)], hull[static_cast<size_t>(k - 1)], pts[static_cast<size_t>(i)]) <= 0)
			--k;
		hull[static_cast<size_t>(k++)] = pts[static_cast<size_t>(i)];
	}
	int lower = k + 1;
	for (int i = n - 2; i >= 0; --i)
	{
		while (k >= lower && Cross2(hull[static_cast<size_t>(k - 2)], hull[static_cast<size_t>(k - 1)], pts[static_cast<size_t>(i)]) <= 0)
			--k;
		hull[static_cast<size_t>(k++)] = pts[static_cast<size_t>(i)];
	}
	hull.resize(static_cast<size_t>(k - 1));
	return hull;
}

std::vector<Pt2> AreaFootprint(const Area &area)
{
	std::vector<Pt2> pts;
	for (const Face &f : area.polytope.faces)
		for (const Vec3 &v : f.winding)
			pts.push_back({v.x, -v.y}); // negate Y to match the flipped-Y convention used everywhere else in this file
	return ConvexHull2D(std::move(pts));
}

struct EdgeStyle
{
	const char *color;
	const char *dash; // nullptr = solid
	double widthMul;  // multiplier on the base stroke width
};

EdgeStyle StyleForEdge(uint8_t edgeType)
{
	using namespace crossbot_nav;
	switch (edgeType)
	{
	case EDGE_JUMP:
		return {"#e08a00", "6,4", 1.0};
	// EDGE_DOOR is retired as of nav_format.h v5 -- a door is its own real
	// area now (NAV_AREA_DOOR), not a special edge type, so this case never
	// actually fires; falls through to the EDGE_WALK default below like any
	// other portal into/out of a door area.
	case EDGE_LADDER:
		return {"#8e44ad", nullptr, 1.6};
	case EDGE_DROP:
		return {"#c0392b", "8,3", 1.3};
	case EDGE_ELEVATOR:
		return {"#27ae60", nullptr, 1.6};
	case EDGE_LAUNCH:
		return {"#d4a017", "2,3", 1.3};
	case EDGE_WALK:
	default:
		return {"#98a6b3", nullptr, 1.0};
	}
}

void WriteHeader(FILE *f, const std::string &mapName, double minX, double minY, double w, double h)
{
	std::fprintf(f,
		"<svg xmlns=\"http://www.w3.org/2000/svg\" viewBox=\"%.1f %.1f %.1f %.1f\" "
		"font-family=\"monospace\">\n",
		minX, minY, w, h);
	std::fprintf(f, "<title>%s nav debug</title>\n", mapName.c_str());
	std::fprintf(f, "<rect x=\"%.1f\" y=\"%.1f\" width=\"%.1f\" height=\"%.1f\" fill=\"#0b1220\"/>\n", minX, minY, w, h);
}

} // namespace

void WriteDebugSvg(const std::string &path, const std::string &mapName, const std::vector<Area> &areas,
	const std::vector<MergedPortal> &portals, const std::vector<bool> &crouchAreas,
	const std::vector<TacticalScore> &tacticalScores, const std::vector<Vec3> &spawnPoints,
	const std::vector<Vec3> &targetPoints, const std::vector<Polytope> &leafPolytopesById,
	const std::vector<DoorAreaInfo> &doorAreaInfo, const std::vector<Vec3> &pathPoints,
	const std::vector<Vec3> &simPathPoints, const std::vector<Vec3> &simFallPoints,
	const std::vector<Vec3> &tracePoints)
{
	FILE *f = std::fopen(path.c_str(), "w");
	if (!f)
		throw std::runtime_error("cannot open output SVG file: " + path);

	// World Y grows "north"; SVG Y grows downward. Flip by negating Y on
	// every emitted coordinate (svgY = -worldY) so north still reads as "up"
	// on screen, and build the viewBox in that same flipped space.
	bool haveAny = false;
	double minX = 1e30, maxX = -1e30, minY = 1e30, maxY = -1e30;
	auto Consider = [&](double x, double y)
	{
		haveAny = true;
		minX = std::min(minX, x);
		maxX = std::max(maxX, x);
		minY = std::min(minY, -y);
		maxY = std::max(maxY, -y);
	};
	for (const Area &area : areas)
		for (const Face &fc : area.polytope.faces)
			for (const Vec3 &v : fc.winding)
				Consider(v.x, v.y);
	for (const Area &area : areas)
		for (int leafId : area.memberLeafIds)
			if (leafId >= 0 && static_cast<size_t>(leafId) < leafPolytopesById.size())
				for (const Face &fc : leafPolytopesById[static_cast<size_t>(leafId)].faces)
					for (const Vec3 &v : fc.winding)
						Consider(v.x, v.y);
	for (const Vec3 &p : spawnPoints)
		Consider(p.x, p.y);
	for (const Vec3 &p : targetPoints)
		Consider(p.x, p.y);

	if (!haveAny)
	{
		minX = minY = 0.0;
		maxX = maxY = 100.0;
	}

	double spanX = std::max(1.0, maxX - minX);
	double spanY = std::max(1.0, maxY - minY);
	double unit = std::max(spanX, spanY) / 100.0; // scale for strokes/fonts/markers, size-independent of the map
	double pad = unit * 6.0;

	WriteHeader(f, mapName, minX - pad, minY - pad, spanX + 2 * pad, spanY + 2 * pad);

	// Arrowhead marker for one-way edges (EDGE_FLAG_ONE_WAY, and EDGE_DROP
	// which is always one-way) -- points from areaA (the "from" side, per
	// TagDropOffs' own convention) toward areaB.
	std::fprintf(f,
		"<defs><marker id=\"arrow\" viewBox=\"0 0 10 10\" refX=\"8\" refY=\"5\" markerWidth=\"5\" markerHeight=\"5\" "
		"orient=\"auto-start-reverse\"><path d=\"M0,0 L10,5 L0,10 z\" fill=\"#c0392b\"/></marker></defs>\n");

	bool haveTactical = tacticalScores.size() == areas.size();
	bool haveCrouch = crouchAreas.size() == areas.size();
	bool haveDoor = doorAreaInfo.size() == areas.size();

	std::vector<Pt2> centroids(areas.size());
	for (size_t i = 0; i < areas.size(); ++i)
	{
		Vec3 c = areas[i].polytope.Centroid();
		centroids[i] = {c.x, -c.y};
	}

	// --- Areas first: filled convex footprint, colored by structural flag.
	// When leafPolytopesById is available, draw one polygon per ORIGINAL
	// member leaf instead of just area.polytope's own boundary -- see this
	// function's header comment (AbsorbTinyAreas can fold a leaf's portal
	// connectivity into a survivor without folding its geometry in, which
	// would otherwise leave that leaf's own volume rendered as an empty
	// hole even though it's genuinely part of the area). ---
	std::fprintf(f, "<g stroke=\"#2c3e50\" stroke-width=\"%.2f\" fill-opacity=\"0.85\">\n", unit * 0.15);
	for (size_t i = 0; i < areas.size(); ++i)
	{
		bool crouch = haveCrouch && crouchAreas[i];
		bool isDoor = haveDoor && doorAreaInfo[i].isDoor;
		const char *fill = isDoor ? "#7fa8d9" : crouch ? "#caa6f2" : "#c9d6e3";

		std::vector<std::vector<Pt2>> footprints;
		if (!leafPolytopesById.empty())
		{
			for (int leafId : areas[i].memberLeafIds)
			{
				if (leafId < 0 || static_cast<size_t>(leafId) >= leafPolytopesById.size())
					continue;
				std::vector<Pt2> pts;
				for (const Face &fc : leafPolytopesById[static_cast<size_t>(leafId)].faces)
					for (const Vec3 &v : fc.winding)
						pts.push_back({v.x, -v.y});
				std::vector<Pt2> hull = ConvexHull2D(std::move(pts));
				if (hull.size() >= 3)
					footprints.push_back(std::move(hull));
			}
		}
		if (footprints.empty())
		{
			std::vector<Pt2> hull = AreaFootprint(areas[i]);
			if (hull.size() >= 3)
				footprints.push_back(std::move(hull));
		}

		for (const std::vector<Pt2> &hull : footprints)
		{
			std::fprintf(f, "<polygon fill=\"%s\" points=\"", fill);
			for (const Pt2 &v : hull)
				std::fprintf(f, "%.1f,%.1f ", v.x, v.y);
			std::fprintf(f, "\"/>\n");
		}
	}
	std::fprintf(f, "</g>\n");

	// --- Portals/special edges drawn on top of the area fills -- many test
	// maps tile areas edge-to-edge with no gap between them at all, and a
	// centroid-to-centroid line drawn *underneath* a later, near-opaque area
	// polygon would be completely hidden the whole way. Labels still draw
	// after this, on top of both. ---
	std::fprintf(f, "<g stroke-linecap=\"round\">\n");
	for (const MergedPortal &p : portals)
	{
		if (static_cast<size_t>(p.areaA) >= centroids.size() || static_cast<size_t>(p.areaB) >= centroids.size())
			continue;
		const Pt2 &a = centroids[static_cast<size_t>(p.areaA)];
		const Pt2 &b = centroids[static_cast<size_t>(p.areaB)];
		EdgeStyle style = StyleForEdge(p.edgeType);
		bool oneWay = (p.edgeFlags & crossbot_nav::EDGE_FLAG_ONE_WAY) != 0;

		std::fprintf(f, "<line x1=\"%.1f\" y1=\"%.1f\" x2=\"%.1f\" y2=\"%.1f\" stroke=\"%s\" stroke-width=\"%.2f\"",
			a.x, a.y, b.x, b.y, style.color, unit * 0.35 * style.widthMul);
		if (style.dash)
			std::fprintf(f, " stroke-dasharray=\"%s\"", style.dash);
		if (oneWay)
			std::fprintf(f, " marker-end=\"url(#arrow)\"");
		std::fprintf(f, "/>\n");
	}
	std::fprintf(f, "</g>\n");

	// --- Button/trigger-gated door areas: a small square at the activator's
	// position, dotted-connected to the door area's own centroid (not a
	// specific portal -- a door area can have more than one portal into it,
	// unlike the old proximity-anchored special edge this replaced), so
	// it's clear at a glance that entering this area needs an extra step
	// first. ---
	if (haveDoor)
	{
		for (size_t i = 0; i < areas.size(); ++i)
		{
			const DoorAreaInfo &d = doorAreaInfo[i];
			if (!d.isDoor || (!d.needsButton && !d.needsTrigger))
				continue;
			Pt2 act{d.activatorPos.x, -d.activatorPos.y};
			const Pt2 &center = centroids[i];
			const char *actColor = d.needsButton ? "#e08a00" : "#16a085";
			std::fprintf(f,
				"<line x1=\"%.1f\" y1=\"%.1f\" x2=\"%.1f\" y2=\"%.1f\" stroke=\"%s\" stroke-width=\"%.2f\" "
				"stroke-dasharray=\"1,%.2f\"/>\n",
				center.x, center.y, act.x, act.y, actColor, unit * 0.15, unit * 0.6);
			double s = unit * 0.9;
			std::fprintf(f,
				"<rect x=\"%.1f\" y=\"%.1f\" width=\"%.1f\" height=\"%.1f\" fill=\"%s\" stroke=\"#0b1220\" stroke-width=\"%.2f\"/>\n",
				act.x - s * 0.5, act.y - s * 0.5, s, s, actColor, unit * 0.1);
		}
	}

	// --- Area labels: id number, plus tactical badge letters. ---
	std::fprintf(f, "<g fill=\"#0b1220\" text-anchor=\"middle\" font-size=\"%.2f\">\n", unit * 3.2);
	for (size_t i = 0; i < areas.size(); ++i)
	{
		const Pt2 &c = centroids[i];
		std::fprintf(f, "<text x=\"%.1f\" y=\"%.1f\">%d</text>\n", c.x, c.y, areas[i].id);

		std::string badges;
		if (haveTactical)
		{
			const TacticalScore &t = tacticalScores[i];
			if (t.isVantage)
				badges += "V";
			if (t.isCamp)
				badges += "C";
			if (t.isChokepointCover)
				badges += "K";
		}
		if (haveDoor && doorAreaInfo[i].isDoor)
			// T=touch (no activator needed), B=button, G=trigger ("g" for
			// the trigger_multiple's own footprint, not "t" -- already taken).
			badges += doorAreaInfo[i].needsButton ? "B" : doorAreaInfo[i].needsTrigger ? "G" : "T";
		if (!badges.empty())
			std::fprintf(f, "<text x=\"%.1f\" y=\"%.1f\" font-size=\"%.2f\" fill=\"#7a1f1f\">%s</text>\n", c.x,
				c.y + unit * 3.0, unit * 2.4, badges.c_str());
	}
	std::fprintf(f, "</g>\n");

	// --- Spawn-to-target planned-path overlay (the shared planner's
	// waypoint polyline) -- drawn as a bright polyline with a small joint
	// marker at each waypoint, on top of the area fills/portal lines/labels
	// but under the spawn/target markers themselves so those stay fully
	// visible at the path's own endpoints. ---
	if (pathPoints.size() >= 2)
	{
		std::fprintf(f, "<polyline fill=\"none\" stroke=\"#f1c40f\" stroke-width=\"%.2f\" stroke-opacity=\"0.9\" "
			"stroke-linecap=\"round\" stroke-linejoin=\"round\" points=\"",
			unit * 0.7);
		for (const Vec3 &p : pathPoints)
			std::fprintf(f, "%.1f,%.1f ", p.x, -p.y);
		std::fprintf(f, "\"/>\n");
		for (const Vec3 &p : pathPoints)
			std::fprintf(f,
				"<circle cx=\"%.1f\" cy=\"%.1f\" r=\"%.2f\" fill=\"#f1c40f\" stroke=\"#7a5c00\" stroke-width=\"%.2f\"/>\n",
				p.x, -p.y, unit * 0.6, unit * 0.12);
	}

	// --- Momentum-simulated trajectory (NavFollow::Simulate) on top of the
	// planned line: where the orange curve bows away from the yellow line is
	// exactly the corner-cut/convergence drift the real bot will drive. A
	// red X marks a predicted fall (the simulated origin left every
	// supported leaf's volume). ---
	if (simPathPoints.size() >= 2)
	{
		std::fprintf(f, "<polyline fill=\"none\" stroke=\"#e67e22\" stroke-width=\"%.2f\" stroke-opacity=\"0.9\" "
			"stroke-linecap=\"round\" stroke-linejoin=\"round\" points=\"",
			unit * 0.35);
		for (const Vec3 &p : simPathPoints)
			std::fprintf(f, "%.1f,%.1f ", p.x, -p.y);
		std::fprintf(f, "\"/>\n");
	}
	for (const Vec3 &p : simFallPoints)
	{
		double s = unit * 1.6;
		std::fprintf(f,
			"<line x1=\"%.1f\" y1=\"%.1f\" x2=\"%.1f\" y2=\"%.1f\" stroke=\"#ff2222\" stroke-width=\"%.2f\"/>"
			"<line x1=\"%.1f\" y1=\"%.1f\" x2=\"%.1f\" y2=\"%.1f\" stroke=\"#ff2222\" stroke-width=\"%.2f\"/>\n",
			p.x - s, -p.y - s, p.x + s, -p.y + s, unit * 0.4, p.x - s, -p.y + s, p.x + s, -p.y - s, unit * 0.4);
	}

	// --- Recorded live-bot trajectory (--trace): what the real engine
	// actually drove, in cyan, over both the plan and the simulation. ---
	if (tracePoints.size() >= 2)
	{
		std::fprintf(f, "<polyline fill=\"none\" stroke=\"#00d0ff\" stroke-width=\"%.2f\" stroke-opacity=\"0.85\" "
			"stroke-linecap=\"round\" stroke-linejoin=\"round\" points=\"",
			unit * 0.3);
		for (const Vec3 &p : tracePoints)
			std::fprintf(f, "%.1f,%.1f ", p.x, -p.y);
		std::fprintf(f, "\"/>\n");
	}

	// --- Spawn/target markers. ---
	for (const Vec3 &p : spawnPoints)
		std::fprintf(f,
			"<circle cx=\"%.1f\" cy=\"%.1f\" r=\"%.2f\" fill=\"#2ecc71\" stroke=\"#0b1220\" stroke-width=\"%.2f\"/>\n",
			p.x, -p.y, unit * 1.6, unit * 0.2);
	for (const Vec3 &p : targetPoints)
		std::fprintf(f,
			"<circle cx=\"%.1f\" cy=\"%.1f\" r=\"%.2f\" fill=\"#e74c3c\" stroke=\"#0b1220\" stroke-width=\"%.2f\"/>\n",
			p.x, -p.y, unit * 1.6, unit * 0.2);

	// --- Legend, fixed in the top-left of the viewBox, on its own opaque
	// panel so it stays legible regardless of whatever area/edge happens to
	// render underneath it. ---
	{
		double lx = minX - pad + unit * 1.0;
		double ly = minY - pad + unit * 4.0;
		double lh = unit * 3.4;
		double panelW = unit * 34.0;
		double legendRows = 14.6;
		if (pathPoints.size() >= 2)
			legendRows += 1.0;
		if (simPathPoints.size() >= 2)
			legendRows += 1.0;
		if (tracePoints.size() >= 2)
			legendRows += 1.0;
		double panelH = lh * legendRows;
		std::fprintf(f, "<rect x=\"%.1f\" y=\"%.1f\" width=\"%.1f\" height=\"%.1f\" fill=\"#0b1220\" fill-opacity=\"0.85\" stroke=\"#4a5a6a\" stroke-width=\"%.2f\"/>\n",
			minX - pad, minY - pad, panelW, panelH, unit * 0.15);
		auto Row = [&](const char *swatchColor, const char *dash, const char *label, bool isArea)
		{
			if (isArea)
				std::fprintf(f, "<rect x=\"%.1f\" y=\"%.1f\" width=\"%.2f\" height=\"%.2f\" fill=\"%s\"/>\n", lx,
					ly - unit * 1.6, unit * 3.0, unit * 1.8, swatchColor);
			else
			{
				std::fprintf(f, "<line x1=\"%.1f\" y1=\"%.1f\" x2=\"%.1f\" y2=\"%.1f\" stroke=\"%s\" stroke-width=\"%.2f\"",
					lx, ly - unit * 0.7, lx + unit * 3.0, ly - unit * 0.7, swatchColor, unit * 0.5);
				if (dash)
					std::fprintf(f, " stroke-dasharray=\"%s\"", dash);
				std::fprintf(f, "/>\n");
			}
			std::fprintf(f, "<text x=\"%.1f\" y=\"%.1f\" font-size=\"%.2f\" fill=\"#e8eef4\">%s</text>\n",
				lx + unit * 4.0, ly, unit * 2.6, label);
			ly += lh;
		};
		std::fprintf(f, "<text x=\"%.1f\" y=\"%.1f\" font-size=\"%.2f\" fill=\"#ffffff\" font-weight=\"bold\">%s</text>\n",
			lx, ly - unit * 2.0, unit * 3.4, mapName.c_str());
		ly += lh * 0.6;
		Row("#c9d6e3", nullptr, "area (normal)", true);
		Row("#caa6f2", nullptr, "area (crouch-only)", true);
		Row("#7fa8d9", nullptr, "area (door)", true);
		Row("#98a6b3", nullptr, "walk portal", false);
		Row("#e08a00", "6,4", "jump (heuristic)", false);
		Row("#c0392b", "8,3", "one-way drop", false);
		Row("#8e44ad", nullptr, "ladder", false);
		Row("#27ae60", nullptr, "elevator", false);
		if (pathPoints.size() >= 2)
			Row("#f1c40f", nullptr, "planned path (spawn->target)", false);
		if (simPathPoints.size() >= 2)
			Row("#e67e22", nullptr, simFallPoints.empty() ? "simulated drive (momentum)" : "simulated drive (X = predicted fall)", false);
		if (tracePoints.size() >= 2)
			Row("#00d0ff", nullptr, "recorded bot trajectory", false);
		std::fprintf(f, "<text x=\"%.1f\" y=\"%.1f\" font-size=\"%.2f\" fill=\"#e8eef4\">V=vantage C=camp K=chokepoint-cover</text>\n",
			lx, ly, unit * 2.4);
		ly += lh;
		std::fprintf(f,
			"<text x=\"%.1f\" y=\"%.1f\" font-size=\"%.2f\" fill=\"#e8eef4\">door area: T=touch B=button G=trigger</text>\n",
			lx, ly, unit * 2.4);
		ly += lh;
		std::fprintf(f,
			"<circle cx=\"%.1f\" cy=\"%.1f\" r=\"%.2f\" fill=\"#2ecc71\"/>"
			"<text x=\"%.1f\" y=\"%.1f\" font-size=\"%.2f\" fill=\"#e8eef4\">spawn</text>"
			"<circle cx=\"%.1f\" cy=\"%.1f\" r=\"%.2f\" fill=\"#e74c3c\"/>"
			"<text x=\"%.1f\" y=\"%.1f\" font-size=\"%.2f\" fill=\"#e8eef4\">target</text>\n",
			lx + unit * 1.5, ly - unit * 0.7, unit * 1.6, lx + unit * 4.0, ly, unit * 2.6, lx + unit * 14.0,
			ly - unit * 0.7, unit * 1.6, lx + unit * 16.5, ly, unit * 2.6);
	}

	std::fprintf(f, "</svg>\n");
	std::fclose(f);
}

} // namespace navgen
