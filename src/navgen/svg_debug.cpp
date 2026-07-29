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
			pts.push_back({v.x, -v.y});
	return ConvexHull2D(std::move(pts));
}

constexpr const char *kBackground = "#12141a";
constexpr const char *kAreaWalkable = "#8d9298";
constexpr const char *kAreaCrouch = "#93b3cf";
constexpr const char *kConnWalk = "#e8eef4";
constexpr const char *kConnAirmove = "#e74c3c";
constexpr const char *kConnTeleport = "#27ae60";
constexpr const char *kGate = "#8e6fc4";
constexpr const char *kGateJump = "#e74c3c";
constexpr const char *kRoute = "#f39c12";
// Distinct from kConnAirmove and kRouteGoal, which are also reds, so the route overlay stays
// readable on top of the connection layer.
constexpr const char *kRouteAirmoveSim = "#2ecc71";
constexpr const char *kRouteAirmoveStored = "#ff3b30";
constexpr const char *kRouteAirmoveMissed = "#f5d020";
constexpr const char *kRouteStart = "#27ae60";
constexpr const char *kRouteGoal = "#e74c3c";
// Where the bot actually went, as opposed to where it was told to go. A cool hue so it reads as a
// separate layer under the warm planned route rather than as part of it.
constexpr const char *kTrace = "#3fd0d4";
constexpr const char *kTargetEntity = "#c86bd8";
constexpr const char *kLabelText = "#e8eef4";

// Areas are see-through so a floor stacked over another still reveals what is beneath it.
constexpr double kAreaFillOpacity = 0.7;

// Lowest point of an area, i.e. its floor height -- the sort key for the painter's order.
double AreaFloorZ(const Area &area)
{
	double lowest = 1e30;
	for (const Face &f : area.polytope.faces)
		for (const Vec3 &v : f.winding)
			lowest = std::min(lowest, v.z);
	return lowest < 1e29 ? lowest : 0.0;
}

struct ConnStyle
{
	const char *color;
	const char *dash;
	double widthMul;
};

ConnStyle StyleForConn(uint8_t connType)
{
	using namespace crossbot_nav;
	switch (connType)
	{
	case CONN_AIRMOVE:
		return {kConnAirmove, "4,2", 1.3};

	case CONN_TELEPORT:
		return {kConnTeleport, nullptr, 1.6};
	case CONN_WALK:
	default:
		return {kConnWalk, nullptr, 1.0};
	}
}

const char *FillForArea(const Area &area)
{
	return (area.areaFlags & crossbot_nav::NAV_AREA_CROUCH) ? kAreaCrouch : kAreaWalkable;
}

void WriteHeader(FILE *f, const std::string &mapName, double minX, double minY, double w, double h)
{
	std::fprintf(f,
		"<svg xmlns=\"http://www.w3.org/2000/svg\" viewBox=\"%.1f %.1f %.1f %.1f\" "
		"font-family=\"monospace\">\n",
		minX, minY, w, h);
	std::fprintf(f, "<title>%s nav debug</title>\n", mapName.c_str());
	std::fprintf(f, "<rect x=\"%.1f\" y=\"%.1f\" width=\"%.1f\" height=\"%.1f\" fill=\"%s\"/>\n", minX, minY, w, h,
		kBackground);
}

}

void WriteDebugSvg(const std::string &path, const std::string &mapName, const std::vector<Area> &areas,
	const std::vector<MergedConnection> &conns, const std::vector<Vec3> &pathPoints,
	const std::vector<Vec3> &targetPoints, const std::vector<uint8_t> &pathSegmentKinds,
	const std::vector<FailedAirmove> &failedAirmoves, const std::vector<Vec3> &tracePoints)
{
	FILE *f = std::fopen(path.c_str(), "w");
	if (!f)
		throw std::runtime_error("cannot open output SVG file: " + path);

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

	for (const Vec3 &p : pathPoints)
		Consider(p.x, p.y);

	for (const Vec3 &p : targetPoints)
		Consider(p.x, p.y);

	for (const Vec3 &p : tracePoints)
		Consider(p.x, p.y);

	if (!haveAny)
	{
		minX = minY = 0.0;
		maxX = maxY = 100.0;
	}

	double spanX = std::max(1.0, maxX - minX);
	double spanY = std::max(1.0, maxY - minY);
	double unit = std::max(spanX, spanY) / 100.0;
	double pad = unit * 6.0;

	WriteHeader(f, mapName, minX - pad, minY - pad, spanX + 2 * pad, spanY + 2 * pad);

	std::fprintf(f,
		"<defs><marker id=\"arrow\" viewBox=\"0 0 10 10\" refX=\"8\" refY=\"5\" markerWidth=\"5\" markerHeight=\"5\" "
		"orient=\"auto-start-reverse\"><path d=\"M0,0 L10,5 L0,10 z\" fill=\"%s\"/></marker></defs>\n", kConnAirmove);

	std::vector<Pt2> centroids(areas.size());
	for (size_t i = 0; i < areas.size(); ++i)
	{
		Vec3 c = areas[i].polytope.Centroid();
		centroids[i] = {c.x, -c.y};
	}

	// Painter's order: this is a top-down view, so the lowest floors have to be laid down
	// first and higher ones paint over them. The natural order is area id, which drops
	// basements on top of the rooms above them at random.
	std::vector<size_t> drawOrder(areas.size());
	std::vector<double> floorZ(areas.size(), 0.0);
	for (size_t i = 0; i < areas.size(); ++i)
	{
		drawOrder[i] = i;
		floorZ[i] = AreaFloorZ(areas[i]);
	}
	std::stable_sort(drawOrder.begin(), drawOrder.end(),
		[&](size_t a, size_t b) { return floorZ[a] < floorZ[b]; });

	std::fprintf(f, "<g stroke=\"%s\" stroke-width=\"%.2f\" fill-opacity=\"%.2f\">\n", kBackground, unit * 0.2,
		kAreaFillOpacity);
	for (size_t i : drawOrder)
	{
		std::vector<Pt2> hull = AreaFootprint(areas[i]);
		if (hull.size() >= 3)
		{
			std::fprintf(f, "<polygon fill=\"%s\" points=\"", FillForArea(areas[i]));
			for (const Pt2 &v : hull)
				std::fprintf(f, "%.1f,%.1f ", v.x, v.y);
			std::fprintf(f, "\"/>\n");
		}
	}
	std::fprintf(f, "</g>\n");

	std::fprintf(f, "<g stroke-linecap=\"round\" stroke-opacity=\"0.85\">\n");
	for (const MergedConnection &c : conns)
	{
		if (static_cast<size_t>(c.areaA) >= centroids.size() || static_cast<size_t>(c.areaB) >= centroids.size())
			continue;
		const Pt2 &a = centroids[static_cast<size_t>(c.areaA)];
		const Pt2 &b = centroids[static_cast<size_t>(c.areaB)];
		ConnStyle style = StyleForConn(c.connType);
		bool oneWay = (c.connFlags & crossbot_nav::CONN_FLAG_ONE_WAY) != 0;

		std::fprintf(f, "<line x1=\"%.1f\" y1=\"%.1f\" x2=\"%.1f\" y2=\"%.1f\" stroke=\"%s\" stroke-width=\"%.2f\"",
			a.x, a.y, b.x, b.y, style.color, unit * 0.35 * style.widthMul);
		if (style.dash)
			std::fprintf(f, " stroke-dasharray=\"%s\"", style.dash);
		if (oneWay)
			std::fprintf(f, " marker-end=\"url(#arrow)\"");
		std::fprintf(f, "/>\n");
	}
	std::fprintf(f, "</g>\n");

	// Walk gates first, then the air-move ones on top: a launch edge is the interesting one when
	// both run along the same lip.
	auto drawGateLayer = [&](const char *color, bool airmove)
	{
		std::fprintf(f, "<g stroke=\"%s\" stroke-opacity=\"0.9\" stroke-width=\"%.2f\" "
			"stroke-dasharray=\"%.1f %.1f\" stroke-linecap=\"round\" fill=\"none\">\n",
			color, unit * 0.14, unit * 0.5, unit * 0.4);

		for (const MergedConnection &c : conns)
		{
			if ((c.connType == crossbot_nav::CONN_AIRMOVE) != airmove)
				continue;

			auto drawGate = [&](bool valid, const Vec3 &left, const Vec3 &right)
			{
				if (!valid)
					return;

				double dx = right.x - left.x, dy = right.y - left.y;
				if (dx * dx + dy * dy < 0.01)
				{
					std::fprintf(f, "<circle cx=\"%.1f\" cy=\"%.1f\" r=\"%.2f\"/>\n",
						left.x, -left.y, unit * 0.35);
					return;
				}

				std::fprintf(f, "<line x1=\"%.1f\" y1=\"%.1f\" x2=\"%.1f\" y2=\"%.1f\"/>\n",
					left.x, -left.y, right.x, -right.y);
			};

			drawGate(c.gateAValid, c.gateALeft, c.gateARight);
			drawGate(c.gateBValid, c.gateBLeft, c.gateBRight);
		}

		std::fprintf(f, "</g>\n");
	};

	drawGateLayer(kGate, false);
	drawGateLayer(kGateJump, true);

	std::fprintf(f, "<g fill=\"%s\">\n", kBackground);
	for (const Pt2 &c : centroids)
		std::fprintf(f, "<circle cx=\"%.1f\" cy=\"%.1f\" r=\"%.2f\"/>\n", c.x, c.y, unit * 0.5);
	std::fprintf(f, "</g>\n");

	// Laid down before the planned route so the two are comparable at a glance: wherever the bot
	// tracked the plan the orange sits on top of the cyan, and any excursion shows as bare cyan.
	if (tracePoints.size() >= 2)
	{
		std::fprintf(f, "<polyline fill=\"none\" stroke=\"%s\" stroke-width=\"%.2f\" stroke-opacity=\"0.9\" "
			"stroke-linecap=\"round\" stroke-linejoin=\"round\" points=\"", kTrace, unit * 0.6);
		for (const Vec3 &p : tracePoints)
			std::fprintf(f, "%.1f,%.1f ", p.x, -p.y);
		std::fprintf(f, "\"/>\n");
	}

	if (pathPoints.size() >= 2)
	{
		std::fprintf(f, "<polyline fill=\"none\" stroke=\"%s\" stroke-width=\"%.2f\" stroke-linecap=\"round\" "
			"stroke-linejoin=\"round\" points=\"", kRoute, unit * 0.9);
		for (const Vec3 &p : pathPoints)
			std::fprintf(f, "%.1f,%.1f ", p.x, -p.y);
		std::fprintf(f, "\"/>\n");

		// Overdraw the air moves so it is obvious which jumps the planner re-simulated from the
		// funnelled launch point and which fell back to the baked one.
		for (size_t i = 0; i + 1 < pathPoints.size() && i < pathSegmentKinds.size(); ++i)
		{
			if (pathSegmentKinds[i] == PATH_SEG_WALK)
				continue;

			const char *colour = (pathSegmentKinds[i] == PATH_SEG_AIRMOVE_SIMULATED)
				? kRouteAirmoveSim : kRouteAirmoveStored;
			const Vec3 &a = pathPoints[i];
			const Vec3 &b = pathPoints[i + 1];
			std::fprintf(f, "<line x1=\"%.1f\" y1=\"%.1f\" x2=\"%.1f\" y2=\"%.1f\" stroke=\"%s\" "
				"stroke-width=\"%.2f\" stroke-linecap=\"round\"/>\n",
				a.x, -a.y, b.x, -b.y, colour, unit * 1.3);
		}

		// Where a re-simulated jump was tried and missed: dashed from the launch to what it was
		// aiming at, with a ring on where it actually came down. A replay that never left the
		// ground ends on its own launch, so the aim is the only part with any length to it.
		for (const FailedAirmove &fa : failedAirmoves)
		{
			std::fprintf(f, "<line x1=\"%.1f\" y1=\"%.1f\" x2=\"%.1f\" y2=\"%.1f\" stroke=\"%s\" "
				"stroke-width=\"%.2f\" stroke-dasharray=\"%.1f %.1f\" stroke-linecap=\"round\"/>\n",
				fa.launch.x, -fa.launch.y, fa.aim.x, -fa.aim.y, kRouteAirmoveMissed,
				unit * 0.8, unit * 2.0, unit * 1.5);
			std::fprintf(f, "<circle cx=\"%.1f\" cy=\"%.1f\" r=\"%.2f\" fill=\"none\" stroke=\"%s\" "
				"stroke-width=\"%.2f\"/>\n",
				fa.landing.x, -fa.landing.y, unit * 1.6, kRouteAirmoveMissed, unit * 0.6);
		}

		const Vec3 &pathStart = pathPoints.front();
		const Vec3 &pathEnd = pathPoints.back();
		std::fprintf(f, "<circle cx=\"%.1f\" cy=\"%.1f\" r=\"%.2f\" fill=\"%s\"/>\n",
			pathStart.x, -pathStart.y, unit * 2.0, kRouteStart);
		std::fprintf(f, "<circle cx=\"%.1f\" cy=\"%.1f\" r=\"%.2f\" fill=\"%s\"/>\n",
			pathEnd.x, -pathEnd.y, unit * 2.0, kRouteGoal);
	}

	if (!targetPoints.empty())
	{
		double r = unit * 2.2;
		std::fprintf(f, "<g stroke=\"%s\" stroke-width=\"%.2f\" fill=\"none\">\n", kTargetEntity, unit * 0.6);
		for (const Vec3 &p : targetPoints)
		{
			double cx = p.x;
			double cy = -p.y;
			std::fprintf(f, "<circle cx=\"%.1f\" cy=\"%.1f\" r=\"%.2f\"/>\n", cx, cy, r);
			std::fprintf(f, "<line x1=\"%.1f\" y1=\"%.1f\" x2=\"%.1f\" y2=\"%.1f\"/>\n",
				cx - r * 1.6, cy, cx + r * 1.6, cy);
			std::fprintf(f, "<line x1=\"%.1f\" y1=\"%.1f\" x2=\"%.1f\" y2=\"%.1f\"/>\n",
				cx, cy - r * 1.6, cx, cy + r * 1.6);
		}
		std::fprintf(f, "</g>\n");
	}

	std::fprintf(f, "<g fill=\"%s\" text-anchor=\"middle\" font-size=\"%.2f\">\n", kBackground, unit * 3.2);
	for (size_t i = 0; i < areas.size(); ++i)
	{
		const Pt2 &c = centroids[i];
		std::fprintf(f, "<text x=\"%.1f\" y=\"%.1f\">%d</text>\n", c.x, c.y, areas[i].id);
	}
	std::fprintf(f, "</g>\n");

	{
		double lx = minX - pad + unit * 1.5;
		double ly = minY - pad + unit * 4.0;
		double lh = unit * 3.4;
		double panelW = unit * 30.0;
		// Derived from the row count rather than hand-tuned, so adding a legend entry cannot
		// silently push it outside the panel.
		constexpr int kLegendRows = 12;
		double panelH = lh * (kLegendRows + 1.6);
		double swatchW = unit * 3.0;
		double fontSize = unit * 2.6;
		double textX = lx + unit * 4.5;

		std::fprintf(f, "<rect x=\"%.1f\" y=\"%.1f\" width=\"%.1f\" height=\"%.1f\" fill=\"%s\" fill-opacity=\"0.85\" stroke=\"#4a5a6a\" stroke-width=\"%.2f\"/>\n",
			minX - pad, minY - pad, panelW, panelH, kBackground, unit * 0.15);

		auto Label = [&](const char *label)
		{
			std::fprintf(f, "<text x=\"%.1f\" y=\"%.1f\" font-size=\"%.2f\" fill=\"%s\">%s</text>\n",
				textX, ly, fontSize, kLabelText, label);
			ly += lh;
		};
		auto SwatchRow = [&](const char *color, const char *label)
		{
			std::fprintf(f, "<rect x=\"%.1f\" y=\"%.1f\" width=\"%.1f\" height=\"%.1f\" fill=\"%s\" fill-opacity=\"%.2f\"/>\n",
				lx, ly - unit * 1.9, swatchW, unit * 2.2, color, kAreaFillOpacity);
			Label(label);
		};
		auto LineRow = [&](const char *color, const char *dash, double widthMul, const char *label)
		{
			std::fprintf(f, "<line x1=\"%.1f\" y1=\"%.1f\" x2=\"%.1f\" y2=\"%.1f\" stroke=\"%s\" stroke-width=\"%.2f\"",
				lx, ly - unit * 0.8, lx + swatchW, ly - unit * 0.8, color, unit * 0.5 * widthMul);
			if (dash)
				std::fprintf(f, " stroke-dasharray=\"%s\"", dash);
			std::fprintf(f, "/>\n");
			Label(label);
		};
		auto DotRow = [&](const char *color, const char *label)
		{
			std::fprintf(f, "<circle cx=\"%.1f\" cy=\"%.1f\" r=\"%.2f\" fill=\"%s\"/>\n",
				lx + swatchW * 0.5, ly - unit * 0.8, unit * 1.2, color);
			Label(label);
		};
		auto CrosshairRow = [&](const char *color, const char *label)
		{
			double cx = lx + swatchW * 0.5;
			double cy = ly - unit * 0.8;
			double r = unit * 1.0;
			std::fprintf(f, "<g stroke=\"%s\" stroke-width=\"%.2f\" fill=\"none\">"
				"<circle cx=\"%.1f\" cy=\"%.1f\" r=\"%.2f\"/>"
				"<line x1=\"%.1f\" y1=\"%.1f\" x2=\"%.1f\" y2=\"%.1f\"/>"
				"<line x1=\"%.1f\" y1=\"%.1f\" x2=\"%.1f\" y2=\"%.1f\"/></g>\n",
				color, unit * 0.3, cx, cy, r,
				cx - r * 1.6, cy, cx + r * 1.6, cy,
				cx, cy - r * 1.6, cx, cy + r * 1.6);
			Label(label);
		};

		std::fprintf(f, "<text x=\"%.1f\" y=\"%.1f\" font-size=\"%.2f\" fill=\"#ffffff\" font-weight=\"bold\">%s</text>\n",
			lx, ly - unit * 2.0, unit * 3.4, mapName.c_str());
		ly += lh * 0.6;

		SwatchRow(kAreaWalkable, "area");
		SwatchRow(kAreaCrouch, "area (crouch)");
		LineRow(kConnWalk, nullptr, 1.0, "walk");
		LineRow(kConnAirmove, "4,2", 1.3, "air move");
		LineRow(kGate, "3,2", 0.5, "funnel gate");
		LineRow(kGateJump, "3,2", 0.5, "funnel gate (air move)");
		LineRow(kRoute, nullptr, 1.8, "planned route");
		LineRow(kRouteAirmoveSim, nullptr, 1.8, "air move, launch re-simulated");
		LineRow(kRouteAirmoveStored, nullptr, 1.8, "air move, stored launch");
		LineRow(kRouteAirmoveMissed, nullptr, 1.8, "air move, re-simulation missed");
		LineRow(kTrace, nullptr, 1.2, "recorded trace");
		DotRow(kRouteStart, "route start");
		DotRow(kRouteGoal, "route goal");
		CrosshairRow(kTargetEntity, "info_target");
	}

	std::fprintf(f, "</svg>\n");
	std::fclose(f);
}

}

