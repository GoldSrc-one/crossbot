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

std::vector<Pt2> AreaFootprint(const Area &area)
{
	Winding winding;
	if (area.alwaysSubmerged)
	{
		std::vector<Vec3> pts;
		for (const Face &fc : area.waterCeiling)
			for (const Vec3 &v : fc.winding)
				pts.push_back(v);
		winding = ConvexHullXY(pts);
	}
	else
		winding = WalkableFootprint(area.polytope);

	std::vector<Pt2> poly;
	for (const Vec3 &v : winding)
		poly.push_back({v.x, -v.y});
	return poly;
}

constexpr const char *kBackground = "#12141a";
constexpr const char *kAreaWalkable = "#8d9298";
constexpr const char *kAreaCrouch = "#93b3cf";
constexpr const char *kAreaDoor = "#b99a5b";
constexpr const char *kAreaBreakable = "#a8705f";
constexpr const char *kAreaElevator = "#6fa8a0";
constexpr const char *kAreaWater = "#1f6f8b";
constexpr const char *kAreaFloodable = "#4d7f8f";
constexpr const char *kConnWalk = "#e8eef4";
constexpr const char *kConnAirmove = "#e74c3c";
constexpr const char *kConnTeleport = "#27ae60";
constexpr const char *kConnLadder = "#f1c40f";
constexpr const char *kConnElevator = "#4aa3df";
constexpr const char *kConnSwim = "#00d0ff";
constexpr const char *kGate = "#8e6fc4";
constexpr const char *kGateJump = "#e74c3c";
constexpr const char *kRoute = "#f39c12";
constexpr const char *kWaterSurface = "#57e0ff";

constexpr const char *kRouteAirmoveSim = "#2ecc71";
constexpr const char *kRouteAirmoveStored = "#ff3b30";
constexpr const char *kRouteAirmoveMissed = "#f5d020";
constexpr const char *kRouteLadder = "#e91e8c";
constexpr const char *kRouteStart = "#27ae60";
constexpr const char *kRouteGoal = "#e74c3c";

constexpr const char *kTrace = "#3fd0d4";
constexpr const char *kTargetEntity = "#c86bd8";
constexpr const char *kLabelText = "#e8eef4";

constexpr double kAreaFillOpacity = 0.7;

constexpr const char *kVantageArea = "#5a6470";
constexpr const char *kWatchRank[4] = {"#ff7043", "#ffd54f", "#4fc3f7", "#ab7ddb"};
constexpr const char *kCampRing = "#ffffff";

constexpr double kVantageAreaOpacity = 0.35;

void ExposureColor(double t, char *out, size_t outSize)
{
	static const double kStops[3][3] = {
		{0.169, 0.227, 0.404},
		{0.941, 0.831, 0.227},
		{0.878, 0.188, 0.125},
	};

	if (t < 0.0)
		t = 0.0;
	if (t > 1.0)
		t = 1.0;

	int lo = (t < 0.5) ? 0 : 1;
	double f = (t < 0.5) ? (t / 0.5) : ((t - 0.5) / 0.5);

	int r = static_cast<int>((kStops[lo][0] + (kStops[lo + 1][0] - kStops[lo][0]) * f) * 255.0);
	int g = static_cast<int>((kStops[lo][1] + (kStops[lo + 1][1] - kStops[lo][1]) * f) * 255.0);
	int b = static_cast<int>((kStops[lo][2] + (kStops[lo + 1][2] - kStops[lo][2]) * f) * 255.0);

	std::snprintf(out, outSize, "#%02x%02x%02x", r, g, b);
}

double AreaFloorZ(const Area &area)
{
	if (area.alwaysSubmerged && !area.waterCeiling.empty())
	{
		double highest = -1e30;
		for (const Face &fc : area.waterCeiling)
			for (const Vec3 &v : fc.winding)
				highest = std::max(highest, v.z);
		return highest;
	}

	double lowest = 1e30;
	for (const Face &f : area.polytope.faces)
	{
		if (!IsFloorFace(f))
			continue;
		for (const Vec3 &v : f.winding)
			lowest = std::min(lowest, v.z);
	}
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

	case CONN_LADDER:
		return {kConnLadder, "2,3", 1.6};

	case CONN_TELEPORT:
		return {kConnTeleport, nullptr, 1.6};

	case CONN_ELEVATOR:
		return {kConnElevator, "6,3", 1.8};

	case CONN_SWIM:
		return {kConnSwim, "1,3", 1.4};
	case CONN_WALK:
	default:
		return {kConnWalk, nullptr, 1.0};
	}
}

const char *FillForArea(const Area &area)
{
	if (area.alwaysSubmerged)
		return kAreaWater;
	if (area.areaFlags & crossbot_nav::NAV_AREA_WATER)
		return kAreaFloodable;
	if (area.areaFlags & crossbot_nav::NAV_AREA_DOOR)
		return kAreaDoor;
	if (area.areaFlags & crossbot_nav::NAV_AREA_BREAKABLE)
		return kAreaBreakable;
	if (area.areaFlags & crossbot_nav::NAV_AREA_ELEVATOR)
		return kAreaElevator;
	if (area.areaFlags & crossbot_nav::NAV_AREA_CROUCH)
		return kAreaCrouch;
	return kAreaWalkable;
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
	const std::vector<FailedAirmove> &failedAirmoves, const std::vector<Vec3> &tracePoints,
	const std::vector<std::vector<Vec3>> &runupTraces, const std::vector<float> &pathSpeeds,
	const std::vector<LiquidVolume> &waterVolumes)
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
		std::vector<Pt2> poly = AreaFootprint(areas[i]);
		if (poly.size() < 3)
			continue;

		std::fprintf(f, "<polygon fill=\"%s\" points=\"", FillForArea(areas[i]));
		for (const Pt2 &p : poly)
			std::fprintf(f, "%.1f,%.1f ", p.x, p.y);
		std::fprintf(f, "\"/>\n");
	}
	std::fprintf(f, "</g>\n");

	if (!waterVolumes.empty())
	{
		std::fprintf(f, "<g fill=\"%s\" fill-opacity=\"0.10\" stroke=\"%s\" stroke-opacity=\"0.80\" "
			"stroke-width=\"%.2f\" stroke-dasharray=\"%.1f %.1f\">\n",
			kWaterSurface, kWaterSurface, unit * 0.3, unit * 1.2, unit * 0.8);

		for (const LiquidVolume &vol : waterVolumes)
		{
			for (const Polytope &piece : vol.pieces)
			{
				std::vector<Vec3> pts;
				for (const Face &fc : piece.faces)
					for (const Vec3 &v : fc.winding)
						pts.push_back(v);

				Winding hull = ConvexHullXY(pts);
				if (hull.size() < 3)
					continue;

				std::fprintf(f, "<polygon points=\"");
				for (const Vec3 &v : hull)
					std::fprintf(f, "%.1f,%.1f ", v.x, -v.y);
				std::fprintf(f, "\"><title>water surface %.0f", vol.LowSurfaceZ());
				if (vol.moving)
					std::fprintf(f, " rising to %.0f", vol.HighSurfaceZ());
				std::fprintf(f, "</title></polygon>\n");
			}
		}

		std::fprintf(f, "</g>\n");
	}

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

		if (c.connType == crossbot_nav::CONN_AIRMOVE && std::fabs(c.gravity - 1.0) > 0.005)
		{
			std::fprintf(f, " stroke-opacity=\"0.55\"><title>air move at gravity x%.2f</title></line>\n",
				c.gravity);
			continue;
		}

		std::fprintf(f, "/>\n");
	}
	std::fprintf(f, "</g>\n");

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

		if (pathSpeeds.size() >= pathPoints.size())
		{
			float fastest = 1.f;
			for (float v : pathSpeeds)
				fastest = std::max(fastest, v);

			for (size_t i = 0; i + 1 < pathPoints.size(); ++i)
			{
				float frac = std::min(1.f, pathSpeeds[i] / fastest);
				if (frac > 0.9f)
					continue;

				int red = 255;
				int green = static_cast<int>(40.f + 150.f * frac);
				int blue = static_cast<int>(20.f + 40.f * frac);
				const Vec3 &a = pathPoints[i];
				const Vec3 &b = pathPoints[i + 1];
				std::fprintf(f, "<line x1=\"%.1f\" y1=\"%.1f\" x2=\"%.1f\" y2=\"%.1f\" "
					"stroke=\"#%02x%02x%02x\" stroke-width=\"%.2f\" stroke-linecap=\"round\"/>\n",
					a.x, -a.y, b.x, -b.y, red, green, blue, unit * 1.1);
			}
		}

		for (size_t i = 0; i + 1 < pathPoints.size() && i < pathSegmentKinds.size(); ++i)
		{
			if (pathSegmentKinds[i] == PATH_SEG_WALK)
				continue;

			const char *colour = (pathSegmentKinds[i] == PATH_SEG_LADDER) ? kRouteLadder
				: (pathSegmentKinds[i] == PATH_SEG_AIRMOVE_SIMULATED) ? kRouteAirmoveSim
				: kRouteAirmoveStored;
			const Vec3 &a = pathPoints[i];
			const Vec3 &b = pathPoints[i + 1];
			std::fprintf(f, "<line x1=\"%.1f\" y1=\"%.1f\" x2=\"%.1f\" y2=\"%.1f\" stroke=\"%s\" "
				"stroke-width=\"%.2f\" stroke-linecap=\"round\"/>\n",
				a.x, -a.y, b.x, -b.y, colour, unit * 1.3);
		}

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

		constexpr int kLegendRows = 18;
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
		SwatchRow(kAreaDoor, "area (door)");
		SwatchRow(kAreaBreakable, "area (breakable)");
		SwatchRow(kAreaElevator, "area (elevator stop)");
		SwatchRow(kAreaWater, "area (under water)");
		SwatchRow(kAreaFloodable, "area (floods)");
		LineRow(kConnWalk, nullptr, 1.0, "walk");
		LineRow(kConnAirmove, "4,2", 1.3, "air move");
		LineRow(kConnLadder, "2,3", 1.6, "ladder");
		LineRow(kConnElevator, "6,3", 1.8, "elevator ride");
		LineRow(kConnSwim, "1,3", 1.4, "swim");
		LineRow(kWaterSurface, "3,2", 0.6, "water surface");
		LineRow(kGate, "3,2", 0.5, "funnel gate");
		LineRow(kGateJump, "3,2", 0.5, "funnel gate (air move)");
		LineRow(kRoute, nullptr, 1.8, "planned route");
		LineRow(kRouteAirmoveSim, nullptr, 1.8, "air move, launch re-simulated");
		LineRow(kRouteAirmoveStored, nullptr, 1.8, "air move, stored launch");
		LineRow(kRouteAirmoveMissed, nullptr, 1.8, "air move, re-simulation missed");
		LineRow(kRouteLadder, nullptr, 1.8, "ladder climb");
		LineRow(kTrace, nullptr, 1.2, "recorded trace");
		DotRow(kRouteStart, "route start");
		DotRow(kRouteGoal, "route goal");
		CrosshairRow(kTargetEntity, "info_target");
	}

	std::fprintf(f, "</svg>\n");
	std::fclose(f);
}

void WriteVantageSvg(const std::string &path, const std::string &mapName, const std::vector<Area> &areas,
	const std::vector<Vantage> &vantages, const VantageStats &stats)
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

	for (const Vantage &v : vantages)
		Consider(v.pos.x, v.pos.y);

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

	std::vector<size_t> drawOrder(areas.size());
	std::vector<double> floorZ(areas.size(), 0.0);
	for (size_t i = 0; i < areas.size(); ++i)
	{
		drawOrder[i] = i;
		floorZ[i] = AreaFloorZ(areas[i]);
	}
	std::stable_sort(drawOrder.begin(), drawOrder.end(),
		[&](size_t a, size_t b) { return floorZ[a] < floorZ[b]; });

	std::fprintf(f, "<g stroke=\"%s\" stroke-width=\"%.2f\" fill=\"%s\" fill-opacity=\"%.2f\">\n",
		kBackground, unit * 0.2, kVantageArea, kVantageAreaOpacity);
	for (size_t i : drawOrder)
	{
		std::vector<Pt2> poly = AreaFootprint(areas[i]);
		if (poly.size() < 3)
			continue;

		std::fprintf(f, "<polygon points=\"");
		for (const Pt2 &p : poly)
			std::fprintf(f, "%.1f,%.1f ", p.x, p.y);
		std::fprintf(f, "\"/>\n");
	}
	std::fprintf(f, "</g>\n");

	double maxExposure = std::max(1e-9, stats.maxExposure);
	double maxRadius = unit * 7.0;
	double minRadius = unit * 1.2;

	for (size_t vi = 0; vi < vantages.size(); ++vi)
	{
		const Vantage &v = vantages[vi];
		double cx = v.pos.x;
		double cy = -v.pos.y;

		std::fprintf(f, "<g>\n");
		std::fprintf(f, "<title>#%zu  pos %.0f %.0f %.0f\nexposure %.4f (%.0f%% of map max)\n"
			"focus 90/45/22.5/11.25: %.3f %.3f %.3f %.3f",
			vi, v.pos.x, v.pos.y, v.pos.z, v.exposure, 100.0 * v.exposure / maxExposure,
			v.focus[0], v.focus[1], v.focus[2], v.focus[3]);
		for (size_t w = 0; w < v.watch.size(); ++w)
		{
			std::fprintf(f, "\nwatch %zu: yaw %.1f pitch %.1f spread %.1f amount %.3f", w,
				v.watch[w].yaw, v.watch[w].pitch, v.watch[w].spread, v.watch[w].amount);
		}
		std::fprintf(f, "</title>\n");

		for (size_t rank = v.watch.size(); rank-- > 0;)
		{
			const WatchDir &wd = v.watch[rank];
			double half = std::max(4.0, std::min(80.0, wd.spread));
			double radius = std::max(minRadius, maxRadius * wd.amount);
			double a0 = (wd.yaw - half) * 3.14159265358979323846 / 180.0;
			double a1 = (wd.yaw + half) * 3.14159265358979323846 / 180.0;

			double x0 = v.pos.x + std::cos(a0) * radius;
			double y0 = -(v.pos.y + std::sin(a0) * radius);
			double x1 = v.pos.x + std::cos(a1) * radius;
			double y1 = -(v.pos.y + std::sin(a1) * radius);

			int largeArc = (half * 2.0 > 180.0) ? 1 : 0;

			std::fprintf(f,
				"<path d=\"M%.1f,%.1f L%.1f,%.1f A%.1f,%.1f 0 %d,1 %.1f,%.1f Z\" fill=\"%s\" "
				"fill-opacity=\"%.2f\"/>\n",
				cx, cy, x0, y0, radius, radius, largeArc, x1, y1,
				kWatchRank[std::min<size_t>(rank, 3)], 0.25 + 0.45 * wd.amount);
		}

		char color[16];
		ExposureColor(v.exposure / maxExposure, color, sizeof(color));
		double ring = unit * 1.1 * v.focus[3];
		std::fprintf(f,
			"<circle cx=\"%.1f\" cy=\"%.1f\" r=\"%.2f\" fill=\"%s\" stroke=\"%s\" stroke-width=\"%.2f\" "
			"stroke-opacity=\"0.9\"/>\n",
			cx, cy, unit * 0.7, color, kCampRing, ring);

		std::fprintf(f, "</g>\n");
	}

	{
		double lx = minX - pad + unit * 1.5;
		double ly = minY - pad + unit * 4.0;
		double lh = unit * 3.4;
		double panelW = unit * 34.0;

		constexpr int kLegendRows = 9;
		double panelH = lh * (kLegendRows + 2.6);
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
			std::fprintf(f, "<rect x=\"%.1f\" y=\"%.1f\" width=\"%.1f\" height=\"%.1f\" fill=\"%s\" fill-opacity=\"0.7\"/>\n",
				lx, ly - unit * 1.9, swatchW, unit * 2.2, color);
			Label(label);
		};

		std::fprintf(f, "<text x=\"%.1f\" y=\"%.1f\" font-size=\"%.2f\" fill=\"#ffffff\" font-weight=\"bold\">%s vantage field</text>\n",
			lx, ly - unit * 2.0, unit * 3.4, mapName.c_str());
		ly += lh * 0.6;

		std::fprintf(f, "<defs><linearGradient id=\"expgrad\"><stop offset=\"0%%\" stop-color=\"#2b3a67\"/>"
			"<stop offset=\"50%%\" stop-color=\"#f0d43a\"/><stop offset=\"100%%\" stop-color=\"#e03020\"/>"
			"</linearGradient></defs>\n");
		std::fprintf(f, "<rect x=\"%.1f\" y=\"%.1f\" width=\"%.1f\" height=\"%.1f\" fill=\"url(#expgrad)\"/>\n",
			lx, ly - unit * 1.9, unit * 12.0, unit * 2.2);
		std::fprintf(f, "<text x=\"%.1f\" y=\"%.1f\" font-size=\"%.2f\" fill=\"%s\">dot: exposure 0 to %.4f</text>\n",
			lx + unit * 13.5, ly, fontSize, kLabelText, stats.maxExposure);
		ly += lh;

		std::fprintf(f, "<circle cx=\"%.1f\" cy=\"%.1f\" r=\"%.2f\" fill=\"#888888\" stroke=\"%s\" stroke-width=\"%.2f\"/>\n",
			lx + swatchW * 0.5, ly - unit * 0.8, unit * 0.7, kCampRing, unit * 1.1);
		Label("ring thickness: focus at 11.25 FOV (camp)");

		SwatchRow(kWatchRank[0], "watch dir 1 (main)");
		SwatchRow(kWatchRank[1], "watch dir 2");
		SwatchRow(kWatchRank[2], "watch dir 3");
		SwatchRow(kWatchRank[3], "watch dir 4");
		Label("wedge radius: amount, angle: spread");

		char line[160];
		std::snprintf(line, sizeof(line), "%zu sampled -&gt; %zu kept (budget %zu)", stats.sampled,
			stats.kept, stats.budget);
		Label(line);
		std::snprintf(line, sizeof(line), "%zu/%zu pairs visible (%zu passed PVS)", stats.visiblePairs,
			stats.totalPairs, stats.pvsPairs);
		Label(line);
	}

	std::fprintf(f, "</svg>\n");
	std::fclose(f);
}

}

