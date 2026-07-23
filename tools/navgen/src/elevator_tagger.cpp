#include "elevator_tagger.h"

#include <cmath>
#include <cstdlib>
#include <limits>
#include <unordered_map>

namespace navgen
{

namespace
{

using namespace crossbot_nav;

int ParseBrushModelIndex(const std::string &model)
{
	if (model.size() < 2 || model[0] != '*')
		return -1;
	char *end = nullptr;
	long idx = std::strtol(model.c_str() + 1, &end, 10);
	if (end == model.c_str() + 1 || *end != '\0' || idx <= 0)
		return -1;
	return static_cast<int>(idx);
}

// Quake/GoldSrc's well-known door movedir convention (dlls/doors.cpp's
// SetMovedir): angle -1 means straight up, -2 means straight down,
// anything else is a compass heading in the XY plane. Vertical lifts (the
// only case this module handles) always use -1 or -2.
bool VerticalMoveDir(double angle, Vec3 &outDir)
{
	if (angle == -1.0)
	{
		outDir = Vec3(0, 0, 1);
		return true;
	}
	if (angle == -2.0)
	{
		outDir = Vec3(0, 0, -1);
		return true;
	}
	return false;
}

// Same point-in-polytope-with-nearest-centroid-fallback pattern
// reachability.cpp's FindSpawnAreas already uses for resolving an
// entity-derived world point to a navigable area -- duplicated here rather
// than shared, matching this codebase's existing convention of small
// per-file utility duplication (see e.g. ParseBrushModelIndex above).
int FindAreaForWorldPoint(const Vec3 &point, const std::vector<Area> &areas)
{
	for (const Area &area : areas)
	{
		bool inside = true;
		for (const Face &f : area.polytope.faces)
		{
			if (point.Dot(f.plane.normal) - f.plane.dist < -1.0)
			{
				inside = false;
				break;
			}
		}
		if (inside)
			return area.id;
	}

	int best = -1;
	double bestDist = std::numeric_limits<double>::max();
	for (const Area &area : areas)
	{
		double d = (area.polytope.Centroid() - point).Length();
		if (d < bestDist)
		{
			bestDist = d;
			best = area.id;
		}
	}
	return best;
}

} // namespace

std::vector<MergedPortal> TagElevators(const BspFile &bsp, const std::vector<Entity> &entities,
	const std::vector<Area> &areas)
{
	std::vector<MergedPortal> result;

	// Map each targetname to the func_door entities that carry it, so a
	// target shared by more than one door (multi-door sequences) can be
	// detected and skipped rather than guessed at.
	std::unordered_map<std::string, std::vector<const Entity *>> doorsByTargetname;
	for (const Entity &ent : entities)
	{
		if (ent.Get("classname") != "func_door")
			continue;
		std::string targetname = ent.Get("targetname");
		if (!targetname.empty())
			doorsByTargetname[targetname].push_back(&ent);
	}

	for (const Entity &button : entities)
	{
		if (button.Get("classname") != "func_button")
			continue;
		std::string target = button.Get("target");
		if (target.empty())
			continue;

		auto it = doorsByTargetname.find(target);
		if (it == doorsByTargetname.end() || it->second.size() != 1)
			continue; // no matching door, or a multi-door target (not a simple lift)

		const Entity &door = *it->second[0];

		Vec3 movedir;
		double angle = std::atof(door.Get("angle", "0").c_str());
		if (!VerticalMoveDir(angle, movedir))
			continue; // not a vertical mover -- a sliding/rotating door, not a lift

		int doorModelIdx = ParseBrushModelIndex(door.Get("model"));
		int buttonModelIdx = ParseBrushModelIndex(button.Get("model"));
		if (doorModelIdx < 0 || static_cast<size_t>(doorModelIdx) >= bsp.Models().size() ||
			buttonModelIdx < 0 || static_cast<size_t>(buttonModelIdx) >= bsp.Models().size())
			continue;

		const dmodel_t &doorModel = bsp.Models()[static_cast<size_t>(doorModelIdx)];
		const dmodel_t &buttonModel = bsp.Models()[static_cast<size_t>(buttonModelIdx)];

		Vec3 mins(doorModel.mins[0], doorModel.mins[1], doorModel.mins[2]);
		Vec3 maxs(doorModel.maxs[0], doorModel.maxs[1], doorModel.maxs[2]);
		Vec3 size = maxs - mins;
		Vec3 position1 = (mins + maxs) * 0.5; // the door's compiled/at-rest position

		double lip = std::atof(door.Get("lip", "0").c_str());
		double travelDistance = std::fabs(movedir.Dot(size)) - lip;
		if (travelDistance <= 0.0)
			continue; // degenerate -- lip consumes the whole brush, nothing to ride

		Vec3 position2 = position1 + movedir * travelDistance;

		double speed = std::atof(door.Get("speed", "0").c_str());
		if (speed <= 0.0)
			speed = 100.0; // CBaseDoor's own default when unset
		double travelTime = travelDistance / speed;

		// Assign areaA/areaB by actual height, not by which position the
		// door's own compiled state happens to start at (mirrors
		// TagDropOffs's "areaA is always the higher/lower special side"
		// convention, applied consistently here too) -- this is what lets
		// path_funnel.cpp determine ride direction just by checking which
		// area it's arriving from.
		bool pos1IsBottom = position1.z <= position2.z;
		const Vec3 &bottomPos = pos1IsBottom ? position1 : position2;
		const Vec3 &topPos = pos1IsBottom ? position2 : position1;

		int bottomArea = FindAreaForWorldPoint(bottomPos, areas);
		int topArea = FindAreaForWorldPoint(topPos, areas);
		if (bottomArea < 0 || topArea < 0 || bottomArea == topArea)
			continue;

		Vec3 buttonMins(buttonModel.mins[0], buttonModel.mins[1], buttonModel.mins[2]);
		Vec3 buttonMaxs(buttonModel.maxs[0], buttonModel.maxs[1], buttonModel.maxs[2]);
		Vec3 buttonPos = (buttonMins + buttonMaxs) * 0.5;

		MergedPortal portal;
		portal.areaA = bottomArea;
		portal.areaB = topArea;
		portal.pointA = bottomPos;
		portal.pointB = topPos;
		portal.area = 0.0; // not a real shared-face overlap, same convention as ProbeJumpGaps' portals
		portal.edgeType = EDGE_ELEVATOR;
		portal.edgeFlags = EDGE_FLAG_NONE; // bidirectional -- auto-return lifts can be ridden either way
		portal.buttonPos = buttonPos;
		portal.travelTime = travelTime;
		result.push_back(portal);
	}

	return result;
}

} // namespace navgen
