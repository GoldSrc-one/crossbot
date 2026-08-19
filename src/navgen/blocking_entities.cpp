#include "blocking_entities.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>

#include "clipnode_tree.h"
#include "hull_defs.h"
#include "polytope.h"

namespace navgen
{

namespace
{

constexpr double kOverlapEpsilon = 0.5;
constexpr double kMinPieceVolume = 1.0;

constexpr int kSfBreakTriggerOnly = 1;
constexpr int kSfDoorStartOpen = 1;
constexpr int kSfDoorPassable = 8;
constexpr int kSfDoorUseOnly = 256;
constexpr int kSfWallStartOff = 1;
constexpr int kSfConveyorNotSolid = 2;

constexpr double kDefaultDoorSpeed = 100.0;
constexpr double kDefaultRotateDistance = 90.0;
constexpr double kDefaultBreakableHealth = 1.0;
constexpr double kBreakSecondsPerHealth = 0.1;

bool IsBlockingEntity(const Entity &ent)
{
	std::string classname = ent.Get("classname");
	if (classname == "func_wall" || classname == "func_healthcharger" || classname == "func_recharge")
		return true;

	int spawnflags = ParseInt(ent.Get("spawnflags"), 0);
	if (classname == "func_wall_toggle")
		return (spawnflags & kSfWallStartOff) == 0;
	if (classname == "func_conveyor")
		return (spawnflags & kSfConveyorNotSolid) == 0;

	return false;
}

bool IsDoorClassname(const std::string &classname)
{
	return classname == "func_door" || classname == "func_door_rotating";
}

bool IsPassableClassname(const std::string &classname)
{
	return IsDoorClassname(classname) || classname == "func_breakable";
}

void PolytopeBounds(const Polytope &poly, Vec3 &mins, Vec3 &maxs)
{
	mins = Vec3(1e30, 1e30, 1e30);
	maxs = Vec3(-1e30, -1e30, -1e30);
	for (const Face &f : poly.faces)
	{
		for (const Vec3 &v : f.winding)
		{
			mins.x = std::min(mins.x, v.x);
			mins.y = std::min(mins.y, v.y);
			mins.z = std::min(mins.z, v.z);
			maxs.x = std::max(maxs.x, v.x);
			maxs.y = std::max(maxs.y, v.y);
			maxs.z = std::max(maxs.z, v.z);
		}
	}
}

bool BoundsOverlap(const Vec3 &aMins, const Vec3 &aMaxs, const Vec3 &bMins, const Vec3 &bMaxs, double epsilon)
{
	return aMins.x <= bMaxs.x + epsilon && aMaxs.x >= bMins.x - epsilon &&
		aMins.y <= bMaxs.y + epsilon && aMaxs.y >= bMins.y - epsilon &&
		aMins.z <= bMaxs.z + epsilon && aMaxs.z >= bMins.z - epsilon;
}

uint8_t ActivatorKindFor(const std::string &classname)
{
	if (classname == "func_button" || classname == "func_rot_button")
		return crossbot_nav::ACT_BUTTON;
	if (classname == "trigger_multiple" || classname == "trigger_once")
		return crossbot_nav::ACT_TRIGGER;
	return crossbot_nav::ACT_NONE;
}

bool TargetReaches(const std::vector<Entity> &entities, const std::string &target,
	const std::string &targetname)
{
	if (target.empty())
		return false;
	if (target == targetname)
		return true;

	for (const Entity &mid : entities)
	{
		std::string midClass = mid.Get("classname");
		if (mid.Get("targetname") != target)
			continue;

		if (midClass == "trigger_relay")
		{
			if (mid.Get("target") == targetname)
				return true;
		}
		else if (midClass == "multi_manager")
		{
			if (mid.Find(targetname) != nullptr)
				return true;
		}
	}
	return false;
}

}

double ParseDouble(const std::string &s, double def)
{
	if (s.empty())
		return def;
	char *end = nullptr;
	double v = std::strtod(s.c_str(), &end);
	return (end == s.c_str()) ? def : v;
}

int ParseInt(const std::string &s, int def)
{
	if (s.empty())
		return def;
	char *end = nullptr;
	long v = std::strtol(s.c_str(), &end, 10);
	return (end == s.c_str()) ? def : static_cast<int>(v);
}

Vec3 MoveDirFromAngles(const Vec3 &angles)
{
	if (angles.x == 0.0 && angles.z == 0.0)
	{
		if (angles.y == -1.0)
			return Vec3(0, 0, 1);
		if (angles.y == -2.0)
			return Vec3(0, 0, -1);
	}

	double pitch = angles.x * 3.14159265358979323846 / 180.0;
	double yaw = angles.y * 3.14159265358979323846 / 180.0;
	return Vec3(std::cos(yaw) * std::cos(pitch), std::sin(yaw) * std::cos(pitch), -std::sin(pitch));
}

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

Vec3 EntityAngles(const Entity &ent)
{
	if (const std::string *angles = ent.Find("angles"))
		return ParseEntityOrigin(*angles);
	if (const std::string *angle = ent.Find("angle"))
		return Vec3(0.0, ParseDouble(*angle, 0.0), 0.0);
	return Vec3(0.0, 0.0, 0.0);
}

Vec3 ParseEntityOrigin(const std::string &origin)
{
	double x = 0, y = 0, z = 0;
	std::sscanf(origin.c_str(), "%lf %lf %lf", &x, &y, &z);
	return Vec3(x, y, z);
}

Vec3 ModelWorldCenter(const dmodel_t &model, const Vec3 &origin)
{
	return Vec3(origin.x + (model.mins[0] + model.maxs[0]) * 0.5,
		origin.y + (model.mins[1] + model.maxs[1]) * 0.5,
		origin.z + (model.mins[2] + model.maxs[2]) * 0.5);
}

double DoorSpeed(const Entity &ent)
{
	double speed = ParseDouble(ent.Get("speed"), 0.0);
	return speed <= 0.0 ? kDefaultDoorSpeed : speed;
}

double DoorTravelDistance(const Entity &ent, const dmodel_t &model)
{
	Vec3 movedir = MoveDirFromAngles(EntityAngles(ent));
	Vec3 size(model.maxs[0] - model.mins[0], model.maxs[1] - model.mins[1], model.maxs[2] - model.mins[2]);

	double travel = std::fabs(movedir.x * (size.x - 2.0)) + std::fabs(movedir.y * (size.y - 2.0)) +
		std::fabs(movedir.z * (size.z - 2.0)) - ParseDouble(ent.Get("lip"), 0.0);

	return travel < 0.0 ? 0.0 : travel;
}

void CollectActivatorsFor(const BspFile &bsp, const std::vector<Entity> &entities,
	const std::string &targetname, std::vector<ActivatorVolume> &out)
{
	for (size_t i = 0; i < entities.size(); ++i)
	{
		const Entity &ent = entities[i];

		uint8_t kind = ActivatorKindFor(ent.Get("classname"));
		if (kind == crossbot_nav::ACT_NONE)
			continue;

		if (!TargetReaches(entities, ent.Get("target"), targetname))
			continue;

		int modelIndex = ParseBrushModelIndex(ent.Get("model"));
		if (modelIndex < 0 || static_cast<size_t>(modelIndex) >= bsp.Models().size())
			continue;

		const dmodel_t &model = bsp.Models()[static_cast<size_t>(modelIndex)];
		Vec3 origin = ParseEntityOrigin(ent.Get("origin"));

		ActivatorVolume act;
		act.kind = kind;
		act.mins = Vec3(model.mins[0], model.mins[1], model.mins[2]) + origin;
		act.maxs = Vec3(model.maxs[0], model.maxs[1], model.maxs[2]) + origin;
		act.center = ModelWorldCenter(model, origin);
		act.entityIndex = static_cast<int>(i);
		out.push_back(act);
	}
}

namespace
{

double DoorOpenSeconds(const Entity &ent, const dmodel_t &model)
{
	double speed = DoorSpeed(ent);

	if (ent.Get("classname") == "func_door_rotating")
	{
		double distance = ParseDouble(ent.Get("distance"), 0.0);
		if (distance <= 0.0)
			distance = kDefaultRotateDistance;
		return distance / speed;
	}

	return DoorTravelDistance(ent, model) / speed;
}

}

std::vector<BlockingVolume> ExtractBlockingVolumes(const BspFile &bsp, const std::vector<Entity> &entities,
	int hullIndex)
{
	Vec3 hullMins(kHumanHullMinX, kHumanHullMinY, kHumanHullMinZ);
	Vec3 hullMaxs(kHumanHullMaxX, kHumanHullMaxY, kHumanHullMaxZ);
	if (hullIndex == kHullDuck)
	{
		hullMins = Vec3(kDuckClipHullMinX, kDuckClipHullMinY, kDuckClipHullMinZ);
		hullMaxs = Vec3(kDuckClipHullMaxX, kDuckClipHullMaxY, kDuckClipHullMaxZ);
	}

	std::vector<BlockingVolume> result;

	for (const Entity &ent : entities)
	{
		if (!IsBlockingEntity(ent))
			continue;

		int modelIndex = ParseBrushModelIndex(ent.Get("model"));
		if (modelIndex < 0 || static_cast<size_t>(modelIndex) >= bsp.Models().size())
			continue;

		const dmodel_t &model = bsp.Models()[static_cast<size_t>(modelIndex)];
		if (model.headnode[hullIndex] < 0)
			continue;

		BlockingVolume vol;
		vol.headnode = model.headnode[hullIndex];
		vol.origin = ParseEntityOrigin(ent.Get("origin"));
		vol.mins = Vec3(model.mins[0] + hullMins.x, model.mins[1] + hullMins.y,
			model.mins[2] + hullMins.z) + vol.origin;
		vol.maxs = Vec3(model.maxs[0] + hullMaxs.x, model.maxs[1] + hullMaxs.y,
			model.maxs[2] + hullMaxs.z) + vol.origin;
		result.push_back(vol);
	}

	return result;
}

std::vector<BlockingVolume> ExtractLadderVolumes(const BspFile &bsp, const std::vector<Entity> &entities,
	int hullIndex)
{
	Vec3 hullMins(kHumanHullMinX, kHumanHullMinY, kHumanHullMinZ);
	Vec3 hullMaxs(kHumanHullMaxX, kHumanHullMaxY, kHumanHullMaxZ);
	if (hullIndex == kHullDuck)
	{
		hullMins = Vec3(kDuckClipHullMinX, kDuckClipHullMinY, kDuckClipHullMinZ);
		hullMaxs = Vec3(kDuckClipHullMaxX, kDuckClipHullMaxY, kDuckClipHullMaxZ);
	}

	std::vector<BlockingVolume> result;

	for (const Entity &ent : entities)
	{
		if (ent.Get("classname") != "func_ladder")
			continue;

		int modelIndex = ParseBrushModelIndex(ent.Get("model"));
		if (modelIndex < 0 || static_cast<size_t>(modelIndex) >= bsp.Models().size())
			continue;

		const dmodel_t &model = bsp.Models()[static_cast<size_t>(modelIndex)];
		if (model.headnode[hullIndex] < 0)
			continue;

		BlockingVolume vol;
		vol.headnode = model.headnode[hullIndex];
		vol.origin = ParseEntityOrigin(ent.Get("origin"));
		vol.mins = Vec3(model.mins[0] + hullMins.x, model.mins[1] + hullMins.y,
			model.mins[2] + hullMins.z) + vol.origin;
		vol.maxs = Vec3(model.maxs[0] + hullMaxs.x, model.maxs[1] + hullMaxs.y,
			model.maxs[2] + hullMaxs.z) + vol.origin;
		result.push_back(vol);
	}

	return result;
}

PassableSet ExtractPassableVolumes(const BspFile &bsp, const std::vector<Entity> &entities, int hullIndex,
	const std::vector<int> &skipEntities)
{
	Vec3 hullMins(kHumanHullMinX, kHumanHullMinY, kHumanHullMinZ);
	Vec3 hullMaxs(kHumanHullMaxX, kHumanHullMaxY, kHumanHullMaxZ);
	if (hullIndex == kHullDuck)
	{
		hullMins = Vec3(kDuckClipHullMinX, kDuckClipHullMinY, kDuckClipHullMinZ);
		hullMaxs = Vec3(kDuckClipHullMaxX, kDuckClipHullMaxY, kDuckClipHullMaxZ);
	}

	PassableSet result;

	for (size_t entIndex = 0; entIndex < entities.size(); ++entIndex)
	{
		const Entity &ent = entities[entIndex];

		std::string classname = ent.Get("classname");
		if (!IsPassableClassname(classname))
			continue;

		if (std::find(skipEntities.begin(), skipEntities.end(), static_cast<int>(entIndex)) !=
			skipEntities.end())
			continue;

		int modelIndex = ParseBrushModelIndex(ent.Get("model"));
		if (modelIndex < 0 || static_cast<size_t>(modelIndex) >= bsp.Models().size())
			continue;

		const dmodel_t &model = bsp.Models()[static_cast<size_t>(modelIndex)];
		if (model.headnode[hullIndex] < 0)
			continue;

		bool isDoor = IsDoorClassname(classname);
		int spawnflags = ParseInt(ent.Get("spawnflags"), 0);

		if (isDoor && ((spawnflags & kSfDoorStartOpen) || (spawnflags & kSfDoorPassable)))
			continue;

		BlockingVolume vol;
		vol.headnode = model.headnode[hullIndex];
		vol.origin = ParseEntityOrigin(ent.Get("origin"));
		vol.mins = Vec3(model.mins[0] + hullMins.x, model.mins[1] + hullMins.y,
			model.mins[2] + hullMins.z) + vol.origin;
		vol.maxs = Vec3(model.maxs[0] + hullMaxs.x, model.maxs[1] + hullMaxs.y,
			model.maxs[2] + hullMaxs.z) + vol.origin;

		PassableVolume pass;
		pass.volume = vol;
		pass.entityIndex = static_cast<int>(entIndex);
		pass.areaFlag = isDoor ? crossbot_nav::NAV_AREA_DOOR : crossbot_nav::NAV_AREA_BREAKABLE;
		pass.openSeconds = isDoor ? DoorOpenSeconds(ent, model)
			: std::max(ParseDouble(ent.Get("health"), kDefaultBreakableHealth), kDefaultBreakableHealth) *
				kBreakSecondsPerHealth;

		std::string targetname = ent.Get("targetname");
		bool triggerOnly = isDoor ? ((spawnflags & kSfDoorUseOnly) != 0)
			: ((spawnflags & kSfBreakTriggerOnly) != 0);

		if (targetname.empty() && !triggerOnly)
		{
			pass.activation = crossbot_nav::ACT_NONE;
			result.passable.push_back(pass);
			continue;
		}

		if (targetname.empty() && isDoor)
		{
			ActivatorVolume self;
			self.kind = crossbot_nav::ACT_BUTTON;
			self.mins = Vec3(model.mins[0], model.mins[1], model.mins[2]) + vol.origin;
			self.maxs = Vec3(model.maxs[0], model.maxs[1], model.maxs[2]) + vol.origin;
			self.center = ModelWorldCenter(model, vol.origin);
			self.entityIndex = static_cast<int>(entIndex);

			pass.activation = crossbot_nav::ACT_BUTTON;
			pass.activators.push_back(self);
			result.passable.push_back(pass);
			continue;
		}

		CollectActivatorsFor(bsp, entities, targetname, pass.activators);

		if (pass.activators.empty())
		{
			result.impassable.push_back(vol);
			continue;
		}

		pass.activation = pass.activators.front().kind;
		for (const ActivatorVolume &act : pass.activators)
		{
			if (act.kind == crossbot_nav::ACT_BUTTON)
			{
				pass.activation = crossbot_nav::ACT_BUTTON;
				break;
			}
		}

		result.passable.push_back(pass);
	}

	return result;
}

bool OverlapsAnyBlockingVolume(const ExtractedLeaf &leaf, const std::vector<BlockingVolume> &blockers)
{
	Vec3 mins, maxs;
	PolytopeBounds(leaf.polytope, mins, maxs);

	for (const BlockingVolume &blocker : blockers)
		if (BoundsOverlap(mins, maxs, blocker.mins, blocker.maxs, kOverlapEpsilon))
			return true;

	return false;
}

bool OverlapsBlockingVolume(const Polytope &poly, const BlockingVolume &blocker)
{
	Vec3 mins, maxs;
	PolytopeBounds(poly, mins, maxs);
	return BoundsOverlap(mins, maxs, blocker.mins, blocker.maxs, kOverlapEpsilon);
}

void SplitByPassableVolume(const BspFile &bsp, const Polytope &poly, const BlockingVolume &blocker,
	double mapRadius, std::vector<Polytope> &inside, std::vector<Polytope> &outside, double epsilon)
{
	if (!OverlapsBlockingVolume(poly, blocker))
	{
		outside.push_back(poly);
		return;
	}

	Polytope local = TranslatePolytope(poly, blocker.origin * -1.0);

	for (const Polytope &piece : CarvePolytopeAgainstTree(bsp, blocker.headnode, local, CONTENTS_SOLID,
			mapRadius, kMinPieceVolume, epsilon))
		outside.push_back(TranslatePolytope(piece, blocker.origin));

	for (const Polytope &piece : CarvePolytopeAgainstTree(bsp, blocker.headnode, local, CONTENTS_EMPTY,
			mapRadius, kMinPieceVolume, epsilon))
		inside.push_back(TranslatePolytope(piece, blocker.origin));
}

std::vector<Polytope> SubtractBoxVolumes(const Polytope &leafPolytope,
	const std::vector<BlockingVolume> &boxes, double mapRadius, double epsilon)
{
	std::vector<Polytope> pieces{leafPolytope};

	for (const BlockingVolume &box : boxes)
	{
		const Plane slabs[6] = {
			{Vec3(1.0, 0.0, 0.0), box.mins.x}, {Vec3(-1.0, 0.0, 0.0), -box.maxs.x},
			{Vec3(0.0, 1.0, 0.0), box.mins.y}, {Vec3(0.0, -1.0, 0.0), -box.maxs.y},
			{Vec3(0.0, 0.0, 1.0), box.mins.z}, {Vec3(0.0, 0.0, -1.0), -box.maxs.z}};

		std::vector<Polytope> next;
		for (const Polytope &piece : pieces)
		{
			Vec3 mins, maxs;
			PolytopeBounds(piece, mins, maxs);
			if (!BoundsOverlap(mins, maxs, box.mins, box.maxs, -kOverlapEpsilon))
			{
				next.push_back(piece);
				continue;
			}

			Polytope remain = piece;
			for (const Plane &slab : slabs)
			{
				Polytope outside = ClipPolytope(remain, slab, false, mapRadius, epsilon);
				if (outside.Volume() > kMinPieceVolume)
					next.push_back(std::move(outside));

				remain = ClipPolytope(remain, slab, true, mapRadius, epsilon);
				if (remain.Empty())
					break;
			}
		}
		pieces = std::move(next);
	}

	return pieces;
}

std::vector<Polytope> SubtractBlockingVolumes(const BspFile &bsp, const Polytope &leafPolytope,
	const std::vector<BlockingVolume> &blockers, double mapRadius, double epsilon)
{
	std::vector<Polytope> pieces{leafPolytope};

	for (const BlockingVolume &blocker : blockers)
	{
		std::vector<Polytope> next;
		for (Polytope &piece : pieces)
		{
			Vec3 mins, maxs;
			PolytopeBounds(piece, mins, maxs);
			if (!BoundsOverlap(mins, maxs, blocker.mins, blocker.maxs, kOverlapEpsilon))
			{
				next.push_back(std::move(piece));
				continue;
			}

			Polytope local = TranslatePolytope(piece, blocker.origin * -1.0);
			std::vector<Polytope> carved = CarvePolytopeAgainstTree(bsp, blocker.headnode, local,
				CONTENTS_SOLID, mapRadius, kMinPieceVolume, epsilon);
			for (const Polytope &c : carved)
				next.push_back(TranslatePolytope(c, blocker.origin));
		}
		pieces = std::move(next);
	}

	return pieces;
}

}

