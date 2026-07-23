#include "door_extractor.h"

#include <cstdio>
#include <cstdlib>

#include "clipnode_tree.h"
#include "hull_defs.h"
#include "leaf_extractor.h"

namespace navgen
{

namespace
{

// "model" keyvalues on brush entities are "*N" (N = index into
// bsp.Models()). Returns -1 if `model` isn't that form. Duplicated per this
// codebase's existing convention (blocking_entities.cpp/edge_tagger.cpp each
// keep their own copy too) rather than shared.
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

// A brush entity built around an "origin" brush (func_door_rotating,
// func_rotating, func_train, ...) gets its solid geometry re-expressed by
// hlcsg in *local* space relative to that origin brush's centroid, with the
// centroid itself written back into the compiled entity lump as an
// "origin" keyvalue the source .map never had (confirmed live on
// test9_door's func_door_rotating during this project's Milestone 2
// development). Returns (0,0,0) -- a safe no-op offset -- for a plain
// func_door with no origin brush at all.
Vec3 ParseEntityOrigin(const std::string &origin)
{
	double x = 0, y = 0, z = 0;
	std::sscanf(origin.c_str(), "%lf %lf %lf", &x, &y, &z);
	return Vec3(x, y, z);
}

// Resolves which func_button (if any) opens a targetname'd door: either
// directly (button.target == doorTargetname), or through one level of
// indirection via a trigger_relay (relay.targetname == button.target &&
// relay.target == doorTargetname) or a multi_manager (mm.targetname ==
// button.target && mm has a keyvalue named doorTargetname -- multi_manager's
// own keyvalues *are* its fire list, "<target> <delay>"). Only one level of
// indirection is resolved, not arbitrary chains -- covers both patterns
// confirmed live (test10_doorbutton's trigger_relay, test11_doorbutton2's
// multi_manager), and going further would be speculative without another
// real test case to validate against.
const Entity *ResolveButtonForDoor(const std::vector<Entity> &entities, const std::string &doorTargetname)
{
	for (const Entity &ent : entities)
	{
		if (ent.Get("classname") != "func_button")
			continue;

		std::string target = ent.Get("target");
		if (target.empty())
			continue;

		if (target == doorTargetname)
			return &ent;

		for (const Entity &mid : entities)
		{
			std::string midClass = mid.Get("classname");
			if (midClass == "trigger_relay")
			{
				if (mid.Get("targetname") == target && mid.Get("target") == doorTargetname)
					return &ent;
			}
			else if (midClass == "multi_manager")
			{
				if (mid.Get("targetname") == target && !mid.Get(doorTargetname).empty())
					return &ent;
			}
		}
	}
	return nullptr;
}

// Resolves a trigger_multiple that fires a targetname'd door directly by
// touch (trigger.target == doorTargetname) -- the other way (besides a
// func_button) HL lets a mapper gate a door that's had its own Touch nulled
// out by SetTouch(NULL). Unlike ResolveButtonForDoor, only direct targeting
// is resolved here, no relay/multi_manager indirection -- no real test case
// has needed it yet.
const Entity *ResolveTriggerForDoor(const std::vector<Entity> &entities, const std::string &doorTargetname)
{
	for (const Entity &ent : entities)
	{
		if (ent.Get("classname") != "trigger_multiple")
			continue;
		if (ent.Get("target") == doorTargetname)
			return &ent;
	}
	return nullptr;
}

Vec3 ModelWorldCenter(const dmodel_t &model, const Vec3 &origin)
{
	return Vec3(origin.x + (model.mins[0] + model.maxs[0]) * 0.5, origin.y + (model.mins[1] + model.maxs[1]) * 0.5,
		origin.z + (model.mins[2] + model.maxs[2]) * 0.5);
}

} // namespace

std::vector<DoorLeaf> ExtractDoorLeaves(const BspFile &bsp, const std::vector<Entity> &entities, double mapRadius)
{
	std::vector<DoorLeaf> result;

	for (const Entity &ent : entities)
	{
		std::string classname = ent.Get("classname");
		bool isDoor = classname == "func_door" || classname == "func_door_rotating";
		if (!isDoor)
			continue;

		int modelIndex = ParseBrushModelIndex(ent.Get("model"));
		if (modelIndex < 0 || static_cast<size_t>(modelIndex) >= bsp.Models().size())
			continue; // not a brush entity, or a malformed/out-of-range reference

		const dmodel_t &model = bsp.Models()[static_cast<size_t>(modelIndex)];
		Vec3 origin = ParseEntityOrigin(ent.Get("origin"));
		std::string targetname = ent.Get("targetname");

		const Entity *button = nullptr;
		const Entity *trigger = nullptr;
		if (!targetname.empty())
		{
			// HL's own CBaseDoor::Spawn does SetTouch(NULL) whenever
			// targetname is set -- try to resolve whatever actually opens
			// it. Unlike the old TagDoor, finding neither isn't fatal: the
			// door still becomes its own leaf either way (just one nothing
			// currently knows how to trigger, same as before this
			// redesign -- not this function's job to judge map design).
			button = ResolveButtonForDoor(entities, targetname);
			if (!button)
				trigger = ResolveTriggerForDoor(entities, targetname);
		}

		WalkClipnodeTreeForContents(bsp, model.headnode[kHullHuman], CONTENTS_SOLID,
			[&](int /*contents*/, const std::vector<PlaneChainEntry> &chain)
			{
				Polytope poly = BuildPolytopeFromChain(bsp, chain, mapRadius);
				if (poly.Empty())
					return;

				DoorLeaf leaf;
				leaf.polytope = TranslatePolytope(poly, origin);
				leaf.targetname = targetname;

				if (button)
				{
					int buttonModelIndex = ParseBrushModelIndex(button->Get("model"));
					if (buttonModelIndex >= 0 && static_cast<size_t>(buttonModelIndex) < bsp.Models().size())
					{
						Vec3 buttonOrigin = ParseEntityOrigin(button->Get("origin"));
						leaf.needsButton = true;
						leaf.activatorPos =
							ModelWorldCenter(bsp.Models()[static_cast<size_t>(buttonModelIndex)], buttonOrigin);
					}
				}
				else if (trigger)
				{
					int triggerModelIndex = ParseBrushModelIndex(trigger->Get("model"));
					if (triggerModelIndex >= 0 && static_cast<size_t>(triggerModelIndex) < bsp.Models().size())
					{
						Vec3 triggerOrigin = ParseEntityOrigin(trigger->Get("origin"));
						leaf.needsTrigger = true;
						leaf.activatorPos =
							ModelWorldCenter(bsp.Models()[static_cast<size_t>(triggerModelIndex)], triggerOrigin);
					}
				}

				result.push_back(std::move(leaf));
			});
	}

	return result;
}

std::vector<DoorAreaInfo> ComputeDoorAreaInfo(const std::vector<Area> &areas,
	const std::vector<DoorAreaInfo> &infoByLeafId)
{
	std::vector<DoorAreaInfo> result(areas.size());
	for (size_t i = 0; i < areas.size(); ++i)
	{
		for (int leafId : areas[i].memberLeafIds)
		{
			if (leafId < 0 || static_cast<size_t>(leafId) >= infoByLeafId.size())
				continue;
			if (!infoByLeafId[static_cast<size_t>(leafId)].isDoor)
				continue;
			result[i] = infoByLeafId[static_cast<size_t>(leafId)];
			break;
		}
	}
	return result;
}

} // namespace navgen
