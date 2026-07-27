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

Vec3 ParseEntityOrigin(const std::string &origin)
{
	double x = 0, y = 0, z = 0;
	std::sscanf(origin.c_str(), "%lf %lf %lf", &x, &y, &z);
	return Vec3(x, y, z);
}

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

}

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
			continue;

		const dmodel_t &model = bsp.Models()[static_cast<size_t>(modelIndex)];
		Vec3 origin = ParseEntityOrigin(ent.Get("origin"));
		std::string targetname = ent.Get("targetname");

		const Entity *button = nullptr;
		const Entity *trigger = nullptr;
		if (!targetname.empty())
		{

			button = ResolveButtonForDoor(entities, targetname);
			if (!button)
				trigger = ResolveTriggerForDoor(entities, targetname);
		}

		WalkClipnodeTreeForContents(bsp, model.headnode[kHullHuman], CONTENTS_SOLID,
			[&](int  , const std::vector<PlaneChainEntry> &chain)
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

}

