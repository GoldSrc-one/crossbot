#pragma once

#include <string>
#include <vector>

#include "area_merger.h"
#include "bsp_file.h"
#include "entity_parser.h"
#include "polytope.h"

namespace navgen
{

struct DoorLeaf
{
	Polytope polytope;
	std::string targetname;

	bool needsButton = false;
	bool needsTrigger = false;
	Vec3 activatorPos;
};

std::vector<DoorLeaf> ExtractDoorLeaves(const BspFile &bsp, const std::vector<Entity> &entities, double mapRadius);

struct DoorAreaInfo
{
	bool isDoor = false;
	bool needsButton = false;
	bool needsTrigger = false;
	Vec3 activatorPos;
};

std::vector<DoorAreaInfo> ComputeDoorAreaInfo(const std::vector<Area> &areas,
	const std::vector<DoorAreaInfo> &infoByLeafId);

}

