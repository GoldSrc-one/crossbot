#pragma once

#include <vector>

#include "blocking_entities.h"
#include "bsp_file.h"
#include "leaf_extractor.h"
#include "polytope.h"

namespace navgen
{

std::vector<Winding> ComputeStandableFloor(const BspFile &bsp, int standHeadnode,
	const std::vector<BlockingVolume> &standBlockers, const Face &floorFace);

std::vector<ExtractedLeaf> SplitLeavesByStandClearance(const BspFile &bsp, int standHeadnode,
	const std::vector<BlockingVolume> &standBlockers, const std::vector<ExtractedLeaf> &leaves, double mapRadius,
	std::vector<bool> &isCrouchLeafOut, size_t *splitOut = nullptr);

}

