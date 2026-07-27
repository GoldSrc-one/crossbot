#pragma once

#include <utility>
#include <vector>

#include "leaf_extractor.h"

namespace navgen
{

struct FloorSplitResult
{

	std::vector<ExtractedLeaf> leaves;

	std::vector<std::pair<int, int>> columnPairs;
};

FloorSplitResult SplitUnsupportedColumns(const std::vector<ExtractedLeaf> &leaves, double mapRadius);

}

