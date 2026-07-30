#pragma once

#include <vector>

#include "leaf_extractor.h"
#include "polytope.h"

namespace navgen
{

struct Portal
{
	int leafA = -1;
	int leafB = -1;
	int faceIndexA = -1;
	int faceIndexB = -1;
	Vec3 pointA;
	Vec3 pointB;
	Winding winding;
	double area = 0.0;
};

std::vector<Portal> BuildPortals(const std::vector<ExtractedLeaf> &leaves);

}

