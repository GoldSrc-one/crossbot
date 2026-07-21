#pragma once

#include <string>
#include <vector>

#include "bsp_file.h"
#include "entity_parser.h"
#include "leaf_extractor.h"

namespace navgen
{

// One blocking entity's solid volume, extracted the same way leaf_extractor
// extracts empty leaves -- just walking a different model's hull-1 tree for
// CONTENTS_SOLID instead of model 0's tree for CONTENTS_EMPTY. A single
// entity can produce more than one convex piece if its brush work isn't
// itself convex.
struct BlockingVolume
{
	std::string classname;
	Polytope polytope;
};

// Classifies each entity by classname (func_wall/func_breakable/func_door*
// block; func_illusionary and everything else does not -- confirmed via
// crossfire.bsp's 8 func_illusionary instances that this classname is the
// standard "looks solid, has zero collision" GoldSrc idiom) and extracts
// the solid volume for every blocking entity found. Entities with no
// resolvable "model" keyvalue (not a brush entity, or malformed) are
// skipped rather than erroring, since a stray point entity misusing one of
// these classnames shouldn't abort the whole run.
std::vector<BlockingVolume> ExtractBlockingVolumes(const BspFile &bsp, const std::vector<Entity> &entities, double mapRadius);

// True if `leaf`'s volume meaningfully overlaps any blocking volume, via a
// face-normal separating-axis test (sufficient for convex polytopes in the
// overwhelming majority of real map geometry; in the rare case neither
// polytope's own face normals happen to separate two genuinely-disjoint
// convex shapes, this conservatively reports overlap -- consistent with
// this module's documented "when in doubt, exclude the leaf" bias, same
// tradeoff area_merger makes by only merging full-coverage matches).
// A leaf this returns true for should be dropped before portal-building/
// area-merging ever sees it.
bool OverlapsAnyBlockingVolume(const ExtractedLeaf &leaf, const std::vector<BlockingVolume> &blockers);

} // namespace navgen
