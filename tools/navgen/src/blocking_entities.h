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
	// The owning entity's hull-1 clipnode headnode and origin-brush offset:
	// SubtractBlockingVolumes carves against the entity's TREE (see
	// clipnode_tree.h's CarvePolytopeAgainstTree for why per-volume
	// subtraction explodes on curved brushwork), and volumes here serve as
	// the overlap pre-filter. All volumes from one entity share these.
	int headnode = -1;
	Vec3 origin;
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
// A leaf this returns true for should be passed through SubtractBlockingVolumes
// before portal-building/area-merging ever sees it.
bool OverlapsAnyBlockingVolume(const ExtractedLeaf &leaf, const std::vector<BlockingVolume> &blockers);

// Carves every overlapping blocking entity out of `leafPolytope` -- by
// splitting the leaf down each overlapping entity's own hull-1 clipnode
// tree (clipnode_tree.h's CarvePolytopeAgainstTree) -- and returns the
// surviving piece(s): zero if the leaf is fully covered, one if a blocker
// only grazes a corner/edge without splitting it, more than one if a
// blocker cuts all the way through (e.g. a func_wall standing free-floating
// in the middle of an open room, splitting hull-1's single unified leaf into
// disjoint pieces on either side). Replaces the old whole-leaf-exclusion
// behavior (OverlapsAnyBlockingVolume used as a drop filter), which threw
// away an entire leaf -- including all the navigable space nowhere near the
// blocker -- the moment it overlapped by any amount; confirmed via
// test17_funcwall.bsp, where a single func_wall entity mid-room made hull-1
// extraction report zero leaves for the whole map, since hull-1's own
// clipnode tree for worldspawn never splits around a *different* model's
// brushwork. portal_builder.cpp's coplanar-face matching (keyed purely on
// canonicalized/quantized plane, independent of which original leaf a face
// came from) automatically reconnects sibling pieces that share an exact
// cut face, so subdividing a leaf here doesn't lose any real connectivity.
std::vector<Polytope> SubtractBlockingVolumes(const BspFile &bsp, const Polytope &leafPolytope,
	const std::vector<BlockingVolume> &blockers, double mapRadius, double epsilon = 0.01);

} // namespace navgen
