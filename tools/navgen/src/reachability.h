#pragma once

#include <vector>

#include "area_merger.h"
#include "entity_parser.h"

namespace navgen
{

// Resolves every spawn-point entity (info_player_start/deathmatch/coop/
// teamspawn) to the area containing its origin. Areas found via a strict
// point-in-convex-polytope test; if none contain a given spawn exactly
// (compiler rounding, a spawn sitting exactly on a boundary), falls back
// to the nearest area by centroid distance rather than dropping that spawn
// -- every spawn should seed the reachability flood fill if at all
// possible, since a real map can rely on just one.
std::vector<int> FindSpawnAreas(const std::vector<Entity> &entities, const std::vector<Area> &areas);

// Resolves every info_target entity to the area containing its origin.
// Used to ensure target areas survive reachability pruning so bots can
// navigate to/from them even if only connected via one-way drops.
std::vector<int> FindTargetAreas(const std::vector<Entity> &entities, const std::vector<Area> &areas);

// Resolves a single arbitrary world point (e.g. an info_target's origin, for
// the --dump-svg path overlay) to the area containing it, with the same
// point-in-convex-polytope-then-nearest-centroid-fallback behavior
// FindSpawnAreas uses per spawn point above. Returns -1 only if `areas` is
// empty.
int FindAreaContainingPoint(const Vec3 &point, const std::vector<Area> &areas);

// BFS-prunes `areas`/`portals` down to only areas reachable from
// `seedAreas` via the portal-adjacency graph, re-indexing both to the
// filtered ids. This is what discards the "void" leaves/areas noted since
// Milestone 2a: raw hull-1 clipnode extraction includes CONTENTS_EMPTY
// space *outside* the sealed map (bounded only by navgen's own artificial
// clipping cube), which is topologically real empty space but never
// reachable from anywhere a player actually spawns.
//
// If `seedAreas` is empty (no spawn point resolved to anything, e.g. a
// malformed or entity-less test fixture), returns the input unchanged --
// pruning with no valid seed would discard the entire graph, which is a
// worse failure than leaving void areas in.
MergeResult PruneUnreachableAreas(const std::vector<Area> &areas, const std::vector<MergedPortal> &portals,
	const std::vector<int> &seedAreas);

} // namespace navgen
