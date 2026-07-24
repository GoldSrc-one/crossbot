#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <stdexcept>
#include <string>
#include <unordered_set>

#include "astar_graph.h"
#include "path_follower.h"
#include "waypoint_plan.h"
#include "area_merger.h"
#include "blocking_entities.h"
#include "bsp_file.h"
#include "door_extractor.h"
#include "edge_tagger.h"
#include "elevator_tagger.h"
#include "entity_parser.h"
#include "floor_splitter.h"
#include "hull_defs.h"
#include "leaf_extractor.h"
#include "nav_file.h"
#include "portal_builder.h"
#include "reachability.h"
#include "svg_debug.h"
#include "tactical_tagger.h"

namespace
{

// GoldSrc maps are bounded to roughly +/-4096 units; this just needs to
// comfortably exceed any real map's extent for BaseWindingForPlane's huge
// starting quads.
constexpr double kMapRadius = 8192.0;

void PrintUsage(const char *argv0)
{
	std::fprintf(stderr,
		"usage: %s <map.bsp> [--dump-leaves] [--dump-areas] [--dump-tactical] [--dump-planes <id>] [--dump-obj <out.obj>] [--dump-svg <out.svg>] [-o <out.nav>]\n"
		"  --dump-leaves     print per-leaf (pre-merge) id/faces/volume/centroid stats\n"
		"  --dump-areas      print per-area (post-merge) id/faces/volume/centroid/portal stats\n"
		"  --dump-tactical   print per-area vantage/camp/chokepoint-cover scores and flags\n"
		"  --dump-planes <id> print one final area's exact plane list (normal+dist) and face windings\n"
		"  --dump-obj <path> write all merged area polytopes as an OBJ mesh, for spot-checking\n"
		"  --dump-svg <path> write a top-down SVG of the final area/portal graph, for offline inspection\n"
		"  --trace <path>    overlay a recorded bot trajectory (nav_trace.txt: one \"x y z\" per line,\n"
		"                    written by the runtime plugin when a goal ends) on the --dump-svg output\n"
		"  -o <path>         write the baked .nav file (crossbot_nav format)\n",
		argv0);
}

struct ObjGroup
{
	std::string name;
	const navgen::Polytope *polytope;
};

void WriteObj(const std::string &path, const std::vector<ObjGroup> &groups)
{
	FILE *f = std::fopen(path.c_str(), "w");
	if (!f)
		throw std::runtime_error("cannot open output OBJ file: " + path);

	int nextVertexIndex = 1; // OBJ vertex indices are 1-based

	for (const ObjGroup &group : groups)
	{
		std::fprintf(f, "g %s\n", group.name.c_str());

		for (const navgen::Face &face : group.polytope->faces)
		{
			if (face.winding.size() < 3)
				continue;

			for (const navgen::Vec3 &v : face.winding)
				std::fprintf(f, "v %.3f %.3f %.3f\n", v.x, v.y, v.z);

			std::fprintf(f, "f");
			for (size_t i = 0; i < face.winding.size(); ++i)
				std::fprintf(f, " %d", nextVertexIndex + static_cast<int>(i));
			std::fprintf(f, "\n");

			nextVertexIndex += static_cast<int>(face.winding.size());
		}
	}

	std::fclose(f);
}

std::string DeriveMapName(const std::string &bspPath)
{
	size_t slash = bspPath.find_last_of("/\\");
	std::string base = (slash == std::string::npos) ? bspPath : bspPath.substr(slash + 1);
	size_t dot = base.find_last_of('.');
	if (dot != std::string::npos)
		base = base.substr(0, dot);
	return base;
}

} // namespace

int main(int argc, char **argv)
{
	if (argc < 2)
	{
		PrintUsage(argv[0]);
		return 1;
	}

	std::string bspPath;
	bool dumpLeaves = false;
	bool dumpAreas = false;
	bool dumpTactical = false;
	bool dumpLadders = false;
	int dumpPlanesAreaId = -1;
	std::string objPath;
	std::string svgPath;
	std::string navPath;
	std::string tracePath;

	for (int i = 1; i < argc; ++i)
	{
		std::string arg = argv[i];
		if (arg == "--dump-leaves")
		{
			dumpLeaves = true;
		}
		else if (arg == "--dump-areas")
		{
			dumpAreas = true;
		}
		else if (arg == "--dump-tactical")
		{
			dumpTactical = true;
		}
		else if (arg == "--dump-ladders")
		{
			dumpLadders = true;
		}
		else if (arg == "--dump-planes")
		{
			if (i + 1 >= argc)
			{
				PrintUsage(argv[0]);
				return 1;
			}
			dumpPlanesAreaId = std::atoi(argv[++i]);
		}
		else if (arg == "--dump-obj")
		{
			if (i + 1 >= argc)
			{
				PrintUsage(argv[0]);
				return 1;
			}
			objPath = argv[++i];
		}
		else if (arg == "--dump-svg")
		{
			if (i + 1 >= argc)
			{
				PrintUsage(argv[0]);
				return 1;
			}
			svgPath = argv[++i];
		}
		else if (arg == "--trace")
		{
			if (i + 1 >= argc)
			{
				PrintUsage(argv[0]);
				return 1;
			}
			tracePath = argv[++i];
		}
		else if (arg == "-o")
		{
			if (i + 1 >= argc)
			{
				PrintUsage(argv[0]);
				return 1;
			}
			navPath = argv[++i];
		}
		else if (bspPath.empty())
		{
			bspPath = arg;
		}
		else
		{
			std::fprintf(stderr, "unrecognized argument: %s\n", arg.c_str());
			PrintUsage(argv[0]);
			return 1;
		}
	}

	if (bspPath.empty())
	{
		PrintUsage(argv[0]);
		return 1;
	}

	try
	{
		navgen::BspFile bsp(bspPath);
		const dmodel_t &worldModel = bsp.Models()[0];

		std::vector<navgen::ExtractedLeaf> leaves = navgen::ExtractHull1Leaves(bsp, worldModel, kMapRadius);

		std::vector<navgen::Entity> entities = navgen::ParseEntities(bsp.EntityText());
		std::vector<navgen::BlockingVolume> blockers = navgen::ExtractBlockingVolumes(bsp, entities, kMapRadius);

		if (dumpLadders)
		{
			for (const navgen::Entity &ent : entities)
			{
				if (ent.Get("classname") != "func_ladder")
					continue;
				std::string model = ent.Get("model");
				if (model.empty() || model[0] != '*')
					continue;
				int modelIndex = std::atoi(model.c_str() + 1);
				if (modelIndex < 0 || static_cast<size_t>(modelIndex) >= bsp.Models().size())
					continue;
				const dmodel_t &m = bsp.Models()[static_cast<size_t>(modelIndex)];
				double cx = (m.mins[0] + m.maxs[0]) * 0.5;
				double cy = (m.mins[1] + m.maxs[1]) * 0.5;
				double cz = (m.mins[2] + m.maxs[2]) * 0.5;
				std::printf("  func_ladder *%d: mins (%.1f, %.1f, %.1f) maxs (%.1f, %.1f, %.1f) center (%.1f, %.1f, %.1f)\n",
					modelIndex, m.mins[0], m.mins[1], m.mins[2], m.maxs[0], m.maxs[1], m.maxs[2], cx, cy, cz);
			}
		}

		size_t rawLeafCount = leaves.size();
		std::vector<navgen::ExtractedLeaf> navigableLeaves;
		navigableLeaves.reserve(leaves.size());
		size_t affectedLeaves = 0;
		size_t fullyRemovedLeaves = 0;
		size_t splitLeaves = 0;
		for (navgen::ExtractedLeaf &leaf : leaves)
		{
			if (!navgen::OverlapsAnyBlockingVolume(leaf, blockers))
			{
				leaf.id = static_cast<int>(navigableLeaves.size()); // re-index to match the filtered vector's positions
				navigableLeaves.push_back(std::move(leaf));
				continue;
			}

			// Carve the overlapping blocker(s) out rather than dropping the
			// whole leaf -- a func_wall/func_door standing free-floating in
			// the middle of an open room only blocks its own footprint, not
			// every other bit of navigable space that happens to share the
			// same hull-1 leaf (worldspawn's own clip tree never splits
			// around a *different* model's brushwork).
			++affectedLeaves;
			std::vector<navgen::Polytope> pieces = navgen::SubtractBlockingVolumes(bsp, leaf.polytope, blockers, kMapRadius);
			if (pieces.empty())
				++fullyRemovedLeaves;
			else if (pieces.size() > 1)
				++splitLeaves;

			for (navgen::Polytope &piece : pieces)
			{
				navgen::ExtractedLeaf split;
				split.id = static_cast<int>(navigableLeaves.size());
				split.polytope = std::move(piece);
				navigableLeaves.push_back(std::move(split));
			}
		}
		leaves = std::move(navigableLeaves);

		double totalLeafVolume = 0.0;
		for (const navgen::ExtractedLeaf &leaf : leaves)
			totalLeafVolume += leaf.polytope.Volume();

		std::printf("%s: %zu hull-1 empty leaves (%zu affected by %zu blocking entities: %zu fully removed, %zu split into "
			"multiple pieces), total volume %.0f cu.units\n",
			bspPath.c_str(), leaves.size(), affectedLeaves, blockers.size(), fullyRemovedLeaves, splitLeaves, totalLeafVolume);

		// Crouch-only (hull-3) passages: a spot can be too low-ceilinged for
		// hull1 (standing, 72 units tall) while still open for hull3
		// (ducking, 50 units) -- ExtractHull1Leaves simply never finds a leaf
		// there at all, since hull1's own clipnode tree marks that space
		// CONTENTS_SOLID outright. Extract hull3 separately and keep only the
		// leaves that don't already correspond to existing (standable) hull1
		// space -- tested via each candidate's centroid, since a hull3 leaf
		// wholly inside an existing hull1 leaf is just redundant, already-
		// covered space, not a genuine duck-only connection -- then append
		// the survivors to the same leaf list with continuing ids. The rest
		// of the pipeline (portal_builder, area_merger, reachability, ...)
		// treats them like any other leaf: portal_builder's coplanar-face
		// matching still works across a hull1/hull3 boundary because both
		// hulls share the same horizontal (X/Y) footprint (hull_defs.h), so a
		// vertical wall's shared face lands at the identical plane either
		// side; only floor/ceiling planes differ between the two hulls, and
		// those aren't what connects two adjacent rooms/corridors sideways
		// anyway.
		// Split unsupported air columns off the hull-1 leaves before any other
		// leaf source appends (crouch/door leaves below) -- this renumbers
		// leaf ids, and everything downstream keys off the final numbering.
		// The recorded column pairs are consumed right after NoMerge
		// (MergeAreaPairs), gluing each column onto the area beneath it. See
		// floor_splitter.h for the full why (a convex hull-1 leaf can span a
		// thin walkway rim AND the open air above the pit next to it --
		// confirmed live on test23_precise).
		size_t leafCountBeforeFloorSplit = leaves.size();
		navgen::FloorSplitResult floorSplit = navgen::SplitUnsupportedColumns(leaves, kMapRadius);
		leaves = std::move(floorSplit.leaves);
		std::printf("%s: floor-support split: %zu leaves -> %zu (%zu unsupported air columns found)\n",
			bspPath.c_str(), leafCountBeforeFloorSplit, leaves.size(), floorSplit.columnPairs.size());

		std::vector<navgen::ExtractedLeaf> duckLeaves =
			navgen::ExtractHullLeaves(bsp, worldModel, kMapRadius, navgen::kHullDuck);

		std::vector<bool> isCrouchLeaf(leaves.size(), false); // parallel to `leaves`, grows below
		size_t crouchLeavesAdded = 0;
		for (navgen::ExtractedLeaf &duckLeaf : duckLeaves)
		{
			std::vector<navgen::Polytope> pieces;
			if (navgen::OverlapsAnyBlockingVolume(duckLeaf, blockers))
				pieces = navgen::SubtractBlockingVolumes(bsp, duckLeaf.polytope, blockers, kMapRadius);
			else
				pieces.push_back(std::move(duckLeaf.polytope));

			for (navgen::Polytope &piece : pieces)
			{
				navgen::Vec3 c = piece.Centroid();
				if (navgen::ClassifyPoint(bsp, worldModel.headnode[navgen::kHullHuman], c) != CONTENTS_SOLID)
					continue; // hull1 already covers this space -- not a crouch-only passage

				navgen::ExtractedLeaf added;
				added.id = static_cast<int>(leaves.size());
				added.polytope = std::move(piece);
				leaves.push_back(std::move(added));
				isCrouchLeaf.push_back(true);
				++crouchLeavesAdded;
			}
		}
		std::printf("%s: %zu additional crouch-only (hull-3) leaves found\n", bspPath.c_str(), crouchLeavesAdded);

		// func_door/func_door_rotating: kept as their own real leaves rather
		// than subtracted away like a permanently-solid blocker (see
		// door_extractor.h's own header comment for the full reasoning) --
		// a door eventually becomes passable, so its own solid-until-opened
		// volume deserves a real area (flagged NAV_AREA_DOOR below) that
		// portal_builder's ordinary coplanar-face matching can portal into
		// and out of on both sides, same as any other adjacent leaf pair.
		std::vector<navgen::DoorLeaf> doorLeaves = navgen::ExtractDoorLeaves(bsp, entities, kMapRadius);
		std::vector<navgen::DoorAreaInfo> doorInfoByLeafId(leaves.size()); // parallel to `leaves`, grows below
		for (navgen::DoorLeaf &doorLeaf : doorLeaves)
		{
			navgen::ExtractedLeaf added;
			added.id = static_cast<int>(leaves.size());
			added.polytope = std::move(doorLeaf.polytope);
			leaves.push_back(std::move(added));
			isCrouchLeaf.push_back(false);

			navgen::DoorAreaInfo info;
			info.isDoor = true;
			info.needsButton = doorLeaf.needsButton;
			info.needsTrigger = doorLeaf.needsTrigger;
			info.activatorPos = doorLeaf.activatorPos;
			doorInfoByLeafId.push_back(info);
		}
		std::printf("%s: %zu func_door/func_door_rotating leaves found\n", bspPath.c_str(), doorLeaves.size());

		if (dumpLeaves)
		{
			for (const navgen::ExtractedLeaf &leaf : leaves)
			{
				navgen::Vec3 c = leaf.polytope.Centroid();
				navgen::Vec3 mn(1e9, 1e9, 1e9), mx(-1e9, -1e9, -1e9);
				for (const navgen::Face &f : leaf.polytope.faces)
				{
					for (const navgen::Vec3 &v : f.winding)
					{
						mn.x = std::min(mn.x, v.x); mn.y = std::min(mn.y, v.y); mn.z = std::min(mn.z, v.z);
						mx.x = std::max(mx.x, v.x); mx.y = std::max(mx.y, v.y); mx.z = std::max(mx.z, v.z);
					}
				}
				std::printf("  leaf %4d: %2zu faces, volume %10.0f, centroid (%.1f, %.1f, %.1f), "
					"mins (%.1f, %.1f, %.1f) maxs (%.1f, %.1f, %.1f)\n",
					leaf.id, leaf.polytope.faces.size(), leaf.polytope.Volume(), c.x, c.y, c.z,
					mn.x, mn.y, mn.z, mx.x, mx.y, mx.z);
			}
		}

		std::vector<navgen::Portal> portals = navgen::BuildPortals(leaves);

		if (const char *dumpPortalsEnv = std::getenv("NAVGEN_DUMP_PORTALS"))
		{
			(void)dumpPortalsEnv;
			for (const navgen::Portal &p : portals)
			{
				navgen::Vec3 mid = (p.pointA + p.pointB) * 0.5;
				std::printf("  portal leaf%d<->leaf%d area=%.1f mid=(%.1f,%.1f,%.1f)\n", p.leafA, p.leafB, p.area,
					mid.x, mid.y, mid.z);
			}
		}

		// MergeAreas disabled for now -- confirmed live (a human player
		// watching the debug area-floor draw in-game, not just coordinate
		// inspection) that it has a real bug: a room's floor rendered right
		// up near its ceiling, meaning MergeAreas folded that leaf together
		// with one from the room *above* into a single area, corrupting the
		// merged polytope. See NoMerge's own header comment (area_merger.h)
		// for the full reasoning. Every leaf becomes its own Area instead --
		// more granular and unoptimized, but each one's polytope is exactly
		// what ExtractHull1Leaves actually found, nothing unioned in wrong.
		navgen::MergeResult merged = navgen::NoMerge(leaves, portals);

		std::printf("%s: %zu raw portals -> %zu areas, %zu inter-area portals (MergeAreas disabled)\n",
			bspPath.c_str(), portals.size(), merged.areas.size(), merged.portals.size());

		// Glue each unsupported air column (floor_splitter.h, above) onto the
		// area beneath it -- the pit and the air you'd fall through to reach
		// it are one place as far as navigation is concerned. Doing this
		// before AbsorbTinyAreas/pruning means TagDropOffs later reads the
		// merged area's floor at its real (deep) height and turns the
		// walkway-to-pit walk portals into one-way drops.
		size_t areaCountBeforePairMerge = merged.areas.size();
		merged = navgen::MergeAreaPairs(merged.areas, merged.portals, floorSplit.columnPairs);
		std::printf("%s: %zu air-column areas merged into their supporting areas (%zu areas remain)\n",
			bspPath.c_str(), areaCountBeforePairMerge - merged.areas.size(), merged.areas.size());

		// Fold BSP-leaf slivers MergeAreas' convexity constraint left as
		// their own standalone areas (a ramp cut into many thin pieces,
		// each far smaller than a standing player's hull -- ~32*32*72 =
		// 73728 cu.units) into their largest real neighbor, before
		// anything downstream can pick one as a waypoint target. 10000 is
		// well under even a crouching hull's footprint (~32*32*36) so this
		// only catches genuine slivers, not small-but-real rooms/closets.
		constexpr double kMinAreaVolume = 10000.0;
		size_t areaCountBeforeAbsorb = merged.areas.size();
		// Two kinds of area are never an eligible absorption target for a
		// tiny neighbor (see AbsorbTinyAreas' own comment on neverAbsorber):
		// door areas, and pit blobs that swallowed an air column above --
		// a tiny walkway sliver bordering a huge pit area would otherwise
		// pick the pit as its "biggest neighbor" and sever the walkable
		// route it was part of. MergeAreaPairs already ran, so ids aren't
		// 1:1 with leaves anymore -- go through memberLeafIds.
		std::vector<bool> isColumnLeaf(leaves.size(), false);
		for (const auto &[columnLeafId, supportLeafId] : floorSplit.columnPairs)
			if (columnLeafId >= 0 && static_cast<size_t>(columnLeafId) < isColumnLeaf.size())
				isColumnLeaf[static_cast<size_t>(columnLeafId)] = true;
		std::vector<bool> neverAbsorber(merged.areas.size(), false);
		for (size_t i = 0; i < merged.areas.size(); ++i)
		{
			for (int leafId : merged.areas[i].memberLeafIds)
			{
				if (leafId < 0)
					continue;
				size_t li = static_cast<size_t>(leafId);
				if ((li < doorInfoByLeafId.size() && doorInfoByLeafId[li].isDoor) ||
					(li < isColumnLeaf.size() && isColumnLeaf[li]))
				{
					neverAbsorber[i] = true;
					break;
				}
			}
		}
		merged = navgen::AbsorbTinyAreas(merged.areas, merged.portals, kMinAreaVolume, neverAbsorber);
		std::printf("%s: %zu tiny-leaf-sliver areas absorbed into neighbors (%zu areas remain)\n",
			bspPath.c_str(), areaCountBeforeAbsorb - merged.areas.size(), merged.areas.size());

		// Connect ramp areas to their supporting flat floors. The portal
		// builder skips horizontal faces, so a ramp sitting on a flat floor
		// has no portal between them even though they're walkably connected.
		// This must run before TagDropOffs/ProbeCrouchLinks so the new
		// walk portals are considered in reachability.
		navgen::ConnectRampAreasToFloors(merged.areas, merged.portals);

		// Bridge crouch-only (hull-3) areas to their hull-1 neighbors before
		// the first reachability pass runs, not after (unlike ProbeJumpGaps
		// below): a crouch area starts with *zero* portals of its own --
		// portal_builder's shared-face matching doesn't work across a
		// hull-1/hull-3 boundary (see ProbeCrouchLinks' header comment) -- so
		// if pruning ran first it would discard every crouch area outright as
		// unreachable void, with nothing left afterward for this to bridge.
		navgen::ProbeCrouchLinks(bsp, worldModel.headnode[navgen::kHullDuck], merged.areas, isCrouchLeaf,
			merged.portals);

		// func_ladder: same reasoning as ProbeCrouchLinks above -- a ladder
		// can be the only connection between two areas, so tagging and
		// converting it into a real portal has to happen before the first
		// reachability pass, not after. TaggedSpecialEdge is still
		// serialized separately too (nav_format.h's SpecialEdgeRecord), but
		// this also promotes it into an ordinary MergedPortal, the same way
		// TagElevators already does below for lifts, so PruneUnreachableAreas
		// and the runtime A* (src/nav/path_astar.cpp) actually see the
		// connection. Doors no longer go through this path at all -- they're
		// their own real leaves now (see ExtractDoorLeaves above), portaled
		// in naturally by portal_builder like any other adjacent leaf pair.
		std::vector<navgen::TaggedSpecialEdge> specialEdges = navgen::TagSpecialEdges(bsp, entities, merged.areas);
		for (const navgen::TaggedSpecialEdge &e : specialEdges)
		{
			navgen::MergedPortal portal;
			portal.areaA = e.areaFrom;
			portal.areaB = e.areaTo;
			portal.pointA = e.anchorFrom;
			portal.pointB = e.anchorTo;
			portal.area = 0.0; // not a real shared-face overlap
			portal.edgeType = e.type;
			merged.portals.push_back(portal);
		}

		// TagDropOffs runs first to classify walk portals as drops/jumps based on
		// height deltas. This MUST run before ProbeJumpGaps so that the
		// DisjointSet in ProbeJumpGaps only unions through truly bidirectional
		// edges (walk/jump/ladder/elevator), not through one-way drops that
		// only appear bidirectional before this classification.
		std::vector<navgen::DoorAreaInfo> earlyDoorAreaInfo =
			navgen::ComputeDoorAreaInfo(merged.areas, doorInfoByLeafId);
		navgen::TagDropOffs(merged.areas, merged.portals, earlyDoorAreaInfo);

		// Connect ramp areas to the flat floor areas they sit on. Ramp areas
		// and their supporting floors share a horizontal face (ramp bottom /
		// floor top) which portal_builder skips because it filters horizontal
		// faces. This creates a navigation gap. We connect them here.
		navgen::ConnectRampAreasToFloors(merged.areas, merged.portals);

		// Preliminary walk-only flood fill on the FULL area set (before pruning)
		// to identify which areas are walk-reachable from spawn. We'll use this
		// to filter jump edges: only keep jumps FROM walk-reachable areas, to
		// avoid spurious jumps from void areas outside the map.
		std::vector<int> spawnAreas = navgen::FindSpawnAreas(entities, merged.areas);
		std::unordered_set<int> walkReachable = navgen::GetWalkReachableAreas(merged.areas, merged.portals, spawnAreas);

		// Probe for jump gaps on the FULL area set (before pruning void areas).
		// This finds jumps between ANY areas, including void areas on the other
		// side of a gap. We'll filter these next.
		size_t portalCountBeforeJumps = merged.portals.size();
		navgen::ProbeJumpGaps(bsp, worldModel.headnode[navgen::kHullHuman], merged.areas, merged.portals);

		// Filter the newly added jump portals: keep jumps where AT LEAST ONE
		// endpoint is walk-reachable from spawn. ProbeJumpGaps creates portals
		// with areaA=i, areaB=j where i<j (index order), not necessarily
		// "from->to" order. A jump is valid if either side is reachable.
		for (size_t pi = portalCountBeforeJumps; pi < merged.portals.size(); ++pi)
		{
			const navgen::MergedPortal &p = merged.portals[pi];
			if (p.edgeType == crossbot_nav::EDGE_JUMP &&
				walkReachable.count(p.areaA) == 0 &&
				walkReachable.count(p.areaB) == 0)
			{
				// Neither endpoint is walk-reachable -- likely a void area
				// outside the map. Discard it.
				merged.portals[pi] = merged.portals.back();
				merged.portals.pop_back();
				--pi; // re-check this index
			}
		}

		// First prune pass: now includes the filtered jump edges, so areas only
		// reachable via a jump from a walk-reachable area will survive.
		size_t rawAreaCount = merged.areas.size();
		merged = navgen::PruneUnreachableAreas(merged.areas, merged.portals, spawnAreas);
		std::printf("%s: %zu spawn point(s) resolved, %zu areas reachable after first pass (%zu void areas pruned)\n",
			bspPath.c_str(), spawnAreas.size(), merged.areas.size(), rawAreaCount - merged.areas.size());

		// Now probe for ADDITIONAL jump gaps on the PRUNED area set. This
		// catches jumps between areas that were both previously void but are now
		// connected via a chain: walk-reachable -> jump -> areaA -> jump -> areaB.
		// The DisjointSet in ProbeJumpGaps respects the updated edge types
		// (drops excluded by TagDropOffs above).
		navgen::ProbeJumpGaps(bsp, worldModel.headnode[navgen::kHullHuman], merged.areas, merged.portals);

		if (std::getenv("NAVGEN_DUMP_EDGES"))
		{
			for (const navgen::MergedPortal &p : merged.portals)
			{
				const char *typeName = p.edgeType == crossbot_nav::EDGE_WALK ? "WALK"
					: p.edgeType == crossbot_nav::EDGE_JUMP    ? "JUMP"
					: p.edgeType == crossbot_nav::EDGE_DROP    ? "DROP"
					: p.edgeType == crossbot_nav::EDGE_LADDER  ? "LADDER"
					: p.edgeType == crossbot_nav::EDGE_ELEVATOR ? "ELEVATOR"
					: "?";
				std::printf("  edge area%d<->area%d type=%s pointA=(%.1f,%.1f,%.1f) pointB=(%.1f,%.1f,%.1f)\n",
					p.areaA, p.areaB, typeName, p.pointA.x, p.pointA.y, p.pointA.z, p.pointB.x, p.pointB.y,
					p.pointB.z);
			}
		}

		// Elevators (button-gated vertical func_door lifts) also run here,
		// before the second prune pass, so an area only reachable by riding
		// one isn't wrongly discarded as unreachable.
		std::vector<navgen::MergedPortal> elevatorPortals = navgen::TagElevators(bsp, entities, merged.areas);
		merged.portals.insert(merged.portals.end(), elevatorPortals.begin(), elevatorPortals.end());

		if (const char *preSvgPath = std::getenv("NAVGEN_DUMP_SVG_PREPRUNE"))
		{
			std::vector<navgen::Vec3> spawnPointsDbg, targetPointsDbg;
			for (const navgen::Entity &ent : entities)
			{
				std::string classname = ent.Get("classname");
				bool isSpawn = classname == "info_player_start" || classname == "info_player_deathmatch" ||
					classname == "info_player_coop" || classname == "info_player_teamspawn";
				bool isTarget = classname == "info_target";
				if (!isSpawn && !isTarget)
					continue;
				double x = 0, y = 0, z = 0;
				if (std::sscanf(ent.Get("origin").c_str(), "%lf %lf %lf", &x, &y, &z) != 3)
					continue;
				(isSpawn ? spawnPointsDbg : targetPointsDbg).push_back(navgen::Vec3(x, y, z));
			}
			navgen::WriteDebugSvg(preSvgPath, DeriveMapName(bspPath) + "_preprune", merged.areas, merged.portals, {},
				{}, spawnPointsDbg, targetPointsDbg);
			std::printf("wrote %s (pre-second-prune diagnostic)\n", preSvgPath);
		}

		// Second prune pass, now that one-way flags exist: an area only
		// connected to the rest of the graph via a one-way drop *out* of it
		// (e.g. jumping off a tower with no climbable way back up) isn't
		// actually reachable *from* the rest of the map, even though the
		// first pass's undirected walk would have called it connected.
		// Confirmed live: a real crossfire.bsp rooftop area survived the
		// old single-pass pruning yet A* permanently failed to route any
		// bot toward it from the ground.
		spawnAreas = navgen::FindSpawnAreas(entities, merged.areas);
		size_t areaCountBeforeSecondPrune = merged.areas.size();
		merged = navgen::PruneUnreachableAreas(merged.areas, merged.portals, spawnAreas);
		std::printf("%s: %zu areas reachable after direction-aware second pass (%zu one-way-only areas pruned)\n",
			bspPath.c_str(), merged.areas.size(), areaCountBeforeSecondPrune - merged.areas.size());

		int ladderEdges = static_cast<int>(specialEdges.size()); // ladders are the only producer left, see TagSpecialEdges
		// Counts every EDGE_JUMP portal in the final graph, not just the
		// ones ProbeJumpGaps added as brand-new connections -- TagDropOffs
		// also reclassifies some already-adjacent EDGE_WALK portals to
		// EDGE_JUMP (a real shared face too tall to just walk across), and
		// those need to show up here too.
		int jumpEdges = 0, dropEdges = 0, elevatorEdges = 0;
		for (const navgen::MergedPortal &p : merged.portals)
		{
			if (p.edgeType == crossbot_nav::EDGE_JUMP)
				++jumpEdges;
			else if (p.edgeType == crossbot_nav::EDGE_DROP)
				++dropEdges;
			else if (p.edgeType == crossbot_nav::EDGE_ELEVATOR)
				++elevatorEdges;
		}

		std::printf("%s: %d ladder edges, %d jump edges, %d drop-off edges, %d elevator edges\n",
			bspPath.c_str(), ladderEdges, jumpEdges, dropEdges, elevatorEdges);

		std::vector<navgen::TacticalScore> tacticalScores =
			navgen::TagTactical(bsp, worldModel.headnode[navgen::kHullHuman], merged.areas, merged.portals);

		int vantageCount = 0, campCount = 0, chokeCount = 0;
		for (const navgen::TacticalScore &t : tacticalScores)
		{
			vantageCount += t.isVantage ? 1 : 0;
			campCount += t.isCamp ? 1 : 0;
			chokeCount += t.isChokepointCover ? 1 : 0;
		}
		std::printf("%s: tactical tagging -> %d vantage, %d camp, %d chokepoint-cover areas (of %zu)\n",
			bspPath.c_str(), vantageCount, campCount, chokeCount, merged.areas.size());

		// Final crouch-area flags: memberLeafIds still refers to the original
		// (pre-merge/pre-prune) leaf ids isCrouchLeaf was built against, so
		// this stays correct through AbsorbTinyAreas' folding and both
		// PruneUnreachableAreas passes regardless of how area ids themselves
		// got renumbered along the way.
		std::vector<bool> crouchAreas = navgen::ComputeCrouchAreaFlags(merged.areas, isCrouchLeaf);
		int crouchAreaCount = 0;
		for (bool c : crouchAreas)
			crouchAreaCount += c ? 1 : 0;
		std::printf("%s: %d areas require crouching (hull-3-only clearance)\n", bspPath.c_str(), crouchAreaCount);

		// Final door-area flags: same reasoning as crouchAreas above --
		// memberLeafIds still refers to the original (pre-merge/pre-prune)
		// leaf ids doorInfoByLeafId was built against, and now also survives
		// AbsorbTinyAreas folding (area_merger.cpp propagates every absorbed
		// leaf's id into the survivor as of this door redesign).
		std::vector<navgen::DoorAreaInfo> doorAreaInfo = navgen::ComputeDoorAreaInfo(merged.areas, doorInfoByLeafId);
		int doorAreaCount = 0;
		for (const navgen::DoorAreaInfo &d : doorAreaInfo)
			doorAreaCount += d.isDoor ? 1 : 0;
		std::printf("%s: %d areas are doors (of %zu found in the map)\n", bspPath.c_str(), doorAreaCount,
			doorLeaves.size());

		if (dumpTactical)
		{
			for (size_t i = 0; i < merged.areas.size(); ++i)
			{
				const navgen::Area &area = merged.areas[i];
				const navgen::TacticalScore &t = tacticalScores[i];
				navgen::Vec3 c = area.polytope.Centroid();
				std::printf("  area %4d: centroid (%.1f, %.1f, %.1f) exposure=%.0f concealment=%.3f danger=%.0f%s%s%s\n",
					area.id, c.x, c.y, c.z, t.exposureScore, t.concealmentScore, t.dangerScore,
					t.isVantage ? " [VANTAGE]" : "", t.isCamp ? " [CAMP]" : "",
					t.isChokepointCover ? " [CHOKEPOINT_COVER]" : "");
			}
		}

		if (dumpAreas)
		{
			for (const navgen::Area &area : merged.areas)
			{
				navgen::Vec3 c = area.polytope.Centroid();
				navgen::Vec3 mn(1e9, 1e9, 1e9), mx(-1e9, -1e9, -1e9);
				for (const navgen::Face &f : area.polytope.faces)
				{
					for (const navgen::Vec3 &v : f.winding)
					{
						mn.x = std::min(mn.x, v.x); mn.y = std::min(mn.y, v.y); mn.z = std::min(mn.z, v.z);
						mx.x = std::max(mx.x, v.x); mx.y = std::max(mx.y, v.y); mx.z = std::max(mx.z, v.z);
					}
				}
				std::printf("  area %4d: %2zu leaves, %2zu faces, volume %10.0f, centroid (%.1f, %.1f, %.1f), "
					"mins (%.1f, %.1f, %.1f) maxs (%.1f, %.1f, %.1f)\n",
					area.id, area.memberLeafIds.size(), area.polytope.faces.size(),
					area.polytope.Volume(), c.x, c.y, c.z, mn.x, mn.y, mn.z, mx.x, mx.y, mx.z);
			}
		}

		if (dumpPlanesAreaId >= 0)
		{
			bool found = false;
			for (const navgen::Area &area : merged.areas)
			{
				if (area.id != dumpPlanesAreaId)
					continue;
				found = true;
				std::printf("area %d: %zu faces\n", area.id, area.polytope.faces.size());
				for (size_t fi = 0; fi < area.polytope.faces.size(); ++fi)
				{
					const navgen::Face &f = area.polytope.faces[fi];
					std::printf("  face %2zu: normal (%.6f, %.6f, %.6f) dist %.3f -- %zu verts\n",
						fi, f.plane.normal.x, f.plane.normal.y, f.plane.normal.z, f.plane.dist, f.winding.size());
					for (const navgen::Vec3 &v : f.winding)
						std::printf("      v (%.2f, %.2f, %.2f)\n", v.x, v.y, v.z);
				}
			}
			if (!found)
				std::fprintf(stderr, "--dump-planes: no area with id %d in the final area set\n", dumpPlanesAreaId);
		}

		if (!objPath.empty())
		{
			std::vector<ObjGroup> groups;
			groups.reserve(merged.areas.size());
			for (const navgen::Area &area : merged.areas)
				groups.push_back({"area_" + std::to_string(area.id), &area.polytope});
			WriteObj(objPath, groups);
			std::printf("wrote %s\n", objPath.c_str());
		}

		if (!svgPath.empty())
		{
			std::vector<navgen::Vec3> spawnPoints, targetPoints;
			for (const navgen::Entity &ent : entities)
			{
				std::string classname = ent.Get("classname");
				bool isSpawn = classname == "info_player_start" || classname == "info_player_deathmatch" ||
					classname == "info_player_coop" || classname == "info_player_teamspawn";
				bool isTarget = classname == "info_target";
				if (!isSpawn && !isTarget)
					continue;
				double x = 0, y = 0, z = 0;
				if (std::sscanf(ent.Get("origin").c_str(), "%lf %lf %lf", &x, &y, &z) != 3)
					continue;
				(isSpawn ? spawnPoints : targetPoints).push_back(navgen::Vec3(x, y, z));
			}

			// Indexed by leaf id (leaves[i].id == i, see NoMerge's own comment)
			// so WriteDebugSvg can draw each area's ORIGINAL member leaves
			// individually -- AbsorbTinyAreas doesn't geometrically union an
			// absorbed leaf into its survivor (see area_merger.cpp), so
			// area.polytope alone would leave that leaf's own volume as an
			// unfilled hole in the rendering.
			std::vector<navgen::Polytope> leafPolytopesById(leaves.size());
			for (const navgen::ExtractedLeaf &leaf : leaves)
				leafPolytopesById[static_cast<size_t>(leaf.id)] = leaf.polytope;

			// Spawn-to-target path overlay: runs the exact same A* the live
			// bot uses (NavAStar::FindPath, src/nav/astar_graph.cpp -- shared
			// verbatim with the runtime's path_astar.cpp, see that header's
			// comment) over this map's own final area/portal graph, so the
			// debug SVG shows literally the same route a bot's Repath() would
			// find, not a hand-reimplemented approximation that could
			// silently drift out of sync -- then plans waypoints with the
			// shared NavPlan::BuildWaypointPlan and simulates driving them
			// with the shared NavFollow controller under modeled momentum
			// physics, flagging any point where the simulated bot leaves
			// supported floor (a predicted fall, before ever starting a
			// server).
			std::vector<navgen::Vec3> pathPoints, simPathPoints, simFallPoints;
			if (!spawnPoints.empty() && !targetPoints.empty())
			{
				int startAreaId = navgen::FindAreaContainingPoint(spawnPoints.front(), merged.areas);
				int goalAreaId = navgen::FindAreaContainingPoint(targetPoints.front(), merged.areas);
				if (startAreaId >= 0 && goalAreaId >= 0)
				{
					std::vector<NavAStar::AreaNode> graphAreas(merged.areas.size());
					for (size_t i = 0; i < merged.areas.size(); ++i)
					{
						navgen::Vec3 c = merged.areas[i].polytope.Centroid();
						graphAreas[i].centroid = Vector(static_cast<float>(c.x), static_cast<float>(c.y),
							static_cast<float>(c.z));
					}

					std::vector<std::vector<uint32_t>> areaPortalIndices(merged.areas.size());
					for (size_t i = 0; i < merged.portals.size(); ++i)
					{
						const navgen::MergedPortal &p = merged.portals[i];
						if (p.areaA >= 0 && static_cast<size_t>(p.areaA) < areaPortalIndices.size())
							areaPortalIndices[static_cast<size_t>(p.areaA)].push_back(static_cast<uint32_t>(i));
						if (p.areaB >= 0 && static_cast<size_t>(p.areaB) < areaPortalIndices.size())
							areaPortalIndices[static_cast<size_t>(p.areaB)].push_back(static_cast<uint32_t>(i));
					}

					std::vector<NavAStar::PortalEdge> graphPortals(merged.portals.size());
					for (size_t i = 0; i < merged.portals.size(); ++i)
					{
						const navgen::MergedPortal &p = merged.portals[i];
						graphPortals[i].areaA = static_cast<uint32_t>(p.areaA);
						graphPortals[i].areaB = static_cast<uint32_t>(p.areaB);
						graphPortals[i].oneWay = (p.edgeFlags & crossbot_nav::EDGE_FLAG_ONE_WAY) != 0;
						// Matches nav_file.cpp's WriteNavFile traverseCost formula exactly.
						graphPortals[i].traverseCost = (p.edgeType == crossbot_nav::EDGE_JUMP) ? 1.5f
							: (p.edgeType == crossbot_nav::EDGE_ELEVATOR)                       ? 2.0f
																								 : 1.0f;
					}

					NavAStar::GraphResult graphResult = NavAStar::FindPath(graphAreas, areaPortalIndices,
						graphPortals, static_cast<uint32_t>(startAreaId), static_cast<uint32_t>(goalAreaId));

					if (graphResult.found)
					{
						// Build the waypoint plan with the exact same shared
						// planner the runtime's BuildWaypointPath calls
						// (src/nav/waypoint_plan.cpp: funnel string-pull
						// between hard anchors, special-crossing expansions,
						// plan-encoded speed limits) -- this block is just
						// the MergedPortal/Area -> PlanArea/PlanPortal
						// adapter, mirroring path_funnel.cpp's own NavMesh
						// adapter. The drawn line IS the walk line.
						auto ToV = [](const navgen::Vec3 &v)
						{ return Vector(static_cast<float>(v.x), static_cast<float>(v.y), static_cast<float>(v.z)); };
						auto ToVec3 = [](const Vector &v) { return navgen::Vec3(v.x, v.y, v.z); };

						std::vector<NavPlan::PlanArea> planAreas(merged.areas.size());
						for (size_t i = 0; i < merged.areas.size(); ++i)
						{
							NavPlan::PlanArea &pa = planAreas[i];
							pa.centroid = ToV(merged.areas[i].polytope.Centroid());
							pa.crouch = crouchAreas[i];
							pa.door = doorAreaInfo[i].isDoor;
							pa.doorActivation = doorAreaInfo[i].needsButton ? crossbot_nav::DOOR_ACTIVATION_BUTTON
								: doorAreaInfo[i].needsTrigger             ? crossbot_nav::DOOR_ACTIVATION_TRIGGER
																		   : crossbot_nav::DOOR_ACTIVATION_TOUCH;
							pa.doorActivatorPos = ToV(doorAreaInfo[i].activatorPos);
							pa.doorWaitTime = 1.5f; // matches nav_file.cpp's baked doorWaitTime
						}
						std::vector<NavPlan::PlanPortal> planPortals(merged.portals.size());
						for (size_t i = 0; i < merged.portals.size(); ++i)
						{
							const navgen::MergedPortal &p = merged.portals[i];
							NavPlan::PlanPortal &pp = planPortals[i];
							pp.areaA = static_cast<uint32_t>(p.areaA);
							pp.areaB = static_cast<uint32_t>(p.areaB);
							pp.pointA = ToV(p.pointA);
							pp.pointB = ToV(p.pointB);
							pp.edgeType = p.edgeType;
							pp.buttonPos = ToV(p.buttonPos);
							pp.travelTime = static_cast<float>(p.travelTime);
							if (p.edgeType == crossbot_nav::EDGE_DROP)
							{
								planAreas[static_cast<size_t>(p.areaA)].hasDropPortal = true;
								planAreas[static_cast<size_t>(p.areaB)].hasDropPortal = true;
							}
						}

						std::vector<NavPlan::PlanWaypoint> planWaypoints = NavPlan::BuildWaypointPlan(planAreas,
							planPortals, graphResult.areaPath, graphResult.portalPath, ToV(spawnPoints.front()),
							ToV(targetPoints.front()));
						for (const NavPlan::PlanWaypoint &wp : planWaypoints)
							pathPoints.push_back(ToVec3(wp.point));

						// Momentum simulation: integrate the shared steering
						// controller (the same NavFollow::Steer the live bot
						// runs) against modeled GoldSrc ground physics, then
						// check every simulated position for supported floor
						// under it -- a position outside every navigable
						// leaf's polytope means the modeled bot left the
						// mesh (walked off an edge / clipped a wall) and the
						// live one would fall there too.
						NavFollow::SimResult sim = NavFollow::Simulate(planWaypoints, ToV(spawnPoints.front()));

						auto PointInPolytope = [](const navgen::Vec3 &pt, const navgen::Polytope &poly, double eps)
						{
							for (const navgen::Face &f : poly.faces)
								if (pt.Dot(f.plane.normal) - f.plane.dist < -eps)
									return false;
							return true;
						};
						std::vector<const navgen::Polytope *> navigableLeafPolys;
						for (const navgen::Area &area : merged.areas)
							for (int leafId : area.memberLeafIds)
							{
								if (leafId < 0 || static_cast<size_t>(leafId) >= leafPolytopesById.size())
									continue;
								if (static_cast<size_t>(leafId) < isColumnLeaf.size() &&
									isColumnLeaf[static_cast<size_t>(leafId)])
									continue; // unsupported air -- being here means falling
								navigableLeafPolys.push_back(&leafPolytopesById[static_cast<size_t>(leafId)]);
							}

						for (size_t si = 0; si < sim.points.size(); ++si)
						{
							navgen::Vec3 p = ToVec3(sim.points[si]);
							bool supported = false;
							for (const navgen::Polytope *poly : navigableLeafPolys)
							{
								if (PointInPolytope(p, *poly, 4.0))
								{
									supported = true;
									break;
								}
							}
							if (!supported)
							{
								simFallPoints.push_back(p);
								std::printf("%s: SIMULATED FALL at (%.1f, %.1f, %.1f), t=%.2fs -- the planned "
											"path is not drivable at planned speeds\n",
									bspPath.c_str(), p.x, p.y, p.z, static_cast<double>(si) / 60.0);
								break;
							}
						}
						if (!sim.finished && simFallPoints.empty())
							std::printf("%s: simulation did not reach the goal within its time budget\n",
								bspPath.c_str());

						// Decimate for drawing (60Hz sim -> every 3rd point);
						// the fall check above already ran at full rate.
						for (size_t si = 0; si < sim.points.size(); si += 3)
							simPathPoints.push_back(ToVec3(sim.points[si]));
					}
					else
					{
						std::printf("%s: --dump-svg path overlay: no A* route from area %d to area %d\n",
							bspPath.c_str(), startAreaId, goalAreaId);
					}
				}
			}

			// Recorded live-bot trajectory (see PrintUsage's --trace): one
			// "x y z" per line, as written by the runtime's WriteTraceLog.
			std::vector<navgen::Vec3> tracePoints;
			if (!tracePath.empty())
			{
				FILE *tf = std::fopen(tracePath.c_str(), "r");
				if (!tf)
					throw std::runtime_error("cannot open trace file: " + tracePath);
				double x = 0, y = 0, z = 0;
				while (std::fscanf(tf, "%lf %lf %lf", &x, &y, &z) == 3)
					tracePoints.push_back(navgen::Vec3(x, y, z));
				std::fclose(tf);
				std::printf("%s: overlaying %zu recorded trajectory points from %s\n", bspPath.c_str(),
					tracePoints.size(), tracePath.c_str());
			}

			std::string mapName = DeriveMapName(bspPath);
			navgen::WriteDebugSvg(svgPath, mapName, merged.areas, merged.portals, crouchAreas, tacticalScores,
				spawnPoints, targetPoints, leafPolytopesById, doorAreaInfo, pathPoints, simPathPoints,
				simFallPoints, tracePoints);
			std::printf("wrote %s\n", svgPath.c_str());
		}

		if (!navPath.empty())
		{
			std::string mapName = DeriveMapName(bspPath);
			navgen::WriteNavFile(navPath, mapName, bsp.Checksum(), merged.areas, merged.portals, specialEdges,
				tacticalScores, crouchAreas, doorAreaInfo);
			std::printf("wrote %s (map \"%s\", checksum 0x%08X)\n", navPath.c_str(), mapName.c_str(), bsp.Checksum());
		}
	}
	catch (const std::exception &e)
	{
		std::fprintf(stderr, "error: %s\n", e.what());
		return 1;
	}

	return 0;
}
