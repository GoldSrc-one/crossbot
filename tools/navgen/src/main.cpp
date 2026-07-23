#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <stdexcept>
#include <string>

#include "area_merger.h"
#include "blocking_entities.h"
#include "bsp_file.h"
#include "edge_tagger.h"
#include "elevator_tagger.h"
#include "entity_parser.h"
#include "hull_defs.h"
#include "leaf_extractor.h"
#include "nav_file.h"
#include "portal_builder.h"
#include "reachability.h"
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
		"usage: %s <map.bsp> [--dump-leaves] [--dump-areas] [--dump-tactical] [--dump-planes <id>] [--dump-obj <out.obj>] [-o <out.nav>]\n"
		"  --dump-leaves     print per-leaf (pre-merge) id/faces/volume/centroid stats\n"
		"  --dump-areas      print per-area (post-merge) id/faces/volume/centroid/portal stats\n"
		"  --dump-tactical   print per-area vantage/camp/chokepoint-cover scores and flags\n"
		"  --dump-planes <id> print one final area's exact plane list (normal+dist) and face windings\n"
		"  --dump-obj <path> write all merged area polytopes as an OBJ mesh, for spot-checking\n"
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
	std::string navPath;

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
		for (navgen::ExtractedLeaf &leaf : leaves)
		{
			if (navgen::OverlapsAnyBlockingVolume(leaf, blockers))
				continue;
			leaf.id = static_cast<int>(navigableLeaves.size()); // re-index to match the filtered vector's positions
			navigableLeaves.push_back(std::move(leaf));
		}
		leaves = std::move(navigableLeaves);

		double totalLeafVolume = 0.0;
		for (const navgen::ExtractedLeaf &leaf : leaves)
			totalLeafVolume += leaf.polytope.Volume();

		std::printf("%s: %zu hull-1 empty leaves (%zu excluded by %zu blocking entities), total volume %.0f cu.units\n",
			bspPath.c_str(), leaves.size(), rawLeafCount - leaves.size(), blockers.size(), totalLeafVolume);

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
		std::vector<navgen::ExtractedLeaf> duckLeaves =
			navgen::ExtractHullLeaves(bsp, worldModel, kMapRadius, navgen::kHullDuck);

		std::vector<bool> isCrouchLeaf(leaves.size(), false); // parallel to `leaves`, grows below
		size_t crouchLeavesAdded = 0;
		for (navgen::ExtractedLeaf &duckLeaf : duckLeaves)
		{
			if (navgen::OverlapsAnyBlockingVolume(duckLeaf, blockers))
				continue;

			navgen::Vec3 c = duckLeaf.polytope.Centroid();
			if (navgen::ClassifyPoint(bsp, worldModel.headnode[navgen::kHullHuman], c) != CONTENTS_SOLID)
				continue; // hull1 already covers this space -- not a crouch-only passage

			duckLeaf.id = static_cast<int>(leaves.size());
			leaves.push_back(std::move(duckLeaf));
			isCrouchLeaf.push_back(true);
			++crouchLeavesAdded;
		}
		std::printf("%s: %zu additional crouch-only (hull-3) leaves found\n", bspPath.c_str(), crouchLeavesAdded);

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

		// Fold BSP-leaf slivers MergeAreas' convexity constraint left as
		// their own standalone areas (a ramp cut into many thin pieces,
		// each far smaller than a standing player's hull -- ~32*32*72 =
		// 73728 cu.units) into their largest real neighbor, before
		// anything downstream can pick one as a waypoint target. 10000 is
		// well under even a crouching hull's footprint (~32*32*36) so this
		// only catches genuine slivers, not small-but-real rooms/closets.
		constexpr double kMinAreaVolume = 10000.0;
		size_t areaCountBeforeAbsorb = merged.areas.size();
		merged = navgen::AbsorbTinyAreas(merged.areas, merged.portals, kMinAreaVolume);
		std::printf("%s: %zu tiny-leaf-sliver areas absorbed into neighbors (%zu areas remain)\n",
			bspPath.c_str(), areaCountBeforeAbsorb - merged.areas.size(), merged.areas.size());

		// Bridge crouch-only (hull-3) areas to their hull-1 neighbors before
		// the first reachability pass runs, not after (unlike ProbeJumpGaps
		// below): a crouch area starts with *zero* portals of its own --
		// portal_builder's shared-face matching doesn't work across a
		// hull-1/hull-3 boundary (see ProbeCrouchLinks' header comment) -- so
		// if pruning ran first it would discard every crouch area outright as
		// unreachable void, with nothing left afterward for this to bridge.
		navgen::ProbeCrouchLinks(bsp, worldModel.headnode[navgen::kHullDuck], merged.areas, isCrouchLeaf,
			merged.portals);

		// func_ladder/func_door(_rotating): same reasoning as ProbeCrouchLinks
		// above -- a door or ladder can be the *only* connection between two
		// areas (confirmed live on test9_door: two rooms with no other
		// shared geometry at all, joined only by a touch-to-open func_door),
		// so tagging and converting these into real portals has to happen
		// before the first reachability pass, not after. TaggedSpecialEdge is
		// still serialized separately too (nav_format.h's SpecialEdgeRecord,
		// e.g. for a door's targetname re-resolution) -- this just also
		// promotes each one into an ordinary MergedPortal, the same way
		// TagElevators already does below for lifts, so PruneUnreachableAreas
		// and the runtime A* (src/nav/path_astar.cpp) actually see the
		// connection. Without this, a door/ladder edge existed only as
		// descriptive metadata nothing in the pathfinding graph ever
		// consulted -- confirmed live: test9_door's door was tagged
		// correctly yet the bot still couldn't route through it at all.
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
			if (e.needsButton)
			{
				portal.edgeFlags |= crossbot_nav::EDGE_FLAG_NEEDS_BUTTON;
				portal.buttonPos = e.buttonPos;
				// Not a reverse-engineered per-door value -- covers a
				// typical func_door's open animation (speed 100, whether
				// sliding or a func_door_rotating's ~90-degree swing) with
				// margin, same as bot_move_nav.cpp's own kDoorOpenWaitTime
				// for a plain touch-opened door.
				portal.travelTime = 1.5;
			}
			merged.portals.push_back(portal);
		}

		// First prune pass: eliminate void/outside-the-map leaves using only
		// genuine shared-face WALK portals, *before* ProbeJumpGaps ever
		// runs. ProbeJumpGaps is a pure coordinate-distance heuristic with
		// no collision/visibility check at all, so if it ran on the
		// unpruned area set it would happily manufacture "jump" connections
		// between a real area and a void leaf sitting just outside a wall
		// (spatially close, nothing checked in between) -- confirmed
		// happening: pruning this pass normally discards ~267 void areas
		// on crossfire.bsp, but running ProbeJumpGaps first dropped that to
		// 26, because most of those void areas picked up a spurious jump
		// edge into the reachable set.
		std::vector<int> spawnAreas = navgen::FindSpawnAreas(entities, merged.areas);
		size_t rawAreaCount = merged.areas.size();
		merged = navgen::PruneUnreachableAreas(merged.areas, merged.portals, spawnAreas);
		std::printf("%s: %zu spawn point(s) resolved, %zu areas reachable after first pass (%zu void areas pruned)\n",
			bspPath.c_str(), spawnAreas.size(), merged.areas.size(), rawAreaCount - merged.areas.size());

		// ProbeJumpGaps/TagDropOffs run on the now-void-free area set, and
		// set EDGE_FLAG_ONE_WAY (a one-way drop/jump-only connection).
		navgen::ProbeJumpGaps(bsp, worldModel.headnode[navgen::kHullHuman], merged.areas, merged.portals);
		navgen::TagDropOffs(merged.areas, merged.portals);

		if (std::getenv("NAVGEN_DUMP_EDGES"))
		{
			for (const navgen::MergedPortal &p : merged.portals)
			{
				const char *typeName = p.edgeType == crossbot_nav::EDGE_WALK ? "WALK"
					: p.edgeType == crossbot_nav::EDGE_JUMP    ? "JUMP"
					: p.edgeType == crossbot_nav::EDGE_DROP    ? "DROP"
					: p.edgeType == crossbot_nav::EDGE_LADDER  ? "LADDER"
					: p.edgeType == crossbot_nav::EDGE_DOOR    ? "DOOR"
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

		int ladderEdges = 0, doorEdges = 0;
		for (const navgen::TaggedSpecialEdge &e : specialEdges)
			(e.type == crossbot_nav::EDGE_LADDER ? ladderEdges : doorEdges)++;
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

		std::printf("%s: %d ladder edges, %d door edges, %d jump edges, %d drop-off edges, %d elevator edges\n",
			bspPath.c_str(), ladderEdges, doorEdges, jumpEdges, dropEdges, elevatorEdges);

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

		if (!navPath.empty())
		{
			std::string mapName = DeriveMapName(bspPath);
			navgen::WriteNavFile(navPath, mapName, bsp.Checksum(), merged.areas, merged.portals, specialEdges,
				tacticalScores, crouchAreas);
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
