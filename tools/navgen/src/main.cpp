#include <cstdio>
#include <cstring>
#include <exception>
#include <stdexcept>
#include <string>

#include "area_merger.h"
#include "blocking_entities.h"
#include "bsp_file.h"
#include "edge_tagger.h"
#include "entity_parser.h"
#include "leaf_extractor.h"
#include "nav_file.h"
#include "portal_builder.h"
#include "reachability.h"

namespace
{

// GoldSrc maps are bounded to roughly +/-4096 units; this just needs to
// comfortably exceed any real map's extent for BaseWindingForPlane's huge
// starting quads.
constexpr double kMapRadius = 8192.0;

void PrintUsage(const char *argv0)
{
	std::fprintf(stderr,
		"usage: %s <map.bsp> [--dump-leaves] [--dump-areas] [--dump-obj <out.obj>] [-o <out.nav>]\n"
		"  --dump-leaves     print per-leaf (pre-merge) id/faces/volume/centroid stats\n"
		"  --dump-areas      print per-area (post-merge) id/faces/volume/centroid/portal stats\n"
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

		if (dumpLeaves)
		{
			for (const navgen::ExtractedLeaf &leaf : leaves)
			{
				navgen::Vec3 c = leaf.polytope.Centroid();
				std::printf("  leaf %4d: %2zu faces, volume %10.0f, centroid (%.1f, %.1f, %.1f)\n",
					leaf.id, leaf.polytope.faces.size(), leaf.polytope.Volume(), c.x, c.y, c.z);
			}
		}

		std::vector<navgen::Portal> portals = navgen::BuildPortals(leaves);
		navgen::MergeResult merged = navgen::MergeAreas(leaves, portals);

		std::printf("%s: %zu raw portals -> %zu areas, %zu inter-area portals\n",
			bspPath.c_str(), portals.size(), merged.areas.size(), merged.portals.size());

		std::vector<int> spawnAreas = navgen::FindSpawnAreas(entities, merged.areas);
		size_t areaCountBeforePrune = merged.areas.size();
		merged = navgen::PruneUnreachableAreas(merged.areas, merged.portals, spawnAreas);
		std::printf("%s: %zu spawn point(s) resolved, %zu areas reachable (%zu void/unreachable areas pruned)\n",
			bspPath.c_str(), spawnAreas.size(), merged.areas.size(), areaCountBeforePrune - merged.areas.size());

		std::vector<navgen::TaggedSpecialEdge> specialEdges = navgen::TagSpecialEdges(bsp, entities, merged.areas);
		size_t portalCountBeforeJumps = merged.portals.size();
		navgen::ProbeJumpGaps(merged.areas, merged.portals);
		navgen::TagDropOffs(merged.areas, merged.portals);

		int ladderEdges = 0, doorEdges = 0;
		for (const navgen::TaggedSpecialEdge &e : specialEdges)
			(e.type == crossbot_nav::EDGE_LADDER ? ladderEdges : doorEdges)++;
		int dropEdges = 0;
		for (const navgen::MergedPortal &p : merged.portals)
			if (p.edgeType == crossbot_nav::EDGE_DROP)
				++dropEdges;

		std::printf("%s: %d ladder edges, %d door edges, %zu jump edges, %d drop-off edges\n",
			bspPath.c_str(), ladderEdges, doorEdges, merged.portals.size() - portalCountBeforeJumps, dropEdges);

		if (dumpAreas)
		{
			for (const navgen::Area &area : merged.areas)
			{
				navgen::Vec3 c = area.polytope.Centroid();
				std::printf("  area %4d: %2zu leaves, %2zu faces, volume %10.0f, centroid (%.1f, %.1f, %.1f)\n",
					area.id, area.memberLeafIds.size(), area.polytope.faces.size(),
					area.polytope.Volume(), c.x, c.y, c.z);
			}
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
			navgen::WriteNavFile(navPath, mapName, bsp.Checksum(), merged.areas, merged.portals, specialEdges);
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
