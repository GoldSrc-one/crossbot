#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <random>
#include <stdexcept>
#include <string>
#include <unordered_set>

#include "area_merger.h"
#include "airmove_sampler.h"
#include "blocking_entities.h"
#include "bsp_file.h"
#include "clipnode_tree.h"
#include "astar_graph.h"
#include "entity_parser.h"
#include "floor_support.h"
#include "hull_defs.h"
#include "leaf_extractor.h"
#include "nav_file.h"
#include "portal_builder.h"
#include "reachability.h"
#include "stand_classifier.h"
#include "svg_debug.h"
#include "waypoint_plan.h"

namespace
{

constexpr double kMapRadius = 8192.0;

void PrintUsage(const char *argv0)
{
	std::fprintf(stderr,
		"usage: %s <map.bsp> [--dump-leaves] [--dump-areas] [--dump-planes <id>] [--dump-obj <out.obj>] [--dump-svg <out.svg>] [--airmove-samples <n>] [--no-merge] [--no-prune] [-o <out.nav>]\n"
		"  --dump-leaves     print per-leaf (pre-merge) id/faces/volume/centroid stats\n"
		"  --dump-areas      print per-area (post-merge) id/faces/volume/centroid/portal stats\n"
		"  --dump-conns      print every final connection: type, flags, endpoints, air-move parameters\n"
		"  --dump-planes <id> print one final area's exact plane list (normal+dist) and face windings\n"
		"  --dump-obj <path> write the walkable floor of every area as an OBJ mesh (plus a sidecar\n"
		"                    .mtl): crouch areas are coloured apart, the planned route is a ribbon\n"
		"  --dump-svg <path> write a top-down SVG of the final area/portal graph, for offline inspection\n"
		"  --airmove-samples <n> air-move launch samples per unit of edge length (default 1: a 128-unit\n"
		"                    edge gets 128 samples; 0.5 halves it; 0 disables air-move sampling)\n"
		"  --no-floor-split  keep leaves whose floor is not solid (debug: pre-floor-support behaviour)\n"
		"  --no-merge        skip MergeAreas (debug only): each BSP hull-3 leaf becomes its own area\n"
		"  --no-prune        skip PruneUnreachableAreas (debug only): keep areas unreachable from any spawn\n"
		"  --validate-areas  report any final area that is non-convex, overlaps another area, or holds\n"
		"                    an internal floor step taller than a bot can climb\n"
		"  -o <path>         write the baked .nav file (crossbot_nav format)\n",
		argv0);
}

// Area polytopes carry inward-facing plane normals, so the walkable floor of an area
// is the face whose normal points up. Same threshold as floor_support/airmove_sampler.
constexpr double kFloorNormalZ = 0.7;

struct ObjMaterial
{
	const char *name;
	double r, g, b;
};

const ObjMaterial kObjMaterials[] = {
	{"nav_floor", 0.35, 0.55, 0.78},
	{"nav_floor_crouch", 0.92, 0.58, 0.18},
	{"nav_path", 0.20, 0.85, 0.35},
};

std::string ReplaceExtension(const std::string &path, const char *ext)
{
	size_t slash = path.find_last_of("/\\");
	size_t dot = path.find_last_of('.');
	if (dot == std::string::npos || (slash != std::string::npos && dot < slash))
		return path + ext;
	return path.substr(0, dot) + ext;
}

std::string BaseName(const std::string &path)
{
	size_t slash = path.find_last_of("/\\");
	return slash == std::string::npos ? path : path.substr(slash + 1);
}

void WriteObjMaterialLib(const std::string &path)
{
	FILE *f = std::fopen(path.c_str(), "w");
	if (!f)
		throw std::runtime_error("cannot open output MTL file: " + path);

	for (const ObjMaterial &m : kObjMaterials)
	{
		std::fprintf(f, "newmtl %s\n", m.name);
		std::fprintf(f, "Ka %.3f %.3f %.3f\n", m.r * 0.3, m.g * 0.3, m.b * 0.3);
		std::fprintf(f, "Kd %.3f %.3f %.3f\n", m.r, m.g, m.b);
		std::fprintf(f, "Ks 0.000 0.000 0.000\nd 1.0\nillum 1\n\n");
	}

	std::fclose(f);
}

// True when the stored vertex order winds the other way round the plane normal, in which
// case the OBJ face has to be reversed so it renders facing up rather than down.
bool WindingOpposesNormal(const navgen::Face &face)
{
	navgen::Vec3 sum(0, 0, 0);
	for (size_t i = 1; i + 1 < face.winding.size(); ++i)
	{
		navgen::Vec3 a = face.winding[i] - face.winding[0];
		navgen::Vec3 b = face.winding[i + 1] - face.winding[0];
		sum = sum + a.Cross(b);
	}
	return sum.Dot(face.plane.normal) < 0.0;
}

struct ObjStats
{
	size_t floorFaces = 0;
	size_t crouchAreas = 0;
	size_t pathSegments = 0;
};

ObjStats WriteFloorObj(const std::string &path, const std::vector<navgen::Area> &areas,
	const std::vector<navgen::Vec3> &pathPoints)
{
	std::string mtlPath = ReplaceExtension(path, ".mtl");
	WriteObjMaterialLib(mtlPath);

	FILE *f = std::fopen(path.c_str(), "w");
	if (!f)
		throw std::runtime_error("cannot open output OBJ file: " + path);

	std::fprintf(f, "mtllib %s\n", BaseName(mtlPath).c_str());

	ObjStats stats;
	int nextVertexIndex = 1;

	for (const navgen::Area &area : areas)
	{
		bool crouch = (area.areaFlags & crossbot_nav::NAV_AREA_CROUCH) != 0;
		bool opened = false;

		for (const navgen::Face &face : area.polytope.faces)
		{
			if (face.plane.normal.z <= kFloorNormalZ || face.winding.size() < 3)
				continue;

			if (!opened)
			{
				std::fprintf(f, "g area_%d%s\nusemtl %s\n", area.id, crouch ? "_crouch" : "",
					crouch ? "nav_floor_crouch" : "nav_floor");
				opened = true;
				if (crouch)
					++stats.crouchAreas;
			}

			for (const navgen::Vec3 &v : face.winding)
				std::fprintf(f, "v %.3f %.3f %.3f\n", v.x, v.y, v.z);

			bool flip = WindingOpposesNormal(face);
			size_t count = face.winding.size();
			std::fprintf(f, "f");
			for (size_t i = 0; i < count; ++i)
			{
				size_t idx = flip ? count - 1 - i : i;
				std::fprintf(f, " %d", nextVertexIndex + static_cast<int>(idx));
			}
			std::fprintf(f, "\n");

			nextVertexIndex += static_cast<int>(count);
			++stats.floorFaces;
		}
	}

	// The planned route as a flat ribbon: waypoints sit at area centroid height, well
	// above the floor polygons, so it reads as a line hovering over the walkable set.
	if (pathPoints.size() >= 2)
	{
		constexpr double kPathHalfWidth = 3.0;

		std::fprintf(f, "g path\nusemtl nav_path\n");

		for (size_t i = 0; i + 1 < pathPoints.size(); ++i)
		{
			const navgen::Vec3 &a = pathPoints[i];
			const navgen::Vec3 &b = pathPoints[i + 1];

			navgen::Vec3 dir = b - a;
			navgen::Vec3 side(-dir.y, dir.x, 0.0);
			double len = side.Length();
			if (len < 1e-6)
				side = navgen::Vec3(kPathHalfWidth, 0.0, 0.0);
			else
				side = side * (kPathHalfWidth / len);

			const navgen::Vec3 quad[4] = {a - side, b - side, b + side, a + side};
			for (const navgen::Vec3 &v : quad)
				std::fprintf(f, "v %.3f %.3f %.3f\n", v.x, v.y, v.z);

			std::fprintf(f, "f %d %d %d %d\n", nextVertexIndex, nextVertexIndex + 1,
				nextVertexIndex + 2, nextVertexIndex + 3);

			nextVertexIndex += 4;
			++stats.pathSegments;
		}
	}

	std::fclose(f);
	return stats;
}

struct ExportRegion
{
	std::vector<navgen::Plane> planes;
	std::vector<navgen::Vec3> verts;
};

ExportRegion BuildExportRegion(const navgen::Area &area)
{
	ExportRegion region;
	region.planes = area.boundPlanes;

	navgen::Vec3 mn(1e30, 1e30, 1e30), mx(-1e30, -1e30, -1e30);
	for (const navgen::Face &f : area.polytope.faces)
	{
		for (const navgen::Vec3 &v : f.winding)
		{
			mn.z = std::min(mn.z, v.z);
			mx.z = std::max(mx.z, v.z);
		}
	}

	for (const navgen::Face &f : area.polytope.faces)
	{
		for (const navgen::Vec3 &v : f.winding)
		{
			if (area.prism)
			{
				region.verts.push_back(navgen::Vec3(v.x, v.y, mn.z));
				region.verts.push_back(navgen::Vec3(v.x, v.y, mx.z));
			}
			else
			{
				region.verts.push_back(v);
			}
		}
	}

	return region;
}

double WorstConcavity(const ExportRegion &region)
{
	double worst = 0.0;
	for (const navgen::Plane &p : region.planes)
		for (const navgen::Vec3 &v : region.verts)
			worst = std::max(worst, -(v.Dot(p.normal) - p.dist));
	return worst;
}

bool SeparatedBy(const ExportRegion &a, const ExportRegion &b, double epsilon)
{
	for (const navgen::Plane &p : a.planes)
	{
		bool allOutside = true;
		for (const navgen::Vec3 &v : b.verts)
		{
			if (v.Dot(p.normal) - p.dist > epsilon)
			{
				allOutside = false;
				break;
			}
		}
		if (allOutside)
			return true;
	}
	return false;
}

double WorstInternalStep(const navgen::Area &area)
{
	const double kAdjacencyEpsilon = 0.5;

	std::vector<const navgen::Face *> floors;
	for (const navgen::Face &f : area.polytope.faces)
		if (f.plane.normal.z > kFloorNormalZ && f.winding.size() >= 3)
			floors.push_back(&f);

	double worst = 0.0;
	for (const navgen::Face *f : floors)
	{
		for (const navgen::Face *g : floors)
		{
			if (f == g)
				continue;
			for (const navgen::Vec3 &v : g->winding)
			{
				if (navgen::Distance2DToWinding(f->winding, v.x, v.y) > kAdjacencyEpsilon)
					continue;
				worst = std::max(worst, std::fabs(navgen::FloorHeightAt(*f, v.x, v.y) - v.z));
			}
		}
	}
	return worst;
}

void ValidateAreas(const std::vector<navgen::Area> &areas)
{
	const double kConcaveTolerance = 1.0;
	const double kOverlapTolerance = 1.0;

	std::vector<ExportRegion> regions;
	regions.reserve(areas.size());
	for (const navgen::Area &area : areas)
		regions.push_back(BuildExportRegion(area));

	size_t prismCount = 0;
	size_t concaveCount = 0;
	size_t steepCount = 0;
	double worstStep = 0.0;
	for (size_t i = 0; i < areas.size(); ++i)
	{
		if (areas[i].prism)
			++prismCount;

		double step = WorstInternalStep(areas[i]);
		worstStep = std::max(worstStep, step);
		if (step > navgen::kStepHeight + kConcaveTolerance)
		{
			++steepCount;
			std::printf("  UNCLIMBABLE area %d: internal floor step of %.2f units\n", areas[i].id, step);
		}

		double worst = WorstConcavity(regions[i]);
		if (worst > kConcaveTolerance)
		{
			++concaveCount;
			std::printf("  NON-CONVEX area %d: %zu leaves, %zu planes, vertex %.2f units behind a bound plane\n",
				areas[i].id, areas[i].memberLeafIds.size(), regions[i].planes.size(), worst);
		}
	}

	size_t overlapCount = 0;
	for (size_t i = 0; i < areas.size(); ++i)
	{
		for (size_t j = i + 1; j < areas.size(); ++j)
		{
			if (SeparatedBy(regions[i], regions[j], kOverlapTolerance))
				continue;
			if (SeparatedBy(regions[j], regions[i], kOverlapTolerance))
				continue;
			++overlapCount;
			if (overlapCount <= 20)
				std::printf("  OVERLAP area %d <-> area %d\n", areas[i].id, areas[j].id);
		}
	}

	std::printf("validate: %zu areas (%zu prism), %zu non-convex, %zu overlapping pairs, "
		"%zu with unclimbable internal steps (worst %.2f units)\n",
		areas.size(), prismCount, concaveCount, overlapCount, steepCount, worstStep);
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

std::vector<navgen::Vec3> CollectTargetPoints(const std::vector<navgen::Entity> &entities)
{
	std::vector<navgen::Vec3> points;
	for (const navgen::Entity &ent : entities)
	{
		if (ent.Get("classname") != "info_target")
			continue;
		double x = 0, y = 0, z = 0;
		if (std::sscanf(ent.Get("origin").c_str(), "%lf %lf %lf", &x, &y, &z) != 3)
			continue;
		points.push_back(navgen::Vec3(x, y, z));
	}
	return points;
}

Vector ClampToAreaBounds(const Vector &point, const navgen::Polytope &poly)
{
	navgen::Vec3 mins(1e9, 1e9, 1e9), maxs(-1e9, -1e9, -1e9);
	for (const navgen::Face &f : poly.faces)
	{
		for (const navgen::Vec3 &v : f.winding)
		{
			mins.x = std::min(mins.x, v.x); mins.y = std::min(mins.y, v.y); mins.z = std::min(mins.z, v.z);
			maxs.x = std::max(maxs.x, v.x); maxs.y = std::max(maxs.y, v.y); maxs.z = std::max(maxs.z, v.z);
		}
	}

	auto clamp = [](float v, double lo, double hi)
	{
		return static_cast<float>(std::max(lo, std::min(hi, static_cast<double>(v))));
	};

	return Vector(clamp(point.x, mins.x, maxs.x), clamp(point.y, mins.y, maxs.y), clamp(point.z, mins.z, maxs.z));
}

std::vector<navgen::Vec3> BuildDebugPath(const std::vector<navgen::Entity> &entities,
	const navgen::MergeResult &merged)
{
	std::vector<navgen::Vec3> points;
	if (merged.areas.empty())
		return points;

	std::vector<navgen::Vec3> spawnPoints;
	std::vector<int> spawnAreas = navgen::FindSpawnAreas(entities, merged.areas, &spawnPoints);
	if (spawnAreas.empty())
		return points;

	auto ToVector = [](const navgen::Vec3 &v)
	{
		return Vector(static_cast<float>(v.x), static_cast<float>(v.y), static_cast<float>(v.z));
	};

	std::vector<NavAStar::AreaNode> nodes(merged.areas.size());
	std::vector<NavPlan::PlanArea> planAreas(merged.areas.size());
	for (size_t i = 0; i < merged.areas.size(); ++i)
	{
		Vector centroid = ToVector(merged.areas[i].polytope.Centroid());
		nodes[i].centroid = centroid;
		planAreas[i].centroid = centroid;
		planAreas[i].crouch = (merged.areas[i].areaFlags & crossbot_nav::NAV_AREA_CROUCH) != 0;
		planAreas[i].door = (merged.areas[i].areaFlags & crossbot_nav::NAV_AREA_DOOR) != 0;
	}

	std::vector<NavAStar::Conn> graphConns(merged.conns.size());
	std::vector<NavPlan::PlanConnection> planConns(merged.conns.size());
	std::vector<std::vector<uint32_t>> areaConnIndices(merged.areas.size());
	for (size_t i = 0; i < merged.conns.size(); ++i)
	{
		const navgen::MergedConnection &c = merged.conns[i];
		Vector pointA = ToVector(c.pointA);
		Vector pointB = ToVector(c.pointB);

		NavAStar::Conn &gc = graphConns[i];
		gc.areaA = static_cast<uint32_t>(c.areaA);
		gc.areaB = static_cast<uint32_t>(c.areaB);
		gc.oneWay = (c.connFlags & crossbot_nav::CONN_FLAG_ONE_WAY) != 0;
		gc.traverseCost = navgen::TraverseCostFor(c);
		gc.pointA = pointA;
		gc.pointB = pointB;

		NavPlan::PlanConnection &pc = planConns[i];
		pc.areaA = gc.areaA;
		pc.areaB = gc.areaB;
		pc.pointA = pointA;
		pc.pointB = pointB;
		pc.connType = c.connType;

		areaConnIndices[static_cast<size_t>(c.areaA)].push_back(static_cast<uint32_t>(i));
		areaConnIndices[static_cast<size_t>(c.areaB)].push_back(static_cast<uint32_t>(i));
	}

	uint32_t startArea = static_cast<uint32_t>(spawnAreas.front());
	Vector startPoint = ToVector(spawnPoints.front());

	std::vector<navgen::Vec3> targetPoints;
	std::vector<int> targetAreas = navgen::FindTargetAreas(entities, merged.areas, &targetPoints);
	uint32_t goalArea = startArea;
	bool goalFromTarget = false;
	Vector goalPoint = nodes[startArea].centroid;
	if (!targetAreas.empty() && targetAreas.front() != static_cast<int>(startArea))
	{
		goalArea = static_cast<uint32_t>(targetAreas.front());
		goalPoint = ToVector(targetPoints.front());
		goalFromTarget = true;
	}

	if (!goalFromTarget)
	{
		double best = -1.0;
		for (size_t i = 0; i < merged.areas.size(); ++i)
		{
			navgen::Vec3 a = merged.areas[startArea].polytope.Centroid();
			navgen::Vec3 b = merged.areas[i].polytope.Centroid();
			double d = (b - a).Length();
			if (d > best)
			{
				best = d;
				goalArea = static_cast<uint32_t>(i);
			}
		}
		goalPoint = nodes[goalArea].centroid;
	}

	if (goalArea == startArea)
		return points;

	NavAStar::GraphResult result = NavAStar::FindPath(nodes, areaConnIndices, graphConns, startArea, goalArea);
	if (!result.found || result.connPath.empty())
	{
		std::printf("debug path: no route from area %u to area %u\n", startArea, goalArea);
		return points;
	}

	goalPoint = ClampToAreaBounds(goalPoint, merged.areas[goalArea].polytope);

	std::vector<NavPlan::PlanWaypoint> plan = NavPlan::BuildWaypointPlan(planAreas, planConns, result.areaPath,
		result.connPath, startPoint, goalPoint);

	for (const NavPlan::PlanWaypoint &wp : plan)
	{
		navgen::Vec3 p(wp.point.x, wp.point.y, wp.point.z);
		if (!points.empty() && (p - points.back()).Length() < 0.5)
			continue;
		points.push_back(p);
	}

	std::printf("debug path: area %u -> %u via %zu connections, %zu waypoints\n", startArea, goalArea,
		result.connPath.size(), points.size());

	return points;
}

}

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
	bool dumpConns = false;
	bool noFloorSplit = false;
	bool dumpLadders = false;
	bool noMerge = false;
	bool noPrune = false;
	bool validateAreas = false;
	float airmoveSamplesPerUnit = navgen::AirmoveSampleParams().samplesPerUnit;
	int dumpPlanesAreaId = -1;
	std::string objPath;
	std::string svgPath;
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
		else if (arg == "--dump-conns")
		{
			dumpConns = true;
		}
		else if (arg == "--dump-ladders")
		{
			dumpLadders = true;
		}
		else if (arg == "--no-floor-split")
		{
			noFloorSplit = true;
		}
		else if (arg == "--no-merge")
		{
			noMerge = true;
		}
		else if (arg == "--no-prune")
		{
			noPrune = true;
		}
		else if (arg == "--validate-areas")
		{
			validateAreas = true;
		}
		else if (arg == "--airmove-samples")
		{
			if (i + 1 >= argc)
			{
				PrintUsage(argv[0]);
				return 1;
			}
			const char *value = argv[++i];
			char *parseEnd = nullptr;
			double parsed = std::strtod(value, &parseEnd);
			if (parseEnd == value || *parseEnd != '\0' || parsed < 0.0)
			{
				std::fprintf(stderr, "--airmove-samples: expected a non-negative number, got \"%s\"\n", value);
				return 1;
			}
			airmoveSamplesPerUnit = static_cast<float>(parsed);
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

		std::vector<navgen::ExtractedLeaf> leaves =
			navgen::ExtractHullLeaves(bsp, worldModel, kMapRadius, navgen::kHullDuck);

		std::vector<navgen::Entity> entities = navgen::ParseEntities(bsp.EntityText());
		std::vector<navgen::BlockingVolume> blockers =
			navgen::ExtractBlockingVolumes(bsp, entities, navgen::kHullDuck);
		std::vector<navgen::BlockingVolume> standBlockers =
			navgen::ExtractBlockingVolumes(bsp, entities, navgen::kHullHuman);

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

		std::vector<navgen::ExtractedLeaf> navigableLeaves;
		navigableLeaves.reserve(leaves.size());
		size_t affectedLeaves = 0;
		size_t fullyRemovedLeaves = 0;
		size_t splitLeaves = 0;
		for (navgen::ExtractedLeaf &leaf : leaves)
		{
			if (!navgen::OverlapsAnyBlockingVolume(leaf, blockers))
			{
				leaf.id = static_cast<int>(navigableLeaves.size());
				navigableLeaves.push_back(std::move(leaf));
				continue;
			}

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

		if (!noFloorSplit)
		{
			size_t unsupportedDropped = 0;
			size_t floorSplitLeaves = 0;
			size_t noFloorDropped = 0;
			size_t beforeFloorSplit = leaves.size();
			leaves = navgen::SplitLeavesByFloorSupport(bsp, worldModel.headnode[navgen::kHullDuck], blockers,
				leaves, kMapRadius, &unsupportedDropped, &floorSplitLeaves, &noFloorDropped);
			std::printf("%s: floor support split %zu leaves, dropped %zu unsupported and %zu steep-bottomed "
				"pieces (%zu -> %zu leaves)\n",
				bspPath.c_str(), floorSplitLeaves, unsupportedDropped, noFloorDropped, beforeFloorSplit,
				leaves.size());
		}

		std::vector<bool> isCrouchLeaf;
		{
			size_t standSplitLeaves = 0;
			size_t beforeStandSplit = leaves.size();
			leaves = navgen::SplitLeavesByStandClearance(bsp, worldModel.headnode[navgen::kHullHuman],
				standBlockers, leaves, kMapRadius, isCrouchLeaf, &standSplitLeaves);
			size_t crouchLeaves = 0;
			for (bool crouch : isCrouchLeaf)
				if (crouch)
					++crouchLeaves;
			std::printf("%s: stand clearance split %zu leaves (%zu -> %zu leaves), %zu crouch-only\n",
				bspPath.c_str(), standSplitLeaves, beforeStandSplit, leaves.size(), crouchLeaves);
		}

		double totalLeafVolume = 0.0;
		for (const navgen::ExtractedLeaf &leaf : leaves)
			totalLeafVolume += leaf.polytope.Volume();

		std::printf("%s: %zu hull-3 empty leaves (%zu affected by %zu blocking entities: %zu fully removed, %zu split into "
			"multiple pieces), total volume %.0f cu.units\n",
			bspPath.c_str(), leaves.size(), affectedLeaves, blockers.size(), fullyRemovedLeaves, splitLeaves, totalLeafVolume);

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

		navgen::MergeResult merged = noMerge
			? navgen::NoMerge(leaves, portals, isCrouchLeaf)
			: navgen::MergeAreas(leaves, portals, isCrouchLeaf);

		std::printf("%s: %zu raw portals -> %zu areas, %zu inter-area connections (%s)\n",
			bspPath.c_str(), portals.size(), merged.areas.size(), merged.conns.size(),
			noMerge ? "NoMerge" : "MergeAreas");

		size_t steepWalkConns = navgen::FilterWalkConnectionsByStep(merged.areas, merged.conns, navgen::kStepHeight);
		if (steepWalkConns > 0)
			std::printf("%s: dropped %zu walk connections exceeding %.0f-unit step height\n",
				bspPath.c_str(), steepWalkConns, navgen::kStepHeight);

		if (validateAreas)
			ValidateAreas(merged.areas);

		navgen::PhysicsParams physics;
		navgen::AirmoveSampleParams airmoveParams;
		airmoveParams.samplesPerUnit = airmoveSamplesPerUnit;
		if (airmoveSamplesPerUnit <= 0.0f)
			std::printf("air move sampling skipped (--airmove-samples 0)\n");
		else
		{
			navgen::HullContext duckHull{worldModel.headnode[navgen::kHullDuck], &blockers};
			navgen::HullContext standHull{worldModel.headnode[navgen::kHullHuman], &standBlockers};
			navgen::ProbeAirMoves(bsp, duckHull, standHull, merged.areas, merged.conns, physics,
				airmoveParams);
		}

		size_t rawAreaCount = merged.areas.size();
		std::vector<int> spawnAreas = navgen::FindSpawnAreas(entities, merged.areas);
		if (noPrune)
		{
			std::printf("%s: %zu spawn point(s) resolved, %zu areas kept (--no-prune)\n",
				bspPath.c_str(), spawnAreas.size(), merged.areas.size());
		}
		else
		{
			merged = navgen::PruneUnreachableAreas(merged.areas, merged.conns, spawnAreas);
			std::printf("%s: %zu spawn point(s) resolved, %zu areas reachable after first pass (%zu void areas pruned)\n",
				bspPath.c_str(), spawnAreas.size(), merged.areas.size(), rawAreaCount - merged.areas.size());
		}

		int walkConns = 0, airMoveConns = 0;
		for (const navgen::MergedConnection &c : merged.conns)
		{
			if (c.connType == crossbot_nav::CONN_WALK)
				++walkConns;
			else if (c.connType == crossbot_nav::CONN_AIRMOVE)
				++airMoveConns;
		}

		std::printf("%s: %d walk connections, %d air move connections\n",
			bspPath.c_str(), walkConns, airMoveConns);

		if (dumpConns)
		{
			for (const navgen::MergedConnection &c : merged.conns)
			{
				const char *typeName = (c.connType == crossbot_nav::CONN_AIRMOVE) ? "airmove"
					: (c.connType == crossbot_nav::CONN_TELEPORT) ? "teleport" : "walk";
				bool oneWay = (c.connFlags & crossbot_nav::CONN_FLAG_ONE_WAY) != 0;

				std::printf("  %-8s %3d %s %3d  from (%.1f, %.1f, %.1f) to (%.1f, %.1f, %.1f)",
					typeName, c.areaA, oneWay ? "->" : "<->", c.areaB,
					c.pointA.x, c.pointA.y, c.pointA.z, c.pointB.x, c.pointB.y, c.pointB.z);

				if (c.connFlags & crossbot_nav::CONN_FLAG_NEEDS_CROUCH)
					std::printf("  [crouch]");

				if (c.connType == crossbot_nav::CONN_AIRMOVE)
					std::printf("  speed %.0f yaw %.1f jumpBack %.1f fallDamage %.1f flightTime %.2f",
						c.exitSpeed, c.exitAngle, c.jumpBackDist, c.fallDamage, c.flightTime);

				std::printf("\n");
			}
		}

		if (dumpAreas)
		{
			for (const navgen::Area &area : merged.areas)
			{
				std::string areaKind;
				if (area.areaFlags & crossbot_nav::NAV_AREA_CROUCH)
					areaKind += " crouch";
				if (area.prism)
					areaKind += " prism";
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
					"mins (%.1f, %.1f, %.1f) maxs (%.1f, %.1f, %.1f)%s\n",
					area.id, area.memberLeafIds.size(), area.polytope.faces.size(),
					area.polytope.Volume(), c.x, c.y, c.z, mn.x, mn.y, mn.z, mx.x, mx.y, mx.z, areaKind.c_str());
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
				std::printf("area %d: %zu exported bound planes%s\n", area.id, area.boundPlanes.size(),
					area.prism ? " (prism)" : "");
				for (const navgen::Plane &p : area.boundPlanes)
					std::printf("  bound: normal (%.6f, %.6f, %.6f) dist %.3f\n",
						p.normal.x, p.normal.y, p.normal.z, p.dist);

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

		std::vector<navgen::Vec3> pathPoints;
		if (!objPath.empty() || !svgPath.empty())
			pathPoints = BuildDebugPath(entities, merged);

		if (!objPath.empty())
		{
			ObjStats stats = WriteFloorObj(objPath, merged.areas, pathPoints);
			std::printf("wrote %s (+ %s): %zu floor faces, %zu crouch areas, %zu path segments\n",
				objPath.c_str(), BaseName(ReplaceExtension(objPath, ".mtl")).c_str(),
				stats.floorFaces, stats.crouchAreas, stats.pathSegments);
		}

		if (!svgPath.empty())
		{
			std::string mapName = DeriveMapName(bspPath);
			navgen::WriteDebugSvg(svgPath, mapName, merged.areas, merged.conns, pathPoints,
				CollectTargetPoints(entities));
			std::printf("wrote %s\n", svgPath.c_str());
		}

		if (!navPath.empty())
		{
			std::string mapName = DeriveMapName(bspPath);
			navgen::WriteNavFile(navPath, mapName, bsp.Checksum(), merged.areas, merged.conns);
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

