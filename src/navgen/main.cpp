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
#include "path_profile.h"
#include "waypoint_plan.h"

namespace
{

constexpr double kMapRadius = 8192.0;

void PrintUsage(const char *argv0)
{
	std::fprintf(stderr,
		"usage: %s <map.bsp> [--dump-leaves] [--dump-areas] [--dump-planes <id>] [--dump-obj <out.obj>] [--dump-svg <out.svg>] [--trace <trace.txt>] [--airmove-samples <n>] [--no-merge] [--no-prune] [-o <out.nav>]\n"
		"  --dump-leaves     print per-leaf (pre-merge) id/faces/volume/centroid stats\n"
		"  --dump-areas      print per-area (post-merge) id/faces/volume/centroid/portal stats\n"
		"  --dump-conns      print every final connection: type, flags, endpoints, air-move parameters\n"
		"  --dump-planes <id> print one final area's exact plane list (normal+dist) and face windings\n"
		"  --dump-obj <path> write the walkable floor of every area as an OBJ mesh (plus a sidecar\n"
		"                    .mtl): crouch areas are coloured apart, the planned route is a ribbon\n"
		"  --dump-svg <path> write a top-down SVG of the final area/portal graph, for offline inspection\n"
		"  --trace <path>    overlay a recorded bot trajectory on the SVG: one \"x y z\" per line, as\n"
		"                    the plugin's crossbot_trace_path writes it. An empty file draws nothing\n"
		"  --airmove-samples <n> air-move launch samples per unit of edge length (default 1: a 128-unit\n"
		"                    edge gets 128 samples; 0.5 halves it; 0 disables air-move sampling)\n"
		"  --no-floor-split  keep leaves whose floor is not solid (debug: pre-floor-support behaviour)\n"
		"  --no-merge        skip MergeAreas (debug only): each BSP hull-3 leaf becomes its own area\n"
		"  --no-prune        skip PruneUnreachableAreas (debug only): keep areas unreachable from any spawn\n"
		"  -o <path>         write the baked .nav file (crossbot_nav format)\n",
		argv0);
}

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

std::vector<navgen::Vec3> ReadTraceFile(const std::string &path)
{
	std::vector<navgen::Vec3> points;

	FILE *f = std::fopen(path.c_str(), "r");
	if (!f)
	{
		std::fprintf(stderr, "--trace: cannot open %s, drawing no trace\n", path.c_str());
		return points;
	}

	char line[256];
	while (std::fgets(line, sizeof(line), f))
	{
		double x = 0, y = 0, z = 0;
		if (std::sscanf(line, "%lf %lf %lf", &x, &y, &z) == 3)
			points.push_back(navgen::Vec3(x, y, z));
	}

	std::fclose(f);
	return points;
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

void ReportTraceDeviation(const NavPlan::PlanProfile &profile,
	const std::vector<navgen::Vec3> &trace)
{
	if (profile.Empty() || trace.empty())
		return;

	double sum = 0.0;
	float worst = 0.f;
	float worstS = 0.f;
	float s = 0.f;
	size_t counted = 0;

	for (const navgen::Vec3 &p : trace)
	{
		Vector pos((float)p.x, (float)p.y, (float)p.z);
		float lateral = 0.f;
		s = NavPlan::ProjectOntoProfile(profile, pos, s - 32.f, s + 128.f, &lateral);

		sum += lateral;
		++counted;
		if (lateral > worst)
		{
			worst = lateral;
			worstS = s;
		}
	}

	NavPlan::ProfileSample at = NavPlan::SampleProfile(profile, worstS);
	std::printf("trace deviation: mean %.1f, max %.1f at s=%.0f (wp %zu, clearance %.1f)\n",
		counted ? sum / (double)counted : 0.0, worst, worstS, at.wpIndex, at.clearance);
}

std::vector<navgen::Vec3> BuildDebugPath(const std::vector<navgen::Entity> &entities,
	const navgen::MergeResult &merged, const NavPlan::PlanTraceContext *traces,
	std::vector<uint8_t> *segmentKindsOut, std::vector<navgen::FailedAirmove> *failedAirmovesOut,
	std::vector<float> *segmentSpeedsOut, bool dumpPath, NavPlan::PlanProfile *profileOut)
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
		planAreas[i].breakable = (merged.areas[i].areaFlags & crossbot_nav::NAV_AREA_BREAKABLE) != 0;
		planAreas[i].doorActivation = merged.areas[i].doorActivation;
		planAreas[i].doorActivatorPos = ToVector(merged.areas[i].doorActivatorPos);
		planAreas[i].doorWaitTime = static_cast<float>(merged.areas[i].openSeconds);

		planAreas[i].boundPlanes.reserve(merged.areas[i].boundPlanes.size());
		for (const navgen::Plane &p : merged.areas[i].boundPlanes)
		{
			NavPlan::PlanPlane pp;
			pp.normal = Vector(static_cast<float>(p.normal.x), static_cast<float>(p.normal.y),
				static_cast<float>(p.normal.z));
			pp.dist = static_cast<float>(p.dist);
			planAreas[i].boundPlanes.push_back(pp);
		}
	}

	std::vector<NavAStar::Connection> graphConns(merged.conns.size());
	std::vector<NavPlan::PlanConnection> planConns(merged.conns.size());
	std::vector<std::vector<uint32_t>> areaConnIndices(merged.areas.size());
	for (size_t i = 0; i < merged.conns.size(); ++i)
	{
		const navgen::MergedConnection &c = merged.conns[i];
		Vector pointA = ToVector(c.pointA);
		Vector pointB = ToVector(c.pointB);

		NavAStar::Connection &gc = graphConns[i];
		gc.areaA = static_cast<uint32_t>(c.areaA);
		gc.areaB = static_cast<uint32_t>(c.areaB);
		gc.oneWay = (c.connFlags & crossbot_nav::CONN_FLAG_ONE_WAY) != 0;
		gc.costSeconds = navgen::ConnectionCostSeconds(c, merged.areas);

		NavPlan::PlanConnection &pc = planConns[i];
		pc.areaA = gc.areaA;
		pc.areaB = gc.areaB;
		pc.connType = c.connType;
		pc.gateA.valid = c.gateAValid;
		pc.gateA.clamped = c.gateAClamped;
		pc.gateA.left = ToVector(c.gateALeft);
		pc.gateA.right = ToVector(c.gateARight);
		pc.gateB.valid = c.gateBValid;
		pc.gateB.clamped = c.gateBClamped;
		pc.gateB.left = ToVector(c.gateBLeft);
		pc.gateB.right = ToVector(c.gateBRight);
		for (int k = 0; k < 2; ++k)
		{
			pc.gateAYaw[k] = static_cast<float>(c.gateAYaw[k]);
			pc.gateASpeed[k] = static_cast<float>(c.gateASpeed[k]);
		}
		pc.connFlags = c.connFlags;
		pc.exitSpeed = static_cast<float>(c.exitSpeed);
		pc.exitAngle = static_cast<float>(c.exitAngle);
		pc.jumpBackDist = static_cast<float>(c.jumpBackDist);
		pc.flightTime = static_cast<float>(c.flightTime);
		pc.launchPos = pointA;
		pc.landingPos = pointB;

		areaConnIndices[static_cast<size_t>(c.areaA)].push_back(static_cast<uint32_t>(i));
		areaConnIndices[static_cast<size_t>(c.areaB)].push_back(static_cast<uint32_t>(i));
	}

	for (size_t i = 0; i < planConns.size(); ++i)
		graphConns[i].crossing = NavPlan::ConnectionCrossing(planAreas, planConns[i]);

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

	NavPlan::PlanStats planStats;
	std::vector<NavPlan::PlanWaypoint> plan = NavPlan::BuildWaypointPlan(planAreas, planConns, result.areaPath,
		result.connPath, startPoint, goalPoint, traces, &planStats);

	NavPlan::PlanProfile profile = NavPlan::BuildPathProfile(plan,
		traces ? traces->physics : PmSim::PmPhysics(), traces);

	if (profileOut)
		*profileOut = profile;

	auto PushPoint = [&](const Vector &v, uint8_t kind, float speed)
	{
		navgen::Vec3 p(v.x, v.y, v.z);
		if (!points.empty() && (p - points.back()).Length() < 0.5)
		{
			if (segmentKindsOut && !segmentKindsOut->empty() && kind != navgen::PATH_SEG_WALK)
				segmentKindsOut->back() = kind;
			return;
		}
		points.push_back(p);
		if (segmentKindsOut)
			segmentKindsOut->push_back(kind);
		if (segmentSpeedsOut)
			segmentSpeedsOut->push_back(speed);
	};

	for (const NavPlan::ProfileSeg &seg : profile.segs)
	{
		const NavPlan::PlanWaypoint &startWp = plan[std::min<size_t>(seg.wpIndex - 1, plan.size() - 1)];

		uint8_t kind = navgen::PATH_SEG_WALK;
		if (seg.gap && startWp.airmoveLaunch)
			kind = startWp.launchSimulated ? navgen::PATH_SEG_AIRMOVE_SIMULATED
				: navgen::PATH_SEG_AIRMOVE_STORED;

		PushPoint(seg.a, kind, seg.vEntry);

		if (seg.isArc)
		{
			int steps = std::max(2, static_cast<int>(std::fabs(seg.turn) * 8.f));
			for (int k = 1; k < steps; ++k)
			{
				float t = static_cast<float>(k) / static_cast<float>(steps);
				NavPlan::ProfileSample s = NavPlan::SampleProfile(profile, seg.sStart + seg.sLen * t);
				PushPoint(s.point, kind, s.speed);
			}
		}
	}

	if (!profile.segs.empty())
		PushPoint(profile.segs.back().b, navgen::PATH_SEG_WALK, profile.segs.back().vExit);

	for (const NavPlan::PlanWaypoint &wp : plan)
	{
		if (failedAirmovesOut && wp.launchSimFailed)
		{
			navgen::FailedAirmove fa;
			fa.launch = navgen::Vec3(wp.launchSimFrom.x, wp.launchSimFrom.y, wp.launchSimFrom.z);
			fa.landing = navgen::Vec3(wp.launchSimTo.x, wp.launchSimTo.y, wp.launchSimTo.z);
			fa.aim = navgen::Vec3(wp.launchSimAim.x, wp.launchSimAim.y, wp.launchSimAim.z);
			failedAirmovesOut->push_back(fa);
		}
	}

	size_t offMesh = 0;
	{
		auto overSomeArea = [&](const Vector &p)
		{
			for (const NavPlan::PlanArea &a : planAreas)
			{
				if (a.boundPlanes.empty())
					continue;
				bool inside = true;
				for (const NavPlan::PlanPlane &pl : a.boundPlanes)
				{
					if (std::fabs(pl.normal.z) > 0.3f)
						continue;
					if (p.x * pl.normal.x + p.y * pl.normal.y - pl.dist < -0.05f)
					{
						inside = false;
						break;
					}
				}
				if (inside)
					return true;
			}
			return false;
		};

		for (size_t i = 0; i + 1 < plan.size(); ++i)
		{
			if (plan[i].airmoveLaunch || plan[i].requiresTeleport)
				continue;
			const Vector &a = plan[i].point;
			const Vector &b = plan[i + 1].point;
			for (int s = 1; s <= 16; ++s)
			{
				float t = static_cast<float>(s) / 17.0f;
				if (!overSomeArea(Vector(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, 0.f)))
				{
					++offMesh;
					break;
				}
			}
		}
	}

	std::printf("debug path: area %u -> %u via %zu connections, %zu waypoints "
		"(%zu air moves re-simulated, %zu from stored, %zu re-speeded, %zu unplannable; %zu corners, "
		"%zu gates short of full clearance, %zu segments leaving the mesh)\n",
		startArea, goalArea, result.connPath.size(), plan.size(), planStats.airmovesSimulated,
		planStats.airmovesFromStored, planStats.airmovesRespeeded, planStats.airmovesUnplannable,
		planStats.corners, planStats.gatesClamped, offMesh);

	std::printf("debug profile: %zu segments, %.0f units, %.1fs, %zu corners arced, "
		"%zu slowed, %zu shrunk by trace\n",
		profile.segs.size(), profile.totalLength, profile.totalTime, profile.cornersArced,
		profile.cornersSlowed, profile.cornersShrunk);

	if (dumpPath)
	{
		for (size_t i = 0; i < plan.size(); ++i)
		{
			const NavPlan::PlanWaypoint &wp = plan[i];
			const char *kind = "corner";
			if (i == 0)
				kind = "start";
			else if (i + 1 == plan.size())
				kind = "goal";
			else if (wp.requiresButtonUse)
				kind = "button";
			else if (wp.requiresTriggerTouch)
				kind = "trigger";
			else if (wp.requiresTeleport)
				kind = "teleport";
			else if (wp.airmoveLaunch)
				kind = wp.requiresJump ? "jump" : "dropoff";
			else if (i > 0 && plan[i - 1].airmoveLaunch)
				kind = "landing";
			else if (wp.requiresDoor)
				kind = "door";

			std::printf("  wp[%2zu] %-8s (%8.1f, %8.1f, %8.1f) area %3d%s%s",
				i, kind, wp.point.x, wp.point.y, wp.point.z, wp.targetAreaId,
				wp.requiresCrouch ? " crouch" : "", wp.requiresJump ? " JUMP" : "");
			if (wp.airmoveLaunch)
				std::printf(" launch %.0f yaw %.1f jumpBack %.1f", wp.launchSpeed, wp.launchYaw,
					wp.jumpBackDist);
			std::printf(" clearance %.1f", wp.clearance);
			std::printf("\n");
		}

		for (size_t i = 0; i < profile.segs.size(); ++i)
		{
			const NavPlan::ProfileSeg &seg = profile.segs[i];
			if (seg.isArc)
				std::printf("  seg[%2zu] arc  wp %2zu r %6.1f turn %6.1fdeg len %6.1f "
					"v %5.0f -> %5.0f cap %5.0f clearance %.1f\n",
					i, seg.wpIndex, seg.radius, seg.turn * 180.f / 3.14159265f, seg.sLen,
					seg.vEntry, seg.vExit, seg.vCap, seg.clearance);
			else
				std::printf("  seg[%2zu] %-4s wp %2zu %26slen %6.1f v %5.0f -> %5.0f cap %5.0f\n",
					i, seg.gap ? "gap" : "line", seg.wpIndex, "", seg.sLen, seg.vEntry, seg.vExit,
					seg.vCap);
		}
	}

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
	bool dumpSteps = false;
	bool dumpSupport = false;
	bool dumpPath = false;
	bool dumpPortals = false;
	bool dumpAirmove = false;
	bool noFloorSplit = false;
	bool noMerge = false;
	bool noPrune = false;
	float airmoveSamplesPerUnit = navgen::AirmoveSampleParams().samplesPerUnit;
	int dumpPlanesAreaId = -1;
	std::string objPath;
	std::string svgPath;
	std::string tracePath;
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
		else if (arg == "--dump-steps")
		{
			dumpSteps = true;
		}
		else if (arg == "--dump-support")
		{
			dumpSupport = true;
		}
		else if (arg == "--dump-path")
		{
			dumpPath = true;
		}
		else if (arg == "--dump-portals")
		{
			dumpPortals = true;
		}
		else if (arg == "--dump-airmove")
		{
			dumpAirmove = true;
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

		std::vector<navgen::ExtractedLeaf> leaves =
			navgen::ExtractHullLeaves(bsp, worldModel, kMapRadius, navgen::kHullDuck);

		std::vector<navgen::Entity> entities = navgen::ParseEntities(bsp.EntityText());
		std::vector<navgen::BlockingVolume> blockers =
			navgen::ExtractBlockingVolumes(bsp, entities, navgen::kHullDuck);
		std::vector<navgen::BlockingVolume> standBlockers =
			navgen::ExtractBlockingVolumes(bsp, entities, navgen::kHullHuman);

		navgen::PassableSet duckPassables = navgen::ExtractPassableVolumes(bsp, entities, navgen::kHullDuck);
		navgen::PassableSet standPassables = navgen::ExtractPassableVolumes(bsp, entities, navgen::kHullHuman);

	blockers.insert(blockers.end(), duckPassables.impassable.begin(), duckPassables.impassable.end());
	standBlockers.insert(standBlockers.end(), standPassables.impassable.begin(),
			standPassables.impassable.end());

	std::vector<navgen::BlockingVolume> supportBlockers = blockers;
	for (const navgen::PassableVolume &p : duckPassables.passable)
			supportBlockers.push_back(p.volume);

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

		size_t passableLeaves = 0;
		for (size_t pi = 0; pi < duckPassables.passable.size(); ++pi)
		{
			const navgen::BlockingVolume &vol = duckPassables.passable[pi].volume;
			std::vector<navgen::ExtractedLeaf> next;
			next.reserve(leaves.size());

			for (navgen::ExtractedLeaf &leaf : leaves)
			{
				if (leaf.passableIndex >= 0 || !navgen::OverlapsBlockingVolume(leaf.polytope, vol))
				{
					leaf.id = static_cast<int>(next.size());
					next.push_back(std::move(leaf));
					continue;
				}

				std::vector<navgen::Polytope> inside, outside;
				navgen::SplitByPassableVolume(bsp, leaf.polytope, vol, kMapRadius, inside, outside);

				for (navgen::Polytope &piece : outside)
				{
					navgen::ExtractedLeaf split;
					split.id = static_cast<int>(next.size());
					split.polytope = std::move(piece);
					split.passableIndex = leaf.passableIndex;
					next.push_back(std::move(split));
				}

				for (navgen::Polytope &piece : inside)
				{
					navgen::ExtractedLeaf split;
					split.id = static_cast<int>(next.size());
					split.polytope = std::move(piece);
					split.passableIndex = static_cast<int>(pi);
					next.push_back(std::move(split));
					++passableLeaves;
				}
			}

			leaves = std::move(next);
		}

		if (!duckPassables.passable.empty() || !duckPassables.impassable.empty())
		{
			std::printf("%s: %zu passable entities split out %zu door/breakable leaves, %zu impassable "
				"entities carved\n",
				bspPath.c_str(), duckPassables.passable.size(), passableLeaves,
				duckPassables.impassable.size());
		}

		if (!noFloorSplit)
		{
			size_t unsupportedDropped = 0;
			size_t floorSplitLeaves = 0;
			size_t noFloorDropped = 0;
			size_t beforeFloorSplit = leaves.size();
			leaves = navgen::SplitLeavesByFloorSupport(bsp, worldModel.headnode[navgen::kHullDuck],
				supportBlockers, leaves, kMapRadius, &unsupportedDropped, &floorSplitLeaves, &noFloorDropped,
				dumpSupport);
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

		if (dumpPortals)
		{
			for (const navgen::Portal &p : portals)
			{
				navgen::Vec3 mid = (p.pointA + p.pointB) * 0.5;
				std::printf("  portal leaf%d<->leaf%d area=%.1f mid=(%.1f,%.1f,%.1f)\n", p.leafA, p.leafB, p.area,
					mid.x, mid.y, mid.z);
			}
		}

		navgen::MergeResult merged = noMerge
			? navgen::NoMerge(leaves, portals, isCrouchLeaf, duckPassables.passable)
			: navgen::MergeAreas(leaves, portals, isCrouchLeaf, duckPassables.passable);

		std::printf("%s: %zu raw portals -> %zu areas, %zu inter-area connections (%s)\n",
			bspPath.c_str(), portals.size(), merged.areas.size(), merged.conns.size(),
			noMerge ? "NoMerge" : "MergeAreas");

		size_t steepWalkConns = navgen::FilterWalkConnectionsByStep(merged.areas, merged.conns, navgen::kStepHeight, dumpSteps);
		if (steepWalkConns > 0)
			std::printf("%s: dropped %zu walk connections exceeding %.0f-unit step height\n",
				bspPath.c_str(), steepWalkConns, navgen::kStepHeight);

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

		size_t snappedConns = navgen::SnapConnectionsToFloor(merged.areas, merged.conns);
		if (snappedConns != merged.conns.size())
			std::printf("%s: floor-snapped %zu of %zu connection endpoints pairs (%zu had no floor beneath)\n",
				bspPath.c_str(), snappedConns, merged.conns.size(), merged.conns.size() - snappedConns);

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

		size_t gatelessConns = navgen::BakeConnectionGates(merged.areas, merged.conns);
		if (gatelessConns > 0)
		{
			size_t walkless = 0, airless = 0;
			for (const navgen::MergedConnection &c : merged.conns)
			{
				if (c.connType == crossbot_nav::CONN_TELEPORT)
					continue;
				if (c.gateAValid && c.gateBValid)
					continue;
				if (c.connType == crossbot_nav::CONN_WALK)
					++walkless;
				else
					++airless;
			}
			std::printf("%s: %zu connections have no gate on one or both sides (%zu walk, %zu air move)\n",
				bspPath.c_str(), gatelessConns, walkless, airless);
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

				std::printf("  %-8s %3d %s %3d  from (%.1f, %.1f, %.1f) to (%.1f, %.1f, %.1f)  planes %d/%d",
					typeName, c.areaA, oneWay ? "->" : "<->", c.areaB,
					c.pointA.x, c.pointA.y, c.pointA.z, c.pointB.x, c.pointB.y, c.pointB.z,
					c.planeA, c.planeB);

				if (c.connFlags & crossbot_nav::CONN_FLAG_NEEDS_CROUCH)
					std::printf("  [crouch]");

				if (c.connFlags & crossbot_nav::CONN_FLAG_JUMP)
					std::printf("  [jump]");

				if (c.connType == crossbot_nav::CONN_AIRMOVE)
					std::printf("  speed %.0f yaw %.1f jumpBack %.1f fallDamage %.1f flightTime %.2f",
						c.exitSpeed, c.exitAngle, c.jumpBackDist, c.fallDamage, c.flightTime);

				if (c.gateAValid)
					std::printf("\n             gateA (%.1f, %.1f, %.1f)-(%.1f, %.1f, %.1f)",
						c.gateALeft.x, c.gateALeft.y, c.gateALeft.z,
						c.gateARight.x, c.gateARight.y, c.gateARight.z);
				if (c.gateBValid)
					std::printf("\n             gateB (%.1f, %.1f, %.1f)-(%.1f, %.1f, %.1f)",
						c.gateBLeft.x, c.gateBLeft.y, c.gateBLeft.z,
						c.gateBRight.x, c.gateBRight.y, c.gateBRight.z);

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
				if (area.areaFlags & (crossbot_nav::NAV_AREA_DOOR | crossbot_nav::NAV_AREA_BREAKABLE))
				{
					static const char *kActivation[] = {"touch", "button", "trigger"};
					char buf[128];
					if (area.doorActivation == crossbot_nav::DOOR_ACTIVATION_TOUCH)
					{
						std::snprintf(buf, sizeof(buf), " %s(touch %.2fs)",
							(area.areaFlags & crossbot_nav::NAV_AREA_DOOR) ? "door" : "breakable",
							area.openSeconds);
					}
					else
					{
						std::snprintf(buf, sizeof(buf), " %s(%s at (%.1f, %.1f, %.1f) %.2fs)",
							(area.areaFlags & crossbot_nav::NAV_AREA_DOOR) ? "door" : "breakable",
							kActivation[area.doorActivation < 3 ? area.doorActivation : 0],
							area.doorActivatorPos.x, area.doorActivatorPos.y, area.doorActivatorPos.z,
							area.openSeconds);
					}
					areaKind += buf;
				}
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
		std::vector<uint8_t> pathSegmentKinds;
		std::vector<float> pathSegmentSpeeds;
		NavPlan::PlanProfile pathProfile;
		std::vector<navgen::FailedAirmove> failedAirmoves;
		if (!objPath.empty() || !svgPath.empty() || dumpPath)
		{

			navgen::PM_MoveSimulator duckSim(bsp, worldModel.headnode[navgen::kHullDuck], blockers,
				airmoveParams, physics);
			navgen::PM_MoveSimulator standSim(bsp, worldModel.headnode[navgen::kHullHuman], standBlockers,
				airmoveParams, physics);

			NavPlan::PlanTraceContext traces;
			traces.duckTrace = duckSim.TraceFn();
			traces.standTrace = standSim.TraceFn();
			traces.physics = standSim.Physics();
			traces.dumpAirmoves = dumpAirmove;

			pathPoints = BuildDebugPath(entities, merged, &traces, &pathSegmentKinds, &failedAirmoves,
				&pathSegmentSpeeds, dumpPath, &pathProfile);
		}

		if (!objPath.empty())
		{
			ObjStats stats = WriteFloorObj(objPath, merged.areas, pathPoints);
			std::printf("wrote %s (+ %s): %zu floor faces, %zu crouch areas, %zu path segments\n",
				objPath.c_str(), BaseName(ReplaceExtension(objPath, ".mtl")).c_str(),
				stats.floorFaces, stats.crouchAreas, stats.pathSegments);
		}

		if (!svgPath.empty())
		{
			std::vector<navgen::Vec3> tracePoints;
					std::vector<std::vector<navgen::Vec3>> runupTraces;
					if (!tracePath.empty())
					{
						tracePoints = ReadTraceFile(tracePath);
						std::printf("read %zu trace points from %s\n", tracePoints.size(), tracePath.c_str());
						ReportTraceDeviation(pathProfile, tracePoints);
					}

					std::string mapName = DeriveMapName(bspPath);
					navgen::WriteDebugSvg(svgPath, mapName, merged.areas, merged.conns, pathPoints,
									CollectTargetPoints(entities), pathSegmentKinds, failedAirmoves, tracePoints, runupTraces,
								pathSegmentSpeeds);
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

