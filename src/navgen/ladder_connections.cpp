#include "ladder_connections.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <string>

#include "clipnode_tree.h"
#include "hull_defs.h"
#include "nav_format.h"
#include "nav_math.h"
#include "pm_sim.h"

namespace navgen
{

namespace
{

constexpr double kLadderProbeInset = 2.0;
constexpr double kLadderMinScanSpan = 1.0;
constexpr double kMapRadius = 8192.0;

struct ClimbFace
{
	Vec3 outward;
	Vec3 centroid;
	Vec3 down;
	Vec3 perp;
	double sideLo = 0.0;
	double sideHi = 0.0;
};

struct Dismount
{
	int area = -1;
	Vec3 landing;
	double fallDamage = 0.0;
	double flightTime = 0.0;
	double height = 0.0;
	const char *direction = "";
	bool jumpOff = false;
	bool reversible = false;
};

int ParseBrushModelIndex(const std::string &model)
{
	if (model.size() < 2 || model[0] != '*')
		return -1;
	char *end = nullptr;
	long idx = std::strtol(model.c_str() + 1, &end, 10);
	if (end == model.c_str() + 1 || *end != '\0' || idx <= 0)
		return -1;
	return static_cast<int>(idx);
}

Vec3 ParseEntityOrigin(const std::string &origin)
{
	double x = 0.0, y = 0.0, z = 0.0;
	std::sscanf(origin.c_str(), "%lf %lf %lf", &x, &y, &z);
	return Vec3(x, y, z);
}

bool InSolid(const BspFile &bsp, const HullContext &hull, const Vec3 &p)
{
	if (ClassifyPoint(bsp, hull.headnode, p) == CONTENTS_SOLID)
		return true;

	if (!hull.blockers)
		return false;

	for (const BlockingVolume &blocker : *hull.blockers)
	{
		if (p.x < blocker.mins.x || p.y < blocker.mins.y || p.z < blocker.mins.z ||
			p.x > blocker.maxs.x || p.y > blocker.maxs.y || p.z > blocker.maxs.z)
			continue;
		if (ClassifyPoint(bsp, blocker.headnode, p - blocker.origin) == CONTENTS_SOLID)
			return true;
	}

	return false;
}

std::vector<Polytope> ExtractSolidPieces(const BspFile &bsp, int headnode, const Vec3 &origin)
{
	std::vector<Polytope> pieces;

	WalkClipnodeTree(bsp, headnode, false,
		[&](int, const std::vector<PlaneChainEntry> &chain)
		{
			Polytope poly = BuildPolytopeFromChain(bsp, chain, kMapRadius);
			if (poly.Empty())
				return;
			pieces.push_back(TranslatePolytope(poly, origin));
		},
		[](int leafContents) { return leafContents == CONTENTS_SOLID; });

	return pieces;
}

bool PolytopeSetBounds(const std::vector<Polytope> &pieces, Vec3 &mins, Vec3 &maxs)
{
	mins = Vec3(1e30, 1e30, 1e30);
	maxs = Vec3(-1e30, -1e30, -1e30);
	bool any = false;

	for (const Polytope &poly : pieces)
	{
		for (const Face &f : poly.faces)
		{
			for (const Vec3 &v : f.winding)
			{
				mins.x = std::min(mins.x, v.x);
				mins.y = std::min(mins.y, v.y);
				mins.z = std::min(mins.z, v.z);
				maxs.x = std::max(maxs.x, v.x);
				maxs.y = std::max(maxs.y, v.y);
				maxs.z = std::max(maxs.z, v.z);
				any = true;
			}
		}
	}

	return any;
}

Vec3 WindingCentroid(const Winding &w)
{
	Vec3 sum(0.0, 0.0, 0.0);
	for (const Vec3 &v : w)
		sum = sum + v;
	return sum * (1.0 / static_cast<double>(w.size()));
}

float YawOf(const Vec3 &dir)
{
	return static_cast<float>(std::atan2(dir.y, dir.x) * 180.0 / NavMath::kPi);
}

bool BuildClimbFace(const Face &face, ClimbFace &out)
{
	if (face.winding.size() < 3)
		return false;

	Vec3 inward = face.plane.normal;
	if (std::fabs(inward.z) > kFloorNormalZ)
		return false;

	out.outward = inward * -1.0;
	out.centroid = WindingCentroid(face.winding);

	Vec3 down(0.0, 0.0, -1.0);
	Vec3 slope = down - out.outward * down.Dot(out.outward);
	if (slope.Length() < 1e-6)
		return false;
	out.down = slope.Normalized();

	Vec3 perp = Vec3(0.0, 0.0, 1.0).Cross(out.outward);
	if (perp.Length() < 1e-6)
		return false;
	out.perp = perp.Normalized();

	out.sideLo = 1e30;
	out.sideHi = -1e30;
	for (const Vec3 &v : face.winding)
	{
		double s = (v - out.centroid).Dot(out.perp);
		out.sideLo = std::min(out.sideLo, s);
		out.sideHi = std::max(out.sideHi, s);
	}

	return true;
}

bool CanRemountAt(const Vec3 &p, const Vec3 &volMins, const Vec3 &volMaxs)
{
	return p.x >= volMins.x && p.x <= volMaxs.x &&
		p.y >= volMins.y && p.y <= volMaxs.y &&
		p.z >= volMins.z - kStandProbeLift && p.z <= volMaxs.z + kStandProbeLift;
}

bool BetterDismount(const Dismount &candidate, const Dismount &current, const Vec3 &areaCentroid)
{
	if (candidate.fallDamage < current.fallDamage - 0.01)
		return true;
	if (candidate.fallDamage > current.fallDamage + 0.01)
		return false;

	if (candidate.reversible != current.reversible)
		return candidate.reversible;

	return (candidate.landing - areaCentroid).Length() < (current.landing - areaCentroid).Length();
}

long long PairKey(int a, int b)
{
	int lo = std::min(a, b);
	int hi = std::max(a, b);
	return static_cast<long long>(lo) * 1000000LL + static_cast<long long>(hi);
}

}

LadderStats BuildLadderConnections(const BspFile &bsp, const std::vector<Entity> &entities,
	const HullContext &duckHull, const std::vector<Area> &areas, std::vector<MergedConnection> &conns,
	const PhysicsParams &physics, bool dump)
{
	LadderStats stats;

	if (duckHull.headnode < 0 || !duckHull.blockers)
		return stats;

	AirmoveSampleParams simParams;
	PM_MoveSimulator sim(bsp, duckHull.headnode, *duckHull.blockers, simParams, physics);

	double bottomReach = 2.0 * kStandProbeLift +
		static_cast<double>(PmSim::PlainJumpHeight(physics.gravity));

	std::map<long long, size_t> emitted;

	for (const Entity &ent : entities)
	{
		if (ent.Get("classname") != "func_ladder")
			continue;

		++stats.entities;

		int modelIndex = ParseBrushModelIndex(ent.Get("model"));
		if (modelIndex < 0 || static_cast<size_t>(modelIndex) >= bsp.Models().size())
		{
			if (dump)
				std::printf("  ladder \"%s\": no brush model\n", ent.Get("model").c_str());
			continue;
		}

		const dmodel_t &model = bsp.Models()[static_cast<size_t>(modelIndex)];
		if (model.headnode[kHullDuck] < 0)
		{
			if (dump)
				std::printf("  ladder *%d: no hull-3 clipnodes\n", modelIndex);
			continue;
		}

		Vec3 origin = ParseEntityOrigin(ent.Get("origin"));
		std::vector<Polytope> pieces = ExtractSolidPieces(bsp, model.headnode[kHullDuck], origin);

		Vec3 volMins, volMaxs;
		if (!PolytopeSetBounds(pieces, volMins, volMaxs))
		{
			if (dump)
				std::printf("  ladder *%d: hull-3 volume is empty\n", modelIndex);
			continue;
		}

		if (dump)
			std::printf("  ladder *%d: hull-3 volume (%.1f, %.1f, %.1f)..(%.1f, %.1f, %.1f), %zu piece(s)\n",
				modelIndex, volMins.x, volMins.y, volMins.z, volMaxs.x, volMaxs.y, volMaxs.z, pieces.size());

		size_t entityConns = 0;

		for (const Polytope &piece : pieces)
		{
			for (const Face &face : piece.faces)
			{
				ClimbFace climb;
				if (!BuildClimbFace(face, climb))
				{
					if (dump && face.winding.size() >= 3)
						std::printf("    face n (%.2f, %.2f, %.2f): rejected, horizontal\n",
							-face.plane.normal.x, -face.plane.normal.y, -face.plane.normal.z);
					continue;
				}

				++stats.climbFaces;

				Vec3 probe = climb.centroid + climb.outward * -kLadderProbeInset;

				if (dump)
					std::printf("    face n (%.2f, %.2f, %.2f) centroid (%.1f, %.1f, %.1f):\n",
						climb.outward.x, climb.outward.y, climb.outward.z,
						climb.centroid.x, climb.centroid.y, climb.centroid.z);

				if (InSolid(bsp, duckHull, probe))
				{
					if (dump)
						std::printf("      rejected: probe (%.1f, %.1f, %.1f) starts in solid\n",
							probe.x, probe.y, probe.z);
					continue;
				}

				double dropSpan = (probe.z - (volMins.z - bottomReach)) / -climb.down.z;
				Vec3 dropEnd = probe + climb.down * dropSpan;
				TraceResult down = TraceHullWithEntities(bsp, duckHull.headnode, *duckHull.blockers,
					probe, dropEnd);

				if (down.startsolid || down.allsolid || down.fraction >= 1.0f)
				{
					if (dump)
						std::printf("      rejected: no floor within %.0f units below the volume\n",
							bottomReach);
					continue;
				}

				Vec3 mount = down.endpos;
				int bottomArea = FindAreaStrict(mount, areas);
				if (bottomArea < 0)
				{
					if (dump)
						std::printf("      rejected: mount (%.1f, %.1f, %.1f) is in no area\n",
							mount.x, mount.y, mount.z);
					continue;
				}

				++stats.mounts;

				Vec3 up = climb.down * -1.0;
				double climbSpan = (volMaxs.z - mount.z) / up.z;
				if (climbSpan < kLadderMinScanSpan)
				{
					if (dump)
						std::printf("      rejected: mount (%.1f, %.1f, %.1f) is above the volume top\n",
							mount.x, mount.y, mount.z);
					continue;
				}

				TraceResult upTrace = TraceHullWithEntities(bsp, duckHull.headnode, *duckHull.blockers,
					mount, mount + up * climbSpan);
				Vec3 top = upTrace.startsolid ? mount : upTrace.endpos;
				bool ladderEnded = !upTrace.startsolid && upTrace.fraction >= 1.0f;

				if (dump)
					std::printf("      mount (%.1f, %.1f, %.1f) area %d, top (%.1f, %.1f, %.1f) %s\n",
						mount.x, mount.y, mount.z, bottomArea, top.x, top.y, top.z,
						ladderEnded ? "(ladder ended)" : "(blocked)");

				double climbSpeed = kLadderClimbSpeed;
				float climbYaw = YawOf(climb.outward * -1.0);

				std::map<int, Dismount> best;

				auto record = [&](const AirmoveTrajectory &traj, int area, double height,
					const char *direction, bool jumpOff)
				{
					if (area < 0)
						return;

					bool reversible = !jumpOff && CanRemountAt(traj.landingPos, volMins, volMaxs);

					if (dump)
						std::printf("        %-9s at z %7.1f -> area %3d, fall %.1f, flight %.2fs, "
							"landing (%.1f, %.1f, %.1f)%s%s\n",
							direction, height, area, traj.fallDamage, traj.flightTime,
							traj.landingPos.x, traj.landingPos.y, traj.landingPos.z,
							reversible ? "  [remountable]" : "",
							(area == bottomArea) ? "  [loopback]" : "");

					if (area == bottomArea)
						return;

					Dismount d;
					d.area = area;
					d.landing = traj.landingPos;
					d.fallDamage = traj.fallDamage;
					d.flightTime = traj.flightTime;
					d.height = height;
					d.direction = direction;
					d.jumpOff = jumpOff;
					d.reversible = reversible;

					Vec3 areaCentroid = areas[static_cast<size_t>(area)].polytope.Centroid();
					auto it = best.find(area);
					if (it == best.end() || BetterDismount(d, it->second, areaCentroid))
						best[area] = d;
				};

				for (double travelled = 0.0;; travelled += kLadderDismountStep)
				{
					Vec3 at = top + climb.down * travelled;
					if (at.z < mount.z + kLadderMinScanSpan)
						break;

					bool atTop = (travelled == 0.0);
					int area = -1;

					if (atTop && ladderEnded)
					{
						AirmoveTrajectory traj = sim.SimulateAirborne(at,
							Vec3(0.0, 0.0, climbSpeed), climbYaw, physics.maxSpeed, areas, -1, area);
						record(traj, area, at.z, "forward", false);
					}

					for (int side = 0; side < 2; ++side)
					{
						double offset = (side == 0) ? climb.sideHi : climb.sideLo;
						Vec3 dir = (side == 0) ? climb.perp : climb.perp * -1.0;
						Vec3 from = at + climb.perp * offset;

						if (InSolid(bsp, duckHull, from))
							continue;

						AirmoveTrajectory traj = sim.SimulateAirborne(from, dir * climbSpeed,
							YawOf(dir), physics.maxSpeed, areas, -1, area);
						record(traj, area, at.z, (side == 0) ? "side +" : "side -", false);
					}

					AirmoveTrajectory jumped = sim.SimulateAirborne(at,
						climb.outward * kLadderJumpOffSpeed, YawOf(climb.outward), 0.0f, areas, -1, area);
					record(jumped, area, at.z, "jump off", true);
				}

				for (const auto &entry : best)
				{
					const Dismount &d = entry.second;
					bool crouch = (areas[static_cast<size_t>(d.area)].areaFlags &
						crossbot_nav::NAV_AREA_CROUCH) != 0;

					MergedConnection conn;
					conn.areaA = bottomArea;
					conn.areaB = d.area;
					conn.pointA = mount;
					conn.pointB = d.landing;
					conn.gateALeft = mount;
					conn.gateARight = mount;
					conn.gateBLeft = d.landing;
					conn.gateBRight = d.landing;
					conn.gateAValid = true;
					conn.gateBValid = true;
					conn.connType = crossbot_nav::CONN_LADDER;
					conn.exitSpeed = crouch
						? kLadderClimbSpeed * static_cast<double>(physics.duckSpeedMultiplier)
						: kLadderClimbSpeed;
					conn.exitAngle = climbYaw;
					conn.fallDamage = d.fallDamage;
					conn.flightTime = d.flightTime;

					if (crouch)
						conn.connFlags |= crossbot_nav::CONN_FLAG_NEEDS_CROUCH;
					if (!d.reversible)
						conn.connFlags |= crossbot_nav::CONN_FLAG_ONE_WAY;

					long long key = PairKey(bottomArea, d.area);
					auto it = emitted.find(key);
					if (it != emitted.end())
					{
						MergedConnection &prev = conns[it->second];
						if (conn.fallDamage < prev.fallDamage - 0.01)
							prev = conn;
						++entityConns;
						continue;
					}

					emitted[key] = conns.size();
					conns.push_back(conn);
					++stats.conns;
					++entityConns;

					if (dump)
						std::printf("      + ladder %d %s %d via %s at z %.1f, fall %.1f\n",
							bottomArea, (conn.connFlags & crossbot_nav::CONN_FLAG_ONE_WAY) ? "->" : "<->",
							d.area, d.direction, d.height, d.fallDamage);
				}
			}
		}

		if (dump && entityConns == 0)
			std::printf("  ladder *%d: no connections emitted\n", modelIndex);
	}

	return stats;
}

}
