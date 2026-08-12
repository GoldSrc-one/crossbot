#include "ladder_connections.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <string>

#include "clipnode_tree.h"
#include "floor_support.h"
#include "hull_defs.h"
#include "nav_format.h"
#include "nav_math.h"
#include "pm_sim.h"

namespace navgen
{

namespace
{

constexpr double kLadderProbeInset = 2.0;
constexpr double kLadderProbeStep = 8.0;
constexpr double kLadderMinScanSpan = 1.0;
constexpr double kLadderStandStep = 4.0;
constexpr double kLadderStandReach = 48.0;
constexpr double kLadderStandProbeStep = 2.0;
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
	Vec3 entry;
	float entryYaw = 0.0f;
	double entrySpeed = 0.0;
	Vec3 exit;
	float exitYaw = 0.0f;
	double fallDamage = 0.0;
	double flightTime = 0.0;
	double height = 0.0;
	const char *direction = "";
	bool jumpOff = false;
	bool reversible = false;
	bool viaDrop = false;
};

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

bool FindFaceProbe(const BspFile &bsp, const HullContext &hull, const ClimbFace &climb, Vec3 &probeOut)
{
	static const double insets[] = {kLadderProbeInset, 1.0, 0.5};

	double span = climb.sideHi - climb.sideLo;
	int steps = (span > kLadderProbeStep) ? static_cast<int>(span / kLadderProbeStep) : 0;

	for (int step = 0; step <= steps; ++step)
	{
		for (int side = 0; side < 2; ++side)
		{
			if (step == 0 && side == 1)
				continue;

			double offset = kLadderProbeStep * step * ((side == 0) ? 1.0 : -1.0);
			if (offset > climb.sideHi - kLadderProbeInset || offset < climb.sideLo + kLadderProbeInset)
				continue;

			for (double inset : insets)
			{
				Vec3 probe = climb.centroid + climb.perp * offset + climb.outward * -inset;
				if (InSolid(bsp, hull, probe))
					continue;

				probeOut = probe;
				return true;
			}
		}
	}

	return false;
}

bool StandFits(const BspFile &bsp, const HullContext &standHull, const Vec3 &at)
{
	return !InSolid(bsp, standHull, at + Vec3(0.0, 0.0, kStandProbeLift));
}

int FindStandArea(const BspFile &bsp, const HullContext &hull, const std::vector<Area> &areas,
	const Vec3 &mount, const Vec3 &outward, Vec3 &standOut)
{
	int area = FindAreaStrict(mount, areas);
	if (area >= 0)
	{
		standOut = mount;
		return area;
	}

	Vec3 horiz(outward.x, outward.y, 0.0);
	if (horiz.Length() < 1e-6)
		return -1;
	horiz = horiz.Normalized();

	for (double out = kLadderStandStep; out <= kLadderStandReach; out += kLadderStandStep)
	{
		Vec3 from = mount + horiz * out + Vec3(0.0, 0.0, kStepHeight);
		if (InSolid(bsp, hull, from))
			continue;

		TraceResult down = TraceHullWithEntities(bsp, hull.headnode, *hull.blockers, from,
			from - Vec3(0.0, 0.0, 2.0 * kStepHeight));
		if (down.startsolid || down.allsolid || down.fraction >= 1.0f)
			continue;

		area = FindAreaStrict(down.endpos, areas);
		if (area < 0)
			continue;

		standOut = down.endpos;
		return area;
	}

	return -1;
}

bool CanRemountAt(const Vec3 &p, const Vec3 &volMins, const Vec3 &volMaxs)
{
	return p.x >= volMins.x && p.x <= volMaxs.x &&
		p.y >= volMins.y && p.y <= volMaxs.y &&
		p.z >= volMins.z - kStandProbeLift && p.z <= volMaxs.z + kStandProbeLift;
}

const char *DropNote(const Vec3 &entry, float yaw, double speed)
{
	static char note[96];
	std::snprintf(note, sizeof(note), ", descent drops from (%.1f, %.1f, %.1f) at yaw %.1f speed %.0f",
		entry.x, entry.y, entry.z, yaw, speed);
	return note;
}

float YawTowardVolume(const Vec3 &from, const Vec3 &volMins, const Vec3 &volMaxs)
{
	Vec3 nearest(std::min(std::max(from.x, volMins.x), volMaxs.x),
		std::min(std::max(from.y, volMins.y), volMaxs.y), from.z);
	return YawOf(nearest - from);
}

bool EdgeDropReachesLadder(const PM_MoveSimulator &ladderSim, const std::vector<Area> &areas,
	int landingArea, float climbYaw, const Vec3 &volMins, const Vec3 &volMaxs,
	const AirmoveSampleParams &params, const PhysicsParams &physics, Vec3 &entryOut, float &yawOut,
	double &speedOut)
{
	if (landingArea < 0 || static_cast<size_t>(landingArea) >= areas.size())
		return false;

	const Area &area = areas[static_cast<size_t>(landingArea)];
	double bestTime = 1e30;

	for (const LaunchEdge &edge : FilterLaunchEdges(ladderSim, area, 0.0, params))
	{
		int samples = static_cast<int>(edge.length / kLadderDismountStep) + 1;

		for (int s = 0; s <= samples; ++s)
		{
			double t = (samples > 0) ? static_cast<double>(s) / samples : 0.5;
			Vec3 launch = edge.a + (edge.b - edge.a) * t - edge.outward * params.inset;
			launch.z += params.lift;

			if (!ladderSim.IsSupported(launch))
				continue;
			if (FindAreaStrict(launch, areas) != landingArea)
				continue;

			for (float yaw : {YawTowardVolume(launch, volMins, volMaxs), climbYaw})
			{
				for (double scale : {1.0, 0.5, 0.25})
				{
					int reached = -1;
					AirmoveTrajectory traj = ladderSim.Simulate(launch, yaw,
						static_cast<float>(physics.maxSpeed * scale), false, areas, landingArea,
						reached);

					if (!traj.grabbedLadder || traj.flightTime >= bestTime)
						continue;

					bestTime = traj.flightTime;
					entryOut = launch;
					yawOut = yaw;
					speedOut = physics.maxSpeed * scale;
				}
			}
		}
	}

	return bestTime < 1e30;
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
	const HullContext &duckHull, const HullContext &standHull, const std::vector<Area> &areas,
	std::vector<MergedConnection> &conns, const PhysicsParams &physics, bool dump)
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

		std::vector<BlockingVolume> thisLadder(1);
		thisLadder[0].headnode = model.headnode[kHullDuck];
		thisLadder[0].origin = origin;
		thisLadder[0].mins = volMins;
		thisLadder[0].maxs = volMaxs;

		PM_MoveSimulator dropSim(bsp, duckHull.headnode, *duckHull.blockers, simParams, physics,
			&thisLadder);

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

				if (dump)
					std::printf("    face n (%.2f, %.2f, %.2f) centroid (%.1f, %.1f, %.1f):\n",
						climb.outward.x, climb.outward.y, climb.outward.z,
						climb.centroid.x, climb.centroid.y, climb.centroid.z);

				Vec3 probe;
				if (!FindFaceProbe(bsp, duckHull, climb, probe))
				{
					if (dump)
						std::printf("      rejected: no free probe anywhere on the face\n");
					continue;
				}

				if (dump && (probe - (climb.centroid + climb.outward * -kLadderProbeInset)).Length() > 0.01)
					std::printf("      probe moved to (%.1f, %.1f, %.1f) to clear solid\n",
						probe.x, probe.y, probe.z);

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
				Vec3 stand;
				int bottomArea = FindStandArea(bsp, duckHull, areas, mount, climb.outward, stand);
				if (bottomArea < 0)
				{
					if (dump)
						std::printf("      rejected: mount (%.1f, %.1f, %.1f) has no area within "
							"%.0f units outward\n", mount.x, mount.y, mount.z, kLadderStandReach);
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

				Vec3 reach = top;
				while (reach.z > mount.z && !StandFits(bsp, standHull, reach))
					reach = reach + climb.down * kLadderStandProbeStep;
				if (reach.z < mount.z)
					reach = mount;

				if (dump)
					std::printf("      mount (%.1f, %.1f, %.1f), stand (%.1f, %.1f, %.1f) area %d, "
						"top (%.1f, %.1f, %.1f) %s, standing reach z %.1f\n",
						mount.x, mount.y, mount.z, stand.x, stand.y, stand.z, bottomArea,
						top.x, top.y, top.z, ladderEnded ? "(ladder ended)" : "(blocked)", reach.z);

				double climbSpeed = kLadderClimbSpeed;
				float climbYaw = YawOf(climb.outward * -1.0);
				float outYaw = YawOf(climb.outward);

				std::map<int, Dismount> best;

				auto record = [&](const AirmoveTrajectory &traj, int area, double height,
					const char *direction, bool jumpOff, const Vec3 &exitPos, float exitYaw, bool duckOnly)
				{
					if (area < 0)
						return;

					bool reversible = !jumpOff && CanRemountAt(traj.landingPos, volMins, volMaxs);
					bool unreachable = duckOnly && (areas[static_cast<size_t>(area)].areaFlags &
						crossbot_nav::NAV_AREA_CROUCH) == 0;

					if (dump)
						std::printf("        %-9s at z %7.1f -> area %3d, fall %.1f, flight %.2fs, "
							"landing (%.1f, %.1f, %.1f)%s%s%s\n",
							direction, height, area, traj.fallDamage, traj.flightTime,
							traj.landingPos.x, traj.landingPos.y, traj.landingPos.z,
							reversible ? "  [remountable]" : "",
							(area == bottomArea) ? "  [loopback]" : "",
							unreachable ? "  [standing hull does not fit]" : "");

					if (area == bottomArea || unreachable)
						return;

					Dismount d;
					d.area = area;
					d.landing = traj.landingPos;
					d.entry = traj.landingPos;
					d.entryYaw = outYaw;
					d.entrySpeed = physics.maxSpeed;
					d.exit = exitPos;
					d.exitYaw = exitYaw;
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

				std::vector<Vec3> samples;
				bool reachAdded = false;
				for (double travelled = 0.0;; travelled += kLadderDismountStep)
				{
					Vec3 at = top + climb.down * travelled;
					if (at.z < mount.z + kLadderMinScanSpan)
						break;

					if (!reachAdded && at.z <= reach.z)
					{
						if (at.z < reach.z)
							samples.push_back(reach);
						reachAdded = true;
					}

					samples.push_back(at);
				}

				for (size_t si = 0; si < samples.size(); ++si)
				{
					const Vec3 &at = samples[si];
					bool atTop = (si == 0);
					bool duckOnly = !StandFits(bsp, standHull, at);
					int area = -1;

					if (atTop && ladderEnded)
					{
						AirmoveTrajectory traj = sim.SimulateAirborne(at,
							Vec3(0.0, 0.0, climbSpeed), climbYaw, physics.maxSpeed, areas, -1, area);
						record(traj, area, at.z, "forward", false, at, climbYaw, duckOnly);
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
						record(traj, area, at.z, (side == 0) ? "side +" : "side -", false, from, YawOf(dir),
							!StandFits(bsp, standHull, from));
					}

					AirmoveTrajectory jumped = sim.SimulateAirborne(at,
						climb.outward * kLadderJumpOffSpeed, YawOf(climb.outward), 0.0f, areas, -1, area);
					record(jumped, area, at.z, "jump off", true, at, outYaw, duckOnly);
				}

				for (auto &entry : best)
				{
					Dismount &d = entry.second;
					bool crouch = (areas[static_cast<size_t>(d.area)].areaFlags &
						crossbot_nav::NAV_AREA_CROUCH) != 0;

					if (!d.reversible)
					{
						Vec3 dropFrom;
						float dropYaw = climbYaw;
						double dropSpeed = physics.maxSpeed;
						if (EdgeDropReachesLadder(dropSim, areas, d.area, climbYaw, volMins, volMaxs,
							simParams, physics, dropFrom, dropYaw, dropSpeed))
						{
							d.entry = dropFrom;
							d.entryYaw = dropYaw;
							d.entrySpeed = dropSpeed;
							d.reversible = true;
							d.viaDrop = true;
						}
					}

					MergedConnection conn;
					conn.areaA = bottomArea;
					conn.areaB = d.area;
					conn.pointA = stand;
					conn.pointB = d.landing;
					conn.gateALeft = stand;
					conn.gateARight = stand;
					conn.gateBLeft = d.entry;
					conn.gateBRight = d.exit;
					conn.gateAYaw[0] = d.entryYaw;
					conn.gateAYaw[1] = d.exitYaw;
					conn.gateASpeed[0] = d.entrySpeed;
					conn.gateASpeed[1] = d.entrySpeed;
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
						std::printf("      + ladder %d %s %d via %s off (%.1f, %.1f, %.1f) yaw %.1f, "
							"fall %.1f%s\n",
							bottomArea, (conn.connFlags & crossbot_nav::CONN_FLAG_ONE_WAY) ? "->" : "<->",
							d.area, d.direction, d.exit.x, d.exit.y, d.exit.z, d.exitYaw, d.fallDamage,
							d.viaDrop ? DropNote(d.entry, d.entryYaw, d.entrySpeed) : "");
				}
			}
		}

		if (dump && entityConns == 0)
			std::printf("  ladder *%d: no connections emitted\n", modelIndex);
	}

	return stats;
}

}
