#include "water_volumes.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "area_merger.h"
#include "bsp_vis.h"
#include "clipnode_tree.h"
#include "floor_support.h"
#include "hull_defs.h"

namespace navgen
{

namespace
{

constexpr double kLiquidGroupSlop = 1.0;

constexpr double kLiquidStandLift = 18.0;

constexpr double kLiquidSampleStep = 8.0;

int LiquidContentsForSkin(int skin)
{
	if (skin == CONTENTS_WATER || skin == CONTENTS_SLIME || skin == CONTENTS_LAVA)
		return skin;
	return 0;
}

bool IsLiquidContents(int contents)
{
	return contents == CONTENTS_WATER || contents == CONTENTS_SLIME || contents == CONTENTS_LAVA;
}

bool BoxesTouch(const Vec3 &aMin, const Vec3 &aMax, const Vec3 &bMin, const Vec3 &bMax, double slop)
{
	return !(aMax.x + slop < bMin.x || bMax.x + slop < aMin.x ||
		aMax.y + slop < bMin.y || bMax.y + slop < aMin.y ||
		aMax.z + slop < bMin.z || bMax.z + slop < aMin.z);
}

struct RawPiece
{
	Polytope poly;
	Vec3 mins;
	Vec3 maxs;
	int contents = CONTENTS_WATER;
};

void CollectHullPieces(const BspFile &bsp, const HullTree &hull, int headnode, double mapRadius,
	const ContentsFilter &filter, const Vec3 &offset, std::vector<RawPiece> &out)
{
	if (headnode < 0)
		return;

	WalkHullTree(hull, headnode, filter,
		[&](int leafContents, const std::vector<PlaneChainEntry> &chain)
		{
			Polytope poly = BuildPolytopeFromChain(bsp, chain, mapRadius);
			if (poly.Empty())
				return;

			if (offset.x != 0.0 || offset.y != 0.0 || offset.z != 0.0)
				poly = TranslatePolytope(poly, offset);

			RawPiece piece;
			piece.contents = leafContents;
			PolytopeAABB(poly, piece.mins, piece.maxs);
			piece.poly = std::move(poly);
			out.push_back(std::move(piece));
		});
}

std::vector<LiquidVolume> GroupPieces(std::vector<RawPiece> &pieces, int contents, int modelIndex,
	int entityIndex)
{
	std::vector<LiquidVolume> bodies;
	std::vector<int> owner(pieces.size(), -1);

	for (size_t i = 0; i < pieces.size(); ++i)
	{
		if (owner[i] >= 0)
			continue;

		size_t bodyId = bodies.size();
		LiquidVolume body;
		body.contents = contents;
		body.modelIndex = modelIndex;
		body.entityIndex = entityIndex;
		body.mins = pieces[i].mins;
		body.maxs = pieces[i].maxs;
		owner[i] = static_cast<int>(bodyId);
		bodies.push_back(std::move(body));

		bool grew = true;
		while (grew)
		{
			grew = false;
			for (size_t j = 0; j < pieces.size(); ++j)
			{
				if (owner[j] >= 0)
					continue;
				if (!BoxesTouch(bodies[bodyId].mins, bodies[bodyId].maxs, pieces[j].mins, pieces[j].maxs,
						kLiquidGroupSlop))
					continue;

				owner[j] = static_cast<int>(bodyId);
				bodies[bodyId].mins.x = std::min(bodies[bodyId].mins.x, pieces[j].mins.x);
				bodies[bodyId].mins.y = std::min(bodies[bodyId].mins.y, pieces[j].mins.y);
				bodies[bodyId].mins.z = std::min(bodies[bodyId].mins.z, pieces[j].mins.z);
				bodies[bodyId].maxs.x = std::max(bodies[bodyId].maxs.x, pieces[j].maxs.x);
				bodies[bodyId].maxs.y = std::max(bodies[bodyId].maxs.y, pieces[j].maxs.y);
				bodies[bodyId].maxs.z = std::max(bodies[bodyId].maxs.z, pieces[j].maxs.z);
				grew = true;
			}
		}
	}

	for (size_t i = 0; i < pieces.size(); ++i)
	{
		if (owner[i] >= 0)
			bodies[static_cast<size_t>(owner[i])].pieces.push_back(std::move(pieces[i].poly));
	}

	for (LiquidVolume &body : bodies)
		body.surfaceZ = body.maxs.z;

	return bodies;
}

void MergeCoincidentBodies(std::vector<LiquidVolume> &bodies)
{
	constexpr double kSurfaceEpsilon = 1.0;

	bool merged = true;
	while (merged)
	{
		merged = false;

		for (size_t i = 0; i < bodies.size() && !merged; ++i)
		{
			if (bodies[i].moving)
				continue;

			for (size_t j = i + 1; j < bodies.size(); ++j)
			{
				if (bodies[j].moving)
					continue;
				if (bodies[i].contents != bodies[j].contents)
					continue;
				if (std::fabs(bodies[i].surfaceZ - bodies[j].surfaceZ) > kSurfaceEpsilon)
					continue;
				if (!BoxesTouch(bodies[i].mins, bodies[i].maxs, bodies[j].mins, bodies[j].maxs,
						kLiquidGroupSlop))
					continue;

				for (Polytope &piece : bodies[j].pieces)
					bodies[i].pieces.push_back(std::move(piece));

				bodies[i].mins.x = std::min(bodies[i].mins.x, bodies[j].mins.x);
				bodies[i].mins.y = std::min(bodies[i].mins.y, bodies[j].mins.y);
				bodies[i].mins.z = std::min(bodies[i].mins.z, bodies[j].mins.z);
				bodies[i].maxs.x = std::max(bodies[i].maxs.x, bodies[j].maxs.x);
				bodies[i].maxs.y = std::max(bodies[i].maxs.y, bodies[j].maxs.y);
				bodies[i].maxs.z = std::max(bodies[i].maxs.z, bodies[j].maxs.z);
				bodies[i].surfaceZ = bodies[i].maxs.z;
				bodies[i].modelIndex = 0;
				bodies[i].entityIndex = -1;

				bodies.erase(bodies.begin() + static_cast<long>(j));
				merged = true;
				break;
			}
		}
	}
}

}

void PolytopeAABB(const Polytope &poly, Vec3 &mins, Vec3 &maxs)
{
	mins = Vec3(1e30, 1e30, 1e30);
	maxs = Vec3(-1e30, -1e30, -1e30);

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
		}
	}
}

bool PointInPolytope(const Polytope &poly, const Vec3 &p, double epsilon)
{
	if (poly.faces.empty())
		return false;

	for (const Face &f : poly.faces)
	{
		if (f.plane.normal.Dot(p) - f.plane.dist < -epsilon)
			return false;
	}
	return true;
}

bool LiquidVolume::ContainsPoint(const Vec3 &p, double epsilon) const
{
	if (p.x < mins.x - epsilon || p.x > maxs.x + epsilon ||
		p.y < mins.y - epsilon || p.y > maxs.y + epsilon ||
		p.z < mins.z - epsilon || p.z > maxs.z + epsilon)
		return false;

	for (const Polytope &piece : pieces)
	{
		if (PointInPolytope(piece, p, epsilon))
			return true;
	}
	return false;
}

bool LiquidVolume::OverlapsColumn(const Vec3 &boxMins, const Vec3 &boxMaxs) const
{
	return !(maxs.x < boxMins.x || boxMaxs.x < mins.x || maxs.y < boxMins.y || boxMaxs.y < mins.y);
}

LiquidSet ExtractLiquidVolumes(const BspFile &bsp, const std::vector<Entity> &entities, double mapRadius)
{
	LiquidSet result;

	const std::vector<dmodel_t> &models = bsp.Models();
	if (models.empty())
		return result;

	std::vector<dclipnode_t> hull0 = BuildHull0Clipnodes(bsp);

	HullTree hull;
	hull.clipnodes = hull0.data();
	hull.clipnodeCount = hull0.size();
	hull.planes = bsp.Planes().data();
	hull.planeCount = bsp.Planes().size();

	auto emit = [&result](std::vector<LiquidVolume> &bodies)
	{
		for (LiquidVolume &body : bodies)
		{
			if (body.pieces.empty())
				continue;
			if (body.contents == CONTENTS_WATER)
				result.water.push_back(std::move(body));
			else
				result.hazard.push_back(std::move(body));
		}
	};

	const int kWorldContents[3] = {CONTENTS_WATER, CONTENTS_SLIME, CONTENTS_LAVA};
	for (int k = 0; k < 3; ++k)
	{
		int contents = kWorldContents[k];
		std::vector<RawPiece> pieces;
		CollectHullPieces(bsp, hull, models[0].headnode[0], mapRadius,
			[contents](int leafContents) { return leafContents == contents; }, Vec3(), pieces);

		std::vector<LiquidVolume> bodies = GroupPieces(pieces, contents, 0, -1);
		emit(bodies);
	}

	for (size_t i = 0; i < entities.size(); ++i)
	{
		const Entity &ent = entities[i];
		std::string cls = ent.Get("classname");
		if (cls.compare(0, 8, "trigger_") == 0)
			continue;

		int modelIndex = ParseBrushModelIndex(ent.Get("model"));
		if (modelIndex < 0 || static_cast<size_t>(modelIndex) >= models.size())
			continue;

		const dmodel_t &model = models[static_cast<size_t>(modelIndex)];
		if (model.headnode[0] < 0)
			continue;

		int contents = LiquidContentsForSkin(ParseInt(ent.Get("skin"), 0));
		if (contents == 0)
		{
			bool anyLiquid = false;
			WalkHullTree(hull, model.headnode[0],
				[&anyLiquid](int leafContents)
				{
					if (IsLiquidContents(leafContents))
						anyLiquid = true;
					return false;
				},
				[](int, const std::vector<PlaneChainEntry> &) {});

			if (!anyLiquid)
				continue;

			contents = CONTENTS_WATER;
		}

		Vec3 origin = ParseEntityOrigin(ent.Get("origin"));

		std::vector<RawPiece> pieces;
		CollectHullPieces(bsp, hull, model.headnode[0], mapRadius,
			[](int leafContents) { return leafContents != CONTENTS_EMPTY; }, origin, pieces);

		std::vector<LiquidVolume> bodies = GroupPieces(pieces, contents, modelIndex, static_cast<int>(i));

		Vec3 movedir = MoveDirFromAngles(EntityAngles(ent));

		bool isDoor = (cls == "func_water" || cls == "func_door") && !ent.Get("targetname").empty() &&
			std::fabs(movedir.z) > kFloorNormalZ;
		double travelDist = isDoor ? DoorTravelDistance(ent, model) : 0.0;

		if (travelDist > 0.0)
		{
			Vec3 travel = movedir * travelDist;
			bool startOpen = (ParseInt(ent.Get("spawnflags"), 0) & 1) != 0;

			for (LiquidVolume &body : bodies)
			{
				Vec3 bodyTravel = travel;
				if (startOpen)
				{
					body.mins = body.mins + travel;
					body.maxs = body.maxs + travel;
					for (Polytope &piece : body.pieces)
						piece = TranslatePolytope(piece, travel);
					body.surfaceZ = body.maxs.z;
					bodyTravel = travel * -1.0;
				}

				body.travel = bodyTravel;
				body.speed = DoorSpeed(ent);
				body.moving = true;
			}
		}

		emit(bodies);
	}

	MergeCoincidentBodies(result.water);
	MergeCoincidentBodies(result.hazard);

	return result;
}

void DumpLiquidVolumes(const std::string &bspPath, const LiquidSet &liquids)
{
	auto dumpOne = [&bspPath](const LiquidVolume &vol, const char *kind, size_t index)
	{
		const char *source = (vol.entityIndex < 0) ? "world" : "entity";
		std::printf("  %s %2zu: %-5s %s *%d, mins (%.0f, %.0f, %.0f) maxs (%.0f, %.0f, %.0f), "
			"%zu piece(s), surface %.1f",
			kind, index, source, (vol.contents == CONTENTS_WATER) ? "water"
				: (vol.contents == CONTENTS_SLIME ? "slime" : "lava"),
			vol.modelIndex, vol.mins.x, vol.mins.y, vol.mins.z, vol.maxs.x, vol.maxs.y, vol.maxs.z,
			vol.pieces.size(), vol.surfaceZ);

		if (vol.moving)
			std::printf(", travel (%.0f, %.0f, %.0f) at %.0f u/s, levels %.1f -> %.1f",
				vol.travel.x, vol.travel.y, vol.travel.z, vol.speed, vol.LowSurfaceZ(),
				vol.HighSurfaceZ());
		else
			std::printf(", static");

		std::printf("\n");
	};

	std::printf("%s: liquid volumes\n", bspPath.c_str());
	for (size_t i = 0; i < liquids.water.size(); ++i)
		dumpOne(liquids.water[i], "water ", i);
	for (size_t i = 0; i < liquids.hazard.size(); ++i)
		dumpOne(liquids.hazard[i], "hazard", i);
	std::fflush(stdout);
}

size_t DropLiquidHazardLeaves(const std::vector<LiquidVolume> &hazards, std::vector<ExtractedLeaf> &leaves)
{
	if (hazards.empty() || leaves.empty())
		return 0;

	std::vector<ExtractedLeaf> kept;
	kept.reserve(leaves.size());

	for (const ExtractedLeaf &leaf : leaves)
	{
		const Face *floorFace = LowestFloorFace(leaf.polytope);
		bool lethal = false;

		if (floorFace)
		{
			Vec3 stand = WindingCentroid(floorFace->winding);
			stand.z += kLiquidStandLift;

			for (const LiquidVolume &vol : hazards)
			{
				if (vol.ContainsPoint(stand))
				{
					lethal = true;
					break;
				}
			}
		}

		if (!lethal)
			kept.push_back(leaf);
	}

	size_t dropped = leaves.size() - kept.size();
	leaves.swap(kept);
	return dropped;
}

size_t PruneLiquidHazardConnections(const std::vector<LiquidVolume> &hazards,
	std::vector<MergedConnection> &conns)
{
	if (hazards.empty() || conns.empty())
		return 0;

	std::vector<MergedConnection> kept;
	kept.reserve(conns.size());

	for (const MergedConnection &c : conns)
	{
		if (c.connType == crossbot_nav::CONN_TELEPORT || c.connType == crossbot_nav::CONN_ELEVATOR)
		{
			kept.push_back(c);
			continue;
		}

		Vec3 delta = c.pointB - c.pointA;
		double length = delta.Length();
		size_t steps = static_cast<size_t>(length / kLiquidSampleStep) + 1;

		bool lethal = false;
		for (size_t s = 0; s <= steps && !lethal; ++s)
		{
			Vec3 p = c.pointA + delta * (static_cast<double>(s) / static_cast<double>(steps));
			p.z += kLiquidStandLift;

			for (const LiquidVolume &vol : hazards)
			{
				if (vol.ContainsPoint(p))
				{
					lethal = true;
					break;
				}
			}
		}

		if (!lethal)
			kept.push_back(c);
	}

	size_t dropped = conns.size() - kept.size();
	conns.swap(kept);
	return dropped;
}

}
