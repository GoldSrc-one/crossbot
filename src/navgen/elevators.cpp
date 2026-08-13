#include "elevators.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "floor_support.h"
#include "hull_defs.h"

namespace navgen
{

namespace
{

constexpr int kSfDoorStartOpen = 1;
constexpr int kSfDoorNoAutoReturn = 32;
constexpr int kSfPlatToggle = 1;

constexpr double kDefaultPlatSpeed = 150.0;
constexpr double kPlatTopDwell = 3.0;
constexpr double kPlatBottomLip = 8.0;

constexpr double kDwellForever = -1.0;
constexpr double kMinStopVolume = 1.0;
constexpr double kMinSupportedFloorArea = 1.0;
constexpr size_t kMaxClaimSplitPlanes = 16;
constexpr double kOverlapSlack = 0.5;

void PolytopeAabb(const Polytope &poly, Vec3 &mins, Vec3 &maxs)
{
	mins = Vec3(1e30, 1e30, 1e30);
	maxs = Vec3(-1e30, -1e30, -1e30);
	for (const Face &f : poly.faces)
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

BlockingVolume MoverVolume(const dmodel_t &model, const Vec3 &origin, double offsetZ, int hullIndex)
{
	Vec3 hullMins(kHumanHullMinX, kHumanHullMinY, kHumanHullMinZ);
	Vec3 hullMaxs(kHumanHullMaxX, kHumanHullMaxY, kHumanHullMaxZ);
	if (hullIndex == kHullDuck)
	{
		hullMins = Vec3(kDuckClipHullMinX, kDuckClipHullMinY, kDuckClipHullMinZ);
		hullMaxs = Vec3(kDuckClipHullMaxX, kDuckClipHullMaxY, kDuckClipHullMaxZ);
	}

	BlockingVolume vol;
	vol.headnode = model.headnode[hullIndex];
	vol.origin = origin + Vec3(0.0, 0.0, offsetZ);
	vol.mins = Vec3(model.mins[0] + hullMins.x, model.mins[1] + hullMins.y,
		model.mins[2] + hullMins.z) + vol.origin;
	vol.maxs = Vec3(model.maxs[0] + hullMaxs.x, model.maxs[1] + hullMaxs.y,
		model.maxs[2] + hullMaxs.z) + vol.origin;
	return vol;
}

BlockingVolume BoxVolume(const Vec3 &mins, const Vec3 &maxs)
{
	BlockingVolume vol;
	vol.mins = mins;
	vol.maxs = maxs;
	return vol;
}

void SwapConnectionEnds(MergedConnection &c)
{
	std::swap(c.areaA, c.areaB);
	std::swap(c.pointA, c.pointB);
	std::swap(c.planeA, c.planeB);
	std::swap(c.gateALeft, c.gateBLeft);
	std::swap(c.gateARight, c.gateBRight);
	std::swap(c.gateAValid, c.gateBValid);
	std::swap(c.gateAClamped, c.gateBClamped);
}

bool OverlapsXY(const ElevatorMover &a, const ElevatorMover &b)
{
	return a.columnMins.x < b.columnMaxs.x && a.columnMaxs.x > b.columnMins.x &&
		a.columnMins.y < b.columnMaxs.y && a.columnMaxs.y > b.columnMins.y;
}

void BuildStops(const dmodel_t &model, double moveOffset, double defaultOffset, double rideSeconds,
	double dwellAtRest, double dwellAtMoved, double mapRadius, ElevatorMover &mover)
{
	double offsets[2] = {0.0, moveOffset};
	if (offsets[0] > offsets[1])
		std::swap(offsets[0], offsets[1]);

	for (int k = 0; k < 2; ++k)
	{
		ElevatorStop stop;
		stop.stopId = k;
		stop.travel = offsets[k];
		stop.topZ = mover.origin.z + model.maxs[2] + offsets[k] + kDuckClipHullMaxZ;
		stop.rideSeconds = rideSeconds;
		stop.isDefault = std::fabs(offsets[k] - defaultOffset) < kElevatorAxisEpsilon;
		stop.waitSeconds = std::fabs(offsets[k]) < kElevatorAxisEpsilon ? dwellAtRest : dwellAtMoved;
		stop.duckVolume = MoverVolume(model, mover.origin, offsets[k], kHullDuck);
		stop.standVolume = MoverVolume(model, mover.origin, offsets[k], kHullHuman);
		mover.stops.push_back(stop);
	}

	for (size_t k = 0; k < mover.stops.size(); ++k)
		mover.stops[k].slabTopZ = (k + 1 < mover.stops.size()) ? mover.stops[k + 1].topZ : mapRadius;
}

uint8_t ClassifyMover(const BspFile &bsp, const std::vector<Entity> &entities, size_t entIndex,
	double mapRadius, ElevatorMover &mover)
{
	const Entity &ent = entities[entIndex];

	mover.entityIndex = static_cast<int>(entIndex);
	mover.classname = ent.Get("classname");
	mover.targetname = ent.Get("targetname");

	bool isDoor = mover.classname == "func_door";
	bool isPlat = mover.classname == "func_plat";
	if (!isDoor && !isPlat)
		return ELEV_NOT_A_MOVER;

	int modelIndex = ParseBrushModelIndex(ent.Get("model"));
	if (modelIndex < 0 || static_cast<size_t>(modelIndex) >= bsp.Models().size())
		return ELEV_NO_MODEL;

	const dmodel_t &model = bsp.Models()[static_cast<size_t>(modelIndex)];
	if (model.headnode[kHullDuck] < 0)
		return ELEV_NO_MODEL;

	mover.origin = ParseEntityOrigin(ent.Get("origin"));
	mover.movedir = isPlat ? Vec3(0.0, 0.0, -1.0) : MoveDirFromAngles(ParseEntityOrigin(ent.Get("angles")));

	if (std::fabs(mover.movedir.z) <= kFloorNormalZ ||
		std::fabs(mover.movedir.x) > kElevatorAxisEpsilon ||
		std::fabs(mover.movedir.y) > kElevatorAxisEpsilon)
		return ELEV_NOT_VERTICAL;

	Vec3 size(model.maxs[0] - model.mins[0], model.maxs[1] - model.mins[1],
		model.maxs[2] - model.mins[2]);
	if (size.x < kElevatorMinFootprint || size.y < kElevatorMinFootprint)
		return ELEV_FOOTPRINT;

	mover.columnMins = Vec3(model.mins[0] + kDuckClipHullMinX, model.mins[1] + kDuckClipHullMinY, 0.0) +
		Vec3(mover.origin.x, mover.origin.y, 0.0);
	mover.columnMaxs = Vec3(model.maxs[0] + kDuckClipHullMaxX, model.maxs[1] + kDuckClipHullMaxY, 0.0) +
		Vec3(mover.origin.x, mover.origin.y, 0.0);

	int spawnflags = ParseInt(ent.Get("spawnflags"), 0);

	if (isDoor)
	{
		double travel = DoorTravelDistance(ent, model);
		if (travel <= kElevatorAxisEpsilon)
			return ELEV_NO_TRAVEL;

		mover.speed = DoorSpeed(ent);

		double moveOffset = mover.movedir.z > 0.0 ? travel : -travel;
		double defaultOffset = (spawnflags & kSfDoorStartOpen) ? moveOffset : 0.0;

		double wait = ParseDouble(ent.Get("wait"), 0.0);
		bool autoReturn = wait >= 0.0 && (spawnflags & kSfDoorNoAutoReturn) == 0;
		double dwell = autoReturn ? wait : kDwellForever;

		bool startOpen = (spawnflags & kSfDoorStartOpen) != 0;
		BuildStops(model, moveOffset, defaultOffset, travel / mover.speed,
			startOpen ? dwell : kDwellForever, startOpen ? kDwellForever : dwell, mapRadius, mover);
	}
	else
	{
		double height = ParseDouble(ent.Get("height"), 0.0);
		double travel = height != 0.0 ? height : size.z - kPlatBottomLip;
		if (travel <= kElevatorAxisEpsilon)
			return ELEV_NO_TRAVEL;

		mover.speed = ParseDouble(ent.Get("speed"), 0.0);
		if (mover.speed <= 0.0)
			mover.speed = kDefaultPlatSpeed;

		bool startsAtTop = !mover.targetname.empty();
		bool autoReturn = (spawnflags & kSfPlatToggle) == 0;

		BuildStops(model, -travel, startsAtTop ? 0.0 : -travel, travel / mover.speed,
			autoReturn ? kPlatTopDwell : kDwellForever, kDwellForever, mapRadius, mover);
	}

	if (!mover.targetname.empty())
	{
		CollectActivatorsFor(bsp, entities, mover.targetname, mover.activators);
		for (const ActivatorVolume &act : mover.activators)
		{
			if (mover.activation == crossbot_nav::ACT_NONE)
				mover.activation = act.kind;
			if (act.kind == crossbot_nav::ACT_BUTTON)
			{
				mover.activation = crossbot_nav::ACT_BUTTON;
				break;
			}
		}
	}

	return ELEV_ACCEPTED;
}

void PrintMover(const ElevatorMover &mover, uint8_t reject, const Vec3 &size)
{
	std::printf("  %s [ent %d] %s \"%s\" movedir (%.2f, %.2f, %.2f) speed %.1f -- %s\n",
		reject == ELEV_ACCEPTED ? "elevator" : "door", mover.entityIndex, mover.classname.c_str(),
		mover.targetname.c_str(), mover.movedir.x, mover.movedir.y, mover.movedir.z, mover.speed,
		ElevatorRejectReason(reject));

	std::printf("    footprint %.1f x %.1f (min %.1f)\n", size.x, size.y, kElevatorMinFootprint);

	if (reject != ELEV_ACCEPTED)
		return;

	std::printf("    column x%.1f..%.1f y%.1f..%.1f\n",
		mover.columnMins.x, mover.columnMaxs.x, mover.columnMins.y, mover.columnMaxs.y);

	for (const ElevatorStop &stop : mover.stops)
	{
		std::printf("    stop %d travel %7.1f topZ %8.1f slabTop %8.1f ride %.2fs dwell ", stop.stopId,
			stop.travel, stop.topZ, stop.slabTopZ, stop.rideSeconds);
		if (stop.waitSeconds < 0.0)
			std::printf("never");
		else
			std::printf("%.2fs", stop.waitSeconds);
		std::printf("%s\n", stop.isDefault ? "  DEFAULT" : "");
	}

	std::printf("    activation %s, %zu activator(s)\n",
		mover.activation == crossbot_nav::ACT_BUTTON ? "button"
			: mover.activation == crossbot_nav::ACT_TRIGGER ? "trigger" : "none",
		mover.activators.size());
}

bool OverlapsVolumeStrict(const Polytope &poly, const BlockingVolume &vol)
{
	Vec3 mins, maxs;
	PolytopeAabb(poly, mins, maxs);

	return mins.x < vol.maxs.x - kOverlapSlack && maxs.x > vol.mins.x + kOverlapSlack &&
		mins.y < vol.maxs.y - kOverlapSlack && maxs.y > vol.mins.y + kOverlapSlack &&
		mins.z < vol.maxs.z - kOverlapSlack && maxs.z > vol.mins.z + kOverlapSlack;
}

double SupportedFloorArea(const BspFile &bsp, int headnode, const std::vector<BlockingVolume> &blockers,
	const Face &floorFace)
{
	double total = 0.0;
	for (const Winding &w : ComputeSupportedFloor(bsp, headnode, blockers, floorFace))
		total += WindingArea(w, floorFace.plane.normal);
	return total;
}

bool MoverIsTheFloor(const BspFile &bsp, int headnode, const std::vector<BlockingVolume> &worldSupport,
	const std::vector<BlockingVolume> &moverOnly, const Polytope &piece)
{
	const Face *floor = LowestFloorFace(piece);
	if (!floor)
		return false;

	if (SupportedFloorArea(bsp, headnode, worldSupport, *floor) > kMinSupportedFloorArea)
		return false;

	return SupportedFloorArea(bsp, CONTENTS_EMPTY, moverOnly, *floor) > kMinSupportedFloorArea;
}

void CollectBoundaryPlanes(const std::vector<Winding> &windings, std::vector<Plane> &planes)
{
	for (const Winding &w : windings)
	{
		if (w.size() < 3)
			continue;

		for (size_t i = 0, j = w.size() - 1; i < w.size(); j = i++)
		{
			Plane plane;
			if (VerticalPlaneThroughEdge(w[j], w[i], plane) && !HasPlane(planes, plane))
				planes.push_back(plane);
		}
	}
}

void SplitByPlanes(const Polytope &poly, const std::vector<Plane> &planes, size_t index, double mapRadius,
	std::vector<Polytope> &out)
{
	if (poly.Empty() || poly.Volume() <= kMinStopVolume)
		return;

	if (index >= planes.size())
	{
		out.push_back(poly);
		return;
	}

	const Plane &plane = planes[index];
	Plane flipped;
	flipped.normal = plane.normal * -1.0;
	flipped.dist = -plane.dist;

	if (PolytopeInsidePlane(poly, plane) || PolytopeInsidePlane(poly, flipped))
	{
		SplitByPlanes(poly, planes, index + 1, mapRadius, out);
		return;
	}

	SplitByPlanes(ClipPolytope(poly, plane, true, mapRadius), planes, index + 1, mapRadius, out);
	SplitByPlanes(ClipPolytope(poly, plane, false, mapRadius), planes, index + 1, mapRadius, out);
}

void ClaimStopFloor(const BspFile &bsp, int headnode, const std::vector<BlockingVolume> &worldSupport,
	const ElevatorStop &stop, const Polytope &piece, double mapRadius, std::vector<Polytope> &claimed,
	std::vector<Polytope> &keep)
{
	Vec3 mins(stop.duckVolume.mins.x, stop.duckVolume.mins.y, stop.topZ);
	Vec3 maxs(stop.duckVolume.maxs.x, stop.duckVolume.maxs.y, stop.slabTopZ);

	Polytope candidate = ClipPolytopeToBox(piece, mins, maxs, mapRadius);
	if (candidate.Empty() || candidate.Volume() <= kMinStopVolume)
	{
		keep.push_back(piece);
		return;
	}

	std::vector<BlockingVolume> moverOnly{stop.duckVolume};
	std::vector<Polytope> parts;

	if (const Face *floor = LowestFloorFace(candidate))
	{
		std::vector<Plane> planes;
		CollectBoundaryPlanes(ComputeSupportedFloor(bsp, headnode, worldSupport, *floor), planes);
		CollectBoundaryPlanes(ComputeSupportedFloor(bsp, CONTENTS_EMPTY, moverOnly, *floor), planes);

		if (planes.size() > kMaxClaimSplitPlanes)
			planes.clear();

		SplitByPlanes(candidate, planes, 0, mapRadius, parts);
	}
	else
	{
		parts.push_back(candidate);
	}

	std::vector<Polytope> unclaimed;
	size_t before = claimed.size();

	for (Polytope &part : parts)
	{
		if (MoverIsTheFloor(bsp, headnode, worldSupport, moverOnly, part))
			claimed.push_back(std::move(part));
		else
			unclaimed.push_back(std::move(part));
	}

	if (claimed.size() == before)
	{
		keep.push_back(piece);
		return;
	}

	for (Polytope &part : unclaimed)
		keep.push_back(std::move(part));

	for (Polytope &rest : SubtractBoxVolumes(piece, {BoxVolume(mins, maxs)}, mapRadius))
		keep.push_back(std::move(rest));
}

}

const ElevatorStop *ElevatorSet::Stop(int globalId) const
{
	if (globalId < 0 || static_cast<size_t>(globalId) >= stopIndex.size())
		return nullptr;
	const ElevatorStopRef &ref = stopIndex[static_cast<size_t>(globalId)];
	return &movers[static_cast<size_t>(ref.moverIndex)].stops[static_cast<size_t>(ref.stopIndex)];
}

const ElevatorMover *ElevatorSet::MoverOf(int globalId) const
{
	if (globalId < 0 || static_cast<size_t>(globalId) >= stopIndex.size())
		return nullptr;
	return &movers[static_cast<size_t>(stopIndex[static_cast<size_t>(globalId)].moverIndex)];
}

const char *ElevatorRejectReason(uint8_t reject)
{
	switch (reject)
	{
	case ELEV_ACCEPTED:
		return "accepted";
	case ELEV_NOT_A_MOVER:
		return "rejected: not a func_door or func_plat";
	case ELEV_NO_MODEL:
		return "rejected: no usable brush model";
	case ELEV_NOT_VERTICAL:
		return "rejected: movedir is not axis-aligned vertical";
	case ELEV_FOOTPRINT:
		return "rejected: footprint too small to stand on";
	case ELEV_NO_TRAVEL:
		return "rejected: zero travel";
	case ELEV_SHAFT_OVERLAP:
		return "rejected: shaft overlaps an earlier elevator";
	}
	return "rejected: unknown";
}

ElevatorSet ExtractElevators(const BspFile &bsp, const std::vector<Entity> &entities, double mapRadius,
	bool dump)
{
	ElevatorSet result;
	size_t rejected = 0;

	for (size_t entIndex = 0; entIndex < entities.size(); ++entIndex)
	{
		const std::string classname = entities[entIndex].Get("classname");
		if (classname != "func_door" && classname != "func_plat")
			continue;

		ElevatorMover mover;
		uint8_t reject = ClassifyMover(bsp, entities, entIndex, mapRadius, mover);

		if (reject == ELEV_ACCEPTED)
			for (const ElevatorMover &other : result.movers)
				if (OverlapsXY(mover, other))
				{
					reject = ELEV_SHAFT_OVERLAP;
					break;
				}

		if (dump)
		{
			Vec3 size;
			int modelIndex = ParseBrushModelIndex(entities[entIndex].Get("model"));
			if (modelIndex >= 0 && static_cast<size_t>(modelIndex) < bsp.Models().size())
			{
				const dmodel_t &model = bsp.Models()[static_cast<size_t>(modelIndex)];
				size = Vec3(model.maxs[0] - model.mins[0], model.maxs[1] - model.mins[1],
					model.maxs[2] - model.mins[2]);
			}
			PrintMover(mover, reject, size);
		}

		if (reject != ELEV_ACCEPTED)
		{
			++rejected;
			continue;
		}

		result.claimedEntities.push_back(static_cast<int>(entIndex));
		result.movers.push_back(std::move(mover));
	}

	for (size_t mi = 0; mi < result.movers.size(); ++mi)
	{
		ElevatorMover &mover = result.movers[mi];

		for (size_t si = 0; si < mover.stops.size(); ++si)
		{
			ElevatorStop &stop = mover.stops[si];
			stop.globalId = static_cast<int>(result.stopIndex.size());

			ElevatorStopRef ref;
			ref.moverIndex = static_cast<int>(mi);
			ref.stopIndex = static_cast<int>(si);
			result.stopIndex.push_back(ref);

			result.supportDuck.push_back(stop.duckVolume);
			if (!stop.isDefault)
				continue;
			result.defaultDuck.push_back(stop.duckVolume);
			result.defaultStand.push_back(stop.standVolume);
		}
	}

	if (!result.movers.empty())
	{
		size_t stops = 0;
		for (const ElevatorMover &mover : result.movers)
			stops += mover.stops.size();
		std::printf("%zu elevator(s) with %zu stops, %zu mover(s) left to the door path\n",
			result.movers.size(), stops, rejected);
	}

	return result;
}

std::vector<ExtractedLeaf> SplitLeavesByElevatorStops(const BspFile &bsp, int headnode,
	const std::vector<BlockingVolume> &worldSupport, const ElevatorSet &elevators,
	const std::vector<ExtractedLeaf> &leaves, double mapRadius, size_t *stopLeavesOut, bool dump)
{
	std::vector<ExtractedLeaf> out;
	size_t stopLeaves = 0;

	auto push = [&](Polytope poly, const ExtractedLeaf &from, int stopId)
	{
		if (poly.Empty() || poly.Volume() <= kMinStopVolume)
			return;

		ExtractedLeaf piece;
		piece.id = static_cast<int>(out.size());
		piece.polytope = std::move(poly);
		piece.passableIndex = from.passableIndex;
		piece.elevatorStopId = stopId;

		if (dump && stopId >= 0)
		{
			Vec3 mins, maxs;
			PolytopeAabb(piece.polytope, mins, maxs);
			std::printf("    stop %d leaf x%.1f..%.1f y%.1f..%.1f z%.1f..%.1f volume %.0f\n", stopId,
				mins.x, maxs.x, mins.y, maxs.y, mins.z, maxs.z, piece.polytope.Volume());
		}

		out.push_back(std::move(piece));
	};

	for (const ExtractedLeaf &leaf : leaves)
	{
		std::vector<Polytope> pending;
		pending.push_back(leaf.polytope);

		for (const ElevatorMover &mover : elevators.movers)
		{
			for (const ElevatorStop &stop : mover.stops)
			{
				std::vector<Polytope> keep;

				for (Polytope &piece : pending)
				{
					std::vector<Polytope> claimed;
					ClaimStopFloor(bsp, headnode, worldSupport, stop, piece, mapRadius, claimed, keep);

					for (Polytope &region : claimed)
					{
						size_t before = out.size();
						push(std::move(region), leaf, stop.globalId);
						if (out.size() != before)
							++stopLeaves;
					}
				}

				pending.swap(keep);
			}
		}

		for (const Polytope &piece : pending)
		{
			std::vector<BlockingVolume> hit;
			for (const BlockingVolume &vol : elevators.defaultDuck)
				if (OverlapsVolumeStrict(piece, vol))
					hit.push_back(vol);

			if (hit.empty())
			{
				push(piece, leaf, -1);
				continue;
			}

			for (Polytope &rest : SubtractBlockingVolumes(bsp, piece, hit, mapRadius))
				push(std::move(rest), leaf, -1);
		}
	}

	if (stopLeavesOut)
		*stopLeavesOut = stopLeaves;

	return out;
}

void TagElevatorAreas(const ElevatorSet &elevators, std::vector<Area> &areas)
{
	for (Area &area : areas)
	{
		const ElevatorStop *stop = elevators.Stop(area.elevatorStopId);
		const ElevatorMover *mover = elevators.MoverOf(area.elevatorStopId);
		if (!stop || !mover || stop->isDefault)
			continue;

		area.activation = mover->activation;
		area.openSeconds = stop->rideSeconds;
		area.activators = mover->activators;
		area.activationEntity = mover->entityIndex;
	}
}

AirmoveWorlds BuildAirmoveWorlds(const ElevatorSet &elevators, std::vector<BlockingVolume> baseDuck,
	std::vector<BlockingVolume> baseStand)
{
	AirmoveWorlds worlds;
	worlds.baseDuck = std::move(baseDuck);
	worlds.baseStand = std::move(baseStand);
	worlds.stops.resize(elevators.stopIndex.size());
	worlds.defaultStopOf.assign(elevators.movers.size(), -1);

	for (size_t globalId = 0; globalId < elevators.stopIndex.size(); ++globalId)
	{
		const ElevatorStopRef &ref = elevators.stopIndex[globalId];
		if (ref.moverIndex < 0 || ref.stopIndex < 0)
			continue;

		const ElevatorMover &mover = elevators.movers[static_cast<size_t>(ref.moverIndex)];
		const ElevatorStop &stop = mover.stops[static_cast<size_t>(ref.stopIndex)];

		AirmoveWorldStop &out = worlds.stops[globalId];
		out.moverId = ref.moverIndex;
		out.isDefault = stop.isDefault;
		out.duckVolume = stop.duckVolume;
		out.standVolume = stop.standVolume;

		if (stop.isDefault)
			worlds.defaultStopOf[static_cast<size_t>(ref.moverIndex)] = static_cast<int>(globalId);
	}

	return worlds;
}

size_t BuildElevatorConnections(const ElevatorSet &elevators, const std::vector<Area> &areas,
	std::vector<MergedConnection> &conns, bool dump)
{
	std::vector<int> stopArea(elevators.stopIndex.size(), -1);
	for (const Area &area : areas)
		if (area.elevatorStopId >= 0 && static_cast<size_t>(area.elevatorStopId) < stopArea.size() &&
			stopArea[static_cast<size_t>(area.elevatorStopId)] < 0)
			stopArea[static_cast<size_t>(area.elevatorStopId)] = area.id;

	int maxAreaId = -1;
	for (const Area &area : areas)
		maxAreaId = std::max(maxAreaId, area.id);

	std::vector<int> areaStop(static_cast<size_t>(maxAreaId + 1), -1);
	for (const Area &area : areas)
		areaStop[static_cast<size_t>(area.id)] = area.elevatorStopId;

	auto stopOfArea = [&](int areaId)
	{
		return (areaId >= 0 && static_cast<size_t>(areaId) < areaStop.size())
			? areaStop[static_cast<size_t>(areaId)] : -1;
	};

	std::vector<uint8_t> areaActivation(areaStop.size(), crossbot_nav::ACT_NONE);
	for (const Area &area : areas)
		areaActivation[static_cast<size_t>(area.id)] = area.activation;

	auto ungatedStopArea = [&](int areaId)
	{
		const ElevatorStop *stop = elevators.Stop(stopOfArea(areaId));
		return stop != nullptr && !stop->isDefault &&
			areaActivation[static_cast<size_t>(areaId)] == crossbot_nav::ACT_NONE;
	};

	size_t droppedWalks = 0;
	size_t oneWayOut = 0;
	conns.erase(std::remove_if(conns.begin(), conns.end(), [&](MergedConnection &c)
		{
			if (c.connType != crossbot_nav::CONN_WALK)
				return false;

			int a = stopOfArea(c.areaA);
			int b = stopOfArea(c.areaB);

			if (a >= 0 && b >= 0 && a != b)
			{
				const ElevatorMover *ma = elevators.MoverOf(a);
				if (ma != nullptr && ma == elevators.MoverOf(b))
				{
					++droppedWalks;
					return true;
				}
			}

			bool outA = ungatedStopArea(c.areaA);
			bool outB = ungatedStopArea(c.areaB);
			if (outA != outB)
			{
				if (outB)
					SwapConnectionEnds(c);
				c.connFlags |= crossbot_nav::CONN_FLAG_ONE_WAY;
				++oneWayOut;
			}

			return false;
		}), conns.end());

	size_t built = 0;
	for (const ElevatorMover &mover : elevators.movers)
	{
		double centerX = (mover.columnMins.x + mover.columnMaxs.x) * 0.5;
		double centerY = (mover.columnMins.y + mover.columnMaxs.y) * 0.5;

		for (const ElevatorStop &from : mover.stops)
		{
			for (const ElevatorStop &to : mover.stops)
			{
				if (from.globalId == to.globalId)
					continue;

				int areaA = stopArea[static_cast<size_t>(from.globalId)];
				int areaB = stopArea[static_cast<size_t>(to.globalId)];
				if (areaA < 0 || areaB < 0)
					continue;

				if (to.isDefault && from.waitSeconds < 0.0)
				{
					if (dump)
						std::printf("    ride %d -> %d skipped: stop %d never leaves on its own\n",
							from.globalId, to.globalId, from.globalId);
					continue;
				}

				MergedConnection mc;
				mc.areaA = areaA;
				mc.areaB = areaB;
				mc.connType = crossbot_nav::CONN_ELEVATOR;
				mc.connFlags = crossbot_nav::CONN_FLAG_ONE_WAY;
				mc.pointA = Vec3(centerX, centerY, from.topZ);
				mc.pointB = Vec3(centerX, centerY, to.topZ);
				mc.rideSeconds = to.rideSeconds;
				mc.rideSecondsMax = to.rideSeconds;
				mc.rideWaitSeconds = to.isDefault ? std::max(from.waitSeconds, 0.0) : 0.0;
				conns.push_back(mc);
				++built;

				if (dump)
					std::printf("    ride area %d -> %d  stop %d -> %d  ride %.2fs wait %.2fs\n", areaA,
						areaB, from.globalId, to.globalId, mc.rideSeconds, mc.rideWaitSeconds);
			}
		}
	}

	if (droppedWalks > 0 && dump)
		std::printf("    dropped %zu walk connection(s) between stops of one elevator\n", droppedWalks);

	if (oneWayOut > 0 && dump)
		std::printf("    %zu walk connection(s) made exit-only off a stop with no way to summon it\n",
			oneWayOut);

	return built;
}

}
