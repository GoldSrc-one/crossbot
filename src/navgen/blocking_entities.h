#pragma once

#include <string>
#include <vector>

#include "bsp_file.h"
#include "entity_parser.h"
#include "leaf_extractor.h"
#include "nav_format.h"

namespace navgen
{

struct BlockingVolume
{
	int headnode = -1;
	Vec3 origin;
	Vec3 mins;
	Vec3 maxs;
	int entityIndex = -1;
};

struct PushVolume
{
	BlockingVolume volume;
	Vec3 push;
};

struct ActivatorVolume
{
	uint8_t kind = crossbot_nav::ACT_BUTTON;
	Vec3 mins;
	Vec3 maxs;
	Vec3 center;
	int entityIndex = -1;
};

struct PassableVolume
{
	BlockingVolume volume;
	uint32_t areaFlag = crossbot_nav::NAV_AREA_NONE;
	uint8_t activation = crossbot_nav::ACT_NONE;
	std::vector<ActivatorVolume> activators;
	double openSeconds = 0.0;
	int entityIndex = -1;
};

struct PassableSet
{
	std::vector<PassableVolume> passable;
	std::vector<BlockingVolume> impassable;
};

struct NullCarveEvent
{
	int entityIndex = -1;
	size_t pieces = 0;
	double volume = 0.0;
	Vec3 centroid;
	double after = 0.0;
	size_t outFaces = 0;
};

struct CarveStats
{
	size_t inputs = 0;
	size_t tests = 0;
	size_t boundsHits = 0;
	size_t carves = 0;
	size_t nullCarves = 0;
	size_t nullCarveExtraPieces = 0;
	size_t inflatedCarves = 0;
	double worstInflation = 0.0;
	size_t solidCarves = 0;
	size_t solidCarveExtraPieces = 0;
	size_t piecesOut = 0;
	size_t peakPieces = 0;
	std::vector<size_t> perBlockerPieces;
	std::vector<size_t> perBlockerNullCarves;
	std::vector<size_t> perBlockerBoundsHits;
	std::vector<NullCarveEvent> nullEvents;
};

double ParseDouble(const std::string &s, double def);

int ParseInt(const std::string &s, int def);

int ParseBrushModelIndex(const std::string &model);

Vec3 ParseEntityOrigin(const std::string &origin);

Vec3 EntityAngles(const Entity &ent);

Vec3 MoveDirFromAngles(const Vec3 &angles);

Vec3 ModelWorldCenter(const dmodel_t &model, const Vec3 &origin);

double DoorSpeed(const Entity &ent);

double DoorTravelDistance(const Entity &ent, const dmodel_t &model);

void CollectActivatorsFor(const BspFile &bsp, const std::vector<Entity> &entities,
	const std::string &targetname, std::vector<ActivatorVolume> &out);

std::vector<BlockingVolume> ExtractBlockingVolumes(const BspFile &bsp, const std::vector<Entity> &entities,
	int hullIndex);

std::vector<BlockingVolume> ExtractLadderVolumes(const BspFile &bsp, const std::vector<Entity> &entities,
	int hullIndex);

std::vector<PushVolume> ExtractPushVolumes(const BspFile &bsp, const std::vector<Entity> &entities,
	int hullIndex);

std::vector<double> CollectGravityMultipliers(const std::vector<Entity> &entities);

PassableSet ExtractPassableVolumes(const BspFile &bsp, const std::vector<Entity> &entities, int hullIndex,
	const std::vector<int> &skipEntities = std::vector<int>());

bool OverlapsAnyBlockingVolume(const ExtractedLeaf &leaf, const std::vector<BlockingVolume> &blockers);

bool OverlapsBlockingVolume(const Polytope &poly, const BlockingVolume &blocker);

std::vector<Polytope> SubtractBlockingVolumes(const BspFile &bsp, const Polytope &leafPolytope,
	const std::vector<BlockingVolume> &blockers, double mapRadius, double epsilon = 0.01,
	CarveStats *stats = nullptr);

std::vector<Polytope> SubtractBoxVolumes(const Polytope &leafPolytope,
	const std::vector<BlockingVolume> &boxes, double mapRadius, double epsilon = 0.01);

void SplitByPassableVolume(const BspFile &bsp, const Polytope &poly, const BlockingVolume &blocker,
	double mapRadius, std::vector<Polytope> &inside, std::vector<Polytope> &outside, double epsilon = 0.01,
	CarveStats *stats = nullptr);

}
