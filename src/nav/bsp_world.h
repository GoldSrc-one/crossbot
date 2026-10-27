#pragma once

#include <cstdint>
#include <memory>
#include <vector>

#include "hull_trace.h"
#include "pm_sim.h"

namespace Nav
{

struct BspWorld
{
	std::vector<dplane_t> planes;
	std::vector<dclipnode_t> clipnodes;
	std::vector<dmodel_t> models;
	uint32_t checksum = 0;

	HullTrace::HullTree Hull() const;
};

std::shared_ptr<const BspWorld> LoadBspWorld(const char *bspPath);

struct WorldSolid
{
	int index = 0;
	int owner = 0;
	int model = -1;
	Vector origin;
	Vector absmin;
	Vector absmax;
	Vector boxMins;
	Vector boxMaxs;
	bool rotated = false;
	Vector forward;
	Vector right;
	Vector up;
	bool counts = false;
	bool breakable = false;
	Vector push;
};

struct TraceFilter
{
	int hull = 1;
	int ignore = 0;
	bool phaseBreakables = false;
};

struct WorldSnapshot
{
	std::shared_ptr<const BspWorld> world;
	std::vector<WorldSolid> solids;
	std::vector<WorldSolid> pushes;
};

void SetSolidAngles(WorldSolid &solid, const Vector &angles);

PmSim::PmTrace TraceWorld(const WorldSnapshot &snap, const TraceFilter &filter, const Vector &start,
	const Vector &end);

Vector PushAt(const WorldSnapshot &snap, int hull, const Vector &origin);

float SafeArcTime(const WorldSnapshot &snap, const TraceFilter &filter, const Vector &start, const Vector &vel,
	float gravity, float maxTime);

}
