#pragma once

#include <extdll.h>

#include <cstdint>
#include <vector>

#include "nav_format.h"

namespace Nav
{

constexpr size_t kVantageBlendNeighbors = 4;
constexpr float kVantageMergeCone = 45.f;
constexpr float kVantageIdwEpsilon = 1.f;
constexpr float kVantageMinAmount = 0.05f;

constexpr float kVantageDuckEyeLift = 12.f;
constexpr float kVantageMeanEyeLift = 29.f;

struct VantageWatch
{
	float yaw = 0.f;
	float pitch = 0.f;
	float spread = 0.f;
	float amount = 0.f;
	float dist = 0.f;
};

struct Vantage
{
	Vector pos;
	float exposure = 0.f;
	VantageWatch watch[crossbot_nav::kNavWatchDirs];
	int watchCount = 0;
};

struct VantageField
{
	std::vector<Vantage> vantages;
	float spacing = 0.f;
	float maxExposure = 0.f;

	Vector gridMins;
	float cell = 0.f;
	int dims[3] = {0, 0, 0};
	std::vector<uint32_t> cellStart;
	std::vector<uint32_t> cellItems;
};

struct VantageBlend
{
	bool valid = false;
	int neighbors = 0;
	float nearestDist = 0.f;
	float exposure = 0.f;
	VantageWatch watch[crossbot_nav::kNavWatchDirs];
	int watchCount = 0;
};

Vector VantageDir(float yaw, float pitch);

void BuildVantageGrid(VantageField &field);

bool BlendVantage(const VantageField &field, const Vector &eye, VantageBlend &out);

// Amount-weighted mean open distance looking along dir. Returns 0 when nothing is
// visible that way, which is correctly the shortest possible engagement range.
float VantageOpenDistance(const VantageBlend &blend, const Vector &dir);

}
