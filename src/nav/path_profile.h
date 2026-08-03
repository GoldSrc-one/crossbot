#pragma once

#include <cstddef>
#include <vector>

#include "waypoint_plan.h"

namespace NavPlan
{

struct ProfileSeg
{
	bool isArc = false;
	bool gap = false;

	Vector a, b;

	Vector center;
	float radius = 0.f;
	float startAngle = 0.f;
	float turn = 0.f;

	float sStart = 0.f;
	float sLen = 0.f;

	float vCap = 0.f;
	float vEntry = 0.f;
	float vExit = 0.f;
	float vReach = 0.f;

	float clearance = kPlanClearance;
	size_t wpIndex = 0;
};

struct ProfileSample
{
	Vector point;
	Vector tangent;
	float speed = 0.f;
	float clearance = kPlanClearance;
	size_t wpIndex = 0;
	bool gap = false;
};

struct PlanProfile
{
	std::vector<ProfileSeg> segs;
	float totalLength = 0.f;
	float totalTime = 0.f;

	float friction = 4.f;
	float stopSpeed = 100.f;

	size_t cornersArced = 0;
	size_t cornersSlowed = 0;
	size_t cornersShrunk = 0;

	bool Empty() const { return segs.empty(); }
};

float SpeedForRadius(float radius, float accel, float friction, float stopSpeed);

float RadiusForSpeed(float speed, float accel, float friction, float stopSpeed);

float PivotSpeed(const PmSim::PmPhysics &physics, float maxSpeed);

PlanProfile BuildPathProfile(const std::vector<PlanWaypoint> &waypoints,
	const PmSim::PmPhysics &physics, const PlanTraceContext *traces = nullptr);

float ProfileReachAtWaypoint(const PlanProfile &profile, size_t wpIndex);

ProfileSample SampleProfile(const PlanProfile &profile, float s);

float ProjectOntoProfile(const PlanProfile &profile, const Vector &pos, float sMin, float sMax,
	float *lateralOut);

}
