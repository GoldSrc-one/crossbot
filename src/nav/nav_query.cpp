#include "nav_query.h"

#include "nav_math.h"

namespace Nav
{

namespace
{
constexpr float kAabbEpsilon = 4.0f;
constexpr float kPlaneEpsilon = 2.0f;

bool OutsideBounds(const Area &area, const Vector &point)
{
	return point.x < area.mins.x - kAabbEpsilon || point.x > area.maxs.x + kAabbEpsilon ||
		point.y < area.mins.y - kAabbEpsilon || point.y > area.maxs.y + kAabbEpsilon ||
		point.z < area.mins.z - kAabbEpsilon || point.z > area.maxs.z + kAabbEpsilon;
}
}

bool AreaContainsPoint(const Area &area, const Vector &point)
{
	if (OutsideBounds(area, point))
		return false;

	return NavMath::PointInsidePlanes(point, area.planes, kPlaneEpsilon);
}

const Area *FindAreaExact(const NavMesh &mesh, const Vector &point)
{
	for (const Area &area : mesh.areas)
		if (AreaContainsPoint(area, point))
			return &area;

	return nullptr;
}

const Area *FindAreaNearest(const NavMesh &mesh, const Vector &point)
{
	if (const Area *exact = FindAreaExact(mesh, point))
		return exact;

	const Area *best = nullptr;
	float bestDist = 0.f;
	for (const Area &area : mesh.areas)
	{
		if (OutsideBounds(area, point))
			continue;

		float dist = (point - area.centroid).Length();
		if (!best || dist < bestDist)
		{
			best = &area;
			bestDist = dist;
		}
	}

	if (best || mesh.areas.empty())
		return best;

	const Area *nearest = nullptr;
	float nearestDist = 0.f;
	for (const Area &area : mesh.areas)
	{
		float dist = (point - area.centroid).Length();
		if (!nearest || dist < nearestDist)
		{
			nearest = &area;
			nearestDist = dist;
		}
	}
	return nearest;
}

const Area *FindAreaById(const NavMesh &mesh, uint32_t id)
{
	if (id < mesh.areas.size() && mesh.areas[id].id == id)
		return &mesh.areas[id];
	for (const Area &area : mesh.areas)
		if (area.id == id)
			return &area;
	return nullptr;
}

}

