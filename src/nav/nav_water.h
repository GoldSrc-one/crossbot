#pragma once

#include <vector>

#include "nav_mesh.h"

namespace Nav
{

const std::vector<float> &WaterSurfaces(const NavMesh &mesh);

float WaterSurfaceZ(const NavMesh &mesh, uint8_t volumeIndex);

bool Submerged(const NavMesh &mesh, uint8_t volumeIndex, float submergeZ);

inline bool AreaSubmerged(const NavMesh &mesh, const Area &area)
{
	return Submerged(mesh, area.waterVolume, area.submergeZ);
}

inline bool ConnSubmerged(const NavMesh &mesh, const Connection &conn)
{
	return Submerged(mesh, conn.waterVolume, conn.submergeZ);
}

void ResetWaterCache();

}
